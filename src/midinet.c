/*
  Hatari - midinet.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  MIDI over TCP: a "MIDI cable" between emulator instances, so that programs
  that network ST machines over their MIDI ports (MIDI-Maze style rings: each
  machine's MIDI OUT to the next one's MIDI IN) can be run and tested with
  several Hatari instances, on one host or across a LAN.

    --midi-net-in  [host:]port   listen there: the byte stream of the one
                                 connection accepted is this machine's MIDI IN
    --midi-net-out host:port     connect there: this machine's MIDI OUT

  A ring of three:  A: --midi-net-in 5001 --midi-net-out localhost:5002
                    B: --midi-net-in 5002 --midi-net-out localhost:5003
                    C: --midi-net-in 5003 --midi-net-out localhost:5001

  The bytes are the raw serial stream (no MIDI message framing, unlike
  PortMidi), so any protocol works.  Like a real cable that is not plugged in
  yet, bytes sent while the output is not connected are lost; the output
  connection is retried every half second, a closed input connection is
  accepted again.  The ACIA emulation (midi.c) paces both directions at
  31250 bit/s in emulated time; the instances themselves are not
  synchronised, so run them at normal speed (no fast forward).
*/
const char MidiNet_fileid[] = "Hatari midinet.c";

#if defined(_WIN32)
# include <winsock2.h>
# include <ws2tcpip.h>
typedef SOCKET net_sock_t;
# define NET_INVALID INVALID_SOCKET
# define net_close closesocket
# define net_errno() WSAGetLastError()
# define NET_WOULDBLOCK(e) ((e) == WSAEWOULDBLOCK)
# define NET_INPROGRESS(e) ((e) == WSAEWOULDBLOCK || (e) == WSAEINPROGRESS)
#else
# include <sys/types.h>
# include <sys/socket.h>
# include <netinet/in.h>
# include <netinet/tcp.h>
# include <netdb.h>
# include <fcntl.h>
# include <unistd.h>
# include <errno.h>
# include <time.h>
typedef int net_sock_t;
# define NET_INVALID (-1)
# define net_close close
# define net_errno() errno
# define NET_WOULDBLOCK(e) ((e) == EAGAIN || (e) == EWOULDBLOCK)
# define NET_INPROGRESS(e) ((e) == EINPROGRESS || (e) == EAGAIN)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "log.h"
#include "midinet.h"

#define RETRY_MS	500
#define OUTQ_SIZE	8192

static bool bActive;
static bool bWsa;
static net_sock_t ListenSock = NET_INVALID;	/* --midi-net-in: the listening socket */
static net_sock_t InSock = NET_INVALID;		/* its accepted connection: MIDI IN */
static net_sock_t OutSock = NET_INVALID;	/* --midi-net-out: MIDI OUT */
static bool bOutConnected;
static uint32_t OutRetryTicks;
static struct sockaddr_storage OutAddr;
static socklen_t OutAddrLen;

static uint8_t InBuf[512];
static int InPos, InLen;
static uint8_t OutQ[OUTQ_SIZE];			/* bytes the socket did not take yet */
static int OutHead, OutLen;


static uint32_t MidiNet_Ticks(void)	/* milliseconds, for the retries */
{
#if defined(_WIN32)
	return (uint32_t)GetTickCount();
#else
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
#endif
}

static bool MidiNet_NonBlocking(net_sock_t s)
{
#if defined(_WIN32)
	u_long on = 1;
	return ioctlsocket(s, FIONBIO, &on) == 0;
#else
	int fl = fcntl(s, F_GETFL, 0);
	return fl != -1 && fcntl(s, F_SETFL, fl | O_NONBLOCK) == 0;
#endif
}

static void MidiNet_NoDelay(net_sock_t s)
{
	int on = 1;
	setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&on, sizeof(on));
}

/**
 * "host:port" or "port" (host: default_host) -> address; false on errors
 */
static bool MidiNet_Resolve(const char *spec, const char *default_host, bool passive,
			    struct sockaddr_storage *addr, socklen_t *len)
{
	char host[256];
	const char *port;
	const char *colon = strrchr(spec, ':');
	struct addrinfo hints, *res = NULL;

	if (colon)
	{
		size_t n = colon - spec;
		if (n >= sizeof(host))
			return false;
		memcpy(host, spec, n);
		host[n] = '\0';
		port = colon + 1;
	}
	else
	{
		strcpy(host, default_host ? default_host : "");
		port = spec;
	}
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	if (passive)
		hints.ai_flags = AI_PASSIVE;
	if (getaddrinfo(host[0] ? host : NULL, port, &hints, &res) != 0 || !res)
		return false;
	memcpy(addr, res->ai_addr, res->ai_addrlen);
	*len = (socklen_t)res->ai_addrlen;
	freeaddrinfo(res);
	return true;
}

/**
 * Open the transport (either direction may be absent).  Returns false when
 * a given spec cannot be resolved or the input port cannot be listened on.
 */
bool MidiNet_Open(const char *in_spec, const char *out_spec)
{
	MidiNet_Close();
#if defined(_WIN32)
	if (!bWsa)
	{
		WSADATA wsa;
		if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
			return false;
		bWsa = true;
	}
#endif
	InPos = InLen = OutHead = OutLen = 0;
	if (in_spec && in_spec[0])
	{
		struct sockaddr_storage a;
		socklen_t alen;
		int on = 1;

		if (!MidiNet_Resolve(in_spec, NULL, true, &a, &alen))
		{
			Log_Printf(LOG_ERROR, "MIDI net: bad input address '%s'\n", in_spec);
			return false;
		}
		ListenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (ListenSock == NET_INVALID)
			return false;
		setsockopt(ListenSock, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof(on));
		if (bind(ListenSock, (struct sockaddr *)&a, alen) != 0
		    || listen(ListenSock, 1) != 0 || !MidiNet_NonBlocking(ListenSock))
		{
			Log_Printf(LOG_ERROR, "MIDI net: cannot listen on '%s' (error %d)\n",
				   in_spec, net_errno());
			MidiNet_Close();
			return false;
		}
		Log_Printf(LOG_INFO, "MIDI net: MIDI IN listening on %s\n", in_spec);
	}
	if (out_spec && out_spec[0])
	{
		if (!MidiNet_Resolve(out_spec, "127.0.0.1", false, &OutAddr, &OutAddrLen))
		{
			Log_Printf(LOG_ERROR, "MIDI net: bad output address '%s'\n", out_spec);
			MidiNet_Close();
			return false;
		}
		OutRetryTicks = 0;
		Log_Printf(LOG_INFO, "MIDI net: MIDI OUT to %s\n", out_spec);
	}
	else
	{
		OutAddrLen = 0;
	}
	bActive = true;
	return true;
}

void MidiNet_Close(void)
{
	if (InSock != NET_INVALID)
		net_close(InSock);
	if (ListenSock != NET_INVALID)
		net_close(ListenSock);
	if (OutSock != NET_INVALID)
		net_close(OutSock);
	InSock = ListenSock = OutSock = NET_INVALID;
	bOutConnected = false;
	bActive = false;
}

bool MidiNet_IsActive(void)
{
	return bActive;
}

/**
 * The output connection: started, or its progress checked
 */
static void MidiNet_PollOut(void)
{
	if (bOutConnected || OutAddrLen == 0)
		return;
	if (OutSock == NET_INVALID)
	{
		uint32_t now = MidiNet_Ticks();
		if (OutRetryTicks && (int32_t)(now - OutRetryTicks) < 0)
			return;
		OutRetryTicks = now + RETRY_MS;
		OutSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (OutSock == NET_INVALID)
			return;
		MidiNet_NonBlocking(OutSock);
		MidiNet_NoDelay(OutSock);
		if (connect(OutSock, (struct sockaddr *)&OutAddr, OutAddrLen) == 0)
		{
			bOutConnected = true;
			Log_Printf(LOG_INFO, "MIDI net: MIDI OUT connected\n");
			return;
		}
		if (!NET_INPROGRESS(net_errno()))
		{
			net_close(OutSock);
			OutSock = NET_INVALID;
			return;
		}
	}
	/* connecting: done? */
	{
		fd_set wr, ex;
		struct timeval tv = { 0, 0 };
		int err = 0;
		socklen_t elen = sizeof(err);

		FD_ZERO(&wr);
		FD_ZERO(&ex);
		FD_SET(OutSock, &wr);
		FD_SET(OutSock, &ex);
		if (select((int)OutSock + 1, NULL, &wr, &ex, &tv) <= 0)
			return;
		if (FD_ISSET(OutSock, &ex)
		    || getsockopt(OutSock, SOL_SOCKET, SO_ERROR, (char *)&err, &elen) != 0 || err)
		{
			net_close(OutSock);		/* refused: try again later */
			OutSock = NET_INVALID;
			return;
		}
		bOutConnected = true;
		Log_Printf(LOG_INFO, "MIDI net: MIDI OUT connected\n");
	}
}

/**
 * Flush the output queue as far as the socket takes it
 */
static void MidiNet_Flush(void)
{
	while (bOutConnected && OutLen > 0)
	{
		int n = OutLen;
		int sent;
		if (OutHead + n > OUTQ_SIZE)
			n = OUTQ_SIZE - OutHead;
		sent = send(OutSock, (const char *)OutQ + OutHead, n, 0);
		if (sent > 0)
		{
			OutHead = (OutHead + sent) % OUTQ_SIZE;
			OutLen -= sent;
			continue;
		}
		if (sent < 0 && NET_WOULDBLOCK(net_errno()))
			return;
		Log_Printf(LOG_INFO, "MIDI net: MIDI OUT disconnected\n");
		net_close(OutSock);			/* the other end went away */
		OutSock = NET_INVALID;
		bOutConnected = false;
		OutLen = 0;
	}
}

/**
 * The next received byte, or -1 when none is waiting
 */
int MidiNet_ReadByte(void)
{
	MidiNet_PollOut();
	MidiNet_Flush();
	if (InPos < InLen)
		return InBuf[InPos++];
	if (InSock == NET_INVALID)
	{
		if (ListenSock == NET_INVALID)
			return -1;
		InSock = accept(ListenSock, NULL, NULL);
		if (InSock == NET_INVALID)
			return -1;
		MidiNet_NonBlocking(InSock);
		MidiNet_NoDelay(InSock);
		Log_Printf(LOG_INFO, "MIDI net: MIDI IN connected\n");
	}
	{
		int n = recv(InSock, (char *)InBuf, sizeof(InBuf), 0);
		if (n > 0)
		{
			InPos = 1;
			InLen = n;
			return InBuf[0];
		}
		if (n == 0 || !NET_WOULDBLOCK(net_errno()))
		{
			Log_Printf(LOG_INFO, "MIDI net: MIDI IN disconnected\n");
			net_close(InSock);
			InSock = NET_INVALID;
		}
	}
	return -1;
}

/**
 * Send a byte (lost when the output is not connected, as on an unplugged
 * cable); always succeeds
 */
bool MidiNet_WriteByte(uint8_t byte)
{
	MidiNet_PollOut();
	if (!bOutConnected)
		return true;
	if (OutLen < OUTQ_SIZE)
	{
		OutQ[(OutHead + OutLen) % OUTQ_SIZE] = byte;
		OutLen++;
	}
	MidiNet_Flush();
	return true;
}
