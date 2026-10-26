/*
  Hatari - midinet.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  MIDI over TCP between emulator instances (midinet.c).
*/

#ifndef HATARI_MIDINET_H
#define HATARI_MIDINET_H

#include <stdbool.h>
#include <stdint.h>

extern bool MidiNet_Open(const char *in_spec, const char *out_spec);
extern void MidiNet_Close(void);
extern bool MidiNet_IsActive(void);
extern int  MidiNet_ReadByte(void);
extern bool MidiNet_WriteByte(uint8_t byte);

#endif
