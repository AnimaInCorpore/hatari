/*
  Hatari - combel.c

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  COMBEL is the Falcon030's memory controller.  Its Memory Control Unit owns
  every ST-RAM access: it multiplexes the DRAM row/column addresses, serves
  the CPU and DMA, runs refresh, and loads display data into Videl.  Atari's
  Falcon030 Service Guide gives the arbitration order as

      Video  >  DMA  >  Refresh  >  Blitter

  and notes that only DMA and the Blitter arbitrate for the system bus in the
  normal manner -- video is not a BR/BG bus master, which is why modelling it
  as one would be wrong.

  Video service works by request, not by schedule: Videl drains its FIFO at
  the pixel rate, asserts VREQ when it needs more, and COMBEL answers by
  driving VLD and moving COMBEL_REFILL_LONGS 32-bit words out of ST-RAM.  The
  video address counter lives in COMBEL, not in Videl.

  What is modelled here
  ---------------------
  The FIFO refill loop, in closed form.  Videl's drain rate is constant across
  a line, so VREQ is periodic with a period of exactly one refill unit's worth
  of pixel time, and the schedule can be answered per access with no event
  queue and no drift.  Two consequences of the request model that a
  "reserve a burst at the first visible pixel" model does not get for free
  fall out of it:

   - the first refill of a line lands *before* the first visible pixel, since
     the FIFO must hold data before the display pipeline consumes any;
   - a line-doubled mode fetches its source line once, not twice.  mikro
     measured line doubling costing only a few percent, not the doubling a
     displayed-pixel bandwidth model predicts, so the demand Videl reports is
     in source lines and the refills for one source line are spread across the
     scan lines it feeds.

  What is not modelled yet
  ------------------------
   - The other three clients.  DMA sound, refresh and the Blitter take slots
     on real hardware; the priority order above is the shape their arbitration
     should take, but nothing measured sizes them yet (items D5 and E1 of
     F030Arcade's docs/falcon-timing-probe.md).
   - FIFO starvation as a *visible* effect.  A mode that outruns the memory
     subsystem is detected and logged here, but the display is still drawn as
     if the data had arrived.
   - Half-line Videl timing.  Hatari's Falcon video engine counts whole lines
     and, for VGA, half as many lines per frame as the Videl does.  The line
     period below is therefore taken from the Videl registers rather than from
     that grid, so the refill rate does not inherit the error; only the phase
     against the frame start does.
   - The video address counter following the fetches, and the CLK32 domain as
     a domain of its own.  Everything here is in emulated CPU cycles (19 of
     them at 16 MHz being the 38 CLK32 ticks COMBEL and Videl actually share).

  Acceptance
  ----------
  Two published bandwidth figures, derived independently of each other and of
  this code, fall out of the constants above:

   - Czuba (CT2 technical notes) puts 640x480 in 2 colours at 60 Hz at about
     4% of ST-RAM.  The model: 80 bytes per line is 20 longs, 2 refills, 23
     cycles of a 448-cycle line over 400 of 501 lines = 4.1% of the frame.
   - mikro puts 320x240 truecolor at 16.1%.  The model: 640 bytes per line is
     160 longs, 10 refills, 179 cycles of a 512-cycle VGA line over 240 of
     525 lines = 16.0% of the frame.

  Neither is a measurement of this machine.  E1 in F030Arcade's
  docs/falcon-timing-probe.md is; until it runs, note that in the mode the
  2026-09-02 hardware session used (RGB, the desktop's ST-low) this model
  predicts only ~0.3 cycles of mean stall per ST-RAM access, so it is not
  what closes dspmark's 2-cycle ST-RAM gap.  That gap is WP7's.
*/

const char Combel_fileid[] = "Hatari combel.c";

#include <string.h>

#include "main.h"
#include "combel.h"
#include "configuration.h"
#include "log.h"
#include "stMemory.h"
#include "video.h"


/* Refill schedule, derived from the demand Videl reports. */
static struct {
	uint32_t line_cycles;		/* CPU cycles in one Videl scan line */
	uint32_t first_line;		/* first scan line that consumes source data */
	uint32_t scan_lines;		/* scan lines over which source data is read */
	uint32_t group_cycles;		/* cycles one source line's refills span */
	uint32_t refills;		/* refills serving one source line */
	uint32_t period;		/* cycles between two VREQs */
	uint32_t lines_per_group;	/* scan lines one source line feeds */
	uint32_t start;			/* cycles into the group before the first refill */
	uint32_t refill_cycles;		/* cycles a full refill holds ST-RAM */
	uint32_t last_cycles;		/* cycles the trailing, short refill holds it */
	uint32_t ram_limit;		/* accesses below this address reach ST-RAM */
	bool     starving;		/* demand outruns what COMBEL can supply */
	uint64_t stall_calls;		/* ST-RAM accesses that consulted the schedule */
	uint64_t stalls;		/* CPU accesses made to wait, since reset */
	uint64_t stall_cycles;		/* cycles they waited, in total */
} Combel;

bool Combel_StRam_Contended;


void Combel_Reset(void)
{
	memset(&Combel, 0, sizeof(Combel));
	Combel_StRam_Contended = false;
}


/**
 * Videl is not asking for data (blanked, no display, or contention disabled).
 */
void Combel_Video_Idle(void)
{
	if (Combel_StRam_Contended)
		LOG_TRACE(TRACE_VIDEL, "COMBEL: video idle, ST-RAM contention off\n");
	Combel_StRam_Contended = false;
}


/**
 * Take Videl's data demand and turn it into the refill schedule COMBEL runs.
 */
void Combel_Video_SetDemand(const COMBEL_VIDEO_DEMAND *demand)
{
	uint32_t full, rem, longs;
	bool starving;

	Combel_StRam_Contended = false;

	if (!ConfigureParams.System.bVidelBusContention)
		return;
	if (demand->line_cycles < 16 || demand->longs_per_line == 0
	    || demand->source_lines == 0 || demand->line_step == 0)
		return;

	if (ConfigureParams.System.nVidelBurstCycles < 1)
		return;
	Combel.refill_cycles = ConfigureParams.System.nVidelBurstCycles;

	/* COMBEL fronts the whole ST-RAM array, all 14 MB of it on an expanded
	 * board -- not just the 4 MB a stock Falcon ships.  TT-RAM is the CPU's
	 * alone and never reaches this path anyway (CE_MEMBANK_FAST32). */
	Combel.ram_limit = STRamEnd;
	if (Combel.ram_limit == 0)
		return;

	Combel.line_cycles = demand->line_cycles;
	Combel.first_line = demand->first_line;
	Combel.scan_lines = demand->source_lines * demand->line_step;

	/* One source line's data is consumed over the pixel-output time of every
	 * scan line it feeds, so a doubled line halves the drain rate. */
	Combel.lines_per_group = demand->line_step;
	Combel.group_cycles = demand->display_cycles * demand->line_step;
	if (Combel.group_cycles == 0 || Combel.group_cycles > demand->line_cycles * demand->line_step)
		Combel.group_cycles = demand->line_cycles * demand->line_step;

	longs = demand->longs_per_line;
	full = longs / COMBEL_REFILL_LONGS;
	rem = longs % COMBEL_REFILL_LONGS;
	Combel.refills = full + (rem ? 1 : 0);
	/* The trailing refill moves fewer longs.  Charged pro rata: splitting its
	 * RAS setup out from its page-mode reads would need a second constant
	 * that no measurement gives us. */
	Combel.last_cycles = rem
		? (Combel.refill_cycles * rem + COMBEL_REFILL_LONGS - 1) / COMBEL_REFILL_LONGS
		: Combel.refill_cycles;

	/* VREQ is periodic: one refill unit's worth of pixel time. */
	Combel.period = Combel.group_cycles / Combel.refills;

	/* A mode can ask for data faster than COMBEL can fetch it.  Real Videl
	 * does not get to hold the bus for longer than the line: it runs the
	 * refills back to back and its FIFO starves, and the display comes out
	 * short of data.  Model the half that is about the bus -- the refills
	 * that fit, back to back -- rather than letting an impossible mode stall
	 * the CPU for more than the line it is drawing.  The starved display is
	 * still drawn intact; that half is not modelled. */
	starving = (Combel.period < Combel.refill_cycles);
	if (starving) {
		Combel.period = Combel.refill_cycles;
		Combel.refills = Combel.group_cycles / Combel.refill_cycles;
		if (Combel.refills == 0) {
			Combel.starving = false;
			return;
		}
		Combel.last_cycles = Combel.refill_cycles;
	}

	/* The FIFO has to hold data before the first pixel is consumed, so the
	 * first refill of a line runs during the border ahead of it.  One refill
	 * unit of lead is the minimum the request loop can work with, and is the
	 * only choice here that does not invent a FIFO depth: the depth Videl
	 * really has is one of the things the die reverse-engineering would
	 * settle. */
	if (demand->display_start >= Combel.refill_cycles)
		Combel.start = demand->display_start - Combel.refill_cycles;
	else
		Combel.start = 0;

	if (starving && !Combel.starving)
		Log_Printf(LOG_WARN, "COMBEL: video mode outruns ST-RAM, Videl FIFO starves: "
			   "%u longs wanted per source line, %u refills fit.  The display is "
			   "still drawn intact.\n",
			   longs, Combel.refills);
	Combel.starving = starving;

	/* The refills span the display window, which sits inside the line, so
	 * the bound is the group of scan lines -- not the window itself, which
	 * would throw the line phase away. */
	if (Combel.start + Combel.refills * Combel.period
	    > Combel.line_cycles * Combel.lines_per_group)
		Combel.start = 0;

	Combel_StRam_Contended = true;

	LOG_TRACE(TRACE_VIDEL, "COMBEL: %u refills/line of %u longs, VREQ every %u cycles "
		  "from +%u, %u-cycle line, lines %u..%u step %u\n",
		  Combel.refills, longs, Combel.period, Combel.start,
		  Combel.line_cycles, Combel.first_line,
		  Combel.first_line + Combel.scan_lines, demand->line_step);
}


/**
 * Cycles a 68030 ST-RAM access starting at 'cycle' has to wait for COMBEL to
 * finish the Videl refill it is running.  'cycle' is an absolute
 * CyclesGlobalClockCounter value at the emulated CPU clock, 'addr' the
 * physical address.  Accesses that the caches absorb, and accesses to TT-RAM
 * or I/O, never get here.
 */
int Combel_StRam_Stall(uint64_t cycle, uint32_t addr)
{
	uint32_t rel, line, pos, d, n, start, cost;
	uint64_t since_vbl;

	Combel.stall_calls++;
	if (addr >= Combel.ram_limit)
		return 0;

	if (cycle <= VBL_ClockCounter)
		return 0;
	since_vbl = cycle - VBL_ClockCounter;
	if (since_vbl > 0x40000000)			/* no usable frame anchor */
		return 0;
	rel = (uint32_t)since_vbl;

	line = rel / Combel.line_cycles;
	if (line < Combel.first_line)
		return 0;
	line -= Combel.first_line;
	if (line >= Combel.scan_lines)
		return 0;

	/* Position inside the group of scan lines that one source line feeds:
	 * with line doubling the refills for a source line are spread over both
	 * of the scan lines it is shown on. */
	pos = rel - (line + Combel.first_line) * Combel.line_cycles;
	if (Combel.lines_per_group > 1)
		pos += (line % Combel.lines_per_group) * Combel.line_cycles;

	if (pos < Combel.start)
		return 0;
	d = pos - Combel.start;

	n = d / Combel.period;
	if (n >= Combel.refills)
		return 0;

	start = n * Combel.period;
	cost = (n == Combel.refills - 1) ? Combel.last_cycles : Combel.refill_cycles;
	if (d < start + cost) {
		Combel.stalls++;
		Combel.stall_cycles += start + cost - d;
		return (int)(start + cost - d);
	}

	return 0;
}


void Combel_Info(FILE *fp, uint32_t dummy)
{
	uint32_t per_line;

	fprintf(fp, "ST-RAM contention : %s\n", Combel_StRam_Contended ? "on" : "off");
	fprintf(fp, "Client priority   : video > DMA > refresh > blitter"
		    " (only video modelled)\n");
	if (!Combel_StRam_Contended)
		return;

	per_line = (Combel.refills - 1) * Combel.refill_cycles + Combel.last_cycles;

	fprintf(fp, "Videl scan line   : %u cycles, source read on lines %u..%u\n",
		Combel.line_cycles, Combel.first_line,
		Combel.first_line + Combel.scan_lines);
	fprintf(fp, "VREQ              : %u per source line, every %u cycles from +%u\n",
		Combel.refills, Combel.period, Combel.start);
	fprintf(fp, "Refill            : %u longs in %u cycles (last one %u)\n",
		COMBEL_REFILL_LONGS, Combel.refill_cycles, Combel.last_cycles);
	fprintf(fp, "ST-RAM held       : %u cycles per source line, %.1f%% of the frame\n",
		per_line,
		100.0 * per_line * (Combel.scan_lines / Combel.lines_per_group)
		/ (double)(Combel.line_cycles * (Combel.first_line + Combel.scan_lines)));
	fprintf(fp, "Charged so far    : %llu accesses stalled, %llu cycles\n",
		(unsigned long long)Combel.stalls,
		(unsigned long long)Combel.stall_cycles);
	if (Combel.starving)
		fprintf(fp, "FIFO              : STARVING (VREQ period below refill time)\n");
}
