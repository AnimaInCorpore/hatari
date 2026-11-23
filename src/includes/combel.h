/*
  Hatari - combel.h

  This file is distributed under the GNU General Public License, version 2
  or at your option any later version. Read the file gpl.txt for details.

  COMBEL, the Falcon030 memory controller: the ST-RAM scheduler its Memory
  Control Unit implements, and the CPU stalls that fall out of it.
*/

#ifndef HATARI_COMBEL_H
#define HATARI_COMBEL_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Longs COMBEL transfers to Videl in one refill.  The Falcon030 Service Guide
 * has COMBEL answering Videl's VREQ by driving VLD and moving display data
 * "32 bits at a time"; the authoritative Falcon video guide and mikro's and
 * Czuba's independent measurements all put one refill at 17 of those longs,
 * the first carrying the DRAM RAS setup and the rest running in fast page
 * mode. */
#define COMBEL_REFILL_LONGS		17

/* CPU cycles that refill holds ST-RAM, at the emulated CPU clock.  The
 * sourced figure was ~19 at 16 MHz (38 CLK32 ticks).  Measured on a Falcon030
 * with F030Arcade's f030probe (2026-11-23, section 6, RGB, TOS 4.02): the
 * 68030's extra cost per cache-missing ST-RAM long read while the display
 * fetches is 0.41 / 0.73 / 1.31 / 3.17 clocks at 80 / 160 / 320 / 640 bytes
 * per line, and 25 clocks per 17-long refill reproduces all four (0.34 /
 * 0.65 / 1.65 / 2.95) and the falling cost across the line after display end;
 * 19 gives 0.20 / 0.46 / 0.92 / 2.17.  --videl-burst-cycles overrides it. */
#define COMBEL_REFILL_CYCLES_DEFAULT	25

/* What Videl tells COMBEL about its appetite for source data.  Filled in by
 * videl.c, which owns the register semantics; COMBEL turns it into a refill
 * schedule and into CPU stalls. */
typedef struct {
	uint32_t line_cycles;		/* CPU cycles in one Videl scan line */
	uint32_t first_line;		/* first scan line that consumes source data */
	uint32_t source_lines;		/* lines of source data read per frame */
	uint32_t line_step;		/* scan lines per source line (2 = line doubling) */
	uint32_t longs_per_line;	/* 32-bit words of source data per source line */
	uint32_t display_start;		/* cycles into a line before pixel output starts */
	uint32_t display_cycles;	/* cycles of pixel output in a line */
} COMBEL_VIDEO_DEMAND;

/* Fast gate for the CPU bus path: true while a video refill schedule is armed,
 * i.e. while an ST-RAM access can be made to wait.  Not an instantaneous
 * "the bus is busy right now" flag -- ask Combel_StRam_Stall() for that. */
extern bool Combel_StRam_Contended;

extern void Combel_Reset(void);
extern void Combel_Video_SetDemand(const COMBEL_VIDEO_DEMAND *demand);
extern void Combel_Video_Idle(void);
extern int  Combel_StRam_Stall(uint64_t cycle, uint32_t addr);
extern void Combel_Info(FILE *fp, uint32_t dummy);

#endif /* HATARI_COMBEL_H */
