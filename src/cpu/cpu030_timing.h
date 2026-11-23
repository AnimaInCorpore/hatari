/*
 * MC68030 instruction timing from the MC68030 User's Manual, chapter 11.
 *
 * The 68030 "cycle-exact" core (gencpu table 23) charges instruction prefetch
 * (2 clocks per word) and the real time of every bus access, but none of the
 * manual's internal times: the head/tail/cache-case figures appear in the
 * generated code only as comments.  On a Falcon (or anywhere with
 * HATARI_CPU030_TIMING=manual) each instruction is padded, after it ran, up
 * to the manual's instruction-cache-case time (formula 11-2, overlap with
 * the previous instruction's tail), with the wait states of its actual data
 * accesses added by the rules of section 11.5.  Bus-bound instructions are never shortened.  On a Falcon a
 * long write to the 16-bit ST-RAM port also waits HATARI_CPU030_STRAM_WR
 * (default 2) clocks between its two word cycles: dspmark's hardware line,
 * 10.44 cycles per move.l Dn,(An)+ against 8.60 without it, 10.66 with it.
 * HATARI_CPU030_TIMING=off gives the bare cycle-exact core.
 */
#ifndef CPU030_TIMING_H
#define CPU030_TIMING_H

extern bool cpu030t_enabled;
extern int cpu030t_stram_long_write;	/* extra clocks between the halves of a long ST-RAM write */

void cpu030t_init(void);
void cpu030t_access(uaecptr addr, evt_t start, evt_t end, bool write);
void cpu030t_instruction_end(uae_u16 opcode, uaecptr instruction_pc);

#endif
