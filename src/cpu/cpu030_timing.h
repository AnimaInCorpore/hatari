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
 * long write to the 16-bit ST-RAM port also waits between its two word
 * cycles: 11/8 clocks on average with the Falcon bus model below (fitted to
 * f030probe), HATARI_CPU030_STRAM_WR (default 2) with HATARI_F030_BUS=off
 * (dspmark's earlier fit: 10.44 cycles per move.l Dn,(An)+ on hardware).
 * HATARI_CPU030_TIMING=off gives the bare cycle-exact core.
 */
#ifndef CPU030_TIMING_H
#define CPU030_TIMING_H

extern bool cpu030t_enabled;
extern int cpu030t_stram_long_write;	/* extra clocks between the halves of a long ST-RAM write */

/* Falcon bus model measured on a Falcon030 with f030probe (F030Arcade
 * f030probe/, 2026-11-23): the clocks a 68030 access costs beyond what the
 * cycle-exact core's bank model charges, per ST-RAM access, per ROM access
 * and per I/O device.  They are CPU clocks: the probe's 8 MHz lines show the
 * same gaps in 8 MHz clocks.  HATARI_F030_BUS=off disables them. */
extern bool cpu030t_bus_on;
extern int cpu030t_bus_pre;
int cpu030t_bus_extra(uaecptr addr, int size, bool write);
int cpu030t_bus_pre_extra(uaecptr addr, int size, bool write);
int cpu030t_stram_lw_clocks(void);

/* The ST-RAM part, when it is charged before the access's bus slot. */
#define CPU030T_BUS_PRE(addr, size, write) do { \
	if (cpu030t_bus_pre) { \
		int bp_ = cpu030t_bus_pre_extra((addr), (size), (write)); \
		if (bp_ > 0) \
			x_do_cycles(bp_ * cpucycleunit); \
	} \
} while (0)

/* Add the measured extra clocks of one access (size 1, 2 or 4 bytes). */
#define CPU030T_BUS_EXTRA(addr, size, write) do { \
	if (cpu030t_bus_on) { \
		int bx_ = cpu030t_bus_extra((addr), (size), (write)); \
		if (bx_ > 0) \
			x_do_cycles(bx_ * cpucycleunit); \
	} \
} while (0)

void cpu030t_init(void);
void cpu030t_access(uaecptr addr, evt_t start, evt_t end, bool write);
void cpu030t_instruction_end(uae_u16 opcode, uaecptr instruction_pc);
void cpu030t_abort(void);

#endif
