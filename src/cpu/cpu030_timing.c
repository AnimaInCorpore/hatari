/*
 * MC68030 instruction timing from the MC68030 User's Manual, chapter 11.
 * See cpu030_timing.h.
 *
 * Model per instruction (clocks):
 *   CC   = CCea + CCop - min(Hop, Tea + Wread) + Wread + Wwrite   (11-2, 11.5)
 *   time = CC - min(Hfirst, Tprev)
 * where Wread/Wwrite are the wait states of the instruction's data accesses
 * as the bus model charged them (access duration beyond the manual's two
 * clocks), and Tprev is the previous instruction's tail plus its last write's
 * wait states (11.5 rule 3a).  The instruction is padded up to that time and
 * never shortened, so bus traffic and instruction-cache misses still count
 * in full.
 *
 * Sources: the operation head/tail/clocks of table68k (the manual's 11.6.x
 * tables as WinUAE transcribed them), the effective-address tables of
 * gencpu.c (11.6.1-11.6.5), and from the manual directly the MOVE table
 * (11.6.6), the branch table (11.6.15) and the ADDA/SUBA/CMPA rows, whose
 * register/EA heads table68k has the wrong way round.
 */

#include "sysconfig.h"
#include "sysdeps.h"

#include "main.h"
#include "hatari-glue.h"
#include "log.h"
#include "options_cpu.h"
#include "memory.h"
#include "newcpu.h"
#include "readcpu.h"
#include "events.h"
#include "configuration.h"
#include "cpu030_timing.h"
#include "stMemory.h"
#include "crossbar.h"

bool cpu030t_enabled;
int cpu030t_stram_long_write;
bool cpu030t_bus_on;

/* Falcon bus extras, CPU clocks (see cpu030_timing.h).  Hardware against
 * the c93698f7 core, f030probe lines io.* / cache.dc0.* / vid.*.burst.*.vbl
 * (Falcon030, TOS 4.02, 2026-11-23):
 *   ST-RAM: a read and a word write cost ~1 clock more per access; a long
 *     write ~0.7 less than the +2 between its halves (dspmark's fit) gave.
 *   ROM:    word read 8.24 (core 6.16), long read 13.32 (9.20).
 *   I/O:    every 16-bit register on the Videl/Blitter side 7.24 per read
 *     (3.16), 5.16 per write (3.20), a long 11.28 / 9.12 (6.16 / 6.16);
 *     DMA sound/crossbar 8.24 / 6.12; MFP 16.2 (7.16); YM 13.32 / 11.12
 *     (7.16); ACIA 16.32 (10.24); SCC 13.28 (3.12); FDC/DMA 14.64 (3.16);
 *     the DSP host port already matches (6.24 / 6.16). */
/* ST-RAM extras in eighths of a clock, spread by an accumulator per access
 * class: the core's ST-RAM port works on a 4-clock slot grid where one more
 * clock costs 0 or 2, while the hardware's gaps are fractional means
 * (+1.28 word read, +1.40 long read, +1.20 word write).  lw8 is the wait
 * between the word halves of a long write, which replaces
 * cpu030t_stram_long_write while the bus model is on. */
enum { BX_RD, BX_RDL, BX_WR, BX_LW, BX_NCLASS };
/* Fitted with f030probe section 6 (sustained and vertical-blank lines in
 * 80, 320 and 640 bytes-per-line modes, TOS 4.02): within ~0.3 of the
 * hardware, which the 4-clock slot grid does not let a constant beat. */
static int bx_st8[BX_NCLASS] = { 6, 5, 3, 11 };
static int bx_acc[BX_NCLASS];
static int bx_st_pre = 1;
static int bx_rom_w = 2, bx_rom_l = 4;
int cpu030t_bus_pre;			/* ST-RAM extras go before the bus-slot alignment */

/* DMA sound steals from the 68030's ST-RAM accesses in proportion to its
 * byte rate. f030probe section 8 (2026-11-24), 8-bit and 16-bit stereo at
 * 49170 Hz (98 / 197 KB/s): a long read 0.24 / 0.48 clocks dearer, a long
 * write 0.24 / 0.40, a word read 0.16 / 0.32, a word write 0.08 / 0.16.
 * Clocks per access per byte/s, in 2^-32, per class (BX_RD, BX_RDL, BX_WR,
 * BX_LW: a long write's wait between its halves), refitted on the slot grid
 * (mean error 0.10 clocks an access).  They go into the same eighths as the
 * fixed extras: charged on their own, a whole clock now and then, the 4-clock
 * slot grid swallowed them.  HATARI_F030_SND=off drops it, =a,b,c,d refits. */
static uae_u32 bx_snd_q32[BX_NCLASS] = { 2500, 12000, 3000, 5000 };
static uae_u64 bx_snd_acc[BX_NCLASS];
static bool bx_snd_on = true;
static uae_u32 bx_snd_rate;		/* bytes/s, sampled per access */

static int bx_take(int cls)
{
	int n;

	if (bx_snd_rate) {
		bx_snd_acc[cls] += (uae_u64)bx_snd_q32[cls] * bx_snd_rate * 8;
		bx_acc[cls] += (int)(bx_snd_acc[cls] >> 32);
		bx_snd_acc[cls] &= 0xffffffffULL;
	}
	bx_acc[cls] += bx_st8[cls];
	n = bx_acc[cls] >> 3;
	bx_acc[cls] &= 7;
	return n;
}

static int bx_st(int size, bool write)
{
	if (write && size == 4)
		return 0;		/* the halves' wait: cpu030t_stram_lw_clocks() */
	return bx_take(write ? BX_WR : (size == 4 ? BX_RDL : BX_RD));
}

/* Clocks between the word halves of a long ST-RAM write. */
int cpu030t_stram_lw_clocks(void)
{
	if (!cpu030t_bus_on)
		return cpu030t_stram_long_write;
	return bx_take(BX_LW);
}

struct bx_dev { uae_u32 lo, hi; uae_s8 rd, wr, rdl, wrl; };
static const struct bx_dev bx_devs[] = {
	{ 0xff8600, 0xff860f, 11, 11, 22, 22 },	/* FDC / DMA */
	{ 0xff8800, 0xff88ff,  6,  4, 12,  8 },	/* YM2149 */
	{ 0xff8900, 0xff89ff,  5,  3,  6,  4 },	/* DMA sound, crossbar */
	{ 0xff8c80, 0xff8cff, 10, 10, 20, 20 },	/* SCC */
	{ 0xffa200, 0xffa2ff,  0,  0,  0,  0 },	/* DSP host port: dspmark-calibrated */
	{ 0xfffa00, 0xfffa3f,  9,  9, 18, 18 },	/* MFP */
	{ 0xfffc00, 0xfffc3f,  2,  2,  4,  4 },	/* ACIAs (the core's E-clock sync gives the rest) */
	{ 0xff8000, 0xffffff,  4,  2,  5,  3 },	/* everything else: Videl, palettes, Blitter, ... */
};

int cpu030t_bus_extra(uaecptr addr, int size, bool write)
{
	uae_u32 a;
	unsigned i;

	if ((addr >> 24) != 0 && (addr >> 24) != 0xff)
		return 0;			/* TT-RAM and beyond */
	a = addr & 0x00ffffff;
	if (a < STRamEnd) {
		if (bx_st_pre)
			return 0;		/* charged by cpu030t_bus_pre_extra() */
		return bx_st(size, write);
	}
	if (a >= 0xe00000 && a < 0xf00000)
		return write ? 0 : (size == 4 ? bx_rom_l : bx_rom_w);
	if (a < 0xff8000)
		return 0;
	for (i = 0; i < sizeof bx_devs / sizeof bx_devs[0]; i++) {
		const struct bx_dev *d = &bx_devs[i];
		if (a >= d->lo && a <= d->hi) {
			if (size == 4)
				return write ? d->wrl : d->rdl;
			return write ? d->wr : d->rd;
		}
	}
	return 0;
}

int cpu030t_bus_pre_extra(uaecptr addr, int size, bool write)
{
	uae_u32 a;

	if ((addr >> 24) != 0 && (addr >> 24) != 0xff)
		return 0;
	a = addr & 0x00ffffff;
	if (a >= STRamEnd)
		return 0;
	bx_snd_rate = bx_snd_on ? Crossbar_DmaByteRate() : 0;
	return bx_st(size, write);
}

static void bus_init(void)
{
	const char *env = getenv("HATARI_F030_BUS");

	cpu030t_bus_on = false;
	cpu030t_bus_pre = 0;
	if (!cpu030t_enabled || !Config_IsMachineFalcon())
		return;
	if (env && strcmp(env, "off") == 0)
		return;
	/* HATARI_F030_ST=read,longread,write,longhalves,pre: the ST-RAM extras in
	 * eighths of a clock, and 1 to charge them before the access's bus-slot
	 * alignment (the default), 0 after it */
	env = getenv("HATARI_F030_ST");
	if (env)
		sscanf(env, "%d,%d,%d,%d,%d", &bx_st8[BX_RD], &bx_st8[BX_RDL], &bx_st8[BX_WR],
		       &bx_st8[BX_LW], &bx_st_pre);
	memset(bx_acc, 0, sizeof bx_acc);
	env = getenv("HATARI_F030_SND");
	bx_snd_on = !(env && strcmp(env, "off") == 0);
	if (env && bx_snd_on)
		sscanf(env, "%u,%u,%u,%u", &bx_snd_q32[0], &bx_snd_q32[1], &bx_snd_q32[2], &bx_snd_q32[3]);
	memset(bx_snd_acc, 0, sizeof bx_snd_acc);
	cpu030t_bus_pre = bx_st_pre;
	cpu030t_bus_on = true;
	Log_Printf(LOG_INFO, "Falcon bus: f030probe-measured extras on (ST-RAM read +%d/8, long read +%d/8, write +%d/8%s, long write halves +%d/8 clocks; ROM, I/O per device); HATARI_F030_BUS=off disables\n",
		   bx_st8[BX_RD], bx_st8[BX_RDL], bx_st8[BX_WR], bx_st_pre ? " before the slot" : "",
		   bx_st8[BX_LW]);
}

enum { K_NONE, K_PLAIN, K_BCC, K_DBCC };

struct t030 {
	uae_s8 kind;
	uae_s8 ea_h, ea_t, ea_c;	/* effective address part */
	uae_s8 ea_read;			/* the EA part reads its operand (rule 1a) */
	uae_s8 op_h, op_t, op_c;	/* operation part */
	uae_s8 head_add;		/* op head added to the EA head (gencpu's SETCE020H) */
};

static struct t030 *model;
static int tail_prev;			/* clocks */
static int wait_read, wait_write, wait_last_write;
static bool wrote;
static int trace_opcode = -1, trace_left;	/* HATARI_CPU030_TIMING_TRACE=opcode,count */

/* Per-mode (h, t, c) triples, indexed by amodes. */
struct htc { int h, t, c, oph; };

static struct htc fea(amodes m, wordsizes sz)
{
	switch ((int)m) {
	case Aind: return (struct htc){ 1, 1, 3, 0 };
	case Aipi: return (struct htc){ 0, 1, 3, 0 };
	case Apdi: return (struct htc){ 2, 2, 4, 0 };
	case Ad8r: case PC8r: return (struct htc){ 4, 2, 6, 0 };
	case Ad16: case PC16: return (struct htc){ 2, 2, 4, 0 };
	case absw: return (struct htc){ 2, 2, 4, 0 };
	case absl: return (struct htc){ 1, 0, 4, 0 };
	case imm: case imm0: case imm1: case imm2:
		return sz == sz_long ? (struct htc){ 4, 0, 4, 0 } : (struct htc){ 2, 0, 2, 0 };
	/* immi is a quick immediate inside the opcode word (MOVEQ, ADDQ, ...):
	 * nothing to fetch */
	default: return (struct htc){ 0, 0, 0, 0 };
	}
}

static struct htc fiea(amodes m, wordsizes sz)
{
	bool l = sz == sz_long;
	switch ((int)m) {
	case Dreg: case Areg: return l ? (struct htc){ 4, 0, 4, 1 } : (struct htc){ 2, 0, 2, 1 };
	case Aind: return l ? (struct htc){ 1, 0, 4, 0 } : (struct htc){ 1, 1, 3, 0 };
	case Aipi: return l ? (struct htc){ 4, 1, 7, 0 } : (struct htc){ 2, 1, 5, 0 };
	case Apdi: return l ? (struct htc){ 2, 0, 4, 0 } : (struct htc){ 2, 2, 4, 0 };
	case Ad8r: case PC8r: return l ? (struct htc){ 8, 2, 10, 0 } : (struct htc){ 6, 2, 8, 0 };
	case Ad16: case PC16: return l ? (struct htc){ 4, 0, 6, 0 } : (struct htc){ 2, 0, 4, 0 };
	case absw: return l ? (struct htc){ 6, 2, 8, 0 } : (struct htc){ 4, 2, 6, 0 };
	case absl: return l ? (struct htc){ 5, 0, 8, 0 } : (struct htc){ 3, 0, 6, 0 };
	default: return (struct htc){ 0, 0, 0, 0 };
	}
}

static struct htc ciea(amodes m, wordsizes sz)
{
	bool l = sz == sz_long;
	switch ((int)m) {
	case Dreg: case Areg: case Aind: case Apdi:
		return l ? (struct htc){ 4, 0, 4, 1 } : (struct htc){ 2, 0, 2, 1 };
	case Aipi: return l ? (struct htc){ 4, 0, 6, 0 } : (struct htc){ 2, 0, 4, 0 };
	case Ad8r: case PC8r: return l ? (struct htc){ 8, 0, 8, 1 } : (struct htc){ 6, 0, 6, 1 };
	case Ad16: case PC16: case absw:
		return l ? (struct htc){ 6, 0, 6, 1 } : (struct htc){ 4, 0, 4, 1 };
	case absl: return l ? (struct htc){ 8, 0, 8, 1 } : (struct htc){ 6, 0, 6, 1 };
	default: return (struct htc){ 0, 0, 0, 0 };
	}
}

static struct htc cea(amodes m)
{
	switch ((int)m) {
	case Aind: return (struct htc){ 2, 0, 2, 1 };
	case Aipi: return (struct htc){ 0, 0, 2, 0 };
	case Apdi: return (struct htc){ 2, 0, 2, 1 };
	case Ad8r: case PC8r: return (struct htc){ 4, 0, 4, 1 };
	case Ad16: case PC16: case absw: return (struct htc){ 2, 0, 2, 1 };
	case absl: return (struct htc){ 4, 0, 4, 1 };
	default: return (struct htc){ 0, 0, 0, 0 };
	}
}

static struct htc jea(amodes m)
{
	switch ((int)m) {
	case Aind: case absw: case absl: return (struct htc){ 2, 0, 2, 1 };
	case Ad16: case PC16: return (struct htc){ 4, 0, 4, 1 };
	default: return (struct htc){ 0, 0, 0, 0 };
	}
}

static bool isreg(amodes m) { return m == Dreg || m == Areg; }
static bool isimm(amodes m) { return m >= imm && m <= immi; }

/* MC68030UM 11.6.6: MOVE destination part; reg = source is a register. */
static struct htc move_dest(amodes d, bool reg)
{
	switch ((int)d) {
	case Dreg: case Areg: return reg ? (struct htc){ 2, 0, 2, 0 } : (struct htc){ 0, 0, 2, 0 };
	case Aind: return reg ? (struct htc){ 0, 1, 3, 0 } : (struct htc){ 2, 0, 4, 0 };
	case Aipi: return reg ? (struct htc){ 0, 1, 3, 0 } : (struct htc){ 2, 0, 4, 0 };
	case Apdi: return reg ? (struct htc){ 0, 2, 4, 0 } : (struct htc){ 2, 0, 4, 0 };
	case Ad16: case absw: return (struct htc){ 2, 0, 4, 0 };
	case absl: return (struct htc){ 0, 0, 6, 0 };
	case Ad8r: return (struct htc){ 4, 0, 6, 0 };
	default: return (struct htc){ 0, 0, 0, 0 };
	}
}

static void build_model(void)
{
	int opc;

	model = calloc(65536, sizeof *model);
	if (!model)
		return;
	for (opc = 0; opc < 65536; opc++) {
		struct instr *in = &table68k[opc];
		struct t030 *t = &model[opc];
		struct htc e = { 0, 0, 0, 0 }, o = { 0, 0, 0, 0 };
		bool has_ea = false;

		if (in->mnemo == i_ILLG || in->clev > 3)
			continue;
		switch (in->mnemo) {
		case i_Bcc:
			t->kind = K_BCC;
			continue;
		case i_DBcc:
			t->kind = K_DBCC;
			continue;
		case i_MOVE:
		case i_MOVEA:
			/* MOVEQ is MOVE.L with a quick immediate: the manual's
			 * MOVEQ row (2, head 2) is MOVE Rn,Dn's */
			if (!isreg(in->smode) && in->smode != immi) {
				e = fea(in->smode, in->size);
				has_ea = true;
			}
			o = move_dest(in->dmode, isreg(in->smode) || in->smode == immi);
			break;
		case i_ADDA:
		case i_SUBA:
		case i_CMPA:
			/* 11.6.8: Rn,An head 4 (.W) or 2 (.L; CMPA.L 4);
			 * EA,An head 0; CC 4 (.W) or 2 (.L); CMPA 4. */
			if (isreg(in->smode)) {
				o.h = (in->size == sz_long && in->mnemo != i_CMPA) ? 2 : 4;
				o.c = o.h;
			} else {
				e = fea(in->smode, in->size);
				has_ea = true;
				o.h = 0;
				o.c = (in->size == sz_long && in->mnemo != i_CMPA) ? 2 : 4;
			}
			o.t = 0;
			o.oph = 0;
			break;
		default:
			if (in->clocks <= 0 && in->head <= 0 && in->tail <= 0 && !in->fetchmode)
				continue;	/* no data in table68k: leave the core's timing alone */
			o = (struct htc){ in->head < 0 ? 0 : in->head, in->tail, in->clocks, 0 };
			switch (in->fetchmode) {
			case 1:		/* fea */
				if (isimm(in->smode) || isreg(in->smode) || in->smode == am_unknown)
					e = fea(in->dmode, in->size);
				else
					e = fea(in->smode, in->size);
				has_ea = true;
				break;
			case 2: e = cea(in->smode); has_ea = true; break;
			case 3: e = fiea(in->dmode, in->size); has_ea = true; break;
			case 4: e = ciea(in->dmode, in->size); has_ea = true; break;
			case 5: e = jea(in->smode); has_ea = true; break;
			}
			break;
		}
		t->kind = K_PLAIN;
		t->ea_h = e.h; t->ea_t = e.t; t->ea_c = e.c;
		t->ea_read = has_ea && e.c > 0 && in->fetchmode != 2 && in->fetchmode != 5;
		t->head_add = e.oph;
		t->op_h = o.h; t->op_t = o.t; t->op_c = o.c;
	}
}

void cpu030t_init(void)
{
	const char *env = getenv("HATARI_CPU030_TIMING");
	bool asked;

	cpu030t_enabled = false;
	cpu030t_bus_on = false;
	cpu030t_stram_long_write = 0;
	/* On by default on a Falcon; HATARI_CPU030_TIMING=off gives the bare
	 * cycle-exact core, =manual turns the model on for any 68030 machine */
	if (env && strcmp(env, "off") == 0)
		return;
	asked = env && strcmp(env, "manual") == 0;
	if (!asked && !Config_IsMachineFalcon())
		return;
	/* (with --mmu too since 2026-11-24: m68k_run_mmu030 calls the same hooks) */
	if (currprefs.cpu_model != 68030 || !currprefs.cpu_cycle_exact) {
		if (asked)
			Log_Printf(LOG_WARN, "HATARI_CPU030_TIMING=manual needs a 68030 in cycle-exact mode; ignored\n");
		return;
	}
	if (!model)
		build_model();
	if (!model)
		return;
	tail_prev = 0;
	wait_read = wait_write = wait_last_write = 0;
	wrote = false;
	env = getenv("HATARI_CPU030_TIMING_TRACE");
	if (env && sscanf(env, "%x,%d", &trace_opcode, &trace_left) != 2)
		trace_opcode = -1;
	cpu030t_enabled = true;
	if (Config_IsMachineFalcon()) {
		cpu030t_stram_long_write = 2;
		env = getenv("HATARI_CPU030_STRAM_WR");
		if (env)
			cpu030t_stram_long_write = atoi(env);
	}
	Log_Printf(LOG_INFO, "68030 timing: MC68030 User's Manual instruction-cache-case model, ST-RAM long write +%d clocks (HATARI_CPU030_TIMING=off for the bare core)\n",
		   cpu030t_stram_long_write);
	bus_init();
}

/* A data access of the bus model, from start to end in cycle units.
 * An instruction that touches an I/O register (the Falcon's $FF8000-$FFFFFF
 * page, in 24- or 32-bit form) keeps the fork's own timing and is not
 * padded: the host-port and I/O wait states were calibrated against
 * hardware as whole-instruction times in a core without internal clocks,
 * and padding shifts the bus-slot phase they depend on (dspmark: +3 to +4
 * clocks per host-port read when padded). */
static bool io_touched;

void cpu030t_access(uaecptr addr, evt_t start, evt_t end, bool write)
{
	int w = (int)((end - start) / cpucycleunit) - 2;

	if ((addr & 0x00ff8000) == 0x00ff8000 && (addr >> 24 == 0 || addr >> 24 == 0xff))
		io_touched = true;
	if (w < 0)
		w = 0;
	if (write) {
		wait_write += w;
		wait_last_write = w;
		wrote = true;
	} else {
		wait_read += w;
	}
}

/* An instruction an MMU fault cut short: its accesses do not carry into the
 * exception handler's first instruction, and nothing overlaps across it. */
void cpu030t_abort(void)
{
	wait_read = wait_write = wait_last_write = 0;
	wrote = false;
	io_touched = false;
	tail_prev = 0;
}

void cpu030t_instruction_end(uae_u16 opcode, uaecptr instruction_pc)
{
	const struct t030 *t = &model[opcode];
	int cc, head, tail, target, elapsed;

	if (io_touched) {
		tail_prev = 0;
		goto done;
	}
	switch (t->kind) {
	case K_BCC: {
		int disp = opcode & 0xff;
		int len = disp == 0 ? 4 : disp == 0xff ? 6 : 2;
		bool taken = m68k_getpc() != instruction_pc + len;
		cc = taken ? 6 : (len == 2 ? 4 : 6);
		head = cc;
		tail = 0;
		break;
	}
	case K_DBCC: {
		bool taken = m68k_getpc() != instruction_pc + 4;
		if (taken)
			cc = 6;
		else
			cc = ((m68k_dreg(regs, opcode & 7) & 0xffff) == 0xffff) ? 10 : 6;
		head = cc;
		tail = 0;
		break;
	}
	case K_PLAIN: {
		int ea_t = t->ea_t + (t->ea_read ? wait_read : 0);
		int ov = t->op_h < ea_t ? t->op_h : ea_t;
		cc = t->ea_c + t->op_c - ov + wait_read + wait_write;
		head = t->ea_c ? t->ea_h + (t->head_add ? t->op_h : 0) : t->op_h;
		tail = t->op_t + (wrote ? wait_last_write : 0);
		break;
	}
	default:
		tail_prev = 0;
		goto done;
	}
	target = cc - (head < tail_prev ? head : tail_prev);
	elapsed = (int)(currcycle / cpucycleunit);
	if (opcode == trace_opcode && trace_left > 0 && instruction_pc < 0x00e00000) {
		trace_left--;
		fprintf(stderr, "cpu030t %04x pc=%08x cc=%d head=%d tail_prev=%d target=%d elapsed=%d (units %ld) wr=%d ww=%d tail=%d\n",
			opcode, instruction_pc, cc, head, tail_prev, target, elapsed, (long)currcycle, wait_read, wait_write, tail);
	}
	/* Straight into this instruction's cycle count: x_do_cycles would hold
	 * back an odd clock until the next call, and the next instruction's
	 * elapsed time would then miss it. */
	if (target > elapsed)
		currcycle += (evt_t)(target - elapsed) * cpucycleunit;
	tail_prev = tail;
done:
	wait_read = wait_write = wait_last_write = 0;
	wrote = false;
	io_touched = false;
}
