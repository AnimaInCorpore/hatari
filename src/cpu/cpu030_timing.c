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

bool cpu030t_enabled;
int cpu030t_stram_long_write;

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
	cpu030t_stram_long_write = 0;
	/* On by default on a Falcon; HATARI_CPU030_TIMING=off gives the bare
	 * cycle-exact core, =manual turns the model on for any 68030 machine */
	if (env && strcmp(env, "off") == 0)
		return;
	asked = env && strcmp(env, "manual") == 0;
	if (!asked && !Config_IsMachineFalcon())
		return;
	if (currprefs.cpu_model != 68030 || !currprefs.cpu_cycle_exact || currprefs.mmu_model) {
		if (asked)
			Log_Printf(LOG_WARN, "HATARI_CPU030_TIMING=manual needs a 68030 in cycle-exact mode without MMU; ignored\n");
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
