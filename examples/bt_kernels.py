#!/usr/bin/env python3
"""qubic3 BT (PLAN_BT_v2 §1 T6): the joint board session's kernels, compiled on my_office at S0 into
`progs_bt.pkl` (the G6' two-stage compile), and run on veneno by the BT kit's `bt_session.py`:

    python examples/bt_kernels.py compile --config software/configs/zcu216-14q-antq.json \
        --cal bt_cal14.yaml --out progs_bt.pkl

Every program fires only the demod channel (14q channel 2, no DAC), except the heralded `k_batched`,
whose DAC-bound pulses all play at amplitude 0 (`Experiment(rf_silent=True)` on an RF-silent config).

  k_bt        P4's `k_grid` structure: every core starts from one barrier, plays one demod per shot on
              the grid t0 + period + k·period, checks the stop words at the top of every shot,
              publishes [shots, reads] after its post, waits out its last demod and stores fin.
              `StopConvention("n", at=True, lead=1, reads_per_shot=1)`. It publishes
              `bt_t` = [t0, the end of its last demod, now() at fin] and measures its own C3 slack:
              right after each post it reads now() and keeps d = (t - LEAD) - now() (`slack` =
              [min d, posts, then a 16-bin histogram: bin 0 counts d < 0, bin i the d in
              [32(i - 1), 32i), bin 15 everything from 448 on]). now() after the post is never
              earlier than the post, so d is conservative. Runtime `hang` = 1 halts after the loop.
  k_bt_poll   `k_bt` that also reads `word[0]` every shot, after its post (`poll` = 1, compile-time):
              `pollc` = [values other than 0 and 0xFFFF_FFFF, value changes] (B4b's collision hammer).
  k_nomark    no completion marker, so its run is UNPROVEN: one demod read now, and one more due
              `ahead` batches later (1 s), never waited for; its absolute start time is out[1]. The
              flush before the next release must cancel it.
  k_batched   P6 C2's dual capture (mode RAW in core RAM, rerun through the uplink), and the heralded
              COUNTS kernel. The heralded one is `riscq.cal.batched.k_batched` itself; its sequence
              header's `seq_shot` gains one now() read after the drive's posts (`slack_header`), into
              `bt_slack` = [min d, posts, histogram as above, the first drive's deadline (its release
              phase)], with d = (the first drive pulse's start - LEAD) - now(): the first pulse starts at
              t_ro - SEP - seq_len (`base.herald_offset`). Its timing is PROVEN only if min d + LEAD
              covers POST_LEAD plus its posting path's bound and HOST_MARGIN (`herald_timing`).

C3 (PLAN_BT_v2 T6, P4 REPORT §3.4): B_C3 = 9 cycles per conditional branch on the posting path plus
5 (one AT request's stalls). `posting_branches` takes n from the disassembly of the compiled image:
the most conditional branches on any control-flow path from the shot loop's result read to the next
post's first store.
`designed_slack` is the posting margin the kernel's constants give at a period P: shot k + 1 is posted
when shot k's result returns, READOUT_LEAD + RESULT_LATENCY after t_k, and is due at t_k + P - LEAD."""

from __future__ import annotations

import argparse
import pickle
import re
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software"))

from riscq.lang import Array, Group, ParamTable, StopConvention, compile_kernel, kernel  # noqa: E402
from riscq.map import LEAD, READOUT_LEAD, SocMap, SocParams, pack16  # noqa: E402

DUR = 40                 # the demod window, batches (one batch = one dspClk cycle, 2 ns at 500 MHz)
DEMOD_FREQ_HZ = 50e6
BATCHES_PER_US = 500
AHEAD_1S = 500_000_000   # k_nomark: one demod due 1 s later
NBINS, BIN_LOG2 = 16, 5
SLACK_WORDS = 2 + NBINS  # [min d, posts, 16 bins]
BRANCH_CYCLES, AT_STALL = 9, 5     # P4 REPORT §3.4: per mispredicted branch; one AT request (M1)
RESULT_LATENCY = 30      # batches from a demod window's end-of-lead to its result at the sink (<= 29 measured)
POST_LEAD = 15           # batches: the shortest lead from now() after a gate post to its start at which the pulse
                         # still starts on time; shorter ones start late or drop (co-sim: test_bt_cosim's lead sweep
                         # asserts this value; the post itself takes ~29 cycles)
HOST_MARGIN = 2          # cycles: host accesses that can stall a herald's posting window (M1: one cycle each; a
                         # heralded run has no policy, its host polls HOST_DONE outside the core's RAM)


@kernel
def k_bt(demod: ParamTable, grp: Group, rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array,
         bt_t: Array, slack: Array, word: Array, pollc: Array, code: int, poll: int, n: int, period: int,
         hang: int):
    """The BT grid kernel (module docstring). `poll` is compile-time (k_bt_poll); `n`, `period` and
    `hang` are runtime params."""
    init_pulse_params(demod.pulses)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    t = barrier(grp) + period  # noqa: F821   (the pre-loop rendezvous: one t0 for every core)
    t0 = t
    e = rq_epoch
    s = 0
    t_end = 0
    dmin = 1073741824
    odd = 0
    flips = 0
    prev = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        play(demod, demod["sq"], t)  # noqa: F821
        d = t - LEAD - now()  # noqa: F821   (the post's slack; now() is after it, so d is conservative)
        if d < dmin:
            dmin = d
        if d < 0:
            b = 0
        else:
            b = (d >> BIN_LOG2) + 1
            if b > NBINS - 1:
                b = NBINS - 1
        slack[2 + b] = slack[2 + b] + 1
        t_end = t + DUR
        s = s + 1
        rq_status[0] = s
        rq_status[1] = s
        if poll == 1:
            v = word[0]
            if v != 0 and v != -1:
                odd = odd + 1
            if v != prev:
                flips = flips + 1
            prev = v
        wait_until(t + READOUT_LEAD)  # noqa: F821
        read_res()  # noqa: F821
        t = t + period
    if hang == 1:
        wait_until(now() + 1073741824)  # noqa: F821
    if s > 0:
        wait_until(t_end + LEAD)  # noqa: F821   (the last demod has ended)
        read_res()  # noqa: F821
    slack[0] = dmin
    slack[1] = s
    if poll == 1:
        pollc[0] = odd
        pollc[1] = flips
    bt_t[0] = t0
    bt_t[1] = t_end
    bt_t[2] = now()  # noqa: F821
    rq_status[2] = e


@kernel
def k_nomark(demod: ParamTable, out: Array, code: int, ahead: int):
    """UNPROVEN on purpose: no completion marker. One demod now, read; one more due `ahead` batches
    later, not waited for, its absolute start time (the 32-bit batch time of this run's time base) in
    out[1] (phase 14: the flush before the next release must cancel it, and the kit observes past
    that time in both time bases)."""
    init_pulse_params(demod.pulses)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    t = now() + LEAD  # noqa: F821
    play(demod, demod["sq"], t)  # noqa: F821
    wait_until(t + READOUT_LEAD)  # noqa: F821
    out[0] = read_res()  # noqa: F821
    out[1] = t + ahead
    play(demod, demod["sq"], t + ahead)  # noqa: F821


def demod_table(m, core):
    from riscq.pulses import Pulse, envelopes
    return ParamTable(m.channel_named("demod", core), 0.0, {"sq": Pulse(envelopes.square(DUR), amp=1.0)})


def demod_code(m):
    from riscq.pulses import units
    return pack16(units.demod_freq_to_code(DEMOD_FREQ_HZ, m.params))


def compile_bt(m, cores, poll=0):
    """`k_bt` (poll 0) or `k_bt_poll` (poll 1) on `cores`, one barrier group of exactly those cores."""
    conv = StopConvention("n", at=True, lead=1, reads_per_shot=1)
    grp = Group(sorted(cores), id=0)
    return {c: compile_kernel(k_bt, m, core=c, tables=dict(demod=demod_table(m, c)), grp=grp,
                              rq_status=Array(3), bt_t=Array(3), slack=Array(SLACK_WORDS),
                              word=Array(1, input=True), pollc=Array(2), code=demod_code(m), poll=poll,
                              stop=conv)
            for c in sorted(cores)}


def compile_nomark(m, cores):
    return {c: compile_kernel(k_nomark, m, core=c, tables=dict(demod=demod_table(m, c)), out=Array(2),
                              code=demod_code(m)) for c in sorted(cores)}


def designed_slack(period: int) -> int:
    """The posting margin of `k_bt` at `period` batches, from its constants: shot k + 1 is posted when
    shot k's result returns, about READOUT_LEAD + RESULT_LATENCY after t_k, and is due at
    t_{k+1} - LEAD = t_k + period - LEAD."""
    return period - LEAD - READOUT_LEAD - RESULT_LATENCY


# ── C3: the conditional branches on the posting path, from the disassembly ───────────────────────

COND = {"beq", "bne", "blt", "bge", "bltu", "bgeu"}
STORES = {"sw", "sh", "sb"}
OBJDUMP = "riscv32-unknown-elf-objdump"
RES_OFFSET = 0x200       # read_res(): a load from the core's result sink, CTRL_RES = control base + 0x200


def disassemble(prog, objdump: str | None = None) -> list:
    """`main` of the flat image, disassembled at its load address (the build's ISA, rv32i): [(addr,
    mnemonic, operands)]."""
    import shutil
    tool = objdump or shutil.which(OBJDUMP) or str(Path.home() / "opt/riscv/bin" / OBJDUMP)
    lo, size = prog.image.symbols["main"]
    with tempfile.NamedTemporaryFile(suffix=".bin") as f:
        f.write(prog.image.data)
        f.flush()
        r = subprocess.run([tool, "-D", "-b", "binary", "-m", "riscv:rv32", "-M", "no-aliases,numeric",
                            f"--adjust-vma=0x{prog.image.entry:x}", f.name], capture_output=True, text=True,
                           check=True)
    out = []
    for line in r.stdout.splitlines():
        mm = re.match(r"^\s*([0-9a-f]+):\s+[0-9a-f]{8}\s+(\S+)\s*(.*?)\s*(#.*)?$", line)
        if mm and lo <= int(mm.group(1), 16) < lo + size:
            out.append((int(mm.group(1), 16), mm.group(2), mm.group(3)))
    return out


def _target(args: str) -> int:
    return int(args.rsplit(",", 1)[-1], 16)


def _tested(ins, i) -> bool:
    """Whether the result loaded by ins[i] is compared by the first conditional branch after it, with no
    store in between (the heralded k_batched's `h = read_res(); if (h == 0) ...`)."""
    rd = ins[i][2].split(",", 1)[0]
    for _, op, x in ins[i + 1:]:
        if op in STORES:
            return False
        if op in COND:
            return rd in x.split(",")[:2]
    return False


def posting_branches(prog, objdump: str | None = None, herald: bool = False) -> dict:
    """The conditional branches between a shot's result read and the next post (PLAN_BT_v2 T6, P4
    REPORT §3.4): from the shot loop's result read (`lw rd, 0x200(rs)`, CTRL_RES) every control-flow
    path is followed to its first store (the post's first write), within the shot loop (the span of
    the backward branches and jumps of `main`); the most conditional branches on any such path is n,
    and B_C3 = 9 n + 5. k_bt has exactly one result read in its shot loop. `herald` (the heralded
    k_batched, two reads per shot) starts instead at the herald read, the one whose value the next
    conditional branch tests: its drive is posted after it."""
    ins = disassemble(prog, objdump)
    at = {a: i for i, (a, _, _) in enumerate(ins)}
    back = [(_target(x), a) for a, op, x in ins if (op in COND or op == "jal") and _target(x) < a]
    if not back:
        raise ValueError("no loop in main")
    lo, hi = min(t for t, _ in back), max(a for _, a in back)
    reads = [i for i, (a, op, x) in enumerate(ins) if op == "lw" and lo <= a <= hi
             and re.match(rf"x\d+,{RES_OFFSET}\(x\d+\)$", x)]
    if herald:
        reads = [i for i in reads if _tested(ins, i)]
    if len(reads) != 1:
        raise ValueError(f"expected one {'tested ' if herald else ''}result read in the shot loop, found {len(reads)}")
    best, paths = -1, []

    def walk(i, n, seen, path):
        nonlocal best
        a, op, x = ins[i]
        if not lo <= a <= hi or a in seen:               # left the loop, or looped back: not a post
            return
        if op in STORES:
            paths.append((n, path + [f"0x{a:x} {op}"]))
            best = max(best, n)
            return
        if op in COND:
            walk(at[_target(x)], n + 1, seen | {a}, path + [f"0x{a:x} {op}"])
            walk(i + 1, n + 1, seen | {a}, path + [f"0x{a:x} {op}"])
        elif op == "jal" and x.startswith("x0,"):
            walk(at[_target(x)], n, seen | {a}, path)
        elif op not in ("jal", "jalr"):
            walk(i + 1, n, seen | {a}, path)
    walk(reads[0] + 1, 0, set(), [])
    if best < 0:
        raise ValueError("no store follows the result read inside the shot loop")
    worst = max(paths)[1]
    return {"loop": [f"0x{lo:x}", f"0x{hi:x}"], "result_read": f"0x{ins[reads[0]][0]:x}",
            "conditional_branches": best, "B_C3": BRANCH_CYCLES * best + AT_STALL, "worst_path": worst}


# ── the heralded k_batched: one now() read after the drive's posts ────────────────────────────────

HERALD_WORDS = SLACK_WORDS + 1    # bt_slack: k_bt's slack words, then the first drive's deadline (its release phase)
SLACK_HDR = """
volatile int32_t bt_slack[%(words)d];
static inline void bt_note(int32_t deadline) {
    int32_t d = deadline - (int32_t)now();
    int32_t b;
    if (bt_slack[1] == 0) { bt_slack[0] = d; bt_slack[%(first)d] = deadline; }
    else if (d < bt_slack[0]) bt_slack[0] = d;
    bt_slack[1] = bt_slack[1] + 1;
    if (d < 0) b = 0; else { b = (d >> %(log2)d) + 1; if (b > %(last)d) b = %(last)d; }
    bt_slack[2 + b] = bt_slack[2 + b] + 1;
}
"""


def slack_header(hdr: str, lead_before_t_ro: int) -> str:
    """The generated sequence header with `seq_shot` ending in `bt_note(t_ro - lead_before_t_ro -
    LEAD)`: the drive's posting deadline, its first pulse starting `lead_before_t_ro` batches before
    t_ro (SEP + seq_len)."""
    sig = "static inline void seq_shot(uint32_t t_ro,"
    i = hdr.index(sig)
    j = hdr.index("\n}", i)
    note = f"    bt_note((int32_t)t_ro - {int(lead_before_t_ro)} - {LEAD});"
    prelude = SLACK_HDR % {"words": HERALD_WORDS, "first": HERALD_WORDS - 1, "log2": BIN_LOG2, "last": NBINS - 1}
    return hdr[:i] + prelude + hdr[i:j] + "\n" + note + hdr[j:]


def herald_timing(min_slack, timing: dict) -> dict:
    """The heralded k_batched's drive-post timing (Codex BT kit gate r1 #4): its post lead is min_slack +
    LEAD (the first drive pulse starts LEAD after the deadline its slack is measured against, and the
    slack is read after the posts). PROVEN only if that lead covers the hardware's post lead (POST_LEAD,
    measured in co-sim on a gate channel and by the non-silent twin) plus the posting path's predictor
    bound (B, from the disassembly, as B_C3) and the host stalls (HOST_MARGIN); otherwise UNPROVEN,
    never a pass."""
    need = int(timing["post_lead"]) + int(timing["B"]) + int(timing["host_margin"])
    lead = None if min_slack is None else int(min_slack) + LEAD
    ok = lead is not None and lead >= need
    return {"lead": lead, "required": need, "verdict": "PROVEN" if ok else
            f"UNPROVEN: lead {lead} < {need} (post lead {timing['post_lead']} + B {timing['B']} + host "
            f"{timing['host_margin']})" if lead is not None else "UNPROVEN: no drive was posted"}


def herald_c3(prog, objdump: str | None = None) -> dict:
    """The heralded image's posting-path bound: the most conditional branches on any path from its herald
    read to a first store (the drive's post, or the next shot's herald readout when the herald fails), as
    `posting_branches`, with the post lead and host margin the timing rule adds."""
    c3 = posting_branches(prog, objdump, herald=True)
    return {"branches": c3["conditional_branches"], "B": c3["B_C3"], "worst_path": c3["worst_path"],
            "post_lead": POST_LEAD, "host_margin": HOST_MARGIN}


def compile_experiment(exp, m, herald_slack=False):
    """Compile an Experiment offline (its `rq.setup` lands in bt_record's RecordingDriver) and return its
    programs (with their C sources: the RF audit reads them), the params of its first rerun, its timeout
    and its point count. `herald_slack` instruments the heralded sequence header."""
    from riscq import run as rq
    from riscq.cal import base, experiment as X
    from bt_record import RecordingDriver
    drv = RecordingDriver(m)                   # its `rq.setup` lands in the recording remote; nothing runs
    orig, orig_setup, srcs = X.emit_header, rq.setup, {}
    if herald_slack:
        def emit(comp, core, meas, label=""):
            return slack_header(orig(comp, core, meas, label), base.SEP + comp.seq_len)
        X.emit_header = emit

    def setup(d, mm, ps):                      # the sources do not cross the wire: keep them from the setup
        if d is drv:
            srcs.clear()
            srcs.update({int(c): p.c_source for c, p in ps.items()})
        return orig_setup(d, mm, ps)
    rq.setup = setup
    try:
        progs, _signs, timeout = exp.compile(drv)
    finally:
        X.emit_header, rq.setup = orig, orig_setup
    wire = [c for c in drv.remote.calls if c["op"] == "setup"][-1]["args"]["progmap"]
    progs = {int(c): rq._prog_from_wire(w) for c, w in wire.items()}
    for c, p in progs.items():
        p.c_source = srcs.get(c)
    comp, axes, rcore, npts = exp.compiled[exp.keys[0]]
    words = exp._pairs(axes, ())
    params = {c: {k: v for k, v in words.items() if k in progs[c].params} for c in progs}
    return progs, params, timeout, npts


def cal_config(path):
    from riscq.cal.config import Config
    return Config.load(path)


def experiments(cfg, m, cores):
    """The two k_batched variants on `cores` (RF-silent): P6 C2's dual capture (RAW in core RAM, 64
    shots) and the heralded COUNTS kernel (one point, `shots` shots)."""
    from riscq.cal.experiment import Experiment
    from riscq.cal.measure import Measure
    qs = sorted(cores)
    dual = Experiment(cfg, qs, {q: [] for q in qs}, {q: () for q in qs}, (), Measure.raw(host=False), 64,
                      label="bt_dual", rf_silent=True)
    from riscq.cal.sequence import Gate
    herald = Experiment(cfg, qs, {q: [Gate("x90")] for q in qs}, {q: () for q in qs}, (),
                        Measure.counts(herald=True), 100, label="bt_herald", rf_silent=True)
    return dual, herald


# ── the compile stage ─────────────────────────────────────────────────────────────────────────────

def cmd_compile(args):
    from riscq import run as rq
    m = SocMap(SocParams.load(args.config))
    if not m.params.with_antq_uplink:
        sys.exit(f"{args.config}: results_path {m.params.results_path}, not antq_uplink")
    n = len(m.params.cores)
    every, low, high = range(n), range(n // 2), range(n // 2, n)
    progs = {"k_bt14": compile_bt(m, every), "k_bt_lo": compile_bt(m, low), "k_bt_poll14": compile_bt(m, every, 1),
             "k_nomark14": compile_nomark(m, every)}
    meta = {"k_nomark14": {"ahead": AHEAD_1S}}
    c3 = {k: posting_branches(p[0], args.objdump) for k, p in progs.items() if k.startswith("k_bt")}
    if args.cal:
        cfg = cal_config(args.cal)
        for name, cores in (("14", every), ("_hi", high)):
            dual, herald = experiments(cfg, m, cores)
            if name == "14":
                p, par, timeout, npts = compile_experiment(dual, m)
                progs["k_dual14"], meta["k_dual14"] = p, {"params": par, "timeout": timeout, "npts": npts,
                                                          "shots": dual.shots}
            p, par, timeout, npts = compile_experiment(herald, m, herald_slack=True)
            progs[f"k_herald{name}"], meta[f"k_herald{name}"] = p, {"params": par, "timeout": timeout,
                                                                    "npts": npts, "shots": herald.shots,
                                                                    "timing": herald_c3(p[min(p)], args.objdump)}
    srcs = {k: {c: p.c_source for c, p in v.items()} for k, v in progs.items()}
    missing = sorted(k for k, v in srcs.items() if any(not s for s in v.values()))
    if missing:                                # the RF audit fails without them (Codex BT kit gate r1 #6)
        sys.exit(f"no C source for {missing}: the RF audit needs every program's generated source")
    blob = {"config": Path(args.config).name, "c3": c3, "meta": meta,
            "designed_slack_1us": designed_slack(BATCHES_PER_US), "c_sources": srcs,
            "progs": {k: {c: rq._prog_to_wire(p) for c, p in v.items()} for k, v in progs.items()}}
    with open(args.out, "wb") as f:
        pickle.dump(blob, f)
    for k, v in progs.items():
        p = next(iter(v.values()))
        print(f"[BT] compiled {k}: cores {sorted(v)}, image {len(p.image.data)} B, runtime params "
              f"{sorted(x for x, y in p.params.items() if y is None)}" + (f", C3 {c3[k]}" if k in c3 else "")
              + (f", timing {meta[k]['timing']}" if "timing" in meta.get(k, {}) else ""))
    print(f"[BT] designed slack at 1 us: {blob['designed_slack_1us']} batches; C sources kept for {len(srcs)} "
          f"program sets; -> {args.out}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("compile")
    c.add_argument("--config", required=True)
    c.add_argument("--cal", default=None, help="the 14-qubit RF-silent cal config (bt_cal14.yaml)")
    c.add_argument("--out", required=True)
    c.add_argument("--objdump", default=None)
    a = ap.parse_args(argv)
    if a.cmd == "compile":
        cmd_compile(a)


if __name__ == "__main__":
    main()
