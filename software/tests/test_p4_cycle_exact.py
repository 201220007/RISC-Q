"""Cycle-exact co-sim tests of P4 (qubic3; plan P4 v2 §6 M1-M4, §8 C2, C10 and the `--slow`
full-period sweep; after-stage r1 #1), on the bench's deterministic scheduling.

`sim.lockstep(True)` keeps the bench from free-running between requests, so sim time moves only
inside them and host accesses land on the cycles chosen; a run released at the same batch-time phase
repeats cycle for cycle once the CPU's branch predictor has seen it
(`test_lockstep_repeats_a_run_cycle_for_cycle`).
`sim.sched` issues host accesses at absolute cycles. `Sched`, a driver wrapper, records the
release cycle R of a run, arms DAC captures there and issues scheduled accesses at R + offset:
traffic lands on the cycle chosen, in any run.

`k_probe` (two cores of `cosim`: sim-2q, or sim-2q-antq in the antq tier) is a stoppable shot loop
on one barrier grid that records, per shot s, `now()` at the top of the shot (just before its stop
check) and right after its posts (just before its count store). Its gate drive alternates two
amplitudes, so a fire posted late, which plays with the previous pulse's parameters, shows in the
DAC. The kernel's records come from the same run as the traffic, so the predictions below use the
real timeline, host stalls included.

- M1: stall cycles per host access type on port1: a write, a progress read and a read-back swept
  across the fixed instruction sequence between the top of a shot and its posts.
- M4: an epoch write swept cycle by cycle across a core's check load (it stops at S or S + 1, with
  one transition; the offset found on one shot predicts the transition on another), and a progress
  read swept across the count store (it returns the old or the new count, with one transition).
- C2: P4's issuer at 12 phases of one period: v_pre and v_post as the run's own record predicts
  them, TOO_LATE and VERIFIED exactly where the arithmetic says, VERIFIED runs FIRED; `--slow` does
  every cycle of the period.
- M3: the AT access pattern of a 14-core run (2 progress reads and 3 accesses per core; the other 13
  cores' publishes go to core 1's RAM) swept cycle by cycle across a shot's critical window, the
  gate DAC bit-exact against the no-traffic run at every offset.
- M2: the slack of k_probe's gate fire, the smallest added delay before the post that corrupts the
  pulse, minus one."""

import contextlib

import numpy as np
import pytest

from riscq import run as rq
from riscq import session as S
from riscq import stop as st
from riscq.lang import Array, Group, ParamTable, StopConvention, compile_kernel, kernel
from riscq.map import LEAD, READOUT_LEAD, pack16
from riscq.pulses import Pulse, envelopes, units

pytestmark = pytest.mark.cosim

G_DUR, DUR, F, P, NMAX = 16, 40, 1024, 256, 14      # gate 16 batches, demod 40, period 256; n <= NMAX


@kernel
def k_probe(gate: ParamTable, demod: ParamTable, grp: Group, rq_epoch: int, rq_stop_epoch: int,
            rq_stop_at: int, rq_status: Array, rec: Array, code: int, n: int, period: int, pad: int):
    """rec[0] = t0, the barrier's release; rec[1 + 2s] = now() at the top of shot s, before its stop
    check; rec[2 + 2s] = now() after its posts, before its count store. `pad` delays the posts."""
    init_pulse_params(gate.pulses)  # noqa: F821
    init_pulse_params(demod.pulses)  # noqa: F821
    set_freq(gate, gate.freq)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    t0 = barrier(grp)  # noqa: F821
    rec[0] = t0
    t = t0 + period
    e = rq_epoch
    s = 0
    t_end = 0
    while s < n:
        rec[1 + 2 * s] = now()  # noqa: F821
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        if pad > 0:
            wait_until(now() + pad)  # noqa: F821
        play(gate, s & 1, t)  # noqa: F821   (alternating amplitudes: a late fire plays the other)
        play(demod, demod["sq"], t + G_DUR)  # noqa: F821
        t_end = t + G_DUR + DUR
        rec[2 + 2 * s] = now()  # noqa: F821
        s = s + 1
        rq_status[0] = s
        rq_status[1] = s
        wait_until(t + G_DUR + READOUT_LEAD)  # noqa: F821
        read_res()  # noqa: F821
        t = t + period
    if s > 0:
        wait_until(t_end + LEAD)  # noqa: F821
        read_res()  # noqa: F821
    rq_status[2] = e


ALIGN = 1024     # every run is released at a batch time that is a multiple of this


class Sched:
    """A driver wrapper for one cycle-exact run (lockstep on). At the release (HOST_RESET <- 0) it
    first advances to the next batch time that is a multiple of ALIGN: the SoC's own timing depends a
    little on where a run falls in absolute time (the ADC and decoder alignment, carrier phases), and
    runs released at different batch times can differ by a few cycles, so every run starts at the
    same phase. Then it arms `captures` [(dac, batches)], records that release cycle R and the batch
    time there, issues the release, then the accesses `ops(R, run_id)` returns, at their absolute
    cycles. With `trace` it records the start cycle of every later read32/write32 (zero sim time in
    lockstep). Every other attribute forwards to the driver, its run session included."""

    def __init__(self, drv, m, ops=None, captures=(), trace=False):
        self._drv, self._reset = drv, m.host_ctrl + m.HOST_RESET
        self.ops, self.captures, self.trace_on = ops, list(captures), trace
        self.R = self.bt_R = None
        self.handles, self.done, self.trace = [], [], []

    def _note(self, op, addr):
        if self.trace_on and self.R is not None:
            self.trace.append((self._drv.sim.cycles(), op, int(addr)))

    def write32(self, addr, value):
        if int(addr) == self._reset and not int(value) & 1 and self.R is None:
            sim = self._drv.sim
            c, _, _, bt = sim.sched([(None, "advance", 0)])[0]
            c, _, _, bt = sim.sched([(c + (-bt) % ALIGN, "advance", 0)])[0]
            assert bt % ALIGN == 0, bt
            self.handles = [sim.dac_capture_arm(d, n) for d, n in self.captures]
            self.R, self.bt_R = c, bt
            self._drv.write32(addr, value)
            if self.ops is not None:
                todo = self.ops(self.R, rq.current_run(self._drv))
                if todo:
                    self.done = sim.sched(todo)
            return None
        self._note("write32", addr)
        return self._drv.write32(addr, value)

    def read32(self, addr):
        self._note("read32", addr)
        return self._drv.read32(addr)

    def cycle(self, batch_time) -> int:
        """The clk cycle of a batch time of this run (both count one per cycle on the bench)."""
        return int(batch_time) - (self.bt_R - self.R)

    def __getattr__(self, name):
        return getattr(self._drv, name)


@contextlib.contextmanager
def _lockstep(drv):
    drv.sim.lockstep(True)
    try:
        yield
    finally:
        drv.sim.lockstep(False)


@pytest.fixture(scope="module")
def probe(cosim):
    """`k_probe` on cores 0 and 1 of `cosim`, one group, loaded once for the module."""
    drv, m = cosim
    progs = {}
    for c in (0, 1):
        # a DC carrier: a carrier's samples depend on the pulse's absolute start time (at 50 MHz on
        # its parity), and every run starts at another batch time; a DC pulse is the same in every run
        gate = ParamTable(m.channel_named("gate", c), 0.0,
                          {"a": Pulse(envelopes.square(4 * G_DUR), amp=0.5),
                           "b": Pulse(envelopes.square(4 * G_DUR), amp=0.25)})
        demod = ParamTable(m.channel_named("demod", c), 0.0, {"sq": Pulse(envelopes.square(DUR), amp=1.0)})
        progs[c] = compile_kernel(k_probe, m, core=c, tables=dict(gate=gate, demod=demod), grp=Group([0, 1], id=0),
                                  rec=Array(1 + 2 * NMAX), code=pack16(4 * F),
                                  stop=StopConvention("n", at=True, lead=1, reads_per_shot=1))
    rq.setup(drv, m, progs)
    drv.sim.set_model({"kind": "zero"})
    return drv, m, progs


def _addr(m, progs, core, name, i=0):
    return st.word_addr(m, core, progs[core], name, i)


def _go(probe, w, n=12, pad=0, stop=None, results=("rq_status", "rec")):
    """One run through the wrapper `w` (lockstep on); returns (out, record or None)."""
    drv, m, progs = probe
    with _lockstep(drv):
        out = rq.rerun(w, m, progs, params={c: {"n": n, "period": P, "pad": pad} for c in progs},
                       results=list(results), stop=stop, timeout=400_000)
    return out, S.session(drv).runs[-1].stop


def _top(w, out, core, s):
    """The cycle (relative to R) of `now()` at the top of shot s on `core`, from the run's record."""
    return w.cycle(int(out[core]["rec"][1 + 2 * s])) - w.R


def _post(w, out, core, s):
    return w.cycle(int(out[core]["rec"][2 + 2 * s])) - w.R


def _request(kind, S_=None):
    """A stop spec whose request the wrapper publishes itself: the policy posts once, the issue
    records the request (unverified) and writes nothing; no core RAM is touched by the poll loop."""
    posted = []

    def policy(ctx):
        if posted:
            return None
        posted.append(1)
        return kind, S_

    def issue(ctx, ticket):
        ticket.info.update(kind=kind, S=S_, verified=False)
        return S.ACCEPTED
    return S.StopSpec(issue=issue, policy=policy, poll_cycles=256, accept_inconsistent=True)


# ── determinism ──

@pytest.mark.batch_cap(50_000)
def test_lockstep_repeats_a_run_cycle_for_cycle(probe):
    """The same stoppable run, released at the same batch-time phase with P4's issuer at the same
    scheduled cycle, repeats cycle for cycle under lockstep once the CPU's branch predictor has seen
    it: after one warm-up run, three runs give equal StopRecords (v_pre, v_post, S, counts) and equal
    kernel timelines relative to the release. (The GShare counters survive the core reset, so the
    first run of a new branch pattern, here the check that starts to see the request, can take a few
    cycles more; the M-tests measure within a run or against warmed references.)

    FLOOR: four runs of ~5 k batches (12 shots at 256, the record read, the phase alignment)."""
    drv, m, progs = probe
    got = []
    for _ in range(4):
        w = Sched(drv, m)

        def policy(ctx, w=w, fired=[]):
            if fired:
                return None
            fired.append(1)
            drv.sim.sched([(w.R + 2_000, "advance", 0)])
            return S.AT, None
        out, rec = _go(probe, w, stop=st.spec(policy, margin=2))
        times = [w.cycle(int(x)) - w.R for x in out[0]["rec"][:1 + 2 * rec.shots[0]]]
        got.append((rec.outcome, rec.shots, {k: rec.request[k] for k in ("v_pre", "v_post", "S", "verified")}, times))
    assert got[1] == got[2] == got[3], got[1:]
    assert got[1][0] == S.FIRED


# ── M1: stall cycles per host access type on port1 ──

@pytest.mark.batch_cap(200_000)
def test_m1_stall_cycles_per_host_access_type(probe):
    """M1: the instruction sequence from the top of a shot (its check) to its posts takes a fixed
    number of cycles in every shot. One host access per shot, at a different offset in each, measures
    how long it gets: every offset of the sequence is covered, ten per run, for a stop-word write, a
    progress read and a read-back. The worst case is one cycle per access (the arbiter hands the
    host one fetch slot), on the addressed core only.

    FLOOR: one reference run and 3 x 8 runs of ~6 k batches (12 shots at 256 and the record read;
    lockstep, no idle ticks)."""
    drv, m, progs = probe
    w = Sched(drv, m)
    out, _ = _go(probe, w)
    shots = range(1, 11)
    base = {(c, s_): _post(w, out, c, s_) - _top(w, out, c, s_) for c in (0, 1) for s_ in shots}
    span0 = {base[(0, s_)] for s_ in shots}
    assert len(span0) == 1, base                                   # the same sequence every shot
    (span,) = span0
    kinds = {"write": ("write32", _addr(m, progs, 0, "rq_stop_at"), 0),
             "progress read": ("read32", _addr(m, progs, 0, "rq_status")),
             "read-back": ("read32", _addr(m, progs, 0, "rq_stop_epoch"))}
    offsets = list(range(0, span))
    stall = {}
    for name, op in kinds.items():
        per = {}
        for i in range(0, len(offsets), len(shots)):
            js = dict(zip(shots, offsets[i:i + len(shots)]))
            w2 = Sched(drv, m, ops=lambda R, rid, js=js, op=op: [(R + _top(w, out, 0, s_) + j, *op)
                                                                 for s_, j in js.items()])
            out2, _ = _go(probe, w2)
            for s_, j in js.items():
                per[j] = _post(w2, out2, 0, s_) - _top(w2, out2, 0, s_) - span
                assert _post(w2, out2, 1, s_) - _top(w2, out2, 1, s_) == base[(1, s_)], (name, s_)  # core 1: none
        stall[name] = (max(per.values()), sorted(j for j, v in per.items() if v))
    print(f"\n[P4 M1] the shot's check-to-post sequence: {span} cycles; per access type the worst stall and the "
          f"offsets that stall: {stall}")
    assert all(0 <= v[0] <= 1 for v in stall.values()), stall
    assert any(v[0] == 1 for v in stall.values()), stall


# ── M4: collisions aligned cycle-exactly ──

def _m4_write_sweep(probe, S_, js, ref_top):
    """Core 1 gets AT(S_) early; core 0 gets rq_stop_at early and its rq_stop_epoch write at
    R + ref_top + j. Returns {j: core 0's count}."""
    drv, m, progs = probe
    res = {}
    for j in js:
        def ops(R, rid, j=j):
            e = rid[1]
            return [(R + 1200, "write32", _addr(m, progs, 1, "rq_stop_at"), S_),
                    (None, "write32", _addr(m, progs, 1, "rq_stop_epoch"), e),
                    (None, "write32", _addr(m, progs, 0, "rq_stop_at"), S_),
                    (R + ref_top + j, "write32", _addr(m, progs, 0, "rq_stop_epoch"), e)]
        w = Sched(drv, m, ops=ops)
        out, rec = _go(probe, w, n=S_ + 4, stop=_request(S.AT, S_))
        assert rec.shots[1] == S_ and rec.shots[0] in (S_, S_ + 1), (j, rec.shots)
        assert rec.outcome == (S.FIRED if rec.shots[0] == S_ else S.INCONSISTENT)
        res[j] = rec.shots[0]
    return res


@pytest.mark.batch_cap(185_000)
def test_m4_an_epoch_write_against_the_check_load(probe):
    """M4 (write against load): core 0's rq_stop_epoch write swept cycle by cycle across its check of
    shot 6. Before some cycle it stops at 6 (it saw the request), from it on at 7 (the check read the
    old word): one transition, nothing else. The offset of that cycle from the recorded top of the
    shot predicts the transition at shot 9 exactly.

    FLOOR: a reference run and ~30 swept runs of ~3 k batches (lockstep)."""
    drv, m, progs = probe
    w = Sched(drv, m)
    out, _ = _go(probe, w, n=13)
    js = list(range(-10, 18))
    res = _m4_write_sweep(probe, 6, js, _top(w, out, 0, 6))
    seq = [res[j] for j in js]
    assert seq == sorted(seq) and seq[0] == 6 and seq[-1] == 7, res       # one transition, monotonic
    jstar = next(j for j in js if res[j] == 7)
    pred = _m4_write_sweep(probe, 9, [jstar - 1, jstar], _top(w, out, 0, 9))
    assert pred == {jstar - 1: 9, jstar: 10}, (jstar, pred)
    print(f"\n[P4 M4] an epoch write started {jstar} cycles after the recorded top of a shot is the first one "
          f"its check misses; the same offset predicts shot 9")


@pytest.mark.batch_cap(220_000)
def test_m4_a_progress_read_against_the_count_store(probe):
    """M4 (read against store): a read of core 0's rq_status[0] swept cycle by cycle across its store
    of the count after shot 5's posts returns 5 (old) and then 6 (new), with one transition and no
    other value; the offset predicts the read across shot 9's store.

    FLOOR: a reference run and ~36 swept runs of ~4 k batches (lockstep)."""
    drv, m, progs = probe
    w = Sched(drv, m)
    out, _ = _go(probe, w)
    a = _addr(m, progs, 0, "rq_status")

    def sweep(s, js):
        got = {}
        for j in js:
            w2 = Sched(drv, m, ops=lambda R, rid, j=j: [(R + _post(w, out, 0, s) + j, "read32", a)])
            _go(probe, w2, results=("rq_status",))
            got[j] = w2.done[0][2]
        return got
    js = list(range(-8, 26))
    got = sweep(5, js)
    seq = [got[j] for j in js]
    assert set(seq) == {5, 6} and seq == sorted(seq), got
    jstar = next(j for j in js if got[j] == 6)
    assert sweep(9, [jstar - 1, jstar]) == {jstar - 1: 9, jstar: 10}
    print(f"\n[P4 M4] a progress read started {jstar} cycles after the recorded post time sees the new count")


# ── C2: P4's issuer at cycle-exact phases ──

def _c2_phase(probe, ref, k, phase, S_):
    """One run: P4's issuer starts at R + (core 0's post of shot k) + phase, AT(S_) explicit."""
    drv, m, progs = probe
    w = Sched(drv, m, trace=True)

    def policy(ctx, fired=[]):
        if fired:
            return None
        fired.append(1)
        drv.sim.sched([(w.R + ref + phase, "advance", 0)])
        return S.AT, S_
    out, rec = _go(probe, w, n=S_ + 3, stop=S.StopSpec(issue=st.Issuer(), policy=policy, poll_cycles=256))
    return w, out, rec


def _count_at(w, out, core, cycle, K):
    """The counts a read started at `cycle` may return by the run's own record (the store lands at
    post + K): one value, or old or new when the read starts within a cycle of the store."""
    def at(c):
        return sum(1 for s in range(NMAX) if int(out[core]["rec"][2 + 2 * s]) and _post(w, out, core, s) + K <= c)
    return {at(cycle - 1), at(cycle), at(cycle + 1)}


def _c2_check(probe, ref, k, phases, S_, K):
    drv, m, progs = probe
    a0 = _addr(m, progs, 0, "rq_status")
    seen = []
    for ph in phases:
        w, out, rec = _c2_phase(probe, ref, k, ph, S_)
        reads = [c - w.R for c, op, adr in w.trace if op == "read32" and adr == a0]
        t = rec.tickets[0]
        v_pre_pred = _count_at(w, out, 0, reads[0], K)
        assert t[2] in (S.ACCEPTED, S.TOO_LATE)
        info = S.session(drv).runs[-1].tickets[0].info
        assert info["v_pre"] in v_pre_pred, (ph, info, reads, v_pre_pred)
        assert (t[2] == S.TOO_LATE) == (S_ < info["v_pre"] + 3), (ph, t, info)
        if t[2] == S.ACCEPTED:
            v_post_pred = _count_at(w, out, 0, reads[1], K)
            assert info["v_post"] in v_post_pred, (ph, info, reads, v_post_pred)
            assert info["verified"] == (S_ >= info["v_post"] + 3), (ph, info)
            assert rec.outcome == S.FIRED and set(rec.shots.values()) == {S_}, (ph, rec)
        else:
            assert rec.outcome == S.NATURAL
        seen.append((ph, t[2], info["v_pre"], info.get("v_post"), bool(info.get("verified"))))
    return seen


@pytest.fixture(scope="module")
def k_store(probe):
    """K: a progress read started K cycles after the recorded post time sees the new count (M4's
    read-against-store offset, bisected here once for the C2 predictions)."""
    drv, m, progs = probe
    w = Sched(drv, m)
    out, _ = _go(probe, w)
    a = _addr(m, progs, 0, "rq_status")

    def new(j):
        w2 = Sched(drv, m, ops=lambda R, rid: [(R + _post(w, out, 0, 5) + j, "read32", a)])
        _go(probe, w2, results=("rq_status",))
        return w2.done[0][2] == 6
    lo, hi = -8, 48
    assert not new(lo) and new(hi)
    while hi - lo > 1:
        mid = (lo + hi) // 2
        lo, hi = (lo, mid) if new(mid) else (mid, hi)
    return hi


@pytest.mark.batch_cap(120_000)
def test_c2_the_issuer_at_twelve_phases_of_a_period(probe, k_store):
    """C2: P4's issuer (AT(S) explicit, S = k + 3) started at 12 phases, every 32 cycles from a period
    before core 0's post of shot k to half a period after it. At every phase v_pre and v_post equal
    the counts the run's own record gives at the cycles the reads started (old or new within a cycle
    of a store); the request is TOO_LATE exactly when S < v_pre + 3, and VERIFIED exactly when
    S >= v_post + 3, i.e. unless a post between the reads brought some core's check of S within the
    bound. Every accepted run FIRES at S. The sweep meets all three cases.

    FLOOR: K's bisection (~7 runs, shared with the slow sweep) and 13 runs of ~5 k batches."""
    drv, m, progs = probe
    w = Sched(drv, m)
    out, _ = _go(probe, w)
    k = 5
    ref = _post(w, out, 0, k)
    phases = [-P + 32 * i for i in range(12)]
    seen = _c2_check(probe, ref, k, phases, k + 3, k_store)
    print("\n[P4 C2] (phase, outcome, v_pre, v_post, VERIFIED):\n  " + "\n  ".join(map(str, seen)))
    cases = {(x[1], x[4]) for x in seen}
    assert {(S.TOO_LATE, False), (S.ACCEPTED, True), (S.ACCEPTED, False)} <= cases, seen


@pytest.mark.slow
def test_c2_full_period_sweep(probe, k_store):
    """--slow: the C2 checks at every cycle of one period (256 phases, from 224 cycles before core 0's
    post of shot k to 32 after it, so TOO_LATE, VERIFIED and unverified all occur)."""
    drv, m, progs = probe
    w = Sched(drv, m)
    out, _ = _go(probe, w)
    k = 5
    seen = _c2_check(probe, _post(w, out, 0, k), k, list(range(-P + 32, 32)), k + 3, k_store)
    tally = {}
    for x in seen:
        tally[(x[1], x[4])] = tally.get((x[1], x[4]), 0) + 1
    print(f"\n[P4 C2 slow] 256 phases: {tally}")
    assert set(tally) == {(S.TOO_LATE, False), (S.ACCEPTED, True), (S.ACCEPTED, False)}, tally


# ── M3: the AT pattern across a shot's critical window ──

def _at_pattern(m, progs, R, e, x):
    """The AT access pattern of a 14-core run as core 0, the reference, sees it, harmlessly: a
    progress read, core 0's publish (rq_stop_at 0 and an epoch that is not this run's, then the
    read-back), 13 publishes to core 1's RAM in its stead, and the second progress read."""
    c0 = lambda name: _addr(m, progs, 0, name)       # noqa: E731
    c1 = lambda name: _addr(m, progs, 1, name)       # noqa: E731
    bad = e ^ 0x5A5A
    ops = [(R + x, "read32", c0("rq_status")),
           (None, "write32", c0("rq_stop_at"), 0), (None, "write32", c0("rq_stop_epoch"), bad),
           (None, "read32", c0("rq_stop_epoch"))]
    for _ in range(13):
        ops += [(None, "write32", c1("rq_stop_at"), 0), (None, "write32", c1("rq_stop_epoch"), bad),
                (None, "read32", c1("rq_stop_epoch"))]
    return ops + [(None, "read32", c0("rq_status"))]


@pytest.mark.batch_cap(530_000)
def test_m3_the_at_pattern_across_a_critical_window_is_bit_exact(probe):
    """M3: the 14-core AT pattern started at every cycle from 8 before the top of shot 6 to 8 after
    its posts on core 0, the span in which core 0 executes (the check and the posts) and a stolen
    fetch slot delays the gate fire: the gate DAC capture of the run, through shot 6's pulse, is
    bit-exact against the run without traffic at every offset.

    FLOOR: a reference run and ~92 swept runs of ~4.2 k batches (7 shots, the record and the DAC
    capture), cycle by cycle as the plan asks."""
    drv, m, progs = probe
    dac = m.channel_named("gate", 0).dac
    w = Sched(drv, m, captures=[(dac, 2_600)])
    out, _ = _go(probe, w, n=7)
    t_ref, cap_ref = drv.sim.dac_capture_get(w.handles[0])
    assert np.abs(cap_ref).max() > 0
    lo, hi = _top(w, out, 0, 6) - 8, _post(w, out, 0, 6) + 8
    bad = []
    for x in range(lo, hi):
        w2 = Sched(drv, m, captures=[(dac, 2_600)], ops=lambda R, rid, x=x: _at_pattern(m, progs, R, rid[1], x))
        _go(probe, w2, n=7, results=("rq_status",))
        _, cap = drv.sim.dac_capture_get(w2.handles[0])
        if not np.array_equal(cap, cap_ref):
            bad.append(x)
    print(f"\n[P4 M3] {hi - lo} offsets of the 44-access pattern across shot 6's window: {len(bad)} corrupt")
    assert not bad, bad


# ── M2: the slack of the gate fire ──

@pytest.mark.batch_cap(80_000)
def test_m2_the_slack_of_the_gate_fire(probe):
    """M2: `pad` cycles of delay before the posts (a stall injected in the shot path). The smallest
    pad that changes the gate DAC capture gives the fire class's slack, pad - 1; the corrupted run
    plays a pulse late with the previous pulse's amplitude, as R5 says. k_probe's gate fire has well
    over the four stall cycles one host access costs (M1).

    FLOOR: a reference run and ~10 bisection runs, each with a 3.5 k-batch DAC capture."""
    drv, m, progs = probe
    dac = m.channel_named("gate", 0).dac

    def cap(pad):
        w = Sched(drv, m, captures=[(dac, 4_000)])
        _go(probe, w, pad=pad, results=("rq_status",))
        return drv.sim.dac_capture_get(w.handles[0])[1]
    ref = cap(0)
    lo, hi = 0, P
    assert not np.array_equal(cap(hi), ref)
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if np.array_equal(cap(mid), ref):
            lo = mid
        else:
            hi = mid
    print(f"\n[P4 M2] k_probe's gate fire: corrupted from pad {hi}: slack {hi - 1} cycles")
    assert hi - 1 >= 16, hi


# ── M2 and M3 on k_batched, the calibration kernel (not stoppable in P4: plan Q3). Last: it reloads core 0 ──

def _batched(m, herald):
    """A k_batched Experiment on core 0: an x90 amplitude sweep of two points of one shot each, so
    consecutive drives differ and a late fire shows; heralded or not; DC carriers (the capture is
    then the same in every run) and a short relax head."""
    from riscq.cal.axes import Axis
    from riscq.cal.experiment import Experiment
    from riscq.cal.measure import Measure
    from riscq.cal.sequence import Gate
    from tests.cal_fixtures import _cfg2
    amp = Axis.amp(0.3, 0.7, 2)
    return Experiment(_cfg2(m, freqs=(0.0, 0.0), relax=64), [0], {0: [Gate("x90", amp=amp)]}, {0: (amp,)}, (),
                      Measure.counts(herald=herald), 1, label=f"m2-{'herald' if herald else 'plain'}")


def _batched_runner(drv, m, exp):
    """Load `exp` once; `run(ops)` reruns it through a `Sched` wrapper capturing core 0's gate DAC."""
    progs, _signs, timeout = exp.compile(drv)
    comp, axes, rcore, npts = exp.compiled[0]
    words = exp._pairs(axes, ())
    par = {c: {k: v for k, v in words.items() if k in progs[c].params} for c in set(comp.cores()) | set(rcore)}
    dac = m.channel_named("gate", 0).dac
    n_cap = progs[0].bindings["period"] * (npts + 1) + 800          # through the second drive

    def run(ops=None):
        w = Sched(drv, m, ops=ops, captures=[(dac, n_cap)])
        with _lockstep(drv):
            rq.rerun(w, m, progs, params=par, results=["out"], timeout=timeout)
        t0, cap = drv.sim.dac_capture_get(w.handles[0])
        return w, t0, cap
    return progs, npts, run


def _harmless_pattern(m, progs, R, x):
    """M3's 44-access AT pattern on a program without stop words: core 0's progress reads and its
    publish land on __rq_magic (read, written back with its own value, read back), core 1's 13
    publishes on an unused word of its RAM (core 1 is parked)."""
    magic = st.word_addr(m, 0, progs[0], "__rq_magic")
    other = m.to_host_addr(1, 0x8000_0400)
    ops = [(R + x, "read32", magic), (None, "write32", magic, rq.MAGIC), (None, "write32", magic, rq.MAGIC),
           (None, "read32", magic)]
    for _ in range(13):
        ops += [(None, "write32", other, 0), (None, "write32", other, 0), (None, "read32", other)]
    return ops + [(None, "read32", magic)]


@pytest.mark.batch_cap(530_000)
def test_m2_m3_k_batched_drives_under_host_traffic(cosim):
    """M2 and M3 on k_batched (plan P4 v2 §6), the calibration kernel P4 keeps non-stoppable (Q3):
    how much host traffic its drive fire takes, heralded (the drive is posted when the herald read
    returns, which by herald_offset's design is at its nominal LEAD deadline, so its post lands just
    after that deadline) and not. M2: one host access, at most one stall cycle (M1), started every 3
    cycles from 60 before the second drive's deadline (start - LEAD) to 100 after it; M1 found that an
    access stalls the core only from runs of three consecutive offsets, so this hits each run. A
    stall that corrupts the drive at some offset means the fire class has no slack there. M3: the
    44-access AT pattern swept the same way across the heralded drive's window. A capture is corrupt
    when it differs from the run without traffic (which a rerun reproduces bit for bit). The offsets
    are recorded for the report; what P4 relies on is asserted: the non-heralded drive takes a stall
    anywhere in its window.

    FLOOR: two image loads and about 170 runs of ~2.5 k batches (two short-relax shots each, with
    their DAC capture through the second drive)."""
    drv, m = cosim
    drv.sim.set_model({"kind": "zero"})
    found = {}
    for herald in (False, True):
        progs, npts, run = _batched_runner(drv, m, _batched(m, herald))
        w, t0, ref = run()
        assert np.array_equal(run()[2], ref)                           # a rerun without traffic: the same
        nz = np.nonzero(np.any(ref != 0, axis=1))[0]
        starts = [int(nz[0])] + [int(nz[i]) for i in range(1, len(nz)) if nz[i] != nz[i - 1] + 1]
        assert len(starts) == npts, starts                             # one drive per point
        dl = w.cycle(t0 + starts[1] - LEAD) - w.R                      # the 2nd drive's posting deadline
        found[("window", herald)] = (f"period {progs[0].bindings['period']}, drives at R + "
                                     f"{[w.cycle(t0 + r) - w.R for r in starts]}, deadline R + {dl}")
        magic = st.word_addr(m, 0, progs[0], "__rq_magic")
        xs = range(dl - 60, dl + 102, 3)
        found[("M2", herald)] = [x - dl for x in xs
                                 if not np.array_equal(run(lambda R, rid, x=x: [(R + x, "read32", magic)])[2], ref)]
        if herald:
            found[("M3", herald)] = [x - dl for x in xs
                                     if not np.array_equal(run(lambda R, rid, x=x: _harmless_pattern(m, progs, R, x))[2],
                                                           ref)]
    print("\n[P4 M2/M3 k_batched] offsets (cycles from the drive's posting deadline) where traffic corrupts the "
          "second drive: " + "; ".join(f"{k[0]} {'heralded' if k[1] else 'plain'}: {v or 'none'}"
                                       for k, v in found.items()))
    assert not found[("M2", False)], found
    assert all(isinstance(found[k], list) for k in found if k[0] != "window")
