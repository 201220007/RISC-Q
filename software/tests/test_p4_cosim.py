"""Co-sim tests of P4, early STOP and exact actual_shots (qubic3; plan P4 v2 §8 L1/L2), on the RTL.

`k_grid` is P4's test kernel under the stop convention (§5.2): every core starts from one barrier,
gates the posting of shot k by `wait_until` on the grid t0 + period + pre + k·period, checks the stop
words at the top of every shot, publishes [posted shots, reads] right after its posts, and after the
loop waits out its last demod and stores fin (C1, C2 with L = 1, C3 with period - ~130 batches of
slack). A reader plays one demod per shot (two on the even shots of a heralded core, so its reads
are not a multiple of its shots) and stores the first NOUT results; a non-reader only paces.

Module A, the stop semantics on three cores (`cosim_2q1c`: sim-2q1c-antq under --results-path
antq_uplink, the hostwindow sim-2q1c otherwise, the plan's parity run): two readers and a
non-reading coupler, uplink-free reruns of one loaded image (on the antq build their results meet
the closed admission: EXPECTED_DISCARD at the next quiesce, no flush; every run is queue-proven).
AT(S) at several shot indices, VERIFIED and FIRED; S >= n (NATURAL); S = 0 refused before any write
(TOO_LATE); NEXT mid-run and before the first check; a margin sweep whose unverified requests are
classified consistently; a deliberately late issue giving INCONSISTENT (raised, certified) and
CONSISTENT_LATE; races against completion; a late writer of an old epoch; TIMEOUT and UNFINISHED;
the next run after each kind of stop.

Module B, the uplink (`cosim_antq`, sim-dio-antq in both modes): two readers on their own ADCs, core 1
heralded. The drain certifies exactly the counted reads of each core after AT(S), NEXT and an empty
run, the DDR words equal the CPU's results, REJECTED stays 0 through G2, the `landed` policy stops on
the uplink's own CUR_ADDR, a hung stoppable run fails and the next uplink run is exact.

C5, completion (`cosim_antq`): a stopped readout that drives the `ro` DAC; every DAC sample precedes
fin, and the next run's first drive is its own.

The remote seam (`cosim`, last): a RemoteDriver sends the stop spec's wire form to the bench's
server-side runner, the board server's twin, and the run's StopRecord comes back with the results.

The issue step's 2N writes, N read-backs and 2 progress reads cost about one bench idle tick (200
cycles) each in co-sim, about 2.4 k cycles for three cores, so the VERIFIED runs take a margin m =
M shots at the 384-batch period (§5.3: S = v_pre + L + 2 + m)."""

import numpy as np
import pytest

from riscq import ddr_regs as R
from riscq import run as rq
from riscq import session as S
from riscq import stop as st
from riscq.lang import Array, Group, ParamTable, StopConvention, compile_kernel, kernel
from riscq.map import LEAD, READOUT_LEAD, pack16
from riscq.pulses import Pulse, envelopes, units

pytestmark = pytest.mark.cosim

DUR, F, NOUT, PERIOD, M = 40, 1024, 16, 384, 16


@kernel
def k_grid(demod: ParamTable, grp: Group, rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int,
           rq_status: Array, out: Array, code: int, reader: int, herald: int,
           n: int, period: int, pre: int, fault: int):
    """P4's stoppable test kernel. `pre` delays the first check by `pre` batches after the barrier;
    `fault` 1 halts ~2^30 batches after the loop (a hung kernel), 2 leaves fin unpublished."""
    init_pulse_params(demod.pulses)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    t = barrier(grp) + period + pre  # noqa: F821   (the pre-loop rendezvous: one t0 for every core)
    e = rq_epoch
    if pre > 0:
        wait_until(t - period)  # noqa: F821
    s = 0
    reads = 0
    t_end = 0
    k = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        two = 0
        if reader == 1:
            play(demod, demod["sq"], t)  # noqa: F821
            t_end = t + DUR
            reads = reads + 1
            if herald == 1:
                if (s & 1) == 0:
                    play(demod, demod["sq"], t + 2 * DUR)  # noqa: F821
                    t_end = t + 3 * DUR
                    reads = reads + 1
                    two = 1
        s = s + 1
        rq_status[0] = s
        rq_status[1] = reads
        wait_until(t + READOUT_LEAD)  # noqa: F821
        if reader == 1:
            read_res()  # noqa: F821
            if k < NOUT:
                out[2 * k] = read_real()  # noqa: F821
                out[2 * k + 1] = read_imag()  # noqa: F821
                k = k + 1
            if two == 1:
                wait_until(t + 2 * DUR + READOUT_LEAD)  # noqa: F821
                read_res()  # noqa: F821
                if k < NOUT:
                    out[2 * k] = read_real()  # noqa: F821
                    out[2 * k + 1] = read_imag()  # noqa: F821
                    k = k + 1
        t = t + period
    if fault == 1:
        wait_until(now() + 1073741824)  # noqa: F821
    if reads > 0:
        wait_until(t_end + LEAD)  # noqa: F821   (the last demod has ended)
        read_res()  # noqa: F821
    if fault != 2:
        rq_status[2] = e


def _program(m, core, members, reader=1, herald=0):
    conv = StopConvention("n", at=True, lead=1, reads_per_shot=None if herald else reader)
    demod = ParamTable(m.channel_named("demod", core), 0.0, {"sq": Pulse(envelopes.square(DUR), amp=1.0)})
    return compile_kernel(k_grid, m, core=core, tables=dict(demod=demod), grp=Group(members, id=0),
                          out=Array(2 * NOUT), code=pack16(4 * F), reader=reader, herald=herald, stop=conv)


def _params(progs, n, period=PERIOD, pre=0, fault=0):
    return {c: {"n": n, "period": period, "pre": pre, "fault": fault} for c in progs}


def _reads_of(shots, herald):
    """A heralded core reads twice on its even shot indices."""
    return shots + (shots + 1) // 2 if herald else shots


def _check(rec, out, n, readers, herald=()):
    """The published counts, their validity and the outcome table, for any outcome."""
    shots = rec.shots
    for c, k in rec.counts.items():
        assert k.valid and k.fin == rec.run_id[1] and 0 <= k.shots <= n, (c, k)
        assert [int(x) & 0xFFFF_FFFF for x in out[c]["rq_status"]] == [k.shots, k.reads, k.fin]
        want = _reads_of(k.shots, c in herald) if c in readers else 0
        assert k.reads == want, (c, k, want)
    assert rec.prefix == min(shots.values())
    req = rec.request
    if rec.outcome == S.NATURAL:
        assert set(shots.values()) == {n}
    elif rec.outcome == S.FIRED:
        assert req["kind"] == S.AT and set(shots.values()) == {min(req["S"], n)} and req["S"] < n
    elif rec.outcome == S.CONSISTENT_LATE:
        (s1,) = set(shots.values())
        assert not req["verified"] and req["S"] < s1 < n and rec.late_by == s1 - req["S"]
    elif rec.outcome == S.INCONSISTENT:
        assert not req["verified"] and len(set(shots.values())) > 1
        assert all(s >= min(req["S"], n) for s in shots.values())
    elif rec.outcome == S.STOPPED_EACH:
        assert req["kind"] == S.NEXT and set(shots.values()) != {n}
    else:
        raise AssertionError(rec)
    if req is not None and req["kind"] == S.AT and req.get("verified"):
        assert rec.outcome in (S.FIRED, S.NATURAL) and set(shots.values()) == {min(req["S"], n)}


# ── module A: the stop semantics on three cores ──

@pytest.fixture(scope="module")
def grid(cosim_2q1c):
    """`k_grid` on the 3-core build: readers on cores 0 and 1, the coupler core 2 a non-reader, one
    group; loaded once for the module."""
    drv, m = cosim_2q1c
    members = [0, 1, 2]
    progs = {0: _program(m, 0, members), 1: _program(m, 1, members), 2: _program(m, 2, members, reader=0)}
    rq.setup(drv, m, progs)
    return drv, m, progs


def _tone(drv, m, adcs=(0,)):
    spec = [{"kind": "tone", "adc": a, "freq_hz": units.code_to_freq(F, m.params), "amp": 9000.0 + 3000.0 * a}
            for a in adcs]
    drv.sim.set_model(spec[0] if len(spec) == 1 else {"kind": "multi", "models": spec})


def _go(fx, n, spec=None, timeout=600_000, results=("rq_status",), **kw):
    drv, m, progs = fx
    out = rq.rerun(drv, m, progs, params=_params(progs, n, **kw), results=list(results),
                   stop=spec, timeout=timeout)
    rec = st.last(drv)
    assert rec is S.session(drv).runs[-1].stop and S.session(drv).runs[-1].outcome == S.CERTIFIED
    _check(rec, out, n, readers=(0, 1))
    return out, rec


def _natural_after(fx, n=3):
    """The run after a stop: no request, so every core runs its n shots, its counts exact."""
    _, rec = _go(fx, n)
    assert rec.outcome == S.NATURAL and rec.shots == {0: n, 1: n, 2: n} and rec.reads == {0: n, 1: n, 2: 0}


def _spec(policy, margin=M, **kw):
    return st.spec(policy, margin=margin, poll_cycles=256, **kw)


@pytest.mark.batch_cap(70_000)
def test_natural_and_at_s_fires_verified_at_several_shot_indices(grid):
    """C1, C4: a run with no request is NATURAL with exact counts (the coupler reads 0); AT(S) at the
    earliest verifiable S after a request at progress k = 0 (as the cores boot) and k = 6 is VERIFIED
    and FIRED, every core at S; the run after it is exact.

    FLOOR: four reruns at ~5 k batches of host ops each (the per-core params, the stop words, the
    issue step and the results, one bench idle tick per op) plus their shots, 12, ~20, ~25 and 3 at
    384 batches."""
    drv, m, progs = grid
    _tone(drv, m)
    out, rec = _go(grid, 12, results=("rq_status", "out"))
    assert rec.outcome == S.NATURAL and rec.shots == {0: 12, 1: 12, 2: 12} and rec.reads == {0: 12, 1: 12, 2: 0}
    assert rec.request is None and rec.tickets == []
    assert np.abs(out[0]["out"]).max() > 0                       # the readers measured the tone
    for k in (0, 6):
        _, rec = _go(grid, 40, _spec(st.AtProgress(k)))
        rqst = rec.request
        assert rqst["verified"] and rqst["v_pre"] >= k and rqst["S"] == rqst["v_pre"] + 1 + 2 + M
        assert rec.outcome == S.FIRED and set(rec.shots.values()) == {rqst["S"]}
        assert rec.tickets == [(S.AT, None, S.ACCEPTED)]
        print(f"\n[P4 C1] k={k}: v_pre {rqst['v_pre']} v_post {rqst['v_post']} S {rqst['S']} "
              f"(broadcast {rqst['v_post'] - rqst['v_pre']} shots)")
    _natural_after(grid)


@pytest.mark.batch_cap(65_000)
def test_s_at_or_past_n_is_natural_and_s_zero_is_too_late(grid):
    """C4: an explicit S = n or n + 9, published early, is VERIFIED and the run NATURAL; an explicit
    S = 0 is below v_pre + L + 2 and refused before any write (TOO_LATE), so the run is NATURAL too.

    FLOOR: three reruns at ~5 k batches of host ops each plus 24 + 24 + 10 shots at 384 batches."""
    drv, m, progs = grid
    _tone(drv, m)
    for S_ in (24, 33):
        _, rec = _go(grid, 24, _spec(st.AtProgress(2, S.AT, S_)))
        assert rec.request["S"] == S_ and rec.request["verified"] and rec.outcome == S.NATURAL
    _, rec = _go(grid, 10, _spec(st.AtProgress(1, S.AT, 0)))
    assert rec.tickets == [(S.AT, 0, S.TOO_LATE)] and rec.request is None and rec.outcome == S.NATURAL


@pytest.mark.batch_cap(50_000)
def test_next_mid_run_and_before_the_first_check(grid):
    """C3, C4: NEXT at progress 6 stops each core at its next boundary after its own publish
    (STOPPED_EACH, per-core counts); NEXT posted right after the release, while every core waits
    out `pre` before its first check, stops them all at shot 0 (an empty run: no shot, no read).
    The run after it is exact.

    FLOOR: three reruns at ~5 k batches of host ops each plus ~11 shots at 384 batches, the
    8 k-batch `pre` and 3 shots."""
    drv, m, progs = grid
    _tone(drv, m)
    _, rec = _go(grid, 40, _spec(st.AtProgress(6, S.NEXT)))
    assert rec.outcome == S.STOPPED_EACH and all(6 <= s < 40 for s in rec.shots.values())
    print(f"\n[P4 C4] NEXT at 6: shots {rec.shots}")
    _, rec = _go(grid, 40, _spec(st.AtProgress(0, S.NEXT)), pre=8_000)
    assert rec.outcome == S.STOPPED_EACH and rec.shots == {0: 0, 1: 0, 2: 0} and rec.reads == {0: 0, 1: 0, 2: 0}
    _natural_after(grid)


@pytest.mark.batch_cap(75_000)
def test_a_margin_sweep_classifies_every_request_consistently(grid):
    """C2: AT(None) at progress 4 with m = 0..3, so S lands where some cores may already have begun
    its check. VERIFIED runs FIRE at S; the others are FIRED, CONSISTENT_LATE or INCONSISTENT as the
    counts say (accepted with truncate), never INTERNAL_ERROR; each run certifies, and so does the
    run after them.

    FLOOR: five reruns at ~5 k batches of host ops each plus ~8-12 shots each at 384 batches."""
    drv, m, progs = grid
    _tone(drv, m)
    seen = []
    for margin in range(4):
        _, rec = _go(grid, 40, _spec(st.AtProgress(4), margin=margin, truncate=True))
        r = rec.request
        seen.append((margin, r["v_pre"], r["v_post"], r["S"], r["verified"], rec.outcome, rec.shots))
        assert r["verified"] == st.verified(r["S"], r["v_post"], 1)
    print("\n[P4 C2] (m, v_pre, v_post, S, VERIFIED, outcome, shots):\n  " + "\n  ".join(map(str, seen)))
    _natural_after(grid)


def _late_issue(gap, claim_verified=False):
    """A deliberately late issue (a test hook, not P4's issuer): publish AT(v - 1), a shot every core
    has passed, core by core with `gap` batches between the cores, recorded as unverified (or, to
    reach INTERNAL_ERROR on the RTL, falsely as VERIFIED)."""
    def issue(ctx, ticket):
        drv, m, progs = ctx.drv, ctx.m, ctx.progs
        v = st.posted(drv, m, 0, progs[0])
        S_ = max(0, v - 1)
        for c in sorted(progs):
            st.publish(drv, m, c, progs[c], ctx.run_id[1], S_)
            if gap:
                drv.sim.advance(gap)
        ticket.info.update(kind=S.AT, S=S_, verified=claim_verified, v_pre=v)
        return S.ACCEPTED
    return issue


@pytest.mark.batch_cap(85_000)
def test_a_late_issue_gives_inconsistent_consistent_late_and_internal_error(grid):
    """C2 (a deliberately late write): published 3 periods apart per core, the cores stop at
    different shots: INCONSISTENT, raised after the run certified, the joint data the common prefix;
    the next run is NATURAL. Published within one long period (6 000 batches) after shot 2 was
    posted, every core stops at its next check: CONSISTENT_LATE, S' - S = 1; the next run is exact.
    The same late request recorded as VERIFIED is INTERNAL_ERROR: the run fails (its counts kept,
    valid) and the next run takes the flush (FLUSH_ALLOWANCE) and is exact.

    FLOOR: six reruns at ~5 k batches of host ops each, ~16 shots at 384 plus 6 periods of
    advance, 6 shots, 3 shots at 6 000 batches, ~5 shots and twice 3 shots."""
    drv, m, progs = grid
    _tone(drv, m)
    spec = S.StopSpec(issue=_late_issue(3 * PERIOD), policy=st.AtProgress(4), poll_cycles=256)
    with pytest.raises(S.StopInconsistent, match="common prefix") as ei:
        rq.rerun(drv, m, progs, params=_params(progs, 40), results=["rq_status", "out"], stop=spec)
    rec = ei.value.record
    _check(rec, ei.value.out, 40, readers=(0, 1))
    assert rec.outcome == S.INCONSISTENT and rec.shots[0] < rec.shots[1] < rec.shots[2]
    assert S.session(drv).runs[-1].outcome == S.CERTIFIED and S.session(drv).pending_flush is None
    print(f"\n[P4 C2] late issue: shots {rec.shots}, prefix {rec.prefix}")
    _, rec = _go(grid, 6)
    assert rec.outcome == S.NATURAL
    spec = S.StopSpec(issue=_late_issue(0), policy=st.AtProgress(2), poll_cycles=256)
    _, rec = _go(grid, 10, spec, period=6_000)
    assert rec.outcome == S.CONSISTENT_LATE and rec.shots == {0: 2, 1: 2, 2: 2} and rec.late_by == 1
    _natural_after(grid)
    spec = S.StopSpec(issue=_late_issue(0, claim_verified=True), policy=st.AtProgress(4), poll_cycles=256)
    with pytest.raises(S.StopInternalError, match="VERIFIED"):
        _go(grid, 40, spec)
    s = S.session(drv)
    assert s.last_failure.kind == "INTERNAL_ERROR" and s.runs[-1].stop.outcome == S.INTERNAL_ERROR
    assert all(k.valid for k in s.runs[-1].stop.counts.values()) and s.pending_flush.reason == S.FLUSH_FAILED
    _natural_after(grid)


@pytest.mark.batch_cap(75_000)
def test_races_against_completion(grid):
    """Requests that meet the end of the run: NEXT when the cores have posted n - 3, n - 1 or all n
    shots, AT(None) at n - 1 and n, and a request after DONE. Every outcome matches the counts
    (STOPPED_EACH or NATURAL for NEXT; NATURAL for an AT whose earliest S is past n, or LATE once DONE
    was seen), and every run certifies.

    FLOOR: five reruns at ~5 k batches of host ops each plus up to 8 shots at 384 batches."""
    drv, m, progs = grid
    _tone(drv, m)
    got = []
    for pol in (st.AtProgress(5, S.NEXT), st.AtProgress(7, S.NEXT), st.AtProgress(8, S.NEXT),
                st.AtProgress(7), st.AtProgress(8)):
        _, rec = _go(grid, 8, _spec(pol, margin=0))
        got.append((pol.kind, pol.k, rec.outcome, rec.shots, rec.tickets))
        if pol.kind == S.AT and rec.request is not None:
            assert rec.outcome == S.NATURAL and rec.request["S"] >= 8
    print("\n[P4 race] " + "\n  ".join(map(str, got)))
    rid = S.session(drv).runs[-1].run_id
    assert rq.request_stop(drv, rid, S.NEXT).outcome == S.LATE


@pytest.mark.batch_cap(40_000)
def test_a_late_writer_of_an_old_epoch_cannot_stop_the_run(grid):
    """C3, B3's co-sim twin: a writer outside the mailbox puts the previous run's stop words
    (rq_stop_at = 0, its epoch) into every core mid-run; the kernel compares with this run's epoch,
    so the run is NATURAL. Requests naming the previous run, this epoch under the previous setup
    generation, or another epoch of this generation are LATE.

    FLOOR: two reruns at ~5 k batches of host ops each plus 10 + 12 shots at 384 batches, and the
    writer's ops."""
    drv, m, progs = grid
    _tone(drv, m)
    _, rec = _go(grid, 10)
    old = rec.run_id
    tickets = []

    def late_writer(ctx):
        if st.posted(ctx.drv, ctx.m, 0, ctx.progs[0]) >= 3 and not tickets:
            for c, p in ctx.progs.items():
                ctx.drv.write32(st.word_addr(ctx.m, c, p, "rq_stop_at"), 0)
                ctx.drv.write32(st.word_addr(ctx.m, c, p, "rq_stop_epoch"), old[1])
            g, e = ctx.run_id
            tickets.extend([rq.request_stop(ctx.drv, old, S.NEXT), rq.request_stop(ctx.drv, (g - 1, e), S.NEXT),
                            rq.request_stop(ctx.drv, (g, e ^ 0x100), S.AT, 0)])
    _, rec = _go(grid, 12, S.StopSpec(issue=st.Issuer(), policy=late_writer, poll_cycles=256))
    assert rec.outcome == S.NATURAL and [t.outcome for t in tickets] == [S.LATE] * 3


@pytest.mark.parametrize("fault,err,kind,why", [(1, TimeoutError, "TIMEOUT", st.TIMEOUT),
                                                (2, S.Unfinished, "UNFINISHED", st.UNFINISHED)])
@pytest.mark.batch_cap(55_000)
def test_a_failed_stoppable_run_records_invalid_counts_and_the_next_run_is_exact(grid, fault, err, kind, why):
    """C9: a hung kernel (TIMEOUT) and one that never publishes fin (UNFINISHED) fail with their
    counts recorded as invalid; the next run takes the hardware flush (FLUSH_ALLOWANCE in the suite)
    and is exact.

    FLOOR: two reruns at ~5 k batches of host ops each, 6 shots each, and the 20 k-cycle timeout of
    the hung run."""
    drv, m, progs = grid
    _tone(drv, m)
    with pytest.raises(err):
        _go(grid, 6, fault=fault, timeout=20_000)
    s = S.session(drv)
    rec = s.last_failure
    assert rec.kind == kind and s.runs[-1].outcome == S.FAILED
    assert {c: (k.valid, k.why) for c, k in rec.counts.items()} == {c: (False, why) for c in progs}
    assert all(k.shots == 6 for k in rec.counts.values())
    _, rec = _go(grid, 6)
    assert rec.outcome == S.NATURAL


@pytest.mark.slow
def test_200_back_to_back_runs_alternating_stop_and_no_stop(grid):
    """--slow (plan P4 v2 §8; B5's co-sim twin): 200 back-to-back runs of one loaded image, every other
    one stopped, alternately AT(None) at progress 1 (VERIFIED, FIRED) and NEXT at progress 2
    (STOPPED_EACH), the others unstopped (NATURAL). Every run certifies with exact counts; none fails."""
    from collections import Counter
    drv, m, progs = grid
    _tone(drv, m)
    tally = Counter()
    for i in range(200):
        if i % 2:
            _, rec = _go(grid, 6)
            assert rec.outcome == S.NATURAL
        elif i % 4 == 0:
            _, rec = _go(grid, 40, _spec(st.AtProgress(1)))
            assert rec.outcome == S.FIRED and rec.request["verified"], rec
        else:
            _, rec = _go(grid, 40, _spec(st.AtProgress(2, S.NEXT)))
            assert rec.outcome == S.STOPPED_EACH, rec
        tally[rec.outcome] += 1
    assert tally == {S.NATURAL: 100, S.FIRED: 50, S.STOPPED_EACH: 50}      # each certified (_go)
    print(f"\n[P4 slow] 200 runs: {dict(tally)}")


# ── module B: the uplink ──

@pytest.fixture(scope="module")
def up2(cosim_antq):
    """`k_grid` on sim-dio-antq: core 0 a reader, core 1 a heralded reader, each on its own ADC."""
    drv, m = cosim_antq
    progs = {0: _program(m, 0, [0, 1]), 1: _program(m, 1, [0, 1], herald=1)}
    rq.setup(drv, m, progs)
    return drv, m, progs


def _trunc28(v):
    """The run layer's `__uplink` estimate of a 32-bit integral: its 28-bit field plus the half step."""
    return ((np.asarray(v, dtype=np.int64).astype(np.int32) >> 4 << 4) + 8).astype(np.int32)


def _up(fx, n, spec=None, base=0x6000, timeout=600_000, **kw):
    drv, m, progs = fx
    nominal = {0: n, 1: _reads_of(n, True)}
    out = rq.rerun(drv, m, progs, params=_params(progs, n, **kw), results=["rq_status", "out"], stop=spec,
                   uplink=rq.UplinkRun(expected=nominal, base=base), timeout=timeout)
    run = S.session(drv).runs[-1]
    rec = run.stop
    assert run.outcome == S.CERTIFIED and run.proven and S.FLUSHED in run.states
    _check(rec, out, n, readers=(0, 1), herald=(1,))
    for c, k in rec.counts.items():                  # exactly the counted reads, equal to the CPU's
        up = out[c].get("__uplink", np.zeros(0, dtype=np.int32))
        assert len(up) == 2 * k.reads, (c, len(up), k)
        j = 2 * min(NOUT, k.reads)
        assert np.array_equal(up[:j], _trunc28(out[c]["out"][:j])), (c, up[:j], out[c]["out"][:j])
    rd = rq._readout(drv, m)
    assert rd.rejected() == [0, 0] and rd._rd(R.FINAL_ADDR) - base == 32 * (-(-sum(rec.reads.values()) // 4))
    return out, rec


@pytest.mark.batch_cap(110_000)
def test_the_uplink_certifies_exactly_the_stopped_shots(up2):
    """C1, C6, C7's reads: AT(S) VERIFIED and FIRED with the uplink, the drain expecting each core's
    counted reads (core 1's heralded count is not a multiple of its shots); NEXT, STOPPED_EACH, each
    core certified on its own count; NEXT before the first check, an empty run (final_addr ==
    run_base); and the next, unstopped run is exact.

    FLOOR: four uplink reruns at ~16 k batches each (the bench free-runs while the host polls
    prepare, flush and the drain; the G3' runs cost the same), plus ~25 + ~10 + 0 + 12 shots at 384
    batches and the 8 k-batch `pre`."""
    drv, m, progs = up2
    _tone(drv, m, adcs=(m.adc_of(0), m.adc_of(1)))
    _, rec = _up(up2, 40, _spec(st.AtProgress(3)))
    assert rec.outcome == S.FIRED and rec.request["verified"]
    S_ = rec.request["S"]
    assert rec.reads == {0: S_, 1: _reads_of(S_, True)}
    _, rec = _up(up2, 40, _spec(st.AtProgress(5, S.NEXT)))
    assert rec.outcome == S.STOPPED_EACH
    _, rec = _up(up2, 40, _spec(st.AtProgress(0, S.NEXT)), pre=8_000)
    assert rec.outcome == S.STOPPED_EACH and rec.shots == {0: 0, 1: 0} and rec.reads == {0: 0, 1: 0}
    _, rec = _up(up2, 12)
    assert rec.outcome == S.NATURAL and rec.reads == {0: 12, 1: 18}


@pytest.mark.batch_cap(45_000)
def test_the_landed_policy_stops_on_the_uplinks_cur_addr(up2):
    """§5.5, B2's co-sim twin: `Landed(20, 2.5)` (core 0 one word per shot, core 1 1.5 on average)
    watches CUR_ADDR, which advances per committed 64-word bank, and requests AT(None) once 20 shots
    have landed: the first bank, at ~26 shots. The run FIRES at S = v_pre + L + 2 + m and certifies.

    FLOOR: one uplink rerun (~16 k batches) and ~45 shots at 384 batches."""
    drv, m, progs = up2
    _tone(drv, m, adcs=(m.adc_of(0), m.adc_of(1)))
    _, rec = _up(up2, 60, _spec(st.Landed(20, 2.5)))
    r = rec.request
    assert r["policy"] == "landed" and r["landed_words"] >= 64 and r["landed_shots"] >= 20
    assert rec.outcome == S.FIRED and r["verified"] and set(rec.shots.values()) == {r["S"]}
    print(f"\n[P4 landed] stop_after 20: landed {r['landed_words']} words ({r['landed_shots']} shots), "
          f"v_pre {r['v_pre']}, S {r['S']}, overshoot {r['S'] - 20} shots")


@pytest.mark.batch_cap(65_000)
def test_a_hung_stoppable_uplink_run_fails_and_the_next_is_exact(up2):
    """C9 with the uplink: a hung stoppable kernel times out (FAILED, the admission flushed, counts
    invalid); the next uplink run takes the hardware flush and certifies exactly after an AT stop.

    FLOOR: two uplink reruns (~16 k batches each), the 20 k-cycle timeout and one flush."""
    drv, m, progs = up2
    _tone(drv, m, adcs=(m.adc_of(0), m.adc_of(1)))
    with pytest.raises(TimeoutError):
        _up(up2, 6, fault=1, timeout=20_000)
    s = S.session(drv)
    assert s.last_failure.kind == "TIMEOUT" and s.last_failure.cleanup[-1] == "admission: flushed (not drained)"
    assert all(not k.valid for k in s.last_failure.counts.values())
    _, rec = _up(up2, 40, _spec(st.AtProgress(2)))
    assert rec.outcome == S.FIRED and s.runs[-1].flushed == S.FLUSH_FAILED


# ── C5, completion after a stop (§5.4): every DAC sample precedes fin, nothing stale plays next ──

RO_DUR = 48                         # the readout drive outlasts the 40-batch demod: t_end is its end


@kernel
def k_ro(ro: ParamTable, demod: ParamTable, grp: Group, rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int,
         rq_status: Array, out: Array, code: int, n: int, period: int):
    """A stoppable readout: each shot plays the readout drive and its demod at t. After the
    epilogue (the last drive has ended, plus LEAD) it records out[0] = now() just before fin,
    out[1] = t of the last shot posted and out[2] = t of the first."""
    init_pulse_params(ro.pulses)  # noqa: F821
    init_pulse_params(demod.pulses)  # noqa: F821
    set_freq(ro, ro.freq)  # noqa: F821   (set_freq regenerates the phasors; without it the drive plays 0)
    set_freq(demod, code)  # noqa: F821
    t = barrier(grp) + period  # noqa: F821
    e = rq_epoch
    out[2] = t
    s = 0
    t_last = 0
    t_end = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        play(ro, ro["m"], t)  # noqa: F821
        play(demod, demod["sq"], t)  # noqa: F821
        t_last = t
        t_end = t + RO_DUR
        s = s + 1
        rq_status[0] = s
        rq_status[1] = s
        wait_until(t + READOUT_LEAD)  # noqa: F821
        read_res()  # noqa: F821
        t = t + period
    if s > 0:
        wait_until(t_end + LEAD)  # noqa: F821
        read_res()  # noqa: F821
    out[1] = t_last
    out[0] = now()  # noqa: F821
    rq_status[2] = e


@pytest.mark.batch_cap(55_000)
def test_completion_after_a_stop_every_dac_sample_precedes_fin(cosim_antq):
    """C5 (plan P4 v2 §5.4) on sim-dio-antq: an AT(S) stop of a readout that drives the `ro` DAC
    every shot. The DAC watch sees the first drive at the first shot's t, the last one start at the
    last posted shot's t and end RO_DUR later, S shots in all, and its last nonzero sample before
    the kernel's now() at fin: the stop posted nothing past its boundary and the epilogue waited out
    everything posted. The run after it (no request) starts its drive at its own first t, so nothing
    stale played in between; both runs certify through the uplink.

    FLOOR: the 1-core image load (~10 k batches) and two uplink reruns at ~16 k batches each plus
    ~22 and 3 shots at 384 batches."""
    drv, m = cosim_antq
    _tone(drv, m, adcs=(m.adc_of(0),))
    ro = ParamTable(m.channel_named("ro", 0), 0.0, {"m": Pulse(envelopes.square(RO_DUR), amp=0.5)})
    demod = ParamTable(m.channel_named("demod", 0), 0.0, {"sq": Pulse(envelopes.square(DUR), amp=1.0)})
    progs = {0: compile_kernel(k_ro, m, core=0, tables=dict(ro=ro, demod=demod), grp=Group([0], id=0),
                               out=Array(3), code=pack16(4 * F), period=PERIOD,
                               stop=StopConvention("n", at=True, lead=1, reads_per_shot=1))}
    rq.setup(drv, m, progs)
    dac = m.ro_dac(0)
    for n, spec in ((40, _spec(st.AtProgress(2))), (3, None)):
        h = drv.sim.dac_watch_start([dac])
        try:
            out = rq.rerun(drv, m, progs, params={0: {"n": n}}, results=["rq_status", "out"], stop=spec,
                           uplink=rq.UplinkRun(expected={0: n}, base=0x8000))
        finally:
            seen = drv.sim.dac_watch_stop(h)[dac]
        rec = st.last(drv)
        shots = rec.shots[0]
        t_fin, t_last, t_first = (int(x) for x in out[0]["out"])
        assert rec.outcome == (S.FIRED if spec is not None else S.NATURAL)
        assert shots == (rec.request["S"] if spec is not None else n) and len(out[0]["__uplink"]) == 2 * shots
        assert seen["first"] == t_first and seen["last_rise"] == t_last == t_first + (shots - 1) * PERIOD, seen
        assert seen["last"] + 1 == t_last + RO_DUR and seen["last"] < t_fin, (seen, t_fin)
        print(f"\n[P4 C5] {rec.outcome}: {shots} shots, drive {seen['first']}..{seen['last']}, fin at {t_fin} "
              f"({t_fin - seen['last'] - 1} batches after the last drive sample)")


# ── the remote seam (§4.3): the server-side runner publishes, the record comes back ──

@pytest.mark.batch_cap(30_000)
def test_a_remote_stoppable_run_through_the_server_side_runner(cosim):
    """A RemoteDriver (the hardware client) against the bench's DriverServer: the stop spec crosses as
    its wire form, the server's poll loop runs the policy and publishes AT(S) next to the sim, and the
    StopRecord comes back with the results (`riscq.stop.last` on the client). Runs last in the module:
    the remote setup loads core 0 of the shared sim-2q bench under the server's own session.

    FLOOR: the server-side 1-core image load (~8 k batches) and one rerun to ~22 shots at 384."""
    from riscq.driver.remote import RemoteDriver
    from riscq.map import SocMap, SocParams
    drv, m = cosim
    rdrv = RemoteDriver(str(drv._proxy._pyroUri))
    try:
        m2 = SocMap(SocParams.from_json(rdrv.board.get_params()))
        progs = {0: _program(m2, 0, [0])}
        rq.setup(rdrv, m2, progs)
        _tone(drv, m, adcs=(m.adc_of(0),))
        out = rq.rerun(rdrv, m2, progs, params=_params(progs, 40), results=["rq_status"],
                       stop=_spec(st.AtProgress(3)))
        rec = st.last(rdrv)
        S_ = rec.request["S"]
        assert rec.outcome == S.FIRED and rec.request["verified"] and rec.shots == {0: S_}
        assert rec.request["policy"] == "at_progress" and rec.tickets == [(S.AT, None, S.ACCEPTED)]
        assert [int(x) & 0xFFFF_FFFF for x in out[0]["rq_status"]] == [S_, S_, rec.run_id[1]]
    finally:
        rdrv.close()
