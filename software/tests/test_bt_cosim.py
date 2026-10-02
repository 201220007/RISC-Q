"""Co-sim of BT's kernels and tooling (qubic3 BT; PLAN_BT_v2 §1 T4 and T6, v1 §1 T4.5 and T6), on the RTL of the antq
builds the plan names, so the module runs under --results-path antq_uplink only (the gate's antq_bt step).

(a) T6 on sim-2q1c-antq (`cosim_2q1c`: three cores, all three demods on ADC 0, the readout DAC 2 shared): `k_bt` on
    cores 0-2, every run checked as the BT kit's `Session.bt` checks it (through the uplink, certified exactly, valid
    counts, reads = shots = words, t_fin - t_end >= LEAD, HUB_STATUS 0, slack posts = shots), plus the histogram
    summing to the posts and B_C3 <= min slack <= designed_slack(P) at P = 500 batches: NATURAL; AT(None) FIRED and
    VERIFIED; NEXT STOPPED_EACH; S = n NATURAL; S = 0 TOO_LATE; an AT through an in-process BoardServer (the kit's
    InProcRemote); `hang` = 1 TIMEOUT, the flush, the next run exact; `k_bt_poll` under the kit's PollWriter (no
    torn value, flips > 0); `k_nomark` UNPROVEN, the next setup's flush landing before its demod queued 6·10^4
    batches ahead (REJECTED and early_late still 0 past the due time, no STRAY), then P6 C2's dual capture (uplink -
    8 = RAM >> 4 << 4); the heralded k_batched with its drive-post slack (posts = kept). One watch of every DAC the
    build drives spans the module's T6 runs: every peak 0 (C7-all on sim-2q1c-antq).
(b) T4 on sim-2q-antq (`cosim`): a reduced ReadoutCalibration and Leakage on the 2-qubit RF-silent cal14 config,
    recorded (`bt_record.record`), run directly, replayed through an in-process BoardServer over the co-sim driver
    (`replay`, after the pinned hashes) and decoded (`decode`): Result.data equal to the direct run's, with a tone
    at the readout frequency so the IQ is nonzero. One watch of every DAC: every peak 0 (C7-all on sim-2q-antq).
(c) C-D on sim-dio-antq (`cosim_antq`, with the DIO bank) and sim-2q1c-antq: the non-silent twins of the BT kernels
    (`k_bt` with a gate and a readout drive every shot, and the DIO bank on sim-dio-antq's core 0: an AT stop, then
    the next run) and of the replayed and batched Experiments (ReadoutCalibration, Leakage, heralded and dual
    k_batched on the cal14 config with its drive amplitudes). One watch of every DAC and DIO bank, with the bench's
    completion monitors (each core's store to its marker word on its RAM port, the DONE rise, the core reset), from
    before the first release to after the last run. Per run and core: every output's last activity ends before the
    marker write, the marker write precedes DONE, the margin exceeds the 14q build's extra DAC alignment stages over
    the model's (Codex BT r2 #6), and nothing moves outside the runs."""

import sys
import types
from pathlib import Path

import numpy as np
import pytest

from riscq import ddr_regs as R
from riscq import run as rq
from riscq import session as S
from riscq import stop as st
from riscq.lang import Array, DioTable, Group, ParamTable, StopConvention, compile_kernel, kernel
from riscq.map import DIO_PIPE, LEAD, READOUT_LEAD, SocMap, SocParams
from riscq.pulses import Pulse, envelopes, units

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples"))
import bt_kernels as K  # noqa: E402
import bt_record as BR  # noqa: E402

pytestmark = [pytest.mark.cosim,
              pytest.mark.skipif("config.getoption('--results-path') != 'antq_uplink'",
                                 reason="PLAN_BT_v2 T4/T6: the antq builds (--results-path antq_uplink)")]

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
M14 = SocMap(SocParams.load(CONFIGS / "zcu216-14q-antq.json"))
P = K.BATCHES_PER_US          # 500 batches: the kit's 1 us period
M = 16                        # the AT margin (shots): the co-sim issue step costs ~2.4 k cycles on three cores
POLL = 256                    # poll_cycles
DUR, NBINS, BIN_LOG2 = K.DUR, K.NBINS, K.BIN_LOG2
AHEAD = 60_000                # k_nomark's lead in co-sim (batches; the kit's is 1 s): ~4x the ~15 k batches
                              # from its release to the next setup's flush pulse on three cores
RELAX = 400                   # the co-sim relax head (batches) in place of cal14's 6.4 us


def _s32(v):
    v = int(v) & 0xFFFF_FFFF
    return v - (1 << 32) if v >> 31 else v


def _seconds(n_batches, m):
    return units.ns(n_batches, m.params) * 1e-9


def _driven_dacs(m):
    return sorted({ch.dac for c in range(len(m.params.cores)) for ch in m.channels(c) if ch.dac is not None})


def _cal14(m):
    """`bt_cal14.yaml` for this build (`bt_record.cal14`, RF-silent) with the co-sim relax."""
    cfg = BR.cal14(m)
    cfg["reset/relax"] = _seconds(RELAX, m)
    return cfg


def _loud(cfg, m):
    """The non-silent twin of a cal14 config: the gate and readout amplitudes `cal14` sets before `rf_silent()`."""
    loud = cfg.copy()
    for q in range(len(m.params.cores)):
        loud[f"qubit/{q}/x90/amp"] = 0.5
        loud[f"readout/{q}/amp"] = 0.5
    return loud


def _tone(drv, m, cores):
    """A tone at the readout frequency (demod code 2048, cal14's readout/<q>/freq) on every ADC of `cores`."""
    f = float(units.demod_code_to_freq(2048, m.params))
    adcs = sorted({m.adc_of(c) for c in cores})
    spec = [{"kind": "tone", "adc": a, "freq_hz": f, "amp": 12000.0} for a in adcs]
    drv.sim.set_model(spec[0] if len(spec) == 1 else {"kind": "multi", "models": spec})


def _herald_exp(cfg, qs, shots, silent=True):
    """bt_kernels.experiments' heralded k_batched (Measure.counts(herald=True), one x90), with `shots` shots."""
    from riscq.cal.experiment import Experiment
    from riscq.cal.measure import Measure
    from riscq.cal.sequence import Gate
    return Experiment(cfg, qs, {q: [Gate("x90")] for q in qs}, {q: () for q in qs}, (), Measure.counts(herald=True),
                      shots, label="bt_herald", rf_silent=silent)


def _dual_exp(cfg, qs, shots, silent=True):
    """bt_kernels.experiments' dual capture (P6 C2: RAW in core RAM, rerun through the uplink), with `shots` shots."""
    from riscq.cal.experiment import Experiment
    from riscq.cal.measure import Measure
    return Experiment(cfg, qs, {q: [] for q in qs}, {q: () for q in qs}, (), Measure.raw(host=False), shots,
                      label="bt_dual", rf_silent=silent)


def _replayed(cfg, qs, ro_shots=8, leak_shots=4, leak_gates=3):
    """bt_record.experiments, reduced for co-sim: ReadoutCalibration (RAW through the uplink) and Leakage
    (levels on the host, the 2 values of LEAK_VALUES), name -> a factory of the cal object."""
    from riscq.cal.cals.readout import ReadoutCalibration
    from riscq.cal.cals.single import Leakage
    clf = BR.classifier3()
    return {"readout_cal": lambda: ReadoutCalibration(cfg, list(qs), shots=ro_shots),
            "leakage": lambda: Leakage(cfg, list(qs), {q: clf for q in qs}, "qubit/{q}/x90/vz",
                                       [list(v) for v in BR.LEAK_VALUES], n_gates=leak_gates, shots=leak_shots)}


# ── the kit's run and policy helpers, for a co-sim driver ──────────────────────────────────────────

class InProcRemote:
    """The BT kit's `InProcRemote` (evidence/BT/kit/bt_session.py): the `.remote` of a client bound to an
    in-process BoardServer, the production server path (remote_setup / remote_rerun) without Pyro."""

    def __init__(self, server):
        self.s = server

    def setup(self, params_json, progmap):
        self.s.remote_setup(params_json, progmap)

    def rerun(self, cores, params, arrays, results, timeout, identities=None, uplink=None, stop=None):
        kw = {} if stop is None else {"stop": stop}
        raw = self.s.remote_rerun(list(cores), dict(params), dict(arrays), results, int(timeout), identities,
                                  uplink, **kw)
        return {(c if c == "__stop" else int(c)): d for c, d in raw.items()}


class PollWriter:
    """The BT kit's phase 20 b4b policy: every poll writes 0 or 0xFFFF_FFFF (alternating) into every core's
    `word[0]`, the kernel's .data input array."""

    def __init__(self, drv, m, progs):
        self.drv, self.v, self.n = drv, 0, 0
        for c, p in progs.items():
            assert "word" in p.arrays and p.image.symbols["word"][0] < p.image.symbols["__bss_start"][0], c
        self.words = [st.word_addr(m, c, p, "word", 0) for c, p in sorted(progs.items())]

    def __call__(self, ctx):
        self.v ^= 0xFFFF_FFFF
        for a in self.words:
            self.drv.write32(a, self.v)
        self.n += 1
        return None


class Fx:
    """One build's run state: the driver, the map, the program sets (compiled once; by default T6's), the loaded
    one, the C7-all watch, and the lower bound of every run's minimum slack: B_C3 from the disassembly of `k_bt`'s
    image on this build, or `b_c3`."""

    def __init__(self, drv, m, sets=None, b_c3=None):
        self.drv, self.m = drv, m
        self.cores = list(range(len(m.params.cores)))
        if sets is None:
            sets = {"k_bt": K.compile_bt(m, self.cores), "k_bt_poll": K.compile_bt(m, self.cores, poll=1),
                    "k_nomark": K.compile_nomark(m, self.cores)}
        self.sets = dict(sets)
        self.c3 = K.posting_branches(self.sets["k_bt"][0]) if "k_bt" in self.sets else None
        self.b_c3 = self.c3["B_C3"] if b_c3 is None else int(b_c3)
        self.loaded = None
        self.watch = None
        self._base = 0

    def load(self, name, progs=None):
        """Set up a program set (once until another is loaded), as the kit's `Session.load`."""
        if progs is not None:
            self.sets[name] = progs
        if self.loaded != name:
            rq.setup(self.drv, self.m, self.sets[name])
            self.loaded = name
        return self.sets[name]

    def base(self):
        self._base = (self._base + 1) % 8
        return 0x10000 * (1 + self._base)


def _bt(fx, n, policy=None, margin=M, hang=0, extra=(), period=P, timeout=None):
    """One `k_bt`-family run as the kit's `Session.bt` makes and checks it, plus the slack histogram summing to
    the posts and B_C3 <= min slack <= designed_slack(period) on every core. Returns (record, out, run)."""
    drv, m = fx.drv, fx.m
    progs = fx.sets[fx.loaded]
    cores = sorted(progs)
    up = rq.UplinkRun(expected={c: n for c in cores}, base=fx.base())
    spec = None if policy is None else st.spec(policy, margin=margin, poll_cycles=POLL)
    out = rq.rerun(drv, m, progs, params={c: {"n": n, "period": period, "hang": hang} for c in cores},
                   results=["rq_status", "bt_t", "slack"] + list(extra),
                   timeout=timeout or 2 * n * period + 200_000, uplink=up, stop=spec)
    run = S.session(drv).runs[-1]
    rec = run.stop
    shots = {c: int(out[c]["rq_status"][0]) for c in cores}
    reads = {c: int(out[c]["rq_status"][1]) for c in cores}
    assert run.outcome == S.CERTIFIED and all(k.valid for k in rec.counts.values()), (run.outcome, rec.counts)
    assert shots == reads, (shots, reads)
    words = {c: len(out[c].get("__uplink", ())) // 2 for c in cores}
    assert words == reads, (words, reads)
    bt_t = {c: [_s32(x) for x in out[c]["bt_t"]] for c in cores}
    assert all(_s32(v[2] - v[1]) >= LEAD for v in bt_t.values()), bt_t
    assert drv.read32(m.host_ctrl + 0x54) == 0                      # HUB_STATUS
    slack = {c: [_s32(x) for x in out[c]["slack"]] for c in cores}
    assert all(slack[c][1] == shots[c] for c in cores), (slack, shots)
    assert all(sum(slack[c][2:2 + NBINS]) == slack[c][1] for c in cores), slack
    mins = {c: slack[c][0] for c in cores if slack[c][1]}
    hi = K.designed_slack(period)
    assert all(fx.b_c3 <= d <= hi for d in mins.values()), (mins, fx.b_c3, hi)
    req = rec.request or {}
    r = {"outcome": rec.outcome, "S": req.get("S"), "kind": req.get("kind"), "verified": req.get("verified"),
         "shots": sorted(set(shots.values())), "words": sum(words.values()),
         "min_slack": min(mins.values()) if mins else None, "flushed": run.flushed, "t0": bt_t[cores[0]][0],
         "hist": [sum(slack[c][2 + i] for c in cores) for i in range(NBINS)]}
    return r, out, run


def _exact_at(r, n, ncores):
    """The kit's `exact_at`: AT FIRED (or NATURAL when S >= n), VERIFIED, every core at min(S, n) shots, ncores·S
    words (proven: _bt's slack bound)."""
    want = min(r["S"], n)
    return r["kind"] == S.AT and r["verified"] and r["outcome"] in (S.FIRED, S.NATURAL) and r["shots"] == [want] \
        and r["words"] == want * ncores


# ── (a) T6 on sim-2q1c-antq ─────────────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def t6(cosim_2q1c):
    """The 3-core build with the T6 program sets compiled, and the C7-all watch of every DAC it drives, started
    before the module's first setup (stopped and checked by the module's last T6 test)."""
    drv, m = cosim_2q1c
    assert m.params.with_antq_uplink and len(m.params.cores) == 3
    drv.sim.set_model({"kind": "zero"})
    fx = Fx(drv, m)
    fx.dacs = _driven_dacs(m)
    fx.watch = drv.sim.dac_watch_start(fx.dacs)
    yield fx
    if fx.watch is not None:
        drv.sim.dac_watch_stop(fx.watch)


@pytest.mark.batch_cap(130_000)
def test_t6_k_bt_natural_and_at_fired_verified_with_its_slack(t6):
    """T6 (kit phases 12, 17): `k_bt` on three cores at P = 500 batches. NATURAL (8 shots); AT(None) at progress 2
    with margin 16, FIRED and VERIFIED, every core at S, 3·S words; the next run exact. Every run: the kit's
    checks and its own slack (B_C3 <= min <= designed_slack(500)).

    FLOOR: the 3-core image load (~30 k batches), three uplink reruns at ~16 k batches of host ops each, and
    8 + ~25 + 4 shots at 500 batches."""
    fx = t6
    fx.load("k_bt")
    r, _, _ = _bt(fx, 8)
    assert r["outcome"] == S.NATURAL and r["shots"] == [8] and r["words"] == 3 * 8, r
    natural = r
    r, _, _ = _bt(fx, 60, st.AtProgress(2))
    assert _exact_at(r, 60, 3) and r["outcome"] == S.FIRED, r
    at = r
    r, _, _ = _bt(fx, 4)
    assert r["outcome"] == S.NATURAL and r["shots"] == [4] and r["words"] == 12, r
    print(f"\n[BT T6] k_bt C3 {fx.c3['conditional_branches']} branches, B_C3 {fx.c3['B_C3']}, designed slack "
          f"{K.designed_slack(P)} | NATURAL min slack {natural['min_slack']} | AT S {at['S']} FIRED VERIFIED, "
          f"min slack {at['min_slack']}, histogram {at['hist']} | next exact")


@pytest.mark.batch_cap(140_000)
def test_t6_next_s_at_n_and_s_zero(t6):
    """T6 (kit phase 17): NEXT at progress 5 is STOPPED_EACH; an explicit S = n is VERIFIED and NATURAL; an explicit
    S = 0 is TOO_LATE, refused before any write, and the run NATURAL; the next run exact.

    FLOOR: four uplink reruns at ~16 k batches of host ops each and ~10 + 24 + 10 + 4 shots at 500 batches."""
    fx = t6
    fx.load("k_bt")
    r, _, _ = _bt(fx, 40, st.AtProgress(5, S.NEXT))
    assert r["outcome"] == S.STOPPED_EACH, r
    nxt = r
    r, _, _ = _bt(fx, 24, st.AtProgress(2, S.AT, 24))
    assert r["outcome"] == S.NATURAL and r["verified"] and r["shots"] == [24], r
    r, _, run = _bt(fx, 10, st.AtProgress(1, S.AT, 0))
    assert r["outcome"] == S.NATURAL and r["kind"] is None and [t.outcome for t in run.tickets] == [S.TOO_LATE], \
        (r, run.tickets)
    r, _, _ = _bt(fx, 4)
    assert r["outcome"] == S.NATURAL and r["shots"] == [4]
    print(f"\n[BT T6] NEXT at 5: STOPPED_EACH at {nxt['shots']}; S = n NATURAL verified; S = 0 TOO_LATE; next exact")


@pytest.mark.batch_cap(110_000)
def test_t6_an_at_through_an_in_process_board_server(t6):
    """T6 (kit phase 17's remote AT): a client whose `.remote` is the kit's InProcRemote over an in-process
    BoardServer on the co-sim driver. The server loads its own copy of `k_bt`; AT(None) at progress n/4 crosses as
    the stop spec's wire form, the server's poll loop issues it, and the StopRecord comes back: FIRED and VERIFIED,
    3·S uplink words. The next direct run is exact (the server's set is the same programs).

    FLOOR: the server-side 3-core image load (~30 k batches), one uplink rerun (~16 k) to ~35 shots at 500 batches,
    and one direct uplink run of 4 shots."""
    from riscq.board.server import BoardServer
    fx = t6
    drv, m = fx.drv, fx.m
    progs = fx.sets["k_bt"]
    text = drv.sim.get_params()
    server = BoardServer(driver=drv, params_text=text)
    client = types.SimpleNamespace(remote=InProcRemote(server), board=types.SimpleNamespace(get_params=lambda: text))
    rq.setup(client, m, progs)
    fx.loaded = "k_bt"                         # the server's copy is the same programs
    n = 60
    up = rq.UplinkRun(expected={c: n for c in progs}, base=fx.base())
    out = rq.rerun(client, m, progs, params={c: {"n": n, "period": P, "hang": 0} for c in progs},
                   results=["rq_status"], timeout=2 * n * P + 200_000, uplink=up,
                   stop=st.spec(st.AtProgress(n // 4), margin=M, poll_cycles=POLL))
    rec = st.last(client)
    req = rec.request or {}
    words = sum(len(out[c].get("__uplink", ())) // 2 for c in progs)
    assert rec.outcome == S.FIRED and req.get("verified") and set(rec.shots.values()) == {req["S"]}, rec
    assert words == 3 * req["S"] and req.get("policy") == "at_progress", (words, req)
    assert S.session(drv).runs[-1].outcome == S.CERTIFIED
    r, _, _ = _bt(fx, 4)
    assert r["outcome"] == S.NATURAL and r["shots"] == [4]
    print(f"\n[BT T6] remote AT at {n // 4}: {rec.outcome}, S {req['S']} VERIFIED, {words} words; next direct run exact")


@pytest.mark.batch_cap(85_000)
def test_t6_a_hung_k_bt_times_out_the_flush_follows_and_the_next_run_is_exact(t6):
    """T6 (kit phase 13): `hang` = 1 halts after the loop: TIMEOUT, FAILED, the counts invalid, a FAILED flush
    pending; the next run takes the hardware flush (FLUSH_ALLOWANCE) and is exact.

    FLOOR: one uplink rerun of 6 shots with its 20 k-cycle timeout, the flush, and one uplink run of 4 shots."""
    fx = t6
    fx.load("k_bt")
    s = S.session(fx.drv)
    with pytest.raises(TimeoutError):
        _bt(fx, 6, hang=1, timeout=20_000)
    f = s.last_failure
    assert f.kind == "TIMEOUT" and s.runs[-1].outcome == S.FAILED, f
    assert f.counts and not any(k.valid for k in f.counts.values()), f.counts
    assert s.pending_flush is not None and s.pending_flush.reason == S.FLUSH_FAILED
    r, _, _ = _bt(fx, 4)
    assert r["outcome"] == S.NATURAL and r["shots"] == [4] and r["flushed"] == S.FLUSH_FAILED, r
    print(f"\n[BT T6] hang: {f.kind} at {f.stage}, counts invalid; next run flushed ({r['flushed']}) and exact")


@pytest.mark.batch_cap(90_000)
def test_t6_k_bt_poll_under_the_poll_writer_sees_no_torn_word(t6):
    """T6 (kit phase 20 b4b): `k_bt_poll` reads `word[0]` after every post while the kit's PollWriter writes 0 or
    0xFFFF_FFFF into every core's word at every poll: no value other than the two (odd = 0), and the word changed
    under the kernel (flips > 0) on every core.

    FLOOR: the 3-core image load (~30 k batches), one uplink rerun (~16 k) of 40 shots at 500 batches with a
    3-write policy at every poll."""
    fx = t6
    progs = fx.load("k_bt_poll")
    w = PollWriter(fx.drv, fx.m, progs)
    r, out, _ = _bt(fx, 40, w, extra=("pollc",))
    odd = {c: int(out[c]["pollc"][0]) for c in progs}
    flips = {c: int(out[c]["pollc"][1]) for c in progs}
    assert r["outcome"] == S.NATURAL and r["shots"] == [40], r
    assert not any(odd.values()) and all(v > 0 for v in flips.values()), (odd, flips, w.n)
    print(f"\n[BT T6] b4b: {w.n} polls wrote every core's word; odd {odd}, flips {flips}")


@pytest.mark.batch_cap(210_000)
def test_t6_k_nomark_is_unproven_and_the_flush_cancels_its_queued_demod(t6):
    """T6 (kit phase 14) and P6 C2: `k_nomark` posts one demod now and one due AHEAD = 6·10^4 batches later and
    has no marker: the run certifies UNPROVEN with a flush pending. The next setup (P6 C2's dual capture) takes the
    flush, its pulse landing before the due time; past the due time in the new time base (a recover() first keeps
    the due time small in both) REJECTED and early_late are still 0. The dual run then finds no stray result and
    certifies exactly: every core's uplink words - 8 equal its RAM IQ >> 4 << 4, with a tone on ADC 0.

    FLOOR: two 3-core image loads (~30 k batches each), two flushes (FLUSH_ALLOWANCE), the k_nomark uplink run
    (~16 k), the advance past the due time (~4·10^4 after the setup), and the dual uplink run (~16 k) of 16
    shots at the co-sim relax."""
    fx = t6
    drv, m = fx.drv, fx.m
    s = S.session(drv)
    rd = rq._readout(drv, m)
    progs = fx.load("k_nomark")
    rq.recover(drv, m)                       # restarts the batch time: the due time stays small in either base
    cores = sorted(progs)
    t_before = drv.sim.batch_time()
    rq.rerun(drv, m, progs, params={c: {"ahead": AHEAD} for c in cores}, results=["out"], timeout=200_000,
             uplink=rq.UplinkRun(expected={c: 1 for c in cores}, base=fx.base()))
    t_after = drv.sim.batch_time()
    run = s.runs[-1]
    assert run.outcome == S.CERTIFIED and run.proven is False, (run.outcome, run.proven)
    assert s.pending_flush is not None and s.pending_flush.reason == S.FLUSH_UNPROVEN
    n0 = drv.sim.pl_reset_snapshot()["n"]
    cfg = _cal14(m)
    dual = _dual_exp(cfg, cores, 16)
    dprogs, par, timeout, npts = K.compile_experiment(dual, m)
    fx.load("dual", dprogs)                  # its setup takes the UNPROVEN flush
    snap = drv.sim.pl_reset_snapshot()
    assert snap["n"] == n0 + 1 and snap["batch_time"] < t_before + AHEAD, (snap["batch_time"], t_before + AHEAD)
    drv.sim.advance(max(0, t_after + AHEAD + 500 - drv.sim.batch_time()))
    rej, early_late = rd.rejected(), rd.status() >> R.S_EARLY_LATE & 1
    assert not any(rej) and not early_late, (rej, early_late)
    _tone(drv, m, cores)
    try:
        k = dual.shots * npts
        out = rq.rerun(drv, m, dprogs, params=par, results=["out"], timeout=timeout,
                       uplink=rq.UplinkRun(expected={c: k for c in cores}, base=fx.base()))
    finally:
        drv.sim.set_model({"kind": "zero"})
    assert s.runs[-1].outcome == S.CERTIFIED and s.runs[-1].flushed is None     # its quiesce found no STRAY
    assert not any(x.startswith("STRAY") for x in list(s.notes)[-4:]), list(s.notes)[-4:]
    for c in cores:
        ram = np.asarray(out[c]["out"][:2 * k], dtype=np.int64)
        up = np.asarray(out[c]["__uplink"], dtype=np.int64)
        field = (ram.astype(np.int32) >> 4 << 4).astype(np.int64)
        assert len(up) == 2 * k and np.array_equal(up - 8, field), c
        assert np.abs(ram).max() > 0, c
    print(f"\n[BT T6] k_nomark UNPROVEN; the flush pulse at batch {snap['batch_time']}, before the due time "
          f"(> {t_before + AHEAD}); REJECTED {rej}, early_late 0 past it; dual capture: {k} shots per core, "
          f"uplink - 8 == RAM >> 4 << 4 on cores {cores}")


@pytest.mark.batch_cap(80_000)
def test_t6_heralded_k_batched_records_its_drive_post_slack(t6):
    """T6 (kit phases 12, 20 b4c, 25): the heralded k_batched (COUNTS in core RAM, uplink-free) instrumented with
    `bt_slack` (`compile_experiment(herald_slack=True)`): every core's drive posts equal its kept shots. With the
    zero ADC model every herald passes (kept = shots), so the outcome variation is a board-only requirement (Codex
    BT r2 #5). The minimum slack is reported, not bounded: the drive posts when the herald read returns, which by
    `herald_offset`'s design is at about the LEAD deadline (P4 REPORT, M2 on k_batched).

    FLOOR: the 3-core image load (~30 k batches) and one rerun of 12 heralded shots at the co-sim relax."""
    fx = t6
    drv, m = fx.drv, fx.m
    cores = fx.cores
    herald = _herald_exp(_cal14(m), cores, 12)
    progs, par, timeout, npts = K.compile_experiment(herald, m, herald_slack=True)
    fx.load("herald", progs)
    out = rq.rerun(drv, m, progs, params=par, results=["out"], timeout=timeout)
    assert S.session(drv).runs[-1].outcome == S.CERTIFIED
    rec = {}
    for c, p in sorted(progs.items()):
        a, size = p.image.symbols["bt_slack"]
        raw = drv.read_block(m.to_host_addr(c, a), size)
        w = [_s32(int.from_bytes(raw[i:i + 4], "little")) for i in range(0, size, 4)]
        kept = int(out[c]["out"][1])
        rec[c] = {"min_slack": w[0], "posts": w[1], "kept": kept, "hist": w[2:2 + NBINS], "first_deadline": w[-1]}
        assert w[1] == kept == herald.shots * npts, (c, w[1], kept)
        assert sum(w[2:2 + NBINS]) == w[1], (c, w)
    print(f"\n[BT T6] heralded k_batched: posts = kept = {herald.shots * npts} per core; min slack per core "
          f"{ {c: v['min_slack'] for c, v in rec.items()} } batches (LEAD {LEAD}); hist core 0 {rec[0]['hist']}")


def test_t6_c7_all_every_dac_of_sim_2q1c_antq_stayed_zero(t6):
    """C7-all on sim-2q1c-antq: the watch of every DAC the build drives (gate DACs 0, 1, 3 and the shared readout
    DAC 2), running since before the module's first setup through every T6 run above, saw no nonzero sample.
    (No batch cap: it only stops the watch.)"""
    fx = t6
    seen = fx.drv.sim.dac_watch_stop(fx.watch)
    fx.watch = None
    assert sorted(seen) == fx.dacs and fx.dacs == [0, 1, 2, 3], (sorted(seen), fx.dacs)
    assert all(seen[d]["peak"] == 0 and seen[d]["stretches"] == [] for d in fx.dacs), seen
    batches = seen[fx.dacs[0]]["batches"]
    assert batches > 100_000, batches
    print(f"\n[BT C7-all] sim-2q1c-antq: DACs {fx.dacs} watched for {batches} batches over the T6 runs: every peak 0")


# ── (b) T4 on sim-2q-antq ───────────────────────────────────────────────────────────────────────────

def _same(a, b):
    """Result.data trees equal: arrays and numbers exactly (NaN equal to NaN), recursively."""
    if isinstance(a, dict):
        return isinstance(b, dict) and a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        a, b = np.asarray(a), np.asarray(b)
        try:
            return np.array_equal(a, b, equal_nan=True)
        except TypeError:
            return np.array_equal(a, b)
    if isinstance(a, (list, tuple)):
        return isinstance(b, (list, tuple)) and len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if isinstance(a, float) and isinstance(b, float) and a != a and b != b:
        return True
    return a == b


@pytest.mark.batch_cap(330_000)
def test_t4_record_replay_decode_equals_the_direct_run_and_every_dac_stays_zero(cosim, tmp_path):
    """T4 (kit phase 22, p6_b1) and C7-all on sim-2q-antq: ReadoutCalibration (8 shots per prep state) and Leakage
    (3 gates, 4 shots, LEAK_VALUES) on the 2-qubit RF-silent cal14 config, with a tone at the readout frequency on
    both ADCs. Each is recorded (twice, zero and seeded-noise replies, equal call lists), audited, run directly on
    the co-sim, replayed through an in-process BoardServer over the co-sim driver after the pinned hashes, and
    decoded from the replies: Result.data equal to the direct run's. One watch of every DAC the build drives (0, 1
    and the shared readout DAC 14), from before the first setup to after the last replay: every peak 0.

    FLOOR: per Experiment and pass (direct, replay) one 2-core image load (~20 k batches) per setup and one uplink
    rerun (~16 k) per point set: ReadoutCalibration 1 setup + 1 rerun, Leakage 2 setups + 2 reruns, so 6 setups
    and 6 reruns, plus their shots at the co-sim relax."""
    from riscq.board.server import BoardServer
    drv, m = cosim
    assert m.params.with_antq_uplink and len(m.params.cores) == 2
    cores = [0, 1]
    cfg = _cal14(m)
    exps = _replayed(cfg, cores)
    dacs = _driven_dacs(m)
    _tone(drv, m, cores)
    h = drv.sim.dac_watch_start(dacs)
    got = {}
    try:
        for name, factory in exps.items():
            rec = BR.record(name, factory, m)
            assert BR.audit_record(rec, m) == [], name
            index = {name: [c["sha256"] for c in rec["calls"]]}
            direct = factory().run(drv)
            BR.check_pinned(rec, index)
            server = BoardServer(driver=drv, params_text=drv.sim.get_params())
            done = BR.replay(server, rec, tmp_path / "replies", log=lambda *a: None)
            assert [d[1] for d in done] == [c["op"] for c in rec["calls"]], done
            assert S.session(drv).runs[-1].outcome == S.CERTIFIED and S.session(drv).runs[-1].uplink
            decoded = BR.decode(name, factory, m, rec, tmp_path / "replies")
            assert _same(decoded.data, direct.data), name
            got[name] = (len(rec["calls"]), sum(d[1] == "rerun" for d in done))
    finally:
        seen = drv.sim.dac_watch_stop(h)
        drv.sim.set_model({"kind": "zero"})
    iq = np.asarray(direct.data[0]["y"]) if "y" in direct.data[0] else None
    assert all(seen[d]["peak"] == 0 and seen[d]["stretches"] == [] for d in dacs), seen
    assert dacs == [0, 1, 14], dacs
    BR.write_manifest(tmp_path / "replies")
    print(f"\n[BT T4] sim-2q-antq: {got} (calls, reruns) recorded, replayed through the BoardServer and decoded: "
          f"Result.data equal to the direct runs (Leakage y {None if iq is None else iq.tolist()}); "
          f"C7-all: DACs {dacs} over {seen[dacs[0]]['batches']} batches, every peak 0")


# ── (c) C-D: completion on the non-silent twins ─────────────────────────────────────────────────────

@kernel
def k_bt_loud(gate: ParamTable, ro: ParamTable, demod: ParamTable, grp: Group, rq_epoch: int, rq_stop_epoch: int,
              rq_stop_at: int, rq_status: Array, bt_t: Array, slack: Array, code: int, n: int, period: int,
              hang: int):
    """`k_bt`'s non-silent twin (C-D): its grid, stop checks, slack, `hang`, epilogue and marker, and every shot
    plays the gate drive (16 batches) and the readout drive (DUR) with the demod at t."""
    init_pulse_params(gate.pulses)  # noqa: F821
    init_pulse_params(ro.pulses)  # noqa: F821
    init_pulse_params(demod.pulses)  # noqa: F821
    set_freq(gate, gate.freq)  # noqa: F821
    set_freq(ro, ro.freq)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    t = barrier(grp) + period  # noqa: F821
    t0 = t
    e = rq_epoch
    s = 0
    t_end = 0
    dmin = 1073741824
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        play(gate, gate["g"], t)  # noqa: F821
        play(ro, ro["m"], t)  # noqa: F821
        play(demod, demod["sq"], t)  # noqa: F821
        d = t - LEAD - now()  # noqa: F821
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
        wait_until(t + READOUT_LEAD)  # noqa: F821
        read_res()  # noqa: F821
        t = t + period
    if hang == 1:
        wait_until(now() + 1073741824)  # noqa: F821
    if s > 0:
        wait_until(t_end + LEAD)  # noqa: F821
        read_res()  # noqa: F821
    slack[0] = dmin
    slack[1] = s
    bt_t[0] = t0
    bt_t[1] = t_end
    bt_t[2] = now()  # noqa: F821
    rq_status[2] = e


@kernel
def k_bt_loud_dio(gate: ParamTable, ro: ParamTable, demod: ParamTable, ttl: ParamTable, grp: Group, rq_epoch: int,
                  rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, bt_t: Array, slack: Array, code: int, n: int,
                  period: int, hang: int):
    """`k_bt_loud` that also pulses the core's DIO bank every shot (on at t, off 6 batches later)."""
    init_pulse_params(gate.pulses)  # noqa: F821
    init_pulse_params(ro.pulses)  # noqa: F821
    init_pulse_params(demod.pulses)  # noqa: F821
    init_pulse_params(ttl.pulses)  # noqa: F821
    set_freq(gate, gate.freq)  # noqa: F821
    set_freq(ro, ro.freq)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    t = barrier(grp) + period  # noqa: F821
    t0 = t
    e = rq_epoch
    s = 0
    t_end = 0
    dmin = 1073741824
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        play(gate, gate["g"], t)  # noqa: F821
        play(ro, ro["m"], t)  # noqa: F821
        play(ttl, ttl["on"], t)  # noqa: F821
        fire(ttl, ttl["off"])  # noqa: F821
        play(demod, demod["sq"], t)  # noqa: F821
        d = t - LEAD - now()  # noqa: F821
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
        wait_until(t + READOUT_LEAD)  # noqa: F821
        read_res()  # noqa: F821
        t = t + period
    if hang == 1:
        wait_until(now() + 1073741824)  # noqa: F821
    if s > 0:
        wait_until(t_end + LEAD)  # noqa: F821
        read_res()  # noqa: F821
    slack[0] = dmin
    slack[1] = s
    bt_t[0] = t0
    bt_t[1] = t_end
    bt_t[2] = now()  # noqa: F821
    rq_status[2] = e


def _compile_loud(m, cores):
    """The non-silent `k_bt` twin on `cores`, one barrier group; a core with a DIO bank pulses it too."""
    conv = StopConvention("n", at=True, lead=1, reads_per_shot=1)
    grp = Group(sorted(cores), id=0)
    progs = {}
    for c in sorted(cores):
        tables = dict(
            gate=ParamTable(m.channel_named("gate", c), 50e6, {"g": Pulse(envelopes.square(64), freq_hz=50e6, amp=0.5)}),
            ro=ParamTable(m.channel_named("ro", c), 0.0, {"m": Pulse(envelopes.square(DUR), amp=0.5)}),
            demod=K.demod_table(m, c))
        dio = [ch for ch in m.channels(c) if ch.kind == "dio"]
        fn = k_bt_loud
        if dio:
            tables["ttl"] = DioTable(dio[0], {"on": (0x0001, 0x0001, 6), "off": (0x0001, 0x0000, 6)})
            fn = k_bt_loud_dio
        progs[c] = compile_kernel(fn, m, core=c, tables=tables, grp=grp, rq_status=Array(3), bt_t=Array(3),
                                  slack=Array(K.SLACK_WORDS), code=K.demod_code(m), stop=conv)
    return progs


def _marker_cpu(prog):
    """The CPU address of a program's completion marker word."""
    name, index = prog.marker
    return prog.var_addr(name) + 4 * index


def _outputs(m):
    """Every watched output of the build and the cores that drive it: DAC id or "dio:<port>" -> [cores]."""
    out = {}
    for c in range(len(m.params.cores)):
        for ch in m.channels(c):
            if ch.dac is not None:
                out.setdefault(ch.dac, []).append(c)
            elif ch.kind == "dio":
                out.setdefault(f"dio:{m.params.cores[c].name}_{ch.name}", []).append(c)
    return out


def _completion(drv, m, monkeypatch, label):
    """C-D on one build: the non-silent twins under one watch of every DAC and DIO bank with the completion
    monitors; returns the per-run, per-core report after asserting the order on every run.

    The twins: `k_bt_loud` (an AT stop, then the next run, as the kit runs `k_bt`); ReadoutCalibration and
    Leakage on the cal14 config with its drive amplitudes (run as cals: their setups and reruns); the heralded
    and the dual k_batched compiled as the kit compiles them (`compile_experiment`) and run as the kit runs them
    (the heralded one uplink-free, the dual one through the uplink)."""
    cores = list(range(len(m.params.cores)))
    cfg = _loud(_cal14(m), m)
    loud = _compile_loud(m, cores)
    cals = _replayed(cfg, cores, ro_shots=4, leak_shots=4)
    herald = _herald_exp(cfg, cores, 8, silent=False)
    hprogs, hpar, htimeout, _ = K.compile_experiment(herald, m, herald_slack=True)
    dual = _dual_exp(cfg, cores, 8, silent=False)
    dprogs, dpar, dtimeout, dnpts = K.compile_experiment(dual, m)
    marks = {c: {_marker_cpu(p) for p in (loud[c], hprogs[c], dprogs[c])} for c in cores}
    for name, factory in cals.items():       # every setup's programs, from the record (the same compile)
        rec = BR.record(name, factory, m)
        for call in (x for x in rec["calls"] if x["op"] == "setup"):
            for c, w in call["args"]["progmap"].items():
                p = rq._prog_from_wire(w)
                assert p.marker is not None, (name, c)
                marks[int(c)].add(_marker_cpu(p))
    runs, step = [], ["k_bt_loud"]
    orig = rq.rerun

    def logged(d, mm, progs, *a, **k):
        if d is not drv:
            return orig(d, mm, progs, *a, **k)
        c0 = drv.sim.cycles()
        try:
            return orig(d, mm, progs, *a, **k)
        finally:
            run = S.session(drv).runs[-1]
            runs.append({"c0": c0, "c1": drv.sim.cycles(), "run_id": run.run_id, "outcome": run.outcome,
                         "label": step[0],
                         "marker": {int(c): (_marker_cpu(p), run.run_id[1] if p.stop else rq.MARKER_DONE)
                                    for c, p in progs.items()}})
    monkeypatch.setattr(rq, "rerun", logged)
    s = S.session(drv)
    rq.quiesce(drv, m)                       # an earlier module's leftovers flush here, before the watch's time base
    flushes0, pulses0 = len(s.flushes), (drv.sim.pl_reset_snapshot() or {"n": 0})["n"]
    outs = _outputs(m)
    dios = [o[4:] for o in outs if isinstance(o, str)]
    for port in dios:
        drv.sim.dio_loopback(port, False)
    _tone(drv, m, cores)
    h = drv.sim.dac_watch_start(sorted(o for o in outs if not isinstance(o, str)), dios=dios,
                                marks={c: sorted(v) for c, v in marks.items()})
    try:
        fx = Fx(drv, m, sets={"k_bt_loud": loud}, b_c3=0)    # the twin's posts carry two drives: every post
        fx.load("k_bt_loud")                                   # must still meet its deadline
        r, _, _ = _bt(fx, 40, st.AtProgress(2))
        assert _exact_at(r, 40, len(cores)) and r["outcome"] == S.FIRED, r
        r2, _, _ = _bt(fx, 3)
        assert r2["outcome"] == S.NATURAL and r2["shots"] == [3], r2
        for name, factory in cals.items():
            step[0] = name
            factory().run(drv)
        step[0] = "herald"
        rq.setup(drv, m, hprogs)
        rq.rerun(drv, m, hprogs, params=hpar, results=["out"], timeout=htimeout)
        step[0] = "dual"
        rq.setup(drv, m, dprogs)
        k = dual.shots * dnpts
        rq.rerun(drv, m, dprogs, params=dpar, results=["out"], timeout=dtimeout,
                 uplink=rq.UplinkRun(expected={c: k for c in cores}, base=0x20000))
    finally:
        seen = drv.sim.dac_watch_stop(h)
        drv.sim.set_model({"kind": "zero"})
    assert len(s.flushes) == flushes0 and (drv.sim.pl_reset_snapshot() or {"n": 0})["n"] == pulses0, \
        "a flush inside the watch: one time base is needed"
    assert [r["label"] for r in runs][:2] == ["k_bt_loud", "k_bt_loud"] and runs[-1]["label"] == "dual", runs
    return _order(m, seen, runs, outs, label)


def _order(m, seen, runs, outs, label):
    """Assert the completion order of every logged run from the watch and the monitors (see the module docstring);
    return the per-run report."""
    mon = seen["mon"]
    resets, dones = mon["reset"], mon["done"]
    wins = []                                                     # [fall cycle, fall tb, rise cycle, rise tb]
    for i, (cy, tb, v) in enumerate(resets):
        if v == 0:
            nxt = resets[i + 1] if i + 1 < len(resets) else None
            assert nxt is not None, "the cores were still released when the watch stopped"
            wins.append((cy, tb, nxt[0], nxt[1]))
    pipe = {o: (DIO_PIPE if isinstance(o, str) else m.dac_pipe(o)) for o in outs}
    extra = M14.dac_pipe(0) - m.dac_pipe(0)                       # the 14q build's extra DAC alignment stages
    used = set()
    report = []
    for run in runs:
        assert run["outcome"] == S.CERTIFIED, run
        mine = [w for w in wins if run["c0"] <= w[0] < run["c1"]]
        assert len(mine) == 1, (run, mine)
        fall, fall_tb, rise, rise_tb = mine[0]
        used.add(mine[0])
        row = {"label": run["label"], "cores": {}}
        for c, (addr, value) in sorted(run["marker"].items()):
            writes = [e for e in mon["marks"][c] if fall <= e[0] < rise and e[2] == addr]
            up = [e for e in dones if fall <= e[0] <= rise and e[2] >> c & 1]
            assert writes and up, (run["label"], c, writes, up)
            done_cy = up[0][0]
            mk = [e for e in writes if e[0] < done_cy]
            assert mk and mk[-1][3] & 0xFFFF_FFFF == value & 0xFFFF_FFFF, (run["label"], c, mk, value)
            mark_cy, mark_tb = mk[-1][0], mk[-1][1]
            assert mark_cy < done_cy, (run["label"], c, mark_cy, done_cy)
            margins = {}
            for o, owners in outs.items():
                if c not in owners:
                    continue
                ends = [b + pipe[o] for a, b in seen[o]["stretches"] if fall_tb <= a + pipe[o] <= rise_tb]
                if ends:
                    margins[o] = mark_tb - max(ends)
            assert margins, (run["label"], c, "no output activity: the order would be vacuous")
            assert all(v > extra for v in margins.values()), (run["label"], c, margins, extra)
            row["cores"][c] = {"marker_minus_last": margins, "done_minus_marker": done_cy - mark_cy}
        report.append(row)
    for o in outs:                                                # nothing moves outside the runs
        for a, b in seen[o]["stretches"]:
            assert any(w[1] <= a + pipe[o] and b + pipe[o] <= w[3] for w in used), (label, o, [a, b])
        assert len(seen[o]["stretches"]) < 4096, o
    return report, extra


def _summary(report):
    """The minimum marker - last-activity margin and DONE - marker gap per run label."""
    out = {}
    for row in report:
        lo = min(v for cc in row["cores"].values() for v in cc["marker_minus_last"].values())
        dm = min(cc["done_minus_marker"] for cc in row["cores"].values())
        a, b = out.get(row["label"], (lo, dm))
        out[row["label"]] = (min(a, lo), min(b, dm))
    return out


@pytest.mark.batch_cap(360_000)
def test_cd_completion_order_on_sim_dio_antq(cosim_antq, monkeypatch):
    """C-D on sim-dio-antq (core 0: gate DAC 0, readout DAC 14, the DIO bank q0_ttl; core 1: gate DAC 1, readout DAC
    14): the non-silent twins under one watch of DACs 0, 1, 14 and q0_ttl with the completion monitors. Per run and
    core: every output's last activity ends before the marker write, by more than the 14q build's extra alignment
    stages (2 here), the marker write precedes the DONE rise, and nothing moves outside the runs.

    FLOOR: the twin's 2-core image load (~20 k batches) and two uplink runs (~16 k each, ~25 + 3 shots at 500);
    ReadoutCalibration (1 setup, 1 uplink rerun), Leakage (2 setups, 2 uplink reruns), the heralded k_batched (1
    setup, 1 rerun) and the dual capture (1 setup, 1 uplink rerun) at ~20 k batches per setup and ~16 k per uplink
    rerun, all under the per-batch watch and monitors."""
    drv, m = cosim_antq
    report, extra = _completion(drv, m, monkeypatch, "sim-dio-antq")
    assert extra == 2
    print(f"\n[BT C-D] sim-dio-antq: {len(report)} runs; per label (min marker - last activity, min DONE - marker) "
          f"{_summary(report)}; > {extra} extra 14q stages; nothing moved outside the runs")


@pytest.mark.batch_cap(440_000)
def test_cd_completion_order_on_sim_2q1c_antq(cosim_2q1c, monkeypatch):
    """C-D on sim-2q1c-antq (gate DACs 0, 1, 3; the readout DAC 2 summed over all three cores): as on sim-dio-antq,
    with the margin against the 14q build's extra alignment stages (1 here), each core's order checked against the
    shared readout DAC's last activity from any core.

    FLOOR: as on sim-dio-antq with three cores: ~30 k batches per image load, the uplink reruns and their shots,
    under the per-batch watch and monitors."""
    drv, m = cosim_2q1c
    report, extra = _completion(drv, m, monkeypatch, "sim-2q1c-antq")
    assert extra == 1
    print(f"\n[BT C-D] sim-2q1c-antq: {len(report)} runs; per label (min marker - last activity, min DONE - marker) "
          f"{_summary(report)}; > {extra} extra 14q stages; nothing moved outside the runs")
