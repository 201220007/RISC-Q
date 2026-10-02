"""L0 tests of the shared run layer S0 (qubic3; plan P4 v2 §4, §8 L0): host-pure, on `FakeSoc` (with
its scripted uplink on an antq_uplink build).

Covered: the loaded-set guard and the setup identity; the run id (generation, epoch) and the epoch
space; the stop mailbox (the P4 seam: accepted, duplicate, LATE, closure, a request from an earlier
generation, a policy, a failing hook); every failure kind (timeout, policy, UNFINISHED, prepare,
flush, drain, DMA, LATE_ACTIVITY at G1 and G2), each FAILED with no data and followed by an exact
run; prepare's branches (nothing opened, err_base_busy, err_start_dropped with its re-arm, a
run_base mismatch, a zero timeout after BASE_RESET, a failing cleanup); G3 (EXPECTED_DISCARD against
STRAY, saturated counts); which transitions take the hardware flush; the flush sequence; POISONED and
recover()."""

import time
from pathlib import Path

import numpy as np
import pytest

from riscq import ddr_regs as R
from riscq import run as rq
from riscq import session as S
from riscq.build import Image, Program
from riscq.ddr import DdrUplinkError, LateActivity
from tests.fake_soc import FakeSoc

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
HW = (CONFIGS / "sim-2q.json").read_text()
ANTQ = (CONFIGS / "sim-2q-antq.json").read_text()
MAGIC_OFF, OUT_OFF, N_OUT = 0x40, 0x80, 4


def prog(salt: int = 0, marker: bool = False, n_out: int = N_OUT, sym: str = "out") -> Program:
    """A hand-made program: the image carries __rq_magic (so check_magic passes) and an `out` array."""
    data = bytearray(0x80)
    data[MAGIC_OFF:MAGIC_OFF + 4] = rq.MAGIC.to_bytes(4, "little")
    data[0x10] = salt & 0xFF
    p = Program(Image(data=bytes(data), symbols={"__rq_magic": (0x8000_0000 + MAGIC_OFF, 4),
                                                 sym: (0x8000_0000 + OUT_OFF, 4 * n_out)},
                      entry=0x8000_0000), arrays={sym: n_out})
    if marker:
        p.marker = (sym, n_out - 1)
    return p


def kernel(results=None, markers=True, done=True):
    """An `on_release` model: post each core's results through the uplink, store every armed marker
    (the run layer wrote 0xFFFF_FFFF there), then raise DONE for every loaded core."""
    def run(fake):
        for core, words in (results or {}).items():
            for re, im in words:
                fake.up.post(core, re, im)
        if markers:
            for a, v in list(fake.mem.items()):
                if v == rq.MARKER_ARMED:
                    fake.mem[a] = rq.MARKER_DONE
        if done:
            fake.done = sum(1 << c for c in fake.loaded)
    return run


@pytest.fixture(autouse=True)
def _short_hardware_polls(monkeypatch):
    """FakeSoc takes the hardware poll, so a hung kernel times out after POLL_MIN_S of wall clock."""
    monkeypatch.setattr(rq, "POLL_MIN_S", 0.01)


def soc(text=ANTQ):
    f = FakeSoc(text)
    return f, f.m


def up(expected, **kw):
    return rq.UplinkRun(expected=expected, settle_s=0.0, **kw)


WORDS = {0: [(0x1230, -0x4560), (-16, 2 ** 31 - 16)], 1: [(160, 320)]}


def _iq(words):
    """The run layer's `__uplink`: each 28-bit field plus the half step."""
    return [x for re, im in words for x in (((re >> 4) << 4) + 8, ((im >> 4) << 4) + 8)]


# ── the loaded-set guard (§4.5) ──

def test_rerun_before_setup_is_refused_before_any_op():
    f, m = soc(HW)
    with pytest.raises(S.LoadedSetError, match="before a complete setup"):
        rq.rerun(f, m, {0: prog()})
    assert f.mem == {} and f.releases == 0


def test_subset_superset_and_changed_program_are_refused():
    f, m = soc(HW)
    a, b = prog(1), prog(2)
    rq.setup(f, m, {0: a, 1: b})
    n = f.releases
    with pytest.raises(S.LoadedSetError, match="would run the omitted"):
        rq.rerun(f, m, {0: a})
    with pytest.raises(S.LoadedSetError, match="differs from the one setup"):
        rq.rerun(f, m, {0: a, 1: prog(3)})
    assert f.releases == n
    rq.rerun(f, m, {0: a, 1: b})
    assert f.releases == n + 1


def test_equal_image_bytes_with_other_symbols_are_another_identity():
    a, b = prog(sym="out"), prog(sym="res")
    assert a.image.data == b.image.data
    assert rq.program_identity(a) != rq.program_identity(b)
    c = prog()
    c.params = {"n": None}
    assert rq.program_identity(c) != rq.program_identity(a)
    assert rq.program_identity(prog()) == rq.program_identity(prog())
    assert rq.program_identity(rq._prog_from_wire(rq._prog_to_wire(prog(marker=True)))) == \
        rq.program_identity(prog(marker=True))


def test_a_failed_setup_leaves_no_loaded_set_and_a_reload_starts_clean():
    f, m = soc(HW)
    rq.setup(f, m, {0: prog()})
    g = S.session(f).generation
    bad = prog()
    bad.envelopes = {0: [(0, np.zeros((1, 3), dtype="<u4"))]}      # wrong line width: the load raises
    with pytest.raises(ValueError):
        rq.setup(f, m, {0: bad})
    assert S.session(f).loaded is None and S.session(f).generation == g
    with pytest.raises(S.LoadedSetError):
        rq.rerun(f, m, {0: prog()})
    f2 = FakeSoc(HW)                                               # a reload: a new driver, a new session
    with pytest.raises(S.LoadedSetError):
        rq.rerun(f2, m, {0: prog()})


def test_client_identities_are_checked_against_the_loaded_set():
    f, m = soc(HW)
    rq.setup(f, m, {0: prog(1)})
    with pytest.raises(S.LoadedSetError, match="differs"):
        rq.rerun(f, m, {0: prog(1)}, identities={0: rq.program_identity(prog(2))})
    rq.rerun(f, m, {0: prog(1)}, identities={0: rq.program_identity(prog(1))})


# ── run ids (§5.1) ──

def test_epochs_are_nonzero_unique_and_survive_setup():
    """Never 0 (no request) and, since P4, never 0xFFFF_FFFF, the sentinel a never-booted core keeps
    in rq_status (plan P4 v2 §4.4): fin == epoch must not hold for a core that never ran."""
    s = S.RunSession(seed=0xFFFF_FFFE)
    assert [s.next_epoch() for _ in range(3)] == [0xFFFF_FFFE, 1, 2]
    for bad in (0, 0xFFFF_FFFF):
        with pytest.raises(ValueError, match="seed"):
            S.RunSession(seed=bad)
    f, m = soc(HW)
    rq.setup(f, m, {0: prog()})
    seen = set()
    for _ in range(3):
        rq.rerun(f, m, {0: prog()})
        seen.add(S.session(f).runs[-1].run_id)
    rq.setup(f, m, {0: prog()})
    rq.rerun(f, m, {0: prog()})
    g, e = S.session(f).runs[-1].run_id
    assert g == 2 and all(r[1] != e for r in seen) and len(seen) == 3 and e != 0


def test_epoch_space_exhaustion():
    s = S.RunSession(seed=5)
    s._next = 4                       # as if every other epoch had been issued
    assert s.next_epoch() == 4
    with pytest.raises(S.EpochExhausted, match="open a session"):
        s.next_epoch()


# ── the stop mailbox (§4.3), the seam P4 plugs into ──

class Hook:
    def __init__(self, policy=None, fail=False):
        self.issued, self.fail = [], fail
        self.spec = S.StopSpec(issue=self.issue, policy=policy)

    def issue(self, ctx, ticket):
        if self.fail:
            raise RuntimeError("stop words could not be written")
        self.issued.append((ctx.run_id, ticket.request.kind, ticket.request.S))


def test_mailbox_accepts_one_request_then_refuses_and_closes():
    f, m = soc(HW)
    p = prog()
    rq.setup(f, m, {0: p})
    hook, tickets = Hook(), []

    def release(fake):
        rid = rq.current_run(fake)
        tickets.extend([rq.request_stop(fake, rid, S.AT, 5), rq.request_stop(fake, rid, S.NEXT),
                        rq.request_stop(fake, (rid[0], rid[1] ^ 1), S.NEXT),
                        rq.request_stop(fake, (rid[0] - 1, rid[1]), S.NEXT)])
        fake.on_release = None
    f.on_release = release
    hook.spec.policy = lambda ctx: setattr(f, "done", 1)       # the kernel ends during the 1st poll
    rq.rerun(f, m, {0: p}, stop=hook.spec)
    assert [t.outcome for t in tickets] == [S.ACCEPTED, S.REFUSED_DUPLICATE, S.LATE, S.LATE]
    assert hook.issued == [(S.session(f).runs[-1].run_id, S.AT, 5)]
    rid = S.session(f).runs[-1].run_id
    assert rq.request_stop(f, rid, S.NEXT).outcome == S.LATE                 # after DONE: closed
    assert rq.current_run(f) is None


def test_a_run_seen_done_closes_its_mailbox_before_the_policy_and_the_requests():
    """A request still queued when the poll loop first reads DONE is LATE: the loop closes the
    mailbox and returns before it runs the policy or decides a request, so neither the policy nor
    the hook runs and a completed run never holds an ACCEPTED ticket."""
    f, m = soc(HW)
    rq.setup(f, m, {0: prog()})
    tickets, calls = [], []

    def release(fake):
        tickets.append(rq.request_stop(fake, rq.current_run(fake), S.NEXT))
        fake.done = 1                                  # DONE before the first poll
    f.on_release = release
    hook = Hook(policy=lambda ctx: calls.append(ctx.done))
    rq.rerun(f, m, {0: prog()}, stop=hook.spec)
    assert [t.outcome for t in tickets] == [S.LATE] and hook.issued == [] and calls == []
    run = S.session(f).runs[-1]
    assert run.outcome == S.CERTIFIED and run.mailbox.accepted is None


def test_a_request_to_a_run_without_a_stop_hook_is_not_stoppable():
    f, m = soc(HW)
    rq.setup(f, m, {0: prog()})
    got = []

    def release(fake):
        got.append(rq.request_stop(fake, rq.current_run(fake), S.NEXT).outcome)
        fake.done = 1
    f.on_release = release
    rq.rerun(f, m, {0: prog()})
    assert got == [S.REFUSED_NOT_STOPPABLE]
    for bad in (-1, 2 ** 31):                         # S is the kernel's int32; None (P4) = the earliest
        with pytest.raises(ValueError):
            S.session(f).post_stop((1, 1), S.AT, bad)
    with pytest.raises(ValueError):
        S.session(f).post_stop((1, 1), "HALT")


def test_policy_request_and_policy_failure():
    f, m = soc(HW)
    rq.setup(f, m, {0: prog()})
    hook = Hook(policy=lambda ctx: (S.AT, 7) if not ctx.done else None)
    f.on_release = lambda fake: None                                           # DONE only on the 2nd poll
    calls = []

    def policy(ctx):
        calls.append(ctx.done)
        if len(calls) == 2:
            f.done = 1
        return (S.AT, 7) if len(calls) == 1 else None
    hook.spec.policy = policy
    rq.rerun(f, m, {0: prog()}, stop=hook.spec)
    assert hook.issued and hook.issued[0][1:] == (S.AT, 7)

    def bad(ctx):
        raise ValueError("policy bug")
    f.on_release = kernel(done=False)
    with pytest.raises(ValueError, match="policy bug"):
        rq.rerun(f, m, {0: prog()}, stop=S.StopSpec(issue=lambda c, t: None, policy=bad))
    rec = S.session(f).last_failure
    assert rec.kind == "POLICY" and f.reset_held
    assert S.session(f).pending_flush.reason == S.FLUSH_FAILED


def test_a_failing_stop_hook_fails_the_run():
    f, m = soc(HW)
    rq.setup(f, m, {0: prog()})
    hook = Hook(fail=True)

    def release(fake):
        S.session(fake).post_stop(rq.current_run(fake), S.NEXT)
    f.on_release = release
    with pytest.raises(RuntimeError, match="stop words"):
        rq.rerun(f, m, {0: prog()}, stop=hook.spec)
    assert S.session(f).runs[-1].outcome == S.FAILED


# ── the hardware poll's wall-clock bound (after-stage r1 #4; the unit mismatch is upstream's) ──

def test_hardware_polls_turn_cycle_timeouts_into_a_bounded_wall_clock_deadline(monkeypatch):
    from riscq.cal.base import batch_timeout
    from riscq.map import SocMap, SocParams
    monkeypatch.setattr(rq, "POLL_MIN_S", 1.0)
    board = SocMap(SocParams.from_json((CONFIGS / "zcu216-14q-antq.json").read_text()))
    assert board.params.dsp_freq_hz == 500e6
    # the cal layer's smallest timeout, 2·10^7 cycles, was 2·10^7 sleeps of 1 ms (5.5 h) upstream
    assert batch_timeout(0) == 20_000_000 and rq.poll_seconds(board, batch_timeout(0)) == 1.0
    n = 10**6 * 2000                                   # 10^6 shots at a 2000-batch period: 4 s of hardware
    assert rq.poll_seconds(board, batch_timeout(n)) == pytest.approx(16.04)
    assert rq.poll_seconds(board, 10**15) == rq.POLL_MAX_S == 600.0
    # a hung kernel behind a hardware driver times out at the capped deadline, with and without a stop hook
    monkeypatch.setattr(rq, "POLL_MAX_S", 0.2)
    f, m = soc(HW)
    rq.setup(f, m, {0: prog()})
    f.on_release = kernel(done=False)
    for stop in (None, Hook().spec):
        t0 = time.monotonic()
        with pytest.raises(TimeoutError, match="a 0.2 s wall-clock bound"):
            rq.rerun(f, m, {0: prog()}, timeout=batch_timeout(10**9), stop=stop)
        assert 0.2 <= time.monotonic() - t0 < 5.0


# ── the failure lifecycle without the uplink (§4.2): FAILED, no data, the flush, an exact next run ──

def test_timeout_fails_resets_and_the_next_rerun_flushes_first():
    f, m = soc(HW)
    rq.set_time_offset(f, m, 0x1_2345_6789)
    rq.setup(f, m, {0: prog()})
    f.on_release = kernel(done=False)                                          # a hung kernel
    with pytest.raises(TimeoutError):
        rq.rerun(f, m, {0: prog()}, timeout=3)
    s = S.session(f)
    rec = s.last_failure
    assert rec.kind == "TIMEOUT" and rec.done == 0 and f.reset_held
    assert s.pending_flush.reason == S.FLUSH_FAILED and f.pl_resets == 0
    f.on_release = kernel()
    out = rq.rerun(f, m, {0: prog()})
    assert f.pl_resets == 1 and s.pending_flush is None and out[0]["out"].shape == (N_OUT,)
    hc = m.host_ctrl
    assert f.mem[hc + m.HOST_TIME_OFF_LO] == 0x2345_6789 and f.mem[hc + m.HOST_TIME_OFF_HI] == 1
    assert f.mem[hc + m.HOST_HOSTWIN_LO] == f.host_base                       # host-window base restored
    assert s.runs[-1].flushed == S.FLUSH_FAILED
    rq.rerun(f, m, {0: prog()})
    assert f.pl_resets == 1                                                   # a good run takes none


def test_unfinished_marker_fails_the_run():
    f, m = soc(HW)
    p = prog(marker=True)
    rq.setup(f, m, {0: p})
    f.on_release = kernel(markers=False)
    with pytest.raises(S.Unfinished, match="never booted"):
        rq.rerun(f, m, {0: p})
    assert S.session(f).last_failure.kind == "UNFINISHED"
    f.on_release = kernel()
    assert rq.rerun(f, m, {0: p})[0]["out"][-1] == rq.MARKER_DONE


def test_a_driver_without_pl_reset_poisons_and_recover_needs_a_reload():
    f, m = soc(HW)
    f.pl_reset = None
    rq.setup(f, m, {0: prog()})
    f.on_release = kernel(done=False)
    with pytest.raises(TimeoutError):
        rq.rerun(f, m, {0: prog()}, timeout=2)
    f.on_release = kernel()
    with pytest.raises(S.SessionPoisoned, match="cannot pulse pl_resetn0"):
        rq.rerun(f, m, {0: prog()})
    with pytest.raises(S.SessionPoisoned, match="POISONED"):
        rq.setup(f, m, {0: prog()})
    with pytest.raises(S.SessionPoisoned, match="reload the PL"):
        rq.recover(f, m)
    del f.pl_reset                                                            # the PL reload
    assert rq.recover(f, m) == ["hardware flush"]
    rq.setup(f, m, {0: prog()})
    rq.rerun(f, m, {0: prog()})


# ── the uplink seam (P6 v2 §4.3) and its failure kinds ──

def _antq(words=WORDS, marker=True):
    f, m = soc(ANTQ)
    progs = {0: prog(1, marker), 1: prog(2, marker)}
    rq.setup(f, m, progs)
    f.on_release = kernel(results=words)
    exp = {c: len(words.get(c, [])) for c in progs}
    return f, m, progs, exp


def test_a_clean_uplink_run_certifies_and_returns_the_iq():
    f, m, progs, exp = _antq()
    out = rq.rerun(f, m, progs, uplink=up(exp, base=0x1000))
    assert list(out[0]["__uplink"]) == _iq(WORDS[0]) and list(out[1]["__uplink"]) == _iq(WORDS[1])
    run = S.session(f).runs[-1]
    assert run.states == [S.IDLE, S.PREPARING, S.PREPARED, S.RELEASED, S.DONE_SEEN, S.RESET, S.FLUSHED,
                          S.CERTIFIED, S.IDLE]
    assert run.proven and S.session(f).pending_flush is None and f.pl_resets == 0


@pytest.mark.parametrize("fault,kind,match", [
    ("flush_refused", "FLUSH", "flush refused"),
    ("bresp", "DRAIN", "bresp"),
    ("dma_error", "DRAIN", "S2MM"),
])
def test_flush_drain_and_dma_errors_fail_then_the_next_run_is_exact(fault, kind, match):
    f, m, progs, exp = _antq()
    f.up.script.add(fault)
    with pytest.raises((DdrUplinkError, RuntimeError), match=match):
        rq.rerun(f, m, progs, uplink=up(exp))
    s = S.session(f)
    assert s.last_failure.kind == kind and f.reset_held
    assert s.pending_flush.reason == S.FLUSH_FAILED
    f.up.script.discard(fault)
    out = rq.rerun(f, m, progs, uplink=up(exp))
    assert f.pl_resets == 1 and list(out[0]["__uplink"]) == _iq(WORDS[0])


def test_g1_a_result_between_commit_and_drain_is_late_activity():
    f, m, progs, exp = _antq()
    f.up.on_flush = lambda u: u.post(0, 16, 16)
    with pytest.raises(LateActivity, match="run invalid"):
        rq.rerun(f, m, progs, uplink=up(exp))
    assert S.session(f).last_failure.kind == "LATE_ACTIVITY"
    f.up.on_flush = None
    rq.rerun(f, m, progs, uplink=up(exp))
    assert f.pl_resets == 1


def test_g2_a_result_after_the_drain_is_late_activity(monkeypatch):
    f, m, progs, exp = _antq()
    real = rq._settle
    monkeypatch.setattr(rq, "_settle", lambda drv, u: (f.up.post(1, 0, 0), real(drv, u)))
    with pytest.raises(LateActivity, match="after the drain"):
        rq.rerun(f, m, progs, uplink=up(exp))
    assert S.session(f).last_failure.kind == "LATE_ACTIVITY"
    monkeypatch.setattr(rq, "_settle", real)
    rq.rerun(f, m, progs, uplink=up(exp))


def test_timeout_and_unfinished_close_the_admission_with_a_flush():
    f, m, progs, exp = _antq()
    f.on_release = kernel(results=WORDS, done=False)
    with pytest.raises(TimeoutError):
        rq.rerun(f, m, progs, uplink=up(exp), timeout=2)
    rec = S.session(f).last_failure
    assert rec.kind == "TIMEOUT" and any("flushed (not drained)" in c for c in rec.cleanup)
    assert not f.up.run_active and rec.status >> R.S_RUN_ACTIVE & 1
    f.on_release = kernel(results=WORDS, markers=False)
    with pytest.raises(S.Unfinished):
        rq.rerun(f, m, progs, uplink=up(exp))
    assert not f.up.run_active
    f.on_release = kernel(results=WORDS)
    rq.rerun(f, m, progs, uplink=up(exp))


# ── prepare's branches (§4.2: prepare is inside the cleanup) ──

@pytest.mark.parametrize("fault,match,cleanup", [
    ("wr_base_mismatch", "wr_base rejected", "nothing open"),
    ("base_busy", "base_reset refused", "the run was not active"),
    ("start_dropped", "start handshake dropped", "re-armed after err_start_dropped, flushed (not drained)"),
    ("run_base_mismatch", "run_base", "flushed (not drained)"),
])
def test_prepare_branches_leave_the_admission_closed(fault, match, cleanup):
    f, m, progs, exp = _antq()
    f.up.script.add(fault)
    with pytest.raises(DdrUplinkError, match=match):
        rq.rerun(f, m, progs, uplink=up(exp))
    rec = S.session(f).last_failure
    assert rec.kind == "PREPARE" and rec.cleanup[-1] == f"admission: {cleanup}"
    assert not f.up.run_active and f.reset_held and f.releases == 0
    f.up.script.discard(fault)
    assert list(rq.rerun(f, m, progs, uplink=up(exp))[1]["__uplink"]) == _iq(WORDS[1])


def test_prepare_timeout_zero_raises_after_base_reset_and_the_cleanup_flushes():
    f, m, progs, exp = _antq()
    f.up.start_delay = 3                            # the start completes after prepare gave up
    with pytest.raises(DdrUplinkError, match="did not start within 0"):
        rq.rerun(f, m, progs, uplink=up(exp, prepare_timeout=0))
    rec = S.session(f).last_failure
    assert rec.kind == "PREPARE" and rec.cleanup[-1] == "admission: flushed (not drained)"
    assert f.up.flushes == 1 and not f.up.run_active
    out = rq.rerun(f, m, progs, uplink=up(exp))
    assert f.pl_resets == 1 and list(out[0]["__uplink"]) == _iq(WORDS[0])


def test_a_base_reset_that_lands_and_then_raises_counts_as_issued():
    """After-stage r1 #3: the issuance flag is set before the BASE_RESET write, so a write that
    completes and then raises (an interrupted MMIO store) still has its admission closed."""
    f, m, progs, exp = _antq()
    real, ctrl = f.write32, f.up.base + R.BASE_RESET

    def write32(addr, value):
        real(addr, value)
        if addr == ctrl:
            raise RuntimeError("the MMIO store raised after it landed")
    f.write32 = write32
    with pytest.raises(RuntimeError, match="after it landed"):
        rq.rerun(f, m, progs, uplink=up(exp))
    rec = S.session(f).last_failure
    assert rec.kind == "PREPARE" and rec.cleanup[-1] == "admission: flushed (not drained)"
    assert not f.up.run_active and f.up.base_resets == 1
    f.write32 = real
    assert list(rq.rerun(f, m, progs, uplink=up(exp))[0]["__uplink"]) == _iq(WORDS[0])


def test_a_failing_cleanup_poisons_until_recover():
    f, m, progs, exp = _antq()
    f.up.script |= {"start_dropped", "start_dropped_twice"}
    with pytest.raises(DdrUplinkError, match="start handshake dropped"):
        rq.rerun(f, m, progs, uplink=up(exp))
    s = S.session(f)
    assert s.poisoned and "admission could not be closed" in s.poisoned
    with pytest.raises(S.SessionPoisoned):
        rq.rerun(f, m, progs, uplink=up(exp))
    f.up.script.clear()
    rq.recover(f, m)
    assert s.poisoned is None
    rq.rerun(f, m, progs, uplink=up(exp))


# ── §4.6: G3 at quiesce, and which transitions take the hardware flush ──

def test_g3_expected_discard_after_uplink_free_runs_takes_no_flush():
    f, m, progs, exp = _antq()
    rq.rerun(f, m, progs)                          # uplink-free: admission closed, results rejected
    assert f.up.rejected == [2, 1, 0][:2] + f.up.rejected[2:]
    rq.rerun(f, m, progs, uplink=up(exp))
    s = S.session(f)
    assert f.pl_resets == 0 and s.runs[-1].discards == {0: 2, 1: 1}
    assert any(n.startswith("EXPECTED_DISCARD") for n in s.notes)


def test_g3_stray_takes_the_flush_and_saturation_is_logged():
    f, m, progs, exp = _antq()
    rq.rerun(f, m, progs, uplink=up(exp))
    f.up.rejected[1] = 0xFFFF                      # a stray, saturated, with no uplink-free rerun since
    rq.rerun(f, m, progs, uplink=up(exp))
    s = S.session(f)
    assert f.pl_resets == 1 and s.runs[-1].flushed == S.FLUSH_STRAY
    assert any("STRAY {1: '≥ 65535'}" in n for n in s.notes)


def _log_flushes_and_releases(f):
    """Record the order of pl_resetn0 pulses and core-reset releases on `f`."""
    events, pulse, kernel_ = [], f.pl_reset, f.on_release
    f.pl_reset = lambda: (events.append("flush"), pulse())
    f.on_release = lambda fake: (events.append("release"), kernel_(fake))
    return events


def test_after_an_unproven_run_every_release_is_preceded_by_the_flush():
    """§4.6: after a run that was not queue-proven, the next release (an uplink-free RAM or COUNTS
    rerun too) is preceded by the hardware flush; a setup runs it before the load."""
    f, m, progs, exp = _antq(marker=False)
    events = _log_flushes_and_releases(f)
    rq.rerun(f, m, progs, uplink=up(exp))
    s = S.session(f)
    assert s.pending_flush.reason == S.FLUSH_UNPROVEN and events == ["release"]
    rq.rerun(f, m, progs)                          # uplink-free: flushed, then released
    assert events == ["release", "flush", "release"] and s.runs[-1].flushed == S.FLUSH_UNPROVEN
    assert s.pending_flush.reason == S.FLUSH_UNPROVEN                    # itself unproven
    rq.setup(f, m, progs)                          # the setup flushes before it loads
    assert events[-1] == "flush" and s.pending_flush is None and f.pl_resets == 2
    rq.rerun(f, m, progs, uplink=up(exp))          # nothing pending: no flush
    rq.rerun(f, m, progs, uplink=up(exp))          # the next one flushes in its quiesce
    assert events == ["release", "flush", "release", "flush", "release", "flush", "release"]
    assert s.runs[-1].flushed == S.FLUSH_UNPROVEN and s.runs[-2].flushed is None


def test_an_uplink_free_rerun_is_refused_while_a_flush_is_pending_and_the_uplink_has_a_run_open():
    """A rerun without uplink= whose caller has prepared the uplink itself cannot take the flush
    (it would reset the caller's run), so it is refused before any write until recover()."""
    from riscq.ddr import readout_for
    f, m, progs, exp = _antq(marker=False)
    rq.rerun(f, m, progs, uplink=up(exp))          # unproven
    rd = readout_for(f, m)
    rd.prepare(0x2000, expected=exp)               # the caller's own protocol
    events = _log_flushes_and_releases(f)
    writes = len(f.mem)
    with pytest.raises(S.RecoveryRequired, match="the uplink has a run open"):
        rq.rerun(f, m, progs)
    assert events == [] and len(f.mem) == writes and f.up.run_active
    rq.recover(f, m)
    assert events == ["flush"] and S.session(f).pending_flush is None
    rd.prepare(0x2000, expected=exp)
    out = rq.rerun(f, m, progs)
    assert events == ["flush", "release"] and out[0]["out"].shape == (N_OUT,)
    rd.flush()
    re, im = rd.drain(0x2000, exp)[0]                 # the raw fields
    assert [x + 8 for p in zip(re, im) for x in p] == _iq(WORDS[0])


def test_proven_runs_take_no_flush():
    f, m, progs, exp = _antq()
    for _ in range(3):
        rq.rerun(f, m, progs, uplink=up(exp))
        rq.rerun(f, m, progs)
    assert f.pl_resets == 0 and S.session(f).pending_flush is None


def test_a_failed_run_is_flushed_before_an_uplink_free_rerun_and_at_setup():
    f, m, progs, exp = _antq()
    f.on_release = kernel(results=WORDS, done=False)
    with pytest.raises(TimeoutError):
        rq.rerun(f, m, progs, uplink=up(exp), timeout=2)
    f.on_release = kernel(results=WORDS)
    events = _log_flushes_and_releases(f)
    rq.rerun(f, m, progs)                          # the cleanup closed the admission: flush, release
    s = S.session(f)
    assert events == ["flush", "release"] and s.runs[-1].flushed == S.FLUSH_FAILED
    assert s.pending_flush is None
    f.on_release = kernel(results=WORDS, done=False)
    with pytest.raises(TimeoutError):
        rq.rerun(f, m, progs, timeout=2)
    rq.setup(f, m, progs)                          # setup runs the pending flush
    assert events[-1] == "flush" and s.pending_flush is None


# ── quiesce and the flush's own failures (POISONED) ──

def test_quiesce_timeout_poisons_and_recover_restores(monkeypatch):
    monkeypatch.setattr(rq, "QUIESCE_TIMEOUT", 0.2)
    monkeypatch.setattr(rq, "FLUSH_IDLE_TIMEOUT", 0.2)
    f, m, progs, exp = _antq()
    f.up.script.add("never_idle")
    with pytest.raises(S.SessionPoisoned, match="run_idle"):
        rq.rerun(f, m, progs, uplink=up(exp))
    with pytest.raises(S.SessionPoisoned, match="POISONED"):
        rq.rerun(f, m, progs, uplink=up(exp))
    with pytest.raises(S.SessionPoisoned):
        rq.recover(f, m)                           # still not idle after the flush: PL reload
    f.up.script.clear()
    rq.recover(f, m)
    rq.rerun(f, m, progs, uplink=up(exp))


def test_the_flush_sequence_and_a_not_quiet_uplink(monkeypatch):
    f, m, progs, exp = _antq()
    seen = []
    S.session(f).rf_bringup = lambda: seen.append(("bringup", f.reset_held, f.pl_resets))
    rq.hardware_flush(f, m, "TEST")
    assert seen == [("bringup", True, 1)]
    real = f.pl_reset

    def leaky():
        real()
        f.up.rejected[0] = 3                       # something keeps arriving after the pulse
    f.pl_reset = leaky
    with pytest.raises(S.RunLayerError, match="not quiet"):
        rq.hardware_flush(f, m, "TEST")


def test_the_dma_channel_is_reset_when_left_busy():
    f, m, progs, exp = _antq()
    f.up.dma = ("armed", 32)                       # an abandoned transfer
    rq.rerun(f, m, progs, uplink=up(exp))
    assert f.up.dma_resets == 1
