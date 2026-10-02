"""L0 tests of P4, early STOP and exact actual_shots (qubic3; plan P4 v2 §4.4, §5, §8 L0), host-pure.
The compile-time convention and its allow-list are in `test_p4_convention.py`.

Covered: the run protocol on a model of a stoppable kernel over `FakeSoc` (the words written before
every release and their order, the
counts and their validity: NOT_BOOTED, UNFINISHED, TIMEOUT; reserved params refused); the issue
step (NEXT on every core, AT(S) with its read-backs and progress reads, TOO_LATE before any write
with the request left open, AT refused on a NEXT-only kernel, a failed read-back); the arithmetic
of S and VERIFIED; the outcome table, with INTERNAL_ERROR, CONSISTENT_LATE, INCONSISTENT and
`accept_inconsistent`; the uplink drain on counted reads and the nominal bound; stale requests across epochs
and generations; the next run after each kind of stop; the policies (`AtProgress`, `Landed` on a
scripted CUR_ADDR); the latency arithmetic; the spec's wire form and the board server's remote
path, with `post_stop` reaching a run that holds the server's lock."""

from __future__ import annotations

import itertools
import threading
from pathlib import Path

import pytest

from riscq import ddr_regs as R
from riscq import run as rq
from riscq import session as S
from riscq import stop as st
from riscq.build import Image, Program
from tests.fake_soc import FakeSoc, TraceDriver

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
HW = (CONFIGS / "sim-2q.json").read_text()
ANTQ = (CONFIGS / "sim-2q-antq.json").read_text()
B = 0x8000_0000
MAGIC_OFF, EPOCH, STOP_EPOCH, STOP_AT, NSYM, STATUS = 0x40, 0x80, 0x84, 0x88, 0x8C, 0x90


@pytest.fixture(autouse=True)
def _short_hardware_polls(monkeypatch):
    monkeypatch.setattr(rq, "POLL_MIN_S", 0.05)


# ── a model of a stoppable kernel over FakeSoc ──

def sprog(salt: int = 0, n: int | None = None, at: bool = True, lead: int = 1, reads: int | None = 1) -> Program:
    """A hand-made stoppable program: __rq_magic, the reserved names, a runtime `n` and rq_status."""
    data = bytearray(0xA0)
    data[MAGIC_OFF:MAGIC_OFF + 4] = rq.MAGIC.to_bytes(4, "little")
    data[0x10] = salt & 0xFF
    syms = {"__rq_magic": (B + MAGIC_OFF, 4), "rq_epoch": (B + EPOCH, 4), "rq_stop_epoch": (B + STOP_EPOCH, 4),
            "rq_stop_at": (B + STOP_AT, 4), "n": (B + NSYM, 4), "rq_status": (B + STATUS, 12)}
    p = Program(Image(data=bytes(data), symbols=syms, entry=B),
                params={"rq_epoch": None, "rq_stop_epoch": None, "rq_stop_at": None, "n": None},
                arrays={"rq_status": 3})
    p.stop = {"shots": "n", "n": n, "at": at, "lead": lead, "reads": reads}
    p.marker = ("rq_status", 2)
    return p


def _s32(v):
    v &= 0xFFFF_FFFF
    return v - (1 << 32) if v >> 31 else v


class StopSoc(FakeSoc):
    """FakeSoc with a model of the stop convention. At the release each programmed core boots
    (zeroes rq_status; `no_boot` cores do not). Each read of the DONE word is one tick: every running
    core whose `every[c]` divides its tick count checks (`rq_stop_epoch == rq_epoch and s >=
    rq_stop_at`, or s == n) and either posts one shot (rq_status[0] = s, [1] += reads_per_shot[c],
    one uplink result per read on an antq build) or ends: fin = rq_epoch (unless `skip_fin`) and its
    DONE bit. `hang` cores never end. `tick_on_status` makes every read of a core's rq_status[0]
    (the issue step's progress reads) a tick too, so the run moves during the broadcast, and
    `tick_on_publish` = k makes every read-back of a core's rq_stop_epoch k ticks, so a core
    published late may already have passed S."""

    def __init__(self, text=HW):
        super().__init__(text)
        self.model = None
        self.on_release = self._release

    def arm(self, progs, every=None, hang=(), skip_fin=(), no_boot=(), reads=None, tick_on_status=False,
            hold=False, tick_on_publish=0):
        self.model = dict(progs=progs, every=every or {}, hang=set(hang), skip_fin=set(skip_fin),
                          no_boot=set(no_boot), reads=reads or {}, tick_on_status=tick_on_status, hold=hold,
                          tick_on_publish=tick_on_publish)
        self.state = {}

    def a(self, core, name, i=0):
        return self.m.to_host_addr(core, self.model["progs"][core].var_addr(name) + 4 * i)

    def _release(self, fake):
        md = self.model
        self.state = {}
        for c, p in md["progs"].items():
            if c not in md["no_boot"]:
                for i in range(3):
                    self.mem[self.a(c, "rq_status", i)] = 0
            n = p.stop["n"] if p.stop["n"] is not None else _s32(self.mem.get(self.a(c, "n"), 0))
            self.state[c] = {"s": 0, "reads": 0, "tick": 0, "run": True, "n": n}

    def tick(self):
        md = self.model
        for c, s_ in self.state.items():
            if not s_["run"]:
                continue
            s_["tick"] += 1
            if s_["tick"] % md["every"].get(c, 1) or c in md["hang"]:
                continue
            if c in md["no_boot"]:                                    # DONE without this run's start.S
                s_["run"] = False
                self.done |= 1 << c
                continue
            e = self.mem.get(self.a(c, "rq_epoch"), 0)
            if md["hold"] and self.mem.get(self.a(c, "rq_stop_epoch"), 0) != e:
                continue                                          # held before the first check
            stop = (self.mem.get(self.a(c, "rq_stop_epoch"), 0) == e
                    and s_["s"] >= _s32(self.mem.get(self.a(c, "rq_stop_at"), 0)))
            if s_["s"] < s_["n"] and not stop:
                s_["s"] += 1
                r = md["reads"].get(c, 1)
                s_["reads"] += r
                self.mem[self.a(c, "rq_status", 0)] = s_["s"]
                self.mem[self.a(c, "rq_status", 1)] = s_["reads"]
                if self.up is not None:
                    for _ in range(r):
                        self.up.post(c, 16 * s_["s"], -16 * s_["s"])
                continue
            s_["run"] = False
            if c not in md["skip_fin"] and c not in md["no_boot"]:
                self.mem[self.a(c, "rq_status", 2)] = e
            self.done |= 1 << c

    def read32(self, addr):
        addr = int(addr)
        if self.model is not None and self.state:
            if addr == self._done:
                self.tick()
            elif self.model["tick_on_status"] and any(addr == self.a(c, "rq_status", 0) for c in self.state):
                self.tick()
            elif self.model["tick_on_publish"] and any(addr == self.a(c, "rq_stop_epoch") for c in self.state):
                for _ in range(self.model["tick_on_publish"]):
                    self.tick()
        return super().read32(addr)


def stopsoc(text=HW, cores=(0, 1), **kw):
    f = StopSoc(text)
    progs = {c: sprog(c, **kw) for c in cores}
    rq.setup(f, f.m, progs)
    return f, f.m, progs


def params(progs, n):
    return {c: {"n": n} for c in progs}


# ── the run protocol (§4.4, §5.1) ──

def test_the_words_before_every_release_and_exact_counts():
    f, m, progs = stopsoc()
    f.arm(progs)
    t = TraceDriver(f)
    t._rq_session = S.session(f)
    out = rq.rerun(t, m, progs, params=params(progs, 7))
    run = S.session(f).runs[-1]
    e = run.run_id[1]
    for c in progs:
        assert list(out[c]["rq_status"]) == [7, 7, _s32(e)]
    a = lambda c, off: m.to_host_addr(c, B + off)            # noqa: E731
    h = lambda *w: _hash(b"".join(int(x).to_bytes(4, "little") for x in w))   # noqa: E731
    for c in progs:
        mine = [op for op in t.trace if op[0].startswith("write")
                and m.to_host_addr(c, B) <= op[1] < m.to_host_addr(c, B) + 0xA0]
        assert mine == [("write32", a(c, NSYM), 7), ("write_block", a(c, EPOCH), 12, h(e, 0, 0)),
                        ("write_block", a(c, STATUS), 12, h(S.SENTINEL, S.SENTINEL, S.SENTINEL))]
    rec = st.last(f)
    assert rec is run.stop and rec.outcome == S.NATURAL and rec.shots == {0: 7, 1: 7} and rec.prefix == 7
    assert all(k.valid for k in rec.counts.values()) and run.proven


def _hash(data):
    from tests.fake_soc import _h
    return _h(data)


def test_scattered_stop_words_are_written_one_by_one():
    f = StopSoc(HW)
    p = sprog()
    syms = dict(p.image.symbols)
    syms["rq_stop_at"] = (B + 0x9C, 4)                          # not next to rq_stop_epoch
    p.image.symbols = syms
    progs = {0: p}
    rq.setup(f, f.m, progs)
    f.arm(progs)
    t = TraceDriver(f)
    t._rq_session = S.session(f)
    rq.rerun(t, f.m, progs, params={0: {"n": 2}})
    e = S.session(f).runs[-1].run_id[1]
    a = lambda off: f.m.to_host_addr(0, B + off)                # noqa: E731
    w = [op[:3] for op in t.trace if op[0] == "write32" and a(0) <= op[1] < a(0xA0)]
    assert w == [("write32", a(NSYM), 2), ("write32", a(EPOCH), e), ("write32", a(STOP_EPOCH), 0),
                 ("write32", a(0x9C), 0)]
    assert st.last(f).outcome == S.NATURAL


def test_n_comes_from_the_binding_the_param_or_the_core_ram():
    f, m, progs = stopsoc(n=5)
    f.arm(progs)
    rq.rerun(f, m, progs)
    assert st.last(f).n == {0: 5, 1: 5}
    f, m, progs = stopsoc()
    f.arm(progs)
    rq.rerun(f, m, progs, params=params(progs, 3))
    rq.rerun(f, m, progs)                                       # n stays 3 in RAM: read back
    assert st.last(f).shots == {0: 3, 1: 3} and st.last(f).n == {0: 3, 1: 3}


def test_reserved_params_and_a_mixed_stoppable_run_are_refused_before_any_op():
    f, m, progs = stopsoc()
    f.arm(progs)
    before = (dict(f.mem), f.releases)
    with pytest.raises(ValueError, match="written by the run layer"):
        rq.rerun(f, m, progs, params={0: {"rq_epoch": 1, "n": 2}})
    from tests.test_s0_run_layer import prog as plain_prog
    g = StopSoc(HW)
    mixed = {0: sprog(0), 1: plain_prog(1)}
    rq.setup(g, g.m, mixed)
    with pytest.raises(ValueError, match="cores \\[1\\] have none"):
        rq.rerun(g, g.m, mixed, stop=st.spec())
    assert (dict(f.mem), f.releases) == before


@pytest.mark.parametrize("fault,err,kind,why", [
    ("no_boot", S.NotBooted, "NOT_BOOTED", st.NOT_BOOTED),
    ("skip_fin", S.Unfinished, "UNFINISHED", st.UNFINISHED),
    ("hang", TimeoutError, "TIMEOUT", st.TIMEOUT),
])
def test_invalid_counts_fail_the_run_and_the_next_run_is_exact(fault, err, kind, why):
    f, m, progs = stopsoc()
    f.arm(progs, **{fault: {1}})
    with pytest.raises(err):
        rq.rerun(f, m, progs, params=params(progs, 4), timeout=3)
    s = S.session(f)
    rec = s.last_failure
    assert rec.kind == kind and s.runs[-1].outcome == S.FAILED and s.pending_flush.reason == S.FLUSH_FAILED
    assert rec.counts[1].valid is False and rec.counts[1].why == why
    if fault != "hang":
        assert rec.counts[0].valid and rec.counts[0].shots == 4
    f.arm(progs)
    out = rq.rerun(f, m, progs, params=params(progs, 4))
    assert f.pl_resets == 1 and st.last(f).outcome == S.NATURAL and out[1]["rq_status"][0] == 4


# ── the issue step (§5.1, §5.3) ──

def _ops_between(trace, lo, hi):
    return [op for op in trace if op[0] in ("read32", "write32") and lo <= op[1] < hi]


def test_next_publishes_on_every_core_in_order_and_each_stops_at_its_next_boundary():
    f, m, progs = stopsoc(cores=(0, 1))
    f.arm(progs)
    t = TraceDriver(f)
    t._rq_session = S.session(f)
    rq.rerun(t, m, progs, params=params(progs, 50), stop=st.spec(st.AtProgress(3, S.NEXT)))
    rec = st.last(f)
    e = rec.run_id[1]
    assert rec.outcome == S.STOPPED_EACH and rec.request["kind"] == S.NEXT
    assert set(rec.shots.values()) == {3}                      # seen at 3: the next check stops
    stop_ops = [op[:3] for op in t.trace if op[0] in ("write32", "read32")
                and any(op[1] == m.to_host_addr(c, B + o) for c in progs for o in (STOP_AT, STOP_EPOCH))
                and not (op[0] == "write32" and op[2] == 0)]
    a = lambda c, off: m.to_host_addr(c, B + off)               # noqa: E731
    assert stop_ops == [("write32", a(0, STOP_EPOCH), e), ("read32", a(0, STOP_EPOCH), e),
                        ("write32", a(1, STOP_EPOCH), e), ("read32", a(1, STOP_EPOCH), e)]
    writes = [op[:3] for op in t.trace if op[0] == "write32" and op[1] in (a(0, STOP_AT), a(1, STOP_AT))]
    assert writes[-2:] == [("write32", a(0, STOP_AT), 0), ("write32", a(1, STOP_AT), 0)]


def test_at_none_takes_the_earliest_verifiable_shot_and_fires_there():
    f, m, progs = stopsoc()
    f.arm(progs)
    rq.rerun(f, m, progs, params=params(progs, 60), stop=st.spec(st.AtProgress(10), margin=2))
    rec = st.last(f)
    rqst = rec.request
    assert rqst["v_pre"] == rqst["v_post"] == 10 and rqst["S"] == 10 + 1 + 2 + 2 and rqst["verified"]
    assert rec.outcome == S.FIRED and rec.shots == {0: 15, 1: 15}
    assert rec.tickets == [(S.AT, None, S.ACCEPTED)]


def test_a_broadcast_that_the_run_outpaces_is_unverified():
    f, m, progs = stopsoc()
    f.arm(progs, tick_on_status=True)                           # each progress read is a tick
    rq.rerun(f, m, progs, params=params(progs, 60), stop=st.spec(st.AtProgress(10), margin=0))
    rqst = st.last(f).request
    assert rqst["v_post"] == rqst["v_pre"] + 1 and rqst["verified"] is False
    assert st.last(f).outcome == S.FIRED                         # unverified, here still on S
    rq.rerun(f, m, progs, params=params(progs, 60), stop=st.spec(st.AtProgress(10), margin=1))
    assert st.last(f).request["verified"] and st.last(f).outcome == S.FIRED


def test_too_late_writes_nothing_and_leaves_the_request_open():
    f, m, progs = stopsoc()
    f.arm(progs)
    t = TraceDriver(f)
    t._rq_session = S.session(f)
    tickets = []

    def policy(ctx):
        v = st.posted(ctx.drv, ctx.m, 0, ctx.progs[0])
        if v == 5 and not tickets:
            tickets.append(rq.request_stop(f, ctx.run_id, S.AT, 0))          # S = 0: below v_pre + 3
            tickets.append(rq.request_stop(f, ctx.run_id, S.AT, 6))          # 6 < 5 + 3: TOO_LATE too
            tickets.append(rq.request_stop(f, ctx.run_id, S.AT, 9))          # valid: accepted
            tickets.append(rq.request_stop(f, ctx.run_id, S.AT, 20))         # duplicate
        return None
    rq.rerun(t, m, progs, params=params(progs, 30), stop=S.StopSpec(issue=st.Issuer(), policy=policy))
    assert [x.outcome for x in tickets] == [S.TOO_LATE, S.TOO_LATE, S.ACCEPTED, S.REFUSED_DUPLICATE]
    assert tickets[0].info["s_min"] == 8 and tickets[0].info["verified"] is False
    rec = st.last(f)
    assert rec.outcome == S.FIRED and rec.shots == {0: 9, 1: 9} and rec.request["S"] == 9
    stop_at_writes = [op[2] for op in t.trace if op[0] == "write32"
                      and op[1] in (m.to_host_addr(0, B + STOP_AT), m.to_host_addr(1, B + STOP_AT))]
    assert stop_at_writes == [9, 9]                  # S = 9 only (the release's clears are block writes)


def test_the_prefix_is_a_joint_shot_count_only_on_a_common_grid():
    """after-stage r1 #11: `common_grid` says every core states C1; a NEXT-only kernel's prefix is
    only the smallest count."""
    for at in (True, False):
        f, m, progs = stopsoc(at=at)
        f.arm(progs, every={1: 2})
        rq.rerun(f, m, progs, params=params(progs, 30), stop=st.spec(st.AtProgress(4, S.NEXT)))
        rec = st.last(f)
        assert rec.outcome == S.STOPPED_EACH and rec.prefix == min(rec.shots.values())
        assert rec.common_grid is at and S.StopRecord.from_wire(rec.to_wire()).common_grid is at


def test_at_on_a_next_only_kernel_is_refused_and_next_still_works():
    f, m, progs = stopsoc(at=False)
    f.arm(progs)
    got = []

    def policy(ctx):
        if st.posted(ctx.drv, ctx.m, 0, ctx.progs[0]) == 4 and not got:
            got.append(rq.request_stop(f, ctx.run_id, S.AT))
            got.append(rq.request_stop(f, ctx.run_id, S.NEXT))
    rq.rerun(f, m, progs, params=params(progs, 30), stop=S.StopSpec(issue=st.Issuer(), policy=policy))
    assert [x.outcome for x in got] == [S.REFUSED_NEXT_ONLY, S.ACCEPTED]
    assert st.last(f).outcome == S.STOPPED_EACH


def test_a_failed_read_back_fails_the_run():
    f, m, progs = stopsoc()
    f.arm(progs)
    real = f.read32
    f.read32 = lambda addr: (0xDEAD if int(addr) == m.to_host_addr(1, B + STOP_EPOCH) else real(addr))
    with pytest.raises(S.StopPublishError, match="core 1: rq_stop_epoch read back 0x0000dead"):
        rq.rerun(f, m, progs, params=params(progs, 30), stop=st.spec(st.AtProgress(2, S.NEXT)))
    s = S.session(f)
    assert s.last_failure.kind == "STOP_PUBLISH" and s.runs[-1].tickets[0].outcome == S.ISSUE_FAILED
    f.read32 = real
    f.arm(progs)
    rq.rerun(f, m, progs, params=params(progs, 6))
    assert st.last(f).outcome == S.NATURAL and f.pl_resets == 1


# ── S and VERIFIED (§5.3) ──

def test_s_and_verified_arithmetic():
    for v_pre, dv, L, m_ in itertools.product(range(0, 6), range(0, 5), (1, 2, 3), range(0, 4)):
        S_ = v_pre + L + 2 + m_
        assert st.at_minimum(v_pre, L) == v_pre + L + 2
        assert st.verified(S_, v_pre + dv, L) == (dv <= m_)
        # the proof's bound: a core's posted count is at most v_post + 1 + L, so the check of S has not
        # begun on any core exactly when S >= v_post + L + 2
        assert st.verified(S_, v_pre + dv, L) == (S_ > (v_pre + dv) + 1 + L)
    assert st.margin_for(50e-6, 500, 500e6) == 50 and st.margin_for(0.0, 500, 500e6) == 0


# ── the outcome table (§5.6) ──

def K(*shots, reads=None):
    return {c: S.Counts(s, s if reads is None else reads[c], 1, True) for c, s in enumerate(shots)}


def AT(S_, verified=False):
    return {"kind": S.AT, "S": S_, "verified": verified}


@pytest.mark.parametrize("counts,n,req,want", [
    (K(10, 10), {0: 10, 1: 10}, None, S.NATURAL),
    (K(9, 10), {0: 10, 1: 10}, None, S.INTERNAL_ERROR),            # a stop with no request
    (K(11, 10), {0: 10, 1: 10}, None, S.INTERNAL_ERROR),           # past n
    (K(4, 7), {0: 10, 1: 10}, {"kind": S.NEXT}, S.STOPPED_EACH),
    (K(10, 10), {0: 10, 1: 10}, {"kind": S.NEXT}, S.NATURAL),
    (K(5, 5), {0: 10, 1: 10}, AT(5, True), S.FIRED),
    (K(5, 5), {0: 10, 1: 10}, AT(5, False), S.FIRED),
    (K(0, 0), {0: 10, 1: 10}, AT(0, True), S.FIRED),
    (K(10, 10), {0: 10, 1: 10}, AT(10, True), S.NATURAL),          # S = n
    (K(10, 10), {0: 10, 1: 10}, AT(25, True), S.NATURAL),          # S > n, VERIFIED
    (K(10, 10), {0: 10, 1: 10}, AT(6, False), S.NATURAL),          # the run ended first
    (K(10, 10), {0: 10, 1: 10}, AT(6, True), S.INTERNAL_ERROR),    # VERIFIED below n, ran out
    (K(7, 7), {0: 10, 1: 10}, AT(5, False), S.CONSISTENT_LATE),
    (K(7, 7), {0: 10, 1: 10}, AT(5, True), S.INTERNAL_ERROR),
    (K(6, 7), {0: 10, 1: 10}, AT(5, False), S.INCONSISTENT),
    (K(6, 10), {0: 10, 1: 10}, AT(5, False), S.INCONSISTENT),
    (K(6, 7), {0: 10, 1: 10}, AT(5, True), S.INTERNAL_ERROR),
    (K(4, 5), {0: 10, 1: 10}, AT(5, False), S.INTERNAL_ERROR),     # below min(S, n)
    (K(3, 5), {0: 3, 1: 10}, AT(5, True), S.FIRED),                # min(S, n) per core
    (K(3, 3), {0: 3, 1: 3}, AT(5, True), S.NATURAL),
])
def test_the_outcome_table(counts, n, req, want):
    got, late_by, why = st.classify(counts, n, req)
    assert got == want, why
    assert (late_by == 2) == (want == S.CONSISTENT_LATE)
    assert bool(why) == (want == S.INTERNAL_ERROR)


def test_an_applied_at_without_its_s_is_an_internal_error():
    assert st.classify(K(5, 5), {0: 9, 1: 9}, {"kind": S.AT})[0] == S.INTERNAL_ERROR


def test_fixed_read_kernels_have_reads_equal_shots_times_r():
    assert st.classify(K(4, 4, reads={0: 8, 1: 8}), {0: 4, 1: 4}, None, {0: 2, 1: 2})[0] == S.NATURAL
    assert st.classify(K(4, 4, reads={0: 8, 1: 7}), {0: 4, 1: 4}, None, {0: 2, 1: 2})[0] == S.INTERNAL_ERROR
    assert st.classify(K(4, 4, reads={0: 8, 1: 7}), {0: 4, 1: 4}, None, {0: 2, 1: None})[0] == S.NATURAL


def test_counts_validity():
    e = 0x1234
    assert st.counts_of([3, 3, e], e, True).valid
    assert st.counts_of([S.SENTINEL] * 3, e, True).why == st.NOT_BOOTED
    assert st.counts_of([S.SENTINEL] * 3, e, False).why == st.NOT_BOOTED
    assert st.counts_of([3, 3, 0], e, True).why == st.UNFINISHED
    assert st.counts_of([3, 3, e], e, False).why == st.TIMEOUT


def _forced(S_, gap=0):
    """A deliberately late issue (a test hook, not P4's issuer): publish AT(S_) without the TOO_LATE
    guard, core 0 first, `gap` model ticks before core 1, and record it unverified."""
    def issue(ctx, ticket):
        f = ctx.drv
        for c in sorted(ctx.progs):
            st.publish(f, ctx.m, c, ctx.progs[c], ctx.run_id[1], S_)
            for _ in range(gap if c == 0 else 0):
                f.tick()
        ticket.info.update(kind=S.AT, S=S_, verified=False)
        return S.ACCEPTED
    return issue


def test_consistent_late_inconsistent_and_accept_inconsistent_on_the_model():
    f, m, progs = stopsoc()
    f.arm(progs)
    rq.rerun(f, m, progs, params=params(progs, 40),
             stop=S.StopSpec(issue=_forced(3), policy=st.AtProgress(8)))
    rec = st.last(f)
    assert rec.outcome == S.CONSISTENT_LATE and rec.shots == {0: 8, 1: 8} and rec.late_by == 5
    f.arm(progs)
    with pytest.raises(S.StopInconsistent, match="shots < 8 are joint on the cores' common grid") as ei:
        rq.rerun(f, m, progs, params=params(progs, 40),
                 stop=S.StopSpec(issue=_forced(3, gap=3), policy=st.AtProgress(8)))
    rec = ei.value.record
    assert rec.outcome == S.INCONSISTENT and rec.shots == {0: 8, 1: 11} and rec.prefix == 8
    assert S.session(f).runs[-1].outcome == S.CERTIFIED and ei.value.out[1]["rq_status"][0] == 11
    assert S.session(f).pending_flush is None                     # not a lifecycle failure
    f.arm(progs)
    out = rq.rerun(f, m, progs, params=params(progs, 40),
                   stop=S.StopSpec(issue=_forced(3, gap=3), policy=st.AtProgress(8), accept_inconsistent=True))
    assert st.last(f).outcome == S.INCONSISTENT and out[0]["rq_status"][0] == 8
    assert out[1]["rq_status"][0] == 11 and st.last(f).common_grid          # the data are not cut


def test_a_verified_request_that_does_not_fire_is_an_internal_error():
    """C2 violated: core 1 runs 3 shots ahead of the reference core 0, which states L = 1."""
    f, m, progs = stopsoc()
    f.arm(progs, every={0: 2})                                   # the reference lags
    with pytest.raises(S.StopInternalError, match="VERIFIED"):
        rq.rerun(f, m, progs, params=params(progs, 60), stop=st.spec(st.AtProgress(4)))
    s = S.session(f)
    assert s.last_failure.kind == "INTERNAL_ERROR" and s.runs[-1].stop.outcome == S.INTERNAL_ERROR
    assert s.pending_flush.reason == S.FLUSH_FAILED
    f.arm(progs)
    rq.rerun(f, m, progs, params=params(progs, 5))
    assert st.last(f).outcome == S.NATURAL


# ── stale requests and the next run (§4.3, §5.1) ──

def test_stale_requests_across_epochs_and_generations_are_late():
    f, m, progs = stopsoc()
    f.arm(progs)
    rq.rerun(f, m, progs, params=params(progs, 5))
    old = S.session(f).runs[-1].run_id
    got = []

    def policy(ctx):
        if not got:
            g, e = ctx.run_id
            got.append(rq.request_stop(f, old, S.NEXT))                       # the previous epoch
            got.append(rq.request_stop(f, (g - 1, e), S.NEXT))                 # an earlier setup
            got.append(rq.request_stop(f, (g, e ^ 0x10), S.AT, 1))
    f.arm(progs)
    rq.rerun(f, m, progs, params=params(progs, 20), stop=S.StopSpec(issue=st.Issuer(), policy=policy))
    assert [x.outcome for x in got] == [S.LATE] * 3 and st.last(f).outcome == S.NATURAL
    assert rq.request_stop(f, S.session(f).runs[-1].run_id, S.NEXT).outcome == S.LATE   # after DONE
    rq.setup(f, m, progs)                                        # a new generation: the old ids are dead
    f.arm(progs)
    rid = []
    rq.rerun(f, m, progs, params=params(progs, 20),
             stop=S.StopSpec(issue=st.Issuer(), policy=lambda ctx: (rid.append(rq.request_stop(f, old, S.NEXT))
                                                                     if not rid else None)))
    assert rid[0].outcome == S.LATE and st.last(f).outcome == S.NATURAL


def test_stop_words_left_in_ram_cannot_stop_the_next_run():
    """A late writer that bypasses the mailbox and leaves the previous run's words: the release
    clears them, and its epoch matches no later run."""
    f, m, progs = stopsoc()
    f.arm(progs)
    rq.rerun(f, m, progs, params=params(progs, 30), stop=st.spec(st.AtProgress(3, S.NEXT)))
    old_e = st.last(f).run_id[1]
    assert st.last(f).outcome == S.STOPPED_EACH

    def late_writer(ctx):                                        # writes the old run's stop words mid-run
        for c in ctx.progs:
            ctx.drv.write32(m.to_host_addr(c, B + STOP_AT), 0)
            ctx.drv.write32(m.to_host_addr(c, B + STOP_EPOCH), old_e)
    f.arm(progs)
    rq.rerun(f, m, progs, params=params(progs, 12), stop=S.StopSpec(issue=st.Issuer(), policy=late_writer))
    assert st.last(f).outcome == S.NATURAL and st.last(f).shots == {0: 12, 1: 12}


@pytest.mark.parametrize("first", ["fired", "next", "too_late", "inconsistent", "internal"])
def test_the_next_run_after_each_kind_of_stop_is_exact(first):
    f, m, progs = stopsoc()
    f.arm(progs, every={0: 2} if first == "internal" else None)
    spec = {"fired": st.spec(st.AtProgress(4), margin=1), "next": st.spec(st.AtProgress(4, S.NEXT)),
            "too_late": st.spec(st.AtProgress(4, S.AT, 2)),
            "inconsistent": S.StopSpec(issue=_forced(1, gap=2), policy=st.AtProgress(4), accept_inconsistent=True),
            "internal": st.spec(st.AtProgress(4))}[first]
    try:
        rq.rerun(f, m, progs, params=params(progs, 40), stop=spec)
    except S.StopInternalError:
        assert first == "internal"
    f.arm(progs)
    out = rq.rerun(f, m, progs, params=params(progs, 9))
    assert st.last(f).outcome == S.NATURAL and [out[c]["rq_status"][0] for c in progs] == [9, 9]
    assert st.last(f).request is None and st.last(f).tickets == []


# ── the uplink drains the counted reads (§4.4, §5.6) ──

def test_the_uplink_drains_exactly_the_counted_reads_after_a_stop():
    f, m, progs = stopsoc(ANTQ, reads=None)
    f.arm(progs, reads={0: 1, 1: 2})                            # core 1 reads twice per shot (heralded-like)
    up = rq.UplinkRun(expected={0: 40, 1: 80}, settle_s=0.0)
    out = rq.rerun(f, m, progs, params=params(progs, 40), uplink=up, stop=st.spec(st.AtProgress(5), margin=1))
    rec = st.last(f)
    run = S.session(f).runs[-1]
    S_ = rec.request["S"]
    assert rec.outcome == S.FIRED and rec.shots == {0: S_, 1: S_} and rec.reads == {0: S_, 1: 2 * S_}
    assert len(out[0]["__uplink"]) == 2 * S_ and len(out[1]["__uplink"]) == 2 * 2 * S_
    assert f.up.accepted[:2] == [S_, 2 * S_] and run.proven and S.FLUSHED in run.states
    assert run.outcome == S.CERTIFIED and S.session(f).pending_flush is None
    f.arm(progs, reads={0: 1, 1: 2})                            # the next run, unstopped, exact
    out = rq.rerun(f, m, progs, params=params(progs, 40), uplink=up)
    assert len(out[1]["__uplink"]) == 2 * 80 and st.last(f).outcome == S.NATURAL


def test_an_empty_run_after_a_stop_before_the_first_check():
    f, m, progs = stopsoc(ANTQ)
    f.arm(progs, every={0: 10 ** 6, 1: 10 ** 6})                # the cores reach their first check late
    up = rq.UplinkRun(expected={0: 40, 1: 40}, settle_s=0.0)
    first = []

    def policy(ctx):
        if not first:
            first.append(rq.request_stop(f, ctx.run_id, S.NEXT))
            f.model["every"] = {}
    out = rq.rerun(f, m, progs, params=params(progs, 40), uplink=up,
                   stop=S.StopSpec(issue=st.Issuer(), policy=policy))
    rec = st.last(f)
    assert rec.outcome == S.STOPPED_EACH and rec.shots == {0: 0, 1: 0} and rec.prefix == 0
    assert "__uplink" not in out[0] and f.up.accepted[:2] == [0, 0] and f.up.final_addr == f.up.run_base


def test_reads_over_the_uplink_nominal_are_an_internal_error():
    f, m, progs = stopsoc(ANTQ, reads=None)
    f.arm(progs, reads={0: 1, 1: 3})
    up = rq.UplinkRun(expected={0: 10, 1: 20}, settle_s=0.0)    # core 1's bound is too small
    with pytest.raises(S.StopInternalError, match="nominal"):
        rq.rerun(f, m, progs, params=params(progs, 10), uplink=up)
    rec = S.session(f).last_failure
    assert rec.kind == "INTERNAL_ERROR" and rec.cleanup[-1] == "admission: flushed (not drained)"


# ── policies (§5.5) ──

def test_at_progress_fires_once_per_run():
    pol = st.AtProgress(3, S.NEXT)
    f, m, progs = stopsoc()
    for _ in range(2):
        f.arm(progs)
        rq.rerun(f, m, progs, params=params(progs, 20), stop=st.spec(pol))
        assert st.last(f).outcome == S.STOPPED_EACH and st.last(f).shots == {0: 3, 1: 3}
        assert st.last(f).request["policy"] == "at_progress"


def test_the_landed_policy_reads_cur_addr_at_bank_granularity():
    f, m, progs = stopsoc(ANTQ)
    f.arm(progs)
    seen = []
    real = f.up.read

    def read(addr):                                              # CUR_ADDR: the committed banks only
        if addr - f.up.base == R.CUR_ADDR:
            committed = (f.up.ptr - f.up.run_base) // 512 * 512
            seen.append(committed // 8)
            return f.up.run_base + committed
        return real(addr)
    f.up.read = read
    up = rq.UplinkRun(expected={0: 200, 1: 200}, base=0x2000, settle_s=0.0)
    rq.rerun(f, m, progs, params=params(progs, 200), uplink=up, stop=st.spec(st.Landed(40, 2), margin=1))
    rec = st.last(f)
    assert rec.request["policy"] == "landed" and rec.request["landed_words"] == 128     # the 2nd bank
    assert rec.request["landed_shots"] == 64 >= 40 and rec.outcome == S.FIRED
    assert rec.request["S"] == rec.request["v_pre"] + 1 + 2 + 1 and rec.shots[0] == rec.request["S"]
    assert seen[0] == 0 and 128 in seen
    with pytest.raises(ValueError, match="rerun\\(uplink"):
        st.Landed(4, 2)(S.RunContext(f, m, progs, (1, 1), S.session(f)))


def test_the_latency_budget():
    """§5.5: overshoot <= (T_pipe + T_backlog + T_poll + T_bcast)/P + 64/W + L + 2 + m, in shots."""
    b = st.overshoot_bound(t_pipe=5e-6, t_backlog=0.0, t_poll=20e-6, t_bcast=25e-6, period_s=1e-6,
                           words_per_shot=14, lead=1, margin=50)
    assert b == pytest.approx(50 + 64 / 14 + 1 + 2 + 50)


# ── the wire form and the remote path (§4.3) ──

def test_spec_wire_round_trip():
    for pol in (None, st.AtProgress(7, S.AT, 30, 1), st.Landed(100, 14)):
        sp = st.spec(pol, margin=3, reference=1, poll_interval=1e-4, poll_cycles=300, accept_inconsistent=True)
        back = st.from_wire(sp.wire)
        assert back.wire == sp.wire and back.accept_inconsistent and back.issue.margin == 3
        assert back.issue.reference == 1
    assert st.spec(lambda ctx: None).wire is None                # an arbitrary callable stays local


@pytest.fixture
def board():
    import Pyro5.api
    from riscq.board.server import BoardServer
    from riscq.driver.remote import RemoteDriver
    fake = StopSoc(HW)
    srv = BoardServer(bits_dir="/nonexistent", driver=fake, params_text=HW)
    daemon = Pyro5.api.Daemon(host="127.0.0.1")
    uri = daemon.register(srv, objectId="riscq.board")
    thread = threading.Thread(target=daemon.requestLoop, daemon=True)
    thread.start()
    yield RemoteDriver(str(uri)), RemoteDriver(str(uri)), fake
    daemon.shutdown()
    thread.join(timeout=5)
    daemon.close()


def test_a_remote_stoppable_run_and_post_stop_outside_the_server_lock(board, monkeypatch):
    monkeypatch.setattr(rq, "POLL_MIN_S", 5.0)
    drv, other, fake = board
    m = fake.m
    progs = {c: sprog(c) for c in (0, 1)}
    rq.setup(drv, m, progs)
    fake.arm(progs)
    out = rq.rerun(drv, m, progs, params=params(progs, 30), stop=st.spec(st.AtProgress(6), margin=1))
    rec = st.last(drv)
    assert rec.outcome == S.FIRED and rec.shots == {0: 10, 1: 10} and list(out[0]["rq_status"][:2]) == [10, 10]
    with pytest.raises(ValueError, match="riscq.stop.spec"):
        rq.rerun(drv, m, progs, stop=S.StopSpec(issue=lambda c, t: None))
    # a second client posts NEXT while remote_rerun holds the server's lock: the cores wait for it
    fake.arm(progs, hold=True)
    posted = []

    def poster():
        other._proxy._pyroClaimOwnership()
        rid = None
        while rid is None:
            rid = rq.current_run(other)
        posted.append(rq.request_stop(other, rid, S.NEXT))
    th = threading.Thread(target=poster)
    th.start()
    rq.rerun(drv, m, progs, params=params(progs, 30), stop=st.spec())
    th.join(timeout=10)
    assert posted == [S.QUEUED]
    rec = st.last(drv)
    assert rec.outcome == S.STOPPED_EACH and rec.shots == {0: 0, 1: 0}
    assert rec.tickets == [(S.NEXT, None, S.ACCEPTED)]


def test_a_remote_inconsistent_run_raises_on_the_client_with_its_data_and_record(board, monkeypatch):
    """after-stage r1 #10: an INCONSISTENT run through the board server. The server's run certifies;
    instead of a RuntimeError naming StopInconsistent it returns the data and the StopRecord, and
    the client raises StopInconsistent with both. With accept_inconsistent the client gets the data.
    The model makes P4's own issuer late: every read-back is 5 ticks, so core 1, published second,
    has passed S when its request lands."""
    monkeypatch.setattr(rq, "POLL_MIN_S", 5.0)
    drv, _, fake = board
    m = fake.m
    progs = {c: sprog(c) for c in (0, 1)}
    rq.setup(drv, m, progs)
    for accept in (False, True):
        fake.arm(progs, tick_on_publish=5)
        spec = st.spec(st.AtProgress(10), margin=0, accept_inconsistent=accept)
        if not accept:
            with pytest.raises(S.StopInconsistent, match="INCONSISTENT") as ei:
                rq.rerun(drv, m, progs, params=params(progs, 40), stop=spec)
            rec, out = ei.value.record, ei.value.out
        else:
            out = rq.rerun(drv, m, progs, params=params(progs, 40), stop=spec)
            rec = st.last(drv)
        assert rec.outcome == S.INCONSISTENT and not rec.request["verified"] and rec is st.last(drv)
        assert rec.shots[0] < rec.shots[1] and rec.prefix == rec.shots[0] and rec.common_grid
        assert [list(out[c]["rq_status"][:2]) for c in progs] == [[rec.shots[c], rec.reads[c]] for c in progs]
