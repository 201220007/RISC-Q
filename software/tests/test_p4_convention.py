"""L0 tests of P4's stop convention at compile time (qubic3; plan P4 v2 §5.2, §8 L0), host-pure.

A stoppable kernel (`compile_kernel(..., stop=StopConvention(...))`) declares the four reserved names,
checks both stop words inside its shot loop and publishes rq_status[0..2]; it may call only the
allow-list of riscq.h primitives that never wait on a peer or an external input, with `barrier` and
`wait_signal` accepted only in top-level statements before the first one that reads the stop words
(the pre-loop rendezvous). Covered: a stoppable kernel compiles and carries its convention (record,
marker, wire form, identity); barrier and wait_signal in or after the shot loop, pop_event, the
other event reads, from_host and a helper of an included header refused; every allow-listed
primitive accepted; the reserved names, their bindings and the structural checks."""

from __future__ import annotations

from pathlib import Path

import pytest

from riscq import run as rq
from riscq.build import Program
from riscq.lang import Array, Group, KernelCompileError, Mailbox, ParamTable, StopConvention, compile_kernel, kernel
from riscq.map import LEAD, READOUT_LEAD, SocMap, SocParams
from riscq.pulses import Pulse, envelopes

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
HW = (CONFIGS / "sim-2q.json").read_text()


# ── the stop convention at compile time (§5.2) ──

def _sim2q():
    return SocMap(SocParams.from_json(HW))


def _demod():
    return ParamTable(2, 0.0, {"sq": Pulse(envelopes.square(40), amp=1.0)})


@kernel
def k_ok(demod: ParamTable, grp: Group, rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int,
         rq_status: Array, n: int, period: int):
    init_pulse_params(demod.pulses)  # noqa: F821
    t = barrier(grp) + period  # noqa: F821   (the pre-loop rendezvous: accepted)
    e = rq_epoch
    s = 0
    t_end = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        play(demod, demod["sq"], t)  # noqa: F821
        t_end = t + 40
        s = s + 1
        rq_status[0] = s
        rq_status[1] = s
        wait_until(t + READOUT_LEAD)  # noqa: F821
        read_res()  # noqa: F821
        t = t + period
    if s > 0:
        wait_until(t_end + LEAD)  # noqa: F821
        read_res()  # noqa: F821
    rq_status[2] = e


def _compile(k, **kw):
    m = _sim2q()
    kw.setdefault("stop", StopConvention("n", at=True, lead=1, reads_per_shot=1))
    tables = kw.pop("tables", dict(demod=_demod()))
    return compile_kernel(k, m, tables=tables, grp=Group([0], id=0), **kw)


def test_a_stoppable_kernel_compiles_and_carries_its_convention():
    p = _compile(k_ok, period=256)
    assert p.stop == {"shots": "n", "n": None, "at": True, "lead": 1, "reads": 1}
    assert p.marker == ("rq_status", 2) and p.arrays["rq_status"] == 3
    assert {"rq_epoch", "rq_stop_epoch", "rq_stop_at", "n"} <= set(p.params)
    assert "if (rq_stop_epoch == e && s >= rq_stop_at)" in p.c_source
    q = _compile(k_ok, period=256, n=40)                                 # a bound n travels with the record
    assert q.stop["n"] == 40 and "n" not in q.params
    w = rq._prog_from_wire(rq._prog_to_wire(q))
    assert w.stop == q.stop and rq.program_identity(w) == rq.program_identity(q)
    plain = Program(q.image, params=q.params, arrays=q.arrays, tables=q.tables, envelopes=q.envelopes)
    plain.marker = q.marker
    assert rq.program_identity(plain) != rq.program_identity(q)          # the convention is identity
    assert "stop" not in rq._prog_to_wire(plain)                          # others keep their S0 form


@kernel
def k_barrier_in_loop(demod: ParamTable, grp: Group, rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int,
                      rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        barrier(grp)  # noqa: F821
        s = s + 1
        rq_status[0] = s
        rq_status[1] = 0
    rq_status[2] = e


@kernel
def k_barrier_after(demod: ParamTable, grp: Group, rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int,
                    rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        s = s + 1
        rq_status[0] = s
        rq_status[1] = 0
    barrier(grp)  # noqa: F821
    rq_status[2] = e


@kernel
def k_wait_signal_in_loop(demod: ParamTable, grp: Group, mb: Mailbox, rq_epoch: int, rq_stop_epoch: int,
                          rq_stop_at: int, rq_status: Array, n: int):
    wait_signal(mb)  # noqa: F821   (pre-loop: fine)
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        wait_signal(mb)  # noqa: F821
        s = s + 1
        rq_status[0] = s
        rq_status[1] = 0
    rq_status[2] = e


@kernel
def k_pop(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        x = pop_event(0)  # noqa: F821
        s = s + 1
        rq_status[0] = s
        rq_status[1] = x
    rq_status[2] = e


@kernel
def k_event_word(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        x = event_word(0, 0)  # noqa: F821
        s = s + 1
        rq_status[0] = s
        rq_status[1] = x
    rq_status[2] = e


@kernel
def k_event_count(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    x = event_count(0)  # noqa: F821   (before the loop: refused all the same)
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        s = s + 1
        rq_status[0] = s
        rq_status[1] = x
    rq_status[2] = e


@kernel
def k_event_time(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        s = s + 1
        rq_status[0] = s
        rq_status[1] = event_time(0)  # noqa: F821
    rq_status[2] = e


@kernel
def k_from_host(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        s = s + from_host()  # noqa: F821
        rq_status[0] = s
        rq_status[1] = 0
    rq_status[2] = e


@kernel
def k_helper(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        seq_shot(s)  # noqa: F821   (an included header's helper: its body is never parsed)
        s = s + 1
        rq_status[0] = s
        rq_status[1] = 0
    rq_status[2] = e


@pytest.mark.parametrize("k,match", [
    (k_pop, "pop_event.. is refused"),
    (k_event_word, "event_word.. is refused"),
    (k_event_count, "event_count.. is refused"),
    (k_event_time, "event_time.. is refused"),
    (k_from_host, "from_host.. is not on the allow-list"),
    (k_helper, "seq_shot.. is not on the allow-list"),
])
def test_event_reads_from_host_and_opaque_helpers_are_refused(k, match):
    with pytest.raises(KernelCompileError, match=match):
        compile_kernel(k, _sim2q(), stop=StopConvention("n"),
                       include=[("seq.h", "static inline void seq_shot(int s) { (void)s; }\n")])


@pytest.mark.parametrize("k,match", [
    (k_barrier_in_loop, "barrier.. halts until a peer"),
    (k_barrier_after, "barrier.. halts until a peer"),
    (k_wait_signal_in_loop, "wait_signal.. halts until a peer"),
])
def test_peer_waits_in_or_after_the_shot_loop_are_refused(k, match):
    kw = {"mb": Mailbox(0, 0)} if k is k_wait_signal_in_loop else {}
    with pytest.raises(KernelCompileError, match=match):
        _compile(k, **kw)


@kernel
def k_every_primitive(demod: ParamTable, grp: Group, mb: Mailbox, rq_epoch: int, rq_stop_epoch: int,
                      rq_stop_at: int, rq_status: Array, n: int):
    init_pulse_params(demod.pulses)  # noqa: F821
    t0 = barrier(grp)  # noqa: F821
    x = wait_signal(mb)  # noqa: F821
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        set_freq(demod, 1)  # noqa: F821
        set_phase(demod, 0, 2)  # noqa: F821
        set_amp(demod, 0, 3)  # noqa: F821
        set_env(demod, 0, 4)  # noqa: F821
        set_dur(demod, 0, 5)  # noqa: F821
        set_start(demod, t0)  # noqa: F821
        set_phase_offset(demod, 6)  # noqa: F821
        set_dc_offset(demod, 7)  # noqa: F821
        fire(demod, 0)  # noqa: F821
        play(demod, demod["sq"], now() + x)  # noqa: F821
        dio_slot(demod, 0, 1, 1, 2)  # noqa: F821
        publish(grp, 1)  # noqa: F821
        r = remote(grp, 0)  # noqa: F821
        signal(mb, r)  # noqa: F821
        wait_until(t0 + 100)  # noqa: F821
        read_res()  # noqa: F821
        x = read_real() + read_imag()  # noqa: F821
        s = s + 1
        rq_status[0] = s
        rq_status[1] = s
    rq_status[2] = e


def test_every_allow_listed_primitive_and_the_pre_loop_rendezvous_are_accepted():
    p = _compile(k_every_primitive, mb=Mailbox(0, 0), stop=StopConvention("n"))
    called = {name for name in st_allowed_names() if f"{name}(" in p.c_source}
    assert called == set(st_allowed_names()), set(st_allowed_names()) - called


def st_allowed_names():
    from riscq.lang.kernel import STOP_ALLOWED
    return sorted(STOP_ALLOWED)


@kernel
def k_plain(rq_epoch: int, out: Array):
    out[0] = rq_epoch


@kernel
def k_no_loop_check(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    if rq_stop_epoch == e and 0 >= rq_stop_at:
        rq_status[0] = 0
    for s in range(n):
        rq_status[0] = s
    rq_status[1] = 0
    rq_status[2] = e


@kernel
def k_no_fin(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        s = s + 1
        rq_status[0] = s
        rq_status[1] = 0


@kernel
def k_missing_name(rq_epoch: int, rq_stop_epoch: int, rq_status: Array, n: int):
    rq_status[0] = n


@kernel
def k_fin_in_loop(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        s = s + 1
        rq_status[0] = s
        rq_status[1] = 0
        rq_status[2] = e
    rq_status[2] = e


@kernel
def k_fin_by_index(rq_epoch: int, rq_stop_epoch: int, rq_stop_at: int, rq_status: Array, n: int):
    e = rq_epoch
    s = 0
    k = 2
    while s < n:
        if rq_stop_epoch == e and s >= rq_stop_at:
            break
        s = s + 1
        rq_status[0] = s
        rq_status[1] = 0
        rq_status[k] = e
    rq_status[2] = e


def test_the_convention_is_checked():
    m = _sim2q()
    conv = StopConvention("n")
    with pytest.raises(KernelCompileError, match="name of the stop convention"):
        compile_kernel(k_plain, m, out=Array(1))
    with pytest.raises(KernelCompileError, match="'rq_stop_at' is missing"):
        compile_kernel(k_missing_name, m, stop=conv)
    with pytest.raises(KernelCompileError, match="never read in a loop"):
        compile_kernel(k_no_loop_check, m, stop=conv)
    with pytest.raises(KernelCompileError, match=r"no store to rq_status\[2\]"):
        compile_kernel(k_no_fin, m, stop=conv)
    for k in (k_fin_in_loop, k_fin_by_index):                  # fin published before the loop ended
        with pytest.raises(KernelCompileError, match=r"rq_status\[2\] \(fin\)"):
            compile_kernel(k, m, stop=conv)
    with pytest.raises(KernelCompileError, match="cannot be bound"):
        _compile(k_ok, rq_epoch=5)
    with pytest.raises(KernelCompileError, match="plain Array"):
        _compile(k_ok, rq_status=Array(4))
    with pytest.raises(KernelCompileError, match="plain Array"):
        _compile(k_ok, rq_status=Array(3, input=True))
    with pytest.raises(KernelCompileError, match="must name an int parameter"):
        _compile(k_ok, stop=StopConvention("shots"))
    with pytest.raises(ValueError):
        StopConvention("n", lead=0)
