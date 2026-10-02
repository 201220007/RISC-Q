"""The results path on the software side (qubic3 plan v2 r2 #9-#11): what an `antq_uplink` build must
refuse, and what it must not allocate. Host-pure: fake pynq/xrfclk/xrfdc, no board, no simulator.

- `PynqDriver` allocates the HostWindow CMA buffer only in hostwindow mode, and applies the numpy-2 /
  pynq-3.0 shim before any pynq allocation in either mode.
- HostWindow programs (`Array(host=True)`) and HostWindow readback are refused early, with the reason,
  on an antq_uplink build: at compile time, in `run.setup` / `run.rerun` before any driver access, and in
  `read_host_array` / `set_host_window`.
- `results_path` survives the remote runner's hop (`to_json` -> the server's same-build guard).
"""

from __future__ import annotations

import importlib
import json
import sys
import types
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

from riscq import run as rq
from riscq.build import Image, Program
from riscq.lang import Array, compile_kernel, kernel
from riscq.lang.kernel import KernelCompileError
from riscq.map import SocMap, SocParams

CONFIGS = Path(__file__).resolve().parents[1] / "configs"


@pytest.fixture
def antq():
    return SocMap(SocParams.load(CONFIGS / "sim-2q-antq.json"))


@pytest.fixture
def hostwin():
    return SocMap(SocParams.load(CONFIGS / "sim-2q.json"))


class RecordingDriver:
    """Records every seam op; any op at all is a failure for a refusal that must come first."""

    def __init__(self):
        self.ops = []
        self.host_base = 0x7000_0000

    def read32(self, addr):
        self.ops.append(("r", addr)); return 0

    def write32(self, addr, value):
        self.ops.append(("w", addr, value))

    def read_block(self, addr, nbytes):
        self.ops.append(("rb", addr)); return bytes(nbytes)

    def write_block(self, addr, data):
        self.ops.append(("wb", addr))

    def read_host(self, offset, nbytes):
        self.ops.append(("rh", offset)); return bytes(nbytes)


def _host_prog():
    return Program(Image(data=b"\x6f\x00\x00\x00", symbols={}, entry=0x8000_0000),
                   arrays={"win": 4}, host_arrays={"win": (0, 4)})


@kernel
def k_win(win: Array, n: int):
    for i in range(n):
        win[i] = i


# ── the map ──────────────────────────────────────────────────────────────────────────────────────

def test_map_has_no_host_window_in_antq(antq, hostwin):
    assert antq.hostwin_bytes_total == 0
    assert hostwin.hostwin_bytes_total == 2 << 24
    with pytest.raises(ValueError, match="results_path='antq_uplink'"):
        antq.hostwin_offset(0)
    assert hostwin.hostwin_offset(1) == 1 << 24


# ── compile time ─────────────────────────────────────────────────────────────────────────────────

def test_compile_refuses_host_arrays_on_antq(antq):
    with pytest.raises(KernelCompileError, match="host=True.*antq_uplink.*no HostWindow"):
        compile_kernel(k_win, antq, win=Array(8, host=True))


def test_compile_accepts_ram_arrays_on_antq(antq):
    prog = compile_kernel(k_win, antq, win=Array(8))
    assert prog.host_arrays == {} and "win" in prog.arrays


def test_hostwindow_compile_unchanged(hostwin):
    assert compile_kernel(k_win, hostwin, win=Array(8, host=True)).host_arrays == {"win": (0, 8)}


# ── run layer: refused before any driver access ──────────────────────────────────────────────────

def test_setup_refuses_a_host_program_before_touching_the_driver(antq):
    drv = RecordingDriver()
    with pytest.raises(ValueError, match="needs the HostWindow.*antq_uplink"):
        rq.setup(drv, antq, {0: _host_prog()})
    assert drv.ops == []


def test_setup_refuses_before_the_remote_hop(antq):
    drv = RecordingDriver()
    drv.remote = mock.Mock()
    with pytest.raises(ValueError, match="needs the HostWindow"):
        rq.setup(drv, antq, {0: _host_prog()})
    drv.remote.setup.assert_not_called()


def test_rerun_refuses_a_host_program(antq):
    drv = RecordingDriver()
    with pytest.raises(ValueError, match="needs the HostWindow"):
        rq.rerun(drv, antq, {0: _host_prog()})
    assert drv.ops == []


def test_host_readback_and_base_are_refused(antq):
    drv = RecordingDriver()
    with pytest.raises(ValueError, match="needs the HostWindow"):
        rq.read_host_array(drv, antq, 0, _host_prog(), "win")
    with pytest.raises(ValueError, match="needs the HostWindow"):
        rq.set_host_window(drv, antq, 0x7000_0000)
    assert drv.ops == []


def test_antq_setup_never_writes_the_hostwin_registers(antq):
    """A driver that happens to carry `host_base` must not make setup program HOSTWIN_LO/HI, which an
    antq_uplink build does not have."""
    drv = RecordingDriver()
    prog = Program(Image(data=b"\x6f\x00\x00\x00", symbols={}, entry=0x8000_0000))
    rq.setup(drv, antq, {0: prog})
    hostwin_regs = {antq.host_ctrl + antq.HOST_HOSTWIN_LO, antq.host_ctrl + antq.HOST_HOSTWIN_HI}
    assert not [op for op in drv.ops if op[0] == "w" and op[1] in hostwin_regs]


def test_hostwindow_setup_still_programs_the_base(hostwin):
    drv = RecordingDriver()
    rq.setup(drv, hostwin, {0: _host_prog()})
    assert ("w", hostwin.host_ctrl + hostwin.HOST_HOSTWIN_LO, 0x7000_0000) in drv.ops


# ── remote round trip ────────────────────────────────────────────────────────────────────────────

def test_results_path_survives_the_remote_hop(antq, hostwin):
    for m in (antq, hostwin):
        wire = rq._params_json(m)
        assert json.loads(wire)["results_path"] == m.params.results_path
        assert SocParams.from_json(wire) == m.params
    # the server's same-build guard compares whole specs: the mode alone makes them differ
    a = SocParams.from_json(rq._params_json(antq))
    h = SocParams.from_json(json.dumps({**json.loads(rq._params_json(antq)), "results_path": "hostwindow"}))
    assert a != h


# ── PynqDriver: CMA only in hostwindow mode, shim first ─────────────────────────────────────────

@pytest.fixture
def fake_board(monkeypatch):
    """Stub pynq / xrfclk / xrfdc and import riscq.board.pynq_driver against them."""
    events = []
    pynq = types.ModuleType("pynq")
    pynq.Overlay = lambda xsa, download=True: (events.append("overlay"),
                                               types.SimpleNamespace(rf_data_converter=mock.MagicMock()))[1]
    pynq.MMIO = lambda base, size: mock.MagicMock()

    class Buf:
        def __init__(self, n):
            self.nbytes, self.device_address, self.freed = n, 0x6000_0000, 0

        def freebuffer(self):
            self.freed += 1

    def allocate(shape, dtype):
        events.append("allocate")
        return Buf(shape[0])

    pynq.allocate = allocate
    xrfclk = types.ModuleType("xrfclk")
    xrfclk.set_ref_clks = lambda **kw: events.append("refclks")
    xrfdc = types.ModuleType("xrfdc")
    for name, mod in (("pynq", pynq), ("xrfclk", xrfclk), ("xrfdc", xrfdc)):
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.delitem(sys.modules, "riscq.board.pynq_driver", raising=False)
    pd = importlib.import_module("riscq.board.pynq_driver")
    monkeypatch.setattr(pd, "numpy2_pynq_shim", lambda: events.append("shim"))
    monkeypatch.setattr(pd, "_check_cma", lambda n: events.append(("cma", n)))
    yield pd, events
    sys.modules.pop("riscq.board.pynq_driver", None)


BOARD = {"mts": None, "dac_nyquist": None, "adc_nyquist": None}


def _board_cfg(tmp_path, name):
    p = tmp_path / name
    p.write_text((CONFIGS / name).read_text())
    return p


def test_pynq_driver_antq_allocates_no_host_window(fake_board, tmp_path):
    pd, events = fake_board
    board = {**pd.BOARD_DEFAULTS, "mts": None, "dac_nyquist": {"default": 2}}
    drv = pd.PynqDriver("x.xsa", str(_board_cfg(tmp_path, "sim-2q-antq.json")), board=board)
    assert "allocate" not in events and not [e for e in events if isinstance(e, tuple)]
    assert drv.host_base is None
    assert events[0] == "shim", events
    with pytest.raises(ValueError, match="no host-window buffer"):
        drv.read_host(0, 4)
    drv.close()                                   # nothing to free, must not raise


def test_pynq_driver_close_tears_down_the_cached_uplink_readout(fake_board, tmp_path):
    """qubic3 S0 after-stage r1 #13: close() also closes the DdrBoard under the run layer's cached
    DdrReadout, so its drain buffer goes back to CMA before a reload, and drops the cache; a fixed
    buffer belongs to its owner and is not freed."""
    from riscq.board.ddr_board import DdrBoard
    from riscq.ddr import DdrReadout, attach_readout, readout_for
    from riscq.map import SocMap
    pd, events = fake_board
    board = {**pd.BOARD_DEFAULTS, "mts": None, "dac_nyquist": {"default": 2}}
    drv = pd.PynqDriver("x.xsa", str(_board_cfg(tmp_path, "sim-2q-antq.json")), board=board)
    m = SocMap(drv.params)

    class Buf:
        nbytes, freed = 1 << 16, 0

        def freebuffer(self):
            self.freed += 1
    port = readout_for(drv, m).drv
    assert isinstance(port, DdrBoard) and port.soc is drv
    port._buf = drain = Buf()                       # as a drain leaves it
    drv.close()
    assert drain.freed == 1 and port._buf is None and drv._rq_readout is None
    owned = Buf()
    attach_readout(drv, DdrReadout(DdrBoard(soc=drv, buffer=owned), soc_map=m))
    drv.close()
    assert owned.freed == 0 and drv._rq_readout is None
    drv.close()                                     # nothing cached: a no-op


def _antq_board_driver(fake_board, tmp_path):
    from riscq.ddr import readout_for
    from riscq.map import SocMap
    pd, _ = fake_board
    board = {**pd.BOARD_DEFAULTS, "mts": None, "dac_nyquist": {"default": 2}}
    drv = pd.PynqDriver("x.xsa", str(_board_cfg(tmp_path, "sim-2q-antq.json")), board=board)
    return drv, readout_for(drv, SocMap(drv.params))


_FREED: list = []


class _CmaBuf:
    """pynq's PynqBuffer as far as freeing goes: `freebuffer()`, which its destructor also calls."""

    nbytes, device_address = 1 << 16, 0x7000_0000

    def __init__(self, name):
        self.name, self.freed = name, False

    def freebuffer(self):
        if not self.freed:
            self.freed = True
            _FREED.append(self.name)

    def __del__(self):
        self.freebuffer()


def _arm(port, buf, stuck=False):
    """Arm a real transfer on `port` into `buf`, over the fake S2MM channel of tests/test_ddr_board
    (`stuck`: a soft reset of it is never confirmed)."""
    from riscq.ddr import DdrMap
    from tests.test_ddr_board import FakeMMIO

    class Channel(FakeMMIO):
        def write(self, off, val):
            if self.stuck and off == 0x30 and val & self.RESET:
                raise OSError("bus error on DMACR")
            super().write(off, val)
    m = DdrMap()
    dma = Channel(m.dma_base, m.dma_size)
    dma.stuck = stuck
    port._mmio[(m.dma_base, m.dma_size)] = dma
    port._buf = buf                                     # the drain buffer `_cma` reuses
    assert port.dma_recv_prepare(4096) is buf
    return dma


def _confirmed_reset():
    """A soft reset of the channel that is confirmed, from a fresh DdrBoard (the recovery path)."""
    from riscq.board.ddr_board import DdrBoard
    from riscq.ddr import DdrMap
    from tests.test_ddr_board import FakeMMIO
    m = DdrMap()
    rescue = DdrBoard()
    rescue._mmio[(m.dma_base, m.dma_size)] = FakeMMIO(m.dma_base, m.dma_size)
    rescue.dma_reset()


def test_a_driver_abandoned_mid_transfer_leaves_its_buffer_alive(fake_board, tmp_path):
    """qubic3 r4: the buffer is registered when the transfer is armed, so dropping the driver and its
    readout mid-transfer, without close(), and collecting garbage frees nothing, although pynq frees in
    the destructor; only a confirmed reset of the channel releases it."""
    import gc
    from riscq.board.ddr_board import inflight
    _FREED.clear()
    drv, rd = _antq_board_driver(fake_board, tmp_path)
    port = rd.drv
    _arm(port, _CmaBuf("drain"))
    del drv, rd, port
    gc.collect()
    assert _FREED == [] and [buf.name for _, buf in inflight()] == ["drain"]
    _confirmed_reset()
    gc.collect()
    assert inflight() == [] and _FREED == ["drain"]


def test_a_failed_close_raises_marks_the_driver_unusable_and_frees_nothing(fake_board, tmp_path):
    """qubic3 r3 #1, #4: close() with a transfer in flight and a reset that is never confirmed raises
    and marks the driver unusable (the uplink's own windows refuse too, the reset excepted); the
    buffer stays registered after the driver is dropped, until a confirmed reset."""
    import gc
    from riscq.board.ddr_board import inflight
    from riscq.ddr import DdrMap
    _FREED.clear()
    drv, rd = _antq_board_driver(fake_board, tmp_path)
    port = rd.drv
    _arm(port, _CmaBuf("drain"), stuck=True)
    with pytest.raises(RuntimeError, match="stays registered as in flight"):
        drv.close()
    with pytest.raises(RuntimeError, match="unusable"):
        port.read32(DdrMap().ctrl_base + 0x2C)          # STATUS, through DdrBoard's own window
    del drv, rd, port
    gc.collect()
    assert _FREED == [] and [buf.name for _, buf in inflight()] == ["drain"]
    _confirmed_reset()
    gc.collect()
    assert inflight() == [] and _FREED == ["drain"]


def test_attach_readout_is_refused_while_a_transfer_is_in_flight_or_the_driver_is_unusable(fake_board,
                                                                                           tmp_path):
    """qubic3 r3 #2, r4: a new readout cannot replace the cached one while a buffer is registered (a
    transfer in flight, or one whose stop is unconfirmed), nor once the driver is unusable."""
    from riscq.board.ddr_board import DdrBoard
    from riscq.ddr import DdrReadout, attach_readout
    from riscq.map import SocMap
    drv, rd = _antq_board_driver(fake_board, tmp_path)
    m, port = SocMap(drv.params), rd.drv
    _arm(port, _CmaBuf("a"))
    with pytest.raises(RuntimeError, match="in flight"):
        attach_readout(drv, DdrReadout(DdrBoard(soc=drv), soc_map=m))
    assert drv._rq_readout is rd
    port.dma_reset()                                    # the stop is confirmed: attach is allowed
    attach_readout(drv, rd)
    dma = _arm(port, _CmaBuf("b"), stuck=True)
    with pytest.raises(RuntimeError, match="stays registered"):
        drv.close()
    dma.stuck = False
    port.dma_reset()                                    # the reset an unusable driver still allows
    with pytest.raises(RuntimeError, match="unusable"):
        attach_readout(drv, DdrReadout(DdrBoard(soc=drv), soc_map=m))
    assert drv._rq_readout is rd


@pytest.mark.parametrize("stuck", [False, True], ids=["stop-confirmed", "stop-failed"])
def test_a_concurrent_attach_waits_for_close_and_sees_its_outcome(fake_board, tmp_path, stuck):
    """qubic3 r5: close() holds the board lock through its teardown, its unusable mark and the cache
    drop, and attach_readout checks under it: an attach started mid-close waits, then lands after a
    confirmed stop, or is refused after a failed one, which keeps the old readout."""
    import threading
    from riscq.board.ddr_board import DdrBoard
    from riscq.ddr import DdrReadout, attach_readout
    from riscq.map import SocMap
    drv, rd = _antq_board_driver(fake_board, tmp_path)
    dma = _arm(rd.drv, _CmaBuf("drain"), stuck=stuck)
    entered, go, errors = threading.Event(), threading.Event(), {}
    write = dma.write

    def held_reset(off, val):
        if off == 0x30 and val & dma.RESET:
            entered.set()                               # close() is resetting, under the board lock
            go.wait(5)
        write(off, val)
    dma.write = held_reset
    new = DdrReadout(DdrBoard(soc=drv), soc_map=SocMap(drv.params))

    def run(name, fn):
        try:
            fn()
        except RuntimeError as e:
            errors[name] = str(e)
    closer = threading.Thread(target=run, args=("close", drv.close))
    closer.start()
    assert entered.wait(5)
    attacher = threading.Thread(target=run, args=("attach", lambda: attach_readout(drv, new)))
    attacher.start()
    attacher.join(0.2)
    assert attacher.is_alive(), "attach_readout ran while close() held the board lock"
    go.set()
    closer.join(5)
    attacher.join(5)
    if stuck:
        assert "unusable" in errors["attach"] and "stays registered" in errors["close"]
        assert drv._rq_readout is rd
    else:
        assert errors == {} and drv._rq_readout is new and _FREED[-1:] == ["drain"]


def test_pynq_driver_close_stops_a_transfer_in_flight_then_frees(fake_board, tmp_path):
    """The success path: the S2MM reset is confirmed, then the buffer leaves the registry and is
    freed, the readout dropped, and close() returns normally."""
    from riscq.board.ddr_board import inflight
    _FREED.clear()
    drv, rd = _antq_board_driver(fake_board, tmp_path)
    dma = _arm(rd.drv, _CmaBuf("drain"))
    drv.close()
    assert any(e[0] == "w" and e[1] == 0x30 and e[2] & dma.RESET for e in dma.log)
    assert _FREED == ["drain"] and drv._rq_readout is None and drv._unusable is None and inflight() == []


def test_pynq_driver_hostwindow_allocates_after_the_shim(fake_board, tmp_path):
    pd, events = fake_board
    board = {**pd.BOARD_DEFAULTS, "mts": None, "dac_nyquist": {"default": 2}}
    drv = pd.PynqDriver("x.xsa", str(_board_cfg(tmp_path, "sim-2q.json")), board=board)
    assert events.count("allocate") == 1 and ("cma", 2 << 24) in events
    assert events.index("shim") < events.index("overlay") < events.index("allocate"), events
    assert drv.host_base == 0x6000_0000


# ── the shim itself ──────────────────────────────────────────────────────────────────────────────

def test_numpy2_shim_patches_pynqbuffer_once(monkeypatch):
    from riscq.board import pynq_compat

    class NdArray:
        device = property(lambda self: "cpu")      # numpy >= 2: a read-only `device`

    class PynqBuffer(NdArray):
        pass

    fake_np = types.SimpleNamespace(ndarray=NdArray, __version__="2.2.6")
    pynq = types.ModuleType("pynq")
    buffer = types.ModuleType("pynq.buffer")
    buffer.PynqBuffer = PynqBuffer
    pynq.buffer = buffer
    monkeypatch.setitem(sys.modules, "numpy", fake_np)
    monkeypatch.setitem(sys.modules, "pynq", pynq)
    monkeypatch.setitem(sys.modules, "pynq.buffer", buffer)
    assert pynq_compat.numpy2_pynq_shim() is True
    b = PynqBuffer()
    b.device = "embedded"                          # the assignment pynq 3.0.0 does: must work now
    assert b.device == "embedded"
    assert pynq_compat.numpy2_pynq_shim() is False  # idempotent


def test_ddr_board_uses_the_shared_shim(monkeypatch):
    from riscq.board import ddr_board, pynq_compat
    calls = []
    monkeypatch.setattr(pynq_compat, "numpy2_pynq_shim", lambda: calls.append(1))
    ddr_board.DdrBoard._numpy2_pynq_shim()
    assert calls == [1]


def test_pynq_driver_no_rf_bringup_writes_no_rf_state(fake_board, tmp_path):
    """g6_ddr_bringup's no-RF board.json (MTS, Nyquist and VOP all off) must construct the driver
    without touching the RFDC tiles (the P3a Nyquist opt-out, carried onto 8300a1c)."""
    pd, events = fake_board
    no_rf = {"mts": None, "dac_nyquist": None, "adc_nyquist": None, "dac_current": {}}
    drv = pd.PynqDriver("x.xsa", str(_board_cfg(tmp_path, "sim-2q-antq.json")), board=no_rf)
    assert drv.rfdc.dac_tiles.mock_calls == [] and drv.rfdc.adc_tiles.mock_calls == []
    assert drv.mts_result is None and drv.host_base is None


# ── riscq.cal on an antq_uplink build (plan v2 r2 #11; the library itself is P6) ──────────────────

def _cal_exp(m, measure, shots=16):
    from riscq.cal.experiment import Experiment
    from riscq.cal.sequence import Gate
    from tests.cal_fixtures import _cfg
    return Experiment(_cfg(m), [0], {0: [Gate("x90")]}, {0: ()}, (), measure, shots, label="p3b")


def test_cal_raw_defaults_to_off_core_capture_through_the_uplink_on_antq(responder, antq):
    """Measure.raw() defaults to host=True, off-core capture. Until P6 an antq_uplink build refused it
    at compile time (plan v2 P3b r2 #11, "until P6"); since qubic3 P6 (plan P6 v2 §4.1) it compiles in
    mode UPLINK with no host-window array, and every rerun passes the uplink spec, one word per shot."""
    from riscq.cal.batched import UPLINK
    from riscq.cal.measure import Measure
    r = responder(CONFIGS / "sim-2q-antq.json")
    exp = _cal_exp(antq, Measure.raw())
    r.answer(lambda progs, params: {c: {"out": np.zeros(2 * exp.shots, int)} for c in progs})
    exp.run(r.drv)
    (prog,) = r.setups[0].values()
    assert prog.bindings["mode"] == UPLINK and not prog.host_arrays and prog.marker == ("out", 0)
    assert r.uplinks and all(u.expected == {0: exp.shots} for u in r.uplinks)


def test_cal_iqsum_and_ram_raw_still_compile_on_antq(responder, antq):
    """IQSUM does not use the host window, and raw with host=False keeps its capture in core RAM:
    both run on an antq_uplink build (their results go through the core, not the uplink)."""
    from riscq.cal.measure import Measure
    for measure, name in ((Measure.iqsum(4), "iqsum"), (Measure.raw(host=False), "raw-ram")):
        r = responder(CONFIGS / "sim-2q-antq.json")
        r.answer(lambda progs, params: {c: {"out": np.zeros(4096, int)} for c in progs})
        try:
            _cal_exp(antq, measure).run(r.drv)
        except Exception as e:                     # a result-shape complaint is fine; a HostWindow one is not
            assert "HostWindow" not in str(e) and "host=True" not in str(e), f"{name}: {e}"
        assert r.setups and all(not p.host_arrays for p in r.setups[0].values()), name
