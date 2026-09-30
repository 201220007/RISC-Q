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
