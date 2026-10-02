"""qubic3 BT T1 (plan BT v2 §1 T1): `PynqDriver.pl_reset`, the RF replay and the one-PL-thread guard,
host-pure on fakes of the PS GPIO page, xrfdc and the SoC, uplink and DMA windows.

`Board` is the fake hardware. Every access lands in one event log with the pl_resetn0 level at that moment
(bank-5 bits 31:28 of the GPIO DATA register), so a test can assert the order of the steps and that nothing
touched the PL while the resets were low, or at all after an unconfirmed release. Driving the resets low
resets the fake RFDC (tile state 0, MTS, Nyquist zones and VOP lost); after the release its tiles come back
to state 15 after a few IPStatus polls. Injections: an exception or a wrong value at a GPIO access, a tile
that never comes back, tile 2's PLL unlocked, a changed clock source, free-run latencies out of the MTS
window, MTS misses, a Nyquist zone that does not take."""

from __future__ import annotations

import importlib
import json
import os
import subprocess
import sys
import threading
import types
from pathlib import Path

import pytest

SW = Path(__file__).resolve().parents[1]
CONFIG = SW / "configs" / "sim-2q-antq.json"
AXI_BASE, GPIO_BASE, UP_BASE, DMA_BASE = 0x8000_0000, 0xFF0A_0000, 0x9000_0000, 0x9001_0000
MASK_DATA_5_MSW, DATA_5, DIRM_5 = 0x02C, 0x054, 0x344
NO_RF = {"mts": None, "dac_nyquist": None, "adc_nyquist": None, "dac_current": {}}
# the BT session's RF state: board_g6.json (= veneno's server_config.yaml) with MTS 240/72 (DECISIONS #26)
RF = {"mts": (240, 72), "adc_nyquist": 1, "dac_nyquist": {"default": 2, "3,0": 1, "3,1": 1, "3,2": 1, "3,3": 1},
      "dac_current": {f"{t},{b}": (7000 if t == 3 else 40000) for t in range(4) for b in range(4)}}


class Board:
    """The fake hardware behind every pynq.MMIO window and the RFDC. `inject` names a fault (see the
    module docstring); `log_path` mirrors every event to a file (the subprocess harness)."""

    def __init__(self, inject=None, log_path=None, ip_form="dict"):
        self.inject = dict(inject or {})
        self.log_path = log_path
        self.events = []
        self.data5 = self.dirm5 = 0xF000_0000
        self.soc = {}
        self.ddr_status = 0xCA1B_0003
        self.rfdc = FakeRFDC(self, ip_form)

    def low(self):
        return (self.data5 >> 28) & 0xF != 0xF

    def ev(self, region, op, off, val=None):
        e = (region, op, off, val, self.low())
        self.events.append(e)
        if self.log_path:
            with open(self.log_path, "a") as f:
                f.write(json.dumps(e) + "\n")

    # the PS GPIO page
    def gpio_read(self, off):
        self.ev("gpio", "r", off)
        if off == DIRM_5:
            return self.dirm5
        if off == DATA_5:
            k = "data5_read_%d" % sum(1 for e in self.events if e[:3] == ("gpio", "r", DATA_5))
            if self.inject.get(k) == "raise":
                raise OSError(f"bus error on DATA_5 (injected, {k})")
            if self.inject.get(k) is not None:
                return self.inject[k]
            return self.data5
        return 0

    def gpio_write(self, off, val):
        self.ev("gpio", "w", off, val)
        if off != MASK_DATA_5_MSW:
            raise AssertionError(f"the driver wrote GPIO register 0x{off:x}")
        if self.inject.get("high_write") == "raise" and val == 0x0FFF_F000:
            raise OSError("bus error on the high write (injected)")
        was = self.low()
        mask, data = val >> 16, val & 0xFFFF
        hi = ((self.data5 >> 16) & mask) | (data & ~mask & 0xFFFF)
        if self.inject.get("stuck_low") and val == 0x0FFF_F000:
            hi &= 0x7FFF                                           # bit 31 does not come back
        self.data5 = (hi << 16) | (self.data5 & 0xFFFF)
        if self.low() and not was:
            self.rfdc.reset()
        elif was and not self.low():
            self.rfdc.release()

    # the SoC window (PynqDriver's own MMIO), the uplink's control slave and the DMA's lite port
    def soc_read(self, off):
        self.ev("soc", "r", off)
        return self.ddr_status if off == 0xA0058 else self.soc.get(off, 0)

    def soc_write(self, off, val):
        self.ev("soc", "w", off, val if isinstance(val, int) else len(val))
        self.soc[off] = val

    def up_read(self, off):
        self.ev("uplink", "r", off)
        return {0x50: 2, 0x54: (8 << 24) | (4 << 16) | (8 << 8) | 16, 0x58: 0x3C1}.get(off, 0)

    def up_write(self, off, val):
        self.ev("uplink", "w", off, val)

    def dma_read(self, off):
        self.ev("dma", "r", off)
        return 0x1 if off == 0x34 else 0

    def dma_write(self, off, val):
        self.ev("dma", "w", off, val)

    def mmio(self, base, size):
        regions = {AXI_BASE: (self.soc_read, self.soc_write), GPIO_BASE: (self.gpio_read, self.gpio_write),
                   UP_BASE: (self.up_read, self.up_write), DMA_BASE: (self.dma_read, self.dma_write)}
        rd, wr = regions[base]

        class MMIO:
            def read(self, off):
                return rd(off)

            def write(self, off, val):
                return wr(off, val)
        return MMIO()


class FakeRFDC:
    """xrfdc's RFdc as the driver uses it: IPStatus, tiles with ClockSource, PLLLockStatus and blocks
    (NyquistZone, SetDACVOP), the two MTS configs and mts_dac/mts_adc (the G6' dry-run model)."""

    def __init__(self, board, ip_form):
        self.b, self.ip_form = board, ip_form
        self.free = {"dac": [240] * 4, "adc": [64] * 4}
        self.flaky = board.inject.get("mts_flaky", 0)
        self.ready_in = 0
        self.pll_bad = False
        self.mts_dac_config = types.SimpleNamespace(RefTile=0, Tiles=0, SysRef_Enable=0, Target_Latency=-1,
                                                    Latency=[0] * 4)
        self.mts_adc_config = types.SimpleNamespace(RefTile=0, Tiles=0, SysRef_Enable=0, Target_Latency=-1,
                                                    Latency=[0] * 4)
        rfdc = self

        class Block:
            def __init__(self, kind, t, i):
                self.kind, self.t, self.i, self._zone, self.vop = kind, t, i, 1, None

            @property
            def NyquistZone(self):
                rfdc.ev(f"{self.kind}{self.t}{self.i} zone?")
                return self._zone

            @NyquistZone.setter
            def NyquistZone(self, z):
                rfdc.ev(f"{self.kind}{self.t}{self.i} zone={z}")
                if not board.inject.get("zone_ignored") == (self.kind, self.t, self.i):
                    self._zone = z

            def SetDACVOP(self, ua):
                rfdc.ev(f"{self.kind}{self.t}{self.i} vop={ua}")
                self.vop = ua

        class Tile:
            def __init__(self, kind, t):
                self.kind, self.t = kind, t
                self.blocks = [Block(kind, t, i) for i in range(4)]
                self.source = 1 if t == 2 else 0

            @property
            def ClockSource(self):
                rfdc.ev(f"{self.kind}{self.t} source?")
                return self.source

            @property
            def PLLLockStatus(self):
                rfdc.ev(f"{self.kind}{self.t} pll?")
                return 1 if self.t == 2 and (not rfdc.ready() or rfdc.pll_bad) else 2
        self.dac_tiles = [Tile("dac", t) for t in range(4)]
        self.adc_tiles = [Tile("adc", t) for t in range(4)]

    def ev(self, what):
        self.b.ev("rf", what, None)

    def ready(self):
        return self.ready_in <= 0

    def reset(self):
        """pl_resetn0 low: the RFDC's AXI reset. Its software state is lost."""
        self.ready_in = 10 ** 9
        for kind in ("dac", "adc"):
            for t in getattr(self, f"{kind}_tiles"):
                for blk in t.blocks:
                    blk._zone, blk.vop = 1, None
        self.mts_dac_config.Latency = [0] * 4
        self.mts_adc_config.Latency = [0] * 4

    def release(self):
        inj = self.b.inject
        self.ready_in = 10 ** 9 if inj.get("never_ready") else 3
        self.pll_bad = bool(inj.get("pll_unlocked"))
        if inj.get("clock_source_changed"):
            self.dac_tiles[0].source = 1
        if inj.get("free_after_pulse"):
            self.free["dac"] = list(inj["free_after_pulse"])

    @property
    def IPStatus(self):
        self.ev("IPStatus?")
        ok = self.ready()
        if 0 < self.ready_in < 10 ** 9:
            self.ready_in -= 1

        def st():
            return {"IsEnabled": 1, "TileState": 15 if ok else 6, "PowerUpState": 1 if ok else 0, "PLLState": 1,
                    "BlockStatusMask": 15}
        tiles = {"DACTileStatus": [st() for _ in range(4)], "ADCTileStatus": [st() for _ in range(4)]}
        if self.ip_form == "attr":
            return types.SimpleNamespace(**{k: [types.SimpleNamespace(**s) for s in v] for k, v in tiles.items()})
        return {**tiles, "State": 1}

    def _sync(self, kind, cfg):
        self.ev(f"mts_{kind} {cfg.Target_Latency}")
        free, tgt = self.free[kind], cfg.Target_Latency
        if tgt == -1:
            cfg.Latency = list(free)
        elif all(f <= tgt <= f + 31 for f in free):
            cfg.Latency = [tgt] * 4
            if kind == "dac" and self.flaky:
                self.flaky -= 1
                cfg.Latency = [tgt + 1] + [tgt] * 3
        else:
            raise RuntimeError(f"Alignment correction delay required exceeds maximum (31) for {kind} tiles")

    def mts_dac(self):
        self._sync("dac", self.mts_dac_config)

    def mts_adc(self):
        self._sync("adc", self.mts_adc_config)


def install(board, setitem):
    """Stub pynq / xrfclk / xrfdc over `board` and import riscq.board.pynq_driver against them."""
    pynq = types.ModuleType("pynq")
    pynq.MMIO = board.mmio
    pynq.Overlay = lambda xsa, download=True: types.SimpleNamespace(rf_data_converter=board.rfdc)
    pynq.allocate = lambda shape, dtype: (_ for _ in ()).throw(AssertionError("no allocation expected"))
    xrfclk = types.ModuleType("xrfclk")
    xrfclk.set_ref_clks = lambda **kw: None
    xrfdc = types.ModuleType("xrfdc")
    for name, mod in (("pynq", pynq), ("xrfclk", xrfclk), ("xrfdc", xrfdc)):
        setitem(sys.modules, name, mod)
    sys.modules.pop("riscq.board.pynq_driver", None)
    pd = importlib.import_module("riscq.board.pynq_driver")
    pd.numpy2_pynq_shim = lambda: None
    pd._check_cma = lambda n: None
    pd.READY_POLL_S = 0.0
    return pd


def bring_up(pd, bind=True):
    """The BT kit's order: the driver with no RF state at load, the PL thread bound at the fork, the
    readout over a DdrBoard with both windows mapped, then rf_init (`rf_replay(RF)`, phase 10)."""
    from riscq.ddr import readout_for
    from riscq.map import SocMap
    drv = pd.PynqDriver("x.xsa", str(CONFIG), board=NO_RF)
    if bind:
        drv.bind_pl_thread()
    m = SocMap(drv.params)
    rd = readout_for(drv, m)
    rd.status()                                                   # maps the uplink window
    rd.drv.dma_idle()                                             # maps the DMA window
    drv.rf_replay(RF)
    return drv, m, rd


@pytest.fixture
def hw(monkeypatch):
    def make(inject=None, bind=True, ip_form="dict"):
        board = Board(inject, ip_form=ip_form)
        pd = install(board, monkeypatch.setitem)
        monkeypatch.setattr(pd, "READY_TIMEOUT_S", 0.2)
        return (pd, board) + bring_up(pd, bind)
    yield make
    sys.modules.pop("riscq.board.pynq_driver", None)


def _gpio_writes(board, start=0):
    return [e for e in board.events[start:] if e[:2] == ("gpio", "w")]


# ── the success path: the step order ─────────────────────────────────────────────────────────────

def test_the_pulse_runs_its_steps_in_order_with_the_drain_before_the_low_write(hw):
    pd, board, drv, m, rd = hw()
    ref = drv.rf_reference
    assert ref is not None and drv.rf_state["mts"] == (240, 72) and drv.mts_result == 0
    n0 = len(board.events)
    rec = drv.pl_reset()
    ev = board.events[n0:]
    lo = next(i for i, e in enumerate(ev) if e[:2] == ("gpio", "w") and e[3] == 0x0FFF_0000)
    hi = next(i for i, e in enumerate(ev) if e[:2] == ("gpio", "w") and e[3] == 0x0FFF_F000)
    # 0: the preconditions (GPIO reads); 1: the drain, a read of every slave the process wrote, all before the low write
    before = ev[:lo]
    assert before[:2] == [("gpio", "r", DIRM_5, None, False), ("gpio", "r", DATA_5, None, False)]
    drained = [(e[0], e[2]) for e in before if e[1] == "r" and e[0] != "gpio"]
    assert ("soc", m.host_ctrl + m.HOST_DONE) in drained and ("soc", m.ddr_status()) in drained
    assert ("uplink", 0x2C) in drained and ("dma", 0x34) in drained
    assert any(e[0] == "rf" and e[1] == "IPStatus?" for e in before)
    assert not [e for e in before if e[1] == "w"], "a write before the low write"
    # 3: between the low write and the high read-back, only the GPIO page
    assert all(e[0] == "gpio" for e in ev[lo:hi + 2]), ev[lo:hi + 2]
    assert ev[lo + 1] == ("gpio", "r", DATA_5, None, True) and ev[hi + 1] == ("gpio", "r", DATA_5, None, False)
    assert not [e for e in ev if e[0] != "gpio" and e[4]], "a PL access while the resets were low"
    # 6, 7: readiness polled until ready, then the replay; 8: the record
    assert rec["data5_low"] == "0x00000000" and rec["data5_high"] == "0xf0000000" and rec["ready"]["polls"] >= 2
    assert rec["replay"]["mts"]["reapplied"] == {"mts_result": 0, "after": [[240] * 4, [72] * 4]}
    assert rec["replay"]["mts"]["free_run"] == {"dac": [240] * 4, "adc": [64] * 4}
    assert rec["replay"]["nyquist"]["dac"]["3,0"] == 1 and rec["replay"]["nyquist"]["dac"]["0,0"] == 2
    assert set(rec["replay"]["nyquist"]["adc"].values()) == {1} and len(rec["replay"]["nyquist"]["adc"]) == 16
    assert rec["replay"]["vop_uA"]["3,3"] == 7000 and board.rfdc.dac_tiles[3].blocks[3].vop == 7000
    assert rec["replay"]["snapshot"] == ref and rec["ready"]["snapshot"] == ref
    assert drv._pulse is None and drv._pulses == 1 and drv.pl_resets == [rec] and drv._unusable is None
    assert list(rec["ms"]) == ["drain", "pulse", "ready", "replay"]
    drv.read32(m.host_ctrl + m.HOST_DONE)                         # usable again


def test_the_reference_snapshot_is_ready_and_records_both_struct_forms(hw):
    """The phase-10 snapshot is the reference: all 8 tiles enabled at state 15, tile 2's PLLs locked, the
    clock sources of rfdc-config.tcl. xrfdc may return IPStatus with dict or attribute tiles."""
    for form in ("dict", "attr"):
        pd, board, drv, m, rd = hw(ip_form=form)
        snap = drv.rf_reference
        assert [s["clock_source"] for s in snap["dac"]] == [0, 0, 1, 0] == [s["clock_source"] for s in snap["adc"]]
        assert all((s["enabled"], s["state"], s["powerup"], s["pll_lock"]) == (1, 15, 1, 2)
                   for k in ("dac", "adc") for s in snap[k])


def test_ip_status_is_read_into_a_buffer_the_driver_owns(hw, monkeypatch):
    """With xrfdc's cffi handles present, `_ip_status` calls XRFdc_GetIPStatus into its own buffer and copies
    the fields out while it is alive (the property hands back views into a buffer it has released)."""
    pd, board, drv, m, rd = hw()
    calls = []

    class Ffi:
        def new(self, decl):
            assert decl == "XRFdc_IPStatus *"
            return [types.SimpleNamespace()]

    class Lib:
        def XRFdc_GetIPStatus(self, inst, buf):
            calls.append(inst)
            tile = lambda st: types.SimpleNamespace(IsEnabled=1, TileState=st, PowerUpState=1)   # noqa: E731
            buf[0].DACTileStatus = [tile(15)] * 4
            buf[0].ADCTileStatus = [tile(15)] * 3 + [tile(6)]
            return 0
    monkeypatch.setattr(pd.xrfdc, "_ffi", Ffi(), raising=False)
    monkeypatch.setattr(pd.xrfdc, "_lib", Lib(), raising=False)
    monkeypatch.setattr(board.rfdc, "_instance", "inst", raising=False)
    snap = drv.rf_snapshot()
    assert calls == ["inst"] and snap["adc"][3]["state"] == 6 and snap["dac"][0]["state"] == 15
    assert pd._not_ready(snap) == ["adc tile 3: enabled 1, state 6, power-up 1"]


# ── step 0: every refusal before any write ───────────────────────────────────────────────────────

REFUSALS = {
    "unusable": ("the driver is unusable", lambda drv, b: setattr(drv, "_unusable", "test")),
    "cap": ("cap of 6 pulses", lambda drv, b: setattr(drv, "_pulses", 6)),
    "dirm": ("DIRM_5 0x70000000", lambda drv, b: setattr(b, "dirm5", 0x7000_0000)),
    "data": ("DATA_5 0xb0000000", lambda drv, b: setattr(b, "data5", 0xB000_0000)),
    "ddr_not_ready": ("DDR side is not ready", lambda drv, b: setattr(b, "ddr_status", 0xCA1B_0002)),
}


@pytest.mark.parametrize("case", list(REFUSALS))
def test_each_refusal_comes_before_any_write(hw, case):
    pd, board, drv, m, rd = hw()
    why, prep = REFUSALS[case]
    prep(drv, board)
    n0 = len(board.events)
    with pytest.raises(pd.RecoveryUnavailable, match=why):
        drv.pl_reset()
    assert not [e for e in board.events[n0:] if e[1] == "w"], board.events[n0:]
    assert drv._pulse is None and drv._pulses == (6 if case == "cap" else 0) and drv.pl_resets == []


def test_a_caller_other_than_the_bound_pl_thread_is_refused_before_any_write(hw):
    pd, board, drv, m, rd = hw()
    n0, err = len(board.events), []

    def other():
        try:
            drv.pl_reset()
        except pd.RecoveryUnavailable as e:
            err.append(str(e))
    t = threading.Thread(target=other)
    t.start()
    t.join(5)
    assert err and "not the driver's PL thread" in err[0] and board.events[n0:] == []
    pd2, board2, drv2, m2, rd2 = hw(bind=False)                  # never bound: the library default refuses
    with pytest.raises(pd2.RecoveryUnavailable, match="not the driver's PL thread"):
        drv2.pl_reset()
    assert not _gpio_writes(board2)


def test_pl_accesses_from_another_thread_are_refused_on_every_path(hw):
    """`_check` (the SoC window), `DdrBoard._win` (the uplink through `_route`, and the DMA methods, which
    bypass `_route`: Codex BT r2 #2) and the RF ops all refuse a thread other than the bound one."""
    pd, board, drv, m, rd = hw()
    port = rd.drv
    paths = {"read32": lambda: drv.read32(m.host_ctrl + m.HOST_DONE),
             "write_block": lambda: drv.write_block(m.host_ctrl, b"\0" * 4),
             "uplink": lambda: port.read32(UP_BASE + 0x2C),
             "dma_reset": port.dma_reset,
             "dma_idle": port.dma_idle,
             "rf_snapshot": drv.rf_snapshot,
             "dacvop": lambda: drv.dacvop(0, 0, 40000),
             "mts": lambda: drv.mts(240, 72)}
    n0, got = len(board.events), {}

    def other():
        for name, fn in paths.items():
            try:
                fn()
                got[name] = "allowed"
            except RuntimeError as e:
                got[name] = str(e)
    t = threading.Thread(target=other)
    t.start()
    t.join(5)
    assert all("PL thread" in v for v in got.values()), got
    assert board.events[n0:] == []
    for name, fn in paths.items():                                # the bound thread: all allowed
        fn()
    pd2, board2, drv2, m2, rd2 = hw(bind=False)                  # unbound (the board server): any thread
    t = threading.Thread(target=lambda: (drv2.read32(m2.host_ctrl), rd2.drv.dma_idle()))
    t.start()
    t.join(5)
    assert [e for e in board2.events if e[0] == "dma"][-1][:3] == ("dma", "r", 0x34)


# ── step 4: an exception after the low write, or an unconfirmed release: fail-stop ────────────────

FAULTS = {
    "low_readback_raises": {"data5_read_2": "raise"},
    "low_readback_not_low": {"data5_read_2": 0x8000_0000},
    "interrupted_while_low": {"sleep_interrupt": True},
    "high_write_raises": {"high_write": "raise"},
    "high_readback_raises": {"data5_read_3": "raise"},
    "release_not_confirmed": {"stuck_low": True},
}


def _faulted(hw, monkeypatch, case):
    pd, board, drv, m, rd = hw(FAULTS[case])
    if FAULTS[case].get("sleep_interrupt"):
        real, once = pd.time.sleep, []

        def sleep(s):                                             # the low phase's sleep, interrupted once
            if s == pd.PULSE_LOW_S and not once:
                once.append(1)
                raise KeyboardInterrupt
            real(s)
        monkeypatch.setattr(pd.time, "sleep", sleep)
    return pd, board, drv, m, rd


@pytest.mark.parametrize("case", list(FAULTS))
def test_an_exception_after_the_low_write_fails_stop_and_nothing_touches_the_pl_after(hw, monkeypatch, case):
    pd, board, drv, m, rd = _faulted(hw, monkeypatch, case)
    hook = []
    n0 = len(board.events)
    with pytest.raises(pd.PulseUnconfirmed):
        drv.pl_reset(on_unconfirmed=lambda: hook.append(drv._unusable))
    ev = board.events[n0:]
    assert [e[3] for e in _gpio_writes(board, n0)] == [0x0FFF_0000, 0x0FFF_F000], "the high write was not issued"
    assert len(hook) == 1 and "unconfirmed" in hook[0]
    assert drv._unusable and drv._pulse and drv.pl_resets[-1]["unconfirmed"]
    hi = max(i for i, e in enumerate(ev) if e[0] == "gpio")
    assert not [e for e in ev[hi + 1:]], f"accesses after the release: {ev[hi + 1:]}"
    # every later PL access is refused before it reaches the hardware, the S2MM reset included
    n1 = len(board.events)
    for fn in (lambda: drv.read32(m.host_ctrl + m.HOST_DONE), lambda: rd.drv.read32(UP_BASE + 0x2C),
               rd.drv.dma_reset, drv.rf_snapshot, lambda: drv.mts(240, 72), drv.rf_replay):
        with pytest.raises(RuntimeError):
            fn()
    with pytest.raises(pd.RecoveryUnavailable, match="unusable"):
        drv.pl_reset()
    assert board.events[n1:] == []


def test_through_the_run_layer_an_unconfirmed_pulse_poisons_and_recover_refuses(hw, monkeypatch):
    from riscq import run as rq
    from riscq import session as S
    pd, board, drv, m, rd = _faulted(hw, monkeypatch, "release_not_confirmed")
    s = S.session(drv)
    s.request_flush(S.FLUSH_FAILED, "test")
    with pytest.raises(S.SessionPoisoned):
        rq.quiesce(drv, m, rd=rd)
    n1 = len(board.events)
    with pytest.raises(S.SessionPoisoned, match="reload the PL"):
        rq.recover(drv, m)
    assert s.poisoned and board.events[n1:] == []


CHILD = r"""
import json, os, sys
sys.path.insert(0, {sw!r})
from tests.test_bt_pl_reset import Board, FAULTS, install, bring_up
board = Board(FAULTS[{case!r}], log_path={log!r})
pd = install(board, lambda d, k, v: d.__setitem__(k, v))
if FAULTS[{case!r}].get("sleep_interrupt"):
    real = pd.time.sleep
    pd.time.sleep = lambda s: (_ for _ in ()).throw(KeyboardInterrupt) if s == pd.PULSE_LOW_S else real(s)
drv, m, rd = bring_up(pd)
from riscq import run as rq
from riscq import session as S
drv.on_unconfirmed = lambda: os._exit(97)
open({log!r}, "a").write("MARK flush\n")
try:
    rq.hardware_flush(drv, m, "RECOVER", rd=rd)      # the run layer's own path: nothing may follow the hook
finally:
    print("CONTINUED", flush=True)
    drv.__dict__.update(_unusable=None, _pulse=None) # even a cleanup that forced its way would be logged
    rq.reset(drv, m, on=True)
"""


@pytest.mark.parametrize("case", ["low_readback_raises", "interrupted_while_low", "high_write_raises",
                                  "release_not_confirmed"])
def test_the_kit_hook_ends_the_process_before_any_further_pl_access(tmp_path, case):
    """The subprocess harness: the BT kit's hook is os._exit; the process ends with its code, no cleanup,
    diagnostics or core-reset write follows, and the fake logs no PL access after the release."""
    log = tmp_path / "events.jsonl"
    r = subprocess.run([sys.executable, "-c", CHILD.format(sw=str(SW), case=case, log=str(log))],
                       capture_output=True, text=True, timeout=120, cwd=SW)
    assert r.returncode == 97, (r.returncode, r.stdout[-2000:], r.stderr[-2000:])
    assert "CONTINUED" not in r.stdout
    lines = log.read_text().splitlines()
    ev = [json.loads(x) for x in lines[lines.index("MARK flush") + 1:]]
    last_gpio = max(i for i, e in enumerate(ev) if e[0] == "gpio")
    assert ev[last_gpio + 1:] == [], ev[last_gpio + 1:]
    assert [e[3] for e in ev if e[:2] == ["gpio", "w"]] == [0x0FFF_0000, 0x0FFF_F000]


# ── steps 6 and 7: readiness and the replay ──────────────────────────────────────────────────────

# name: (an injection at construction, one made after the bring-up, the error)
READINESS = {
    "never_ready": ({"never_ready": True}, {}, "not ready 0.2 s after the pl_resetn0 pulse: dac tile 0: enabled 1, "
                                                "state 6, power-up 0"),
    "pll_unlocked": ({"pll_unlocked": True}, {}, "dac tile 2: PLL lock status 1 != 2"),
    "clock_source_changed": ({"clock_source_changed": True}, {}, "dac tile 0: clock source 1 != 0"),
    "mts_unreachable": ({"free_after_pulse": [190] * 4}, {}, r"MTS 240/72 is not reachable from the free-running "
                                                             r"latencies \[190, 190, 190, 190\]"),
    "mts_missed_twice": ({}, {"flaky": 2}, "MTS 240/72 missed twice"),
    "zone_does_not_take": ({}, {"zone_ignored": ("dac", 1, 2)}, "the Nyquist zones do not read back as set: "
                                                                "dac 1,2: 1 != 2"),
}


@pytest.mark.parametrize("case", list(READINESS))
def test_readiness_and_replay_failures_raise_and_leave_the_pulse_released(hw, case):
    inject, later, why = READINESS[case]
    pd, board, drv, m, rd = hw(inject)
    board.rfdc.flaky = later.get("flaky", 0)
    board.inject.update({k: v for k, v in later.items() if k != "flaky"})
    with pytest.raises(pd.RfReplayError, match=why):
        drv.pl_reset()
    assert drv._pulse is None and drv._unusable is None and drv._pulses == 1
    assert drv.pl_resets[-1]["data5_high"] == "0xf0000000"


def test_one_mts_retry_is_allowed_after_a_pulse(hw):
    pd, board, drv, m, rd = hw()
    board.rfdc.flaky = 1
    rec = drv.pl_reset()
    assert [a["mts_result"] for a in rec["replay"]["mts"]["attempts"]] == [1, 0]


def test_the_cap_holds_across_successful_pulses(hw):
    pd, board, drv, m, rd = hw()
    for _ in range(pd.PULSE_CAP):
        drv.pl_reset()
    n0 = len(board.events)
    with pytest.raises(pd.RecoveryUnavailable, match="cap of 6"):
        drv.pl_reset()
    assert not [e for e in board.events[n0:] if e[1] == "w"]


# ── S0's flush through PynqDriver ─────────────────────────────────────────────────────────────────

def test_the_run_layers_flush_through_pynq_driver(hw):
    """`hardware_flush`: the core reset, the pulse (the drain first), the time offset written back after it,
    the replay, then the uplink's quiet check; recover() takes the same path plus the S2MM reset. A driver
    with no bound PL thread refuses as before BT (RecoveryUnavailable)."""
    from riscq import run as rq
    from riscq import session as S
    pd, board, drv, m, rd = hw()
    rq.set_time_offset(drv, m, 1 << 28)
    n0 = len(board.events)
    rq.hardware_flush(drv, m, "RECOVER", rd=rd)
    ev = board.events[n0:]
    reset_on = ev.index(("soc", "w", m.host_ctrl + m.HOST_RESET, 1, False))
    lo = ev.index(("gpio", "w", MASK_DATA_5_MSW, 0x0FFF_0000, False))
    lo_off = ev.index(("soc", "w", m.host_ctrl + m.HOST_TIME_OFF_LO, 1 << 28, False))
    hi_off = ev.index(("soc", "w", m.host_ctrl + m.HOST_TIME_OFF_HI, 0, False))
    mts = max(i for i, e in enumerate(ev) if e[0] == "rf" and e[1].startswith("mts_dac 240"))
    quiet = [i for i, e in enumerate(ev) if e[0] == "uplink" and e[2] in (0x58, 0x2C, 0x100, 0x180)]
    assert reset_on < lo < mts < lo_off < hi_off < min(i for i in quiet if i > hi_off)
    s = S.session(drv)
    assert s.pending_flush is None and s.flushes[-1][0] == "RECOVER" and len(drv.pl_resets) == 1
    notes = rq.recover(drv, m)
    assert notes[:2] == ["hardware flush", "S2MM reset"] and len(drv.pl_resets) == 2
    pd2, board2, drv2, m2, rd2 = hw(bind=False)
    with pytest.raises(S.RecoveryUnavailable, match="PL thread"):
        rq.hardware_flush(drv2, m2, "RECOVER", rd=rd2)
    assert not _gpio_writes(board2)


def test_a_flush_after_the_cap_poisons_the_session(hw):
    from riscq import run as rq
    from riscq import session as S
    pd, board, drv, m, rd = hw()
    drv._pulses = pd.PULSE_CAP
    s = S.session(drv)
    s.request_flush(S.FLUSH_FAILED, "test")
    n0 = len(board.events)
    with pytest.raises(S.SessionPoisoned, match="cap of 6"):
        rq.quiesce(drv, m, rd=rd)
    assert s.poisoned and not _gpio_writes(board, n0)
