"""PynqDriver: the 4-method Driver over the ZCU216 AXI window + the RFDC bring-up ops
(spec 10 §3). The ONLY module that imports pynq/xrfclk/xrfdc — it only ever runs on the board;
the server imports it lazily inside load(). The RFDC operations are reproduced from two working
references — QubiC's PLInterface and qcal-riscq's RiscqPlInterface (this gateware's previous
driver, same board) — and stay unverified until M6 hardware bring-up (spec 10 §7)."""

from __future__ import annotations

import asyncio
import logging
import re
import threading
import time
from pathlib import Path

import numpy as np
import pynq
import xrfclk
import xrfdc  # noqa: F401 — registers the RFdc driver so overlay.rf_data_converter binds

from riscq.board.pynq_compat import numpy2_pynq_shim
from riscq.session import PulseUnconfirmed, RecoveryUnavailable, RfReplayError

log = logging.getLogger(__name__)

AXI_BASE = 0x8000_0000   # riscvsoc-bd flow: bd-build.tcl assign_bd_address -offset
AXI_SIZE = 0x1000_0000   # ... -range

# ── qubic3 BT: the pl_resetn0 pulse (plan BT v2 §1 T1) ──
# The PS GPIO controller (LPD, not a PL address). pl_resetn0..3 are EMIO 92-95, bank-5 bits 31:28.
# xilfpga's XFpga_PostConfigPcap pulses them after every full PL load with exactly these two writes to
# MASK_DATA_5_MSW ([31:16] a mask, 1 = keep the bit; [15:0] the data of bank bits 31:16). DATA_5 is the
# output register, the read-back; DATA_5_RO reads the EMIO inputs (0 on veneno) and is no read-back.
GPIO_BASE, GPIO_SIZE = 0xFF0A_0000, 0x1000
GPIO_MASK_DATA_5_MSW, GPIO_DATA_5, GPIO_DIRM_5 = 0x02C, 0x054, 0x344
RESETS_LOW, RESETS_HIGH = 0x0FFF_0000, 0x0FFF_F000
PULSE_CAP = 6                  # pulses per driver (the BT session uses 3 by design)
PULSE_LOW_S = 1e-3             # the resets held low
PULSE_SETTLE_S = 10e-3         # proc_sys_reset releases tens of cycles after its input; rstHold 8 ui cycles
READY_TIMEOUT_S = 5.0          # the RFDC restarts its power-on sequence when its AXI reset is released
READY_POLL_S = 0.02
# rfdc-config.tcl: the PLLs are on DAC2 and ADC2 only (Clock_Dist 2 there); the other tiles take the
# distributed clock. xrfdc ClockSource: 1 = internal PLL, 0 = external (distributed); PLLLockStatus 2 =
# locked (reported for every tile without a PLL by definition). IPStatus' raw PLLState bit is not used.
PLL_TILE, PLL_LOCKED, TILE_READY = 2, 2, 15
CLOCK_SOURCE = (0, 0, 1, 0)

# board.json defaults (spec 10 §4); tile/block keys are "tile,block" strings
BOARD_DEFAULTS = {
    "lmk_freq": 500.25,                             # qcal-riscq's proven value on this board
    "lmx_freq": None,
    "adc_nyquist": 1,
    "dac_nyquist": {"default": 2},
    "dac_current": {},
    # measured on the ZCU216 with this RFDC config: free-run DAC 224 / ADC 64. The target must sit
    # in [measured, measured + 31] — xrfdc only ADDS delay, at most 31 steps (QubiC's 260/60 miss
    # that window on both sides). 240/72 leave headroom both ways.
    "mts": {"daclatency": 240, "adclatency": 72},
}

_refclks_done = False    # LMK/LMX setup runs once per server process, not per load (spec 10 §3.2)


class PynqDriver:
    """MMIO Driver + overlay + RFDC ops. Construction IS bring-up, in the reference order:
    ref clocks -> overlay download -> MMIO -> MTS -> Nyquist zones -> DAC VOP.

    qubic3 BT (plan BT v2 §1 T1): `pl_reset()` is the run layer's hardware-flush pulse of pl_resetn0,
    followed by the RFDC readiness check and the mandatory RF replay. The driver records every RF
    setting it applies (`rf_state`: the MTS target, the Nyquist zones, the VOP currents) and the replay
    re-applies them. The pulse needs one PL-access thread: `bind_pl_thread()` makes the calling thread
    the only one whose PL accesses this driver, and the `DdrBoard` windows over it, allow; without a
    bound thread `pl_reset` refuses (RecoveryUnavailable), as before BT, and the session needs a PL
    reload after a failed run."""

    board_soc_window = True     # read32/write32 take offsets in the SoC AXI window (riscq.ddr.readout_for)

    def __init__(self, xsa: str, params_json: str, board: dict | None = None,
                 download: bool = True):
        # pynq's Xrt-backed EmbeddedDevice does `asyncio.get_event_loop()` when it is first
        # constructed (XrtDevice.__init__, triggered by the Overlay below). The board server
        # runs load() in a Pyro5 worker thread, which on Python 3.10+ has no implicit loop — give
        # it one so the probe doesn't raise "no current event loop in thread ...". We never run
        # the loop (the driver polls, it never waits on interrupts); the probe just needs the
        # call to succeed. No-op on the main thread, which already has a loop.
        try:
            asyncio.get_event_loop()
        except RuntimeError:
            asyncio.set_event_loop(asyncio.new_event_loop())

        # numpy >= 2 / pynq 3.0.0: before ANY pynq allocation in this process (pynq_compat)
        numpy2_pynq_shim()

        # qubic3 BT: the RF state the replay after a pl_resetn0 pulse re-applies, recorded by every RF
        # setter below (the bring-up here included), and the pulse's own state
        self.rf_state = {"mts": None, "dac_nyquist": {}, "adc_nyquist": {}, "dac_current": {}}
        self.rf_reference = None   # the snapshot after the first rf_replay(state): what every replay must restore
        self.pl_resets = []        # one record per pulse
        self.on_unconfirmed = None  # called at once if a pulse's release cannot be confirmed (the BT kit: os._exit)
        self._pl_thread = None     # bind_pl_thread(): the one thread allowed to access the PL
        self._pulse = None         # set while pl_resetn0 is pulsed: every PL access refuses
        self._pulses = 0
        self._gpio = None

        cfg = {**BOARD_DEFAULTS, **(board or {})}
        self.params_text = Path(params_json).read_text()

        global _refclks_done
        if not _refclks_done:
            self.refclks(cfg["lmk_freq"], cfg["lmx_freq"])
            _refclks_done = True
        log.info(f"loading overlay: {xsa}")
        self.overlay = pynq.Overlay(str(xsa), download=download)
        self.rfdc = self.overlay.rf_data_converter
        self.mmio = pynq.MMIO(AXI_BASE, AXI_SIZE)

        # Auto-MTS is opt-out and non-fatal: board.json "mts": null skips it at bring-up (run
        # drv.board.mts() by hand instead; mts_result stays None = "not run"), and a hard MTS
        # miss with it enabled is logged + surfaced via info(), never aborts load().
        if cfg["mts"]:
            try:
                self.mts_result = self.mts(**cfg["mts"])
            except RuntimeError as e:
                log.error(f"auto-MTS failed, continuing (set board.json \"mts\": null to skip): {e}")
                self.mts_result = 1
        else:
            self.mts_result = None
        log.info(f"mts: {self.mts_result}")
        # Nyquist zones are opt-out the same way MTS is: board.json "dac_nyquist"/"adc_nyquist" = null
        # skips them. A no-RF bring-up (the DDR injector self-test, examples/g6_ddr_bringup.py) must not
        # write RF state on a board another project shares.
        zones = cfg["dac_nyquist"]
        if zones is not None:
            for tile in range(4):
                for block in range(4):
                    self.dac_nyquist_zone(tile, block,
                                          zones.get(f"{tile},{block}", zones.get("default", 2)))
        if cfg["adc_nyquist"] is not None:
            self.adc_nyquist_zone(cfg["adc_nyquist"])
        for tileblock, uA in cfg["dac_current"].items():
            tile, block = (int(x) for x in tileblock.split(","))
            self.dacvop(tile, block, uA)

        # ── host-window result buffer (specs/software/22) ──
        # One contiguous CMA buffer, 16 MB per core, allocated ONCE for the session: the PL writes
        # results straight into it over S_AXI_HP0_FPD, so a raw run is no longer bounded by the
        # core's 16 KB RAM and readback is a numpy copy instead of word-at-a-time MMIO.
        # `pynq.allocate` is non-cacheable by default, so a read sees DDR with no cache maintenance.
        # Only a hostwindow build has a HostWindow: an antq_uplink build (results_path) allocates no
        # buffer here -- its results leave through the uplink, whose drain buffer DdrBoard allocates
        # lazily -- and has no `host_base`, so run.setup never programs HOSTWIN registers it lacks.
        from riscq.map import SocMap, SocParams
        self.params = SocParams.from_json(self.params_text)
        self._host_buf = None
        self.host_base = None
        self._unusable = None      # set by a close() that could not stop the uplink's S2MM channel
        if self.params.with_host_window:
            nbytes = SocMap(self.params).hostwin_bytes_total
            _check_cma(nbytes)
            self._host_buf = pynq.allocate(shape=(nbytes,), dtype=np.uint8)
            self.host_base = int(self._host_buf.device_address)
            log.info(f"host window: {nbytes / (1 << 20):.0f} MB at {self.host_base:#x}")
        else:
            log.info(f"results_path={self.params.results_path}: no host-window buffer allocated")

    # ── the Driver protocol over pynq.MMIO (a numpy uint32 view of the /dev/mem mmap) ──

    def bind_pl_thread(self) -> None:
        """qubic3 BT: from now on only the calling thread may access the PL through this driver and
        the `DdrBoard` windows over it (the BT kit calls this in its MMIO child right after the fork)."""
        self._pl_thread = threading.get_ident()

    def _pl_guard(self) -> None:
        """Every PL access through this driver: refused once the driver is unusable, while pl_resetn0
        is pulsed, and from any thread but the bound PL thread (qubic3 BT)."""
        if getattr(self, "_unusable", None):
            raise RuntimeError(f"this driver is unusable: {self._unusable}")
        if getattr(self, "_pulse", None):
            raise RuntimeError(f"PL access refused: {self._pulse}")
        t = getattr(self, "_pl_thread", None)
        if t is not None and threading.get_ident() != t:
            raise RuntimeError("PL access refused: only the driver's PL thread may access the PL "
                               "(bind_pl_thread)")

    def _check(self, addr: int, nbytes: int = 4) -> None:
        self._pl_guard()
        if addr % 4:
            raise ValueError(f"unaligned address {addr:#x}")
        if addr < 0 or addr + nbytes > AXI_SIZE:
            raise ValueError(f"[{addr:#x}, {addr + nbytes:#x}) outside the AXI window "
                             f"(size {AXI_SIZE:#x})")

    def read32(self, addr: int) -> int:
        self._check(addr)
        return self.mmio.read(addr)

    def write32(self, addr: int, value: int) -> None:
        self._check(addr)
        self.mmio.write(addr, int(value) & 0xFFFFFFFF)

    def read_block(self, addr: int, nbytes: int) -> bytes:
        self._check(addr, nbytes)
        # Word-at-a-time single-beat reads (like read32/write_block). A numpy slice read
        # (self.mmio.array[a:b].tobytes()) issues a multi-beat/wide AXI burst that this gateware's
        # BRAM host read port DECERRs -> SIGBUS ("Bus error"); the PL only handles 32-bit beats.
        nwords = -(-nbytes // 4)
        buf = b"".join((self.mmio.read(addr + 4 * i) & 0xFFFFFFFF).to_bytes(4, "little")
                       for i in range(nwords))
        return buf[:nbytes]

    def close(self) -> None:
        """Release the CMA buffers. Called before a reload so the next driver's `pynq.allocate`
        sees the pool free (specs/software/22 §3). On an antq_uplink build that includes the run
        layer's cached uplink readout (qubic3 S0 r1): its `DdrBoard.close()` stops an S2MM transfer
        still in flight before it frees the drain buffer (a fixed buffer stays its owner's). If the
        channel cannot be stopped (r3, r4), the buffer stays in `riscq.board.ddr_board`'s in-flight
        registry, and the driver marks itself unusable and raises: its MMIO, the uplink's windows and
        `attach_readout` refuse from then on, all but the S2MM reset a later close() retries. The
        teardown, the unusable mark and dropping the cache happen under the board lock (r5), which
        `attach_readout` takes too, so a concurrent attach sees either the driver before close() or
        its outcome."""
        from riscq.board.ddr_board import board_lock
        from riscq.ddr import DdrMap
        with board_lock(DdrMap().dma_base):
            rd = getattr(self, "_rq_readout", None)
            port = getattr(rd, "drv", None)
            if port is not None and port is not self and hasattr(port, "close"):
                try:
                    (port._close if hasattr(port, "_close") else port.close)()
                except Exception as e:
                    self._unusable = (f"close() could not stop the uplink's S2MM channel ({type(e).__name__}: "
                                      f"{e}); its drain buffer stays registered as in flight, and the PL needs "
                                      f"a reload or a power cycle")
                    raise RuntimeError(self._unusable) from e
            self._rq_readout = None
        buf, self._host_buf = getattr(self, "_host_buf", None), None
        if buf is not None:
            buf.freebuffer()

    def read_host(self, offset: int, nbytes: int) -> bytes:
        """Read the CMA result buffer at buffer-relative `offset` (specs/software/22 §2.6). Only
        valid after the program's DONE — the window writes are posted (§2.4)."""
        if self._host_buf is None:
            raise ValueError(f"read_host: {self.params.name} is built with results_path="
                             f"{self.params.results_path!r} and has no host-window buffer")
        if not 0 <= offset <= self._host_buf.nbytes - nbytes:
            raise ValueError(f"read_host [{offset}, {offset + nbytes}) outside the "
                             f"{self._host_buf.nbytes} B host buffer")
        view = self._host_buf[offset:offset + nbytes]
        invalidate = getattr(self._host_buf, "invalidate", None)
        if invalidate is not None:
            invalidate()      # no-op on a non-cacheable buffer; correct if one is ever cacheable
        return view.tobytes()

    def write_block(self, addr: int, data: bytes) -> None:
        data = bytes(data)
        if len(data) % 4:
            raise ValueError(f"write_block length {len(data)} is not a multiple of 4 "
                             "(pynq MMIO word-loops the payload)")
        self._check(addr, len(data))
        self.mmio.write(addr, data)

    # ── RFDC ops, reproduced verbatim from the references (spec 10 §3.3 / §7) ──

    def refclks(self, lmk_freq: float, lmx_freq: float | None = None) -> None:
        if lmx_freq is None:
            xrfclk.set_ref_clks(lmk_freq=lmk_freq)
        else:
            xrfclk.set_ref_clks(lmk_freq=lmk_freq, lmx_freq=lmx_freq)
        log.info(f"ref clocks: lmk={lmk_freq} lmx={lmx_freq}")

    def config_mts(self, daclatency: int = -1, adclatency: int = -1) -> None:
        self._pl_guard()
        for mts_cfg in (self.rfdc.mts_dac_config, self.rfdc.mts_adc_config):
            mts_cfg.RefTile = 2      # the references' choice — see xrfdc MTS restrictions
            mts_cfg.Tiles = 0xF      # bitmask: sync all 4 tiles
            mts_cfg.SysRef_Enable = 1
        self.rfdc.mts_dac_config.Target_Latency = daclatency
        self.rfdc.mts_adc_config.Target_Latency = adclatency

    def mts(self, daclatency: int = 240, adclatency: int = 72) -> int:
        """Two-pass MTS (QubiC): free sync to measure the latencies; if all tiles agree and are
        within target, re-sync pinned to the targets. 0 iff every measured latency == target.
        Raises (XRFdc_MultiConverter_Sync) if a tile never reaches the started state — run it by
        hand via drv.board.mts() to see the full converter error; __init__ guards the auto call
        so an MTS miss never aborts bring-up (spec 10 §7: MTS is re-pinned at bring-up). qubic3 BT:
        the target is recorded for the replay after a pl_resetn0 pulse."""
        self._pl_guard()
        self.rf_state["mts"] = (int(daclatency), int(adclatency))
        self.config_mts()
        self.rfdc.mts_dac()
        self.rfdc.mts_adc()
        dac_lat, adc_lat = self._mts_latencies()
        log.debug(f"mts before: dac={dac_lat} adc={adc_lat}")
        if (all(l == dac_lat[0] for l in dac_lat) and dac_lat[0] <= daclatency
                and all(l == adc_lat[0] for l in adc_lat) and adc_lat[0] <= adclatency):
            self.config_mts(daclatency, adclatency)
            self.rfdc.mts_dac()
            self.rfdc.mts_adc()
            dac_lat, adc_lat = self._mts_latencies()
            log.debug(f"mts after: dac={dac_lat} adc={adc_lat}")
        return 0 if (all(l == daclatency for l in dac_lat)
                     and all(l == adclatency for l in adc_lat)) else 1

    def _mts_latencies(self) -> tuple[list, list]:
        return ([self.rfdc.mts_dac_config.Latency[i] for i in range(4)],
                [self.rfdc.mts_adc_config.Latency[i] for i in range(4)])

    def adc_nyquist_zone(self, n: int) -> None:
        self._pl_guard()
        for tile in range(4):        # rfdc-config.tcl enables every slice: full 4x4
            for block in range(4):
                self.rf_state["adc_nyquist"][f"{tile},{block}"] = int(n)
                self.rfdc.adc_tiles[tile].blocks[block].NyquistZone = n

    def dac_nyquist_zone(self, tile: int, block: int, n: int) -> None:
        self._pl_guard()
        self.rf_state["dac_nyquist"][f"{tile},{block}"] = int(n)
        self.rfdc.dac_tiles[tile].blocks[block].NyquistZone = n

    def dacvop(self, tile: int, block: int, uA: int) -> None:
        self._pl_guard()
        log.info(f"dac vop: tile {tile} block {block} -> {uA} uA")
        self.rf_state["dac_current"][f"{tile},{block}"] = int(uA)
        self.rfdc.dac_tiles[tile].blocks[block].SetDACVOP(uA)

    # ── qubic3 BT: the RF replay, the readiness snapshot and the pl_resetn0 pulse (plan BT v2 §1 T1) ──

    def rf_replay(self, state: dict | None = None) -> dict:
        """(Re-)apply the recorded RF state under G6' session 2's rules: the one code path of the first
        RF bring-up (`state` given, in board.json form: it replaces the recorded state, and the snapshot
        after it becomes `rf_reference`) and of the replay after every pl_resetn0 pulse.
          MTS, if a target is recorded: a free-running sync; the target reachable on every tile (equal
          free-run latencies, the target within +31 of them, as xrfdc only adds delay); at most one retry;
          mts_result 0 with every tile exactly at the target; then re-applied and checked again.
          Every recorded Nyquist zone, read back; every recorded VOP current.
          Then the readiness snapshot, equal to `rf_reference` once one exists.
        Any miss raises RfReplayError. Returns the record."""
        if state is not None:
            mts = state.get("mts")
            self.rf_state = {"mts": None if mts is None else (int(mts[0]), int(mts[1])),
                             "dac_nyquist": _zones(state.get("dac_nyquist")),
                             "adc_nyquist": _zones(state.get("adc_nyquist")),
                             "dac_current": {_tb(k): int(v) for k, v in (state.get("dac_current") or {}).items()}}
        rs = self.rf_state
        out = {"mts": None if rs["mts"] is None else self._mts_strict(*rs["mts"])}
        for tb, z in sorted(rs["dac_nyquist"].items()):
            t, b = (int(x) for x in tb.split(","))
            self.dac_nyquist_zone(t, b, z)
        for tb, z in sorted(rs["adc_nyquist"].items()):
            t, b = (int(x) for x in tb.split(","))
            self._pl_guard()
            self.rfdc.adc_tiles[t].blocks[b].NyquistZone = z
        got = {"dac": {}, "adc": {}}
        for kind, tiles in (("dac", self.rfdc.dac_tiles), ("adc", self.rfdc.adc_tiles)):
            for tb in sorted(rs[f"{kind}_nyquist"]):
                t, b = (int(x) for x in tb.split(","))
                self._pl_guard()
                got[kind][tb] = int(tiles[t].blocks[b].NyquistZone)
        bad = [f"{k} {tb}: {got[k][tb]} != {rs[f'{k}_nyquist'][tb]}" for k in got for tb in got[k]
               if got[k][tb] != rs[f"{k}_nyquist"][tb]]
        if bad:
            raise RfReplayError(f"the Nyquist zones do not read back as set: {'; '.join(bad)}")
        out["nyquist"] = got
        for tb, ua in sorted(rs["dac_current"].items()):
            t, b = (int(x) for x in tb.split(","))
            self.dacvop(t, b, ua)
        out["vop_uA"] = dict(rs["dac_current"])
        snap = self.rf_snapshot()
        out["snapshot"] = snap
        bad = _not_ready(snap)
        if bad:
            raise RfReplayError(f"the RF data converter is not ready after the RF replay: {'; '.join(bad)}")
        if state is not None and self.rf_reference is None:
            self.rf_reference = snap
        elif self.rf_reference is not None and snap != self.rf_reference:
            raise RfReplayError(f"the RFDC snapshot after the replay differs from the reference: {snap} "
                                f"!= {self.rf_reference}")
        return out

    def _mts_strict(self, d: int, a: int) -> dict:
        """MTS under G6' session 2's rules (`rf_replay`); raises RfReplayError on any miss."""
        out = {"target": f"{d}/{a}"}
        self.config_mts()                                    # the free-running sync, QubiC's first pass
        self.rfdc.mts_dac()
        self.rfdc.mts_adc()
        dac, adc = self._mts_latencies()
        out["free_run"] = {"dac": list(dac), "adc": list(adc)}
        reach = (all(x == dac[0] for x in dac) and all(x == adc[0] for x in adc)
                 and all(x <= d <= x + 31 for x in dac) and all(x <= a <= x + 31 for x in adc))
        if not reach:
            raise RfReplayError(f"MTS {d}/{a} is not reachable from the free-running latencies {dac}/{adc}")
        out["attempts"] = []
        for _ in range(2):                                   # at most one retry
            try:
                r, err = self.mts(d, a), None
            except RuntimeError as e:
                r, err = 1, str(e)[:200]
            dac, adc = self._mts_latencies()
            out["attempts"].append({"mts_result": r, "after": [list(dac), list(adc)], "error": err})
            if r == 0 and all(x == d for x in dac) and all(x == a for x in adc):
                break
        else:
            raise RfReplayError(f"MTS {d}/{a} missed twice: {out['attempts']}")
        r = self.mts(d, a)                                   # re-applied and checked
        dac, adc = self._mts_latencies()
        out["reapplied"] = {"mts_result": r, "after": [list(dac), list(adc)]}
        if r != 0 or any(x != d for x in dac) or any(x != a for x in adc):
            raise RfReplayError(f"re-applying MTS {d}/{a} gave mts_result {r}, latencies {dac}/{adc}")
        self.mts_result = 0
        return out

    def _ip_status(self):
        """`XRFdc_GetIPStatus` into a buffer this function owns, read while it is alive: xrfdc's own
        `IPStatus` property returns the struct's tile arrays as views into a cffi buffer it has already
        released. Falls back to the property where xrfdc exposes no `_ffi`/`_lib`/`_instance`."""
        ffi, lib = getattr(xrfdc, "_ffi", None), getattr(xrfdc, "_lib", None)
        inst = getattr(self.rfdc, "_instance", None)
        if ffi is None or lib is None or inst is None:
            return self.rfdc.IPStatus
        buf = ffi.new("XRFdc_IPStatus *")
        rc = lib.XRFdc_GetIPStatus(inst, buf)
        if rc:
            raise RfReplayError(f"XRFdc_GetIPStatus returned {rc}")
        s = buf[0]
        return {key: [{f: int(getattr(getattr(s, key)[t], f)) for f in ("IsEnabled", "TileState", "PowerUpState")}
                      for t in range(4)] for key in ("DACTileStatus", "ADCTileStatus")}

    def rf_snapshot(self) -> dict:
        """The RFDC's readiness facts per tile: enabled, tile state and power-up state (IPStatus), the
        tile's clock source and its PLL lock status (xrfdc 2.0 exposes both on each tile)."""
        self._pl_guard()
        ip = self._ip_status()
        out = {}
        for kind, key, tiles in (("dac", "DACTileStatus", self.rfdc.dac_tiles),
                                 ("adc", "ADCTileStatus", self.rfdc.adc_tiles)):
            st = _field(ip, key)
            out[kind] = [{"enabled": int(_field(st[t], "IsEnabled")), "state": int(_field(st[t], "TileState")),
                          "powerup": int(_field(st[t], "PowerUpState")),
                          "clock_source": int(tiles[t].ClockSource), "pll_lock": int(tiles[t].PLLLockStatus)}
                         for t in range(4)]
        return out

    def _gpio_win(self):
        if self._gpio is None:
            self._gpio = pynq.MMIO(GPIO_BASE, GPIO_SIZE)
        return self._gpio

    def _drain(self) -> dict:
        """Step 1 of `pl_reset`: one read from every PL slave this process may have written. Device
        accesses to one peripheral stay in program order, so each read returns only after the earlier
        writes to its slave: the SoC host port (HOST_DONE), the uplink's control slave (STATUS) and the
        DMA's lite port (DMASR) if their `DdrBoard` windows were ever mapped (after the host-domain DDR
        status shows the DDR side ready: its slave answers only then), and the RFDC (the snapshot)."""
        from riscq.ddr_regs import STATUS
        from riscq.map import SocMap
        m = SocMap(self.params)
        out = {"host_done": f"0x{self.read32(m.host_ctrl + m.HOST_DONE):08x}"}
        port = getattr(getattr(self, "_rq_readout", None), "drv", None)
        wins = getattr(port, "_mmio", None) if port is not self else None
        if wins:
            dm = port.map
            w = self.read32(m.ddr_status())
            out["ddr_status"] = f"0x{w:08x}"
            if (w >> 16) != m.DDR_STATUS_MAGIC or (w & 3) != 3:
                raise RecoveryUnavailable(f"pl_reset refused: the uplink's DDR side is not ready (DDR status "
                                          f"0x{w:08x}), so its control slave cannot be drained: reload the PL")
            if (dm.ctrl_base, dm.ctrl_size) in wins:
                out["uplink_status"] = f"0x{port.read32(dm.ctrl_base + STATUS):08x}"
            if (dm.dma_base, dm.dma_size) in wins:
                out["dmasr"] = f"0x{int(port._dma_win().read(port.S2MM_DMASR)):08x}"
        out["rf"] = self.rf_snapshot()
        return out

    def pl_reset(self, on_unconfirmed=None) -> dict:
        """The hardware flush's pulse of pl_resetn0 (`riscq.run.hardware_flush`, cores held), from the
        bound PL thread. pl_resetn0 resets the host and DSP domains and the whole RFDC through its AXI
        reset; the MIG, the AXI DMA and the SmartConnects are not on it.
          0. refusals before any write (RecoveryUnavailable): the driver unusable; the cap of PULSE_CAP
             pulses; a caller other than the bound PL thread; DIRM_5[31:28] or DATA_5[31:28] not 0xF (the
             state xilfpga leaves); the uplink's DDR side not ready (`_drain`);
          1. drain (`_drain`); 2. block: `_pulse` set, every PL access through this driver and its
             DdrBoard windows refuses;
          3. pulse: the low write, read back 0; 1 ms; in a `finally` the high write, read back 0xF;
          4. unconfirmed (anything raised after the low write, or the last read-back not 0xF): fail-stop.
             The driver is marked unusable and `_pulse` stays set; `on_unconfirmed` (else
             `self.on_unconfirmed`; the BT kit passes os._exit) is called at once; if it returns, or there
             is none, PulseUnconfirmed is raised. No PL access can follow; a PL reload recovers;
          5. settle 10 ms, clear `_pulse`;
          6. readiness within READY_TIMEOUT_S: all 8 tiles enabled, state 15, powered up; DAC2's and
             ADC2's PLLs locked; every tile's clock source as rfdc-config.tcl distributes it;
          7. the RF replay (`rf_replay`), mandatory;
          8. the record (the drain reads, the read-backs, the step times, `t_high` = time.monotonic() at the
             high read-back, the readiness snapshot, the replay's latencies and zones), appended to
             `pl_resets` and returned.
        Readiness and replay failures raise RfReplayError: the run layer POISONs, a reload recovers."""
        t0 = time.monotonic()
        why = getattr(self, "_unusable", None)
        if why:
            raise RecoveryUnavailable(f"pl_reset refused: the driver is unusable ({why})")
        if self._pulses >= PULSE_CAP:
            raise RecoveryUnavailable(f"pl_reset refused: the cap of {PULSE_CAP} pulses per driver is reached")
        if self._pl_thread is None or threading.get_ident() != self._pl_thread:
            raise RecoveryUnavailable("pl_reset refused: the caller is not the driver's PL thread "
                                      "(bind_pl_thread() in the one thread that accesses the PL)")
        g = self._gpio_win()
        dirm, data = int(g.read(GPIO_DIRM_5)), int(g.read(GPIO_DATA_5))
        if (dirm >> 28) & 0xF != 0xF or (data >> 28) & 0xF != 0xF:
            raise RecoveryUnavailable(f"pl_reset refused: GPIO bank 5 is not as xilfpga leaves it (DIRM_5 "
                                      f"0x{dirm:08x}, DATA_5 0x{data:08x}: bits 31:28 must be 0xF in both)")
        rec = {"n": self._pulses + 1, "dirm5": f"0x{dirm:08x}", "data5_before": f"0x{data:08x}", "ms": {}}

        def mark(step):
            rec["ms"][step] = round((time.monotonic() - t0) * 1e3, 3)
        rec["drain"] = self._drain()
        mark("drain")
        self.pl_resets.append(rec)
        self._pulse = "pl_resetn0 is being pulsed"
        err = high = None
        try:
            try:
                self._pulses += 1                            # counted as issued before the write
                g.write(GPIO_MASK_DATA_5_MSW, RESETS_LOW)
                low = int(g.read(GPIO_DATA_5))
                rec["data5_low"] = f"0x{low:08x}"
                if (low >> 28) & 0xF:
                    raise PulseUnconfirmed(f"DATA_5 reads 0x{low:08x} after the low write")
                time.sleep(PULSE_LOW_S)
            finally:
                g.write(GPIO_MASK_DATA_5_MSW, RESETS_HIGH)
                high = int(g.read(GPIO_DATA_5))
                rec["data5_high"], rec["t_high"] = f"0x{high:08x}", time.monotonic()
        except BaseException as e:                           # noqa: BLE001 -- after the low write: fail-stop
            err = e
        if err is not None or high is None or (high >> 28) & 0xF != 0xF:
            rec["unconfirmed"] = (f"{type(err).__name__}: {err}" if err is not None
                                  else f"DATA_5 reads 0x{high:08x} after the high write")
            self._unusable = f"the pl_resetn0 release is unconfirmed ({rec['unconfirmed']}): reload the PL"
            hook = on_unconfirmed or self.on_unconfirmed
            if hook is not None:
                hook()                                       # the BT kit: os._exit, nothing below runs
            raise PulseUnconfirmed(self._unusable) from err
        mark("pulse")
        time.sleep(PULSE_SETTLE_S)
        self._pulse = None
        t, polls = time.monotonic(), 0
        while True:
            snap = self.rf_snapshot()
            polls += 1
            bad = _not_ready(snap)
            if not bad:
                break
            if time.monotonic() - t >= READY_TIMEOUT_S:
                raise RfReplayError(f"the RF data converter is not ready {READY_TIMEOUT_S} s after the pl_resetn0 "
                                    f"pulse: {'; '.join(bad)}")
            time.sleep(READY_POLL_S)
        rec["ready"] = {"polls": polls, "snapshot": snap}
        mark("ready")
        rec["replay"] = self.rf_replay()
        mark("replay")
        log.info(f"pl_resetn0 pulse {rec['n']}: {rec['ms']} ms")
        return rec


def _field(x, name):
    """A field of an xrfdc struct: a dict (pynq's property conversion) or an attribute (a cffi struct)."""
    return x[name] if isinstance(x, dict) else getattr(x, name)


def _tb(key) -> str:
    """A board.json "tile,block" key, spaces removed ("3, 0" -> "3,0")."""
    return str(key).replace(" ", "")


def _zones(z) -> dict:
    """board.json Nyquist zones -> {"tile,block": zone} over all 16 blocks: an int for every block, or a
    dict whose "default" covers the blocks it does not name; None -> nothing recorded."""
    if z is None:
        return {}
    if isinstance(z, int):
        return {f"{t},{b}": int(z) for t in range(4) for b in range(4)}
    d = {_tb(k): v for k, v in z.items()}
    default = d.pop("default", None)
    if default is None:
        return {k: int(v) for k, v in d.items()}
    return {f"{t},{b}": int(d.get(f"{t},{b}", default)) for t in range(4) for b in range(4)}


def _not_ready(snap: dict) -> list:
    """What keeps a `rf_snapshot` from the ready state (`pl_reset` step 6); [] when ready."""
    bad = []
    for kind in ("dac", "adc"):
        for t, s in enumerate(snap[kind]):
            if (s["enabled"], s["state"], s["powerup"]) != (1, TILE_READY, 1):
                bad.append(f"{kind} tile {t}: enabled {s['enabled']}, state {s['state']}, power-up {s['powerup']}")
            if s["clock_source"] != CLOCK_SOURCE[t]:
                bad.append(f"{kind} tile {t}: clock source {s['clock_source']} != {CLOCK_SOURCE[t]}")
        if snap[kind][PLL_TILE]["pll_lock"] != PLL_LOCKED:
            bad.append(f"{kind} tile {PLL_TILE}: PLL lock status {snap[kind][PLL_TILE]['pll_lock']} != "
                       f"{PLL_LOCKED} (locked)")
    return bad


def _check_cma(nbytes: int) -> None:
    """Fail loudly and early if the kernel's CMA pool cannot hold the result buffer — otherwise
    `pynq.allocate` dies deep in the driver with an opaque error (specs/software/22 §3). Fix by
    rebuilding with a narrower window (`"hostwin_bits": 23` in the build JSON) or rebuilding the
    image with a larger `cma=` boot arg."""
    try:
        info = Path("/proc/meminfo").read_text()
    except OSError:
        return                     # not Linux/procfs — let allocate() speak for itself
    fields = {k: int(v) * 1024 for k, v in re.findall(r"^(Cma\w+):\s+(\d+) kB", info, re.M)}
    total, free = fields.get("CmaTotal"), fields.get("CmaFree")
    if total is None:
        return
    if total < nbytes or (free is not None and free < nbytes):
        raise RuntimeError(
            f"host-window buffer needs {nbytes / (1 << 20):.0f} MB of CMA, but CmaTotal="
            f"{total / (1 << 20):.0f} MB / CmaFree={(free or 0) / (1 << 20):.0f} MB. Rebuild with a "
            f"smaller `hostwin_bits` in the build JSON, boot with a larger `cma=`, or restart the "
            f"server to free a leaked buffer.")
