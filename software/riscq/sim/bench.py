"""The cocotb co-sim bench: clocks/reset on PulseTableSoc, a hand-rolled single-beat AXI4
master BFM, and a Pyro5 daemon (in a python thread) whose requests a cocotb coroutine services
as AXI transactions. Between requests the sim free-runs the clock in bounded ticks, so the
cores keep executing while the host thinks.

Runs inside the verilator sim process (MODULE=riscq.sim.bench, driven by riscq.sim.server)."""

from __future__ import annotations

import json
import os
import queue
import threading
from collections import deque
from pathlib import Path

import cocotb
import numpy as np
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer
from cocotb.utils import get_sim_time

import Pyro5.api
import serpent

from riscq.map import ADC_BATCH, BATCH_SIZE, DIO_PIPE, MEM_BASE, SocMap, SocParams
from riscq.sim import models

CLK_PERIOD_NS = 10        # both clk and dspClk (period equality is fine in sim)
IDLE_TICK = 200           # cycles free-run per idle service-loop pass
POLL_INTERVAL = 200       # cycles between re-reads inside poll_word
AXI_TIMEOUT = 100_000     # cycles before an AXI handshake is declared dead (loud, not hung)
DAC_GET_TIMEOUT = 1_000_000  # cycles dac_capture_get waits for an armed capture to finish

# Modelled physical base of the PS DDR4 result buffer (specs/software/22). On hardware this is what
# `pynq.allocate` handed back; here it is any plausible constant — the funnel adds it to
# `(core << 24) + offset`, and read_host is buffer-relative, so the value only has to round-trip.
HOST_BASE = 0x70000000

# dspClk cycles from refTime's free-running origin (the dspRst-release cycle, captured once at sim
# start via TimeMirror.set_origin) to the cycle whose io_dac_* sample carries batch time 0 for a
# single-channel DAC (dac_pipe = 1). refTime lives in dspCd and is NOT reset by riscqReset (spec 08),
# so batch time is monotonic across runs and there is one fixed anchor for the whole session — not a
# per-run riscqReset-release anchor. Empirical for this bench's fixed stimulus (deterministic under
# Verilator: both clocks 10 ns, in phase); every M1 pulse test asserts window position against it, so
# drift fails loudly. Calibrated: a program's pulse at startTime = t lands at capture stamps [t, t+dur).
SIMSTART_TO_TIME0 = 2


class AxiMaster:
    """Single-beat AXI4 master (len=0, size=2, INCR, strb=0xF); AW/W handshakes independent,
    B/R awaited. Proven mechanics from the old cosim driver. `prefix`/`clk` select the port: the SoC's
    host slave `io_axi` on `clk` (default), or an antq_uplink build's `s_axi_ddr_ctrl` on `ddrClk`."""

    def __init__(self, dut, prefix: str = "io_axi", clk=None, aw_w_together: bool = False):
        self.dut = dut
        self.p = prefix
        self.clk = dut.clk if clk is None else clk
        # SpinalHDL's Axi4SlaveFactory (the uplink's control slave) takes AW only together with W, so
        # that master raises both VALIDs at once; the host path keeps its proven AW-then-W order.
        self.aw_w_together = aw_w_together

    def _s(self, name: str):
        return getattr(self.dut, f"{self.p}_{name}")

    async def _await_ready(self, valid, ready, what: str):
        valid.value = 1
        for _ in range(AXI_TIMEOUT):
            await RisingEdge(self.clk)
            if ready.value == 1:
                valid.value = 0
                return
        raise RuntimeError(f"AXI {what} handshake timeout after {AXI_TIMEOUT} cycles")

    async def _await_both(self):
        """AW and W VALID together; each drops on its own handshake (they may complete apart)."""
        aw_v, aw_r, w_v, w_r = (self._s("aw_valid"), self._s("aw_ready"), self._s("w_valid"), self._s("w_ready"))
        aw_v.value = 1
        w_v.value = 1
        aw_done = w_done = False
        for _ in range(AXI_TIMEOUT):
            await RisingEdge(self.clk)
            if not aw_done and aw_r.value == 1:
                aw_done = True
                aw_v.value = 0
            if not w_done and w_r.value == 1:
                w_done = True
                w_v.value = 0
            if aw_done and w_done:
                return
        raise RuntimeError(f"AXI AW/W handshake timeout after {AXI_TIMEOUT} cycles "
                           f"(aw done {aw_done}, w done {w_done})")

    def _addr_phase(self, prefix: str, addr: int):
        width = len(self._s(f"{prefix}_payload_addr"))
        self._s(f"{prefix}_payload_addr").value = addr & ((1 << width) - 1)
        for field, value in (("id", 0), ("region", 0), ("len", 0), ("size", 2),
                             ("burst", 1), ("lock", 0), ("cache", 0), ("qos", 0), ("prot", 0)):
            self._s(f"{prefix}_payload_{field}").value = value

    async def write_word(self, addr: int, data: int) -> None:
        self._addr_phase("aw", addr)
        self._s("w_payload_data").value = data & 0xFFFFFFFF
        self._s("w_payload_strb").value = 0xF
        self._s("w_payload_last").value = 1
        self._s("b_ready").value = 1
        if self.aw_w_together:
            await self._await_both()
        else:
            await self._await_ready(self._s("aw_valid"), self._s("aw_ready"), "AW")
            await self._await_ready(self._s("w_valid"), self._s("w_ready"), "W")
        for _ in range(AXI_TIMEOUT):
            if self._s("b_valid").value == 1:
                break
            await RisingEdge(self.clk)
        else:
            raise RuntimeError(f"AXI B response timeout at addr {addr:#x}")
        self._s("b_ready").value = 0

    async def read_word(self, addr: int) -> int:
        self._addr_phase("ar", addr)
        self._s("r_ready").value = 1
        await self._await_ready(self._s("ar_valid"), self._s("ar_ready"), "AR")
        for _ in range(AXI_TIMEOUT):
            if self._s("r_valid").value == 1:
                break
            await RisingEdge(self.clk)
        else:
            raise RuntimeError(f"AXI R response timeout at addr {addr:#x}")
        data = int(self._s("r_payload_data").value) & 0xFFFFFFFF
        self._s("r_ready").value = 0
        return data


def _cycle() -> int:
    """Current dspClk cycle index (both clocks are fixed 10 ns from t=0)."""
    return int(get_sim_time(units="ps")) // (CLK_PERIOD_NS * 1000)


class TimeMirror:
    """Bench-side mirror of the SoC batch time. refTime is a free-running dspCd counter (spec 08: NOT
    reset by riscqReset), so batch time is MONOTONIC across runs — one fixed anchor for the whole
    session, set at refTime's dspRst-release origin (set_origin), plus the host-written timeOffset.
    Updated by watching the AXI writes the bench performs. Convention: time_of_cycle(c) is the batch
    time whose pulse-generator output a dac_pipe=1 DAC port carries in cycle c (set timeOffset only
    while riscqReset is asserted)."""

    def __init__(self, m: SocMap):
        self._reset_addr = m.host_ctrl + m.HOST_RESET
        self._lo_addr = m.host_ctrl + m.HOST_TIME_OFF_LO
        self._hi_addr = m.host_ctrl + m.HOST_TIME_OFF_HI
        self.origin_cycle: int | None = None   # refTime free-running origin (dspRst release), fixed once
        self.release_cycle: int | None = None  # latest riscqReset release — gates ADC injection only
        self._off_lo = 0
        self._off_hi = 0

    def set_origin(self, cycle: int) -> None:
        """Pin the batch-time anchor to refTime's origin (dspRst release): at sim start, and again
        after a `pl_reset` pulse (qubic3 S0), which restarts refTime."""
        self.origin_cycle = cycle

    def host_reset(self) -> None:
        """The host-domain reset zeroes the timeOffset registers (init 0)."""
        self._off_lo = self._off_hi = 0

    def on_write(self, addr: int, data: int) -> None:
        if addr == self._reset_addr and data == 0:
            self.release_cycle = _cycle()
        elif addr == self._lo_addr:
            self._off_lo = data
        elif addr == self._hi_addr:
            self._off_hi = data

    @property
    def offset(self) -> int:
        return ((self._off_hi << 32) | self._off_lo) & 0xFFFFFFFF   # 32-bit batch time

    def time_of_cycle(self, cycle: int) -> int:
        if self.origin_cycle is None:
            raise RuntimeError("refTime origin not set (dspRst not released) — batch time undefined")
        return cycle - self.origin_cycle - SIMSTART_TO_TIME0 + self.offset

    def cycle_of_time(self, t: int) -> int:
        if self.origin_cycle is None:
            raise RuntimeError("refTime origin not set (dspRst not released) — batch time undefined")
        return t - self.offset + self.origin_cycle + SIMSTART_TO_TIME0


class DacCapture:
    """One armed capture: samples a port on consecutive dspClk cycles — `io_dac_<id>_payload` for a
    DAC (`pipe` = the DAC's output pipe, modelled out of the stamps) or `io_dio_<name>_out` for a
    timed-DIO bank (`pipe` = DIO_PIPE)."""

    def __init__(self, dac_id: int, n_batches: int, start_batch: int | None,
                 sig: str | None = None, pipe: int | None = None):
        self.dac_id = dac_id
        self.sig = sig if sig is not None else f"io_dac_{dac_id}_payload"
        self.pipe = pipe
        self.n_batches = n_batches
        self.start_batch = start_batch
        self.first_cycle: int | None = None
        self.vals: list[int] = []
        self.origin_cycle: int | None = None
        self.offset = 0
        self.done = False
        self.error: str | None = None


async def _capture_run(dut, m: SocMap, mirror: TimeMirror, cap: DacCapture) -> None:
    """Sample the DAC payload each dspClk falling edge (registered outputs settled), starting
    now (arm) or at the cycle stamped `start_batch`. The batch stamp of sample j resolves to
    time_of_cycle(first_cycle + j) - dac_pipe(dac_id) at get time, so a pulse played at t
    occupies stamps [t, t+dur) on EVERY DAC (the summed-DAC extra RegNext is modeled out)."""
    try:
        sig = getattr(dut, cap.sig)
        pipe = m.dac_pipe(cap.dac_id) if cap.pipe is None else cap.pipe
        clk = dut.dspClk
        await FallingEdge(clk)
        if cap.start_batch is not None:
            target = mirror.cycle_of_time(cap.start_batch) + pipe
            delta = target - _cycle()
            if delta < 0:
                raise RuntimeError(f"start_batch {cap.start_batch} is {-delta} cycles in the past")
            if delta > 0:
                await ClockCycles(clk, delta)
                await FallingEdge(clk)
        cap.first_cycle = _cycle()
        for _ in range(cap.n_batches):
            cap.vals.append(int(sig.value))
            await FallingEdge(clk)
        # snapshot the mirror NOW (origin is fixed for the session; offset may change between runs),
        # so a later timeOffset write cannot skew the stamps of an already-finished capture.
        cap.origin_cycle = mirror.origin_cycle
        cap.offset = mirror.offset
    except Exception as exc:
        cap.error = f"{type(exc).__name__}: {exc}"
    finally:
        cap.done = True


class DacWatch:
    """qubic3 P6: a test observation of whole DAC outputs over a run of unknown length. Per DAC: the
    batches watched, the peak |sample|, the first and last batch stamp with a nonzero sample, and
    the stamp where the last nonzero stretch began (the start of the last pulse). qubic3 P4 adds the
    timed-DIO outputs `dios` (board port names, active while the level is nonzero, DIO_PIPE modelled
    out) and, per output, every nonzero stretch [first, last] (up to STRETCHES), so one watch can
    follow several runs and the gaps between them. qubic3 BT (C-D) adds the completion monitors: with
    `marks` = {core: [CPU addresses]}, every store of that core's CPU to one of the words (the cycle it
    is on port 0 of the core's RAM, the batch time then, the address, the data), and every change of
    the host-domain DONE bits (what HOST_DONE reads) and of the cores' reset, each as [cycle, batch
    time, value], so one watch orders each core's last output activity, marker write and DONE rise."""

    STRETCHES = 4096

    def __init__(self, dac_ids, dios=(), marks=None):
        self.dac_ids = list(dac_ids)
        self.dios = list(dios)
        self.stats = {d: {"batches": 0, "peak": 0, "first": None, "last": None, "last_rise": None,
                          "stretches": []}
                      for d in self.dac_ids + [f"dio:{n}" for n in self.dios]}
        self.marks = {int(c): sorted({(int(a) - MEM_BASE) >> 2 for a in addrs}) for c, addrs in (marks or {}).items()}
        self.events = {"marks": {c: [] for c in self.marks}, "done": [], "reset": []}
        self.stop = False
        self.done = False


def _core_ram(dut, core: int):
    """Core `core`'s unified RAM (`RiscvSoc.mem`); its port 0 carries the CPU's data stores."""
    return getattr(getattr(dut, f"riscqArea_riscqCores_{core}_riscvSoc"), "mem")


async def _watch_run(dut, st: "_BenchState", w: DacWatch) -> None:
    keys = w.dac_ids + [f"dio:{n}" for n in w.dios]
    prev = {d: False for d in keys}
    pipes = {d: DIO_PIPE if isinstance(d, str) else st.m.dac_pipe(d) for d in keys}
    ram = {c: _core_ram(dut, c) for c in w.marks}
    words = {c: set(v) for c, v in w.marks.items()}
    mon = [(dut.doneHostCd, w.events["done"]), (dut.riscqReset, w.events["reset"])] if w.marks else []
    seen = [None] * len(mon)
    try:
        while not w.stop:
            await FallingEdge(dut.dspClk)
            if w.marks:
                now = _cycle()
                tb = None if st.mirror.origin_cycle is None else st.mirror.time_of_cycle(now)
                for c, p in ram.items():
                    if _int_or_zero(p.io_port0_enable) and _int_or_zero(p.io_port0_write):
                        a = _int_or_zero(p.io_port0_address)
                        if a in words[c]:
                            w.events["marks"][c].append([now, tb, MEM_BASE + 4 * a, _int_or_zero(p.io_port0_wdata)])
                for i, (sig, ev) in enumerate(mon):
                    v = _int_or_zero(sig)
                    if v != seen[i]:
                        ev.append([now, tb, v])
                        seen[i] = v
            if st.mirror.origin_cycle is None:
                continue
            t = st.mirror.time_of_cycle(_cycle())
            for d in keys:
                if isinstance(d, str):
                    peak = _int_or_zero(getattr(dut, f"io_dio_{d[4:]}_out"))
                else:
                    peak = int(np.abs(_read_dac(dut, d)).max())
                stamp = t - pipes[d]
                rec = w.stats[d]
                rec["batches"] += 1
                if peak:
                    rec["peak"] = max(rec["peak"], peak)
                    rec["first"] = stamp if rec["first"] is None else rec["first"]
                    rec["last"] = stamp
                    if not prev[d]:
                        rec["last_rise"] = stamp
                        if len(rec["stretches"]) < w.STRETCHES:
                            rec["stretches"].append([stamp, stamp])
                    elif rec["stretches"] and rec["stretches"][-1][1] == stamp - 1:
                        rec["stretches"][-1][1] = stamp
                prev[d] = bool(peak)
    finally:
        w.done = True


def _read_dac(dut, dac_id: int) -> np.ndarray:
    """The current io_dac_<id> payload as BATCH_SIZE signed int16 lanes (lane k = bits [16k+15:16k])."""
    raw = int(getattr(dut, f"io_dac_{dac_id}_payload").value)
    return np.frombuffer(raw.to_bytes(BATCH_SIZE * 2, "little"), dtype="<i2").astype(np.int64)


def _pack_adc(lanes) -> int:
    """Pack ADC_BATCH int16 lanes into the little-endian io_adc payload word (lane j at bits
    [16j+15:16j])."""
    return int.from_bytes(np.asarray(lanes, dtype="<i2").tobytes(), "little")


async def _adc_stimulus(dut, st: "_BenchState") -> None:
    """Background loop closing the ADC loop through the active QuantumModel (spec 05 §3): every
    dspClk batch, sample the DACs the model reads, call adc_batch(batch_time, dac), and drive the
    returned io_adc payloads. With the default ZeroModel (or before the reset release, when batch
    time is undefined) there is nothing to do, so it free-runs in coarse chunks and leaves every
    ADC at 0 — keeping the DAC-only tests untouched. When a model is set (drv.sim.set_model) it
    ticks per batch. The falling edge matches the DAC-capture sampling (registered outputs settled)."""
    clk = dut.dspClk
    adc_sigs = {i: getattr(dut, f"io_adc_{i}_payload") for i in range(st.m.params.adc_num)}
    while True:
        model = st.model
        if isinstance(model, models.ZeroModel) or st.mirror.release_cycle is None:
            await ClockCycles(clk, IDLE_TICK)
            continue
        await FallingEdge(clk)
        t = st.mirror.time_of_cycle(_cycle())
        dac = {did: _read_dac(dut, did) for did in model.dac_ids()}
        for aid, lanes in model.adc_batch(t, dac).items():
            adc_sigs[aid].value = _pack_adc(lanes)


def _int_or_zero(sig) -> int:
    """A port value as an int, or 0 while it is still X (pre-reset)."""
    try:
        return int(sig.value)
    except ValueError:
        return 0


async def _host_window_slave(dut, st: "_BenchState") -> None:
    """Model of `S_AXI_HP0_FPD`: accept the host window's write-only AXI master, answer `b`, and
    store every beat into `st.host_mem` at `addr - HOST_BASE` under its `wstrb` (specs/software/22
    §2.2). Single-beat writes with one ID, so `aw` and `w` pair in issue order.

    Timing: runs on the falling edge and *commits* that cycle's `ready` there — AXI holds `valid`
    and its payload until the handshake, so "valid now && I drive ready now" IS the transfer at the
    coming rising edge. It **sleeps on `aw_valid`** whenever the funnel is idle, so the per-cycle
    python costs nothing between shots (the same reason `_adc_stimulus` free-runs in coarse chunks).

    `ready` is held high rather than randomised: back-pressure on this port is signed off in
    SpinalSim by `HostWindowFunnelSim`/`HostWindowCpuSim` (random ready stalls, golden memory), and
    stalling here would only slow the co-sim down.
    """
    clk = dut.clk
    awq: deque = deque()
    wq: deque = deque()
    pending_b = 0
    dut.io_hostMem_b_payload_id.value = 0
    dut.io_hostMem_b_payload_resp.value = 0
    dut.io_hostMem_aw_ready.value = 1
    dut.io_hostMem_w_ready.value = 1
    dut.io_hostMem_b_valid.value = 0
    while True:
        if not (_int_or_zero(dut.io_hostMem_aw_valid) or _int_or_zero(dut.io_hostMem_w_valid)
                or pending_b or awq or wq):
            await RisingEdge(dut.io_hostMem_aw_valid)   # idle: wake only when a write starts
            continue
        await FallingEdge(clk)
        if _int_or_zero(dut.io_hostMem_aw_valid):
            awq.append(_int_or_zero(dut.io_hostMem_aw_payload_addr))
        if _int_or_zero(dut.io_hostMem_w_valid):
            wq.append((_int_or_zero(dut.io_hostMem_w_payload_data),
                       _int_or_zero(dut.io_hostMem_w_payload_strb)))
        while awq and wq:
            addr = awq.popleft()
            data, strb = wq.popleft()
            st.host_write(addr, data, strb)
            pending_b += 1
        if pending_b and _int_or_zero(dut.io_hostMem_b_ready):
            dut.io_hostMem_b_valid.value = 1
            pending_b -= 1
        else:
            dut.io_hostMem_b_valid.value = 0

async def _wr_loopback(dut):
    """Self-loopback stand-in for the WR phy (with_white_rabbit builds; specs/white-rabbit 06 §4):
    62.5 MHz clkRef/clkRx togglers, each TX word replayed to RX after a constant small queue
    delay — bit offset 0 by construction, so `aligned` needs no dice-throw model — with
    ready/aligned mirroring the node's resetAll. Enough for the W5 cosim smoke: bring-up,
    self-exchange timestamps, marker; the real dice-throw/latency model is the SpinalSim
    GtySimPhy (WrNodeSim / WrTwoNodeSim)."""
    half_ns = 8
    q = deque([0] * 4)
    dut.wrPhy_clkRef.value = 0
    dut.wrPhy_clkRx.value = 0
    dut.wrPhy_rxDataRaw.value = 0
    dut.wrPhy_ready.value = 0
    dut.wrPhy_aligned.value = 0
    dut.wrPhy_diceCount.value = 1
    up = 0
    while True:
        dut.wrPhy_clkRef.value = 1
        dut.wrPhy_clkRx.value = 1
        await Timer(1, units="ns")               # let the TX PCS regs settle after the edge
        q.append(int(dut.wrPhy_txDataRaw.value))
        await Timer(half_ns - 1, units="ns")
        dut.wrPhy_clkRef.value = 0
        dut.wrPhy_clkRx.value = 0
        dut.wrPhy_rxDataRaw.value = q.popleft()  # data moves on the falling edge — clean setup
        if int(dut.wrPhy_resetAll.value) or int(dut.wrPhy_resetRxDatapath.value):
            dut.wrPhy_ready.value = 0
            dut.wrPhy_aligned.value = 0
            up = 0
        elif up < 8:
            up += 1
            if up == 8:                          # token bring-up delay (the dice-throw stand-in)
                dut.wrPhy_ready.value = 1
                dut.wrPhy_aligned.value = 1
        await Timer(half_ns, units="ns")


# ── Ant-Q uplink (results_path antq_uplink): the PL DDR4 behind the MIG, and the S2MM DMA ─────────────
DDR_CLK_PERIOD_NS = 7     # the MIG ui_clk stand-in: asynchronous to clk / dspClk (10 ns)
DDR_BEAT = 32             # 256-bit AXI beats


class DdrModel:
    """The MIG's AXI slave (256-bit, INCR bursts) over a sparse byte store, plus the knobs a test turns:
    `b_delay` (ui cycles from WLAST to BVALID: delayed writes), `aw_stall` / `ar_stall` / `b_stall`
    (per-cycle probability of holding the channel off), `bresp` / `rresp` (the response of the NEXT
    burst, then back to OKAY). The S2MM side (`dma_*`) stands in for axi_dma: armed with a length it
    holds TREADY (randomly deasserted with `tready_stall`; low for good once a transfer has taken
    `tready_after` beats, if that is >= 0), collects beats, and completes on TLAST."""

    def __init__(self):
        self.mem: dict[int, int] = {}          # byte address -> byte (absent = 0)
        self.rng = np.random.default_rng(20260929)
        self.b_delay = 0
        self.aw_stall = self.ar_stall = self.b_stall = 0.0
        self.tready_stall = 0.0
        self.tready_after = -1
        self.bresp_next = 0
        self.rresp_next = 0
        self.stats = {"aw": 0, "ar": 0, "w": 0, "r": 0, "b": 0, "aw_stalled": 0, "ar_stalled": 0,
                      "b_stalled": 0, "axis": 0, "axis_stalled": 0}
        self.dma = None                          # the armed S2MM transfer, or None (TREADY low)
        self._next_dma = 0
        self.reset_axi()

    def reset_axi(self) -> None:
        """The slave's open transactions (`_ddr_slave`): AW queue (bursts whose W is not complete), the
        current W beat, B queue (due cycle, id, resp), AR queue, the R burst in progress, BVALID up.
        Called again by the fabric reset (`ddr_reset`): on the board psr_ddr resets the MIG's AXI port,
        and the transactions it still owed no longer exist."""
        self.awq: deque = deque()
        self.wbeat = 0
        self.bq: deque = deque()
        self.arq: deque = deque()
        self.rcur = None                         # [addr, beats_left, id, resp]
        self.b_up = False

    def configure(self, cfg: dict) -> dict:
        for k in ("b_delay", "aw_stall", "ar_stall", "b_stall", "tready_stall", "tready_after", "bresp_next",
                  "rresp_next"):
            if k in cfg:
                setattr(self, k, type(getattr(self, k))(cfg[k]))
        return dict(self.stats)

    def open_state(self, dut) -> dict:
        """What is outstanding now on `m_axi_ddr` and `m_axis_rd` (a test observation, qubic3 BT): write
        bursts with AW accepted and W not complete (`aw_open`), with W complete and B not yet taken
        (`b_owed`); read bursts accepted and not complete (`reads_open`) and the R beats they still owe;
        the VALIDs the uplink holds up, the AXIS handshake pins, the armed S2MM transfer, the counters."""
        sig = lambda n: _int_or_zero(getattr(dut, n))      # noqa: E731
        t = self.dma
        return {"aw_open": len(self.awq), "b_owed": len(self.bq),
                "reads_open": len(self.arq) + (self.rcur is not None),
                "r_beats_owed": (self.rcur[1] if self.rcur else 0) + sum(a[1] for a in self.arq),
                "aw_valid": sig("m_axi_ddr_aw_valid"), "w_valid": sig("m_axi_ddr_w_valid"),
                "ar_valid": sig("m_axi_ddr_ar_valid"),
                "axis_valid": sig("m_axis_rd_valid"), "axis_ready": sig("m_axis_rd_ready"),
                "dma": None if t is None else {"nbytes": t.nbytes, "got": len(t.data), "tlast": t.tlast},
                "stats": dict(self.stats)}

    def stall(self, p: float) -> bool:
        return p > 0 and self.rng.random() < p

    def write_beat(self, addr: int, data: int, strb: int) -> None:
        for b in range(DDR_BEAT):
            if strb >> b & 1:
                self.mem[addr + b] = data >> (8 * b) & 0xFF

    def read_beat(self, addr: int) -> int:
        return int.from_bytes(bytes(self.mem.get(addr + b, 0) for b in range(DDR_BEAT)), "little")

    def read(self, addr: int, nbytes: int) -> bytes:
        return bytes(self.mem.get(addr + i, 0) for i in range(nbytes))


async def _ddr_slave(dut, dm: DdrModel) -> None:
    """`m_axi_ddr` slave on ddrClk. Same falling-edge discipline as `_host_window_slave`: at the falling
    edge the master's VALID/READY are stable since the last rising edge, so a READY/VALID driven now
    decides the transfer at the coming rising edge. One burst is written or read at a time per channel;
    AW/AR are queued (up to 4). The open transactions live on `dm` (`DdrModel.reset_axi`)."""
    clk = dut.ddrClk
    p = "m_axi_ddr"
    sig = lambda n: getattr(dut, f"{p}_{n}")      # noqa: E731
    cyc = 0
    for n in ("aw_ready", "w_ready", "b_valid", "ar_ready", "r_valid"):
        sig(n).value = 0
    while True:
        await FallingEdge(clk)
        cyc += 1
        # ── AW ──
        aw_ready = len(dm.awq) < 4 and not dm.stall(dm.aw_stall)
        if _int_or_zero(sig("aw_valid")):
            if aw_ready:
                dm.awq.append([_int_or_zero(sig("aw_payload_addr")), _int_or_zero(sig("aw_payload_len")) + 1,
                               _int_or_zero(sig("aw_payload_id"))])
                dm.stats["aw"] += 1
            else:
                dm.stats["aw_stalled"] += 1
        sig("aw_ready").value = int(aw_ready)
        # ── W: accepted only against a known burst (the uplink sends W after its AW) ──
        w_ready = bool(dm.awq)
        if w_ready and _int_or_zero(sig("w_valid")):
            addr, beats, bid = dm.awq[0]
            dm.write_beat(addr + DDR_BEAT * dm.wbeat, _int_or_zero(sig("w_payload_data")),
                          _int_or_zero(sig("w_payload_strb")))
            dm.stats["w"] += 1
            dm.wbeat += 1
            last = _int_or_zero(sig("w_payload_last"))
            if last != (dm.wbeat == beats):
                raise RuntimeError(f"m_axi_ddr: WLAST={last} on beat {dm.wbeat} of a {beats}-beat burst at {addr:#x}")
            if last:
                dm.awq.popleft()
                dm.wbeat = 0
                # AXI: BVALID only AFTER the last W handshake. At b_delay 0 the response used to be raised in this
                # very cycle, so the handshake of WLAST and of B fell on the same edge, and a master holding
                # BREADY high outside its wait-for-B state (CbufAxiWriter) lost it (qubic3 S0: every run whose
                # first bank was its final one hung its flush; the G3' runs always set b_delay >= 50).
                dm.bq.append((cyc + max(1, dm.b_delay), bid, dm.bresp_next))   # (due cycle, id, resp)
                dm.bresp_next = 0
        sig("w_ready").value = int(w_ready)
        # ── B: presented once due (and not stalled); once BVALID is up it stays up until the handshake
        # (AXI), which happens at the coming edge if BREADY is high now ──
        if not dm.b_up and dm.bq and dm.bq[0][0] <= cyc:
            if dm.stall(dm.b_stall):
                dm.stats["b_stalled"] += 1
            else:
                dm.b_up = True
        if dm.b_up:
            sig("b_payload_id").value = dm.bq[0][1]
            sig("b_payload_resp").value = dm.bq[0][2]
        sig("b_valid").value = int(dm.b_up)
        if dm.b_up and _int_or_zero(sig("b_ready")):
            dm.bq.popleft()
            dm.b_up = False
            dm.stats["b"] += 1
        # ── AR ──
        ar_ready = len(dm.arq) < 4 and not dm.stall(dm.ar_stall)
        if _int_or_zero(sig("ar_valid")):
            if ar_ready:
                dm.arq.append([_int_or_zero(sig("ar_payload_addr")), _int_or_zero(sig("ar_payload_len")) + 1,
                               _int_or_zero(sig("ar_payload_id")), dm.rresp_next])
                dm.rresp_next = 0
                dm.stats["ar"] += 1
            else:
                dm.stats["ar_stalled"] += 1
        sig("ar_ready").value = int(ar_ready)
        # ── R: one burst at a time, in order ──
        if dm.rcur is None and dm.arq:
            dm.rcur = dm.arq.popleft()
        if dm.rcur is not None:
            addr, left, rid, resp = dm.rcur
            sig("r_payload_data").value = dm.read_beat(addr)
            sig("r_payload_id").value = rid
            sig("r_payload_resp").value = resp
            sig("r_payload_last").value = int(left == 1)
            sig("r_valid").value = 1
            if _int_or_zero(sig("r_ready")):
                dm.stats["r"] += 1
                dm.rcur = None if left == 1 else [addr + DDR_BEAT, left - 1, rid, resp]
        else:
            sig("r_valid").value = 0


class DmaTransfer:
    """One armed S2MM transfer: `nbytes` programmed, beats collected until TLAST."""

    def __init__(self, nbytes: int):
        self.nbytes = nbytes
        self.data = bytearray()
        self.tlast = False
        self.error: str | None = None


async def _axis_sink(dut, dm: DdrModel) -> None:
    """`m_axis_rd` -> the S2MM DMA stand-in. TREADY is high only while a transfer is armed (a halted
    DMA holds it low), minus the random `tready_stall`. Like axi_dma it completes on TLAST: a packet
    longer than the programmed length is an error, a shorter one is returned short (the driver's
    `ddr.py` length checks then refuse it)."""
    clk = dut.ddrClk
    dut.m_axis_rd_ready.value = 0
    while True:
        await FallingEdge(clk)
        t = dm.dma
        ready = (t is not None and not t.tlast and t.error is None and not dm.stall(dm.tready_stall)
                 and (dm.tready_after < 0 or len(t.data) < DDR_BEAT * dm.tready_after))
        if _int_or_zero(dut.m_axis_rd_valid):
            if ready:
                beat = _int_or_zero(dut.m_axis_rd_payload_fragment)
                t.data += beat.to_bytes(DDR_BEAT, "little")
                dm.stats["axis"] += 1
                if _int_or_zero(dut.m_axis_rd_payload_last):
                    t.tlast = True
                if len(t.data) > t.nbytes:
                    t.error = f"S2MM overrun: {len(t.data)} B received for a {t.nbytes} B transfer"
            elif t is not None:
                dm.stats["axis_stalled"] += 1
        dut.m_axis_rd_ready.value = int(ready)


class _Req:
    __slots__ = ("op", "args", "done", "result", "error")

    def __init__(self, op: str, args: tuple):
        self.op, self.args = op, args
        self.done = threading.Event()
        self.result = None
        self.error: Exception | None = None


def _wire_errors(fn):
    """qubic3 S0: a server-side runner's exception crosses Pyro only if its class is a builtin; any
    other (DdrUplinkError, SessionPoisoned, ...) is re-raised as RuntimeError naming it, so the client
    sees what failed instead of a deserialisation error."""
    import functools

    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except Exception as e:
            if type(e).__module__ == "builtins":
                raise
            raise RuntimeError(f"{type(e).__name__}: {e}") from None
    return wrapper


@Pyro5.api.expose
class DriverServer:
    """Pyro5 face of the bench: marshals every call onto the request queue the cocotb
    coroutine services; blocks the (daemon-thread) caller until the sim replies."""

    def __init__(self, reqs: queue.Queue, params_text: str):
        self._reqs = reqs
        self._params = params_text
        self._m = None            # server-side SocMap, built on remote_setup (spec 08 §5)
        self._progs = {}          # core -> Program, rebuilt from the wire on remote_setup
        self.sim = self           # so riscq.run.poll_done finds `.sim.poll_word` locally
        self.host_base = HOST_BASE  # riscq.run.setup programs this into HOSTWIN_BASE_LO/HI

    def _submit(self, op: str, *args):
        req = _Req(op, args)
        self._reqs.put(req)
        if not req.done.wait(timeout=600):
            raise RuntimeError(f"cosim bench did not service {op!r} within 600 s (sim stalled?)")
        if req.error is not None:
            raise req.error
        return req.result

    def read32(self, addr):
        return self._submit("read32", int(addr))

    def write32(self, addr, value):
        return self._submit("write32", int(addr), int(value))

    def read_block(self, addr, nbytes):
        return self._submit("read_block", int(addr), int(nbytes))

    def write_block(self, addr, data):
        data = serpent.tobytes(data) if isinstance(data, dict) else bytes(data)
        return self._submit("write_block", int(addr), data)

    def advance(self, cycles):
        return self._submit("advance", int(cycles))

    def batch_time(self):
        """Current batch time (refTime + timeOffset). Monotonic across runs — the host reads it to
        schedule an absolute-time capture ahead of `now` (spec 08: refTime free-runs in dspCd)."""
        return self._submit("batch_time")

    def cycles(self):
        return self._submit("cycles")

    def pl_reset(self, cycles=16):
        """qubic3 S0: pulse `dspRst` and `reset` (the bench's pl_resetn0), see CosimDriver.sim.pl_reset."""
        return self._submit("pl_reset", int(cycles))

    def pl_reset_snapshot(self):
        """qubic3 BT: what was outstanding when the last pl_reset pulse began; see CosimDriver.sim."""
        return self._submit("pl_reset_snapshot")

    def ddr_reset(self, cycles=16):
        """qubic3 BT: pulse `ddrRst` (psr_ddr's fabric reset); see CosimDriver.sim.ddr_reset."""
        return self._submit("ddr_reset", int(cycles))

    def poll_word(self, addr, not_equal, timeout_cycles):
        return self._submit("poll_word", int(addr), int(not_equal), int(timeout_cycles))

    # ── qubic3 P4 (after-stage r1 #1): deterministic bench scheduling ──
    def lockstep(self, on):
        """Lockstep (on) or free-running (off, the default) between requests; returns the cycle.
        See CosimDriver.sim.lockstep."""
        return self._submit("lockstep", bool(on))

    def sched(self, ops):
        """Host accesses at exact cycles, in one request; see CosimDriver.sim.sched."""
        return self._submit("sched", [list(o) for o in ops])

    def dac_capture_arm(self, dac_id, n_batches, start_batch=None):
        return self._submit("dac_arm", int(dac_id), int(n_batches),
                            None if start_batch is None else int(start_batch))

    def dac_capture_get(self, handle):
        return self._submit("dac_get", int(handle))

    def dac_watch_start(self, dac_ids, dios=None, marks=None):
        return self._submit("watch_start", [int(d) for d in dac_ids], [str(n) for n in (dios or ())],
                            {int(c): [int(a) for a in v] for c, v in dict(marks or {}).items()})

    def dac_watch_stop(self, handle):
        return self._submit("watch_stop", int(handle))

    def dio_capture_arm(self, name, n_batches, start_batch=None):
        return self._submit("dio_arm", str(name), int(n_batches),
                            None if start_batch is None else int(start_batch))

    def dio_capture_get(self, handle):
        return self._submit("dac_get", int(handle))

    def dio_set(self, name, value):
        return self._submit("dio_set", str(name), int(value))

    def dio_loopback(self, name, on):
        return self._submit("dio_loopback", str(name), bool(on))

    def set_model(self, spec):
        return self._submit("set_model", dict(spec))

    def model_state(self):
        return self._submit("model_state")

    def read_host(self, offset, nbytes):
        return self._submit("read_host", int(offset), int(nbytes))

    def get_host_base(self):
        return int(self.host_base)

    def get_params(self):
        return self._params

    # ── antq_uplink builds: the uplink's control slave, the modelled PL DDR4 and the S2MM stand-in ──
    def ddr_read32(self, off):
        return self._submit("ddr_read32", int(off))

    def ddr_write32(self, off, value):
        return self._submit("ddr_write32", int(off), int(value))

    def ddr_config(self, cfg):
        return self._submit("ddr_config", dict(cfg))

    def ddr_mem(self, addr, nbytes):
        return self._submit("ddr_mem", int(addr), int(nbytes))

    def dma_arm(self, nbytes):
        return self._submit("dma_arm", int(nbytes))

    def dma_get(self, timeout_cycles):
        return self._submit("dma_get", int(timeout_cycles))

    # ── server-side batch runner (spec 08 §5): run the SAME riscq.run functions next to the sim,
    # so a whole batch is one RPC instead of ~10 per-op round trips. The seam ops each method issues
    # go straight onto the request queue (self._submit — no network hop); `self.sim = self` gives
    # run.poll_done its `.sim.poll_word`, and DriverServer has no `.remote` attr so run.setup/rerun
    # take their LOCAL per-op path here.
    @_wire_errors
    def remote_setup(self, params_json, progmap):
        from riscq import run as _run
        from riscq.map import SocMap, SocParams
        self._m = SocMap(SocParams.from_json(self._params))   # the server's own build is ground truth
        self._progs = {int(c): _run._prog_from_wire(w) for c, w in progmap.items()}
        _run.setup(self, self._m, self._progs)
        return None

    @_wire_errors
    def remote_rerun(self, cores, params, arrays, results, timeout, identities=None, uplink=None, stop=None):
        from riscq import run as _run
        from riscq import stop as _stop
        progs = {int(c): self._progs[int(c)] for c in cores}
        up = None if uplink is None else _run.UplinkRun.from_wire(dict(uplink))
        if up is not None:
            up.remote_reply = True
        try:
            out = _run.rerun(self, self._m, progs,
                             params={int(c): v for c, v in dict(params).items()},
                             arrays={int(c): v for c, v in dict(arrays).items()},
                             results=(None if results is None else list(results)), timeout=int(timeout),
                             uplink=up, identities=(None if identities is None else
                                                    {int(c): str(i) for c, i in dict(identities).items()}),
                             stop=None if stop is None else _stop.from_wire(dict(stop)))
        except _run.StopInconsistent as exc:    # qubic3 P4: certified; the client raises it with the data
            out = exc.out
        reply = {c: {n: bytes(a.astype("<i4").tobytes()) for n, a in d.items()} for c, d in out.items()}
        rec = _run.session(self).runs[-1].stop      # qubic3 P4: the run's StopRecord goes back too
        if rec is not None:
            reply["__stop"] = rec.to_wire()
        return reply

    # ── qubic3 S0: the stop seam's remote twin (no MMIO; never waits on the running run) and recovery ──
    def post_stop(self, run_id, kind, S=None):
        from riscq import run as _run
        return _run.request_stop(self, tuple(run_id), kind, S).outcome

    def current_run(self):
        from riscq import run as _run
        r = _run.current_run(self)
        return None if r is None else list(r)

    @_wire_errors
    def remote_recover(self):
        from riscq import run as _run
        return _run.recover(self, self._m or SocMap(SocParams.from_json(self._params)))

    def shutdown(self):
        return self._submit("shutdown")


class _BenchState:
    """The bench's non-AXI state: the SocMap, the time mirror, and the armed captures."""

    def __init__(self, m: SocMap):
        self.m = m
        self.mirror = TimeMirror(m)
        self.captures: dict[int, DacCapture] = {}
        self._next_handle = 0
        self.model = models.ZeroModel()   # ADC seam; replaced at runtime via set_model
        # the modelled PS DDR4 result buffer: one 16 MB slice per core (specs/software/22 §3); none
        # on an antq_uplink build, which has instead the uplink's PL DDR4 and S2MM models
        self.host_mem = bytearray(m.hostwin_bytes_total)
        self.dm = DdrModel() if m.params.with_antq_uplink else None
        self.dio_loop: dict[str, bool] = {}
        self.ddr_axi: AxiMaster | None = None
        self.lockstep = False   # qubic3 P4: sim time advances only inside requests (no idle free-run)
        self.pulses = 0         # qubic3 BT: pl_reset pulses so far, and the snapshot taken as the last began
        self.pulse_snapshot: dict | None = None

    def host_write(self, addr: int, data: int, strb: int) -> None:
        """Apply one AXI beat to the modelled buffer. An address outside the buffer is a real bug
        (a bad HOSTWIN_BASE or a runaway offset), so it is loud rather than silently dropped."""
        off = addr - HOST_BASE
        if not 0 <= off <= len(self.host_mem) - 4:
            raise RuntimeError(f"host-window write to {addr:#x} outside the "
                               f"{len(self.host_mem)} B buffer at {HOST_BASE:#x}")
        for b in range(4):
            if strb >> b & 1:
                self.host_mem[off + b] = data >> (8 * b) & 0xFF

    def new_capture(self, cap: DacCapture) -> int:
        self._next_handle += 1
        self.captures[self._next_handle] = cap
        return self._next_handle


async def _handle(axi: AxiMaster, dut, st: _BenchState, op: str, args: tuple):
    if op == "read32":
        return await axi.read_word(args[0])
    if op == "write32":
        result = await axi.write_word(args[0], args[1])
        st.mirror.on_write(args[0], args[1])
        return result
    if op == "dac_arm":
        dac_id, n_batches, start_batch = args
        if not hasattr(dut, f"io_dac_{dac_id}_payload"):
            raise ValueError(f"no such DAC port: io_dac_{dac_id}_payload")
        cap = DacCapture(dac_id, n_batches, start_batch)
        handle = st.new_capture(cap)
        cocotb.start_soon(_capture_run(dut, st.m, st.mirror, cap))
        return handle
    if op == "dac_get":
        cap = st.captures.get(args[0])
        if cap is None:
            raise ValueError(f"unknown capture handle {args[0]}")
        spent = 0
        while not cap.done and spent < DAC_GET_TIMEOUT:
            await ClockCycles(dut.dspClk, POLL_INTERVAL)
            spent += POLL_INTERVAL
        if not cap.done:
            raise RuntimeError(f"capture {args[0]} not finished after {DAC_GET_TIMEOUT} cycles")
        if cap.error is not None:
            raise RuntimeError(f"capture failed: {cap.error}")
        if cap.origin_cycle is None:
            raise RuntimeError("capture finished with no refTime origin (dspRst not released) — no time base")
        pipe = st.m.dac_pipe(cap.dac_id) if cap.pipe is None else cap.pipe
        t0 = cap.first_cycle - cap.origin_cycle - SIMSTART_TO_TIME0 + cap.offset - pipe
        lane_bytes = BATCH_SIZE * 2 if cap.pipe is None else 4
        data = b"".join(v.to_bytes(lane_bytes, "little") for v in cap.vals)
        del st.captures[args[0]]
        return t0, cap.n_batches, data
    if op == "watch_start":
        for d in args[0]:
            if not hasattr(dut, f"io_dac_{d}_payload"):
                raise ValueError(f"no such DAC port: io_dac_{d}_payload")
        dios = list(args[1]) if len(args) > 1 else []
        for n in dios:
            if not hasattr(dut, f"io_dio_{n}_out"):
                raise ValueError(f"no such DIO port: io_dio_{n}_out")
        marks = {int(c): [int(a) for a in v] for c, v in dict(args[2] if len(args) > 2 and args[2] else {}).items()}
        for c in marks:
            if not 0 <= c < len(st.m.params.cores):
                raise ValueError(f"no core {c} on {st.m.params.name}")
            if not all(hasattr(_core_ram(dut, c), f"io_port0_{s}") for s in ("enable", "write", "address", "wdata")):
                raise ValueError(f"core {c}: no RAM port 0 to monitor")
        if marks and not (hasattr(dut, "doneHostCd") and hasattr(dut, "riscqReset")):
            raise ValueError("no doneHostCd / riscqReset signal to monitor")
        w = DacWatch(args[0], dios, marks)
        handle = st.new_capture(w)
        cocotb.start_soon(_watch_run(dut, st, w))
        return handle
    if op == "watch_stop":
        w = st.captures.pop(args[0], None)
        if not isinstance(w, DacWatch):
            raise ValueError(f"unknown watch handle {args[0]}")
        w.stop = True
        while not w.done:
            await ClockCycles(dut.dspClk, 1)
        out = {str(d): rec for d, rec in w.stats.items()}
        if w.marks:
            out["mon"] = {"marks": {str(c): ev for c, ev in w.events["marks"].items()}, "done": w.events["done"],
                          "reset": w.events["reset"]}
        return out
    if op == "dio_arm":
        name, n_batches, start_batch = args
        sig = f"io_dio_{name}_out"
        if not hasattr(dut, sig):
            raise ValueError(f"no such DIO port: {sig}")
        cap = DacCapture(-1, n_batches, start_batch, sig=sig, pipe=DIO_PIPE)
        handle = st.new_capture(cap)
        cocotb.start_soon(_capture_run(dut, st.m, st.mirror, cap))
        return handle
    if op == "dio_set":
        name, value = args
        sig = f"io_dio_{name}_in"
        if not hasattr(dut, sig):
            raise ValueError(f"no such DIO port: {sig}")
        getattr(dut, sig).value = int(value) & 0xFFFF
        await ClockCycles(dut.dspClk, 1)
        return None
    if op == "dio_loopback":
        # a wire from the bank's outputs back to its inputs: every scheduled output edge then posts an
        # input event on the core's up-link (the traffic an uplink tap must not count)
        name, on = args
        out_sig, in_sig = f"io_dio_{name}_out", f"io_dio_{name}_in"
        if not (hasattr(dut, out_sig) and hasattr(dut, in_sig)):
            raise ValueError(f"no such DIO bank: {name}")
        st.dio_loop[name] = bool(on)
        if on:
            async def _loop():
                o, i = getattr(dut, out_sig), getattr(dut, in_sig)
                while st.dio_loop.get(name):
                    await FallingEdge(dut.dspClk)
                    i.value = _int_or_zero(o)
            cocotb.start_soon(_loop())
        return None
    if op == "read_block":
        addr, nbytes = args
        if nbytes % 4:
            raise ValueError(f"read_block nbytes {nbytes} not word-aligned")
        out = bytearray()
        for i in range(nbytes // 4):
            out += (await axi.read_word(addr + 4 * i)).to_bytes(4, "little")
        return bytes(out)
    if op == "write_block":
        addr, data = args
        if len(data) % 4:
            data = data + b"\x00" * (4 - len(data) % 4)
        for i in range(len(data) // 4):
            await axi.write_word(addr + 4 * i, int.from_bytes(data[4 * i:4 * i + 4], "little"))
        return None
    if op == "read_host":
        off, nbytes = args
        if not 0 <= off <= len(st.host_mem) - nbytes:
            raise ValueError(f"read_host [{off}, {off + nbytes}) outside the "
                             f"{len(st.host_mem)} B host buffer")
        return bytes(st.host_mem[off:off + nbytes])
    if op in ("ddr_read32", "ddr_write32", "ddr_config", "ddr_mem", "dma_arm", "dma_get", "ddr_reset"):
        if st.dm is None:
            raise ValueError(f"{op}: this build has results_path={st.m.params.results_path!r}, no uplink")
        if op == "ddr_reset":
            # qubic3 BT: psr_ddr's peripheral reset, a bench stimulus on the toplevel's ddrRst (no RTL change),
            # what a PL reload does to the DDR side: the uplink's whole DDR clock domain (axi_rst_fault with it,
            # which nothing else clears) and the AXI fabric, so the MIG port's open transactions are dropped.
            # The MIG stays calibrated; the S2MM stand-in is left to the driver's own DMA reset.
            dut.ddrRst.value = 1
            st.dm.reset_axi()
            await ClockCycles(dut.ddrClk, max(2, int(args[0])))
            dut.ddrRst.value = 0
            await ClockCycles(dut.clk, 64)          # the host-domain DDR status and the uplink's synchronisers
            return None
        if op == "ddr_read32":
            return await st.ddr_axi.read_word(args[0])
        if op == "ddr_write32":
            return await st.ddr_axi.write_word(args[0], args[1])
        if op == "ddr_config":
            return st.dm.configure(args[0])
        if op == "ddr_mem":
            return st.dm.read(args[0], args[1])
        if op == "dma_arm":
            if st.dm.dma is not None and not st.dm.dma.tlast and st.dm.dma.error is None:
                raise RuntimeError("dma_arm: a transfer is still in flight")
            st.dm.dma = DmaTransfer(args[0])
            return None
        if op == "dma_get":
            t = st.dm.dma
            if t is None:
                raise RuntimeError("dma_get: nothing armed")
            spent = 0
            while not t.tlast and t.error is None and spent < args[0]:
                await ClockCycles(dut.ddrClk, 50)
                spent += 50
            st.dm.dma = None
            return bytes(t.data), bool(t.tlast), t.error
    if op == "set_model":
        st.model = models.build_model(dict(args[0]), st.m)
        return None
    if op == "model_state":
        return st.model.ground_truth()
    if op == "advance":
        await ClockCycles(dut.clk, args[0])
        return None
    if op == "batch_time":
        return st.mirror.time_of_cycle(_cycle())
    if op == "cycles":
        return _cycle()
    if op == "pl_reset_snapshot":
        return st.pulse_snapshot
    if op == "pl_reset":
        # qubic3 S0: the bench's pl_resetn0, a stimulus on the toplevel's reset inputs (no RTL change).
        # Both proc_sys_reset outputs pl_resetn0 drives on the board: dspRst (dsp_rst) and the host
        # domain's reset (ps_rst). Released together, like at sim start, where the origin is pinned.
        # qubic3 BT: first, what is outstanding as the pulse begins (the batch time is the old base's).
        st.pulses += 1
        now = _cycle()
        st.pulse_snapshot = {"n": st.pulses, "cycle": now,
                             "batch_time": None if st.mirror.origin_cycle is None else st.mirror.time_of_cycle(now),
                             **({} if st.dm is None else st.dm.open_state(dut))}
        dut.dspRst.value = 1
        dut.reset.value = 1
        await ClockCycles(dut.clk, max(2, int(args[0])))
        await Timer(CLK_PERIOD_NS, units="ns")  # release at an edge time from a Timer, as at sim start, so
        dut.reset.value = 0                     # SIMSTART_TO_TIME0 holds for the new origin
        dut.dspRst.value = 0
        st.mirror.set_origin(_cycle())
        st.mirror.host_reset()
        await ClockCycles(dut.clk, 64)          # the reset synchronisers and the uplink's DDR-half follow
        return None
    if op == "lockstep":
        st.lockstep = bool(args[0])
        return _cycle()
    if op == "sched":
        # qubic3 P4 (after-stage r1 #1): each item [at, kind, *args] starts at clk cycle `at` (None: right
        # after the previous one); a cycle already passed is an error, so a schedule is either kept or loud.
        # Kinds: write32 (addr, value), read32 (addr), advance (cycles). Returns per item
        # [start cycle, end cycle, value or None, batch time at the start].
        out = []
        for item in args[0]:
            at, kind, rest = item[0], item[1], item[2:]
            now = _cycle()
            if at is not None:
                if at < now:
                    raise RuntimeError(f"sched: cycle {at} has passed (now {now}); no access was issued late")
                if at > now:
                    await ClockCycles(dut.clk, at - now)
            t0 = _cycle()
            value = None
            if kind == "write32":
                await axi.write_word(rest[0], rest[1])
                st.mirror.on_write(rest[0], rest[1])
            elif kind == "read32":
                value = await axi.read_word(rest[0])
            elif kind == "advance":
                await ClockCycles(dut.clk, rest[0])
            else:
                raise ValueError(f"sched: unknown kind {kind!r}")
            out.append([t0, _cycle(), value, st.mirror.time_of_cycle(t0) if st.mirror.origin_cycle is not None else None])
        return out
    if op == "poll_word":
        addr, not_equal, timeout_cycles = args
        value = await axi.read_word(addr)
        spent = 0
        while value == not_equal and spent < timeout_cycles:
            step = min(POLL_INTERVAL, timeout_cycles - spent)
            await ClockCycles(dut.clk, step)
            spent += step
            value = await axi.read_word(addr)
        return value
    raise ValueError(f"unknown op {op!r}")


@cocotb.test()
async def cosim_server(dut):
    uri_file = Path(os.environ["RISCQ_COSIM_URI_FILE"])
    params_text = Path(os.environ["RISCQ_COSIM_CONFIG"]).read_text()
    cfg = json.loads(params_text)
    st = _BenchState(SocMap(SocParams.from_json(params_text)))

    # idle inputs before the first edge (SOC_TIPS: never let bus valids float during reset)
    dut.reset.value = 1
    dut.dspRst.value = 1
    for sig in ("aw", "w", "ar"):
        getattr(dut, f"io_axi_{sig}_valid").value = 0
    dut.io_axi_b_ready.value = 0
    dut.io_axi_r_ready.value = 0
    if st.m.params.with_host_window:
        dut.io_hostMem_aw_ready.value = 0
        dut.io_hostMem_w_ready.value = 0
        dut.io_hostMem_b_valid.value = 0
    else:                                   # antq_uplink: the uplink's own ports, DDR side in reset
        dut.ddrRst.value = 1
        dut.ddrCalibDone.value = 0
        for sig in ("aw", "w", "ar"):
            getattr(dut, f"s_axi_ddr_ctrl_{sig}_valid").value = 0
        dut.s_axi_ddr_ctrl_b_ready.value = 0
        dut.s_axi_ddr_ctrl_r_ready.value = 0
    for i in range(cfg["dac_num"]):
        getattr(dut, f"io_dac_{i}_ready").value = 1
    for i in range(cfg["adc_num"]):
        getattr(dut, f"io_adc_{i}_valid").value = 1
        getattr(dut, f"io_adc_{i}_payload").value = 0

    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units="ns").start())
    cocotb.start_soon(Clock(dut.dspClk, CLK_PERIOD_NS, units="ns").start())
    cocotb.start_soon(_adc_stimulus(dut, st))   # ADC seam (idle until a model is set)
    if st.m.params.with_host_window:
        cocotb.start_soon(_host_window_slave(dut, st))   # PS DDR4 result buffer (specs/software/22)
    else:
        cocotb.start_soon(Clock(dut.ddrClk, DDR_CLK_PERIOD_NS, units="ns").start())
        cocotb.start_soon(_ddr_slave(dut, st.dm))        # the MIG + PL DDR4
        cocotb.start_soon(_axis_sink(dut, st.dm))        # axi_dma S2MM
        st.ddr_axi = AxiMaster(dut, prefix="s_axi_ddr_ctrl", clk=dut.ddrClk, aw_w_together=True)
    if cfg.get("with_white_rabbit", False):
        cocotb.start_soon(_wr_loopback(dut))    # WR phy self-loopback (test_wr.py cosim smoke)
    await Timer(200, units="ns")
    dut.reset.value = 0
    dut.dspRst.value = 0
    if not st.m.params.with_host_window:
        dut.ddrCalibDone.value = 1          # the MIG calibrated ...
        dut.ddrRst.value = 0                # ... and psr_ddr released the ui_clk reset tree
    # refTime (dspCd, free-running) starts counting from this dspRst release — pin the batch-time anchor
    # here. Batch time is monotonic across runs, so this is the single session-wide origin (spec 08).
    st.mirror.set_origin(_cycle())
    await Timer(200, units="ns")
    # riscqReset stays asserted (powers up held); the host releases it over AXI (riscq.run.reset)

    axi = AxiMaster(dut)
    reqs: queue.Queue = queue.Queue()
    daemon = Pyro5.api.Daemon(host="127.0.0.1", port=0)
    uri = daemon.register(DriverServer(reqs, params_text), objectId="riscq.cosim")
    threading.Thread(target=daemon.requestLoop, daemon=True).start()
    tmp = uri_file.with_suffix(".tmp")
    tmp.write_text(str(uri))
    os.replace(tmp, uri_file)   # atomic: the client never sees a partial file
    dut._log.info(f"[riscq cosim] serving {uri}")

    while True:
        try:
            req = reqs.get_nowait()
        except queue.Empty:
            if not st.lockstep:
                await ClockCycles(dut.clk, IDLE_TICK)   # bounded free-run between requests
                continue
            # qubic3 P4 lockstep: block without advancing the clock, so the client's think time moves no
            # sim time and every host access lands on a cycle fixed by the requests alone
            req = reqs.get()
        if req.op == "shutdown":
            req.result = True
            req.done.set()
            break
        try:
            req.result = await _handle(axi, dut, st, req.op, req.args)
        except Exception as exc:  # keep the sim alive; the client re-raises
            req.error = RuntimeError(f"{type(exc).__name__}: {exc}")
        req.done.set()

    dut._log.info("[riscq cosim] shutdown")
