"""A host-pure SoC double for the run-layer tests (qubic3 S0, plan P4 v2 §8).

`FakeSoc` holds every word the host writes in a sparse store and answers the few registers the run
layer reads back: the host control block's DONE word (set for every loaded core when the shared
core reset is released, cleared when it is asserted again) and, on an antq_uplink build, the
HOST_DDR_STATUS readiness word. A core counts as loaded once a block write lands on its reset
vector, and as parked after the `j .` park word. `read_host` returns zeros: the host-pure tests
check the run protocol, not results.

It has no `.sim`, so `riscq.run.poll_done` takes its hardware branch (one read, no sleep: DONE is
already set), and it publishes its SocParams JSON on `.board`, the way `RemoteDriver` does, so
`riscq.cal.base.socmap(drv)` works."""

from __future__ import annotations

import hashlib

from riscq import ddr_regs as R
from riscq.ddr import DdrMap
from riscq.map import SocMap, SocParams


class _Board:
    def __init__(self, params_json: str):
        self._params_json = params_json

    def get_params(self) -> str:
        return self._params_json


class FakeSoc:
    """`on_release(fake)`, when given, runs at every reset release in place of the default kernel
    (which sets DONE for every loaded core): a test models its kernels there (post results with
    `fake.up.post`, write markers into `fake.mem`, leave DONE low for a hang). `pl_reset()` models
    the pl_resetn0 pulse; a test sets `fake.pl_reset = None` for a driver without it."""

    PARK = 0x6F

    def __init__(self, params_json: str, host_base: int | None = 0x7000_0000):
        self.board = _Board(params_json)
        self.m = SocMap(SocParams.from_json(params_json))
        self.mem: dict[int, int] = {}         # word address -> 32-bit value
        self.loaded: set[int] = set()
        self.done = 0
        self.reset_held = True
        self.releases = 0
        self.pl_resets = 0
        self.on_release = None
        # an antq_uplink build has no host window and so no result buffer (riscq.run.setup reads this)
        self.host_base = host_base if self.m.params.with_host_window else None
        self._imem = {self.m.imem(c): c for c in range(len(self.m.params.cores))}
        self._reset = self.m.host_ctrl + self.m.HOST_RESET
        self._done = self.m.host_ctrl + self.m.HOST_DONE
        self._ddr_status = self.m.ddr_status() if self.m.params.with_antq_uplink else None
        self.up = FakeUplink(len(self.m.params.cores)) if self.m.params.with_antq_uplink else None

    def pl_reset(self) -> None:
        """pl_resetn0: the uplink back to idle with its accounting cleared, the host domain's
        registers with a reset value cleared (time offset, host-window base), DONE low. The core
        hold survives."""
        self.pl_resets += 1
        self.done = 0
        hc = self.m.host_ctrl
        for off in (self.m.HOST_TIME_OFF_LO, self.m.HOST_TIME_OFF_HI, 0x48, 0x4C):
            self.mem.pop(hc + off, None)
        if self.up is not None:
            self.up.dsp_reset()

    # ── the Driver seam ──
    def write32(self, addr: int, value: int) -> None:
        addr, value = int(addr), int(value) & 0xFFFFFFFF
        if self.up is not None and self.up.owns(addr):
            self.up.write(addr, value)
            return
        if addr == self._reset:
            held = bool(value & 1)
            if held and not self.reset_held:
                self.done = 0
            self.reset_held = held
            if not held:
                self.releases += 1
                if self.on_release is not None:
                    self.on_release(self)
                else:
                    self.done = sum(1 << c for c in self.loaded)
            return
        if addr in self._imem:
            core = self._imem[addr]
            (self.loaded.discard if value == self.PARK else self.loaded.add)(core)
        self.mem[addr] = value

    def read32(self, addr: int) -> int:
        addr = int(addr)
        if self.up is not None and self.up.owns(addr):
            return self.up.read(addr)
        if addr == self._done:
            return self.done
        if addr == self._ddr_status:
            return (self.m.DDR_STATUS_MAGIC << 16) | 0b11
        return self.mem.get(addr, 0)

    # ── the DdrReadout DMA surface (antq_uplink) ──
    def dma_recv_prepare(self, nbytes: int):
        return self.up.dma_prepare(nbytes)

    def dma_recv_wait(self, buf, nbytes: int) -> bytes:
        return self.up.dma_wait(buf, nbytes)

    def dma_idle(self) -> bool:
        return self.up.dma is None

    def dma_reset(self) -> None:
        self.up.dma_resets += 1
        self.up.dma = None

    def write_block(self, addr: int, data: bytes) -> None:
        addr, data = int(addr), bytes(data)
        if len(data) % 4:
            data += b"\x00" * (4 - len(data) % 4)
        for i in range(0, len(data), 4):
            self.mem[addr + i] = int.from_bytes(data[i:i + 4], "little")
        if addr in self._imem:
            self.loaded.add(self._imem[addr])

    def read_block(self, addr: int, nbytes: int) -> bytes:
        addr = int(addr)
        return b"".join(self.mem.get(addr + i, 0).to_bytes(4, "little") for i in range(0, int(nbytes), 4))

    def read_host(self, offset: int, nbytes: int) -> bytes:
        return bytes(int(nbytes))


class FakeUplink:
    """The uplink's register semantics as the run layer relies on them (src/riscq/ddr/CONTRACT.md I3,
    I7): W1C stickies, all cleared by an accepted BASE_RESET; admission from the start acknowledge to
    the flush commit; a result outside it counted in REJECTED (saturating at 0xFFFF) with
    `early_late` set; `final_addr`; a drain through the S2MM stand-in. The `script` set changes one
    behaviour each: wr_base_mismatch, base_busy, start_dropped, start_dropped_twice, never_start,
    run_base_mismatch, flush_refused (once), flush_stuck, bresp, dma_error, never_idle. `start_delay`
    register reads pass before a start completes (so a zero-timeout prepare raises first)."""

    def __init__(self, num_ch: int):
        self.num_ch = num_ch
        self.base = DdrMap().ctrl_base
        self.script: set = set()
        self.start_delay = 1
        self.mem = bytearray()
        self.dma = None
        self.dma_resets = 0
        self.flushes = self.base_resets = 0
        self.on_flush = None                       # called after a FLUSH commit (a late arrival hook)
        self.dsp_reset()

    def dsp_reset(self) -> None:
        self.wr_base = self.run_base = self.ptr = self.final_addr = 0
        self.sticky = 0
        self.run_active = self.admit = False
        self.start_in = None                       # STATUS reads until a pending start completes
        self.accepted = [0] * self.num_ch
        self.rejected = [0] * self.num_ch
        self.rd_base = self.rd_size = 0

    def owns(self, addr: int) -> bool:
        return self.base <= addr < self.base + 0x1_0000

    def post(self, core: int, real: int, imag: int) -> None:
        """One decoder result of `core`, as the RTL takes it from ReadoutResultLink."""
        if self.admit:
            word = (core << 56) | ((real & 0xFFFFFFFF) >> 4 << 28) | ((imag & 0xFFFFFFFF) >> 4)
            off = self.ptr
            if len(self.mem) < off + 8:
                self.mem.extend(bytes(off + 8 - len(self.mem)))
            self.mem[off:off + 8] = word.to_bytes(8, "little")
            self.ptr += 8
            self.accepted[core] += 1
        else:
            self.rejected[core] = min(0xFFFF, self.rejected[core] + 1)
            self.sticky |= 1 << R.S_EARLY_LATE

    def _tick(self) -> None:
        """Every register read lets the model's time advance: a pending start completes after
        `start_delay` reads."""
        if self.start_in is not None:
            self.start_in -= 1
            if self.start_in <= 0:
                self.start_in = None
                if "start_dropped" in self.script:
                    self.sticky |= 1 << R.S_ERR_START_DROPPED
                    self.admit = True                  # the DSP side admits, run_active stays low
                    if "start_dropped_twice" not in self.script:
                        self.script.discard("start_dropped")
                elif "never_start" not in self.script:
                    self.run_active = self.admit = True

    def _status(self) -> int:
        return (self.sticky | (self.run_active << R.S_RUN_ACTIVE) | (self.admit << R.S_DSP_ADMIT)
                | (("flush_stuck" in self.script and self.run_active) << R.S_FLUSH_BUSY))

    def _diag(self) -> int:
        idle = "never_idle" not in self.script and not self.run_active and self.start_in is None
        pend = self.start_in is not None
        return (idle << 7) | (pend << 3) | (pend << 4)

    def read(self, addr: int) -> int:
        self._tick()
        off = addr - self.base
        if off == R.STATUS:
            return self._status()
        if off == R.DIAG:
            return self._diag()
        if off == R.WR_BASE:
            return self.wr_base ^ (0x200 if "wr_base_mismatch" in self.script else 0)
        if off == R.RUN_BASE:
            return self.run_base ^ (0x200 if "run_base_mismatch" in self.script else 0)
        if off == R.FINAL_ADDR:
            return self.final_addr
        if off == R.NUM_CH:
            return self.num_ch
        if off == R.GEOMETRY:
            return (8 << 24) | (4 << 16) | (8 << 8) | 16          # 16 beats x 32 B = 512 B banks
        if R.ACCEPTED <= off < R.ACCEPTED + 4 * self.num_ch:
            return self.accepted[(off - R.ACCEPTED) // 4]
        if R.REJECTED <= off < R.REJECTED + 4 * self.num_ch:
            return self.rejected[(off - R.REJECTED) // 4]
        return 0

    def write(self, addr: int, value: int) -> None:
        off = addr - self.base
        if off == R.STATUS:
            self.sticky &= ~(value & R.STICKY_MASK)
        elif off == R.WR_BASE:
            self.wr_base = value
        elif off == R.BASE_RESET:
            self.base_resets += 1
            idle = "never_idle" not in self.script and not self.run_active and self.start_in is None
            if "base_busy" in self.script or not idle:
                self.sticky |= 1 << R.S_ERR_BASE_BUSY
                return
            self.sticky = 0
            self.accepted = [0] * self.num_ch
            self.rejected = [0] * self.num_ch
            self.run_base = self.ptr = self.wr_base
            self.admit = False
            self.start_in = self.start_delay
        elif off == R.FLUSH:
            self.flushes += 1
            if "flush_refused" in self.script or not self.run_active:
                self.script.discard("flush_refused")      # one refusal: the run's own FLUSH
                self.sticky |= 1 << R.S_ERR_FLUSH_REFUSED
                return
            if "flush_stuck" in self.script:
                return
            total = sum(self.accepted)
            self.final_addr = self.run_base + 32 * (-(-total // 4))
            self.run_active = self.admit = False
            self.sticky |= 1 << R.S_WRITE_DONE
            if "bresp" in self.script:
                self.sticky |= 1 << R.S_BRESP_ERR
            if self.on_flush is not None:
                self.on_flush(self)
        elif off == R.RD_BASE:
            self.rd_base = value
        elif off == R.RD_SIZE:
            self.rd_size = value

    def dma_prepare(self, nbytes: int):
        if self.dma is not None:
            raise RuntimeError("an S2MM transfer is already in flight")
        self.dma = ("armed", int(nbytes))
        return self.dma

    def dma_wait(self, buf, nbytes: int) -> bytes:
        self.dma = None
        if "dma_error" in self.script:
            raise RuntimeError("S2MM_DMASR error during the drain: 0x00000040")
        data = bytes(self.mem[self.rd_base:self.rd_base + nbytes])
        return data + bytes(nbytes - len(data))


def _h(data) -> str:
    return hashlib.sha256(bytes(data)).hexdigest()[:16]


class TraceDriver:
    """Wraps a Driver and records every seam operation as a tuple `(op, addr, value or length,
    data hash)`: the operation sequence itself, not a count (plan P4 v2 §8; a `CountingDriver`
    tally cannot tell two different sequences of equal length apart). Reads record the value or
    the hash of the bytes they returned, writes what they wrote. Every other attribute (`.sim`,
    `.remote`, `.board`, `host_base`) forwards to the wrapped driver."""

    def __init__(self, drv):
        self._drv = drv
        self.trace: list[tuple] = []

    def read32(self, addr):
        v = self._drv.read32(addr)
        self.trace.append(("read32", int(addr), int(v)))
        return v

    def write32(self, addr, value):
        self.trace.append(("write32", int(addr), int(value) & 0xFFFFFFFF))
        return self._drv.write32(addr, value)

    def read_block(self, addr, nbytes):
        data = self._drv.read_block(addr, nbytes)
        self.trace.append(("read_block", int(addr), int(nbytes), _h(data)))
        return data

    def write_block(self, addr, data):
        self.trace.append(("write_block", int(addr), len(data), _h(data)))
        return self._drv.write_block(addr, data)

    def read_host(self, offset, nbytes):
        data = self._drv.read_host(offset, nbytes)
        self.trace.append(("read_host", int(offset), int(nbytes), _h(data)))
        return data

    def __getattr__(self, name):
        return getattr(self._drv, name)


def program_digest(prog) -> str:
    """sha256 over everything `riscq.run` loads or addresses by name: the image bytes and entry, the
    symbol table (addresses and sizes), params, arrays, host arrays, the table slot codes and the
    envelope images. Two programs with equal digests load identically and are read identically."""
    h = hashlib.sha256()
    img = prog.image
    h.update(b"data"); h.update(bytes(img.data))
    h.update(b"entry%d" % int(img.entry))
    for name in sorted(img.symbols):
        addr, size = img.symbols[name]
        h.update(f"sym {name} {int(addr)} {int(size)}\n".encode())
    for name in sorted(prog.params):
        h.update(f"param {name} {prog.params[name]}\n".encode())
    for name in sorted(prog.arrays):
        h.update(f"array {name} {int(prog.arrays[name])}\n".encode())
    for name in sorted(prog.host_arrays):
        off, n = prog.host_arrays[name]
        h.update(f"host {name} {int(off)} {int(n)}\n".encode())
    for name in sorted(prog.tables):
        h.update(f"table {name} {[[int(c) for c in slot] for slot in prog.tables[name]]}\n".encode())
    for chan in sorted(prog.envelopes):
        for line0, lines in prog.envelopes[chan]:
            import numpy as np
            arr = np.ascontiguousarray(lines, dtype="<u4")
            h.update(f"env {int(chan)} {int(line0)} {arr.shape}\n".encode())
            h.update(arr.tobytes())
    return h.hexdigest()
