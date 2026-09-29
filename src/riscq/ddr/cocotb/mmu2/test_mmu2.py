"""G1' unit tests for the drain engine (`mmu2`): AXI4 read master (256-bit) -> AXI-Stream (256-bit, TLAST).

P3a: the DUT is the SpinalHDL `riscq.ddr.DrainEngine`, generated under the vendored module/port names by
`riscq.ddr.sim.GenUplinkUnits` (+ outputs `start_rejected`, `idle`, `dbg_fifo_used`; maxBytes = 32 MiB). The
bench and the scoreboard are the vendored-RTL suite's. What changed is the oracle for the three fixed
non-conformances (see src/riscq/ddr/CONTRACT.md):
  F1  bursts are page-bounded: min(remaining, 256, beats to the next 4 KiB boundary) -> ref_bursts();
  F2  base/size are latched at start and a start is accepted only after the previous chunk's TLAST;
  F3  size 0 / not a multiple of 32 / > maxBytes, or an unaligned base, is rejected (no busy, no AR).
The two former expect_fail cases are positive tests of F1 and F2.

Bench:
  * AXI read side  -> Mmu2AxiSlave (cocotbext-axi AxiRamRead fork: records every AR burst, counts 4 KiB crossings
                      instead of asserting on them, injects RRESP=SLVERR + marker data for an address range)
                      with random ARREADY / RVALID stalls (cocotbext pause generators).
  * AXIS side      -> cocotbext-axi AxiStreamSink with random TREADY back-pressure, plus a falling-edge beat monitor
                      (per-beat TLAST, AXIS/AR stability while stalled, done/busy protocol).
  * Reference model in Python: expected bursts (page-bounded, see ref_bursts) and expected
    stream bytes = preloaded memory[base : base + (size//32)*32].
All random stimulus comes from Python `random` seeded through ddrtb.seed (SEED env var, default 20260823); every
test re-seeds, so each test is reproducible on its own.
"""
import logging
import random

import cocotb
from cocotb.triggers import RisingEdge, FallingEdge
from cocotbext.axi import AxiReadBus, AxiRamRead, AxiStreamBus, AxiStreamSink
from cocotbext.axi.constants import AxiResp, AxiBurstType

from ddrtb import start_clock, reset_low, seed

BEAT = 32                     # bytes per 256-bit beat
MEM_SIZE = 1 << 20            # 1 MiB backing memory, preloaded with seeded random bytes
CLK_NS = 4
MAX_BURST_BEATS = 256
FIFO_DEPTH = 16               # mmu2 FIFO_ADDR_BITS=4
FIFO_AFULL = FIFO_DEPTH - 4   # async_fifo_same almost_full = used >= DEPTH-4


# ----------------------------------------------------------------------------------------------------------------
# Reference model
# ----------------------------------------------------------------------------------------------------------------
def ref_bursts(base, size):
    """Burst plan (F1): min(256, remaining, beats to the next 4 KiB boundary) per AR, contiguous INCR."""
    words = size // BEAT
    addr, out = base, []
    while words > 0:
        n = min(MAX_BURST_BEATS, words, (0x1000 - (addr & 0xFFF)) // BEAT)
        out.append((addr, n - 1))
        addr += n * BEAT
        words -= n
    return out


def ref_stream(mem, base, size):
    words = size // BEAT
    return bytes(mem[base:base + words * BEAT])


def crosses_4k(addr, nbeats):
    return (0x1000 - (addr & 0xFFF)) < nbeats * BEAT


def err_marker(addr):
    """Marker payload the slave returns on an SLVERR beat (distinct from the memory contents)."""
    return (0xBAD0_0000 | ((addr >> 5) & 0xFFFF)).to_bytes(4, 'little') * (BEAT // 4)


def beats_bytes(beats):
    return b''.join(x[0].to_bytes(BEAT, 'little') for x in beats)


# ----------------------------------------------------------------------------------------------------------------
# AXI slave fork: burst recorder, 4 KiB crossing counter, SLVERR injection
# ----------------------------------------------------------------------------------------------------------------
class Mmu2AxiSlave(AxiRamRead):
    def __init__(self, bus, clock, reset, size):
        self.bursts = []          # (araddr, arlen, arsize, arburst, arid)
        self.crossings = []       # (araddr, arlen) of bursts that cross a 4 KiB boundary
        self.err_ranges = []      # [lo, hi) byte ranges answered with RRESP=SLVERR + marker data
        self.err_beats = []       # addresses of beats that were answered with SLVERR
        super().__init__(bus, clock, reset, reset_active_level=False, size=size)

    def _in_err(self, addr):
        return any(lo <= addr < hi for lo, hi in self.err_ranges)

    async def _process_read(self):
        while True:
            ar = await self.ar_channel.recv()
            arid = int(getattr(ar, 'arid', 0))
            addr = int(ar.araddr)
            length = int(ar.arlen)
            size = int(ar.arsize)
            burst = AxiBurstType(int(ar.arburst))
            self.bursts.append((addr, length, size, burst, arid))
            num_bytes = 2 ** size
            assert 0 < num_bytes <= self.byte_lanes
            if burst == AxiBurstType.INCR and crosses_4k(addr, length + 1):
                self.crossings.append((addr, length))
            cur = (addr // num_bytes) * num_bytes
            for n in range(length + 1):
                r = self.r_channel._transaction_obj()
                r.rid = arid
                r.rlast = (n == length)
                if self._in_err(cur):
                    r.rresp = AxiResp.SLVERR
                    data = err_marker(cur)
                    self.err_beats.append(cur)
                else:
                    r.rresp = AxiResp.OKAY
                    data = self.read(cur % self.size, self.byte_lanes)
                r.rdata = int.from_bytes(data, 'little')
                await self.r_channel.send(r)
                if burst != AxiBurstType.FIXED:
                    cur += num_bytes


# ----------------------------------------------------------------------------------------------------------------
# Stall generators (seeded)
# ----------------------------------------------------------------------------------------------------------------
def pause_gen(p_pause, max_run=8):
    """Bursty random pause pattern: runs of 1..max_run paused / un-paused cycles."""
    while True:
        paused = random.random() < p_pause
        for _ in range(random.randint(1, max_run)):
            yield paused


def const_gen(val):
    while True:
        yield val


STALL_PROFILES = {
    'none':  dict(ar=0.0, r=0.0, axis=0.0, run=1),
    'light': dict(ar=0.3, r=0.3, axis=0.3, run=4),
    'heavy': dict(ar=0.6, r=0.6, axis=0.7, run=12),
}


# ----------------------------------------------------------------------------------------------------------------
# Falling-edge protocol monitor + recorder (samples the settled post-edge values half a cycle after each posedge)
# ----------------------------------------------------------------------------------------------------------------
class Rec:
    def __init__(self):
        self.cycle = 0
        self.beats = []          # (tdata int, tlast, cycle) at every AXIS handshake
        self.ar = []             # (araddr, arlen, arsize, arburst, arid, cycle) at every AR handshake
        self.rbeats = 0
        self.rerr = 0
        self.ar_stall = 0        # cycles arvalid && !arready
        self.r_stall = 0         # cycles rvalid && !rready  (DUT-side back-pressure: FIFO almost full)
        self.axis_stall = 0      # cycles tvalid && !tready
        self.done_cycles = []
        self.busy = []           # busy per cycle
        self.errors = []         # protocol violations (asserted by the tests at check time)

    def busy_rises(self, c0, c1):
        return [c for c in range(max(c0, 1), min(c1, len(self.busy) - 1) + 1) if self.busy[c] and not self.busy[c - 1]]


async def monitor(dut, rec):
    prev_axis = None   # (tvalid, tready, tdata, tlast)
    prev_ar = None     # (arvalid, arready, fields)
    prev_done = 0
    while True:
        await FallingEdge(dut.clk)
        c = rec.cycle
        busy = int(dut.busy.value)
        done = int(dut.done.value)
        # ---- AXIS ----
        tv, tr = int(dut.m_axis_tvalid.value), int(dut.m_axis_tready.value)
        tl = int(dut.m_axis_tlast.value)
        td = int(dut.m_axis_tdata.value) if tv else None
        if prev_axis and prev_axis[0] and not prev_axis[1]:
            if not (tv == 1 and td == prev_axis[2] and tl == prev_axis[3]):
                rec.errors.append(f"cycle {c}: AXIS payload/valid changed while stalled")
        if tl and not tv:
            rec.errors.append(f"cycle {c}: TLAST without TVALID")
        if tv and tr:
            rec.beats.append((td, tl, c))
        if tv and not tr:
            rec.axis_stall += 1
        prev_axis = (tv, tr, td, tl)
        # ---- AR ----
        av, ar = int(dut.arvalid.value), int(dut.arready.value)
        fields = (int(dut.araddr.value), int(dut.arlen.value), int(dut.arsize.value),
                  int(dut.arburst.value), int(dut.arid.value))
        if prev_ar and prev_ar[0] and not prev_ar[1]:
            if not (av == 1 and fields == prev_ar[2]):
                rec.errors.append(f"cycle {c}: AR fields/valid changed while stalled")
        if av and not busy:
            rec.errors.append(f"cycle {c}: ARVALID while !busy")
        if av and ar:
            rec.ar.append(fields + (c,))
        if av and not ar:
            rec.ar_stall += 1
        prev_ar = (av, ar, fields)
        # ---- R ----
        rv, rr = int(dut.rvalid.value), int(dut.rready.value)
        if rv and rr:
            rec.rbeats += 1
            if int(dut.rresp.value) != 0:
                rec.rerr += 1
        if rv and not rr:
            rec.r_stall += 1
        # ---- control ----
        if done:
            rec.done_cycles.append(c)
            if busy:
                rec.errors.append(f"cycle {c}: done asserted while busy")
            if prev_done:
                rec.errors.append(f"cycle {c}: done wider than one cycle")
        prev_done = done
        rec.busy.append(busy)
        rec.cycle += 1


# ----------------------------------------------------------------------------------------------------------------
# Bench
# ----------------------------------------------------------------------------------------------------------------
class Bench:
    def __init__(self, dut, rec, slave, sink, mem):
        self.dut, self.rec, self.slave, self.sink, self.mem = dut, rec, slave, sink, mem

    def set_stalls(self, profile):
        p = STALL_PROFILES[profile]
        self.slave.ar_channel.set_pause_generator(pause_gen(p['ar'], p['run']) if p['ar'] else const_gen(False))
        self.slave.r_channel.set_pause_generator(pause_gen(p['r'], p['run']) if p['r'] else const_gen(False))
        self.sink.set_pause_generator(pause_gen(p['axis'], p['run']) if p['axis'] else const_gen(False))

    def hold_axis(self, paused):
        """Force TREADY low (paused=True) or high."""
        self.sink.set_pause_generator(const_gen(bool(paused)))

    def rand_base(self, size, align=BEAT):
        hi = (MEM_SIZE - size) // align
        return random.randrange(0, hi + 1) * align

    async def pulse_start(self, base, size, immediate=False):
        """Drive base/size and a single-cycle start pulse. Returns the monitor cycle count before the pulse
        (a lower bound for locating the resulting busy rising edge). immediate=True drives start in the current
        (falling-edge) time step so the very next posedge samples it."""
        c0 = self.rec.cycle
        self.dut.base_addr.value = base
        self.dut.size_bytes.value = size
        if not immediate:
            await RisingEdge(self.dut.clk)
        self.dut.start.value = 1
        await RisingEdge(self.dut.clk)
        self.dut.start.value = 0
        return c0

    async def wait_cycles(self, n):
        for _ in range(n):
            await FallingEdge(self.dut.clk)

    async def wait_done(self, timeout):
        """Poll the DUT `done` pin at falling edges (post-edge value). Independent of coroutine ordering."""
        for _ in range(timeout):
            await FallingEdge(self.dut.clk)
            if int(self.dut.done.value):
                return
        raise AssertionError(f"timeout ({timeout} cycles) waiting for done")

    async def wait_tlast(self, timeout):
        """Poll for a TLAST handshake (tvalid && tready && tlast) at falling edges."""
        d = self.dut
        for i in range(timeout):
            if i:                      # the handshake may already be pending at the falling edge we are on
                await FallingEdge(d.clk)
            if int(d.m_axis_tvalid.value) and int(d.m_axis_tready.value) and int(d.m_axis_tlast.value):
                return
        raise AssertionError(f"timeout ({timeout} cycles) waiting for TLAST")

    async def read(self, base, size, timeout=None, wait_drain=True):
        """One mmu2 transaction: start -> done -> TLAST (+2 settle cycles so monitor and sink have recorded
        everything). Returns dict with beats/bursts/cycles."""
        words = size // BEAT
        if timeout is None:
            timeout = 40 * words + 2000
        nb0, na0, nd0 = len(self.rec.beats), len(self.rec.ar), len(self.rec.done_cycles)
        c0 = await self.pulse_start(base, size)
        await self.wait_done(timeout)
        if wait_drain:
            await self.wait_tlast(timeout)
        await self.wait_cycles(2)
        d = self.rec.done_cycles[nd0]
        lasts = [x[2] for x in self.rec.beats[nb0:] if x[1]]
        return dict(base=base, size=size, c0=c0, done=d, tlast=(lasts[0] if lasts else None),
                    beats=self.rec.beats[nb0:], ar=self.rec.ar[na0:], nb0=nb0, na0=na0)

    def check_chunk(self, tx, expect_bytes=None):
        """Self-check one transaction against the reference model."""
        rec = self.rec
        base, size, beats, ar = tx['base'], tx['size'], tx['beats'], tx['ar']
        words = size // BEAT
        assert not rec.errors, "protocol violations: " + "; ".join(rec.errors[:5])
        # beat count / TLAST placement
        assert len(beats) == words, f"beats delivered {len(beats)} != size/32 = {words}"
        lasts = [i for i, b in enumerate(beats) if b[1]]
        assert lasts == [words - 1], f"TLAST on beats {lasts}, expected only on beat {words - 1}"
        # data byte-exact vs preloaded memory (or an explicit expectation)
        got = beats_bytes(beats)
        exp = ref_stream(self.mem, base, size) if expect_bytes is None else expect_bytes
        if got != exp:
            first = next(i for i in range(len(exp)) if got[i] != exp[i])
            raise AssertionError(f"stream data mismatch at byte {first} (beat {first // BEAT}): "
                                 f"got {got[first:first+8].hex()} exp {exp[first:first+8].hex()}")
        # cocotbext sink cross-check (frames are delimited by TLAST)
        frame = self.sink.recv_nowait()
        assert bytes(frame.tdata) == exp, "AxiStreamSink frame differs from beat monitor / reference"
        # AR burst plan: arlen <= 255, contiguous, sizes, INCR, id 0
        plan = [(a[0], a[1]) for a in ar]
        assert plan == ref_bursts(base, size), f"burst plan {plan[:4]}... != ref {ref_bursts(base, size)[:4]}..."
        assert all(a[1] <= 255 for a in ar)
        assert not any(crosses_4k(a[0], a[1] + 1) for a in ar), "F1: an AR crosses a 4 KiB boundary"
        assert all(a[2] == 5 and a[3] == 1 and a[4] == 0 for a in ar), "arsize/arburst/arid wrong"
        assert sum(a[1] + 1 for a in ar) == words
        slave_plan = [(b[0], b[1]) for b in self.slave.bursts[tx['na0']:tx['na0'] + len(ar)]]
        assert slave_plan == plan, "slave saw different bursts than the monitor"
        # busy/done semantics: busy rises once after start, stays 1, falls in the done cycle; done is 1 cycle
        d = tx['done']
        rises = rec.busy_rises(tx['c0'], d)
        assert len(rises) == 1, f"busy rising edges in window: {rises}"
        s = rises[0]
        dones = [c for c in rec.done_cycles if tx['c0'] < c <= d]
        assert dones == [d], f"done pulses in window: {dones}"
        assert all(rec.busy[c] == 1 for c in range(s, d)), "busy dropped before done"
        assert rec.busy[d] == 0, "busy still set in the done cycle"
        # monitor convention: done is sampled post-edge at F(E); the last AXIS handshake is sampled at F(E') for the
        # transfer at E'+1, so done (last R beat into the FIFO at E) is never later than the last AXIS beat sample.
        assert d <= beats[-1][2], "done must not come after the last AXIS beat"
        tx['start'] = s
        return got


async def make_bench(dut, profile='none', salt=0):
    """Fresh bench: clock, reset, preloaded memory, slave, sink, monitor. `salt` perturbs the stimulus RNG (not the
    memory image) so that tests sharing the same call pattern draw different bases/stall patterns."""
    s = seed(dut)
    await start_clock(dut.clk, CLK_NS)
    dut.start.value = 0
    dut.base_addr.value = 0
    dut.size_bytes.value = 0
    mem = bytearray(random.randbytes(MEM_SIZE))
    if salt:
        random.seed(s * 1000003 + salt)
    slave = Mmu2AxiSlave(AxiReadBus.from_entity(dut), dut.clk, dut.rst_n, size=MEM_SIZE)
    slave.write(0, mem)
    sink = AxiStreamSink(AxiStreamBus.from_prefix(dut, "m_axis"), dut.clk, dut.rst_n, reset_active_level=False)
    sink.log.setLevel(logging.WARNING)     # the frame dump of a 64 KiB frame would flood the log
    slave.log = logging.getLogger("cocotb.mmu2.axi_slave")   # own logger (default one aliases dut._log)
    slave.log.setLevel(logging.WARNING)    # silence the per-burst "Read burst" lines
    rec = Rec()
    cocotb.start_soon(monitor(dut, rec))
    await reset_low(dut.rst_n, dut.clk)
    bench = Bench(dut, rec, slave, sink, mem)
    bench.set_stalls(profile)
    await bench.wait_cycles(3)
    return bench


async def reset_dut(bench):
    await reset_low(bench.dut.rst_n, bench.dut.clk)
    await bench.wait_cycles(3)


def assert_idle_after(bench, cycles_checked):
    """No AXIS beat, no AR, no done for the last `cycles_checked` monitor cycles."""
    rec = bench.rec
    c0 = rec.cycle - cycles_checked
    assert not [b for b in rec.beats if b[2] >= c0], "AXIS beat after the transaction ended"
    assert not [a for a in rec.ar if a[5] >= c0], "AR issued after the transaction ended"
    assert not [d for d in rec.done_cycles if d >= c0], "done after the transaction ended"


# ================================================================================================================
# (1) randomized AR / R / AXIS stalls, mandatory sizes
# ================================================================================================================
async def _rand_stalls(dut, size, profile='heavy'):
    b = await make_bench(dut, profile, salt=size)
    base = b.rand_base(size)
    tx = await b.read(base, size)
    b.check_chunk(tx)
    await b.wait_cycles(50)
    assert_idle_after(b, 50)
    assert len(b.rec.beats) == size // BEAT
    r = b.rec
    dut._log.info(f"size={size} base=0x{base:x} bursts={len(tx['ar'])} 4k-crossings={len(b.slave.crossings)} "
                  f"stalls: ar={r.ar_stall} r(rready low)={r.r_stall} axis={r.axis_stall} | "
                  f"busy@{tx['start']} done@{tx['done']} tlast@{tx['tlast']} "
                  f"(done leads TLAST by {tx['tlast'] - tx['done']} cyc)")
    # stimulus coverage (probabilistic, thresholds chosen so a miss is < 1e-6 for the heavy profile)
    if size // BEAT >= FIFO_DEPTH:
        assert r.axis_stall > 0, "AXIS back-pressure was never exercised"
    if size // BEAT >= 256:
        assert r.r_stall > 0, "R back-pressure (FIFO almost-full -> rready low) was never exercised"
    if len(tx['ar']) >= 4:
        assert r.ar_stall > 0, "ARREADY stall was never exercised"


@cocotb.test()
async def test_rand_stalls_size32(dut):
    await _rand_stalls(dut, 32)


@cocotb.test()
async def test_rand_stalls_size64(dut):
    await _rand_stalls(dut, 64)


@cocotb.test()
async def test_rand_stalls_size4096(dut):
    await _rand_stalls(dut, 4096)


@cocotb.test()
async def test_rand_stalls_size8224(dut):
    await _rand_stalls(dut, 8192 + 32)


@cocotb.test()
async def test_rand_stalls_size65536(dut):
    await _rand_stalls(dut, 65536)


@cocotb.test()
async def test_rand_stalls_light_mixed_sizes(dut):
    """Light stall profile, 12 random aligned sizes (32 B .. 64 KiB) at random bases, sequential."""
    b = await make_bench(dut, 'light')
    for _ in range(12):
        size = random.randint(1, 2048) * BEAT
        tx = await b.read(b.rand_base(size), size)
        b.check_chunk(tx)


@cocotb.test()
async def test_no_stalls_throughput(dut):
    """No stalls: stream must run close to 1 beat/cycle (loose bound: words + 10*bursts + 20 cycles)."""
    b = await make_bench(dut, 'none')
    size = 16384
    tx = await b.read(b.rand_base(size), size)
    b.check_chunk(tx)
    span = tx['beats'][-1][2] - tx['beats'][0][2] + 1
    words, nb = size // BEAT, len(tx['ar'])
    dut._log.info(f"no-stall stream span {span} cycles for {words} beats in {nb} bursts")
    assert span <= words + 10 * nb + 20, f"throughput too low: {span} cycles for {words} beats"
    assert b.rec.r_stall == 0 and b.rec.axis_stall == 0


# ================================================================================================================
# (2) burst boundaries: > 256 beats, 4 KiB crossings
# ================================================================================================================
@cocotb.test()
async def test_burst_split_over_256_beats(dut):
    """257 / 512 / 2048 / 300 beats: arlen <= 127 (a 4 KiB page is 128 beats), page-bounded contiguous plan, sum ==
    beats, data exact, no crossing recorded by the slave."""
    b = await make_bench(dut, 'light')
    for size in (8192 + 32, 16384, 65536, 300 * BEAT):
        n0 = len(b.slave.crossings)
        tx = await b.read(b.rand_base(size), size)
        b.check_chunk(tx)
        assert max(a[1] for a in tx['ar']) <= 127
        assert len(tx['ar']) == len(ref_bursts(tx['base'], size))
        assert len(b.slave.crossings) == n0, "F1: the slave saw a 4 KiB crossing"
        dut._log.info(f"size={size}: {len(tx['ar'])} bursts, none crossed a 4 KiB boundary")


@cocotb.test()
async def test_burst_crossing_4k_small(dut):
    """A short read straddling a 4 KiB boundary (base = n*4 KiB - 128, 8 beats): F1 splits it at the page into
    two ARs of 4 beats, neither crossing; data exact. (Vendored: one AR that crossed the boundary.)"""
    b = await make_bench(dut, 'light')
    for _ in range(3):
        base = (random.randrange(1, MEM_SIZE // 0x1000) * 0x1000) - 4 * BEAT
        size = 8 * BEAT
        tx = await b.read(base, size)
        b.check_chunk(tx)
        assert [(a[0], a[1]) for a in tx['ar']] == [(base, 3), (base + 4 * BEAT, 3)]
        assert not b.slave.crossings, f"4 KiB crossings recorded: {b.slave.crossings}"


@cocotb.test()
async def test_axi_no_4k_crossing_strict(dut):
    """STRICT AXI rule (A3.4.1): a burst must not cross a 4 KiB boundary. Positive test of F1 (the vendored mmu2 issued
    one 8 KiB burst here and this case was expect_fail): 8 KiB aligned -> exactly two 128-beat bursts; plus 20 random
    unaligned bases/sizes, none of whose bursts may cross."""
    b = await make_bench(dut, 'none')
    tx = await b.read(0x10000, 8192)
    b.check_chunk(tx)
    assert [(a[0], a[1]) for a in tx['ar']] == [(0x10000, 127), (0x11000, 127)]
    for _ in range(20):
        size = random.randint(1, 1024) * BEAT
        tx = await b.read(b.rand_base(size), size)
        b.check_chunk(tx)
    assert not b.slave.crossings, f"AXI 4 KiB boundary crossed by bursts {b.slave.crossings}"


# ================================================================================================================
# (3) size handling (F3): invalid requests are rejected explicitly; the engine stays usable without a reset
# ================================================================================================================
async def _size_rejected(dut, size, base=None, watch=300, b=None):
    if b is None:
        b = await make_bench(dut, 'none', salt=size + 1)
    if base is None:
        base = b.rand_base(4096)
    rej = {"n": 0}
    nd0, na0, nb0 = len(b.rec.done_cycles), len(b.rec.ar), len(b.rec.beats)

    async def count_rej():
        while True:
            await FallingEdge(dut.clk)
            rej["n"] += int(dut.start_rejected.value)
    t = cocotb.start_soon(count_rej())
    c0 = await b.pulse_start(base, size)
    await b.wait_cycles(watch)
    t.cancel()
    r = b.rec
    assert rej["n"] == 1, f"start_rejected pulsed {rej['n']} times (expected exactly once)"
    assert not r.busy_rises(c0, r.cycle) and not any(r.busy[c0:]), "busy rose for an invalid request"
    assert len(r.done_cycles) == nd0, "done fired for an invalid request"
    assert len(r.ar) == na0, "an AR burst was issued for an invalid request"
    assert len(r.beats) == nb0, "AXIS beats for an invalid request"
    assert int(dut.idle.value) == 1 and int(dut.m_axis_tvalid.value) == 0 and int(dut.arvalid.value) == 0
    assert not r.errors
    dut._log.info(f"size={size} base=0x{base:x}: rejected (1 start_rejected pulse), no busy/AR/beats/done")
    # the engine is immediately usable again -- no reset needed (the vendored mmu2 hung until rst_n)
    tx = await b.read(b.rand_base(4096), 4096)
    b.check_chunk(tx)
    return b


@cocotb.test()
async def test_size0_hangs_busy(dut):
    """F3: size_bytes=0 is rejected (vendored: busy=1 forever, recovery only via rst_n)."""
    await _size_rejected(dut, 0)


@cocotb.test()
async def test_size16_hangs_busy(dut):
    """F3: size_bytes=16 (< 32) is rejected (vendored: the same hang as size 0)."""
    await _size_rejected(dut, 16)


@cocotb.test()
async def test_size48_truncates_to_one_beat(dut):
    """F3: size_bytes=48 (not a multiple of 32) is rejected (vendored: silently truncated to one 32-B beat)."""
    await _size_rejected(dut, 48)


@cocotb.test()
async def test_size_over_max_and_unaligned_base_rejected(dut):
    """F3: size > maxBytes (32 MiB + 32) and a base that is not 32-B aligned are rejected too."""
    b = await _size_rejected(dut, 0x200_0000 + BEAT)
    await _size_rejected(dut, 4 * BEAT, base=0x1010, b=b)


@cocotb.test()
async def test_aligned_sizes_sweep(dut):
    """Every aligned size 32..32*40 plus 255/256/257/511/512/513 beats: exact beats, TLAST, data (light stalls)."""
    b = await make_bench(dut, 'light')
    for words in list(range(1, 41)) + [255, 256, 257, 511, 512, 513]:
        size = words * BEAT
        tx = await b.read(b.rand_base(size), size)
        b.check_chunk(tx)


# ================================================================================================================
# (4) chunk sequencing
# ================================================================================================================
@cocotb.test()
async def test_chunk_sequence_4(dut):
    """4 chunks with different base/size, each started right after the previous chunk's TLAST handshake
    (done + drained), heavy random stalls on all three channels: no leakage, every chunk exact."""
    b = await make_bench(dut, 'heavy')
    sizes = [4096, 64, 8192 + 32, 32]
    random.shuffle(sizes)
    total = 0
    for size in sizes:
        tx = await b.read(b.rand_base(size), size)
        b.check_chunk(tx)
        total += size // BEAT
        assert len(b.rec.beats) == total, "beat count drifted across chunks (leak)"
    assert len(b.rec.done_cycles) == 4
    await b.wait_cycles(50)
    assert_idle_after(b, 50)


class RejCounter:
    """Counts `start_rejected` pulses (sampled at falling edges)."""
    def __init__(self, dut):
        self.n = 0
        self.cycles = []
        self.task = cocotb.start_soon(self._run(dut))

    async def _run(self, dut):
        c = 0
        while True:
            await FallingEdge(dut.clk)
            c += 1
            if int(dut.start_rejected.value):
                self.n += 1
                self.cycles.append(c)


@cocotb.test()
async def test_chunk_sequence_start_right_after_done(dut):
    """F2: 8 chunks with a different base AND size each, TREADY always 1, random AR/R stalls. A start driven in the
    cycle after `done` -- while the chunk's last beat is still in the FIFO -- is REJECTED (start_rejected, no second
    busy, no AR); the next chunk is then started right after the TLAST handshake and is exact. (Vendored: the early
    start was accepted and only happened to be exact because the FIFO held one beat and the size was unchanged.)"""
    b = await make_bench(dut, 'none')
    b.slave.ar_channel.set_pause_generator(pause_gen(0.3, 4))
    b.slave.r_channel.set_pause_generator(pause_gen(0.3, 4))
    rej = RejCounter(dut)
    txs = []
    for k in range(8):
        size = random.randint(2, 12) * BEAT
        base = b.rand_base(size)
        nb0, na0 = len(b.rec.beats), len(b.rec.ar)
        c0 = await b.pulse_start(base, size, immediate=(k > 0))
        await b.wait_done(2000)
        n_rej = rej.n
        # the last beat entered the FIFO at the edge that raised done and leaves at the next one, so this start,
        # sampled at that next edge, arrives before the TLAST handshake has completed: it must be refused
        await b.pulse_start(b.rand_base(size), BEAT, immediate=True)
        await FallingEdge(dut.clk)
        assert rej.n == n_rej + 1, "a start between done and TLAST was not rejected"
        for _ in range(100):                             # idle again right after the TLAST handshake
            if int(dut.idle.value):
                break
            await FallingEdge(dut.clk)
        assert int(dut.idle.value) == 1
        txs.append(dict(base=base, size=size, c0=c0, nb0=nb0, na0=na0))
    await b.wait_cycles(100)
    assert len(b.rec.done_cycles) == 8 and rej.n == 8
    for i, tx in enumerate(txs):
        tx['done'] = b.rec.done_cycles[i]
        words = tx['size'] // BEAT
        tx['beats'] = b.rec.beats[tx['nb0']:tx['nb0'] + words]
        tx['ar'] = b.rec.ar[tx['na0']:txs[i + 1]['na0'] if i + 1 < len(txs) else len(b.rec.ar)]
        b.check_chunk(tx)
    assert len(b.rec.beats) == sum(t['size'] for t in txs) // BEAT


async def _start_before_drain(dut):
    """Shared stimulus: chunk 1 (8 beats) fully parked in the FIFO with TREADY low; chunk 2 (6 beats) started the
    cycle after chunk 1's done; TREADY released only then. F2: that start is rejected."""
    b = await make_bench(dut, 'none')
    rej = RejCounter(dut)
    b.hold_axis(True)                                   # TREADY = 0: nothing drains
    base1, size1 = b.rand_base(4096), 8 * BEAT          # 8 beats < FIFO almost-full (12): done can fire
    base2, size2 = b.rand_base(4096), 6 * BEAT
    await b.pulse_start(base1, size1)
    await b.wait_done(500)
    assert not b.rec.beats, "no beat may have been delivered with TREADY low"
    assert int(dut.dbg_fifo_used.value) == 8
    na0 = len(b.rec.ar)
    await b.pulse_start(base2, size2, immediate=True)   # start sampled at the very next posedge after done
    b.hold_axis(False)
    await b.wait_cycles(100)
    return b, rej, na0, base1, size1, base2, size2


@cocotb.test()
async def test_chunk_start_at_done_before_drain_strict(dut):
    """STRICT (positive test of F2; was expect_fail on the vendored mmu2): start chunk 2 right after chunk 1's `done`
    while chunk 1's beats are still in the FIFO. Chunk 1 must come out whole: exactly 8 beats, byte-exact, TLAST
    on beat 7 and nowhere else."""
    b, rej, na0, base1, size1, base2, size2 = await _start_before_drain(dut)
    beats = b.rec.beats
    lasts = [i for i, x in enumerate(beats) if x[1]]
    assert len(beats) == 8 and beats_bytes(beats) == ref_stream(b.mem, base1, size1) and lasts == [7], \
        f"chunk 1 corrupted: {len(beats)} beats out, TLAST at {lasts} (expected 8 beats, TLAST at beat 7)"


@cocotb.test()
async def test_chunk_start_at_done_before_drain_mechanism(dut):
    """F2 mechanism (same stimulus): the early start pulses start_rejected exactly once, raises no busy and issues no
    AR; the FIFO drains to empty and the engine returns to idle; chunk 2 re-issued after the TLAST and a third chunk
    are exact, with no reset. (Vendored: 6 beats out with a misplaced TLAST, 8 beats stuck in the FIFO and every
    later chunk shifted by 8 beats until reset.)"""
    b, rej, na0, base1, size1, base2, size2 = await _start_before_drain(dut)
    assert rej.n == 1, f"start_rejected pulsed {rej.n} times"
    assert len(b.rec.ar) == na0, "the rejected start issued an AR"
    assert len(b.rec.done_cycles) == 1, "the rejected start produced a done"
    assert int(dut.busy.value) == 0 and int(dut.idle.value) == 1 and int(dut.dbg_fifo_used.value) == 0
    assert int(dut.m_axis_tvalid.value) == 0
    b.sink.recv_nowait()                               # chunk 1's frame (checked by the strict test)
    for base, size in ((base2, size2), (b.rand_base(4096), 4 * BEAT)):
        tx = await b.read(base, size)
        b.check_chunk(tx)
    assert not b.rec.errors


@cocotb.test()
async def test_done_precedes_stream_drain(dut):
    """`done`/`busy` report the last R beat entering the FIFO, not AXIS delivery: with TREADY low, done fires with
    zero beats delivered; the data then streams out exactly once TREADY returns. (RTL finding / wrapper contract)"""
    b = await make_bench(dut, 'none')
    b.hold_axis(True)
    base, size = b.rand_base(4096), 8 * BEAT
    nb0 = len(b.rec.beats)
    c0 = await b.pulse_start(base, size)
    await b.wait_done(500)
    assert not b.rec.beats and int(dut.busy.value) == 0 and int(dut.m_axis_tvalid.value) == 1
    assert int(dut.dbg_fifo_used.value) == 8
    await b.wait_cycles(20)
    b.hold_axis(False)
    await b.wait_tlast(200)
    await b.wait_cycles(2)
    d = b.rec.done_cycles[0]
    t = [x[2] for x in b.rec.beats if x[1]][0]
    tx = dict(base=base, size=size, c0=c0, done=d, tlast=t, beats=b.rec.beats[nb0:], ar=b.rec.ar, na0=0)
    b.check_chunk(tx)
    dut._log.info(f"done@{d}, TLAST@{t}: done led the last AXIS beat by {t - d} cycles")
    assert t - d > 20


# ================================================================================================================
# (5) RRESP ignored
# ================================================================================================================
@cocotb.test()
async def test_rresp_slverr_ignored(dut):
    """3-burst read (768 beats); the slave answers the whole 2nd burst with RRESP=SLVERR and marker data.
    mmu2 forwards every beat unchanged (rresp/rid are unused), completes with done, and TLAST is where it should be:
    an interconnect/DDR error is indistinguishable from data at this level (the wrapper's sticky rresp monitor is
    the only place it becomes visible)."""
    b = await make_bench(dut, 'heavy')
    base, size = b.rand_base(3 * 8192), 3 * 8192
    b.slave.err_ranges = [(base + 8192, base + 2 * 8192)]
    exp = ref_stream(b.mem, base, 8192) \
        + b''.join(err_marker(a) for a in range(base + 8192, base + 2 * 8192, BEAT)) \
        + ref_stream(b.mem, base + 2 * 8192, 8192)
    tx = await b.read(base, size)
    b.check_chunk(tx, expect_bytes=exp)
    assert len(b.slave.err_beats) == 256 and b.rec.rerr == 256
    assert len(tx['ar']) == len(ref_bursts(base, size))
    # and a clean read afterwards is unaffected
    b.slave.err_ranges = []
    tx = await b.read(b.rand_base(4096), 4096)
    b.check_chunk(tx)


@cocotb.test()
async def test_rresp_slverr_single_beat(dut):
    """SLVERR on one isolated beat in the middle of a burst: still forwarded as data, beat count exact."""
    b = await make_bench(dut, 'light')
    base, size = b.rand_base(4096), 4096
    bad = base + 37 * BEAT
    b.slave.err_ranges = [(bad, bad + BEAT)]
    exp = bytearray(ref_stream(b.mem, base, size))
    exp[37 * BEAT:38 * BEAT] = err_marker(bad)
    tx = await b.read(base, size)
    b.check_chunk(tx, expect_bytes=bytes(exp))
    assert b.rec.rerr == 1


# ================================================================================================================
# (6) start while busy is ignored; size_bytes is not latched
# ================================================================================================================
@cocotb.test()
async def test_start_while_busy_ignored(dut):
    """Extra start pulses (different base, same size) while busy are ignored (and, F2, reported on start_rejected):
    one transaction, one done, AR plan and data of the FIRST start only; a start after completion works normally."""
    b = await make_bench(dut, 'heavy')
    rej = RejCounter(dut)
    base, size = b.rand_base(4096), 4096
    nb0, na0 = len(b.rec.beats), len(b.rec.ar)
    c0 = await b.pulse_start(base, size)
    for _ in range(3):
        await b.wait_cycles(random.randint(5, 40))
        assert int(dut.busy.value) == 1
        await b.pulse_start(b.rand_base(size), size)       # base differs, size identical (size_bytes is live!)
    await b.wait_done(20000)
    await b.wait_tlast(20000)
    await b.wait_cycles(100)
    d = b.rec.done_cycles[0]
    t = [x[2] for x in b.rec.beats if x[1]][0]
    tx = dict(base=base, size=size, c0=c0, done=d, tlast=t, beats=b.rec.beats[nb0:], ar=b.rec.ar[na0:], na0=na0)
    b.check_chunk(tx)
    assert len(b.rec.done_cycles) == 1 and rej.n == 3
    assert_idle_after(b, 100)
    tx = await b.read(b.rand_base(8192), 8192)
    b.check_chunk(tx)


@cocotb.test()
async def test_size_bytes_not_latched(dut):
    """F2: base_addr AND size_bytes are latched at start. Growing size_bytes mid-transfer (4096 -> 8192 after ~20
    beats) and moving base_addr change nothing: 128 beats, TLAST on the 128th, one done, busy falls. (Vendored:
    size_bytes was sampled live -> 128 beats with NO TLAST and busy stuck at 1 until reset.)"""
    b = await make_bench(dut, 'none')
    base = b.rand_base(8192)
    nb0 = len(b.rec.beats)
    c0 = await b.pulse_start(base, 4096)
    while b.rec.rbeats < 20:
        await FallingEdge(dut.clk)
    dut.size_bytes.value = 8192
    dut.base_addr.value = b.rand_base(8192)
    await b.wait_cycles(1500)
    r = b.rec
    assert [(a[0], a[1]) for a in r.ar] == ref_bursts(base, 4096), "AR plan should follow the size at start"
    assert len(r.beats[nb0:]) == 128, f"expected 128 beats streamed, got {len(r.beats[nb0:])}"
    assert [i for i, x in enumerate(r.beats[nb0:]) if x[1]] == [127], "TLAST must sit on beat 127 only"
    rises = r.busy_rises(c0, r.cycle)
    assert len(r.done_cycles) == 1 and len(rises) == 1 and r.busy[-1] == 0, "expected one done and busy low"
    assert beats_bytes(r.beats[nb0:]) == ref_stream(b.mem, base, 4096)
    assert int(dut.idle.value) == 1
    b.sink.recv_nowait()
    tx = await b.read(b.rand_base(4096), 4096)
    b.check_chunk(tx)


@cocotb.test()
async def test_reset_mid_transfer_recovers(dut):
    """rst_n asserted in the middle of a long transfer: all state clears (busy=0, FIFO empty, no beats/AR after reset
    until a new start); the next transaction is exact. Bench-level robustness for the wrapper's reset path."""
    b = await make_bench(dut, 'light')
    base = b.rand_base(65536)
    await b.pulse_start(base, 65536)
    while b.rec.rbeats < 700:
        await FallingEdge(dut.clk)
    await reset_dut(b)
    assert int(dut.busy.value) == 0 and int(dut.m_axis_tvalid.value) == 0 and int(dut.dbg_fifo_used.value) == 0
    c0 = b.rec.cycle
    await b.wait_cycles(50)
    assert not [x for x in b.rec.beats if x[2] >= c0] and not [a for a in b.rec.ar if a[5] >= c0]
    b.rec.errors.clear()                 # the reset legitimately dropped an in-flight AR/AXIS handshake
    b.slave.bursts.clear()
    b.rec.ar.clear()
    b.sink.clear()
    tx = await b.read(b.rand_base(8192 + 32), 8192 + 32)
    b.check_chunk(tx)
