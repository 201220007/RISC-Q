"""G1 'writer' suite -- circular_buffer_axi_writer (forks C, C2, C3, C4).

DUT           : circular_buffer_axi_writer (inside writer_tb_top.sv together with the REAL
                circular_buffer3 + cbuf_ram_read_wider, so the read-side contract is the board RTL).
AXI slave     : cocotbext.axi AxiRamWrite, 256-bit data / 32-bit addr / 4-bit id, with RANDOM
                backpressure on AW, W and B (pause generators, seeded).
Reference     : Python scoreboard (class Scoreboard) that tracks the cbuf RAM image word-by-word from
                the accepted writes, snapshots the expected beats at every AW handshake, checks every
                W beat, every burst's AxiRam contents, AW contiguity, final_addr, current_user_done,
                addr_fault and the full in-order delivery of the written word stream.
Protocol      : WriterTB._monitor samples at every falling edge (settled values) and asserts AXI A3.2.1
                stability (AW and W payload + VALID hold while !READY), WLAST position, full WSTRB, and
                writer_idle == (state == ST_IDLE)  (Fork C2).

All random stimulus derives from ddrtb.seed() (env SEED overrides) and is reproducible.
"""
import math
import random

import cocotb
from cocotb.triggers import RisingEdge, FallingEdge, Timer
from cocotbext.axi import AxiWriteBus, AxiRamWrite

from ddrtb import start_clock, reset_low, seed

# ---------------------------------------------------------------- constants (must match Makefile/wrapper)
WRAP_LIMIT   = 63999                 # -DSIM_WRAP_LIMIT (0xF9FF)
WRAP_SIZE    = WRAP_LIMIT + 1        # 0xFA00
ADDR_WIDTH   = 4
BEATS_PER_BANK = 1 << ADDR_WIDTH     # 16
WORDS_PER_BEAT = 4                   # 256 / 64
WORDS_PER_BANK = BEATS_PER_BANK * WORDS_PER_BEAT   # 64
BEAT_BYTES   = 32
BANK_BYTES   = BEATS_PER_BANK * BEAT_BYTES          # 512
MEM_SIZE     = 0x20000
ST_IDLE, ST_AW, ST_READ_WRITE, ST_WAIT_B = 0, 1, 2, 3
CLK_PERIOD_NS = 4
FLUSH_QUIET  = 8                     # plan v3 B.3 flushQuiet
WORD_MASK    = (1 << 64) - 1


def beat_int(words):
    """4 x 64-bit words -> 256-bit beat integer (word 0 in bits [63:0] = lowest byte address)."""
    v = 0
    for j, w in enumerate(words):
        v |= (w & WORD_MASK) << (64 * j)
    return v


def beat_bytes(words):
    return b"".join((w & WORD_MASK).to_bytes(8, "little") for w in words)


def pause_gen(rng, p):
    """Infinite generator of per-cycle pause decisions for cocotbext-axi (True = not ready / hold)."""
    while True:
        yield rng.random() < p


def words_gen(rng, n, tag=0):
    """Random 64-bit words, each carrying the global index in its low byte pattern for easy diagnosis."""
    return [(rng.getrandbits(64) & ~0xFFFF) | (tag << 8) | (i & 0xFF) for i in range(n)]


# ================================================================ reference model / scoreboard
class Burst:
    def __init__(self, addr, awlen, rd_bank, exp_beats, exp_fresh, t_aw, t_issue):
        self.t_issue = t_issue          # cycle the writer left ST_IDLE for this burst (AWVALID first high)
        self.addr = addr
        self.awlen = awlen
        self.nbeats = awlen + 1
        self.rd_bank = rd_bank
        self.exp_beats = exp_beats      # list[int] 256-bit
        self.exp_fresh = exp_fresh      # list[int] 64-bit words new in this presentation
        self.got_beats = []
        self.t_aw = t_aw
        self.t_b = None

    @property
    def end(self):
        return self.addr + self.nbeats * BEAT_BYTES

    @property
    def exp_bytes(self):
        return b"".join(b.to_bytes(BEAT_BYTES, "little") for b in self.exp_beats)


class Scoreboard:
    """Tracks the circular-buffer RAM image (2 banks x 64 words) from the accepted writes and the
    in-order word stream; the writer is then checked against it burst by burst."""

    def __init__(self, log):
        self.log = log
        self.ram = [[0] * WORDS_PER_BANK for _ in range(2)]
        self.seq = [[-1] * WORDS_PER_BANK for _ in range(2)]
        self.stream = []          # every word ever written, in order (global)
        self.consumed = 0         # index into stream of the next word that must appear in DDR
        self.max_consumed_seq = -1
        self.bursts = []          # all bursts (global)
        self.run_bursts = []      # bursts of the current run
        self.done_cycles = []     # cycles where current_user_done == 1
        self.fault_rise_cycles = []
        self.n_presentations = 0  # read_finished pulses
        self.placement = []       # (bank, addr) the RAM used for every accepted word, in order

    def load_ram_image(self, image):
        """Seed the model with the BRAM contents present at test start (stale words from earlier tests in the
        same simulator process; the array is only initialised at time 0).  They carry seq=-1 = never fresh."""
        for bank in range(2):
            for addr in range(WORDS_PER_BANK):
                self.ram[bank][addr] = image[bank * WORDS_PER_BANK + addr]
                self.seq[bank][addr] = -1

    # -- write side (called by the driver for every ACCEPTED word)
    def on_write(self, bank, addr, word):
        idx = len(self.stream)
        self.stream.append(word)
        self.placement.append((bank, addr))
        self.ram[bank][addr] = word
        self.seq[bank][addr] = idx

    # -- read side
    def on_aw(self, addr, awlen, rd_bank, t, t_issue):
        nbeats = awlen + 1
        nwords = nbeats * WORDS_PER_BEAT
        words = self.ram[rd_bank][:nwords]
        seqs = self.seq[rd_bank][:nwords]
        fresh_pos = [a for a in range(nwords) if seqs[a] > self.max_consumed_seq]
        assert fresh_pos, (f"burst @0x{addr:08x} len {nbeats} from bank {rd_bank}: no fresh words "
                           f"(writer presented a bank with nothing new)")
        assert fresh_pos == list(range(len(fresh_pos))), \
            f"fresh words not contiguous from word 0 of bank {rd_bank}: {fresh_pos}"
        nf = len(fresh_pos)
        assert nbeats == math.ceil(nf / WORDS_PER_BEAT), \
            f"awlen+1={nbeats} but {nf} fresh words need {math.ceil(nf/WORDS_PER_BEAT)} beats"
        fresh = words[:nf]
        expect = self.stream[self.consumed:self.consumed + nf]
        assert fresh == expect, (f"word-order mismatch at stream index {self.consumed}: "
                                 f"bank image {[hex(w) for w in fresh[:6]]}... vs stream "
                                 f"{[hex(w) for w in expect[:6]]}...")
        self.consumed += nf
        self.max_consumed_seq = max(seqs[:nf])
        exp_beats = [beat_int(words[4 * k:4 * k + 4]) for k in range(nbeats)]
        b = Burst(addr, awlen, rd_bank, exp_beats, fresh, t, t_issue)
        self.bursts.append(b)
        self.run_bursts.append(b)
        return b

    def new_run(self):
        self.run_bursts = []
        self.run_done_before = len(self.done_cycles)

    @property
    def run_done_count(self):
        return len(self.done_cycles) - self.run_done_before


# ================================================================ testbench
class WriterTB:
    def __init__(self, dut):
        self.dut = dut
        self.log = dut._log
        self.sb = Scoreboard(self.log)
        self.cycle = 0
        self.n_w_stall = 0          # cycles with WVALID && !WREADY (WLAST/WDATA stability exercised)
        self.n_aw_stall = 0
        self.n_b_wait = 0           # cycles in ST_WAIT_B with BVALID low (B response delayed by the slave)
        self.open_burst = None      # Burst between AW handshake and B handshake
        self.beats_seen = 0
        self.idle_fall_cycles = []
        self.idle_rise_cycles = []
        self.base_reset_cycles = []   # cycles in which base_reset was sampled high, with the writer view
        self.c3_cycles = []           # subset: base_reset && able_to_read && !rd_empty && ST_IDLE
        self.prev = None
        self.axi_ram = AxiRamWrite(AxiWriteBus.from_prefix(dut, "m_axi"), dut.clk, dut.rst_n,
                                   reset_active_level=False, size=MEM_SIZE)
        self._mon_task = None

    # ---------------------------------------------------------------- bring-up
    async def start(self, base_seed, p_aw=0.3, p_w=0.3, p_b=0.3):
        d = self.dut
        await start_clock(d.clk, CLK_PERIOD_NS)
        d.wr_en.value = 0
        d.wr_data.value = 0
        d.cb_write_finished_ext.value = 0
        d.wr_write_finished_ext.value = 0
        d.base_addr.value = 0
        d.base_reset.value = 0
        d.dbg_ram_addr.value = 0
        await reset_low(d.rst_n, d.clk, cycles=5)
        self.sb.load_ram_image(await self.peek_ram())
        self.set_backpressure(base_seed, p_aw, p_w, p_b)
        await FallingEdge(d.clk)
        self._mon_task = cocotb.start_soon(self._monitor())
        await FallingEdge(d.clk)

    async def peek_ram(self):
        """Read the 2 x 64 words of the cbuf BRAM through the wrapper's combinational peek port."""
        d = self.dut
        img = []
        for a in range(2 * WORDS_PER_BANK):
            d.dbg_ram_addr.value = a
            await Timer(1, unit="ps")
            img.append(int(d.dbg_ram_q.value))
        d.dbg_ram_addr.value = 0
        return img

    def set_backpressure(self, base_seed, p_aw, p_w, p_b):
        self.axi_ram.aw_channel.set_pause_generator(pause_gen(random.Random(base_seed + 11), p_aw))
        self.axi_ram.w_channel.set_pause_generator(pause_gen(random.Random(base_seed + 22), p_w))
        self.axi_ram.b_channel.set_pause_generator(pause_gen(random.Random(base_seed + 33), p_b))

    def hold(self, aw=None, w=None, b=None):
        """Deterministic stall control (clears the random generators for the given channels)."""
        for ch, v in ((self.axi_ram.aw_channel, aw), (self.axi_ram.w_channel, w), (self.axi_ram.b_channel, b)):
            if v is not None:
                ch.clear_pause_generator()
                ch.pause = bool(v)

    # ---------------------------------------------------------------- per-cycle monitor + protocol checker
    def _sample(self):
        d = self.dut
        return dict(
            awvalid=int(d.m_axi_awvalid.value), awready=int(d.m_axi_awready.value),
            awaddr=int(d.m_axi_awaddr.value), awlen=int(d.m_axi_awlen.value),
            awsize=int(d.m_axi_awsize.value), awburst=int(d.m_axi_awburst.value),
            wvalid=int(d.m_axi_wvalid.value), wready=int(d.m_axi_wready.value),
            wdata=int(d.m_axi_wdata.value), wstrb=int(d.m_axi_wstrb.value), wlast=int(d.m_axi_wlast.value),
            bvalid=int(d.m_axi_bvalid.value), bready=int(d.m_axi_bready.value), bresp=int(d.m_axi_bresp.value),
            state=int(d.dbg_state.value), idle=int(d.writer_idle.value),
            done=int(d.current_user_done.value), fault=int(d.addr_fault.value),
            cur=int(d.cur_axi_addr_out.value), final=int(d.final_addr.value),
            rd_bank=int(d.dbg_rd_bank.value), read_finished=int(d.read_finished.value),
            able=int(d.able_to_read.value), rd_empty=int(d.rd_empty.value),
            base_reset=int(d.base_reset.value),
        )

    async def _monitor(self):
        d = self.dut
        while True:
            await FallingEdge(d.clk)
            self.cycle += 1
            s = self._sample()
            p = self.prev
            t = self.cycle
            # ---- Fork C2: writer_idle is exactly state==ST_IDLE and implies no channel activity
            assert s["idle"] == (s["state"] == ST_IDLE), f"t={t}: writer_idle={s['idle']} state={s['state']}"
            if s["idle"]:
                assert not s["awvalid"] and not s["wvalid"], f"t={t}: idle but AWVALID/WVALID asserted"
            if p is not None:
                if p["idle"] and not s["idle"]:
                    self.idle_fall_cycles.append(t)
                    assert s["awvalid"] and s["state"] == ST_AW, f"t={t}: idle fell without AW issue"
                if not p["idle"] and s["idle"]:
                    self.idle_rise_cycles.append(t)
                    assert p["state"] == ST_WAIT_B and p["bvalid"] and p["bready"], \
                        f"t={t}: idle rose without a B handshake in the previous cycle (state={p['state']})"
            # ---- AXI A3.2.1 stability (Fork C): payload and VALID hold while !READY
            if p is not None:
                if p["awvalid"] and not p["awready"]:
                    self.n_aw_stall += 1
                    assert s["awvalid"], f"t={t}: AWVALID dropped while stalled"
                    for k in ("awaddr", "awlen", "awsize", "awburst"):
                        assert s[k] == p[k], f"t={t}: {k} changed while AWVALID && !AWREADY"
                if p["wvalid"] and not p["wready"]:
                    self.n_w_stall += 1
                    assert s["wvalid"], f"t={t}: WVALID dropped while stalled"
                    assert s["wlast"] == p["wlast"], f"t={t}: WLAST changed while WVALID && !WREADY"
                    assert s["wdata"] == p["wdata"], f"t={t}: WDATA changed while WVALID && !WREADY"
                    assert s["wstrb"] == p["wstrb"], f"t={t}: WSTRB changed while WVALID && !WREADY"
                if s["state"] == ST_WAIT_B and not s["bvalid"]:
                    self.n_b_wait += 1
            # ---- AW handshake
            if s["awvalid"] and s["awready"]:
                assert self.open_burst is None, f"t={t}: AW issued while a burst is still open"
                assert s["awsize"] == 5 and s["awburst"] == 1, f"t={t}: awsize/awburst {s['awsize']}/{s['awburst']}"
                assert s["awaddr"] == s["cur"], f"t={t}: AWADDR 0x{s['awaddr']:x} != cur_axi_addr 0x{s['cur']:x}"
                self.open_burst = self.sb.on_aw(s["awaddr"], s["awlen"], s["rd_bank"], t, self.idle_fall_cycles[-1])
                self.beats_seen = 0
                self.log.debug(f"t={t}: AW addr=0x{s['awaddr']:08x} len={s['awlen']} bank={s['rd_bank']}")
            # ---- W handshake
            if s["wvalid"] and s["wready"]:
                b = self.open_burst
                assert b is not None, f"t={t}: W beat with no open burst"
                i = self.beats_seen
                assert i < b.nbeats, f"t={t}: more W beats than awlen+1"
                assert s["wstrb"] == (1 << BEAT_BYTES) - 1, f"t={t}: WSTRB not full: 0x{s['wstrb']:x}"
                assert s["wlast"] == (1 if i == b.awlen else 0), f"t={t}: WLAST={s['wlast']} on beat {i}/{b.awlen}"
                assert s["wdata"] == b.exp_beats[i], \
                    (f"t={t}: beat {i} of burst @0x{b.addr:x} (bank {b.rd_bank}) data mismatch:\n"
                     f"   got 0x{s['wdata']:064x}\n   exp 0x{b.exp_beats[i]:064x}")
                b.got_beats.append(s["wdata"])
                self.beats_seen += 1
            elif s["wvalid"]:
                assert self.open_burst is not None, f"t={t}: WVALID with no open burst"
            # ---- B handshake
            if s["bvalid"] and s["bready"]:
                b = self.open_burst
                assert b is not None, f"t={t}: B with no open burst"
                assert self.beats_seen == b.nbeats, f"t={t}: B before all beats ({self.beats_seen}/{b.nbeats})"
                assert s["bresp"] == 0
                b.t_b = t
                self.open_burst = None
                got = self.axi_ram.read(b.addr, b.nbeats * BEAT_BYTES)
                assert got == b.exp_bytes, f"t={t}: AxiRam contents @0x{b.addr:x} len {b.nbeats} beats differ"
            # ---- pulses / stickies
            if s["done"]:
                assert p is None or not p["done"], f"t={t}: current_user_done wider than 1 cycle"
                self.sb.done_cycles.append(t)
            if s["read_finished"]:
                assert self.open_burst is None, f"t={t}: read_finished while a burst is open"
                self.sb.n_presentations += 1
            if p is not None and s["fault"] and not p["fault"]:
                self.sb.fault_rise_cycles.append(t)
            if s["base_reset"]:
                self.base_reset_cycles.append(t)
                if s["able"] and not s["rd_empty"] and s["state"] == ST_IDLE:
                    self.c3_cycles.append(t)
            self.prev = s

    # ---------------------------------------------------------------- drivers
    async def base_reset(self, base):
        """Pulse base_reset for one cycle while the writer is idle (the only admissible time)."""
        d = self.dut
        await FallingEdge(d.clk)
        assert int(d.writer_idle.value) == 1, "base_reset requested while writer not idle"
        d.base_addr.value = base
        d.base_reset.value = 1
        await FallingEdge(d.clk)
        d.base_reset.value = 0
        assert int(d.cur_axi_addr_out.value) == base, \
            f"cur_axi_addr 0x{int(d.cur_axi_addr_out.value):x} != base 0x{base:x} after base_reset"
        self.sb.new_run()

    async def write_words(self, words, gap_fn=None):
        """Drive the cbuf write port: wr_en only while wr_ready (Fork A contract); record every accepted
        word in the scoreboard with the (bank, addr) the RAM actually used."""
        d = self.dut
        i = 0
        await FallingEdge(d.clk)
        while i < len(words):
            gap = gap_fn() if gap_fn else 0
            for _ in range(gap):
                d.wr_en.value = 0
                await FallingEdge(d.clk)
            while True:
                if int(d.wr_ready.value):
                    bank = int(d.dbg_wr_bank.value)
                    addr = int(d.dbg_wr_addr.value)
                    d.wr_en.value = 1
                    d.wr_data.value = words[i]
                    await RisingEdge(d.clk)       # accepted here
                    self.sb.on_write(bank, addr, words[i])
                    i += 1
                    await FallingEdge(d.clk)
                    d.wr_en.value = 0
                    break
                d.wr_en.value = 0
                await FallingEdge(d.clk)
        d.wr_en.value = 0

    async def flush(self, quiet=FLUSH_QUIET, writer_ext=True, cbuf_ext=True):
        """Plan v3 B.3 flush: after `quiet` idle cycles, one-cycle write_finished_ext to the cbuf and to
        the writer in the same cycle (the real design crosses the writer copy dsp->ddr, i.e. later)."""
        d = self.dut
        for _ in range(quiet):
            await FallingEdge(d.clk)
        d.cb_write_finished_ext.value = 1 if cbuf_ext else 0
        d.wr_write_finished_ext.value = 1 if writer_ext else 0
        await FallingEdge(d.clk)
        d.cb_write_finished_ext.value = 0
        d.wr_write_finished_ext.value = 0

    async def wait_done(self, timeout=20000):
        d = self.dut
        n0 = len(self.sb.done_cycles)
        for _ in range(timeout):
            await FallingEdge(d.clk)
            if len(self.sb.done_cycles) > n0:
                # let the post-done cycle settle (cur_axi_addr re-based, state IDLE)
                await FallingEdge(d.clk)
                return
        raise AssertionError(f"timeout waiting for current_user_done (state={int(d.dbg_state.value)}, "
                             f"able={int(d.able_to_read.value)} rd_empty={int(d.rd_empty.value)} "
                             f"cur=0x{int(d.cur_axi_addr_out.value):x})")

    async def wait_cycles(self, n):
        for _ in range(n):
            await FallingEdge(self.dut.clk)

    async def wait_for(self, cond, timeout=20000, what="condition"):
        for _ in range(timeout):
            await FallingEdge(self.dut.clk)
            if cond():
                return
        raise AssertionError(f"timeout waiting for {what}")

    async def wait_bursts_done(self, n, timeout=20000):
        """Returns at least one cycle AFTER the n-th burst's B handshake (pointer/fault updates visible)."""
        await self.wait_for(lambda: len(self.sb.run_bursts) >= n and self.sb.run_bursts[-1].t_b is not None,
                            timeout, f"{n} bursts completed")
        await FallingEdge(self.dut.clk)

    # ---------------------------------------------------------------- run-level checks
    def check_run(self, base, expect_final=None, expect_aw=None, expect_fault=0, wrapped=False):
        d = self.dut
        sb = self.sb
        assert sb.consumed == len(sb.stream), \
            f"{len(sb.stream) - sb.consumed} written words never reached DDR"
        assert sb.run_done_count == 1, f"current_user_done pulsed {sb.run_done_count} times in this run"
        assert int(d.writer_idle.value) == 1
        assert int(d.addr_fault.value) == expect_fault, f"addr_fault={int(d.addr_fault.value)} expected {expect_fault}"
        assert int(d.cur_axi_addr_out.value) == base, "cur_axi_addr not re-based after the run"
        bursts = sb.run_bursts
        # AW contiguity (ring arithmetic only when the test expects the wrap branch)
        nxt = base
        for b in bursts:
            assert b.addr == nxt, f"burst @0x{b.addr:x} not contiguous (expected 0x{nxt:x})"
            nxt = b.end
            if wrapped and nxt > WRAP_LIMIT:
                nxt -= WRAP_SIZE
        final = int(d.final_addr.value)
        exp_final = expect_final if expect_final is not None else (bursts[-1].end if bursts else base)
        assert final == exp_final, f"final_addr 0x{final:x} != expected 0x{exp_final:x}"
        if expect_aw is not None:
            got_aw = [(b.addr, b.awlen) for b in bursts]
            assert got_aw == expect_aw, f"AW sequence {[(hex(a), l) for a, l in got_aw]} != {[(hex(a), l) for a, l in expect_aw]}"
        if not wrapped and bursts:
            total = b"".join(b.exp_bytes for b in bursts)
            got = self.axi_ram.read(base, len(total))
            assert got == total, "whole-run DDR image differs from the expected byte stream"
            assert final == base + len(total)

    async def run(self, base, words, gap_fn=None, quiet=FLUSH_QUIET, **chk):
        await self.base_reset(base)
        await self.write_words(words, gap_fn)
        await self.flush(quiet)
        await self.wait_done()
        self.check_run(base, **chk)


async def setup(dut, **bp):
    s = seed(dut)
    tb = WriterTB(dut)
    await tb.start(s, **bp)
    return tb, s


# ================================================================ tests
@cocotb.test()
async def test_reset_state(dut):
    """After reset: idle, no AXI activity, addr_fault=0, final_addr=0, cur_axi_addr=0, cbuf empty."""
    tb, s = await setup(dut)
    d = dut
    assert int(d.writer_idle.value) == 1 and int(d.dbg_state.value) == ST_IDLE
    assert int(d.m_axi_awvalid.value) == 0 and int(d.m_axi_wvalid.value) == 0
    assert int(d.m_axi_bready.value) == 1                # writer always ready for B once out of reset
    assert int(d.addr_fault.value) == 0 and int(d.final_addr.value) == 0
    assert int(d.cur_axi_addr_out.value) == 0 and int(d.current_user_done.value) == 0
    assert int(d.rd_empty.value) == 1 and int(d.wr_ready.value) == 1
    await tb.wait_cycles(30)
    assert tb.sb.bursts == [] and tb.sb.done_cycles == []
    assert int(d.writer_idle.value) == 1


@cocotb.test()
async def test_wlast_stable_under_stall(dut):
    """(1) Fork C: under heavy random AW/W/B backpressure WLAST/WDATA/WSTRB and VALID never change while
    WVALID && !WREADY (protocol checker), WLAST sits exactly on beat awlen, data byte-exact."""
    tb, s = await setup(dut, p_aw=0.6, p_w=0.75, p_b=0.6)
    rng = random.Random(s + 1)
    base = 0x1000
    words = words_gen(rng, 4 * WORDS_PER_BANK + 9, tag=1)      # 4 full banks + 3-beat partial
    await tb.run(base, words, expect_final=base + 4 * BANK_BYTES + 3 * BEAT_BYTES)
    assert tb.n_w_stall >= 40, f"W stall cycles too few ({tb.n_w_stall}) -- stability check not exercised"
    assert tb.n_aw_stall >= 3 and tb.n_b_wait >= 3, (tb.n_aw_stall, tb.n_b_wait)
    # every burst ended with exactly one WLAST on its final beat (checked beat-by-beat in the monitor)
    assert len(tb.sb.run_bursts) == 5
    dut._log.info(f"W stall cycles={tb.n_w_stall} AW stall={tb.n_aw_stall} B wait={tb.n_b_wait}")


@cocotb.test()
async def test_exact_bursts_final_addr(dut):
    """(2) N=3 full banks + 5-word partial (2 beats): exact AW sequence, AxiRam byte-exact incl. the
    stale padding lanes predicted by a pure Python bank model, final_addr = base + whole beats,
    current_user_done exactly once, addr_fault=0."""
    tb, s = await setup(dut, p_aw=0.3, p_w=0.4, p_b=0.3)
    rng = random.Random(s + 2)
    N, partial = 3, 5
    base = 0x2000
    words = words_gen(rng, N * WORDS_PER_BANK + partial, tag=2)
    await tb.base_reset(base)
    await tb.write_words(words)                        # continuous: bank k <- words[64k:64k+64]
    # pure prediction of the bank mapping under continuous writes: word i -> bank (i//64)%2, addr i%64
    exp_place = [((i // WORDS_PER_BANK) % 2, i % WORDS_PER_BANK) for i in range(len(words))]
    assert tb.sb.placement == exp_place, "cbuf placement deviates from the continuous-write prediction"
    await tb.flush()
    await tb.wait_done()
    expect_aw = [(base + k * BANK_BYTES, BEATS_PER_BANK - 1) for k in range(N)] + [(base + N * BANK_BYTES, 1)]
    tb.check_run(base, expect_final=base + N * BANK_BYTES + 2 * BEAT_BYTES, expect_aw=expect_aw)
    # independent pure-model image: the partial bank is bank N%2; its last beat's 3 stale lanes are the
    # words that sat at the same RAM positions two fills ago (or 0 if that bank was never filled).
    exp = b"".join(beat_bytes(words[4 * k:4 * k + 4]) for k in range(N * BEATS_PER_BANK))
    pb = N % 2
    prev_fill = (N - 2) * WORDS_PER_BANK if N >= 2 else None
    tail = list(words[N * WORDS_PER_BANK:])
    for a in range(partial, 2 * WORDS_PER_BEAT):
        tail.append(words[prev_fill + a] if prev_fill is not None else 0)
    exp += beat_bytes(tail[:4]) + beat_bytes(tail[4:8])
    got = tb.axi_ram.read(base, len(exp))
    assert got == exp, "AxiRam image differs from the pure Python bank model (incl. stale padding lanes)"
    assert int(dut.final_addr.value) == base + len(exp)
    assert len(tb.sb.done_cycles) == 1
    dut._log.info(f"final_addr=0x{int(dut.final_addr.value):x} bursts={[(hex(b.addr), b.awlen) for b in tb.sb.run_bursts]}")


@cocotb.test()
async def test_exact_multiple_flush_empty_bank(dut):
    """Exact multiple of the bank size (2 full banks, no partial): the flush presents an EMPTY bank and
    current_user_done comes from the rd_empty path with final_addr = base + 2*512."""
    tb, s = await setup(dut)
    rng = random.Random(s + 3)
    base = 0x3000
    words = words_gen(rng, 2 * WORDS_PER_BANK, tag=3)
    await tb.run(base, words, expect_final=base + 2 * BANK_BYTES,
                 expect_aw=[(base, 15), (base + BANK_BYTES, 15)])


@cocotb.test()
async def test_zero_words_flush(dut):
    """Flush with nothing written: no AW ever issued, current_user_done once, final_addr == base."""
    tb, s = await setup(dut)
    base = 0x4200
    await tb.run(base, [], expect_final=base, expect_aw=[])
    assert tb.sb.bursts == []


@cocotb.test()
async def test_c3_base_reset_same_cycle_as_bank_ready(dut):
    """(3) Fork C3: base_reset asserted in the SAME cycle a non-empty bank becomes ready while idle.
    The burst must not start with the old pointer: cur_axi_addr == new base the next cycle and the
    (single) AW carries the new base; data lands at the new base; final_addr follows."""
    tb, s = await setup(dut, p_aw=0.2, p_w=0.3, p_b=0.2)
    rng = random.Random(s + 4)
    d = dut
    old_base, new_base = 0x5000, 0x7000
    await tb.base_reset(old_base)
    words = words_gen(rng, WORDS_PER_BANK, tag=4)
    hit = {}

    async def arm():
        # wait (at falling edges, i.e. on settled registered values) for the first cycle in which the
        # writer WOULD start a burst, and assert base_reset in exactly that cycle
        while True:
            await FallingEdge(d.clk)
            if int(d.able_to_read.value) and not int(d.rd_empty.value) and int(d.dbg_state.value) == ST_IDLE:
                assert tb.sb.bursts == [], "a burst was issued before the armed cycle"
                d.base_addr.value = new_base
                d.base_reset.value = 1
                hit["cycle"] = tb.cycle
                hit["cur_before"] = int(d.cur_axi_addr_out.value)
                await FallingEdge(d.clk)
                d.base_reset.value = 0
                # the cycle after: pointer re-based, still idle, no AW issued
                hit["state_after"] = int(d.dbg_state.value)
                hit["cur_after"] = int(d.cur_axi_addr_out.value)
                hit["awvalid_after"] = int(d.m_axi_awvalid.value)
                return

    arm_task = cocotb.start_soon(arm())
    await tb.write_words(words)
    await arm_task
    assert hit["cur_before"] == old_base
    assert hit["state_after"] == ST_IDLE and hit["awvalid_after"] == 0, \
        f"writer left ST_IDLE in the base_reset cycle: {hit}"
    assert hit["cur_after"] == new_base, f"cur_axi_addr 0x{hit['cur_after']:x} != new base after C3 reset"
    await tb.wait_bursts_done(1)
    b = tb.sb.run_bursts[0]
    assert b.addr == new_base and b.awlen == 15, f"burst went to 0x{b.addr:x}, expected new base 0x{new_base:x}"
    assert len(tb.base_reset_cycles) == 2, tb.base_reset_cycles       # the initial one + the C3 one
    t_rst = tb.base_reset_cycles[-1]
    assert tb.c3_cycles == [t_rst], "base_reset was not sampled in a cycle with able_to_read && !rd_empty && ST_IDLE"
    assert b.t_issue == t_rst + 1, f"burst issued at t={b.t_issue}, expected the cycle right after the reset (t={t_rst+1})"
    await tb.flush()
    await tb.wait_done()
    tb.check_run(new_base, expect_final=new_base + BANK_BYTES, expect_aw=[(new_base, 15)])
    # nothing was ever written at the old base
    assert tb.axi_ram.read(old_base, BANK_BYTES) == bytes(BANK_BYTES)
    d._log.info(f"C3 hit at t={hit['cycle']}: cur 0x{hit['cur_before']:x} -> 0x{hit['cur_after']:x}, AW @0x{b.addr:x} t={b.t_aw}")


@cocotb.test()
async def test_c4a_wrap_branch_nonfinal_fault(dut):
    """(4a) Fork C4: a NON-final bank whose cur_axi_addr_plus_bank > WRAP_LIMIT takes the ring-wrap branch
    -> addr_fault=1 at its B handshake (the IDLE-time check stays silent because the bank ends exactly at
    the limit), the pointer wraps to (end - WRAP_SIZE) and the next (partial, final) bank lands there."""
    tb, s = await setup(dut)
    rng = random.Random(s + 5)
    d = dut
    base = WRAP_SIZE - BANK_BYTES           # 0xF800: bank ends at 0xF9FF == WRAP_LIMIT
    words = words_gen(rng, WORDS_PER_BANK + 5, tag=5)
    await tb.base_reset(base)
    await tb.write_words(words)
    await tb.wait_bursts_done(1)
    b0 = tb.sb.run_bursts[0]
    assert b0.addr == base and b0.awlen == 15
    assert int(d.addr_fault.value) == 1, "addr_fault not set after the wrap-branch B handshake"
    assert tb.sb.fault_rise_cycles == [b0.t_b + 1], \
        f"fault rose at {tb.sb.fault_rise_cycles}, expected exactly at B+1 = {b0.t_b + 1} (IDLE check must stay silent)"
    assert int(d.cur_axi_addr_out.value) == 0, f"pointer did not wrap: 0x{int(d.cur_axi_addr_out.value):x}"
    await tb.flush()
    await tb.wait_done()
    tb.check_run(base, expect_final=0 + 2 * BEAT_BYTES, expect_aw=[(base, 15), (0, 1)], expect_fault=1, wrapped=True)
    assert tb.axi_ram.read(base, BANK_BYTES) == b0.exp_bytes
    assert tb.axi_ram.read(0, 2 * BEAT_BYTES) == tb.sb.run_bursts[1].exp_bytes


@cocotb.test()
async def test_c4b_final_burst_overrun_fault(dut):
    """(4b) Fork C4: the FINAL burst ends past WRAP_LIMIT (the last-burst path bypasses the wrap branch) ->
    addr_fault=1 from the IDLE-time check at burst issue.  Sub-case 1: flushed partial bank (2 beats)
    starting 32 B before the limit.  Sub-case 2: a full bank marked last (writer write_finished_ext
    latched before the bank is presented) starting 32 B too high."""
    tb, s = await setup(dut)
    rng = random.Random(s + 6)
    d = dut
    # sub-case 1: partial final bank 0xF9E0..0xFA1F
    base = WRAP_SIZE - BEAT_BYTES
    words = words_gen(rng, 5, tag=6)
    await tb.base_reset(base)
    assert int(d.addr_fault.value) == 0
    await tb.write_words(words)
    await tb.flush()
    await tb.wait_done()
    tb.check_run(base, expect_final=base + 2 * BEAT_BYTES, expect_aw=[(base, 1)], expect_fault=1)
    b = tb.sb.run_bursts[0]
    assert tb.sb.fault_rise_cycles == [b.t_issue], \
        f"fault rose at {tb.sb.fault_rise_cycles}, expected at burst issue t={b.t_issue}"
    assert b.end == WRAP_LIMIT + 1 + BEAT_BYTES
    # sub-case 2: full bank as last burst, 0xF820..0xFA1F
    base2 = WRAP_SIZE - BANK_BYTES + BEAT_BYTES
    await tb.base_reset(base2)                          # also clears the sticky (4d)
    assert int(d.addr_fault.value) == 0
    await tb.flush(quiet=0, cbuf_ext=False)             # writer-side finish latched early -> first bank is 'last'
    words2 = words_gen(rng, WORDS_PER_BANK, tag=7)
    await tb.write_words(words2)
    await tb.wait_done()
    tb.check_run(base2, expect_final=base2 + BANK_BYTES, expect_aw=[(base2, 15)], expect_fault=1)
    b2 = tb.sb.run_bursts[0]
    assert tb.sb.fault_rise_cycles[-1] == b2.t_issue
    assert tb.sb.done_cycles[-1] == b2.t_b + 1          # done from the last-burst path, not the empty-bank path


@cocotb.test()
async def test_c4c_boundary_exact_no_fault(dut):
    """(4c) Fork C4 boundary: a final burst ending EXACTLY at WRAP_LIMIT -> addr_fault stays 0 and
    final_addr == WRAP_LIMIT+1.  Sub-case 1: partial 2-beat bank at 0xF9C0; sub-case 2: full bank marked
    last at 0xF800 (it must NOT take the wrap branch: last_burst re-bases instead)."""
    tb, s = await setup(dut)
    rng = random.Random(s + 8)
    d = dut
    base = WRAP_SIZE - 2 * BEAT_BYTES
    await tb.run(base, words_gen(rng, 5, tag=8), expect_final=WRAP_LIMIT + 1, expect_aw=[(base, 1)], expect_fault=0)
    base2 = WRAP_SIZE - BANK_BYTES
    await tb.base_reset(base2)
    await tb.flush(quiet=0, cbuf_ext=False)
    await tb.write_words(words_gen(rng, WORDS_PER_BANK, tag=9))
    await tb.wait_done()
    tb.check_run(base2, expect_final=WRAP_LIMIT + 1, expect_aw=[(base2, 15)], expect_fault=0)
    assert tb.sb.fault_rise_cycles == []
    assert int(d.cur_axi_addr_out.value) == base2       # re-based by last_burst, not wrapped to 0


@cocotb.test()
async def test_c4d_fault_cleared_only_by_idle_base_reset(dut):
    """(4d) addr_fault is sticky: it survives idle cycles, a later clean run and a base_reset pulsed while
    NOT idle; it clears on a base_reset in ST_IDLE, after which a clean run keeps it 0."""
    tb, s = await setup(dut, p_aw=0.9, p_w=0.3, p_b=0.3)       # slow AW so we can catch ST_AW
    rng = random.Random(s + 10)
    d = dut
    base = WRAP_SIZE - BEAT_BYTES
    await tb.run(base, words_gen(rng, 5, tag=10), expect_final=base + 2 * BEAT_BYTES, expect_fault=1)
    await tb.wait_cycles(100)
    assert int(d.addr_fault.value) == 1, "addr_fault did not stick"
    # a run WITHOUT base_reset continues at cur (== base after the flush re-base): same overrun, still 1
    # and a base_reset pulsed while the burst is in ST_AW must be ignored (no clear)
    tb.sb.new_run()
    await tb.write_words(words_gen(rng, 5, tag=11))
    await tb.flush()                                   # presents the partial bank -> burst issue
    await tb.wait_for(lambda: int(d.dbg_state.value) == ST_AW, what="ST_AW")
    d.base_addr.value = 0x1000
    d.base_reset.value = 1
    await FallingEdge(d.clk)
    d.base_reset.value = 0
    assert int(d.addr_fault.value) == 1, "non-idle base_reset cleared addr_fault"
    assert int(d.cur_axi_addr_out.value) == base, "non-idle base_reset moved cur_axi_addr"
    await tb.wait_done()
    assert int(d.addr_fault.value) == 1
    assert int(d.final_addr.value) == base + 2 * BEAT_BYTES
    # idle base_reset clears it and a clean run keeps it clear
    await tb.base_reset(0x1000)
    assert int(d.addr_fault.value) == 0, "idle base_reset did not clear addr_fault"
    await tb.wait_cycles(20)
    assert int(d.addr_fault.value) == 0
    await tb.run(0x1000, words_gen(rng, WORDS_PER_BANK + 3, tag=12), expect_final=0x1000 + BANK_BYTES + BEAT_BYTES,
                 expect_fault=0)


@cocotb.test()
async def test_writer_idle_tracks_state(dut):
    """(5) Fork C2: writer_idle == (state==ST_IDLE) every cycle (monitor); black-box: idle before the run,
    falls exactly when AW is issued, stays 0 through W and the B wait, rises the cycle after B, idle after."""
    tb, s = await setup(dut, p_aw=0.5, p_w=0.5, p_b=0.7)
    rng = random.Random(s + 13)
    d = dut
    base = 0x6000
    await tb.wait_cycles(10)
    assert int(d.writer_idle.value) == 1
    await tb.run(base, words_gen(rng, 2 * WORDS_PER_BANK + 7, tag=13), expect_final=base + 2 * BANK_BYTES + 2 * BEAT_BYTES)
    bursts = tb.sb.run_bursts
    assert len(bursts) == 3
    assert len(tb.idle_fall_cycles) == 3 and len(tb.idle_rise_cycles) == 3, (tb.idle_fall_cycles, tb.idle_rise_cycles)
    for b, tf, tr in zip(bursts, tb.idle_fall_cycles, tb.idle_rise_cycles):
        assert tf <= b.t_aw, f"idle fell at {tf} after AW handshake {b.t_aw}"
        assert tr == b.t_b + 1, f"idle rose at {tr}, B handshake at {b.t_b}"
        assert tr - tf >= b.nbeats + 2, "busy window shorter than AW + beats + B"
    assert int(d.writer_idle.value) == 1


@cocotb.test()
async def test_base_reset_ignored_when_not_idle(dut):
    """(6) documented behaviour: base_reset pulsed in ST_AW, ST_READ_WRITE and ST_WAIT_B is IGNORED --
    cur_axi_addr, AWADDR and the burst are unaffected and the pointer advances from the old value;
    the same pulse in ST_IDLE afterwards is honoured."""
    tb, s = await setup(dut)
    tb.hold(aw=True, w=True, b=True)            # deterministic stalls so every state can be caught
    rng = random.Random(s + 14)
    d = dut
    base, other = 0x8000, 0x9000
    await tb.base_reset(base)
    words = words_gen(rng, WORDS_PER_BANK, tag=14)
    await tb.write_words(words)

    async def pulse_in(state, name):
        await tb.wait_for(lambda: int(d.dbg_state.value) == state, what=name)
        cur0, aw0 = int(d.cur_axi_addr_out.value), int(d.m_axi_awaddr.value)
        d.base_addr.value = other
        d.base_reset.value = 1
        await FallingEdge(d.clk)
        d.base_reset.value = 0
        assert int(d.dbg_state.value) == state, f"{name}: state changed during the pulse"
        assert int(d.cur_axi_addr_out.value) == cur0 == base, f"{name}: base_reset moved cur_axi_addr"
        assert int(d.m_axi_awaddr.value) == aw0 == base, f"{name}: AWADDR changed"
        await FallingEdge(d.clk)
        assert int(d.cur_axi_addr_out.value) == base

    await pulse_in(ST_AW, "ST_AW")
    tb.hold(aw=False)
    await pulse_in(ST_READ_WRITE, "ST_READ_WRITE")
    tb.hold(w=False)
    await pulse_in(ST_WAIT_B, "ST_WAIT_B")
    tb.hold(b=False)
    await tb.wait_bursts_done(1)
    b = tb.sb.run_bursts[0]
    assert b.addr == base and int(d.cur_axi_addr_out.value) == base + BANK_BYTES, \
        f"pointer after the burst 0x{int(d.cur_axi_addr_out.value):x} != base+512"
    assert tb.axi_ram.read(base, BANK_BYTES) == b.exp_bytes
    assert tb.axi_ram.read(other, BANK_BYTES) == bytes(BANK_BYTES)
    # now idle: the pulse IS honoured
    d.base_addr.value = other
    d.base_reset.value = 1
    await FallingEdge(d.clk)
    d.base_reset.value = 0
    assert int(d.cur_axi_addr_out.value) == other
    tb.sb.new_run()
    await tb.flush()
    await tb.wait_done()
    assert int(d.final_addr.value) == other and tb.sb.run_done_count == 1


@cocotb.test()
async def test_random_mixed_runs(dut):
    """(7) R seeded random runs: random base (512-aligned, below the ring), random word count
    (0..~9 banks), random idle gaps between writes (which also exercise the cbuf's spontaneous empty/
    partial bank presentations), random AW/W/B stalls re-drawn per run; every word lands byte-exact and
    in order, final_addr / done / contiguity / addr_fault checked per run."""
    tb, s = await setup(dut)
    rng = random.Random(s + 15)
    R = 12
    for r in range(R):
        p_aw, p_w, p_b = (rng.choice([0.0, 0.2, 0.5, 0.8]) for _ in range(3))
        tb.set_backpressure(s + 100 * (r + 1), p_aw, p_w, p_b)
        n = rng.choice([0, 1, 3, 4, 5, 63, 64, 65, 128, 129, rng.randrange(0, 9 * WORDS_PER_BANK)])
        base = rng.randrange(0, (WRAP_SIZE - 10 * BANK_BYTES) // BANK_BYTES) * BANK_BYTES
        gap_mode = rng.choice(["none", "sparse", "bursty"])
        if gap_mode == "none":
            gap_fn = None
        elif gap_mode == "sparse":
            gap_fn = lambda: rng.choice([0, 0, 0, 1, 2, 5])
        else:
            gap_fn = lambda: 0 if rng.random() < 0.9 else rng.randrange(10, 90)
        dut._log.info(f"run {r}: base=0x{base:x} words={n} gaps={gap_mode} p_aw={p_aw} p_w={p_w} p_b={p_b}")
        words = words_gen(rng, n, tag=16 + r)
        await tb.run(base, words, gap_fn=gap_fn, expect_fault=0)
        nbeats = sum(b.nbeats for b in tb.sb.run_bursts)
        assert nbeats * WORDS_PER_BEAT >= n and nbeats >= math.ceil(n / WORDS_PER_BEAT)
        assert int(dut.final_addr.value) == base + nbeats * BEAT_BYTES
        dut._log.info(f"run {r}: {len(tb.sb.run_bursts)} bursts, {nbeats} beats, final=0x{int(dut.final_addr.value):x}, "
                      f"presentations so far={tb.sb.n_presentations}")
    assert tb.n_w_stall > 0 and tb.n_aw_stall > 0 and tb.n_b_wait > 0
