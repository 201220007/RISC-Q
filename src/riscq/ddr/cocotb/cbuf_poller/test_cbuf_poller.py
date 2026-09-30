"""G1'(b) suite 'cbuf_poller': roll_poll_reader2 (NUM_CH=14, 64b) -> skid(8)/throttle(>=5) -> circular_buffer3 Fork A,
two clocks (wr_clk 2 ns / rd_clk 3 ns, co-prime).

P3a: both DUT modules are the SpinalHDL RollPollReader / CircularBuffer generated under the vendored names
(riscq.ddr.sim.GenUplinkUnits). The scoreboard, the reference bank model and the monitors are the vendored-RTL
suite's. What changed is fix F4 (src/riscq/ddr/CONTRACT.md): a bank is presented only when it is full or when a
flush closes it (then it carries rd_final), so the vendored observations -- the post-reset empty one-shots, the
empty "phantom" presentation after a full bank followed by a pause, and a late write cancelling a pending flush --
are now positive tests of the opposite behaviour (setup(), test_03, test_09, test_10, test_08 allow_phantom=False).
New: test_11 (rd_final in band + rd_empty after every run), test_12 (inverted clock ratio), test_13 (poller
saturation: 1 word / 3 cycles, fairness, throttle-in-flight capacity).

Wrapper: cbuf_poller_tb.sv (skid FIFO + throttle + glue gating live in the wrapper, see its header).
Python side (this file):
  * WrSide  — per-channel data_buffer model (valid holds until rd_en, write priority over clear), scoreboard of
              every consumed word, one-hot / valid-only / throttle-slip / stall-invariant monitors, optional direct
              override of the cbuf write port.
  * Reader  — circular_buffer_axi_writer protocol model on rd_clk: able_to_read && !rd_empty -> read rows
              0..rd_addr_valid (1-cycle data latency) -> pulse read_finished; able_to_read && rd_empty -> one-shot
              read_finished (as the real writer does, never delayed); non-empty reads can be held / delayed
              (= AXI back-pressure) to starve the write side of credit.
  * check_events — pure-Python reference of the cbuf bank sequencing + RAM (2 physical banks x 64 words,
              stale lanes included) built ONLY from the scoreboard and the flush points; every bank the reader
              collected must match it exactly (rd_addr_valid and every 256-bit row, lane k = word k in [64k+63:64k]).
Every test is self-checking; random stimulus is seeded via ddrtb.seed (SEED env var overrides).
"""
import random
from collections import deque

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, Timer
from ddrtb import seed

NUM_CH = 14
WORDS_PER_BANK = 64
RATIO = 4
SKID_DEPTH = 8
THROTTLE_AT = 5
MAX_SLIP = 2                      # plan v3 B.2: <=2-word BARREL->ENCODE->CONSUME slip past the mask
WR_PERIOD_NS = 2
RD_PERIOD_NS = 3
M64 = (1 << 64) - 1
POISON = 0xDEAD_BEEF_BAD0_CAFE


def mkword(ch, seq):
    """Unique, self-describing 64-bit word: [63:56]=channel [55:32]=seq [31:0]=random."""
    return ((ch & 0xFF) << 56) | ((seq & 0xFFFFFF) << 32) | random.getrandbits(32)


def lanes(row):
    return [(row >> (64 * k)) & M64 for k in range(RATIO)]


# ----------------------------------------------------------------------------------------------------------------
# Write side: channel sources + monitors
# ----------------------------------------------------------------------------------------------------------------
class WrSide:
    def __init__(self, dut):
        self.dut = dut
        self.q = [deque() for _ in range(NUM_CH)]
        self.seq = [0] * NUM_CH
        self.pending_pop = None
        self.consumed = []            # (ch, word, cycle) in poller consumption order
        self.accepted = []            # words seen with wr_en_out==1 (what the RAM strobe actually wrote)
        self.errors = []
        self.cycle = 0
        self.running = True
        # throttle bookkeeping
        self.throttle_prev = 0
        self.episode = None           # words consumed while the mask is active, current episode
        self.slips = []               # per finished episode
        self.max_skid = 0
        # stall bookkeeping
        self.stall_cycles = 0
        self.stall_snap = None
        self.stall_episodes = 0
        self.wr_ready_min = 1
        # override of the cbuf write port: callable(cycle, wr_ready) -> (wr_en, data)
        self.ovr = None
        self.ovr_checks = []          # (wr_en_in, wr_ready, wr_en_out)
        self.ovr_accepted = []

    def err(self, msg):
        self.errors.append(f"[wr cycle {self.cycle}] {msg}")
        self.dut._log.error(self.errors[-1])

    def load(self, ch, n):
        ws = []
        for _ in range(n):
            w = mkword(ch, self.seq[ch]); self.seq[ch] += 1
            self.q[ch].append(w); ws.append(w)
        return ws

    def queues_empty(self):
        return all(len(x) == 0 for x in self.q) and self.pending_pop is None

    def snapshot(self):
        d = self.dut
        return (int(d.dbg_wr_addr.value), int(d.dbg_bank_sel_wr.value), int(d.dbg_bank_sel_rd.value),
                int(d.dbg_lva0.value), int(d.dbg_lva1.value), int(d.dbg_empty0.value), int(d.dbg_empty1.value),
                int(d.write_finished_out.value), int(d.dbg_credit.value))

    async def run(self):
        d = self.dut
        while self.running:
            await FallingEdge(d.wr_clk)
            self.cycle += 1
            rd_en = int(d.rd_en.value)
            thr = int(d.throttle.value)
            cnt = int(d.skid_count.value)
            wr_ready = int(d.wr_ready.value)
            self.max_skid = max(self.max_skid, cnt)
            self.wr_ready_min = min(self.wr_ready_min, wr_ready)
            if int(d.skid_overflow.value):
                self.err("skid_overflow sticky set (push while full)")
            if cnt > SKID_DEPTH:
                self.err(f"skid_count {cnt} > depth")
            # throttle episodes
            if thr and not self.throttle_prev:
                self.episode = 0
            if (not thr) and self.throttle_prev:
                self.slips.append(self.episode); self.episode = None
            self.throttle_prev = thr
            # the pop announced last cycle lands now (data persisted through the consume edge)
            if self.pending_pop is not None:
                self.q[self.pending_pop].popleft(); self.pending_pop = None
            # rd_en (combinational from the CONSUME phase) is honoured at the upcoming rising edge
            if rd_en:
                if rd_en & (rd_en - 1):
                    self.err(f"rd_en not one-hot: {rd_en:#06x}")
                ch = rd_en.bit_length() - 1
                if not self.q[ch]:
                    self.err(f"rd_en to channel {ch} which has no valid word")
                else:
                    w = self.q[ch][0]
                    self.consumed.append((ch, w, self.cycle))
                    self.pending_pop = ch
                    if thr:
                        self.episode += 1
            # present heads (data_buffer contract)
            v = 0; data = 0
            for i in range(NUM_CH):
                if self.q[i]:
                    v |= 1 << i
                    data |= self.q[i][0] << (64 * i)
            d.ch_valid.value = v
            d.ch_data.value = data
            # optional direct drive of the cbuf write port
            if self.ovr is not None:
                en, od = self.ovr(self.cycle, wr_ready)
                d.tb_ovr_sel.value = 1; d.tb_ovr_wr_en.value = en; d.tb_ovr_wr_data.value = od
            else:
                en = od = None
                d.tb_ovr_sel.value = 0; d.tb_ovr_wr_en.value = 0
            await Timer(100, unit="ps")
            # values the upcoming rising edge will sample
            weo = int(d.wr_en_out.value)
            cwe = int(d.cbuf_wr_en.value)
            if weo != (cwe & wr_ready):
                self.err(f"wr_en_out={weo} but cbuf_wr_en={cwe} wr_ready={wr_ready}")
            if weo:
                self.accepted.append(int(d.cbuf_wr_data.value))
            if en is not None:
                self.ovr_checks.append((en, wr_ready, weo))
                if en and wr_ready:
                    self.ovr_accepted.append(od)
            # stall invariants: while wr_ready==0 nothing on the write side may move
            if wr_ready == 0:
                self.stall_cycles += 1
                if weo:
                    self.err("wr_en_out=1 while wr_ready=0")
                snap = self.snapshot()
                if self.stall_snap is None:
                    self.stall_snap = snap; self.stall_episodes += 1
                elif snap != self.stall_snap:
                    self.err(f"cbuf metadata changed during stall: {self.stall_snap} -> {snap}")
                if self.stall_cycles > 1 and int(d.dbg_ram_we_d.value):
                    self.err("RAM write strobe (weA_d) active during stall")
            else:
                self.stall_snap = None
                self.stall_cycles = 0


# ----------------------------------------------------------------------------------------------------------------
# Read side: circular_buffer_axi_writer protocol model
# ----------------------------------------------------------------------------------------------------------------
class Reader:
    def __init__(self, dut):
        self.dut = dut
        self.events = []              # ('empty', phys) | ('bank', rd_addr_valid, rows, phys)
        self.hold = False             # AXI back-pressure model: non-empty bank reads do not start
        self.start_delay = lambda: 0  # rd cycles between seeing a bank and reading it
        self.beat_gap = lambda: 0     # idle rd cycles between beats
        self.finish_count = 0
        self.beats = 0
        self.finals = []              # rd_final_out of each presentation, in event order (P3a F4)

    async def finish(self):
        d = self.dut
        d.read_finished.value = 1
        await FallingEdge(d.rd_clk)
        d.read_finished.value = 0
        self.finish_count += 1

    async def run(self):
        d = self.dut
        d.rd_addr.value = 0; d.cbuf_rd_en.value = 0; d.read_finished.value = 0
        while True:
            await FallingEdge(d.rd_clk)
            if not int(d.able_to_read_out.value):
                continue
            phys = int(d.dbg_rd_bank_sel.value)
            fin = int(d.rd_final_out.value)
            if int(d.rd_empty.value):
                # real writer: empty bank -> immediate one-shot read_finished (never delayed)
                self.events.append(("empty", phys))
                self.finals.append(fin)
                await self.finish()
                continue
            if self.hold:
                continue
            for _ in range(self.start_delay()):
                await FallingEdge(d.rd_clk)
            n = int(d.rd_addr_valid_out.value) + 1
            rows = []
            for a in range(n):
                d.rd_addr.value = a; d.cbuf_rd_en.value = 1
                await FallingEdge(d.rd_clk)          # rising edge sampled addrB=a -> doB = row a (1-cycle contract)
                rows.append(int(d.rd_data.value))
                self.beats += 1
                for _ in range(self.beat_gap()):
                    d.cbuf_rd_en.value = 0
                    await FallingEdge(d.rd_clk)
            d.cbuf_rd_en.value = 0; d.rd_addr.value = 0
            self.events.append(("bank", n - 1, rows, phys))
            self.finals.append(fin)
            await self.finish()


# ----------------------------------------------------------------------------------------------------------------
# Reference model of bank sequencing + RAM, and the event checker
# ----------------------------------------------------------------------------------------------------------------
def describe(ev):
    if ev[0] == "empty":
        return f"empty(phys={ev[1]})"
    return f"bank(rd_addr_valid={ev[1]}, rows={len(ev[2])}, phys={ev[3]})"


# The BRAM inside cbuf_ram_read_wider is initialised once per simulation and is never cleared by a reset, exactly like
# the board across runs; the reference RAM therefore persists across tests too (stale lanes of a partial bank show
# whatever an earlier run left there).
RAM_MODEL = [[0] * WORDS_PER_BANK, [0] * WORDS_PER_BANK]


def check_events(actual, words, closes, allow_phantom=False, initial=()):
    """actual: Reader.events. words: ordered list of words accepted by the cbuf. closes: sorted word indices at which an
    explicit bank close happens (flush pulse, or an expected empty recovery switch). A bank also closes by itself
    after 64 words (seamless switch). allow_phantom: accept an OPTIONAL ('empty') event right after a full bank
    (the cbuf's write_finished-driven empty recovery switch when the traffic pauses; see REPORT 'RTL findings')."""
    ram = RAM_MODEL
    st = {"phys": 0, "ai": 0}
    cur = []

    def expect(ev):
        i = st["ai"]
        assert i < len(actual), f"reader event #{i} missing: expected {describe(ev)}"
        got = actual[i]
        assert got[0] == ev[0], f"event #{i}: expected {describe(ev)}, got {describe(got)}"
        if ev[0] == "empty":
            assert got[1] == ev[1], f"event #{i}: empty bank phys expected {ev[1]} got {got[1]}"
        else:
            assert got[3] == ev[3], f"event #{i}: physical bank expected {ev[3]} got {got[3]}"
            assert got[1] == ev[1], f"event #{i}: rd_addr_valid expected {ev[1]} got {got[1]}"
            assert len(got[2]) == len(ev[2])
            for r, (er, gr) in enumerate(zip(ev[2], got[2])):
                if er != gr:
                    el, gl = lanes(er), lanes(gr)
                    bad = [(k, f"{el[k]:016x}", f"{gl[k]:016x}") for k in range(RATIO) if el[k] != gl[k]]
                    raise AssertionError(f"event #{i} row {r}: lane mismatch (lane, expected, got) = {bad}")
        st["ai"] += 1

    def close():
        p = st["phys"]
        for k, w in enumerate(cur):
            ram[p][k] = w
        if not cur:
            expect(("empty", p))
        else:
            nrows = (len(cur) - 1) // RATIO + 1
            rows = [sum(ram[p][RATIO * r + k] << (64 * k) for k in range(RATIO)) for r in range(nrows)]
            expect(("bank", nrows - 1, rows, p))
        st["phys"] ^= 1
        cur.clear()

    for ev in initial:                 # post-reset one-shot(s) on the empty initial bank(s), validated in setup()
        expect(ev)
    closes = sorted(closes); ci = 0
    after_full = False
    for i in range(len(words) + 1):
        if allow_phantom and after_full and not cur:
            j = st["ai"]
            if j < len(actual) and actual[j][0] == "empty":
                assert actual[j][1] == st["phys"], "phantom empty switch on the wrong physical bank"
                st["ai"] += 1; st["phys"] ^= 1
            after_full = False
        while ci < len(closes) and closes[ci] == i:
            close(); ci += 1; after_full = False
        if i == len(words):
            break
        cur.append(words[i])
        if len(cur) == WORDS_PER_BANK:
            close(); after_full = True
    assert ci == len(closes), f"closes beyond the word count: {closes[ci:]}"
    assert st["ai"] == len(actual), (f"{len(actual) - st['ai']} unexpected trailing reader event(s): "
                                     f"{[describe(e) for e in actual[st['ai']:]]}")
    for k, w in enumerate(cur):          # words left in the open bank are in the RAM even though never read
        ram[st["phys"]][k] = w
    return list(cur)


# ----------------------------------------------------------------------------------------------------------------
# Environment
# ----------------------------------------------------------------------------------------------------------------
class Env:
    def __init__(self, dut):
        self.dut = dut
        self.wr = WrSide(dut)
        self.rd = Reader(dut)
        self.closes = []
        self.n_init = 0
        self.init_events = ()

    async def wr_cycles(self, n):
        for _ in range(n):
            await FallingEdge(self.dut.wr_clk)

    async def rd_cycles(self, n):
        for _ in range(n):
            await FallingEdge(self.dut.rd_clk)

    def load_round_robin(self, n):
        """n words spread over the channels in order i%NUM_CH (all channels become valid together)."""
        per = [[] for _ in range(NUM_CH)]
        ws = []
        for i in range(n):
            ch = i % NUM_CH
            w = self.wr.load(ch, 1)[0]
            per[ch].append(w); ws.append(w)
        return ws

    async def wait_quiet(self, quiet=8, timeout=200000):
        """Plan v5 flush-FSM quiet window: no channel valid, skid empty, poller wr_en low for `quiet` cycles."""
        d = self.dut; n = 0
        for _ in range(timeout):
            await FallingEdge(d.wr_clk)
            if self.wr.queues_empty() and int(d.skid_count.value) == 0 and int(d.poller_wr_en.value) == 0 \
                    and int(d.wr_en_out.value) == 0:
                n += 1
                if n >= quiet:
                    return
            else:
                n = 0
        raise AssertionError("wait_quiet timeout")

    async def flush(self):
        await self.wait_quiet()
        mark = len(self.wr.accepted)
        d = self.dut
        d.write_finished_ext.value = 1
        await FallingEdge(d.wr_clk)
        d.write_finished_ext.value = 0
        self.closes.append(mark)
        return mark

    async def wait_flushed(self, timeout_wr_cycles=40000):
        """After flush(): write_finished_out is 1 now; wait for the recovery switch AND the reader's read_finished."""
        d = self.dut
        assert int(d.write_finished_out.value) == 1, "write_finished_out not latched after the flush pulse"
        await self.wait_until(lambda: int(d.write_finished_out.value) == 0 and int(d.dbg_credit.value) == 1,
                              timeout_wr_cycles, what="flush recovery switch + reader done")
        await self.rd_cycles(30)

    def ev(self):
        """Reader events after the post-reset one-shots."""
        return self.rd.events[self.n_init:]

    async def wait_events(self, n, timeout_rd_cycles=40000):
        for _ in range(timeout_rd_cycles):
            if len(self.rd.events) >= n:
                return
            await FallingEdge(self.dut.rd_clk)
        raise AssertionError(f"timeout waiting for {n} reader events, have {len(self.rd.events)}: "
                             f"{[describe(e) for e in self.rd.events]}")

    async def wait_until(self, pred, timeout_wr_cycles=40000, what="condition"):
        for _ in range(timeout_wr_cycles):
            if pred():
                return
            await FallingEdge(self.dut.wr_clk)
        raise AssertionError(f"timeout waiting for {what}")

    def finish_check(self, words=None, allow_phantom=False):
        self.wr.running = False
        assert not self.wr.errors, f"{len(self.wr.errors)} monitor error(s); first: {self.wr.errors[0]}"
        if words is None:
            words = [w for _, w, _ in self.wr.consumed]
            assert self.wr.accepted == words, "cbuf accepted stream != poller consumed stream (skid/gating)"
        assert self.wr.max_skid <= THROTTLE_AT + MAX_SLIP, f"skid occupancy reached {self.wr.max_skid}"
        for s in self.wr.slips + ([self.wr.episode] if self.wr.episode is not None else []):
            assert s <= MAX_SLIP, f"{s} words slipped past the throttle mask in one episode (> {MAX_SLIP})"
        left = check_events(self.rd.events, words, self.closes, allow_phantom, self.init_events)
        self.dut._log.info(f"OK: {len(words)} words, {len(self.rd.events)} reader events "
                           f"({sum(1 for e in self.rd.events if e[0]=='bank')} banks), {len(left)} words left unread, "
                           f"max_skid={self.wr.max_skid}, throttle episodes={len(self.wr.slips)} slips={self.wr.slips}, "
                           f"stall episodes={self.wr.stall_episodes}")
        return left


async def setup(dut, wr_clk=True, wr_period=WR_PERIOD_NS, rd_period=RD_PERIOD_NS):
    seed(dut)
    env = Env(dut)
    d = dut
    d.N_shot_finished.value = 0; d.write_finished_ext.value = 0
    d.ch_valid.value = 0; d.ch_data.value = 0
    d.tb_ovr_sel.value = 0; d.tb_ovr_wr_en.value = 0; d.tb_ovr_wr_data.value = 0
    d.rd_addr.value = 0; d.cbuf_rd_en.value = 0; d.read_finished.value = 0
    d.wr_rst_n.value = 1; d.rd_rst_n.value = 1
    d.wr_clk.value = 0
    cocotb.start_soon(Clock(d.rd_clk, rd_period, unit="ns").start())
    if wr_clk:
        cocotb.start_soon(Clock(d.wr_clk, wr_period, unit="ns").start())
    await Timer(1, unit="ns")
    d.rd_rst_n.value = 0                 # negedge -> async reset branch
    if wr_clk:
        d.wr_rst_n.value = 0
    await Timer(20, unit="ns")
    await FallingEdge(d.rd_clk)
    d.rd_rst_n.value = 1
    if wr_clk:
        await FallingEdge(d.wr_clk)
        d.wr_rst_n.value = 1
        cocotb.start_soon(env.wr.run())
    cocotb.start_soon(env.rd.run())
    await env.rd_cycles(12)
    # F4: the reader owns no bank after reset (able_to_read=0, rd_empty=1), so there is nothing to return. (The
    # vendored buffer presented its empty initial bank(s) and the reader answered with one or two one-shots.)
    assert tuple(env.rd.events) == (), env.rd.events
    assert env.rd.finish_count == 0 and int(d.able_to_read_out.value) == 0 and int(d.rd_empty.value) == 1
    env.init_events = tuple(env.rd.events)
    env.n_init = len(env.rd.events)
    return env


# ================================================================================================================
# (7) bootskew: rd domain alive and out of reset while wr_clk is dead 2 us; first switch still happens.
#     Defined FIRST on purpose: at sim time 0 the wr domain is at true power-up (declaration initialisers only, no
#     reset ever clocked) = the board's boot order. When not run first (COCOTB_TEST_FILTER), the previous test's wr
#     state would leak in, so the async reset branch is applied once with the clock still dead (second variant).
# ================================================================================================================
@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_07_bootskew_wr_clk_dead(dut):
    from cocotb.utils import get_sim_time
    power_up = get_sim_time("ns") == 0
    if not power_up:
        dut.wr_clk.value = 0
        dut.wr_rst_n.value = 1
        await Timer(1, unit="ns")
        dut.wr_rst_n.value = 0           # async reset branch, no clock edge involved
        await Timer(5, unit="ns")
        dut.wr_rst_n.value = 1
    env = await setup(dut, wr_clk=False)
    dut._log.info(f"bootskew variant: {'POWER-UP initialisers only' if power_up else 'async reset branch, clock dead'}")
    # rd domain runs; wr_clk is held low, wr_rst_n never asserted (power-up initial values only)
    empties = {"bad": 0}

    async def mon():
        while True:
            await FallingEdge(dut.rd_clk)
            if int(dut.rd_empty.value) == 0:
                empties["bad"] += 1
    m = cocotb.start_soon(mon())
    await Timer(2000, unit="ns")
    m.cancel()
    assert empties["bad"] == 0, "rd side saw a non-empty bank while the wr domain was dead (power-up init)"
    assert tuple(env.rd.events) == env.init_events and env.rd.finish_count == env.n_init and env.rd.beats == 0
    # the reader's one-shot read_finished above was emitted into a DEAD wr domain: it is lost there.
    cocotb.start_soon(Clock(dut.wr_clk, WR_PERIOD_NS, unit="ns").start())
    await Timer(3, unit="ns")
    dut.wr_rst_n.value = 0
    await env.wr_cycles(5)
    dut.wr_rst_n.value = 1
    cocotb.start_soon(env.wr.run())
    await env.wr_cycles(2)
    assert int(dut.dbg_credit.value) == 1, "credit must be born in the wr domain"
    env.load_round_robin(WORDS_PER_BANK + WORDS_PER_BANK + 20)
    await env.wait_until(lambda: env.wr.queues_empty() and int(dut.skid_count.value) == 0, what="drain")
    await env.flush()
    await env.wait_flushed()
    env.finish_check()
    assert [e[1] for e in env.ev()] == [15, 15, 4] and env.wr.stall_episodes == 0


# ================================================================================================================
# (1) full-bank stall: reader starved -> wr_ready=0 at the last slot; nothing moves, even with wr_en forced
# ================================================================================================================
@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_01_full_bank_stall_wr_ready(dut):
    env = await setup(dut)
    env.rd.hold = True
    K = 30
    env.load_round_robin(WORDS_PER_BANK + (WORDS_PER_BANK - 1) + K)
    # bank 0 fills and switches seamlessly (credit born =1); bank 1 reaches slot 63 with no credit -> stall
    await env.wait_until(lambda: int(dut.wr_ready.value) == 0, what="wr_ready=0")
    snap0 = env.wr.snapshot()
    assert snap0[0] == WORDS_PER_BANK - 1 and snap0[8] == 0, f"stall state {snap0}"
    assert int(dut.dbg_bank_sel_wr.value) == 1, "expected to be stalled in bank 1 after one seamless switch"
    await env.wr_cycles(100)
    assert int(dut.wr_ready.value) == 0 and env.wr.stall_cycles >= 100
    assert int(dut.throttle.value) == 1 and int(dut.skid_count.value) >= THROTTLE_AT, "skid did not fill/throttle"
    # force wr_en with poison data straight into cbuf while wr_ready=0 (Fork A defensive gate)
    env.wr.ovr = lambda cyc, rdy: (1, POISON)
    await env.wr_cycles(40)
    env.wr.ovr = None
    await env.wr_cycles(5)
    forced = [c for c in env.wr.ovr_checks if c[0] == 1]
    assert len(forced) >= 38 and all(c[1] == 0 and c[2] == 0 for c in forced), \
        "wr_en_out must stay 0 while wr_en is forced during wr_ready=0"
    assert env.wr.snapshot() == snap0, "metadata moved during the stall"
    assert int(dut.wr_ready.value) == 0
    # release the reader: bank 0 drains, credit returns, word 63 (from the skid) completes bank 1, then the rest
    env.rd.hold = False
    await env.wait_until(lambda: env.wr.queues_empty() and int(dut.skid_count.value) == 0, what="drain")
    await env.flush()
    await env.wait_flushed()
    env.finish_check()
    assert POISON not in env.wr.accepted
    assert [e[0] for e in env.ev()] == ["bank"] * 3 and env.ev()[2][1] == (K - 1 - 1) // RATIO
    assert env.wr.stall_episodes >= 1


# ================================================================================================================
# (2) wr_accept gating: the cbuf writes exactly the words presented while wr_en && wr_ready
# ================================================================================================================
@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_02_wr_accept_gating(dut):
    env = await setup(dut)
    env.rd.hold = True
    st = {"n": 0, "phase": "A", "forced": 0}
    first_stall = WORDS_PER_BANK + WORDS_PER_BANK - 1

    def ovr(cyc, rdy):
        if st["phase"] == "A":
            en = 1 if random.random() < 0.7 else 0
        elif st["phase"] == "B":
            en = 1
        else:
            en = 1 if random.random() < 0.6 else 0
        w = mkword(0xEE, st["n"])        # a fresh word every cycle; only the ones with en&&ready may land
        if en and rdy:
            st["n"] += 1
        if en and not rdy:
            st["forced"] += 1
        return en, w

    env.wr.ovr = ovr
    await env.wait_until(lambda: len(env.wr.ovr_accepted) == first_stall and int(dut.wr_ready.value) == 0,
                         what="first stall")
    st["phase"] = "B"
    await env.wr_cycles(60)
    assert st["forced"] >= 60 and int(dut.wr_ready.value) == 0
    st["phase"] = "C"
    env.rd.hold = False
    await env.wr_cycles(150)
    env.wr.ovr = None
    await env.wr_cycles(5)
    # every cycle: wr_en_out == wr_en && wr_ready
    bad = [c for c in env.wr.ovr_checks if c[2] != (c[0] & c[1])]
    assert not bad, f"wr_en_out != wr_en && wr_ready in {len(bad)} cycle(s), first {bad[0]}"
    assert len(env.wr.ovr_accepted) > first_stall + 20, "expected writes to resume after the credit returned"
    await env.flush()
    await env.wait_flushed()
    assert env.wr.accepted == env.wr.ovr_accepted, "RAM strobe stream != (wr_en && wr_ready) stream"
    env.finish_check(words=env.wr.ovr_accepted)
    dut._log.info(f"forced wr_en cycles while wr_ready=0: {st['forced']} (all rejected)")


# ================================================================================================================
# (3) seamless switch: reader keeps up -> zero stall, banks alternate, contents exact; F4: the pause after the last
#     full bank does NOT produce an empty presentation (vendored: a write_finished-driven EMPTY recovery switch)
# ================================================================================================================
@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_03_seamless_switch_zero_stall(dut):
    env = await setup(dut)
    NB = 5
    ws = env.load_round_robin(NB * WORDS_PER_BANK)
    await env.wait_until(lambda: len(env.wr.accepted) == NB * WORDS_PER_BANK, what="all words accepted")
    await env.wait_events(env.n_init + NB)
    await env.rd_cycles(400)                      # traffic pauses: nothing more may be presented
    assert len(env.ev()) == NB, f"unexpected presentation(s) after the pause: {[describe(e) for e in env.ev()[NB:]]}"
    assert env.rd.finals == [0] * NB and int(dut.write_finished_out.value) == 0
    assert env.wr.wr_ready_min == 1, "wr_ready dropped although the reader kept up"
    assert env.wr.stall_episodes == 0
    assert env.wr.max_skid <= 1, f"skid accumulated ({env.wr.max_skid}) with a fast reader"
    assert [w for _, w, _ in env.wr.consumed] == ws
    env.finish_check()
    banks = [e for e in env.rd.events if e[0] == "bank"]
    assert [b[3] for b in banks] == [i % 2 for i in range(NB)], "banks did not alternate"
    assert int(dut.dbg_bank_sel_wr.value) == NB % 2 and int(dut.rd_empty.value) == 1


# ================================================================================================================
# (4) recovery switch via flush: partial bank exact (rd_addr_valid, stale lanes), with / without credit present
# ================================================================================================================
@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_04a_flush_partial_bank(dut):
    env = await setup(dut)
    env.load_round_robin(37)                      # -> rows 0..9, lanes 1..3 of row 9 stale (=0, RAM init)
    await env.flush()
    await env.wait_flushed()
    env.load_round_robin(WORDS_PER_BANK + 5)      # seamless switch then 5 words -> rows 0..1, stale lanes = old bank
    await env.flush()
    await env.wait_flushed()
    env.finish_check()
    ev = env.ev()
    assert [e[1] for e in ev] == [9, 15, 1] and [e[3] for e in ev] == [0, 1, 0], [describe(e) for e in ev]
    assert lanes(ev[2][2][1])[1:] == lanes(ev[0][2][1])[1:] != [0, 0, 0], "stale lanes must show the previous bank"


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_04b_flush_zero_words_no_credit(dut):
    env = await setup(dut)
    env.rd.hold = True
    env.load_round_robin(WORDS_PER_BANK)          # seamless switch consumes the credit; reader holds bank 0
    await env.flush()                             # write_finished already 1 from Case A; nothing until credit
    await env.wr_cycles(60)
    assert len(env.ev()) == 0 and int(dut.dbg_bank_sel_wr.value) == 1 and int(dut.write_finished_out.value) == 1
    env.rd.hold = False
    await env.wait_flushed()                      # bank 0, then the empty bank 1 (recovery switch) one-shot
    env.finish_check()
    assert [e[0] for e in env.ev()] == ["bank", "empty"] and env.ev()[1][1] == 1


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_04c_flush_partial_no_credit(dut):
    env = await setup(dut)
    env.rd.hold = True
    env.load_round_robin(WORDS_PER_BANK)
    await env.wait_until(lambda: len(env.wr.accepted) == WORDS_PER_BANK, what="switch")
    env.load_round_robin(20)
    await env.flush()                             # latched: write_finished_out=1, no switch while credit absent
    await env.wr_cycles(100)
    assert int(dut.write_finished_out.value) == 1 and int(dut.dbg_bank_sel_wr.value) == 1
    assert len(env.ev()) == 0
    env.rd.hold = False
    await env.wait_flushed()
    env.finish_check()
    assert [e[0] for e in env.ev()] == ["bank", "bank"] and env.ev()[1][1] == 4 and len(env.ev()[1][2]) == 5


# ================================================================================================================
# (5) throttled pending pipeline: words slipping past the mask <= 2 per episode, skid never overflows, no loss
# ================================================================================================================
@cocotb.test(timeout_time=400, timeout_unit="us")
async def test_05_throttle_pending_pipeline(dut):
    env = await setup(dut)
    env.rd.start_delay = lambda: 150              # slow DDR writer: every bank read starts 150 rd cycles late
    for ch in range(NUM_CH):
        env.wr.load(ch, 30)                       # 420 words = 6 banks + 36
    await env.wait_until(lambda: env.wr.queues_empty() and int(dut.skid_count.value) == 0,
                         timeout_wr_cycles=200000, what="drain")
    env.rd.start_delay = lambda: 0
    await env.flush()
    await env.wait_flushed()
    env.finish_check()
    assert len(env.ev()) == 7
    assert len(env.wr.slips) >= 3, f"throttle engaged only {len(env.wr.slips)} time(s)"
    assert env.wr.stall_episodes >= 3
    assert max(env.wr.slips) <= MAX_SLIP and env.wr.max_skid <= THROTTLE_AT + MAX_SLIP
    dut._log.info(f"slips per throttle episode: {env.wr.slips}, max skid occupancy {env.wr.max_skid}")


# ================================================================================================================
# (6) poller fairness / no loss / one-hot / 1 word per 3 cycles with all 14 channels valid
# ================================================================================================================
@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_06_poller_fairness_no_loss_onehot(dut):
    env = await setup(dut)
    R = 8
    ws = env.load_round_robin(NUM_CH * R)
    await env.wait_until(lambda: env.wr.queues_empty(), what="drain")
    chs = [c for c, _, _ in env.wr.consumed]
    assert chs == list(range(NUM_CH)) * R, f"not round-robin: {chs[:30]}..."
    assert [w for _, w, _ in env.wr.consumed] == ws, "per-channel order / exactly-once violated"
    cyc = [c for _, _, c in env.wr.consumed]
    gaps = [b - a for a, b in zip(cyc, cyc[1:])]
    assert all(g == 3 for g in gaps), f"service interval not 3 cycles: {sorted(set(gaps))}"
    await env.flush()
    await env.wait_flushed()
    env.finish_check()
    assert [e[1] for e in env.ev()] == [15, 11]                         # 64 + 48 words -> rows 0..15, 0..11


# ================================================================================================================
# (8) co-prime clocks, random traffic, random reader delays, 2000 words exact
# ================================================================================================================
@cocotb.test(timeout_time=2000, timeout_unit="us")
async def test_08_coprime_random_2000(dut):
    env = await setup(dut)
    TOTAL = 2000
    # DDR writer model: mostly prompt, sometimes slow enough to stall the write side and engage the throttle
    env.rd.start_delay = lambda: random.choice([0, 0, 0, random.randint(0, 60), random.randint(150, 400)])
    env.rd.beat_gap = lambda: random.choice([0, 0, 0, 1, 2])

    async def injector():
        n = 0
        while n < TOTAL:
            chs = random.sample(range(NUM_CH), random.randint(1, NUM_CH))
            for ch in chs:
                k = min(random.randint(1, 3), TOTAL - n)
                if k <= 0:
                    break
                env.wr.load(ch, k); n += k
            await env.wr_cycles(random.randint(0, 40))
    inj = cocotb.start_soon(injector())
    await inj
    await env.wait_until(lambda: env.wr.queues_empty() and int(dut.skid_count.value) == 0,
                         timeout_wr_cycles=400000, what="drain")
    env.rd.start_delay = lambda: 0
    await env.flush()
    assert len(env.wr.accepted) == TOTAL
    await env.wait_flushed()
    env.finish_check(allow_phantom=False)         # F4: no phantom empty presentation, ever
    banks = sum(1 for e in env.rd.events if e[0] == "bank")
    assert len(env.rd.events) == banks and env.rd.finals == [0] * (banks - 1) + [1]
    assert banks == TOTAL // WORDS_PER_BANK + 1
    assert env.wr.stall_episodes >= 2 and len(env.wr.slips) >= 1, "random run did not exercise stall + throttle"
    dut._log.info(f"stall episodes={env.wr.stall_episodes}, throttle episodes={len(env.wr.slips)}, "
                  f"phantom empties={sum(1 for e in env.rd.events if e[0]=='empty') - env.n_init}")


# ================================================================================================================
# (9) F4: a full bank followed by a traffic pause is NOT followed by an empty presentation; the write bank stays put
#     and the next words land in it (vendored: an EMPTY recovery switch, and the words went to the other bank)
# ================================================================================================================
@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_09_obs_empty_switch_after_full_bank_pause(dut):
    env = await setup(dut)
    env.load_round_robin(WORDS_PER_BANK)
    await env.wait_until(lambda: len(env.wr.accepted) == WORDS_PER_BANK, what="switch")
    assert int(dut.dbg_bank_sel_wr.value) == 1 and int(dut.write_finished_out.value) == 0
    await env.wait_events(env.n_init + 1)         # bank 0
    await env.rd_cycles(400)
    assert len(env.ev()) == 1, f"presentation after the pause: {[describe(e) for e in env.ev()]}"
    assert int(dut.dbg_bank_sel_wr.value) == 1 and int(dut.dbg_credit.value) == 1
    assert int(dut.rd_empty.value) == 1 and int(dut.able_to_read_out.value) == 0
    env.load_round_robin(10)                      # lands in physical bank 1, the current write bank
    await env.flush()
    await env.wait_flushed()
    env.finish_check()
    assert env.ev()[1][3] == 1 and env.ev()[1][1] == 2
    assert env.rd.finals == [0, 1]


# ================================================================================================================
# (10) F4: a write accepted while a flush is pending (no credit yet) does NOT cancel the flush: it joins the FINAL
#      bank. (Vendored: the Case C write cleared write_finished and a second flush was needed. In the SoC the flush
#      FSM closes admission when it pulses write_finished_ext, so no such write exists there anyway.)
# ================================================================================================================
@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_10_obs_late_write_cancels_pending_flush(dut):
    env = await setup(dut)
    env.rd.hold = True
    env.load_round_robin(WORDS_PER_BANK)
    await env.wait_until(lambda: len(env.wr.accepted) == WORDS_PER_BANK, what="switch")
    env.load_round_robin(20)
    await env.flush()                             # pending: write_finished=1, credit=0
    assert int(dut.write_finished_out.value) == 1
    env.load_round_robin(5)
    await env.wait_until(lambda: len(env.wr.accepted) == WORDS_PER_BANK + 25, what="late words")
    env.closes[-1] = WORDS_PER_BANK + 25          # the late words close with the pending flush
    await env.wr_cycles(3)
    assert int(dut.write_finished_out.value) == 1, "the late write cancelled the pending flush"
    env.rd.hold = False
    await env.wait_flushed()                      # bank 0, then ONE final bank with all 25 words
    env.finish_check()
    assert len(env.ev()) == 2 and env.ev()[1][1] == 6 and len(env.ev()[1][2]) == 7
    assert env.rd.finals == [0, 1]


# ================================================================================================================
# (11) F4: rd_final is in band -- exactly the flush-closed presentation of each run carries it -- and after every run
#      the reader owns no bank (rd_empty=1, able_to_read=0), so rd_empty alone is the quiescence predicate
# ================================================================================================================
@cocotb.test(timeout_time=400, timeout_unit="us")
async def test_11_final_flag_and_quiescence_after_every_run(dut):
    env = await setup(dut)
    exp_finals = []
    for n in (0, 1, 63, 64, 65, 128, 130, 37):
        env.load_round_robin(n)
        await env.wait_until(lambda: env.wr.queues_empty() and int(dut.skid_count.value) == 0, what="drain")
        await env.flush()
        await env.wait_flushed()
        exp_finals += [0] * (n // WORDS_PER_BANK) + [1]
        assert env.rd.finals == exp_finals, f"run of {n} words: finals {env.rd.finals} != {exp_finals}"
        for _ in range(60):                       # quiescent: nothing owned, nothing pending
            await FallingEdge(dut.rd_clk)
            assert int(dut.rd_empty.value) == 1 and int(dut.able_to_read_out.value) == 0 \
                and int(dut.rd_final_out.value) == 0, f"after a run of {n} words the reader still owns a bank"
    env.finish_check()
    assert [e[0] for e in env.ev()].count("empty") == 3     # the n=0, n=64 and n=128 runs end on an empty final


# ================================================================================================================
# (12) CDC at an inverted clock ratio: wr (DSP) side 7 ns, rd (DDR) side 2 ns. The reader's return is a toggle, so a
#      one-rd-cycle read_finished cannot be lost in the slower wr domain (the vendored 2-FF pulse sync would miss it).
# ================================================================================================================
@cocotb.test(timeout_time=2000, timeout_unit="us")
async def test_12_inverted_clock_ratio(dut):
    env = await setup(dut, wr_period=7, rd_period=2)
    env.rd.start_delay = lambda: random.choice([0, 0, 1, random.randint(0, 30)])
    TOTAL = 700
    n = 0
    while n < TOTAL:
        for ch in random.sample(range(NUM_CH), random.randint(1, NUM_CH)):
            k = min(random.randint(1, 3), TOTAL - n)
            if k <= 0:
                break
            env.wr.load(ch, k); n += k
        await env.wr_cycles(random.randint(0, 30))
    await env.wait_until(lambda: env.wr.queues_empty() and int(dut.skid_count.value) == 0,
                         timeout_wr_cycles=200000, what="drain")
    await env.flush()
    await env.wait_flushed()
    env.finish_check()
    assert len(env.wr.accepted) == TOTAL and len(env.ev()) == TOTAL // WORDS_PER_BANK + 1
    assert env.rd.finals == [0] * (TOTAL // WORDS_PER_BANK) + [1]


# ================================================================================================================
# (13) poller saturation (plan r2 item 3): all 14 channels continuously valid.
#   A: fast reader -> sustained service of exactly 1 word / 3 wr cycles, strict round-robin (per-channel fairness);
#   B: reader held -> the cbuf stalls, the skid fills to the throttle level and the throttle engages while every
#      channel is valid: words already in the BARREL/ENCODE/CONSUME pipeline when the mask rises still land (the
#      throttle-in-flight capacity), at most MAX_SLIP per episode, and the skid never overflows.
# ================================================================================================================
@cocotb.test(timeout_time=2000, timeout_unit="us")
async def test_13_poller_saturation(dut):
    env = await setup(dut)
    K = 100
    ws = env.load_round_robin(NUM_CH * K)
    await env.wait_until(lambda: env.wr.queues_empty() and int(dut.skid_count.value) == 0,
                         timeout_wr_cycles=200000, what="phase A drain")
    consA = list(env.wr.consumed)
    chs = [c for c, _, _ in consA]
    cyc = [c for _, _, c in consA]
    gaps = [b - a for a, b in zip(cyc, cyc[1:])]
    assert chs == list(range(NUM_CH)) * K, "not strictly round-robin under saturation"
    assert [w for _, w, _ in consA] == ws
    assert set(gaps) == {3}, f"service interval under saturation not exactly 3 cycles: {sorted(set(gaps))}"
    rate = len(consA) / (cyc[-1] - cyc[0] + 3)
    assert rate == 1 / 3, rate
    per_ch = [chs.count(c) for c in range(NUM_CH)]
    assert per_ch == [K] * NUM_CH
    assert env.wr.wr_ready_min == 1 and not env.wr.slips, "phase A should never stall or throttle"
    dut._log.info(f"saturation A: {len(consA)} words in {cyc[-1] - cyc[0] + 3} cycles = {rate:.4f} word/cycle, "
                  f"per-channel {per_ch[0]} each, max_skid={env.wr.max_skid}")
    # phase B: stall the reader; all channels still saturated
    env.rd.hold = True
    nA = len(env.wr.consumed)
    wsB = env.load_round_robin(NUM_CH * K)
    await env.wait_until(lambda: int(dut.throttle.value) == 1 and int(dut.wr_ready.value) == 0, what="throttle")
    await env.wr_cycles(300)
    assert int(dut.throttle.value) == 1 and int(dut.wr_ready.value) == 0
    assert int(dut.poller_data_valid.value) == 0 and int(dut.ch_valid.value) == (1 << NUM_CH) - 1, \
        "saturated channels must all be valid and all masked"
    held = int(dut.skid_count.value)
    env.rd.hold = False
    await env.wait_until(lambda: env.wr.queues_empty() and int(dut.skid_count.value) == 0,
                         timeout_wr_cycles=400000, what="phase B drain")
    consB = env.wr.consumed[nA:]
    assert [c for c, _, _ in consB] == list(range(NUM_CH)) * K, "round-robin lost across throttle episodes"
    assert [w for _, w, _ in consB] == wsB
    await env.flush()
    await env.wait_flushed()
    env.finish_check()                            # MAX_SLIP per episode, max_skid <= THROTTLE_AT + MAX_SLIP, exact data
    assert len(env.wr.slips) >= 1 and env.wr.stall_episodes >= 1
    dut._log.info(f"saturation B: throttle episodes={len(env.wr.slips)}, slips per episode={sorted(set(env.wr.slips))}, "
                  f"skid while stalled={held}, max_skid={env.wr.max_skid} of {SKID_DEPTH} "
                  f"(throttle at {THROTTLE_AT}, in-flight capacity {SKID_DEPTH - THROTTLE_AT})")
