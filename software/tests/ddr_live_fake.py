"""A time-stepped model of the readout uplink for the live-read tests (qubic3 S1).

`tests/test_ddr_driver.py`'s `FakeUplink` holds a FINISHED run. A live read needs the run to happen while it
is read, so this model advances one time unit per register access (and a few per DMA transfer):

  - a producer accepts the scheduled words at `rate` words per unit while the run is active and the program
    runs (`start()`), and stops at FLUSH: later words are rejected (REJECTED, early_late), as in the RTL;
  - the writer commits a full 512-B bank `commit_lag` units after its 64th word, and only then moves CUR_ADDR;
    FLUSH commits the partial final bank after `flush_lag` units, latches FINAL_ADDR, parks CUR_ADDR at run_base
    and sets write_done (I3/I4/I7 of `src/riscq/ddr/CONTRACT.md`);
  - DDR outside what was written holds 0xEE garbage, so a read past the frontier returns wrong data;
  - the drain engine + S2MM follow the contract's rules: RD_BASE/RD_SIZE/RD_START are refused (err_badsize) from
    an accepted RD_START until the chunk's DMA completed, and a RD_START with no S2MM armed loses the stream
    (the DMA wait then times out). Every RD_START is logged with the frontier it was issued against.

`events` injects faults at a time: `fake.at(t, fn)`. S2MM faults: `dma_stall_at` (no TLAST: the lock stays),
`dma_err_after_tlast_at` (TLAST, then an S2MM error), `dma_reset_fails`, `uplink_stuck`.
"""

from __future__ import annotations

from collections import deque

from riscq import ddr_regs as R
from riscq.ddr import DdrMap

BANK = 512
WORDS_PER_BANK = BANK // R.WORD_BYTES


def tag_word(tag, real, imag):
    return (tag << 56) | ((real & 0xFFFFFFFF) >> 4 << 28) | ((imag & 0xFFFFFFFF) >> 4)


def schedule(num_ch, per_core, salt=0):
    """Round-robin words: per_core[c] results on core c, in arrival order. Returns [(core, real, imag)]."""
    left = list(per_core)
    out, k = [], 0
    while any(left):
        for c in range(num_ch):
            if left[c]:
                out.append((c, (0x100000 * (salt + 1) + 16 * k + c) & 0x7FFFFFF0, (0x7000000 + 16 * k + c)))
                left[c] -= 1
                k += 1
    return out


class LiveFakeUplink:
    def __init__(self, num_ch=4, words=(), rate=0.25, commit_lag=3, flush_lag=6, mem_bytes=1 << 22,
                 dma_units_per_beat=0.05, extra=()):
        self.num_ch = num_ch
        self.map = DdrMap()
        self.regs = {R.NUM_CH: num_ch, R.GEOMETRY: (8 << 24) | (4 << 16) | (8 << 8) | 16}
        self.mem = bytearray(b"\xee") * mem_bytes
        self.sched = [tag_word(c, re, im) for c, re, im in words]
        self.extra = [tag_word(c, re, im) for c, re, im in extra]   # produced after the schedule: overproduction
        self.offer_list = self.sched + self.extra
        self.rate, self.commit_lag, self.flush_lag = rate, commit_lag, flush_lag
        self.dma_units_per_beat = dma_units_per_beat
        self.t = 0
        self._acc = 0.0
        self.running = False          # the program runs (the control's start/stop)
        self.run_active = False
        self.flush_at = None
        self.run_base = 0
        self.wr_base = 0
        self.cur = 0
        self.final = 0
        self.offered = 0              # scheduled words the program has produced (accepted or rejected)
        self.produced = 0             # words accepted in this run
        self.prod_list = []           # the accepted words, in DDR order
        self.banks_due = deque()      # (t_due, bank)
        self.banks_done = 0
        self.written = 0              # bytes of the run written to DDR (the safe region)
        self.sticky = 0
        self.live = 0                 # live (non-sticky) status bits to OR in
        self.accepted = [0] * num_ch
        self.rejected = [0] * num_ch
        self.counts = [0] * num_ch
        self.rd_locked = False
        self.armed = None
        self.rd_base = self.rd_size = 0
        self.reads = []               # (t, base, size, written_at_start, armed_ok)
        self.events = []              # (t, fn)
        self.flush_requests = 0
        self.dma_stall_at = set()     # transfer numbers (1-based) whose S2MM stalls: no TLAST, the read lock stays
        self.dma_err_after_tlast_at = set()   # transfers whose S2MM takes the chunk up to TLAST (the lock ends), then
                                              # reports an error instead of completing
        self.dma_reset_fails = False  # the S2MM's soft reset never completes
        self.uplink_stuck = False     # the drain engine itself is stuck: no chunk reaches TLAST any more
        self.transfers = 0
        self.dma_resets = 0
        self.drains = 0               # dma_drain_to_tlast calls
        self.max_transfer = None      # a fixed DMA buffer (DdrBoard(grow=False)): the largest transfer, or None
        self.on_read = None           # fn(fake, off), called before a register read is answered
        self.on_write = None          # fn(fake, off, val), called before a register write takes effect
        self.corrupt = {}             # word index -> tag written instead of the real one (DDR corruption)

    # -- fault injection ------------------------------------------------------------------
    def at(self, t, fn):
        self.events.append((t, fn))

    # -- time -------------------------------------------------------------------------------
    def tick(self, n=1):
        for _ in range(int(n)):
            self.t += 1
            for ev in [e for e in self.events if e[0] <= self.t]:
                self.events.remove(ev)
                ev[1](self)
            if self.running:
                self._acc += self.rate
                while self._acc >= 1:
                    self._acc -= 1
                    self._offer()
            while self.banks_due and self.banks_due[0][0] <= self.t:
                _, bank = self.banks_due.popleft()
                self._write_words(bank * WORDS_PER_BANK, (bank + 1) * WORDS_PER_BANK)
                self.banks_done = bank + 1
                self.cur = self.run_base + (bank + 1) * BANK
            if self.flush_at is not None and self.t >= self.flush_at and not self.banks_due and self.run_active:
                self._final_bank()

    def _offer(self):
        if self.offered >= len(self.offer_list):
            return
        w = self.offer_list[self.offered]
        self.offered += 1
        core = w >> 56
        if not self.run_active or self.flush_at is not None:     # admission closed: rejected, flagged
            self.rejected[core] += 1
            self.sticky |= 1 << R.S_EARLY_LATE
            return
        self.prod_list.append(w)
        self.counts[core] += 1
        self.produced += 1
        if self.produced % WORDS_PER_BANK == 0:
            self.banks_due.append((self.t + self.commit_lag, self.produced // WORDS_PER_BANK - 1))

    def _write_words(self, a, b):
        for i in range(a, b):
            off = self.run_base + 8 * i
            w = self.prod_list[i]
            if i in self.corrupt:
                w = (w & ((1 << 56) - 1)) | self.corrupt[i] << 56
            self.mem[off:off + 8] = int(w).to_bytes(8, "little")
        self.written = max(self.written, 8 * b)

    def _final_bank(self):
        full = self.banks_done * WORDS_PER_BANK
        self._write_words(full, self.produced)
        nbeats = -(-self.produced // 4)
        self.written = 32 * nbeats                      # the final beat's pad lanes count as written (stale RAM)
        self.final = self.run_base + 32 * nbeats
        self.cur = self.run_base                        # the park
        self.run_active = False
        self.flush_at = None
        self.sticky |= 1 << R.S_WRITE_DONE

    def done(self):
        """The program finished: it produced every scheduled word."""
        return self.offered >= len(self.sched)

    # -- register surface -----------------------------------------------------------------
    def _status(self):
        s = self.sticky | self.live
        if self.run_active:
            s |= 1 << R.S_RUN_ACTIVE | 1 << R.S_DSP_ADMIT
        if self.flush_at is not None:
            s |= 1 << R.S_FLUSH_BUSY
        if self.rd_locked:
            s |= 1 << R.S_RD_BUSY
        return s

    def read32(self, addr):
        self.tick()
        off = addr - self.map.ctrl_base
        if self.on_read is not None:
            self.on_read(self, off)
        if off == R.STATUS:
            return self._status()
        if off == R.RUN_BASE:
            return self.run_base
        if off == R.WR_BASE:
            return self.wr_base
        if off == R.CUR_ADDR:
            return self.cur
        if off == R.FINAL_ADDR:
            return self.final
        if off == R.RD_BASE:
            return self.rd_base
        if off == R.RD_SIZE:
            return self.rd_size
        if R.ACCEPTED <= off < R.ACCEPTED + 4 * 32:
            return self.accepted[(off - R.ACCEPTED) // 4]
        if R.REJECTED <= off < R.REJECTED + 4 * 32:
            return self.rejected[(off - R.REJECTED) // 4]
        if off == R.DIAG:                          # [0] writer_idle, [1] cbuf_rd_empty, [7] run_idle, [9] calib
            writer_idle = not self.banks_due
            run_idle = not self.run_active and self.flush_at is None and not self.rd_locked and writer_idle
            return int(writer_idle) | 1 << 1 | int(run_idle) << 7 | 1 << 9
        return self.regs.get(off, 0)

    def write32(self, addr, val):
        self.tick()
        off = addr - self.map.ctrl_base
        if self.on_write is not None:
            self.on_write(self, off, val)
        if off == R.STATUS:
            self.sticky &= ~(val & R.STICKY_MASK)
        elif off == R.WR_BASE:
            if val % R.WR_BASE_ALIGN or val >= R.RING_LIMIT:
                self.sticky |= 1 << R.S_ERR_BADBASE
            else:
                self.wr_base = val
        elif off == R.BASE_RESET:
            if self.run_active or self.rd_locked:
                self.sticky |= 1 << R.S_ERR_BASE_BUSY
                return
            self.run_base = self.cur = self.wr_base
            self.final = 0
            self.sticky &= ~R.STICKY_MASK
            self.accepted = [0] * self.num_ch
            self.rejected = [0] * self.num_ch
            self.counts = [0] * self.num_ch
            self.produced, self.prod_list, self.banks_done, self.written = 0, [], 0, 0
            self._acc = 0.0
            self.banks_due.clear()
            self.run_active = True
        elif off == R.FLUSH:
            self.flush_requests += 1
            if not self.run_active or self.flush_at is not None:
                self.sticky |= 1 << R.S_ERR_FLUSH_REFUSED
                return
            self.accepted = list(self.counts)          # the snapshot at the flush commit
            self.flush_at = self.t + self.flush_lag
        elif off in (R.RD_BASE, R.RD_SIZE):
            if self.rd_locked:
                self.sticky |= 1 << R.S_ERR_BADSIZE
            elif off == R.RD_BASE:
                self.rd_base = val
            else:
                self.rd_size = val
        elif off == R.RD_START:
            ok = (32 <= self.rd_size <= R.MAX_RD_SIZE and self.rd_size % 32 == 0 and self.rd_base % 32 == 0)
            if self.rd_locked or not ok:
                self.sticky |= 1 << R.S_ERR_BADSIZE
                return
            self.rd_locked = True
            self.reads.append((self.t, self.rd_base, self.rd_size, self.run_base + self.written,
                               self.armed == self.rd_size))
        else:
            self.regs[off] = val

    # -- the S2MM DMA -------------------------------------------------------------------
    def dma_recv_prepare(self, nbytes):
        if self.armed is not None:
            raise RuntimeError("an S2MM transfer is already in flight")
        self.armed = int(nbytes)
        return ("buf", self.armed)

    def dma_recv_wait(self, buf, nbytes):
        if buf != ("buf", self.armed) or nbytes != self.armed:
            raise RuntimeError("dma_recv_wait(%d) does not match the armed transfer" % nbytes)
        if self.max_transfer is not None and nbytes > self.max_transfer:
            raise ValueError("a %d-B transfer does not fit the fixed %d-B DMA buffer" % (nbytes, self.max_transfer))
        self.armed = None
        self.transfers += 1
        if self.transfers in self.dma_stall_at or self.uplink_stuck:
            # TREADY stayed low (a stalled or failed S2MM) or the engine is stuck: the chunk never reached TLAST,
            # so its read lock stays set (CONTRACT.md I7) -- nothing here clears it
            raise RuntimeError("the S2MM DMA did not complete within 5.0s (fake: transfer %d stalled)" % self.transfers)
        if not self.rd_locked or not self.reads or not self.reads[-1][4]:
            raise RuntimeError("S2MM timeout: the stream started before the channel was armed (or never)")
        self.tick(1 + nbytes // 32 * self.dma_units_per_beat)
        data = bytes(self.mem[self.rd_base:self.rd_base + nbytes])
        self.rd_locked = False
        if self.transfers in self.dma_err_after_tlast_at:
            # the whole chunk reached TLAST, so the uplink's lock is over; the S2MM itself failed (e.g. SlvErr writing
            # PS memory) -- nothing here says whether its channel is quiescent
            raise RuntimeError("S2MM_DMASR error during the drain: 0x00000020 (fake: transfer %d, after TLAST)"
                               % self.transfers)
        return data

    def dma_reset(self):
        self.dma_resets += 1
        if self.dma_reset_fails:
            raise RuntimeError("the S2MM soft reset did not clear within 1 s (fake)")
        self.armed = None

    def dma_drain_to_tlast(self, nbytes, timeout=1.0):
        """Arm for the rest of an interrupted chunk and take it up to TLAST: that ends the read lock. Fails if the
        drain engine is stuck, or if there is no chunk to finish (the real S2MM would wait for data in vain)."""
        self.drains += 1
        self.tick(2)
        if self.uplink_stuck or not self.rd_locked:
            raise RuntimeError("the S2MM DMA did not complete within %ss (fake: no TLAST)" % timeout)
        self.rd_locked = False

    # -- what a test checks ---------------------------------------------------------------
    def past_frontier(self):
        """RD_STARTs that asked for bytes the writer had not written yet."""
        return [r for r in self.reads if r[1] + r[2] > r[3]]


class FakeControl:
    """The run control the worker drives (`start` / `done` / `stop`), over the fake's producer."""

    def __init__(self, fake):
        self.fake = fake
        self.started = self.stopped = 0

    def start(self):
        self.started += 1
        self.fake.running = True

    def done(self):
        self.fake.tick()
        return self.fake.done()

    def stop(self):
        self.stopped += 1
        self.fake.running = False
