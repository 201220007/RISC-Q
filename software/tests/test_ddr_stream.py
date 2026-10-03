"""`riscq.ddr.DdrStream`, the live read (qubic3 S1), against the time-stepped uplink model of `ddr_live_fake`.

What is pinned here is the host side of the live read: the frontier (max-hold of CUR_ADDR, the park, the tail
from FINAL_ADDR), the chunking (whole banks, at most `max_chunk`, never past what the writer wrote, never past
the admitted footprint), the S2MM order (armed before RD_START, completion from the DMA), the incremental
accounting, the early warnings, and every gate of `drain()` at write_done. The RTL side of the same claims is
G2's live scenarios (`src/riscq/ddr/sim/ReadoutDdrUplinkSim.scala`) and the co-sim (`test_ddr_cosim.py`).
"""

import numpy as np
import pytest

from riscq import ddr_regs as R
from riscq.ddr import DdrReadout, DdrUplinkError

from tests.ddr_live_fake import BANK, FakeControl, LiveFakeUplink, schedule, tag_word

BASE = 0x10000


def _setup(per_core, num_ch=4, base=BASE, **kw):
    words = schedule(num_ch, per_core)
    fake = LiveFakeUplink(num_ch=num_ch, words=words, **kw)
    ro = DdrReadout(fake, legacy_no_ddr_status=True)
    expected = {c: n for c, n in enumerate(per_core) if n}
    return fake, ro, expected, words


def _run(fake, ro, expected, base=BASE, max_chunk=None, flush=True, hook=None):
    """prepare -> stream -> start -> step until certified (flush once the program is DONE)."""
    ctl = FakeControl(fake)
    ro.prepare(base, expected)
    st = ro.stream(base, expected, max_chunk=max_chunk)
    ctl.start()
    chunks, t_flush = [], None
    for _ in range(200000):
        if st.finished:
            break
        c = st.step()
        if c is not None:
            chunks.append((fake.t, c))
        if hook is not None:
            hook(st, fake)
        if flush and t_flush is None and ctl.done():
            ctl.stop()
            t_flush = fake.t
            ro.flush()
    else:
        raise AssertionError("the stream never finished")
    return st, chunks, t_flush


def _words(chunks):
    return [int(w) for _, c in chunks for w in c.words]


def test_live_read_is_exact_and_certified():
    per_core = [150, 140, 130, 120]                      # 540 words: 8 full banks and a 28-word tail
    fake, ro, exp, words = _setup(per_core)
    st, chunks, t_flush = _run(fake, ro, exp, max_chunk=2 * BANK)
    assert _words(chunks) == [tag_word(*w) for w in words], "the stream is not the run, word for word"
    assert st.certificate["total"] == 540 and st.certificate["accepted"] == per_core
    live = [c for t, c in chunks if t < t_flush]
    assert len(live) >= 4, "data must reach PS memory before the program ends (live, not post-run)"
    # chunking: whole banks before write_done, at most max_chunk, the tail in whole beats
    for t, c in chunks[:-1]:
        assert len(c.data) % BANK == 0 and len(c.data) <= 2 * BANK
    # no read ever asked for bytes the writer had not written, and every read had its S2MM armed first
    assert fake.past_frontier() == [] and all(r[4] for r in fake.reads)
    assert list(st.hist) == per_core and st.stray == 0 and st.warnings == []
    # the post-run path still works on the same run and agrees
    out = ro.drain(BASE, exp)
    for c in range(4):
        assert len(out[c][0]) == per_core[c]


def test_the_frontier_is_max_held_across_the_park():
    """The final bank's B parks CUR_ADDR at run_base: the stream must not fall back, and reads the tail from
    FINAL_ADDR only after write_done."""
    fake, ro, exp, words = _setup([64, 64, 64, 64])           # 256 words: exactly 4 banks, empty final bank
    seen = []

    def hook(st, f):
        seen.append((st.committed, f.cur - f.run_base))
    st, chunks, _ = _run(fake, ro, exp, hook=hook)
    assert any(cur == 0 and com > 0 for com, cur in seen), "the park was never observed by the test"
    committed = [com for com, _ in seen]
    assert committed == sorted(committed), "the frontier went backwards"
    assert st.certificate["final_addr"] == BASE + 256 * 8 and st.certificate["pad"] == 0
    assert _words(chunks) == [tag_word(*w) for w in words]


@pytest.mark.parametrize("n", [0, 1, 3, 4, 5, 63, 64, 65, 127, 128, 129, 300])
def test_small_and_odd_tails(n):
    per_core = [n // 4 + (1 if i < n % 4 else 0) for i in range(4)]
    fake, ro, exp, words = _setup(per_core)
    st, chunks, _ = _run(fake, ro, exp)
    cert = st.certificate
    assert cert["total"] == n and cert["pad"] == (-n) % 4
    assert cert["final_addr"] == BASE + 32 * (-(-n // 4))
    assert _words(chunks) == [tag_word(*w) for w in words]
    assert fake.past_frontier() == []


def test_chunks_are_bounded_by_max_chunk():
    """A backlog (the program finished before the stream read anything) is read in max_chunk pieces."""
    fake, ro, exp, words = _setup([400, 400, 400, 400], rate=50)
    st, chunks, _ = _run(fake, ro, exp, max_chunk=BANK)
    assert max(len(c.data) for _, c in chunks) == BANK
    assert _words(chunks) == [tag_word(*w) for w in words]


def test_max_chunk_must_be_whole_banks():
    fake, ro, exp, _ = _setup([10, 0, 0, 0])
    ro.prepare(BASE, exp)
    for bad in (100, BANK + 32, R.MAX_RD_SIZE + BANK):
        with pytest.raises(ValueError, match="max_chunk"):
            ro.stream(BASE, exp, max_chunk=bad)


def test_overproduction_warns_never_reads_past_the_footprint_and_refuses():
    words = schedule(4, [64, 64, 64, 64])
    fake = LiveFakeUplink(num_ch=4, words=words, extra=schedule(4, [64, 0, 0, 0], salt=3))
    ro = DdrReadout(fake, legacy_no_ddr_status=True)
    exp = {c: 64 for c in range(4)}
    ctl = FakeControl(fake)
    ro.prepare(BASE, exp)
    st = ro.stream(BASE, exp)
    ctl.start()
    after = 0
    with pytest.raises(DdrUplinkError, match="core 0: hardware accepted 128 results, program expected 64"):
        for _ in range(100000):
            st.step()
            if fake.banks_done == 5:                   # the fifth (extra) bank is committed: poll twice, then flush
                after += 1
            if after == 2 and fake.flush_requests == 0:
                ro.flush()
    assert "overproduction" in [w["what"] for w in st.warnings]
    assert st.committed <= st.footprint or st.ended
    assert all(r[1] + r[2] <= BASE + st.footprint for r in fake.reads[:-1] if r[0] < fake.t)


@pytest.mark.parametrize("bit", R.FATAL_BITS, ids=[R.STATUS_NAMES[b] for b in R.FATAL_BITS])
def test_a_fatal_bit_raised_mid_run_is_warned_at_once_and_refuses(bit):
    fake, ro, exp, _ = _setup([200, 200, 200, 200])
    fake.at(300, lambda f: setattr(f, "sticky", f.sticky | 1 << bit))
    t_seen = {}

    def hook(st, f):
        if any(w["what"] == R.STATUS_NAMES[bit] for w in st.warnings) and "t" not in t_seen:
            t_seen["t"] = f.t
    # err_badsize is also what a refused RD_START raises at once (as in drain())
    msg = r"run invalid \(.*%s|rd_start rejected" % R.STATUS_NAMES[bit]
    with pytest.raises(DdrUplinkError, match=msg):
        _run(fake, ro, exp, hook=hook)
    assert t_seen["t"] < 330, "the live warning came late (t=%d, raised at 300)" % t_seen["t"]


def test_a_fatal_bit_cleared_before_the_end_still_refuses():
    """Stronger than drain(): a bit seen during the run fails the certificate even if a W1C cleared it later."""
    fake, ro, exp, _ = _setup([200, 200, 200, 200])
    fake.at(200, lambda f: setattr(f, "sticky", f.sticky | 1 << R.S_RRESP_ERR))
    fake.at(260, lambda f: setattr(f, "sticky", f.sticky & ~(1 << R.S_RRESP_ERR)))
    with pytest.raises(DdrUplinkError, match=r"run invalid \(rresp_err\)"):
        _run(fake, ro, exp)


def test_ovf_any_live_level_warns_and_refuses():
    fake, ro, exp, _ = _setup([200, 200, 200, 200])
    fake.at(250, lambda f: setattr(f, "live", 1 << R.S_OVF_ANY))
    with pytest.raises(DdrUplinkError, match=r"run invalid \(ovf_any\)"):
        st, _, _ = _run(fake, ro, exp)


def _on_final_read(fn):
    """A callback that runs `fn(fake)` once, when the stream reads FINAL_ADDR (inside the write_done gates)."""
    state = {"done": False}

    def cb(fake, off):
        if off == R.FINAL_ADDR and not state["done"]:
            state["done"] = True
            fn(fake)
    return cb


def test_an_error_during_the_tail_read_refuses():
    fake, ro, exp, _ = _setup([100, 100, 100, 100])
    fake.on_read = _on_final_read(lambda f: setattr(f, "sticky", f.sticky | 1 << R.S_RRESP_ERR))
    with pytest.raises(DdrUplinkError, match="error raised DURING the drain"):
        _run(fake, ro, exp, max_chunk=BANK)


def test_write_done_vanishing_before_the_certificate_refuses():
    fake, ro, exp, _ = _setup([100, 100, 100, 100])
    fake.on_read = _on_final_read(lambda f: setattr(f, "sticky", f.sticky & ~(1 << R.S_WRITE_DONE)))
    with pytest.raises(DdrUplinkError, match="write_done vanished"):
        _run(fake, ro, exp, max_chunk=BANK)


def test_run_base_moving_before_the_certificate_refuses():
    fake, ro, exp, _ = _setup([100, 100, 100, 100])
    fake.on_read = _on_final_read(lambda f: setattr(f, "run_base", f.run_base + BANK))
    with pytest.raises(DdrUplinkError, match="run_base changed during the drain"):
        _run(fake, ro, exp, max_chunk=BANK)


def test_rejected_results_warn_with_the_cores_and_refuse():
    fake, ro, exp, _ = _setup([100, 100, 100, 100])
    fake.at(150, lambda f: (f.rejected.__setitem__(2, 3), setattr(f, "sticky", f.sticky | 1 << R.S_EARLY_LATE)))
    st_box = {}

    def hook(st, f):
        st_box["st"] = st
    with pytest.raises(DdrUplinkError, match=r"run invalid \(early_late_result\)"):
        _run(fake, ro, exp, hook=hook)
    w = [w for w in st_box["st"].warnings if w["what"] == "early_late_result"]
    assert w and "rejected per core [0, 0, 3, 0]" in w[0]["detail"]


def test_rejected_without_early_late_still_refuses():
    fake, ro, exp, _ = _setup([20, 20, 20, 20])

    def cb(f, off, val):
        if off == R.FLUSH:
            f.rejected[1] = 1
    fake.on_write = cb
    with pytest.raises(DdrUplinkError, match="results were rejected per core"):
        _run(fake, ro, exp)


def test_accepted_short_of_the_expectation_refuses():
    fake, ro, exp, _ = _setup([20, 20, 20, 20])
    exp[3] = 21
    ctl = FakeControl(fake)
    ro.prepare(BASE, exp)
    st = ro.stream(BASE, exp)
    ctl.start()
    with pytest.raises(DdrUplinkError, match="core 3: hardware accepted 20 results, program expected 21"):
        for _ in range(100000):
            st.step()
            if ctl.done() and fake.flush_requests == 0:
                ro.flush()


def test_a_bad_final_addr_refuses():
    fake, ro, exp, _ = _setup([20, 20, 20, 20])
    fake.on_read = _on_final_read(lambda f: setattr(f, "final", f.final + 32))    # one beat too many: pad 4
    with pytest.raises(DdrUplinkError, match=r"pad 4, expected 0\.\.3"):
        _run(fake, ro, exp)


def test_a_pointer_running_ahead_of_the_data_refuses():
    """CUR_ADDR claims a full bank that the run never filled: FINAL_ADDR then ends below that frontier."""
    fake, ro, exp, _ = _setup([50, 50, 50, 50])            # 200 words: 3 full banks, footprint 4 banks

    def hook(st, f):
        if f.banks_done == 3 and not st.ended and f.flush_at is None:
            f.cur = f.run_base + 4 * BANK                    # a bogus fourth bank
    with pytest.raises(DdrUplinkError, match="below the .* CUR_ADDR showed committed"):
        _run(fake, ro, exp, max_chunk=BANK, hook=hook)


def test_a_reset_mid_run_ends_the_stream_with_an_error():
    """A DDR-side reset clears the register file: run_active falls without write_done."""
    fake, ro, exp, _ = _setup([200, 200, 200, 200])

    def reset(f):
        f.run_active = False
        f.sticky = 0
        f.cur = f.run_base = 0
    fake.at(400, reset)
    with pytest.raises(DdrUplinkError, match="ended without write_done|not a bank boundary"):
        _run(fake, ro, exp)


@pytest.mark.parametrize("cur", ["unaligned", "below", "backwards"])
def test_an_inconsistent_cur_addr_raises(cur):
    fake, ro, exp, _ = _setup([200, 200, 200, 200])

    def hook(st, f):
        if st.committed >= 2 * BANK and not st.ended:
            f.cur = {"unaligned": f.run_base + st.committed + 32, "below": f.run_base - BANK,
                     "backwards": f.run_base + st.committed - BANK}[cur]
    with pytest.raises(DdrUplinkError, match="CUR_ADDR .* not a bank boundary"):
        _run(fake, ro, exp, hook=hook)


def test_a_dma_failure_propagates_and_the_lock_waits_for_tlast():
    """A stalled S2MM: the stream raises, names the chunk in flight, and the read lock (to TLAST, CONTRACT.md I7)
    stays -- FLUSH does not clear it. release_drain() lets the rest of the chunk reach TLAST; then the port reads
    idle. Nothing clears the lock by hand."""
    fake, ro, exp, _ = _setup([100, 100, 100, 100])
    fake.dma_stall_at = {3}
    ctl = FakeControl(fake)
    ro.prepare(BASE, exp)
    st = ro.stream(BASE, exp, max_chunk=BANK)
    ctl.start()
    with pytest.raises(RuntimeError, match="S2MM DMA did not complete"):
        for _ in range(100000):
            st.step()
    base, n = st.inflight
    assert base == BASE + 2 * BANK and n == BANK and fake.rd_locked
    ctl.stop()
    ro.flush()
    assert "read lock" in ro.drain_idle() and fake.rd_locked, "FLUSH must not have cleared the read lock"
    with pytest.raises(DdrUplinkError, match="base_reset refused"):
        ro.prepare(BASE + 0x8000, exp)
    assert ro.release_drain(n) is None and ro.drain_idle() is None
    assert fake.drains == 1 and fake.dma_resets == 1, "one reset, then one drain of the chunk the lock still held"
    st.close()
    ro.prepare(BASE + 0x8000, exp)


def test_release_drain_needs_a_confirmed_s2mm_and_drains_only_a_held_lock():
    """release_drain() returns None only with the S2MM confirmed: its soft reset completed and, where the uplink
    still held the chunk's read lock, the rest of the chunk completed at TLAST. A lock that TLAST already ended is
    owed no drain. A reset that does not complete is reported although the uplink reads idle -- that alone is not
    enough to call the port free."""
    fake, ro, exp, _ = _setup([100, 100, 100, 100])
    _run(fake, ro, exp)
    assert ro.drain_idle() is None
    assert ro.release_drain(BANK) is None and fake.dma_resets == 1 and fake.drains == 0
    fake.dma_reset_fails = True
    why = ro.release_drain(BANK)
    assert why is not None and "S2MM soft reset was not confirmed" in why
    assert ro.drain_idle() is None and fake.drains == 0, "the uplink is idle: only the S2MM is in doubt"


def test_an_error_seen_only_by_the_dma_check_refuses():
    """A fatal bit visible only in the STATUS sample _dma_read() takes after RD_START (gone by the next poll) is in
    the stream's history: the certificate refuses."""
    fake, ro, exp, _ = _setup([200, 200, 200, 200])
    state = {"last_w": None, "done": False}

    def on_write(f, off, val):
        state["last_w"] = off

    def on_read(f, off):
        if off == R.STATUS and state["last_w"] == R.RD_START and not state["done"] and f.reads and len(f.reads) == 3:
            state["done"] = True
            f.live |= 1 << R.S_RRESP_ERR
            f.at(f.t + 1, lambda g: setattr(g, "live", g.live & ~(1 << R.S_RRESP_ERR)))
    fake.on_write, fake.on_read = on_write, on_read
    with pytest.raises(DdrUplinkError, match=r"run invalid \(rresp_err\)"):
        _run(fake, ro, exp, max_chunk=BANK)
    assert state["done"]


def test_an_error_seen_only_by_the_tail_dma_check_refuses():
    """The same after write_done, in the tail's DMA check: 'error raised DURING the drain', from the history, though
    the final STATUS is clean."""
    fake, ro, exp, _ = _setup([100, 100, 100, 100])
    state = {"last_w": None, "done": False}

    def on_write(f, off, val):
        state["last_w"] = off

    def on_read(f, off):
        if off == R.STATUS and state["last_w"] == R.RD_START and f.final and not state["done"]:
            state["done"] = True
            f.live |= 1 << R.S_RRESP_ERR
            f.at(f.t + 1, lambda g: setattr(g, "live", g.live & ~(1 << R.S_RRESP_ERR)))
    fake.on_write, fake.on_read = on_write, on_read
    with pytest.raises(DdrUplinkError, match="error raised DURING the drain \\(rresp_err\\)"):
        _run(fake, ro, exp, max_chunk=BANK)
    assert state["done"] and not fake.live


def test_an_error_seen_only_during_the_flush_refuses():
    """A fatal bit visible only while FLUSH is busy (flush()'s own STATUS polls) is in the history too."""
    fake, ro, exp, _ = _setup([100, 100, 100, 100])
    seen = {"n": 0}

    def on_read(f, off):
        if off == R.STATUS and f.flush_at is not None and not seen["n"]:
            seen["n"] += 1
            f.live |= 1 << R.S_BRESP_ERR
            f.at(f.t + 1, lambda g: setattr(g, "live", g.live & ~(1 << R.S_BRESP_ERR)))
    fake.on_read = on_read
    with pytest.raises(DdrUplinkError, match=r"run invalid \(bresp_err\)"):
        _run(fake, ro, exp)
    assert seen["n"] == 1 and not fake.live


def test_the_poll_log_brackets_every_commit_it_shows():
    """Board L5's production timestamps. With `poll_log` set, every poll records its STATUS and CUR_ADDR reads, each
    bracketed by a timestamp just before and just after it, on the stream's clock (here the fake's own). For every
    bank seen through CUR_ADDR, the commit lies after the start of the previous CUR_ADDR read, which did not show it,
    and no later than the end of the read that first did. Polls that saw write_done read no CUR_ADDR."""
    fake, ro, exp, _ = _setup([400, 400, 400, 400], rate=0.5, commit_lag=5)
    ctl = FakeControl(fake)
    ro.prepare(BASE, exp)
    st = ro.stream(BASE, exp, max_chunk=2 * BANK, clock=lambda: float(fake.t))
    st.poll_log = log = []
    ctl.start()
    for _ in range(200000):
        if st.finished:
            break
        st.step()
        if not st.ended and ctl.done() and fake.run_active and fake.flush_at is None:
            ctl.stop()
            ro.flush()
    assert st.finished and len(log) == st.n_polls
    for ts0, ts1, s, tc0, tc1, cur, committed, ended in log:
        assert ts0 < ts1 and (tc0 is None) == (cur is None) == bool(s >> R.S_WRITE_DONE & 1)
        assert tc0 is None or ts1 <= tc0 < tc1
    reads = [(tc0, tc1, cur - BASE) for _, _, _, tc0, tc1, cur, _, _ in log if tc0 is not None]
    checked = 0
    for b, t_commit in sorted(fake.commit_t.items()):
        end = (b + 1) * BANK
        i = next((k for k, r in enumerate(reads) if r[2] >= end), None)
        if not i:
            continue                                 # first seen at write_done, or at the very first read
        assert reads[i - 1][2] < end and reads[i - 1][0] < t_commit <= reads[i][1], (b, t_commit, reads[i - 1:i + 1])
        checked += 1
    assert checked >= 20


def test_reads_are_cut_to_a_fixed_dma_buffer():
    """A fixed DMA buffer (DdrBoard(grow=False)): drain() cuts its read into buffer-sized transfers, the stream's
    default chunk is the buffer, and a larger max_chunk is refused -- nothing ever asks to reallocate."""
    fake, ro, exp, words = _setup([120, 120, 120, 120], rate=100)
    fake.max_transfer = 1024
    st, chunks, _ = _run(fake, ro, exp)
    assert st.max_chunk == 1024 and max(len(c.data) for _, c in chunks) <= 1024
    before = len(fake.reads)
    out = ro.drain(BASE, exp)
    assert sum(len(v[0]) for v in out.values()) == 480
    assert [r[2] for r in fake.reads[before:]] == [1024, 1024, 1024, 768]
    ro.prepare(BASE + 0x8000, exp)
    with pytest.raises(ValueError, match="max_chunk"):
        ro.stream(BASE + 0x8000, exp, max_chunk=2048)


def test_a_word_with_no_core_tag_warns_and_refuses():
    """A word whose tag names no core (DDR corruption) is an early warning and fails the certificate, although
    ACCEPTED matches the expectation."""
    fake, ro, exp, _ = _setup([64, 64, 64, 64])
    fake.corrupt[10] = 9                                # core 9 does not exist on a 4-core build
    st_box = {}

    def hook(st, f):
        st_box["st"] = st
    with pytest.raises(DdrUplinkError, match="1 words carry no core's tag"):
        _run(fake, ro, exp, hook=hook)
    assert "stray_tag" in [w["what"] for w in st_box["st"].warnings]


def test_the_stream_needs_a_run():
    fake, ro, exp, _ = _setup([10, 10, 10, 10])
    fake.wr_base = fake.run_base = BASE
    with pytest.raises(DdrUplinkError, match="no run to stream"):
        ro.stream(BASE, exp)


def test_the_stream_refuses_a_run_base_mismatch():
    fake, ro, exp, _ = _setup([10, 10, 10, 10])
    ro.prepare(BASE, exp)
    with pytest.raises(DdrUplinkError, match="run_base"):
        ro.stream(BASE + BANK, exp)


def test_the_no_wrap_rule_holds_for_the_stream():
    fake, ro, exp, _ = _setup([64, 0, 0, 0])
    with pytest.raises(ValueError, match="no-wrap"):
        ro.stream(R.RING_LIMIT - BANK // 2, {0: 64})


def test_expected_cores_out_of_range_are_refused():
    fake, ro, exp, _ = _setup([10, 10, 10, 10])
    ro.prepare(BASE, exp)
    with pytest.raises(DdrUplinkError, match="names cores"):
        ro.stream(BASE, {0: 1, 7: 1})


def test_a_run_that_ended_before_the_stream_opened_is_read_whole():
    fake, ro, exp, words = _setup([70, 70, 70, 70], rate=100)
    ctl = FakeControl(fake)
    ro.prepare(BASE, exp)
    ctl.start()
    while not ctl.done():
        pass
    ctl.stop()
    ro.flush()
    st = ro.stream(BASE, exp, max_chunk=BANK)
    chunks = []
    while not st.finished:
        c = st.step()
        if c is not None:
            chunks.append((fake.t, c))
    assert _words(chunks) == [tag_word(*w) for w in words]


def test_accounting_is_incremental_and_exact():
    fake, ro, exp, words = _setup([90, 0, 45, 30])
    counts = []

    def hook(st, f):
        counts.append(st.hist.copy())
    st, chunks, _ = _run(fake, ro, exp, hook=hook)
    assert all((b >= a).all() for a, b in zip(counts, counts[1:])), "the histogram must only grow"
    assert list(st.hist) == [90, 0, 45, 30]
    tags = np.array([w >> 56 for w in _words(chunks)])
    assert list(np.bincount(tags, minlength=4)) == [90, 0, 45, 30]
