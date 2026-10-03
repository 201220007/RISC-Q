"""`riscq.board.ddr_stream`: the PS-side streaming worker (qubic3 S1), host-pure, over `ddr_live_fake`.

Pinned here: one worker drives a whole run (prepare, start, live chunks, DONE, core reset, FLUSH, certificate) and
ends every stream with exactly one END or ERROR frame; the consumer bound (a slow consumer is waited for at most
`consumer_timeout_s`, then reading stops but the run is still driven to its end and stays recoverable in PL DDR);
the hardware is always left idle (core reset asserted, run flushed, S2MM reset after a failure); and the frame
reader's end-to-end check of what arrived against the certificate.
"""

import json
import threading
import time

import numpy as np
import pytest

from riscq import ddr_regs as R
from riscq.board.ddr_stream import (F_DATA, F_END, F_ERROR, F_WARN, FrameQueue, FrameReader, KernelControl,
                                    StreamWorker, data_frame, frame, iter_frames, json_frame)
from riscq.build import Image, Program
from riscq.ddr import DdrReadout, DdrUplinkError, StreamChunk
from riscq.map import SocMap, SocParams

from tests.ddr_live_fake import BANK, FakeControl, LiveFakeUplink, schedule, tag_word

BASE = 0x20000


def _setup(per_core, **kw):
    words = schedule(4, per_core)
    fake = LiveFakeUplink(num_ch=4, words=words, **kw)
    ro = DdrReadout(fake, legacy_no_ddr_status=True)
    return fake, ro, {c: n for c, n in enumerate(per_core) if n}, words


def _worker(fake, ro, exp, control=None, **kw):
    kw.setdefault("idle", lambda s: None)
    return StreamWorker(ro, BASE, exp, control or FakeControl(fake), **kw)


def _consume(w, reader=None, delay=0.0):
    """Read every frame of the worker's stream; returns (reader, chunks, error)."""
    reader = reader or FrameReader()
    chunks, err = [], None
    while not (w.frames.drained and reader.done) and not reader.done:
        blob = w.frames.get(1 << 20, 0.05)
        if delay:
            time.sleep(delay)
        try:
            chunks += reader.feed(blob)
        except DdrUplinkError as e:
            err = e
            break
        if w.frames.drained and not blob:
            break
    return reader, chunks, err


def test_the_worker_streams_a_run_and_ends_with_its_certificate():
    fake, ro, exp, words = _setup([300, 280, 260, 240], rate=0.2)
    ctl = FakeControl(fake)
    w = _worker(fake, ro, exp, control=ctl, max_chunk=2 * BANK)
    w.start()
    reader, chunks, err = _consume(w)
    w.join(5)
    assert err is None and reader.done and reader.certificate["total"] == 1080
    assert [int(x) for c in chunks for x in c.words] == [tag_word(*x) for x in words]
    st = reader.end["stats"]
    assert st["bytes_before_done"] > 0, "nothing reached PS memory before the program was DONE"
    assert st["bytes"] == 1080 * 8 and st["chunks"] == len(chunks)
    assert ctl.started == 1 and ctl.stopped == 1 and fake.flush_requests == 1
    assert not fake.run_active and fake.sticky >> R.S_WRITE_DONE & 1
    assert st["port"] == "idle" and st["t_first_read"] is not None and st["chunks_before_done"] >= 1
    assert fake.past_frontier() == [] and all(r[4] for r in fake.reads)


def test_a_slow_consumer_is_bounded_and_the_run_stays_recoverable():
    """The consumer reads nothing: the worker waits `consumer_timeout_s` with at most the bound waiting, then stops
    reading DDR, still drives the run to its end (DONE, core reset, FLUSH) and reports a recoverable stall."""
    fake, ro, exp, words = _setup([200, 200, 200, 200], rate=0.5)
    ctl = FakeControl(fake)
    w = _worker(fake, ro, exp, control=ctl, max_chunk=BANK, buffer_bytes=3 * BANK, consumer_timeout_s=0.2)
    t0 = time.monotonic()
    w.start()
    w.join(10)
    assert not w.is_alive() and time.monotonic() - t0 < 5
    assert w.frames.max_data <= 3 * BANK + 2 * 24, "more data waited than the stated bound"
    reader, chunks, err = _consume(w)
    assert err is not None and "ConsumerStalled" in str(err) and "drain() can certify" in str(err)
    assert reader.end["recoverable"] is True and reader.end["retained"] is None and reader.end["port"] == "idle"
    assert ctl.stopped == 1 and not fake.run_active and fake.sticky >> R.S_WRITE_DONE & 1
    out = ro.drain(BASE, exp)                               # the post-run path recovers the whole run
    assert sum(len(v[0]) for v in out.values()) == 800


def test_a_consumer_that_keeps_up_is_never_stalled():
    fake, ro, exp, words = _setup([200, 200, 200, 200], rate=0.5)
    w = _worker(fake, ro, exp, max_chunk=BANK, buffer_bytes=4 * BANK, consumer_timeout_s=5.0)
    w.start()
    reader, chunks, err = _consume(w, delay=0.001)
    w.join(5)
    assert err is None and reader.certificate["total"] == 800
    assert w.frames.max_data <= 4 * BANK + 2 * 24


def test_abort_stops_the_program_and_flushes_the_run():
    fake, ro, exp, words = _setup([2000, 2000, 2000, 2000], rate=0.05)
    ctl = FakeControl(fake)
    w = _worker(fake, ro, exp, control=ctl, max_chunk=BANK)
    w.start()
    reader = FrameReader()
    while reader.next_word < 64:
        reader.feed(w.frames.get(1 << 20, 0.05))
    w.abort()
    w.join(5)
    _, _, err = _consume(w, reader)
    assert err is not None and "aborted by the consumer" in str(err)
    assert ctl.stopped == 1 and not fake.run_active


def test_a_fatal_bit_is_warned_live_and_the_stream_ends_in_error():
    fake, ro, exp, words = _setup([300, 300, 300, 300], rate=0.3)
    fake.at(400, lambda f: setattr(f, "sticky", f.sticky | 1 << R.S_BRESP_ERR))
    w = _worker(fake, ro, exp)
    w.start()
    reader, chunks, err = _consume(w)
    w.join(5)
    assert err is not None and "run invalid (bresp_err)" in str(err)
    assert [x["what"] for x in reader.warnings] == ["bresp_err"]
    assert reader.end["recoverable"] is False
    assert not fake.run_active


def test_a_program_that_never_finishes_is_bounded():
    fake, ro, exp, words = _setup([100, 100, 100, 100], rate=0.5)

    class Never(FakeControl):
        def done(self):
            self.fake.tick()
            return False
    ctl = Never(fake)
    w = _worker(fake, ro, exp, control=ctl, run_timeout_s=0.2, idle=time.sleep, poll_s=1e-3)
    w.start()
    w.join(10)
    _, _, err = _consume(w)
    assert err is not None and "not DONE within" in str(err)
    assert ctl.stopped == 1 and not fake.run_active


def test_a_dma_timeout_with_data_outstanding_is_drained_to_tlast_and_the_port_read_idle():
    """The second chunk's S2MM stalls: the transfer times out with the rest of the chunk still in the uplink, whose
    read lock lasts until that chunk's TLAST (CONTRACT.md I7) and survives FLUSH. Nothing clears it by hand: the
    worker drains the rest of the chunk into a re-armed S2MM, then READS the port idle (DIAG.run_idle), and the
    next prepare() is accepted."""
    fake, ro, exp, words = _setup([100, 100, 100, 100], rate=0.5)
    fake.dma_stall_at = {2}
    w = _worker(fake, ro, exp, max_chunk=BANK)
    w.start()
    w.join(5)
    reader, _, err = _consume(w)
    assert err is not None and "did not complete" in str(err)
    assert any("drained to TLAST" in n for n in reader.end["notes"]), reader.end["notes"]
    assert reader.end["port"] == "idle" and w.port_unusable is None
    assert not fake.rd_locked and not fake.run_active and fake.dma_resets >= 1
    ro.prepare(BASE + 0x10000, exp)                         # BASE_RESET accepted: the port really is free


def test_a_chunk_that_cannot_reach_tlast_leaves_the_port_unusable():
    """If the interrupted chunk cannot be drained to TLAST (the drain engine itself is stuck), the read lock stays:
    the worker reports the port UNUSABLE pending the established recovery, never idle, and BASE_RESET is refused."""
    fake, ro, exp, words = _setup([100, 100, 100, 100], rate=0.5)
    orig = fake.dma_recv_wait

    def wait(buf, n):
        if fake.transfers == 1:                # the second chunk: the drain engine stops for good
            fake.uplink_stuck = True
        return orig(buf, n)
    fake.dma_recv_wait = wait
    w = _worker(fake, ro, exp, max_chunk=BANK)
    w.start()
    w.join(5)
    reader, _, err = _consume(w)
    assert err is not None and "UNUSABLE" in reader.end["port"] and "read lock" in w.port_unusable
    assert reader.end["recoverable"] is False and "drain port is unusable" in reader.end["retained"]
    assert fake.rd_locked
    with pytest.raises(DdrUplinkError, match="base_reset refused"):
        ro.prepare(BASE + 0x10000, exp)


@pytest.mark.parametrize("reset_ok", [True, False], ids=["reset_confirmed", "reset_fails"])
def test_a_chunk_past_tlast_whose_s2mm_fails_is_idle_only_with_a_confirmed_reset(reset_ok):
    """A chunk reaches TLAST -- the uplink's read lock is over, nothing is owed to drain -- but its S2MM then reports
    an error. The uplink alone reads idle. The port is idle, and the complete run recoverable, only once the S2MM's
    soft reset is confirmed; if the reset does not complete, the port is UNUSABLE and nothing is recoverable."""
    fake, ro, exp, words = _setup([100, 100, 100, 100], rate=200)     # every word produced at once: a complete run
    fake.dma_err_after_tlast_at = {2}
    fake.dma_reset_fails = not reset_ok
    w = _worker(fake, ro, exp, max_chunk=BANK)
    w.start()
    w.join(5)
    reader, _, err = _consume(w)
    assert err is not None and "S2MM_DMASR error" in str(err)
    assert not fake.rd_locked and ro.drain_idle() is None, "TLAST ended the lock: the uplink alone reads idle"
    assert fake.drains == 0, "no lock was left, so nothing was drained"
    if reset_ok:
        assert reader.end["port"] == "idle" and w.port_unusable is None
        assert reader.end["recoverable"] is True and reader.end["retained"] is None
        ro.prepare(BASE + 0x10000, exp)
    else:
        assert "UNUSABLE" in reader.end["port"] and "S2MM soft reset was not confirmed" in w.port_unusable
        assert reader.end["recoverable"] is False and "drain port is unusable" in reader.end["retained"]


def test_a_stalled_run_that_fails_its_gates_is_retained_not_recoverable():
    """A stalled consumer leaves the run in PL DDR; it is advertised recoverable only if the certificate's
    prerequisite gates pass. A rejected result fails them: 'data retained in PL DDR; certification pending'."""
    fake, ro, exp, words = _setup([200, 200, 200, 200], rate=0.5)

    def cb(f, off, val):
        if off == R.FLUSH:
            f.rejected[3] = 2
    fake.on_write = cb
    w = _worker(fake, ro, exp, max_chunk=BANK, buffer_bytes=2 * BANK, consumer_timeout_s=0.1)
    w.start()
    w.join(10)
    reader, _, err = _consume(w)
    assert err is not None and "ConsumerStalled" in str(err)
    assert reader.end["recoverable"] is False
    assert reader.end["retained"].startswith("data retained in PL DDR; certification pending: results were rejected")
    assert "certification pending" in str(err) and reader.end["port"] == "idle"


def test_a_chunk_that_lands_after_done_is_not_counted_early():
    """DONE is sampled after each chunk lands; a program that is already DONE then contributes nothing early."""
    fake, ro, exp, words = _setup([100, 100, 100, 100], rate=200)     # the program ends within a few accesses
    w = _worker(fake, ro, exp, max_chunk=BANK)
    w.start()
    reader, chunks, err = _consume(w)
    w.join(5)
    assert err is None and reader.certificate["total"] == 400
    assert reader.end["stats"]["bytes_before_done"] == 0 and reader.end["stats"].get("chunks_before_done") is None


def test_once_caught_up_the_worker_polls_once_per_period():
    """A bank every few time units and a long poll period, on the fake's clock: once the worker has read up to the
    frontier it waits out the rest of the period, so a chunk carries about rate x poll_s bytes -- not the one or two
    banks a spin of the loop finds, which is what made the per-chunk cost dominate at realistic rates."""
    fake, ro, exp, words = _setup([4000] * 4, rate=16.0)               # 4 units per bank, ~1000 units in all
    w = _worker(fake, ro, exp, poll_s=100.0, run_timeout_s=1e9, clock=lambda: float(fake.t),
                idle=lambda s: fake.tick(max(1, round(s))))
    w.start()
    reader, chunks, err = _consume(w)
    w.join(5)
    assert err is None and reader.certificate["total"] == 16000
    assert [int(x) for c in chunks for x in c.words] == [tag_word(*x) for x in words]
    sizes = sorted(len(c.words) * 8 for c in chunks)
    assert len(sizes) <= 16 and sizes[len(sizes) // 2] >= 16 * BANK, sizes


def test_the_programs_end_is_bracketed_even_behind_a_long_chunk_transfer():
    """L4's DONE->END must not hide a backlog behind a late observation of DONE. The worker records the start of the
    last DONE read that still saw the program running (t_still_running) and the end of the first that saw it done
    (t_done). A program that ends during a long chunk transfer lies between the two: t_end - t_still_running is never
    shorter than the true DONE->END, while t_end - t_done is."""
    fake, ro, exp, words = _setup([400] * 4, rate=1.0, dma_units_per_beat=8.0)   # a 4-bank chunk takes ~500 units
    end = {}
    orig = fake._offer

    def offer():
        orig()
        if "t" not in end and fake.offered == len(fake.sched):
            end["t"] = fake.t                                  # the program's true end, on the fake's clock
    fake._offer = offer
    w = _worker(fake, ro, exp, max_chunk=4 * BANK, poll_s=1.0, run_timeout_s=1e9, clock=lambda: float(fake.t),
                idle=lambda s: fake.tick(max(1, round(s))))
    w.start()
    reader, chunks, err = _consume(w)
    w.join(5)
    assert err is None and reader.certificate["total"] == 1600
    st, t_end = reader.end["stats"], end["t"] - w.stream.t_open
    assert st["t_still_running"] < t_end <= st["t_done"], (st, t_end)
    assert st["t_done"] - t_end > 100, "the end must have been seen late, behind a chunk transfer"
    assert st["t_end"] - st["t_still_running"] > st["t_end"] - t_end > st["t_end"] - st["t_done"]


def test_frames_round_trip():
    c = StreamChunk(5, bytes(range(16)), 1.5)
    blob = data_frame(c, 1.0) + json_frame(F_WARN, {"what": "x"}) + frame(F_END, b"{}")
    got = list(iter_frames(blob))
    assert [k for k, _ in got] == [F_DATA, F_WARN, F_END]
    assert got[0][1][16:] == c.data and json.loads(got[1][1]) == {"what": "x"}
    with pytest.raises(DdrUplinkError, match="truncated"):
        list(iter_frames(blob[:-1]))
    with pytest.raises(DdrUplinkError, match="truncated frame header"):
        list(iter_frames(blob + b"\x01"))


def _end(total, acc):
    return json_frame(F_END, {"certificate": {"total": total, "accepted": acc}, "stats": {}})


def _data(first, words):
    return data_frame(StreamChunk(first, np.asarray(words, dtype="<u8").tobytes(), 0.0), 0.0)


def test_the_reader_checks_contiguity_and_the_certificate():
    r = FrameReader()
    r.feed(_data(0, [tag_word(0, 1, 2), tag_word(1, 3, 4)]))
    with pytest.raises(DdrUplinkError, match="DATA for word 3 .* where word 2 was due"):
        r.feed(_data(3, [tag_word(0, 1, 2)]))
    r = FrameReader()
    r.feed(_data(0, [tag_word(0, 1, 2), tag_word(1, 3, 4)]))
    with pytest.raises(DdrUplinkError, match="received 2 words, per core \\[1, 1\\], but the certificate says 2, "
                                             "per core \\[2, 0\\]"):
        r.feed(_end(2, [2, 0]))
    r = FrameReader()
    r.feed(_data(0, [tag_word(0, 1, 2)]) + _end(1, [1, 0]))
    assert r.done and r.certificate["total"] == 1
    with pytest.raises(DdrUplinkError, match="after the stream's last frame"):
        r.feed(_data(1, [tag_word(0, 1, 2)]))
    r = FrameReader()
    with pytest.raises(DdrUplinkError, match="not certified \\(ConsumerStalled\\).*drain\\(\\) can certify"):
        r.feed(json_frame(F_ERROR, {"type": "ConsumerStalled", "error": "x", "recoverable": True, "port": "idle"}))
    r = FrameReader()
    with pytest.raises(DdrUplinkError, match="data retained in PL DDR; certification pending: why"):
        r.feed(json_frame(F_ERROR, {"type": "ConsumerStalled", "error": "x", "recoverable": False,
                                    "retained": "data retained in PL DDR; certification pending: why"}))


def test_the_frame_queue_bounds_data_but_never_control_frames():
    q = FrameQueue(100)
    assert q.put_data(b"a" * 80, 0.0)
    assert not q.put_data(b"b" * 30, 0.05), "a second data frame over the bound must wait, then fail"
    q.put_ctrl(b"c" * 500)                              # control frames are always admitted
    assert q.get(1 << 20, 0.0) == b"a" * 80 + b"c" * 500
    assert q.put_data(b"d" * 300, 0.0), "an empty queue takes one frame of any size"
    got = []
    t = threading.Thread(target=lambda: got.append(q.put_data(b"e" * 10, 2.0)))
    t.start()
    time.sleep(0.05)
    assert q.get(1 << 20, 0.0) == b"d" * 300
    t.join(2)
    assert got == [True] and q.get(1 << 20, 0.0) == b"e" * 10
    q.close()
    assert q.get(1 << 20, 1.0) == b"" and q.drained


def test_a_lingering_get_gathers_frames_but_never_holds_back_a_backlog():
    q = FrameQueue(1000)
    q.put_data(b"a" * 100, 0.0)
    t0 = time.monotonic()
    assert q.get(1 << 20, 0.0, linger=0.1) == b"a" * 100            # nothing more came: back after the linger
    assert 0.08 < time.monotonic() - t0 < 1.0
    q.put_data(b"b" * 300, 0.0)
    q.put_data(b"c" * 300, 0.0)
    t0 = time.monotonic()
    assert q.get(500, 0.0, linger=5.0) == b"b" * 300                 # max_bytes already waiting: no linger
    assert q.get(300, 0.0, linger=5.0) == b"c" * 300
    assert time.monotonic() - t0 < 0.5

    def later():
        time.sleep(0.05)
        q.put_data(b"e" * 100, 0.0)
        time.sleep(0.05)
        q.put_data(b"f" * 400, 0.0)
    q.put_data(b"d" * 100, 0.0)
    th = threading.Thread(target=later)
    th.start()
    t0 = time.monotonic()
    assert q.get(600, 0.0, linger=5.0) == b"d" * 100 + b"e" * 100 + b"f" * 400   # gathered until max_bytes
    assert time.monotonic() - t0 < 1.0
    th.join(2)

    q = FrameQueue(250)                                               # a put waiting for room cuts it short
    q.put_data(b"g" * 200, 0.0)
    th = threading.Thread(target=lambda: q.put_data(b"h" * 100, 5.0))
    th.start()
    time.sleep(0.05)
    t0 = time.monotonic()
    assert q.get(1 << 20, 0.0, linger=5.0) == b"g" * 200
    th.join(2)
    q.close()                                                         # and so does the stream's end
    assert q.get(1 << 20, 0.0, linger=5.0) == b"h" * 100
    assert time.monotonic() - t0 < 1.0


# ── KernelControl: the run-layer sequence, over a recording SoC fake ─────────────────────────────────────────
class RecSoc:
    def __init__(self, m):
        self.m = m
        self.mem = {}
        self.log = []

    def read32(self, addr):
        self.log.append(("r", addr))
        return self.mem.get(addr, 0)

    def write32(self, addr, val):
        self.log.append(("w", addr, val))
        self.mem[addr] = val


def test_kernel_control_writes_params_then_releases_the_reset_and_reads_done():
    from riscq import run as rq
    m = SocMap(SocParams.from_json((__import__("pathlib").Path(__file__).resolve().parents[1]
                                    / "configs" / "sim-dio-antq.json").read_text()))
    img = Image(data=b"", symbols={"__rq_magic": (0x80000100, 4), "n": (0x80000104, 4), "out": (0x80000110, 8)})
    prog = Program(img, params={"n": None}, arrays={"out": 2})
    soc = RecSoc(m)
    for core in (0, 1):
        soc.mem[m.to_host_addr(core, 0x80000100)] = rq.MAGIC
    ctl = KernelControl(soc, m, {0: prog, 1: prog}, params={0: {"n": 7}, 1: {"n": 9}})
    ctl.start()
    writes = [e for e in soc.log if e[0] == "w"]
    assert writes[-1] == ("w", m.host_ctrl + m.HOST_RESET, 0), "the core reset is released last"
    assert ("w", m.to_host_addr(0, 0x80000104), 7) in writes and ("w", m.to_host_addr(1, 0x80000104), 9) in writes
    soc.mem[m.host_ctrl + m.HOST_DONE] = 0b01
    assert not ctl.done()
    soc.mem[m.host_ctrl + m.HOST_DONE] = 0b11
    assert ctl.done()
    ctl.stop()
    assert soc.log[-1] == ("w", m.host_ctrl + m.HOST_RESET, 1)
    with pytest.raises(RuntimeError, match="__rq_magic"):
        soc.mem[m.to_host_addr(1, 0x80000100)] = 0
        ctl.start()
