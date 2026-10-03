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
    assert err is not None and "ConsumerStalled" in str(err) and "drain() can still certify it" in str(err)
    assert reader.end["recoverable"] is True
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


def test_a_dma_failure_ends_the_stream_and_resets_the_channel():
    fake, ro, exp, words = _setup([100, 100, 100, 100], rate=0.5)
    resets = []
    fake.dma_reset = lambda: resets.append(1)
    orig = fake.dma_recv_wait

    def boom(buf, nbytes):
        if len(fake.reads) == 2:
            fake.armed, fake.rd_locked = None, False
            raise RuntimeError("the S2MM DMA did not complete within 5.0s")
        return orig(buf, nbytes)
    fake.dma_recv_wait = boom
    w = _worker(fake, ro, exp, max_chunk=BANK)
    w.start()
    w.join(5)
    _, _, err = _consume(w)
    assert err is not None and "did not complete" in str(err)
    assert resets == [1] and not fake.run_active


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
    with pytest.raises(DdrUplinkError, match="not certified \\(ConsumerStalled\\).*drain\\(\\) can still"):
        r.feed(json_frame(F_ERROR, {"type": "ConsumerStalled", "error": "x", "recoverable": True}))


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
