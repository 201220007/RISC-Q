"""The host transport of the live readout (qubic3 S1), end to end over a real Pyro5 daemon on loopback:
`RemoteDriver.ddr_stream` -> `BoardServer.ddr_stream_start` -> the board's `StreamWorker` -> frames pulled by the
host's `DdrStreamClient`, with the time-stepped uplink model of `ddr_live_fake` behind the server. Pinned: the
run reaches the host while it runs and arrives certified and exact; both serializers carry it; a slow host is
bounded by the board (it ends the run with a recoverable error instead of stalling); and the start guards."""

from __future__ import annotations

import threading
import time
from pathlib import Path

import Pyro5.api
import pytest

from riscq import run as rq
from riscq.board.server import BoardServer
from riscq.ddr import DdrMap, DdrUplinkError
from riscq.driver.remote import RemoteDriver
from riscq.map import SocMap, SocParams

from tests.ddr_live_fake import FakeControl, LiveFakeUplink, schedule, tag_word

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
ANTQ = (CONFIGS / "sim-dio-antq.json").read_text()
HOSTWIN = (CONFIGS / "sim-dio.json").read_text()
BASE = 0x40000


class SocFake:
    """The SoC window (PynqDriver's relative addresses): a RAM dict that reports a calibrated MIG."""

    def __init__(self, m):
        self.mem = {m.ddr_status(): m.DDR_STATUS_MAGIC << 16 | 0b11}

    def read32(self, addr):
        return self.mem.get(addr, 0)

    def write32(self, addr, value):
        self.mem[addr] = value & 0xFFFFFFFF

    def write_block(self, addr, data):
        for i in range(0, len(data), 4):
            self.mem[addr + i] = int.from_bytes(bytes(data[i:i + 4]).ljust(4, b"\0"), "little")


class Port:
    """DdrBoard's surface: the uplink windows go to the live model, everything else to the SoC fake."""

    def __init__(self, uplink, soc):
        self.uplink, self.soc, self.map = uplink, soc, DdrMap()

    def _up(self, addr):
        return self.map.ctrl_base <= addr < self.map.ctrl_base + self.map.ctrl_size

    def read32(self, addr):
        return self.uplink.read32(addr) if self._up(addr) else self.soc.read32(addr)

    def write32(self, addr, value):
        (self.uplink.write32 if self._up(addr) else self.soc.write32)(addr, value)

    def dma_recv_prepare(self, n):
        return self.uplink.dma_recv_prepare(n)

    def dma_recv_wait(self, buf, n):
        return self.uplink.dma_recv_wait(buf, n)

    def dma_reset(self):
        return self.uplink.dma_reset()

    def dma_drain_to_tlast(self, n, timeout=1.0):
        return self.uplink.dma_drain_to_tlast(n, timeout)


@pytest.fixture
def board(tmp_path):
    m = SocMap(SocParams.from_json(ANTQ))
    soc = SocFake(m)
    box = {}

    def control(drv, mm, progs, params=None, arrays=None):
        return FakeControl(box["fake"])

    srv = BoardServer(bits_dir=tmp_path / "bits", driver=soc, params_text=ANTQ, ddr_port=None,
                      stream_control=control)
    daemon = Pyro5.api.Daemon(host="127.0.0.1")
    uri = daemon.register(srv, objectId="riscq.board")
    thread = threading.Thread(target=daemon.requestLoop, daemon=True)
    thread.start()
    drv = RemoteDriver(str(uri))

    def new_run(per_core, **kw):
        fake = LiveFakeUplink(num_ch=2, words=schedule(2, per_core), **kw)
        box["fake"] = fake
        srv._ddr = Port(fake, soc)
        return fake

    yield drv, srv, m, new_run
    drv.close()
    daemon.shutdown()
    thread.join(timeout=5)
    daemon.close()


@pytest.mark.parametrize("serializer", ["marshal", "serpent"])
def test_a_streamed_run_reaches_the_host_live_and_certified(board, serializer):
    drv, srv, m, new_run = board
    fake = new_run([400, 380], rate=0.3)
    rq.setup(drv, m, {})                                  # remote_setup: the server's SocMap
    cl = drv.ddr_stream(m, {}, {0: 400, 1: 380}, BASE, serializer=serializer, read_timeout=0.2,
                        poll_s=1e-5, max_chunk=1024)
    words = [int(w) for c in cl for w in c.words]
    assert cl.certificate["total"] == 780 and cl.certificate["accepted"] == [400, 380]
    assert words == [tag_word(*w) for w in schedule(2, [400, 380])]
    assert cl.end["stats"]["bytes_before_done"] > 0, "nothing reached the host before the program was DONE"
    assert fake.past_frontier() == [] and not fake.run_active
    cl.close()


def test_the_host_batches_its_reads(board, monkeypatch):
    """One bank per chunk: lingering, the host pulls many frames per call instead of one call per chunk (the
    per-call cost on the board's A53 is what `linger` amortises)."""
    from riscq.board.ddr_stream import FrameQueue
    calls = []
    get = FrameQueue.get

    def counted(self, max_bytes, timeout, linger=0.0):
        out = get(self, max_bytes, timeout, linger)
        calls.append(len(out))
        return out
    monkeypatch.setattr(FrameQueue, "get", counted)
    drv, srv, m, new_run = board
    new_run([3000, 3000], rate=0.1)
    rq.setup(drv, m, {})
    cl = drv.ddr_stream(m, {}, {0: 3000, 1: 3000}, BASE, read_timeout=0.2, linger=0.1, poll_s=1e-5,
                        max_chunk=512)
    words = [int(w) for c in cl for w in c.words]
    assert words == [tag_word(*w) for w in schedule(2, [3000, 3000])]
    chunks = cl.end["stats"]["chunks"]
    assert chunks >= 90 and len(calls) <= chunks // 2, (len(calls), chunks)
    cl.close()


def test_a_slow_host_cannot_stall_the_board(board):
    """The host reads nothing for a while: the board keeps at most its bound queued, waits consumer_timeout_s,
    then ends the run itself (DONE, flush) with a recoverable error -- the PS never waits on the host forever."""
    drv, srv, m, new_run = board
    fake = new_run([600, 600], rate=0.5)
    rq.setup(drv, m, {})
    cl = drv.ddr_stream(m, {}, {0: 600, 1: 600}, BASE, read_timeout=0.2, poll_s=1e-5, max_chunk=512,
                        buffer_bytes=2048, consumer_timeout_s=0.3)
    t0 = time.monotonic()
    while srv._stream.is_alive() and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    assert not srv._stream.is_alive(), "the board waited on the host without a bound"
    assert srv._stream.frames.max_data <= 2048 + 2 * 24
    with pytest.raises(DdrUplinkError, match="ConsumerStalled.*prerequisite gates pass, drain\\(\\) can certify"):
        for _ in cl:
            pass
    assert fake.sticky >> 2 & 1 and not fake.run_active          # write_done: the run ended in hardware
    cl.close()


def test_a_slow_then_silent_host_blocks_several_enqueues_then_stalls(board):
    """Chunks capped well below the queue budget: a host that reads slowly makes several enqueues wait (bounded),
    then stops reading altogether, and the board ends the run with ConsumerStalled -- never one oversized frame."""
    drv, srv, m, new_run = board
    fake = new_run([1500, 1500], rate=0.5)
    rq.setup(drv, m, {})
    cl = drv.ddr_stream(m, {}, {0: 1500, 1: 1500}, BASE, read_timeout=0.2, read_bytes=600, poll_s=1e-5,
                        max_chunk=512, buffer_bytes=4 * 536, consumer_timeout_s=0.4)
    it = iter(cl)
    for _ in range(8):                                            # slow: one frame every 0.15 s
        next(it)
        time.sleep(0.15)
    t0 = time.monotonic()                                         # then silent
    while srv._stream.is_alive() and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    with pytest.raises(DdrUplinkError, match="ConsumerStalled"):
        for _ in it:
            pass
    st = cl.end["stats"]
    assert st["blocked_puts"] >= 3 and st["max_chunk_bytes"] <= 512 and st["queue_max_bytes"] <= 4 * 536
    cl.close()


def test_an_unusable_port_refuses_the_next_stream(board):
    """A chunk that could not reach TLAST leaves the read lock set: the worker reads the port unusable and the
    server refuses every further stream until a reload (the established recovery)."""
    drv, srv, m, new_run = board
    fake = new_run([300, 300], rate=0.5)
    orig = fake.dma_recv_wait

    def wait(buf, n):
        if fake.transfers == 1:
            fake.uplink_stuck = True
        return orig(buf, n)
    fake.dma_recv_wait = wait
    rq.setup(drv, m, {})
    cl = drv.ddr_stream(m, {}, {0: 300, 1: 300}, BASE, read_timeout=0.2, poll_s=1e-5, max_chunk=512)
    with pytest.raises(DdrUplinkError, match="UNUSABLE"):
        for _ in cl:
            pass
    cl.close()
    with pytest.raises(Exception, match="unusable since streamed run 1"):
        drv._proxy.ddr_stream_start(rq._params_json(m), [], {}, {}, {0: 300, 1: 300}, BASE + 0x10000, {})


def test_the_default_port_is_one_fixed_buffer(tmp_path):
    from riscq.board.server import STREAM_DMA_BYTES
    srv = BoardServer(bits_dir=tmp_path / "b", driver=SocFake(SocMap(SocParams.from_json(ANTQ))), params_text=ANTQ)
    port = srv._ddr_port()
    assert port.max_transfer == STREAM_DMA_BYTES and srv._ddr_port() is port and port._cacheable


def test_the_start_guards(board, tmp_path):
    drv, srv, m, new_run = board
    new_run([10, 10])
    with pytest.raises(Exception, match="before remote_setup"):
        drv._proxy.ddr_stream_start(rq._params_json(m), [], {}, {}, {0: 10, 1: 10}, BASE, {})
    rq.setup(drv, m, {})
    other = SocMap(SocParams.from_json(HOSTWIN))
    with pytest.raises(Exception, match="wrong bundle"):
        drv._proxy.ddr_stream_start(rq._params_json(other), [], {}, {}, {0: 10}, BASE, {})
    with pytest.raises(Exception, match="unknown stream options"):
        drv._proxy.ddr_stream_start(rq._params_json(m), [], {}, {}, {0: 10}, BASE, {"bogus": 1})
    with pytest.raises(Exception, match="no program loaded"):
        drv._proxy.ddr_stream_start(rq._params_json(m), [1], {}, {}, {0: 10}, BASE, {})
    with pytest.raises(Exception, match="no streamed run"):
        drv._proxy.ddr_stream_read(99, 1024, 0.0)


def test_a_hostwindow_bundle_has_nothing_to_stream(tmp_path):
    m = SocMap(SocParams.from_json(HOSTWIN))
    srv = BoardServer(bits_dir=tmp_path / "b", driver=SocFake(SocMap(SocParams.from_json(ANTQ))),
                      params_text=HOSTWIN)
    srv._m = m
    with pytest.raises(RuntimeError, match="no readout uplink"):
        srv.ddr_stream_start(rq._params_json(m), [], {}, {}, {0: 1}, BASE)
