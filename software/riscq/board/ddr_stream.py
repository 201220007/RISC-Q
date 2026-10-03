"""The PS-side streaming worker of the readout uplink (qubic3 S1): results reach PS memory while the run runs.

One `StreamWorker` runs one live-read run end to end, next to the MMIO, in its own thread:

    prepare(wr_base, expected)              BASE_RESET: the run is open, its accounting cleared
    control.start()                         the program runs (KernelControl: params, then the core reset released)
    loop:  stream.step()                    one poll of STATUS / CUR_ADDR, at most one chunk into PS memory;
                                            once caught up with the frontier, at most one poll per poll_s
           frames.put_data(chunk)           to the consumer, bounded (below)
           control.done() -> stop(), flush()     DONE ends the run: core reset back on, FLUSH, write_done
    stream.certificate                      every gate of drain() (riscq.ddr.DdrStream)

Bounded. Frames wait for the consumer (the host transport, or code on the PS) in a `FrameQueue` holding at most
`buffer_bytes` of data. A consumer that does not make room within `consumer_timeout_s` stalls the stream: the worker
stops reading DDR -- the rest of the run stays in PL DDR, where nothing overwrites it before the next prepare() --
but still drives the run to its end (DONE, core reset, FLUSH), so the uplink is left idle and `drain()` can certify
the run afterwards. The ERROR frame says so. Nothing here waits without a bound: the DMA (DdrBoard's timeout), the
program (`run_timeout_s`), the flush (`flush_timeout_s`) and the consumer (`consumer_timeout_s`) each have one.

Frames, little-endian: u32 kind, u32 payload length, payload.
    DATA   u64 first word, u64 landing time [ns after the stream opened], the words (8 B each, never pad lanes)
    WARN   JSON {"t", "what", "detail", "sent"}: an early warning (riscq.ddr.DdrStream.warnings)
    END    JSON {"certificate", "stats", "results"}: the run is certified
    ERROR  JSON {"error", "type", "stats", "recoverable", ...}: the run is not certified
END or ERROR is always the last frame of a stream.

Python, not C: the measurement and the reasoning are in qubic3 evidence/C1_LIVE/REPORT.md.
"""

from __future__ import annotations

import contextlib
import json
import struct
import threading
import time
from collections import deque

import numpy as np

from riscq.ddr import DdrUplinkError, StreamChunk
from riscq.ddr_regs import S_FLUSH_BUSY, S_RUN_ACTIVE

F_DATA, F_WARN, F_END, F_ERROR = 1, 2, 3, 4
_HDR = struct.Struct("<II")          # kind, payload bytes
_DATA = struct.Struct("<QQ")         # first word, landing time [ns]


def frame(kind: int, payload: bytes) -> bytes:
    return _HDR.pack(kind, len(payload)) + payload


def json_frame(kind: int, obj) -> bytes:
    return frame(kind, json.dumps(obj, sort_keys=True, default=_jsonable).encode())


def _jsonable(x):
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.ndarray):
        return x.tolist()
    raise TypeError("not JSON-serializable: %r" % type(x))


def data_frame(chunk: StreamChunk, t_open: float) -> bytes:
    payload = _DATA.pack(chunk.first, max(0, int(round((chunk.t - t_open) * 1e9)))) + chunk.data
    return frame(F_DATA, payload)


def iter_frames(blob: bytes):
    """(kind, payload) for each frame of `blob`; a truncated frame is an error (the transport lost bytes)."""
    off, n = 0, len(blob)
    while off < n:
        if n - off < _HDR.size:
            raise DdrUplinkError("stream transport: a truncated frame header at byte %d of %d" % (off, n))
        kind, ln = _HDR.unpack_from(blob, off)
        off += _HDR.size
        if n - off < ln:
            raise DdrUplinkError("stream transport: frame of %d B truncated at byte %d of %d" % (ln, off, n))
        yield kind, blob[off:off + ln]
        off += ln


class FrameReader:
    """The consumer's side of the frames. It checks the framing and the counts: every frame whole, DATA contiguous
    from word 0, and at END the words received, counted per core by their tags, equal to the certificate's
    ACCEPTED -- so a lost, repeated or misplaced DATA frame, or a short stream, cannot pass as a certified run. It
    does not check the payload bits: their integrity and order on the way are TCP's (the transport is one TCP
    connection), and the certificate itself is computed on the PS from the words it read. ERROR raises
    DdrUplinkError."""

    def __init__(self):
        self.next_word = 0
        self.hist = np.zeros(256, dtype=np.int64)
        self.warnings = []
        self.certificate = None
        self.end = None
        self.done = False

    def feed(self, blob: bytes):
        """Process frames; returns the DATA chunks among them, in order."""
        out = []
        for kind, payload in iter_frames(blob):
            if self.done:
                raise DdrUplinkError("stream transport: a frame after the stream's last frame")
            if kind == F_DATA:
                first, t_ns = _DATA.unpack_from(payload, 0)
                data = payload[_DATA.size:]
                if first != self.next_word or len(data) % 8:
                    raise DdrUplinkError("stream transport: DATA for word %d (%d B) where word %d was due"
                                         % (first, len(data), self.next_word))
                self.next_word += len(data) // 8
                if data:
                    self.hist += np.bincount(np.frombuffer(data, dtype=np.uint8)[7::8], minlength=256)
                out.append(StreamChunk(first, data, t_ns * 1e-9))
            elif kind == F_WARN:
                self.warnings.append(json.loads(payload))
            elif kind == F_END:
                end = json.loads(payload)
                cert = end["certificate"]
                acc = cert["accepted"]
                got = self.hist[:len(acc)].tolist()
                if self.next_word != cert["total"] or got != list(acc) or self.hist[len(acc):].any():
                    raise DdrUplinkError("stream transport: received %d words, per core %s, but the certificate "
                                         "says %d, per core %s" % (self.next_word, got, cert["total"], acc))
                self.certificate, self.end, self.done = cert, end, True
            elif kind == F_ERROR:
                err = json.loads(payload)
                self.end, self.done = err, True
                raise DdrUplinkError("streamed run not certified (%s): %s; %s; drain port %s"
                                     % (err.get("type"), err.get("error"),
                                        "its prerequisite gates pass, drain() can certify the data kept in PL DDR"
                                        if err.get("recoverable") else
                                        err.get("retained") or "the run is not certifiable",
                                        err.get("port")))
            else:
                raise DdrUplinkError("stream transport: unknown frame kind %d" % kind)
        return out


class FrameQueue:
    """Frames from the worker to its consumer. DATA frames are bounded: `put_data` waits at most `timeout` for
    room under `limit` bytes and returns False if the consumer made none. Control frames (WARN, END, ERROR) are
    always admitted: they are small, and they are how a stalled consumer learns what happened."""

    def __init__(self, limit: int):
        self.limit = int(limit)
        self._q = deque()            # (frame, is_data)
        self._data = 0
        self._cv = threading.Condition()
        self.closed = False
        self.max_data = 0            # the largest amount of data ever waiting
        self.wait_total = 0.0        # time put_data spent waiting for room
        self.wait_max = 0.0
        self.blocked_puts = 0        # put_data calls that found no room at first and had to wait
        self._put_waiting = False    # a put_data is waiting for room now (a lingering get() then returns)

    def put_data(self, f: bytes, timeout: float, abort: threading.Event | None = None) -> bool:
        with self._cv:
            t0 = time.monotonic()
            deadline = t0 + timeout
            ok = True
            if self._data and self._data + len(f) > self.limit:
                self.blocked_puts += 1
            while self._data and self._data + len(f) > self.limit:   # an empty queue takes any one frame
                left = deadline - time.monotonic()
                if left <= 0 or (abort is not None and abort.is_set()):
                    ok = False
                    break
                self._put_waiting = True
                self._cv.notify_all()
                self._cv.wait(min(left, 0.1))
            self._put_waiting = False
            waited = time.monotonic() - t0
            self.wait_total += waited
            self.wait_max = max(self.wait_max, waited)
            if ok:
                self._q.append((f, True))
                self._data += len(f)
                self.max_data = max(self.max_data, self._data)
                self._cv.notify_all()
            return ok

    def put_ctrl(self, f: bytes) -> None:
        with self._cv:
            self._q.append((f, False))
            self._cv.notify_all()

    def close(self) -> None:
        with self._cv:
            self.closed = True
            self._cv.notify_all()

    def wake(self) -> None:
        with self._cv:
            self._cv.notify_all()

    def get(self, max_bytes: int, timeout: float, linger: float = 0.0) -> bytes:
        """Up to `max_bytes` of whole frames (at least one if any is waiting), or b"" if none arrives in `timeout`
        or the stream is over. With `linger`, once a frame is waiting, up to that long more for `max_bytes` of data
        to gather -- cut short by the stream's end or a put_data waiting for room -- so that a consumer makes few
        large reads instead of one per chunk, and a backlog is still read at full size without waiting."""
        with self._cv:
            deadline = time.monotonic() + timeout
            while not self._q and not self.closed:
                left = deadline - time.monotonic()
                if left <= 0:
                    return b""
                self._cv.wait(left)
            t_end = time.monotonic() + linger
            while self._data < max_bytes and not (self.closed or self._put_waiting):
                left = t_end - time.monotonic()
                if left <= 0:
                    break
                self._cv.wait(left)
            out, n = [], 0
            while self._q and (not out or n + len(self._q[0][0]) <= max_bytes):
                f, is_data = self._q.popleft()
                if is_data:
                    self._data -= len(f)
                out.append(f)
                n += len(f)
            self._cv.notify_all()
            return b"".join(out)

    @property
    def drained(self) -> bool:
        return self.closed and not self._q


class KernelControl:
    """`start` / `done` / `stop` (and `results`) over riscq.run, for programs `riscq.run.setup()` already loaded: the
    middle of `riscq.run.rerun`, split so that a stream runs between the core reset's release and DONE."""

    def __init__(self, drv, m, progs, params=None, arrays=None, results=None):
        self.drv, self.m, self.progs = drv, m, progs
        self.params, self.arrays, self.result_names = params or {}, arrays or {}, results
        self.mask = 0
        for core in progs:
            self.mask |= 1 << core

    def start(self):
        from riscq import run as rq
        rq._check_results_path(self.m, self.progs)
        for core, prog in self.progs.items():
            rq.check_magic(self.drv, self.m, core, prog)
            rq.write_params(self.drv, self.m, core, prog, self.params.get(core, {}))
            for name, values in self.arrays.get(core, {}).items():
                rq.write_array(self.drv, self.m, core, prog, name, values)
        rq.reset(self.drv, self.m, on=False)

    def done(self) -> bool:
        return self.drv.read32(self.m.host_ctrl + self.m.HOST_DONE) & self.mask == self.mask

    def stop(self):
        from riscq import run as rq
        rq.reset(self.drv, self.m, on=True)

    def results(self):
        """The programs' arrays, as rerun returns them (read with the core reset asserted)."""
        from riscq import run as rq
        return {core: {name: rq.read_array(self.drv, self.m, core, prog, name)
                       for name in (list(prog.arrays) if self.result_names is None else self.result_names)}
                for core, prog in self.progs.items()}


class StreamAborted(RuntimeError):
    pass


class StreamWorker(threading.Thread):
    """One streamed run (see the module docstring). `lock` (the board server's) is held for the whole run, because
    the worker owns the MMIO while the run lasts. `clock` / `idle` are the time base and the wait between empty
    polls: time.monotonic / time.sleep on the board, the simulated time base in co-sim."""

    OPTS = ("buffer_bytes", "max_chunk", "poll_s", "consumer_timeout_s", "run_timeout_s", "flush_timeout_s",
            "prepare_timeout_s", "results")

    def __init__(self, ro, wr_base, expected, control, *, lock=None, buffer_bytes=64 << 20, max_chunk=4 << 20,
                 poll_s=100e-6, consumer_timeout_s=10.0, run_timeout_s=600.0, flush_timeout_s=5.0,
                 prepare_timeout_s=1.0, results=False, clock=time.monotonic, idle=time.sleep):
        super().__init__(name="ddr-stream", daemon=True)
        self.ro, self.wr_base, self.expected, self.control = ro, int(wr_base), dict(expected), control
        self.lock = lock
        self.max_chunk, self.poll_s = int(max_chunk), float(poll_s)
        self.consumer_timeout_s, self.run_timeout_s = float(consumer_timeout_s), float(run_timeout_s)
        self.flush_timeout_s, self.want_results = float(flush_timeout_s), bool(results)
        self.prepare_timeout_s = float(prepare_timeout_s)
        self.clock, self.idle = clock, idle
        self.frames = FrameQueue(buffer_bytes)
        self._abort = threading.Event()
        self.stream = None
        self.port_unusable = None     # why the drain port may not be used again (read, never assumed), or None
        self.port_state = "not opened"
        self.stats = {"chunks": 0, "bytes": 0, "polls": 0, "idle_polls": 0, "max_chunk_bytes": 0,
                      "bytes_before_done": 0, "t_first_data": None, "t_done": None, "t_write_done": None,
                      "t_end": None, "buffer_bytes": self.frames.limit}

    def abort(self):
        """Ask the worker to stop: it stops the program, flushes the run and ends the stream with ERROR."""
        self._abort.set()
        self.frames.wake()

    def run(self):
        try:
            with self.lock if self.lock is not None else contextlib.nullcontext():
                self._run()
        except BaseException as e:            # noqa: BLE001 -- the stream must always end with a frame
            self.port_unusable = self.port_unusable or "the worker crashed before checking the port: %r" % e
            self.frames.put_ctrl(json_frame(F_ERROR, {"error": "worker crashed: %r" % e, "type": type(e).__name__,
                                                      "stats": self.stats, "recoverable": False,
                                                      "port": "unknown (worker crashed)"}))
        finally:
            self.frames.close()

    def _run(self):
        ro, ctl, clk = self.ro, self.control, self.clock
        st = None
        prepared = started = prog_done = False
        stalled = None
        err = None
        results = None
        t_start = clk()
        try:
            ro.prepare(self.wr_base, self.expected, timeout=self.prepare_timeout_s)
            prepared = True
            st = self.stream = ro.stream(self.wr_base, self.expected, max_chunk=self.max_chunk, clock=clk)
            t_open = st.t_open
            ctl.start()
            started = True
            nwarn = 0
            while not st.finished:
                if self._abort.is_set():
                    raise StreamAborted("aborted by the consumer")
                t_iter = clk()
                chunk = st.step() if stalled is None else None
                # DONE is sampled AFTER the chunk landed: a chunk counts as early only if the program was still
                # running then (a DONE seen later says nothing about when the chunk arrived)
                done_now = prog_done or ctl.done()
                if st.ended and self.stats["t_write_done"] is None:
                    self.stats["t_write_done"] = clk() - t_open
                while nwarn < len(st.warnings):
                    self.frames.put_ctrl(json_frame(F_WARN, st.warnings[nwarn]))
                    nwarn += 1
                if chunk is not None:
                    self._count(chunk, t_open, early=not done_now)
                if not prog_done:
                    if done_now:
                        prog_done = True
                        self.stats["t_done"] = clk() - t_open      # an upper bound of the program's end
                        ctl.stop()
                        ro.flush(self.flush_timeout_s)
                    elif clk() - t_start > self.run_timeout_s:
                        raise DdrUplinkError("the program was not DONE within %.0f s" % self.run_timeout_s)
                if chunk is not None:
                    if not self.frames.put_data(data_frame(chunk, t_open), self.consumer_timeout_s, self._abort):
                        if self._abort.is_set():
                            raise StreamAborted("aborted by the consumer")
                        stalled = ("the consumer made no room in %.1f s with %d B of data waiting (bound %d B); "
                                   "reading stopped at byte %d of the run"
                                   % (self.consumer_timeout_s, self.frames.max_data, self.frames.limit, st.sent))
                if prog_done and stalled is not None:
                    break                     # the run is complete in PL DDR; nothing more is read
                # once caught up with the frontier, one poll per poll_s (a backlog is read without waiting): a
                # chunk then carries about rate x poll_s bytes, not one bank per spin of the loop
                if not st.finished and (stalled is not None or st.committed <= st.sent):
                    rest = self.poll_s - (clk() - t_iter)
                    if rest > 0:
                        self.stats["idle_polls"] += 1
                        self.idle(rest)
            if stalled is not None:
                raise ConsumerStalled(stalled)
            if self.want_results:
                results = {str(c): {n: np.asarray(a).tolist() for n, a in d.items()}
                           for c, d in ctl.results().items()}
        except Exception as e:                # noqa: BLE001 -- reported in the ERROR frame below
            err = e
        notes = self._leave_idle(prepared, started, prog_done, st)
        self.stats["polls"] = st.n_polls if st is not None else 0
        self.stats["t_first_read"] = (st.t_first_read - st.t_open) if st is not None and st.t_first_read else None
        self.stats["t_end"] = clk() - (st.t_open if st is not None else t_start)
        self.stats.update(queue_max_bytes=self.frames.max_data, consumer_wait_s=round(self.frames.wait_total, 6),
                          consumer_wait_max_s=round(self.frames.wait_max, 6), blocked_puts=self.frames.blocked_puts,
                          port=self.port_state)
        if err is None:
            self.frames.put_ctrl(json_frame(F_END, {"certificate": st.certificate, "stats": self.stats,
                                                    "results": results}))
            return
        pending = None if st is None else (st.certifiable() if self.port_unusable is None else
                                           "the drain port is unusable: %s" % self.port_unusable)
        if st is not None:
            st.close()
        self.frames.put_ctrl(json_frame(F_ERROR, {
            "error": str(err), "type": type(err).__name__, "stats": self.stats, "notes": notes,
            "sent": st.sent if st is not None else 0, "wr_base": self.wr_base, "port": self.port_state,
            # recoverable only if the certificate's prerequisite gates pass on the run as it now is, together with
            # every fault the stream saw, and the drain port is free: then drain() can certify the retained data
            "recoverable": st is not None and pending is None,
            "retained": None if st is None or pending is None else
            "data retained in PL DDR; certification pending: %s" % pending}))

    def _count(self, chunk, t_open, early):
        s = self.stats
        s["chunks"] += 1
        s["bytes"] += len(chunk.data)
        s["max_chunk_bytes"] = max(s["max_chunk_bytes"], len(chunk.data))
        if s["t_first_data"] is None and chunk.data:
            s["t_first_data"] = chunk.t - t_open
        if s["t_first_data"] is not None:
            s["t_last_data"] = chunk.t - t_open
        if early:
            s["bytes_before_done"] += len(chunk.data)
            s["chunks_before_done"] = s.get("chunks_before_done", 0) + 1

    def _leave_idle(self, prepared, started, prog_done, st):
        """Leave the hardware as rerun() leaves it, whatever happened -- core reset asserted, the run flushed -- and
        never claim the drain port free without having read it so. A chunk whose DMA failed keeps the read lock
        until its AXIS TLAST (CONTRACT.md I7), and FLUSH does not clear it: release_drain() lets the rest of that
        chunk drain into a re-armed S2MM up to TLAST, then drain_idle() reads STATUS and DIAG.run_idle, which is
        what the next BASE_RESET needs. If the lock is still held, the port is reported unusable until the
        established recovery (a PL reset, whose DSP-reset hold drains any owed R burst and resets the uplink's DDR
        half; or the image reload of a board session's restore)."""
        notes = []
        if started and not prog_done:
            try:
                self.control.stop()
            except Exception as e:            # noqa: BLE001
                notes.append("core reset failed: %r" % e)
        if prepared:
            try:
                s = self.ro._status()
                if s >> S_RUN_ACTIVE & 1 and not s >> S_FLUSH_BUSY & 1:
                    self.ro.flush(self.flush_timeout_s)
            except Exception as e:            # noqa: BLE001
                notes.append("flush failed: %r" % e)
        if st is not None and st.inflight is not None:
            why = self.ro.release_drain(st.inflight[1])
            notes.append("interrupted chunk at 0x%x (%d B): %s"
                         % (st.inflight[0], st.inflight[1], why or "drained to TLAST, data discarded"))
        try:                                  # read, never assumed; behind the readiness gate (no ui_clk access
            ready = self.ro.ddr_status()      # on an uncalibrated MIG)
            if ready is not None and not all(ready):
                self.port_unusable = "the DDR side is not ready (calib_done, ui_reset_released) = %s" % (ready,)
            else:
                self.port_unusable = self.ro.drain_idle()
        except Exception as e:                # noqa: BLE001
            self.port_unusable = "the port state could not be read: %r" % e
        self.port_state = "idle" if self.port_unusable is None else \
            "UNUSABLE until the established recovery (PL reset or image reload): %s" % self.port_unusable
        return notes


class ConsumerStalled(RuntimeError):
    pass
