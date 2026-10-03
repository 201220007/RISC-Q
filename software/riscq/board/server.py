"""BoardServer: the Pyro5 face of the board (spec 10 §5) — the 4-method Driver, the server-side
batch runner (byte-for-byte the co-sim DriverServer's, spec 08 §5), the RFDC board ops, and the
XSA bundle store. Driver-generic: wraps any object with the Driver methods (PynqDriver in
production, a RAM fake in CI); pynq is only imported inside load(), so this module stays
CI-importable. `python -m riscq.board.server` / `riscq-board-server` starts the daemon."""

from __future__ import annotations

import argparse
import functools
import hashlib
import json
import sys
import threading
from pathlib import Path

import Pyro5.api
import serpent

DEFAULT_BITS = "~/riscq-bits"
DEFAULT_PORT = 9091
CHUNK_MAX = 4 * 1024 * 1024   # spec 10 §4: <= 4 MB per store_chunk
STREAM_DMA_BYTES = 4 << 20    # qubic3 S1: the live stream's (cacheable) S2MM buffer = its largest chunk


def _locked(fn):
    """Every exposed method serializes on one RLock: MMIO and xrfdc are not concurrency-safe and
    Pyro5's threaded daemon interleaves clients. RLock (not Lock) because remote_setup/
    remote_rerun re-enter the locked driver methods on self."""
    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return fn(self, *args, **kwargs)
    return wrapper


@Pyro5.api.expose
class BoardServer:
    """One Pyro5 object; serpent on the wire (Pyro5 default), LAN-trust security model.
    Starts empty (driver ops fail loud) until load() — or CI passes a fake driver directly."""

    def __init__(self, bits_dir: str | Path = DEFAULT_BITS, driver=None,
                 params_text: str | None = None, ddr_port=None, stream_control=None):
        """`ddr_port` is the uplink's driver surface for the live readout (qubic3 S1). A board session passes the
        `DdrBoard` its buffer owner allocated before the fork, so the server never makes a second DMA buffer; by
        default the server makes one fixed (grow=False) cacheable `DdrBoard` around the loaded PynqDriver, and
        every read is cut to its size. `stream_control` is the run control factory (default
        `riscq.board.ddr_stream.KernelControl`), a CI seam."""
        self._lock = threading.RLock()
        self._bits = Path(bits_dir).expanduser()
        self._drv = driver
        self._params = params_text
        self._bundle = None
        self._xsa_sha = None
        self._m = None                # server-side SocMap, built on remote_setup
        self._progs = {}              # core -> Program, rebuilt from the wire on remote_setup
        self._upload = None           # in-flight store_begin state (one at a time)
        self._ddr = ddr_port          # the uplink's driver surface, made on first use (S1)
        self._stream_control = stream_control
        self._stream = None           # the StreamWorker of the current / last streamed run
        self._stream_id = 0

    def _driver(self):
        if self._drv is None:
            raise RuntimeError("no bundle loaded — call load(<bundle>) first")
        return self._drv

    # ── the Driver face ──

    @_locked
    def read32(self, addr):
        return self._driver().read32(int(addr))

    @_locked
    def write32(self, addr, value):
        self._driver().write32(int(addr), int(value))

    @_locked
    def read_block(self, addr, nbytes):
        return self._driver().read_block(int(addr), int(nbytes))

    @_locked
    def write_block(self, addr, data):
        data = serpent.tobytes(data) if isinstance(data, dict) else bytes(data)
        self._driver().write_block(int(addr), data)

    @_locked
    def read_host(self, offset, nbytes):
        """The host result buffer, buffer-relative (specs/software/22 §2.6). In practice this runs
        server-side inside `remote_rerun`; the RPC exists for ad-hoc client reads."""
        return self._driver().read_host(int(offset), int(nbytes))

    @_locked
    def get_host_base(self):
        """Physical base of the driver's CMA result buffer — what `riscq.run.setup` programs into
        `HOSTWIN_BASE_LO/HI`. An antq_uplink build (results_path) has no such buffer."""
        base = self._driver().host_base
        if base is None:
            raise RuntimeError("the loaded build has no host-window buffer (results_path antq_uplink)")
        return int(base)

    # ── handshake ──

    @_locked
    def get_params(self):
        if self._params is None:
            raise RuntimeError("no bundle loaded — call load(<bundle>) first")
        return self._params

    @_locked
    def info(self):
        try:
            from importlib.metadata import version
            riscq_version = version("riscq")
        except Exception:
            riscq_version = "unknown"
        return {"bundle": self._bundle, "xsa_sha": self._xsa_sha,
                "mts_result": getattr(self._drv, "mts_result", None),
                "versions": {"python": sys.version.split()[0], "riscq": riscq_version}}

    # ── server-side batch runner: the SAME riscq.run functions next to the MMIO window, one RPC
    # per batch (spec 08 §5). poll_done takes its hardware branch (no `.sim` here). ──

    @_locked
    def remote_setup(self, params_json, progmap):
        from riscq import run as _run
        from riscq.map import SocMap, SocParams

        mine = SocParams.from_json(self.get_params())
        theirs = SocParams.from_json(params_json)
        if theirs != mine:   # same-build guard: co-sim can't fail this, hardware can (spec 10 §5)
            raise ValueError(f"client SocParams ({theirs.name!r}) != the loaded bundle's "
                             f"({mine.name!r}) — wrong bundle loaded?")
        self._m = SocMap(mine)
        self._progs = {int(c): _run._prog_from_wire(w) for c, w in progmap.items()}
        _run.setup(self._driver(), self._m, self._progs)
        return None

    @_locked
    def remote_rerun(self, cores, params, arrays, results, timeout):
        from riscq import run as _run
        progs = {int(c): self._progs[int(c)] for c in cores}
        out = _run.rerun(self._driver(), self._m, progs,
                         params={int(c): v for c, v in dict(params).items()},
                         arrays={int(c): v for c, v in dict(arrays).items()},
                         results=(None if results is None else list(results)),
                         timeout=int(timeout))
        return {c: {n: bytes(a.astype("<i4").tobytes()) for n, a in d.items()}
                for c, d in out.items()}

    # ── the live readout (qubic3 S1): one streamed run at a time. The worker thread holds the server lock
    # for the whole run (it owns the MMIO meanwhile); `ddr_stream_read` / `ddr_stream_abort` only touch the
    # worker's frame queue, so they are NOT locked and the host can pull while the run is live. ──

    def _ddr_port(self):
        if self._ddr is None:
            from riscq.board.ddr_board import DdrBoard
            self._ddr = DdrBoard(soc=self._driver(), cma_bytes=STREAM_DMA_BYTES, cacheable=True,
                                 wait_poll_s=50e-6, grow=False)
        return self._ddr

    @_locked
    def ddr_stream_start(self, params_json, cores, params, arrays, expected, wr_base, opts=None):
        """Run the programs remote_setup() loaded once, with the live readout: prepare the uplink at `wr_base`,
        start the programs, stream the results as they are committed, end the run at DONE (core reset, FLUSH)
        and certify it. Returns the stream id for `ddr_stream_read`. `opts`: StreamWorker.OPTS."""
        from riscq.board.ddr_stream import KernelControl, StreamWorker
        from riscq.ddr import DdrReadout
        from riscq.map import SocParams

        theirs, mine = SocParams.from_json(params_json), SocParams.from_json(self.get_params())
        if theirs != mine:
            raise ValueError(f"client SocParams ({theirs.name!r}) != the loaded bundle's "
                             f"({mine.name!r}) — wrong bundle loaded?")
        if self._m is None:
            raise RuntimeError("ddr_stream_start before remote_setup")
        if not self._m.params.with_antq_uplink:
            raise RuntimeError(f"{self._m.params.name} has results_path={self._m.params.results_path!r}: "
                               "there is no readout uplink to stream")
        if self._stream is not None and self._stream.is_alive():
            raise RuntimeError("a streamed run is still in progress")
        if self._stream is not None and self._stream.port_unusable is not None:
            raise RuntimeError(f"the DDR drain port is unusable since streamed run {self._stream_id}: "
                               f"{self._stream.port_unusable}. It needs the established recovery (a PL reset, "
                               "or an image reload: load())")
        opts = dict(opts or {})
        bad = sorted(set(opts) - set(StreamWorker.OPTS))
        if bad:
            raise ValueError(f"unknown stream options {bad} (known: {list(StreamWorker.OPTS)})")
        missing = [int(c) for c in cores if int(c) not in self._progs]
        if missing:
            raise ValueError(f"cores {missing} have no program loaded (remote_setup)")
        progs = {int(c): self._progs[int(c)] for c in cores}
        params = {int(c): v for c, v in dict(params or {}).items()}
        arrays = {int(c): v for c, v in dict(arrays or {}).items()}
        make = self._stream_control or KernelControl
        ctl = make(self._driver(), self._m, progs, params=params, arrays=arrays)
        ro = DdrReadout(self._ddr_port(), soc_map=self._m)
        self._stream_id += 1
        self._stream = StreamWorker(ro, int(wr_base), {int(c): int(n) for c, n in dict(expected).items()}, ctl,
                                    lock=self._lock, **opts)
        self._stream.start()
        return self._stream_id

    def _stream_of(self, sid):
        w = self._stream
        if w is None or int(sid) != self._stream_id:
            raise RuntimeError(f"no streamed run {sid} (the current one is {self._stream_id or None})")
        return w

    def ddr_stream_read(self, sid, max_bytes=4 << 20, timeout=1.0, linger=0.0):
        """The next frames of streamed run `sid` (riscq.board.ddr_stream): up to `max_bytes` of whole frames,
        b"" if none arrived within `timeout` s; once one is waiting, up to `linger` s more for `max_bytes` to
        gather (FrameQueue.get). END or ERROR is the last frame."""
        return self._stream_of(sid).frames.get(int(max_bytes), float(timeout), float(linger))

    def ddr_stream_abort(self, sid):
        """Stop streamed run `sid`: the worker stops the programs, flushes the run and ends with ERROR."""
        self._stream_of(sid).abort()

    # ── board ops: thin delegates (spec 10 §3.3) ──

    @_locked
    def mts(self, daclatency=240, adclatency=72):
        return self._driver().mts(daclatency=int(daclatency), adclatency=int(adclatency))

    @_locked
    def refclks(self, lmk_freq, lmx_freq=None):
        self._driver().refclks(lmk_freq, lmx_freq)

    @_locked
    def adc_nyquist_zone(self, n):
        self._driver().adc_nyquist_zone(int(n))

    @_locked
    def dac_nyquist_zone(self, tile, block, n):
        self._driver().dac_nyquist_zone(int(tile), int(block), int(n))

    @_locked
    def dacvop(self, tile, block, uA):
        self._driver().dacvop(int(tile), int(block), int(uA))

    # ── bundle store (spec 10 §4): chunked upload into ~/riscq-bits/<name>/ ──

    @_locked
    def store_begin(self, bundle, filename, nbytes, sha256):
        if self._upload is not None:   # a crashed client's upload: discard it, start fresh
            self._upload["fh"].close()
            self._upload["tmp"].unlink(missing_ok=True)
        d = self._bits / str(bundle)
        d.mkdir(parents=True, exist_ok=True)
        tmp = d / (str(filename) + ".part")
        self._upload = {"fh": open(tmp, "wb"), "tmp": tmp, "final": d / str(filename),
                        "nbytes": int(nbytes), "sha256": str(sha256),
                        "hash": hashlib.sha256(), "got": 0}

    @_locked
    def store_chunk(self, data):
        if self._upload is None:
            raise RuntimeError("store_chunk without store_begin")
        data = serpent.tobytes(data) if isinstance(data, dict) else bytes(data)
        if len(data) > CHUNK_MAX:
            raise ValueError(f"chunk of {len(data)} B > {CHUNK_MAX} B")
        self._upload["fh"].write(data)
        self._upload["hash"].update(data)
        self._upload["got"] += len(data)

    @_locked
    def store_end(self):
        up, self._upload = self._upload, None
        if up is None:
            raise RuntimeError("store_end without store_begin")
        up["fh"].close()
        digest = up["hash"].hexdigest()
        if up["got"] != up["nbytes"] or digest != up["sha256"]:
            up["tmp"].unlink(missing_ok=True)   # no partial file survives a bad upload
            raise ValueError(f"upload of {up['final'].name} corrupt: got {up['got']} of "
                             f"{up['nbytes']} B, sha256 {digest} != {up['sha256']}")
        up["tmp"].replace(up["final"])

    @_locked
    def bundles(self):
        if not self._bits.is_dir():
            return {}
        return {d.name: sorted(f.name for f in d.iterdir()
                               if f.is_file() and f.suffix != ".part")
                for d in sorted(self._bits.iterdir()) if d.is_dir()}

    @_locked
    def load(self, bundle, download=True):
        d = self._bits / str(bundle)
        xsa, params = d / "top.xsa", d / "params.json"
        missing = [p.name for p in (xsa, params) if not p.exists()]
        if missing:
            have = sorted(f.name for f in d.iterdir()) if d.is_dir() else "<no bundle dir>"
            raise FileNotFoundError(f"bundle {bundle!r}: missing {missing} in {d} (have: {have})")
        board_file = d / "board.json"
        board = json.loads(board_file.read_text()) if board_file.exists() else None

        from riscq.board.pynq_driver import PynqDriver   # lazy: only importable on the board
        if self._stream is not None and self._stream.is_alive():
            raise RuntimeError("a streamed run is in progress; abort it before loading another bundle")
        if self._ddr is not None:                         # S1: the stream's DMA buffer goes first
            self._ddr.close()
            self._ddr = None
        self._stream = None                               # a reload is the recovery of an unusable port
        # free the previous driver's CMA result buffer BEFORE allocating the next one: it is
        # 16 MB per core (224 MB on the 14q build), so waiting for the GC to reclaim it would make
        # a reload fail the CMA pre-check for no reason (specs/software/22 §3).
        close = getattr(self._drv, "close", None)
        if close is not None:
            close()
        self._drv = None
        self._drv = PynqDriver(str(xsa), str(params), board=board, download=bool(download))
        self._params = params.read_text()
        self._bundle = str(bundle)
        self._xsa_sha = hashlib.sha256(xsa.read_bytes()).hexdigest()
        self._m, self._progs = None, {}
        return self.info()


def main(argv=None):
    ap = argparse.ArgumentParser(description="riscq board server (spec 10 §5)")
    ap.add_argument("--bits", default=DEFAULT_BITS, help="bundle store dir")
    ap.add_argument("--bundle", default=None, help="load this bundle at startup")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--no-download", action="store_true",
                    help="attach to an already-configured PL (Overlay download=False)")
    args = ap.parse_args(argv)

    server = BoardServer(bits_dir=args.bits)
    if args.bundle:
        server.load(args.bundle, download=not args.no_download)
    daemon = Pyro5.api.Daemon(host=args.host, port=args.port)
    uri = daemon.register(server, objectId="riscq.board")
    print(f"riscq board server @ {uri}   (bundle: {server._bundle})", flush=True)
    daemon.requestLoop()


if __name__ == "__main__":
    main()
