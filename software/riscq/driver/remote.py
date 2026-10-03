"""RemoteDriver: Pyro5 proxy to the board server (spec 10 §6). Mirrors CosimDriver — the 4
Driver methods proxy 1:1, `.remote` is set UNCONDITIONALLY (run.setup/rerun route server-side,
one RPC per batch, so LAN latency never multiplies per register poke), `.board` exposes the
RFDC ops + bundle store. No `.sim` attribute: sim-only operations don't exist on hardware, and
poll_done (only ever executed server-side) uses its hardware branch."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import Pyro5.api

from riscq.board.ddr_stream import FrameReader
from riscq.driver.cosim import _RemoteExtras, _to_bytes

CHUNK = 4 * 1024 * 1024   # bundle-upload chunk size (spec 10 §4: <= 4 MB, one in flight)


class _BoardExtras:
    """The board-ops + store surface of the server (spec 10 §5), thin delegates."""

    def __init__(self, proxy: Pyro5.api.Proxy):
        self._proxy = proxy

    def get_params(self) -> str:
        """The loaded bundle's SocParams JSON (the config handshake, spec 04 §2)."""
        return self._proxy.get_params()

    def info(self) -> dict:
        return self._proxy.info()

    def mts(self, daclatency: int = 240, adclatency: int = 72) -> int:
        return self._proxy.mts(daclatency, adclatency)

    def refclks(self, lmk_freq: float, lmx_freq: float | None = None) -> None:
        self._proxy.refclks(lmk_freq, lmx_freq)

    def adc_nyquist_zone(self, n: int) -> None:
        self._proxy.adc_nyquist_zone(int(n))

    def dac_nyquist_zone(self, tile: int, block: int, n: int) -> None:
        self._proxy.dac_nyquist_zone(int(tile), int(block), int(n))

    def dacvop(self, tile: int, block: int, uA: int) -> None:
        self._proxy.dacvop(int(tile), int(block), int(uA))

    def bundles(self) -> dict:
        return self._proxy.bundles()

    def load(self, bundle: str, download: bool = True) -> dict:
        """Construct/replace the server's PynqDriver from a stored bundle (full bring-up)."""
        return self._proxy.load(str(bundle), bool(download))


class RemoteDriver:
    """Driver over Pyro5 to the board server. `host` is a hostname/IP (+ `port`) or a full
    PYRO: uri."""

    def __init__(self, host: str, port: int = 9091):
        uri = host if host.startswith("PYRO:") else f"PYRO:riscq.board@{host}:{port}"
        self._uri = uri
        self._proxy = Pyro5.api.Proxy(uri)
        self.remote = _RemoteExtras(self._proxy)
        self.board = _BoardExtras(self._proxy)
        self._host_base = None

    def read32(self, addr: int) -> int:
        return self._proxy.read32(int(addr))

    def write32(self, addr: int, value: int) -> None:
        self._proxy.write32(int(addr), int(value) & 0xFFFFFFFF)

    def read_block(self, addr: int, nbytes: int) -> bytes:
        return _to_bytes(self._proxy.read_block(int(addr), int(nbytes)))

    def write_block(self, addr: int, data: bytes) -> None:
        self._proxy.write_block(int(addr), bytes(data))

    def read_host(self, offset: int, nbytes: int) -> bytes:
        """Read the board's CMA result buffer at buffer-relative `offset`, chunked like the bundle
        store. In practice this runs server-side inside `remote_rerun`; the client path exists for
        ad-hoc inspection (specs/software/22 §2.6)."""
        out = bytearray()
        while len(out) < nbytes:
            n = min(CHUNK, nbytes - len(out))
            out += _to_bytes(self._proxy.read_host(int(offset) + len(out), int(n)))
        return bytes(out)

    @property
    def host_base(self) -> int:
        """Physical base of the server's `pynq.allocate` result buffer."""
        if self._host_base is None:
            self._host_base = int(self._proxy.get_host_base())
        return self._host_base

    def ddr_stream(self, m, progs, expected: dict, wr_base: int, params=None, arrays=None,
                   serializer: str = "marshal", read_bytes: int = 4 << 20, read_timeout: float = 1.0,
                   **opts) -> "DdrStreamClient":
        """Run the programs `riscq.run.setup` loaded once, with the live readout (qubic3 S1), and return the
        iterator over its provisional chunks (`riscq.ddr.StreamChunk`, in DDR order). The run is valid only once
        the iterator ends with `certificate` set; an uncertified run raises `riscq.ddr.DdrUplinkError`.
        `opts` go to the board's StreamWorker (buffer_bytes, max_chunk, poll_s, consumer_timeout_s, ...)."""
        from riscq import run as rq
        rq._check_results_path(m, progs)
        sid = self._proxy.ddr_stream_start(rq._params_json(m), [int(c) for c in progs], params or {},
                                           arrays or {}, {int(c): int(n) for c, n in expected.items()},
                                           int(wr_base), opts)
        return DdrStreamClient(self._uri, sid, serializer=serializer, read_bytes=read_bytes,
                               read_timeout=read_timeout)

    def close(self) -> None:
        self._proxy._pyroRelease()


class DdrStreamClient:
    """The host end of a streamed run (qubic3 S1): size-prefixed frames pulled over this client's OWN Pyro5
    connection, so the pulls never queue behind other calls. marshal by default: bytes cross as bytes, where
    serpent would base64 them (three times slower on loopback, more on the board's A53). Every frame passes
    `riscq.board.ddr_stream.FrameReader`'s checks (contiguous DATA; at END the per-core count of what arrived
    equals the certificate). The board waits for this client at most its `consumer_timeout_s` with at most its
    `buffer_bytes` of data queued, so a slow host cannot stall the PS drain beyond that bound: the run then ends
    with a recoverable ConsumerStalled error, and stays in PL DDR for `drain()`."""

    def __init__(self, uri: str, sid: int, serializer: str = "marshal", read_bytes: int = 4 << 20,
                 read_timeout: float = 1.0):
        self._proxy = Pyro5.api.Proxy(uri)
        self._proxy._pyroSerializer = serializer
        self._proxy._pyroTimeout = read_timeout + 60.0     # a dead board raises instead of hanging the host
        self.sid, self.read_bytes, self.read_timeout = int(sid), int(read_bytes), float(read_timeout)
        self.reader = FrameReader()

    certificate = property(lambda self: self.reader.certificate)
    warnings = property(lambda self: self.reader.warnings)
    end = property(lambda self: self.reader.end)

    def __iter__(self):
        while not self.reader.done:
            blob = _to_bytes(self._proxy.ddr_stream_read(self.sid, self.read_bytes, self.read_timeout))
            yield from self.reader.feed(blob)

    def abort(self) -> None:
        """Ask the board to stop the run (call between iterations: a Pyro5 proxy is not shared across threads)."""
        self._proxy.ddr_stream_abort(self.sid)

    def close(self) -> None:
        self._proxy._pyroRelease()


def upload_bundle(drv: RemoteDriver, name: str, xsa: str | Path, params_json: str | Path,
                  board: dict | None = None) -> None:
    """Chunk a bundle's files up to the server's store (spec 10 §4): top.xsa + params.json
    (+ board.json when `board` is given). Activate it with drv.board.load(name)."""
    files = [("top.xsa", Path(xsa).read_bytes()),
             ("params.json", Path(params_json).read_bytes())]
    if board is not None:
        files.append(("board.json", json.dumps(board, indent=2).encode()))
    for filename, data in files:
        drv._proxy.store_begin(name, filename, len(data), hashlib.sha256(data).hexdigest())
        for off in range(0, len(data), CHUNK):
            drv._proxy.store_chunk(data[off:off + CHUNK])
        drv._proxy.store_end()
