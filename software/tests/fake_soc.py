"""A host-pure SoC double for the run-layer tests (qubic3 S0, plan P4 v2 §8).

`FakeSoc` holds every word the host writes in a sparse store and answers the few registers the run
layer reads back: the host control block's DONE word (set for every loaded core when the shared
core reset is released, cleared when it is asserted again) and, on an antq_uplink build, the
HOST_DDR_STATUS readiness word. A core counts as loaded once a block write lands on its reset
vector, and as parked after the `j .` park word. `read_host` returns zeros: the host-pure tests
check the run protocol, not results.

It has no `.sim`, so `riscq.run.poll_done` takes its hardware branch (one read, no sleep: DONE is
already set), and it publishes its SocParams JSON on `.board`, the way `RemoteDriver` does, so
`riscq.cal.base.socmap(drv)` works."""

from __future__ import annotations

import hashlib

from riscq.map import SocMap, SocParams


class _Board:
    def __init__(self, params_json: str):
        self._params_json = params_json

    def get_params(self) -> str:
        return self._params_json


class FakeSoc:
    PARK = 0x6F

    def __init__(self, params_json: str, host_base: int | None = 0x7000_0000):
        self.board = _Board(params_json)
        self.m = SocMap(SocParams.from_json(params_json))
        self.mem: dict[int, int] = {}         # word address -> 32-bit value
        self.loaded: set[int] = set()
        self.done = 0
        self.reset_held = True
        self.releases = 0
        # an antq_uplink build has no host window and so no result buffer (riscq.run.setup reads this)
        self.host_base = host_base if self.m.params.with_host_window else None
        self._imem = {self.m.imem(c): c for c in range(len(self.m.params.cores))}
        self._reset = self.m.host_ctrl + self.m.HOST_RESET
        self._done = self.m.host_ctrl + self.m.HOST_DONE
        self._ddr_status = self.m.ddr_status() if self.m.params.with_antq_uplink else None

    # ── the Driver seam ──
    def write32(self, addr: int, value: int) -> None:
        addr, value = int(addr), int(value) & 0xFFFFFFFF
        if addr == self._reset:
            self.reset_held = bool(value & 1)
            if self.reset_held:
                self.done = 0
            else:
                self.releases += 1
                self.done = sum(1 << c for c in self.loaded)
            return
        if addr in self._imem:
            core = self._imem[addr]
            (self.loaded.discard if value == self.PARK else self.loaded.add)(core)
        self.mem[addr] = value

    def read32(self, addr: int) -> int:
        addr = int(addr)
        if addr == self._done:
            return self.done
        if addr == self._ddr_status:
            return (self.m.DDR_STATUS_MAGIC << 16) | 0b11
        return self.mem.get(addr, 0)

    def write_block(self, addr: int, data: bytes) -> None:
        addr, data = int(addr), bytes(data)
        if len(data) % 4:
            data += b"\x00" * (4 - len(data) % 4)
        for i in range(0, len(data), 4):
            self.mem[addr + i] = int.from_bytes(data[i:i + 4], "little")
        if addr in self._imem:
            self.loaded.add(self._imem[addr])

    def read_block(self, addr: int, nbytes: int) -> bytes:
        addr = int(addr)
        return b"".join(self.mem.get(addr + i, 0).to_bytes(4, "little") for i in range(0, int(nbytes), 4))

    def read_host(self, offset: int, nbytes: int) -> bytes:
        return bytes(int(nbytes))


def _h(data) -> str:
    return hashlib.sha256(bytes(data)).hexdigest()[:16]


class TraceDriver:
    """Wraps a Driver and records every seam operation as a tuple `(op, addr, value or length,
    data hash)`: the operation sequence itself, not a count (plan P4 v2 §8; a `CountingDriver`
    tally cannot tell two different sequences of equal length apart). Reads record the value or
    the hash of the bytes they returned, writes what they wrote. Every other attribute (`.sim`,
    `.remote`, `.board`, `host_base`) forwards to the wrapped driver."""

    def __init__(self, drv):
        self._drv = drv
        self.trace: list[tuple] = []

    def read32(self, addr):
        v = self._drv.read32(addr)
        self.trace.append(("read32", int(addr), int(v)))
        return v

    def write32(self, addr, value):
        self.trace.append(("write32", int(addr), int(value) & 0xFFFFFFFF))
        return self._drv.write32(addr, value)

    def read_block(self, addr, nbytes):
        data = self._drv.read_block(addr, nbytes)
        self.trace.append(("read_block", int(addr), int(nbytes), _h(data)))
        return data

    def write_block(self, addr, data):
        self.trace.append(("write_block", int(addr), len(data), _h(data)))
        return self._drv.write_block(addr, data)

    def read_host(self, offset, nbytes):
        data = self._drv.read_host(offset, nbytes)
        self.trace.append(("read_host", int(offset), int(nbytes), _h(data)))
        return data

    def __getattr__(self, name):
        return getattr(self._drv, name)


def program_digest(prog) -> str:
    """sha256 over everything `riscq.run` loads or addresses by name: the image bytes and entry, the
    symbol table (addresses and sizes), params, arrays, host arrays, the table slot codes and the
    envelope images. Two programs with equal digests load identically and are read identically."""
    h = hashlib.sha256()
    img = prog.image
    h.update(b"data"); h.update(bytes(img.data))
    h.update(b"entry%d" % int(img.entry))
    for name in sorted(img.symbols):
        addr, size = img.symbols[name]
        h.update(f"sym {name} {int(addr)} {int(size)}\n".encode())
    for name in sorted(prog.params):
        h.update(f"param {name} {prog.params[name]}\n".encode())
    for name in sorted(prog.arrays):
        h.update(f"array {name} {int(prog.arrays[name])}\n".encode())
    for name in sorted(prog.host_arrays):
        off, n = prog.host_arrays[name]
        h.update(f"host {name} {int(off)} {int(n)}\n".encode())
    for name in sorted(prog.tables):
        h.update(f"table {name} {[[int(c) for c in slot] for slot in prog.tables[name]]}\n".encode())
    for chan in sorted(prog.envelopes):
        for line0, lines in prog.envelopes[chan]:
            import numpy as np
            arr = np.ascontiguousarray(lines, dtype="<u4")
            h.update(f"env {int(chan)} {int(line0)} {arr.shape}\n".encode())
            h.update(arr.tobytes())
    return h.hexdigest()
