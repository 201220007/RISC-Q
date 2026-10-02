"""The run layer: pure functions over (Driver, SocMap) implementing the boot/run protocol.
Backend-agnostic — the same code drives co-sim and hardware.

qubic3 S0 (plan P4 v2 §4) adds the shared run layer on top: a per-driver `RunSession`
(`riscq.session`) with the run state machine, the failure lifecycle (every exception from the first
hardware write of a rerun on, the uplink's `prepare` included, asserts the core reset, closes the stop
mailbox and the uplink admission, records the failure and re-raises; a FAILED run returns no data),
quiescence and the `pl_resetn0` hardware flush with `recover()`, the loaded-set guard, the
generation + epoch run ids with the stop mailbox (the seam P4's STOP plugs into), and the optional
`uplink=UplinkRun` of `rerun` (P6 v2 §4.3). On a hostwindow build a valid setup or rerun issues
exactly the driver operations it issued before S0 (`tests/test_hostwindow_pins.py`)."""

from __future__ import annotations

import hashlib
import json
import time as _time
from dataclasses import dataclass

import numpy as np
import serpent

from riscq.build import Image, Program
from riscq import ddr_regs as rd_regs
from riscq.map import SocMap, pack16
from riscq.session import (  # noqa: F401  (re-exported: the run layer's public names)
    CERTIFIED, DONE_SEEN, FAILED, FLUSHED, FLUSH_FAILED, FLUSH_STRAY, FLUSH_UNPROVEN, IDLE, PREPARED,
    PREPARING, RELEASED, RESET, FailureRecord, LoadedSetError, PreflightRefused, RecoveryRequired,
    RecoveryUnavailable, RunContext, RunLayerError, SessionPoisoned, StopMailbox, StopSpec, Unfinished,
    session,
)

STATUS_DONE_MASK = 0xFFFF0000
STATUS_DONE = 0xD04E0000
STATUS_RUNNING = 1
MAGIC = 0x52515121
MARKER_ARMED = 0xFFFF_FFFF      # written over a completion marker with the reset held (P6 v2 §4.2)
MARKER_DONE = 1
SIM_SETTLE_BATCHES = 2048       # the co-sim G2 settle, in batches (UplinkRun.sim_settle)
QUIESCE_TIMEOUT = 5.0           # seconds per quiesce step
# after the flush pulse the uplink's DDR half resets once its AXI master is quiet, a hold the RTL bounds
# (2^16 ui cycles, then a forced reset and axi_rst_fault); the wait is well past that, which matters only
# in co-sim, where those cycles take seconds
FLUSH_IDLE_TIMEOUT = 30.0


def reset(drv, m: SocMap, on: bool) -> None:
    """Assert (on=True) / release the shared core reset. Powers up asserted."""
    drv.write32(m.host_ctrl + m.HOST_RESET, 1 if on else 0)


def _write_time_offset(drv, m: SocMap, t: int) -> None:
    drv.write32(m.host_ctrl + m.HOST_TIME_OFF_LO, t & 0xFFFFFFFF)
    drv.write32(m.host_ctrl + m.HOST_TIME_OFF_HI, (t >> 32) & 0xFFFFFFFF)


def set_time_offset(drv, m: SocMap, t: int) -> None:
    """Write the 64-bit host time offset. The session remembers it: a hardware flush resets the host
    domain (ps_rst), and `hardware_flush` writes it back (plan P4 v2 §4.6)."""
    _write_time_offset(drv, m, t)
    session(drv).time_offset = int(t)


def set_host_window(drv, m: SocMap, base: int, enable: bool = True) -> None:
    """Point the host-window funnel at the driver's result buffer (specs/software/22 §2.3): a core's
    store to `HOSTWIN + off` then lands at `base + (core << 24) + off`. Write it while the core reset
    is ASSERTED — the funnel is idle then, so the 40-bit base can never be seen torn. `enable` powers
    up low, so until this runs a window store stalls instead of writing DDR address 0."""
    m.require_host_window("set_host_window")
    drv.write32(m.host_ctrl + m.HOST_HOSTWIN_LO, base & 0xFFFFFFFF)
    drv.write32(m.host_ctrl + m.HOST_HOSTWIN_HI,
                ((base >> 32) & 0xFF) | (0x80000000 if enable else 0))


def load_program(drv, m: SocMap, core: int, image: Image) -> None:
    """Block-write the flat image into the core's RAM window (load address 0x80000000)."""
    drv.write_block(m.to_host_addr(core, image.entry), image.data)


def write_var(drv, m: SocMap, core: int, program: Program, name: str, value: int) -> None:
    """Write a named int32 global — before or during a run (the RAM host port is live)."""
    drv.write32(m.to_host_addr(core, program.var_addr(name)), int(value) & 0xFFFFFFFF)


def read_var(drv, m: SocMap, core: int, program: Program, name: str) -> int:
    """Read a named global; returns the raw unsigned 32-bit value."""
    return drv.read32(m.to_host_addr(core, program.var_addr(name)))


def read_array(drv, m: SocMap, core: int, program: Program, name: str) -> np.ndarray:
    """Fetch a named array global as int32 (element count from the ELF symbol size)."""
    addr, size = program.var_addr(name), program.var_size(name)
    if size < 4 or size % 4:
        raise ValueError(f"symbol {name!r} has size {size}, not an int32 array")
    return np.frombuffer(drv.read_block(m.to_host_addr(core, addr), size), dtype="<i4").copy()


def read_host_array(drv, m: SocMap, core: int, program: Program, name: str) -> np.ndarray:
    """Fetch a host-window array as int32 — the result never entered RAM, so it is read straight out
    of the driver's result buffer at `hostwin_offset(core) + <the array's window offset>`
    (specs/software/22 §2.6). Valid only after DONE (§2.4: the writes are posted, and the ordering
    contract is that the host reads from python after `poll_done`)."""
    m.require_host_window(f"reading host-window array {name!r}")
    off, count = program.host_arrays[name]
    buf = drv.read_host(m.hostwin_offset(core) + off, 4 * count)
    return np.frombuffer(buf, dtype="<i4").copy()


def write_array(drv, m: SocMap, core: int, program: Program, name: str, values) -> None:
    """Fill a named int32 array global — a host-preloaded input Array (`slots`/`times`) lives in
    .data (RQ_PARAM) so this pre-run write survives boot (spec 02 §3.1)."""
    if name in program.host_arrays:
        raise ValueError(f"array {name!r} lives in the write-only host window — it cannot be "
                         f"host-written (drop host=True to make it an input)")
    addr, size = program.var_addr(name), program.var_size(name)
    buf = np.asarray(values, dtype="<i4").tobytes()
    if len(buf) > size:
        raise ValueError(f"{len(buf)} B into array {name!r} of {size} B")
    drv.write_block(m.to_host_addr(core, addr), buf)


# rq_slot = { int32 phase, amp, env, dur } — the pulse-table entry layout (fw/riscq.h)
_SLOT_BYTES = 16
_SLOT_FIELD_OFF = {"phase": 0, "amp": 4, "env": 8, "dur": 12}


def load_tables(drv, m: SocMap, core: int, program: Program) -> None:
    """Fill each live ParamTable's .data `struct rq_slot[]` by symbol+offset with its compiled
    design-time codes — the kernel's init_pulse_params programs the hardware slots from it. .data
    (RQ_PARAM) survives start.S's .bss zeroing, like a param global (spec 02 §3.2). Every field is
    seated in data[31:16] (pack16, spec 12) so init_pulse_params' set_* write it raw."""
    for name, slot_codes in program.tables.items():
        buf = b"".join((pack16(c) & 0xFFFFFFFF).to_bytes(4, "little")
                       for slot in slot_codes for c in slot)
        drv.write_block(m.to_host_addr(core, program.var_addr(name)), buf)


def write_slot(drv, m: SocMap, core: int, program: Program, table: str, slot: int,
               field: str, value: int) -> None:
    """Retune one field of one ParamTable slot in .data by name (`gate[0].amp`) — no recompile,
    the next init_pulse_params picks it up (spec 02 §3.2). `value` is a plain code; it is seated in
    data[31:16] (pack16, spec 12) to match the compiled table fields."""
    addr = program.var_addr(table) + _SLOT_BYTES * slot + _SLOT_FIELD_OFF[field]
    drv.write32(m.to_host_addr(core, addr), pack16(value) & 0xFFFFFFFF)


def write_params(drv, m: SocMap, core: int, program: Program,
                 values: dict[str, int] | None = None) -> None:
    """Write the program's param globals: any compile-time auto value first, then the caller's
    `values` — an explicit value overrides an auto one (host-side retuning without recompiling).
    Kernel params are all user (auto=None) now; the pulse codes live in the .data tables
    (load_tables) instead of auto-param globals."""
    values = values or {}
    for name, auto in program.params.items():
        if auto is not None and name not in values:
            write_var(drv, m, core, program, name, auto)
    for name, value in values.items():
        write_var(drv, m, core, program, name, value)


def load_envelopes(drv, m: SocMap, core: int, program: Program) -> None:
    """Upload the program's envelope-RAM images (compile_kernel's allocator output)."""
    for channel, image in program.envelopes.items():
        for line0, lines in image:
            write_envelope(drv, m, core, channel, line0, lines)


def park_core(drv, m: SocMap, core: int) -> None:
    """Write a `j .` self-loop (0x6f) at the core's reset vector. Reset release boots ALL cores;
    a core whose RAM holds stale/garbage words would run wild (bus decode errors), so every core
    must hold either a program or this park word before the release."""
    drv.write32(m.imem(core), 0x6F)


def poll_done(drv, m: SocMap, cores, timeout: int = 2_000_000) -> int:
    """Wait until every core in `cores` has raised its hardware DONE bit; returns the DONE word.

    Completion is a register, not a memory word (specs/software/23): ONE read of the host control
    block covers the whole SoC, and it never touches a core's RAM — the port that instruction fetch
    shares with the host image-load master. `riscqReset` clears the bits, so `rerun` needs no
    clearing write and a stale DONE from the previous run cannot race this poll.

    `cores` is an iterable of core indices; parked cores never raise their bit, so pass only the
    cores that were given a program. `timeout` is in dsp cycles (batches): the co-sim poll counts
    them, and on hardware it is a wall-clock deadline, `poll_seconds(m, timeout)`. Loud TimeoutError
    naming the cores still missing."""
    cores = list(cores)
    mask = 0
    for core in cores:
        mask |= 1 << core
    addr = m.host_ctrl + m.HOST_DONE
    sim = getattr(drv, "sim", None)
    chunk = 20_000
    spent = 0
    deadline = None if sim is not None else _time.monotonic() + poll_seconds(m, timeout)
    word = drv.read32(addr)
    while word & mask != mask:
        if (spent >= timeout) if sim is not None else (_time.monotonic() >= deadline):
            raise TimeoutError(_not_done(cores, word, timeout, m, sim))
        if sim is not None:
            word = sim.poll_word(addr, not_equal=word, timeout_cycles=min(chunk, timeout - spent))
            spent += chunk
        else:
            _time.sleep(0.001)
            word = drv.read32(addr)
    return word


# qubic3 S0 r1: the hardware poll's wall-clock bounds. Upstream counted the timeout as 1 ms sleeps, so
# the cal layer's cycle-derived timeouts (`riscq.cal.base.batch_timeout`, at least 2·10^7) meant 5.5 h.
POLL_MIN_S = 1.0             # host-side latency and scheduling: the shortest hardware poll
POLL_MAX_S = 600.0           # the longest (10^6 shots x 14 cores at a 2000-batch period: 4 s at 500 MHz)


def poll_seconds(m: SocMap, timeout: int) -> float:
    """The wall-clock bound of a hardware DONE poll whose `timeout` is in dsp cycles (batches): that
    many cycles at the build's `dsp_freq_hz`, at least POLL_MIN_S and at most POLL_MAX_S."""
    return min(POLL_MAX_S, max(POLL_MIN_S, int(timeout) / float(m.params.dsp_freq_hz)))


def _not_done(cores, word: int, timeout: int, m: SocMap, sim) -> str:
    missing = sorted(c for c in cores if not (word >> c) & 1)
    bound = "" if sim is not None else f", a {poll_seconds(m, timeout):.3g} s wall-clock bound"
    return f"cores {missing} not DONE after {timeout} cycles{bound} (DONE word = {word:#010x})"


def check_magic(drv, m: SocMap, core: int, program: Program) -> None:
    magic = read_var(drv, m, core, program, "__rq_magic")
    if magic != MAGIC:
        raise RuntimeError(f"core {core}: __rq_magic = {magic:#010x} != {MAGIC:#010x} "
                           f"— image not loaded / wrong map")


# ── remote batching: serpent-safe Program reduction for the server-side runner (spec 08 §5) ──
# Only JSON-ish primitives + `bytes` cross the Pyro5/serpent wire, and `bytes` arrive on the far
# side as a {'data': <base64>, 'encoding': 'base64'} dict — so a Program (bytes image + np.uint32
# envelope arrays) is reduced to bytes + int-lists here and rebuilt with `_prog_from_wire`.

def _wire_bytes(x) -> bytes:
    """Unwrap a bytes field that may have crossed serpent as a base64 dict."""
    return serpent.tobytes(x) if isinstance(x, dict) else bytes(x)


def _params_json(m: SocMap) -> str:
    return m.params.to_json()


def _prog_to_wire(prog: Program) -> dict:
    """Reduce a Program to serpent-safe primitives (bytes + dicts + int lists). Envelope lines
    (np.uint32 (n, wpl)) become bytes + shape; everything else is ints/strings."""
    img = prog.image
    return {
        "data": img.data,
        "entry": int(img.entry),
        "symbols": {name: [int(addr), int(size)] for name, (addr, size) in img.symbols.items()},
        "params": {name: (None if v is None else int(v)) for name, v in prog.params.items()},
        "arrays": {name: int(n) for name, n in prog.arrays.items()},
        "host_arrays": {name: [int(off), int(n)] for name, (off, n) in prog.host_arrays.items()},
        "tables": {name: [[int(c) for c in slot] for slot in slots]
                   for name, slots in prog.tables.items()},
        "envelopes": {int(chan): [_env_to_wire(line0, lines) for line0, lines in image]
                      for chan, image in prog.envelopes.items()},
        "marker": None if prog.marker is None else [str(prog.marker[0]), int(prog.marker[1])],
    }


def _env_to_wire(line0, lines) -> list:
    arr = np.ascontiguousarray(lines, dtype="<u4")
    return [int(line0), arr.tobytes(), int(arr.shape[0]), int(arr.shape[1])]


def _prog_from_wire(wire: dict) -> Program:
    """Inverse of `_prog_to_wire`: rebuild a real Image + Program, unwrapping any serpent
    base64-dict byte fields (the image `data` and each envelope's packed lines)."""
    symbols = {name: (int(pair[0]), int(pair[1])) for name, pair in wire["symbols"].items()}
    image = Image(data=_wire_bytes(wire["data"]), symbols=symbols, entry=int(wire["entry"]))
    params = {name: (None if v is None else int(v)) for name, v in wire["params"].items()}
    arrays = {name: int(n) for name, n in wire["arrays"].items()}
    host_arrays = {name: (int(p[0]), int(p[1])) for name, p in wire.get("host_arrays", {}).items()}
    tables = {name: [tuple(int(c) for c in slot) for slot in slots]
              for name, slots in wire["tables"].items()}
    envelopes = {int(chan): [_env_from_wire(entry) for entry in entries]
                 for chan, entries in wire["envelopes"].items()}
    prog = Program(image, params=params, arrays=arrays, envelopes=envelopes, tables=tables,
                   host_arrays=host_arrays)
    marker = wire.get("marker")
    prog.marker = None if marker is None else (str(marker[0]), int(marker[1]))
    return prog


def _canon(x):
    """A wire-form value as JSON-able data with every byte string replaced by its sha256."""
    if isinstance(x, (bytes, bytearray, memoryview)):
        return {"sha256": hashlib.sha256(bytes(x)).hexdigest()}
    if isinstance(x, dict):
        return {str(k): _canon(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_canon(v) for v in x]
    return x


def program_identity(prog: Program) -> str:
    """A program's setup identity (plan P4 v2 §4.5): sha256 over its whole record in the canonical
    wire form: image bytes and entry, symbols with addresses and sizes, params, arrays, host arrays,
    tables, envelopes and the completion marker. The image bytes alone do not fix the symbol and
    parameter metadata that `write_var` and `read_array` address by."""
    text = json.dumps(_canon(_prog_to_wire(prog)), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode()).hexdigest()


def _env_from_wire(entry) -> tuple:
    line0, data, n_lines, wpl = entry
    arr = np.frombuffer(_wire_bytes(data), dtype="<u4").reshape(int(n_lines), int(wpl))
    return int(line0), arr


def _check_results_path(m: SocMap, progs: dict[int, Program]) -> None:
    """Refuse HostWindow programs on a build without the HostWindow (results_path antq_uplink)."""
    if m.params.with_host_window:
        return
    bad = {core: sorted(prog.host_arrays) for core, prog in progs.items() if prog.host_arrays}
    if bad:
        m.require_host_window(f"the host-window arrays {bad} (core: names)")


def setup(drv, m: SocMap, progs: dict[int, Program]) -> None:
    """Once per session: hold reset, load each core's image + envelopes + tables, and park the
    unassigned cores. Leaves reset ASSERTED so the first `rerun` writes into a quiescent, loaded
    core — the image then stays put across reruns (spec 03 §2, 08 §4). With a `drv.remote` extras
    object present, the whole load runs server-side in one RPC (spec 08 §5).

    S0 (plan P4 v2 §4.2, §4.5): the session's loaded set is cleared first and recorded only after a
    complete load, so a failed setup leaves none; on an antq_uplink build `quiesce()` runs first
    (with the cores in reset), and a pending FAILED or STRAY flush runs here. A POISONED session
    refuses."""
    # antq_uplink: no HostWindow chain, so a program with host-window arrays is refused before anything
    # is loaded (compile_kernel already refuses host=True against such a map; this catches a Program
    # compiled for another build). Checked before the remote hop too, so it fails client-side.
    _check_results_path(m, progs)
    remote = getattr(drv, "remote", None)
    if remote is not None:
        remote.setup(_params_json(m), {core: _prog_to_wire(prog) for core, prog in progs.items()})
        return
    s = session(drv)
    with s.lock:
        s.refuse_if_poisoned("setup")
        s.loaded = None
        idents = {int(core): program_identity(prog) for core, prog in progs.items()}
        reset(drv, m, on=True)
        rd = _readout(drv, m)
        if rd is not None:
            quiesce(drv, m, rd=rd, certify=False, held=True)
        elif s.pending_flush is not None and s.pending_flush.reason == FLUSH_FAILED:
            _flush_or_poison(drv, m, s)
        # point the funnel at this driver's result buffer while the reset is held (spec 22 §2.3). A
        # host-pure test double carries no buffer: leave the funnel disabled — correct, since without a
        # buffer there is nowhere for a window store to go — but refuse a program that needs one.
        base = getattr(drv, "host_base", None) if m.params.with_host_window else None
        if base is not None:
            set_host_window(drv, m, int(base))
            s.host_window_base = int(base)
        elif any(prog.host_arrays for prog in progs.values()):
            raise RuntimeError(f"{type(drv).__name__} has no `host_base`, but "
                               f"{sorted(n for p in progs.values() for n in p.host_arrays)} live in the "
                               f"host window — the driver must allocate/model a result buffer")
        for core, prog in progs.items():
            load_program(drv, m, core, prog.image)
            load_envelopes(drv, m, core, prog)
            load_tables(drv, m, core, prog)
        for core in range(len(m.params.cores)):   # un-programmed cores boot too: park them
            if core not in progs:
                park_core(drv, m, core)
        s.loaded = idents
        s.generation += 1


@dataclass
class UplinkRun:
    """The uplink hook of one rerun (plan P6 v2 §4.3). `expected` gives the words every programmed
    core must leave in PL DDR (0 for a core that reads nothing, so the drain also proves that it
    produced nothing); `nominal` bounds `prepare`'s footprint check (default `expected`); `base` is
    the run's DDR base, reused per rerun. `readout` is the `riscq.ddr.DdrReadout` (None: the
    session's, `riscq.ddr.readout_for`). G2's settle is `settle_s` on hardware and `sim_settle`
    batches in co-sim."""

    expected: dict
    base: int = 0
    nominal: dict | None = None
    readout: object = None
    prepare_timeout: float = 1.0
    flush_timeout: float = 5.0
    settle_s: float = 200e-6
    sim_settle: int = SIM_SETTLE_BATCHES
    mem_budget: int | None = None        # the preflight's PS-memory budget (None: half of MemAvailable)
    remote_reply: bool = False           # set server-side: the result goes back over Pyro (preflight)

    def to_wire(self) -> dict:
        nominal = None if self.nominal is None else {int(c): int(n) for c, n in self.nominal.items()}
        return {"expected": {int(c): int(n) for c, n in self.expected.items()}, "base": int(self.base),
                "nominal": nominal, "prepare_timeout": float(self.prepare_timeout),
                "flush_timeout": float(self.flush_timeout), "settle_s": float(self.settle_s),
                "sim_settle": int(self.sim_settle),
                "mem_budget": None if self.mem_budget is None else int(self.mem_budget)}

    @classmethod
    def from_wire(cls, w: dict, readout=None) -> "UplinkRun":
        nominal = w.get("nominal")
        return cls(expected={int(c): int(n) for c, n in dict(w["expected"]).items()},
                   base=int(w.get("base", 0)),
                   nominal=None if nominal is None else {int(c): int(n) for c, n in dict(nominal).items()},
                   readout=readout, prepare_timeout=float(w.get("prepare_timeout", 1.0)),
                   flush_timeout=float(w.get("flush_timeout", 5.0)),
                   settle_s=float(w.get("settle_s", 200e-6)),
                   sim_settle=int(w.get("sim_settle", SIM_SETTLE_BATCHES)),
                   mem_budget=None if w.get("mem_budget") is None else int(w["mem_budget"]))


def rerun(drv, m: SocMap, progs: dict[int, Program],
          params: dict[int, dict[str, int]] | None = None,
          arrays: dict[int, dict[str, object]] | None = None,
          results: list[str] | None = None,
          timeout: int = 2_000_000, uplink: UplinkRun | None = None, stop: StopSpec | None = None,
          identities: dict | None = None) -> dict[int, dict[str, np.ndarray]]:
    """Re-run an already-`setup` batch without any reload: check magic -> write params + host input
    arrays -> one reset release for all cores -> poll the hardware DONE word -> re-assert reset ->
    read results. Reuses the loaded image/envelopes/tables, so a whole sweep costs O(1) driver
    ops (spec 08 §4). Reset is held on entry (`setup` or the previous `rerun` left it asserted), and
    it is re-asserted again after the poll and before the results are read; the hardware DONE bits
    clear under that reset, so no stale-DONE clearing write is needed (specs/software/23). With a
    `drv.remote` extras object present, the whole batch (params + arrays in, poll, results out) runs
    server-side in one RPC (spec 08 §5).

    S0 (plan P4 v2 §4): the cores and programs must be the loaded set; each run takes the run id
    (generation, epoch) and goes IDLE -> [PREPARING -> PREPARED] -> RELEASED -> DONE_SEEN -> RESET ->
    [FLUSHED] -> CERTIFIED -> IDLE, or FAILED. From the first hardware write on, any exception asserts
    the core reset, closes the stop mailbox and the uplink admission, records the failure in the
    session and propagates: a FAILED run returns no data, and the next release (or setup) is preceded
    by the hardware flush. A program with a completion `marker` must show it after DONE (UNFINISHED
    otherwise). `uplink=UplinkRun(...)` adds quiesce, prepare, flush, drain (G1) and the G2 settle, and
    returns each reading core's IQ as `out[c]["__uplink"]` = int32 [re0, im0, re1, im1, ...], the
    midpoint estimates of the drained 28-bit fields (`riscq.ddr.reconstruct`, field + 8 LSB).
    `stop=StopSpec(...)` opens the stop mailbox (the P4 seam). `identities` are the setup identities a
    remote client expects; they are checked against the loaded set too."""
    _check_results_path(m, progs)
    remote = getattr(drv, "remote", None)
    if remote is not None:
        if stop is not None:
            raise ValueError("a stop hook runs next to the hardware: pass it to the server-side runner")
        idents = {int(c): program_identity(p) for c, p in progs.items()}
        kw = {"identities": idents}
        if uplink is not None:
            kw["uplink"] = uplink.to_wire()
        raw = remote.rerun(list(progs), params or {}, arrays or {}, results, timeout, **kw)
        return {int(core): {name: np.frombuffer(buf, dtype="<i4").copy() for name, buf in d.items()}
                for core, d in raw.items()}
    params = params or {}
    arrays = arrays or {}
    s = session(drv)
    with s.lock:
        s.refuse_if_poisoned("rerun")
        idents = {int(c): program_identity(p) for c, p in progs.items()}
        s.check_loaded(idents)
        if identities is not None:
            s.check_loaded({int(c): str(i) for c, i in dict(identities).items()})
        pend = s.pending_flush
        if (uplink is None and m.params.with_antq_uplink and pend is not None
                and pend.reason in (FLUSH_FAILED, FLUSH_STRAY)):
            raise RecoveryRequired(
                f"a {pend.reason} flush is pending ({pend.detail}); a rerun without uplink= leaves the "
                f"uplink to its caller, and the flush would reset it underneath them: call "
                f"riscq.run.recover(drv, m) or setup() first")
        run = s.begin(progs, uplink is not None)
        if stop is not None:
            run.mailbox = StopMailbox(run.run_id)
        rd = None
        try:
            if uplink is not None:
                rd = uplink.readout if uplink.readout is not None else _readout(drv, m)
                if rd is None:
                    raise ValueError(f"{m.params.name} has results_path={m.params.results_path!r}: "
                                     f"no uplink to run through")
                run.stage = "QUIESCE"
                n_flush = len(s.flushes)
                quiesce(drv, m, rd=rd, certify=True)
                if len(s.flushes) > n_flush:
                    run.flushed = s.flushes[-1][0]
                run.stage = "PREFLIGHT"
                from riscq.ddr import preflight
                nominal = uplink.expected if uplink.nominal is None else uplink.nominal
                span = {int(c): max(int(n), int(nominal.get(c, 0))) for c, n in uplink.expected.items()}
                run.preflight = preflight(span, uplink.base, rd.bank_bytes, rd.chunk_bytes(),
                                          remote_reply=uplink.remote_reply, budget=uplink.mem_budget)
            elif s.pending_flush is not None and s.pending_flush.reason == FLUSH_FAILED:
                run.stage = "FLUSH_HW"
                _flush_or_poison(drv, m, s)
                run.flushed = FLUSH_FAILED
            run.stage = "PARAMS"
            run.wrote = True
            for core, prog in progs.items():
                check_magic(drv, m, core, prog)                    # guard: image still loaded
                write_params(drv, m, core, prog, params.get(core, {}))
                for name, values in arrays.get(core, {}).items():
                    write_array(drv, m, core, prog, name, values)
                if prog.marker is not None:
                    drv.write32(_marker_addr(m, core, prog), MARKER_ARMED)
            if uplink is not None:
                run.to(PREPARING)
                nominal = uplink.expected if uplink.nominal is None else uplink.nominal
                rd.prepare(uplink.base, expected=nominal, timeout=uplink.prepare_timeout)
                s.uplink_free_since = 0                            # BASE_RESET cleared REJECTED
                run.to(PREPARED)
            reset(drv, m, on=False)
            run.to(RELEASED)
            if stop is None:
                poll_done(drv, m, progs, timeout=timeout)
            else:
                _poll_stoppable(drv, m, progs, timeout, run, stop, s)
            run.to(DONE_SEEN)
            if run.mailbox is not None:
                run.mailbox.close()
            # Reset goes back on BEFORE the results are read, so host reads of a core's RAM never overlap live
            # instruction fetch on the shared RAM port (specs/software/23 §1.2 option B). The RAM keeps its
            # contents through reset, and window beats already accepted drain regardless.
            reset(drv, m, on=True)
            run.to(RESET)
            out = {core: {name: (read_host_array(drv, m, core, prog, name) if name in prog.host_arrays
                                 else read_array(drv, m, core, prog, name))
                          for name in (list(prog.arrays) if results is None else results)}
                   for core, prog in progs.items()}
            run.proven = _check_markers(drv, m, progs, out)
            if uplink is not None:
                run.stage = "FLUSH"
                st = rd.flush(timeout=uplink.flush_timeout)
                run.to(FLUSHED)
                run.stage = "DRAIN"
                got = rd.drain(uplink.base, uplink.expected, status=st)          # G1
                run.stage = "SETTLE"
                _settle(drv, uplink)
                _check_g2(rd)                                                    # G2
                from riscq.ddr import reconstruct
                for core, n in uplink.expected.items():
                    if n:
                        re, im = got[int(core)]
                        iq = np.empty(2 * len(re), dtype=np.int32)
                        iq[0::2], iq[1::2] = reconstruct(re), reconstruct(im)    # the fields + 8
                        out.setdefault(int(core), {})["__uplink"] = iq
            run.to(CERTIFIED)
            if uplink is None and m.params.with_antq_uplink:
                s.uplink_free_since += 1
            if m.params.with_antq_uplink and not run.proven:
                s.request_flush(FLUSH_UNPROVEN, f"run {run.run_id}: not every programmed core has a "
                                                f"completion marker")
            s.end(run, CERTIFIED)
            return out
        except BaseException as exc:
            _fail(drv, m, s, run, exc, rd, progs)
            raise


def run(drv, m: SocMap, progs: dict[int, Program],
        params: dict[int, dict[str, int]] | None = None,
        arrays: dict[int, dict[str, object]] | None = None,
        results: list[str] | None = None,
        timeout: int = 2_000_000) -> dict[int, dict[str, np.ndarray]]:
    """The one-shot convenience: `setup` (load) then a single `rerun` (spec 08 §4)."""
    setup(drv, m, progs)
    return rerun(drv, m, progs, params, arrays, results, timeout)


# ── S0 internals: markers, the stoppable poll loop, the failure cleanup ──

def _marker_addr(m: SocMap, core: int, prog: Program) -> int:
    name, index = prog.marker
    if index < 0 or 4 * (index + 1) > prog.var_size(name):
        raise ValueError(f"marker {name}[{index}] outside the {prog.var_size(name)} B array")
    return m.to_host_addr(core, prog.var_addr(name) + 4 * index)


def _check_markers(drv, m: SocMap, progs: dict[int, Program], out: dict | None = None) -> bool:
    """After DONE, with the reset held: every marker must read MARKER_DONE (P6 v2 §4.2), taken from
    the results already read when they hold the marker's array, else read. Returns whether the run
    is queue-proven, i.e. every programmed core carries a marker."""
    bad = {}
    for core, prog in progs.items():
        if prog.marker is not None:
            name, index = prog.marker
            got = (out or {}).get(core, {}).get(name)
            v = int(got[index]) & 0xFFFF_FFFF if got is not None and name not in prog.host_arrays \
                else drv.read32(_marker_addr(m, core, prog))
            if v != MARKER_DONE:
                bad[core] = v
    if bad:
        raise Unfinished(f"cores {sorted(bad)} raised DONE without their completion marker "
                         f"({', '.join(f'core {c}: {v:#010x}' for c, v in sorted(bad.items()))}; "
                         f"{MARKER_ARMED:#x} = never booted): UNFINISHED")
    return bool(progs) and all(prog.marker is not None for prog in progs.values())


def _poll_stoppable(drv, m: SocMap, progs: dict, timeout: int, run, stop: StopSpec, s) -> int:
    """The poll loop of a stoppable run (plan P4 v2 §4.3): each iteration reads DONE; once DONE is
    seen it closes the mailbox (every request still queued is LATE) and returns, before any policy
    or request is looked at, so a completed run never accepts a stop. Otherwise it runs the policy
    (which may post a request), decides the mailbox and hands an accepted request to the stop hook.
    Paced by `poll_interval` (hardware, against the wall-clock deadline `poll_seconds` gives) or
    `poll_cycles` (co-sim); `timeout` is in cycles, as in `poll_done`."""
    cores = list(progs)
    mask = sum(1 << c for c in cores)
    addr = m.host_ctrl + m.HOST_DONE
    sim = getattr(drv, "sim", None)
    ctx = RunContext(drv, m, dict(progs), run.run_id, s)
    deadline = None if sim is not None else _time.monotonic() + poll_seconds(m, timeout)
    spent = 0
    word = drv.read32(addr)
    while True:
        ctx.done = word
        if word & mask == mask:
            run.mailbox.close()
            return word
        if stop.policy is not None:
            try:
                req = stop.policy(ctx)
            except BaseException:
                run.policy_error = True
                raise
            if req is not None:
                s.post_stop(run.run_id, *req)
        accepted = run.mailbox.drain()
        if accepted is not None:
            stop.issue(ctx, accepted)
        if (spent >= timeout) if sim is not None else (_time.monotonic() >= deadline):
            raise TimeoutError(_not_done(cores, word, timeout, m, sim))
        if sim is not None:
            step = min(int(stop.poll_cycles), timeout - spent)
            word = sim.poll_word(addr, not_equal=word, timeout_cycles=step)
            spent += step
        else:
            if stop.poll_interval:
                _time.sleep(stop.poll_interval)
            word = drv.read32(addr)


def _kind(run, exc) -> str:
    from riscq.ddr import LateActivity
    if isinstance(exc, LateActivity):
        return "LATE_ACTIVITY"
    if isinstance(exc, Unfinished):
        return "UNFINISHED"
    if isinstance(exc, PreflightRefused):
        return "PREFLIGHT"
    if isinstance(exc, SessionPoisoned):
        return "POISONED"
    if run.policy_error:
        return "POLICY"
    if isinstance(exc, TimeoutError) and run.stage == RELEASED:
        return "TIMEOUT"
    return {PREPARING: "PREPARE", "FLUSH": "FLUSH", "DRAIN": "DRAIN", "SETTLE": "LATE_ACTIVITY",
            "FLUSH_HW": "FLUSH_HW", "QUIESCE": "QUIESCE", RELEASED: "POLL"}.get(run.stage, "ERROR")


def _fail(drv, m: SocMap, s, run, exc, rd, progs) -> None:
    """The failure cleanup (plan P4 v2 §4.2). Reads the DONE word, STATUS and DIAG first (the reset
    clears DONE), asserts the core reset, closes the stop mailbox and the uplink admission (however
    far `prepare` got), reads every `rq_status`, and records it all. The run is FAILED; a failing
    cleanup leaves the session POISONED; a run that wrote to the hardware leaves the hardware flush
    pending for the next release."""
    rec = FailureRecord(_kind(run, exc), run.stage, f"{type(exc).__name__}: {exc}", run.run_id)
    poison = []
    try:
        rec.done = int(drv.read32(m.host_ctrl + m.HOST_DONE))
    except Exception as e:                              # noqa: BLE001 - best effort, the cleanup goes on
        rec.cleanup.append(f"DONE unreadable: {e!r}")
    if rd is not None:
        try:
            rec.status = int(rd.status(timeout=0.5))
            rec.diag = int(rd.rd(rd_regs.DIAG, timeout=0.5))
        except Exception as e:                          # noqa: BLE001
            rec.cleanup.append(f"STATUS/DIAG unreadable: {e!r}")
    try:
        reset(drv, m, on=True)
        rec.cleanup.append("core reset asserted")
    except Exception as e:                              # noqa: BLE001
        poison.append(f"the core reset could not be asserted: {e!r}")
    if run.mailbox is not None:
        run.mailbox.close()
    if rd is not None:
        try:
            rec.cleanup.append(f"admission: {rd.close_admission(timeout=1.0)}")
        except Exception as e:                          # noqa: BLE001
            poison.append(f"the uplink admission could not be closed: {e!r}")
    for core, prog in progs.items():
        if "rq_status" in prog.image.symbols:
            try:
                rec.rq_status[core] = [int(x) & 0xFFFF_FFFF
                                       for x in read_array(drv, m, core, prog, "rq_status")]
            except Exception as e:                      # noqa: BLE001
                rec.cleanup.append(f"core {core} rq_status unreadable: {e!r}")
    run.failure = rec
    run.to(FAILED)
    s.end(run, FAILED)
    if run.wrote:
        s.request_flush(FLUSH_FAILED, f"run {run.run_id}: {rec.kind} at {rec.stage}")
    if poison and s.poisoned is None:
        s.poisoned = "; ".join(poison)
    s.notes.append(f"FAILED {run.run_id} {rec.kind}: {rec.error}")


# ── S0: quiescence, the hardware flush and recovery (plan P4 v2 §4.2, §4.6) ──

def _readout(drv, m: SocMap):
    from riscq.ddr import readout_for
    return readout_for(drv, m)


def _pl_reset_hook(drv):
    fn = getattr(drv, "pl_reset", None)
    if fn is None:
        sim = getattr(drv, "sim", None)
        fn = getattr(sim, "pl_reset", None) if sim is not None else None
    return fn


def _settle(drv, up: UplinkRun) -> None:
    sim = getattr(drv, "sim", None)
    if sim is not None:
        sim.advance(int(up.sim_settle))
    else:
        _time.sleep(float(up.settle_s))


def _check_g2(rd) -> None:
    """G2 (§4.6): REJECTED and early_late, already 0 at the drain (G1), are still 0 a settle later."""
    from riscq.ddr import LateActivity
    st = rd._status()
    rej = rd._rejected()
    if any(rej) or st >> rd_regs.S_EARLY_LATE & 1:
        raise LateActivity(f"results arrived after the drain (G2): REJECTED {rej}, "
                           f"{rd_regs.status_str(st)}: LATE_ACTIVITY")


def _wait(cond, timeout: float, what: str) -> None:
    deadline = _time.monotonic() + timeout
    while not cond():
        if _time.monotonic() >= deadline:
            raise RunLayerError(f"{what} within {timeout}s")
        _time.sleep(0.001)


def _quiet(rd, s) -> list:
    """What keeps the uplink from being quiet, with the cores in reset; [] when quiet."""
    st, diag = rd._status(), rd._diag()
    acc, rej = rd._accepted(), rd._rejected()
    bad = []
    if not diag["run_idle"]:
        bad.append("run_idle low")
    if st >> rd_regs.S_AXI_RST_FAULT & 1:
        bad.append("axi_rst_fault (only a PL reload clears it)")
    if st >> rd_regs.S_EARLY_LATE & 1:
        bad.append("early_late set")
    if any(acc):
        bad.append(f"ACCEPTED {acc}")
    if any(rej):
        bad.append(f"REJECTED {rej}")
    return bad


def hardware_flush(drv, m: SocMap, reason: str = "RECOVER", rd=None, settle: UplinkRun | None = None) -> None:
    """The hardware flush (§4.6): with the cores in reset, pulse pl_resetn0, which resets dspCd
    (the timed queues, the channel pipelines, `refTime`, the uplink's DSP half, its DDR half following
    at AXI quiescence) and the host domain; write back the host state that reset cleared (the time
    offset, the host-window base); repeat the session's RF bring-up; then, on an antq_uplink build,
    the quiet check: run_idle, no axi_rst_fault, and ACCEPTED, REJECTED and early_late at 0 now and
    still 0 a settle later. Raises RecoveryUnavailable when the driver cannot pulse pl_resetn0, and
    RunLayerError when the uplink is not quiet afterwards (the remaining step is a PL reload)."""
    s = session(drv)
    pulse = _pl_reset_hook(drv)
    if pulse is None:
        raise RecoveryUnavailable(f"{type(drv).__name__} cannot pulse pl_resetn0, so the hardware flush "
                                  f"({reason}) is impossible: reload the PL")
    reset(drv, m, on=True)
    pulse()
    if s.time_offset is not None:
        _write_time_offset(drv, m, s.time_offset)
    if m.params.with_host_window and s.host_window_base is not None:
        set_host_window(drv, m, s.host_window_base)
    if s.rf_bringup is not None:
        s.rf_bringup()
    if rd is None:
        rd = _readout(drv, m)
    if rd is not None:
        rd._gate(1.0)
        _wait(lambda: rd._diag()["run_idle"], FLUSH_IDLE_TIMEOUT,
              "the uplink did not return to run_idle after the flush")
        bad = _quiet(rd, s)
        if not bad:
            _settle(drv, settle or UplinkRun(expected={}))
            bad = _quiet(rd, s)
        if bad:
            raise RunLayerError(f"the uplink is not quiet after the hardware flush ({', '.join(bad)}): "
                                f"reload the PL")
    s.pending_flush = None
    s.flushes.append((reason, _time.time(), s.current))


def _flush_or_poison(drv, m: SocMap, s) -> None:
    """The pending FAILED flush of a build without the uplink; a flush that fails POISONs."""
    reason = s.pending_flush.reason
    try:
        hardware_flush(drv, m, reason)
    except Exception as e:                              # noqa: BLE001
        s.poisoned = f"the hardware flush ({reason}) failed: {type(e).__name__}: {e}"
        raise SessionPoisoned(f"{s.poisoned}; call riscq.run.recover(drv, m), and if that fails, "
                              f"reload the PL") from e


def quiesce(drv, m: SocMap, rd=None, certify: bool = True, timeout: float | None = None,
            held: bool = False) -> list:
    """Quiescence (§4.2), before every uplink prepare (`certify=True`) and every setup of an
    antq_uplink build: the cores in reset; `run_idle` with no flush or drain in flight; the S2MM
    channel idle (otherwise reset); then G3 (§4.6): REJECTED or early_late is EXPECTED_DISCARD if an
    uplink-free rerun ran since the last uplink run, else STRAY. A FAILED or STRAY flush runs here;
    an UNPROVEN one only when `certify` (before an uplink run). Any failure POISONs the session.
    `held` says the caller has just asserted the core reset. Returns the notes it made."""
    s = session(drv)
    rd = _readout(drv, m) if rd is None else rd
    if rd is None:
        return []
    timeout = QUIESCE_TIMEOUT if timeout is None else timeout
    notes = []
    try:
        if not held:
            reset(drv, m, on=True)
        rd._gate(timeout)
        p = s.pending_flush
        if p is not None and p.reason != FLUSH_UNPROVEN:
            # after a FAILED run the uplink may be wedged (a drain stalled mid-stream): flush first
            hardware_flush(drv, m, p.reason, rd=rd)
            notes.append(f"hardware flush ({p.reason}: {p.detail})")
        # run_idle (DIAG bit 7) with no flush or drain in flight; one DIAG and one STATUS read per try
        # (in co-sim every register access costs an idle tick of the bench)
        deadline = _time.monotonic() + timeout
        while True:
            diag, st = rd._rd(rd_regs.DIAG), rd._status()
            if (diag >> 7 & 1 and not st >> rd_regs.S_FLUSH_BUSY & 1
                    and not st >> rd_regs.S_RD_BUSY & 1):
                break
            if _time.monotonic() >= deadline:
                raise RunLayerError(f"the uplink did not reach run_idle with no flush or drain in flight "
                                    f"within {timeout}s: {rd_regs.status_str(st)}")
            _time.sleep(0.001)
        port = rd.drv
        idle = getattr(port, "dma_idle", None)
        if idle is not None and not idle():
            port.dma_reset()
            notes.append("the S2MM channel was busy: reset")
            st = rd._status()
        if st >> rd_regs.S_AXI_RST_FAULT & 1:
            raise RunLayerError("axi_rst_fault is set: only a PL reload clears it")
        rej = rd._rejected()
        if any(rej) or st >> rd_regs.S_EARLY_LATE & 1:
            counts = {c: ("≥ 65535" if r >= 0xFFFF else r) for c, r in enumerate(rej) if r}
            if s.uplink_free_since > 0:
                notes.append(f"EXPECTED_DISCARD {counts} after {s.uplink_free_since} uplink-free rerun(s)")
                if s.current is not None:
                    s.current.discards = counts
            else:
                notes.append(f"STRAY {counts}, early_late {st >> rd_regs.S_EARLY_LATE & 1}")
                s.request_flush(FLUSH_STRAY, f"REJECTED {counts}")
        p = s.pending_flush
        if p is not None and (certify or p.reason != FLUSH_UNPROVEN):
            hardware_flush(drv, m, p.reason, rd=rd)
            notes.append(f"hardware flush ({p.reason}: {p.detail})")
    except SessionPoisoned:
        raise
    except Exception as e:                              # noqa: BLE001
        s.poisoned = f"quiesce failed: {type(e).__name__}: {e}"
        s.notes.extend(notes)
        raise SessionPoisoned(f"quiescence could not be established ({s.poisoned}); call "
                              f"riscq.run.recover(drv, m), and if that fails, reload the PL") from e
    s.notes.extend(notes)
    return notes


def recover(drv, m: SocMap) -> list:
    """Recovery (§4.2): the hardware flush, the S2MM soft reset and a fresh quiesce. Clears POISONED
    on success; on failure the session stays POISONED and the remaining step is a PL reload. The
    loaded programs survive (the flush resets logic, not the core RAM), so reruns may follow."""
    remote = getattr(drv, "remote", None)
    if remote is not None:
        return remote.recover()
    s = session(drv)
    with s.lock:
        notes = []
        try:
            rd = _readout(drv, m)
            hardware_flush(drv, m, "RECOVER", rd=rd)
            notes.append("hardware flush")
            if rd is not None:
                reset_dma = getattr(rd.drv, "dma_reset", None)
                if reset_dma is not None:
                    reset_dma()
                    notes.append("S2MM reset")
                s.poisoned = None
                notes += quiesce(drv, m, rd=rd, certify=True)
        except Exception as e:                          # noqa: BLE001
            s.poisoned = f"recover failed: {type(e).__name__}: {e}"
            raise SessionPoisoned(f"{s.poisoned}: reload the PL") from e
        s.poisoned = None
        s.pending_flush = None
        s.notes.extend(notes)
        return notes


# ── S0: the stop-request seam (plan P4 v2 §4.3, §5.1); P4 issues the stop words ──

def request_stop(drv, run_id, kind: str, S: int | None = None):
    """Post a stop request for the run `run_id` = (generation, epoch). It only enqueues: the run's
    poll loop decides it (ACCEPTED, REFUSED_DUPLICATE, LATE). Locally returns the `Ticket`; through
    a remote driver, the outcome known at posting (QUEUED, LATE, REFUSED_NOT_STOPPABLE)."""
    remote = getattr(drv, "remote", None)
    if remote is not None:
        return remote.post_stop(list(run_id), kind, S)
    return session(drv).post_stop(run_id, kind, S)


def current_run(drv):
    """The (generation, epoch) of the run in progress, or None."""
    remote = getattr(drv, "remote", None)
    if remote is not None:
        r = remote.current_run()
        return None if r is None else tuple(r)
    run = session(drv).current
    return None if run is None else run.run_id


def _env_window(m: SocMap, core: int, channel: int):
    """(host base, the ChannelInfo) of one core's envelope RAM for `channel` — the channel's index
    in THAT core's channel list, so a heterogeneous build lands on the right grid."""
    return m.env_base(channel, core), m.channel(channel, core)


def write_envelope(drv, m: SocMap, core: int, channel: int, line0: int, lines) -> None:
    """Upload packed envelope lines ((n_lines, words_per_line) uint32, from riscq.pulses) at
    RAM line `line0`, for the core's channel index `channel`. A gate line is 4 words at host
    line*16 + {0,4,8,12} — contiguous, so this is one block write; readout is 1 word per line at
    line*4. The envelope banks are host WRITE-ONLY (BramWriteFiber — the generator reads them
    internally), so there is no read_envelope; the write→DAC path is verified bit-exact in
    tests/test_pulse.test_dac_window_bit_exact_*."""
    lines = np.ascontiguousarray(lines, dtype="<u4")
    base, ch = _env_window(m, core, channel)
    if lines.ndim != 2 or lines.shape[1] != ch.samples_per_line:
        raise ValueError(f"channel {channel} lines must be (n, {ch.samples_per_line}) words, "
                         f"got shape {lines.shape}")
    if line0 < 0 or line0 + lines.shape[0] > ch.env_depth:
        raise ValueError(f"lines [{line0}, {line0 + lines.shape[0]}) outside envelope RAM "
                         f"depth {ch.env_depth}")
    drv.write_block(base + line0 * ch.line_bytes, lines.tobytes())


def read_robs(drv, m: SocMap, nbytes: int | None = None) -> np.ndarray:
    """Diagnostic fetch of the shared readout trace BRAM (int32 lanes)."""
    nbytes = m.rob_bytes if nbytes is None else nbytes
    return np.frombuffer(drv.read_block(m.robs(), nbytes), dtype="<i4").copy()
