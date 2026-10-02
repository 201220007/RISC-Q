"""The run session (qubic3 S0, plan P4 v2 §4): what the run layer remembers about one hardware
connection between calls.

A session belongs to one driver object: the client's driver, the co-sim bench's `DriverServer`, or
the board server's `PynqDriver` (a new driver after a bitstream reload is a new session).
`riscq.run` finds it with `session(drv)`. It holds

  - the run state machine and the failure lifecycle (§4.2): the record of every run, a pending
    hardware flush, and POISONED, under which every run and setup is refused until `recover()`;
  - the loaded-set guard (§4.5): {core: setup identity} of the last complete setup;
  - the request seam (§4.3, §5.1): the setup generation, the 32-bit run counter and the stop
    mailbox of the running run;
  - the §4.6 bookkeeping: uplink-free reruns since the last uplink run, and the host state a
    hardware flush must restore (the time offset, the host-window base, the RF bring-up).

qubic3 P4 (plan P4 v2 §4.4, §5) adds the records of a stoppable program's run: the per-core counts
with their validity (`Counts`), the stop record (`StopRecord`: the outcome of §5.6, the applied
request with its S and VERIFIED, the common prefix), the TOO_LATE request outcome and the P4
errors. The run counter also skips 0xFFFF_FFFF, the sentinel a core that never boots keeps.

Nothing in this module touches the hardware. `riscq.run` does every MMIO access, from the one
thread that owns the run."""

from __future__ import annotations

import os
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass, field

# ── run states (§4.2). A run without the uplink skips PREPARING, PREPARED and FLUSHED. ──
IDLE = "IDLE"
PREPARING = "PREPARING"
PREPARED = "PREPARED"            # uplink admission open
RELEASED = "RELEASED"
DONE_SEEN = "DONE_SEEN"
RESET = "RESET"
FLUSHED = "FLUSHED"
CERTIFIED = "CERTIFIED"
FAILED = "FAILED"

# why the next release must be preceded by the hardware flush (§4.6)
FLUSH_FAILED = "FAILED"          # a run failed after it wrote to the hardware
FLUSH_STRAY = "STRAY"            # quiesce() found results outside a run window that no uplink-free rerun explains
FLUSH_UNPROVEN = "UNPROVEN"      # a run on an antq_uplink build where not every programmed core showed its marker

# stop-request kinds and outcomes (§4.3, §5.2, §5.6); the issuing is P4's
NEXT, AT = "NEXT", "AT"
QUEUED = "QUEUED"
ACCEPTED = "ACCEPTED"
REFUSED_DUPLICATE = "REFUSED_DUPLICATE"
LATE = "LATE"
REFUSED_NOT_STOPPABLE = "REFUSED_NOT_STOPPABLE"
# P4: refused by the issue step before any write, leaving the run's one request open
TOO_LATE = "TOO_LATE"                    # an explicit S below the safe minimum v_pre + L + 2 (§5.3)
REFUSED_NEXT_ONLY = "REFUSED_NEXT_ONLY"  # AT on a run whose kernels do not state C1-C3 (§5.3)
ISSUE_FAILED = "ISSUE_FAILED"            # the issue step raised: the run FAILED

# P4: the outcome of a stoppable program's run (§5.6). INTERNAL_ERROR fails the run.
FIRED = "FIRED"                          # AT(S), S < n: every core stopped at S
NATURAL = "NATURAL"                      # every core ran its n shots
CONSISTENT_LATE = "CONSISTENT_LATE"      # AT(S) not VERIFIED, every core stopped at one S' > S
STOPPED_EACH = "STOPPED_EACH"            # NEXT: each core stopped at its next boundary
INCONSISTENT = "INCONSISTENT"            # AT(S) not VERIFIED and the counts differ
INTERNAL_ERROR = "INTERNAL_ERROR"

# P4: reserved run-counter values. rq_stop_epoch = 0 is "no request"; 0xFFFF_FFFF is the sentinel the
# run layer writes over rq_status before every release, which a core that never boots keeps (§4.4)
SENTINEL = 0xFFFF_FFFF
_RESERVED_EPOCHS = (0, SENTINEL)

HISTORY = 32                     # run records kept per session


class RunLayerError(RuntimeError):
    """A run-layer refusal or failure that is not a driver or uplink error of its own."""


class SessionPoisoned(RunLayerError):
    """Quiescence could not be established (§4.2): every run and setup is refused until
    `riscq.run.recover()` succeeds; if recovery fails too, the remaining step is a PL reload."""


class RecoveryRequired(RunLayerError):
    """A run without the uplink seam on an antq_uplink build while a hardware flush is pending and
    the uplink has a run open: its caller runs its own uplink protocol, which the flush (due before
    the release, §4.6) would reset underneath it. Call `riscq.run.recover()` (or `setup()`) before
    the caller's prepare."""


class LoadedSetError(RunLayerError):
    """A rerun whose cores or programs differ from the last complete setup (§4.5)."""


class Unfinished(RunLayerError):
    """A programmed core raised DONE without its completion marker (P6 v2 §4.2): UNFINISHED."""


class PreflightRefused(RunLayerError):
    """The many-shot preflight (P6 v2 §4.5) refused the rerun: after quiescence, and before the
    run's own first write (no params, marker, BASE_RESET or release)."""


class EpochExhausted(RunLayerError):
    """The session's 32-bit run counter came back to its seed (§5.1): open a new session."""


class RecoveryUnavailable(RunLayerError):
    """The driver cannot pulse pl_resetn0, so the hardware flush is impossible here: PL reload."""


class NotBooted(Unfinished):
    """P4 (§4.4): a programmed core raised DONE while its `rq_status` still holds the sentinel the
    run layer wrote before the release: it never ran this run's start.S. NOT_BOOTED."""


class StopPublishError(RunLayerError):
    """P4 (§5.1): the read-back of a core's `rq_stop_epoch` did not return the epoch just written."""


class StopInternalError(RunLayerError):
    """P4 (§5.6): INTERNAL_ERROR. A count below min(S, n) under AT(S), a VERIFIED request that did not
    end FIRED (S < n) or NATURAL (S >= n), a core past its n, a stop without a request, reads that
    are not shots x r on a fixed-read kernel, or more reads than the uplink's nominal: a stated
    kernel condition (C1-C3) or the run layer is wrong. The run is FAILED."""


class StopInconsistent(RunLayerError):
    """P4 (§5.6): INCONSISTENT. An AT(S) that was not VERIFIED ended with cores at different counts.
    Not a lifecycle failure: the run is CERTIFIED, each core's data exact. Raised unless the stop spec
    has `accept_inconsistent=True` (locally, and on a remote client, whose server returns the data and
    the record instead of raising); `record` is the run's StopRecord and `out` its certified per-core
    data, not cut: which shots are joint is `record.prefix` under the qualifications there."""

    def __init__(self, msg: str, record: "StopRecord", out: dict):
        super().__init__(msg)
        self.record = record
        self.out = out


@dataclass
class Counts:
    """One core's published counts (P4 §4.4): `rq_status` = [shots, reads, fin]. Valid only if the
    core raised DONE and fin == the run's epoch; otherwise `why` is NOT_BOOTED (the sentinel is still
    there), UNFINISHED (DONE without fin) or TIMEOUT (no DONE)."""

    shots: int
    reads: int
    fin: int
    valid: bool
    why: str = ""

    def to_wire(self) -> list:
        return [int(self.shots), int(self.reads), int(self.fin), bool(self.valid), str(self.why)]

    @classmethod
    def from_wire(cls, w) -> "Counts":
        return cls(int(w[0]), int(w[1]), int(w[2]), bool(w[3]), str(w[4]))


@dataclass
class StopRecord:
    """The P4 record of a run of stoppable programs (§5.6): its outcome, every core's counts and n,
    the applied request (kind, S, VERIFIED, v_pre, v_post, L, m, the reference core and the issue
    timestamps), every request's (kind, S, outcome), S' - S for CONSISTENT_LATE, and `prefix`, the
    smallest count over the cores (min of `shots`).

    `prefix` is a joint shot count only on a common grid: `common_grid` says every core states C1
    (StopConvention(at=True)), so shot k of every core sits at t0 + kP and shots < prefix were
    played by all of them. On a NEXT-only kernel the cores share no stated grid and `prefix` is just
    the smallest count. Neither case cuts the data: a fixed-read core's first prefix x r reads are
    its part of the joint shots, but a heralded core's mapping from shots to reads is the kernel's,
    and the run layer does not know it."""

    run_id: tuple
    outcome: str
    counts: dict
    n: dict
    request: dict | None = None
    tickets: list = field(default_factory=list)
    late_by: int | None = None
    prefix: int = 0
    common_grid: bool = False

    @property
    def shots(self) -> dict:
        return {c: k.shots for c, k in self.counts.items()}

    @property
    def reads(self) -> dict:
        return {c: k.reads for c, k in self.counts.items()}

    def to_wire(self) -> dict:
        return {"run_id": [int(x) for x in self.run_id], "outcome": self.outcome,
                "counts": {str(c): k.to_wire() for c, k in self.counts.items()},
                "n": {str(c): int(v) for c, v in self.n.items()},
                "request": None if self.request is None else dict(self.request),
                "tickets": [list(t) for t in self.tickets], "late_by": self.late_by,
                "prefix": int(self.prefix), "common_grid": bool(self.common_grid)}

    @classmethod
    def from_wire(cls, w: dict) -> "StopRecord":
        return cls(tuple(int(x) for x in w["run_id"]), str(w["outcome"]),
                   {int(c): Counts.from_wire(k) for c, k in dict(w["counts"]).items()},
                   {int(c): int(v) for c, v in dict(w["n"]).items()},
                   None if w.get("request") is None else dict(w["request"]),
                   [tuple(t) for t in w.get("tickets", [])], w.get("late_by"), int(w.get("prefix", 0)),
                   bool(w.get("common_grid", False)))


@dataclass
class FailureRecord:
    """What the cleanup saw (§4.2): the failure kind, the stage the run had reached, the error, and
    STATUS, DIAG, the DONE word and every `rq_status` it could read (P4: with each core's `Counts`,
    invalid unless the core raised DONE and published fin == epoch)."""

    kind: str
    stage: str
    error: str
    run_id: tuple
    status: int | None = None
    diag: int | None = None
    done: int | None = None
    rq_status: dict = field(default_factory=dict)
    counts: dict = field(default_factory=dict)
    cleanup: list = field(default_factory=list)
    t: float = field(default_factory=time.time)


@dataclass
class RunRecord:
    run_id: tuple                 # (generation, epoch)
    cores: tuple
    uplink: bool
    states: list = field(default_factory=list)
    stage: str = IDLE
    outcome: str | None = None    # CERTIFIED | FAILED
    proven: bool | None = None    # queue-proven: every programmed core showed its marker
    failure: FailureRecord | None = None
    discards: dict = field(default_factory=dict)   # quiesce's EXPECTED_DISCARD per-core counts
    flushed: str | None = None    # the reason of a hardware flush done before this run's release
    tickets: list = field(default_factory=list)
    mailbox: "StopMailbox | None" = None
    wrote: bool = False           # the run wrote to the hardware (its params): a failure then needs the flush
    policy_error: bool = False    # the stop policy raised
    preflight: dict | None = None # the many-shot preflight's numbers (P6 v2 §4.5)
    stop: "StopRecord | None" = None  # P4: the record of a run of stoppable programs
    done_word: int = 0            # P4: the DONE word the poll saw (the core reset clears it afterwards)

    def to(self, state: str) -> None:
        self.stage = state
        self.states.append(state)


@dataclass
class StopRequest:
    run_id: tuple
    kind: str
    S: int | None = None
    t: float = field(default_factory=time.monotonic)


class Ticket:
    """The handle `request_stop` returns. `outcome` is QUEUED until the run's poll loop (or the
    mailbox's closure at DONE) decides it; `wait()` blocks until then. `info` holds what the issue
    step recorded (P4: S, VERIFIED, v_pre, v_post, L, m, timestamps)."""

    def __init__(self, request: StopRequest):
        self.request = request
        self.outcome = QUEUED
        self.info: dict = {}
        self._ev = threading.Event()

    def _decide(self, outcome: str) -> None:
        self.outcome = outcome
        self._ev.set()

    def wait(self, timeout: float | None = None) -> str:
        self._ev.wait(timeout)
        return self.outcome


class StopMailbox:
    """The stop mailbox of one stoppable run (§4.3): any thread may `post`, only the run's poll loop
    `drain`s. The first request naming this run is ACCEPTED and immutable; a later one gets
    REFUSED_DUPLICATE, one naming another run LATE, and everything still queued when the run
    closes the mailbox at DONE gets LATE."""

    def __init__(self, run_id: tuple):
        self.run_id = tuple(run_id)
        self._q: queue.SimpleQueue = queue.SimpleQueue()
        self._lock = threading.Lock()
        self._open = True
        self.accepted: Ticket | None = None
        self.decided: list[Ticket] = []

    def post(self, req: StopRequest) -> Ticket:
        t = Ticket(req)
        with self._lock:
            if self._open:
                self._q.put(t)
                return t
        t._decide(LATE)
        return t

    def drain(self, issue=None) -> Ticket | None:
        """Decide every queued request, in order; returns the newly accepted one, if any. `issue(t)`,
        when given, runs for a valid request before it is decided (P4: it publishes the request) and
        returns its outcome: ACCEPTED (None counts as ACCEPTED), or a refusal that wrote nothing
        (TOO_LATE, REFUSED_NEXT_ONLY), which leaves the run's one request open for a later one. If
        `issue` raises, the request is ISSUE_FAILED and the exception propagates (the run fails)."""
        new = None
        while True:
            try:
                t = self._q.get_nowait()
            except queue.Empty:
                return new
            if tuple(t.request.run_id) != self.run_id:
                t._decide(LATE)
            elif self.accepted is not None:
                t._decide(REFUSED_DUPLICATE)
            else:
                outcome = ACCEPTED
                if issue is not None:
                    try:
                        outcome = issue(t) or ACCEPTED
                    except BaseException:
                        t._decide(ISSUE_FAILED)
                        self.decided.append(t)
                        raise
                if outcome == ACCEPTED:
                    self.accepted = new = t
                t._decide(outcome)
            self.decided.append(t)

    def close(self) -> None:
        with self._lock:
            self._open = False
        while True:
            try:
                t = self._q.get_nowait()
            except queue.Empty:
                return
            t._decide(LATE)
            self.decided.append(t)

    @property
    def is_open(self) -> bool:
        return self._open


@dataclass
class StopSpec:
    """The P4 hook of a stoppable run (§4.3). `issue(ctx, ticket)` performs a valid request (P4
    writes the stop words) and returns its outcome (see `StopMailbox.drain`); `policy(ctx)` runs
    once per poll iteration and may return a request `(kind, S)`. Either raising fails the run.
    `poll_interval` (seconds, 0 = busy poll) paces the loop on hardware, `poll_cycles` in co-sim.
    P4: `accept_inconsistent` returns an INCONSISTENT run's certified data instead of raising
    `StopInconsistent` (nothing is cut; see `StopRecord.prefix`); `wire` is the serpent-safe form
    `riscq.stop.spec` builds, which a remote driver sends to its server (None: this spec runs only
    next to the hardware)."""

    issue: object
    policy: object = None
    poll_interval: float = 0.0
    poll_cycles: int = 2_000
    accept_inconsistent: bool = False
    wire: dict | None = None


@dataclass
class RunContext:
    """What a stop hook sees of the running run (P4: and the run's `UplinkRun`, or None)."""

    drv: object
    m: object
    progs: dict
    run_id: tuple
    session: "RunSession"
    done: int = 0                 # the last DONE word the poll loop read
    uplink: object = None


@dataclass
class PendingFlush:
    reason: str                   # FLUSH_FAILED | FLUSH_STRAY | FLUSH_UNPROVEN
    detail: str = ""


def _seed() -> int:
    while True:
        s = int.from_bytes(os.urandom(4), "little")
        if s not in _RESERVED_EPOCHS:
            return s


class RunSession:
    def __init__(self, seed: int | None = None):
        self.lock = threading.RLock()        # serialises setups and reruns (§4.1)
        self.loaded: dict | None = None      # {core: setup identity}; None: nothing loaded
        self.generation = 0                  # +1 per complete setup (§5.1)
        self._seed = _seed() if seed is None else int(seed) & 0xFFFF_FFFF
        if self._seed in _RESERVED_EPOCHS:
            raise ValueError("the run counter's seed must be neither 0 (no request) nor 0xFFFF_FFFF "
                             "(the never-booted sentinel)")
        self._next = self._seed
        self._exhausted = False
        self.pending_flush: PendingFlush | None = None
        self.poisoned: str | None = None
        self.uplink_free_since = 0           # uplink-free reruns since the last uplink run (§4.6 G3)
        self.time_offset: int | None = None  # the host state a hardware flush clears (ps_rst)
        self.host_window_base: int | None = None
        self.rf_bringup = None               # callable replaying the session's RF bring-up
        self.runs: deque = deque(maxlen=HISTORY)
        self.current: RunRecord | None = None
        self.flushes: list = []              # (reason, t) of every hardware flush
        self.notes: deque = deque(maxlen=HISTORY)   # EXPECTED_DISCARD / STRAY / cleanup lines
        self.last_stop: StopRecord | None = None    # P4: the stop record of the last stoppable run

    # ── epochs and run ids (§5.1) ──
    def next_epoch(self) -> int:
        if self._exhausted:
            raise EpochExhausted("epoch space exhausted, open a session")
        e = self._next
        n = (e + 1) & 0xFFFF_FFFF
        while n in _RESERVED_EPOCHS:         # never 0, never the sentinel (P4 §4.4)
            n = (n + 1) & 0xFFFF_FFFF
        if n == self._seed:
            self._exhausted = True
        self._next = n
        return e

    # ── refusals before any hardware access ──
    def refuse_if_poisoned(self, what: str) -> None:
        if self.poisoned is not None:
            raise SessionPoisoned(f"{what} refused: the session is POISONED ({self.poisoned}); "
                                  f"call riscq.run.recover(drv, m), and if that fails, reload the PL")

    def check_loaded(self, idents: dict) -> None:
        if self.loaded is None:
            raise LoadedSetError("rerun before a complete setup in this session (a failed setup, or a "
                                 "driver or bitstream reload, clears the loaded set): call setup() first")
        if set(idents) != set(self.loaded):
            raise LoadedSetError(
                f"rerun of cores {sorted(idents)}, but setup() loaded {sorted(self.loaded)}: the shared "
                f"core reset boots every loaded core, so a subset would run the omitted ones (and a "
                f"superset would boot parked ones); set up the cores you run")
        bad = sorted(c for c, i in idents.items() if self.loaded[c] != i)
        if bad:
            raise LoadedSetError(f"cores {bad}: the program differs from the one setup() loaded "
                                 f"(image, symbols, params, arrays, tables or envelopes); call setup() "
                                 f"with it first")

    # ── the run record ──
    def begin(self, cores, uplink: bool) -> RunRecord:
        run = RunRecord((self.generation, self.next_epoch()), tuple(sorted(cores)), bool(uplink))
        run.to(IDLE)
        self.current = run
        self.runs.append(run)
        return run

    def end(self, run: RunRecord, outcome: str) -> None:
        run.outcome = outcome
        if outcome != FAILED:
            run.to(IDLE)
        if self.current is run:
            self.current = None

    def request_flush(self, reason: str, detail: str = "") -> None:
        """Keep the strongest pending reason: FAILED and STRAY over UNPROVEN."""
        if self.pending_flush is None or self.pending_flush.reason == FLUSH_UNPROVEN:
            self.pending_flush = PendingFlush(reason, detail)

    # ── the stop mailbox (§4.3): no run lock here, a poster must never wait on a running run ──
    def post_stop(self, run_id, kind: str, S: int | None = None) -> Ticket:
        """P4: AT takes a shot index 0 <= S < 2^31 (the kernel compares int32), or None for the
        earliest S the issue step can verify, v_pre + L + 2 + m (§5.3); NEXT takes none."""
        if kind not in (NEXT, AT):
            raise ValueError(f"stop kind must be {NEXT!r} or {AT!r}, got {kind!r}")
        if kind == AT and S is not None and not 0 <= int(S) <= 0x7FFF_FFFF:
            raise ValueError(f"an AT request takes a shot index 0 <= S < 2^31, or None for the earliest "
                             f"verifiable one; got {S}")
        if kind == NEXT and S is not None:
            raise ValueError("a NEXT request takes no shot index")
        req = StopRequest(tuple(int(x) for x in run_id), kind, None if S is None else int(S))
        run = self.current
        mb = run.mailbox if run is not None else None
        if mb is None:
            t = Ticket(req)
            t._decide(REFUSED_NOT_STOPPABLE if run is not None and run.run_id == req.run_id else LATE)
            return t
        t = mb.post(req)
        run.tickets.append(t)
        return t

    @property
    def last_failure(self) -> FailureRecord | None:
        for run in reversed(self.runs):
            if run.failure is not None:
                return run.failure
        return None


_ATTR = "_rq_session"


def session(drv) -> RunSession:
    """The driver's run session, created on first use. A wrapper that forwards attribute lookups to
    the driver it wraps (CountingDriver, TraceDriver) shares that driver's session once it exists."""
    s = getattr(drv, _ATTR, None)
    if s is None:
        s = RunSession()
        setattr(drv, _ATTR, s)
    return s
