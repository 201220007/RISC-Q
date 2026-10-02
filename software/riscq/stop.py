"""Early STOP and exact actual_shots on the shared run layer (qubic3 P4, plan P4 v2 §4.4, §5).

A stoppable kernel (`riscq.lang.StopConvention`) publishes `rq_status` = [posted shots, reads, fin]
and checks the stop words `rq_stop_epoch` / `rq_stop_at` at the top of every shot. `riscq.run.rerun`
writes `rq_epoch = e`, `rq_stop_epoch = 0`, `rq_stop_at = 0` and the sentinel 0xFFFF_FFFF over
`rq_status` on every opting core before every release, so the RAM state at the release does not
depend on history; after DONE it requires fin == e on every opting core (valid counts, §4.4),
drains exactly `reads` uplink words per core, and classifies the run (§5.6). This module holds what
that uses:

- `Issuer`, the issue step: the poll loop runs it for the run's one valid request, so it is the only
  writer of the stop words. NEXT publishes rq_stop_at = 0 on every core (each stops at its next
  boundary, best effort). AT(S) reads the reference core's posted count v_pre, refuses an explicit
  S below v_pre + L + 2 before any write (TOO_LATE) or takes S = v_pre + L + 2 + m for S = None,
  publishes S on every core, then reads the reference again (v_post): VERIFIED iff
  S >= v_post + L + 2 (§5.3). Publishing a core is `write32(rq_stop_at)`, `write32(rq_stop_epoch
  = e)` and one `read32(rq_stop_epoch)` that must return e (§5.1): single-word accesses only.
- `counts_of` (validity, §4.4) and `classify` (the outcome table, §5.6), and `finish`, which the run
  layer calls after DONE.
- The policies `AtProgress` and `Landed` (Ant-Q's rule, on the uplink's CUR_ADDR), and `spec(...)`,
  a `StopSpec` with a serpent-safe wire form: a remote driver sends it, and its server builds the
  same spec next to the hardware (`from_wire`).
- `last(drv)`: the stop record of the driver's last run of stoppable programs.

Why VERIFIED is a proof (§5.3), under the kernel's stated C1-C3: once a core's read-back returned,
its CPU sees the request. The v_post read may meet the kernel's store of rq_status[0] and return the
old value (the URAM template in NO_CHANGE mode), so the reference's posted count is at most
v_post + 1 and every core's at most v_post + 1 + L. A core whose posted count is p has finished the
checks of shots < p and may be inside that of shot p, so no core has begun the check of a shot
>= v_post + L + 2, and S is one: every core's check of S sees the request, `s >= S` first holds at
s = S, and every core stops exactly there (or runs its n shots, when S >= n). The sentinel reads as
0: such a core has not zeroed its .bss, so it has begun no check."""

from __future__ import annotations

import math
import time

from riscq.session import (ACCEPTED, AT, CONSISTENT_LATE, FIRED, INCONSISTENT, INTERNAL_ERROR, NATURAL,
                           NEXT, REFUSED_NEXT_ONLY, SENTINEL, STOPPED_EACH, TOO_LATE, Counts,
                           StopInternalError, StopPublishError, StopRecord, StopSpec, session)

STATUS = "rq_status"
NOT_BOOTED, UNFINISHED, TIMEOUT = "NOT_BOOTED", "UNFINISHED", "TIMEOUT"
_MASK = 0xFFFF_FFFF


def _s32(v: int) -> int:
    v = int(v) & _MASK
    return v - (1 << 32) if v >> 31 else v


# ── the stop words and the progress read (§5.1, §5.3) ──

def word_addr(m, core: int, prog, name: str, index: int = 0) -> int:
    """The host address of a named int32 global (or element `index` of an array) of `core`."""
    return m.to_host_addr(core, prog.var_addr(name) + 4 * index)


def posted(drv, m, core: int, prog) -> int:
    """`core`'s posted-shot count, rq_status[0]: one read32. The sentinel reads as 0 (the core has
    not yet zeroed its .bss, so it has begun no check)."""
    v = int(drv.read32(word_addr(m, core, prog, STATUS, 0))) & _MASK
    return 0 if v == SENTINEL else v


def publish(drv, m, core: int, prog, epoch: int, stop_at: int) -> None:
    """Publish a request on one core (§5.1): rq_stop_at first, then rq_stop_epoch = epoch, then one
    read of rq_stop_epoch, ordered behind the write on the core's RAM port: once it returns e the
    core's CPU sees the request, and since the kernel reads rq_stop_epoch before rq_stop_at, a check
    that sees e also sees this rq_stop_at."""
    drv.write32(word_addr(m, core, prog, "rq_stop_at"), int(stop_at) & _MASK)
    drv.write32(word_addr(m, core, prog, "rq_stop_epoch"), int(epoch) & _MASK)
    got = int(drv.read32(word_addr(m, core, prog, "rq_stop_epoch"))) & _MASK
    if got != int(epoch) & _MASK:
        raise StopPublishError(f"core {core}: rq_stop_epoch read back {got:#010x} after the epoch "
                               f"{int(epoch) & _MASK:#010x} was written")


def at_minimum(v: int, lead: int) -> int:
    """The smallest S a progress read v makes safe: v + L + 2 (§5.3)."""
    return int(v) + int(lead) + 2


def verified(S: int, v_post: int, lead: int) -> bool:
    """§5.3 step 3: an AT(S) published before the progress read v_post is VERIFIED iff
    S >= v_post + L + 2."""
    return int(S) >= at_minimum(v_post, lead)


def margin_for(budget_s: float, period_batches: int, dsp_freq_hz: float) -> int:
    """m = ceil(B_budget / P) (§5.3): the shots that pass during a broadcast of `budget_s` seconds
    at a shot period of `period_batches` dsp cycles."""
    return max(0, math.ceil(float(budget_s) * float(dsp_freq_hz) / int(period_batches)))


def overshoot_bound(t_pipe: float, t_backlog: float, t_poll: float, t_bcast: float, period_s: float,
                    words_per_shot: float, lead: int = 1, margin: int = 0, bank_words: int = 64) -> float:
    """The latency budget of the `landed` policy (§5.5), in shots of period P: (T_pipe + T_backlog
    + T_poll + T_bcast)/P + B/W + L + 2 + m, with T_pipe the uplink FIFO, poller, cbuf and bank,
    T_backlog the DDR write backlog, T_poll one poll iteration, T_bcast 2N writes, N read-backs and 2
    progress reads, and B/W the bank granularity (B words per bank, W words per shot over all
    cores). The board stage measures every term (B2)."""
    return ((t_pipe + t_backlog + t_poll + t_bcast) / float(period_s) + bank_words / float(words_per_shot)
            + int(lead) + 2 + int(margin))


class Issuer:
    """The issue step of a stoppable run (§5.1, §5.3): the poll loop calls it for the run's one
    valid request and decides the ticket by what it returns. `margin` is m of S = v_pre + L + 2 + m
    for an AT(None); `reference` the core whose progress is read (default: the run's lowest core).
    The ticket's `info` records the kind, S, v_pre, v_post, L, m, VERIFIED and the timestamps."""

    needs_convention = True          # rerun refuses it unless every programmed core is stoppable

    def __init__(self, margin: int = 0, reference: int | None = None):
        if isinstance(margin, bool) or not isinstance(margin, int) or margin < 0:
            raise ValueError(f"margin m must be an int >= 0, got {margin!r}")
        self.margin = margin
        self.reference = None if reference is None else int(reference)

    def __call__(self, ctx, ticket) -> str:
        req, info = ticket.request, ticket.info
        drv, m, progs = ctx.drv, ctx.m, ctx.progs
        e = int(ctx.run_id[1])
        cores = sorted(progs)
        info.update(kind=req.kind, t_decide=time.monotonic())
        if req.kind == NEXT:
            for c in cores:
                publish(drv, m, c, progs[c], e, 0)
            info["t_written"] = time.monotonic()
            return ACCEPTED
        if not all(progs[c].stop.get("at") for c in cores):
            info["why"] = (f"cores {[c for c in cores if not progs[c].stop.get('at')]} state no C1-C3 "
                           f"(StopConvention(at=False)): NEXT only")
            return REFUSED_NEXT_ONLY
        lead = max(int(progs[c].stop["lead"]) for c in cores)
        ref = cores[0] if self.reference is None else self.reference
        if ref not in progs:
            raise ValueError(f"reference core {ref} is not in the run's cores {cores}")
        v_pre = posted(drv, m, ref, progs[ref])
        s_min = at_minimum(v_pre, lead)
        S = s_min + self.margin if req.S is None else int(req.S)
        info.update(S=S, v_pre=v_pre, s_min=s_min, lead=lead, ref=ref, explicit=req.S is not None,
                    margin=None if req.S is not None else self.margin)
        if S < s_min:
            info["verified"] = False
            return TOO_LATE
        if S > 0x7FFF_FFFF:
            raise ValueError(f"S = {S} does not fit the kernel's int32 compare")
        for c in cores:
            publish(drv, m, c, progs[c], e, S)
        info["t_written"] = time.monotonic()
        v_post = posted(drv, m, ref, progs[ref])
        info.update(v_post=v_post, verified=verified(S, v_post, lead), t_verified=time.monotonic())
        return ACCEPTED


# ── counts and the outcome table (§4.4, §5.6) ──

def counts_of(words, epoch: int, done: bool) -> Counts:
    """A core's `Counts` from its rq_status words: valid only if it raised DONE and fin == epoch;
    else NOT_BOOTED (the sentinel is still in all three words), TIMEOUT (no DONE) or UNFINISHED."""
    shots, reads, fin = (int(x) & _MASK for x in list(words)[:3])
    if done and fin == int(epoch) & _MASK:
        return Counts(shots, reads, fin, True)
    if shots == reads == fin == SENTINEL:
        why = NOT_BOOTED
    else:
        why = TIMEOUT if not done else UNFINISHED
    return Counts(shots, reads, fin, False, why)


def classify(counts: dict, n: dict, request: dict | None, reads_per_shot: dict | None = None):
    """The outcome of a run of stoppable programs (§5.6) from valid counts: (outcome, late_by, why).
    `n` is each core's shot count, `request` the applied request's info (None: none applied: no
    request, or every one LATE, REFUSED_DUPLICATE, TOO_LATE or REFUSED_NEXT_ONLY), `reads_per_shot`
    the r of each fixed-read core. INTERNAL_ERROR comes with `why`."""
    shots = {c: int(k.shots) for c, k in counts.items()}
    n = {c: max(0, int(n[c])) for c in shots}
    over = {c: (s, n[c]) for c, s in shots.items() if s > n[c]}
    if over:
        return INTERNAL_ERROR, None, f"cores past their n (shots, n): {over}"
    bad_r = {c: (k.reads, k.shots, r) for c, k in counts.items()
             if (r := (reads_per_shot or {}).get(c)) is not None and k.reads != k.shots * r}
    if bad_r:
        return INTERNAL_ERROR, None, f"reads != shots x r on fixed-read cores (reads, shots, r): {bad_r}"
    natural = all(shots[c] == n[c] for c in shots)
    if request is None:
        if natural:
            return NATURAL, None, ""
        return INTERNAL_ERROR, None, (f"cores stopped early with no request applied (shots, n): "
                                      f"{ {c: (shots[c], n[c]) for c in shots if shots[c] != n[c]} }")
    if request["kind"] == NEXT:
        return (NATURAL if natural else STOPPED_EACH), None, ""
    if request.get("S") is None:
        return INTERNAL_ERROR, None, ("the applied AT request recorded no S (a stop hook of its own must put "
                                      "kind, S and verified into ticket.info)")
    S, ver = int(request["S"]), bool(request.get("verified"))
    low = {c: (s, min(S, n[c])) for c, s in shots.items() if s < min(S, n[c])}
    if low:
        return INTERNAL_ERROR, None, f"AT({S}): cores below min(S, n) (shots, min): {low}"
    if natural:
        if ver and any(S < n[c] for c in shots):
            return INTERNAL_ERROR, None, f"AT({S}) was VERIFIED below n, but every core ran its n shots"
        return NATURAL, None, ""
    if all(shots[c] == min(S, n[c]) for c in shots):
        return FIRED, None, ""
    if ver:
        return INTERNAL_ERROR, None, f"AT({S}) was VERIFIED, but the counts are {shots} (n {n})"
    vals = set(shots.values())
    if len(vals) == 1:
        s1 = vals.pop()
        if S < s1 and all(s1 < n[c] for c in shots):
            return CONSISTENT_LATE, s1 - S, ""
    return INCONSISTENT, None, ""


def finish(run, opting: dict, out: dict, n: dict, uplink=None) -> StopRecord:
    """After DONE, with every opting core's marker (fin == epoch) checked: the counts, the applied
    request and the outcome, as the run's StopRecord. Raises StopInternalError for INTERNAL_ERROR
    (the run then fails), including a core that read more results than the uplink's nominal
    (`prepare`'s footprint)."""
    e = int(run.run_id[1])
    counts = {c: counts_of(out[c][STATUS], e, True) for c in sorted(opting)}
    tickets = list(run.tickets)
    applied = None if run.mailbox is None or run.mailbox.accepted is None else run.mailbox.accepted
    request = None
    if applied is not None:
        request = {k: v for k, v in applied.info.items() if not k.startswith("_")}
        request.setdefault("kind", applied.request.kind)
    rps = {c: opting[c].stop.get("reads") for c in opting}
    outcome, late_by, why = classify(counts, n, request, rps)
    if outcome != INTERNAL_ERROR and uplink is not None:
        nominal = uplink.expected if uplink.nominal is None else uplink.nominal
        over = {c: (k.reads, int(nominal.get(c, 0))) for c, k in counts.items()
                if k.reads > int(nominal.get(c, 0))}
        if over:
            outcome, late_by = INTERNAL_ERROR, None
            why = f"cores read more results than the uplink's nominal, prepare's footprint (reads, nominal): {over}"
    rec = StopRecord(tuple(run.run_id), outcome, counts, {c: int(n[c]) for c in counts}, request,
                     [(t.request.kind, t.request.S, t.outcome) for t in tickets], late_by,
                     min((k.shots for k in counts.values()), default=0),
                     all(bool(opting[c].stop.get("at")) for c in opting))
    run.stop = rec
    if outcome == INTERNAL_ERROR:
        raise StopInternalError(f"INTERNAL_ERROR in run {tuple(run.run_id)}: {why}")
    return rec


def shots_of(drv, m, core: int, prog, params: dict) -> int:
    """The shot count n of a stoppable program for this rerun: the bound value, the caller's param,
    or (a runtime param left as last written) its value in the core's RAM."""
    st = prog.stop
    if st.get("n") is not None:
        return int(st["n"])
    name = st["shots"]
    if name in params:
        return _s32(params[name])
    from riscq.run import read_var
    return _s32(read_var(drv, m, core, prog, name))


# ── policies (run once per poll iteration; they may return a request) ──

class AtProgress:
    """Request `(kind, S)` once the reference core's posted count reaches `k` (default reference:
    the run's lowest core); once per run. A test and tooling policy: the request lands at a known
    point of the run. AT with S = None takes the issue step's v_pre + L + 2 + m."""

    def __init__(self, k: int, kind: str = AT, S: int | None = None, reference: int | None = None):
        self.k, self.kind, self.S, self.reference = int(k), kind, S, reference
        self._fired = None

    def __call__(self, ctx):
        if self._fired == ctx.run_id:
            return None
        ref = min(ctx.progs) if self.reference is None else self.reference
        v = posted(ctx.drv, ctx.m, ref, ctx.progs[ref])
        if v < self.k:
            return None
        self._fired = ctx.run_id
        return self.kind, self.S, {"policy": "at_progress", "seen": v}

    def to_wire(self) -> dict:
        return {"policy": "at_progress", "k": self.k, "kind": self.kind, "S": self.S,
                "reference": self.reference}


class Landed:
    """Ant-Q's rule (plan P4 v1 §1, §3.3; v2 §5.5): AT at the earliest verifiable S once the uplink
    has landed `stop_after` shots, i.e. (CUR_ADDR - the run's base) / 8 >= stop_after x W, with W
    = `words_per_shot` over all cores; once per run. CUR_ADDR advances per committed bank, so the
    decision sees landed shots at bank granularity (64/W shots on this uplink). It reads only
    CUR_ADDR and needs `rerun(uplink=...)`."""

    def __init__(self, stop_after: int, words_per_shot: float):
        if float(words_per_shot) <= 0:
            raise ValueError("words_per_shot must be > 0 (a heralded kernel's mean is fine)")
        self.stop_after, self.words_per_shot = int(stop_after), float(words_per_shot)
        self._fired = None

    def __call__(self, ctx):
        if self._fired == ctx.run_id:
            return None
        up = ctx.uplink
        if up is None:
            raise ValueError("the landed policy reads the uplink's CUR_ADDR: run it with rerun(uplink=...)")
        from riscq import ddr_regs
        from riscq.ddr import readout_for
        rd = up.readout if up.readout is not None else readout_for(ctx.drv, ctx.m)
        words = (int(rd._rd(ddr_regs.CUR_ADDR)) - int(up.base)) // ddr_regs.WORD_BYTES
        if words < self.stop_after * self.words_per_shot:
            return None
        self._fired = ctx.run_id
        return AT, None, {"policy": "landed", "stop_after": self.stop_after,
                          "landed_words": words, "landed_shots": int(words // self.words_per_shot)}

    def to_wire(self) -> dict:
        return {"policy": "landed", "stop_after": self.stop_after, "words_per_shot": self.words_per_shot}


def _policy_from_wire(w):
    if w is None:
        return None
    w = dict(w)
    kind = w.get("policy")
    if kind == "at_progress":
        return AtProgress(int(w["k"]), str(w["kind"]), None if w.get("S") is None else int(w["S"]),
                          None if w.get("reference") is None else int(w["reference"]))
    if kind == "landed":
        return Landed(int(w["stop_after"]), float(w["words_per_shot"]))
    raise ValueError(f"unknown stop policy {kind!r} (at_progress, landed)")


def spec(policy=None, margin: int = 0, reference: int | None = None, poll_interval: float = 0.0,
         poll_cycles: int = 2_000, accept_inconsistent: bool = False) -> StopSpec:
    """The StopSpec of a stoppable run with P4's issue step (`Issuer(margin, reference)`). With
    `policy` None or one of this module's policies it also carries its wire form, so it works through
    a remote driver too (the server builds the same spec next to its hardware).
    `accept_inconsistent` returns an INCONSISTENT run's certified data and record instead of raising
    `StopInconsistent`; it cuts nothing (see `StopRecord.prefix`)."""
    to_wire = getattr(policy, "to_wire", None)
    wire = None
    if policy is None or to_wire is not None:
        wire = {"margin": int(margin), "reference": reference, "poll_interval": float(poll_interval),
                "poll_cycles": int(poll_cycles), "accept_inconsistent": bool(accept_inconsistent),
                "policy": None if policy is None else to_wire()}
    return StopSpec(issue=Issuer(margin, reference), policy=policy, poll_interval=float(poll_interval),
                    poll_cycles=int(poll_cycles), accept_inconsistent=bool(accept_inconsistent), wire=wire)


def from_wire(w: dict) -> StopSpec:
    """The server side of `spec(...)`'s wire form."""
    w = dict(w)
    return spec(_policy_from_wire(w.get("policy")), int(w.get("margin", 0)),
                None if w.get("reference") is None else int(w["reference"]),
                float(w.get("poll_interval", 0.0)), int(w.get("poll_cycles", 2_000)),
                bool(w.get("accept_inconsistent", False)))


def last(drv) -> StopRecord | None:
    """The StopRecord of `drv`'s last run of stoppable programs (through a remote driver, the one
    its server sent back with the results)."""
    return session(drv).last_stop
