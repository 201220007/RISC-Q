#!/usr/bin/env python3
"""qubic3 BT T4 (PLAN_BT_v2 §1 T4, P6 v2 §8): the board session's replayed Experiments.

  python examples/bt_record.py cal14  --config software/configs/zcu216-14q-antq.json --out bt_cal14.yaml
  python examples/bt_record.py record --config ... --cal bt_cal14.yaml --out calls/   (my_office, S0)
  python examples/bt_record.py audit  --config ... calls/calls_*.pkl progs_bt.pkl     (S0; the child repeats it)
  python examples/bt_record.py decode --config ... --cal bt_cal14.yaml calls/ replies/ (my_office, after D)

1. Record. Each Experiment runs against a `RecordingDriver`: its `.remote` stores every `setup` and
   `rerun` call with all its arguments (a sha256 per call over its canonical form, `call_hash`) and
   answers with shape-correct synthetic replies. It runs twice, with zero replies and with seeded-noise
   replies, and the two call lists must be equal: the sequence does not depend on the data. The record
   also keeps each setup's C sources, for the audit. -> calls/calls_<name>.pkl and calls/index.json (the
   pinned per-call hashes).
2. Replay, in the BT kit's MMIO child: an in-process `BoardServer(driver=drv, params_text=...)` replays the
   calls in order through `remote_setup` / `remote_rerun`, the production server path (remote_reply, the
   StopInconsistent return), after every call's hash is checked against the pinned index. Each reply goes
   to replies/<name>_<i>.pkl, listed with its sha256 in replies/MANIFEST.sha256.
3. Decode, on my_office. Each Experiment re-runs against a `ReplayDriver`: every call must hash equal to
   the recorded one and gets its real reply; a divergence fails. No physics is judged.
4. The RF audit (`audit_programs`), of every recorded setup and of every program of progs_bt.pkl: every
   table slot of a channel with a DAC plays at amplitude 0, and the C source writes no amplitude
   (`set_amp`) and no DC offset (`set_dc_offset`) other than 0 to a channel with a DAC, so no rerun param
   can write one. A table whose channel cannot be named fails the audit."""

from __future__ import annotations

import argparse
import hashlib
import json
import pickle
import re
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software"))

import numpy as np  # noqa: E402

from riscq import run as rq  # noqa: E402
from riscq.map import SocMap, SocParams  # noqa: E402

QUBITS = tuple(range(14))
RO_SHOTS = 100_000          # p6_b1: ReadoutCalibration, RAW, 1 point x 100 000 shots
LEAK_SHOTS, LEAK_GATES = 32, 101
LEAK_VALUES = ([0.0, 0.0], [0.05, 0.05])


class ReplayRefused(RuntimeError):
    """A recorded call differs from the pinned one, or the audit failed: nothing is replayed."""


def _plain(x):
    """A call argument as plain data: numpy arrays and tuples become lists, dict keys stay."""
    if isinstance(x, np.ndarray):
        return [int(v) for v in x.ravel()]
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, dict):
        return {k: _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    return x


def call_hash(op: str, args: dict) -> str:
    """sha256 over the canonical form of one call (`riscq.run._canon`: bytes as their sha256, keys as
    strings, sorted)."""
    text = json.dumps({"op": op, "args": rq._canon(_plain(args))}, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode()).hexdigest()


# ── the 14-qubit RF-silent cal config ─────────────────────────────────────────────────────────────

def cal14(m):
    """`bt_cal14.yaml`: placeholder frequencies and timings for every qubit of the 14q build, then
    `Config.rf_silent()` (every gate and readout drive amplitude 0, every DC offset 0)."""
    from riscq.cal.config import Config
    from riscq.pulses import units
    c = Config()
    for q in range(len(m.params.cores)):
        c[f"qubit/{q}/freq"] = 50e6
        c[f"qubit/{q}/x90/amp"] = 0.5
        c[f"qubit/{q}/x90/vz"] = [0.0, 0.0]
        c[f"qubit/{q}/T1"] = 2.4e-7
        c[f"readout/{q}/freq"] = float(units.demod_code_to_freq(2048, m.params))
        c[f"readout/{q}/amp"] = 0.5
        c[f"readout/{q}/dur"] = 1.12e-7
        c[f"readout/{q}/demod/dur"] = 8e-8
    c["reset/relax"] = 6.4e-6
    return c.rf_silent()


def classifier3():
    """A placeholder 3-level classifier for Leakage's decode (no physics is judged)."""
    from riscq.cal import ClassifierN
    rng = np.random.default_rng(3)
    means = np.array([[1e5, 0.0], [-5e4, 8.66e4], [-5e4, -8.66e4]])
    return ClassifierN([means[k] + 1e3 * rng.standard_normal((30, 2)) for k in range(3)])


def experiments(cfg, qubits=QUBITS, ro_shots=RO_SHOTS):
    """The replayed Experiments (phase 22, p6_b1): name -> a factory of the cal object."""
    from riscq.cal.cals.readout import ReadoutCalibration
    from riscq.cal.cals.single import Leakage
    clf = classifier3()
    return {"readout_cal": lambda: ReadoutCalibration(cfg, list(qubits), shots=ro_shots),
            "leakage": lambda: Leakage(cfg, list(qubits), {q: clf for q in qubits}, "qubit/{q}/x90/vz",
                                       [list(v) for v in LEAK_VALUES], n_gates=LEAK_GATES, shots=LEAK_SHOTS)}


# ── record ────────────────────────────────────────────────────────────────────────────────────────

class RecordingRemote:
    """`.remote` of the RecordingDriver: keeps every call, answers with shape-correct replies, zeros or
    seeded noise (`seed`)."""

    def __init__(self, params_json: str, seed=None):
        self.params_json = params_json
        self.rng = None if seed is None else np.random.default_rng(seed)
        self.calls = []
        self.loaded = {}
        self.c_sources = []

    def setup(self, params_json, progmap):
        args = {"params_json": params_json, "progmap": {int(c): w for c, w in progmap.items()}}
        self.calls.append({"op": "setup", "args": args, "sha256": call_hash("setup", args)})
        self.loaded = {int(c): rq._prog_from_wire(w) for c, w in progmap.items()}

    def rerun(self, cores, params, arrays, results, timeout, identities=None, uplink=None, stop=None):
        args = {"cores": [int(c) for c in cores], "params": {int(c): dict(v) for c, v in params.items()},
                "arrays": {int(c): dict(v) for c, v in arrays.items()},
                "results": None if results is None else list(results), "timeout": int(timeout),
                "identities": None if identities is None else {int(c): str(i) for c, i in identities.items()},
                "uplink": uplink, "stop": stop}
        self.calls.append({"op": "rerun", "args": args, "sha256": call_hash("rerun", args)})
        out = {}
        for c in args["cores"]:
            prog = self.loaded[c]
            names = list(prog.arrays) if results is None else list(results)
            out[c] = {n: self._words(prog.arrays[n]) for n in names}
            n_up = int(((uplink or {}).get("expected") or {}).get(c, 0))
            if n_up:
                out[c]["__uplink"] = (self._words(2 * n_up) >> 4 << 4) + 8
            out[c] = {n: np.asarray(v, dtype="<i4").tobytes() for n, v in out[c].items()}
        return out

    def _words(self, n):
        if self.rng is None:
            return np.zeros(n, dtype=np.int32)
        return self.rng.integers(-(1 << 20), 1 << 20, size=n).astype(np.int32)

    def recover(self):
        raise ReplayRefused("a recorded Experiment called recover(): not replayable")

    def post_stop(self, *a):
        raise ReplayRefused("a recorded Experiment posted a stop: not replayable")

    def current_run(self):
        return None


class RecordingDriver:
    """A driver whose `.remote` records (the run layer takes its remote path); `.board` answers the
    build's SocParams. It has no MMIO: an Experiment that touched the hardware directly would fail."""

    def __init__(self, m, seed=None):
        pj = rq._params_json(m)
        self.remote = RecordingRemote(pj, seed)
        self.board = types.SimpleNamespace(get_params=lambda: pj)

    def _refuse(self, *a, **k):
        raise ReplayRefused("record: an Experiment made a direct driver access")
    read32 = write32 = read_block = write_block = read_host = _refuse


def _run_recording(factory, drv):
    """Run the cal against `drv`, keeping each setup's C sources (the audit reads them)."""
    orig = rq.setup

    def setup(d, m, progs):
        if d is drv:
            drv.remote.c_sources.append({int(c): p.c_source for c, p in progs.items()})
        return orig(d, m, progs)
    rq.setup = setup
    try:
        factory().run(drv)
        return None
    except Exception as e:                                 # noqa: BLE001 -- recorded; the call lists decide
        return f"{type(e).__name__}: {e}"
    finally:
        rq.setup = orig


def record(name, factory, m) -> dict:
    """Record one Experiment twice (zero replies, seeded noise); the call lists must be equal."""
    runs = []
    for seed in (None, 1234):
        drv = RecordingDriver(m, seed)
        err = _run_recording(factory, drv)
        runs.append((drv.remote, err))
    (zero, zerr), (noise, nerr) = runs
    hz, hn = [c["sha256"] for c in zero.calls], [c["sha256"] for c in noise.calls]
    if nerr is not None:
        raise RuntimeError(f"{name}: the seeded-noise record raised: {nerr}")
    if hz != hn:
        raise RuntimeError(f"{name}: the call sequence depends on the data ({len(hz)} calls with zero "
                           f"replies, {len(hn)} with noise; first difference at "
                           f"{next((i for i, (a, b) in enumerate(zip(hz, hn)) if a != b), min(len(hz), len(hn)))})")
    if not any(c["op"] == "rerun" for c in noise.calls):
        raise RuntimeError(f"{name}: no rerun recorded")
    setups = [c for c in noise.calls if c["op"] == "setup"]
    for call, src in zip(setups, noise.c_sources):
        call["c_source"] = src
    return {"name": name, "calls": noise.calls, "zero_reply_error": zerr,
            "params_json_sha256": hashlib.sha256(noise.params_json.encode()).hexdigest()}


# ── the RF audit ──────────────────────────────────────────────────────────────────────────────────

_SET = re.compile(r"\b(set_amp|set_dc_offset)\s*\(\s*(RF_CH\d+)\s*,([^;]*)\)\s*;")


def audit_programs(progs: dict, m, c_sources: dict | None = None) -> list:
    """Every violation of RF silence in one setup's programs ([] = silent)."""
    bad = []
    for core, prog in sorted(progs.items()):
        names = {ch.name: ch for ch in m.channels(core)}
        cnames = {ch.cname: ch for ch in names.values()}
        for sym, slots in prog.tables.items():
            ch = names.get(sym[4:] if sym.startswith("tbl_") else sym)
            if ch is None:
                bad.append(f"core {core} table {sym}: no channel of that name, so not provably silent")
            elif ch.dac is not None and any(int(slot[1]) != 0 for slot in slots):
                bad.append(f"core {core} table {sym} (DAC {ch.dac}): amplitude codes "
                           f"{[int(s[1]) for s in slots]}")
        src = (c_sources or {}).get(core)
        if c_sources is not None and src is None:
            bad.append(f"core {core}: no C source recorded")
        for fn, cname, rest in _SET.findall(src or ""):
            ch = cnames.get(cname)
            value = rest.split(",")[-1].strip() if fn == "set_amp" else rest.strip()
            if ch is not None and ch.dac is not None and value not in ("0", "0x0", "(0)"):
                bad.append(f"core {core}: {fn}({cname}, ...{value}) writes DAC {ch.dac} at run time")
    return bad


def audit_record(rec: dict, m) -> list:
    bad = []
    for i, call in enumerate(rec["calls"]):
        if call_hash(call["op"], call["args"]) != call["sha256"]:
            bad.append(f"{rec['name']} call {i}: its hash does not match its arguments")
        if call["op"] == "setup":
            progs = {int(c): rq._prog_from_wire(w) for c, w in call["args"]["progmap"].items()}
            bad += [f"{rec['name']} setup {i}: {b}" for b in audit_programs(progs, m, call.get("c_source"))]
        elif call["args"].get("stop") is not None:
            bad.append(f"{rec['name']} rerun {i}: a stop spec (not part of the replay)")
    return bad


def audit_progs_bt(blob: dict, m) -> list:
    bad = []
    for name, wires in blob["progs"].items():
        progs = {int(c): rq._prog_from_wire(w) for c, w in wires.items()}
        bad += [f"progs_bt {name}: {b}" for b in audit_programs(progs, m, (blob.get("c_sources") or {}).get(name))]
    return bad


# ── replay (the BT kit's MMIO child) and decode (my_office) ──────────────────────────────────────

def check_pinned(rec: dict, index: dict) -> None:
    """Before the first call: every call's hash equals its stored one and the pinned index."""
    pinned = index.get(rec["name"])
    got = [call_hash(c["op"], c["args"]) for c in rec["calls"]]
    if pinned is None or got != pinned or got != [c["sha256"] for c in rec["calls"]]:
        raise ReplayRefused(f"{rec['name']}: the recorded calls differ from the pinned hashes")


def replay(server, rec: dict, out_dir: Path, log=print) -> list:
    """Replay one record through the in-process BoardServer; write each reply. Returns
    [(i, op, seconds, reply file or None)]."""
    import time
    out_dir.mkdir(parents=True, exist_ok=True)
    done = []
    for i, call in enumerate(rec["calls"]):
        a, t = call["args"], time.monotonic()
        if call["op"] == "setup":
            server.remote_setup(a["params_json"], a["progmap"])
            done.append((i, "setup", round(time.monotonic() - t, 3), None))
            continue
        kw = {} if a.get("stop") is None else {"stop": a["stop"]}
        reply = server.remote_rerun(a["cores"], a["params"], a["arrays"], a["results"], a["timeout"],
                                    a["identities"], a["uplink"], **kw)
        f = out_dir / f"{rec['name']}_{i:03d}.pkl"
        f.write_bytes(pickle.dumps(reply, protocol=4))
        done.append((i, "rerun", round(time.monotonic() - t, 3), f.name))
        log(f"  replay {rec['name']} call {i}: rerun of cores {a['cores']} in {done[-1][2]} s -> {f.name}")
    return done


class ReplayRemote:
    """`.remote` of the ReplayDriver: each call must hash equal to the recorded one; a rerun gets its
    recorded real reply."""

    def __init__(self, rec, replies_dir: Path):
        self.rec, self.dir, self.i = rec, Path(replies_dir), 0

    def _next(self, op, args):
        if self.i >= len(self.rec["calls"]):
            raise ReplayRefused(f"{self.rec['name']}: more calls than recorded ({self.i + 1})")
        call = self.rec["calls"][self.i]
        if call["op"] != op or call_hash(op, args) != call["sha256"]:
            raise ReplayRefused(f"{self.rec['name']} call {self.i}: diverges from the record")
        self.i += 1
        return self.i - 1

    def setup(self, params_json, progmap):
        self._next("setup", {"params_json": params_json, "progmap": {int(c): w for c, w in progmap.items()}})

    def rerun(self, cores, params, arrays, results, timeout, identities=None, uplink=None, stop=None):
        args = {"cores": [int(c) for c in cores], "params": {int(c): dict(v) for c, v in params.items()},
                "arrays": {int(c): dict(v) for c, v in arrays.items()},
                "results": None if results is None else list(results), "timeout": int(timeout),
                "identities": None if identities is None else {int(c): str(i) for c, i in identities.items()},
                "uplink": uplink, "stop": stop}
        i = self._next("rerun", args)
        return pickle.loads((self.dir / f"{self.rec['name']}_{i:03d}.pkl").read_bytes())


class ReplayDriver(RecordingDriver):
    def __init__(self, m, rec, replies_dir):
        super().__init__(m)
        self.remote = ReplayRemote(rec, replies_dir)


def decode(name, factory, m, rec, replies_dir):
    """Re-run one Experiment on its recorded replies; every call must match the record."""
    drv = ReplayDriver(m, rec, replies_dir)
    result = factory().run(drv)
    if drv.remote.i != len(rec["calls"]):
        raise ReplayRefused(f"{name}: {drv.remote.i} of {len(rec['calls'])} recorded calls were made")
    return result


def write_manifest(out_dir: Path) -> Path:
    lines = [f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.name}"
             for p in sorted(out_dir.glob("*.pkl"))]
    (out_dir / "MANIFEST.sha256").write_text("\n".join(lines) + "\n")
    return out_dir / "MANIFEST.sha256"


# ── the command line ──────────────────────────────────────────────────────────────────────────────

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("cal14", "record", "audit", "decode"))
    ap.add_argument("--config", required=True)
    ap.add_argument("--cal", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("paths", nargs="*")
    a = ap.parse_args(argv)
    m = SocMap(SocParams.load(a.config))
    if a.cmd == "cal14":
        cal14(m).save(a.out)
        print(f"[BT record] {a.out}")
        return 0
    if a.cmd == "record":
        from riscq.cal.config import Config
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        index = {}
        for name, factory in experiments(Config.load(a.cal), list(range(len(m.params.cores)))).items():
            rec = record(name, factory, m)
            bad = audit_record(rec, m)
            if bad:
                print(f"[BT record] AUDIT FAIL {name}: " + "; ".join(bad[:5]))
                return 1
            (out / f"calls_{name}.pkl").write_bytes(pickle.dumps(rec, protocol=4))
            index[name] = [c["sha256"] for c in rec["calls"]]
            n = sum(c["op"] == "rerun" for c in rec["calls"])
            print(f"[BT record] {name}: {len(rec['calls'])} calls ({n} reruns), equal with zero and noise replies"
                  f"{'' if rec['zero_reply_error'] is None else ' (zero replies: ' + rec['zero_reply_error'][:80] + ')'}")
        (out / "index.json").write_text(json.dumps(index, indent=1))
        print("[BT record] RECORD PASS")
        return 0
    if a.cmd == "audit":
        bad = []
        for p in a.paths:
            blob = pickle.loads(Path(p).read_bytes())
            bad += audit_progs_bt(blob, m) if "progs" in blob else audit_record(blob, m)
        print("[BT record] AUDIT " + ("PASS" if not bad else "FAIL: " + "; ".join(bad[:20])))
        return 0 if not bad else 1
    if a.cmd == "decode":
        from riscq.cal.config import Config
        calls_dir, replies_dir = Path(a.paths[0]), Path(a.paths[1])
        exps = experiments(Config.load(a.cal), list(range(len(m.params.cores))))
        for name, factory in exps.items():
            rec = pickle.loads((calls_dir / f"calls_{name}.pkl").read_bytes())
            res = decode(name, factory, m, rec, replies_dir)
            print(f"[BT record] decode {name}: {len(rec['calls'])} calls identical; ok={getattr(res, 'ok', None)}")
        print("[BT record] DECODE PASS")
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main())
