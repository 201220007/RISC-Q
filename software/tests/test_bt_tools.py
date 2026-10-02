"""qubic3 BT T4 and T6, host-pure (PLAN_BT_v2 §1 T4, T6): the BT kernels (`examples/bt_kernels.py`) and
the record / replay / decode / audit tooling (`examples/bt_record.py`).

- k_bt and k_bt_poll compile on the 14q antq map with the stop convention; their C3 bound comes from the
  disassembly (3 conditional branches between the result read and the next post: B_C3 = 32 cycles) and
  the posting margin the constants give at 1 us is several times it; k_nomark has no marker.
- The heralded k_batched's sequence header gains one now() read at the end of `seq_shot`, nothing else.
- Record (twice, zero and noise replies, equal call lists), the audit, the replay through an in-process
  BoardServer over FakeSoc, the replies' manifest, and the decode equal to the direct run; a mutated call,
  a diverging decode and a non-silent Experiment are refused; a run-time amplitude write is flagged."""

from __future__ import annotations

import pickle
import sys
from pathlib import Path

import numpy as np
import pytest

from riscq import run as rq
from riscq.map import SocMap, SocParams

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples"))
import bt_kernels as K  # noqa: E402
import bt_record as BR  # noqa: E402

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
M14 = SocMap(SocParams.load(CONFIGS / "zcu216-14q-antq.json"))
M2 = SocMap(SocParams.load(CONFIGS / "sim-2q-antq.json"))


# ── T6: the kernels ──────────────────────────────────────────────────────────────────────────────

def test_k_bt_compiles_with_the_stop_convention_and_its_c3_bound():
    progs = K.compile_bt(M14, range(14))
    poll = K.compile_bt(M14, [0, 1], poll=1)
    for p in (progs[0], progs[13], poll[0]):
        assert p.stop == {"shots": "n", "n": None, "at": True, "lead": 1, "reads": 1}
        assert p.marker == ("rq_status", 2) and p.arrays["slack"] == K.SLACK_WORDS
        assert {"bt_t", "slack", "pollc", "word"} <= set(p.image.symbols)
        assert sorted(k for k, v in p.params.items() if v is None) == \
            ["hang", "n", "period", "rq_epoch", "rq_stop_at", "rq_stop_epoch"]
    c3 = K.posting_branches(progs[0])
    assert c3["conditional_branches"] == 3 and c3["B_C3"] == 32, c3
    assert [x.split()[1] for x in c3["worst_path"]] == ["bge", "bne", "bge", "sw"]
    assert K.posting_branches(poll[0])["B_C3"] == 32
    assert K.designed_slack(K.BATCHES_PER_US) >= 8 * c3["B_C3"]          # the margin is designed in
    nomark = K.compile_nomark(M14, [0])[0]
    assert nomark.marker is None and nomark.stop is None and "ahead" in nomark.params


def test_the_heralded_header_gains_one_now_read_at_the_end_of_seq_shot():
    from riscq.cal.experiment import Experiment
    from riscq.cal.measure import Measure
    from riscq.cal.sequence import Gate
    from tests.cal_fixtures import _cfg
    cfg = _cfg(M2).rf_silent()

    def herald():
        return Experiment(cfg, [0], {0: [Gate("x90")]}, {0: ()}, (), Measure.counts(herald=True), 8,
                          rf_silent=True)
    plain, _, _, _ = K.compile_experiment(herald(), M2)
    inst, par, timeout, npts = K.compile_experiment(herald(), M2, herald_slack=True)
    assert "bt_slack" in inst[0].image.symbols and "bt_slack" not in plain[0].image.symbols
    assert inst[0].bindings == plain[0].bindings and inst[0].tables == plain[0].tables
    hdr = K.slack_header("static inline void seq_shot(uint32_t t_ro, int32_t a) {\n    play(1, 0, t_ro);\n}\n", 77)
    assert hdr.index("bt_note((int32_t)t_ro - 77 - 96);") > hdr.index("play(1, 0, t_ro);")
    assert hdr.count("bt_note(") == 2                                    # the definition and one call


# ── T4: record, audit, replay, decode ────────────────────────────────────────────────────────────

def _cfg2():
    from tests.cal_fixtures import _cfg2 as cfg2
    return cfg2(M2, freqs=(50e6, 50e6)).rf_silent()


def _raw_exp(cfg, shots=4):
    from riscq.cal.experiment import Experiment
    from riscq.cal.measure import Measure
    from riscq.cal.sequence import Gate
    return Experiment(cfg, [0, 1], {0: [Gate("x90")], 1: [Gate("x90")]}, {0: (), 1: ()}, (), Measure.raw(), shots,
                      rf_silent=True)


def _fake(seed=7, shots=4):
    """FakeSoc (sim-2q-antq) answering every loaded core with `shots` seeded results per run and storing
    every armed marker."""
    from tests.fake_soc import FakeSoc
    f = FakeSoc((CONFIGS / "sim-2q-antq.json").read_text())
    rng = np.random.default_rng(seed)

    def release(fake):
        for c in sorted(fake.loaded):
            for _ in range(shots):
                fake.up.post(c, int(rng.integers(-1 << 20, 1 << 20)) << 4, int(rng.integers(-1 << 20, 1 << 20)) << 4)
        for a, v in list(fake.mem.items()):
            if v == rq.MARKER_ARMED:
                fake.mem[a] = rq.MARKER_DONE
        fake.done = sum(1 << c for c in fake.loaded)
    f.on_release = release
    return f


def test_record_replay_decode_equals_the_direct_run(tmp_path):
    from riscq.board.server import BoardServer
    cfg = _cfg2()
    direct = _raw_exp(cfg).run(_fake())
    rec = BR.record("raw2", lambda: _raw_exp(cfg), M2)
    ops = [c["op"] for c in rec["calls"]]
    assert ops == ["setup", "rerun"] and rec["zero_reply_error"] is None
    assert BR.audit_record(rec, M2) == []
    index = {"raw2": [c["sha256"] for c in rec["calls"]]}
    BR.check_pinned(rec, index)
    fake = _fake()
    server = BoardServer(driver=fake, params_text=(CONFIGS / "sim-2q-antq.json").read_text())
    done = BR.replay(server, rec, tmp_path / "replies", log=lambda *a: None)
    assert [d[1] for d in done] == ops and (tmp_path / "replies" / "raw2_001.pkl").exists()
    man = BR.write_manifest(tmp_path / "replies").read_text().split()
    assert man[1] == "raw2_001.pkl" and len(man[0]) == 64
    decoded = BR.decode("raw2", lambda: _raw_exp(cfg), M2, rec, tmp_path / "replies")
    for q in (0, 1):
        assert np.array_equal(decoded[q].y, direct[q].y), q


def test_a_mutated_call_and_a_diverging_decode_are_refused(tmp_path):
    cfg = _cfg2()
    rec = BR.record("raw2", lambda: _raw_exp(cfg), M2)
    index = {"raw2": [c["sha256"] for c in rec["calls"]]}
    bad = pickle.loads(pickle.dumps(rec))
    bad["calls"][1]["args"]["timeout"] += 1                      # one argument changed after the record
    with pytest.raises(BR.ReplayRefused, match="pinned"):
        BR.check_pinned(bad, index)
    assert any("hash does not match" in b for b in BR.audit_record(bad, M2))
    with pytest.raises(BR.ReplayRefused, match="diverges"):          # another Experiment than recorded
        BR.decode("raw2", lambda: _raw_exp(cfg, shots=8), M2, rec, tmp_path)


def test_the_audit_refuses_a_non_silent_experiment_and_a_run_time_amplitude():
    from tests.cal_fixtures import _cfg2 as cfg2
    loud = cfg2(M2, freqs=(50e6, 50e6))                          # not rf_silent: x90 and readout amps
    from riscq.cal.experiment import Experiment
    from riscq.cal.measure import Measure
    from riscq.cal.sequence import Gate
    rec = BR.record("loud", lambda: Experiment(loud, [0], {0: [Gate("x90")]}, {0: ()}, (), Measure.raw(), 4), M2)
    bad = BR.audit_record(rec, M2)
    assert any("table tbl_gate (DAC 0)" in b for b in bad) and any("tbl_ro" in b for b in bad), bad
    src = "void f(void) {\n    set_amp(RF_CH0, 0, r0);\n    set_dc_offset(RF_CH1, 5);\n    set_amp(RF_CH2, 0, r1);\n}\n"
    prog = K.compile_nomark(M2, [0])[0]
    got = BR.audit_programs({0: prog}, M2, {0: src})
    assert got == ["core 0: set_amp(RF_CH0, ...r0) writes DAC 0 at run time",
                   "core 0: set_dc_offset(RF_CH1, ...5) writes DAC 14 at run time"], got
    unnamed = K.compile_nomark(M2, [0])[0]
    unnamed.tables = {"mystery": [(0, 0, 0, 0)]}
    assert "no channel of that name" in BR.audit_programs({0: unnamed}, M2)[0]


def test_the_bt_kernels_pass_the_audit():
    """Every BT kernel plays the demod only: its table is the demod channel's (no DAC)."""
    progs = K.compile_bt(M2, [0, 1])
    assert BR.audit_programs(progs, M2) == []
    assert BR.audit_programs(K.compile_nomark(M2, [0, 1]), M2) == []
