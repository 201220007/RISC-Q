# ddr gate runners

The P3b/P3c gate bill of the antq_uplink results path, as repo scripts (P3c-2). Each checks its prerequisites first
(exit 2 if one is missing), and exits non-zero on any failure; none of them hides a child's failure.

| script | gate | pass condition |
|---|---|---|
| `run-cocotb.sh [tag]` | G1' cocotb (writer, cbuf_poller, mmu2, smoke) | every seed rc 0 with `FAIL=0`; `sim_build` cached, rebuilt when the RTL content changes |
| `run-spinal-sims.sh <tag> <list> [par]` | G2, CDC (plain and with stalls), PulseCross, G3' | every sim exits 0 |
| `run-pytest.sh <tag> <args>` | software tests | 0 failed, >0 passed, collection errors exactly the four known upstream modules inside the selection |
| `run-g4.sh [config]` | G4a' (BFM) then G4b' (Unisim) | stops at the first failing step; `run-sim-bd.tcl`'s transcript verdict |
| `run-n2.sh <out-dir> [label]` | N2 (P3c-3): `PipeTraceSim` with the timing pipeline off and on | both simulations exit 0, print their own success line and log no failure line (an HDL `ERROR`/`FAILURE` message, a SpinalSim `[Error]`, an exception, a failed mill task); each trace ends in its one END marker; the two traces are byte-identical. Earlier outputs in `<out-dir>` are removed first |

Env: `GATE_LOG_DIR` (default `build/gate-logs`), `GATE_NICE` (10), `GATE_TASKSET`, `GATE_VIVADO_GUARD` (a command that
must succeed before every Vivado step), `RISCQ_VIVADO_BIN`, `SEEDS`, `COCOTB_CLEAN`, `GATE_G4_PREFIX`,
`GATE_G4_SELFTEST` / `GATE_G4_SELFTEST_FAIL` (a dry run of the G4 stop rule). The cocotb runner needs `cocotb-config`,
`verilator` and the `GenUplinkUnits` RTL; the Spinal runner `mill-1.1.0` and `java`; the N2 runner `mill-1.1.0`; the
pytest runner a Python with pytest.
