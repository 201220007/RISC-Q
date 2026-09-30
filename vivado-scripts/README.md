# vivado-scripts — Vivado flows for `PulseTableSoc` on the ZCU216 (xczu49dr)

Two self-contained Vivado flows for the SpinalHDL [`PulseTableSoc`](../src/riscq/soc/PulseTableSoc.scala)
on the Zynq UltraScale+ RFSoC `xczu49dr-ffvf1760-2-e` (ZCU216; built with the `vivado` on `PATH`,
currently 2026.1). Each lives in its own
subfolder and is driven by a single `build-*.sh` script.

| | [`riscvsoc/`](riscvsoc/) | [`riscvsoc-bd/`](riscvsoc-bd/) |
|---|---|---|
| what | standalone **OOC place&route bench** — the SoC in isolation | **block-design** build — the SoC in a real ZCU216 image |
| driver | `riscvsoc/build-riscvsoc.sh` | `riscvsoc-bd/build-riscvsoc-bd.sh` |
| RTL | `GenPulseTableSocOoc` — `vivado=false`, plain `dspClk`/`clk` ports | `GenPulseTableSocVivado` — `vivado=true`, `hostClk` + `X_INTERFACE`, `ClockInterface.v` |
| context | SoC alone, OOC | `PulseTableSoc` packaged as a user IP + Zynq PS + RF Data Converter + AXI SmartConnect |
| P&R | non-project `synth → place → route` (pure Tcl) | project `synth_1` (BD wrapper) + SoC as an OOC IP child run, then `impl_1` |
| purpose | reproduce the floorplanned timing number (`dspClk` WNS ≈ −0.156 ns) | measure the block-design / real-device penalty vs that number |

Both bake the same recipe (per-core X0 floorplan + datapath confine + `df`/`1h`/retiming levers); the
RTL-level levers are constructor defaults of `PulseTableSoc`, the floorplan is the per-flow `pblocks-*.tcl`.
See [`docs/soc/ARCH.md`](../docs/soc/ARCH.md) §6 and `docs/soc/floorplan_plan.md` §10–§11.

## Build outputs — one folder per design under `<repo>/build/`

Each flow generates its RTL, runs Vivado, and writes every report + `vivado.log` into a single
git-ignored folder under the **repo root** `build/`:

- `riscvsoc/build-riscvsoc.sh`  → `../../build/riscvsoc/`
- `riscvsoc-bd/build-riscvsoc-bd.sh` → `../../build/riscvsoc-bd/`

Nothing is written under `vivado-scripts/` itself, so the scripts tree stays clean. Set
`RISCQ_PROJ_NAME=<name>` to pick a different folder and **evaluate several designs in parallel** without
clobbering each other:

```bash
RISCQ_PROJ_NAME=riscvsoc-a ./riscvsoc/build-riscvsoc.sh &
RISCQ_PROJ_NAME=riscvsoc-b ./riscvsoc/build-riscvsoc.sh &   # → build/riscvsoc-a, build/riscvsoc-b
```

## Quick start

```bash
cd vivado-scripts/riscvsoc      && ./build-riscvsoc.sh        # OOC bench, 14q  → <repo>/build/riscvsoc
cd vivado-scripts/riscvsoc-bd   && ./build-riscvsoc-bd.sh     # block design, 14q → <repo>/build/riscvsoc-bd
```

`RISCQ_QUBITS=3 ./build-*.sh` runs a smaller config for fast iteration. Each subfolder's `README.md`
documents its full env-knob set, recipe, and the reports it writes.

### Results path: HostWindow or the Ant-Q readout uplink (qubic3)

The config's `results_path` is the only switch (`software/configs/*.json`, `docs/soc/SocSpec.md`). The RTL
and the block design read it from the same file:

| `results_path` | RTL | block design (HP0) |
|---|---|---|
| `hostwindow` (default) | upstream's HostWindow chain, `M_AXI_HOST` | upstream's: `M_AXI_HOST` → HP0, 32-bit, hostClk |
| `antq_uplink` | `ReadoutDdrUplink` (`m_axi_ddr`, `s_axi_ddr_ctrl`, `m_axis_rd`, `ddrClk`), no HostWindow | DDR4 MIG (`M_AXI_DDR` direct), `axi_dma` S2MM, `smc_dma` / `smc_ctrl`; `smc_dma` → HP0, 128-bit, MIG ui_clk |

```bash
cd vivado-scripts/riscvsoc-bd
RISCQ_CONFIG=software/configs/zcu216-14q-antq.json ./build-riscvsoc-bd.sh
```

Ready-made `antq_uplink` configs are:
- `zcu216-14q-antq.json`, the production build with 14 qubits;
- `lbl-readout-emu-antq.json`, the veneno BPF-loopback bench;
- `sim-2q-antq.json`, a fast BD/xsim smoke;
- `sim-dio-antq.json`, the co-sim with DIO.

The old `RISCQ_DDR_READOUT` switch and a `ddr_readout` key are errors.

## Per-cone timing tracking — `report-cones.tcl`

Both flows classify every failing endpoint of the routed design into the named logic cones of
[`specs/riscv-fmax.md`](../specs/riscv-fmax.md) §2 (core C1 jumpAt-broadcast / C2 fetch-front-CE /
C3 operand-ALU) and [`specs/new-readout-decoder/soc-fmax.md`](../specs/new-readout-decoder/soc-fmax.md)
§2.1 (RF channel/buf/link, CORDIC, TimedQueue, ADC pipe, decoder, …), writing into the build folder:

- `cones_impl.rpt` — cone × {n, TNS, worst slack, CE/D/SR endpoint-pin split} + unmatched shapes
- `cones_paths.tsv` — the raw worst-path-per-endpoint dump for offline drill-down

The cone table is the **stable unit of timing tracking** (individual failing paths shuffle between
builds; the cone totals move coherently) — gate every timing lever on a cone-level number, not a WNS
glance. Runs standalone on any routed checkpoint too:

```bash
vivado -mode batch -source report-cones.tcl -tclargs <routed.dcp> [outdir]
```
