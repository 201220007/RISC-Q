# G4b — full-PHY (Unisim) DDR4 simulation with the Micron model

**Status: harness in progress.** G4b is not claimed until a `[G4] PASS` transcript exists from a build
with `Simulation_Mode = Unisim` and the Micron DDR4 model attached (Codex r13-#12, r14-#10).

## Why it is mandatory even though G4a passes

G4a runs the DDR4 IP in `Simulation_Mode = BFM`, which replaces the XiPhy primitives and the DRAM device
with a behavioural AXI-side model. That leaves five things unverified anywhere in the flow:

1. the generated **pinout and IO timing** for `MT40A1G8WE-075E`,
2. the **clamshell `CS_WIDTH = 2`** wiring (the property that was missing from our MIG dict until r11-#12),
3. ~~PHY reset and calibration~~ — **NOT covered by G4b either.** The IP bypasses calibration under
   `` `ifdef SIMULATION `` in both modes (`BYPASS_CAL="TRUE"`, `CAL_DQS_GATE="SKIP"`, `BISC_EN=0`), which
   is why `c0_init_calib_complete` asserts at the same simulated time in BFM and Unisim runs. The currently supported G4
   simulations therefore do not cover calibration; the hardware branch of that same `` `ifdef `` exists,
   so driving the digital training FSM is not *impossible*, but it is not a supported flow and would stay
   non-authoritative for analog calibration. G6 is the first authoritative test. What G4b does add is the
   real XiPhy datapath,
4. the **DDR4 device protocol** itself (activate/read/write/precharge, refresh, timing),
5. the **real read latency and its jitter**, which is what `circular_buffer3`'s ping-pong exists to absorb.

**What G4b does NOT cover** (Codex r25-#4): its Micron model terminates the **PL MIG** interface only.
HP0 still terminates in the same Zynq VIP, whose slave port answers writes without updating the store
`read_mem()` reads. So **PS-memory persistence is not provable in simulation at all** — not in G4a and
not in G4b. It is a hardware claim, settled in G6.

## Pieces

| file | role |
|---|---|
| `gen-ddr4-model.tcl` | runs `open_example_project` on `riscq_bd_ddr4_0_0`, which materialises the memory model **customised for this part**, and copies it to `<build>/ddr4_model_sim/` |
| `ddr4_mem_c0.sv` | the DUT↔DRAM hookup: `tran` primitives on DQ/DQS/DM, the DQ steering, the `DDR4_ADRMOD` address remap, and one `ddr4_model` per byte lane — transcribed from the generated `sim_tb_top.sv` for this part |
| `tb_ddr_uplink.sv` | **shared verbatim with G4a**; under `` `ifdef G4B_PHY `` it instantiates `ddr4_mem_c0` on the wrapper's `ddr4_sdram_c0_*` pins and raises the runtime limit |

The template these are transcribed from lives at
`$XILINX_VIVADO/data/ip/xilinx/ddr4_v2_2/data/dlib/ultrascale/ddr4_sdram/tb/sim_tb_top.sv`. It must NOT
be used directly: it is a template whose `` `define DDR4_16G_X8 `` / `` `define DDR4_938_Timing ``  and
package includes describe a different device — simulating it would answer the wrong question, which is
exactly what G4b exists to rule out.

## Running (once complete)

```bash
cd vivado-scripts/riscvsoc-bd
RISCQ_DDR_READOUT=1 RISCQ_DDR_SIM_PHY=1 \
  RISCQ_CONFIG=../../software/configs/sim-2q-ddr.json \
  RISCQ_PROJ_NAME=ddr-bd-g4b RISCQ_RUN_SYNTH=0 ./build-riscvsoc-bd.sh   # BD only, no synthesis needed
vivado -mode batch -source sim/gen-ddr4-model.tcl -tclargs ../../build/ddr-bd-g4b
vivado -mode batch -source sim/run-sim-bd.tcl    -tclargs ../../build/ddr-bd-g4b 3ms phy
```

`Simulation_Mode` is a **simulation-only** IP property: synthesis and the bitstream are byte-identical to
a BFM build, so G4b cannot perturb the G5 artifact.

The test body is the same 12-word byte-exact injector run as G4a — per Codex r14, a single word would
exercise neither multiple 256-bit beats, nor lane placement, nor ordering, nor the final partial-burst
pad, nor TLAST framing.
