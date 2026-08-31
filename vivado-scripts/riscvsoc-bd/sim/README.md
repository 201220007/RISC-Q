# G4a — block-design xsim of the readout → DDR uplink (AXI/integration gate)

The only gate that runs the design with the **real Vivado IP in the loop**. G2 (`ReadoutDdrUplinkSim`)
and G3 (`PulseTableSocDdrSim`) drive the SoC's ports against behavioural AXI models; everything
*between* those ports and the PS exists only here:

| exercised only by G4 | why it can't be covered by G2/G3 |
|---|---|
| PS address decode (`0x9000_0000` ctrl, `0x9001_0000` DMA) via **M_AXI_HPM0_LPD** | G2 drives the control slave directly |
| `smc_ctrl` — the cross-clock SmartConnect from `pl_clk0` into `ui_clk` | there is one clock in G2's control path |
| `smc_ddr` — 4 KiB burst legalisation for mmu2's `arlen=255` reads | `AxiMemorySim` accepts illegal bursts silently |
| the DDR4 MIG's own AXI slave (`Simulation_Mode = BFM`) | ditto |
| `axi_dma_0` S2MM: AXIS → `smc_dma` (256→128 down-size) → **PS HP0** | G2 collects the AXIS beats in the testbench |
| the `ui_clk` reset tree (stretcher → `proc_sys_reset`) | G2/G3 drive resets directly |

Stimulus is the built-in **test injector**, so no RF and no `ReadoutDecoder` are involved — the point of
this gate is the plumbing, not the DSP (G3 already proved decoder ⇄ DDR equivalence).

```
base_reset → 12 × inj_fire → flush → arm S2MM DMA → rd_start → read PS memory → byte-exact compare
```

## Running

```bash
# 1. build a feature-on BD (once)
cd vivado-scripts/riscvsoc-bd
RISCQ_DDR_READOUT=1 RISCQ_CONFIG=../../software/configs/sim-2q-ddr.json \
  RISCQ_PROJ_NAME=ddr-bd-smoke RISCQ_RUN_BITSTREAM=0 ./build-riscvsoc-bd.sh

# 2. simulate it
vivado -mode batch -source sim/run-sim-bd.tcl -tclargs ../../build/ddr-bd-smoke
```

The testbench self-checks and prints exactly one verdict line, `[G4] PASS: …` or `[G4] FAIL: …`, and has
a 500 µs hard timeout so a hang fails instead of blocking.

## Where the verdict is taken, and why not from memory

The PASS/FAIL is decided on the **`M_AXI_S2MM` write beats** — every 64-bit word and its byte strobes,
captured as the DMA hands them to the PS — not on a readback of PS memory. Two reasons, both learned the
hard way:

1. **It is stronger.** A memory image only shows the end state; the beat log checks every beat, every
   lane and every strobe, so an all-zero-`WSTRB` burst (which AXI answers `OKAY` and which writes
   nothing) fails loudly instead of looking like a memory that "did not update".
2. **The Vivado 2022.1 Zynq VIP does not update its `read_mem()` store from HP0 slave writes.** Proven
   here, not assumed: with the destination pre-poisoned and verified readable, a run produced 3 AXIS
   beats with TLAST, one `AW` at the programmed address, 3 `W` beats with `WLAST`, `BRESP = OKAY` and
   `S2MM_LENGTH` reading back 96 — and every destination word still held its poison.

`read_mem()` is still consulted per word and any disagreement is printed, but it is informational.
Note that **G4b does not close this either** — its Micron model terminates the PL MIG interface, while
HP0 still terminates in the same VIP. PS-memory persistence is a hardware claim (G6), not a simulation
one.

**Do not add a PS "front-door" read of the destination.** The VIP's `read_data`/`write_data` model
PS→PL transactions only: `check_master_address()` accepts addresses only inside the
`M_AXI_GP0/1/2` apertures, and on anything else the task prints an error and — with
`set_stop_on_error(1)` — calls `$stop`, which hangs a batch xsim with no diagnostic. That cost one full
run.

## Notes

- The PS is the **Zynq UltraScale+ VIP**, reached at `DUT.riscq_bd_i.zynq_ps.inst`. Its `write_data` /
  `read_data` tasks act as the PS master (HPM0_LPD), and `read_mem` reads the same modelled memory the
  HP0 slave port writes into — so the DMA's output is checked at the exact place software would read it.
- The DDR4 IP is generated with `Simulation_Mode = BFM` (the IP default), so there is **no Micron DRAM
  model and no calibration wait**: the MIG presents its AXI slave backed by a behavioural memory.
  This is deliberate for the *fast* gate — the AXI-side behaviour (4 KiB legalisation, ordering) is what
  we want in the loop here. It is **not** a substitute for a full-PHY run; see "Scope" below.
- RFDC analog ports are left unconnected: nothing in this test touches the converter.

## Scope — what this gate does NOT cover

Stated explicitly so the PASS is not read for more than it is worth (Codex r13-#12):

- **No DDR PHY, no DRAM.** BFM mode replaces the PHY and the memory device with an AXI-side behavioural
  model, so the generated MIG pinout/timing, the clamshell `CS_WIDTH=2` wiring, PHY reset and
  calibration, and the DDR4 device protocol are **not** exercised here. Those need a full-PHY run with
  the Micron model — tracked separately as **G4b**, which is mandatory once before the board.
- **No controlled memory backpressure.** The BFM answers at its own pace; this gate does not *drive* a
  slow or stalling DDR. Backpressure through `wr_ready` / the skid FIFO is covered in G2
  (`cbuf_wr_stall`, where `wr_ready` is observed to fall and the throttle to engage).
- **No RF.** The stimulus is the register-paced injector. Decoder ⇄ DDR equivalence is G3's job.

What it *does* cover is the plumbing between the SoC's ports and the PS, which exists nowhere else:
address decode over HPM0_LPD, the `smc_ctrl` clock crossing, `smc_ddr` in front of the MIG's AXI slave,
the `axi_dma` S2MM path into HP0 memory, and the `ui_clk` reset tree.
