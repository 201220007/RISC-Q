# ===========================================================================================
# DDR4 controller for the readout uplink (qubic3). Config taken from the QubiC board-proven readout
# MIG (gateware_data_comm_readout_cmd top/src/bd_ddr_streaming.tcl:36-51, bank **c0**) -- the same
# physical DDR4 on the same ZCU216. Sourced only when results_path = antq_uplink (inc/config.tcl).
#
# The board preset supplies the pinout/timing via apply_board_connection, so no hand-written XDC.
#
# DELTAS vs the QubiC source, each deliberate (r11-#12 -- the previous version claimed "verbatim" and
# was not; in particular CS_WIDTH was missing, which on this clamshell part is a PINOUT property):
#   + C0.DDR4_AxiSelection {true}  - QubiC leans on the IP default; stated explicitly because the whole
#                                    uplink assumes an AXI4 slave rather than the native user interface.
#   + C0.DDR4_AxiIDWidth   {4}     - QubiC leans on the default (4). mmu2 drives arid, so pin it.
#   - ADDN_UI_CLKOUT1_FREQ_HZ {100}- QubiC exports a second 100 MHz ui clock for its command-path logic.
#                                    C1 has no consumer for it; an unused MMCM output is dead area.
# Everything else below is byte-for-byte the QubiC c0 dict.
# ===========================================================================================
set DDR4 [create_bd_cell -type ip -vlnv xilinx.com:ip:ddr4:2.2 ddr4_0]
set_property -dict [list \
    CONFIG.C0_DDR4_BOARD_INTERFACE  {ddr4_sdram_c0} \
    CONFIG.C0_CLOCK_BOARD_INTERFACE {default_sysclk_c0_300mhz} \
    CONFIG.System_Clock             {Differential} \
    CONFIG.C0.DDR4_AxiSelection     {true} \
    CONFIG.C0.DDR4_AxiDataWidth     {256} \
    CONFIG.C0.DDR4_AxiAddressWidth  {32} \
    CONFIG.C0.DDR4_AxiIDWidth       {4} \
    CONFIG.C0.DDR4_CLKOUT0_DIVIDE   {3} \
    CONFIG.C0.DDR4_InputClockPeriod {3334} \
    CONFIG.C0.DDR4_MemoryPart       {MT40A1G8WE-075E} \
    CONFIG.C0.DDR4_DataWidth        {32} \
    CONFIG.C0.CS_WIDTH              {2} \
    CONFIG.C0.DDR4_Clamshell        {true} \
    CONFIG.Debug_Signal             {Disable} \
] [get_bd_cells ddr4_0]
# G4b: `Simulation_Mode` selects what the IP's SIMULATION model contains -- BFM (default) swaps the
# XiPhy primitives for a behavioural model, so there is no calibration and no DRAM device; Unisim keeps
# the real PHY, which is the only way to simulate the generated pinout, the clamshell CS wiring, PHY
# reset and calibration against a Micron model (Codex r13-#12: mandatory once before the board).
# It has NO effect on synthesis or the bitstream -- it is a simulation-only knob.
if {[info exists ::env(RISCQ_DDR_SIM_PHY)] && $::env(RISCQ_DDR_SIM_PHY)} {
  set_property CONFIG.Simulation_Mode {Unisim} [get_bd_cells ddr4_0]
  puts "\[ddr-config\] Simulation_Mode = Unisim (full PHY; simulation only, bitstream unchanged)"
}
apply_board_connection -board_interface "ddr4_sdram_c0"            -ip_intf "ddr4_0/C0_DDR4"    -diagram $BD_NAME
apply_board_connection -board_interface "default_sysclk_c0_300mhz" -ip_intf "ddr4_0/C0_SYS_CLK" -diagram $BD_NAME

# ---- S2MM-only DMA: PL AXIS (readout drain) -> PS HP0. Config mirrors QubiC's (bd_ddr_streaming.tcl:77-92)
# minus the MM2S half (and its DRE + burst-size knobs), which belong to the command path -- out of
# scope for C1. The S2MM half below is byte-for-byte QubiC's.
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:7.1 axi_dma_0
set_property -dict [list \
    CONFIG.c_include_sg              {0} \
    CONFIG.c_include_mm2s            {0} \
    CONFIG.c_include_s2mm            {1} \
    CONFIG.c_include_s2mm_dre        {0} \
    CONFIG.c_m_axi_s2mm_data_width   {256} \
    CONFIG.c_s_axis_s2mm_tdata_width {256} \
    CONFIG.c_s2mm_burst_size         {32} \
    CONFIG.c_sg_length_width         {26} \
] [get_bd_cells axi_dma_0]

# ---- reset tree for the MIG ui_clk domain (QubiC pattern: a 5000-cycle stretcher feeding both the
# MIG's sys_rst and a proc_sys_reset, clocked BY the ui_clk itself -- bd_ddr_streaming.tcl:103-113,178).
create_bd_cell -type module -reference reset ddr_rst_stretch
set_property -dict [list CONFIG.N {5000}] [get_bd_cells ddr_rst_stretch]
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 psr_ddr

# ---- SmartConnects.
# smc_ddr: the vendored QubiC mmu2 emitted arlen=255 (8 KiB) bursts that crossed 4 KiB boundaries
# (AXI A3.4.1), and this SmartConnect legalised them (QubiC bd_ddr_streaming.tcl:122-123, "Required, NOT a
# bare direct net"). Since the P3a SpinalHDL rewrite every uplink burst is page-bounded by construction
# (src/riscq/ddr/CONTRACT.md F1), so smc_ddr is no longer needed for correctness; the BD keeps it unchanged.
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smc_ddr
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {1}] [get_bd_cells smc_ddr]
# DMA M_AXI_S2MM (256-bit, ui_clk) -> PS HP0 (128-bit, ui_clk): width down-size.
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smc_dma
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {1}] [get_bd_cells smc_dma]
