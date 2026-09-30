# ===========================================================================================
# Wiring for the readout-uplink DDR path (qubic3). Ported from QubiC bd_ddr_streaming.tcl:174-263,
# reduced to the single readout MIG. Sourced only when results_path = antq_uplink (inc/config.tcl),
# after ddr-config.tcl.
#
# Domains: the MIG's own c0_ddr4_ui_clk (~333 MHz) is the uplink's `ddrClk`. Everything on the uplink
# side of the design (top/ddrClk, the DMA, smc_dma, the PS HP0 port) runs on it, so there is
# exactly ONE new clock domain in the BD.
# ===========================================================================================
# Guard (belt to create-project.tcl's RTL check): the packaged SoC must have the uplink ports.
foreach _p {ddrClk ddrCalibDone} {
  if {[llength [get_bd_pins -quiet $TOP/$_p]] == 0} {
    error "results_path=antq_uplink but the packaged SoC has no `$_p` port: the RTL was not generated\
           from an antq_uplink config (e.g. software/configs/zcu216-14q-antq.json)."
  }
}
set UI_CLK [get_bd_pins ddr4_0/c0_ddr4_ui_clk]

# ---- S_AXI_HP0_FPD for the drain: 128-bit, on ui_clk (hostwindow mode has it 32-bit on hostClk).
# Enabled FIRST: its saxihp0_fpd_aclk pin exists only once the port is on, and is clocked below. ----
set_property -dict {
  CONFIG.PSU__USE__S_AXI_GP2       {1}
  CONFIG.PSU__SAXIGP2__DATA_WIDTH  {128}
} [get_bd_cells zynq_ps]

# ---- clocks ----
connect_bd_net $UI_CLK \
    [get_bd_pins $TOP/ddrClk] \
    [get_bd_pins ddr_rst_stretch/clk] \
    [get_bd_pins psr_ddr/slowest_sync_clk] \
    [get_bd_pins axi_dma_0/s_axi_lite_aclk] \
    [get_bd_pins axi_dma_0/m_axi_s2mm_aclk] \
    [get_bd_pins smc_dma/aclk] \
    [get_bd_pins zynq_ps/saxihp0_fpd_aclk]

# ---- resets. The `reset` module_ref emits an ACTIVE_HIGH stretched reset; proc_sys_reset turns it
# into the ACTIVE_LOW aresetn the AXI IP wants and the ACTIVE_HIGH peripheral_reset the SoC top wants
# (matching the existing hostRst/dspRst convention). ----
connect_bd_net [get_bd_pins ddr_rst_stretch/rst] \
    [get_bd_pins ddr4_0/sys_rst] \
    [get_bd_pins psr_ddr/ext_reset_in]
connect_bd_net [get_bd_pins psr_ddr/peripheral_aresetn] \
    [get_bd_pins ddr4_0/c0_ddr4_aresetn] \
    [get_bd_pins axi_dma_0/axi_resetn] \
    [get_bd_pins smc_dma/aresetn]
connect_bd_net [get_bd_pins psr_ddr/peripheral_reset] [get_bd_pins $TOP/ddrRst]

# ---- MIG calibration status -> the SoC IP (Codex r19-B1) ----
# Previously connected to NOTHING, so on the board a failed DDR4 calibration was indistinguishable from
# a wedged uplink. The SoC publishes it in its HOST-domain status register (readable even when the whole
# ui_clk side is dead) and mirrors it in the uplink's DIAG[9].
connect_bd_net [get_bd_pins ddr4_0/c0_init_calib_complete] [get_bd_pins $TOP/ddrCalibDone]

# ---- data paths ----
# uplink AXI write/read master -> MIG, direct (P3c: page-bounded bursts, same clock/width/ID; ddr-config.tcl).
# The net is named, so the G4 testbench can force the MIG's BRESP/RRESP at a stable path
# (tb_ddr_uplink.DUT.riscq_bd_i.uplink_ddr_axi_*).
connect_bd_intf_net -intf_net uplink_ddr_axi [get_bd_intf_pins $TOP/M_AXI_DDR] [get_bd_intf_pins ddr4_0/C0_DDR4_S_AXI]
# uplink AXIS drain -> DMA S2MM -> SmartConnect (256->128 down-size) -> PS HP0
connect_bd_intf_net [get_bd_intf_pins $TOP/M_AXIS_RD]        [get_bd_intf_pins axi_dma_0/S_AXIS_S2MM]
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXI_S2MM]  [get_bd_intf_pins smc_dma/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins smc_dma/M00_AXI]       [get_bd_intf_pins zynq_ps/S_AXI_HP0_FPD]

# ---- control plane: the existing PS SmartConnect gains the uplink control slave + the DMA lite port.
# Both live in the LPD window (0x9...) so the Zynq VIP's address dispatch in the G4 xsim selects
# M_AXI_HPM0_LPD, exactly as production does. These constants are mirrored in software/riscq/ddr.py
# (DdrMap) and pinned by software/tests/test_ddr_contract.py. ----
# A DEDICATED cross-clock SmartConnect carries the control plane from the PS clock into the ui_clk
# domain (QubiC does the same with its ps8_0_axi_periph). Extending the RFDC SmartConnect instead would
# leave its M02/M03 annotated with the PS clock and fail validation.
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smc_ctrl
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {2} CONFIG.NUM_CLKS {2}] [get_bd_cells smc_ctrl]
connect_bd_intf_net [get_bd_intf_pins $AXI_CONNECT/M02_AXI] [get_bd_intf_pins smc_ctrl/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins smc_ctrl/M00_AXI]     [get_bd_intf_pins $TOP/S_AXI_DDR_CTRL]
connect_bd_intf_net [get_bd_intf_pins smc_ctrl/M01_AXI]     [get_bd_intf_pins axi_dma_0/S_AXI_LITE]
connect_bd_net [get_bd_pins zynq_ps/pl_clk0] [get_bd_pins smc_ctrl/aclk]
connect_bd_net $UI_CLK                       [get_bd_pins smc_ctrl/aclk1]
connect_bd_net [get_bd_pins psr_ddr/peripheral_aresetn] [get_bd_pins smc_ctrl/aresetn]

assign_bd_address -offset 0x90000000 -range 0x00010000 \
  -target_address_space [get_bd_addr_spaces zynq_ps/Data] [get_bd_addr_segs $TOP/S_AXI_DDR_CTRL/reg0] -force
assign_bd_address -offset 0x90010000 -range 0x00010000 \
  -target_address_space [get_bd_addr_spaces zynq_ps/Data] [get_bd_addr_segs axi_dma_0/S_AXI_LITE/Reg] -force
