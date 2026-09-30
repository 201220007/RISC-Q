# ===========================================================================================
# Assemble the block design: ClockInterface + Zynq PS + user top IP + proc_sys_resets + RF Data
# Converter + AXI SmartConnect. Cell handles ($CLKIFC / $ZYNQ_PS / $TOP / $RFDC) are published at
# script scope for bd-finalize.tcl. Mirrors the RISC-Q utils/*.tcl, merged and proc-free.
# ===========================================================================================

create_bd_design -dir $BUILD_DIR/bd $BD_NAME

# ---- ClockInterface: three external LVDS clock pairs -> single-ended dspClk / hostClk / user_sysref ----
set CLKIFC [create_bd_cell -type module -reference ClockInterface clkifc]

create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 dspClk
connect_bd_intf_net [get_bd_intf_ports dspClk] [get_bd_intf_pins $CLKIFC/dspClk_diff]
create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 hostClk
connect_bd_intf_net [get_bd_intf_ports hostClk] [get_bd_intf_pins $CLKIFC/hostClk_diff]
create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 user_sysref
connect_bd_intf_net [get_bd_intf_ports user_sysref] [get_bd_intf_pins $CLKIFC/user_sysref_diff]

set_property -dict [list CONFIG.FREQ_HZ $DSP_FREQ]  [get_bd_intf_ports dspClk]
set_property -dict [list CONFIG.FREQ_HZ $HOST_FREQ] [get_bd_intf_ports hostClk]
set_property -dict [list CONFIG.FREQ_HZ $DSP_FREQ]  [get_bd_pins $CLKIFC/dspClk]
set_property -dict [list CONFIG.FREQ_HZ $HOST_FREQ] [get_bd_pins $CLKIFC/hostClk]

# ---- Zynq UltraScale+ PS: supplies pl_clk0 / pl_resetn0 / M_AXI_HPM0_LPD ----
set ZYNQ_PS [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ps]
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} \
  [get_bd_cells zynq_ps]
# antq_uplink: the project has the ZCU216 board part (create-project.tcl), whose PS preset turns
# M_AXI_HPM0_LPD off and HPM0/HPM1_FPD on. Set the ports this design uses explicitly: HPM0_LPD (the
# control plane) on, the two FPD masters off (an enabled but unclocked master fails validation).
if {$ANTQ_UPLINK} {
  set_property -dict {
    CONFIG.PSU__USE__M_AXI_GP0 {0}
    CONFIG.PSU__USE__M_AXI_GP1 {0}
    CONFIG.PSU__USE__M_AXI_GP2 {1}
  } $ZYNQ_PS
}
connect_bd_net [get_bd_pins zynq_ps/pl_clk0] [get_bd_pins zynq_ps/maxihpm0_lpd_aclk]

# ---- User top IP ----
set TOP [create_bd_cell -type ip -vlnv user.org:user:${TOP_MODULE}:1.0 top]
connect_bd_net [get_bd_pins $CLKIFC/hostClk] [get_bd_pins $TOP/hostClk]
connect_bd_net [get_bd_pins $CLKIFC/dspClk]  [get_bd_pins $TOP/dspClk]

# ---- proc_sys_reset for each clock domain ----
set PS_RST  [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 ps_rst]
set DSP_RST [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 dsp_rst]
connect_bd_net [get_bd_pins zynq_ps/pl_resetn0] [get_bd_pins ps_rst/ext_reset_in] [get_bd_pins dsp_rst/ext_reset_in]
connect_bd_net [get_bd_pins dsp_rst/peripheral_reset] [get_bd_pins $TOP/dspRst]
connect_bd_net [get_bd_pins ps_rst/peripheral_reset]  [get_bd_pins $TOP/hostRst]
connect_bd_net [get_bd_pins $CLKIFC/hostClk] [get_bd_pins ps_rst/slowest_sync_clk]
connect_bd_net [get_bd_pins $CLKIFC/dspClk]  [get_bd_pins dsp_rst/slowest_sync_clk]

# ---- RF Data Converter (16 DAC + 16 ADC), config verbatim from the RISC-Q reference ----
set RFDC [create_bd_cell -type ip -vlnv xilinx.com:ip:usp_rf_data_converter:2.6 rf_data_converter]
set RFDC_TARGET [get_bd_cells rf_data_converter]
source $INC/rfdc-config.tcl
source $INC/rfdc-connect.tcl

# ---- AXI SmartConnect: PS HPM0_LPD -> { top S_AXIS, rfdc s_axi } ----
set AXI_CONNECT [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smartconnect]
set_property CONFIG.NUM_SI 1 $AXI_CONNECT
# antq_uplink: +1 master, feeding the cross-clock smc_ctrl that carries the control plane into ui_clk
set_property CONFIG.NUM_MI [expr {$ANTQ_UPLINK ? 3 : 2}] $AXI_CONNECT
set_property CONFIG.NUM_CLKS {2} $AXI_CONNECT
set_property CONFIG.HAS_ARESETN {0} $AXI_CONNECT

connect_bd_intf_net [get_bd_intf_pins $AXI_CONNECT/M00_AXI]      [get_bd_intf_pins $TOP/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins $AXI_CONNECT/M01_AXI]      [get_bd_intf_pins rf_data_converter/s_axi]
connect_bd_intf_net [get_bd_intf_pins zynq_ps/M_AXI_HPM0_LPD]    [get_bd_intf_pins $AXI_CONNECT/S00_AXI]
connect_bd_net      [get_bd_pins zynq_ps/pl_clk0]                [get_bd_pins $AXI_CONNECT/aclk]
connect_bd_net      [get_bd_pins $CLKIFC/hostClk]                [get_bd_pins $AXI_CONNECT/aclk1]

if {$ANTQ_UPLINK} {
  source $INC/ddr-config.tcl     ;# DDR4 MIG + S2MM DMA + reset stretcher + smc_ddr / smc_dma
  source $INC/ddr-connect.tcl    ;# ui_clk wiring, reset tree, interfaces, control-plane addresses
}

assign_bd_address -offset 0x80000000 -range 0x10000000 \
  -target_address_space [get_bd_addr_spaces zynq_ps/Data] [get_bd_addr_segs $TOP/S_AXIS/reg0] -force
assign_bd_address

# ---- S_AXI_HP0_FPD: exactly one results path owns it (SocSpec results_path) ----
if {!$ANTQ_UPLINK} {
# ---- Host window: per-core results -> PS DDR4 over S_AXI_HP0_FPD (specs/software/22 §2.5) ----
# HP0 owns a DDRC port of its own (HP1/HP2 share one); the width is pinned to 32 so IP Integrator
# inserts no width converter, and the port is a DIRECT connection, not through the SmartConnect.
set_property -dict [list CONFIG.PSU__USE__S_AXI_GP2 {1} CONFIG.PSU__SAXIGP2__DATA_WIDTH {32}] $ZYNQ_PS
connect_bd_intf_net [get_bd_intf_pins $TOP/M_AXI_HOST] [get_bd_intf_pins zynq_ps/S_AXI_HP0_FPD]
connect_bd_net      [get_bd_pins $CLKIFC/hostClk]      [get_bd_pins zynq_ps/saxihp0_fpd_aclk]
assign_bd_address -target_address_space [get_bd_addr_spaces $TOP/M_AXI_HOST] \
  [get_bd_addr_segs zynq_ps/SAXIGP2/HP0_DDR_LOW] -force
assign_bd_address -target_address_space [get_bd_addr_spaces $TOP/M_AXI_HOST] \
  [get_bd_addr_segs zynq_ps/SAXIGP2/HP0_DDR_HIGH] -force
} else {
# ---- Ant-Q uplink drain: axi_dma S2MM (256-bit) -> smc_dma (down-size) -> S_AXI_HP0_FPD, 128-bit on
# the MIG ui_clk. The DMA has a 32-bit address port, so it reaches HP0_DDR_LOW only (where the CMA
# buffer lives); ddr_board.py DA_WIDTH mirrors that. (Enabled + widened in ddr-connect.tcl.)
assign_bd_address -target_address_space [get_bd_addr_spaces axi_dma_0/Data_S2MM] \
  [get_bd_addr_segs zynq_ps/SAXIGP2/HP0_DDR_LOW] -force
}

# ---- White Rabbit GTY pins (with_white_rabbit builds only — the IP then has wr* pins): the GT
# refclk + serial lanes go straight out as BD ports; constraints-wr.xdc places them (SFP0/X0Y4).
# wrMarker (the digital marker pin) stays unconnected — the marker rides a spare DAC instead. ----
if {[llength [get_bd_pins -quiet $TOP/wrRefClkP]]} {
  foreach p {wrRefClkP wrRefClkN wrRxP wrRxN} {
    create_bd_port -dir I $p
    connect_bd_net [get_bd_ports $p] [get_bd_pins $TOP/$p]
  }
  foreach p {wrTxP wrTxN} {
    create_bd_port -dir O $p
    connect_bd_net [get_bd_ports $p] [get_bd_pins $TOP/$p]
  }
}
