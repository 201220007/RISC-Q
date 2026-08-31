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
# VLNV 3.* : Vivado 2022.1 ships zynq_ultra_ps_e 3.3/3.4, not 3.5 (the pinned 3.5 fails to instantiate).
set ZYNQ_PS [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.* zynq_ps]
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} \
  [get_bd_cells zynq_ps]


# The ZCU216 board preset (applied only in the DDR flow, see create-project.tcl) does NOT enable
# M_AXI_HPM0_LPD, which the control plane needs, so the PS ports are set explicitly here:
#   M_AXI_GP2 = M_AXI_HPM0_LPD (control), S_AXI_GP2 = S_AXI_HP0_FPD (the uplink DMA's target).
# The preset also turns ON HPM0/HPM1_FPD, which this design does not use -- an enabled-but-unclocked
# master fails validate_bd_design, so they are explicitly turned off.
# Mirrors QubiC ps.tcl:4-16.
if {$DDR_READOUT} {
  set_property -dict {
    CONFIG.PSU__USE__M_AXI_GP0       {0}
    CONFIG.PSU__USE__M_AXI_GP1       {0}
    CONFIG.PSU__USE__M_AXI_GP2       {1}
    CONFIG.PSU__USE__S_AXI_GP2       {1}
    CONFIG.PSU__SAXIGP2__DATA_WIDTH  {128}
  } [get_bd_cells zynq_ps]
}

# ---- User top IP ----
connect_bd_net [get_bd_pins zynq_ps/pl_clk0] [get_bd_pins zynq_ps/maxihpm0_lpd_aclk]

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
# +2 masters (uplink control block, DMA lite) and +1 clock (the MIG ui_clk) when the uplink is built.
# +1 master when the uplink is built: it feeds a dedicated cross-clock SmartConnect (smc_ctrl) that
# carries the control plane into the MIG ui_clk domain, so this one stays single-PS-clock as before.
set_property CONFIG.NUM_MI [expr {$DDR_READOUT ? 3 : 2}] $AXI_CONNECT
set_property CONFIG.NUM_CLKS {2} $AXI_CONNECT
set_property CONFIG.HAS_ARESETN {0} $AXI_CONNECT

connect_bd_intf_net [get_bd_intf_pins $AXI_CONNECT/M00_AXI]      [get_bd_intf_pins $TOP/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins $AXI_CONNECT/M01_AXI]      [get_bd_intf_pins rf_data_converter/s_axi]
connect_bd_intf_net [get_bd_intf_pins zynq_ps/M_AXI_HPM0_LPD]    [get_bd_intf_pins $AXI_CONNECT/S00_AXI]
connect_bd_net      [get_bd_pins zynq_ps/pl_clk0]                [get_bd_pins $AXI_CONNECT/aclk]
connect_bd_net      [get_bd_pins $CLKIFC/hostClk]                [get_bd_pins $AXI_CONNECT/aclk1]

if {$DDR_READOUT} {
  source $INC/ddr-config.tcl     ;# ddr4 MIG + S2MM DMA + reset stretcher + the two SmartConnects
  source $INC/ddr-connect.tcl    ;# ui_clk domain wiring, reset tree, interfaces, LPD addresses
}

assign_bd_address -offset 0x80000000 -range 0x10000000 \
  -target_address_space [get_bd_addr_spaces zynq_ps/Data] [get_bd_addr_segs $TOP/S_AXIS/reg0] -force
assign_bd_address
