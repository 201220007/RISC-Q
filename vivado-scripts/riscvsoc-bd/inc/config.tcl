# ---- riscvsoc-bd configuration (script-scope; override any of these from the environment) -----------
source [file join [file dirname [info script]] threads.tcl]   ;# general.maxThreads for this session

set PART          xczu49dr-ffvf1760-2-e
set TOP_MODULE    PulseTableSoc
set BD_NAME       riscq_bd
set DSP_FREQ      500000000
set HOST_FREQ     100000000

# Run stages. Synthesis is on by default; implementation / bitstream are long, so opt-in.
set RUN_SYNTH     1
set RUN_IMPL      0
set RUN_BITSTREAM 0

if {[info exists ::env(RISCQ_DEVICE)]}        { set PART          $::env(RISCQ_DEVICE) }
if {[info exists ::env(RISCQ_TOP)]}           { set TOP_MODULE    $::env(RISCQ_TOP) }
if {[info exists ::env(RISCQ_DSP_FREQ)]}      { set DSP_FREQ      $::env(RISCQ_DSP_FREQ) }
if {[info exists ::env(RISCQ_HOST_FREQ)]}     { set HOST_FREQ     $::env(RISCQ_HOST_FREQ) }
if {[info exists ::env(RISCQ_RUN_SYNTH)]}     { set RUN_SYNTH     $::env(RISCQ_RUN_SYNTH) }
if {[info exists ::env(RISCQ_RUN_IMPL)]}      { set RUN_IMPL      $::env(RISCQ_RUN_IMPL) }
if {[info exists ::env(RISCQ_RUN_BITSTREAM)]} { set RUN_BITSTREAM $::env(RISCQ_RUN_BITSTREAM) }

# Bitstream implies implementation.
if {$RUN_BITSTREAM} { set RUN_IMPL 1 }

# ---- results path (qubic3): the SocSpec JSON's `results_path` is the ONE build authority ----------
# `hostwindow` (the default) is upstream's HostWindow: M_AXI_HOST -> S_AXI_HP0_FPD, 32-bit, hostClk.
# `antq_uplink` is the Ant-Q readout uplink: the DDR4 MIG + axi_dma + SmartConnects, with smc_dma ->
# S_AXI_HP0_FPD at 128-bit on the MIG ui_clk. The mode is read from the same JSON the RTL was generated
# from (RISCQ_CONFIG, exported by build-riscvsoc-bd.sh), never from a separate switch, and
# create-project.tcl checks that the RTL's ports agree with it.
set CONFIG_JSON [file normalize $SCRIPT_DIR/../../software/configs/zcu216-14q.json]
if {[info exists ::env(RISCQ_CONFIG)]} { set CONFIG_JSON [file normalize $::env(RISCQ_CONFIG)] }
if {[info exists ::env(RISCQ_DDR_READOUT)]} {
  error "RISCQ_DDR_READOUT is gone: the results path comes only from the config JSON's\
         `results_path` (\"antq_uplink\" or \"hostwindow\"). Unset RISCQ_DDR_READOUT and set RISCQ_CONFIG,\
         e.g. software/configs/zcu216-14q-antq.json."
}
if {![file exists $CONFIG_JSON]} { error "config JSON $CONFIG_JSON not found (RISCQ_CONFIG)" }
set _fh [open $CONFIG_JSON r]; set _cfgtext [read $_fh]; close $_fh
if {[regexp {"ddr_readout"\s*:} $_cfgtext]} {
  error "$CONFIG_JSON carries the legacy `ddr_readout` key: use `\"results_path\": \"antq_uplink\"`"
}
set _rp_hits [regexp -all -inline {"results_path"\s*:\s*("[^"]*"|[^,\}\s]+)} $_cfgtext]
if {[llength $_rp_hits] > 2} { error "$CONFIG_JSON states `results_path` more than once" }
set RESULTS_PATH hostwindow
if {[llength $_rp_hits] == 2} {
  set _rp [lindex $_rp_hits 1]
  if {![regexp {^"([^"]*)"$} $_rp -> RESULTS_PATH]} { error "results_path must be a JSON string, got $_rp" }
}
if {$RESULTS_PATH ni {hostwindow antq_uplink}} {
  error "results_path '$RESULTS_PATH' in $CONFIG_JSON is not one of {hostwindow, antq_uplink}"
}
set ANTQ_UPLINK [expr {$RESULTS_PATH eq "antq_uplink"}]
# ZCU216 board part: the MIG's board interfaces (ddr4_sdram_c0 / default_sysclk_c0_300mhz) exist only in
# a board-part project, which is therefore used in antq_uplink only (hostwindow keeps upstream's bare part).
set BOARD_PART    xilinx.com:zcu216:part0:2.0
if {[info exists ::env(RISCQ_BOARD_PART)]}    { set BOARD_PART    $::env(RISCQ_BOARD_PART) }
# MIG ui_clk (300 MHz sysclk / CLKOUT0_DIVIDE 3). Must equal PulseTableSoc.DdrClkFreqHz, or
# validate_bd_design fails on the FREQ_HZ of every uplink bus.
set DDR_FREQ      333250000

# Paths. One folder per project under the repo-root build/ (git-ignored), so several designs build in
# parallel without clobbering each other. The RTL (PulseTableSoc.v + ClockInterface.v + register-file
# .bin) is emitted into that same folder by build-riscvsoc-bd.sh's GenPulseTableSocJson, so SOURCE_PATH
# is the build dir itself. RISCQ_PROJ_NAME names the folder; RISCQ_BUILD_DIR overrides the full path.
set PROJ_NAME   riscvsoc-bd
if {[info exists ::env(RISCQ_PROJ_NAME)]}     { set PROJ_NAME   $::env(RISCQ_PROJ_NAME) }
set BUILD_DIR   [file normalize $SCRIPT_DIR/../../build/$PROJ_NAME]
if {[info exists ::env(RISCQ_BUILD_DIR)]}     { set BUILD_DIR   $::env(RISCQ_BUILD_DIR) }
set SOURCE_PATH $BUILD_DIR
if {[info exists ::env(RISCQ_RTL_DIR)]}       { set SOURCE_PATH $::env(RISCQ_RTL_DIR) }
set IP_REPO     $BUILD_DIR/ip
# Vivado project name (the .xpr / .runs / .gen prefix) — sanitise the folder name to the underscore-safe
# subset create_project accepts.
set PRJ         [regsub -all {[^A-Za-z0-9_]} $PROJ_NAME _]

puts "\[config\] top=$TOP_MODULE part=$PART dsp=${DSP_FREQ}Hz host=${HOST_FREQ}Hz  synth=$RUN_SYNTH impl=$RUN_IMPL bit=$RUN_BITSTREAM"
puts "\[config\] rtl=$SOURCE_PATH  build=$BUILD_DIR"
puts "\[config\] results_path=$RESULTS_PATH (from $CONFIG_JSON)"

if {![file exists $SOURCE_PATH/$TOP_MODULE.v]} {
  error "missing $SOURCE_PATH/$TOP_MODULE.v — run ./build-riscvsoc-bd.sh (it generates the RTL first)"
}
file mkdir $BUILD_DIR
