# ===========================================================================================
# Emit the DDR4 memory model CUSTOMISED FOR THIS PART.
#
#   vivado -mode batch -source gen-ddr4-model.tcl -tclargs <build_dir>
#
# WHY A MODEL IS NEEDED AT ALL
#   `Simulation_Mode = BFM` makes the DDR4 IP's XiPhy PRIMITIVES behavioural. It does NOT supply a
#   memory -- the controller still drives a real DDR4 device on the `c0_ddr4_*` pins. With those pins
#   unconnected every write goes nowhere and every read returns zeros (G4a run 6: perfect burst framing,
#   full byte strobes, BRESP=OKAY, all-zero payload). A model is required in BOTH simulation modes.
#
# WHY NOT THE VIVADO TEMPLATE
#   `$XILINX_VIVADO/data/ip/xilinx/ddr4_v2_2/data/dlib/.../tb/` is a TEMPLATE describing a different
#   device: its wrapper defines `DDR4_16G_X8` / `DDR4_938_Timing`. The IP's generated example design for
#   MT40A1G8WE-075E defines `DDR4_8G_X8` / `DDR4_750_Timing` / `FIXED_2666`. Simulating the template
#   would answer the wrong question.
#
# WHY A THROWAWAY PROJECT
#   Running `open_example_project` against the REAL build project registers a stray `ddr4_0` IP in it and
#   leaves a dangling `.Xil/ddr4_0/ddr4_0.xci` reference behind, after which the project will not even
#   open ("Could not find IP file for IP 'ddr4_0'"). Cost one repair. So the example design is generated
#   in a scratch in-memory project holding a standalone DDR4 IP whose CONFIG.* dictionary is COPIED FROM
#   the build's own `riscq_bd_ddr4_0_0.xci`, which guarantees the model matches the configured device
#   without touching the build.
#
# Output: <build_dir>/ddr4_model_sim/  (model sources), listed at the end.
# ===========================================================================================
if {[llength $argv] < 1} { error "usage: -tclargs <build_dir>" }
set BUILD [file normalize [lindex $argv 0]]
set OUT   $BUILD/ddr4_model_sim
set SCRATCH $BUILD/ddr4_model_gen

set XCI [lindex [glob -nocomplain $BUILD/bd/*/ip/*ddr4*/*.xci] 0]
if {$XCI eq ""} { error "no ddr4 .xci under $BUILD/bd/*/ip -- was the BD built from a results_path = antq_uplink config?" }
puts "\[g4-model\] reading the configured device from [file tail $XCI]"

# ---- read the configured dictionary out of the build's own IP, without opening the build project ----
set _fh [open $XCI r]; set _x [read $_fh]; close $_fh
set _cfg {}
# Vivado <= 2022.x writes .xci as IP-XACT XML; 2026.1 writes it as JSON
# (`"schema": "xilinx.com:schema:json_instance:1.0"`). Both forms are read here, because a silent
# empty result would hand the MIG a device-less pin set and every read would come back zero -- the
# exact failure this generator exists to prevent.
if {[string match "*<spirit:*" $_x]} {
  set _fmt xml
  foreach {_all _k _v} [regexp -all -inline {<spirit:configurableElementValue spirit:referenceId="PARAM_VALUE\.([^"]+)">([^<]*)</spirit:configurableElementValue>} $_x] {
    if {[regexp {^(C0\.|C0_|System_Clock$|Simulation_Mode$)} $_k]} { lappend _cfg CONFIG.$_k $_v }
  }
} else {
  set _fmt json
  # "<PARAM>": [ { "value": "<V>", ... } ] under ip_inst/parameters/component_parameters.
  # SCOPE THE SCAN to that section: model_parameters carries 116 more C0_* names (the IP's read-only
  # MODELPARAMs), and the XML branch above excludes them by requiring the PARAM_VALUE. prefix. Without
  # the same scoping every one of them is offered to set_property, fails, and lands in the "rejected"
  # list -- where a genuine component-parameter rejection would be invisible.
  set _i0 [string first "\"component_parameters\"" $_x]
  set _i1 [string first "\"model_parameters\"" $_x]
  if {$_i1 < 0} { set _i1 [string first "\"project_parameters\"" $_x] }
  if {$_i0 < 0 || $_i1 <= $_i0} {
    error "cannot locate the component_parameters section in [file tail $XCI] (i0=$_i0 i1=$_i1)"
  }
  set _cps [string range $_x $_i0 $_i1]
  # No braces in the pattern: a backslash-escaped brace still unbalances Tcl's word parser here.
  foreach {_all _k _v} [regexp -all -inline {"((?:C0\.|C0_)[A-Za-z0-9_.]+|System_Clock|Simulation_Mode)"\s*:\s*\[[^]]*?"value"\s*:\s*"([^"]*)"} $_cps] {
    lappend _cfg CONFIG.$_k $_v
  }
}
if {[llength $_cfg] == 0} { error "could not extract any CONFIG.* from $XCI (detected format: $_fmt)" }
puts "\[g4-model\] .xci format = $_fmt; [expr {[llength $_cfg]/2}] device parameters recovered"
puts "\[g4-model\] [expr {[llength $_cfg]/2}] configuration values copied"

# the part comes from the .xci too, so this script needs no project open
if {![regexp {<spirit:configurableElementValue spirit:referenceId="PROJECT_PARAM\.ARCHITECTURE">([^<]*)<} $_x -> _arch]} { set _arch "" }
# Same XML-vs-JSON split as the parameter block above: 2026.1 keeps the part under
# ip_inst/parameters/project_parameters/<NAME>/0/value instead of a PROJECT_PARAM.* element.
proc _xci_proj_param {blob fmt name} {
  if {$fmt eq "xml"} {
    if {[regexp "<spirit:configurableElementValue spirit:referenceId=\"PROJECT_PARAM\\.$name\">(\[^<\]*)<" $blob -> _v]} { return $_v }
  } else {
    if {[regexp "\"$name\"\\s*:\\s*\\\[\[^\]\]*?\"value\"\\s*:\\s*\"(\[^\"\]*)\"" $blob -> _v]} { return $_v }
  }
  return ""
}
set _dev [_xci_proj_param $_x $_fmt DEVICE]
set _pkg [_xci_proj_param $_x $_fmt PACKAGE]
set _spd [_xci_proj_param $_x $_fmt SPEEDGRADE]
set _tmp [_xci_proj_param $_x $_fmt TEMPERATURE_GRADE]
if {$_dev eq "" || $_pkg eq "" || $_spd eq ""} {
  error "could not read the part out of $XCI (device='$_dev' package='$_pkg' speed='$_spd')"
}
# Vivado part names are <device>-<package>-<speed digits>-<temp>, e.g. xczu49dr-ffvf1760-2-e
set PART "${_dev}-${_pkg}${_spd}-[string tolower $_tmp]"
if {[llength [get_parts -quiet $PART]] == 0} {
  error "reconstructed part '$PART' is not in this Vivado's catalogue (device=$_dev package=$_pkg speed=$_spd temp=$_tmp)"
}
puts "\[g4-model\] part = $PART"

file delete -force $SCRATCH
file mkdir $SCRATCH
create_project -force ddr4_model_gen $SCRATCH -part $PART
set_property board_part xilinx.com:zcu216:part0:2.0 [current_project]
create_ip -vlnv xilinx.com:ip:ddr4:2.2 -module_name ddr4_model_src
if {[catch {set_property -dict $_cfg [get_ips ddr4_model_src]} _e]} {
  puts "\[g4-model\] bulk set_property was rejected ($_e); applying one at a time"
  # r27-#3: a rejection must be RECORDED, not swallowed -- a dropped width/clamshell/mirroring/timing
  # knob would silently generate a model for a different device.
  set _rejected {}
  foreach {_k _v} $_cfg {
    if {[catch {set_property $_k $_v [get_ips ddr4_model_src]} _e2]} { lappend _rejected $_k }
  }
  if {[llength $_rejected]} { puts "\[g4-model\] rejected: $_rejected" }
}

# r27-#3: ROUND-TRIP ASSERT every load-bearing setting. Printing them is not checking them: the model's
# device identity comes from exactly these, so a silently dropped one produces the wrong memory and a
# data test against it means nothing.
array set _want $_cfg
set _must {CONFIG.C0.DDR4_MemoryPart CONFIG.C0.DDR4_DataWidth CONFIG.C0.DDR4_Clamshell
           CONFIG.C0.CS_WIDTH CONFIG.C0.DDR4_CasLatency CONFIG.C0.DDR4_CasWriteLatency
           CONFIG.C0.DDR4_TimePeriod CONFIG.C0.DDR4_InputClockPeriod CONFIG.C0.DDR4_MemoryType
           CONFIG.C0.DDR4_AxiDataWidth CONFIG.C0.DDR4_AxiAddressWidth}
# r28-#5: a key that is ABSENT from the .xci must not silently pass. Either it is genuinely optional for
# this IP version (list it in _optional) or its absence means the extraction is broken.
set _optional {CONFIG.C0.DDR4_CasLatency CONFIG.C0.DDR4_CasWriteLatency}
set _missing {}
foreach _k $_must {
  if {![info exists _want($_k)] && [lsearch -exact $_optional $_k] < 0} { lappend _missing $_k }
}
if {[llength $_missing]} {
  error "these load-bearing settings are absent from [file tail $XCI]: $_missing.\
         The extraction regexp or the IP version changed -- the device-identity check would be partly
         vacuous, so the model must not be generated."
}
set _bad {}
foreach _k $_must {
  if {![info exists _want($_k)]} { continue }        ;# in _optional and genuinely absent
  set _got [get_property $_k [get_ips ddr4_model_src]]
  if {$_got ne $_want($_k)} { lappend _bad "$_k: want '$_want($_k)' got '$_got'" }
  puts [format "\[g4-model\]   %-40s %s" [string range $_k 7 end] $_got]
}
if {[llength $_bad]} {
  error "the scratch IP does not match the build's configured device:\n  [join $_bad "\n  "]\
         \nGenerating a model from it would simulate the WRONG memory."
}

# capture what the assertions at the end need, while the scratch project is still open
set _id_part  [get_property CONFIG.C0.DDR4_MemoryPart [get_ips ddr4_model_src]]
set _id_width [get_property CONFIG.C0.DDR4_DataWidth  [get_ips ddr4_model_src]]
set _id_tck   [get_property CONFIG.C0.DDR4_TimePeriod [get_ips ddr4_model_src]]

puts "\[g4-model\] generating the example design ..."
set EX $SCRATCH/example
file mkdir $EX
open_example_project -force -dir $EX [get_ips ddr4_model_src]

# `ddr4_sdram_model_wrapper.sv` is the only file COMPILED: it `include`s the rest, so adding them to the
# fileset separately would double-define the packages.
# The full dependency closure. `arch_defines.v` is included BY arch_package.sv and is device-INdependent
# glue that lives only in the IP's model directory, not in the example design's imports -- so it is taken
# from the Vivado install below. Everything else is part-specific and must come from the example design.
set _files {ddr4_sdram_model_wrapper.sv arch_package.sv proj_package.sv interface.sv ddr4_model.sv
           MemoryArray.sv timing_tasks.sv StateTable.sv StateTableCore.sv}
file delete -force $OUT
file mkdir $OUT
set _n 0
foreach _d [lsort -unique [glob -nocomplain -type d $EX/*/*/imports $EX/*/imports $EX/*/*/*/imports]] {
  foreach _f [glob -nocomplain $_d/*.sv $_d/*.svh $_d/*.vh] {
    set _t [file tail $_f]
    # Harvest ONLY the device model and what its wrapper includes. The example design also ships a
    # traffic generator, an AXI wrapper and its own top -- compiling those would pull in a second MIG.
    if {[lsearch -exact $_files $_t] < 0} { continue }
    if {![file exists $OUT/$_t]} { file copy -force $_f $OUT/$_t; incr _n }
  }
}
# the generated sim_tb_top for THIS part is kept alongside, as the provenance record for
# sim/ddr4_mem_c0.sv (which is transcribed from it) -- not compiled.
foreach _f [lsort -unique [glob -nocomplain $EX/*/*/imports/sim_tb_top.sv $EX/*/imports/sim_tb_top.sv]] {
  file copy -force $_f $OUT/REFERENCE_sim_tb_top.sv.txt
}
close_project
file delete -force $SCRATCH

foreach _t $_files {
  if {![file exists $OUT/$_t]} { error "the example design did not provide $_t -- the model is incomplete" }
}
# arch_defines.v: included by arch_package.sv, present only in the IP's own model directory.
set _ad ""
foreach _c [list [glob -nocomplain $EX/*/*/imports/arch_defines.v] \
                 [glob -nocomplain $::env(XILINX_VIVADO)/data/ip/xilinx/ddr4_v2_2/data/dlib/*/ddr4_sdram/tb/ddr4_model/arch_defines.v]] {
  if {[llength $_c]} { set _ad [lindex $_c 0]; break }
}
if {$_ad eq ""} { error "arch_defines.v not found -- arch_package.sv includes it and will not compile" }
file copy -force $_ad $OUT/arch_defines.v
incr _n
puts "\[g4-model\] arch_defines.v taken from $_ad"
puts "\[g4-model\] copied $_n model file(s) into $OUT (only ddr4_sdram_model_wrapper.sv is compiled; it includes the rest):"
foreach _f [lsort [glob -nocomplain $OUT/*]] { puts "        [file tail $_f]" }
if {$_n == 0} { error "no model files were harvested -- inspect $EX by hand" }
# r27-#3: the wrapper's `define`s ARE the device identity in the model source. Assert them, do not just
# print them -- the Vivado template's are `DDR4_16G_X8` / `DDR4_938_Timing`, i.e. a different part, and a
# model built from those would answer every read plausibly and wrongly.
set _w $OUT/ddr4_sdram_model_wrapper.sv
if {![file exists $_w]} { error "no ddr4_sdram_model_wrapper.sv in $OUT" }
set _fh [open $_w r]; set _wt [read $_fh]; close $_fh
set _macros {}
foreach {_all _m} [regexp -all -inline {`define\s+(\S+)} $_wt] { lappend _macros $_m }
puts "\[g4-model\] wrapper defines: $_macros"
# captured above, before close_project -- these queries need the scratch project open
set _dw $_id_width
set _cw $_id_part
# r28-#4: "any DDR4_*_X* and any *_Timing" would accept the Vivado template's DDR4_16G_X8 /
# DDR4_938_Timing -- i.e. exactly the wrong device this check exists to catch. Derive the EXACT macros
# from the configured part and require them.
#   MT40A1G8WE-075E : 8 Gbit, x8, tCK 750 ps (DDR4-2666)
set _dens ""
if {[regexp {MT40A(\d+)G(\d+)} $_cw -> _gbit _dqbits]} {
  # the density macro counts the DEVICE size in Gbit: 1G x8 = 8 Gbit
  set _dens "DDR4_[expr {$_gbit * $_dqbits}]G_X$_dqbits"
}
set _tck $_id_tck
set _timing "DDR4_${_tck}_Timing"
set _expect [list $_dens $_timing]
foreach _e $_expect {
  if {$_e eq "" || [lsearch -exact $_macros $_e] < 0} {
    error "the generated wrapper does not declare the macro this part requires.\
           \n  expected (from $_cw, tCK $_tck): $_expect\
           \n  found:                           $_macros\
           \nA model built from the wrong macros answers every read plausibly and WRONGLY."
  }
}
# and the speed-bin macro must be present at all (FIXED_2666 for this part)
if {[lsearch -glob $_macros FIXED_*] < 0} {
  error "the generated wrapper declares no FIXED_<speed> macro (got: $_macros) -- the model would not be
         pinned to this part's speed bin."
}
puts "\[g4-model\] device identity ASSERTED for $_cw (DQ $_dw): macros $_expect present in $_macros"
