# ---- Package the SpinalHDL top as a user IP -------------------------------------------------------
# Vivado infers the AXI / AXI-Stream / clock bus interfaces from the X_INTERFACE_INFO attributes the
# `vivado=true` RTL carries; here we only have to (a) stamp the clock FREQ_HZ and reset POLARITY bus
# parameters and (b) bind each bus interface to its clock — S_AXIS to the 100 MHz hostClk, every
# DAC{i}_AXIS / ADC{i}_AXIS to the 500 MHz dspClk. Ported from the RISC-Q `plip.tcl`.

# OOC synthesis clocks for the packaged IP. Vivado gives a packaged user IP NO clocks in its
# out-of-context child synth run, so that run maps UNTIMED — measured on the 14q SoC: the DSP
# mapper then leaves the ComplexMul MREG stage empty (108 DSP48s at DRC DPOP-4, mult→ALU→P
# combinational at 500 MHz) and the pg-cordic cone collapses the whole build; the identical
# netlist synthesized WITH these clocks maps MREG correctly (DPOP-4 = 0). The XDC must be a
# project source BEFORE ipx::package_project -import_files, or the packager drops it
# (IP_Flow 19-5109); the `out_of_context` USED_IN tag scopes it to the OOC child run only —
# in-context, the BD's real clocks rule.
set fh [open $SOURCE_PATH/PulseTableSoc_ooc.xdc w]
puts $fh "create_clock -name dspClk -period [format %.3f [expr {1e9 / $DSP_FREQ}]] \[get_ports dspClk\]"
puts $fh "create_clock -name hostClk -period [format %.3f [expr {1e9 / $HOST_FREQ}]] \[get_ports hostClk\]"
# r11-#11: without this the uplink's whole clock domain is UNTIMED during out-of-context synthesis of
# the packaged IP -- every path inside it would be reported as met regardless of what it costs.
if {$DDR_READOUT} {
  puts $fh "create_clock -name ddrClk -period [format %.3f [expr {1e9 / $DDR_FREQ}]] \[get_ports ddrClk\]"
}
close $fh
add_files -fileset constrs_1 $SOURCE_PATH/PulseTableSoc_ooc.xdc
set_property USED_IN {synthesis implementation out_of_context} [get_files $SOURCE_PATH/PulseTableSoc_ooc.xdc]

ipx::package_project -root_dir $IP_REPO -vendor user.org -library user -taxonomy /UserIP \
  -import_files -set_current false -force -quiet
ipx::open_ipxact_file $IP_REPO/component.xml

# ensure the packaged copy keeps the OOC scoping, then drop the project-side entry (the IP holds
# its own imported copy; the outer BD project must not carry an IP-port create_clock).
foreach _g [ipx::get_file_groups -of_objects [ipx::current_core]] {
  foreach _f [ipx::get_files -of_objects $_g "*PulseTableSoc_ooc.xdc"] {
    set_property USED_IN {synthesis implementation out_of_context} $_f
  }
}
remove_files [get_files $SOURCE_PATH/PulseTableSoc_ooc.xdc]

# clock frequencies
ipx::add_bus_parameter FREQ_HZ [ipx::get_bus_interfaces hostClk -of_objects [ipx::current_core]]
set_property value $HOST_FREQ [ipx::get_bus_parameters FREQ_HZ \
  -of_objects [ipx::get_bus_interfaces hostClk -of_objects [ipx::current_core]]]
ipx::add_bus_parameter FREQ_HZ [ipx::get_bus_interfaces dspClk -of_objects [ipx::current_core]]
set_property value $DSP_FREQ [ipx::get_bus_parameters FREQ_HZ \
  -of_objects [ipx::get_bus_interfaces dspClk -of_objects [ipx::current_core]]]

# bus<->clock associations
ipx::associate_bus_interfaces -busif S_AXIS -clock hostClk [ipx::current_core]
ipx::associate_bus_interfaces -busif S_AXIS -clock dspClk -remove [ipx::current_core]
for {set i 0} {$i < 16} {incr i} {
  ipx::associate_bus_interfaces -busif DAC${i}_AXIS -clock dspClk [ipx::current_core]
  ipx::associate_bus_interfaces -busif DAC${i}_AXIS -clock hostClk -remove [ipx::current_core]
  ipx::associate_bus_interfaces -busif ADC${i}_AXIS -clock dspClk [ipx::current_core]
  ipx::associate_bus_interfaces -busif ADC${i}_AXIS -clock hostClk -remove [ipx::current_core]
}

# qubic3 uplink buses: all three live in the ddrClk domain.
if {$DDR_READOUT} {
  ipx::add_bus_parameter FREQ_HZ [ipx::get_bus_interfaces ddrClk -of_objects [ipx::current_core]]
  set_property value $DDR_FREQ [ipx::get_bus_parameters FREQ_HZ \
    -of_objects [ipx::get_bus_interfaces ddrClk -of_objects [ipx::current_core]]]
  foreach b {M_AXI_DDR S_AXI_DDR_CTRL M_AXIS_RD} {
    ipx::associate_bus_interfaces -busif $b -clock ddrClk [ipx::current_core]
    foreach other {hostClk dspClk} {
      catch { ipx::associate_bus_interfaces -busif $b -clock $other -remove [ipx::current_core] }
    }
  }
  ipx::add_bus_parameter POLARITY [ipx::get_bus_interfaces ddrRst -of_objects [ipx::current_core]]
  set_property value ACTIVE_HIGH [ipx::get_bus_parameters POLARITY \
    -of_objects [ipx::get_bus_interfaces ddrRst -of_objects [ipx::current_core]]]
}

# reset polarities
ipx::add_bus_parameter POLARITY [ipx::get_bus_interfaces hostRst -of_objects [ipx::current_core]]
set_property value ACTIVE_HIGH [ipx::get_bus_parameters POLARITY \
  -of_objects [ipx::get_bus_interfaces hostRst -of_objects [ipx::current_core]]]
ipx::add_bus_parameter POLARITY [ipx::get_bus_interfaces dspRst -of_objects [ipx::current_core]]
set_property value ACTIVE_HIGH [ipx::get_bus_parameters POLARITY \
  -of_objects [ipx::get_bus_interfaces dspRst -of_objects [ipx::current_core]]]

# ---- memory-initialisation data (Codex r23 / r24 / r25 / r26) ------------------------------------------------
# `PulseTableSoc.v` contains `$readmemb "<name>.bin"` for three memories. `ipx::package_project
# -import_files` imports the .v but NOT those data files, so the SoC's out-of-context synthesis emitted
#   CRITICAL WARNING [Synth 8-4445] could not open $readmem data file '...bin' ... ignoring
# three times in EVERY build so far -- the hardware image differed from simulation and nothing said so
# out loud. Import them into the same file groups as the RTL.
#
# r24-#6: the recorded IP-XACT path must be the path the file is actually AT (`src/<name>.bin`, beside
# the imported PulseTableSoc.v), not a bare name at the component root.
# r24-#7: and none of this may fail quietly -- the checks below require that the expected files existed,
# that a synthesis group was found, and that every file is registered AND present on disk afterwards.
# r27-#4: discover RECURSIVELY, so a reference below a subdirectory is actually found rather than just
# failing the set comparison. (Today every reference is a bare name, but the comparison below claims to
# support subdirectories, so the discovery has to as well.)
set _bins [lsort [concat [glob -nocomplain $SOURCE_PATH/*.bin] \
                         [glob -nocomplain -directory $SOURCE_PATH */*.bin] \
                         [glob -nocomplain -directory $SOURCE_PATH */*/*.bin]]]
set _core_dir [file dirname [get_property XML_FILE_NAME [ipx::current_core]]]
set _added 0
set _groups {}
foreach _g [ipx::get_file_groups -of_objects [ipx::current_core]] {
  set _gn [get_property NAME $_g]
  if {![string match "*synthesis*" $_gn] && ![string match "*simulation*" $_gn]} { continue }
  lappend _groups $_gn
  foreach _b $_bins {
    # keep the packaged layout identical to how the RTL refers to the file (r26-#7)
    set _rel [string range [file normalize $_b] [expr {[string length [file normalize $SOURCE_PATH]] + 1}] end]
    set _rel [file join {*}[file split $_rel]]
    file mkdir [file dirname $_core_dir/src/$_rel]
    file copy -force $_b $_core_dir/src/$_rel
    if {[llength [ipx::get_files -quiet -of_objects $_g "src/$_rel"]] > 0} { continue }
    set _f [ipx::add_file src/$_rel $_g]
    set_property type unknown $_f
    incr _added
  }
}
# r25-#6: "non-empty" is not the requirement -- the requirement is that the packaged set MATCHES the
# files the RTL actually references. One missing file, or an unrelated stale .bin left in the directory,
# would otherwise pass every check while a referenced image stays absent. Parse the references out of
# PulseTableSoc.v and compare the two sets exactly.
set _fh [open $SOURCE_PATH/$TOP_MODULE.v r]
set _rtl [read $_fh]
close $_fh
# `regexp -all -inline` with one capture group returns {fullMatch capture fullMatch capture ...},
# so take every second element rather than relying on a pattern filter.
set _hits [regexp -all -inline {\$readmem[bh]\s*\(\s*"([^"]+)"} $_rtl]
# `regexp -all -inline` with one capture group returns {fullMatch capture fullMatch capture ...}, so
# take every second element rather than relying on a pattern filter.
# r26-#7: keep each reference as the RTL WRITES it (separators normalised only). Reducing it with
# `file tail` would let a future `subdir/x.bin` falsely match `SOURCE_PATH/x.bin` while the packaged
# path would not satisfy the RTL literal.
set _refs {}
for {set _i 1} {$_i < [llength $_hits]} {incr _i 2} {
  lappend _refs [file join {*}[file split [lindex $_hits $_i]]]
}
set _refs [lsort -unique $_refs]
# The available set, as paths RELATIVE TO SOURCE_PATH (same shape as the references above).
# r26-#6: no `lmap` -- it is Tcl 8.6 only, and a Vivado embedding 8.5 would hard-fail here before
# ever reaching the comparison this check exists for.
set _srcnorm [file normalize $SOURCE_PATH]
set _have {}
foreach _b $_bins {
  set _rel [string range [file normalize $_b] [expr {[string length $_srcnorm] + 1}] end]
  lappend _have [file join {*}[file split $_rel]]
}
set _have [lsort $_have]
if {[llength $_refs] == 0} {
  error "no \$readmem reference found in $TOP_MODULE.v -- the parser in inc/package-ip.tcl no longer\
         matches the generated RTL, so the memory-init check would be vacuous."
}
if {$_refs ne $_have} {
  error "the packaged memory-init set does not match what $TOP_MODULE.v references.\
         RTL references: $_refs\
         found in $SOURCE_PATH: $_have\
         A missing file ships an uninitialised memory; an extra one means a stale artefact."
}
if {![llength [lsearch -all -inline $_groups "*synthesis*"]]} {
  error "no synthesis file group in the packaged core (found: $_groups) -- the .bin files would not\
         reach out-of-context synthesis."
}
foreach _rel $_have {
  if {![file exists $_core_dir/src/$_rel]} { error "memory-init file $_rel was not copied into the IP" }
  set _seen 0
  foreach _g [ipx::get_file_groups -of_objects [ipx::current_core]] {
    if {[string match "*synthesis*" [get_property NAME $_g]] &&
        [llength [ipx::get_files -quiet -of_objects $_g "src/$_rel"]] > 0} { set _seen 1 }
  }
  if {!$_seen} { error "memory-init file $_rel is not registered in the IP's synthesis file group" }
}
puts "\[package-ip\] memory-init: [llength $_refs] file(s) referenced by $TOP_MODULE.v, all matched and\
      packaged ($_added registration(s) across {$_groups}); present under $_core_dir/src and in the\
      synthesis group. A fresh OOC synthesis must now show ZERO `Synth 8-4445` warnings."

ipx::merge_project_changes ports [ipx::current_core]
ipx::create_xgui_files [ipx::current_core]
ipx::update_checksums [ipx::current_core]
ipx::check_integrity [ipx::current_core]
ipx::save_core [ipx::current_core]
set_property ip_repo_paths $IP_REPO [current_project]
update_ip_catalog

# ClockInterface is a plain BD module reference (instantiated as `clkifc`), not part of the user IP —
# add it now, after packaging, so it does not get swept into the IP archive.
add_files $SOURCE_PATH/ClockInterface.v
update_compile_order -fileset sources_1
