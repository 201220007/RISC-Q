# The G4 transcript's model-violation / error verdict (P3b r1; the waiver restricted in P3c). Pure Tcl, so
# it is testable on saved transcripts: sim/test/g4-transcript-verdict-test.sh. Sourced by run-sim-bd.tcl.
#
# The DDR4 device model reports protocol and timing violations (tRRD, tFAW, ...) as non-fatal "VIOLATION"
# messages, and simulator errors are non-fatal too, so both are counted and any fails the gate.
# RISCQ_G4_ALLOW_MODEL_VIOLATIONS=1 (`allow`) is the waiver for ONE case only, the board-clock probe: with
# RISCQ_G4_SYSCLK_3333=1 (`probe`) the MIG, configured for 3.334 ns, spaces ACTs 1-2 ps under the model's
# exact tRRD_S / tFAW minimums (P3b REPORT r1.4, accepted by the orchestrator). `allow` without `probe` is
# refused, and even with it every violation must be a tRRD_S or tFAW shortfall of 1 or 2 ps.
#
# Returns {PASS|FAIL msg...}; the last message of a FAIL is the reason.
proc g4_transcript_verdict {txt allow probe} {
  set viol {}
  set vlines [regexp -all -inline -line {VIOLATION: .*\n\s*(\S+)} $txt]
  for {set i 1} {$i < [llength $vlines]} {incr i 2} { dict incr viol [lindex $vlines $i] }
  set nviol [regexp -all -line {VIOLATION:} $txt]
  set nerr  [regexp -all -line {^(ERROR|Error|FATAL|Fatal)[: ]} $txt]
  set msgs [list "\[G4\] transcript: $nviol DDR4-model VIOLATION message(s) [expr {$nviol ? "by type $viol" : ""}], $nerr simulator error line(s)"]
  if {$nerr != 0} { return [concat FAIL $msgs [list "$nerr error line(s)"]] }
  if {$allow && !$probe} {
    return [concat FAIL $msgs [list "RISCQ_G4_ALLOW_MODEL_VIOLATIONS is only the 3.333 ns probe's waiver and needs RISCQ_G4_SYSCLK_3333=1"]]
  }
  if {$nviol == 0} { return [concat PASS $msgs] }
  if {!$allow} { return [concat FAIL $msgs [list "$nviol DDR4-model VIOLATION message(s) ($viol)"]] }
  # the line after each VIOLATION names the check and the shortfall, e.g. "\ttRRD_S - 1ps | 1 clocks."
  set waived [regexp -all -line {VIOLATION: .*\n\s*(tRRD_S|tFAW) - [12]ps\M} $txt]
  if {$waived != $nviol} {
    return [concat FAIL $msgs [list "$nviol model violation(s), only $waived of them tRRD_S/tFAW shortfalls of 1-2 ps (the only waived case, at 3.333 ns): ($viol)"]]
  }
  return [concat PASS $msgs [list "\[G4\] WAIVED: $nviol model violation(s), all tRRD_S/tFAW shortfalls of 1-2 ps at the 3.333 ns probe"]]
}
