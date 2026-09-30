#!/bin/bash
# Tests of sim/g4-transcript-verdict.tcl on saved G4 transcripts.
#   g4-transcript-verdict-test.sh <clean log> <3.333 ns probe log> <3.332 ns r0 log>
# Wanted: clean PASS in every mode; probe FAIL unless allow+probe; r0 (shortfalls up to 9 ps) FAIL even with
# allow+probe; allow without probe FAIL; synthetic 3 ps shortfall / other check / error line FAIL.
D=$(cd "$(dirname "$0")/.." && pwd)
clean=$1; probe=$2; r0=$3; bad=0
tmp=$(mktemp -d)
printf 'x:VIOLATION: cmdACT @1 ns Required:\n\ttFAW - 3ps | 1 clocks.\n' > $tmp/3ps.log
printf 'x:VIOLATION: cmdRD @1 ns Required:\n\ttCCD_L - 1ps | 1 clocks.\n' > $tmp/other.log
printf 'x:VIOLATION: cmdACT @1 ns Required:\n\ttRRD_S - 1ps | 1 clocks.\nERROR: something\n' > $tmp/err.log
t() {  # t <name> <want> <log> <allow> <probe>
  got=$(tclsh <<TCL
source $D/g4-transcript-verdict.tcl
set fh [open $3]; set t [read \$fh]; close \$fh
set v [g4_transcript_verdict \$t $4 $5]
puts "[lindex \$v 0]|[lindex \$v end]"
TCL
)
  r=${got%%|*}; ok=ok; [ "$r" = "$2" ] || { ok=BAD; bad=$((bad+1)); }
  printf "%-4s %-46s allow=%s probe=%s want %-4s got %-4s %s\n" $ok "$1" $4 $5 $2 "$r" "$(echo ${got#*|} | cut -c1-110)"
}
t "clean 3.334 ns transcript"          PASS $clean 0 0
t "clean, probe define only"           PASS $clean 0 1
t "clean, allow without probe"         FAIL $clean 1 0
t "3.333 ns probe, no waiver"          FAIL $probe 0 1
t "3.333 ns probe, waiver"             PASS $probe 1 1
t "3.333 ns probe, allow without probe" FAIL $probe 1 0
t "r0 3.332 ns (1-9 ps), waiver"       FAIL $r0 1 1
t "synthetic tFAW 3 ps, waiver"        FAIL $tmp/3ps.log 1 1
t "synthetic tCCD_L 1 ps, waiver"      FAIL $tmp/other.log 1 1
t "synthetic tRRD_S 1 ps + ERROR, waiver" FAIL $tmp/err.log 1 1
rm -rf $tmp
echo "$bad wrong"; exit $bad
