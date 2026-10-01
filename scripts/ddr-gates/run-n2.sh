#!/bin/bash
# N2 of P3c-3: PipeTraceSim with the timing pipeline off (the reference) and on; the two traces must be identical.
#   run-n2.sh <out-dir> [label]   -> <out-dir>/trace_{0,1}.txt, sim_{0,1}.log, result.txt (ends "N2 <label>: PASS|FAIL")
# Exit 0 only if both runs succeed and both traces are complete and byte-identical; 1 otherwise; 2 if a prerequisite is
# missing. A run succeeds when the simulator exits 0, prints the success line of the variant asked for, and logs no
# failure line: an HDL ERROR or FAILURE message (an ERROR-severity assertion only prints), a SpinalSim "[Error]", an
# exception, or a failed mill task. A trace is complete when its one END marker is its last line. Earlier outputs in
# <out-dir> are removed first, so a run never judges a stale trace. (P3c-3a after-stage review #6: the runner before
# this one ignored the simulator's exit status and passed two traces that both lacked END.)
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
O=$1; LABEL=${2:-$(basename "${1:-n2}")}
[ -n "$O" ] || gate_die "usage: run-n2.sh <out-dir> [label]"
gate_need mill-1.1.0 sha256sum cmp
mkdir -p "$O" && O=$(cd "$O" && pwd) || gate_die "cannot create $1"
R=$O/result.txt
rm -f "$O"/trace_0.txt "$O"/trace_1.txt "$O"/sim_0.log "$O"/sim_1.log; : > "$R"
echo "repo $GATE_REPO HEAD $(git -C "$GATE_REPO" rev-parse --short HEAD 2>/dev/null)" \
  "dirty=$(git -C "$GATE_REPO" status --porcelain --untracked-files=no 2>/dev/null | wc -l) $(date +%F_%T)" | tee -a "$R"
FAILRE='^(ERROR|FAILURE) |\[[Ee]rror\]|Exception|SimFailure|Simulation failed|[0-9]+ FAILED\]'   # not the out-dir path
fail=0
bad() { echo "FAIL $*" | tee -a "$R"; fail=1; }
cd "$GATE_REPO" || gate_die "no repo at $GATE_REPO"
for v in 0 1; do
  T=$O/trace_$v.txt; L=$O/sim_$v.log; want=$([ $v = 1 ] && echo true || echo false); t0=$(date +%s)
  RISCQ_TIMING_PIPE=$v gate_run mill-1.1.0 --no-server runMain riscq.soc.sim.PipeTraceSim "$T" > "$L" 2>&1; rc=$?
  echo "variant $v rc=$rc $(( $(date +%s)-t0 ))s $([ -f "$T" ] && grep -c . "$T" || echo no) lines" | tee -a "$R"
  [ $rc -eq 0 ] || bad "variant $v: the simulator exited $rc"
  grep -qF "[PipeTraceSim] timingPipe=$want: trace written to $T (" "$L" || bad "variant $v: no success line for timingPipe=$want"
  e=$(grep -c -E "$FAILRE" "$L")
  [ "$e" -eq 0 ] || bad "variant $v: $e failure lines in the log, first: $(grep -m1 -E "$FAILRE" "$L" | cut -c1-150)"
  if [ -f "$T" ]; then
    n=$(grep -c "^END " "$T"); last=$(tail -n 1 "$T")
    [ "$n" -eq 1 ] && [[ $last =~ ^END\ dsp=[0-9]+\ ddr=[0-9]+\ host=[0-9]+$ ]] \
      || bad "variant $v: $n END markers, last line \"$(echo "$last" | cut -c1-60)\" (needs one END, as the last line)"
  else
    bad "variant $v: no trace written"
  fi
done
# the streams, for the record; the verdict compares the whole files
for tag in D U H T END; do
  a=$(grep "^$tag " "$O/trace_0.txt" 2>/dev/null | sha256sum | cut -c1-16); b=$(grep "^$tag " "$O/trace_1.txt" 2>/dev/null | sha256sum | cut -c1-16)
  n=$(grep -c "^$tag " "$O/trace_0.txt" 2>/dev/null)
  if [ "$a" = "$b" ]; then echo "stream $tag: ${n:-0} lines, identical ($a)" | tee -a "$R"
  else echo "stream $tag: DIFFER ($a vs $b); first difference:" | tee -a "$R"
       diff <(grep "^$tag " "$O/trace_0.txt" 2>/dev/null) <(grep "^$tag " "$O/trace_1.txt" 2>/dev/null) | head -6 | tee -a "$R"; fi
done
[ -f "$O/trace_0.txt" ] && [ -f "$O/trace_1.txt" ] && cmp -s "$O/trace_0.txt" "$O/trace_1.txt" || bad "the two traces are not byte-identical"
echo "N2 $LABEL: $([ $fail = 0 ] && echo PASS || echo FAIL)" | tee -a "$R"
exit $fail
