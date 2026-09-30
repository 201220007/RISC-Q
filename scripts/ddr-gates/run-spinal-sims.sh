#!/bin/bash
# SpinalSim Apps in parallel, each in its own work dir under build/gate-sims/<name>.
#   run-spinal-sims.sh <tag> <list-file> [parallel]   list lines: "<name> <fully.qualified.Main> [args...]"
#   -> $GATE_LOG_DIR/sims_<tag>/<name>.log, results.tsv
# Exit 1 if any sim exits non-zero (P3b re-audit item 2); 2 if a prerequisite is missing.
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
TAG=$1; LIST=$(realpath "$2" 2>/dev/null); PAR=${3:-4}
[ -n "$TAG" ] && [ -f "$LIST" ] || gate_die "usage: run-spinal-sims.sh <tag> <list-file> [parallel]"
gate_need mill-1.1.0 java python3 xargs
LD=$GATE_LOG_DIR/sims_$TAG; mkdir -p "$LD" "$GATE_REPO/build/gate-sims"; : > "$LD/results.tsv"
CP=$(cd "$GATE_REPO" && mill-1.1.0 --no-server show runClasspath 2>/dev/null | python3 -c "import json,sys; print(':'.join(p.split(':',3)[-1] if p.startswith('ref:') or p.startswith('qref:') else p for p in json.load(sys.stdin)))")
[ -n "$CP" ] || gate_die "could not get the runClasspath from mill (does the tree compile?)"
export CP LD GATE_REPO GATE_NICE
one() {
  set -- $1; local name=$1 main=$2; shift 2
  local d=$GATE_REPO/build/gate-sims/$name; mkdir -p "$d"; ln -sfn "$GATE_REPO/src" "$d/src"; ln -sfn "$GATE_REPO/software" "$d/software"; cd "$d"
  local t0=$(date +%s)
  nice -n "$GATE_NICE" java -XX:ActiveProcessorCount=2 -cp "$CP" $main "$@" > "$LD/$name.log" 2>&1; local rc=$?
  echo -e "$name\trc=$rc\twall=$(( $(date +%s)-t0 ))s\t$(date +%T)" >> "$LD/results.tsv"
  return $rc
}
export -f one
${GATE_TASKSET:-} xargs -a "$LIST" -P "$PAR" -d "\n" -I{} bash -c "one \"{}\""; rc=$?
echo "ALLDONE xargs_rc=$rc $(date +%T)" >> "$LD/results.tsv"
cat "$LD/results.tsv"
[ $rc -eq 0 ]
