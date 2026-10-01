package riscq.misc

/**
 * Sim-only knob for the P3c-3 antq timing pipeline (evidence/P3c/PLAN_P3c3_pipelining_v2.md): with
 * `RISCQ_TIMING_PIPE=1` a sim builds the timing-pipeline variant of the block it tests, so the modified RTL is
 * what runs. RTL generators never read it: generation takes the switch from the spec
 * (`PulseTableSoc.timingPipe`, on exactly for `results_path = antq_uplink`).
 */
object TimingPipeKnob {
  def enabled: Boolean = sys.env.get("RISCQ_TIMING_PIPE").exists(v => v == "1" || v.equalsIgnoreCase("true"))
}
