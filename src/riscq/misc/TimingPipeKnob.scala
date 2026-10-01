package riscq.misc

/**
 * Sim-only knob for the P3c-3 antq timing pipeline (evidence/P3c/PLAN_P3c3_pipelining_v2.md): with
 * `RISCQ_TIMING_PIPE=1` a sim builds the timing-pipeline variant of the block it tests, so the modified RTL is
 * what runs. RTL generators never read it: generation takes the switch from the spec
 * (`PulseTableSoc.timingPipe`, on exactly for `results_path = antq_uplink`).
 */
object TimingPipeKnob {
  def enabled: Boolean = sys.env.get("RISCQ_TIMING_PIPE").exists(v => v == "1" || v.equalsIgnoreCase("true"))

  /** A knob-mode sim's config: flip-flops start at 0, as they do on the FPGA (INIT = 0). The antq pre-decode stage
   *  (PulseParamPreDecode) has no reset, like the posted link stages it stands for, and SpinalSim holds an async reset
   *  with the clock stopped, so without this the first edge after reset would act on Verilator's random power-up
   *  value of that stage. (In the SoC, dspClk also runs during dspRst, which flushes it.) */
  def sim(c: spinal.core.sim.SpinalSimConfig): spinal.core.sim.SpinalSimConfig =
    if (enabled) c.addSimulatorFlag("--x-initial 0") else c

  /** A core sim's param with the core flags the antq SoC sets (`PulseTableSoc.cp`) when the knob is on. */
  def core(p: riscq.riscv.RiscqParam): riscq.riscv.RiscqParam =
    if (enabled) p.copy(aluNoFastForward = true, lsuByteOffPrecompute = true, jalrComparePrecompute = true) else p

  /** The reference for the zero-cycle checks: the antq core flags without the P3c-3 ones, i.e. the antq SoC's core as
   *  it was before P3c-3 (`aluNoFastForward` and its 1-ahead interlocks). The plain param when the knob is off. */
  def coreRef(p: riscq.riscv.RiscqParam): riscq.riscv.RiscqParam =
    if (enabled) p.copy(aluNoFastForward = true) else p
}
