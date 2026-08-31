package riscq.soc

import spinal.core._
import riscq.riscv.RiscqParam

/**
 * Generate `PulseTableSoc.v` from a per-build `SocParams` JSON (under `software/configs/`) — the file
 * the python `riscq.map.SocParams` also loads, so hardware and software derive their address maps from
 * one parameter set.
 *
 * Defaults to the plain-port (`vivado = false`) form the co-sim / OOC flows use. Pass `vivado` as a third
 * arg for the IP-packager / block-design form (the old `GenPulseTableSocVivado`): the
 * `X_INTERFACE_INFO`/`FREQ_HZ` attrs, the `hostClk`/`hostRst` rename, `LutInputs = 6`, plus the companion
 * `ClockInterface.v` clock-buffer wrapper.
 *
 *   mill runMain riscq.soc.GenPulseTableSocJson software/configs/sim-2q.json software/build/sim-2q/rtl
 *   mill runMain riscq.soc.GenPulseTableSocJson software/configs/zcu216-14q.json build/rtl vivado
 */
object GenPulseTableSocJson extends App {
  require(args.length == 2 || args.length == 3,
    "usage: GenPulseTableSocJson <config.json> <targetDir> [vivado]")
  val cfg = ujson.read(scala.io.Source.fromFile(args(0)).mkString)
  val dir = args(1)
  val vivadoMode = args.lift(2).contains("vivado")

  def int(key: String): Int = cfg(key).num.toInt
  def intOr(key: String, default: Int): Int = cfg.obj.get(key).map(_.num.toInt).getOrElse(default)
  def boolOr(key: String, default: Boolean): Boolean = cfg.obj.get(key).map(_.bool).getOrElse(default)

  // Fixed by architecture — the JSON may state them, but they are asserted, never plumbed.
  val fixed = Map("batch_size" -> 16, "adc_batch" -> 4, "data_width" -> 16,
    "readout_max_win_log2" -> 14, "readout_acc_width" -> 32)
  for ((key, value) <- fixed if cfg.obj.contains(key))
    require(int(key) == value, s"$key is fixed by architecture at $value, got ${int(key)}")

  val qubitNum = int("qubit_num")

  // Optional per-build converter maps (else the generic ZCU216 `SocChannelMap` layout):
  //   "dac_map": one list per core of physical DAC ids per channel — [gateDac, readoutDriveDac]
  //   "adc_map": one physical ADC id per core (feeds that core's demod)
  // Software ignores these (it addresses every converter by dac_num/adc_num); they are hardware-only.
  val dacMap: Map[(Int, Int), Int] = cfg.obj.get("dac_map") match {
    case Some(m) =>
      require(m.arr.length == qubitNum, s"dac_map needs $qubitNum entries, got ${m.arr.length}")
      m.arr.zipWithIndex.flatMap { case (chans, c) =>
        chans.arr.zipWithIndex.map { case (dac, ch) => (c, ch) -> dac.num.toInt }
      }.toMap
    case None => SocChannelMap.dacMap(qubitNum)
  }
  val adcMap: Map[Int, Int] = cfg.obj.get("adc_map") match {
    case Some(m) =>
      require(m.arr.length == qubitNum, s"adc_map needs $qubitNum entries, got ${m.arr.length}")
      m.arr.zipWithIndex.map { case (adc, c) => c -> adc.num.toInt }.toMap
    case None => SocChannelMap.adcMap(qubitNum)
  }

  // `def` (not `val`) so the Component is constructed inside `.generate` — building it at the App's top
  // level throws `GlobalData ... null` (no elaboration context yet).
  def buildSoc() = PulseTableSoc(
    qubitNum      = qubitNum,
    dacMap        = dacMap,
    adcMap        = adcMap,
    dacNum        = int("dac_num"),
    adcNum        = int("adc_num"),
    readoutInterp = int("readout_interp"),
    gateInterp    = int("gate_interp"),
    demodInterp   = intOr("demod_interp", 4),
    linkPipe      = int("link_pipe"),
    memDepth      = int("mem_depth"),
    envDepth      = int("env_depth"),
    robDepth      = intOr("rob_depth", 1024),
    gatePulseNum  = int("gate_pulse_num"),
    // per-parameter TimedQueue depth (scheduled-ahead pulses per channel param); default 4. Hardware-only —
    // software addresses channels by slot count, not queue depth, so `riscq.map.SocParams` ignores it.
    queueDepth    = intOr("queue_depth", 4),
    // The bit-exact residual-depth levers (B3/B4/E1–E3) and the fanout caps (B2 jump / E4 fetch /
    // dsp-fmax B2 dcOffset) plus B1-alt/B3-leanpop are all baked into the RTL now — no longer knobs;
    // the C1 registered head is a TimedQueue-level option, no longer plumbed through the SoC.
    coreParam     = RiscqParam(gshareMem = true, csrWarl = true,
      aluNoFastForward = true, aluResultOneHot = true, pcRegMaxFanout = 16,
      withMul = boolOr("with_mul", true)),
    // specs/dsp-fmax.md converter-edge lever (C2), default off — a confirmation build turns it on in
    // its JSON so the lever state is part of the recorded build geometry (soc-fmax R5).
    adcPipe           = intOr("adc_pipe", 3),
    // qubic3 readout->DDR uplink. Absent/false => not one signal is elaborated and the emitted RTL is
    // byte-identical to the baseline (gated by evidence/G0's hash).
    ddrReadout    = boolOr("ddr_readout", false),
    vivado        = vivadoMode)

  // vivado mode: LUT6 packing + the companion ClockInterface.v BUFG wrapper, matching GenPulseTableSocVivado.
  val spinal = SpinalConfig(mode = Verilog, targetDirectory = dir, romReuse = true)
  val emit   = if (vivadoMode) spinal.setScopeProperty(LutInputs, 6) else spinal
  emit.generate(buildSoc())
  if (vivadoMode) emit.generate(riscq.misc.ClockInterface())

  val extra = if (vivadoMode) " + ClockInterface.v" else ""
  println(s"[GenPulseTableSocJson] emitted $dir/PulseTableSoc.v$extra " +
    s"(${cfg("name").str}, qubitNum=$qubitNum, vivado=$vivadoMode)")
}
