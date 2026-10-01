package riscq.soc.sim

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import riscq.soc.link.Put
import riscq.soc.rf.{PulseParamBuffer, PulseParamBufferParams}
import scala.collection.mutable.ArrayBuffer
import scala.util.Random

/**
 * N3 (qubic3 P3c-3): lockstep equivalence of the antq posted register file against today's [[PulseParamBuffer]].
 *
 * The reference is built the way the shell builds it today: the link's last `getPipe` stage (a RegNext of the
 * whole Flow, no reset) in front of a `preDecode = false` buffer. The candidate gets the stage before it and holds
 * the last stage itself (`preDecode = true`, the address decode in front of that register). Both see one Put
 * stream:
 *   - a directed prefix: every ordered pair of {fire, set_start, phaseOffset, freq, dcOffset, table write} at beat
 *     offsets 0..3 (offset 0 = adjacent cycles);
 *   - a random body over the whole 16-bit window (mapped and unmapped addresses), mostly adjacent beats;
 *   - random address and data on the idle cycles too, so a decode that ignores `valid` would show.
 * Every output (all five parameter Flows, valid and payload, `time`, `startTime`, `dcOffset`, `phaseOffset`)
 * must be equal on every cycle after reset, for the three buffer shapes the antq SoC builds.
 *
 * Run with `mill runMain riscq.soc.sim.ParamBufferLockstepSim` (`RISCQ_N3_BEATS` sets the random body length).
 */
object ParamBufferLockstepSim extends App {
  case class Cfg(name: String, pulseNum: Int, envAddrWidth: Int)
  val cfgs = Seq(
    Cfg("gate", pulseNum = 8, envAddrWidth = 10),   // a drive channel: 8-slot table in distributed RAM
    Cfg("ro",   pulseNum = 1, envAddrWidth = 10),   // ro and demod: a 1-slot register-file table
    Cfg("dio",  pulseNum = 8, envAddrWidth = 1))    // TimedDio's buffer
  val beats = sys.env.get("RISCQ_N3_BEATS").map(_.toInt).getOrElse(100000)
  val startTimeAddr = 0x4100

  case class Dut(c: Cfg) extends Component {
    def params(pre: Boolean) = PulseParamBufferParams(pulseNum = c.pulseNum, dataWidth = 16,
      envAddrWidth = c.envAddrWidth, durWidth = 16, timeWidth = 32, addrWidth = 16, preDecode = pre)
    val cmd       = slave port Flow(Put(16))
    val timeBcast = in port UInt(32 bits)

    val ref  = PulseParamBuffer(params(pre = false))
    val cand = PulseParamBuffer(params(pre = true))
    val stage = RegNext(cmd)             // the getPipe stage the candidate absorbs
    stage.addAttribute("DONT_TOUCH")
    ref.io.cmd << stage
    cand.io.cmd << cmd
    ref.io.timeBcast  := timeBcast
    cand.io.timeBcast := timeBcast

    def flat(b: PulseParamBuffer): Bits = Cat(
      b.io.phase.valid, b.io.phase.payload, b.io.amp.valid, b.io.amp.payload, b.io.addr.valid, b.io.addr.payload,
      b.io.dur.valid, b.io.dur.payload, b.io.freq.valid, b.io.freq.payload,
      b.io.time, b.io.startTime, b.io.dcOffset, b.io.phaseOffset)
    val refFlat = flat(ref); val candFlat = flat(cand)
    val refOut  = out port Bits(widthOf(refFlat) bits); refOut  := refFlat
    val candOut = out port Bits(widthOf(refFlat) bits); candOut := candFlat
    val same = out port Bool()
    same := refOut === candOut
    // activity, so the comparison cannot pass on an idle buffer
    val fireOut   = out port Bool();  fireOut   := ref.io.phase.valid
    val freqOut   = out port Bool();  freqOut   := ref.io.freq.valid
    val startOut  = out port UInt(32 bits); startOut  := ref.io.startTime
    val phOffOut  = out port SInt(16 bits); phOffOut  := ref.io.phaseOffset
    val dcOut     = out port SInt(16 bits); dcOut     := ref.io.dcOffset
  }

  // INIT = 0 power-up, as on the FPGA: the candidate's pre-decode stage and the reference's link stage have no reset
  // (see TimingPipeKnob.sim), and their random power-up values would differ
  for (c <- cfgs) SimConfig.addSimulatorFlag("--x-initial 0").compile(Dut(c)).doSim(s"n3_${c.name}", seed = 11) { dut =>
    val cd  = dut.clockDomain
    val rnd = new Random(0x3c3 + c.pulseNum + c.envAddrWidth)
    def r32(): Long = rnd.nextLong() & 0xFFFFFFFFL

    // one entry per cycle: (valid, address, data)
    val sched = ArrayBuffer[(Boolean, Int, Long)]()
    def idle(n: Int): Unit = for (_ <- 0 until n) sched += ((false, rnd.nextInt(1 << 16), r32()))
    def beat(a: Int, d: Long): Unit = sched += ((true, a, d))
    def tableAddr(): Int = ((rnd.nextInt(c.pulseNum) + 1) << 4) | (rnd.nextInt(4) << 2)
    def fireData(): Long = if (rnd.nextInt(8) == 0) r32() else rnd.nextInt(c.pulseNum).toLong
    val kinds: Seq[() => (Int, Long)] = Seq(
      () => (0x0, fireData()),          // fire
      () => (startTimeAddr, r32()),     // set_start
      () => (0xC, r32()),               // phaseOffset
      () => (0x4, r32()),               // freq
      () => (0x8, r32()),               // dcOffset
      () => (tableAddr(), r32()))       // one table field

    // load every table field first, so fires read non-trivial entries
    for (s <- 1 to c.pulseNum; f <- 0 until 4) { beat((s << 4) | (f << 2), r32()); idle(rnd.nextInt(2)) }
    // directed: every ordered pair at beat offsets 0..3
    for (a <- kinds; b <- kinds; g <- 0 to 3) {
      val (aa, ad) = a(); beat(aa, ad); idle(g)
      val (ba, bd) = b(); beat(ba, bd); idle(5)
    }
    // random body: mostly the mapped addresses, some anywhere in the window, mostly adjacent beats
    for (_ <- 0 until beats) {
      val (a, d) = rnd.nextInt(10) match {
        case 0     => (rnd.nextInt(1 << 16), r32())
        case 1     => (startTimeAddr + 4 * (1 + rnd.nextInt(16)), r32())   // past startTime: unmapped
        case 2     => (((c.pulseNum + 1 + rnd.nextInt(32)) << 4) | (rnd.nextInt(4) << 2), r32())  // past the table
        case _     => kinds(rnd.nextInt(kinds.length))()
      }
      beat(a, d)
      rnd.nextInt(16) match {
        case x if x < 9  => ()                               // adjacent
        case x if x < 15 => idle(1 + rnd.nextInt(3))
        case _           => idle(4 + rnd.nextInt(40))
      }
    }
    idle(16)

    dut.cmd.valid #= false; dut.cmd.payload.address #= 0; dut.cmd.payload.data #= 0
    var time = 0xFFFF0000L                                   // the local time wraps inside the run
    dut.timeBcast #= time
    cd.forkStimulus(10)
    cd.waitSampling(20)

    var fires = 0; var freqs = 0; var starts = 0; var phOffs = 0; var dcs = 0
    var lastStart = dut.startOut.toBigInt; var lastPh = dut.phOffOut.toBigInt; var lastDc = dut.dcOut.toBigInt
    for (((v, a, d), cyc) <- sched.zipWithIndex) {
      dut.cmd.valid #= v; dut.cmd.payload.address #= a; dut.cmd.payload.data #= d
      time = (time + 1) & 0xFFFFFFFFL; dut.timeBcast #= time
      cd.waitSampling()
      if (!dut.same.toBoolean)
        simFailure(s"[N3 ${c.name}] cycle $cyc: candidate ${dut.candOut.toBigInt.toString(16)} != reference " +
          s"${dut.refOut.toBigInt.toString(16)} (beat $v @0x${a.toHexString} = 0x${d.toHexString})")
      if (dut.fireOut.toBoolean) fires += 1
      if (dut.freqOut.toBoolean) freqs += 1
      val st = dut.startOut.toBigInt; if (st != lastStart) { starts += 1; lastStart = st }
      val ph = dut.phOffOut.toBigInt; if (ph != lastPh) { phOffs += 1; lastPh = ph }
      val dc = dut.dcOut.toBigInt;    if (dc != lastDc) { dcs += 1; lastDc = dc }
    }
    val minN = beats / 50
    assert(fires >= minN && freqs >= minN && starts >= minN && phOffs >= minN && dcs >= minN,
      s"[N3 ${c.name}] too little activity: fires $fires freq $freqs startTime $starts phaseOffset $phOffs dcOffset $dcs")
    println(s"[ParamBufferLockstepSim] PASS ${c.name} (pulseNum ${c.pulseNum}): ${sched.length} cycles, ${beats} random " +
      s"beats + all ordered pairs at offsets 0..3; every output equal every cycle (fires $fires, freq $freqs, " +
      s"startTime changes $starts, phaseOffset $phOffs, dcOffset $dcs)")
    simSuccess()
  }
}
