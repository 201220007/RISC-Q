package riscq.ddr.sim

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import riscq.ddr.UplinkResultFifo
import scala.util.Random

/**
 * P3c-2: UplinkResultFifo against StreamFifo in lockstep. Both get the same push valid/payload and pop ready; every
 * cycle, push.ready, pop.valid and (while valid) pop.payload must be equal. Bursty random traffic (phases of mostly
 * pushing, mostly popping, and both) drives each FIFO through full, empty and every occupancy in between.
 * Run: mill-1.1.0 runMain riscq.ddr.sim.UplinkResultFifoEquivSim
 */
object UplinkResultFifoEquivSim extends App {
  case class Pair(depth: Int) extends Component {
    val io = new Bundle {
      val pushValid = in Bool(); val pushPayload = in Bits(16 bits); val popReady = in Bool()
      val aPushReady, bPushReady, aPopValid, bPopValid = out Bool()
      val aPop, bPop = out Bits(16 bits)
    }
    val a = StreamFifo(Bits(16 bits), depth)
    val b = UplinkResultFifo(16, depth)
    for (f <- Seq(a.io.push, b.io.push)) { f.valid := io.pushValid; f.payload := io.pushPayload }
    a.io.pop.ready := io.popReady; b.io.pop.ready := io.popReady
    io.aPushReady := a.io.push.ready; io.bPushReady := b.io.push.ready
    io.aPopValid := a.io.pop.valid; io.bPopValid := b.io.pop.valid
    io.aPop := a.io.pop.payload; io.bPop := b.io.pop.payload
  }
  var fails = 0
  for ((depth, seed) <- Seq((16, 1), (16, 2), (8, 3), (4, 4), (2, 5))) {
    SimConfig.withConfig(SpinalConfig(defaultClockDomainFrequency = FixedFrequency(500 MHz))).compile(Pair(depth)).doSim(s"d$depth", seed) { dut =>
      val rng = new Random(seed)
      dut.clockDomain.forkStimulus(2)
      dut.io.pushValid #= false; dut.io.popReady #= false; dut.io.pushPayload #= 0
      dut.clockDomain.waitSampling(5)
      var pPush = 0.5; var pPop = 0.5; var full = 0; var empty = 0; var pushes = 0; var pops = 0
      for (cyc <- 0 until 100000) {
        if (cyc % 500 == 0) { val m = rng.nextInt(4); pPush = Seq(0.9, 0.1, 0.5, 0.97)(m); pPop = Seq(0.1, 0.9, 0.5, 0.97)(m) }
        dut.io.pushValid #= rng.nextDouble() < pPush
        dut.io.pushPayload #= rng.nextInt(1 << 16)
        dut.io.popReady #= rng.nextDouble() < pPop
        dut.clockDomain.waitSampling()
        val ar = dut.io.aPushReady.toBoolean; val br = dut.io.bPushReady.toBoolean
        val av = dut.io.aPopValid.toBoolean; val bv = dut.io.bPopValid.toBoolean
        if (ar != br || av != bv || (av && dut.io.aPop.toInt != dut.io.bPop.toInt)) {
          fails += 1
          if (fails < 10) println(s"[FIFOEQ] depth $depth cycle $cyc MISMATCH ready $ar/$br valid $av/$bv payload ${dut.io.aPop.toInt}/${dut.io.bPop.toInt}")
        }
        if (!ar) full += 1; if (!av) empty += 1
        if (dut.io.pushValid.toBoolean && ar) pushes += 1
        if (dut.io.popReady.toBoolean && av) pops += 1
      }
      println(s"[FIFOEQ] depth $depth seed $seed: 100000 cycles, $pushes pushes, $pops pops, full $full cycles, empty $empty cycles")
      assert(full > 100 && empty > 100, s"depth $depth: the traffic did not reach full ($full) and empty ($empty) often enough")
    }
  }
  assert(fails == 0, s"$fails mismatching cycles")
  println("[FIFOEQ] all PASS: UplinkResultFifo == StreamFifo at the ports, cycle for cycle")
}
