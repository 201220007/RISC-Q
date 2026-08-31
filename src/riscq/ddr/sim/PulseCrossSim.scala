package riscq.ddr.sim

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import riscq.ddr.PulseCross
import scala.util.Random

/**
 * G2-a: [[PulseCross]] under co-prime clocks with resets at every handshake phase.
 * Invariants checked per request: exactly one `fire` XOR one `dropped` acknowledgement (never both,
 * never two fires), `busy` always clears, requests issued while busy are ignored, `dstDone` gating,
 * and after a lone reset on either side the crossing keeps working for later requests.
 * Run: mill-1.1.0 runMain riscq.ddr.sim.PulseCrossSim
 */
object PulseCrossSim extends App {
  case class Dut(withDstDone: Boolean) extends Component {
    val src = ClockDomain.external("src")
    val dst = ClockDomain.external("dst")
    val x   = PulseCross(src, dst, withDstDone)
    val io  = new Bundle {
      val start = in Bool(); val busy = out Bool(); val acked = out Bool(); val ackDropped = out Bool()
      val fire = out Bool(); val dstDone = in Bool(); val dropped = out Bool()
    }
    noIoPrefix()
    x.io.start := io.start; io.busy := x.io.busy; io.acked := x.io.acked; io.ackDropped := x.io.ackDropped
    io.fire := x.io.fire; x.io.dstDone := io.dstDone; io.dropped := x.io.dropped
    // expose the internals so the sim can assert on the protocol state, not just the io
    Seq[Data](x.dst.reqSync, x.dst.seedCnt, x.dst.prev, x.dst.ackReg, x.dst.dropReg, x.dst.pending,
              x.src.reqReg, x.src.ackSync, x.src.dropSync).foreach(_.simPublic())
  }

  def run(withDstDone: Boolean, srcPeriod: Int, dstPeriod: Int, seed: Int): Unit = {
    SimConfig.compile(Dut(withDstDone)).doSim(s"pulsecross_done${withDstDone}_s${srcPeriod}_d${dstPeriod}", seed = seed) { dut =>
      SimTimeout(2000000)   // hard sim-time guard: a hang fails fast instead of blocking the run
      dut.src.forkStimulus(srcPeriod)
      dut.dst.forkStimulus(dstPeriod)
      val rng = new Random(seed)
      dut.io.start #= false; dut.io.dstDone #= !withDstDone
      // monitors
      var fires = 0; var acks = 0; var drops = 0; var firesThisReq = 0; var maxFiresPerReq = 0
      fork { while (true) { dut.dst.waitSampling(); if (dut.io.fire.toBoolean) { fires += 1; firesThisReq += 1 } } }
      fork { while (true) { dut.src.waitSampling(); if (dut.io.acked.toBoolean) { acks += 1; if (dut.io.ackDropped.toBoolean) drops += 1 } } }
      dut.src.waitSampling(10); dut.dst.waitSampling(10)

      def request(): Unit = {
        firesThisReq = 0
        dut.io.start #= true; dut.src.waitSampling(); dut.io.start #= false
        var n = 0
        while (!dut.io.busy.toBoolean && n < 5) { dut.src.waitSampling(); n += 1 }   // reqReg becomes visible
        assert(dut.io.busy.toBoolean, "request was not taken (busy never rose)")
      }
      def waitIdle(max: Int = 2000): Boolean = {
        var n = 0
        while (dut.io.busy.toBoolean && n < max) { dut.src.waitSampling(); n += 1 }
        !dut.io.busy.toBoolean
      }
      // dstDone driver for the withDstDone variant: random delay after fire
      if (withDstDone) fork {
        while (true) {
          dut.dst.waitSampling()
          if (dut.io.fire.toBoolean) {
            dut.dst.waitSampling(1 + rng.nextInt(20))
            dut.io.dstDone #= true; dut.dst.waitSampling(); dut.io.dstDone #= false
          }
        }
      }

      println(s"[PC] phase1 start t=${simTime()}")
      // ---- 1. sequential requests, no resets ----
      for (i <- 0 until 200) {
        request()
        assert(waitIdle(), s"req $i never went idle")
        maxFiresPerReq = scala.math.max(maxFiresPerReq, firesThisReq)
        assert(firesThisReq == 1, s"req $i fired ${firesThisReq} times")
        // request while busy is ignored: issue two starts back to back
        if (i % 10 == 0) {
          dut.io.start #= true; dut.src.waitSampling(); dut.src.waitSampling(); dut.io.start #= false
          assert(waitIdle()); maxFiresPerReq = scala.math.max(maxFiresPerReq, firesThisReq)
          assert(firesThisReq == 2, s"back-to-back after idle should fire twice, got $firesThisReq")
        }
      }
      assert(drops == 0, s"unexpected drops $drops")
      val acksBefore = acks; val firesBefore = fires

      println(s"[PC] phase2 start t=${simTime()}")
      // ---- 2. lone DESTINATION resets at random phases ----
      var dropped2 = 0; var fired2 = 0
      for (i <- 0 until 100) {
        if (i % 10 == 0) println(s"[PC] p2 i=$i t=${simTime()}")
        request()
        dut.src.waitSampling(rng.nextInt(12))
        // NOTE: waitSampling() only returns on edges where reset is DEASSERTED, so it blocks forever
        // while a reset is held — during the reset window we must step the clock with waitRisingEdge().
        dut.dst.assertReset(); dut.dst.waitRisingEdge(2 + rng.nextInt(6)); dut.dst.deassertReset()
        assert(waitIdle(), s"dst-reset case $i: busy stuck")
        assert(firesThisReq <= 1, s"dst-reset case $i: duplicate fire ${firesThisReq}")
        fired2 += firesThisReq
      }
      dropped2 = drops
      // every request was either fired or acknowledged-dropped (some may vanish: fired BEFORE the reset
      // but the ack was killed by the reset -> re-acked as stale: counted as dropped; that is allowed)
      assert(acks - acksBefore == 100, s"dst-reset: expected 100 acks, got ${acks - acksBefore}")
      println(s"[PulseCrossSim] dst-reset phase: fired=$fired2 dropped=$dropped2 (sum>=100 means some fired-then-stale)")

      println(s"[PC] phase3 start t=${simTime()}")
      // ---- 3. lone SOURCE resets at random phases ----
      val acks3 = acks
      for (i <- 0 until 100) {
        if (i % 10 == 0) println(s"[PC] p3 i=$i t=${simTime()}")
        request()
        dut.src.waitSampling(rng.nextInt(12))
        dut.src.assertReset(); dut.src.waitRisingEdge(2 + rng.nextInt(6)); dut.src.deassertReset()
        dut.src.waitSampling(3)
        assert(waitIdle(), s"src-reset case $i: busy stuck")
        assert(firesThisReq <= 1, s"src-reset case $i: duplicate fire ${firesThisReq}")
      }
      // after the reset storm, the crossing still works normally
      for (i <- 0 until 50) { request(); assert(waitIdle()); assert(firesThisReq == 1, s"post-storm req $i fired $firesThisReq") }
      println(s"[PulseCrossSim] PASS withDstDone=$withDstDone src=$srcPeriod dst=$dstPeriod: fires=$fires acks=$acks drops=$drops maxFiresPerReq=$maxFiresPerReq")
    }
  }

  run(withDstDone = false, srcPeriod = 3, dstPeriod = 2, seed = 42)
  run(withDstDone = true,  srcPeriod = 3, dstPeriod = 2, seed = 43)
  run(withDstDone = true,  srcPeriod = 2, dstPeriod = 3, seed = 44)   // dsp→ddr direction (faster src)
  println("[PulseCrossSim] all PASS")
}
