package riscq.ddr.sim

import spinal.core._
import spinal.core.sim._
import scala.util.Random

/**
 * P3b: random AW / AR / B (and optionally W) stall injection for the uplink SpinalSims, through the
 * test-only stall inputs of [[ReadoutDdrUplinkDut]]. In P3a these channels never stalled: AxiMemorySim
 * takes every address at once and answers B at once (Codex P3a audit r3 #3).
 *
 * Each channel gets its own seeded bursty pattern: runs of 1..`maxRun` DDR cycles, each run stalled
 * with probability `p`. The counters record the cycles the uplink actually saw a stall (VALID high and
 * the stall asserted), so a run can assert the injection was not vacuous.
 */
case class StallProfile(pAw: Double, pAr: Double, pB: Double, pW: Double, maxRun: Int) {
  def any: Boolean = pAw > 0 || pAr > 0 || pB > 0 || pW > 0
}

object StallProfile {
  val none  = StallProfile(0, 0, 0, 0, 1)
  /** G2 / CDC stall pass: every channel stalled about a third of the time, in runs of up to 12 cycles */
  val heavy = StallProfile(0.35, 0.35, 0.35, 0.25, 12)
  /** AW/AR/B only (W is driven by hand in the CDC W-backpressure scenario) */
  val addrB = StallProfile(0.35, 0.35, 0.35, 0.0, 12)
}

class StallInjector(dut: ReadoutDdrUplinkDut, ddrCd: ClockDomain, prof: StallProfile, seed: Long) {
  var awStalled, arStalled, bStalled, wStalled = 0L
  @volatile var enabled = true    // false: stop driving the stall pins (a scenario takes them over)
  private def pattern(rng: Random, p: Double): Iterator[Boolean] =
    Iterator.continually { val on = p > 0 && rng.nextDouble() < p; Iterator.fill(1 + rng.nextInt(prof.maxRun))(on) }.flatten

  dut.io.awStall #= false; dut.io.arStall #= false; dut.io.bStall #= false
  if (prof.pW > 0) dut.io.wStall #= false

  if (prof.any) fork {
    val aw = pattern(new Random(seed * 31 + 1), prof.pAw)
    val ar = pattern(new Random(seed * 31 + 2), prof.pAr)
    val b  = pattern(new Random(seed * 31 + 3), prof.pB)
    val w  = pattern(new Random(seed * 31 + 4), prof.pW)
    while (true) {
      ddrCd.waitSampling()
      // count what the uplink saw in the cycle that just ended
      if (dut.io.awStall.toBoolean && dut.up.io.ddr.aw.valid.toBoolean) awStalled += 1
      if (dut.io.arStall.toBoolean && dut.up.io.ddr.ar.valid.toBoolean) arStalled += 1
      if (dut.io.bStall.toBoolean && dut.io.ddr.b.valid.toBoolean) bStalled += 1
      if (prof.pW > 0 && dut.io.wStall.toBoolean && dut.up.io.ddr.w.valid.toBoolean) wStalled += 1
      // disabled: the pins are left alone, so a scenario can drive them by hand
      val (sa, sr, sb, sw) = (aw.next(), ar.next(), b.next(), w.next())
      if (enabled) {
        dut.io.awStall #= sa; dut.io.arStall #= sr; dut.io.bStall #= sb
        if (prof.pW > 0) dut.io.wStall #= sw
      }
    }
  }

  def summary: String = s"injected stalls seen: AW=$awStalled AR=$arStalled B=$bStalled W=$wStalled"
}
