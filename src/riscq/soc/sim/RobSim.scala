package riscq.soc.sim

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import spinal.lib.bus.tilelink.DebugId
import spinal.lib.bus.tilelink.sim.{IdAllocator, IdCallback, MasterAgent}
import spinal.lib.bus.amba4.axi.sim.Axi4Master
import riscq.soc.PulseTableSoc

/**
 * Simplest possible check that the readout-observation buffer (`robs`) in [[PulseTableSoc]] works. The
 * rob captures the per-lane sum of the MAPPED ADC inputs into `robs(0)` while any core's readout-drive
 * pulse is valid. So: hold the two mapped ADCs at known constants, fire the readout-drive pulse on
 * core 0, wait past the window, then read `robs(0)` row 0 back over AXI and assert every lane equals
 * the per-lane ADC sum.
 *
 * (The readout-drive pulse's `valid` window is purely timing-driven — `[startTime, startTime+dur)`,
 * independent of envelope content — so no readout envelope is loaded; the rob only needs the fire.)
 * Run with `mill runMain riscq.soc.sim.RobSim`.
 */
object RobSim extends App {
  val qubitNum = 2
  val dacMap   = Map((0, 0) -> 8, (0, 1) -> 8, (1, 0) -> 1, (1, 1) -> 1)
  val adcMap   = Map(0 -> 12, 1 -> 13)      // rob sums the mapped ADCs 12 + 13 per lane
  val adcN = 4; val w = 16
  val JAL_SELF = BigInt("6f", 16)           // jal x0,0 — 1-instruction self-loop (keeps dBus idle)

  def w16(v: Int): Int = (((v & 0xFFFF) << 16) & 0xFFFFFFFFL).toInt
  // pack `adcN` 16-bit real lanes into one io.adc payload word (lane k in bits [k*w .. k*w+w)).
  def adcWord(lanes: Seq[Int]): BigInt =
    (0 until adcN).foldLeft(BigInt(0))((acc, k) => acc | (BigInt(lanes(k)) & 0xFFFF) << (k * w))
  def leBytes(v: BigInt, n: Int): List[Byte] = List.tabulate(n)(i => ((v >> (8 * i)) & 0xFF).toByte)
  def leToBig(bs: Seq[Byte]): BigInt = bs.reverse.foldLeft(BigInt(0))((a, b) => (a << 8) | (b & 0xFF))

  SimConfig.addSimulatorFlag("-Wno-MULTIDRIVEN")     // clock-crossing Bram blackbox written from clka+clkb
    .addSimulatorFlag("--x-initial 0")               // 0-init the host→dsp CDC FIFO state (spurious decoder-miss guard)
    .compile(PulseTableSoc(qubitNum, dacMap, adcMap, withTest = true, linkPipe = 8))
    .doSim("rob", seed = 42) { dut =>
    val hostCd = dut.clockDomain
    val dspCd  = dut.dspCd

    // quiet the undriven bus masters during reset
    dut.io.axi.ar.valid #= false; dut.io.axi.aw.valid #= false; dut.io.axi.w.valid #= false
    dut.io.axi.r.ready #= false;  dut.io.axi.b.ready #= false
    dut.riscqArea.testMasters(0).node.bus.a.valid #= false

    // hold the mapped ADCs at known constants: ADC12 lane k = 1000+k, ADC13 lane k = 5.
    val adc12    = (0 until adcN).map(1000 + _)
    val adc13    = List.fill(adcN)(5)
    val expected = (0 until adcN).map(k => adc12(k) + adc13(k))     // per-lane rob content
    for (i <- dut.io.adc.indices) { dut.io.adc(i).valid #= true; dut.io.adc(i).payload #= 0 }
    dut.io.adc(12).payload #= adcWord(adc12)
    dut.io.adc(13).payload #= adcWord(adc13)

    hostCd.forkStimulus(10); dspCd.forkStimulus(10)
    hostCd.waitSampling(40)                          // io.dspRst deasserts

    val axi = Axi4Master(dut.io.axi, hostCd)
    for (c <- 0 until qubitNum) axi.write(BigInt(dut.map.coreMemOffset(c)), leBytes(JAL_SELF, 4))

    // release the cores from reset (region 5, riscqReset @ offset 0): assert then deassert for a clean edge
    val hostCtrlAddr = BigInt(dut.map.hostCtrlBase)
    axi.write(hostCtrlAddr, List(0x01, 0x00, 0x00, 0x00).map(_.toByte))
    hostCd.waitSampling(20)
    axi.write(hostCtrlAddr, List(0x00, 0x00, 0x00, 0x00).map(_.toByte))
    hostCd.waitSampling(60)                          // CDC + settle ⇒ batch time advancing, cores running

    // ── fire the readout-drive pulse on core 0 (RF window @0x10000 + roChannel base 0x10000 = 0x20000) ──
    implicit val idAllocator = new IdAllocator(DebugId.width)
    implicit val idCallback  = new IdCallback
    val agent = new MasterAgent(dut.riscqArea.testMasters(0).node.bus, dspCd)
    val src = 0
    def wr(addr: Int, word: Int): Unit = { agent.putInt(src, addr, word); dspCd.waitSampling(2) }

    val dur       = 8
    val startTime = dut.riscqArea.time.toBigInt.toInt + 400        // schedule well ahead of live batch time
    wr(0x24100, startTime)                                         // roChannel per-buffer startTime
    wr(0x20004, w16(1800))                                         // freq
    wr(0x20010, w16(0)); wr(0x20014, w16(9000))                    // table[0]: phase, amp
    wr(0x20018, w16(0)); wr(0x2001C, w16(dur))                     // table[0]: env base, dur = capture window
    wr(0x20000, 0)                                                 // fire outId 0

    waitUntil(dut.riscqArea.time.toBigInt.toInt >= startTime + dur + 60)  // past the window ⇒ rob writes done
    hostCd.waitSampling(10)

    // ── read rob row 0 back over AXI (lane k at byte offset k*4) and check == the per-lane ADC sum ──
    val robBase = BigInt(dut.map.robBase)
    val got     = (0 until adcN).map(k => leToBig(axi.read(robBase + k * 4, 4)))
    for (k <- 0 until adcN)
      assert(got(k) == BigInt(expected(k)), s"[rob] lane $k: got ${got(k)} != expected ${expected(k)}")
    println(s"[RobSim] PASS: robs captured mapped-ADC sum per lane = ${got.mkString(",")} " +
      s"(ADC12 ${adc12.mkString(",")} + ADC13 ${adc13.mkString(",")}) on readout-pulse fire.")
    simSuccess()
  }
}
