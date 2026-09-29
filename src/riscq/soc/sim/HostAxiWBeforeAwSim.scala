package riscq.soc.sim

import spinal.core._
import spinal.core.fiber.Fiber
import spinal.core.sim._
import spinal.lib.bus.tilelink
import riscq.soc.{PulseTableSoc, SocChannelMap}
import riscq.soc.spec.SocSpec

import scala.collection.mutable.ArrayBuffer

/**
 * Directed check of the SoC's host-side AXI slave (`io.axi`, the port the PS reaches through HPM0):
 * a single-beat write whose W beat is presented 1, 2 or 3 host cycles **before** its AW (plus 0, the
 * usual together case, as the control) must land, and land exactly once. A QubiC AXI-Lite slave once
 * dropped such writes silently, so this pins the behaviour of `Axi4ToTilelinkFiber` + the host fabric.
 *
 * Two targets, both behind the same bridge:
 *   - the host control block's write-only `timeOffset` (0x40 / 0x44), checked on the register itself;
 *   - a core RAM word (core 1), read back over AXI with its two neighbours as a clobber guard.
 * "Exactly once" is counted where it matters: every Put that reaches the control block's and the core
 * RAM region's TileLink nodes, and every B response, over the write and a quiet tail after it.
 * Run with `mill runMain riscq.soc.sim.HostAxiWBeforeAwSim`.
 */
object HostAxiWBeforeAwSim extends App {
  /** PulseTableSoc with the two host-side leaf buses and `timeOffset` exposed to the sim. */
  class Dut(spec: SocSpec) extends PulseTableSoc(spec) {
    timeOffset.simPublic()
    Fiber build new Area {
      hostCtrlDriver.up.bus.get.simPublic()
      riscqMemBus.bus.get.simPublic()
    }
  }

  val qubitNum = 2
  val spec     = SocSpec.qubits(qubitNum, SocChannelMap.dacMap(qubitNum), SocChannelMap.adcMap(qubitNum))
  val leads    = Seq(0, 1, 2, 3)   // cycles W is presented ahead of AW (0 = the together control)
  val quiet    = 64                // host cycles after B in which no further Put / B may appear

  SimConfig.addSimulatorFlag("-Wno-MULTIDRIVEN")   // the clock-crossing Bram blackbox is written from clka+clkb
    .addSimulatorFlag("--x-initial 0")             // 0-init pre-reset X state (the host→dsp CDC FIFO), as PulseTableSocSim
    .compile(new Dut(spec))
    .doSim("wBeforeAw", seed = 42) { dut =>
    val hostCd = dut.clockDomain
    val axi    = dut.io.axi
    val ctrlBus = dut.hostCtrlDriver.up.bus.get
    val memBus  = dut.riscqMemBus.bus.get

    axi.ar.valid #= false; axi.aw.valid #= false; axi.w.valid #= false
    axi.r.ready #= true; axi.b.ready #= true
    for (a <- Seq(axi.aw, axi.ar)) {
      a.id #= 0; a.addr #= 0; a.len #= 0; a.size #= 2; a.burst #= 1
      a.lock #= 0; a.cache #= 0; a.prot #= 0; a.qos #= 0; a.region #= 0
    }
    axi.w.data #= 0; axi.w.strb #= 0; axi.w.last #= false
    dut.io.hostMem.aw.ready #= true; dut.io.hostMem.w.ready #= true; dut.io.hostMem.b.valid #= false
    for (i <- dut.io.adc.indices) { dut.io.adc(i).valid #= true; dut.io.adc(i).payload #= 0 }

    hostCd.forkStimulus(10); dut.dspCd.forkStimulus(10)
    hostCd.waitSampling(40)                        // io.dspRst deasserts; the cores stay in reset (power-up)

    // ── monitors: Puts reaching each leaf node, and AXI write responses ──
    def isPut(b: tilelink.Bus) = b.a.valid.toBoolean && b.a.ready.toBoolean && b.a.opcode.toEnum != tilelink.Opcode.A.GET
    var ctrlPuts, memPuts = 0
    val bResps = ArrayBuffer[(Int, Int)]()         // (id, resp)
    hostCd.onSamplings {
      if (isPut(ctrlBus)) ctrlPuts += 1
      if (isPut(memBus))  memPuts  += 1
      if (axi.b.valid.toBoolean && axi.b.ready.toBoolean) bResps += ((axi.b.id.toInt, axi.b.resp.toInt))
    }

    def timeout(n: Int, what: String): Unit = assert(n < 2000, s"timeout waiting for $what")

    /** One single-beat write; W is valid `lead` cycles before AW. Returns whether W's handshake
      * completed before AW was presented (i.e. the slave took the early beat). */
    def write(addr: BigInt, data: BigInt, lead: Int, id: Int): Boolean = {
      axi.w.data #= data; axi.w.strb #= 0xF; axi.w.last #= true; axi.w.valid #= true
      var awOn, awDone, wDone, wEarly = false
      var n = 0
      while (!(awDone && wDone)) {
        if (n == lead && !awOn) {
          axi.aw.addr #= addr; axi.aw.id #= id; axi.aw.valid #= true; awOn = true
        }
        hostCd.waitSampling(); n += 1; timeout(n, s"write handshake @0x${addr.toString(16)}")
        if (!wDone && axi.w.valid.toBoolean && axi.w.ready.toBoolean) {
          wDone = true; wEarly = !awOn; axi.w.valid #= false
        }
        if (awOn && !awDone && axi.aw.valid.toBoolean && axi.aw.ready.toBoolean) {
          awDone = true; axi.aw.valid #= false
        }
      }
      wEarly
    }

    def read(addr: BigInt, id: Int = 1): BigInt = {
      axi.ar.addr #= addr; axi.ar.id #= id; axi.ar.valid #= true
      var n = 0
      do { hostCd.waitSampling(); n += 1; timeout(n, "AR") } while (!axi.ar.ready.toBoolean)
      axi.ar.valid #= false
      do { hostCd.waitSampling(); n += 1; timeout(n, "R") } while (!axi.r.valid.toBoolean)
      assert(axi.r.resp.toInt == 0 && axi.r.last.toBoolean, s"read @0x${addr.toString(16)}: bad R")
      axi.r.data.toBigInt
    }

    /** Write, wait for its B, then a quiet tail; assert exactly one B (OKAY, right id) and exactly the
      * expected Put count on each leaf node. */
    def checkedWrite(tag: String, addr: BigInt, data: BigInt, lead: Int, ctrl: Int, mem: Int): Unit = {
      val (c0, m0, b0) = (ctrlPuts, memPuts, bResps.length)
      val id = lead & 3
      val early = write(addr, data, lead, id)
      var n = 0
      while (bResps.length == b0) { hostCd.waitSampling(); n += 1; timeout(n, s"$tag B") }
      hostCd.waitSampling(quiet)
      val bs = bResps.drop(b0)
      assert(bs == Seq((id, 0)), s"$tag lead=$lead: B responses $bs, want exactly one ($id, OKAY)")
      assert(ctrlPuts - c0 == ctrl, s"$tag lead=$lead: ${ctrlPuts - c0} Puts at the control block, want $ctrl")
      assert(memPuts - m0 == mem, s"$tag lead=$lead: ${memPuts - m0} Puts at the core RAM region, want $mem")
      println(f"[HostAxiWBeforeAwSim] $tag%-14s lead=$lead: 1 B, ${ctrlPuts - c0} ctrl Put, ${memPuts - m0} RAM Put" +
        (if (early) " (W taken before AW)" else " (W held until AW)"))
    }

    // ── host control block: timeOffset LO / HI ──
    val ctrl = BigInt(dut.map.hostCtrlBase)
    for (lead <- leads) {
      val lo = BigInt(0x10000000L + 0x1111 * (lead + 1))
      val hi = BigInt(0x20000000L + 0x0101 * (lead + 1))
      checkedWrite("timeOffset.lo", ctrl + 0x40, lo, lead, ctrl = 1, mem = 0)
      checkedWrite("timeOffset.hi", ctrl + 0x44, hi, lead, ctrl = 1, mem = 0)
      val got = dut.timeOffset.toBigInt
      assert(got == ((hi << 32) | lo), f"timeOffset lead=$lead: got 0x$got%x, want 0x${(hi << 32) | lo}%x")
    }

    // ── core RAM word (core 1, word 5), neighbours 4 and 6 as the clobber guard ──
    val word = BigInt(dut.map.coreMemOffset(1)) + 5 * 4
    val guard = Seq(word - 4 -> BigInt(0xA5A50004L), word + 4 -> BigInt(0xA5A50006L))
    for ((a, v) <- guard) checkedWrite("ram guard", a, v, 0, ctrl = 0, mem = 1)
    for (lead <- leads) {
      val v = BigInt(0xC0DE0000L + lead)
      checkedWrite("ram word", word, v, lead, ctrl = 0, mem = 1)
      val got = read(word)
      assert(got == v, f"ram lead=$lead: read back 0x$got%x, want 0x$v%x")
      for ((a, gv) <- guard) assert(read(a) == gv, f"ram lead=$lead: neighbour 0x$a%x clobbered")
    }

    val writes = 2 * leads.length + guard.length + leads.length
    assert(bResps.length == writes, s"${bResps.length} B responses for $writes writes")
    println(s"[HostAxiWBeforeAwSim] PASS  W 0/1/2/3 cycles before AW: every write landed exactly once " +
      s"(timeOffset lo/hi on the register, core-1 RAM word read back, neighbours intact); $writes writes, $writes B.")
    simSuccess()
  }
}
