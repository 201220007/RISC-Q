package riscq.ddr.sim

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import spinal.lib.bus.amba4.axi.sim.{AxiMemorySim, AxiMemorySimConfig, Axi4Master}
import spinal.lib.bus.tilelink.sim.{MasterAgent, IdAllocator, IdCallback}
import spinal.lib.bus.tilelink.DebugId
import riscq.soc.PulseTableSoc
import riscq.ddr.ReadoutDdrRegs._
import scala.collection.mutable

/**
 * G3 — SoC-level EQUIVALENCE: a real ADC tone, demodulated by the real `ReadoutDecoder` inside
 * `PulseTableSoc`, must land in DDR as a tagged word whose I/Q is bit-exactly the CPU-visible
 * `real`/`imag` truncated to the 28-bit DDR fields.
 *
 * That is the whole claim of the uplink — "the DDR copy IS the result the CPU sees" — so it is checked
 * against the CPU's own `ReadoutResultSink` read-back, not against a model. The stimulus is the same
 * VNA-style tone `PulseTableSocSim` uses (matched and detuned LO), so the decoder is genuinely running.
 *
 * Run: mill-1.1.0 runMain riscq.ddr.sim.PulseTableSocDdrSim
 */
object PulseTableSocDdrSim extends App {
  // r11-#8: the approved gate is the PRODUCTION geometry (14 qubits) with more than one core actually
  // producing readouts, so the per-core tag/ordering claims are exercised, not just core 0.
  val qubitNum = 14
  val dacMap   = riscq.soc.SocChannelMap.dacMap(qubitNum)
  val adcMap   = riscq.soc.SocChannelMap.adcMap(qubitNum)
  val hotCores = Seq(0, 7)     // one per readout ADC group (SocChannelMap splits at core 7)
  val N = 16; val w = 16; val adcN = 4
  val JAL_SELF = BigInt("6f", 16)
  val BASE     = 0x2000L

  val resAddr = 0x4200; val realAddr = 0x4204; val imagAddr = 0x4208
  val demodStAddr = 0x34100

  def leBytes(v: BigInt, n: Int): List[Byte] = List.tabulate(n)(i => ((v >> (8 * i)) & 0xFF).toByte)
  def w16(v: Int): Int = ((v & 0xFFFF) << 16) & 0xFFFFFFFF

  SimConfig.addSimulatorFlag("-Wno-MULTIDRIVEN").addSimulatorFlag("--x-initial 0")
    .compile {
      val spec = riscq.soc.spec.SocSpec.qubits(qubitNum, dacMap, adcMap).copy(resultsPath = "antq_uplink")
      val soc = new PulseTableSoc(spec, withTest = true)
      soc.ddrUplink.up.io.dspAdmit.simPublic()
      soc.ddrUplink.calibDone.simPublic()
      soc
    }
    .doSim("soc_ddr_equivalence", seed = 7) { dut =>
      SimTimeout(80000000)
      val hostCd = dut.clockDomain
      val dspCd  = dut.dspCd
      val ddrCd  = ClockDomain(dut.ddrUplink.ddrClk, dut.ddrUplink.ddrRst)

      dut.io.axi.ar.valid #= false; dut.io.axi.aw.valid #= false; dut.io.axi.w.valid #= false
      dut.io.axi.r.ready #= false;  dut.io.axi.b.ready #= false
      dut.riscqArea.testMasters(0).node.bus.a.valid #= false
      for (i <- dut.io.adc.indices) { dut.io.adc(i).valid #= true; dut.io.adc(i).payload #= 0 }
      dut.io.dac.foreach(_.ready #= true)
      dut.ddrUplink.rd.ready #= true

      // r20-#6: prove the DDR status register is readable while the DDR side is DEAD -- the case it
      // exists for. Start the HOST domain only, hold ddrRst asserted and calibDone low, and read the
      // register over the host bus. If this needed ddrClk or the DDR reset tree it would hang here.
      dut.ddrUplink.calibDone #= false
      hostCd.forkStimulus(10)
      ddrCd.assertReset()                       // ddrClk is not even toggling yet
      hostCd.waitSampling(40)
      val axiEarly = Axi4Master(dut.io.axi, hostCd)
      val ddrStatusAddr = BigInt(dut.map.hostCtrlBase) + 0x58
      def readHost(addr: BigInt): BigInt = {
        var r: Option[BigInt] = None
        axiEarly.readSingle(addr, 4) { bs => r = Some(bs.reverse.foldLeft(BigInt(0))((a, b) => (a << 8) | (b & 0xff))) }
        var n = 0; while (r.isEmpty && n < 20000) { hostCd.waitSampling(); n += 1 }
        assert(r.isDefined, f"host read 0x$addr%x timed out"); r.get
      }
      val dead = readHost(ddrStatusAddr)
      assert((dead >> 16) == 0xCA1B,
        f"DDR status magic missing while the DDR side is dead: 0x$dead%x")
      assert(((dead >> 0) & 1) == 0, f"calib_done should be 0 with calibDone held low: 0x$dead%x")
      assert(((dead >> 1) & 1) == 0, f"ui_reset_released should be 0 with ddrRst asserted: 0x$dead%x")
      println(f"[G3] DDR status readable with the DDR side DEAD: 0x$dead%08x (magic ok, calib=0, ui_rst=0)")

      // now bring the DDR side up and watch the same register follow
      dspCd.forkStimulus(10); ddrCd.forkStimulus(14)
      dut.ddrUplink.calibDone #= true
      hostCd.waitSampling(60)
      val alive = readHost(ddrStatusAddr)
      assert((alive >> 16) == 0xCA1B, f"DDR status magic lost: 0x$alive%x")
      assert(((alive >> 0) & 1) == 1, f"calib_done did not follow calibDone: 0x$alive%x")
      assert(((alive >> 1) & 1) == 1, f"ui_reset_released did not follow ddrRst: 0x$alive%x")
      println(f"[G3] DDR status after bring-up: 0x$alive%08x (calib=1, ui_rst=1)")

      val axi  = axiEarly
      val mem  = AxiMemorySim(dut.ddrUplink.ddr, ddrCd, AxiMemorySimConfig(maxOutstandingWrites = 4))
      mem.start()
      val ctrl = Axi4Master(dut.ddrUplink.ctrl, ddrCd, "ddrctrl")

      def dwr(off: Int, v: BigInt): Unit = {
        var done = false; ctrl.writeCB(off, leBytes(v, 4)) { done = true }
        var n = 0; while (!done && n < 20000) { ddrCd.waitSampling(); n += 1 }
        assert(done, f"ddr ctrl write 0x$off%x timed out")
      }
      def drd(off: Int): BigInt = {
        var r: Option[BigInt] = None
        ctrl.readSingle(off, 4) { bs => r = Some(bs.reverse.foldLeft(BigInt(0))((a, b) => (a << 8) | (b & 0xff))) }
        var n = 0; while (r.isEmpty && n < 20000) { ddrCd.waitSampling(); n += 1 }
        assert(r.isDefined, f"ddr ctrl read 0x$off%x timed out"); r.get
      }
      def bit(s: BigInt, b: Int) = ((s >> b) & 1) == 1

      // ── boot the SoC exactly as PulseTableSocSim does ──────────────────────────────────
      def loadInstr(core: Int, word: Int, v: BigInt): Unit =
        axi.write(BigInt(dut.map.coreMemOffset(core)) + word.toLong * 4, leBytes(v, 4))
      for (c <- 0 until qubitNum) loadInstr(c, 0, JAL_SELF)
      val hostCtrlAddr = BigInt(dut.map.hostCtrlBase)
      axi.write(hostCtrlAddr, List(0x01, 0, 0, 0).map(_.toByte)); hostCd.waitSampling(20)
      axi.write(hostCtrlAddr, List(0x00, 0, 0, 0).map(_.toByte)); hostCd.waitSampling(60)
      val tA = dut.riscqArea.time.toBigInt.toInt; dspCd.waitSampling(50)
      assert(dut.riscqArea.time.toBigInt.toInt != tA, "batch time not advancing")

      // core-local TileLink master to read the CPU-visible ReadoutResultSink
      implicit val idAlloc: IdAllocator = new IdAllocator(DebugId.width)
      implicit val idCb: IdCallback = new IdCallback
      val agents  = hotCores.map(c => new MasterAgent(dut.riscqArea.testMasters(c).node.bus, dspCd))
      val src     = 0
      def signed32(u: BigInt): BigInt = { val m = BigInt(1) << 32; val r = ((u % m) + m) % m; if (r >= (BigInt(1) << 31)) r - m else r }
      def wrC(ci: Int, a: Int, v: Int): Unit = agents(ci).putFullData(src, a, leBytes(BigInt(v & 0xFFFFFFFFL), 4))

      // demod envelope (square, ~unity) into core 0's demod bank
      def loadDemodEnv(core: Int, a: Int, word: BigInt): Unit =
        axi.write(BigInt(dut.map.envOffset(core, 2)) + a.toLong * 4, leBytes(word, 4))
      for (c <- hotCores; a <- 0 until 64) loadDemodEnv(c, a, BigInt(0x7FFF))

      // Free-running ADC tone, phase-locked to batch time (as in PulseTableSocSim). r12-#8: each hot core
      // gets a DISTINCT amplitude, so its demod result is a distinct number. With one shared waveform the
      // two cores produced identical values and swapping their tags would have passed every comparison
      // below — the tag routing was not actually being tested.
      val Fcarrier = 4096; val Fdetuned = 12288
      val adcIds = hotCores.map(adcMap)
      assert(adcIds.distinct.size == adcIds.size,
        s"hotCores ${hotCores.mkString(",")} share ADC ${adcIds.mkString(",")} — pick cores in different groups")
      val ampOf = hotCores.indices.map(ci => 28000 - 12000 * ci)      // 28000, 16000, ...
      def adcWord(t: Long, f: Int, amp: Int): BigInt = {
        var word = BigInt(0)
        for (k <- 0 until adcN) {
          val s = t * adcN + k
          val ang = scala.math.Pi * (f.toDouble / 32768.0) * s
          word |= BigInt(scala.math.round(amp * scala.math.cos(ang)).toInt & 0xFFFF) << (k * w)
        }
        word
      }
      var adcFreq = Fcarrier
      fork { while (true) {
        val t = dut.riscqArea.time.toBigInt.toLong
        for (ci <- hotCores.indices) dut.io.adc(adcIds(ci)).payload #= adcWord(t, adcFreq, ampOf(ci))
        dspCd.waitSampling()
      } }

      // ── start the DDR run BEFORE any readout window, so every result is admitted ───────
      dwr(WR_BASE, BigInt(BASE)); dwr(BASE_RESET, BigInt(1))
      var n = 0
      while (!(bit(drd(STATUS), S_RUN_ACTIVE) && bit(drd(STATUS), S_DSP_ADMIT)) && n < 400) { ddrCd.waitSampling(10); n += 1 }
      assert(bit(drd(STATUS), S_RUN_ACTIVE), f"run never became active: 0x${drd(STATUS)}%x")

      // ── fire real demod windows and record what the CPU sees ──────────────────────────
      val demodAmp = 12000; val roDur = 20
      // (real, imag) per core, in the order the CPU read them
      val cpuSeen = Array.fill(qubitNum)(mutable.ArrayBuffer[(BigInt, BigInt)]())
      def runWindow(ci: Int, label: String): Double = {
        val core = hotCores(ci)
        val st = dut.riscqArea.time.toBigInt.toInt + 200
        wrC(ci, demodStAddr, st)
        wrC(ci, 0x30004, w16(Fcarrier))
        wrC(ci, 0x30010, w16(0)); wrC(ci, 0x30014, w16(demodAmp))
        wrC(ci, 0x30018, w16(0)); wrC(ci, 0x3001C, w16(roDur))
        wrC(ci, 0x30000, 0)                                   // fire the demod = the readout
        waitUntil(dut.riscqArea.time.toBigInt.toInt >= st + roDur + 60)
        agents(ci).getInt(src, resAddr)                       // halts until the integral settles
        val re = signed32(agents(ci).getInt(src, realAddr))
        val im = signed32(agents(ci).getInt(src, imagAddr))
        cpuSeen(core) += ((re, im))
        val mag = scala.math.sqrt((re * re + im * im).toDouble)
        println(f"[G3] core $core%2d window $label: CPU real=$re imag=$im |z|=${mag.toLong}")
        mag
      }

      adcFreq = Fcarrier
      val magMatched = hotCores.indices.map(ci => runWindow(ci, "matched"))
      adcFreq = Fdetuned; dspCd.waitSampling(60)
      val magDetuned = hotCores.indices.map(ci => runWindow(ci, "detuned"))
      for (ci <- hotCores.indices) {
        assert(magMatched(ci) > 100000, s"core ${hotCores(ci)} matched magnitude ${magMatched(ci).toLong} too small")
        assert(magMatched(ci) > 4 * magDetuned(ci),
          s"core ${hotCores(ci)}: no selectivity (${magMatched(ci).toLong} vs ${magDetuned(ci).toLong})")
      }
      // r12-#8: the cores must be TELLING APART, not just both non-zero — the demod magnitudes have to
      // track the per-core ADC amplitudes. Without this a tag swap in the uplink is invisible.
      for (ci <- 1 until hotCores.size) {
        val want = magMatched(0) * ampOf(ci).toDouble / ampOf(0).toDouble
        assert(scala.math.abs(magMatched(ci) - want) < 0.15 * want,
          f"core ${hotCores(ci)} magnitude ${magMatched(ci).toLong} is not the expected " +
          f"${want.toLong} for amplitude ${ampOf(ci)} vs ${ampOf(0)} — the ADC drive is not per-core")
        assert(scala.math.abs(magMatched(ci) - magMatched(0)) > 0.2 * magMatched(0),
          s"cores ${hotCores(0)} and ${hotCores(ci)} produce indistinguishable results — a tag swap would pass")
      }

      // ── flush + drain the DDR copy and compare it to what the CPU read ────────────────
      dwr(FLUSH, BigInt(1))
      var m = 0; var st = drd(STATUS)
      while (bit(st, S_FLUSH_BUSY) && m < 8000) { ddrCd.waitSampling(20); st = drd(STATUS); m += 1 }
      assert(!bit(st, S_FLUSH_BUSY), "flush never completed")
      assert(bit(st, S_WRITE_DONE), f"write_done not set: 0x$st%x")
      assert(!bit(st, S_OVF_ANY) && !bit(st, S_WRAPPED) && !bit(st, S_BRESP_ERR) && !bit(st, S_EARLY_LATE),
             f"error sticky set: 0x$st%x")

      val acc   = (0 until qubitNum).map(i => drd(ACCEPTED + 4 * i).toInt)
      val total = acc.sum
      println(s"[G3] accepted per core = ${acc.zipWithIndex.filter(_._1 != 0).map { case (n, i) => s"$i:$n" }.mkString(" ")}")
      // r11-#8: EXACT per-core counts, not `>=`
      for (c <- 0 until qubitNum) {
        assert(acc(c) == cpuSeen(c).size,
          s"core $c: hardware accepted ${acc(c)} results, the CPU read ${cpuSeen(c).size} windows")
        assert(drd(REJECTED + 4 * c) == 0, s"core $c had rejections")
      }
      assert(total == cpuSeen.map(_.size).sum && total > 0, s"total $total")

      val nbytes = (drd(FINAL_ADDR) - BASE).toInt
      assert(nbytes % 32 == 0, s"final_addr-base = $nbytes is not beat-aligned")
      assert(nbytes / 8 - total >= 0 && nbytes / 8 - total <= 3, s"pad: ${nbytes / 8} words for $total results")

      val bytes = mem.memory.readArray(BASE, total.toLong * 8)
      val words = (0 until total).map(k => (0 until 8).foldLeft(BigInt(0))((a, b) =>
        a | (BigInt(bytes(k * 8 + b) & 0xff) << (8 * b))))

      // r11-#8: ORDERED, ONE-TO-ONE comparison per tag -- `exists` would let a duplicate or an extra pass.
      def trunc(v: BigInt): BigInt = (v & 0xFFFFFFFFL) >> 4
      for (c <- 0 until qubitNum) {
        val got = words.filter(x => ((x >> 56) & 0xff) == c)
        assert(got.size == cpuSeen(c).size, s"tag $c: ${got.size} words in DDR, CPU saw ${cpuSeen(c).size}")
        for (((w, (re, im)), k) <- got.zip(cpuSeen(c)).zipWithIndex) {
          assert(((w >> 28) & 0x0FFFFFFF) == trunc(re),
            f"core $c word $k real: DDR 0x${(w >> 28) & 0x0FFFFFFF}%x != CPU 0x${trunc(re)}%x (raw $re)")
          assert((w & 0x0FFFFFFF) == trunc(im),
            f"core $c word $k imag: DDR 0x${w & 0x0FFFFFFF}%x != CPU 0x${trunc(im)}%x (raw $im)")
        }
      }
      // r12-#8: state the discriminating power explicitly in the verdict line.
      println(f"[G3] per-core |z| matched = ${hotCores.indices.map(ci => f"${hotCores(ci)}:${magMatched(ci).toLong}").mkString(" ")}" +
              f" (ADC amplitudes ${ampOf.mkString(",")}) — the tags are distinguishable")
      println(s"[G3] PASS: $total CPU-visible demod results on cores ${hotCores.mkString(",")} matched " +
              s"ONE-TO-ONE and IN ORDER in DDR; ${nbytes} B drained at ${qubitNum} qubits")
      simSuccess()
    }
}
