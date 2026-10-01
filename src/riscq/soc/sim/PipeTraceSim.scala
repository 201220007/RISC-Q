package riscq.soc.sim

import java.io.{File, PrintWriter}
import spinal.core._
import spinal.core.sim._
import spinal.lib.bus.amba4.axi.sim.{AxiMemorySim, AxiMemorySimConfig, Axi4Master}
import spinal.lib.bus.tilelink.sim.{MasterAgent, IdAllocator, IdCallback}
import spinal.lib.bus.tilelink.DebugId
import spinal.lib.misc.Elf
import spinal.lib.sim.SparseMemory
import riscq.soc.PulseTableSoc
import riscq.soc.spec.SocSpec
import riscq.riscv.regfile.RegFilePlugin
import riscq.ddr.ReadoutDdrRegs._
import scala.util.Random

/**
 * N2 of qubic3 P3c-3 (evidence/P3c/PLAN_P3c3_pipelining_v2.md §2.2): the SoC lockstep, as a trace comparison.
 *
 * One deterministic stimulus drives a `PulseTableSoc` built from the antq spec `sim-dio-antq` with the P3c-3
 * timing pipeline OFF (the reference: today's antq RTL, N1) or ON (`RISCQ_TIMING_PIPE`). Every observable is
 * written to a change log stamped with its clock cycle; `scripts/ddr-gates/run-n2.sh` runs both variants and requires
 * the two logs to be byte-identical, each from a successful simulation and ending in its END line. The 3a changes are
 * cycle-exact by construction, so every stream is compared with zero offset, `now()` values and barrier stamps
 * included. Comparing the two designs (not a design with its own sink) makes the readout check independent: the
 * reference's integrals are the ones the decoder goldens verify.
 *
 * Observables (dspClk unless noted): every DAC and DIO output; each channel's time input; each core's posted
 * command stream and commit stream (PC, rd write); each decoder's res/real/imag/valid; the hub's broadcast; the
 * DDR write channel and the drain stream (ddrClk); every host AXI read result (hostClk).
 *
 * Stimulus (Codex gate finding 9): two runs separated by `riscqReset` and DONE; run 1 with `pulse_sched.elf` on core
 * 0 and run 2 with `xcore.elf` on both cores (CPU loads/stores, branches, hub barrier and mailboxes); a `timeOffset`
 * that wraps batch time past 2^32 inside run 1; gate fires swept across the push-margin boundary (deadline ±1); five
 * fires into a 4-deep queue; demod readouts with a nonzero carrier and a phase offset against a real ADC tone;
 * `phaseOffset`/`dcOffset`/`startTime` writes right after fires; DIO output trains and input edges; the `robs` trace
 * read back over host AXI; the uplink captures, flushes and drains each run.
 *
 *   RISCQ_TIMING_PIPE=0|1 mill runMain riscq.soc.sim.PipeTraceSim <trace.txt>
 */
object PipeTraceSim extends App {
  val pipe  = riscq.misc.TimingPipeKnob.enabled
  val out   = args.headOption.getOrElse(s"simWorkspace/pipe_trace_${if (pipe) "on" else "off"}.txt")
  val spec  = SocSpec.load("software/configs/sim-dio-antq.json")
  require(spec.withAntqUplink && spec.qubitNum == 2)
  val JAL_SELF = BigInt("6f", 16)
  val N = 16; val w = 16; val adcN = 4

  def leBytes(v: BigInt, n: Int): List[Byte] = List.tabulate(n)(i => ((v >> (8 * i)) & 0xFF).toByte)
  def w16(v: Int): Int = ((v & 0xFFFF) << 16)

  SimConfig.addSimulatorFlag("-Wno-MULTIDRIVEN").addSimulatorFlag("--x-initial 0")
    .compile {
      val soc = new PulseTableSoc(spec, withTest = true, timingPipeOverride = Some(pipe))
      soc.riscqArea.time.simPublic()
      for (c <- soc.riscqArea.riscqCores) {
        c.riscvSoc.cmd.simPublic()
        c.decoderRd.io.res.simPublic(); c.decoderRd.io.real.simPublic(); c.decoderRd.io.imag.simPublic()
        for (ch <- c.posted.channels) ch.timeBcast.simPublic()
      }
      soc.riscqArea.hub.out.simPublic()
      soc
    }
    .doSim("pipe_trace", seed = 11) { dut =>
      SimTimeout(400000000L)
      val log = new PrintWriter(new File(out))
      val hostCd = dut.clockDomain
      val dspCd  = dut.dspCd
      val ddrCd  = ClockDomain(dut.ddrUplink.ddrClk, dut.ddrUplink.ddrRst)

      dut.io.axi.ar.valid #= false; dut.io.axi.aw.valid #= false; dut.io.axi.w.valid #= false
      dut.io.axi.r.ready #= false;  dut.io.axi.b.ready #= false
      dut.riscqArea.testMasters.foreach(_.node.bus.a.valid #= false)
      for (i <- dut.io.adc.indices) { dut.io.adc(i).valid #= true; dut.io.adc(i).payload #= 0 }
      dut.io.dac.foreach(_.ready #= true)
      dut.io.dioIn.foreach(_ #= 0)
      dut.ddrUplink.rd.ready #= true
      dut.ddrUplink.calibDone #= true
      hostCd.forkStimulus(10); dspCd.forkStimulus(10); ddrCd.forkStimulus(14)
      hostCd.waitSampling(40)

      // ── the recorder: one line per observable change, stamped with its own clock's cycle ──
      var dcyc = 0L
      val cores = dut.riscqArea.riscqCores
      val rf = cores.map(_.riscvSoc.riscqFiber.riscq.host[RegFilePlugin].logic.exec)
      val last = scala.collection.mutable.HashMap[String, String]()
      def note(tag: String, cyc: Long, name: String, v: String): Unit =
        if (!last.get(name).contains(v)) { last(name) = v; log.println(s"$tag $cyc $name $v") }
      fork { while (true) {
        dspCd.waitSampling(); dcyc += 1
        for (d <- dut.io.dac.indices) note("D", dcyc, s"dac$d", dut.io.dac(d).payload.toBigInt.toString(16))
        for (d <- dut.io.dioOut.indices) note("D", dcyc, s"dio$d", dut.io.dioOut(d).toBigInt.toString(16))
        for ((c, i) <- cores.zipWithIndex) {
          for ((ch, k) <- c.posted.channels.zipWithIndex) note("D", dcyc, s"c${i}t$k", ch.timeBcast.toBigInt.toString(16))
          val cmd = c.riscvSoc.cmd
          if (cmd.valid.toBoolean) log.println(s"D $dcyc c${i}cmd ${cmd.payload.address.toBigInt.toString(16)} ${cmd.payload.data.toBigInt.toString(16)}")
          if (rf(i).dbgFiring.toBoolean)
            log.println(s"D $dcyc c${i}commit ${rf(i).dbgPc.toBigInt.toString(16)} ${if (rf(i).dbgWrite.toBoolean) s"x${rf(i).dbgRd.toInt}=${rf(i).dbgRdData.toBigInt.toString(16)}" else "-"}")
          val dec = c.decoderRd.io
          note("D", dcyc, s"c${i}res", s"${dec.res.valid.toBoolean} ${dec.res.payload.toBoolean} ${dec.real.toBigInt} ${dec.imag.toBigInt}")
        }
        val hb = dut.riscqArea.hub.out
        if (hb.valid.toBoolean)
          log.println(s"D $dcyc hub ${hb.payload.mask.toBigInt.toString(16)} ${hb.payload.put.address.toBigInt.toString(16)} ${hb.payload.put.data.toBigInt.toString(16)}")
      } }
      var ucyc = 0L
      fork { while (true) {
        ddrCd.waitSampling(); ucyc += 1
        val wch = dut.ddrUplink.ddr.w; val aw = dut.ddrUplink.ddr.aw
        if (aw.valid.toBoolean && aw.ready.toBoolean) log.println(s"U $ucyc aw ${aw.payload.addr.toBigInt.toString(16)} ${aw.payload.len.toInt}")
        if (wch.valid.toBoolean && wch.ready.toBoolean) log.println(s"U $ucyc w ${wch.payload.data.toBigInt.toString(16)}")
        val rd = dut.ddrUplink.rd
        if (rd.valid.toBoolean && rd.ready.toBoolean) log.println(s"U $ucyc rd ${rd.fragment.toBigInt.toString(16)} ${rd.last.toBoolean}")
      } }
      var hcyc = 0L
      fork { while (true) { hostCd.waitSampling(); hcyc += 1 } }

      // ── host, DDR model, uplink control (as G3) ──
      val axi = Axi4Master(dut.io.axi, hostCd)
      def readHost(addr: BigInt, what: String): BigInt = {
        var r: Option[BigInt] = None
        axi.readSingle(addr, 4) { bs => r = Some(bs.reverse.foldLeft(BigInt(0))((a, b) => (a << 8) | (b & 0xff))) }
        var n = 0; while (r.isEmpty && n < 20000) { hostCd.waitSampling(); n += 1 }
        assert(r.isDefined, f"host read 0x$addr%x timed out")
        log.println(s"H $hcyc $what ${r.get.toString(16)}"); r.get
      }
      def writeHost(addr: BigInt, v: BigInt): Unit = axi.write(addr, leBytes(v, 4))
      val mem  = AxiMemorySim(dut.ddrUplink.ddr, ddrCd, AxiMemorySimConfig(maxOutstandingWrites = 4, writeResponseDelay = 40))
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
      val hostCtrl = BigInt(dut.map.hostCtrlBase)

      def loadElf(core: Int, file: String): Unit = {
        val image = SparseMemory(seed = 0)
        new Elf(new File(file), 32).load(image, 0)
        for (i <- 0 until 512) writeHost(BigInt(dut.map.coreMemOffset(core)) + 4L * i,
          BigInt(image.readInt(0x80000000L + 4L * i).toLong & 0xFFFFFFFFL))
      }
      def park(core: Int): Unit = writeHost(BigInt(dut.map.coreMemOffset(core)), JAL_SELF)
      // envelopes: core 0 gate lines (distinct values), both cores' demod banks a square 0x7FFF window
      val gateEnvBytes = (N / 4) * 2 * w / 8
      for (a <- 0 until 64; lane <- 0 until gateEnvBytes / 4)
        writeHost(BigInt(dut.map.envOffset(0, 0)) + a.toLong * gateEnvBytes + lane * 4, BigInt((a * 977 + lane * 131 + 7) & 0x7FFF7FFF))
      for (c <- 0 until 2; a <- 0 until 64) writeHost(BigInt(dut.map.envOffset(c, 2)) + a.toLong * 4, BigInt(0x7FFF))
      for (c <- 0 until 2; a <- 0 until 64) writeHost(BigInt(dut.map.envOffset(c, 1)) + a.toLong * 4, BigInt((a * 409 + c * 77 + 3) & 0x7FFF7FFF))

      // batch time wraps past 2^32 about 3000 batches into run 1 (written while the cores are held in reset)
      writeHost(hostCtrl + 64, BigInt(0xFFFFFFFFL - 6000))
      writeHost(hostCtrl + 68, BigInt(0))

      // ADC: a free-running tone phase-locked to batch time, a different amplitude per core
      @volatile var adcAmp = 20000
      fork { while (true) {
        val t = dut.riscqArea.time.toBigInt.toLong
        for ((adc, k0) <- Seq((0, 0), (1, 1))) {
          var word = BigInt(0)
          for (k <- 0 until adcN) {
            val s = t * adcN + k
            val ang = scala.math.Pi * (4096.0 / 32768.0) * s + 0.7 * k0
            word |= BigInt(scala.math.round((adcAmp - 3000 * k0) * scala.math.cos(ang)).toInt & 0xFFFF) << (k * w)
          }
          dut.io.adc(adc).payload #= word
        }
        dspCd.waitSampling()
      } }
      // DIO inputs on core 0's ttl bank: edges at a deterministic cadence
      val dioIdx = PulseTableSoc.dioNames(spec).indexOf("q0_ttl")
      require(dioIdx >= 0)
      fork { val r = new Random(5); while (true) { dspCd.waitSampling(31 + r.nextInt(50)); dut.io.dioIn(dioIdx) #= r.nextInt(1 << 16) } }

      implicit val idCb: IdCallback = new IdCallback
      val agents = (0 until 2).map(c => new MasterAgent(dut.riscqArea.testMasters(c).node.bus, dspCd)(new IdAllocator(DebugId.width)))
      def put(core: Int, a: Int, v: Int): Unit = agents(core).putFullData(0, a, leBytes(BigInt(v & 0xFFFFFFFFL), 4))
      def get(core: Int, a: Int, what: String): BigInt = {
        val v = BigInt(agents(core).getInt(0, a)) & 0xFFFFFFFFL
        log.println(s"T $dcyc $what $v"); v
      }
      def now(): Int = dut.riscqArea.time.toBigInt.toInt

      def uplinkArm(base: Long): Unit = {
        dwr(STATUS, BigInt(STICKY_MASK)); dwr(WR_BASE, BigInt(base)); dwr(BASE_RESET, BigInt(1))
        var n = 0
        while (!(bit(drd(STATUS), S_RUN_ACTIVE) && bit(drd(STATUS), S_DSP_ADMIT)) && n < 400) { ddrCd.waitSampling(10); n += 1 }
        assert(bit(drd(STATUS), S_RUN_ACTIVE), "uplink run never became active")
      }
      def uplinkFlushDrain(base: Long): Unit = {
        dwr(FLUSH, BigInt(1))
        var m = 0; var st = drd(STATUS)
        while (bit(st, S_FLUSH_BUSY) && m < 8000) { ddrCd.waitSampling(20); st = drd(STATUS); m += 1 }
        log.println(s"U $ucyc status ${st.toString(16)}")
        val nbytes = (drd(FINAL_ADDR) - base).toInt
        log.println(s"U $ucyc final $nbytes")
        if (nbytes > 0) {
          dwr(RD_BASE, BigInt(base)); dwr(RD_SIZE, BigInt(nbytes)); dwr(RD_START, BigInt(1))
          var n = 0; while (n < 4000 && !(dut.ddrUplink.rd.valid.toBoolean && dut.ddrUplink.rd.last.toBoolean)) { ddrCd.waitSampling(); n += 1 }
          ddrCd.waitSampling(20)
        }
      }

      // the tap's schedule for one run: readouts, gate fires across the margin, a full queue, DIO, offsets
      def tapTraffic(seed: Int, cycles: Int): Unit = {
        val r = new Random(seed)
        val t0 = now()
        var k = 0
        while (now() - t0 < cycles) {
          val core = k % 2
          (k % 6) match {
            case 0 => // demod readout: nonzero carrier + a phase offset
              val st = now() + 150
              put(core, 0x3000C, w16(r.nextInt(1 << 16)))      // phaseOffset
              put(core, 0x30004, w16(4096))                      // freq
              put(core, 0x34100, st)                             // startTime
              put(core, 0x30010, w16(r.nextInt(4000))); put(core, 0x30014, w16(12000))
              put(core, 0x30018, w16(0)); put(core, 0x3001C, w16(20 + r.nextInt(8)))
              put(core, 0x30000, 0)                              // fire = the readout
              put(core, 0x3000C, w16(r.nextInt(1 << 16)))      // phaseOffset right after the fire
              dspCd.waitSampling(260)
              get(core, 0x4200, s"c${core}res"); get(core, 0x4204, s"c${core}real"); get(core, 0x4208, s"c${core}imag")
            case 1 => // gate + ro fires swept across the push-margin boundary (deadline -1 .. +1 and beyond)
              val gc = (k / 6) % 2
              val lead = 24 + (k / 6) % 48
              put(gc, 0x10010, w16(r.nextInt(1 << 16))); put(gc, 0x10014, w16(9000 + r.nextInt(9000)))
              put(gc, 0x10018, w16(0)); put(gc, 0x1001C, w16(12))
              put(gc, 0x10004, w16(1024 + 256 * r.nextInt(8)))
              put(gc, 0x14100, now() + lead)
              put(gc, 0x10000, 0)
              put(gc, 0x10008, w16(r.nextInt(512)))             // dcOffset right after the fire
              // the readout drive (traced into robs): ro table entry 0, fired at the same lead
              put(gc, 0x20010, w16(r.nextInt(1 << 16))); put(gc, 0x20014, w16(6000 + r.nextInt(6000)))
              put(gc, 0x20018, w16(0)); put(gc, 0x2001C, w16(16))
              put(gc, 0x20004, w16(2048))
              put(gc, 0x24100, now() + lead + 20)
              put(gc, 0x20000, 0)
              put(gc, 0x2000C, w16(r.nextInt(1 << 16)))        // phaseOffset right after the fire
            case 2 => // five fires into a 4-deep queue, far in the future: the fifth is dropped
              put(1, 0x14100, now() + 900)
              for (_ <- 0 until 5) put(1, 0x10000, r.nextInt(2))
            case 3 => // DIO train on core 0's ttl (node 3): slot 0/1 set/clear, dur 6
              put(0, 0x40010, w16(0xFFFF)); put(0, 0x40014, w16(r.nextInt(1 << 16))); put(0, 0x4001C, w16(6))
              put(0, 0x40020, w16(0xFFFF)); put(0, 0x40024, w16(0)); put(0, 0x4002C, w16(6))
              put(0, 0x44100, now() + 120)
              put(0, 0x40000, 0); put(0, 0x40000, 1); put(0, 0x40000, 0)
            case 4 => // random fields of a random channel window (garbage addresses included)
              val node = r.nextInt(4); val a = (node + 1) << 16 | (r.nextInt(0x50) & ~3)
              if (!(core == 1 && node == 3)) put(core, a, r.nextInt())
            case 5 => dspCd.waitSampling(40 + r.nextInt(80))
          }
          k += 1
        }
      }

      def releaseReset(): Unit = { writeHost(hostCtrl, 1); hostCd.waitSampling(20); writeHost(hostCtrl, 0); hostCd.waitSampling(60) }
      def holdReset(): Unit = { writeHost(hostCtrl, 1); hostCd.waitSampling(40) }

      // ── run 1: pulse_sched on core 0, core 1 parked; the tap's schedule on both ──
      loadElf(0, "src/riscq/soc/sim/sw/pulse_sched.elf"); park(1)
      uplinkArm(0x2000L)
      releaseReset()
      tapTraffic(1, 9000)
      for (c <- 0 until 2) put(c, 0x4010, 1)                     // DONE: the program's last store
      dspCd.waitSampling(50)
      readHost(hostCtrl + 0x50, "done1")
      for (a <- 0 until 32) readHost(BigInt(dut.map.robBase) + 4L * a, s"robs$a")
      holdReset()
      readHost(hostCtrl + 0x50, "done1-after-reset")
      uplinkFlushDrain(0x2000L)

      // ── run 2: xcore on both cores (publish, barrier, mailbox, conditional fire) ──
      for (c <- 0 until 2) loadElf(c, "src/riscq/soc/sim/sw/xcore.elf")
      uplinkArm(0x40000L)
      releaseReset()
      tapTraffic(2, 4000)
      dspCd.waitSampling(3000)
      for (c <- 0 until 2) put(c, 0x4010, 1)
      dspCd.waitSampling(50)
      readHost(hostCtrl + 0x50, "done2")
      holdReset()
      uplinkFlushDrain(0x40000L)

      log.println(s"END dsp=$dcyc ddr=$ucyc host=$hcyc")
      log.close()
      println(s"[PipeTraceSim] timingPipe=$pipe: trace written to $out ($dcyc dsp cycles)")
      simSuccess()
    }
}
