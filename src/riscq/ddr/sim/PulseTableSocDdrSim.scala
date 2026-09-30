package riscq.ddr.sim

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import spinal.lib.bus.amba4.axi.sim.{AxiMemorySim, AxiMemorySimConfig, Axi4Master}
import spinal.lib.bus.tilelink.sim.{MasterAgent, IdAllocator, IdCallback}
import spinal.lib.bus.tilelink.DebugId
import riscq.soc.PulseTableSoc
import riscq.soc.link.EventLink
import riscq.soc.spec.{ChannelSpec, SocSpec, SocSpecMap}
import riscq.ddr.ReadoutDdrRegs._
import scala.collection.mutable
import scala.util.Random

/**
 * G3 — SoC-level EQUIVALENCE on the antq_uplink build: a real ADC tone, demodulated by the real
 * `ReadoutDecoder` inside `PulseTableSoc`, must land in DDR as a tagged word whose I/Q is bit-exactly the
 * CPU-visible `real`/`imag` truncated to the 28-bit DDR fields, one-to-one and in order per core.
 *
 * The claim of the uplink is "the DDR copy IS the result the CPU sees", so it is checked against the
 * CPU's own `ReadoutResultSink` read-back, not against a model. The stimulus is the VNA-style tone
 * `PulseTableSocSim` uses (matched and detuned LO), so the decoder is genuinely running.
 *
 * P3b (plan v2 r2 #8 / #10) on top of the P3a gate:
 *   - the uplink taps the decoder's level-valued `ReadoutResultLink.source`, and the EventLink up-link of
 *     the same cores carries concurrent traffic the tap must NOT count: timed-DIO input events (the hot
 *     cores get a `ttl` dio channel whose inputs toggle all the time) and hub broadcasts (every other core
 *     publishes to the board groups, which the hub re-puts to every core's `board` sink);
 *   - every result of a core is a distinct value (per-window ADC amplitude), so the per-core order and a
 *     duplicate or a drop are visible, not just the count;
 *   - two consecutive runs (BASE_RESET = the next prepare) at different bases, with delayed write
 *     responses (the memory answers B 150 cycles late) and a stalled drain (random AXIS TREADY);
 *   - the host-domain DDR status word at HOST_DDR_STATUS (0x58) is readable while the DDR side is dead.
 *
 * Run: mill-1.1.0 runMain riscq.ddr.sim.PulseTableSocDdrSim
 */
object PulseTableSocDdrSim extends App {
  // the PRODUCTION geometry (14 qubits) with more than one core producing readouts, one per ADC group
  val qubitNum = 14
  val dacMap   = riscq.soc.SocChannelMap.dacMap(qubitNum)
  val adcMap   = riscq.soc.SocChannelMap.adcMap(qubitNum)
  val hotCores = Seq(0, 7)
  val hubCores = (0 until qubitNum).filterNot(hotCores.contains)   // publishers of the hub traffic
  val N = 16; val w = 16; val adcN = 4
  val JAL_SELF = BigInt("6f", 16)
  val BASES    = Seq(0x2000L, 0x40000L)          // run 1, run 2
  val WINDOWS  = 4                               // matched windows per hot core per run (+1 detuned)
  val B_DELAY  = 150                             // memory write-response delay, DDR cycles

  val resAddr = 0x4200; val realAddr = 0x4204; val imagAddr = 0x4208
  val demodStAddr = 0x34100
  val HOST_DDR_STATUS = 0x58

  def leBytes(v: BigInt, n: Int): List[Byte] = List.tabulate(n)(i => ((v >> (8 * i)) & 0xFF).toByte)
  def w16(v: Int): Int = ((v & 0xFFFF) << 16) & 0xFFFFFFFF

  // the qubit build plus a timed-DIO channel on each hot core (its input events share the up-link)
  val spec = {
    val q = SocSpec.qubits(qubitNum, dacMap, adcMap)
    q.copy(resultsPath = SocSpec.AntqUplink, cores = q.cores.zipWithIndex.map { case (c, i) =>
      if (hotCores.contains(i)) c.copy(channels = c.channels :+ ChannelSpec("ttl", "dio", 8, 0, 1, None, None, trace = false))
      else c })
  }

  SimConfig.addSimulatorFlag("-Wno-MULTIDRIVEN").addSimulatorFlag("--x-initial 0")
    .compile {
      val soc = new PulseTableSoc(spec, withTest = true)
      soc.ddrUplink.up.io.dspAdmit.simPublic()
      soc.ddrUplink.calibDone.simPublic()
      for (c <- hotCores) soc.riscqArea.riscqCores(c).posted.upSrc.simPublic()
      soc
    }
    .doSim("soc_ddr_equivalence", seed = 7) { dut =>
      SimTimeout(400000000)
      val rng    = new Random(7)
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

      // ── the DDR status register must be readable while the DDR side is DEAD (the case it exists for):
      // start the HOST domain only, hold ddrRst asserted and calibDone low, read it over the host bus ──
      dut.ddrUplink.calibDone #= false
      hostCd.forkStimulus(10)
      ddrCd.assertReset()                       // ddrClk is not even toggling yet
      hostCd.waitSampling(40)
      val axi = Axi4Master(dut.io.axi, hostCd)
      val ddrStatusAddr = BigInt(dut.map.hostCtrlBase) + HOST_DDR_STATUS
      def readHost(addr: BigInt): BigInt = {
        var r: Option[BigInt] = None
        axi.readSingle(addr, 4) { bs => r = Some(bs.reverse.foldLeft(BigInt(0))((a, b) => (a << 8) | (b & 0xff))) }
        var n = 0; while (r.isEmpty && n < 20000) { hostCd.waitSampling(); n += 1 }
        assert(r.isDefined, f"host read 0x$addr%x timed out"); r.get
      }
      val dead = readHost(ddrStatusAddr)
      assert((dead >> 16) == 0xCA1B, f"DDR status magic missing while the DDR side is dead: 0x$dead%x")
      assert(((dead >> 0) & 1) == 0, f"calib_done should be 0 with calibDone held low: 0x$dead%x")
      assert(((dead >> 1) & 1) == 0, f"ui_reset_released should be 0 with ddrRst asserted: 0x$dead%x")
      assert(readHost(BigInt(dut.map.hostCtrlBase) + 0x5C) == 0, "0x5C (STOP, reserved) must read 0")
      println(f"[G3] DDR status at +0x58 readable with the DDR side DEAD: 0x$dead%08x (magic ok, calib=0, ui_rst=0); STOP +0x5C reads 0")

      dspCd.forkStimulus(10); ddrCd.forkStimulus(14)
      dut.ddrUplink.calibDone #= true
      hostCd.waitSampling(60)
      val alive = readHost(ddrStatusAddr)
      assert((alive >> 16) == 0xCA1B && (alive & 3) == 3, f"DDR status after bring-up: 0x$alive%x")
      println(f"[G3] DDR status after bring-up: 0x$alive%08x (calib=1, ui_rst=1)")

      // delayed writes: every B comes B_DELAY DDR cycles after its burst
      val mem  = AxiMemorySim(dut.ddrUplink.ddr, ddrCd, AxiMemorySimConfig(maxOutstandingWrites = 4,
        writeResponseDelay = B_DELAY))
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

      // ── boot the SoC exactly as PulseTableSocSim does (cores parked, riscqReset released: the hub runs) ──
      def loadInstr(core: Int, word: Int, v: BigInt): Unit =
        axi.write(BigInt(dut.map.coreMemOffset(core)) + word.toLong * 4, leBytes(v, 4))
      for (c <- 0 until qubitNum) loadInstr(c, 0, JAL_SELF)
      val hostCtrlAddr = BigInt(dut.map.hostCtrlBase)
      axi.write(hostCtrlAddr, List(0x01, 0, 0, 0).map(_.toByte)); hostCd.waitSampling(20)
      axi.write(hostCtrlAddr, List(0x00, 0, 0, 0).map(_.toByte)); hostCd.waitSampling(60)
      val tA = dut.riscqArea.time.toBigInt.toInt; dspCd.waitSampling(50)
      assert(dut.riscqArea.time.toBigInt.toInt != tA, "batch time not advancing")

      // a core-local TileLink master per core: the hot cores read their ReadoutResultSink and program the
      // demod; the others publish to the board groups (hub traffic onto EVERY core's up-link)
      // one debug-id pool per agent: the publishers run concurrently, and one shared pool of
      // 2^DebugId.width ids runs dry (each core's bus is separate, so per-bus ids suffice)
      implicit val idCb: IdCallback = new IdCallback
      val agents = (0 until qubitNum).map(c =>
        new MasterAgent(dut.riscqArea.testMasters(c).node.bus, dspCd)(new IdAllocator(DebugId.width)))
      val src = 0
      def signed32(u: BigInt): BigInt = { val m = BigInt(1) << 32; val r = ((u % m) + m) % m; if (r >= (BigInt(1) << 31)) r - m else r }
      def wrC(core: Int, a: Int, v: Int): Unit = agents(core).putFullData(src, a, leBytes(BigInt(v & 0xFFFFFFFFL), 4))

      for (c <- hotCores; a <- 0 until 64) axi.write(BigInt(dut.map.envOffset(c, 2)) + a.toLong * 4, leBytes(BigInt(0x7FFF), 4))

      // Free-running ADC tone, phase-locked to batch time. The amplitude is set per hot core AND per
      // window, so every result of a core is a distinct number: a drop, a duplicate or a reorder in the
      // uplink changes the sequence, not only the count.
      val Fcarrier = 4096; val Fdetuned = 12288
      val adcIds = hotCores.map(adcMap)
      assert(adcIds.distinct.size == adcIds.size, s"hot cores share an ADC: ${adcIds.mkString(",")}")
      val amp = mutable.ArrayBuffer.fill(hotCores.size)(20000)
      def adcWord(t: Long, f: Int, a: Int): BigInt = {
        var word = BigInt(0)
        for (k <- 0 until adcN) {
          val s = t * adcN + k
          val ang = scala.math.Pi * (f.toDouble / 32768.0) * s
          word |= BigInt(scala.math.round(a * scala.math.cos(ang)).toInt & 0xFFFF) << (k * w)
        }
        word
      }
      var adcFreq = Fcarrier
      fork { while (true) {
        val t = dut.riscqArea.time.toBigInt.toLong
        for (ci <- hotCores.indices) dut.io.adc(adcIds(ci)).payload #= adcWord(t, adcFreq, amp(ci))
        dspCd.waitSampling()
      } }

      // ── concurrent up-link traffic the tap must not count ──
      @volatile var traffic = false
      // hub: the non-hot cores publish to groups 0..3 ({slot, bit}), which the hub re-puts to every core
      for (c <- hubCores) fork {
        val r = new Random(100 + c)
        while (true) {
          dspCd.waitSampling(60 + r.nextInt(80))
          if (traffic) {
            val g = r.nextInt(SocSpecMap.groupNodes)
            wrC(c, SocSpecMap.putWindow + (SocSpecMap.groupNode(g) << 16), (c << 1) | r.nextInt(2))
          }
        }
      }
      // DIO: the hot cores' ttl inputs toggle; every change posts an event onto that core's up-link
      val dioIdx = hotCores.map(c => PulseTableSoc.dioNames(spec).indexOf(s"q${c}_ttl"))
      assert(dioIdx.forall(_ >= 0), s"dio ports ${PulseTableSoc.dioNames(spec)}")
      fork {
        val r = new Random(99)
        while (true) {
          dspCd.waitSampling(25 + r.nextInt(40))
          if (traffic) for (k <- dioIdx) dut.io.dioIn(k) #= r.nextInt(1 << 16)
        }
      }
      // count what the hot cores' up-links carry, by sink: the demod result sink vs everything else
      val demodPuts = Array.fill(hotCores.size)(0L); val dioPuts = Array.fill(hotCores.size)(0L)
      val hubPuts = Array.fill(hotCores.size)(0L)
      def sinkOff(c: Int, kind: String): Int =
        dut.riscqArea.riscqCores(c).eventPlan.sinks.find(_.kind == kind).get.base - EventLink.sinkBase
      val demodOff = hotCores.map(c => sinkOff(c, EventLink.resultKind))
      val dioOff   = hotCores.map(c => sinkOff(c, EventLink.fifoKind))
      fork { while (true) {
        dspCd.waitSampling()
        for ((c, ci) <- hotCores.zipWithIndex) {
          val u = dut.riscqArea.riscqCores(c).posted.upSrc
          if (u.valid.toBoolean) {
            val a = u.payload.address.toInt
            if ((a >> 5) == (demodOff(ci) >> 5)) demodPuts(ci) += 1
            else if ((a >> 5) == (dioOff(ci) >> 5)) dioPuts(ci) += 1
            else hubPuts(ci) += 1
          }
        }
      } }

      // ── one run: BASE_RESET (the "prepare"), windows, flush, stalled drain, one-to-one compare ──
      val demodAmp = 12000; val roDur = 20
      def runWindow(ci: Int, label: String, seen: mutable.ArrayBuffer[(BigInt, BigInt)]): Double = {
        val core = hotCores(ci)
        val st = dut.riscqArea.time.toBigInt.toInt + 200
        wrC(core, demodStAddr, st)
        wrC(core, 0x30004, w16(Fcarrier))
        wrC(core, 0x30010, w16(0)); wrC(core, 0x30014, w16(demodAmp))
        wrC(core, 0x30018, w16(0)); wrC(core, 0x3001C, w16(roDur))
        wrC(core, 0x30000, 0)                                  // fire the demod = the readout
        waitUntil(dut.riscqArea.time.toBigInt.toInt >= st + roDur + 60)
        agents(core).getInt(src, resAddr)                      // halts until the integral settles
        val re = signed32(agents(core).getInt(src, realAddr))
        val im = signed32(agents(core).getInt(src, imagAddr))
        seen += ((re, im))
        val mag = scala.math.sqrt((re * re + im * im).toDouble)
        println(f"[G3]   core $core%2d window $label%-9s amp=${amp(ci)}%5d: CPU real=$re imag=$im |z|=${mag.toLong}")
        mag
      }

      def drain(base: Long, nBytes: Int): Seq[BigInt] = {
        dwr(RD_BASE, BigInt(base)); dwr(RD_SIZE, BigInt(nBytes))
        val beats = mutable.ArrayBuffer[BigInt](); var sawLast = false; var stalled = 0
        val sink = fork {
          val r = new Random(base)
          while (!sawLast) {
            dut.ddrUplink.rd.ready #= r.nextInt(3) == 0         // TREADY low 2/3 of the time
            ddrCd.waitSampling()
            if (dut.ddrUplink.rd.valid.toBoolean) {
              if (dut.ddrUplink.rd.ready.toBoolean) {
                beats += dut.ddrUplink.rd.fragment.toBigInt
                if (dut.ddrUplink.rd.last.toBoolean) sawLast = true
              } else stalled += 1
            }
          }
          dut.ddrUplink.rd.ready #= true
        }
        dwr(RD_START, BigInt(1))
        var n = 0; while (!sawLast && n < 40000) { ddrCd.waitSampling(10); n += 1 }
        assert(sawLast, "TLAST never arrived")
        sink.join()
        assert(beats.size == nBytes / 32, s"${beats.size} AXIS beats for $nBytes B")
        assert(stalled > 0, "the drain was never stalled")
        println(s"[G3]   drain of $nBytes B: ${beats.size} beats, TREADY stalled on $stalled beat-cycles")
        beats.flatMap(b => (0 until 4).map(k => (b >> (64 * k)) & ((BigInt(1) << 64) - 1))).toSeq
      }

      def oneRun(r: Int): Unit = {
        val base = BASES(r)
        println(s"[G3] run ${r + 1}: BASE_RESET at 0x${base.toHexString}")
        dwr(STATUS, BigInt(STICKY_MASK)); dwr(WR_BASE, BigInt(base)); dwr(BASE_RESET, BigInt(1))
        var n = 0
        while (!(bit(drd(STATUS), S_RUN_ACTIVE) && bit(drd(STATUS), S_DSP_ADMIT)) && n < 400) { ddrCd.waitSampling(10); n += 1 }
        assert(bit(drd(STATUS), S_RUN_ACTIVE), f"run never became active: 0x${drd(STATUS)}%x")
        // (ACCEPTED is the flush snapshot, so it still shows the previous run here; that the new run's
        // counts start from zero is checked after its own flush: accepted == this run's windows)

        val seen = Array.fill(qubitNum)(mutable.ArrayBuffer[(BigInt, BigInt)]())
        val d0 = demodPuts.clone(); val i0 = dioPuts.clone(); val h0 = hubPuts.clone()
        traffic = true
        // interleave the hot cores window by window, each window at a new amplitude
        val mags = Array.fill(hotCores.size)(mutable.ArrayBuffer[Double]())
        adcFreq = Fcarrier
        for (k <- 0 until WINDOWS; ci <- hotCores.indices) {
          amp(ci) = 26000 - 9000 * ci - 2200 * k - 700 * r
          mags(ci) += runWindow(ci, s"matched$k", seen(hotCores(ci)))
        }
        adcFreq = Fdetuned; dspCd.waitSampling(60)
        val detuned = hotCores.indices.map(ci => runWindow(ci, "detuned", seen(hotCores(ci))))
        adcFreq = Fcarrier
        traffic = false
        dspCd.waitSampling(400)                                 // let the last hub/DIO puts drain

        for (ci <- hotCores.indices) {
          assert(mags(ci).forall(_ > 100000), s"core ${hotCores(ci)} matched magnitudes ${mags(ci)} too small")
          assert(mags(ci).min > 4 * detuned(ci), s"core ${hotCores(ci)}: no selectivity")
          // distinguishable: every matched window differs from the previous one by >= 3 %
          for (k <- 1 until WINDOWS)
            assert(scala.math.abs(mags(ci)(k) - mags(ci)(k - 1)) > 0.03 * mags(ci)(k - 1),
              s"core ${hotCores(ci)}: windows ${k - 1} and $k indistinguishable (${mags(ci)})")
        }
        for (ci <- hotCores.indices) {
          val dp = demodPuts(ci) - d0(ci); val ip = dioPuts(ci) - i0(ci); val hp = hubPuts(ci) - h0(ci)
          println(s"[G3]   core ${hotCores(ci)} up-link during run ${r + 1}: $dp demod-sink puts, $ip DIO-event puts, $hp hub puts")
          assert(ip > dp && hp > dp, s"core ${hotCores(ci)}: too little concurrent up-link traffic (dio $ip, hub $hp vs demod $dp)")
        }

        // flush: quiet -> snapshot -> final bank -> its (delayed) B -> write_done
        dwr(FLUSH, BigInt(1))
        var m = 0; var st = drd(STATUS)
        while (bit(st, S_FLUSH_BUSY) && m < 8000) { ddrCd.waitSampling(20); st = drd(STATUS); m += 1 }
        assert(!bit(st, S_FLUSH_BUSY), "flush never completed")
        assert(bit(st, S_WRITE_DONE), f"write_done not set: 0x$st%x")
        val fatal = Seq(S_OVF_ANY, S_WRAPPED, S_BRESP_ERR, S_RRESP_ERR, S_EARLY_LATE, S_CROSS_DROPPED, S_SKID_OVF,
          S_ERR_START_DROPPED, S_ERR_FLUSH_DROPPED, S_AXI_RST_FAULT)
        for (b <- fatal) assert(!bit(st, b), f"fatal bit $b set: 0x$st%x")

        val acc = (0 until qubitNum).map(i => drd(ACCEPTED + 4 * i).toInt)
        val total = acc.sum
        for (c <- 0 until qubitNum) {
          assert(acc(c) == seen(c).size, s"run ${r + 1} core $c: accepted ${acc(c)}, the CPU read ${seen(c).size} windows")
          assert(drd(REJECTED + 4 * c) == 0, s"core $c had rejections")
        }
        val nbytes = (drd(FINAL_ADDR) - base).toInt
        assert(nbytes == 32 * ((total + 3) / 4), s"final_addr - base = $nbytes for $total results")

        // the drain (stalled AXIS) must equal the DDR image, and the DDR image the CPU's values
        val drained = drain(base, nbytes).take(total)
        val bytes = mem.memory.readArray(base, total.toLong * 8)
        val words = (0 until total).map(k => (0 until 8).foldLeft(BigInt(0))((a, b) => a | (BigInt(bytes(k * 8 + b) & 0xff) << (8 * b))))
        assert(drained == words, "the AXIS drain differs from the DDR image")
        def trunc(v: BigInt): BigInt = (v & 0xFFFFFFFFL) >> 4
        for (c <- 0 until qubitNum) {
          val got = words.filter(x => ((x >> 56) & 0xff) == c)
          assert(got.size == seen(c).size, s"tag $c: ${got.size} words in DDR, the CPU saw ${seen(c).size}")
          for (((wd, (re, im)), k) <- got.zip(seen(c)).zipWithIndex) {
            assert(((wd >> 28) & 0x0FFFFFFF) == trunc(re), f"run ${r + 1} core $c word $k real: DDR 0x${(wd >> 28) & 0x0FFFFFFF}%x != CPU 0x${trunc(re)}%x")
            assert((wd & 0x0FFFFFFF) == trunc(im), f"run ${r + 1} core $c word $k imag: DDR 0x${wd & 0x0FFFFFFF}%x != CPU 0x${trunc(im)}%x")
          }
        }
        println(s"[G3] run ${r + 1}: $total CPU-visible results (${hotCores.map(c => s"core $c: ${seen(c).size}").mkString(", ")}) " +
                s"match DDR and the stalled drain one-to-one and in order; $nbytes B at 0x${base.toHexString}")
      }

      for (r <- BASES.indices) oneRun(r)
      println(s"[G3] PASS: 2 consecutive runs at ${qubitNum} qubits, ${WINDOWS + 1} distinct results per hot core per run, " +
              s"exact per-core order in DDR and in the drain, with concurrent hub + DIO up-link traffic, B delayed " +
              s"$B_DELAY cycles and TREADY stalled; DDR status at +0x58 live with the DDR side dead")
      simSuccess()
    }
}
