package riscq.ddr.sim

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import spinal.lib.bus.amba4.axi.sim.{AxiMemorySim, AxiMemorySimConfig, Axi4Master}
import riscq.ddr._
import scala.collection.mutable
import scala.util.Random

/**
 * G2 — uplink-level verification of [[ReadoutDdrUplink]] against a behavioural AXI memory, with the
 * two clock domains at co-prime periods. Covers the run protocol, the loss model (overflow/accepted/
 * rejected accounting), the flush/drain contract, the injector, register validation and the error
 * stickies. Run: mill-1.1.0 runMain riscq.ddr.sim.ReadoutDdrUplinkSim
 */
object ReadoutDdrUplinkSim extends App {
  import ReadoutDdrRegs._

  // P3b: `runMain riscq.ddr.sim.ReadoutDdrUplinkSim stalls` runs every scenario again with random AW / AR /
  // B / W stalls injected at the memory (StallInjector); without the argument it is the P3a G2 run.
  val STALLS  = args.contains("stalls")
  val stallTotals = Array.fill(4)(0L)   // AW, AR, B, W stall cycles the uplink saw, over all scenarios
  val NCH     = 4   // local channel count; NOTE: do NOT call it NCH — that is a register offset                      // small enough for fast sims; geometry is derived, not literal
  val BASE0   = 0x1000L
  val p       = ReadoutDdrUplinkParams(numCh = NCH)

  /** Host-side model of the DDR word format (ground truth: QubiC ddr_readout_data.py). */
  def tagWord(tag: Int, real: Int, imag: Int): BigInt = {
    val r = BigInt(real & 0xFFFFFFFFL) >> 4
    val i = BigInt(imag & 0xFFFFFFFFL) >> 4
    (BigInt(tag) << 56) | (r << 28) | i
  }
  def splitWord(w: BigInt): (Int, BigInt, BigInt) =
    (((w >> 56) & 0xFF).toInt, (w >> 28) & 0x0FFFFFFF, w & 0x0FFFFFFF)

  // P3c-2: the running scenario's stall injector, so a scenario can take the stall pins over (`enabled = false`)
  var curStalls: StallInjector = null
  def run(name: String, seed: Int, nch: Int = NCH, memDelay: Int = 0, rejW: Int = 16)(body: (ReadoutDdrUplinkDut, Helper) => Unit): Unit = {
    val pp = ReadoutDdrUplinkParams(numCh = nch, rejectedWidth = rejW)
    SimConfig.withConfig(SpinalConfig()).addSimulatorFlag("-Wno-MULTIDRIVEN").addSimulatorFlag("--x-initial 0")
      .compile(ReadoutDdrUplinkDut(pp))
      .doSim(name, seed = seed) { dut =>
        SimTimeout(20000000)
        val ddrCd = ClockDomain(dut.io.ddrClk, dut.io.ddrRst)
        val dspCd = ClockDomain(dut.io.dspClk, dut.io.dspRst)
        ddrCd.forkStimulus(3)     // ~333 MHz
        dspCd.forkStimulus(2)     // ~500 MHz  (co-prime)
        for (i <- 0 until nch) { dut.io.results(i).valid #= false; dut.io.results(i).payload.res #= false
          dut.io.results(i).payload.real #= 0; dut.io.results(i).payload.imag #= 0 }
        dut.io.rd.ready #= true
        dut.io.wStall #= false
        dut.io.rStall #= false; dut.io.bErr #= false; dut.io.rErr #= false
        val stalls = new StallInjector(dut, ddrCd, if (STALLS) StallProfile.heavy else StallProfile.none, seed)
        curStalls = stalls
        val mem = AxiMemorySim(dut.io.ddr, ddrCd, AxiMemorySimConfig(
          maxOutstandingReads = 2, maxOutstandingWrites = 2,
          readResponseDelay = memDelay, writeResponseDelay = memDelay))
        mem.start()
        // P3a r2: generic AXI4/AXIS valid-ready protocol monitor on the uplink's DDR master and AXIS drain
        val mons = AxiProtocolMonitor(dut.up.io.ddr, dut.io.rd, ddrCd, () => dut.up.ddrURst.toBoolean)
        val ctrl = Axi4Master(dut.io.ctrl, ddrCd, "ctrl")
        ddrCd.waitSampling(20); dspCd.waitSampling(20)
        body(dut, new Helper(dut, ddrCd, dspCd, ctrl, mem, new Random(seed), nch))
        AxiProtocolMonitor.check(mons, name)
        stallTotals(0) += stalls.awStalled; stallTotals(1) += stalls.arStalled
        stallTotals(2) += stalls.bStalled;  stallTotals(3) += stalls.wStalled
        println(s"[G2] PASS $name (AXI protocol monitor clean; stall cycles ${AxiProtocolMonitor.summary(mons)}" +
                (if (STALLS) s"; ${stalls.summary})" else ")"))
      }
  }

  class Helper(dut: ReadoutDdrUplinkDut, val ddrCd: ClockDomain, val dspCd: ClockDomain,
               val ctrl: Axi4Master, val mem: AxiMemorySim, val rng: Random, val nch: Int) {
    def le(v: BigInt, n: Int): List[Byte] = List.tabulate(n)(i => ((v >> (8 * i)) & 0xff).toByte)
    // Axi4Master queues transactions and returns immediately: both helpers must BLOCK until the
    // transaction actually completed, otherwise every read returns its initial 0 (cost: 1 debug round).
    def wr(off: Int, v: BigInt): Unit = {
      var done = false
      ctrl.writeCB(off, le(v, 4)) { done = true }
      var n = 0
      while (!done && n < 5000) { ddrCd.waitSampling(); n += 1 }
      assert(done, s"ctrl write to 0x${off.toHexString} never completed")
    }
    /** Non-blocking control write: queues the transaction and returns. Needed when a property must be
     *  exercised INSIDE a short hardware window (e.g. "refused while inj_busy") — a blocking write takes
     *  longer than the window itself. */
    def wrNb(off: Int, v: BigInt): Unit = ctrl.write(off, le(v, 4))
    def rd(off: Int): BigInt = {
      var r: Option[BigInt] = None
      ctrl.readSingle(off, 4) { bs => r = Some(bs.reverse.foldLeft(BigInt(0))((a, b) => (a << 8) | (b & 0xff))) }
      var n = 0
      while (r.isEmpty && n < 5000) { ddrCd.waitSampling(); n += 1 }
      assert(r.isDefined, s"ctrl read from 0x${off.toHexString} never completed")
      r.get
    }
    def status(): BigInt = rd(STATUS)
    def bit(s: BigInt, b: Int): Boolean = ((s >> b) & 1) == 1

    def startRun(base: Long = BASE0): Unit = {
      wr(WR_BASE, BigInt(base)); wr(BASE_RESET, BigInt(1))
      var n = 0
      while (!bit(status(), S_RUN_ACTIVE) && n < 200) { ddrCd.waitSampling(10); n += 1 }
      if (!bit(status(), S_RUN_ACTIVE)) {
        val s = status()
        println(f"[G2-DBG] status=0x${s}%x run_active=${bit(s,S_RUN_ACTIVE)} err_base_busy=${bit(s,S_ERR_BASE_BUSY)} " +
                f"err_start_dropped=${bit(s,S_ERR_START_DROPPED)} cross_dropped=${bit(s,S_CROSS_DROPPED)} " +
                f"flush_busy=${bit(s,S_FLUSH_BUSY)} rd_busy=${bit(s,S_RD_BUSY)} dsp_in_reset=${bit(s,S_DSP_IN_RESET)} " +
                f"dsp_admit=${bit(s,S_DSP_ADMIT)} run_base=0x${rd(RUN_BASE)}%x wr_base=0x${rd(WR_BASE)}%x")
        val d = rd(DIAG)
        println(f"[G2-DBG] diag=0x${d}%x writer_idle=${bit(d,0)} rd_empty=${bit(d,1)} able_to_read=${bit(d,2)} " +
                f"start_busy=${bit(d,3)} start_pend=${bit(d,4)} flush_busy=${bit(d,5)} write_done_seen=${bit(d,6)} " +
                f"run_idle=${bit(d,7)} snap=${bit(d,8)}")
      }
      assert(bit(status(), S_RUN_ACTIVE), "run never became active")
      var m = 0
      while (!dut.io.dspAdmit.toBoolean && m < 200) { dspCd.waitSampling(10); m += 1 }
      assert(dut.io.dspAdmit.toBoolean, "dsp_admit never rose")
    }
    def flushRun(): BigInt = {
      wr(FLUSH, BigInt(1))
      var n = 0; var s = status()
      while (bit(s, S_FLUSH_BUSY) && n < 4000) { ddrCd.waitSampling(20); s = status(); n += 1 }
      assert(!bit(s, S_FLUSH_BUSY), "flush never completed")
      assert(bit(s, S_WRITE_DONE), s"write_done not set, status=0x${s.toString(16)}")
      s
    }
    /** Push one result on core `i` as the decoder does: res.valid is a LEVEL that must fall between shots. */
    def result(i: Int, real: Int, imag: Int, holdCycles: Int = 2, gapCycles: Int = 4): Unit = {
      val f = dut.io.results(i)
      f.payload.real #= real; f.payload.imag #= imag; f.payload.res #= real < 0
      f.valid #= true;  dspCd.waitSampling(holdCycles)
      f.valid #= false; dspCd.waitSampling(gapCycles)
    }
    /** Simultaneous results on every core in `cores` (one shared level pulse). */
    def resultAll(cores: Seq[Int], vals: Map[Int, (Int, Int)], holdCycles: Int = 2, gapCycles: Int = 4): Unit = {
      for (i <- cores) { val (r, im) = vals(i)
        dut.io.results(i).payload.real #= r; dut.io.results(i).payload.imag #= im
        dut.io.results(i).payload.res #= r < 0; dut.io.results(i).valid #= true }
      dspCd.waitSampling(holdCycles)
      for (i <- cores) dut.io.results(i).valid #= false
      dspCd.waitSampling(gapCycles)
    }
    def accepted(i: Int): BigInt = rd(ACCEPTED + 4 * i)
    def rejected(i: Int): BigInt = rd(REJECTED + 4 * i)
    def finalAddr(): BigInt = rd(FINAL_ADDR)
    /** Read the DDR words straight out of the memory model (byte-exact reference). */
    def ddrWords(base: Long, nWords: Int): Seq[BigInt] = {
      val bytes = mem.memory.readArray(base, nWords * 8L)
      (0 until nWords).map(k => (0 until 8).foldLeft(BigInt(0))((a, b) => a | (BigInt(bytes(k * 8 + b) & 0xff) << (8 * b))))
    }

    /** Drain THROUGH the real path: program rd_base/rd_size, pulse rd_start, collect the AXIS stream
     *  until TLAST (never trusting `rd_done` — see VENDORED.md non-conformance 2), with random tready
     *  backpressure. Returns the 64-bit words in stream order. */
    def drainViaMmu(base: Long, nBytes: Int, stall: Boolean = true, ready: () => Boolean = null): Seq[BigInt] = {
      require(nBytes % 32 == 0 && nBytes >= 32, s"drain size $nBytes must be a positive multiple of 32")
      wr(RD_BASE, BigInt(base)); wr(RD_SIZE, BigInt(nBytes))
      val beats = scala.collection.mutable.ArrayBuffer[BigInt]()
      var sawLast = false
      val sink = fork {
        var guard = 0
        while (!sawLast && guard < 400000) {
          dut.io.rd.ready #= (if (ready != null) ready() else (!stall || rng.nextInt(4) != 0))
          ddrCd.waitSampling()
          if (dut.io.rd.valid.toBoolean && dut.io.rd.ready.toBoolean) {
            beats += dut.io.rd.fragment.toBigInt
            if (dut.io.rd.last.toBoolean) sawLast = true
          }
          guard += 1
        }
        dut.io.rd.ready #= true
      }
      wr(RD_START, BigInt(1))
      var n = 0
      while (!sawLast && n < 200000) { ddrCd.waitSampling(10); n += 1 }
      sink.join()
      assert(sawLast, s"AXIS TLAST never arrived draining $nBytes B from 0x${java.lang.Long.toHexString(base)}")
      assert(beats.size == nBytes / 32, s"got ${beats.size} beats, expected ${nBytes / 32}")
      // each 256-bit beat carries 4 little-endian 64-bit words
      beats.flatMap(b => (0 until 4).map(k => (b >> (64 * k)) & ((BigInt(1) << 64) - 1))).toSeq
    }
    /** Full drain contract check (plan v5 §3 / v4 §1). */
    def checkDrain(base: Long, expected: Map[Int, Seq[(Int, Int)]], s: BigInt): Unit = {
      val fa = finalAddr(); val nbytes = (fa - base).toInt
      assert(nbytes >= 0, s"final_addr 0x${fa.toString(16)} < base 0x${java.lang.Long.toHexString(base)}")
      val nwords = nbytes / 8
      val accs = (0 until nch).map(i => accepted(i).toInt)
      val S = accs.sum
      assert(nwords - S >= 0 && nwords - S <= 3, s"pad out of range: nwords=$nwords S=$S")
      for (i <- 0 until nch) assert(accs(i) == expected.getOrElse(i, Nil).size,
        s"core $i accepted=${accs(i)} expected=${expected.getOrElse(i, Nil).size}")
      for (i <- 0 until nch) assert(rejected(i) == 0, s"core $i rejected=${rejected(i)}")
      assert(!bit(s, S_OVF_ANY), "overflow set")
      assert(!bit(s, S_WRAPPED) && !bit(s, S_BRESP_ERR) && !bit(s, S_RRESP_ERR) && !bit(s, S_SKID_OVF),
        s"error sticky set: 0x${s.toString(16)}")
      // the OVERFLOW register must agree with the status summary bit
      assert(rd(OVERFLOW) == 0, s"OVERFLOW register = ${rd(OVERFLOW)}")
      for (b <- Seq(S_ERR_BADSIZE, S_ERR_BADBASE, S_ERR_INJ_BUSY, S_ERR_INJ_RANGE, S_ERR_BASE_BUSY,
                    S_ERR_FLUSH_REFUSED, S_ERR_FLUSH_TIMEOUT, S_ERR_START_DROPPED, S_ERR_FLUSH_DROPPED,
                    S_CROSS_DROPPED, S_EARLY_LATE))
        assert(!bit(s, b), s"error/flag bit $b set: status=0x${s.toString(16)}")
      val words = ddrWords(base, S)
      // and the SAME data must come back through mmu2 + the AXIS drain (whole 32-B beats)
      if (S > 0) {
        val nb = ((S + 3) / 4) * 32
        val streamed = drainViaMmu(base, nb)
        for (k <- 0 until S) assert(streamed(k) == words(k),
          f"drain-vs-memory mismatch at word $k: 0x${streamed(k)}%x != 0x${words(k)}%x")
      }
      val perTag = mutable.Map[Int, mutable.ArrayBuffer[BigInt]]()
      for (w <- words) { val (t, r, im) = splitWord(w); perTag.getOrElseUpdate(t, mutable.ArrayBuffer()) += w }
      for (i <- 0 until nch) {
        val exp = expected.getOrElse(i, Nil)
        val got = perTag.getOrElse(i, mutable.ArrayBuffer())
        assert(got.size == exp.size, s"tag $i: ${got.size} words, expected ${exp.size}")
        for ((w, (r, im)) <- got.zip(exp)) assert(w == tagWord(i, r, im),
          f"tag $i word mismatch: got 0x${w}%x expected 0x${tagWord(i, r, im)}%x (real=$r imag=$im)")
      }
    }
  }

  // ─────────────────────────── scenarios ───────────────────────────

  /** 1. Basic run: a few results per core, flush, drain, full contract. */
  run("basic", 1) { (dut, h) =>
    h.startRun()
    val exp = mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]()
    for (round <- 0 until 3; i <- 0 until NCH) {
      val (r, im) = (0x11110000 + round * 0x100 + i, 0x22220000 + round * 0x100 + i)
      h.result(i, r, im)
      exp.getOrElseUpdate(i, mutable.ArrayBuffer()) += ((r, im))
    }
    val s = h.flushRun()
    h.checkDrain(BASE0, exp.map { case (k, v) => k -> v.toSeq }.toMap, s)
  }

  /** 2. All cores fire simultaneously, many rounds — poller fairness + no loss end to end. */
  run("simultaneous", 2) { (dut, h) =>
    h.startRun()
    val exp = mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]()
    for (round <- 0 until 12) {
      val vals = (0 until NCH).map(i => i -> ((0x1000 * (round + 1) + i, 0x7000 * (round + 1) + i))).toMap
      h.resultAll(0 until NCH, vals, holdCycles = 2, gapCycles = 60)  // >= 3*NCH drain
      for (i <- 0 until NCH) exp.getOrElseUpdate(i, mutable.ArrayBuffer()) += vals(i)
    }
    val s = h.flushRun()
    h.checkDrain(BASE0, exp.map { case (k, v) => k -> v.toSeq }.toMap, s)
  }

  /** 3. Results before the run is started are REJECTED (not counted, flagged), then a clean run works. */
  run("admission", 3) { (dut, h) =>
    for (i <- 0 until NCH) h.result(i, 0xdead0000 + i, 0xbeef0000 + i)
    assert(!dut.io.dspAdmit.toBoolean, "admit should be closed before base_reset")
    val s0 = h.status()
    assert(h.bit(s0, S_EARLY_LATE), "early_late_result should be set by pre-run results")
    h.startRun()   // base_reset clears the stickies and the counters
    val exp = mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]()
    for (i <- 0 until NCH) { val v = (0x100 + i, 0x200 + i); h.result(i, v._1, v._2)
      exp.getOrElseUpdate(i, mutable.ArrayBuffer()) += v }
    val s = h.flushRun()
    assert(!h.bit(s, S_EARLY_LATE), "early_late should have been cleared by base_reset")
    h.checkDrain(BASE0, exp.map { case (k, v) => k -> v.toSeq }.toMap, s)
    // results AFTER the flush snapshot are rejected again
    h.result(0, 1, 2)
    assert(h.bit(h.status(), S_EARLY_LATE), "post-flush result should be flagged")
  }

  /** 4. Overflow: hammer EVERY core back-to-back so the aggregate rate beats the poller.
   *  (At numCh <= roll_poll_reader2's PIPELINE_THRESHOLD=8 the poller drains 1 word/cycle, so a single
   *  core producing 1 result per 2 cycles can never overflow — the loss model is about the AGGREGATE
   *  arrival rate across cores, which is what this drives.) */
  run("overflow", 4) { (dut, h) =>
    h.startRun()
    val N = 300
    for (k <- 0 until N) {
      for (i <- 0 until NCH) { dut.io.results(i).payload.real #= k * 16 + i
        dut.io.results(i).payload.imag #= k; dut.io.results(i).valid #= true }
      h.dspCd.waitSampling()
      for (i <- 0 until NCH) dut.io.results(i).valid #= false
      h.dspCd.waitSampling()
    }
    h.dspCd.waitSampling(4000)
    val s = h.flushRun()
    val accs = (0 until NCH).map(i => h.accepted(i).toInt)
    println(s"[G2] overflow: accepted=${accs.mkString(",")} of $N each; ovf=${h.bit(s, S_OVF_ANY)}")
    assert(h.bit(s, S_OVF_ANY), s"overflow must be set; accepted=${accs.mkString(",")}")
    assert(accs.forall(a => a > 0 && a < N), s"accepted should be partial: ${accs.mkString(",")}")
    assert(!h.bit(s, S_SKID_OVF), "skid must never overflow (the throttle is what prevents it)")
  }

  /** 4b. DDR backpressure: a SLOW memory (response delay + only 2 outstanding) must throttle back
   *  through cbuf.wr_ready + the skid FIFO without losing a word. */
  run("ddr_backpressure", 41, memDelay = 40) { (dut, h) =>
    h.startRun()
    // r11-#6: watch the actual backpressure signals, not just the end result.
    var sawWrNotReady = false; var sawThrottle = false; var maxSkid = 0
    val watch = fork {
      while (true) {
        h.dspCd.waitSampling()
        if (!dut.up.dsp.cbufWrReady.toBoolean) sawWrNotReady = true
        if (dut.up.dsp.throttle.toBoolean) sawThrottle = true
        val occ = dut.up.dsp.skid.io.occupancy.toInt
        if (occ > maxSkid) maxSkid = occ
      }
    }
    val exp = scala.collection.mutable.Map[Int, scala.collection.mutable.ArrayBuffer[(Int, Int)]]()
    for (round <- 0 until 40; i <- 0 until NCH) {
      val v = (0x3000 + round * 16 + i, 0x9000 + round * 16 + i)
      h.result(i, v._1, v._2, holdCycles = 2, gapCycles = 3)
      exp.getOrElseUpdate(i, scala.collection.mutable.ArrayBuffer()) += v
    }
    val s = h.flushRun()
    h.checkDrain(BASE0, exp.map { case (k, v) => k -> v.toSeq }.toMap, s)
    println(s"[G2] ddr_backpressure: wr_ready fell=$sawWrNotReady throttle=$sawThrottle maxSkid=$maxSkid")
    assert(maxSkid > 0, "the skid FIFO never held a word — no backpressure propagated at all")
    assert(!h.bit(s, S_SKID_OVF), "skid overflowed")
    watch.terminate()
  }

  /** 4c. The Fork-A stall itself. `wr_ready` only falls when the write side reaches the LAST slot of its
   *  bank (WR_DEPTH-1 = 64 words at cbufAddrWidth=4, RATIO=4) while the reader still holds the credit —
   *  so it needs BOTH a saturated arrival rate and a memory slow enough that the drain of the other bank
   *  outlasts the fill of this one. Words may legitimately be dropped at the per-core FIFOs here, so the
   *  data contract checked is conservation: what lands must be an in-order SUBSEQUENCE of what was
   *  offered, exactly `accepted[i]` long, with `accepted+rejected <= offered` and every shortfall
   *  flagged in OVERFLOW (`rejected` counts arrivals while admission is CLOSED; FIFO-full drops are
   *  counted by OVERFLOW only — see r12-#6). (r11-#6) */
  run("cbuf_wr_stall", 42, memDelay = 400) { (dut, h) =>
    h.startRun()
    var sawWrNotReady = false; var sawThrottle = false; var maxSkid = 0
    val watch = fork {
      while (true) {
        h.dspCd.waitSampling()
        if (!dut.up.dsp.cbufWrReady.toBoolean) sawWrNotReady = true
        if (dut.up.dsp.throttle.toBoolean) sawThrottle = true
        val occ = dut.up.dsp.skid.io.occupancy.toInt
        if (occ > maxSkid) maxSkid = occ
      }
    }
    // offer at the maximum rate the decoder interface allows (level high 1 cycle, low 1 cycle)
    val N = 400
    val offered = Array.tabulate(NCH)(i => (0 until N).map(k => (0x40000 + k * 16 + i, 0x50000 + k * 16 + i)))
    for (k <- 0 until N) {
      for (i <- 0 until NCH) { dut.io.results(i).payload.real #= offered(i)(k)._1
        dut.io.results(i).payload.imag #= offered(i)(k)._2
        dut.io.results(i).payload.res #= false; dut.io.results(i).valid #= true }
      h.dspCd.waitSampling()
      for (i <- 0 until NCH) dut.io.results(i).valid #= false
      h.dspCd.waitSampling()
    }
    h.dspCd.waitSampling(20000)
    watch.terminate()
    val s = h.flushRun()
    println(s"[G2] cbuf_wr_stall: wr_ready fell=$sawWrNotReady throttle=$sawThrottle maxSkid=$maxSkid")
    assert(sawWrNotReady, "the slow memory never stalled the circular buffer (wr_ready never fell)")
    assert(sawThrottle, "the poller throttle never engaged even though the cbuf stalled")
    assert(!h.bit(s, S_SKID_OVF), "skid overflowed — the throttle failed to protect it")
    // conservation + in-order subsequence, per core
    val fa = h.finalAddr(); val nwords = ((fa - BASE0) / 8).toInt
    val S = (0 until NCH).map(i => h.accepted(i).toInt).sum
    assert(nwords - S >= 0 && nwords - S <= 3, s"pad out of range: nwords=$nwords S=$S")
    // only the first S words are payload; the tail of the last 256-bit beat is STALE cbuf RAM (pad)
    val words = h.ddrWords(BASE0, nwords).take(S)
    val perTag = mutable.Map[Int, mutable.ArrayBuffer[BigInt]]()
    for (w <- words) perTag.getOrElseUpdate(splitWord(w)._1, mutable.ArrayBuffer()) += w
    val ovfMask = h.rd(OVERFLOW)
    for (i <- 0 until NCH) {
      val acc = h.accepted(i).toInt; val rej = h.rejected(i).toInt
      // Loss model: `rejected` counts results that arrived while admission was CLOSED; results dropped
      // because the per-core FIFO was full are counted by the OVERFLOW bit, not by `rejected`. So the
      // invariant is acc+rej <= offered, and any shortfall MUST be flagged in OVERFLOW.
      assert(acc + rej <= N, s"core $i: accepted=$acc + rejected=$rej > offered=$N")
      val lost = N - acc - rej
      assert(lost == 0 || ((ovfMask >> i) & 1) == 1,
        s"core $i lost $lost results with no OVERFLOW flag (mask=0x${ovfMask.toString(16)})")
      val got = perTag.getOrElse(i, mutable.ArrayBuffer())
      assert(got.size == acc, s"core $i: ${got.size} words in DDR but accepted=$acc")
      var k = 0
      for (w <- got) {
        while (k < N && w != tagWord(i, offered(i)(k)._1, offered(i)(k)._2)) k += 1
        assert(k < N, f"core $i: DDR word 0x${w}%x is not in the offered sequence, or arrived out of order")
        k += 1
      }
      println(s"[G2]   core $i: accepted=$acc rejected=$rej overflowed=$lost (in-order subsequence of $N offered)")
    }
  }

  /** 4d. P3c-2: skid headroom under SUSTAINED stalls, for the registered poller throttle. All cores offer at the
   *  maximum rate while the W channel is held stalled for long random spans (released briefly in between), so the cbuf
   *  backs up, the poller throttles, and the skid sits at its throttle level for thousands of cycles at a time, against
   *  every phase alignment of the throttle register and the poller. The skid must never overflow and its occupancy
   *  must stay within throttleLevel + 1 (the analytical worst case, see ReadoutDdrUplink's throttle); the data
   *  delivered is an in-order subsequence of each core's offers, exactly accepted[i] long, every loss flagged. */
  def skidHeadroom(name: String, seed: Int, nch: Int): Unit = run(name, seed, nch = nch) { (dut, h) =>
    h.startRun()
    // the scenario drives W itself (long holds); in `stalls` mode the injector would override that every cycle
    curStalls.enabled = false; dut.io.awStall #= false; dut.io.arStall #= false; dut.io.bStall #= false
    var sawThrottle = 0L; var maxSkid = 0; var stalled = 0L
    val watch = fork {
      while (true) {
        h.dspCd.waitSampling()
        if (dut.up.dsp.throttle.toBoolean) sawThrottle += 1
        val occ = dut.up.dsp.skid.io.occupancy.toInt
        if (occ > maxSkid) maxSkid = occ
      }
    }
    val stall = fork {
      while (true) {
        dut.io.wStall #= true;  val on = 1500 + h.rng.nextInt(2500); h.dspCd.waitSampling(on); stalled += on
        dut.io.wStall #= false; h.dspCd.waitSampling(50 + h.rng.nextInt(400))
      }
    }
    val N = 3000
    val offered = Array.tabulate(nch)(i => (0 until N).map(k => (0x60000 + k * 16 + i, 0x70000 + k * 16 + i)))
    for (k <- 0 until N) {
      for (i <- 0 until nch) { dut.io.results(i).payload.real #= offered(i)(k)._1
        dut.io.results(i).payload.imag #= offered(i)(k)._2
        dut.io.results(i).payload.res #= false; dut.io.results(i).valid #= true }
      h.dspCd.waitSampling()
      for (i <- 0 until nch) dut.io.results(i).valid #= false
      h.dspCd.waitSampling(1 + h.rng.nextInt(2))
    }
    stall.terminate(); dut.io.wStall #= false; curStalls.enabled = true
    h.dspCd.waitSampling(20000)
    watch.terminate()
    val s = h.flushRun()
    // the analytical worst case of the registered throttle: +1 word for the pipelined poller, +2 for the single-cycle one
    val pp = ReadoutDdrUplinkParams(numCh = nch)
    val lim = pp.throttleLevel + (if (nch > 8) 1 else 2)
    println(s"[G2] $name: $nch ch (${if (nch > 8) "pipelined" else "single-cycle"} poller), W stalled $stalled dsp cycles, throttle high $sawThrottle cycles, maxSkid=$maxSkid (limit $lim, depth ${pp.skidDepth})")
    assert(sawThrottle > 2000, s"the throttle was barely exercised ($sawThrottle cycles)")
    assert(maxSkid <= lim, s"skid occupancy $maxSkid exceeds throttleLevel + 1 = $lim")
    assert(!h.bit(s, S_SKID_OVF), "skid overflowed under sustained stalls")
    val fa = h.finalAddr(); val nwords = ((fa - BASE0) / 8).toInt
    val S = (0 until nch).map(i => h.accepted(i).toInt).sum
    assert(nwords - S >= 0 && nwords - S <= 3, s"pad out of range: nwords=$nwords S=$S")
    val words = h.ddrWords(BASE0, nwords).take(S)
    val perTag = mutable.Map[Int, mutable.ArrayBuffer[BigInt]]()
    for (w <- words) perTag.getOrElseUpdate(splitWord(w)._1, mutable.ArrayBuffer()) += w
    val ovfMask = h.rd(OVERFLOW)
    for (i <- 0 until nch) {
      val acc = h.accepted(i).toInt; val rej = h.rejected(i).toInt
      assert(acc + rej <= N, s"core $i: accepted=$acc + rejected=$rej > offered=$N")
      val lost = N - acc - rej
      assert(lost == 0 || ((ovfMask >> i) & 1) == 1, s"core $i lost $lost results with no OVERFLOW flag")
      val got = perTag.getOrElse(i, mutable.ArrayBuffer())
      assert(got.size == acc, s"core $i: ${got.size} words in DDR but accepted=$acc")
      var k = 0
      for (w <- got) {
        while (k < N && w != tagWord(i, offered(i)(k)._1, offered(i)(k)._2)) k += 1
        assert(k < N, f"core $i: DDR word 0x${w}%x is not in the offered sequence, or arrived out of order")
        k += 1
      }
    }
  }
  skidHeadroom("skid_headroom_sustained_stall", 43, 4)
  skidHeadroom("skid_headroom_sustained_stall_14ch", 44, 14)

  /** 5. Register validation: bad wr_base, bad rd_size, base_reset while busy. */
  run("regvalidate", 5) { (dut, h) =>
    h.wr(WR_BASE, BigInt(0x1234))                        // not 512-B aligned
    assert(h.bit(h.status(), S_ERR_BADBASE), "unaligned wr_base must be rejected")
    assert(h.rd(WR_BASE) != BigInt(0x1234), "rejected wr_base must not be stored")
    h.wr(STATUS, BigInt(1) << S_ERR_BADBASE)     // W1C
    assert(!h.bit(h.status(), S_ERR_BADBASE), "err_badbase should be W1C-clearable")
    h.wr(WR_BASE, BigInt(0x80000200L))                   // bit31 set
    assert(h.bit(h.status(), S_ERR_BADBASE), "wr_base >= 2 GiB must be rejected")
    h.wr(STATUS, BigInt(1) << S_ERR_BADBASE)
    h.wr(RD_BASE, BigInt(0x2000)); h.wr(RD_SIZE, BigInt(17))     // not a multiple of 32
    h.wr(RD_START, BigInt(1))
    assert(h.bit(h.status(), S_ERR_BADSIZE), "unaligned rd_size must be rejected")
    h.wr(STATUS, BigInt(1) << S_ERR_BADSIZE)
    h.wr(RD_SIZE, BigInt(MAX_RD_SIZE + 32))              // above the DMA 26-bit cap
    h.wr(RD_START, BigInt(1))
    assert(h.bit(h.status(), S_ERR_BADSIZE), "oversized rd_size must be rejected")
    h.wr(STATUS, BigInt(1) << S_ERR_BADSIZE)
    h.startRun()
    h.wr(BASE_RESET, BigInt(1))                          // second base_reset while the run is active
    assert(h.bit(h.status(), S_ERR_BASE_BUSY), "base_reset during a run must be refused")
    h.flushRun()
  }

  /** 6. Geometry registers are derived, not literal. */
  run("geometry", 6) { (dut, h) =>
    assert(h.rd(ReadoutDdrRegs.NUM_CH) == NCH, s"NUM_CH reg = ${h.rd(ReadoutDdrRegs.NUM_CH)} expected $NCH")
    val g = h.rd(GEOMETRY)
    assert((g & 0xff) == p.fifoDepth, s"fifoDepth ${g & 0xff}")
    assert(((g >> 8) & 0xff) == p.skidDepth, s"skidDepth ${(g >> 8) & 0xff}")
    assert(((g >> 16) & 0xff) == p.cbufAddrWidth, s"cbufAddrWidth ${(g >> 16) & 0xff}")
    assert(((g >> 24) & 0xff) == p.flushQuiet, s"flushQuiet ${(g >> 24) & 0xff}")
  }

  /** 7. Two consecutive runs at different bases: run 2 must not see run 1's accounting or data. */
  run("two_runs", 7) { (dut, h) =>
    val base2 = BASE0 + 0x4000
    for ((base, mark) <- Seq((BASE0, 0x1000), (base2, 0x5000))) {
      h.startRun(base)
      val exp = mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]()
      for (round <- 0 until 2; i <- 0 until NCH) {
        val v = (mark + round * 16 + i, mark + 0x800 + round * 16 + i)
        h.result(i, v._1, v._2); exp.getOrElseUpdate(i, mutable.ArrayBuffer()) += v
      }
      val s = h.flushRun()
      assert(h.rd(RUN_BASE) == base, s"run_base should be 0x${base.toHexString}")
      h.checkDrain(base, exp.map { case (k, v) => k -> v.toSeq }.toMap, s)
    }
  }

  /** 8. FOURTEEN channels — this is the production geometry and the only one that exercises
   *  roll_poll_reader2's 3-phase pipeline (NUM_CH > PIPELINE_THRESHOLD=8, 1 word / 3 cycles). */
  run("fourteen_ch", 8, nch = 14) { (dut, h) =>
    h.startRun()
    val exp = mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]()
    for (round <- 0 until 6) {
      val vals = (0 until 14).map(i => i -> ((0x2000 * (round + 1) + i, 0x6000 * (round + 1) + i))).toMap
      h.resultAll(0 until 14, vals, holdCycles = 2, gapCycles = 3 * 14 + 20)   // >= 3N poller drain
      for (i <- 0 until 14) exp.getOrElseUpdate(i, mutable.ArrayBuffer()) += vals(i)
    }
    val s = h.flushRun()
    h.checkDrain(BASE0, exp.map { case (k, v) => k -> v.toSeq }.toMap, s)
  }

  /** 9. Injector: a valid injection lands as a normal word; out-of-range core is rejected without
   *  hanging the handshake; firing while busy or outside a run is refused. */
  run("injector", 9) { (dut, h) =>
    // outside a run -> refused
    h.wr(INJ_FIRE, BigInt(1))
    assert(h.bit(h.status(), S_ERR_INJ_BUSY), "inj_fire outside a run must be refused")
    h.wr(STATUS, BigInt(1) << S_ERR_INJ_BUSY)
    // out-of-range core -> refused at the register write, and NOTHING hangs
    for (bad <- Seq(BigInt(NCH + 3), BigInt(0x100), BigInt(0x10000), BigInt("FFFFFFFF", 16))) {
      // 0x100 is the regression for the old low-byte-only check: it used to alias to core 0.
      h.wr(INJ_CORE, bad)
      assert(h.bit(h.status(), S_ERR_INJ_RANGE), s"inj_core 0x${bad.toString(16)} must be refused")
      h.wr(STATUS, BigInt(1) << S_ERR_INJ_RANGE)
    }
    h.startRun()
    val exp = mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]()
    for (i <- 0 until NCH) {
      val v = (0x5150000 + i * 16, 0x2620000 + i * 16)
      h.wr(INJ_REAL, BigInt(v._1)); h.wr(INJ_IMAG, BigInt(v._2)); h.wr(INJ_CORE, BigInt(i))
      h.wr(INJ_FIRE, BigInt(1))
      var n = 0
      while (h.bit(h.status(), S_INJ_BUSY) && n < 500) { h.ddrCd.waitSampling(10); n += 1 }
      assert(!h.bit(h.status(), S_INJ_BUSY), s"injector never completed for core $i")
      exp.getOrElseUpdate(i, mutable.ArrayBuffer()) += v
    }
    val s = h.flushRun()
    h.checkDrain(BASE0, exp.map { case (k, v) => k -> v.toSeq }.toMap, s)
  }

  /** 10. `rejected[]` accounting: results outside a run must be COUNTED per core (not just flagged),
   *  including a real result and a rejected injection hitting the same core in the same cycle, and the
   *  saturating Gray counter must survive a long burst. (r09-#2/#9) */
  run("rejected_accounting", 10) { (dut, h) =>
    // (a) N late results per core, admission closed -> rejected[i] == N
    val N = 7
    for (_ <- 0 until N; i <- 0 until NCH) h.result(i, 0x1234 + i, 0x5678 + i)
    val rej = (0 until NCH).map(i => h.rejected(i).toInt)
    assert(rej.forall(_ == N), s"expected $N rejections per core, got ${rej.mkString(",")}")
    assert(h.bit(h.status(), S_EARLY_LATE), "early_late must be set")

    // (b) an injection fired INSIDE a run but rejected on the DSP side (admission closed by the flush
    // snapshot) must also be counted, including when it collides with a late real result on the same
    // core in the same cycle (the case that used to undercount, r09-#2 / r10-#2).
    h.startRun()
    h.wr(INJ_REAL, BigInt(0x1111)); h.wr(INJ_IMAG, BigInt(0x2222)); h.wr(INJ_CORE, BigInt(0))
    h.flushRun()                                   // admission is now closed; the run is over
    val rejBefore = h.rejected(0).toInt
    // Once the run is OVER, `run_active` is clear and INJ_FIRE is refused at the register. (The window
    // in which admission is already closed but the run is still active IS reachable — see
    // `rejected_collision`; this case is the one AFTER it.) Confirm the guard, then count exactly.
    h.wrNb(INJ_FIRE, BigInt(1))
    val f0 = dut.io.results(0)
    for (_ <- 0 until 6) {
      f0.valid #= true; h.dspCd.waitSampling(); f0.valid #= false; h.dspCd.waitSampling()
    }
    h.ddrCd.waitSampling(200)
    assert(h.bit(h.status(), S_ERR_INJ_BUSY), "INJ_FIRE after the run must be refused at the register")
    h.wr(STATUS, BigInt(1) << S_ERR_INJ_BUSY)
    val rejAfter = h.rejected(0).toInt
    assert(rejAfter == rejBefore + 6,
      s"core 0 counted ${rejAfter - rejBefore} rejections, expected exactly 6")
    println(s"[G2] rejected_accounting: core0 counted ${rejAfter - rejBefore} real rejections; " +
            s"the injector was refused at the register (run_active gate)")

    // (c) the Gray crossing must track a long burst exactly (every increment is +1 by construction)
    val f = dut.io.results(0)
    val before = h.rejected(0).toInt
    val BURST = 400
    for (_ <- 0 until BURST) { f.valid #= true; h.dspCd.waitSampling(); f.valid #= false; h.dspCd.waitSampling() }
    h.ddrCd.waitSampling(200)
    val r0 = h.rejected(0).toInt
    assert(r0 == before + BURST, s"rejected(0) = $r0, expected ${before + BURST} (Gray crossing lost counts)")

    // (d) a base_reset clears the accounting
    h.startRun()
    for (i <- 0 until NCH) assert(h.rejected(i) == 0, s"rejected($i) not cleared by base_reset")
    assert(!h.bit(h.status(), S_EARLY_LATE), "early_late not cleared by base_reset")
    h.flushRun()
  }

  /** 11. OVERFLOW snapshot must be NON-ZERO and per-core exact when the FIFOs are overrun. (r09-#10) */
  run("overflow_snapshot", 11, nch = 14) { (dut, h) =>
    h.startRun()
    // 14 channels => roll_poll_reader2 uses its 3-phase branch (1 word / 3 cycles), so hammering three
    // cores at 1 result / 2 cycles each is far beyond the drain rate. The other 11 cores stay idle, so
    // the snapshot must show EXACTLY the hammered bits.
    val hot = Seq(0, 2, 5)
    for (k <- 0 until 300) {
      for (i <- hot) { dut.io.results(i).payload.real #= k; dut.io.results(i).valid #= true }
      h.dspCd.waitSampling()
      for (i <- hot) dut.io.results(i).valid #= false
      h.dspCd.waitSampling()
    }
    h.dspCd.waitSampling(4000)
    val s = h.flushRun()
    val ovf = h.rd(OVERFLOW)
    println(s"[G2] overflow_snapshot: OVERFLOW=0x${ovf.toString(16)} hot=${hot.mkString(",")}")
    assert(ovf != 0, "OVERFLOW snapshot must be non-zero after an overrun")
    assert(h.bit(s, S_OVF_ANY), "ovf_any must agree with the OVERFLOW register")
    for (i <- 0 until h.nch)
      if (!hot.contains(i)) assert(((ovf >> i) & 1) == 0, s"core $i was idle but its overflow bit is set")
    for (i <- hot) assert(((ovf >> i) & 1) == 1, s"core $i was hammered but its overflow bit is clear")
  }

  /** 12. Injector payload integrity across the crossing. The busy window is deliberately SHORT (the
   *  arbiter grants as soon as the FIFO has room), so racing it with bus writes is not a meaningful
   *  test. What IS architecturally guaranteed — and what makes the multi-bit `crossClockDomain` tag
   *  legitimate — is that the payload can never TEAR: whatever lands must be one coherent
   *  {real, imag, core} triple, never a mix of the old and new writes. (r09-#10) */
  run("injector_payload_integrity", 12) { (dut, h) =>
    h.startRun()
    val a = (0x11223340, 0x55667780, 0)
    val b = (0x7ffffff0, 0x7fffffe0, 1 % NCH)
    h.wr(INJ_REAL, BigInt(a._1)); h.wr(INJ_IMAG, BigInt(a._2)); h.wr(INJ_CORE, BigInt(a._3))
    // fire, then immediately queue the OTHER payload without waiting: some of these writes land inside
    // the busy window (refused, err_inj_busy) and some may land after it (accepted) — both are legal.
    h.wrNb(INJ_FIRE, BigInt(1))
    h.wrNb(INJ_REAL, BigInt(b._1)); h.wrNb(INJ_IMAG, BigInt(b._2)); h.wrNb(INJ_CORE, BigInt(b._3))
    var n = 0
    while (h.bit(h.status(), S_INJ_BUSY) && n < 500) { h.ddrCd.waitSampling(10); n += 1 }
    assert(!h.bit(h.status(), S_INJ_BUSY), "injector never completed")
    assert(h.bit(h.status(), S_ERR_INJ_BUSY), "at least one write should have been refused while busy")
    h.wr(STATUS, BigInt(1) << S_ERR_INJ_BUSY)
    val s = h.flushRun()
    // exactly ONE injection was fired, so exactly one word must exist, and it must be a COHERENT triple
    val accs = (0 until NCH).map(i => h.accepted(i).toInt)
    assert(accs.sum == 1, s"expected exactly 1 injected word, got ${accs.mkString(",")}")
    val core = accs.indexWhere(_ == 1)
    val exp = if (core == a._3) Seq((a._1, a._2)) else { assert(core == b._3, s"word on core $core"); Seq((b._1, b._2)) }
    h.checkDrain(BASE0, Map(core -> exp), s)
    println(s"[G2] injector_payload_integrity: coherent payload landed on core $core")
  }

  /** 13. STATUS W1C must not erase a sticky that is set in the SAME cycle (r09-#4 / r10-#1/#8).
   *  Blocking writes are serialised and can never collide, so the colliding pair is QUEUED
   *  non-blocking: an illegal RD_SIZE (raises err_badsize) immediately followed by a W1C of that very
   *  bit. With a clear-dominant update the set is swallowed; with set-dominant it survives. */
  run("w1c_race", 13) { (dut, h) =>
    for (k <- 0 until 24) {
      h.wr(STATUS, BigInt(1) << S_ERR_BADSIZE)                 // start clean
      h.wrNb(RD_SIZE, BigInt(17 + 2 * k))                      // illegal -> hardware sets err_badsize
      h.wrNb(RD_START, BigInt(1))                              // (also illegal size -> sets it again)
      h.wrNb(STATUS, BigInt(1) << S_ERR_BADSIZE)               // ... colliding W1C
      h.wrNb(RD_SIZE, BigInt(19 + 2 * k))                      // and another set right behind it
      h.wrNb(RD_START, BigInt(1))
      assert(h.bit(h.status(), S_ERR_BADSIZE),
        s"iteration $k: a set that follows the W1C was swallowed (clear-dominant STATUS update)")
    }
    h.wr(STATUS, BigInt(1) << S_ERR_BADSIZE)
    // drive a stream of illegal wr_base writes (each sets err_badbase) while hammering STATUS W1C
    assert(!h.bit(h.status(), S_ERR_BADSIZE), "W1C must still clear when nothing sets the bit")
    h.wr(WR_BASE, BigInt(0x1235))
    assert(h.bit(h.status(), S_ERR_BADBASE), "a set that is not followed by a clear must survive")
    h.wr(STATUS, BigInt(1) << S_ERR_BADBASE)
    assert(!h.bit(h.status(), S_ERR_BADBASE), "W1C must still clear")
    // and a bit that was never written must be untouched by a W1C of another bit
    h.wr(RD_SIZE, BigInt(17)); h.wr(RD_START, BigInt(1))
    assert(h.bit(h.status(), S_ERR_BADSIZE))
    h.wr(STATUS, BigInt(1) << S_ERR_BADBASE)
    assert(h.bit(h.status(), S_ERR_BADSIZE), "W1C of an unrelated bit must not clear err_badsize")
  }

  /** 14. rd_base/rd_size are frozen while a drain is in flight. (r09-#5) */
  run("rd_regs_frozen", 14) { (dut, h) =>
    h.startRun()
    val exp = scala.collection.mutable.Map[Int, scala.collection.mutable.ArrayBuffer[(Int, Int)]]()
    for (round <- 0 until 20; i <- 0 until NCH) {
      val v = (0x4000 + round * 16 + i, 0x8000 + round * 16 + i)
      h.result(i, v._1, v._2); exp.getOrElseUpdate(i, scala.collection.mutable.ArrayBuffer()) += v
    }
    val s = h.flushRun()
    h.checkDrain(BASE0, exp.map { case (k, v) => k -> v.toSeq }.toMap, s)
    // now start a drain and try to move the window mid-flight
    val total = (0 until NCH).map(i => h.accepted(i).toInt).sum
    val nb = ((total + 3) / 4) * 32
    h.wr(RD_BASE, BigInt(BASE0)); h.wr(RD_SIZE, BigInt(nb))
    // Hold the AXIS sink OFF so the stream stalls: mmu2's `busy` falls when the last R beat enters its
    // FIFO, but `size_bytes` stays live in the TLAST logic until the beats actually leave. Writing
    // RD_SIZE in THAT window used to reframe the outstanding stream (r10-#3).
    // mmu2's internal FIFO is 16 beats (FIFO_ADDR_BITS=4). With the sink held off, `busy` can only fall
    // if the WHOLE transfer fits in that FIFO -- so use a small drain (8 beats) to open the window
    // between "last R beat accepted" (busy falls) and "TLAST leaves" (the stream is really done).
    val small = 8 * 32
    h.wr(RD_BASE, BigInt(BASE0)); h.wr(RD_SIZE, BigInt(small))
    dut.io.rd.ready #= false
    h.wr(RD_START, BigInt(1))
    var g = 0
    while (h.bit(h.status(), S_RD_BUSY) && g < 4000) { h.ddrCd.waitSampling(); g += 1 }
    assert(!h.bit(h.status(), S_RD_BUSY), "mmu2 busy never fell although the transfer fits its FIFO")
    h.wr(RD_SIZE, BigInt(32))                      // busy is LOW but TLAST has not left -> must be refused
    assert(h.rd(RD_SIZE) == small, s"rd_size changed after busy fell but before TLAST: ${h.rd(RD_SIZE)} != $small")
    assert(h.bit(h.status(), S_ERR_BADSIZE), "the refused write should have raised err_badsize")
    h.wr(STATUS, BigInt(1) << S_ERR_BADSIZE)
    // let the stalled stream finish and confirm its framing is intact (TLAST on the 8th beat)
    var beats = 0; var sawLast = false; var gg = 0
    while (!sawLast && gg < 4000) {
      dut.io.rd.ready #= true
      h.ddrCd.waitSampling()
      if (dut.io.rd.valid.toBoolean) { beats += 1; if (dut.io.rd.last.toBoolean) sawLast = true }
      gg += 1
    }
    assert(sawLast, "the stalled stream never produced TLAST")
    assert(beats == small / 32, s"stalled stream emitted $beats beats, expected ${small / 32} (reframed)")
    h.wr(STATUS, BigInt(1) << S_ERR_BADSIZE)
  }

  /** 15. Set-vs-W1C COLLISION. The property is: when `stickySet[b]` and `stickyClr[b]` are asserted in
   *  the SAME ddr cycle, the bit must end up SET (`(sticky & ~clr) | set`, r10-#1). A W1C that merely
   *  arrives after the set is allowed to clear it, so the test cannot assert on the end state alone —
   *  it watches both strobes directly and checks the register on the following cycle. The stimulus
   *  scans the W1C offset in 1-cycle steps across the window in which the hardware raises `write_done`
   *  at the end of a flush, so a genuine same-cycle hit is produced rather than hoped for. */
  run("w1c_hw_collision", 15) { (dut, h) =>
    var collisions = 0; var bitsSeen = BigInt(0); var pendMask = BigInt(0); var checked = 0
    val mon = fork {
      while (true) {
        h.ddrCd.waitSampling()
        if (pendMask != 0) {                       // verify the value clocked in by THIS edge
          val s = dut.up.ddr.sticky.toBigInt
          assert((s & pendMask) == pendMask,
            f"set/clear collision on mask 0x$pendMask%x lost the set: sticky=0x$s%x (clear-dominant bug)")
          checked += 1; pendMask = 0
        }
        val c = dut.up.ddr.stickySet.toBigInt & dut.up.ddr.stickyClr.toBigInt
        if (c != 0) { collisions += 1; bitsSeen |= c; pendMask = c }
      }
    }
    val mask = BigInt(1) << S_WRITE_DONE
    for (k <- 0 until 60) {
      h.startRun()
      for (i <- 0 until NCH) h.result(i, 0x777 + k * 16 + i, 0x888 + k * 16 + i)
      h.wrNb(FLUSH, BigInt(1))
      h.ddrCd.waitSampling(k)
      h.wrNb(STATUS, mask)                          // lands somewhere around the hardware set
      var n = 0; var s = h.status()
      while (h.bit(s, S_FLUSH_BUSY) && n < 4000) { h.ddrCd.waitSampling(20); s = h.status(); n += 1 }
      assert(!h.bit(s, S_FLUSH_BUSY), s"k=$k: flush never completed")
    }
    h.ddrCd.waitSampling(10)     // let the monitor verify a collision seen on the very last edge
    mon.terminate()
    println(f"[G2] w1c_hw_collision: $collisions same-cycle set/clear events over 60 flushes " +
            f"(bits 0x$bitsSeen%x), $checked verified set-dominant")
    assert(collisions > 0, "the offset scan never produced a same-cycle set/clear — test is vacuous")
    // r12-#2: every collision must have been CHECKED, and none may be left pending at the end.
    assert(pendMask == 0, "a collision was still unverified when the monitor stopped")
    assert(checked == collisions, s"$collisions collisions but only $checked verified")
  }

  /** 16. `rejected[]` saturation. Uses a deliberately NARROW counter (6 bits) so the max is reachable in
   *  a short sim. Scope: the COUNTER saturates and is cleared by `base_reset`. It does not (and cannot,
   *  with a rejReal-only stimulus that increments by 1) say anything about the queue — that is
   *  `rejected_collision`'s job. (r11-#4 / r12-#3) */
  run("rejected_saturation", 16, rejW = 6) { (dut, h) =>
    val MAX = (1 << 6) - 1
    val f = dut.io.results(0)
    for (_ <- 0 until MAX + 40) { f.valid #= true; h.dspCd.waitSampling(); f.valid #= false; h.dspCd.waitSampling() }
    h.ddrCd.waitSampling(200)
    assert(h.rejected(0) == MAX, s"rejected(0)=${h.rejected(0)} should saturate at $MAX")
    assert(dut.up.dsp.rejPend(0).toInt == 0, "a rejReal-only stimulus must never queue a credit")
    h.startRun()
    for (i <- 0 until NCH) assert(h.rejected(i) == 0, s"rejected($i) not cleared by base_reset")
    val exp = scala.collection.mutable.Map[Int, scala.collection.mutable.ArrayBuffer[(Int, Int)]]()
    for (i <- 0 until NCH) { val v = (0x10 + i, 0x20 + i); h.result(i, v._1, v._2)
      exp.getOrElseUpdate(i, scala.collection.mutable.ArrayBuffer()) += v }
    val s = h.flushRun()
    h.checkDrain(BASE0, exp.map { case (k, v) => k -> v.toSeq }.toMap, s)   // requires rejected == 0
    println(s"[G2] rejected_saturation: counter saturated at $MAX and was cleared by base_reset")
  }

  /** 16b. The `rejPend` QUEUE — the r10-#2 fix — exercised for real.
   *
   *  r12-#4 corrected my claim that a DSP-side injection rejection is unreachable from the register
   *  interface. It IS reachable, and the window is software-visible: the flush snapshot sets
   *  `admit := False` on the DSP side, but `run_active` (DDR side) stays set until the writer's final
   *  BVALID. Between those two events `INJ_FIRE` passes the register guard (`!injBusy && runActive`)
   *  and then lands on `injPending && !admit` -> `injReject` -> `rejInj`. With a slow memory that
   *  window is hundreds of cycles wide.
   *
   *  Driving late real results on the same core throughout the window makes `rejReal` and `rejInj`
   *  collide, which is the only way `rejPend` becomes non-zero. Checks: the queue is observed non-zero,
   *  every increment is accounted for exactly (nothing lost, never +2), and `base_reset` clears the
   *  queue as well as the counter. (r12-#3, r12-#4) */
  run("rejected_collision", 18, memDelay = 300) { (dut, h) =>
    h.startRun()
    for (i <- 0 until NCH) h.result(i, 0x900 + i, 0xA00 + i)     // give the writer something to drain
    h.wr(INJ_REAL, BigInt(0x3333)); h.wr(INJ_IMAG, BigInt(0x4444)); h.wr(INJ_CORE, BigInt(0))
    var maxPend = 0
    val watch = fork {
      while (true) { h.dspCd.waitSampling(); val v = dut.up.dsp.rejPend(0).toInt; if (v > maxPend) maxPend = v }
    }
    h.wrNb(FLUSH, BigInt(1))
    // spin (on the bus) until admission is closed but the run is still active
    var n = 0; var s = h.status()
    while (!(!h.bit(s, S_DSP_ADMIT) && h.bit(s, S_RUN_ACTIVE)) && n < 3000) { s = h.status(); n += 1 }
    assert(!h.bit(s, S_DSP_ADMIT) && h.bit(s, S_RUN_ACTIVE),
      f"the admit-closed / run-active window never appeared: status=0x$s%x")
    val rejBefore = h.rejected(0).toInt
    // Hammer core 0 with late results throughout the window. `edge = valid && !RegNext(valid)` means the
    // fastest possible late-result rate is one edge every TWO dsp cycles, so a single injection has only
    // a ~50% chance of landing on an edge cycle. Fire repeatedly instead, letting the natural bus jitter
    // walk the phase, and stop as soon as the queue is observed non-zero.
    val f0 = dut.io.results(0)
    val edges = 300
    val hammer = fork {
      for (_ <- 0 until edges) { f0.valid #= true; h.dspCd.waitSampling(); f0.valid #= false; h.dspCd.waitSampling() }
    }
    var injAccepted = 0; var fires = 0
    while (maxPend == 0 && fires < 16 && h.bit(h.status(), S_RUN_ACTIVE)) {
      h.wr(INJ_FIRE, BigInt(1))
      if (!h.bit(h.status(), S_ERR_INJ_BUSY)) injAccepted += 1 else h.wr(STATUS, BigInt(1) << S_ERR_INJ_BUSY)
      fires += 1
      h.dspCd.waitSampling(fires % 2)      // walk the phase against the 2-cycle edge cadence
    }
    hammer.join()
    h.ddrCd.waitSampling(400)
    watch.terminate()
    var m = 0
    while (h.bit(h.status(), S_FLUSH_BUSY) && m < 8000) { h.ddrCd.waitSampling(20); m += 1 }
    assert(!h.bit(h.status(), S_FLUSH_BUSY), "flush never completed")
    val got = h.rejected(0).toInt - rejBefore
    val want = edges + injAccepted
    println(s"[G2] rejected_collision: maxRejPend=$maxPend, $injAccepted of $fires injections entered the " +
            s"closed window, core0 counted $got rejections (expected $want)")
    assert(injAccepted > 0, "no INJ_FIRE was accepted at the register — the admit-closed/run-active window was missed")
    assert(maxPend > 0, "rejPend never became non-zero: rejReal and rejInj never collided (test vacuous)")
    assert(got == want, s"core 0 counted $got rejections, expected exactly $want (queue lost or double-counted)")
    // r13-#4: this does NOT prove that `base_reset` clears `rejPend` — by the time the bus gets here the
    // one queued credit has long since been applied, so removing `rejPend := 0` from the run-start
    // handler would still pass. It cannot be proven from the bus either: a queued credit is applied on
    // the very NEXT cycle whenever `fire` is unblocked, and the only state where it would persist —
    // saturation — now discards the queue outright (r11-#1). The `rejPend := 0` at run start is
    // therefore belt-and-braces against a future change to that policy, and what is checked here is the
    // observable consequence: the next run starts from a clean slate.
    h.startRun()
    assert(dut.up.dsp.rejPend(0).toInt == 0, "rejPend non-zero at the start of a fresh run")
    for (i <- 0 until NCH) assert(h.rejected(i) == 0, s"rejected($i) not cleared by base_reset")
  }

  /** 17. `INJ_FIRE` while the injector is busy must be REFUSED and must not produce a word.
   *
   *  r12-#5: the previous version accepted either outcome, so a design that queued both fires passed.
   *  The busy window is only a few cycles when the pipeline is empty — shorter than a bus write — so
   *  this scenario first STALLS the datapath (slow memory + saturated arrival, the `cbuf_wr_stall`
   *  state) until core 0's FIFO cannot accept anything. The injector's grant is then blocked for
   *  hundreds of cycles, `injPending` is observed high across the second write, and the outcome is
   *  pinned: refused, and exactly ONE word carrying the injected payload in DDR. */
  run("injector_fire_while_busy", 17, memDelay = 400) { (dut, h) =>
    h.startRun()
    // 1. stall the datapath so the injector cannot be granted
    for (k <- 0 until 200) {
      for (i <- 0 until NCH) { dut.io.results(i).payload.real #= 0x70000 + k * 16 + i
        dut.io.results(i).payload.imag #= 0x80000 + k * 16 + i
        dut.io.results(i).payload.res #= false; dut.io.results(i).valid #= true }
      h.dspCd.waitSampling()
      for (i <- 0 until NCH) dut.io.results(i).valid #= false
      h.dspCd.waitSampling()
    }
    assert(dut.up.dsp.throttle.toBoolean || !dut.up.dsp.cbufWrReady.toBoolean,
      "the datapath did not stall — the injector's busy window will be too short to test")
    // 2. two fires back to back; watch injPending across both
    val INJ_R = 0x1234500; val INJ_I = 0x6789A00
    h.wr(INJ_REAL, BigInt(INJ_R)); h.wr(INJ_IMAG, BigInt(INJ_I)); h.wr(INJ_CORE, BigInt(0))
    var pendingHigh = 0
    val watch = fork { while (true) { h.dspCd.waitSampling(); if (dut.up.dsp.injPending.toBoolean) pendingHigh += 1 } }
    h.wr(INJ_FIRE, BigInt(1))
    val busyAtSecond = dut.up.dsp.injPending.toBoolean || h.bit(h.status(), S_INJ_BUSY)
    h.wr(INJ_FIRE, BigInt(1))
    val refused = h.bit(h.status(), S_ERR_INJ_BUSY)
    // 3. let the stalled pipeline drain, then flush and look for the payload
    var n = 0
    while (h.bit(h.status(), S_INJ_BUSY) && n < 4000) { h.ddrCd.waitSampling(20); n += 1 }
    assert(!h.bit(h.status(), S_INJ_BUSY), "injector never went idle")
    h.dspCd.waitSampling(20000)
    val s = h.flushRun()
    val S = (0 until NCH).map(i => h.accepted(i).toInt).sum
    val words = h.ddrWords(BASE0, ((h.finalAddr() - BASE0) / 8).toInt).take(S)
    val hits = words.count(_ == tagWord(0, INJ_R, INJ_I))
    println(s"[G2] injector_fire_while_busy: busy at 2nd write=$busyAtSecond refused=$refused " +
            s"injPending high for $pendingHigh cycles, injected payload appears $hits time(s)")
    watch.terminate()
    assert(busyAtSecond, "the injector was already idle at the second write — the test would be vacuous")
    assert(refused, "a second INJ_FIRE while busy must set err_inj_busy")
    assert(hits == 1, s"the injected payload landed $hits times; a refused fire must produce NOTHING extra")
  }

  // ───────────────── qubic3 S1: live reads while the writer runs ─────────────────
  /** What the AXI bus shows during a live read: B handshakes (each bank is one burst at a 512-B-aligned base, so
   *  the writer may claim `512 * bCount` bytes at most), R beats accepted while a write burst is open (AW taken, B
   *  not yet: the reader really ran while banks were being written) and in the same cycle as a W beat. */
  class LiveBus(dut: ReadoutDdrUplinkDut, cd: ClockDomain) {
    var bCount = 0L; var rDuringW = 0L; var rwSameCycle = 0L; var wOpen = 0; var rBeats = 0L
    var awCount = 0L; var wBeats = 0L; var openCycles = 0L; var cycles = 0L
    private val d = dut.up.io.ddr
    private val mon = fork {
      while (true) {
        cd.waitSampling()
        cycles += 1
        val aw = d.aw.valid.toBoolean && d.aw.ready.toBoolean
        val w = d.w.valid.toBoolean && d.w.ready.toBoolean
        val b = d.b.valid.toBoolean && d.b.ready.toBoolean
        val r = d.r.valid.toBoolean && d.r.ready.toBoolean
        if (aw) { wOpen += 1; awCount += 1 }
        if (w) wBeats += 1
        if (wOpen > 0) openCycles += 1
        if (r) { rBeats += 1; if (wOpen > 0) rDuringW += 1; if (w) rwSameCycle += 1 }
        if (b) { bCount += 1; wOpen -= 1 }
      }
    }
    def stop(): Unit = mon.terminate()
    def summary: String = s"bus: $cycles cycles, AW $awCount, W $wBeats, B $bCount, R $rBeats, write burst open " +
                          s"$openCycles cycles, R inside open writes $rDuringW, R with W in the same cycle $rwSameCycle"
  }

  /** The PS side of the live read, as `software/riscq/ddr.py::DdrStream` does it. It keeps the largest
   *  CUR_ADDR - base it has seen (the writer parks CUR_ADDR at the base on the final bank), reads only
   *  [sent, committed) through the drain engine (the AXIS sink, armed before RD_START, stands for the S2MM DMA;
   *  completion is TLAST, never rd_done) in whole banks up to `chunkMax`, and after write_done reads the tail to
   *  FINAL_ADDR. Every CUR_ADDR it samples must be a bank boundary that the B responses already cover. */
  class LivePs(h: Helper, bus: LiveBus, base: Long, footprint: Long, chunkMax: Long,
               ready: () => Boolean = null, chunkPick: () => Long = null) {
    var sent = 0L; var committed = 0L; var finalB = -1L; var parkSeen = false; var frontierAtEnd = 0L
    var statusSeen = BigInt(0); var curSamples = 0; var reads = 0; var readsLive = 0
    val words = mutable.ArrayBuffer[BigInt]()
    def poll(): Unit = {
      val s = h.status(); statusSeen |= s
      if (finalB >= 0) return
      if (h.bit(s, S_WRITE_DONE)) {
        finalB = (h.rd(FINAL_ADDR) - base).toLong
        assert(finalB >= committed && finalB % 32 == 0, s"final_addr - base = $finalB below the frontier $committed")
        if ((h.rd(CUR_ADDR) - base).toLong == 0) parkSeen = true
        frontierAtEnd = committed
        committed = finalB
      } else {
        val cur = h.rd(CUR_ADDR).toLong; curSamples += 1
        val rel = cur - base
        if (rel == 0) { if (committed > 0) parkSeen = true }
        else {
          assert(rel > 0 && rel % 512 == 0 && rel >= committed && rel <= footprint,
            f"CUR_ADDR 0x$cur%x is not a bank boundary in [committed $committed, footprint $footprint] above 0x$base%x")
          assert(rel <= 512L * bus.bCount, s"CUR_ADDR claims $rel B but only ${bus.bCount} write bursts completed")
          committed = rel
        }
      }
    }
    def readChunk(): Boolean = {
      val lim = if (chunkPick != null) chunkPick() else chunkMax
      val n = scala.math.min(committed - sent, lim)
      if (n <= 0) return false
      words ++= h.drainViaMmu(base + sent, n.toInt, ready = ready)
      sent += n; reads += 1; if (finalB < 0) readsLive += 1
      true
    }
    def done: Boolean = finalB >= 0 && sent >= finalB
    /** Poll and read until the run is read whole; `flushWhen` says when the program is over (then FLUSH once). */
    def loop(flushWhen: () => Boolean, idle: Int = 8): Unit = {
      var flushed = false; var guard = 0
      while (!done && guard < 400000) {
        poll()
        if (!readChunk()) h.ddrCd.waitSampling(idle)
        if (!flushed && flushWhen()) { h.wr(FLUSH, BigInt(1)); flushed = true }
        guard += 1
      }
      assert(done, s"the live read never finished: sent=$sent committed=$committed final=$finalB")
    }
  }

  /** The run as `drain()` certifies it, applied to what the live read collected: the words read live are the DDR
   *  image, word for word, and every core's words are its offers in order. */
  def checkLive(h: Helper, ps: LivePs, base: Long, exp: Map[Int, Seq[(Int, Int)]], what: String): Unit = {
    val s = h.status()
    val S = exp.values.map(_.size).sum
    for (i <- 0 until h.nch) assert(h.accepted(i) == exp.getOrElse(i, Nil).size, s"$what: core $i accepted")
    for (i <- 0 until h.nch) assert(h.rejected(i) == 0, s"$what: core $i rejected")
    val fatal = Seq(S_BRESP_ERR, S_RRESP_ERR, S_WRAPPED, S_OVF_ANY, S_CROSS_DROPPED, S_EARLY_LATE, S_ERR_BADSIZE,
      S_ERR_BADBASE, S_ERR_FLUSH_TIMEOUT, S_ERR_START_DROPPED, S_ERR_FLUSH_DROPPED, S_SKID_OVF, S_ERR_INJ_RANGE)
    for (b <- fatal) assert(!h.bit(s | ps.statusSeen, b), f"$what: fatal bit $b seen: 0x${s | ps.statusSeen}%x")
    assert(h.bit(s, S_WRITE_DONE), s"$what: write_done")
    assert(ps.finalB == 32L * ((S + 3) / 4), s"$what: final_addr - base = ${ps.finalB} for $S words")
    val got = ps.words.take(S)
    assert(ps.words.size - S >= 0 && ps.words.size - S <= 3, s"$what: ${ps.words.size} words read for $S")
    val mem = h.ddrWords(base, S)
    for (k <- 0 until S) assert(got(k) == mem(k), f"$what: live word $k 0x${got(k)}%x != DDR 0x${mem(k)}%x")
    val perTag = got.groupBy(w => ((w >> 56) & 0xff).toInt)
    for (i <- 0 until h.nch) {
      val e = exp.getOrElse(i, Nil).map { case (r, im) => tagWord(i, r, im) }
      assert(perTag.getOrElse(i, Nil).toSeq == e, s"$what: core $i's words differ from its offers")
    }
    assert(perTag.keySet.subsetOf((0 until h.nch).toSet), s"$what: stray tags ${perTag.keySet}")
  }

  /** Results on all `nch` cores, one per core per round, `gap` dsp cycles apart; returns the offers per core. */
  def produce(h: Helper, rounds: Int, gap: Int, salt: Int): mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]] = {
    val exp = mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]()
    for (round <- 0 until rounds) {
      val vals = (0 until h.nch).map(i => i -> ((0x100000 * salt + 0x100 * round + 16 * i) & 0x7fffffff,
                                                 (0x3000000 + 0x100 * round + i) & 0x7fffffff)).toMap
      h.resultAll(0 until h.nch, vals, holdCycles = 2, gapCycles = gap)
      for (i <- 0 until h.nch) exp.getOrElseUpdate(i, mutable.ArrayBuffer()) += vals(i)
    }
    exp
  }
  def frozen(e: mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]): Map[Int, Seq[(Int, Int)]] =
    e.map { case (k, v) => k -> v.toSeq }.toMap
  def footprintOf(words: Int): Long = ((8L * words + 511) / 512) * 512

  /** 18. S1 live frontier: 300 rounds on 4 cores (1200 words, 18 full banks and a 48-word tail), a bank every ~128
   *  DDR cycles into a memory that answers a write after 80 (so a write burst is open most of the time). The PS polls
   *  every ~200 cycles and reads what is committed in chunks of 1 to 8 banks -- reads longer than the drain engine's
   *  16-beat FIFO, behind a slow DMA, so they span the following banks' writes. Exact words, and the reader really
   *  ran while write bursts were open. (A one-bank read issued right after its B fits the FIFO in ~20 cycles and
   *  can finish before the next AW: AxiMemorySim answers reads at once.) */
  run("live_frontier", 19, memDelay = 80) { (dut, h) =>
    h.startRun()
    val bus = new LiveBus(dut, h.ddrCd)
    var prodDone = false
    var exp: mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]] = null
    val prod = fork { exp = produce(h, 300, 6, 1); prodDone = true }
    val ps = new LivePs(h, bus, BASE0, footprintOf(1200), 4096, ready = () => h.rng.nextInt(4) == 0,
                        chunkPick = () => 512L * (1 + h.rng.nextInt(8)))
    ps.loop(() => prodDone, idle = 200)
    prod.join(); bus.stop()
    checkLive(h, ps, BASE0, frozen(exp), "live_frontier")
    println(s"[G2] live_frontier: ${ps.reads} reads (${ps.readsLive} before write_done), ${ps.curSamples} CUR_ADDR " +
            s"samples; ${bus.summary}")
    assert(ps.readsLive >= 8, s"only ${ps.readsLive} reads before write_done: the read was not live")
    assert(bus.rDuringW > 0, "no R beat was accepted while a write burst was open: reads and writes never overlapped")
  }

  /** 19. S1 frontier race, 14 channels (the production poller): every bank is read the moment CUR_ADDR moves past it
   *  (one bank per read, the PS polls without pause), while the next bank is already being written. A bank fills every
   *  ~240 DDR cycles and the memory answers a write after 190, so the writer is busy ~90 % of the time: the next
   *  bank's AW follows a bank's B within a few dozen cycles, while the PS is reading the bank that B committed. */
  run("live_frontier_race_14ch", 20, nch = 14, memDelay = 190) { (dut, h) =>
    h.startRun()
    val bus = new LiveBus(dut, h.ddrCd)
    var prodDone = false
    var exp: mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]] = null
    val prod = fork { exp = produce(h, 64, 3 * 14 + 4, 2); prodDone = true }       // 896 words, 14 banks
    val ps = new LivePs(h, bus, BASE0, footprintOf(896), 512, ready = () => h.rng.nextInt(8) == 0)
    ps.loop(() => prodDone, idle = 1)
    prod.join(); bus.stop()
    checkLive(h, ps, BASE0, frozen(exp), "live_frontier_race_14ch")
    println(s"[G2] live_frontier_race_14ch: ${ps.reads} one-bank reads (${ps.readsLive} live); ${bus.summary}")
    assert(ps.readsLive >= 10 && bus.rDuringW > 0, "the race was not exercised")
  }

  /** 20. S1 park and small tails: runs of 0..200 words, each read live; the frontier must survive the park
   *  (CUR_ADDR back at the base), and the tail comes from FINAL_ADDR with 0..3 pad lanes. */
  run("live_park_small_tails", 21) { (dut, h) =>
    var base = BASE0
    for ((n, k) <- Seq(0, 1, 3, 5, 63, 64, 65, 127, 128, 129, 200).zipWithIndex) {
      h.startRun(base)
      val bus = new LiveBus(dut, h.ddrCd)
      var prodDone = false
      val exp = mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]()
      val prod = fork {
        for (j <- 0 until n) {
          val i = j % NCH; val v = (0x200000 * (k + 1) + 16 * j + i, 0x5000000 + 16 * j + i)
          h.result(i, v._1, v._2); exp.getOrElseUpdate(i, mutable.ArrayBuffer()) += v
        }
        prodDone = true
      }
      val ps = new LivePs(h, bus, base, footprintOf(n), 1024)
      ps.loop(() => prodDone)
      prod.join(); bus.stop()
      checkLive(h, ps, base, frozen(exp), s"live_park_small_tails n=$n")
      assert(h.rd(CUR_ADDR) == base, "CUR_ADDR must be parked at the base after the run")
      println(s"[G2] live_park_small_tails: $n words, final_addr - base = ${ps.finalB}, frontier before write_done " +
              s"${ps.frontierAtEnd} B (tail ${ps.finalB - ps.frontierAtEnd} B), park seen by the PS = ${ps.parkSeen}")
      base += 0x2000
    }
  }

  /** 21. S1 DMA back-pressure: the AXIS sink is held off for long spans (the DMA not draining) while results keep
   *  arriving at a slow memory; the writer must keep writing banks under a stalled read, and the read stays exact. */
  run("live_dma_backpressure", 22, memDelay = 30) { (dut, h) =>
    h.startRun()
    val bus = new LiveBus(dut, h.ddrCd)
    var prodDone = false
    var exp: mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]] = null
    var wDuringHold = 0L; var holding = false
    val hold = fork {
      while (true) { holding = true; h.ddrCd.waitSampling(300 + h.rng.nextInt(1500)); holding = false
                     h.ddrCd.waitSampling(50 + h.rng.nextInt(200)) }
    }
    val wmon = fork {
      while (true) { h.ddrCd.waitSampling()
        if (holding && dut.up.io.ddr.w.valid.toBoolean && dut.up.io.ddr.w.ready.toBoolean) wDuringHold += 1 }
    }
    val prod = fork { exp = produce(h, 200, 20, 3); prodDone = true }
    val ps = new LivePs(h, bus, BASE0, footprintOf(800), 2048, ready = () => !holding && h.rng.nextInt(3) != 0)
    ps.loop(() => prodDone)
    prod.join(); hold.terminate(); wmon.terminate(); bus.stop()
    checkLive(h, ps, BASE0, frozen(exp), "live_dma_backpressure")
    println(s"[G2] live_dma_backpressure: ${ps.readsLive} live reads, $wDuringHold W beats written while the DMA held " +
            s"TREADY low, ${bus.rDuringW} R beats inside open write bursts")
    assert(wDuringHold > 0, "no bank was written while the DMA back-pressured the read")
  }

  /** 22. S1 R-channel stalls on top of the bus stalls: the memory withholds R beats in random runs (the test wrapper
   *  never withdraws a beat the uplink has seen), so live reads stretch across many write bursts. */
  run("live_r_stall", 23) { (dut, h) =>
    h.startRun()
    val bus = new LiveBus(dut, h.ddrCd)
    var rStalled = 0L
    val rs = fork {
      var on = false; var left = 0
      while (true) {
        h.ddrCd.waitSampling()
        if (dut.io.rStall.toBoolean && dut.io.ddr.r.valid.toBoolean && !dut.up.io.ddr.r.valid.toBoolean) rStalled += 1
        if (left == 0) { on = h.rng.nextInt(3) == 0; left = 1 + h.rng.nextInt(12) } else left -= 1
        dut.io.rStall #= on
      }
    }
    var prodDone = false
    var exp: mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]] = null
    val prod = fork { exp = produce(h, 250, 18, 4); prodDone = true }
    val ps = new LivePs(h, bus, BASE0, footprintOf(1000), 4096)
    ps.loop(() => prodDone)
    prod.join(); rs.terminate(); dut.io.rStall #= false; bus.stop()
    checkLive(h, ps, BASE0, frozen(exp), "live_r_stall")
    println(s"[G2] live_r_stall: $rStalled R-stall cycles with a beat waiting, ${ps.readsLive} live reads, " +
            s"${bus.rDuringW} R beats inside open write bursts")
    assert(rStalled > 100, s"the R channel was barely stalled ($rStalled cycles)")
  }

  /** 23. S1 AXI error responses during a live read: a SLVERR on one write burst's B, then (next run) on one R beat of a
   *  live read. Each raises its sticky while the run is live, the PS sees it at its next poll, and the run must not
   *  certify. The words themselves still arrive (the beat is forwarded), and the next clean live run is exact. */
  run("live_axi_errors", 24) { (dut, h) =>
    for ((kind, k) <- Seq("B", "R").zipWithIndex) {
      val base = BASE0 + 0x10000L * k
      h.startRun(base)
      val bus = new LiveBus(dut, h.ddrCd)
      var prodDone = false
      var exp: mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]] = null
      var fired = false
      val errFork = fork {
        val d = dut.up.io.ddr
        if (kind == "B") {
          while (bus.bCount < 3) h.ddrCd.waitSampling()
          dut.io.bErr #= true                                    // the next B (the uplink takes B at once)
          var seenResp = -1
          while (seenResp <= 0) {
            h.ddrCd.waitSampling()
            if (d.b.valid.toBoolean && d.b.ready.toBoolean) seenResp = d.b.resp.toInt
          }
          h.ddrCd.waitSampling(); dut.io.bErr #= false; fired = true
        } else {
          while (bus.rBeats < 40) h.ddrCd.waitSampling()
          dut.io.rErr #= true                                    // the wrapper applies it to the next beat's first cycle
          while (!(d.r.valid.toBoolean && d.r.ready.toBoolean && d.r.resp.toInt != 0)) h.ddrCd.waitSampling()
          h.ddrCd.waitSampling(); dut.io.rErr #= false; fired = true
        }
      }
      val prod = fork { exp = produce(h, 150, 18, 5 + k); prodDone = true }
      val ps = new LivePs(h, bus, base, footprintOf(600), 1024)
      var seenLiveAt = -1L
      var guard = 0; var flushed = false
      while (!ps.done && guard < 400000) {
        ps.poll()
        val bitNow = if (kind == "B") S_BRESP_ERR else S_RRESP_ERR
        if (seenLiveAt < 0 && h.bit(ps.statusSeen, bitNow)) seenLiveAt = ps.sent
        if (!ps.readChunk()) h.ddrCd.waitSampling(8)
        if (!flushed && prodDone) { h.wr(FLUSH, BigInt(1)); flushed = true }
        guard += 1
      }
      prod.join(); errFork.join(); bus.stop()
      val bitWant = if (kind == "B") S_BRESP_ERR else S_RRESP_ERR
      assert(fired && seenLiveAt >= 0, s"$kind: the error response was not seen by the live read")
      assert(seenLiveAt < ps.finalB, s"$kind: the error was seen only after the run was read whole")
      assert(h.bit(h.status(), bitWant), s"$kind: the sticky did not survive to the end of the run")
      // the data still arrived: the live words are the DDR image
      val S = exp.values.map(_.size).sum
      val mem = h.ddrWords(base, S)
      for (j <- 0 until S) assert(ps.words(j) == mem(j), s"$kind: live word $j differs from DDR")
      println(s"[G2] live_axi_errors: SLVERR on $kind seen by the PS at byte $seenLiveAt of ${ps.finalB}; the run would " +
              s"be refused (status 0x${h.status().toString(16)})")
    }
    // and a clean live run afterwards
    h.startRun(BASE0 + 0x40000L)
    val bus = new LiveBus(dut, h.ddrCd)
    var prodDone = false
    var exp: mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]] = null
    val prod = fork { exp = produce(h, 100, 18, 9); prodDone = true }
    val ps = new LivePs(h, bus, BASE0 + 0x40000L, footprintOf(400), 1024)
    ps.loop(() => prodDone)
    prod.join(); bus.stop()
    checkLive(h, ps, BASE0 + 0x40000L, frozen(exp), "live_axi_errors (clean run after)")
  }

  /** 24. S1 a DMA timeout with AXIS data outstanding, and the recovery the worker uses. A live read of two committed
   *  banks meets a DMA that never takes a beat (TREADY low): the drain engine's FIFO fills, R beats stay owed, and the
   *  chunk stays in flight -- its read lock lasts until its TLAST (CONTRACT.md I7). The run is flushed meanwhile:
   *  write_done comes, but FLUSH does not clear the lock: DIAG.run_idle stays 0 and BASE_RESET is refused
   *  (err_base_busy). Re-arming the DMA lets the whole chunk drain to TLAST, intact and in order; then run_idle reads
   *  1, BASE_RESET is accepted and the next run, read live, is exact. Nothing is cleared by hand. */
  run("live_dma_timeout_recovery", 25) { (dut, h) =>
    h.startRun()
    val bus = new LiveBus(dut, h.ddrCd)
    val exp1 = produce(h, 48, 18, 7)                                  // 192 words: three full banks
    var n = 0
    while ((h.rd(CUR_ADDR).toLong - BASE0) < 1024 && n < 10000) { h.ddrCd.waitSampling(10); n += 1 }
    assert(h.rd(CUR_ADDR).toLong - BASE0 >= 1024, "two banks were never committed")
    dut.io.rd.ready #= false                                          // the DMA takes nothing: its transfer times out
    h.wr(RD_BASE, BigInt(BASE0)); h.wr(RD_SIZE, BigInt(1024)); h.wr(RD_START, BigInt(1))
    var taken = 0
    val watch = fork { while (true) { h.ddrCd.waitSampling()
      if (dut.io.rd.valid.toBoolean && dut.io.rd.ready.toBoolean) taken += 1 } }
    h.ddrCd.waitSampling(3000)
    assert(taken == 0, s"the stalled DMA took $taken beats")
    val s = h.flushRun()                                              // write_done, with the chunk still in flight
    val d0 = h.rd(DIAG)
    assert(((d0 >> 7) & 1) == 0, f"run_idle with a chunk in flight: diag=0x$d0%x (FLUSH must not clear the read lock)")
    h.wr(BASE_RESET, BigInt(1))
    assert(h.bit(h.status(), S_ERR_BASE_BUSY), "BASE_RESET was accepted while a chunk held the read lock")
    h.wr(STATUS, BigInt(1) << S_ERR_BASE_BUSY)
    // the recovery: re-arm the DMA and take the rest of the chunk up to TLAST (here: all of it, nothing was taken)
    val beats = mutable.ArrayBuffer[BigInt](); var sawLast = false; var g = 0
    while (!sawLast && g < 100000) {
      dut.io.rd.ready #= h.rng.nextInt(3) != 0
      h.ddrCd.waitSampling(); g += 1
      if (dut.io.rd.valid.toBoolean && dut.io.rd.ready.toBoolean) {
        beats += dut.io.rd.fragment.toBigInt; if (dut.io.rd.last.toBoolean) sawLast = true }
    }
    dut.io.rd.ready #= true
    watch.terminate()
    assert(sawLast && beats.size == 32, s"the drained chunk: TLAST=$sawLast after ${beats.size} beats (32 expected)")
    val words = beats.flatMap(b => (0 until 4).map(k => (b >> (64 * k)) & ((BigInt(1) << 64) - 1)))
    assert(words.toSeq == h.ddrWords(BASE0, 128), "the drained chunk is not the DDR image of the two banks")
    h.ddrCd.waitSampling(20)
    val d1 = h.rd(DIAG)
    assert(((d1 >> 7) & 1) == 1, f"not idle after the chunk's TLAST: diag=0x$d1%x")
    bus.stop()
    println(f"[G2] live_dma_timeout_recovery: lock held across FLUSH (diag 0x$d0%x, BASE_RESET refused); drained " +
            f"${beats.size} beats to TLAST, then diag 0x$d1%x (run_idle); next live run follows")
    // the port is usable again
    val base2 = BASE0 + 0x10000L
    h.startRun(base2)
    val bus2 = new LiveBus(dut, h.ddrCd)
    var prodDone = false
    var exp2: mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]] = null
    val prod = fork { exp2 = produce(h, 100, 18, 8); prodDone = true }
    val ps = new LivePs(h, bus2, base2, footprintOf(400), 1024)
    ps.loop(() => prodDone)
    prod.join(); bus2.stop()
    checkLive(h, ps, base2, frozen(exp2), "live_dma_timeout_recovery (the next run)")
  }

  if (STALLS) {
    println(s"[G2] stall injection totals: AW=${stallTotals(0)} AR=${stallTotals(1)} B=${stallTotals(2)} W=${stallTotals(3)} cycles")
    assert(stallTotals.forall(_ > 0), "a channel was never stalled: the stall pass would be vacuous for it")
  }
  println(s"[G2] all scenarios PASS${if (STALLS) " (with AW/AR/B/W stall injection)" else ""}")
}
