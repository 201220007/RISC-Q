package riscq.ddr.sim

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import spinal.lib.bus.amba4.axi.sim.{AxiMemorySim, AxiMemorySimConfig, Axi4Master}
import riscq.ddr._
import scala.collection.mutable
import scala.util.Random

/**
 * P3a uplink-level checks of the rewritten datapath that G2 does not make (plan r2 items 2 and 5):
 *   - `deterministic_banks`: runs of 0, 1, 63, 64, 65, 200 words. Each run writes exactly floor(S/64) full-bank
 *     bursts plus one burst for a non-empty final bank (F4), and afterwards DIAG reads rd_empty=1, able_to_read=0,
 *     run_idle=1. The same run at a base whose data and drain cross a 4 KiB page: no AW/AR crosses one (F1).
 *   - `start_dsp_dead_reset_held`, `start_dsp_dead_no_reset`, `start_ddr_dead`: stopped-clock startup of one side.
 *   - `reset_dsp_mid_run`, `reset_ddr_mid_run`: a lone reset of either domain while the writer waits for the B of
 *     the first bank, with a partial bank and results pending; `reset_dsp_second_bank`: the same during the SECOND
 *     bank's B wait (the other parity of the cbuf's hand-over toggles) with a flush pending on the dsp side.
 * Every scenario checks that nothing reaches the AXI master while it must not, and that the next run is byte-exact
 * with exactly the expected bursts. Clocks are driven by hand here (dsp 4, ddr 6 time units: G2's 2:3 ratio), so
 * either can be stopped.
 * Verilator runs with `--x-initial 0`, i.e. never-reset registers start at the FPGA power-up value 0.
 * Run: mill-1.1.0 runMain riscq.ddr.sim.ReadoutDdrUplinkCdcSim
 */
object ReadoutDdrUplinkCdcSim extends App {
  import ReadoutDdrRegs._

  val NCH = 4

  def tagWord(tag: Int, real: Int, imag: Int): BigInt = {
    val r = BigInt(real & 0xFFFFFFFFL) >> 4
    val i = BigInt(imag & 0xFFFFFFFFL) >> 4
    (BigInt(tag) << 56) | (r << 28) | i
  }

  /** A clock we can stop and start. */
  class ManualClock(sig: Bool, halfPeriod: Long) {
    var running = false
    private var level = false
    sig #= false
    fork { while (true) { if (running) { level = !level; sig #= level }; sleep(halfPeriod) } }
  }

  case class AxiEvent(t: Long, kind: String, addr: Long, len: Int)

  class Bench(val dut: ReadoutDdrUplinkDut, val rng: Random, memDelay: Int) {
    val dspClk = new ManualClock(dut.io.dspClk, 2)
    val ddrClk = new ManualClock(dut.io.ddrClk, 3)
    val ddrCd  = ClockDomain(dut.io.ddrClk, dut.io.ddrRst)
    val dspCd  = ClockDomain(dut.io.dspClk, dut.io.dspRst)
    for (i <- 0 until NCH) { dut.io.results(i).valid #= false; dut.io.results(i).payload.res #= false
      dut.io.results(i).payload.real #= 0; dut.io.results(i).payload.imag #= 0 }
    dut.io.rd.ready #= true
    val axi = mutable.ArrayBuffer[AxiEvent]()
    var mem: AxiMemorySim = null
    var ctrl: Axi4Master = null
    var wOpen = false                  // last W beat of a burst accepted, its B not yet
    var nB = 0

    def now: Long = simTime()
    def startMonitors(): Unit = {
      mem = AxiMemorySim(dut.io.ddr, ddrCd, AxiMemorySimConfig(maxOutstandingReads = 2, maxOutstandingWrites = 2,
        readResponseDelay = memDelay, writeResponseDelay = memDelay))
      mem.start()
      ctrl = Axi4Master(dut.io.ctrl, ddrCd, "ctrl")
      fork {
        while (true) {
          ddrCd.waitRisingEdge()
          val d = dut.io.ddr
          if (d.aw.valid.toBoolean && d.aw.ready.toBoolean) axi += AxiEvent(now, "AW", d.aw.addr.toLong, d.aw.len.toInt + 1)
          if (d.ar.valid.toBoolean && d.ar.ready.toBoolean) axi += AxiEvent(now, "AR", d.ar.addr.toLong, d.ar.len.toInt + 1)
          if (d.w.valid.toBoolean && d.w.ready.toBoolean && d.w.last.toBoolean) wOpen = true
          if (d.b.valid.toBoolean && d.b.ready.toBoolean) { wOpen = false; nB += 1 }
          if (d.w.valid.toBoolean && d.w.ready.toBoolean) axi += AxiEvent(now, "W", 0, 0)
        }
      }
    }
    def events(kind: String, from: Long = 0): Seq[AxiEvent] = axi.filter(e => e.kind == kind && e.t >= from).toSeq
    def noAxiSince(t0: Long, what: String): Unit = {
      val ev = axi.filter(_.t >= t0)
      assert(ev.isEmpty, s"$what: unexpected AXI activity ${ev.take(4)}")
    }

    def le(v: BigInt, n: Int): List[Byte] = List.tabulate(n)(i => ((v >> (8 * i)) & 0xff).toByte)
    def wr(off: Int, v: BigInt): Unit = {
      var done = false
      ctrl.writeCB(off, le(v, 4)) { done = true }
      var n = 0
      while (!done && n < 20000) { ddrCd.waitSampling(); n += 1 }
      assert(done, s"ctrl write to 0x${off.toHexString} never completed")
    }
    def rd(off: Int): BigInt = {
      var r: Option[BigInt] = None
      ctrl.readSingle(off, 4) { bs => r = Some(bs.reverse.foldLeft(BigInt(0))((a, b) => (a << 8) | (b & 0xff))) }
      var n = 0
      while (r.isEmpty && n < 20000) { ddrCd.waitSampling(); n += 1 }
      assert(r.isDefined, s"ctrl read from 0x${off.toHexString} never completed")
      r.get
    }
    def status(): BigInt = rd(STATUS)
    def bit(s: BigInt, b: Int): Boolean = ((s >> b) & 1) == 1

    def startRun(base: Long): Unit = {
      wr(STATUS, BigInt(STICKY_MASK))
      wr(WR_BASE, BigInt(base)); wr(BASE_RESET, BigInt(1))
      var n = 0
      while (!bit(status(), S_RUN_ACTIVE) && n < 400) { ddrCd.waitSampling(10); n += 1 }
      val s = status()
      assert(bit(s, S_RUN_ACTIVE), f"run never became active, status=0x$s%x diag=0x${rd(DIAG)}%x")
      var m = 0
      while (!dut.io.dspAdmit.toBoolean && m < 400) { dspCd.waitSampling(10); m += 1 }
      assert(dut.io.dspAdmit.toBoolean, "dsp_admit never rose")
      assert(rd(RUN_BASE) == base)
    }
    def flushRun(): BigInt = {
      wr(FLUSH, BigInt(1))
      var n = 0; var s = status()
      while (bit(s, S_FLUSH_BUSY) && n < 20000) { ddrCd.waitSampling(20); s = status(); n += 1 }
      assert(!bit(s, S_FLUSH_BUSY), "flush never completed")
      assert(bit(s, S_WRITE_DONE), f"write_done not set, status=0x$s%x")
      s
    }
    def result(i: Int, real: Int, imag: Int): Unit = {
      val f = dut.io.results(i)
      f.payload.real #= real; f.payload.imag #= imag; f.payload.res #= real < 0
      f.valid #= true;  dspCd.waitSampling(2)
      f.valid #= false; dspCd.waitSampling(4)
    }
    /** `n` results round-robin over the cores; returns the expected per-core sequences. */
    def push(n: Int, salt: Int): Map[Int, Seq[(Int, Int)]] = {
      val exp = mutable.Map[Int, mutable.ArrayBuffer[(Int, Int)]]()
      for (k <- 0 until n) {
        val i = k % NCH
        val v = (0x100000 * (salt + 1) + 16 * k + i, 0x7000000 + 0x10000 * salt + 16 * k + i)
        result(i, v._1, v._2)
        exp.getOrElseUpdate(i, mutable.ArrayBuffer()) += v
      }
      exp.map { case (k, v) => k -> v.toSeq }.toMap
    }
    def ddrWords(base: Long, nWords: Int): Seq[BigInt] = {
      val bytes = mem.memory.readArray(base, nWords * 8L)
      (0 until nWords).map(k => (0 until 8).foldLeft(BigInt(0))((a, b) => a | (BigInt(bytes(k * 8 + b) & 0xff) << (8 * b))))
    }
    /** The drain contract, as `ddr.py::drain` checks it, plus the exact write-burst plan (F1 + F4). */
    def checkRun(base: Long, exp: Map[Int, Seq[(Int, Int)]], s: BigInt, awFrom: Long): Unit = {
      val S = exp.values.map(_.size).sum
      val fa = rd(FINAL_ADDR).toLong
      assert(fa == base + 32L * ((S + 3) / 4), f"final_addr 0x$fa%x != base + 32*ceil($S/4)")
      for (i <- 0 until NCH) {
        assert(rd(ACCEPTED + 4 * i) == exp.getOrElse(i, Nil).size, s"core $i accepted")
        assert(rd(REJECTED + 4 * i) == 0, s"core $i rejected")
      }
      val fatal = Seq(S_BRESP_ERR, S_RRESP_ERR, S_WRAPPED, S_OVF_ANY, S_CROSS_DROPPED, S_EARLY_LATE, S_ERR_BADSIZE,
        S_ERR_BADBASE, S_ERR_FLUSH_TIMEOUT, S_ERR_START_DROPPED, S_ERR_FLUSH_DROPPED, S_SKID_OVF, S_ERR_INJ_RANGE)
      for (b <- fatal) assert(!bit(s, b), f"fatal bit $b set: status=0x$s%x")
      val words = ddrWords(base, S)
      val perTag = mutable.Map[Int, mutable.ArrayBuffer[BigInt]]()
      for (w <- words) perTag.getOrElseUpdate(((w >> 56) & 0xff).toInt, mutable.ArrayBuffer()) += w
      for (i <- 0 until NCH) {
        val e = exp.getOrElse(i, Nil).map { case (r, im) => tagWord(i, r, im) }
        assert(perTag.getOrElse(i, mutable.ArrayBuffer()).toSeq == e, s"tag $i: DDR words differ from the expectation")
      }
      assert(perTag.keySet.subsetOf((0 until NCH).toSet), s"stray tags ${perTag.keySet}")
      // write bursts of this run: floor(S/64) full banks + one non-empty final bank, contiguous, page-bounded
      val aws = events("AW", awFrom)
      val nFull = S / 64; val tail = S % 64
      val plan = mutable.ArrayBuffer[(Long, Int)]()
      var a = base
      for (_ <- 0 until nFull) { plan ++= split(a, 16); a += 512 }
      if (tail > 0) plan ++= split(a, (tail + 3) / 4)
      assert(aws.map(e => (e.addr, e.len)) == plan.toSeq, s"AW plan ${aws.map(e => (e.addr.toHexString, e.len))} != ${plan.map(p => (p._1.toHexString, p._2))}")
      // quiescence (F4): the writer owns nothing and base_reset would be accepted
      val d = rd(DIAG)
      assert(bit(d, 1) && !bit(d, 2) && bit(d, 7) && bit(d, 0), f"not quiescent after the run: diag=0x$d%x")
    }
    def split(addr: Long, beats: Int): Seq[(Long, Int)] = {
      val first = scala.math.min(beats, ((0x1000 - (addr & 0xFFF)) / 32).toInt)
      if (first == beats) Seq((addr, beats)) else Seq((addr, first), (addr + 32L * first, beats - first))
    }
    /** Drain [base, base+n) through the drain engine and the AXIS port; AR bursts must be page-bounded (F1). */
    def drain(base: Long, nBytes: Int): Seq[BigInt] = {
      val t0 = now
      wr(RD_BASE, BigInt(base)); wr(RD_SIZE, BigInt(nBytes))
      val beats = mutable.ArrayBuffer[BigInt]()
      var sawLast = false
      val sink = fork {
        while (!sawLast) {
          dut.io.rd.ready #= rng.nextInt(4) != 0
          ddrCd.waitSampling()
          if (dut.io.rd.valid.toBoolean && dut.io.rd.ready.toBoolean) {
            beats += dut.io.rd.fragment.toBigInt
            if (dut.io.rd.last.toBoolean) sawLast = true
          }
        }
        dut.io.rd.ready #= true
      }
      wr(RD_START, BigInt(1))
      var n = 0
      while (!sawLast && n < 40000) { ddrCd.waitSampling(10); n += 1 }
      assert(sawLast, "TLAST never arrived")
      sink.join()
      val ars = events("AR", t0)
      assert(ars.map(_.len).sum == nBytes / 32, "AR beats != drain size")
      for (e <- ars) assert((0x1000 - (e.addr & 0xFFF)) >= 32L * e.len, f"AR 0x${e.addr}%x x${e.len} crosses 4 KiB")
      assert(beats.size == nBytes / 32)
      beats.flatMap(b => (0 until 4).map(k => (b >> (64 * k)) & ((BigInt(1) << 64) - 1))).toSeq
    }
    def waitNs(ns: Long): Unit = sleep(ns)
  }

  def run(name: String, seed: Int, memDelay: Int = 0)(body: Bench => Unit): Unit = {
    SimConfig.withConfig(SpinalConfig()).addSimulatorFlag("-Wno-MULTIDRIVEN").addSimulatorFlag("--x-initial 0")
      .compile(ReadoutDdrUplinkDut(ReadoutDdrUplinkParams(numCh = NCH)))
      .doSim(name, seed = seed) { dut =>
        SimTimeout(40000000)
        val b = new Bench(dut, new Random(seed), memDelay)
        body(b)
        println(s"[P3a-CDC] PASS $name")
      }
  }

  /** Both clocks running, both resets pulsed together: the normal bring-up. */
  def normalStart(b: Bench): Unit = {
    b.dut.io.dspRst #= true; b.dut.io.ddrRst #= true
    b.dspClk.running = true; b.ddrClk.running = true
    b.waitNs(200)
    b.dut.io.dspRst #= false; b.dut.io.ddrRst #= false
    b.startMonitors()
    b.ddrCd.waitSampling(20); b.dspCd.waitSampling(20)
  }

  /** One complete run at `base` with `n` words; returns the time the run started (for the AW window). */
  def fullRun(b: Bench, base: Long, n: Int, salt: Int, drainToo: Boolean = true): Unit = {
    val t0 = b.now
    b.startRun(base)
    val exp = b.push(n, salt)
    val s = b.flushRun()
    b.checkRun(base, exp, s, t0)
    if (drainToo && n > 0) {
      val words = b.ddrWords(base, n)
      val got = b.drain(base, 32 * ((n + 3) / 4)).take(n)
      assert(got == words, "AXIS drain differs from DDR")
    }
  }

  // 1. deterministic bank presentation and quiescence (F4), page-bounded AW/AR (F1)
  run("deterministic_banks", 1) { b =>
    normalStart(b)
    var base = 0x10000L
    for ((n, k) <- Seq(0, 1, 63, 64, 65, 200).zipWithIndex) {
      fullRun(b, base, n, k)
      println(s"[P3a-CDC] deterministic_banks: $n words -> ${b.events("AW").size} AW so far, quiescent")
      base += 0x1000
    }
    // a run whose data (and so its drain) crosses a 4 KiB page: [0x21E00, 0x22440)
    fullRun(b, 0x21E00L, 200, 9)
    assert(b.events("AR").exists(e => ((e.addr + 32L * e.len - 1) & ~0xFFFL) != (e.addr & ~0xFFFL)) == false)
  }

  // 2. dsp clock dead at startup, dsp reset held: the symmetric reset keeps the ddr side in reset -> no AXI at all
  run("start_dsp_dead_reset_held", 2) { b =>
    b.dut.io.dspRst #= true; b.dut.io.ddrRst #= true
    b.ddrClk.running = true
    b.waitNs(200)
    b.dut.io.ddrRst #= false
    // no ctrl master yet: the control slave is held in reset along with the rest of the ddr side
    val mon = { b.startMonitors(); b.now }
    b.waitNs(20000)
    b.noAxiSince(0, "dsp clock dead, dsp reset held")
    b.dspClk.running = true
    b.waitNs(400)
    b.dut.io.dspRst #= false
    b.ddrCd.waitSampling(50)
    fullRun(b, 0x4000L, 70, 1)
  }

  // 3. dsp clock dead, dsp reset never asserted (power-up: the dsp registers sit at their INIT value 0). The ddr side
  //    runs; a BASE_RESET issued meanwhile pends on the dead crossing. Nothing may reach the AXI master; when the
  //    clock starts the pending start is either completed or reported dropped, and a run afterwards is exact.
  run("start_dsp_dead_no_reset", 3) { b =>
    b.dut.io.dspRst #= false; b.dut.io.ddrRst #= true
    b.ddrClk.running = true
    b.waitNs(200)
    b.dut.io.ddrRst #= false
    b.startMonitors()
    b.ddrCd.waitSampling(20)
    val s0 = b.status()
    assert(!b.bit(s0, S_RUN_ACTIVE), f"run active with the dsp clock dead: 0x$s0%x")
    b.wr(WR_BASE, BigInt(0x6000)); b.wr(BASE_RESET, BigInt(1))
    b.waitNs(20000)
    assert(!b.bit(b.status(), S_RUN_ACTIVE), "a start completed across a dead clock")
    b.noAxiSince(0, "dsp clock dead, no reset")
    b.dspClk.running = true
    b.waitNs(2000)
    val s1 = b.status()
    println(f"[P3a-CDC] start_dsp_dead_no_reset: after the dsp clock started status=0x$s1%x " +
            s"(run_active=${b.bit(s1, S_RUN_ACTIVE)} err_start_dropped=${b.bit(s1, S_ERR_START_DROPPED)})")
    assert(b.bit(s1, S_RUN_ACTIVE) || b.bit(s1, S_ERR_START_DROPPED), "the pending start neither completed nor dropped")
    b.noAxiSince(0, "dsp clock just started, no data yet")
    if (b.bit(s1, S_RUN_ACTIVE)) { b.flushRun() }
    fullRun(b, 0x8000L, 70, 2)
  }

  // 4. ddr clock dead at startup: results offered to the dsp side meanwhile are rejected (admission closed); the run
  //    after the ddr clock starts is exact.
  run("start_ddr_dead", 4) { b =>
    b.dut.io.dspRst #= true; b.dut.io.ddrRst #= true
    b.dspClk.running = true
    b.waitNs(200)
    b.dut.io.dspRst #= false
    for (k <- 0 until 12) b.result(k % NCH, 0x55500 + k, 0x66600 + k)   // no run: must not be admitted
    assert(!b.dut.io.dspAdmit.toBoolean)
    b.waitNs(10000)
    b.ddrClk.running = true
    b.waitNs(200)
    b.dut.io.ddrRst #= false
    b.startMonitors()
    b.ddrCd.waitSampling(50)
    fullRun(b, 0xA000L, 70, 3)
  }

  /** Put a run in the state "a bank burst waits for its B, a partial bank and results are pending", then hit one
   *  domain's reset alone. Afterwards the uplink must be quiescent and the next run exact, with no stale data. */
  def midRunReset(b: Bench, dsp: Boolean): Unit = {
    normalStart(b)
    b.startRun(0x20000L)
    // 70 words: one full bank goes to DDR (slow B), 6 more wait in the cbuf, then 10 more fill FIFOs/skid/cbuf
    b.push(70, 7)
    var n = 0
    while (!b.wOpen && n < 100000) { b.ddrCd.waitSampling(); n += 1 }
    assert(b.wOpen, "the first bank burst never reached its B wait")
    val nb = b.nB
    for (k <- 0 until 10) { val f = b.dut.io.results(k % NCH); f.payload.real #= 0x999000 + k; f.valid #= true
      b.dspCd.waitSampling(); f.valid #= false; b.dspCd.waitSampling() }
    val t0 = b.now
    if (dsp) { b.dut.io.dspRst #= true; b.waitNs(160); b.dut.io.dspRst #= false }
    else     { b.dut.io.ddrRst #= true; b.waitNs(240); b.dut.io.ddrRst #= false }
    b.ddrCd.waitSampling(20)
    // the reset hit both halves (symmetric uplink reset): the run is gone, no write_done, nothing pending
    val s = b.status()
    assert(!b.bit(s, S_RUN_ACTIVE) && !b.bit(s, S_WRITE_DONE), f"run survived a lone ${if (dsp) "dsp" else "ddr"} reset: 0x$s%x")
    var m = 0
    while (b.wOpen && m < 20000) { b.ddrCd.waitSampling(); m += 1 }   // the stray B of the cut burst drains
    b.waitNs(5000)
    assert(b.events("AW", t0).isEmpty && b.events("W", t0).isEmpty, s"AXI writes after the reset: ${b.events("AW", t0)}")
    val d = b.rd(DIAG)
    assert(((d >> 1) & 1) == 1 && ((d >> 7) & 1) == 1, f"not quiescent after the reset: diag=0x$d%x")
    println(s"[P3a-CDC] ${if (dsp) "dsp" else "ddr"} reset in the B wait: status=0x${s.toString(16)} diag=0x${d.toString(16)} " +
            s"strayB=${b.nB - nb}")
    // the next run is exact and carries none of the pre-reset words
    fullRun(b, 0x30000L, 130, 8)
  }

  // 5./6. lone reset of either domain in the middle of a run (writer waiting for B, data pending everywhere)
  run("reset_dsp_mid_run", 5, memDelay = 300) { b => midRunReset(b, dsp = true) }
  run("reset_ddr_mid_run", 6, memDelay = 300) { b => midRunReset(b, dsp = false) }

  // 7. lone dsp reset during the SECOND bank's B wait (hand-over toggles at even parity), a flush pending on the dsp
  //    side (quiet never reached: the datapath is stalled) and a full third bank waiting at its last slot
  run("reset_dsp_second_bank", 7, memDelay = 400) { b =>
    normalStart(b)
    b.startRun(0x40000L)
    b.push(3 * 64 + 5, 11)
    b.wr(FLUSH, BigInt(1))
    var n = 0
    while (!(b.nB >= 1 && b.wOpen) && n < 400000) { b.ddrCd.waitSampling(); n += 1 }
    assert(b.nB >= 1 && b.wOpen, "the second bank burst never reached its B wait")
    assert(b.events("AW").size == 2)
    val t0 = b.now
    b.dut.io.dspRst #= true; b.waitNs(160); b.dut.io.dspRst #= false
    var m = 0
    while (b.wOpen && m < 20000) { b.ddrCd.waitSampling(); m += 1 }
    b.waitNs(8000)
    assert(b.events("AW", t0).isEmpty, s"a stale bank was written after the reset: ${b.events("AW", t0)}")
    val s = b.status()
    assert(!b.bit(s, S_FLUSH_BUSY) && !b.bit(s, S_RUN_ACTIVE) && !b.bit(s, S_WRITE_DONE), f"status 0x$s%x")
    fullRun(b, 0x50000L, 90, 12)
  }

  println("[P3a-CDC] all scenarios PASS")
}
