package riscq.ddr

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba4.axi.{Axi4, Axi4Config, Axi4SlaveFactory}
import riscq.soc.link.ReadoutResult

/**
 * Geometry of the readout→DDR uplink. Everything is derived from these; no literal channel counts
 * anywhere (qubic3 plan: "适用到其他任意版本").
 */
case class ReadoutDdrUplinkParams(
    numCh: Int,                       // = qubitNum
    accWidth: Int = 32,               // decoder integral width (ReadoutResult)
    fifoDepth: Int = 16,              // per-core result FIFO (plan v3 §A)
    skidDepth: Int = 8,               // poller→cbuf skid FIFO (plan v2 §2.2 / v3 §B.2)
    flushQuiet: Int = 8,              // consecutive quiet cycles before the flush commits (v3 §B.3)
    cbufAddrWidth: Int = 4,           // 2^4 = 16 × 256-bit beats per bank (QubiC CBUF_ADDR_WIDTH)
    axiAddrWidth: Int = 32,
    axiDataWidth: Int = 256,
    axiIdWidth: Int = 4,
    ctrlIdWidth: Int = 2,
    flushTimeoutLog2: Int = 22,       // flush watchdog (v5 §1): 2^22 ui cycles ≈ 12 ms @ 333 MHz
    rejectedWidth: Int = 16,          // per-core saturating Gray counters (v7 §2)
    rstHoldLog2: Int = 16             // r1: bound of the dsp-reset hold, 2^16 ui cycles ≈ 197 µs @ 333 MHz
) {
  // tag is 8 bits, but the register map bounds this harder: OVERFLOW is a single 32-bit word and the
  // ACCEPTED array (0x100 + 4i) must not reach REJECTED (0x180).  (r09-#8)
  require(numCh >= 1 && numCh <= 32, s"numCh=$numCh: the register map supports 1..32 channels")
  require(skidDepth >= 4)
  val wordWidth     = 64
  val beatWords     = axiDataWidth / wordWidth           // 4
  val bankBeats     = 1 << cbufAddrWidth
  val bankBytes     = bankBeats * axiDataWidth / 8        // 512
  val throttleLevel = skidDepth - 3
  // full sideband set so the existing Axi4VivadoHelper tags every field (sidebands are tied to 0)
  val ddrAxiConfig  = Axi4Config(addressWidth = axiAddrWidth, dataWidth = axiDataWidth, idWidth = axiIdWidth)
  val ctrlAxiConfig = Axi4Config(addressWidth = 16, dataWidth = 32, idWidth = ctrlIdWidth)
}

/** Register map (byte offsets on the `ctrl` AXI4 slave). Mirrored in software/riscq/ddr.py. */
object ReadoutDdrRegs {
  val RD_START    = 0x00   // W pulse
  val WR_BASE     = 0x08   // RW (validated: 512-B aligned, bit31 clear)
  val RUN_BASE    = 0x0C   // RO
  val RD_BASE     = 0x10   // RW (32-B aligned)
  val RD_SIZE     = 0x14   // RW (bytes, multiple of 32, <= 32 MiB)
  val FINAL_ADDR  = 0x18   // RO
  val CUR_ADDR    = 0x20   // RO
  val BASE_RESET  = 0x24   // W pulse  (run start)
  val FLUSH       = 0x28   // W pulse  (run stop)
  val STATUS      = 0x2C   // RO; W1C on sticky bits
  val OVERFLOW    = 0x30   // RO (snapshot)
  val INJ_REAL    = 0x40   // W
  val INJ_IMAG    = 0x44   // W
  val INJ_CORE    = 0x48   // W
  val INJ_FIRE    = 0x4C   // W pulse
  val NUM_CH      = 0x50   // RO geometry
  val GEOMETRY    = 0x54   // RO: [7:0] fifoDepth [15:8] skidDepth [23:16] cbufAddrWidth [31:24] flushQuiet
  // r20-#7: keep this list in step with the `diag(n) :=` assignments below AND with DIAG_NAMES in
  // software/riscq/ddr_regs.py (pinned by tests/test_ddr_contract.py).
  val DIAG        = 0x58   // RO: [0]writer_idle [1]cbuf_rd_empty [2]cbuf_able_to_read [3]start_busy
                           //     [4]start_pend [5]flush_cross_busy [6]write_done_seen [7]run_idle
                           //     [8]snap_arrived [9]ddr_calib_done (MIG c0_init_calib_complete; the
                           //     authoritative copy is the SoC's HOST-domain status register, which is
                           //     readable even when this whole ui_clk block is dead -- r19-#4)
  // r1 (plan r2 #6): RESERVED for the future host STOP word. No hardware: reads 0, writes are ignored.
  val STOP        = 0x5C
  val ACCEPTED    = 0x100  // RO  + 4*i (snapshot)
  val REJECTED    = 0x180  // RO  + 4*i (live, Gray-crossed)
  val MAX_RD_SIZE = 0x2000000 // 32 MiB
  // STATUS bits (plan v7)
  val S_RD_BUSY = 0; val S_RD_DONE = 1; val S_WRITE_DONE = 2; val S_BRESP_ERR = 3; val S_RRESP_ERR = 4
  val S_ERR_BADSIZE = 5; val S_ERR_BADBASE = 6; val S_FLUSH_BUSY = 7; val S_INJ_BUSY = 8; val S_WRAPPED = 9
  val S_OVF_ANY = 10; val S_CROSS_DROPPED = 11; val S_RUN_ACTIVE = 12; val S_EARLY_LATE = 13
  val S_ERR_INJ_BUSY = 14; val S_ERR_BASE_BUSY = 15; val S_ERR_FLUSH_REFUSED = 16; val S_ERR_FLUSH_TIMEOUT = 17
  val S_DSP_IN_RESET = 18; val S_DSP_ADMIT = 19; val S_DDR_IN_RESET = 20; val S_ERR_START_DROPPED = 21
  val S_ERR_FLUSH_DROPPED = 22; val S_SKID_OVF = 23; val S_ERR_INJ_RANGE = 24
  // r1: the uplink's DDR half had to be FORCED into reset (dsp-side reset request, hold timed out) while AXI
  // transactions were still outstanding. Not W1C and not cleared by BASE_RESET: only the raw DDR reset, which
  // also resets the AXI fabric (psr_ddr), clears it. A run cannot be certified while it is set.
  val S_AXI_RST_FAULT = 25
  val STICKY_MASK: Long = Seq(S_RD_DONE, S_WRITE_DONE, S_BRESP_ERR, S_RRESP_ERR, S_ERR_BADSIZE, S_ERR_BADBASE,
    S_WRAPPED, S_CROSS_DROPPED, S_ERR_INJ_BUSY, S_ERR_BASE_BUSY, S_ERR_FLUSH_REFUSED,
    S_ERR_FLUSH_TIMEOUT, S_ERR_START_DROPPED, S_ERR_FLUSH_DROPPED, S_EARLY_LATE, S_SKID_OVF,
    S_ERR_INJ_RANGE).map(1L << _).sum
  // All of the above are W1C stickies, also cleared by an accepted `base_reset`. S_EARLY_LATE and
  // S_SKID_OVF are captured on the RISING EDGE of their (2-FF synced) DSP-domain source level so the
  // clear cannot be immediately re-armed by the still-high level -- see ReadoutDdrUplink.
}

/**
 * Readout → DDR uplink (qubic3 PLAN_READOUT_DDR v2..v7, Codex-approved r07).
 *
 * dsp side: per-core result FIFOs (+ overflow/accepted/rejected accounting, injector arbiter) →
 * [[RollPollReader]] → skid FIFO (throttle) → [[CircularBuffer]] (write side).
 * ddr side: [[CircularBuffer]] read side → [[CbufAxiWriter]] → `ddr` AXI master (AW/W/B);
 * [[DrainEngine]] drains DDR (AR/R) → `rd` AXI-Stream; control/status registers on the `ctrl` AXI4
 * slave; run protocol (base_reset → start handshake → dsp_admit; flush → quiet → snapshot → write_done).
 * All pure SpinalHDL (P3a); the external contract is `CONTRACT.md` in this directory.
 *
 * Must be instantiated with `ddrCd` as the current clock domain; `dspCd` is the converter/result domain.
 * Both domains use the SYMMETRIC uplink reset (own reset | synced peer reset), plan v6 §2.
 */
case class ReadoutDdrUplink(p: ReadoutDdrUplinkParams, dspCd: ClockDomain) extends Component {
  import ReadoutDdrRegs._
  val ddrCd = ClockDomain.current

  val io = new Bundle {
    val results = Vec(slave(Flow(ReadoutResult(p.accWidth))), p.numCh)   // dspCd
    val ctrl    = slave(Axi4(p.ctrlAxiConfig))                            // ddrCd
    val ddr     = master(Axi4(p.ddrAxiConfig))                            // ddrCd
    val rd      = master(Stream(Fragment(Bits(p.axiDataWidth bits))))     // ddrCd, AXIS + tlast
    val dspAdmit = out Bool()                                             // dspCd (observability)
    // MIG `c0_init_calib_complete`, ddrCd. Observability only -- nothing in the datapath gates on it;
    // the authoritative, reset-independent copy is the SoC's host-domain DDR status register
    // (PulseTableSoc.ddrStatusHost). Surfaced here as DIAG[9] for convenience. (Codex r19-B1)
    val calibDone = in Bool() default(True)
  }

  // ───────────────────────────── symmetric uplink resets ─────────────────────────────
  // A raw DDR reset (psr_ddr peripheral_reset) also resets the AXI fabric behind `ddr` (the MIG's AXI
  // port, the DMA: vivado-scripts/riscvsoc-bd/inc/ddr-connect.tcl), so it may cut the DDR half at once. A raw DSP
  // reset does not reset that fabric, so its effect on the DDR half is HELD (r1) until the AXI master is quiescent:
  // no bank burst started, and no AR pending or R beat due. While held (`quiesce`) nothing new starts and the R
  // beats of an issued burst are drained and discarded. If the master is still busy after 2^rstHoldLog2 cycles the
  // reset is forced anyway and the sticky `axi_rst_fault` (STATUS bit 25) is raised. When the DDR half resets, the
  // DSP half is reset again with it, so both restart from the same state.
  val dspRstRaw = dspCd.isResetActive
  val ddrRstRaw = ddrCd.isResetActive
  val dspRstInDdr = ddrCd(BufferCC(dspRstRaw, init = True, bufferDepth = 2))
  val axiBusy = Bool()                                   // driven by the ddr side below (same clock)
  val rstHold = new ClockingArea(ddrCd) {
    val pending = Reg(Bool()) init False                 // dsp reset requested, waiting for AXI quiescence
    val applied = Reg(Bool()) init True                  // the DDR half is in reset because of the dsp side
    val cnt     = Reg(UInt(p.rstHoldLog2 + 1 bits)) init 0
    val minCnt  = Reg(UInt(3 bits)) init 0               // keep it applied >= 8 cycles (the dsp side re-syncs it)
    val fault   = Reg(Bool()) init False                 // cleared only by the raw DDR (= fabric) reset
    when(applied) {
      when(minCnt =/= 7)(minCnt := minCnt + 1)
      when(!dspRstInDdr && minCnt === 7)(applied := False)
    } elsewhen (pending) {
      cnt := cnt + 1
      when(!axiBusy) { applied := True; pending := False; minCnt := 0 }
        .elsewhen(cnt.msb) { applied := True; pending := False; minCnt := 0; fault := True }
    } elsewhen (dspRstInDdr) {
      pending := True; cnt := 0
    }
  }
  val quiesce = rstHold.pending
  val ddrURst = ddrRstRaw | rstHold.applied
  val ddrRstInDsp = dspCd(BufferCC(ddrRstRaw, init = True, bufferDepth = 2))
  val holdInDsp   = dspCd(BufferCC(rstHold.applied, init = True, bufferDepth = 2))
  // P3c: the three reset sources are OR-ed into ONE register, so the dspU reset net starts at a flop (no LUT
  // between the synchronizers and the ~2k reset pins). One dsp cycle more reset latency, far inside rstHold's
  // 8-cycle minimum.
  val dspURst = dspCd(RegNext(dspRstRaw | ddrRstInDsp | holdInDsp) init (True))
  val dspU = ClockDomain(dspCd.readClockWire, dspURst,
                         config = dspCd.config.copy(resetKind = SYNC, resetActiveLevel = HIGH))
  val ddrU = ClockDomain(ddrCd.readClockWire, ddrURst,
                         config = ddrCd.config.copy(resetKind = SYNC, resetActiveLevel = HIGH))

  // ───────────────────────────── clock crossings ─────────────────────────────
  val xStart = PulseCross(ddrU, dspU, withDstDone = true)
  val xFlush = PulseCross(ddrU, dspU, withDstDone = true)
  val xInj   = PulseCross(ddrU, dspU, withDstDone = true)
  // (P3a: no dsp→ddr flush pulse. The flush reaches the writer in band, as the cbuf's FINAL bank.)

  // injector payload: ddrU registers, held from INJ_FIRE until the acknowledge (writes are refused while
  // the injector is busy). P3b r1: the DSP side does NOT read them directly any more. On xInj's
  // synchronized request it CAPTURES them into its own registers (dsp.injRealC/ImagC/CoreC), so these
  // registers' only DSP-domain loads are those capture flops, enabled by the synchronized fire; the payload
  // is then consumed from the captures, and the acknowledge (xInj dstDone) goes back only after the
  // consumption (the FIFO push, or the central reject). ddr-timing.xdc bounds payload + request with one
  // bus-skew group ending at the capture flops and the request synchronizer.
  val injReal = ddrU(Reg(SInt(p.accWidth bits)) init 0)
  val injImag = ddrU(Reg(SInt(p.accWidth bits)) init 0)
  val injCore = ddrU(Reg(UInt(8 bits)) init 0)
  Seq(injReal, injImag, injCore).foreach(_.addTag(crossClockDomain))

  // ───────────────────────────── dsp side ─────────────────────────────
  val dsp = new ClockingArea(dspU) {
    val admit   = Reg(Bool()) init False
    val ovf     = Vec(Reg(Bool()) init False, p.numCh)
    val acc     = Vec(Reg(UInt(32 bits)) init 0, p.numCh)
    val rej     = Vec(Reg(UInt(p.rejectedWidth bits)) init 0, p.numCh)
    val accSnap = Vec(Reg(UInt(32 bits)) init 0, p.numCh)
    val ovfSnap = Reg(Bits(p.numCh bits)) init 0
    accSnap.foreach(_.addTag(crossClockDomain)); ovfSnap.addTag(crossClockDomain)   // frozen + toggle handshake
    val earlyLate = Reg(Bool()) init False
    val snapToggle = Reg(Bool()) init False
    val skidOvf = Reg(Bool()) init False

    val rejReal = Vec(Bool(), p.numCh)     // a real result arrived while admission was closed (this cycle)
    val rejInj  = Vec(Bool(), p.numCh)     // an injection targeted at this core was rejected
    val injPending = Reg(Bool()) init False
    // the DSP-domain captures of the held payload, loaded on the synchronized request only
    val injRealC   = Reg(SInt(p.accWidth bits)) init 0
    val injImagC   = Reg(SInt(p.accWidth bits)) init 0
    val injCoreC   = Reg(UInt(8 bits)) init 0
    when(xInj.io.fire) {
      injPending := True
      injRealC := injReal; injImagC := injImag; injCoreC := injCore
    }
    val injDone = False
    when(injDone)(injPending := False)
    xInj.io.dstDone := injDone

    // per-core: edge detect → admission → priority arbiter (real > inj) → FIFO
    val fifos = for (i <- 0 until p.numCh) yield new Area {
      val f      = io.results(i)
      val edge   = f.valid && !RegNext(f.valid, False)
      val real   = Stream(Bits(p.wordWidth bits))
      real.valid   := edge && admit
      real.payload := f.payload.real.asBits ## f.payload.imag.asBits     // {real hi, imag lo} (plan v2 §1)
      when(edge && !admit)(earlyLate := True)
      rejReal(i) := edge && !admit
      when(real.valid && !real.ready)(ovf(i) := True)                  // exact overflow detection

      val inj = Stream(Bits(p.wordWidth bits))
      inj.valid   := injPending && admit && injCoreC === i
      inj.payload := injRealC.asBits ## injImagC.asBits
      when(inj.fire)(injDone := True)

      val arb  = StreamArbiterFactory().lowerFirst.noLock.onArgs(real, inj)
      // (P3c: not forFMax. Its empty/full trackers encode "empty" as a SET msb, so a FIFO that powers up at 0
      // without a reset -- ReadoutDdrUplinkCdcSim start_dsp_dead_no_reset -- reads as non-empty and pops junk;
      // the pointer form reads empty from all-zero.)
      val fifo = StreamFifo(Bits(p.wordWidth bits), p.fifoDepth)
      fifo.io.push << arb
      when(fifo.io.push.fire)(acc(i) := acc(i) + 1)
      // tagged 64-bit DDR word: [63:56]=tag [55:28]=real[31:4] [27:0]=imag[31:4]
      val word = B(i, 8 bits) ## fifo.io.pop.payload(63 downto 36) ## fifo.io.pop.payload(31 downto 4)
    }
    // r08-#4/#5: reject an injection centrally — an out-of-range `inj_core` matches NO per-core arbiter,
    // so without this `injPending` (and therefore `inj_busy` and the flush quiet predicate) hangs forever.
    val injCoreOk = injCoreC < p.numCh
    val injReject = injPending && (!admit || !injCoreOk)
    when(injReject) { earlyLate := True; injDone := True }
    for (i <- 0 until p.numCh) rejInj(i) := injReject && injCoreOk && injCoreC === i

    // r09-#2 / r10-#2: a late real result and a rejected injection can hit the SAME core in the SAME
    // cycle. Two separate `rej := rej + 1` statements let the later one win (undercount), but summing
    // them would step the counter by 2 — and `rej` is Gray-crossed to the DDR domain, which requires at
    // most ONE bit change per source cycle. So the extra increment is QUEUED and applied on a later
    // cycle: the counter never advances by more than 1 per cycle and nothing is lost.
    val rejPend = Vec(Reg(UInt(3 bits)) init 0, p.numCh)
    for (i <- 0 until p.numCh) {
      val inc   = rejReal(i).asUInt(2 bits) +^ rejInj(i).asUInt(2 bits)        // 0..2 this cycle
      val avail = rejPend(i) +^ inc                                            // 0..9
      val saturated = rej(i) === rej(i).maxValue
      val fire  = (avail =/= 0) && !saturated
      when(fire)(rej(i) := rej(i) + 1)
      val rest  = avail - fire.asUInt.resized
      // r11-#1: once the counter saturates the queue is meaningless -- drop it rather than hold credits
      // that can never be applied (and would otherwise survive into the next run).
      rejPend(i) := saturated ? U(0, 3 bits) | ((rest > 7) ? U(7, 3 bits) | rest.resized)
    }

    // the registered overflow summary for the DDR side's STATUS (one flop into the synchronizer). It
    // follows `ovf` one cycle late, so it clears with the flags at BASE_RESET and with the dspU reset.
    val ovfAny = Reg(Bool()) init False
    ovfAny := ovf.asBits.orR

    // P3c: `quiet` uses `admit && anyEdge` in place of the OR of the 14 push fires. Equivalent: a result that
    // is not pushed only because its FIFO is full leaves that FIFO non-empty (anyFifoData), and an injection
    // push needs injPending, which `quiet` excludes anyway. It takes the FIFO-full term out of the cone.
    val anyEdge     = fifos.map(_.edge).orR
    val anyFifoData = fifos.map(_.fifo.io.pop.valid).orR

    // round-robin poller (data_buffer contract: valid holds until rd_en, pops on rd_en)
    val poller = RollPollReader(p.numCh, p.wordWidth)
    val throttle  = Bool()
    for (i <- 0 until p.numCh) {
      poller.io.dataValid(i) := fifos(i).fifo.io.pop.valid && !throttle
      poller.io.dataIn(i)    := fifos(i).word
      fifos(i).fifo.io.pop.ready := poller.io.rdEn(i)
    }

    // skid FIFO → cbuf write side (only when wr_ready)
    val skid = StreamFifo(Bits(p.wordWidth bits), p.skidDepth)
    skid.io.push.valid   := poller.io.wrEn
    skid.io.push.payload := poller.io.wrData
    when(skid.io.push.valid && !skid.io.push.ready)(skidOvf := True)   // must never happen (throttle)
    throttle := skid.io.occupancy >= p.throttleLevel

    val cbuf = CircularBuffer(p.wordWidth, p.axiDataWidth, p.cbufAddrWidth, wrCd = dspU, rdCd = ddrU)
    // the Fork-A backpressure flag, kept under its G2 name (ReadoutDdrUplinkDut makes it simPublic)
    val cbufWrReady = CombInit(cbuf.io.wrReady)
    cbuf.io.wrEn    := skid.io.pop.valid && cbuf.io.wrReady
    cbuf.io.wrData  := skid.io.pop.payload
    skid.io.pop.ready := cbuf.io.wrReady
    val writeFinishedExt = False
    cbuf.io.writeFinishedExt := writeFinishedExt
    // run start: clear accounting, open admission, then ack
    // P3c: the clear and the admission open one cycle after the crossing's fire (a registered enable for the
    // ~1k cleared flops); the acknowledge follows one cycle later, as before.
    val startFire = RegNext(xStart.io.fire, False)
    when(startFire) {
      // r11-#1: rejPend must be cleared too. At saturation `fire` stays low and credits accumulate, so a
      // BASE_RESET that cleared only `rej` would leak the previous run's pending increments into the next.
      for (i <- 0 until p.numCh) { ovf(i) := False; acc(i) := 0; rej(i) := 0; accSnap(i) := 0; rejPend(i) := 0 }
      ovfSnap := 0; earlyLate := False; skidOvf := False
      admit := True
    }
    val startDone = RegNext(startFire, False)
    xStart.io.dstDone := startDone

    // flush: wait quiet × flushQuiet, then snapshot + commit (closes admission)
    val quiet    = !(admit && anyEdge) && !anyFifoData && (skid.io.occupancy === 0) && !poller.io.wrEn && !injPending
    val flushing = Reg(Bool()) init False
    val quietCnt = Reg(UInt(log2Up(p.flushQuiet + 1) bits)) init 0
    val flushDone = False
    when(xFlush.io.fire) { flushing := True; quietCnt := 0 }
    // P3c: admission closes at t, the cycle the quiet run completes; the snapshot, the FINAL bank and the
    // acknowledge follow at t+1 from a registered enable (`snapNow`), so accSnap/ovfSnap load from a flop and
    // not through the quiet cone. Nothing changes in between: at t there was no push and no FIFO, skid or
    // poller data, and from t+1 admission is closed, so acc, ovf and the cbuf contents at t+1 equal those at t.
    val commitNow = flushing && quiet && quietCnt === p.flushQuiet - 1
    val snapNow   = RegNext(commitNow, False)
    when(flushing) {
      when(quiet)(quietCnt := quietCnt + 1) otherwise (quietCnt := 0)
      when(commitNow) {
        admit := False
        flushing := False
      }
    }
    when(snapNow) {
      for (i <- 0 until p.numCh) accSnap(i) := acc(i)
      ovfSnap := ovf.asBits
      snapToggle := !snapToggle
      writeFinishedExt := True     // closes the current cbuf bank as the run's FINAL bank
      flushDone := True
    }
    xFlush.io.dstDone := flushDone

    // per-core Gray-coded live rejected counters (v7 §2): +1 steps ⇒ adjacent codes
    val rejGray = Vec(rej.map(r => RegNext((r ^ (r >> 1).resize(p.rejectedWidth)).asBits) init 0))
    io.dspAdmit := admit
  }

  // ───────────────────────────── ddr side ─────────────────────────────
  val ddr = new ClockingArea(ddrU) {
    // ---- writer + cbuf read side ----
    val writer = CbufAxiWriter(p.axiDataWidth, p.cbufAddrWidth, p.axiAddrWidth)
    writer.io.quiesce := quiesce
    val cb = dsp.cbuf.io
    writer.io.ableToRead  := cb.ableToRead
    writer.io.rdEmpty     := cb.rdEmpty
    writer.io.rdFinal     := cb.rdFinal
    writer.io.rdAddrValid := cb.rdAddrValid
    writer.io.rdData      := cb.rdData
    cb.rdAddr       := writer.io.rdAddr
    cb.readFinished := writer.io.readFinished
    cb.rdFreeze     := quiesce          // r2: no presentation taken during the reset hold

    // ---- drain engine ----
    val mmu = DrainEngine(p.axiAddrWidth, p.axiDataWidth, p.axiIdWidth, maxBytes = MAX_RD_SIZE)
    mmu.io.quiesce := quiesce
    // AXI master quiescence for the reset hold: a started bank burst (AW, W or B due) or an AR/R in flight
    axiBusy := !writer.io.writerIdle || mmu.io.axiBusy

    // ---- AXI master: writer owns AW/W/B, the drain engine owns AR/R ----
    val a = io.ddr
    a.aw.valid := writer.io.aw.valid
    a.aw.addr  := writer.io.aw.addr
    a.aw.len   := writer.io.aw.len
    a.aw.size  := writer.io.aw.size
    a.aw.burst := writer.io.aw.burst
    a.aw.id    := 0
    a.aw.region := 0; a.aw.lock := 0; a.aw.cache := 0; a.aw.qos := 0; a.aw.prot := 0
    writer.io.aw.ready := a.aw.ready
    a.w.valid  := writer.io.w.valid
    a.w.data   := writer.io.w.data
    a.w.strb   := writer.io.w.strb
    a.w.last   := writer.io.w.last
    writer.io.w.ready := a.w.ready
    writer.io.b.valid   := a.b.valid
    writer.io.b.payload := a.b.resp
    a.b.ready  := writer.io.b.ready
    a.ar.valid := mmu.io.ar.valid
    a.ar.addr  := mmu.io.ar.addr
    a.ar.len   := mmu.io.ar.len
    a.ar.size  := mmu.io.ar.size
    a.ar.burst := mmu.io.ar.burst
    a.ar.id    := mmu.io.ar.id
    a.ar.region := 0; a.ar.lock := 0; a.ar.cache := 0; a.ar.qos := 0; a.ar.prot := 0
    mmu.io.ar.ready := a.ar.ready
    mmu.io.r.valid        := a.r.valid
    mmu.io.r.payload.data := a.r.data
    mmu.io.r.payload.resp := a.r.resp
    mmu.io.r.payload.last := a.r.last
    mmu.io.r.payload.id   := a.r.id
    a.r.ready  := mmu.io.r.ready
    // AXIS out
    io.rd << mmu.io.axis

    // ---- registers / run protocol ----
    val wrBase   = Reg(UInt(p.axiAddrWidth bits)) init 0
    val runBase  = Reg(UInt(p.axiAddrWidth bits)) init 0
    val rdBase   = Reg(UInt(p.axiAddrWidth bits)) init 0
    val rdSize   = Reg(UInt(32 bits)) init 0                 // bytes (<= 32 MiB enforced)
    val sticky   = Reg(Bits(32 bits)) init 0
    val runActive  = Reg(Bool()) init False
    val flushBusy  = Reg(Bool()) init False
    val startPend  = Reg(Bool()) init False        // base_reset accepted at t → writer pulse at t+1
    // r08-#1: `run_base` is latched at t (the accepted BASE_RESET write); at t+1 BOTH the writer's
    // base_reset pulse and the start crossing are issued — no extra register between them.
    val writerBaseReset = startPend
    val rdBusy   = mmu.io.busy
    // r10-#3: the engine's `busy` falls when the last AXI-R beat enters its FIFO, while `size_bytes` is still
    // live in the AXIS valid/TLAST logic. The drain is only really over at TLAST, so the window during
    // which rd_base/rd_size are frozen extends to it.
    val drainInFlight = Reg(Bool()) init False
    when(io.rd.valid && io.rd.ready && io.rd.last)(drainInFlight := False)
    // P3a F2: the engine itself also refuses a start until TLAST (`idle`); both locks agree.
    val rdLocked = rdBusy || drainInFlight || !mmu.io.idle
    val dspAdmitSync = BufferCC(dsp.admit, init = False, bufferDepth = 2)
    // P3b r1: the OR of the per-core flags is registered in dspU (dsp.ovfAny), so the synchronizer sees
    // one flop, not combinational logic (report_cdc CDC-10)
    val ovfAnySync   = BufferCC(dsp.ovfAny, init = False, bufferDepth = 2)
    val earlyLateSync = BufferCC(dsp.earlyLate, init = False, bufferDepth = 2)
    val skidOvfSync  = BufferCC(dsp.skidOvf, init = False, bufferDepth = 2)
    // P3a F4: the cbuf presents a bank only when it is full or FINAL, and `rdEmpty` reads 1 whenever
    // the writer owns no bank. `rdEmpty` alone is therefore the "no pending bank" predicate (the vendored
    // buffer could leave a consumed bank looking non-empty, which forced `!(able && !rdEmpty)`). Both
    // signals are already in this domain.
    val cbufNoPendingBank = cb.rdEmpty
    // r11-#2: `drainInFlight` (up to AXIS TLAST) must gate a new run too -- otherwise a BASE_RESET could
    // re-base the writer while the previous drain is still streaming out.
    val runIdle  = !runActive && !flushBusy && !rdLocked && writer.io.writerIdle && cbufNoPendingBank &&
                   !startPend && !xStart.io.busy && !xFlush.io.busy && !xInj.io.busy && !quiesce

    // snapshot capture (toggle handshake; bundle is static after the dsp commit)
    val snapTogSync = BufferCC(dsp.snapToggle, init = False, bufferDepth = 2)
    val snapSeen    = RegNext(snapTogSync, False)
    val accSnapDdr  = Vec(Reg(UInt(32 bits)) init 0, p.numCh)
    val ovfSnapDdr  = Reg(Bits(p.numCh bits)) init 0
    val snapArrived = Reg(Bool()) init False
    when(snapTogSync =/= snapSeen) {
      // the dsp bundle was frozen ≥ 2 dsp cycles before the toggle became visible here
      for (i <- 0 until p.numCh) accSnapDdr(i) := dsp.accSnap(i)
      ovfSnapDdr  := dsp.ovfSnap
      snapArrived := True
    }
    // rejected: Gray → bin per core
    val rejDdr = Vec(dsp.rejGray.map { g =>
      val gs = BufferCC(g, init = B(0, p.rejectedWidth bits), bufferDepth = 2)
      val b  = Bits(p.rejectedWidth bits)
      for (k <- 0 until p.rejectedWidth) b(k) := gs(p.rejectedWidth - 1 downto k).xorR
      b.asUInt
    })

    // writer controls
    writer.io.baseAddr  := runBase
    writer.io.baseReset := writerBaseReset
    val wrapped = writer.io.addrFault
    val writeDonePulse = writer.io.currentUserDone

    // drain engine controls
    val rdStartReq = False
    val rdSizeOk = (rdSize >= 32) && (rdSize(4 downto 0) === 0) && (rdSize <= MAX_RD_SIZE) && (rdBase(4 downto 0) === 0)
    mmu.io.start     := rdStartReq
    mmu.io.baseAddr  := rdBase
    mmu.io.sizeBytes := rdSize.resize(p.axiAddrWidth + 1)

    // flush watchdog
    val wdog = Reg(UInt(p.flushTimeoutLog2 + 1 bits)) init 0

    // r09-#4: sticky updates are a SINGLE masked assignment. Previously `set()` wrote individual bits
    // and the W1C / base_reset paths later assigned the whole register, so a BRESP/RRESP/wrap/drop/
    // timeout raised in the same cycle as a STATUS write was silently erased. Set dominates clear.
    val stickySet = Bits(32 bits); stickySet := 0
    val stickyClr = Bits(32 bits); stickyClr := 0
    // set DOMINATES clear: `(sticky | set) & ~clr` would drop an error raised in the same cycle as its
    // W1C write, which is exactly the race this restructuring was meant to close (r10-#1).
    sticky := (sticky & ~stickyClr) | stickySet
    def set(bit: Int): Unit = stickySet(bit) := True
    when(a.b.fire && a.b.resp =/= 0)(set(S_BRESP_ERR))
    when(a.r.fire && a.r.resp =/= 0)(set(S_RRESP_ERR))
    when(mmu.io.done)(set(S_RD_DONE))
    when(rise(wrapped))(set(S_WRAPPED))
    // r08-#8/#9: capture DSP-sourced levels on their RISING EDGE. A level-triggered `set` re-armed the
    // sticky in the cycles between a base_reset/W1C clear and the DSP-side clear (G2 `admission`), while a
    // live mirror broke the W1C contract. Rising-edge capture satisfies both.
    def rise(x: Bool): Bool = x && !RegNext(x, False)
    when(rise(earlyLateSync))(set(S_EARLY_LATE))
    when(rise(skidOvfSync))(set(S_SKID_OVF))
    // F2/F3 defence in depth: the register guard below already refuses these starts, so this only fires
    // if the two ever disagree -- reported on the same bit as the guard.
    when(mmu.io.startRejected)(set(S_ERR_BADSIZE))
    // xStart/xFlush/xInj have their SOURCE in this (ddr) domain, so `ackDropped` is readable here.
    // (P3a: the dsp->ddr xWfin crossing is gone -- the flush travels in band as the cbuf's FINAL bank.)
    when(xStart.io.ackDropped || xFlush.io.ackDropped || xInj.io.ackDropped)(set(S_CROSS_DROPPED))

    // flush bookkeeping (declared before the handshakes that reference them — Scala evaluates in order)
    val flushCommitted = Reg(Bool()) init False
    val writeDoneSeen  = Reg(Bool()) init False

    // start handshake
    xStart.io.start := False
    when(startPend) { startPend := False; xStart.io.start := True }
    when(xStart.io.acked) {
      when(xStart.io.ackDropped)(set(S_ERR_START_DROPPED)) otherwise (runActive := True)
    }
    // flush handshake
    xFlush.io.start := False
    when(xFlush.io.acked) {
      when(xFlush.io.ackDropped) { set(S_ERR_FLUSH_DROPPED); flushBusy := False; runActive := False; flushCommitted := False }
    }
    // r08-#2: the watchdog only runs once the flush was ACKNOWLEDGED by the DSP side (admission closed
    // + snapshot frozen). Counting from the register write could time out during a long legitimate DSP
    // drain and leave the DSP half running while DDR declared the run over.
    // r08-#6: `write_done` additionally requires the snapshot to have ARRIVED, so `accepted`/`overflow`
    // can never be read stale.
    when(xFlush.io.acked && !xFlush.io.ackDropped) { flushCommitted := True; wdog := 0 }
    when(writeDonePulse)(writeDoneSeen := True)
    when(flushBusy && flushCommitted) {
      wdog := wdog + 1
      when(writeDoneSeen && snapArrived) {
        set(S_WRITE_DONE); flushBusy := False; runActive := False; flushCommitted := False
      }
      when(wdog(p.flushTimeoutLog2)) {
        set(S_ERR_FLUSH_TIMEOUT); flushBusy := False; runActive := False; flushCommitted := False
      }
    }
    // injector handshake
    xInj.io.start := False
    val injBusy = xInj.io.busy

    // ---- AXI4 register file ----
    val bus = Axi4SlaveFactory(io.ctrl)
    bus.onWrite(RD_START) {
      when(rdSizeOk && !rdLocked) { rdStartReq := True; drainInFlight := True }
        .otherwise(set(S_ERR_BADSIZE))
    }
    val wrBaseW = bus.createAndDriveFlow(Bits(32 bits), WR_BASE)
    when(wrBaseW.valid) {
      val v = wrBaseW.payload.asUInt
      when(v(8 downto 0) === 0 && !v(31))(wrBase := v.resized) otherwise (set(S_ERR_BADBASE))
    }
    bus.read(runBase, RUN_BASE)
    bus.read(wrBase, WR_BASE)
    // r09-#5: writes are refused from RD_START until TLAST (contract I7). The vendored mmu2 consumed
    // `base_addr`/`size_bytes` live; the DrainEngine latches them at start, and the lock is kept as the
    // advertised register behaviour.
    val rdBaseW = bus.createAndDriveFlow(Bits(32 bits), RD_BASE)
    val rdSizeW = bus.createAndDriveFlow(Bits(32 bits), RD_SIZE)
    when(rdBaseW.valid) { when(!rdLocked)(rdBase := rdBaseW.payload.asUInt.resized) otherwise (set(S_ERR_BADSIZE)) }
    when(rdSizeW.valid) { when(!rdLocked)(rdSize := rdSizeW.payload.asUInt) otherwise (set(S_ERR_BADSIZE)) }
    bus.read(rdBase, RD_BASE)
    bus.read(rdSize, RD_SIZE)
    bus.read(writer.io.finalAddr, FINAL_ADDR)
    bus.read(writer.io.curAxiAddr, CUR_ADDR)
    bus.onWrite(BASE_RESET) {
      when(runIdle) {
        runBase := wrBase; startPend := True; snapArrived := False; wdog := 0
        stickyClr := B(STICKY_MASK, 32 bits)
      } otherwise (set(S_ERR_BASE_BUSY))
    }
    bus.onWrite(FLUSH) {
      when(runActive && !flushBusy && !xFlush.io.busy) {
        flushBusy := True; wdog := 0; flushCommitted := False; writeDoneSeen := False; snapArrived := False
        xFlush.io.start := True
      }
        .otherwise(set(S_ERR_FLUSH_REFUSED))
    }
    val status = Bits(32 bits)
    status := sticky
    status(S_RD_BUSY) := rdBusy
    status(S_FLUSH_BUSY) := flushBusy
    status(S_INJ_BUSY) := injBusy
    status(S_OVF_ANY) := ovfAnySync
    status(S_RUN_ACTIVE) := runActive
    status(S_DSP_IN_RESET) := dspRstInDdr
    status(S_DSP_ADMIT) := dspAdmitSync
    // S_DDR_IN_RESET cannot be reported from here: a DDR-domain reset also resets this register file, so
    // the bit could only ever read 0. The host detects an interrupted run through `write_done == 0`
    // (plan v7 §3). The bit is kept reserved-zero for ABI stability.
    status(S_DDR_IN_RESET) := False
    status(S_AXI_RST_FAULT) := rstHold.fault
    bus.read(status, STATUS)
    val statusW = bus.createAndDriveFlow(Bits(32 bits), STATUS)
    when(statusW.valid)(stickyClr := statusW.payload & B(STICKY_MASK, 32 bits))
    bus.read(ovfSnapDdr.resize(32), OVERFLOW)
    // r08-#3: the payload must be STATIC for the whole crossing (that is what makes the multi-bit
    // `crossClockDomain` tag legitimate), so writes are refused while the injector handshake is in flight.
    val injRealW = bus.createAndDriveFlow(Bits(32 bits), INJ_REAL)
    val injImagW = bus.createAndDriveFlow(Bits(32 bits), INJ_IMAG)
    val injCoreW = bus.createAndDriveFlow(Bits(32 bits), INJ_CORE)
    when(injRealW.valid) { when(!injBusy)(injReal := injRealW.payload.asSInt.resized) otherwise (set(S_ERR_INJ_BUSY)) }
    when(injImagW.valid) { when(!injBusy)(injImag := injImagW.payload.asSInt.resized) otherwise (set(S_ERR_INJ_BUSY)) }
    when(injCoreW.valid) {
      when(injBusy)(set(S_ERR_INJ_BUSY))
        .elsewhen(injCoreW.payload.asUInt >= p.numCh)(set(S_ERR_INJ_RANGE))   // r08-#4 / r09-#1: FULL word
        .otherwise(injCore := injCoreW.payload(7 downto 0).asUInt)
    }
    bus.onWrite(INJ_FIRE) {
      when(!injBusy && runActive && injCore < p.numCh && !quiesce)(xInj.io.start := True) otherwise (set(S_ERR_INJ_BUSY))
    }
    bus.read(U(p.numCh, 32 bits), NUM_CH)
    // diagnostics: why is a base_reset being refused? (also useful during board bring-up)
    val diag = Bits(32 bits)
    diag := 0
    diag(0) := writer.io.writerIdle
    diag(1) := cb.rdEmpty
    diag(2) := cb.ableToRead
    diag(3) := xStart.io.busy
    diag(4) := startPend
    diag(5) := xFlush.io.busy
    diag(6) := writeDoneSeen   // r09-#3: was xWfin.busy, a DSP-domain signal (unguarded CDC)
    diag(7) := runIdle
    diag(8) := snapArrived
    diag(9) := BufferCC(io.calibDone, init = False, bufferDepth = 2)   // MIG calibration (r19-B1)
    bus.read(diag, DIAG)
    bus.read(B(0, 32 bits), STOP)      // r1: reserved STOP word, no hardware
    bus.read(U(p.flushQuiet, 8 bits) ## U(p.cbufAddrWidth, 8 bits) ## U(p.skidDepth, 8 bits) ## U(p.fifoDepth, 8 bits), GEOMETRY)
    for (i <- 0 until p.numCh) {
      bus.read(accSnapDdr(i), ACCEPTED + 4 * i)
      bus.read(rejDdr(i).resize(32), REJECTED + 4 * i)
    }
  }
}
