package riscq.soc

import spinal.core._
import spinal.core.sim._
import spinal.core.fiber.Fiber
import spinal.lib._
import spinal.lib.bus.tilelink
import spinal.lib.bus.tilelink.fabric.{Node, MasterBus, WidthAdapter}
import spinal.lib.bus.amba4.axi.{Axi4, Axi4ToTilelinkFiber}
import riscq.misc.{Axi4VivadoHelper, Axi4StreamVivadoHelper, VivadoClkHelper}
import spinal.lib.bus.misc.SizeMapping
import riscq.dsp.{AdderTree, ComplexBatch}
import riscq.riscv.RiscqParam
import riscq.soc.fabric.{BramFiber, MemMapDriverFiber}
import scala.collection.mutable.LinkedHashMap
import scala.collection.mutable

/**
 * Multi-qubit control SoC — the agentic reproduction of the RISC-Q reference `PulseTableSoc`. One
 * shared 32-bit batch-time counter drives `qubitNum` identical [[RiscqRfWithPulseTableFiber]] qubit
 * cores; the host AXI bus loads programs / pulse tables / control while the faster dsp domain runs the
 * real-time datapath.
 *
 * The toplevel: bridges `io.axi` → Tilelink and fans it to per-core instruction memory, per-core
 * pulse memory, the readout buffers (`robs`) and a host control block (`riscqReset` /
 * 64-bit `timeOffset`); builds the shared `time` from a free-running `refTime + timeOffset`; maps
 * logical DAC/ADC **channels** to physical converters (`dacMap`/`adcMap`), summing channels that share
 * a DAC with [[AdderTree]]; and streams a readout trace into `robs` on pulse fire.
 *
 * `withTest` exposes each core's CPU data-bus decode (`dMemPortDec`) to a second Tilelink master so a
 * sim can configure the RF (schedule pulses) without a CPU program — in the real SoC the CPU is the
 * sole `dBus` master.
 *
 * @param dacMap (core, channel) → physical DAC id.   @param adcMap core → physical ADC id.
 */
case class PulseTableSoc(
    qubitNum: Int,
    dacMap: Map[(Int, Int), Int],
    adcMap: Map[Int, Int],
    dacNum: Int = 16,
    adcNum: Int = 16,
    withTest: Boolean = false,
    vivado: Boolean = false,
    ddrReadout: Boolean = false,
    // RISC-V core plugin config, replicated across all `qubitNum` cores. Defaults to the verified
    // timing-closure stack for the packed multi-core floorplan, every flag RVLS-bit-exact:
    //   - `gshareMem` moves the GShare 2-bit counter table from a flip-flop array + one-hot write decode
    //     into a synchronous-read LUTRAM (cuts per-core control sets and reset FFs);
    //   - `csrWarl` applies the CSR WARL latitude (trims the reset group);
    //   - `aluNoFastForward` drops the srcA fast-forward → interlocks every 1-ahead RAW (small IPC cost),
    //     taking the ALU-result-mux→RD_DATA→forward loop off the path;
    //   - `aluResultOneHot` builds the ALU result mux as a balanced one-hot masked-OR cone (zero IPC);
    //   - `pcRegMaxFanout = 16` replicates the route-dominated fetch predicted-PC register.
    // Pass `RiscqParam()` to A/B the pre-opt core.
    coreParam: RiscqParam = RiscqParam(gshareMem = true, csrWarl = true,
      aluNoFastForward = true, aluResultOneHot = true, pcRegMaxFanout = 16),
    // routability levers: readout-drive envelope interpolation (readout = 16) shrinks the widest BRAM
    // bank 16×. fmax is soft, so the extra latency is acceptable.
    readoutInterp: Int = 16,
    gateInterp: Int = 4,
    // demod-carrier envelope interpolation (default 4 = fully interpolated at adcBatch 4 ⇒ 32-bit line,
    // direct host wire). Lower for finer per-ADC-lane matched-filter weights; the envelope is typically
    // square, so the coarse default suffices and keeps the demod bank cheap.
    demodInterp: Int = 4,
    // narrow posted-link RF architecture: each core funnels its RF writes onto a per-core posted link
    // (bridge + `linkPipe` RegNext stages + demux to converter-edge channels), so cores can be
    // floorplanned far from the converters. `linkPipe` is the per-direction depth.
    linkPipe: Int = 4,
    memDepth: Int = 4096,
    envDepth: Int = 1024,
    robDepth: Int = 1024,
    gatePulseNum: Int = 8,
    // per-parameter TimedQueue depth in every drive/demod PulseGenerator — how many pulses software
    // can schedule ahead per channel parameter before the queue back-pressures. Deeper = more
    // scheduled-ahead headroom at a per-queue FF/LUT cost (the six queues per generator × every channel).
    queueDepth: Int = 4,
    // specs/dsp-fmax.md converter-edge lever, default off / bit-exact (the B1-alt gate-table distributed
    // RAM and the B2 dcOffset MAX_FANOUT cap are baked into PulseParamBuffer; the B3 queue lean-pop into
    // TimedQueue; the C1 registered head is a TimedQueue-level option, no longer plumbed here).
    adcPipe: Int = 3,                   // C2: register stages on the ADC nets off the RFDC edge
    // qubic3 readout->DDR uplink (JSON key `ddr_readout`). OFF by default, and when off NOTHING is
    // elaborated: the generated RTL is byte-identical to the baseline (gated by evidence/G0's hash).
) extends Zcu216Top(dacNum = dacNum, adcNum = adcNum, dacBatch = 16, adcBatch = 4, dataWidth = 16, vivado = vivado) {
  val N        = 16    // DAC drive batch
  val adcBatch = 4
  val w        = 16
  val envWidth = N * 2 * w           // 512-bit complex pulse-envelope line
  val demodEnvFull = adcBatch * 2 * w             // full demod-carrier envelope line (128 at adcBatch 4)
  val readoutEnvWidth = envWidth / readoutInterp  // interpolated readout-drive envelope line (32 at 16)
  val gateEnvWidth    = envWidth / gateInterp     // interpolated gate-drive envelope line (128 at 4)
  val demodEnvWidth   = demodEnvFull / demodInterp // interpolated demod-carrier envelope line (32 at 4)
  val robWidth = adcBatch * 32       // 4 readout lanes × 32-bit (no overflow summing ≤16 ADCs)

  // ── host AXI → Tilelink ── blockSize ≥ the widest full-word transfer (the robs WidthAdapter's
  // 128-bit / 16-byte line); each fiber's decoder restricts the size down to what it supports. The
  // write-only envelope banks load 32-bit sub-word (no WidthAdapter), so they never need a wide burst.
  val bridge = new Axi4ToTilelinkFiber(blockSize = 64, slotsCount = 4)
  bridge.up load io.axi
  val hostBus = Node()
  hostBus at 0 of bridge.down
  hostBus.setDownConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)

  val map = SocMemoryMap(
    qubitNum        = qubitNum,
    coreMemBytes    = 1 << 16,
    pulseMemBytes   = 1 << log2Up(gateEnvWidth * envDepth / 8),      // gate-drive bank (interpolated)
    readoutEnvBytes = 1 << log2Up(readoutEnvWidth * envDepth / 8),   // readout-drive bank (interpolated)
    demodEnvBytes   = 1 << log2Up(demodEnvWidth * envDepth / 8),     // demod-carrier bank (interpolated)
    readoutBufBytes = 2 * (1 << log2Up(robWidth * robDepth / 8))     // 2 robs
  )

  // The three envelope banks are write-only (BramWriteFiber): each core's fiber steers a 32-bit host
  // beat straight into the addressed sub-word lane of its wide line, so NO WidthAdapter is needed. The
  // per-bank regions are decoded off hostBus as NARROW 32-bit fan-out buses, so only 32-bit nets cross
  // the die and every wide envelope net stays local to its core (saving routing). The 32-bit
  // instruction memory (riscqMemBus) likewise wires direct.
  val pulseMemBus   = Node()
  val readoutEnvBus = Node()
  val demodEnvBus   = Node()
  val riscqMemBus   = Node()
  pulseMemBus   at SizeMapping(map.pulseMemBase,   map.regionSize) of hostBus
  pulseMemBus.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)
  readoutEnvBus at SizeMapping(map.readoutEnvBase, map.regionSize) of hostBus
  readoutEnvBus.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)
  demodEnvBus   at SizeMapping(map.demodEnvBase,   map.regionSize) of hostBus
  demodEnvBus.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)
  riscqMemBus   at SizeMapping(map.coreMemBase,    map.regionSize) of hostBus
  riscqMemBus.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)

  val robs = BramFiber(1, robWidth, robDepth, hostCd, dspCd, withOutReg = true)
  val robAdapter = WidthAdapter()
  robAdapter.up at SizeMapping(map.readoutBufBase, map.regionSize) of hostBus
  robs.up at SizeMapping(0, map.regionSize) of robAdapter.down
  robs.up.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)

  // ── reset/control crossing into the dsp/riscq domain ──
  val riscqReset = Bool()
  val riscqCd    = ClockDomain(dspCd.readClockWire, riscqReset)

  /** Converter-boundary pipeline: `converterPipe` extra register stages on the long DAC/ADC nets
   *  into/out of the RFDC edge. converterPipe = 0 ⇒ identity (no behavioural change). */
  def pipe[T <: Data](x: T, converterPipe: Int): T = (0 until converterPipe).foldLeft(x)((s, _) => RegNext(s))

  val riscqArea = new ClockingArea(dspCd) {
    // Free-running batch-time counter in dspCd: it is NOT reset by riscqReset, so batch time is monotonic
    // across runs (only the external dspRst zeroes it). This keeps a leftover TimedQueue entry from a prior
    // run in the *past* rather than the future, so it drains on its own and a run needs no per-run flush.
    // Software works in now()-relative time, so the non-zero base is transparent.
    val refTime    = Reg(UInt(64 bit)) init 0
    refTime := refTime + 1
    val timeOffset = Reg(UInt(64 bit)) init 0
    val syncTime   = RegNext(refTime + timeOffset)
    val time       = RegNext(syncTime(0, 32 bits))
    time.addAttribute("MAX_FANOUT", 16)

    // per-core batch-clock replica: each core gets an independent register fed from the same `syncTime`,
    // so it is value-identical to `time` every cycle (no skew); EQUIVALENT_REGISTER_REMOVAL=NO stops
    // Vivado folding the identical replicas back into one shared high-fanout net. The floorplan bench
    // pins each `coreTime_i` register to its core's region so the replicated time stays local.
    val coreTimes = List.tabulate(qubitNum) { i =>
      val t = RegNext(syncTime(0, 32 bits))
      t.addAttribute("EQUIVALENT_REGISTER_REMOVAL", "NO")
      t.addAttribute("MAX_FANOUT", 16)
      t.setName(s"coreTime_$i")
      t
    }

    val cp = coreParam.copy(
      fetchPcWidth = Some(log2Up(memDepth) + 2),
      fetchLatency = 4)
    val riscqCores = List.tabulate(qubitNum)(i =>
      RiscqRfWithPulseTableFiber(
        plugins = cp.plugins(), dspCd = dspCd, hostCd = hostCd, riscqCd = riscqCd,
        time = coreTimes(i), batchSize = N, dataWidth = w, adcBatch = adcBatch,
        envDepth = envDepth, readoutInterp = readoutInterp, gateInterp = gateInterp, demodInterp = demodInterp,
        linkPipe = linkPipe, withTestTap = withTest, memDepth = memDepth, gatePulseNum = gatePulseNum,
        queueDepth = queueDepth))

    // floorplan: keep each core's RiscvSoc a hard synth boundary so opt can't merge logic across the
    // identical cores into a MUXF7/F8 macro that straddles two per-core pblocks. The shared host AXI fans
    // instruction-load to every core, so opt_design otherwise shares equivalent iLoad-response logic
    // between cores. Synthesis-only attribute — zero behavioural change, sims ignore it.
    riscqCores.foreach(_.riscvSoc.addAttribute("KEEP_HIERARCHY", "TRUE"))

    // host fan-out: per-core instruction memory + the three write-only envelope banks all wire DIRECT to
    // their narrow 32-bit region bus — each envelope fiber bridges a 32-bit host beat into its wide line
    // locally (no WidthAdapter). Offsets are relative to each region bus (rebased 0).
    for ((core, i) <- riscqCores.zipWithIndex) {
      core.iMemPortArb        at SizeMapping(i * map.coreStride,       map.coreStride)       of riscqMemBus
      core.iMemPortArb.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)
      core.pulseMemFiber.up   at SizeMapping(i * map.pulseStride,      map.pulseStride)      of pulseMemBus
      core.pulseMemFiber.up.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)
      core.readoutMemFiber.up at SizeMapping(i * map.readoutEnvStride, map.readoutEnvStride) of readoutEnvBus
      core.readoutMemFiber.up.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)
      core.demodMemFiber.up   at SizeMapping(i * map.demodEnvStride,   map.demodEnvStride)   of demodEnvBus
      core.demodMemFiber.up.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)
    }

    // optional test masters so a sim can drive each core's RF/control (CPU is held in reset). Each core
    // re-exposes its data-bus decode as a `dMemPortDec` tap node (bridged into RiscvSoc's `dTap`
    // slave-IO, withTestTap = withTest); the sim's MasterAgent drives this master.
    val testMasters = withTest generate riscqCores.map { core =>
      val mb = riscqCd(new MasterBus(tilelink.M2sParameters(
        addressWidth = 32, dataWidth = 32,
        masters = List(tilelink.M2sAgent(name = null, mapping = List(tilelink.M2sSource(
          id = SizeMapping(0, 4),
          emits = tilelink.M2sTransfers(
            get = tilelink.SizeRange.upTo(0x40), putFull = tilelink.SizeRange.upTo(0x40),
            putPartial = tilelink.SizeRange.upTo(0x40)))))))))
      core.dMemPortDec at 0 of mb.node
      mb
    }

    // ── DAC: per physical DAC, sum the real lanes of every logical channel mapped to it, then pad
    // every DRIVEN DAC to one shared pipeline depth from a core's `dac(ch)` to `io.dac`, so every pulse
    // generator sees the same register count to the converter edge — gate and readout drives (whether on
    // their own DAC or summed with others) all leave time-aligned. The natural "combine" latency differs
    // per DAC — a single channel is a pass-through (0), an n-channel sum costs `AdderTree.latency(n)+1` —
    // so pad each path up to the deepest one.
    //
    // NB: `.toList` BEFORE `.collect` — collecting `(c, ch)` tuples straight off the Map would rebuild a
    // Map[c, ch], so two channels of the same core (gate ch0 + readout ch1) would collide on key `c` and
    // the gate would be silently dropped. On a List the result is a plain List[pulse].
    val dacChannels = (0 until dacNum).map { dacId =>
      dacMap.toList.collect { case ((c, ch), id) if id == dacId => riscqCores(c).dac(ch) }
    }
    def combineLatency(n: Int) = if (n <= 1) 0 else AdderTree.latency(n) + 1
    val dacAlignStages = dacChannels.map(p => combineLatency(p.size)).max

    val dacPayloads = dacChannels.zipWithIndex.map { case (pulses, dacId) =>
      val payload = cloneOf(io.dac(dacId).payload)
      io.dac(dacId).payload := pipe(payload, 1)   // one shared stage into the RFDC edge
      if (pulses.isEmpty) {
        payload := 0                              // no generator maps here — nothing to align
      } else {
        val combined =
          if (pulses.size == 1) Vec(pulses.head.map(_.re)).asBits    // single channel: pass-through
          else {
            // No saturation on the co-mapped channel sum: it wraps modulo 2^w (keep the low w bits),
            // matching the QubiC reference (elementsum "not checking overflow, depends on user"). Software
            // keeps the summed channels within full-scale; dropping the saturating clamp also takes its
            // comparators/muxes off the DAC converter-boundary path.
            val accW = w + log2Up(pulses.size)
            Vec.tabulate(N)(k => RegNext(AdderTree(pulses.map(_(k).re), accW).resize(w))).asBits
          }
        // pad this DAC's combine path up to the deepest driven DAC so every generator sees one depth.
        payload := pipe(combined, dacAlignStages - combineLatency(pulses.size))
      }
      payload
    }

    // ── ADC: buffer the real lanes of only the MAPPED converters, fan to the mapped cores (im = 0).
    // Unmapped physical ADCs are left untouched — no buffer registers, no contribution to the trace. ──
    val mappedAdcIds = adcMap.values.toList.distinct.sorted
    val adcBufs = mappedAdcIds.map { adcId =>
      val a = Vec.fill(adcBatch)(SInt(w bits))
      a.assignFromBits(io.adc(adcId).payload)
      adcId -> pipe(a, adcPipe).addAttribute("max_fanout", 4)   // extra register stages off the RFDC edge
    }.toMap
    for ((coreId, adcId) <- adcMap) {
      (riscqCores(coreId).adc zip adcBufs(adcId)).foreach { case (o, i) => o.re := i; o.im := 0 }
    }

    // ── readout trace into robs on any drive-pulse fire ──
    val anyPulseValid = riscqCores.map(_.readoutPulse.valid).reduceBalancedTree(_ | _, (s, _) => RegNext(s))
    val fire   = RegNext(anyPulseValid)
    val rbAddr = Reg(UInt(log2Up(robDepth) bits))
    when(fire)(rbAddr := rbAddr + 1).otherwise(rbAddr := 0)

    // robs(0): per-lane sum of the MAPPED ADC inputs (a 32-bit-per-lane integrated trace).
    val adcSum = Vec.tabulate(adcBatch)(k => RegNext(AdderTree(mappedAdcIds.map(id => adcBufs(id)(k)), 32)))
    val rb0 = robs.rams(0).fastPort
    rb0.enable := True; rb0.mask.setAllTo(True)
    rb0.address := RegNext(rbAddr); rb0.write := fire
    rb0.wdata   := Vec(adcSum).asBits
  }

  // ── host control block (host clock domain) ──
  val riscqResetHostCd = Bool()
  val bufferedReset    = dspCd(BufferCC(riscqResetHostCd, 3))
  // Power up ASSERTED (init True): the cores — and any stateful data-bus adapter like the posted-store
  // shim's FIFO — must come up held in reset until the host releases them, otherwise they run with
  // uninitialised state before the first host reset pulse (a posted store would drain a garbage entry).
  val riscqResetBuf = dspCd(RegNext(bufferedReset | io.dspRst) init True)
  riscqReset := dspCd(RegNext(riscqResetBuf) init True)

  val timeOffset = Reg(UInt(64 bit)) init 0
  riscqArea.timeOffset := dspCd(BufferCC(timeOffset))

  // ── qubic3: optional readout → DDR uplink ────────────────────────────────────────────────────────
  // Everything is `generate`-gated on `ddrReadout`, so with the feature off not one signal is created
  // and the emitted RTL keeps the baseline hash (evidence/G0). Geometry is derived from `qubitNum`, so
  // the same code builds for any config.
  // The MIG's ui_clk on this board/config is 333.25 MHz (300 MHz input, CLKOUT0_DIVIDE=3) -- Vivado
  // annotates exactly that, and a mismatched FREQ_HZ fails validate_bd_design. Mirrored by DDR_FREQ in
  // vivado-scripts/riscvsoc-bd/inc/config.tcl.
  def DdrClkFreqHz: Long = 333250000L

  val ddrUplink = ddrReadout generate new Area {
    val ddrClk = in Bool()
    val ddrRst = in Bool()
    ddrClk.setName("ddrClk"); ddrRst.setName("ddrRst")
    val ddrCd  = ClockDomain(ddrClk, ddrRst)
    val p      = riscq.ddr.ReadoutDdrUplinkParams(numCh = qubitNum, accWidth = 32)   // = READOUT_ACC_WIDTH (riscq.map); the decoder integral width
    // Codex r19-B1: the MIG's `c0_init_calib_complete` is otherwise connected to NOTHING in the block
    // design, so on the board a failed DDR4 calibration is indistinguishable from a wedged uplink -- an
    // AXI transaction that never returns. Bring it in and publish it (see `ddrStatus` below).
    val calibDone = in Bool();  calibDone.setName("ddrCalibDone")
    val ctrl   = slave(Axi4(p.ctrlAxiConfig));  ctrl.setName("s_axi_ddr_ctrl")
    val ddr    = master(Axi4(p.ddrAxiConfig));  ddr.setName("m_axi_ddr")
    val rd     = master(Stream(Fragment(Bits(p.axiDataWidth bits)))); rd.setName("m_axis_rd")

    val up = ddrCd(riscq.ddr.ReadoutDdrUplink(p, dspCd))
    up.io.calibDone := calibDone
    for (i <- 0 until qubitNum) up.io.results(i) << riscqArea.riscqCores(i).readoutResult
    up.io.ctrl << ctrl
    ddr << up.io.ddr
    rd  << up.io.rd

    if (vivado) {
      VivadoClkHelper.addInference(ddrClk, ddrRst, DdrClkFreqHz)
      Axi4VivadoHelper.addInference(ddr,  "M_AXI_DDR")
      Axi4VivadoHelper.addInference(ctrl, "S_AXI_DDR_CTRL")
      // r11-#10: this is a Stream[Fragment], so the generic helper (which tags the whole `payload` as
      // TDATA) would give `last` both TDATA and TLAST. Tag the members explicitly.
      def axis(d: Data, sig: String): Unit =
        d.addAttribute("X_INTERFACE_INFO", s"xilinx.com:interface:axis:1.0 M_AXIS_RD $sig")
      axis(rd.valid, "TVALID"); axis(rd.ready, "TREADY")
      axis(rd.fragment, "TDATA"); axis(rd.last, "TLAST")
    }
  }

  // Codex r19-#4: publishing calibration ONLY inside the uplink's own DIAG helps just while the ui_clk
  // register path is clocked and out of reset -- exactly the condition that is NOT established when the
  // symptom is "an AXI transaction that never returns". So the same two facts are also mirrored here, in
  // the HOST clock domain, whose reset tree is independent of the DDR one: this register is readable
  // even when the whole ui_clk side is dead, which is the case it exists to diagnose.
  //   [31:16] magic 0xCA1B -- a positive capability marker. An older bitstream returns 0 for this
  //           unmapped offset, so a newer driver can tell "no such register" from "calibration failed"
  //           instead of refusing to run forever (Codex r19-B2).
  //   [1]     ui_reset_released -- the DDR reset tree let go (Codex r19-B3): distinguishes
  //           "MIG never calibrated" from "calibrated, but the downstream reset never released".
  //   [0]     calib_done
  val ddrStatusHost = ddrReadout generate hostCd(new Area {
    val calib = BufferCC(ddrUplink.calibDone, init = False, bufferDepth = 3)
    val rstOk = BufferCC(!ddrUplink.ddrRst,   init = False, bufferDepth = 3)
    val word  = B(0xCA1B, 16 bits) ## B(0, 14 bits) ## rstOk ## calib
  })

  val hostCtrlDriver = MemMapDriverFiber(addressWidth = 10, dataWidth = 32, driveProc = { factory =>
    factory.drive(riscqResetHostCd, 0)
    factory.write(timeOffset(0, 32 bits), 64)
    factory.write(timeOffset(32, 32 bits), 68)
    if (ddrReadout) factory.read(ddrStatusHost.word, 0x50)   // see ddrStatusHost above
  })
  hostCtrlDriver.up.setUpConnection(a = StreamPipe.FULL, d = StreamPipe.FULL)
  hostCtrlDriver.up at SizeMapping(map.hostCtrlBase, map.regionSize) of hostBus

  Fiber build new Area {
    riscqArea.riscqCores(0).riscvSoc.dMemPortDec.bus.get.simPublic()
    if (withTest) {
      riscqArea.time.simPublic()
      riscqArea.riscqCores(0).gatePulse.valid.simPublic()
      riscqArea.testMasters.foreach(_.node.bus.get.simPublic())
    }
  }
}

/**
 * DAC/ADC channel→converter maps for the ZCU216 build, generic in `qubitNum` (matches the 14-qubit
 * `GenPulseTableSoc` layout when `qubitNum == 14`): each qubit's gate-drive channel gets its own DAC
 * `0..qubitNum-1`; its readout-drive channel and its ADC share converter 14 (qubits 0–6) or 15 (7+).
 */
object SocChannelMap {
  def readoutDriverConverter(core: Int): Int = if (core < 7) 14 else 15
  def readoutConverter(core: Int): Int = if (core < 7) 0 else 4
  def gateConverter(core: Int): Int = core
  def dacMap(qubitNum: Int): Map[(Int, Int), Int] =
    (0 until qubitNum).flatMap(c => List((c, 0) -> gateConverter(c), (c, 1) -> readoutDriverConverter(c))).toMap
  def adcMap(qubitNum: Int): Map[Int, Int] =
    (0 until qubitNum).map(c => c -> readoutConverter(c)).toMap
}

/**
 * RTL generation for the Vivado ZCU216 flow: emits `PulseTableSoc.v` *with* the
 * `X_INTERFACE_INFO`/`FREQ_HZ` bus-interface attributes (host clock renamed `hostClk`/`hostRst`) plus the
 * `ClockInterface.v` clock-buffer wrapper, both into `build/rtl`. The qubit count is `args(0)` (default 14,
 * the full ZCU216 config); use a small count for quick script iteration. `romReuse` shares the per-core
 * register-file ROM init across the identical cores.
 */
object GenPulseTableSocVivado extends App {
  // args: `[N]` qubit count (default 14) and `[dir]` target directory (the first non-numeric arg;
  // default `./build/rtl`), so the `.v` + `ClockInterface.v` + register-file `.bin` land in the
  // per-project build dir the Vivado flow runs from.
  // Builds the narrow posted-link RF architecture — pair with the per-core / two-region floorplan.
  // PulseTableSoc tags each core's `RiscvSoc` `(* KEEP_HIERARCHY = "TRUE" *)` so synthesis can't
  // dissolve or cross-merge the identical cores; the per-core pblocks pin each core's `RiscvSoc`, so
  // that boundary must remain a distinct macro for the floorplan to bind.
  val qubitNum = args.filter(_.forall(_.isDigit)).headOption.map(_.toInt).getOrElse(14)
  val dir      = args.find(a => a.nonEmpty && !a.forall(_.isDigit)).getOrElse("./build/rtl")
  val cfg      = SpinalConfig(mode = Verilog, targetDirectory = dir, romReuse = true).setScopeProperty(LutInputs, 6)
  cfg.generate(PulseTableSoc(
    qubitNum = qubitNum,
    dacMap   = SocChannelMap.dacMap(qubitNum),
    adcMap   = SocChannelMap.adcMap(qubitNum),
    vivado   = true))
  cfg.generate(riscq.misc.ClockInterface())
  println(s"[GenPulseTableSocVivado] emitted $dir/PulseTableSoc.v + ClockInterface.v (qubitNum=$qubitNum, vivado=true)")
}

/**
 * RTL for the standalone OOC fmax/floorplan flow — the faithful per-core-pblock form.
 *
 *   - `vivado = false` — plain `dspClk` / `clk` ports, no `X_INTERFACE_INFO`/`FREQ_HZ` IP-packager attrs
 *     and no separate `ClockInterface.v` wrapper. The OOC bench constrains the raw ports directly, so it
 *     must NOT be the `vivado = true` (IP) form `GenPulseTableSocVivado` emits, whose host clock is
 *     renamed `hostClk`/`hostRst`.
 *   - PulseTableSoc tags `(* KEEP_HIERARCHY *)` on every `RiscvSoc`, so the separately-pblocked cores
 *     stay distinct macros (otherwise `opt_design` merges cross-core iLoad-response logic into a
 *     MUXF7/F8 that straddles two per-core pblocks and can't place).
 *
 * Every lever is the `PulseTableSoc` constructor default (replicated batch clock, the `aluNoFastForward`
 * + `aluResultOneHot` core levers, `linkPipe = 4`, congestion-lean `coreParam`), so this generator only
 * has to pick the OOC (`vivado = false`) form.
 *
 * args: `[N]` qubit count (default 14) and `[dir]` target directory (the first non-numeric arg; default
 * `./build/rtl-riscvsoc`), so the `.v` + register-file `.bin` land where the Tcl runs Vivado.
 */
object GenPulseTableSocOoc extends App {
  val qubitNum = args.filter(_.forall(_.isDigit)).headOption.map(_.toInt).getOrElse(14)
  val dir      = args.find(a => a.nonEmpty && !a.forall(_.isDigit)).getOrElse("./build/rtl-riscvsoc")
  SpinalConfig(mode = Verilog, targetDirectory = dir, romReuse = true)
    .generate(PulseTableSoc(
      qubitNum = qubitNum,
      dacMap   = SocChannelMap.dacMap(qubitNum),
      adcMap   = SocChannelMap.adcMap(qubitNum)))
  println(s"[GenPulseTableSocOoc] emitted $dir/PulseTableSoc.v (qubitNum=$qubitNum, vivado=false)")
}
