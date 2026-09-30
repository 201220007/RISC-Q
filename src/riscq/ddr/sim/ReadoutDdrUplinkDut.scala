package riscq.ddr.sim

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba4.axi.Axi4
import riscq.ddr._
import spinal.core.sim._
import riscq.soc.link.ReadoutResult

/**
 * Standalone DUT around [[ReadoutDdrUplink]] with two external clock domains (dsp = results side,
 * ddr = AXI/control side), used by the G2 SpinalSim and by the RTL-generation smoke.
 */
case class ReadoutDdrUplinkDut(p: ReadoutDdrUplinkParams) extends Component {
  val io = new Bundle {
    val dspClk = in Bool()
    val dspRst = in Bool()
    val ddrClk = in Bool()
    val ddrRst = in Bool()
    val results = Vec(slave(Flow(ReadoutResult(p.accWidth))), p.numCh)
    val ctrl    = slave(Axi4(p.ctrlAxiConfig))
    val ddr     = master(Axi4(p.ddrAxiConfig))
    val rd      = master(Stream(Fragment(Bits(p.axiDataWidth bits))))
    val dspAdmit = out Bool()
    // r2 (test only): hold the memory side's WREADY low towards the uplink, so a sim can stall W beats
    // (AxiMemorySim cannot back-pressure W). Undriven it is 0 and the W channel passes straight through.
    val wStall  = in Bool()
    // P3b (test only): the same for AW, AR and B, which in P3a never stalled (AxiMemorySim accepts every
    // address at once and answers B at once). AW/AR: VALID towards the memory and READY towards the
    // uplink are held low; B: VALID towards the uplink and READY towards the memory are held low, so the
    // response is delayed, never lost. All undriven = 0 = pass-through. Driven by `StallInjector`.
    val awStall = in Bool()
    val arStall = in Bool()
    val bStall  = in Bool()
  }
  noIoPrefix()
  val dspCd = ClockDomain(io.dspClk, io.dspRst)
  val ddrCd = ClockDomain(io.ddrClk, io.ddrRst)
  val up = ddrCd(ReadoutDdrUplink(p, dspCd))
  // r11-#6: the backpressure claim is only meaningful if the sim can SEE it happen.
  up.dsp.cbufWrReady.simPublic()
  up.dsp.skid.io.occupancy.simPublic()
  up.dsp.throttle.simPublic()
  // r11-#3: the set-vs-W1C collision property is only checkable if the sim can see BOTH strobes.
  up.ddr.stickySet.simPublic()
  up.ddr.stickyClr.simPublic()
  up.ddr.sticky.simPublic()
  // r12-#3/#4: the rejection queue and the injector handshake must be observable, otherwise the tests
  // that claim to exercise them are vacuous.
  up.dsp.rejPend.foreach(_.simPublic())
  up.dsp.injPending.simPublic()
  // r1: the dsp-reset hold must be observable to prove the DDR half resets only at AXI quiescence
  up.rstHold.pending.simPublic()
  up.rstHold.applied.simPublic()
  up.rstHold.fault.simPublic()
  up.ddrURst.simPublic()
  for (i <- 0 until p.numCh) up.io.results(i) << io.results(i)
  up.io.calibDone := True          // the MIG is calibrated in every G2 scenario
  up.io.ctrl  << io.ctrl
  io.ddr.aw.payload  := up.io.ddr.aw.payload
  io.ddr.aw.valid    := up.io.ddr.aw.valid && !io.awStall
  up.io.ddr.aw.ready := io.ddr.aw.ready && !io.awStall
  io.ddr.ar.payload  := up.io.ddr.ar.payload
  io.ddr.ar.valid    := up.io.ddr.ar.valid && !io.arStall
  up.io.ddr.ar.ready := io.ddr.ar.ready && !io.arStall
  up.io.ddr.b.payload := io.ddr.b.payload
  up.io.ddr.b.valid   := io.ddr.b.valid && !io.bStall
  io.ddr.b.ready      := up.io.ddr.b.ready && !io.bStall
  up.io.ddr.r << io.ddr.r
  io.ddr.w.payload  := up.io.ddr.w.payload
  io.ddr.w.valid    := up.io.ddr.w.valid && !io.wStall
  up.io.ddr.w.ready := io.ddr.w.ready && !io.wStall
  // r2: the AXI4 protocol monitor watches the uplink's OWN master interface (upstream of the test stall)
  up.io.ddr.flatten.foreach(_.simPublic())
  io.rd       << up.io.rd
  io.dspAdmit := up.io.dspAdmit
}

object GenReadoutDdrUplinkDut extends App {
  val numCh = if (args.length > 0) args(0).toInt else 14
  SpinalConfig(targetDirectory = "build/rtl-ddr", defaultClockDomainFrequency = FixedFrequency(333 MHz))
    .generateVerilog(ReadoutDdrUplinkDut(ReadoutDdrUplinkParams(numCh = numCh)))
}
