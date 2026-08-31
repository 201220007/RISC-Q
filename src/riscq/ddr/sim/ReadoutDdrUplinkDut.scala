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
  for (i <- 0 until p.numCh) up.io.results(i) << io.results(i)
  up.io.calibDone := True          // the MIG is calibrated in every G2 scenario
  up.io.ctrl  << io.ctrl
  io.ddr      << up.io.ddr
  io.rd       << up.io.rd
  io.dspAdmit := up.io.dspAdmit
}

object GenReadoutDdrUplinkDut extends App {
  val numCh = if (args.length > 0) args(0).toInt else 14
  SpinalConfig(targetDirectory = "build/rtl-ddr", defaultClockDomainFrequency = FixedFrequency(333 MHz))
    .generateVerilog(ReadoutDdrUplinkDut(ReadoutDdrUplinkParams(numCh = numCh)))
}
