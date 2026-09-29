package riscq.ddr.sim

import spinal.core._
import spinal.lib._
import riscq.ddr._

/**
 * Stand-alone Verilog of the rewritten uplink internals under the VENDORED module and port names, so the
 * G1 cocotb scoreboards (src/riscq/ddr/cocotb/) run unchanged on the generated RTL (plan r2 item 6).
 * Each shell only renames ports and builds its clock domains from them; the logic is the very component
 * `ReadoutDdrUplink` instantiates. Reset: synchronous, active-low `*rst_n` (in the uplink the same
 * components run on the synchronous active-high `dspU`/`ddrU`). `dbg_*` ports are read-only taps for the
 * scoreboards (they replace the hierarchical references into the vendored internals).
 *
 * Run: mill-1.1.0 runMain riscq.ddr.sim.GenUplinkUnits [targetDir]   (default build/uplink-units)
 */
object UnitShells {
  val lowSync = ClockDomainConfig(resetKind = SYNC, resetActiveLevel = LOW)

  /** `roll_poll_reader2` (N_shot_finished / write_almost_finished accepted and ignored). */
  case class RollPollReader2(numCh: Int, dataWidth: Int) extends Component {
    val io = new Bundle {
      val clk = in Bool(); val rst_n = in Bool()
      val data_valid = in Bits(numCh bits)
      val data_in    = in Bits(numCh * dataWidth bits)
      val write_almost_finished = in Bool()
      val N_shot_finished       = in Bool()
      val wr_en   = out Bool()
      val wr_data = out Bits(dataWidth bits)
      val rd_en   = out Bits(numCh bits)
      val write_finished_external = out Bool()
    }
    noIoPrefix(); setDefinitionName("roll_poll_reader2")
    val cd = ClockDomain(io.clk, io.rst_n, config = lowSync)
    val u = cd(RollPollReader(numCh, dataWidth))
    u.io.dataValid := io.data_valid
    for (i <- 0 until numCh) u.io.dataIn(i) := io.data_in(i * dataWidth, dataWidth bits)
    io.wr_en := u.io.wrEn; io.wr_data := u.io.wrData; io.rd_en := u.io.rdEn
    io.write_finished_external := False
  }

  /** `circular_buffer3` (+ `rd_final_out`; `rd_en` accepted and ignored, as in the vendored module). */
  case class CircularBuffer3(wrWidth: Int, rdWidth: Int, addrWidth: Int) extends Component {
    val wrAddrW = addrWidth + log2Up(rdWidth / wrWidth)
    val io = new Bundle {
      val wr_clk = in Bool(); val wr_rst_n = in Bool(); val rd_clk = in Bool(); val rd_rst_n = in Bool()
      val wr_en = in Bool(); val wr_data = in Bits(wrWidth bits); val write_finished_ext = in Bool()
      val rd_en = in Bool(); val rd_addr = in Bits(addrWidth bits); val read_finished = in Bool()
      val rd_data = out Bits(rdWidth bits)
      val wr_en_out = out Bool(); val write_finished_out = out Bool(); val write_almost_finished_out = out Bool()
      val able_to_read_out = out Bool(); val rd_addr_valid_out = out Bits(addrWidth bits)
      val rd_empty = out Bool(); val wr_ready = out Bool(); val rd_final_out = out Bool()
      // taps
      val dbg_wr_addr = out Bits(wrAddrW bits); val dbg_bank_sel_wr = out Bool(); val dbg_credit = out Bool()
      val dbg_lva0 = out Bits(addrWidth bits); val dbg_lva1 = out Bits(addrWidth bits)
      val dbg_empty0 = out Bool(); val dbg_empty1 = out Bool(); val dbg_final0 = out Bool(); val dbg_final1 = out Bool()
      val dbg_ram_we_d = out Bool(); val dbg_rd_bank_sel = out Bool()
    }
    noIoPrefix(); setDefinitionName("circular_buffer3")
    val wrCd = ClockDomain(io.wr_clk, io.wr_rst_n, config = lowSync)
    val rdCd = ClockDomain(io.rd_clk, io.rd_rst_n, config = lowSync)
    val u = CircularBuffer(wrWidth, rdWidth, addrWidth, wrCd, rdCd)
    u.io.wrEn := io.wr_en; u.io.wrData := io.wr_data; u.io.writeFinishedExt := io.write_finished_ext
    u.io.rdAddr := io.rd_addr.asUInt; u.io.readFinished := io.read_finished
    io.rd_data := u.io.rdData; io.wr_en_out := u.io.wrEnOut; io.write_finished_out := u.io.writeFinishedOut
    io.write_almost_finished_out := False
    io.able_to_read_out := u.io.ableToRead; io.rd_addr_valid_out := u.io.rdAddrValid.asBits
    io.rd_empty := u.io.rdEmpty; io.wr_ready := u.io.wrReady; io.rd_final_out := u.io.rdFinal
    io.dbg_wr_addr     := u.wr.wr_addr.pull().asBits
    io.dbg_bank_sel_wr := u.wr.bank_sel_wr.pull()
    io.dbg_credit      := !u.wr.readerHolds.pull()
    io.dbg_lva0        := u.wr.last_valid_addr(0).pull().asBits
    io.dbg_lva1        := u.wr.last_valid_addr(1).pull().asBits
    io.dbg_empty0      := !u.wr.bank_used(0).pull()
    io.dbg_empty1      := !u.wr.bank_used(1).pull()
    io.dbg_final0      := u.wr.bank_final(0).pull()
    io.dbg_final1      := u.wr.bank_final(1).pull()
    io.dbg_ram_we_d    := u.wr.weA_d.pull()
    io.dbg_rd_bank_sel := u.rd.rd_bank_sel.pull()
  }

  /** `circular_buffer_axi_writer` (`write_finished_ext` replaced by the cbuf's in-band `rd_final`). */
  case class CircularBufferAxiWriter(rdWidth: Int, addrWidth: Int, axiAddrWidth: Int, wrapLimit: BigInt) extends Component {
    val io = new Bundle {
      val clk = in Bool(); val rst_n = in Bool()
      val base_addr = in Bits(axiAddrWidth bits); val base_reset = in Bool()
      val able_to_read = in Bool(); val rd_empty = in Bool(); val rd_final = in Bool()
      val rd_addr_valid = in Bits(addrWidth bits); val rd_data = in Bits(rdWidth bits)
      val rd_addr = out Bits(addrWidth bits); val rd_en = out Bool(); val read_finished = out Bool()
      val final_addr = out Bits(axiAddrWidth bits); val cur_axi_addr_out = out Bits(axiAddrWidth bits)
      val writer_idle = out Bool(); val addr_fault = out Bool(); val current_user_done = out Bool()
      val m_axi_awaddr = out Bits(axiAddrWidth bits); val m_axi_awlen = out Bits(8 bits)
      val m_axi_awsize = out Bits(3 bits); val m_axi_awburst = out Bits(2 bits)
      val m_axi_awvalid = out Bool(); val m_axi_awready = in Bool()
      val m_axi_wdata = out Bits(rdWidth bits); val m_axi_wstrb = out Bits(rdWidth / 8 bits)
      val m_axi_wlast = out Bool(); val m_axi_wvalid = out Bool(); val m_axi_wready = in Bool()
      val m_axi_bresp = in Bits(2 bits); val m_axi_bvalid = in Bool(); val m_axi_bready = out Bool()
      val dbg_state = out Bits(3 bits)
    }
    noIoPrefix(); setDefinitionName("circular_buffer_axi_writer")
    val cd = ClockDomain(io.clk, io.rst_n, config = lowSync)
    val u = cd(CbufAxiWriter(rdWidth, addrWidth, axiAddrWidth, wrapLimit))
    u.io.baseAddr := io.base_addr.asUInt; u.io.baseReset := io.base_reset
    u.io.ableToRead := io.able_to_read; u.io.rdEmpty := io.rd_empty; u.io.rdFinal := io.rd_final
    u.io.rdAddrValid := io.rd_addr_valid.asUInt; u.io.rdData := io.rd_data
    io.rd_addr := u.io.rdAddr.asBits; io.rd_en := u.io.rdEn; io.read_finished := u.io.readFinished
    io.final_addr := u.io.finalAddr.asBits; io.cur_axi_addr_out := u.io.curAxiAddr.asBits
    io.writer_idle := u.io.writerIdle; io.addr_fault := u.io.addrFault; io.current_user_done := u.io.currentUserDone
    io.m_axi_awaddr := u.io.aw.addr.asBits; io.m_axi_awlen := u.io.aw.len.asBits
    io.m_axi_awsize := u.io.aw.size.asBits; io.m_axi_awburst := u.io.aw.burst
    io.m_axi_awvalid := u.io.aw.valid; u.io.aw.ready := io.m_axi_awready
    io.m_axi_wdata := u.io.w.data; io.m_axi_wstrb := u.io.w.strb; io.m_axi_wlast := u.io.w.last
    io.m_axi_wvalid := u.io.w.valid; u.io.w.ready := io.m_axi_wready
    u.io.b.payload := io.m_axi_bresp; u.io.b.valid := io.m_axi_bvalid; io.m_axi_bready := u.io.b.ready
    io.dbg_state := u.state.pull().asBits.resize(3)
  }

  /** `mmu2` (+ `start_rejected`, `idle`, and the FIFO occupancy tap `dbg_fifo_used`). */
  case class Mmu2(axiAddrWidth: Int, axiDataWidth: Int, axiIdWidth: Int, maxBytes: BigInt) extends Component {
    val io = new Bundle {
      val clk = in Bool(); val rst_n = in Bool()
      val start = in Bool(); val busy = out Bool(); val done = out Bool()
      val idle = out Bool(); val start_rejected = out Bool()
      val base_addr = in Bits(axiAddrWidth bits); val size_bytes = in Bits(axiAddrWidth + 1 bits)
      val arid = out Bits(axiIdWidth bits); val araddr = out Bits(axiAddrWidth bits); val arlen = out Bits(8 bits)
      val arsize = out Bits(3 bits); val arburst = out Bits(2 bits); val arvalid = out Bool(); val arready = in Bool()
      val rid = in Bits(axiIdWidth bits); val rdata = in Bits(axiDataWidth bits); val rresp = in Bits(2 bits)
      val rlast = in Bool(); val rvalid = in Bool(); val rready = out Bool()
      val m_axis_tdata = out Bits(axiDataWidth bits); val m_axis_tvalid = out Bool()
      val m_axis_tready = in Bool(); val m_axis_tlast = out Bool()
      val dbg_fifo_used = out UInt(5 bits)
    }
    noIoPrefix(); setDefinitionName("mmu2")
    val cd = ClockDomain(io.clk, io.rst_n, config = lowSync)
    val u = cd(DrainEngine(axiAddrWidth, axiDataWidth, axiIdWidth, maxBytes = maxBytes))
    u.io.start := io.start; u.io.baseAddr := io.base_addr.asUInt; u.io.sizeBytes := io.size_bytes.asUInt
    io.busy := u.io.busy; io.done := u.io.done; io.idle := u.io.idle; io.start_rejected := u.io.startRejected
    io.arid := u.io.ar.id.asBits; io.araddr := u.io.ar.addr.asBits; io.arlen := u.io.ar.len.asBits
    io.arsize := u.io.ar.size.asBits; io.arburst := u.io.ar.burst; io.arvalid := u.io.ar.valid
    u.io.ar.ready := io.arready
    u.io.r.valid := io.rvalid; u.io.r.payload.id := io.rid.asUInt; u.io.r.payload.data := io.rdata
    u.io.r.payload.resp := io.rresp; u.io.r.payload.last := io.rlast; io.rready := u.io.r.ready
    io.m_axis_tdata := u.io.axis.fragment; io.m_axis_tvalid := u.io.axis.valid; io.m_axis_tlast := u.io.axis.last
    u.io.axis.ready := io.m_axis_tready
    io.dbg_fifo_used := u.fifo_used.pull().resize(5)
  }
}

object GenUplinkUnits extends App {
  import UnitShells._
  val dir = if (args.nonEmpty) args(0) else "build/uplink-units"
  val cfg = SpinalConfig(targetDirectory = dir, oneFilePerComponent = false)
  cfg.copy(netlistFileName = "roll_poll_reader2.v").generateVerilog(RollPollReader2(14, 64))
  cfg.copy(netlistFileName = "circular_buffer3.v").generateVerilog(CircularBuffer3(64, 256, 4))
  // G1 writer suite ring limit (was -DSIM_WRAP_LIMIT=63999): 0xF9FF, ring = 125 banks, not 4 KiB aligned
  cfg.copy(netlistFileName = "circular_buffer_axi_writer.v")
    .generateVerilog(CircularBufferAxiWriter(256, 4, 32, wrapLimit = 63999))
  cfg.copy(netlistFileName = "mmu2.v").generateVerilog(Mmu2(32, 256, 4, maxBytes = ReadoutDdrRegs.MAX_RD_SIZE))
}
