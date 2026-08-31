package riscq.ddr

import spinal.core._
import spinal.lib._
import scala.io.Source

/**
 * BlackBoxes over the vendored QubiC readout-MMU Verilog in `src/riscq/ddr/rtl/` (see VENDORED.md there
 * for provenance + the four reviewed forks). Each blackbox inlines its module AND its private
 * dependencies, so the generated `PulseTableSoc.v` stays self-contained (same approach as
 * [[riscq.memory.BramBlackBox]]).
 */
private object VendoredRtl {
  val dir = "src/riscq/ddr/rtl/"
  def read(files: String*): String = files.map(f => Source.fromFile(dir + f).mkString).mkString("\n\n")
}

/** `roll_poll_reader2` — round-robin N→1 merger with the 3-phase pipeline for NUM_CH > 8. */
case class RollPollReader2BB(numCh: Int, dataWidth: Int) extends BlackBox {
  addGeneric("NUM_CH", numCh)
  addGeneric("DATA_WIDTH", dataWidth)
  val io = new Bundle {
    val clk                     = in  Bool()
    val rst_n                   = in  Bool()
    val data_valid              = in  Bits(numCh bits)
    val data_in                 = in  Bits(numCh * dataWidth bits)
    val write_almost_finished   = in  Bool()
    val N_shot_finished         = in  Bool()
    val wr_en                   = out Bool()
    val wr_data                 = out Bits(dataWidth bits)
    val rd_en                   = out Bits(numCh bits)
    val write_finished_external = out Bool()
  }
  noIoPrefix()
  setDefinitionName("roll_poll_reader2")
  setInlineVerilog(VendoredRtl.read("roll_poll_reader2.v"))
}

/** `circular_buffer3` (Fork A: `wr_ready`) + its dedicated 1-cycle `cbuf_ram_read_wider` (Fork B). */
case class CircularBuffer3BB(wrDataWidth: Int, rdDataWidth: Int, addrWidth: Int) extends BlackBox {
  addGeneric("WR_DATA_WIDTH", wrDataWidth)
  addGeneric("RD_DATA_WIDTH", rdDataWidth)
  addGeneric("ADDR_WIDTH", addrWidth)
  val io = new Bundle {
    val wr_clk                    = in  Bool()
    val wr_rst_n                  = in  Bool()
    val rd_clk                    = in  Bool()
    val rd_rst_n                  = in  Bool()
    val wr_en                     = in  Bool()
    val wr_data                   = in  Bits(wrDataWidth bits)
    val write_finished_ext        = in  Bool()
    val rd_en                     = in  Bool()
    val rd_addr                   = in  Bits(addrWidth bits)
    val read_finished             = in  Bool()
    val rd_data                   = out Bits(rdDataWidth bits)
    val wr_en_out                 = out Bool()
    val write_finished_out        = out Bool()
    val write_almost_finished_out = out Bool()
    val able_to_read_out          = out Bool()
    val rd_addr_valid_out         = out Bits(addrWidth bits)
    val rd_empty                  = out Bool()
    val wr_ready                  = out Bool()
  }
  noIoPrefix()
  setDefinitionName("circular_buffer3")
  setInlineVerilog(VendoredRtl.read("cbuf_ram_read_wider.v", "circular_buffer3.v"))
}

/** `circular_buffer_axi_writer` (Forks C/C2/C3/C4). AXI write-master half of the uplink. */
case class CbufAxiWriterBB(rdDataWidth: Int, addrWidth: Int, axiAddrWidth: Int) extends BlackBox {
  addGeneric("RD_DATA_WIDTH", rdDataWidth)
  addGeneric("ADDR_WIDTH", addrWidth)
  addGeneric("AXI_ADDR_WIDTH", axiAddrWidth)
  val io = new Bundle {
    val clk                = in  Bool()
    val rst_n              = in  Bool()
    val base_addr          = in  Bits(axiAddrWidth bits)
    val base_reset         = in  Bool()
    val able_to_read       = in  Bool()
    val rd_empty           = in  Bool()
    val rd_addr_valid      = in  Bits(addrWidth bits)
    val rd_data            = in  Bits(rdDataWidth bits)
    val write_finished_ext = in  Bool()
    val rd_addr            = out Bits(addrWidth bits)
    val rd_en              = out Bool()
    val read_finished      = out Bool()
    val final_addr         = out Bits(axiAddrWidth bits)
    val cur_axi_addr_out   = out Bits(axiAddrWidth bits)
    val writer_idle        = out Bool()
    val addr_fault         = out Bool()
    val current_user_done  = out Bool()
    val m_axi_awaddr       = out Bits(axiAddrWidth bits)
    val m_axi_awlen        = out Bits(8 bits)
    val m_axi_awsize       = out Bits(3 bits)
    val m_axi_awburst      = out Bits(2 bits)
    val m_axi_awvalid      = out Bool()
    val m_axi_awready      = in  Bool()
    val m_axi_wdata        = out Bits(rdDataWidth bits)
    val m_axi_wstrb        = out Bits(rdDataWidth / 8 bits)
    val m_axi_wlast        = out Bool()
    val m_axi_wvalid       = out Bool()
    val m_axi_wready       = in  Bool()
    val m_axi_bresp        = in  Bits(2 bits)
    val m_axi_bvalid       = in  Bool()
    val m_axi_bready       = out Bool()
  }
  noIoPrefix()
  setDefinitionName("circular_buffer_axi_writer")
  setInlineVerilog(VendoredRtl.read("circular_buffer_axi_writer.v"))
}

/** `mmu2` + `async_fifo_same` — AXI4 read master → AXI-Stream (with tlast) drain engine. */
case class Mmu2BB(axiAddrWidth: Int, axiDataWidth: Int, axiIdWidth: Int, fifoAddrBits: Int = 4) extends BlackBox {
  addGeneric("AXI_ADDR_WIDTH", axiAddrWidth)
  addGeneric("AXI_DATA_WIDTH", axiDataWidth)
  addGeneric("AXI_ID_WIDTH", axiIdWidth)
  addGeneric("FIFO_ADDR_BITS", fifoAddrBits)
  val io = new Bundle {
    val clk           = in  Bool()
    val rst_n         = in  Bool()
    val start         = in  Bool()
    val busy          = out Bool()
    val done          = out Bool()
    val base_addr     = in  Bits(axiAddrWidth bits)
    val size_bytes    = in  Bits(axiAddrWidth + 1 bits)
    val arid          = out Bits(axiIdWidth bits)
    val araddr        = out Bits(axiAddrWidth bits)
    val arlen         = out Bits(8 bits)
    val arsize        = out Bits(3 bits)
    val arburst       = out Bits(2 bits)
    val arvalid       = out Bool()
    val arready       = in  Bool()
    val rid           = in  Bits(axiIdWidth bits)
    val rdata         = in  Bits(axiDataWidth bits)
    val rresp         = in  Bits(2 bits)
    val rlast         = in  Bool()
    val rvalid        = in  Bool()
    val rready        = out Bool()
    val m_axis_tdata  = out Bits(axiDataWidth bits)
    val m_axis_tvalid = out Bool()
    val m_axis_tready = in  Bool()
    val m_axis_tlast  = out Bool()
  }
  noIoPrefix()
  setDefinitionName("mmu2")
  setInlineVerilog(VendoredRtl.read("async_fifo_same.v", "mmu2.v"))
}
