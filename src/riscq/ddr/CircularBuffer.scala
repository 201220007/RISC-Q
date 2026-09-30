package riscq.ddr

import spinal.core._
import spinal.lib._

/**
 * Two-bank (ping-pong) circular buffer between the DSP write side and the DDR read side
 * (SpinalHDL port of QubiC `circular_buffer3` + `cbuf_ram_read_wider`, with Fork A `wr_ready`).
 *
 * Write side (`wrCd`): 64-bit words fill the write bank; `wrReady` is low only while the LAST slot of
 * the write bank is reached and the reader still holds the other bank (Fork A stall). Every write-side
 * effect (RAM strobe, metadata, switch) is gated by `wrAccept = wrEn && wrReady`.
 * Read side (`rdCd`): the reader sees one presented bank at a time as 256-bit rows with a 1-cycle
 * registered read (`rdData` is the row addressed by `rdAddr` in the previous cycle).
 *
 * Bank ownership (contract fix F4). A bank is handed to the reader ("presented") only
 *   - when it is FULL: the write of its last slot switches banks seamlessly, if the reader has returned
 *     the other bank (credit); otherwise that write stalls;
 *   - when a flush (`writeFinishedExt`) closes it: the flush stays pending until the credit is back and
 *     then presents the current write bank, with 0..63 words, marked FINAL. Words accepted while the
 *     flush is pending join that final bank; they do not cancel the flush.
 * The presentation carries its metadata in band: `rdAddrValid` (last row), `rdEmpty` and `rdFinal`.
 * The reader owns the presented bank until it pulses `readFinished`; while it owns none, `ableToRead`
 * is 0 and `rdEmpty` is 1, so `rdEmpty` alone says "no pending bank".
 *
 * Crossings. `presTog` (wr -> rd) toggles once per presentation, one wr cycle after the switch (so the
 * last word's registered RAM write has landed) and after the bank's metadata was frozen; the read side
 * sees it through three flops and the metadata through two, then captures the metadata into its own
 * registers. `retTog` (rd -> wr) toggles once per `readFinished` of an owned bank and returns the credit.
 * The metadata of a presented bank is not touched by the write side until the credit is back, so the
 * capture is of quasi-static data. Toggles work at any clock ratio (the vendored 1-cycle pulse did not).
 *
 * r2: a presentation is taken only while the reader owns NO bank (one arriving while it owns one is deferred until
 * `readFinished`), and none at all while `rdFreeze` is high (the uplink's reset hold). The DSP-side reset clears
 * `presTog` while the DDR side is still finishing a burst; without these gates that edge looked like a new
 * presentation and switched `rd_bank_sel` (hence `rdData`) under the writer's open burst.
 *
 * Reset/power-up: every register resets to 0 (credit is encoded as `readerHolds`, 0 = credit present),
 * so an FPGA power-up (INIT = 0) is the reset state even if one clock is still dead.
 */
case class CircularBuffer(wrWidth: Int, rdWidth: Int, addrWidth: Int,
                          wrCd: ClockDomain, rdCd: ClockDomain) extends Component {
  val ratio    = rdWidth / wrWidth
  require(ratio >= 1 && isPow2(ratio) && ratio * wrWidth == rdWidth)
  val ratioLg2 = log2Up(ratio)
  val rdDepth  = 1 << addrWidth            // rows per bank
  val wrDepth  = rdDepth * ratio           // words per bank
  val wrAddrW  = addrWidth + ratioLg2

  val io = new Bundle {
    // write side (wrCd)
    val wrEn             = in  Bool()
    val wrData           = in  Bits(wrWidth bits)
    val writeFinishedExt = in  Bool()       // flush: close the current write bank as FINAL
    val wrReady          = out Bool()
    val wrEnOut          = out Bool()       // = wrAccept (also the RAM strobe)
    val writeFinishedOut = out Bool()       // a flush is pending (not yet presented)
    // read side (rdCd)
    val rdAddr           = in  UInt(addrWidth bits)
    val readFinished     = in  Bool()
    val rdData           = out Bits(rdWidth bits)
    val ableToRead       = out Bool()
    val rdAddrValid      = out UInt(addrWidth bits)
    val rdEmpty          = out Bool()
    val rdFinal          = out Bool()
    val rdFreeze         = in  Bool() default(False)   // rdCd: take no new presentation (reset hold)
  }

  // 2 banks x rdDepth rows of rdWidth bits, written one wrWidth lane at a time (lane-masked write:
  // the Verilog backend cannot emit true mixed-width ports), read one full row.
  val ram = Mem(Bits(rdWidth bits), 2 * rdDepth)
  ram.addAttribute("ram_style", "block")

  // ───────────────────────── write side ─────────────────────────
  val wr = new ClockingArea(wrCd) {
    val bank_sel_wr     = Reg(Bool()) init False                      // bank being written
    val wr_addr         = Reg(UInt(wrAddrW bits)) init 0
    val last_valid_addr = Vec(Reg(UInt(addrWidth bits)) init 0, 2)
    val bank_used       = Vec(Reg(Bool()) init False, 2)              // bank holds >= 1 word
    val bank_final      = Vec(Reg(Bool()) init False, 2)
    val readerHolds     = Reg(Bool()) init False                      // 0 = credit present
    val flushPend       = Reg(Bool()) init False
    val presTog         = Reg(Bool()) init False

    // credit return (rd -> wr toggle)
    val retSync = Bool()
    val retPrev = Reg(Bool()) init False
    when(retSync =/= retPrev) { retPrev := retSync; readerHolds := False }

    val atLast   = wr_addr === wrDepth - 1
    val wrReady  = !(atLast && readerHolds)
    val wrAccept = io.wrEn && wrReady
    io.wrReady  := wrReady
    io.wrEnOut  := wrAccept

    when(io.writeFinishedExt)(flushPend := True)

    def switchBank(isFinal: Bool): Unit = {
      bank_final(bank_sel_wr.asUInt) := isFinal
      bank_sel_wr := !bank_sel_wr
      wr_addr     := 0
      readerHolds := True
      presTog     := !presTog
      // the new write bank is the one the reader returned: clear its metadata
      bank_used((!bank_sel_wr).asUInt)       := False
      last_valid_addr((!bank_sel_wr).asUInt) := U(0, addrWidth bits)
      bank_final((!bank_sel_wr).asUInt)      := False
    }

    when(wrAccept) {
      bank_used(bank_sel_wr.asUInt)       := True
      last_valid_addr(bank_sel_wr.asUInt) := wr_addr(wrAddrW - 1 downto ratioLg2)
      when(atLast) { switchBank(False) } otherwise { wr_addr := wr_addr + 1 }
    }
    when(flushPend && !readerHolds && !wrAccept) {
      switchBank(True)
      flushPend := False
    }
    io.writeFinishedOut := flushPend

    // registered RAM write (the vendored BRAM input stage; keeps the 500 MHz write path short)
    val weA_d   = RegNext(wrAccept) init False
    val addrA_d = RegNext(bank_sel_wr ## wr_addr)
    val diA_d   = RegNext(io.wrData)
    ram.write(
      address = (addrA_d >> ratioLg2).asUInt,
      data    = Cat(Seq.fill(ratio)(diA_d)),
      enable  = weA_d,
      mask    = UIntToOh(addrA_d(ratioLg2 - 1 downto 0).asUInt, ratio))

    // the presentation becomes visible one cycle after the switch, together with the last RAM write
    val presTogOut = RegNext(presTog) init False
  }

  // ───────────────────────── read side ─────────────────────────
  val rd = new ClockingArea(rdCd) {
    val presSync = BufferCC(wr.presTogOut, init = False, bufferDepth = 3)   // one flop more than the metadata
    val selSync  = BufferCC(wr.bank_sel_wr,     init = False, bufferDepth = 2)
    val lvaSync  = BufferCC(wr.last_valid_addr.asBits, init = B(0, 2 * addrWidth bits), bufferDepth = 2)
    val usedSync = BufferCC(wr.bank_used.asBits,  init = B(0, 2 bits), bufferDepth = 2)
    val finSync  = BufferCC(wr.bank_final.asBits, init = B(0, 2 bits), bufferDepth = 2)

    val presPrev     = Reg(Bool()) init False
    val own          = Reg(Bool()) init False
    val rd_bank_sel  = Reg(Bool()) init False
    val rd_lva       = Reg(UInt(addrWidth bits)) init 0
    val rd_used      = Reg(Bool()) init False
    val rd_final     = Reg(Bool()) init False
    val retTog       = Reg(Bool()) init False

    when(presSync =/= presPrev && !own && !io.rdFreeze) {
      // a new presentation: the presented bank is the one the write side just left
      presPrev    := presSync
      own         := True
      val b = (!selSync).asUInt
      rd_bank_sel := !selSync
      rd_lva      := lvaSync.subdivideIn(addrWidth bits)(b).asUInt
      rd_used     := usedSync(b)
      rd_final    := finSync(b)
    }
    when(io.readFinished && own) {
      own    := False
      retTog := !retTog
    }

    io.ableToRead  := own && !io.readFinished
    io.rdEmpty     := !own || !rd_used
    io.rdAddrValid := own ? rd_lva | U(0, addrWidth bits)
    io.rdFinal     := own && rd_final
    io.rdData      := ram.readSync((rd_bank_sel ## io.rdAddr).asUInt, clockCrossing = true)
  }
  wr.retSync := wrCd(BufferCC(rd.retTog, init = False, bufferDepth = 2))
}
