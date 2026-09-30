package riscq.ddr

import spinal.core._
import spinal.lib._

/**
 * AXI4 write master that empties the [[CircularBuffer]] read side into a DDR ring
 * (SpinalHDL port of QubiC `circular_buffer_axi_writer`, Forks C/C2/C3/C4).
 *
 * One presented bank = one run of beats at `curAxiAddr`:
 *   - IDLE: `ableToRead && !rdEmpty && !baseReset` -> issue the bank; `ableToRead && rdEmpty` (an empty
 *     FINAL bank) -> complete the run without a burst; either way `readFinished` returns the bank.
 *   - The bank is written as page-bounded bursts: a burst never crosses a 4 KiB boundary (fix F1). With
 *     the 512-B-aligned bases the register file enforces a bank never straddles a page, so there is one
 *     burst per bank; an unaligned base splits the bank into two bursts, each waiting for its own B.
 *   - WLAST is stable while WVALID && !WREADY (Fork C); `rdAddr` runs one row ahead of an accepted beat
 *     against the 1-cycle RAM read.
 *   - r2: WDATA is registered from the first cycle of a W stall (WVALID && !WREADY) until the handshake, so the
 *     payload cannot follow the RAM output while stalled, whatever happens to the buffer behind it (resets included).
 *     WSTRB is constant and WLAST changes only on a handshake.
 *   - The last bank of a run is the one the buffer presents with `rdFinal` (fix F4): its B (or its empty
 *     presentation) sets `finalAddr`, pulses `currentUserDone` and re-bases the pointer.
 *   - `baseReset` is honoured only in IDLE and has priority over a burst start in that cycle (Fork C3).
 *   - `addrFault` (Fork C4, sticky until an IDLE `baseReset`): a bank written past the ring end (the pointer
 *     wraps to 0 for it), or any bank whose last byte lies past `wrapLimit`.
 *   - Ring end (r1 fix): a non-final bank that ends exactly at the ring limit does NOT wrap the pointer yet.
 *     The pointer is parked at `wrapLimit + 1` (`pendingWrap`); if the next presentation is the empty FINAL
 *     bank, the run ends legally with `finalAddr = wrapLimit + 1` and no fault. Only a further non-empty bank
 *     wraps the pointer to 0, raises `addrFault` and is then written at the wrapped address.
 *   - `quiesce` (r1 reset hold): no new bank is started while it is high; a bank already started completes.
 *   - `writerIdle` = IDLE (Fork C2).
 * `wrapLimit` replaces the vendored `SIM_WRAP_LIMIT` define (production 0x7FFF_FFFF, ring size limit+1).
 */
case class CbufAxiWriter(rdWidth: Int, addrWidth: Int, axiAddrWidth: Int,
                         wrapLimit: BigInt = BigInt("7FFFFFFF", 16)) extends Component {
  val beatBytes = rdWidth / 8
  val axiSize   = log2Up(beatBytes)
  val pageBeats = 4096 / beatBytes
  require(isPow2(beatBytes) && pageBeats >= 1)
  val wrapSize  = wrapLimit + 1
  require(wrapSize < (BigInt(1) << axiAddrWidth), "the parked ring-end pointer (wrapLimit + 1) must fit the address")

  val io = new Bundle {
    val baseAddr        = in  UInt(axiAddrWidth bits)
    val baseReset       = in  Bool()
    val ableToRead      = in  Bool()
    val rdEmpty         = in  Bool()
    val rdFinal         = in  Bool()
    val rdAddrValid     = in  UInt(addrWidth bits)
    val rdData          = in  Bits(rdWidth bits)
    val rdAddr          = out UInt(addrWidth bits)
    val rdEn            = out Bool()
    val readFinished    = out Bool()
    val finalAddr       = out UInt(axiAddrWidth bits)
    val curAxiAddr      = out UInt(axiAddrWidth bits)
    val writerIdle      = out Bool()
    val addrFault       = out Bool()
    val currentUserDone = out Bool()
    val quiesce         = in  Bool() default(False)
    val aw = master(Stream(new Bundle {
      val addr  = UInt(axiAddrWidth bits)
      val len   = UInt(8 bits)
      val size  = UInt(3 bits)
      val burst = Bits(2 bits)
    }))
    val w = master(Stream(new Bundle {
      val data = Bits(rdWidth bits)
      val strb = Bits(rdWidth / 8 bits)
      val last = Bool()
    }))
    val b = slave(Stream(Bits(2 bits)))          // payload = BRESP
  }

  object St extends SpinalEnum(binarySequential) { val IDLE, AW, READ_WRITE, WAIT_B = newElement() }
  val state = Reg(St()) init St.IDLE

  val bankLen    = Reg(UInt(addrWidth + 1 bits)) init 0   // beats in this bank
  val beatCount  = Reg(UInt(addrWidth + 1 bits)) init 0   // beats sent in this bank
  val segLeft    = Reg(UInt(addrWidth + 1 bits)) init 0   // beats left in the current burst
  val curAxiAddr = Reg(UInt(axiAddrWidth bits)) init 0    // start of the current bank
  val lastBurst  = Reg(Bool()) init False
  val finalAddr  = Reg(UInt(axiAddrWidth bits)) init 0
  val addrFault  = Reg(Bool()) init False
  val done       = Reg(Bool()) init False
  val readFin    = Reg(Bool()) init False
  val rdEn       = Reg(Bool()) init False
  val awValid    = Reg(Bool()) init False
  val awAddr     = Reg(UInt(axiAddrWidth bits)) init 0
  val awLen      = Reg(UInt(8 bits)) init 0
  val wValid     = Reg(Bool()) init False
  val pendingWrap = Reg(Bool()) init False                 // pointer parked at the ring end (wrapLimit + 1)
  done := False; readFin := False; rdEn := False

  val wrapLimitU    = U(wrapLimit, axiAddrWidth + 1 bits)
  val wrapSizeU     = U(wrapSize, axiAddrWidth + 1 bits)
  val bankBytes     = (bankLen << axiSize).resize(axiAddrWidth + 1)
  val curPlusBank   = curAxiAddr.resize(axiAddrWidth + 1) + bankBytes
  def beatsToPage(a: UInt): UInt =
    (U(pageBeats, log2Up(pageBeats) + 1 bits) - a(11 downto axiSize).resize(log2Up(pageBeats) + 1))
  def minU(a: UInt, b: UInt): UInt = (a < b) ? a | b
  // first burst of a bank: min(bank beats, beats to the next 4 KiB boundary)
  val newBankLen = io.rdAddrValid.resize(addrWidth + 1) + 1
  val firstSeg   = minU(newBankLen.resize(log2Up(pageBeats) + 1 max addrWidth + 1),
                        beatsToPage(curAxiAddr).resize(log2Up(pageBeats) + 1 max addrWidth + 1)).resize(addrWidth + 1)

  val wHs = io.w.valid && io.w.ready
  io.rdAddr := (state === St.IDLE) ? U(0, addrWidth bits) |
               ((state === St.READ_WRITE && wHs) ? (beatCount + 1).resize(addrWidth) | beatCount.resize(addrWidth))
  io.rdEn         := rdEn
  io.readFinished := readFin
  io.finalAddr    := finalAddr
  io.curAxiAddr   := curAxiAddr
  io.writerIdle   := state === St.IDLE
  io.addrFault    := addrFault
  io.currentUserDone := done

  io.aw.valid         := awValid
  io.aw.payload.addr  := awAddr
  io.aw.payload.len   := awLen
  io.aw.payload.size  := axiSize
  io.aw.payload.burst := B"01"
  io.w.valid          := wValid
  val wStall    = io.w.valid && !io.w.ready
  val wStallD   = RegNext(wStall) init False
  val wDataHold = Reg(Bits(rdWidth bits))
  when(wStall && !wStallD)(wDataHold := io.rdData)
  io.w.payload.data   := wStallD ? wDataHold | io.rdData
  io.w.payload.strb   := B((BigInt(1) << (rdWidth / 8)) - 1, rdWidth / 8 bits)
  io.w.payload.last   := wValid && segLeft === 1
  io.b.ready          := True

  // C3: base_reset in IDLE re-bases the pointer and clears the per-run state
  val idleRebase = state === St.IDLE && io.baseReset
  when(idleRebase) {
    curAxiAddr := io.baseAddr
    lastBurst  := False
    addrFault  := False
    pendingWrap := False
  }
  // C4 (b): a bank whose last byte lies past the ring limit
  when(state === St.IDLE && io.ableToRead && !io.rdEmpty && !io.baseReset &&
       (curAxiAddr.resize(axiAddrWidth + 1) + (newBankLen << axiSize).resize(axiAddrWidth + 1) - 1) > wrapLimitU) {
    addrFault := True
  }

  switch(state) {
    is(St.IDLE) {
      when(io.ableToRead && !io.baseReset && !io.quiesce) {
        when(!io.rdEmpty && pendingWrap) {
          // data beyond the ring end: a real wrap. Re-base the parked pointer to the ring start, flag it,
          // and issue the bank from there in the next cycle.
          curAxiAddr  := (curAxiAddr.resize(axiAddrWidth + 1) - wrapSizeU).resize(axiAddrWidth)
          addrFault   := True
          pendingWrap := False
        } elsewhen(!io.rdEmpty) {
          lastBurst := io.rdFinal
          bankLen   := newBankLen
          beatCount := 0
          segLeft   := firstSeg
          awAddr    := curAxiAddr
          awLen     := (firstSeg - 1).resize(8)
          awValid   := True
          state     := St.AW
        } otherwise {
          when(io.rdFinal) {      // empty final bank: the run ends where the last full bank ended
            done       := True
            finalAddr  := curAxiAddr
            curAxiAddr := io.baseAddr
            lastBurst  := False
            pendingWrap := False
          }
          readFin := True
        }
      }
    }
    is(St.AW) {
      when(io.aw.ready) {
        awValid := False
        rdEn    := True
        wValid  := True
        state   := St.READ_WRITE
      }
    }
    is(St.READ_WRITE) {
      when(wHs) {
        rdEn      := True
        beatCount := beatCount + 1
        segLeft   := segLeft - 1
        when(segLeft === 1) { wValid := False; state := St.WAIT_B }
      }
    }
    is(St.WAIT_B) {
      when(io.b.valid) {
        when(beatCount === bankLen) {
          // bank complete
          when(lastBurst) {
            finalAddr  := curPlusBank.resize(axiAddrWidth)
            done       := True
            lastBurst  := False
            curAxiAddr := io.baseAddr
          } otherwise {
            // C4 (a), r1: at the ring end the pointer parks at wrapLimit + 1; the wrap (and the fault) happen
            // only if another non-empty bank follows
            curAxiAddr := curPlusBank.resize(axiAddrWidth)
            when(curPlusBank > wrapLimitU)(pendingWrap := True)
          }
          readFin := True
          state   := St.IDLE
        } otherwise {
          // next burst of the same bank starts on a page boundary
          val left = bankLen - beatCount
          segLeft := left
          awAddr  := (curAxiAddr.resize(axiAddrWidth + 1) + (beatCount << axiSize).resize(axiAddrWidth + 1)).resize(axiAddrWidth)
          awLen   := (left - 1).resize(8)
          awValid := True
          state   := St.AW
        }
      }
    }
  }
}
