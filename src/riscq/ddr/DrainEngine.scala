package riscq.ddr

import spinal.core._
import spinal.lib._

/**
 * DDR drain engine: AXI4 read master -> AXI-Stream with TLAST (SpinalHDL port of QubiC `mmu2` +
 * `async_fifo_same`).
 *
 * A chunk is `sizeBytes` bytes from `baseAddr`, streamed as `sizeBytes/32` beats with TLAST on the last.
 *   - Page-bounded bursts (fix F1): each AR asks for min(remaining, 256, beats to the next 4 KiB
 *     boundary) beats, so no burst crosses a 4 KiB page (at 32-B beats: <= 128 beats). One burst is
 *     outstanding at a time; the next AR waits for the previous RLAST and for FIFO room.
 *   - Start locking (fix F2): `baseAddr`/`sizeBytes` are latched at an accepted start, and a start is
 *     accepted only when the engine is idle: not busy AND the previous chunk's TLAST handshake is done.
 *   - Invalid requests (fix F3): size 0, size not a multiple of 32, size > `maxBytes`, or a base that is
 *     not 32-B aligned are rejected. A rejected start (for any reason, including "not idle") raises no
 *     busy, issues no AR, streams nothing and pulses `startRejected`.
 *   - `busy` rises the cycle after an accepted start and falls, with a one-cycle `done`, when the last R
 *     beat of the chunk is accepted into the FIFO (the vendored, advertised timing: before TLAST).
 *   - RRESP is not interpreted here; every beat is forwarded (the uplink's sticky monitor watches it).
 * FIFO: `StreamFifo` with asynchronous read (the vendored first-word-fall-through `async_fifo_same`);
 * RREADY drops when it holds `fifoDepth - 4` beats (the vendored almost-full hysteresis).
 */
case class DrainEngine(addrWidth: Int, dataWidth: Int, idWidth: Int, fifoDepth: Int = 16,
                       maxBytes: BigInt = -1) extends Component {
  val beatBytes = dataWidth / 8
  val lsb       = log2Up(beatBytes)
  val pageBeats = 4096 / beatBytes
  val maxB      = if (maxBytes < 0) BigInt(1) << addrWidth else maxBytes
  require(isPow2(beatBytes) && pageBeats >= 1 && maxB % beatBytes == 0 && maxB <= (BigInt(1) << addrWidth))
  val wordsW    = addrWidth + 1 - lsb

  val io = new Bundle {
    val start         = in  Bool()
    val baseAddr      = in  UInt(addrWidth bits)
    val sizeBytes     = in  UInt(addrWidth + 1 bits)
    val busy          = out Bool()
    val done          = out Bool()
    val idle          = out Bool()      // no chunk in flight: a start would be accepted (if valid)
    val startRejected = out Bool()      // one-cycle pulse
    val ar = master(Stream(new Bundle {
      val id    = UInt(idWidth bits)
      val addr  = UInt(addrWidth bits)
      val len   = UInt(8 bits)
      val size  = UInt(3 bits)
      val burst = Bits(2 bits)
    }))
    val r = slave(Stream(new Bundle {
      val id   = UInt(idWidth bits)
      val data = Bits(dataWidth bits)
      val resp = Bits(2 bits)
      val last = Bool()
    }))
    val axis = master(Stream(Fragment(Bits(dataWidth bits))))
  }

  val fifo = new StreamFifo(Bits(dataWidth bits), fifoDepth, withAsyncRead = true)
  val fifo_used = CombInit(fifo.io.occupancy)
  val afull = fifo.io.occupancy >= fifoDepth - 4

  val busy        = Reg(Bool()) init False
  val done        = Reg(Bool()) init False
  val streaming   = Reg(Bool()) init False       // from an accepted start to its TLAST handshake
  val rejected    = Reg(Bool()) init False
  val totalWords  = Reg(UInt(wordsW bits)) init 0
  val wordIdx     = Reg(UInt(wordsW bits)) init 0 // beats accepted into the FIFO
  val beatsSent   = Reg(UInt(wordsW bits)) init 0 // beats handed to AXIS
  val remaining   = Reg(UInt(wordsW bits)) init 0 // beats not yet requested
  val araddr      = Reg(UInt(addrWidth bits)) init 0
  val arlen       = Reg(UInt(8 bits)) init 0
  val arBeats     = Reg(UInt(9 bits)) init 0
  val arvalid     = Reg(Bool()) init False
  val outstanding = Reg(Bool()) init False
  done := False; rejected := False

  val idle   = !busy && !streaming
  val sizeOk = io.sizeBytes =/= 0 && io.sizeBytes(lsb - 1 downto 0) === 0 &&
               io.sizeBytes <= U(maxB, addrWidth + 1 bits) && io.baseAddr(lsb - 1 downto 0) === 0
  val accept = io.start && idle && sizeOk
  when(io.start && !accept)(rejected := True)
  io.busy := busy; io.done := done; io.idle := idle; io.startRejected := rejected

  // ---- AR ----
  val toPage  = U(pageBeats, 9 bits) - araddr(11 downto lsb).resize(9)
  val lenComb = {
    val a = (remaining > 256) ? U(256, 9 bits) | remaining.resize(9)
    (a < toPage) ? a | toPage
  }
  val canIssue = busy && !outstanding && !arvalid && !afull && remaining =/= 0
  when(accept) {
    busy       := True
    streaming  := True
    araddr     := io.baseAddr
    totalWords := (io.sizeBytes >> lsb).resize(wordsW)
    remaining  := (io.sizeBytes >> lsb).resize(wordsW)
    wordIdx    := 0
    beatsSent  := 0
  }
  when(canIssue) {
    arvalid := True
    arlen   := (lenComb - 1).resize(8)
    arBeats := lenComb
  }
  when(io.ar.fire) {
    arvalid     := False
    outstanding := True
    remaining   := remaining - arBeats.resize(wordsW)
    araddr      := araddr + (arBeats.resize(addrWidth) << lsb).resize(addrWidth)
  }
  io.ar.valid         := arvalid
  io.ar.payload.id    := 0
  io.ar.payload.addr  := araddr
  io.ar.payload.len   := arlen
  io.ar.payload.size  := lsb
  io.ar.payload.burst := B"01"

  // ---- R -> FIFO ----
  io.r.ready := outstanding && !afull
  fifo.io.push.valid   := io.r.fire
  fifo.io.push.payload := io.r.payload.data
  when(io.r.fire) {
    when(io.r.payload.last)(outstanding := False)
    wordIdx := wordIdx + 1
    when(wordIdx + 1 === totalWords) { busy := False; done := True }
  }

  // ---- FIFO -> AXIS ----
  val inChunk = streaming && beatsSent < totalWords
  io.axis.valid    := fifo.io.pop.valid && inChunk
  io.axis.fragment := fifo.io.pop.payload
  io.axis.last     := io.axis.valid && beatsSent + 1 === totalWords
  fifo.io.pop.ready := io.axis.ready && inChunk
  when(io.axis.fire) {
    beatsSent := beatsSent + 1
    when(io.axis.last)(streaming := False)
  }
}
