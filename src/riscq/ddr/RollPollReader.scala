package riscq.ddr

import spinal.core._
import spinal.lib._

/**
 * Round-robin N -> 1 merger of the per-core result FIFOs (SpinalHDL port of QubiC `roll_poll_reader2`).
 *
 * Source contract (QubiC `data_buffer`): `dataValid(i)` holds until `rdEn(i)`, and the data stays put
 * through the cycle in which `rdEn(i)` is high (the FIFO pops on that edge).
 *
 * `numCh > pipelineThreshold` (8): the non-overlapped 3-phase pipeline BARREL -> ENCODE -> CONSUME, one
 * registered stage per phase, so each cycle holds one barrel shift OR one priority encode OR one data mux:
 *   - BARREL : register `dataValid` rotated so that `curIdx` maps to bit 0;
 *   - ENCODE : priority-encode the REGISTERED vector, register the absolute index;
 *   - CONSUME: `rdEn(pipeIdx)` (combinational from registers) and latch the data in the same cycle.
 * Service rate is therefore exactly 1 word / 3 cycles, and the vector seen in CONSUME is two cycles old:
 * a channel masked by the throttle after BARREL is still popped (the skid FIFO budget covers it).
 *
 * `numCh <= pipelineThreshold`: the single-cycle scan (1 word / cycle).
 *
 * Output: `wrEn`/`wrData` registered, one cycle after the pop; `wrData` is 0 when `wrEn` is low.
 * The vendored flush path (`N_shot_finished` + a 250-cycle delay) is not ported: the uplink never used
 * it (its flush is the DSP-side quiet/snapshot FSM, plan v3 §B.3).
 */
case class RollPollReader(numCh: Int, dataWidth: Int, pipelineThreshold: Int = 8) extends Component {
  require(numCh >= 1)
  val idxW = log2Up(numCh) max 1
  val pipelined = numCh > pipelineThreshold

  val io = new Bundle {
    val dataValid = in  Bits(numCh bits)
    val dataIn    = in  Vec(Bits(dataWidth bits), numCh)
    val wrEn      = out Bool()
    val wrData    = out Bits(dataWidth bits)
    val rdEn      = out Bits(numCh bits)
  }

  def wrapInc(i: UInt): UInt = (i === numCh - 1) ? U(0, idxW bits) | (i + 1)

  val curIdx = Reg(UInt(idxW bits)) init 0
  val wrEn   = Reg(Bool()) init False
  val wrData = Reg(Bits(dataWidth bits)) init 0
  wrEn   := False
  wrData := 0
  io.wrEn   := wrEn
  io.wrData := wrData

  val pipe = pipelined generate new Area {
    val phase     = Reg(UInt(2 bits)) init 0
    val dvRotR    = Reg(Bits(numCh bits)) init 0
    val pipeIdx   = Reg(UInt(idxW bits)) init 0
    val pipeFound = Reg(Bool()) init False
    // P3c: the CONSUME-phase pop strobe, registered one-hot (set with pipeIdx at the end of ENCODE, cleared
    // at the end of CONSUME), so `rdEn` is a plain flop and not a decoder of pipeIdx. Same cycles as before.
    val rdEnOH    = Reg(Bits(numCh bits)) init 0

    // BARREL: bit j of dvRot = dataValid((curIdx + j) mod numCh)
    val dvRot = (io.dataValid ## io.dataValid) >> curIdx
    // ENCODE: first set bit of the registered vector, then back to an absolute index
    val found  = dvRotR.orR
    val offset = OHToUInt(OHMasking.first(dvRotR)).resize(idxW + 1)
    val absSum = curIdx.resize(idxW + 1) + offset
    val selIdx = (absSum >= numCh) ? (absSum - numCh).resize(idxW) | absSum.resize(idxW)

    io.rdEn := rdEnOH

    switch(phase) {
      is(0) { dvRotR := dvRot.resize(numCh); phase := 1 }
      is(1) { pipeIdx := selIdx; pipeFound := found; rdEnOH := found ? (B(1, numCh bits) |<< selIdx) | B(0, numCh bits); phase := 2 }
      is(2) {
        phase := 0
        rdEnOH := 0
        when(pipeFound) {
          wrEn   := True
          wrData := io.dataIn(pipeIdx)
          curIdx := wrapInc(pipeIdx)
        }
      }
      default { phase := 0 }
    }
  }

  val direct = !pipelined generate new Area {
    // scan order curIdx, curIdx+1, ... (mod numCh); first valid wins
    val rot    = ((io.dataValid ## io.dataValid) >> curIdx).resize(numCh)
    val found  = rot.orR
    val offset = OHToUInt(OHMasking.first(rot)).resize(idxW + 1)
    val absSum = curIdx.resize(idxW + 1) + offset
    val selIdx = (absSum >= numCh) ? (absSum - numCh).resize(idxW) | absSum.resize(idxW)
    io.rdEn := 0
    when(found) {
      io.rdEn := B(1, numCh bits) |<< selIdx
      wrEn    := True
      wrData  := io.dataIn(selIdx)
      curIdx  := wrapInc(selIdx)
    }
  }
}
