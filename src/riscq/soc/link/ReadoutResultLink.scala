package riscq.soc.link

import spinal.core._
import spinal.lib._

/**
 * Readout decoder result, carried **upstream** (DSP → core) on the link's second narrow posted `Flow`.
 * The decoder's integrated point — the 1-bit discrimination `res` plus the integrated I/Q
 * (`real`/`imag`) — travels up into a near-core [[ReadoutResultSink]] the CPU polls locally, so the
 * halting `res` read is a short local arc instead of a long bus round-trip.
 */
case class ReadoutResult(accWidth: Int) extends Bundle {
  val res  = Bool()
  val real = SInt(accWidth bits)
  val imag = SInt(accWidth bits)
}

object ReadoutResultLink {
  /**
   * DSP-side source: forward the decoder's `res.valid` **as a level** (not an edge) with the current
   * `res`/`real`/`imag`. The carrier-triggered decoder already shapes `res.valid` exactly right — high
   * from a window's settle until the next window's `winStart` clears it, i.e. low exactly while a fresh
   * window integrates — so the sink can mirror it directly: no edge-detect, no stale-beat bookkeeping.
   */
  def source(resValid: Bool, res: Bool, real: SInt, imag: SInt, accWidth: Int): Flow[ReadoutResult] = {
    val out = Flow(ReadoutResult(accWidth))
    out.valid        := resValid
    out.payload.res  := res
    out.payload.real := real
    out.payload.imag := imag
    out
  }
}
