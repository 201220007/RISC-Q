package riscq.ddr

import spinal.core._
import spinal.lib._

/**
 * The uplink's per-core result FIFO (P3c-2). At its ports it behaves exactly as `StreamFifo(Bits(w), depth)` does
 * for a power-of-two depth (synchronous read, no bypass, no flush): `push.ready`, `pop.valid`, `pop.payload`, cycle for
 * cycle. Two things differ inside, both for 500 MHz timing (the write-enable cone of the 14q build):
 *  - `full` is a register. It is loaded with what StreamFifo's combinational `full` will be in the next cycle:
 *    (push pointer after this cycle's push) - (the pop-at-the-port pointer after this cycle's port pop) == depth;
 *  - the RAM is written in EVERY cycle that the FIFO is not full, at the push pointer, and only a push (`push.fire`)
 *    advances the pointer. The write enable is therefore `!full`, a flop, and not the push handshake (the decoder
 *    valid edge, the admission, the arbiter and the full compare). Writing the free slot with an unused value is
 *    harmless: that slot holds no entry, and the read and write addresses can only be equal when the FIFO is empty
 *    (no read is issued) or full (no write happens).
 * Power-up without a reset (all registers 0, `ReadoutDdrUplinkCdcSim start_dsp_dead_no_reset`) reads as empty and
 * not full, as StreamFifo's pointers do.
 */
case class UplinkResultFifo(width: Int, depth: Int) extends Component {
  require(isPow2(depth) && depth >= 2, s"UplinkResultFifo: depth $depth must be a power of two >= 2")
  val io = new Bundle {
    val push = slave(Stream(Bits(width bits)))
    val pop  = master(Stream(Bits(width bits)))
  }
  val aw = log2Up(depth)
  val ram = Mem(Bits(width bits), depth)
  val pushPtr = Reg(UInt(aw + 1 bits)) init 0
  val popPtr  = Reg(UInt(aw + 1 bits)) init 0
  val popOnIo = Reg(UInt(aw + 1 bits)) init 0      // StreamFifo's popReg: the pop pointer as seen at the port
  val full    = Reg(Bool()) init False

  io.push.ready := !full
  val doPush = io.push.fire
  ram.write(pushPtr.resize(aw), io.push.payload, enable = !full)
  when(doPush)(pushPtr := pushPtr + 1)

  // the read side, as StreamFifo's synchronous-read path
  val addressGen = Stream(UInt(aw bits))
  addressGen.valid   := pushPtr =/= popPtr
  addressGen.payload := popPtr.resize(aw)
  when(addressGen.fire)(popPtr := popPtr + 1)
  val readArbitration = addressGen.m2sPipe()
  val rsp = ram.readSync(addressGen.payload, addressGen.fire)
  io.pop << readArbitration.translateWith(rsp)
  when(readArbitration.fire)(popOnIo := popPtr)

  val pushNext    = pushPtr + doPush.asUInt.resize(aw + 1)
  val popOnIoNext = readArbitration.fire ? popPtr | popOnIo
  full := (pushNext - popOnIoNext) === depth
}
