package riscq.ddr

import spinal.core._
import spinal.lib._

/**
 * Four-phase request/acknowledge event crossing between two unrelated clock domains
 * (qubic3 PLAN_READOUT_DDR v3 §C → v7; replaces the vendored QubiC `pulse_sync.v`).
 *
 * Source side (`srcCd`): a one-cycle `start` raises the level `req` (ignored while `busy`).
 * Destination side (`dstCd`): `req` is 2-FF synchronised (ASYNC_REG) into an edge detector whose
 * `prev` is SEEDED from the first synced samples after a destination reset, so a request that was
 * already high when the destination came out of reset is never mistaken for a new edge: it is treated
 * as STALE — acknowledged without a `fire` and flagged `dropped`. A genuine rising edge produces
 * exactly one `fire` pulse; `ack` is raised when `dstDone` is seen (or immediately when
 * `withDstDone == false`). The source drops `req` on `ack`, the destination drops `ack` on `req`↓,
 * and `busy` clears when the source sees `ack`↓ — full 4-phase, so requests can never overlap.
 *
 * Reset semantics: a lone destination reset → stale handling above (`ackDropped` reported with the
 * acknowledge, never a duplicate pulse). A lone source reset drops `req`; the destination then drops
 * `ack` and the transaction vanishes (`busy` is recomputed from the synced `ack`, nothing sticks);
 * the run-level protocol of the controller makes that observable (write_done never set).
 */
case class PulseCross(srcCd: ClockDomain, dstCd: ClockDomain, withDstDone: Boolean = false) extends Component {
  val io = new Bundle {
    // source domain
    val start      = in  Bool()   // one-cycle request (srcCd); ignored while busy
    val busy       = out Bool()   // level (srcCd): request in flight (req || synced ack)
    val acked      = out Bool()   // one-cycle pulse (srcCd): the destination acknowledged
    val ackDropped = out Bool()   // valid with `acked`: acknowledged as STALE (no fire happened)
    // destination domain
    val fire       = out Bool()   // one-cycle pulse (dstCd)
    val dstDone    = in  Bool()   // dstCd: completion of the fired action (used iff withDstDone)
    val dropped    = out Bool()   // level (dstCd): current/last transaction was acknowledged stale
  }
  noIoPrefix()

  // cross-domain levels (each driven in exactly one domain, consumed through a 2-FF sync in the other)
  val reqLevel  = Bool()
  val ackLevel  = Bool()
  val dropLevel = Bool()

  // ---------------- source side ----------------
  val src = new ClockingArea(srcCd) {
    val reqReg   = Reg(Bool()) init False
    val ackSync  = BufferCC(ackLevel,  init = False, bufferDepth = 2)
    val dropSync = BufferCC(dropLevel, init = False, bufferDepth = 2)
    val busy     = reqReg || ackSync
    when(io.start && !busy)(reqReg := True)
    when(reqReg && ackSync)(reqReg := False)
    val ackRise  = ackSync && !RegNext(ackSync, False)
    // report the ack two cycles after its rise so the independently-synced `dropped` level has settled
    val acked    = Delay(ackRise, 2, init = False)
    reqLevel      := reqReg
    io.busy       := busy
    io.acked      := acked
    io.ackDropped := acked && dropSync
  }

  // ---------------- destination side ----------------
  val dst = new ClockingArea(dstCd) {
    val reqSync = BufferCC(reqLevel, init = False, bufferDepth = 2)
    // seed window: 3 cycles after reset `prev` only tracks reqSync (the 2-FF sync needs 2 cycles to show
    // the real level; no edge can fire meanwhile); armed from the 4th cycle on.
    val seedCnt = Reg(UInt(2 bits)) init 0
    val armed   = seedCnt === 3
    when(!armed)(seedCnt := seedCnt + 1)
    val prev    = Reg(Bool()) init False
    val ackReg  = Reg(Bool()) init False
    val dropReg = Reg(Bool()) init False
    val pending = Reg(Bool()) init False     // fired, waiting for dstDone
    val fireReg = False

    prev := reqSync
    when(armed) {
      val rising = reqSync && !prev
      when(rising && !ackReg && !pending) {
        fireReg := True
        if (withDstDone) pending := True else ackReg := True
      }
    }
    // stale request: the synced level is already high one cycle before arming (reqSync is valid from
    // seedCnt==2 on): acknowledge WITHOUT fire, flag dropped; `prev` tracks it so no edge fires at arming.
    when(seedCnt === 2 && reqSync && !ackReg) {
      ackReg  := True
      dropReg := True
    }
    when(pending && io.dstDone) {
      pending := False
      ackReg  := True
    }
    when(!reqSync && ackReg) {
      ackReg  := False
      dropReg := False
    }
    ackLevel   := ackReg
    dropLevel  := dropReg
    io.fire    := fireReg
    io.dropped := dropReg
  }
}
