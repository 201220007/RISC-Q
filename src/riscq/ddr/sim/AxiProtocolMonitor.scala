package riscq.ddr.sim

import spinal.core._
import spinal.core.sim._
import spinal.lib._
import spinal.lib.bus.amba4.axi.Axi4
import scala.collection.mutable

/**
 * Generic valid/ready protocol monitor (AXI A3.2.1, also AXI-Stream): on every channel, once VALID is high and
 * READY low, VALID must stay high and the payload must not change until the handshake. Checked every cycle of
 * `cd`, except across a cycle in which `inReset` is true (a reset may drop VALID). Used by G2 and the P3a CDC sim
 * on the uplink's DDR master (AW, W, B, AR, R) and on its AXIS drain (P3a r2).
 */
class ValidReadyMonitor(val name: String, valid: Bool, ready: Bool, payload: Seq[BaseType], cd: ClockDomain,
                        inReset: () => Boolean) {
  val violations = mutable.ArrayBuffer[String]()
  var stalls = 0L
  private def sample(): Seq[BigInt] = payload.map {
    case b: Bool => if (b.toBoolean) BigInt(1) else BigInt(0)
    case b: BitVector => b.toBigInt
    case x => throw new Exception(s"unsupported payload $x")
  }
  fork {
    var prev: Option[(Boolean, Boolean, Seq[BigInt])] = None
    var prevRst = true
    while (true) {
      cd.waitRisingEdge()
      val rst = inReset()
      val cur = (valid.toBoolean, ready.toBoolean, sample())
      prev match {
        case Some((true, false, pl)) if !rst && !prevRst =>
          stalls += 1
          if (!cur._1) violations += s"$name: VALID dropped while stalled at t=${simTime()}"
          else if (cur._3 != pl) {
            val idx = pl.indices.filter(i => pl(i) != cur._3(i)).map(i => payload(i).getName())
            violations += s"$name: payload changed while VALID && !READY at t=${simTime()} (${idx.mkString(",")})"
          }
        case _ =>
      }
      prev = Some(cur); prevRst = rst
    }
  }
}

object AxiProtocolMonitor {
  /** One monitor per channel of `axi` plus the AXIS `rd` stream. */
  def apply(axi: Axi4, rd: Stream[Fragment[Bits]], cd: ClockDomain, inReset: () => Boolean): Seq[ValidReadyMonitor] = {
    def pl(d: Data): Seq[BaseType] = d.flatten
    Seq(
      new ValidReadyMonitor("AW", axi.aw.valid, axi.aw.ready, pl(axi.aw.payload), cd, inReset),
      new ValidReadyMonitor("W",  axi.w.valid,  axi.w.ready,  pl(axi.w.payload),  cd, inReset),
      new ValidReadyMonitor("B",  axi.b.valid,  axi.b.ready,  pl(axi.b.payload),  cd, inReset),
      new ValidReadyMonitor("AR", axi.ar.valid, axi.ar.ready, pl(axi.ar.payload), cd, inReset),
      new ValidReadyMonitor("R",  axi.r.valid,  axi.r.ready,  pl(axi.r.payload),  cd, inReset),
      new ValidReadyMonitor("AXIS", rd.valid, rd.ready, pl(rd.payload), cd, inReset))
  }
  def check(ms: Seq[ValidReadyMonitor], what: String): Unit = {
    val v = ms.flatMap(_.violations)
    assert(v.isEmpty, s"$what: ${v.size} AXI protocol violation(s), first: ${v.take(3).mkString(" | ")}")
  }
  def summary(ms: Seq[ValidReadyMonitor]): String = ms.map(m => s"${m.name}:${m.stalls}").mkString(" ")
}
