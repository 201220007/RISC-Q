#!/usr/bin/env python3
"""C1 on hardware with REAL readout results: every core executes demod (= readout) windows, the
DSP-produced results flow through the uplink into PL-DDR4, and the host drains them via DMA.

The ADC content does not matter (the ReadoutDecoder integrates on the demod carrier's `valid`, not on
signal), so this needs no RF loopback -- but the RF converters ARE brought up the production way
(MTS + Nyquist zones + DAC VOP, QubiC's values on veneno) so the run is representative.

Two stages, because the RISC-V toolchain lives on the host and pynq lives on the board:

  host:   python examples/c1_readout_ddr.py compile --config software/configs/zcu216-14q-ddr.json \
              --out progs.pkl
  board:  sudo bash -lc 'PYTHONPATH=software /usr/local/share/pynq-venv/bin/python3 \
              examples/c1_readout_ddr.py run --bit bits/PulseTableSoc.bit \
              --config software/configs/zcu216-14q-ddr.json --progs progs.pkl \
              --shots 1000 --gaps 512,256,128,96,64,48'

Kernel: `paced` plays one demod window per shot on a fixed grid (period = gap batches) and reads the
result every shot (the halting read_res keeps the core in step with the decoder). Per-core results
per run = shots; the uplink must deliver exactly 14 x shots tagged words with REJECTED = 0 and no
OVERFLOW; `drain()` certifies all of that. A gap below the poller's fair share (3 x 14 = 42 cycles,
documented safe spacing 64) is EXPECTED to overflow -- that run is reported, not failed.
"""
from __future__ import annotations

import argparse
import json
import logging
import pickle
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software"))

from riscq.lang import Array, ParamTable, compile_kernel, kernel  # noqa: E402
from riscq.map import LEAD, READOUT_LEAD, SocMap, SocParams, pack16  # noqa: E402

TAG = "[C1]"
RO_DUR = 40          # demod window, batches (one batch = one dspClk tick, 2 ns at 500 MHz)
DEMOD_FREQ_HZ = 50e6


@kernel
def k_paced(demod: ParamTable, out: Array, code: int, gap: int, k: int):
    """One demod window per shot on a fixed grid; the halting read_res keeps the core in step."""
    init_pulse_params(demod.pulses)   # noqa: F821
    set_freq(demod, code)             # noqa: F821  ADC-rate demod code; content irrelevant here
    t = now() + LEAD                  # noqa: F821
    for s in range(k):
        play(demod, demod["sq"], t)   # noqa: F821  firing the demod IS the readout
        wait_until(t + READOUT_LEAD)  # noqa: F821
        out[0] = read_res()           # noqa: F821  halts until this shot's integral settled
        t = t + gap


BURST_AHEAD = LEAD + 8   # the CPU stays this many batches ahead of the schedule (see below)
BURST_GAP = 2            # idle batches between windows: the decoder emits a result on the carrier
                         # valid FALLING edge, so contiguous windows merge into ONE (measured 2026-09-11:
                         # period == dur gave exactly 1 result per core)
BURST_DURS = (64, 48, 40, 36, 32)   # period = dur + BURST_GAP; aggregate 14/period results per cycle


@kernel
def k_burst(demod: ParamTable, out: Array, code: int, dur: int, k: int):
    """Densest window train: period = window + BURST_GAP, no per-shot read. A fire into a FULL timed
    queue (depth 4) is dropped, and a fire less than ~LEAD ahead lands with stale params, so the CPU
    is throttled to BURST_AHEAD batches ahead of each window's start: at most 1 + BURST_AHEAD/period
    windows are ever pending. One halting read at the end lands the last result before DONE."""
    init_pulse_params(demod.pulses)   # noqa: F821
    set_freq(demod, code)             # noqa: F821
    t = now() + LEAD + LEAD           # noqa: F821
    for s in range(k):
        wait_until(t - BURST_AHEAD)   # noqa: F821
        play(demod, demod["sq"], t)   # noqa: F821
        t = t + dur + BURST_GAP
    wait_until(t + READOUT_LEAD)      # noqa: F821
    out[0] = read_res()               # noqa: F821


def _hex(s):
    return int(s, 0)


# ── host: compile ─────────────────────────────────────────────────────────────────────────
def cmd_compile(args):
    from riscq import run as rq
    from riscq.pulses import Pulse, envelopes, units

    m = SocMap(SocParams.load(args.config))
    code = pack16(units.demod_freq_to_code(DEMOD_FREQ_HZ, m.params))

    def table(dur):
        return dict(demod=ParamTable(2, 0.0, {"sq": Pulse(envelopes.square(dur), amp=1.0)}))

    progs = {"paced": (RO_DUR, compile_kernel(k_paced, m, tables=table(RO_DUR), out=Array(1), code=code))}
    for dur in BURST_DURS:
        progs[f"burst{dur}"] = (dur, compile_kernel(k_burst, m, tables=table(dur), out=Array(1),
                                                    code=code, dur=dur))
    with open(args.out, "wb") as f:
        pickle.dump({"config": Path(args.config).name,
                     "progs": {n: {"dur": d, "wire": rq._prog_to_wire(p)} for n, (d, p) in progs.items()}}, f)
    for n, (d, p) in progs.items():
        print(f"{TAG} compiled {n}: dur {d}, image {len(p.image.data)} B, runtime params "
              f"{[k for k, v in p.params.items() if v is None]}")
    print(f"{TAG} -> {args.out}")


# ── board: run ────────────────────────────────────────────────────────────────────────────
def cmd_run(args):
    from riscq import run as rq
    from riscq.board.ddr_board import DdrBoard
    from riscq.board.pynq_driver import PynqDriver
    from riscq.ddr import DdrMap, DdrReadout, DdrUplinkError, status_str
    from riscq.ddr_regs import DIAG_NAMES, OVERFLOW

    logging.basicConfig(level=logging.INFO, format="%(name)s %(levelname)s %(message)s")
    logging.getLogger("riscq.board.pynq_driver").setLevel(logging.DEBUG)   # MTS latencies

    params = SocParams.load(args.config)
    if not params.ddr_readout:
        sys.exit(f"{args.config} has ddr_readout=false -- not a DDR bitstream config")
    m = SocMap(params)
    with open(args.progs, "rb") as f:
        blob = pickle.load(f)
    if blob["config"] != Path(args.config).name:
        sys.exit(f"progs were compiled for {blob['config']}, run asked for {Path(args.config).name}")
    kernels = args.kernels.split(",")
    for kn in kernels:
        if kn not in blob["progs"]:
            sys.exit(f"kernel {kn!r} not in {args.progs}: have {sorted(blob['progs'])}")
    shots = args.shots
    gaps = [int(g) for g in args.gaps.split(",")]
    cores = list(range(params.qubit_num))
    counts = {c: shots for c in cores}
    # one run per (kernel, period): paced sweeps --gaps, a burst kernel's period is its window
    runs_todo = []
    for kn in kernels:
        dur = blob["progs"][kn]["dur"]
        if kn == "paced":
            runs_todo += [(kn, dur, g, {"gap": g, "k": shots}) for g in gaps]
        else:
            runs_todo.append((kn, dur, dur + BURST_GAP, {"k": shots}))
    print(f"{TAG} {params.name}: {params.qubit_num} qubits, {shots} shots/core, runs "
          f"{[(kn, per) for kn, _, per, _ in runs_todo]}")

    # ---- RF bring-up: refclks -> overlay -> MTS -> Nyquist -> VOP (PynqDriver.__init__) ----------
    board_cfg = json.load(open(args.board)) if args.board else None
    print(f"{TAG} board.json: {board_cfg if board_cfg else 'defaults (MTS 260/60, DAC z2, ADC z1)'}")
    t0 = time.monotonic()
    pynq_drv = PynqDriver(args.bit, args.config, board=board_cfg, download=not args.no_download)
    print(f"{TAG} overlay + RF init done in {time.monotonic() - t0:.1f} s; "
          f"mts_result={pynq_drv.mts_result} (0 = every tile at target latency)")
    if pynq_drv.mts_result != 0:
        dac_lat, adc_lat = pynq_drv._mts_latencies()
        print(f"{TAG} MTS NOT at target: measured dac={dac_lat} adc={adc_lat}")
        if not args.allow_mts_miss:
            sys.exit(f"{TAG} FAIL: MTS did not converge (use --allow-mts-miss to run anyway)")
    _pl0_report(pynq_drv)
    board = DdrBoard(soc=pynq_drv, m=DdrMap(), cma_bytes=args.cma_mib << 20)

    # ---- DDR side alive? poll the host-domain status (calibration finishes ~100 ms after config) --
    ddr = DdrReadout(board, soc_map=m)
    t0 = time.monotonic()
    ddr.wait_ddr_ready(timeout=args.calib_timeout)
    raw = ddr.ddr_status()
    print(f"{TAG} DDR status {raw} after {(time.monotonic() - t0) * 1e3:.0f} ms of polling")
    print(f"{TAG} NUM_CH={ddr.num_ch} fifo={ddr.fifo_depth} skid={ddr.skid_depth} "
          f"bank={ddr.bank_bytes} B flush_quiet={ddr.flush_quiet}")
    if ddr.num_ch != params.qubit_num:
        sys.exit(f"{TAG} FAIL: hardware NUM_CH={ddr.num_ch} != config {params.qubit_num}")

    def dump(why):
        print(f"{TAG} {why}")
        print(f"{TAG}   STATUS: {status_str(ddr.status())}")
        d = ddr.diag()
        print(f"{TAG}   DIAG:   " + " ".join(f"{n}={int(d[n])}" for n in DIAG_NAMES))
        print(f"{TAG}   accepted={ddr.accepted()} rejected={ddr.rejected()} "
              f"OVERFLOW=0x{ddr.rd(OVERFLOW):x}")

    results = []
    base = args.wr_base
    ok_all = True
    loaded = None
    try:
        for kn, dur, gap, rparams in runs_todo:
            if loaded != kn:                                       # same image on every core, reset held
                progs = {c: rq._prog_from_wire(blob["progs"][kn]["wire"]) for c in cores}
                t0 = time.monotonic()
                rq.setup(pynq_drv, m, progs)
                print(f"{TAG} {kn}: 14 programs loaded in {time.monotonic() - t0:.2f} s")
                loaded = kn
            row = {"kernel": kn, "dur": dur, "gap": gap, "shots": shots,
                   "offered_rate_per_cycle": round(len(cores) / gap, 3)}
            t_run = None
            try:
                ddr.prepare(base, expected=counts)                 # admission OPEN before any window
                t0 = time.monotonic()
                rq.rerun(pynq_drv, m, progs, params={c: rparams for c in cores},
                         timeout=args.poll_timeout)
                t_run = time.monotonic() - t0
                row["t_run_s"] = round(t_run, 4)
                row["schedule_s"] = shots * gap * 2e-9            # what the grid alone would take
                st = ddr.flush()
                got = ddr.drain(base, counts, status=st)         # certifies counts/tags/no-overflow
                n = sum(len(v[0]) for v in got.values())
                row.update(result="PASS", t_run_s=round(t_run, 4), words=n,
                           bytes=n * 8, accepted=ddr.accepted())
                # a value sample per group (cores 0-6 share ADC 0, 7-13 share ADC 4)
                for c in (0, 7, 13):
                    re, im = got[c]
                    row[f"core{c}_first"] = (int(re[0]), int(im[0]))
                    row[f"core{c}_std"] = (float(re.std()), float(im.std()))
                print(f"{TAG} {kn} period={gap} ({len(cores) / gap:.3f} results/cycle offered): PASS  "
                      f"{n} results ({n * 8} B) in {t_run * 1e3:.1f} ms program time; "
                      f"per-core accepted={ddr.accepted()}")
            except DdrUplinkError as e:
                row.update(result="UPLINK-REJECT", error=str(e), accepted=ddr.accepted(),
                           rejected=ddr.rejected(), status=status_str(ddr.status()),
                           overflow=ddr.rd(OVERFLOW))
                print(f"{TAG} {kn} period={gap}: uplink refused the run: {e}")
                dump("state after refusal")
                if gap >= args.min_pass_gap:
                    ok_all = False
            results.append(row)
            base += max(ddr.max_bytes(counts), ddr.bank_bytes)     # fresh footprint every run
    except Exception as e:                                          # noqa: BLE001 operator's net
        ok_all = False
        print(f"{TAG} FAIL: {type(e).__name__}: {e}")
        try:
            dump("state at failure")
        except Exception as e2:                                     # noqa: BLE001
            print(f"{TAG}   (diagnostic failed: {type(e2).__name__}: {e2})")
    finally:
        try:
            board.close()
        except Exception as e3:                                     # noqa: BLE001
            print(f"{TAG}   (board.close() failed: {type(e3).__name__}: {e3})")
        if args.report:
            with open(args.report, "w") as f:
                json.dump({"config": params.name, "shots": shots, "mts_result": pynq_drv.mts_result,
                           "board": board_cfg, "runs": results}, f, indent=1, default=str)
            print(f"{TAG} report -> {args.report}")
    print(f"{TAG} {'PASS' if ok_all else 'FAIL'}: every gap >= {args.min_pass_gap} delivered "
          f"14 x {shots} DSP-produced results through PL-DDR4" if ok_all else f"{TAG} FAIL")
    sys.exit(0 if ok_all else 1)


def _pl0_report(drv):
    """Read CRL_APB.PL0_REF_CTRL (0xFF5E00C0) -- the deployment hazard QubiC guards against."""
    try:
        import mmap
        import os
        import struct
        f = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
        try:
            mm = mmap.mmap(f, 4096, mmap.MAP_SHARED, mmap.PROT_READ, offset=0xFF5E0000)
            v = struct.unpack("<I", mm[0xC0:0xC4])[0]
            mm.close()
        finally:
            os.close(f)
        src = v & 7
        div0 = (v >> 8) & 0x3F
        div1 = (v >> 16) & 0x3F
        srcname = {0: "IOPLL", 2: "RPLL", 3: "DPLL"}.get(src, str(src))
        mhz = (1499.985 if src == 0 else 1500.0) / max(div0, 1) / max(div1, 1)
        print(f"{TAG} PL0_REF_CTRL=0x{v:08x}: src={srcname} div={div0}x{div1} -> ~{mhz:.1f} MHz")
    except Exception as e:                                          # noqa: BLE001
        print(f"{TAG} PL0 read skipped: {type(e).__name__}: {e}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("compile")
    c.add_argument("--config", required=True)
    c.add_argument("--out", default="progs.pkl")
    c.set_defaults(fn=cmd_compile)
    r = sub.add_parser("run")
    r.add_argument("--bit", required=True, help=".bit with its .hwh alongside")
    r.add_argument("--config", required=True)
    r.add_argument("--progs", required=True)
    r.add_argument("--board", default=None, help="board.json (MTS/Nyquist/VOP); default = driver defaults")
    r.add_argument("--kernels", default="paced", help="comma list: paced, burst64, burst48, ...")
    r.add_argument("--shots", type=int, default=1000)
    r.add_argument("--gaps", default="512,256,128,96,64,48", help="per-shot period in batches, one run each")
    r.add_argument("--min-pass-gap", type=int, default=64, help="gaps below this may overflow without failing")
    r.add_argument("--wr-base", type=_hex, default=0x0010_0000)
    r.add_argument("--cma-mib", type=int, default=16)
    r.add_argument("--calib-timeout", type=float, default=5.0)
    r.add_argument("--poll-timeout", type=int, default=2_000_000)
    r.add_argument("--no-download", action="store_true")
    r.add_argument("--allow-mts-miss", action="store_true")
    r.add_argument("--report", default=None)
    r.set_defaults(fn=cmd_run)
    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
