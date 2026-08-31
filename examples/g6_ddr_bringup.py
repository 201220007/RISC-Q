#!/usr/bin/env python3
"""G6 -- readout-to-DDR uplink bring-up on the board (qubic3 C1).

Runs ON the board (veneno), where pynq and the bitstream live. Nothing here needs RF: the stimulus is
the uplink's built-in test injector, so this is the same experiment G4a runs in simulation and the
numbers are directly comparable.

    python3 g6_ddr_bringup.py --xsa ~/riscq-bits/PulseTableSoc.xsa \
                              --config ~/riscq-bits/zcu216-14q-ddr.json \
                              --wr-base 0x00100000 --shots 12

Order is deliberate and matches plan/G6_BOARD_PLAN.md:

  0. read the HOST-domain DDR status register FIRST. It is the one thing that still answers when the
     whole ui_clk side is dead, so a failed DDR4 calibration reports itself here instead of hanging an
     AXI transaction. Without this step a calibration failure looks exactly like a wedged uplink.
  1. geometry + a clean status.
  2. injector self-test: prepare -> N x inject -> flush -> drain, byte-exact against what was injected.
  3. a second run at a different base, to prove run-to-run isolation on hardware.

Every step prints what it verified. On any failure it stops and dumps STATUS + DIAG decoded, because on
hardware the next thing anyone asks is "which bit".
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software"))

from riscq.ddr import DdrMap, DdrReadout, DdrUplinkError, status_str   # noqa: E402
from riscq.ddr_regs import DIAG_NAMES                                  # noqa: E402
from riscq.map import SocMap, SocParams                                # noqa: E402


def _hex(s):
    return int(s, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xsa", required=True, help="the hardware platform to load")
    ap.add_argument("--config", required=True, help="the SocParams JSON that MATCHES the bitstream")
    ap.add_argument("--wr-base", type=_hex, default=0x0010_0000,
                    help="DDR address for the run (512-B aligned)")
    ap.add_argument("--shots", type=int, default=12, help="injected results (must be > 0)")
    ap.add_argument("--core", type=int, default=None,
                    help="inject all shots on this core (default: round-robin over all)")
    ap.add_argument("--no-download", action="store_true", help="reuse the already-loaded bitstream")
    ap.add_argument("--calib-timeout", type=float, default=5.0,
                    help="seconds to wait for the MIG to report calib_done (it finishes after the PL "
                         "is configured, so this must be polled, not sampled once)")
    args = ap.parse_args()

    if args.shots <= 0:
        sys.exit("[G6] --shots must be > 0: a run that injects nothing would pass vacuously")
    params = SocParams.load(args.config)
    if not params.ddr_readout:
        sys.exit(f"{args.config} has ddr_readout=false -- that config does not match a DDR bitstream")
    soc_map = SocMap(params)
    print(f"[G6] config {params.name}: {params.qubit_num} qubits, ddr_readout=True")

    # ---- load the overlay -------------------------------------------------------------
    from riscq.board.pynq_driver import PynqDriver
    from riscq.board.ddr_board import DdrBoard

    # NO RF STATE IS WRITTEN. This test's stimulus is the uplink's own injector, so MTS, the Nyquist
    # zones and the DAC VOP are all skipped: veneno is shared with another project, and every RF
    # register left untouched is one less thing to restore -- and one less unaudited hardware wait
    # standing between here and the DDR status read below, which is the step that turns a failed
    # calibration into a message instead of a hung AXI transaction.
    # The LMK/LMX reference clocks ARE programmed: dspClk comes from the board's LVDS clocks, so the
    # SoC does not run without them. lmk_freq 500.25 with lmx defaulted is byte-for-byte what the
    # board's own QubiC loader does (rfsoc/pl_interface.py refclks(lmk, lmx=None)), so this leaves the
    # clock chips in exactly the state that board already runs in.
    no_rf = {"mts": None, "dac_nyquist": None, "adc_nyquist": None, "dac_current": {}}
    pynq_drv = PynqDriver(args.xsa, args.config, board=no_rf, download=not args.no_download)
    board = DdrBoard(soc=pynq_drv, m=DdrMap())
    print("[G6] overlay loaded (RF init skipped: no MTS, no Nyquist, no VOP)")

    # ---- step 0: is the DDR side even alive? ------------------------------------------
    # This read goes to the HOST clock domain, whose reset tree is independent of the DDR one.
    # POLL, do not sample once. Measured on veneno 2026-08-26: the first read after the overlay
    # download returned 0xca1b0002 -- magic right, ui_clk reset already released, calib_done still 0 --
    # and a re-read moments later returned 0xca1b0003. DDR4 calibration on real hardware finishes some
    # time AFTER the PL is configured, so a single read races it and reports a working MIG as dead.
    # The failure path below is unchanged: if it never completes, this still stops before 0x9000_0000.
    def _status():
        raw = board.read32(soc_map.ddr_status())
        return (raw, raw >> 16, bool(raw >> soc_map.DDR_STATUS_CALIB & 1),
                bool(raw >> soc_map.DDR_STATUS_UI_RST_OK & 1))

    t0 = time.monotonic()
    raw, magic, calib, rst_ok = _status()
    if magic != soc_map.DDR_STATUS_MAGIC:
        print(f"[G6] DDR status 0x{raw:08x}: magic=0x{magic:04x}")
        sys.exit(f"[G6] FAIL: magic 0x{magic:04x} != 0x{soc_map.DDR_STATUS_MAGIC:04x}. The loaded "
                 f"bitstream does not match {args.config} (or predates the status register).")
    while not (calib and rst_ok) and time.monotonic() - t0 < args.calib_timeout:
        time.sleep(0.02)
        raw, magic, calib, rst_ok = _status()
    dt = time.monotonic() - t0
    print(f"[G6] DDR status 0x{raw:08x}: magic=0x{magic:04x} calib_done={calib} "
          f"ui_reset_released={rst_ok} (after {dt * 1e3:.0f} ms of polling)")
    if not calib:
        sys.exit("[G6] FAIL: the MIG never finished DDR4 calibration. Check the DDR4 part/pinout and the "
                 "board -- do NOT touch the uplink registers, they will not answer.")
    if not rst_ok:
        sys.exit("[G6] FAIL: the MIG calibrated but the ui_clk reset tree never released. Check psr_ddr "
                 "and the reset stretcher.")

    ddr = DdrReadout(board, soc_map=soc_map)

    # ---- step 1: geometry ------------------------------------------------------------
    print(f"[G6] NUM_CH={ddr.num_ch} fifo={ddr.fifo_depth} skid={ddr.skid_depth} "
          f"cbuf_addr_width={ddr.cbuf_addr_width} bank={ddr.bank_bytes} B flush_quiet={ddr.flush_quiet}")
    if ddr.num_ch != params.qubit_num:
        sys.exit(f"[G6] FAIL: hardware reports {ddr.num_ch} channels, the config says "
                 f"{params.qubit_num} -- bitstream/config mismatch")

    def dump(why):
        print(f"[G6] {why}")
        print(f"[G6]   STATUS: {status_str(ddr.status())}")
        d = ddr.diag()
        print("[G6]   DIAG:   " + " ".join(f"{n}={int(d[n])}" for n in DIAG_NAMES))
        print(f"[G6]   accepted={ddr.accepted()} rejected={ddr.rejected()}")

    # ---- step 2: injector self-test ---------------------------------------------------
    def one_run(base, shots, tag, salt=0):
        """`salt` makes run 2's payload DIFFERENT from run 1's. Without it, a stale DMA buffer holding
        run 1's bytes would pass run 2 byte-exactly and prove nothing about isolation (r27-#9)."""
        expect = {}
        for k in range(shots):
            core = args.core if args.core is not None else k % ddr.num_ch
            real = 0x0011_0000 + (k << 8) + salt
            imag = 0x0022_0000 + (k << 8) + salt
            expect.setdefault(core, []).append((real, imag))
        counts = {c: len(v) for c, v in expect.items()}
        # r27-#10: expectations first, so prepare() can preflight the footprint against the ring limit
        ddr.prepare(base, expected=counts)
        for core, vals in expect.items():
            for real, imag in vals:
                ddr.inject(core, real, imag)
        st = ddr.flush()
        got = ddr.drain(base, counts, status=st)
        for core, vals in expect.items():
            re_got, im_got = got[core]
            re_want = [(v[0] >> 4) << 4 for v in vals]
            im_want = [(v[1] >> 4) << 4 for v in vals]
            if list(re_got) != re_want or list(im_got) != im_want:
                dump(f"{tag}: core {core} MISMATCH")
                print(f"[G6]   got  real={list(re_got)} imag={list(im_got)}")
                print(f"[G6]   want real={re_want} imag={im_want}")
                sys.exit(1)
        used = ddr.max_bytes(counts)
        print(f"[G6] {tag}: {shots} injected results returned BYTE-EXACT from DDR (footprint {used} B)")
        return used

    # r27-#11: ANY failure gets the diagnostic and the cleanup, not just DdrUplinkError -- a DMA
    # RuntimeError, a bad --core, an allocation failure or a fault inside dump() must not bypass them.
    ok = False
    try:
        used = one_run(args.wr_base, args.shots, "run 1")
        # r27-#10: the second base clears run 1's FULL footprint, not a nominal 4 KiB
        second = args.wr_base + max(used, ddr.bank_bytes)
        # r27-#9: and with a different payload, so a stale buffer cannot pass
        one_run(second, args.shots, f"run 2 (base 0x{second:x}, salted)", salt=0x37)
        ok = True
    except Exception as e:                                   # noqa: BLE001 - this is the operator's net
        print(f"[G6] FAIL: {type(e).__name__}: {e}")
        try:
            dump("state at failure")
        except Exception as e2:                              # noqa: BLE001
            print(f"[G6]   (the diagnostic itself failed: {type(e2).__name__}: {e2} -- if the ui_clk side "
                  f"is dead, re-read the host-domain DDR status)")
    finally:
        try:
            board.close()
        except Exception as e3:                              # noqa: BLE001
            print(f"[G6]   (board.close() failed: {type(e3).__name__}: {e3})")
    if not ok:
        sys.exit(1)

    print("[G6] PASS: DDR calibrated, uplink alive, injector self-test byte-exact across two runs")
    print("[G6] next: the RF path over the BPF loopback with lbl-readout-emu-ddr.json (see "
          "plan/G6_BOARD_PLAN.md step G6.2)")


if __name__ == "__main__":
    main()
