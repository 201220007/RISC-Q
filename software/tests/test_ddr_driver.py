"""Behavioural tests for `riscq.ddr.DdrReadout` against a fake register/DMA model.

These do not simulate the RTL (that is G2's job); they pin the HOST-side contract: every gate in
`drain()` must actually reject, and a clean run must decode byte-exactly. The fake model implements
the register semantics the RTL guarantees (sticky W1C, snapshot counters, final_addr).
"""
import time

import numpy as np
import pytest

from riscq import ddr_regs as R
from riscq.ddr import DdrMap, DdrReadout, DdrUplinkError, parse_words


def tag_word(tag, real, imag):
    return (tag << 56) | ((real & 0xFFFFFFFF) >> 4 << 28) | ((imag & 0xFFFFFFFF) >> 4)


class FakeUplink:
    """Minimal model of the uplink register file + DDR + DMA."""

    def __init__(self, num_ch=4, base=0x1000):
        self.num_ch = num_ch
        self.base = base
        self.regs = {R.NUM_CH: num_ch, R.GEOMETRY: (8 << 24) | (4 << 16) | (8 << 8) | 16}
        self.sticky = 0
        self.accepted = [0] * num_ch
        self.rejected = [0] * num_ch
        self.run_base = 0
        self.final_addr = 0
        self.mem = bytearray(1 << 20)
        self.status_extra = 0

    # -- register surface --------------------------------------------------------------
    def read32(self, addr):
        off = addr - DdrMap().ctrl_base
        if off == R.STATUS:
            return self.sticky | self.status_extra
        if off == R.RUN_BASE:
            return self.run_base
        if off == R.FINAL_ADDR:
            return self.final_addr
        if R.ACCEPTED <= off < R.ACCEPTED + 4 * 32:
            return self.accepted[(off - R.ACCEPTED) // 4]
        if R.REJECTED <= off < R.REJECTED + 4 * 32:
            return self.rejected[(off - R.REJECTED) // 4]
        return self.regs.get(off, 0)

    def write32(self, addr, val):
        off = addr - DdrMap().ctrl_base
        if off == R.STATUS:
            self.sticky &= ~(val & R.STICKY_MASK)
            return
        self.regs[off] = val

    # -- DMA -------------------------------------------------------------------------
    def dma_recv_prepare(self, nbytes):
        return ("buf", nbytes)

    def dma_recv_wait(self, buf, nbytes):
        base = self.regs.get(R.RD_BASE, 0)
        return bytes(self.mem[base:base + nbytes])

    # -- helpers to stage a run ------------------------------------------------------
    def stage(self, words_per_core, pad_words=0):
        """Write a finished, valid run into the model."""
        self.run_base = self.base
        self.accepted = [len(words_per_core.get(c, [])) for c in range(self.num_ch)]
        flat = []
        for c in range(self.num_ch):
            for (re, im) in words_per_core.get(c, []):
                flat.append(tag_word(c, re, im))
        total = len(flat)
        beats = -(-(total + pad_words) // 4)
        for i, w in enumerate(flat + [0] * pad_words):
            self.mem[self.base + 8 * i: self.base + 8 * i + 8] = int(w).to_bytes(8, "little")
        self.final_addr = self.base + beats * R.BEAT_BYTES
        self.sticky = 1 << R.S_WRITE_DONE
        return {c: len(v) for c, v in words_per_core.items()}


@pytest.fixture
def fake():
    return FakeUplink()


@pytest.fixture
def drv(fake):
    # r21-#4: `soc_map=None` is no longer an implicit bypass -- most tests here exercise the register
    # protocol, not the readiness gate, so they opt out explicitly. The gate's own tests build their
    # own driver with a real SocMap (see the readiness section at the end).
    return DdrReadout(fake, legacy_no_ddr_status=True)


def _run(fake, per_core, pad=0):
    exp = fake.stage(per_core, pad_words=pad)
    return exp


def test_geometry_is_read_from_hardware(drv):
    assert drv.num_ch == 4 and drv.fifo_depth == 16 and drv.skid_depth == 8
    assert drv.cbuf_addr_width == 4 and drv.flush_quiet == 8
    assert drv.bank_bytes == 16 * 32


def test_clean_run_decodes_byte_exactly(fake, drv):
    per_core = {0: [(0x11110000, 0x22220000), (-0x30000, 0x40000)], 2: [(0x7FFFFFF0, -0x10)]}
    exp = _run(fake, per_core, pad=1)
    out = drv.drain(fake.base, exp)
    assert set(out) == {0, 2}
    for c, vals in per_core.items():
        real, imag = out[c]
        assert list(real) == [(v[0] >> 4) << 4 for v in vals]
        assert list(imag) == [(v[1] >> 4) << 4 for v in vals]


# r11-#7: parametrized over R.FATAL_BITS itself, not a hand-copied subset, so a bit added to the
# fatal list can never silently go untested (the previous list covered 8 of the 13).
@pytest.mark.parametrize("bit", R.FATAL_BITS, ids=[R.STATUS_NAMES[b] for b in R.FATAL_BITS])
def test_every_fatal_bit_rejects_the_run(fake, drv, bit):
    exp = _run(fake, {0: [(1, 2)]})
    fake.sticky |= 1 << bit
    with pytest.raises(DdrUplinkError, match="run invalid"):
        drv.drain(fake.base, exp)


def test_missing_write_done_rejects(fake, drv):
    exp = _run(fake, {0: [(1, 2)]})
    fake.sticky &= ~(1 << R.S_WRITE_DONE)
    with pytest.raises(DdrUplinkError, match="write_done"):
        drv.drain(fake.base, exp)


def test_error_raised_during_the_drain_rejects(fake, drv):
    """The status is clean before the DMA and dirty after — the run must NOT be certified (r10-#5)."""
    exp = _run(fake, {0: [(1, 2)]})
    orig = fake.dma_recv_wait

    def dirty(buf, nbytes):
        fake.sticky |= 1 << R.S_RRESP_ERR
        return orig(buf, nbytes)

    fake.dma_recv_wait = dirty
    with pytest.raises(DdrUplinkError, match="DURING the drain"):
        drv.drain(fake.base, exp)


def test_write_done_vanishing_during_the_drain_rejects(fake, drv):
    exp = _run(fake, {0: [(1, 2)]})
    orig = fake.dma_recv_wait

    def reset(buf, nbytes):
        fake.sticky = 0            # a DDR-domain reset clears the whole register file
        return orig(buf, nbytes)

    fake.dma_recv_wait = reset
    with pytest.raises(DdrUplinkError, match="vanished"):
        drv.drain(fake.base, exp)


def test_run_base_mutating_during_the_drain_rejects(fake, drv):
    """A concurrent `base_reset` (another thread starting the next run) moves `run_base` out from under
    the DMA: the bytes already copied belong to a run that no longer exists. (r11-#7)"""
    exp = _run(fake, {0: [(1, 2)]})
    orig = fake.dma_recv_wait

    def restart(buf, nbytes):
        out = orig(buf, nbytes)
        fake.run_base = fake.base + 0x1000     # the next run took the pointer
        return out

    fake.dma_recv_wait = restart
    with pytest.raises(DdrUplinkError, match="run_base changed during the drain"):
        drv.drain(fake.base, exp)


def test_run_base_wrong_before_the_drain_rejects(fake, drv):
    """The pre-drain gate: the hardware never latched the base this program asked for."""
    exp = _run(fake, {0: [(1, 2)]})
    fake.run_base = fake.base + 0x40
    with pytest.raises(DdrUplinkError, match="run_base 0x"):
        drv.drain(fake.base, exp)


def test_accepted_mismatch_rejects(fake, drv):
    exp = _run(fake, {0: [(1, 2), (3, 4)]})
    exp[0] = 3                     # the program expected one more shot than the hardware took
    with pytest.raises(DdrUplinkError, match="accepted 2 results, program expected 3"):
        drv.drain(fake.base, exp)


def test_nonzero_rejected_rejects(fake, drv):
    exp = _run(fake, {0: [(1, 2)]})
    fake.rejected[1] = 5
    with pytest.raises(DdrUplinkError, match="rejected per core"):
        drv.drain(fake.base, exp)


def test_expectation_for_a_nonexistent_core_rejects(fake, drv):
    exp = _run(fake, {0: [(1, 2)]})
    exp[fake.num_ch] = 1           # a core this build does not have (r09-#7)
    with pytest.raises(DdrUplinkError, match="names cores"):
        drv.drain(fake.base, exp)


def test_wrong_run_base_rejects(fake, drv):
    exp = _run(fake, {0: [(1, 2)]})
    with pytest.raises(DdrUplinkError, match="run_base"):
        drv.drain(fake.base + 0x200, exp)


@pytest.mark.parametrize("offset", [8, 16, 24])
def test_unaligned_final_addr_rejects(fake, drv, offset):
    """final_addr-run_base must be a whole number of 256-bit beats (r10-#4)."""
    exp = _run(fake, {0: [(1, 2)]})
    fake.final_addr = fake.base + offset
    with pytest.raises(DdrUplinkError, match="not a multiple"):
        drv.drain(fake.base, exp)


def test_pad_out_of_range_rejects(fake, drv):
    exp = _run(fake, {0: [(1, 2)]})
    fake.final_addr = fake.base + 4 * R.BEAT_BYTES   # 16 words for 1 accepted result
    with pytest.raises(DdrUplinkError, match="pad"):
        drv.drain(fake.base, exp)


def test_empty_run_still_checks_padding(fake, drv):
    """An empty run with a non-empty final_addr is NOT certifiable (r09-#7 / r10-#4)."""
    exp = _run(fake, {})
    fake.final_addr = fake.base + R.BEAT_BYTES       # the writer emitted a beat we cannot account for
    with pytest.raises(DdrUplinkError, match="pad"):
        drv.drain(fake.base, exp)


def test_empty_run_with_empty_final_addr_is_ok(fake, drv):
    exp = _run(fake, {})
    assert drv.drain(fake.base, exp) == {}


def test_tag_histogram_mismatch_rejects(fake, drv):
    exp = _run(fake, {0: [(1, 2), (3, 4)]})
    # corrupt the second word's tag so the per-tag histogram disagrees with `accepted`
    w = tag_word(1, 3, 4)
    fake.mem[fake.base + 8: fake.base + 16] = int(w).to_bytes(8, "little")
    with pytest.raises(DdrUplinkError, match="carry its tag"):
        drv.drain(fake.base, exp)


def test_prepare_rejects_a_misaligned_or_high_base(drv):
    with pytest.raises(ValueError, match="aligned"):
        drv.prepare(0x1234)
    with pytest.raises(ValueError, match="aligned"):
        drv.prepare(0x8000_0200)


def test_prepare_rejects_a_run_that_would_wrap(drv):
    huge = {0: (R.RING_LIMIT // 8)}
    with pytest.raises(ValueError, match="ring limit"):
        drv.prepare(R.RING_LIMIT - 4096, expected=huge)


def test_prepare_refuses_after_a_forced_axi_reset(fake, drv):
    """r1: axi_rst_fault (not W1C) blocks the next run until the DDR-domain reset clears it."""
    fake.status_extra = 1 << R.S_AXI_RST_FAULT
    with pytest.raises(DdrUplinkError, match="axi_rst_fault"):
        drv.prepare(0x1000)
    assert R.S_AXI_RST_FAULT in R.FATAL_BITS


def test_inject_rejects_a_bad_core(drv):
    with pytest.raises(ValueError, match="out of range"):
        drv.inject(99, 1, 2)


def test_parse_words_round_trips_the_sign():
    words = np.array([tag_word(3, -0x1234560, 0x7654320)], dtype="<u8")
    tag, real, imag = parse_words(words)
    assert tag[0] == 3
    assert real[0] == (-0x1234560 >> 4) << 4
    assert imag[0] == (0x7654320 >> 4) << 4


# ── DDR readiness gate (host-domain status register) ────────────────────────────────────────────
# Codex r19: publishing calibration only in the uplink's own DIAG is useless in the case it exists for
# (a dead ui_clk side), so the authoritative copy lives in the HOST domain. These tests pin all three
# outcomes apart: ready / not ready / this build cannot report.

from pathlib import Path

from riscq.map import SocParams, SocMap


class _SocFake:
    """The SoC's own host-AXI window, on top of the uplink fake. Only read32 matters here."""

    def __init__(self, inner, soc_map, word):
        self.inner = inner
        self.addr = soc_map.ddr_status()
        self.word = word

    def read32(self, addr):
        return self.word if addr == self.addr else self.inner.read32(addr)

    def write32(self, addr, val):
        return self.inner.write32(addr, val)

    def __getattr__(self, k):
        return getattr(self.inner, k)


def _ddr_map():
    return SocMap(SocParams.load(Path(__file__).resolve().parents[1] / "configs" / "sim-2q-antq.json"))


def _status_word(calib, rst_ok, magic=None):
    m = _ddr_map()
    magic = m.DDR_STATUS_MAGIC if magic is None else magic
    return (magic << 16) | (int(rst_ok) << m.DDR_STATUS_UI_RST_OK) | (int(calib) << m.DDR_STATUS_CALIB)


def _drv_with(fake, word):
    sm = _ddr_map()
    return DdrReadout(_SocFake(fake, sm, word), soc_map=sm)


def test_ddr_ready_reports_both_bits(fake):
    d = _drv_with(fake, _status_word(calib=True, rst_ok=True))
    assert d.ddr_status() == (True, True)
    assert d.wait_ddr_ready(timeout=0.05) == (True, True)


def test_uncalibrated_mig_refuses_and_names_the_cause(fake):
    d = _drv_with(fake, _status_word(calib=False, rst_ok=True))
    with pytest.raises(DdrUplinkError, match="never finished calibration"):
        d.wait_ddr_ready(timeout=0.02)


def test_stuck_reset_tree_is_distinguished_from_a_bad_mig(fake):
    """The whole point of bit 1: 'calibrated but the reset tree never released' must not be reported
    as a calibration failure."""
    d = _drv_with(fake, _status_word(calib=True, rst_ok=False))
    with pytest.raises(DdrUplinkError, match="reset tree never released"):
        d.wait_ddr_ready(timeout=0.02)


def test_prepare_refuses_before_touching_any_register(fake):
    d = _drv_with(fake, _status_word(calib=False, rst_ok=False))
    before = dict(fake.regs)
    with pytest.raises(DdrUplinkError, match="DDR not ready"):
        d.prepare(0x1000, timeout=0.02)
    assert fake.regs == before, "prepare() must not write anything before the readiness gate passes"


def test_missing_magic_on_a_ddr_build_is_a_MISMATCH_not_a_skip(fake):
    """r20-#3: the config declares antq_uplink, so the register must exist. If the hardware does not
    answer with the magic, the loaded bitstream does not match the config -- failing OPEN here would
    disable the very gate that prevents the hang. It must raise, and name the cause."""
    d = _drv_with(fake, _status_word(calib=False, rst_ok=False, magic=0))
    with pytest.raises(DdrUplinkError, match="does not match this config"):
        d.ddr_status()
    with pytest.raises(DdrUplinkError, match="does not match this config"):
        d.prepare(0x1000, timeout=0.02)


def test_legacy_optout_skips_the_gate_explicitly(fake):
    """Running a new driver against a pre-status bitstream is legitimate, but it has to be ASKED for
    (Codex r20-#3). With the opt-out, the check is skipped and is NOT reported as a calibration
    failure."""
    sm = _ddr_map()
    d = DdrReadout(_SocFake(fake, sm, _status_word(calib=False, rst_ok=False, magic=0)),
                   soc_map=sm, legacy_no_ddr_status=True)
    assert d.ddr_status() is None
    assert d.wait_ddr_ready(timeout=0.02) is None
    with pytest.raises(DdrUplinkError) as ei:
        d.prepare(0x1000, timeout=0.02)          # this fake models registers, not the run handshake
    assert "DDR not ready" not in str(ei.value)
    assert "does not match this config" not in str(ei.value)


def test_construction_touches_no_ui_clock_register(fake):
    """r20-#2 (critical): NUM_CH / GEOMETRY live behind the ui_clk control slave. Reading them in
    __init__ would hang on exactly the uncalibrated-MIG failure the readiness gate exists to catch, so
    construction must read NOTHING and the geometry must be fetched only after the gate passes."""
    seen = []
    orig = fake.read32
    fake.read32 = lambda a: (seen.append(a), orig(a))[1]
    sm = _ddr_map()
    d = DdrReadout(_SocFake(fake, sm, _status_word(calib=False, rst_ok=True)), soc_map=sm)
    assert seen == [], f"the constructor read {[hex(a) for a in seen]}"
    # and touching the geometry now runs the gate first, so it refuses instead of hanging
    with pytest.raises(DdrUplinkError, match="never finished calibration"):
        _ = d.num_ch


def test_geometry_is_fetched_after_the_gate_passes(fake):
    sm = _ddr_map()
    d = DdrReadout(_SocFake(fake, sm, _status_word(calib=True, rst_ok=True)), soc_map=sm)
    assert d.num_ch == fake.num_ch
    assert d.bank_bytes == (1 << d.cbuf_addr_width) * R.BEAT_BYTES


def test_status_register_going_dark_mid_poll_is_reported(fake):
    """r20-#4: `ddr_status()` returning None inside the poll must not fall through to an incidental
    TypeError from unpacking it."""
    sm = _ddr_map()
    box = {"w": _status_word(calib=False, rst_ok=True)}
    soc = _SocFake(fake, sm, 0)
    soc.read32 = lambda a, _s=soc: (box["w"] if a == _s.addr else fake.read32(a))
    d = DdrReadout(soc, soc_map=sm, legacy_no_ddr_status=False)

    calls = {"n": 0}
    real = d.ddr_status

    def flaky():
        calls["n"] += 1
        return real() if calls["n"] < 2 else None

    d.ddr_status = flaky
    with pytest.raises(DdrUplinkError, match="stopped reporting"):
        d.wait_ddr_ready(timeout=0.5)


def test_no_soc_map_requires_an_explicit_optout(fake, drv):
    """r21-#4: without a SocMap the gate cannot run, and silently proceeding could still hang on the
    first ui_clk access. Constructing without either a SocMap or the opt-out must be refused."""
    with pytest.raises(ValueError, match="soc_map"):
        DdrReadout(fake)
    # with the opt-out it is allowed, and reports "cannot check" rather than "not calibrated"
    assert drv.ddr_status() is None
    assert drv.wait_ddr_ready(timeout=0.02) is None


def test_a_non_ddr_socmap_is_refused(fake):
    """r22-#4: a SocMap for a build WITHOUT the feature disables the gate as silently as no map at all,
    and almost always means the wrong config was loaded next to a DDR bitstream."""
    plain = SocMap(SocParams.load(Path(__file__).resolve().parents[1] / "configs" / "sim-2q.json"))
    with pytest.raises(ValueError, match="not .antq_uplink."):
        DdrReadout(fake, soc_map=plain)
    DdrReadout(fake, soc_map=plain, legacy_no_ddr_status=True)      # explicit opt-out is fine


def test_the_readiness_budget_is_compositional(fake):
    """r23-#3: `_gate()` waits for readiness and then loads the geometry. If those were two independent
    timeouts, a readiness loss between them could consume nearly twice what the caller asked for. One
    deadline covers both, so a total loss must be reported within the requested budget."""
    # r24-#2: making readiness false from the start never reaches `_ensure_geometry()`, so the old
    # two-independent-deadline code passed too. The first wait must CONSUME most of the budget and then
    # succeed; only then does a second, fresh deadline show up as roughly double the elapsed time.
    #   shared deadline : 0.15 s in the gate + 0.05 s left for the geometry  = ~0.20 s
    #   two deadlines   : 0.15 s in the gate + a fresh 0.20 s                = ~0.35 s
    # (Verified by mutation: reverting ddr.py to two deadlines makes this test fail.)
    sm = _ddr_map()
    soc = _SocFake(fake, sm, _status_word(calib=True, rst_ok=True))
    d = DdrReadout(soc, soc_map=sm)
    budget, arrive_at = 0.20, 0.15
    ready, unready = _status_word(calib=True, rst_ok=True), _status_word(calib=False, rst_ok=True)
    state = {"n": 0, "gate_passed": False, "t0": time.monotonic()}

    def flip(a, _s=soc):
        if a != _s.addr:
            return fake.read32(a)
        state["n"] += 1
        if not state["gate_passed"]:
            if time.monotonic() - state["t0"] < arrive_at:
                return unready                       # the gate burns most of the budget waiting
            state["gate_passed"] = True
            return ready                             # ... then readiness arrives, once
        return unready                               # the geometry load never gets it

    soc.read32 = flip
    t0 = time.monotonic()
    with pytest.raises(DdrUplinkError, match="DDR not ready"):
        d.prepare(0x1000, timeout=budget)
    spent = time.monotonic() - t0
    assert state["gate_passed"], "the gate never passed -- the geometry load was never reached"
    assert spent < budget * 1.4, (
        f"the {budget}s budget took {spent:.3f}s -- the gate and the geometry load are using separate "
        f"deadlines again")


def test_every_public_op_passes_the_gate(fake):
    """r21-#3: readiness can be lost BETWEEN operations, so flush/drain/inject must each re-check --
    not rely on prepare() having checked once."""
    sm = _ddr_map()
    soc = _SocFake(fake, sm, _status_word(calib=True, rst_ok=True))
    d = DdrReadout(soc, soc_map=sm)
    exp = _run(fake, {0: [(1, 2)]})
    d.drain(fake.base, exp)                                  # works while ready
    soc.word = _status_word(calib=False, rst_ok=True)        # the MIG drops out underneath us
    # r22-#2/#3: the RAW helpers are private and ungated on purpose (they run inside loops that already
    # passed the gate); every PUBLIC accessor must gate, or an interactive caller could hang the host on
    # an uncalibrated MIG just by reading a register.
    for call in (lambda: d.flush(timeout=0.02),
                 lambda: d.drain(fake.base, exp),
                 lambda: d.inject(0, 1, 2, timeout=0.02),
                 lambda: d.prepare(0x1000, timeout=0.02),
                 lambda: d.status(timeout=0.02),
                 lambda: d.diag(timeout=0.02),
                 lambda: d.clear_sticky(timeout=0.02),
                 lambda: d.accepted(timeout=0.02),
                 lambda: d.rejected(timeout=0.02),
                 lambda: d.rd(R.NUM_CH, timeout=0.02),
                 lambda: d.wr(R.WR_BASE, 0, timeout=0.02)):
        with pytest.raises(DdrUplinkError, match="DDR not ready"):
            call()
    # ... while the private raw ones still work, which is what the loops rely on
    assert d._status() is not None
    assert isinstance(d._diag(), dict)
