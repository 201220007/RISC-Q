"""The hardware contract in one place: SocParams (per-build JSON) -> SocMap (every address).

The formulas mirror the SpinalHDL elaboration (SocMemoryMap in src/riscq/soc/Zcu216Top.scala and the
strides computed in PulseTableSoc.scala); no address may appear literally anywhere else in the stack.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

# ── fixed architecture constants (assert, never parameterize) ──
BATCH_SIZE = 16          # DAC samples per batch (= per dsp cycle)
ADC_BATCH = 4            # ADC samples per batch
DATA_WIDTH = 16          # SInt sample width
READOUT_MAX_WIN_LOG2 = 14  # decoder no-overflow bound: longest demod window <= 2^this batches
READOUT_ACC_WIDTH = 32   # decoder accumulator width (= one-word readback)
MEM_BASE = 0x80000000    # CPU boot vector = base of the unified I+D RAM

# Readout-result freshness lead (batches): the carrier-triggered decoder's res.valid is forwarded up
# the link as a LEVEL that holds the PREVIOUS shot until this window's winStart reaches the sink
# (~Ldemod + carrier-stage + link_pipe + sink ~ 16-29 batches after the demod startTime). After firing
# the demod at `t`, software waits `t + READOUT_LEAD` before read_res so the stale level has dropped and
# the halting read returns THIS window. Conservative (covers link_pipe up to ~32); over-waiting is free.
READOUT_LEAD = 48

# conservative schedule lead (batches): covers linkPipe + worst timed-queue lead + CPU jitter.
# Pinned by the M1 margin test: the CPU's now()-read -> fire-landing path measured ~40 batches,
# on top of the worst FIRED queue lead (phase, 35) — the M0 guess of 64 left the first ~9 window
# batches with stale amp/phase on a fresh SoC. 96 = 40 + 35 + ~20 margin; the freq (phasor
# regen) lead of linkPipe+52 only binds when freq is written against the pulse's startTime —
# programs write freq early against startTime 0 (see test_pulse.PULSE_SRC).
LEAD = 96


# 16-bit pulse-parameter fields live at data[31:16] of the posted write word (PulseParamBuffer
# bitOffset = 16; the hardware ignores data[15:0] on those addresses — only startTime/fire read the
# low bits). pack16 seats a code there — THE software-side field packer, replacing the firmware
# RQ_PACK16 (spec 12). A seated code is code << 16: its low 16 bits are zero, so a compile-time
# literal loads in one `lui`, and an on-core Q16 accumulator carries its fraction in the ignored low
# half and is written raw (no >> 16 extraction). Codes wider than 16 bits (phase wraps) fold mod 2^16.
# The result is a SIGNED int32 (the value a C int32_t register holds): a code with bit 15 set seats
# to a NEGATIVE word — so it passes the kernel's int32 binding range check and emits as a clean C
# literal. Byte/word writers mask with & 0xFFFFFFFF for serialization.
def pack16(code: int) -> int:
    w = (int(code) & 0xFFFF) << 16
    return w - (1 << 32) if w >= (1 << 31) else w


_FIELDS = {
    "name": str,
    "qubit_num": int,
    "dac_num": int,
    "adc_num": int,
    "mem_depth": int,
    "env_depth": int,
    "rob_depth": int,
    "gate_pulse_num": int,
    "gate_interp": int,
    "readout_interp": int,
    "demod_interp": int,
    "link_pipe": int,
    "with_mul": bool,
    "dsp_freq_hz": float,
}

_FIXED = {
    "batch_size": BATCH_SIZE,
    "adc_batch": ADC_BATCH,
    "data_width": DATA_WIDTH,
    "readout_max_win_log2": READOUT_MAX_WIN_LOG2,
    "readout_acc_width": READOUT_ACC_WIDTH,
}


def pow2ceil(x: int) -> int:
    """1 << log2Up(x), matching SpinalHDL's log2Up = (x-1).bitLength."""
    assert x > 0
    return 1 << (x - 1).bit_length()


@dataclass(frozen=True)
class SocParams:
    """One build's elaboration parameters — the python view of software/configs/<name>.json,
    the same file GenPulseTableSocJson feeds to the SpinalHDL elaboration."""

    name: str
    qubit_num: int
    dac_num: int
    adc_num: int
    mem_depth: int
    env_depth: int
    rob_depth: int
    gate_pulse_num: int
    gate_interp: int
    readout_interp: int
    demod_interp: int
    link_pipe: int
    with_mul: bool
    dsp_freq_hz: float
    # Optional per-build converter maps (the same GenPulseTableSocJson reads to wire the elaboration; None
    # = the generic ZCU216 SocChannelMap layout). Software reads them too so the co-sim model drives/reads
    # the SAME converters the hardware wired: dac_map[core] = (gateDac, readoutDriveDac); adc_map[core] =
    # the core's demod ADC. Held as tuples (a frozen dataclass must stay hashable).
    dac_map: tuple | None = None
    adc_map: tuple | None = None
    # Readout->DDR uplink (qubic3 C1). Hardware-and-software: the elaboration instantiates
    # ReadoutDdrUplink, and software needs it to know whether the DDR status register exists. Kept OUT of
    # _FIELDS deliberately -- it is optional, so every pre-existing config still round-trips byte-for-byte
    # through to_json(). (Before this, loading any *-ddr.json raised "unknown SocParams fields".)
    ddr_readout: bool = False

    @classmethod
    def load(cls, json_path: str | Path) -> "SocParams":
        return cls.from_json(Path(json_path).read_text())

    @classmethod
    def from_json(cls, text: str) -> "SocParams":
        raw = json.loads(text)
        for key, value in _FIXED.items():  # fixed constants may be stated, never changed
            if key in raw and raw.pop(key) != value:
                raise ValueError(f"{key} is fixed by architecture at {value}")
        raw.setdefault("demod_interp", 4)   # optional (added with the envelope-shaped demod); default 4
        raw.setdefault("with_mul", True)    # optional; Zmmul multiply is on by default (RiscqParam.withMul)
        raw.setdefault("rob_depth", 1024)   # optional; readout-trace ROB depth default (PulseTableSoc)
        dac_map = raw.pop("dac_map", None)  # optional converter maps: kept (the channel-role mapping — the
        adc_map = raw.pop("adc_map", None)  # co-sim model needs them), unlike the hardware-only knobs below
        ddr_readout = bool(raw.pop("ddr_readout", False))   # optional; software-visible (DDR status reg)
        raw.pop("queue_depth", None)        # hardware-only TimedQueue depth; software uses slot count, not depth
        unknown = set(raw) - set(_FIELDS)
        if unknown:
            raise ValueError(f"unknown SocParams fields: {sorted(unknown)}")
        missing = set(_FIELDS) - set(raw)
        if missing:
            raise ValueError(f"missing SocParams fields: {sorted(missing)}")
        kwargs = {k: _FIELDS[k](raw[k]) for k in _FIELDS}
        if dac_map is not None:
            kwargs["dac_map"] = tuple(tuple(int(x) for x in row) for row in dac_map)
        if adc_map is not None:
            kwargs["adc_map"] = tuple(int(x) for x in adc_map)
        kwargs["ddr_readout"] = ddr_readout
        return cls(**kwargs)

    def to_json(self) -> str:
        out = {k: getattr(self, k) for k in _FIELDS}
        if self.dac_map is not None:
            out["dac_map"] = [list(row) for row in self.dac_map]
        if self.adc_map is not None:
            out["adc_map"] = list(self.adc_map)
        if self.ddr_readout:
            out["ddr_readout"] = True
        return json.dumps(out, indent=4)


@dataclass(frozen=True)
class MapEntry:
    """One host-AXI window, for the generated contract test. kind:
    ram_rw (core RAM, host R/W), env_gate (via 512/gate_interp-bit WidthAdapter, R/W),
    env_ro (direct 32-bit, R/W), env_demod (demod-carrier bank, direct 32-bit at interp 4, R/W),
    robs_ro (shared trace BRAM, read-only for software), ctrl_wo (host control block, write-only)."""

    name: str
    host_addr: int
    nbytes: int
    kind: str


@dataclass(frozen=True)
class ChannelInfo:
    """One per-core logical RF channel (spec 02 §3.2): the compiler folds a ParamTable to this.
    `base` is the CPU-local RF sub-window base, `slot_count` the pulse-table depth,
    `samples_per_line`/`line_bytes` the channel's envelope-RAM grid (stored samples and bytes per
    line), `cname` the riscq_map.h macro (RF_CHi) the backend emits for it. A channel is identified
    end-to-end by its numeric `index` (0/1/2) — the same int a ParamTable carries; there is no
    separate channel name."""

    index: int
    cname: str
    base: int
    slot_count: int
    samples_per_line: int
    line_bytes: int


class SocMap:
    """Every base/stride/offset of one build, derived from its SocParams with the elaboration's
    own formulas. Host-AXI addresses are absolute on the SoC's AXI slave; core-local addresses
    (CTRL_*/RF_*) are CPU addresses shared with the generated riscq_map.h."""

    # ── core-local control block (CPU addresses; ControlMemMaps.scala + ReadoutResultSink) ──
    CTRL_FROM_HOST = 0x2000
    CTRL_TIME_CMP = 0x4000
    CTRL_WAIT_TIME_CMP = 0x4008     # read HALTS until time + 3 >= timeCmp
    CTRL_RES = 0x4200               # read HALTS until the armed readout completes
    CTRL_REAL = 0x4204
    CTRL_IMAG = 0x4208
    CTRL_TIME = 0xBFF8

    # ── RF window (CPU addresses; RfLinkBridge at 0x10000, demux bases in RiscqRfWithPulseTableFiber) ──
    RF_GATE = 0x10000               # gate drive channel base
    RF_READOUT = 0x20000            # readout drive channel base
    RF_DEMOD = 0x30000              # demod carrier: a drive channel (fire@0/freq@4/table@0x10../startTime@0x4100)
    #                                 firing the demod IS the readout (carrier-triggered decoder, no arm);
    #                                 0x40000 stays reserved/unmapped (no decoder register window).
    # per drive channel (offsets from the channel base; PulseParamBufferParams):
    RF_FIRE = 0x0                   # write slot index (low bits) -> enqueue
    RF_FREQ = 0x4                   # carrier freq code at data[31:16]
    RF_DC_OFFSET = 0x8              # DC bias added to the real output lanes, code at data[31:16]
    RF_PHASE_OFFSET = 0xC           # virtual-Z phase added to the generator's phase input, code at data[31:16]
    RF_SLOT_STRIDE = 0x10           # slot i at (i+1)*0x10: +0 phase, +4 amp, +8 env, +12 dur
    RF_START_TIME = 0x4100          # full 32-bit batch time (low bits)

    # ── host control block (offsets from host_ctrl; PulseTableSoc hostCtrlDriver) ──
    HOST_RESET = 0x0                # write 1 = assert (power-up state), 0 = release
    HOST_FROM_HOST = 0x10           # legacy mailbox, unused by this framework
    HOST_TIME_OFF_LO = 0x40
    HOST_TIME_OFF_HI = 0x44
    # ── DDR uplink status (read-only; present only when params.ddr_readout) ──
    # Lives in the HOST clock domain on purpose: its reset tree is independent of the DDR one, so this
    # register still answers when the whole ui_clk side is dead -- which is the failure it exists to
    # diagnose. The uplink's own DIAG[9] mirrors bit 0, but DIAG is unreadable in exactly that case.
    HOST_DDR_STATUS = 0x50
    DDR_STATUS_MAGIC = 0xCA1B       # [31:16]; an older bitstream reads 0 here -> capability absent
    DDR_STATUS_CALIB = 0            # MIG c0_init_calib_complete
    DDR_STATUS_UI_RST_OK = 1        # the ui_clk reset tree released

    LEAD = LEAD

    def __init__(self, params: SocParams):
        self.params = params
        p = params

        env_width = BATCH_SIZE * 2 * DATA_WIDTH            # 512-bit complex envelope line
        demod_full = ADC_BATCH * 2 * DATA_WIDTH            # 128-bit demod-carrier line (ADC batch = 4 lanes)
        self.gate_env_width = env_width // p.gate_interp   # bits per stored gate-envelope line
        self.ro_env_width = env_width // p.readout_interp  # bits per stored readout-envelope line
        self.demod_env_width = demod_full // p.demod_interp  # bits per stored demod-carrier line (32 at 4)
        self.rob_width = ADC_BATCH * 32

        # envelope grid: stored samples per RAM line (= per batch), each held x interp at the converter
        self.gate_samples_per_line = BATCH_SIZE // p.gate_interp     # 4 at gateInterp 4
        self.ro_samples_per_line = BATCH_SIZE // p.readout_interp    # 1 at readoutInterp 16
        self.demod_samples_per_line = ADC_BATCH // p.demod_interp    # 1 at demodInterp 4
        self.gate_line_bytes = self.gate_env_width // 8              # 16 (4 words per line)
        self.ro_line_bytes = self.ro_env_width // 8                  # 4  (1 word per line)
        self.demod_line_bytes = self.demod_env_width // 8            # 4  (1 word per line)

        self.mem_bytes = p.mem_depth * 4                   # unified I+D RAM bytes
        self.gate_env_bytes = self.gate_env_width * p.env_depth // 8
        self.ro_env_bytes = self.ro_env_width * p.env_depth // 8
        self.demod_env_bytes = self.demod_env_width * p.env_depth // 8
        self.rob_bytes = self.rob_width * p.rob_depth // 8

        # SocMemoryMap formulas (PulseTableSoc passes coreMemBytes = 1 << 16)
        self.core_stride = pow2ceil(1 << 16)
        self.pulse_stride = pow2ceil(self.gate_env_bytes)
        self.ro_env_stride = pow2ceil(self.ro_env_bytes)
        self.demod_env_stride = pow2ceil(self.demod_env_bytes)
        self.rob_stride = pow2ceil(2 * pow2ceil(self.rob_bytes))
        strides = (self.core_stride, self.pulse_stride, self.ro_env_stride,
                   self.demod_env_stride, self.rob_stride)
        self.region_size = pow2ceil(max(strides) * p.qubit_num)

        self.core_mem_base = 0
        self.pulse_mem_base = 1 * self.region_size
        self.ro_env_base = 2 * self.region_size
        self.demod_env_base = 3 * self.region_size
        self.rob_base = 4 * self.region_size
        self.host_ctrl = 5 * self.region_size

    # ── host-AXI address helpers ──
    def ddr_status(self) -> int:
        """Absolute host-AXI address of the DDR uplink status register (see HOST_DDR_STATUS)."""
        if not self.params.ddr_readout:
            raise ValueError(f"{self.params.name} was not built with ddr_readout; there is no DDR status register")
        return self.host_ctrl + self.HOST_DDR_STATUS

    def _core(self, core: int) -> int:
        if not 0 <= core < self.params.qubit_num:
            raise ValueError(f"core {core} out of range (qubit_num={self.params.qubit_num})")
        return core

    def imem(self, core: int) -> int:
        """Host base of the core's unified I+D RAM window (CPU 0x80000000)."""
        return self.core_mem_base + self._core(core) * self.core_stride

    def gate_env(self, core: int) -> int:
        return self.pulse_mem_base + self._core(core) * self.pulse_stride

    def ro_env(self, core: int) -> int:
        return self.ro_env_base + self._core(core) * self.ro_env_stride

    def demod_env(self, core: int) -> int:
        return self.demod_env_base + self._core(core) * self.demod_env_stride

    def robs(self) -> int:
        """The ONE shared readout trace BRAM (not per-core)."""
        return self.rob_base

    # ── per-core logical RF channel table (spec 02 §3.2) ──
    def channels(self) -> list[ChannelInfo]:
        """The logical RF channels, indexed by the ParamTable `channel` field: 0 = gate drive
        (gate_pulse_num slots, 4 samples/line), 1 = readout drive (1 slot, 1 sample/line), 2 = demod
        carrier (1 slot, 1 ADC-batch sample/line — an envelope fed to the readout decoder; its
        carrier freq is set separately via set_freq with an ADC-rate demod_freq_to_code code)."""
        return [
            ChannelInfo(0, "RF_CH0", self.RF_GATE, self.params.gate_pulse_num,
                        self.gate_samples_per_line, self.gate_line_bytes),
            ChannelInfo(1, "RF_CH1", self.RF_READOUT, 1,
                        self.ro_samples_per_line, self.ro_line_bytes),
            ChannelInfo(2, "RF_CH2", self.RF_DEMOD, 1,
                        self.demod_samples_per_line, self.demod_line_bytes),
        ]

    def channel(self, index: int) -> ChannelInfo:
        chans = self.channels()
        if not 0 <= index < len(chans):
            raise ValueError(f"unknown channel index {index} (have {len(chans)} channels)")
        return chans[index]

    def env_base(self, index: int, core: int) -> int:
        """Host base of `core`'s envelope RAM for channel `index` (0 gate / 1 ro / 2 demod)."""
        self.channel(index)   # validate the index (ValueError on unknown)
        return (self.gate_env, self.ro_env, self.demod_env)[index](core)

    # ── channel -> converter map: the build's dac_map/adc_map when it carries them (sim-2q1c and any
    # per-build JSON), else the generic ZCU216 SocChannelMap layout (PulseTableSoc.scala) the default
    # configs elaborate. The co-sim model reads these to drive/read the converters the hardware wired. ──
    def gate_dac(self, core: int) -> int:
        core = self._core(core)
        if self.params.dac_map is not None:
            return self.params.dac_map[core][0]
        return core

    def ro_dac(self, core: int) -> int:
        """Readout-drive DAC, shared/summed. Config dac_map when present; else the default layout
        (cores 0-6 -> DAC 14, 7+ -> DAC 15)."""
        core = self._core(core)
        if self.params.dac_map is not None:
            return self.params.dac_map[core][1]
        return 14 if core < 7 else 15

    def adc_of(self, core: int) -> int:
        """Demod ADC. Config adc_map when present; else the default layout (cores 0-6 -> ADC 0, 7+ -> 4)."""
        core = self._core(core)
        if self.params.adc_map is not None:
            return self.params.adc_map[core]
        return 0 if core < 7 else 4

    def dac_pipe(self, dac_id: int) -> int:
        """dspClk register stages from a channel's scheduled pulse to its io_dac port. `PulseTableSoc`
        pads EVERY driven DAC to one shared depth `dacAlignStages` (the deepest combine among the
        physical DACs) so all pulses leave the converter edge time-aligned, so this is **uniform** across
        driven DACs (the `dac_id` arg is kept for the per-DAC caller contract). A DAC's natural combine is
        0 for a pass-through single channel and `AdderTree.latency(n)+1` for an n-channel sum; on top of
        the shared align there is the channel's own dcOffset output register and one shared RFDC-edge
        stage (`dacAlignStages + 2`). E.g. 4 on the 2-core map, 6 on the 14-core map."""
        def combine_latency(n: int) -> int:
            return 0 if n <= 1 else (n - 1).bit_length() + 1   # AdderTree.latency(n) + 1
        def channels_on(d: int) -> int:
            return sum(1 for c in range(self.params.qubit_num)
                       for cd in (self.gate_dac(c), self.ro_dac(c)) if cd == d)
        align = max(combine_latency(channels_on(d)) for d in range(self.params.dac_num))
        return align + 2

    def to_host_addr(self, core: int, cpu_addr: int) -> int:
        """Translate a core-local (CPU) RAM address into the core's host window."""
        if not MEM_BASE <= cpu_addr < MEM_BASE + self.mem_bytes:
            raise ValueError(f"cpu addr {cpu_addr:#x} outside RAM "
                             f"[{MEM_BASE:#x}, {MEM_BASE + self.mem_bytes:#x})")
        return self.imem(core) + (cpu_addr - MEM_BASE)

    # ── contract-test descriptors, generated from this object ──
    def entries(self) -> list[MapEntry]:
        out = []
        for c in range(self.params.qubit_num):
            out.append(MapEntry(f"core{c}_ram", self.imem(c), self.mem_bytes, "ram_rw"))
            out.append(MapEntry(f"core{c}_gate_env", self.gate_env(c), self.gate_env_bytes, "env_gate"))
            out.append(MapEntry(f"core{c}_ro_env", self.ro_env(c), self.ro_env_bytes, "env_ro"))
            out.append(MapEntry(f"core{c}_demod_env", self.demod_env(c), self.demod_env_bytes, "env_demod"))
        out.append(MapEntry("robs", self.robs(), self.rob_bytes, "robs_ro"))
        out.append(MapEntry("host_ctrl", self.host_ctrl, 0x48, "ctrl_wo"))
        return out

    # ── generated firmware inputs ──
    def gen_header(self) -> str:
        """riscq_map.h — the evaluated core-local contract for this build."""
        p = self.params
        defines = [
            ("RQ_MEM_BASE", MEM_BASE), ("RQ_MEM_BYTES", self.mem_bytes),
            ("RQ_CTRL_FROM_HOST", self.CTRL_FROM_HOST),
            ("RQ_CTRL_TIME_CMP", self.CTRL_TIME_CMP),
            ("RQ_CTRL_WAIT_TIME_CMP", self.CTRL_WAIT_TIME_CMP),
            ("RQ_CTRL_RES", self.CTRL_RES), ("RQ_CTRL_REAL", self.CTRL_REAL),
            ("RQ_CTRL_IMAG", self.CTRL_IMAG), ("RQ_CTRL_TIME", self.CTRL_TIME),
            ("RQ_GATE", self.RF_GATE), ("RQ_READOUT", self.RF_READOUT),
            ("RQ_DEMOD", self.RF_DEMOD),
            ("RQ_FIRE", self.RF_FIRE), ("RQ_FREQ", self.RF_FREQ),
            ("RQ_DC_OFFSET", self.RF_DC_OFFSET),
            ("RQ_PHASE_OFFSET", self.RF_PHASE_OFFSET),
            ("RQ_SLOT_STRIDE", self.RF_SLOT_STRIDE), ("RQ_START_TIME", self.RF_START_TIME),
            ("RQ_GATE_PULSE_NUM", p.gate_pulse_num), ("RQ_ENV_DEPTH", p.env_depth),
            ("RQ_LEAD", LEAD), ("RQ_RO_LEAD", READOUT_LEAD),
            ("RQ_RO_MAX_WIN", 1 << READOUT_MAX_WIN_LOG2),
        ]
        lines = [f"/* GENERATED from SocParams '{p.name}' by riscq.map — do not edit. */",
                 "#ifndef RISCQ_MAP_H", "#define RISCQ_MAP_H", ""]
        lines += [f"#define {name} 0x{value:x}" for name, value in defines]
        # per-core logical RF channel bases (spec 02 §3.2): a ParamTable folds to its RF_CHi
        lines += [f"#define {ch.cname} 0x{ch.base:x}" for ch in self.channels()]
        lines += ["", "#endif", ""]
        return "\n".join(lines)

    def gen_linker(self) -> str:
        """link.ld — one unified RAM region; .text first so entry = 0x80000000; 1 KB stack reserve."""
        return f"""\
/* GENERATED from SocParams '{self.params.name}' by riscq.map — do not edit. */
ENTRY(_start)
MEMORY {{ RAM (rwx) : ORIGIN = 0x{MEM_BASE:x}, LENGTH = {self.mem_bytes} }}
SECTIONS {{
  .text   : {{ KEEP(*(.text.init)) *(.text*) }} > RAM
  .rodata : {{ *(.rodata*) *(.srodata*) }} > RAM
  .data   : {{ *(.data*) *(.sdata*) }} > RAM
  .bss (NOLOAD) : {{
    . = ALIGN(4);
    __bss_start = .;
    *(.bss*) *(.sbss*) *(COMMON)
    . = ALIGN(4);
    __bss_end = .;
  }} > RAM
  /DISCARD/ : {{ *(.eh_frame*) *(.comment) *(.riscv.attributes) }}
  _end = .;
  __stack_top = ORIGIN(RAM) + LENGTH(RAM);
  ASSERT(_end <= __stack_top - 1024, "riscq: program overflows RAM (1 KB stack reserve)")
}}
"""
