"""Measure (specs/universal-cal/01 §5): how a sequence's readout is taken and decoded.

`Measure.counts / raw / levels / iqsum` name the result mode of `k_batched`; `tables()` builds
the reading core's ro/demod slots and the `MeasInfo` the sequence header needs (base.readout_tables
is the source of every code); `decode()` turns a core's `out` into the population / IQ array the
analyses consume, res-sign and herald pairs folded (base.population*). A `levels` measure
captures in the classifier's zero frame — the invariant the six old call sites restated.

`host=True` means off-core capture (qubic3 P6, plan P6 v2 §4.1): the HostWindow on a hostwindow
build, the readout uplink on an antq_uplink build (`backend`). The uplink records one decoder
result per demod window, so it carries RAW and levels IQ as 28-bit fields (multiples of 16 counts);
COUNTS, IQSUM and `host=False` keep their results in core RAM on both builds."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from riscq.cal import base
from riscq.cal.batched import COUNTS, IQSUM, RAW, UPLINK
from riscq.cal.sequence import Meas, MeasInfo


@dataclass(frozen=True)
class Measure:
    mode: str                  # counts | raw | levels | iqsum
    herald: bool = False
    phase: float | None = None     # demod discrimination phase override (levels/raw: 0.0)
    win: float | None = None       # demod window override (seconds)
    classifiers: dict | None = None
    level: int = 2
    sh: int = 0
    host: bool = False             # off-core capture: the host window (spec 22) or the antq uplink
    meas: Meas = Meas()
    reads: tuple = ()              # the qubits that read out (default: the key itself); a pair
    #                                experiment reads both members, each on its own core

    @staticmethod
    def counts(herald: bool = False, meas: Meas = Meas(), reads: tuple = ()) -> "Measure":
        return Measure("counts", herald, meas=meas, reads=tuple(reads))

    @staticmethod
    def raw(phase: float | None = 0.0, meas: Meas = Meas(), host: bool = True) -> "Measure":
        return Measure("raw", phase=phase, meas=meas, host=host)

    @staticmethod
    def levels(classifiers: dict, level: int = 2, meas: Meas = Meas(), host: bool = False) -> "Measure":
        return Measure("levels", phase=0.0, classifiers=classifiers, level=level, meas=meas,
                       host=host)

    @staticmethod
    def iqsum(sh: int, meas: Meas = Meas()) -> "Measure":
        return Measure("iqsum", sh=sh, meas=meas)

    @property
    def kernel_mode(self) -> int:
        return {"counts": COUNTS, "raw": RAW, "levels": RAW, "iqsum": IQSUM}[self.mode]

    def backend(self, m) -> str:
        """Where this measure's results go on the build `m` (P6 v2 §4.1): "ram" for `host=False`;
        for `host=True`, "hostwindow" on a hostwindow build and "uplink" on an antq_uplink build. The
        uplink carries one decoder result per demod window, so only RAW and levels IQ can take it."""
        if not self.host:
            return "ram"
        if m.params.with_host_window:
            return "hostwindow"
        if m.params.with_antq_uplink:
            if self.mode in ("raw", "levels"):
                return "uplink"
            raise ValueError(f"Measure.{self.mode}(host=True) on {m.params.name}: the uplink carries one "
                             f"decoder result per shot, not {self.mode} aggregates; use host=False")
        raise ValueError(f"Measure(host=True) needs an off-core capture path, but {m.params.name} has "
                         f"results_path={m.params.results_path!r}, with neither the HostWindow nor the uplink")

    def kernel_mode_for(self, m) -> int:
        return UPLINK if self.backend(m) == "uplink" else self.kernel_mode

    def out_size_for(self, m, npts: int, shots: int) -> int:
        """The result words `k_batched` stores in `out` on the build `m` (the completion marker, when
        the build has one, comes after them): 0 for the uplink, else `out_size`."""
        return 0 if self.backend(m) == "uplink" else self.out_size(npts, shots)

    def out_size(self, npts: int, shots: int) -> int:
        if self.mode == "counts":
            return 2 * npts if self.herald else npts
        if self.mode == "iqsum":
            return 2 * npts
        return 2 * npts * shots

    def tables(self, cfg, q, m, tables, core=None) -> MeasInfo:
        """Register qubit q's ro/demod slots in `tables` (riscq.cal.gates.Tables) and return the
        header's MeasInfo. Codes are base.readout_tables' (the config's readout/{q}/* in physical
        units). `core` puts the slots on another core's readout channels — a non-reading core
        (a coupler, a spectator) carries them uninitialised-never-fired so its init preamble is
        the readers' and the cores' grids stay aligned."""
        ro, demod, code, win, ddly = base.readout_tables(cfg, q, m, phase=self.phase, win=self.win)
        c = q if core is None else core
        ro_ch, demod_ch = base.ro_ch(m, c), base.demod_ch(m, c)
        tables.add_table(ro_ch, "meas", ro.pulses["meas"], ro.freq_hz)
        tables.add_table(demod_ch, "sq", demod.pulses["sq"], 0.0)
        return MeasInfo(ro_ch, demod_ch, code, win, ddly, self.meas)

    def decode(self, out: np.ndarray, q, npts: int, shots: int, sign: int = 1):
        """A core's `out` → P (counts / levels), IQ shots (raw: (npts·shots, 2)), or the complex
        per-point sums (iqsum)."""
        if self.mode == "counts":
            return (base.population_heralded(out, sign) if self.herald
                    else base.population(out, shots, sign))
        if self.mode == "levels":
            return base._levels_pop(out, npts, shots, self.classifiers[q], self.level)
        if self.mode == "raw":
            return np.asarray(out, dtype=float).reshape(npts * shots, 2)
        z = np.asarray(out, dtype=float).reshape(npts, 2)
        return z[:, 0] + 1j * z[:, 1]
