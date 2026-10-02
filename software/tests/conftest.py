from pathlib import Path

import pytest

SW_ROOT = Path(__file__).resolve().parents[1]
CONFIGS = SW_ROOT / "configs"


def pytest_addoption(parser):
    parser.addoption("--cosim", action="store_true", default=False,
                     help="run the verilator co-simulation tests")
    parser.addoption("--slow", action="store_true", default=False,
                     help="also run the full-loop anchor tests (minutes each; implies --cosim)")
    parser.addoption("--batch-cap", type=int, default=0, metavar="N",
                     help="fail a co-sim test that simulates more than N batches (0 = report only)")
    parser.addoption("--results-path", choices=("hostwindow", "antq_uplink"), default="hostwindow",
                     help="build every co-sim fixture with this results path: antq_uplink loads each "
                          "config's `-antq` sibling, so the cycle-exact co-sim tiers run on the shipped "
                          "antq RTL (qubic3 P3c-3 N5); tests marked `hostwindow` are skipped there")


def pytest_configure(config):
    if config.getoption("--slow"):
        config.option.cosim = True          # the anchors are co-sim tests: --slow implies --cosim
    config.addinivalue_line("markers", "cosim: verilator co-simulation test (needs --cosim)")
    config.addinivalue_line(
        "markers",
        "hostwindow: needs the HostWindow results path (RAW host readback, the host window itself); "
        "skipped under --results-path antq_uplink, where an antq image rejects such programs (plan v2 P3b #11)")
    config.addinivalue_line(
        "markers",
        "batch_cap(n): raise this test's simulated-batch cap to n. ONLY for a structural floor "
        "that cannot be cut without deleting the claim — a co-sim AXI word costs ~22 dspClk "
        "cycles, so one core's image load alone is ~7-12k batches and an N-core lock-step claim "
        "floors near N x 10k. Every use must name the floor in the test's docstring.")
    config.addinivalue_line(
        "markers",
        "slow: full-loop anchor — real shots, noise and fits end-to-end (needs --slow). The "
        "regression net for the tier split: if a host-pure responder or an L2 analytic target "
        "drifts from what the hardware really does, these are what notice "
        "(specs/software-test-refactor/01 §5).")
    config.addinivalue_line(
        "markers",
        "expected_limit: documents a known precision limit of a results path. The test asserts the "
        "limit itself (the check fails where the limit says it must), so a change that lifts or "
        "moves the limit shows up (qubic3 P6, the 28-bit uplink IQ).")


def pytest_collection_modifyitems(config, items):
    cosim, slow = config.getoption("--cosim"), config.getoption("--slow")
    skip_cosim = pytest.mark.skip(reason="co-sim test: pass --cosim to run")
    skip_slow = pytest.mark.skip(reason="full-loop anchor: pass --slow to run")
    antq = config.getoption("--results-path") == "antq_uplink"
    skip_hw = pytest.mark.skip(reason="needs the HostWindow results path (skipped under --results-path antq_uplink)")
    for item in items:
        if "slow" in item.keywords and not slow:
            item.add_marker(skip_slow)
        elif "cosim" in item.keywords and not cosim:
            item.add_marker(skip_cosim)
        elif antq and item.get_closest_marker("hostwindow") is not None:   # the marker, not a param id
            item.add_marker(skip_hw)


def _cosim_build(request, name: str):
    """(config, build dir) of the co-sim fixture `name`: the config itself, or its `-antq` sibling under
    --results-path antq_uplink (one build dir per config, so the two variants never share a model)."""
    if request.config.getoption("--results-path") == "antq_uplink":
        name = f"{name}-antq"
    return CONFIGS / f"{name}.json", SW_ROOT / "build" / name


# ── the simulated-batch meter (specs/software-test-refactor/02 §1, E2) ──
#
# A co-sim test's wall time is set by how many dspClk batches it makes the RTL simulate:
# ~11.5k batches/s with the ADC model off, ~7k with one attached, ~6k for multi/two-qubit. So
# simulated batches — NOT seconds — is the suite's cost unit: it is machine-independent, and it
# is the number a test author can actually reason about (points x shots x grid_period).
#
# The meter reads the bench's simulated clk cycles (`sim.cycles()`), one per batch (clk and dspClk are
# both 10 ns on the bench). That is the same delta `batch_time()` gave while refTime never reset, and it
# stays monotonic across a qubic3 S0 hardware flush, whose pl_resetn0 pulse restarts refTime.

_batch_log: list[tuple[str, int]] = []

# qubic3 S0 r1 (plan P4 v2 §4.6): on an antq_uplink build, after a run that was not queue-proven (a test
# kernel without a completion marker) the next release takes the hardware flush. One flush costs about
# 6.1 k co-sim batches on a 2-core build (the bench's pl_resetn0, ~14 uplink register reads at an idle
# tick each, the 2 048-batch settle of the quiet check, ~800 more per extra core). A co-sim test starts
# from a session with no flush pending, so it never pays for an earlier test's run, and every flush its
# own runs take adds FLUSH_ALLOWANCE to its cap.
FLUSH_ALLOWANCE = 8_000
_FLUSHED_FIXTURES = ("cosim", "cosim_2q1c", "cosim_antq", "cosim_mm", "cosim_dio")


def _flushes(drv) -> int:
    s = getattr(drv, "_rq_session", None)
    return 0 if s is None else len(s.flushes)


def _start_flushed(drv, m) -> None:
    s = getattr(drv, "_rq_session", None)
    if s is not None and s.pending_flush is not None and s.poisoned is None:
        from riscq import run as rq
        rq.hardware_flush(drv, m, s.pending_flush.reason)


@pytest.fixture(autouse=True)
def sim_batches(request):
    """Record the simulated batches each co-sim test costs; optionally enforce a cap.

    Inert for host-pure tests: it only engages when a co-sim fixture is already in the test's
    fixture closure (directly or transitively, e.g. through `sub` / `remote` / `demod_phase`),
    so it never starts a simulator that the test did not ask for.
    """
    names = [n for n in ("cosim", "cosim_2q1c", "cosim_antq") if n in request.fixturenames]
    for n in (n for n in _FLUSHED_FIXTURES if n in request.fixturenames):
        _start_flushed(*request.getfixturevalue(n))
    if not names:
        yield
        return
    drvs = [request.getfixturevalue(n)[0] for n in names]
    before = [d.sim.cycles() for d in drvs]
    flushes = [_flushes(d) for d in drvs]
    yield
    spent = sum(max(0, d.sim.cycles() - t0) for d, t0 in zip(drvs, before))
    flushed = sum(_flushes(d) - f0 for d, f0 in zip(drvs, flushes))
    _batch_log.append((request.node.nodeid, spent))
    cap = request.config.getoption("--batch-cap")
    override = request.node.get_closest_marker("batch_cap")
    if override:                       # a documented structural floor, not a licence to be slow
        cap = int(override.args[0])
    if cap and flushed:                # the §4.6 flushes of the test's own unproven runs
        cap += flushed * FLUSH_ALLOWANCE
    if cap and spent > cap and "slow" not in request.node.keywords:   # anchors are not budgeted
        pytest.fail(f"simulated {spent:,} batches, over the {cap:,} cap"
                    f"{f' ({flushed} hardware flushes allowed for)' if flushed else ''} "
                    f"(~{spent / 7000:.1f}s of co-sim). Cut points/shots, shrink the relax head, "
                    f"or move the assertion to a cheaper tier "
                    f"(specs/software-test-refactor/01-test-tiers.md).", pytrace=False)


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    if not _batch_log:
        return
    total = sum(n for _, n in _batch_log)
    tr = terminalreporter
    tr.write_sep("=", "simulated co-sim batches")
    tr.write_line("(seconds are an upper bound: 7k batches/s with an ADC model attached, "
                  "11.5k with it off)")
    for nodeid, n in sorted(_batch_log, key=lambda kv: -kv[1])[:25]:
        tr.write_line(f"{n:>12,}  (<={n / 7000:6.1f}s)  {nodeid}")
    tr.write_line(f"{total:>12,}  TOTAL over {len(_batch_log)} tests "
                  f"(<={total / 7000 / 60:.1f} min of simulated RTL)")


@pytest.fixture(autouse=True)
def _dma_quarantine():
    """qubic3 r3: the DMA buffer quarantine (`riscq.board.ddr_board`) is process-wide; each test
    leaves it as it found it, so a test's deliberately stuck channel cannot refuse a later attach."""
    from riscq.board import ddr_board
    saved = {k: list(v) for k, v in ddr_board._QUARANTINE.items()}
    yield
    ddr_board._QUARANTINE.clear()
    ddr_board._QUARANTINE.update(saved)


@pytest.fixture(scope="session")
def socmap():
    """The sim-2q `SocMap`, without starting a simulator — the host-pure tests derive every code
    and grid from it exactly as the co-sim ones do."""
    from riscq.map import SocMap, SocParams

    return SocMap(SocParams.load(CONFIGS / "sim-2q.json"))


@pytest.fixture
def responder(monkeypatch):
    """Factory for the host-pure calibration harness (specs/software-test-refactor/01 §2.2):
    `responder(config_json_path)` → a `Responder` whose `.drv` satisfies `socmap(drv)`."""
    from tests.responder import Responder

    def make(config_json) -> "Responder":
        return Responder(monkeypatch, Path(config_json).read_text())

    return make


@pytest.fixture(scope="session")
def cosim(request):
    """A running verilator co-sim of the sim-2q build: (CosimDriver, SocMap)."""
    if not request.config.getoption("--cosim"):
        pytest.skip("needs --cosim")
    from riscq.map import SocMap, SocParams
    from riscq.sim import server

    drv = server.start(*_cosim_build(request, "sim-2q"))
    m = SocMap(SocParams.from_json(drv.sim.get_params()))
    yield drv, m
    server.stop(drv)


@pytest.fixture(scope="session")
def cosim_2q1c(request):
    """A running verilator co-sim of the sim-2q1c (3-core: 2 qubits + 1 coupler) build, with the
    explicit dac_map/adc_map (specs/two-qubit/01 §1): (CosimDriver, SocMap)."""
    if not request.config.getoption("--cosim"):
        pytest.skip("needs --cosim")
    from riscq.map import SocMap, SocParams
    from riscq.sim import server

    drv = server.start(*_cosim_build(request, "sim-2q1c"))
    m = SocMap(SocParams.from_json(drv.sim.get_params()))
    yield drv, m
    server.stop(drv)


@pytest.fixture(scope="session")
def cosim_mm(request):
    """A running verilator co-sim of the sim-mm build (universal-control/01 P3): core 0 has five
    channels (gate / f0g1 / flux / ro / demod), core 1 the plain three: (CosimDriver, SocMap)."""
    if not request.config.getoption("--cosim"):
        pytest.skip("needs --cosim")
    from riscq.map import SocMap, SocParams
    from riscq.sim import server

    drv = server.start(*_cosim_build(request, "sim-mm"))
    m = SocMap(SocParams.from_json(drv.sim.get_params()))
    yield drv, m
    server.stop(drv)


@pytest.fixture(scope="session")
def cosim_antq(request):
    """A running verilator co-sim of the sim-dio-antq build: the sim-dio cores (core 0 with a timed-DIO
    bank, core 1 on its own ADC) with results_path antq_uplink, so the Ant-Q uplink with a modelled PL
    DDR4 and S2MM DMA replaces the HostWindow: (CosimDriver, SocMap)."""
    if not request.config.getoption("--cosim"):
        pytest.skip("needs --cosim")
    from riscq.map import SocMap, SocParams
    from riscq.sim import server

    drv = server.start(CONFIGS / "sim-dio-antq.json", SW_ROOT / "build" / "sim-dio-antq")
    m = SocMap(SocParams.from_json(drv.sim.get_params()))
    yield drv, m
    server.stop(drv)


@pytest.fixture(scope="session")
def cosim_dio(request):
    """A running verilator co-sim of the sim-dio build (universal-control/01 P5): core 0 carries a
    timed-DIO bank `ttl` next to gate / ro / demod: (CosimDriver, SocMap). Under --results-path
    antq_uplink its sibling is the sim-dio-antq build `cosim_antq` already runs, so that session is shared."""
    if not request.config.getoption("--cosim"):
        pytest.skip("needs --cosim")
    if request.config.getoption("--results-path") == "antq_uplink":
        yield request.getfixturevalue("cosim_antq")
        return
    from riscq.map import SocMap, SocParams
    from riscq.sim import server

    drv = server.start(CONFIGS / "sim-dio.json", SW_ROOT / "build" / "sim-dio")
    m = SocMap(SocParams.from_json(drv.sim.get_params()))
    yield drv, m
    server.stop(drv)
