"""pynq compatibility fixes shared by the board drivers (`PynqDriver`, `DdrBoard`). Importable without
pynq, so it can be unit-tested off the board."""

from __future__ import annotations

import logging

log = logging.getLogger(__name__)


def numpy2_pynq_shim() -> bool:
    """Make `pynq.allocate()` work on numpy >= 2 with pynq 3.0.0. Returns True if it patched pynq.

    numpy 2.0 gave `ndarray` a READ-ONLY `device` property (array-API standard). pynq 3.0.0's
    `PynqBuffer.__new__` assigns `self.device = device`, so on any board whose venv has numpy >= 2
    every `pynq.allocate()` dies with
    `AttributeError: attribute 'device' of 'numpy.ndarray' objects is not writable`
    -- measured on veneno, pynq 3.0.0 / numpy 2.2.6.

    A plain class attribute on the SUBCLASS shadows the base class's getset descriptor, so the
    assignment lands in the instance __dict__ again and `buf.device` still reads back the
    EmbeddedDevice. This runs in-process and writes nothing to the pynq installation, which is
    shared with the other project that uses this board. It must run before the first allocation in
    the process -- `PynqDriver.__init__` calls it first thing, `DdrBoard` before its drain buffer.
    """
    try:
        import numpy
        import pynq.buffer
    except ImportError:      # a pynq without `buffer` is the unit tests' stand-in; nothing to shim
        return False
    if "device" not in pynq.buffer.PynqBuffer.__dict__ and hasattr(numpy.ndarray, "device"):
        pynq.buffer.PynqBuffer.device = None
        log.info("applied the numpy>=2 PynqBuffer.device shim (numpy %s)", numpy.__version__)
        return True
    return False
