# Shared cocotb 2.0 + Verilator harness for the vendored riscq.ddr RTL unit tests.
# Each test dir sets: TOPLEVEL, VERILOG_SOURCES (relative to RTL), COCOTB_TEST_MODULES, optional EXTRA_ARGS.
# Run from a test dir:  make            (conda env qubic_clean must be on PATH: cocotb 2.0.1, cocotbext-axi)
SIM ?= verilator
TOPLEVEL_LANG ?= verilog
RTL := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../rtl)
COMMON := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/common)
export PYTHONPATH := $(COMMON):$(PYTHONPATH)
EXTRA_ARGS += -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-GENUNNAMED \
              -Wno-TIMESCALEMOD -Wno-PINCONNECTEMPTY -Wno-SYNCASYNCNET -Wno-PROCASSINIT -Wno-UNOPTFLAT \
              -Wno-DECLFILENAME --timescale 1ns/1ps
WAVES ?= 0
ifeq ($(WAVES),1)
EXTRA_ARGS += --trace-fst --trace-structs
endif
COCOTB_RESOLVE_X ?= ZEROS
include $(shell cocotb-config --makefiles)/Makefile.sim
