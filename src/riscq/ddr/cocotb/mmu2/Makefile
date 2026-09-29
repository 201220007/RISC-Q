TOPLEVEL = mmu2
COCOTB_TEST_MODULES = test_mmu2
# NOTE: VERILOG_SOURCES must be set BEFORE the include: cocotb's Makefile.sim expands the
# Vtop.mk prerequisites at parse time, so sources listed after it never trigger a rebuild.
VERILOG_SOURCES = $(RTL)/mmu2.v
include ../Makefile.inc
