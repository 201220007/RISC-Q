TOPLEVEL = cbuf_poller_tb
COCOTB_TEST_MODULES = test_cbuf_poller
# NOTE: VERILOG_SOURCES must be set BEFORE the include: cocotb's Makefile.sim expands the
# Vtop.mk prerequisites at parse time, so sources listed after it never trigger a rebuild.
VERILOG_SOURCES = $(RTL)/roll_poll_reader2.v $(RTL)/circular_buffer3.v $(CURDIR)/cbuf_poller_tb.sv
include ../Makefile.inc
EXTRA_ARGS += -GNUM_CH=14 -GDATA_WIDTH=64 -GRD_DATA_WIDTH=256 -GADDR_WIDTH=4 -GSKID_DEPTH=8 -GTHROTTLE_AT=5
