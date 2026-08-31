# G1 'writer' suite: circular_buffer_axi_writer (forks C/C2/C3/C4) against the real circular_buffer3 read side
# and a cocotbext-axi AxiRamWrite with random AW/W/B backpressure.  See REPORT.md.
TOPLEVEL = writer_tb_top
COCOTB_TEST_MODULES = test_writer
# NOTE: VERILOG_SOURCES / EXTRA_ARGS must be set BEFORE the include -- cocotb's Makefile.sim expands the
# "$(SIM_BUILD)/Vtop.mk: $(VERILOG_SOURCES)" prerequisites when parsed, so sources defined after the include
# (as in smoke/) never trigger a rebuild after an RTL or wrapper edit.
RTL_DIR := $(abspath $(CURDIR)/../../rtl)
VERILOG_SOURCES = $(RTL_DIR)/cbuf_ram_read_wider.v $(RTL_DIR)/circular_buffer3.v $(RTL_DIR)/circular_buffer_axi_writer.v \
                  $(CURDIR)/writer_tb_top.sv
# Diagnostic ring limit (RTL-supported define): WRAP_LIMIT = 0xF9FF, WRAP_SIZE = 0xFA00 = 125 x 512-B banks.
# 0xFA00 is 512-B aligned but NOT 4 KiB aligned, so the overrun/boundary bursts never cross a 4 KiB page
# (cocotbext-axi asserts on 4 KiB crossings, which would otherwise mask the WRAP_LIMIT scenarios).
SIM_WRAP_LIMIT ?= 63999
EXTRA_ARGS += -DSIM_WRAP_LIMIT=$(SIM_WRAP_LIMIT)
include ../Makefile.inc
