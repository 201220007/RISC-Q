import cocotb
from cocotb.triggers import RisingEdge
from ddrtb import start_clock, reset_low, seed

@cocotb.test()
async def smoke(dut):
    seed(dut)
    await start_clock(dut.clk, 2)
    dut.data_valid.value = 0; dut.data_in.value = 0
    dut.write_almost_finished.value = 0; dut.N_shot_finished.value = 0
    await reset_low(dut.rst_n, dut.clk)
    for _ in range(20):
        await RisingEdge(dut.clk)
    assert int(dut.wr_en.value) == 0
    dut._log.info("harness OK: roll_poll_reader2 NUM_CH=14 elaborates under Verilator+cocotb 2.0")
