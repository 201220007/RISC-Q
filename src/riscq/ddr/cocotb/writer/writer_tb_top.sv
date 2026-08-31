// writer_tb_top.sv -- cocotb wrapper for the G1 'writer' suite.
//
// DUT = circular_buffer_axi_writer (forks C/C2/C3/C4).  The circular-buffer READ-side contract
// (able_to_read / rd_empty / rd_addr_valid / rd_addr / rd_en / read_finished, rd_data 1 cycle after
// rd_addr) is NOT re-modelled in Python: the real circular_buffer3 + cbuf_ram_read_wider are
// instantiated here so the writer talks to the exact RTL it will meet on the board.  The cbuf WRITE
// side is driven from Python (wr_en/wr_data honouring Fork-A wr_ready).  One clock feeds wr_clk,
// rd_clk and the AXI side (justification in REPORT.md).  The dbg_* outputs expose the cbuf bank /
// address pointers and the writer FSM state for the Python scoreboard only; nothing is fed back.
`timescale 1ns/1ps
module writer_tb_top #(
    parameter WR_DATA_WIDTH  = 64,
    parameter RD_DATA_WIDTH  = 256,
    parameter ADDR_WIDTH     = 4,
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_ID_WIDTH   = 4
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // ---- circular_buffer3 write side (Python driver) ----
    input  wire                       wr_en,
    input  wire [WR_DATA_WIDTH-1:0]   wr_data,
    output wire                       wr_ready,
    input  wire                       cb_write_finished_ext,     // flush -> cbuf (wr domain)
    output wire                       wr_en_out,
    output wire                       cb_write_finished_out,
    output wire                       cb_write_almost_finished_out,

    // ---- writer control / status ----
    input  wire [AXI_ADDR_WIDTH-1:0]  base_addr,
    input  wire                       base_reset,
    input  wire                       wr_write_finished_ext,     // flush -> writer (rd domain)
    output wire [AXI_ADDR_WIDTH-1:0]  final_addr,
    output wire [AXI_ADDR_WIDTH-1:0]  cur_axi_addr_out,
    output wire                       writer_idle,
    output wire                       addr_fault,
    output wire                       current_user_done,

    // ---- cbuf <-> writer read-side contract (observability) ----
    output wire                       able_to_read,
    output wire                       rd_empty,
    output wire [ADDR_WIDTH-1:0]      rd_addr_valid,
    output wire [ADDR_WIDTH-1:0]      rd_addr,
    output wire                       rd_en,
    output wire                       read_finished,
    output wire [RD_DATA_WIDTH-1:0]   rd_data,

    // ---- white-box pointers (scoreboard only) ----
    output wire                       dbg_wr_bank,
    output wire [ADDR_WIDTH+1:0]      dbg_wr_addr,               // WR_ADDR_WIDTH = ADDR_WIDTH + log2(256/64)
    output wire                       dbg_rd_bank,
    output wire [2:0]                 dbg_state,
    input  wire [ADDR_WIDTH+2:0]      dbg_ram_addr,              // {bank, wr_addr}: peek the BRAM image
    output wire [WR_DATA_WIDTH-1:0]   dbg_ram_q,                 // (the array is not cleared by rst_n)

    // ---- AXI4 write master (to cocotbext.axi AxiRamWrite) ----
    output wire [AXI_ID_WIDTH-1:0]    m_axi_awid,
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr,
    output wire [7:0]                 m_axi_awlen,
    output wire [2:0]                 m_axi_awsize,
    output wire [1:0]                 m_axi_awburst,
    output wire                       m_axi_awlock,
    output wire [3:0]                 m_axi_awcache,
    output wire [2:0]                 m_axi_awprot,
    output wire                       m_axi_awvalid,
    input  wire                       m_axi_awready,
    output wire [RD_DATA_WIDTH-1:0]   m_axi_wdata,
    output wire [RD_DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire                       m_axi_wlast,
    output wire                       m_axi_wvalid,
    input  wire                       m_axi_wready,
    input  wire [AXI_ID_WIDTH-1:0]    m_axi_bid,
    input  wire [1:0]                 m_axi_bresp,
    input  wire                       m_axi_bvalid,
    output wire                       m_axi_bready
);

    assign m_axi_awid    = {AXI_ID_WIDTH{1'b0}};
    assign m_axi_awlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot  = 3'b000;

    circular_buffer3 #(
        .WR_DATA_WIDTH (WR_DATA_WIDTH),
        .RD_DATA_WIDTH (RD_DATA_WIDTH),
        .ADDR_WIDTH    (ADDR_WIDTH)
    ) u_cbuf (
        .wr_clk                    (clk),
        .wr_rst_n                  (rst_n),
        .rd_clk                    (clk),
        .rd_rst_n                  (rst_n),
        .wr_en                     (wr_en),
        .wr_data                   (wr_data),
        .write_finished_ext        (cb_write_finished_ext),
        .rd_en                     (rd_en),
        .rd_addr                   (rd_addr),
        .read_finished             (read_finished),
        .rd_data                   (rd_data),
        .wr_en_out                 (wr_en_out),
        .write_finished_out        (cb_write_finished_out),
        .write_almost_finished_out (cb_write_almost_finished_out),
        .able_to_read_out          (able_to_read),
        .rd_addr_valid_out         (rd_addr_valid),
        .rd_empty                  (rd_empty),
        .wr_ready                  (wr_ready)
    );

    circular_buffer_axi_writer #(
        .RD_DATA_WIDTH  (RD_DATA_WIDTH),
        .ADDR_WIDTH     (ADDR_WIDTH),
        .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH)
    ) u_writer (
        .clk                (clk),
        .rst_n              (rst_n),
        .base_addr          (base_addr),
        .base_reset         (base_reset),
        .able_to_read       (able_to_read),
        .rd_empty           (rd_empty),
        .rd_addr_valid      (rd_addr_valid),
        .rd_addr            (rd_addr),
        .rd_en              (rd_en),
        .read_finished      (read_finished),
        .write_finished_ext (wr_write_finished_ext),
        .final_addr         (final_addr),
        .cur_axi_addr_out   (cur_axi_addr_out),
        .writer_idle        (writer_idle),
        .addr_fault         (addr_fault),
        .current_user_done  (current_user_done),
        .rd_data            (rd_data),
        .m_axi_awaddr       (m_axi_awaddr),
        .m_axi_awlen        (m_axi_awlen),
        .m_axi_awsize       (m_axi_awsize),
        .m_axi_awburst      (m_axi_awburst),
        .m_axi_awvalid      (m_axi_awvalid),
        .m_axi_awready      (m_axi_awready),
        .m_axi_wdata        (m_axi_wdata),
        .m_axi_wstrb        (m_axi_wstrb),
        .m_axi_wlast        (m_axi_wlast),
        .m_axi_wvalid       (m_axi_wvalid),
        .m_axi_wready       (m_axi_wready),
        .m_axi_bresp        (m_axi_bresp),
        .m_axi_bvalid       (m_axi_bvalid),
        .m_axi_bready       (m_axi_bready)
    );

    // white-box taps (read-only observation of the real cbuf pointers / writer FSM)
    assign dbg_wr_bank = u_cbuf.bank_sel_wr;
    assign dbg_wr_addr = u_cbuf.wr_addr;
    assign dbg_rd_bank = u_cbuf.rd_bank_sel_sync2;
    assign dbg_state   = u_writer.state;
    assign dbg_ram_q   = u_cbuf.u_ram.RAM[dbg_ram_addr];

endmodule
