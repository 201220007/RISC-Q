`timescale 1ns/1ps
// cbuf_poller_tb — G1(b) unit-test wrapper: roll_poll_reader2 -> skid FIFO -> circular_buffer3 (+cbuf_ram_read_wider).
//
// Models the SpinalHDL glue of PLAN_READOUT_DDR (v2 §2.2 / v3 §B.2) in RTL so that the gating under test is the
// gating the SoC will use:
//   * per-channel sources  : driven from Python (data_buffer contract: valid holds until rd_en, write priority over clear)
//   * throttle             : poller.data_valid[i] = ch_valid[i] && !throttle, throttle = (skid_count >= THROTTLE_AT)
//   * skid FIFO            : depth SKID_DEPTH, push = poller.wr_en (never back-pressured; sticky skid_overflow if full),
//                            pop  = skid_valid && cbuf.wr_ready
//   * cbuf.wr_en           : skid_valid && wr_ready   (== "poller-side wr_en && wr_ready", the Fork A contract)
//   * tb override          : tb_ovr_sel=1 disconnects the skid and drives cbuf.wr_en/wr_data straight from Python,
//                            so a test can assert wr_en while wr_ready=0 and prove nothing happens (Fork A defensive gate).
// The read side (rd_clk) is driven from a Python model of circular_buffer_axi_writer's ST_IDLE/READ/read_finished protocol.
// Debug outputs are hierarchical reads of cbuf/RAM state (metadata-freeze evidence during a stall).
module cbuf_poller_tb #(
    parameter integer NUM_CH        = 14,
    parameter integer DATA_WIDTH    = 64,
    parameter integer RD_DATA_WIDTH = 256,
    parameter integer ADDR_WIDTH    = 4,
    parameter integer SKID_DEPTH    = 8,
    parameter integer THROTTLE_AT   = 5      // = SKID_DEPTH-3 (plan v3 §B.2)
)(
    input  wire                         wr_clk,
    input  wire                         wr_rst_n,
    input  wire                         rd_clk,
    input  wire                         rd_rst_n,

    // ---- channel sources (Python data_buffer model), wr_clk domain ----
    input  wire [NUM_CH-1:0]            ch_valid,
    input  wire [NUM_CH*DATA_WIDTH-1:0] ch_data,
    output wire [NUM_CH-1:0]            rd_en,              // poller one-hot pop
    output wire [NUM_CH-1:0]            poller_data_valid,  // after throttle mask
    input  wire                         N_shot_finished,    // tied 0 by tests (plan: poller flush disabled)
    input  wire                         write_finished_ext,

    // ---- poller -> skid ----
    output wire                         poller_wr_en,
    output wire [DATA_WIDTH-1:0]        poller_wr_data,
    output wire                         poller_write_finished_external,
    output reg  [$clog2(SKID_DEPTH):0]  skid_count,
    output wire                         throttle,
    output reg                          skid_overflow,      // sticky: push while full (must never happen)

    // ---- test override of the cbuf write port ----
    input  wire                         tb_ovr_sel,
    input  wire                         tb_ovr_wr_en,
    input  wire [DATA_WIDTH-1:0]        tb_ovr_wr_data,

    // ---- cbuf write side ----
    output wire                         cbuf_wr_en,         // what cbuf actually sees
    output wire [DATA_WIDTH-1:0]        cbuf_wr_data,
    output wire                         wr_ready,
    output wire                         wr_en_out,          // = wr_accept (also the RAM weA)
    output wire                         write_finished_out,
    output wire                         write_almost_finished_out,

    // ---- cbuf read side (rd_clk) ----
    output wire                         able_to_read_out,
    output wire                         rd_empty,
    output wire [ADDR_WIDTH-1:0]        rd_addr_valid_out,
    input  wire [ADDR_WIDTH-1:0]        rd_addr,
    input  wire                         cbuf_rd_en,
    input  wire                         read_finished,
    output wire [RD_DATA_WIDTH-1:0]     rd_data,

    // ---- debug (hierarchical reads; wr_clk domain unless noted) ----
    output wire [ADDR_WIDTH+1:0]        dbg_wr_addr,        // WR_ADDR_WIDTH = ADDR_WIDTH + log2(RATIO) = ADDR_WIDTH+2
    output wire                         dbg_bank_sel_wr,
    output wire                         dbg_bank_sel_rd,
    output wire                         dbg_credit,         // read_finished_reg_wr
    output wire [ADDR_WIDTH-1:0]        dbg_lva0,
    output wire [ADDR_WIDTH-1:0]        dbg_lva1,
    output wire                         dbg_empty0,
    output wire                         dbg_empty1,
    output wire                         dbg_ram_we_d,       // RAM write strobe (registered stage inside cbuf_ram_read_wider)
    output wire                         dbg_rd_bank_sel     // rd_clk domain: bank the reader is looking at
);

    // ------------------------------------------------------------------
    // Poller
    // ------------------------------------------------------------------
    assign throttle          = (skid_count >= THROTTLE_AT);
    assign poller_data_valid = ch_valid & {NUM_CH{~throttle}};

    roll_poll_reader2 #(.NUM_CH(NUM_CH), .DATA_WIDTH(DATA_WIDTH)) u_poller (
        .clk                     (wr_clk),
        .rst_n                   (wr_rst_n),
        .data_valid              (poller_data_valid),
        .data_in                 (ch_data),
        .write_almost_finished   (1'b0),
        .N_shot_finished         (N_shot_finished),
        .wr_en                   (poller_wr_en),
        .wr_data                 (poller_wr_data),
        .rd_en                   (rd_en),
        .write_finished_external (poller_write_finished_external)
    );

    // ------------------------------------------------------------------
    // Skid FIFO (registered, depth SKID_DEPTH) — stands in for StreamFifo(64b, D=8)
    // ------------------------------------------------------------------
    localparam integer SKID_AW = $clog2(SKID_DEPTH);
    reg [DATA_WIDTH-1:0] skid_mem [0:SKID_DEPTH-1];
    reg [SKID_AW-1:0]    skid_rp, skid_wp;
    wire skid_valid = (skid_count != 0);
    wire skid_full  = (skid_count == SKID_DEPTH);
    wire skid_push  = poller_wr_en && !skid_full;
    wire skid_pop   = skid_valid && wr_ready && !tb_ovr_sel;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            skid_rp       <= {SKID_AW{1'b0}};
            skid_wp       <= {SKID_AW{1'b0}};
            skid_count    <= {(SKID_AW+1){1'b0}};
            skid_overflow <= 1'b0;
        end else begin
            if (poller_wr_en && skid_full) skid_overflow <= 1'b1;
            if (skid_push) begin
                skid_mem[skid_wp] <= poller_wr_data;
                skid_wp           <= skid_wp + 1'b1;
            end
            if (skid_pop) skid_rp <= skid_rp + 1'b1;
            skid_count <= skid_count + {{SKID_AW{1'b0}}, skid_push} - {{SKID_AW{1'b0}}, skid_pop};
        end
    end

    // ------------------------------------------------------------------
    // cbuf write port: glue gating (skid.pop.valid && wr_ready) or tb override
    // ------------------------------------------------------------------
    assign cbuf_wr_en   = tb_ovr_sel ? tb_ovr_wr_en   : skid_pop;
    assign cbuf_wr_data = tb_ovr_sel ? tb_ovr_wr_data : skid_mem[skid_rp];

    circular_buffer3 #(
        .WR_DATA_WIDTH (DATA_WIDTH),
        .RD_DATA_WIDTH (RD_DATA_WIDTH),
        .ADDR_WIDTH    (ADDR_WIDTH)
    ) u_cbuf (
        .wr_clk                    (wr_clk),
        .wr_rst_n                  (wr_rst_n),
        .rd_clk                    (rd_clk),
        .rd_rst_n                  (rd_rst_n),
        .wr_en                     (cbuf_wr_en),
        .wr_data                   (cbuf_wr_data),
        .write_finished_ext        (write_finished_ext),
        .rd_en                     (cbuf_rd_en),
        .rd_addr                   (rd_addr),
        .read_finished             (read_finished),
        .rd_data                   (rd_data),
        .wr_en_out                 (wr_en_out),
        .write_finished_out        (write_finished_out),
        .write_almost_finished_out (write_almost_finished_out),
        .able_to_read_out          (able_to_read_out),
        .rd_addr_valid_out         (rd_addr_valid_out),
        .rd_empty                  (rd_empty),
        .wr_ready                  (wr_ready)
    );

    // ------------------------------------------------------------------
    // Debug taps
    // ------------------------------------------------------------------
    assign dbg_wr_addr     = u_cbuf.wr_addr;
    assign dbg_bank_sel_wr = u_cbuf.bank_sel_wr;
    assign dbg_bank_sel_rd = u_cbuf.bank_sel_rd;
    assign dbg_credit      = u_cbuf.read_finished_reg_wr;
    assign dbg_lva0        = u_cbuf.last_valid_addr[0];
    assign dbg_lva1        = u_cbuf.last_valid_addr[1];
    assign dbg_empty0      = u_cbuf.buffer_empty[0];
    assign dbg_empty1      = u_cbuf.buffer_empty[1];
    assign dbg_ram_we_d    = u_cbuf.u_ram.weA_d;
    assign dbg_rd_bank_sel = u_cbuf.rd_bank_sel;

endmodule
