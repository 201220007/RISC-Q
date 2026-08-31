`timescale 1 ns / 1 ps
////////////////////////////////////////////////////////////
// fifo_sc.v - simple single-clock FIFO (block-RAM based) //
////////////////////////////////////////////////////////////
module async_fifo_same #(
    parameter integer DW    = 512,   // data width
    parameter integer DEPTH = 1024,  // number of words (power-of-2 recommended)
    localparam ADDR_W = $clog2(DEPTH)
)(
    input  wire              clk,
    input  wire              rstn,
    // write port
    input  wire              wr_en,
    input  wire [DW-1:0]     wr_data,
    output wire              full,
    output wire              almost_full,
    // read port (same clock)
    input  wire              rd_en,
    output wire [DW-1:0]     rd_data,
    output wire              empty
);
    // storage - will be inferred as block RAM
    (* ram_style = "block" *)
    reg [DW-1:0] mem [0:DEPTH-1];

    reg [ADDR_W:0] used;               // one bit wider than needed
    reg [ADDR_W-1:0] wptr, rptr;

    assign full        = (used == DEPTH);
    assign almost_full = (used >= DEPTH-4);   // 3-word hysteresis
    assign empty       = (used == 0);
    assign rd_data     = mem[rptr];

    // RAM write — separate always block without async reset for BRAM inference
    always @(posedge clk) begin
        if (wr_en && !full)
            mem[wptr] <= wr_data;
    end

    // Pointers and counter — with async reset
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            used <= 0;
            wptr <= 0;
            rptr <= 0;
        end else begin
            if (wr_en && !full)
                wptr <= wptr + 1'b1;
            if (rd_en && !empty)
                rptr <= rptr + 1'b1;
            case ({wr_en && !full, rd_en && !empty})
                2'b10: used <= used + 1'b1;
                2'b01: used <= used - 1'b1;
                default: ;
            endcase
        end
    end
endmodule
