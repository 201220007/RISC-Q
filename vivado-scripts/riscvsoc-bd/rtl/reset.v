`timescale 1ns / 1ps
//
// reset — hold `rst` HIGH for the first N clocks after configuration, then LOW forever.
//
// Implemented as a saturating counter: ceil(log2(N+1)) FFs, not N. The original form was an
// N-bit shift register with `rst = |shift`, which for the N=5000 the DDR flow asks for cost
// 5000 FFs and 1000 LUTs (an OR-reduction over 5000 bits) — measured in the 14q feature-ON
// build, where CLB occupancy hit 89.7% and timing missed by 0.058 ns. Same behaviour, ~0.3%
// of the area.
//
module reset
#(  parameter integer N = 1000                // stretch length, in clocks
 )( input  wire clk,                          // clock to count on
    (* X_INTERFACE_INFO     = "xilinx.com:signal:reset:1.0 RST RST",
       X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH"            *)
    output wire rst                           // stretched reset
 );

    localparam integer W = (N <= 1) ? 1 : $clog2(N + 1);

    // Starts at 0 and counts to N, then parks. `rst` is HIGH while cnt < N, i.e. for exactly N
    // rising edges — the same count of asserted cycles the shift register produced.
    reg [W-1:0] cnt = {W{1'b0}};
    reg         run = 1'b1;

    always @(posedge clk) begin
        if (run) begin
            if (cnt == N[W-1:0] - 1'b1) run <= 1'b0;
            else                        cnt <= cnt + 1'b1;
        end
    end

    assign rst = run;
endmodule
