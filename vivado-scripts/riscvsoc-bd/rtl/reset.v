`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/16/2025 03:46:57 PM
// Design Name: 
// Module Name: reset
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module reset
#(  parameter integer N = 1000               // stretch length
 )( input  wire clk,                          // clock to count on
    (* X_INTERFACE_INFO     = "xilinx.com:signal:reset:1.0 RST RST",
       X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH"            *)
    output wire rst                           // stretched reset
 );

    // N-bit shift register initialised to all 1's
    reg [N-1:0] shift = {N{1'b1}};

    always @(posedge clk)
        shift <= {shift[N-2:0], 1'b0};        // shift-in a zero

    assign rst = |shift;                      // HIGH while any '1'
endmodule
