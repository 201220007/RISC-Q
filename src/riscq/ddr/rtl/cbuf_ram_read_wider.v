// cbuf_ram_read_wider.v — DEDICATED read-wider RAM for circular_buffer3 (readout circular buffer).
//
// WHY A SEPARATE MODULE (2026-07-01, beat-duplication root cause): the generic
// asym_ram_sdp_read_wider is SHARED — in the integrated (ddr_streaming) build the plps filelist
// basename-dedup resolves it to submodules/common-hdl's copy, whose read latency is 2 cycles
// (doB = readB_d), as the qubic dsp env/freq table timing requires. But circular_buffer_axi_writer
// advances rd_addr only ONE beat ahead (rd_addr = beat_count+1 on an accepted W beat), so a 2-cycle
// read ships beat N's data on beat N+1 under back-to-back WREADY — board-observed as DDR beat 1
// duplicating beat 0 (build 02317191, 2026-07-01). bitfile3 (bak29b2993) never hit this because its
// mmu was a SEPARATE IP compiling its own 1-cycle copy. This module pins the circular-buffer read
// to the 1-cycle contract (doB = readB) without touching the shared dsp RAM.
//
// READ LATENCY CONTRACT: doB = registered BRAM read = EXACTLY 1 clkB cycle from addrB.
// Guard test: plsv_sim tests/test_plsv_v2b_readout_beats.py (asserts both DDR beats, all 8 lanes).
//
// Body is the UG901 asymmetric-port RAM (read wider than write), identical to
// asym_ram_sdp_read_wider except the module name and the exposed read stage.

module cbuf_ram_read_wider (clkA, clkB, weA, addrA, addrB, diA, doB);
parameter integer DATAWIDTHA = 32;
parameter integer SIZEA = 16384;
parameter integer ADDRWIDTHA = 14;
parameter integer DATAWIDTHB = 512;
parameter integer SIZEB = 1024;
parameter integer ADDRWIDTHB = 10;
parameter RAM_STYLE= "block";
parameter INIT_FILE="";
input clkA;
input clkB;
input weA;
input [ADDRWIDTHA-1:0] addrA;
input [ADDRWIDTHB-1:0] addrB;
input [DATAWIDTHA-1:0] diA;
output [DATAWIDTHB-1:0] doB;
`define max(a,b) ((a) > (b)) ? (a) : (b)
`define min(a,b) ((a) < (b)) ? (a) : (b)
localparam maxSIZE = `max(SIZEA, SIZEB);
localparam maxWIDTH = `max(DATAWIDTHA, DATAWIDTHB);
localparam minWIDTH = `min(DATAWIDTHA, DATAWIDTHB);
localparam RATIO = maxWIDTH / minWIDTH;
localparam log2RATIO = $clog2(RATIO);
(* ram_style= RAM_STYLE ,cascade_height=2 *)
reg [minWIDTH-1:0] RAM [0:maxSIZE-1];
reg [DATAWIDTHB-1:0] readB=0;
reg weA_d=0;
reg [ADDRWIDTHA-1:0] addrA_d=0;
reg [DATAWIDTHA-1:0] diA_d=0;
always @(posedge clkA) begin
	weA_d<=weA;
	addrA_d<=addrA;
	diA_d<=diA;
	if (weA_d)
		RAM[addrA_d] <= diA_d;
end
always @(posedge clkB) begin : ramread
	integer i;
	reg [log2RATIO-1:0] lsbaddr;
	for (i = 0; i < RATIO; i = i+1) begin
		lsbaddr = i;
		if (log2RATIO>0)
			readB[(i+1)*minWIDTH-1 -: minWIDTH] <= RAM[{addrB, lsbaddr}];
		else
			readB[(i+1)*minWIDTH-1 -: minWIDTH] <= RAM[{addrB}];
	end
end
// 1-cycle read: doB = the registered BRAM output, NO extra pipeline stage (see contract above).
assign doB = readB;

// [qubic3 Fork B2] The upstream module had an `INIT_FILE!=""` branch that preloaded the array from a
// hex file. INIT_FILE is a PARAMETER, not a string literal, so SpinalSim's memory-preload path-rewrite
// pass (SimBootstraps.scala:1034, triggered by the mere substring of that system task on any line,
// comments included) cannot resolve it and corrupts the generated Verilog. This RAM is always
// instantiated with INIT_FILE="" (the circular buffer never preloads), so the branch is removed and the
// array is unconditionally zero-initialised. INIT_FILE is kept for parameter compatibility.
// NOTE: do not write that system-task name anywhere in this file, not even in a comment.
integer i;
initial begin
	for (i=0;i<maxSIZE;i=i+1)   // [qubic3 Fork B] was i<=SIZEA (one past the array)
		RAM[i]={minWIDTH{1'b0}};
end

endmodule
