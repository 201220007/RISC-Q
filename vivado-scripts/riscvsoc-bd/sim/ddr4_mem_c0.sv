//****************************************************************************************************
// ddr4_mem_c0 -- the DDR4 device behind the c0 MIG, for the G4 block-design simulation.
//
// WHY THIS EXISTS
//   `Simulation_Mode = BFM` on the DDR4 IP makes the **XiPhy primitives** behavioural. It does NOT
//   supply a memory: the controller still drives a real DDR4 device on the `c0_ddr4_*` pins. Leaving
//   those pins unconnected (which is what G4a did at first) means every write goes nowhere and every
//   read returns zeros -- which is exactly what run 6 showed: perfect burst framing, full byte strobes,
//   BRESP=OKAY, and an all-zero payload. A memory model is required in BOTH simulation modes.
//
// PROVENANCE
//   Transcribed from the DDR4 IP's OWN generated example design for THIS part -- `sim_tb_top.sv` under
//   `<build>/ddr4_example/ddr4_0_ex/imports/`, produced by `sim/gen-ddr4-model.tcl`. The template under
//   `$XILINX_VIVADO/data/ip/xilinx/ddr4_v2_2/data/dlib/.../tb/` must NOT be used directly: its
//   `` `define DDR4_16G_X8 ``/`DDR4_938_Timing` describe a different device. The generated wrapper for
//   MT40A1G8WE-075E defines `DDR4_8G_X8` / `DDR4_750_Timing` / `FIXED_2666`, and
//   `ddr4_sdram_model_wrapper.sv` (copied next to this file) is what pulls the packages in.
//
// GEOMETRY (from the generated sim_tb_top.sv, this part only)
//   DQ_WIDTH 32, DRAM_WIDTH 8  => NUM_PHYSICAL_PARTS 4, CLAMSHELL_PARTS 2, ODD_PARTS 0
//   RANK_WIDTH 1, CS_WIDTH 2 (clamshell: CS_n[0] the upper parts, CS_n[1] the lower, CA mirrored)
//   ADDR_WIDTH 17, CONFIGURED_DENSITY _8G, CA_MIRROR "ON"
//****************************************************************************************************
`timescale 1ps / 1ps

// The whole device model is pulled into THIS file's compilation unit, in dependency order. Two things
// make that the only reliable arrangement here:
//   * `ddr4_sdram_model_wrapper.sv` as GENERATED contains only `define`s (DDR4_8G_X8 / DDR4_750_Timing /
//     SILENT / FIXED_2666) -- unlike the Vivado TEMPLATE of the same name, which also includes the model.
//     A file of pure defines declares nothing, so Vivado drops it from the simulation compile order as
//     unused and its defines never reach anything (cost: two runs).
//   * `interface.sv` and `proj_package.sv` each include the wrapper themselves for those defines, and
//     `arch_package.sv` includes `arch_defines.v`, so the set is a small dependency graph rather than a
//     flat list. Ordering it here makes it explicit instead of relying on fileset order.
// `include_dirs` (set by run-sim-bd.tcl to <build>/ddr4_model_sim) is what resolves all of these.
`include "arch_package.sv"     // package arch_package -- UTYPE_density, _8G, timing tables
`include "proj_package.sv"     // package proj_package
`include "interface.sv"        // interface DDR4_if
`include "ddr4_model.sv"       // module ddr4_model (pulls in MemoryArray/StateTable/timing_tasks)

// xsim has no `tran` primitive ("Primitive \"tran\" is not supported"), which is why the generated
// testbench shorts the model's bidirectional pins with THIS two-line module instead -- two ports of the
// same name, an xsim idiom. It is declared inside the generated `sim_tb_top.sv`, not in any library, so
// it has to be declared here too (cost: one run each way).
`ifdef XILINX_SIMULATOR
module short(in1, in1);
  inout in1;
endmodule
`endif

module ddr4_mem_c0 (
  input  wire        ck_t,
  input  wire        ck_c,
  input  wire        act_n,
  input  wire [16:0] adr,
  input  wire [1:0]  ba,
  input  wire [1:0]  bg,
  input  wire [0:0]  cke,
  input  wire [1:0]  cs_n,
  input  wire [0:0]  odt,
  input  wire        reset_n,
  inout  wire [31:0] dq,
  inout  wire [3:0]  dqs_t,
  inout  wire [3:0]  dqs_c,
  inout  wire [3:0]  dm_n
);

  import arch_package::*;

  localparam ADDR_WIDTH          = 17;
  localparam DQ_WIDTH            = 32;
  localparam DRAM_WIDTH          = 8;
  localparam NUM_PHYSICAL_PARTS  = DQ_WIDTH / DRAM_WIDTH;      // 4
  localparam CLAMSHELL_PARTS     = NUM_PHYSICAL_PARTS / 2;     // 2
  localparam ODD_PARTS           = ((CLAMSHELL_PARTS*2) < NUM_PHYSICAL_PARTS) ? 1 : 0;
  localparam RANK_WIDTH          = 1;
  localparam CS_WIDTH            = 2;
  localparam CA_MIRROR           = "ON";
  localparam WR                  = 3'b100;
  localparam RD                  = 3'b101;
  parameter  UTYPE_density CONFIGURED_DENSITY = _8G;

  // `model_enable` gates the models past power-up, exactly as the example testbench does.
  bit  en_model;
  tri  model_enable = en_model;
  initial begin en_model = 1'b0; #205 en_model = 1'b1; end

  // ---- clamshell command/address mirroring (verbatim from the generated sim_tb_top.sv) ----
  reg [16:0] adr_sdram [1:0];
  reg [1:0]  ba_sdram  [1:0];
  reg [1:0]  bg_sdram  [1:0];
  always @(*) begin
    adr_sdram[0] <= adr;
    adr_sdram[1] <= (CA_MIRROR == "ON") ?
                      {adr[ADDR_WIDTH-1:14], adr[11], adr[12], adr[13], adr[10:9],
                       adr[7], adr[8], adr[5], adr[6], adr[3], adr[4], adr[2:0]} : adr;
    ba_sdram[0]  <= ba;
    ba_sdram[1]  <= (CA_MIRROR == "ON") ? {ba[0], ba[1]} : ba;
    bg_sdram[0]  <= bg;
    bg_sdram[1]  <= (CA_MIRROR == "ON" && DRAM_WIDTH != 16) ? {bg[0], bg[1]} : bg;
  end

  reg [17:0] DDR4_ADRMOD [1:0];
  genvar rnk;
  generate
    for (rnk = 0; rnk < CS_WIDTH; rnk++) begin : rankup
      always @(*)
        if (act_n)
          casez (adr_sdram[0][16:14])
            WR, RD:  DDR4_ADRMOD[rnk] = adr_sdram[rnk] & 18'h1C7FF;
            default: DDR4_ADRMOD[rnk] = adr_sdram[rnk];
          endcase
        else
          DDR4_ADRMOD[rnk] = adr_sdram[rnk];
    end
  endgenerate

  // ---- the devices ----
  genvar i, r, s;
  generate
    DDR4_if #(.CONFIGURED_DQ_BITS(8)) iDDR4 [0:(RANK_WIDTH*NUM_PHYSICAL_PARTS)-1] ();

    for (r = 0; r < RANK_WIDTH; r++) begin : memModels
      for (i = 0; i < NUM_PHYSICAL_PARTS; i++) begin : memModel
        ddr4_model #(.CONFIGURED_DQ_BITS(8), .CONFIGURED_DENSITY(CONFIGURED_DENSITY))
          ddr4_model (.model_enable(model_enable), .iDDR4(iDDR4[(r*NUM_PHYSICAL_PARTS)+i]));
      end
    end

    for (r = 0; r < RANK_WIDTH; r++) begin : tranDQ
      for (i = 0; i < NUM_PHYSICAL_PARTS; i++) begin : tranDQ1
        for (s = 0; s < 8; s++) begin : tranDQp
          short bidiDQ(iDDR4[(r*NUM_PHYSICAL_PARTS)+i].DQ[s], dq[s+i*8]);
        end
      end
    end

    for (r = 0; r < RANK_WIDTH; r++) begin : tranDQS
      for (i = 0; i < NUM_PHYSICAL_PARTS; i++) begin : tranDQS1
        short bidiDQS (iDDR4[(r*NUM_PHYSICAL_PARTS)+i].DQS_t, dqs_t[i]);
        short bidiDQS_(iDDR4[(r*NUM_PHYSICAL_PARTS)+i].DQS_c, dqs_c[i]);
        short bidiDM  (iDDR4[(r*NUM_PHYSICAL_PARTS)+i].DM_n,  dm_n[i]);
      end
    end

    // clamshell: the even parts take rank 0's command/address and CS_n[0], the odd parts rank 1's
    for (i = 0; i < (CLAMSHELL_PARTS+ODD_PARTS); i++) begin : upperparts
      assign iDDR4[i*2].BG      = bg_sdram[0];
      assign iDDR4[i*2].BA      = ba_sdram[0];
      assign iDDR4[i*2].ADDR_17 = (ADDR_WIDTH == 18) ? DDR4_ADRMOD[0][ADDR_WIDTH-1] : 1'b0;
      assign iDDR4[i*2].ADDR    = DDR4_ADRMOD[0][13:0];
      assign iDDR4[i*2].CS_n    = cs_n[0];
    end
    for (i = 0; i < CLAMSHELL_PARTS; i++) begin : lowerparts
      assign iDDR4[(i*2)+1].BG      = bg_sdram[1];
      assign iDDR4[(i*2)+1].BA      = ba_sdram[1];
      assign iDDR4[(i*2)+1].ADDR_17 = (ADDR_WIDTH == 18) ? DDR4_ADRMOD[1][ADDR_WIDTH-1] : 1'b0;
      assign iDDR4[(i*2)+1].ADDR    = DDR4_ADRMOD[1][13:0];
      assign iDDR4[(i*2)+1].CS_n    = cs_n[1];
    end

    for (r = 0; r < RANK_WIDTH; r++) begin : tranADCTL_RANKS
      for (i = 0; i < NUM_PHYSICAL_PARTS; i++) begin : tranADCTL
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].CK        = {ck_t, ck_c};
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].ACT_n     = act_n;
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].RAS_n_A16 = DDR4_ADRMOD[r][16];
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].CAS_n_A15 = DDR4_ADRMOD[r][15];
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].WE_n_A14  = DDR4_ADRMOD[r][14];
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].CKE       = cke[r];
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].ODT       = odt[r];
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].PARITY    = 1'b0;
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].TEN       = 1'b0;
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].ZQ        = 1'b1;
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].PWR       = 1'b1;
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].VREF_CA   = 1'b1;
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].VREF_DQ   = 1'b1;
        assign iDDR4[(r*NUM_PHYSICAL_PARTS)+i].RESET_n   = reset_n;
      end
    end
  endgenerate

endmodule
