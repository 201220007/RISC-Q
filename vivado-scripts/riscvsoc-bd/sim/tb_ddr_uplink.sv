// =====================================================================================================
// G4 — block-design xsim of the readout->DDR uplink (qubic3 C1).
//
// This is the ONLY gate that runs the design with the real Vivado IP in the loop: the Zynq UltraScale+
// VIP as the PS, the DDR4 MIG (Simulation_Mode = BFM), the two SmartConnects and the S2MM AXI DMA. G2/G3
// use behavioural AXI models; everything BETWEEN the SoC's ports and the PS -- address decode, the
// ui_clk reset tree, SmartConnect burst legalisation, the DMA's AXIS->memory path -- exists only here.
//
// Stimulus is the built-in test injector (register-paced), so no RF is needed:
//   base_reset -> N x inj_fire -> flush -> program the DMA -> rd_start -> compare, byte-exactly, the
//   words the design puts on the wire at three points: the uplink AXIS, the DMA's M_AXI_S2MM master
//   port, and the HP0 slave port. PS-memory PERSISTENCE is a separate claim and is NOT made here (this
//   VIP does not update its read_mem() store from HP0 writes); it is settled on hardware in G6.
//
// P3b (results_path antq_uplink, 8300a1c): the same run on the new BD (smc_dma -> HP0 at 128-bit on ui_clk),
// then four hardware-backed phases (plan v2 r2 #12, P3a "carried into P3b"):
//   B  a SLVERR, then a DECERR, forced on the MIG's write response (smc_ddr M00 BRESP) -> bresp_err;
//   R  a SLVERR, then a DECERR, forced on the MIG's read data response (RRESP) during a drain -> rresp_err;
//   C  a DSP-domain reset in the middle of a run -> the run is gone and cannot be certified; the next is exact;
//   D  a DSP-domain reset in the middle of a DRAIN -> the S2MM packet is truncated (no TLAST), the DMA never
//      completes; the ddr_board recovery (timeout, S2MM soft reset) leaves a channel on which the next drain
//      is byte-exact.
// Each phase prints "[G4] ok-<phase>: ..."; the single "[G4] PASS:" line comes only after all of them.
//
// Instance path: tb_ddr_uplink.DUT.riscq_bd_i.zynq_ps.inst  (the VIP; see .../ip/riscq_bd_zynq_ps_0/sim).
// =====================================================================================================
`timescale 1ps / 1ps

module tb_ddr_uplink;

  // ---- register map (mirror of ReadoutDdrRegs / software/riscq/ddr_regs.py) ----------------------
  localparam CTRL       = 64'h9000_0000;
  localparam DMA        = 64'h9001_0000;
  localparam O_RD_START = 'h00, O_WR_BASE = 'h08, O_RUN_BASE = 'h0C;
  localparam O_RD_BASE  = 'h10, O_RD_SIZE = 'h14, O_FINAL_ADDR = 'h18;
  localparam O_BASE_RST = 'h24, O_FLUSH   = 'h28, O_STATUS = 'h2C;
  localparam O_INJ_REAL = 'h40, O_INJ_IMAG= 'h44, O_INJ_CORE = 'h48, O_INJ_FIRE = 'h4C;
  localparam O_NUM_CH   = 'h50, O_ACCEPTED= 'h100;
  localparam S_DSP_IN_RESET = 18;

  // r15-#8: every time limit in this file scales with the mode. A full-PHY run spends ~100 us in DDR4
  // calibration alone, so the G4a numbers would kill it before the test even starts.
`ifdef G4B_PHY
  localparam integer CALIB_LIMIT = 2_000_000;   // 2 ms of 1 ns polls
  localparam integer POLL_LIMIT  = 200_000;
  // r16-#4: 5e9 ps does NOT fit in a 32-bit `integer` -- it truncates to 705_032_704 ps (705 us), i.e.
  // the "5 ms" guard would have fired during calibration. Must be 64-bit.
  localparam longint unsigned WALL_PS = 64'd5_000_000_000;
`else
  // Even in BFM mode calibration is REAL now that a device model is attached (BFM only makes the XiPhy
  // behavioural), so these are sized for an actual calibration, just a faster one than Unisim's.
  localparam integer CALIB_LIMIT = 1_000_000;   // 1 ms
  localparam integer POLL_LIMIT  = 20_000;
  localparam longint unsigned WALL_PS = 64'd3_000_000_000;  // 3 ms
`endif
  localparam S_WRITE_DONE = 2, S_FLUSH_BUSY = 7, S_INJ_BUSY = 8, S_RUN_ACTIVE = 12, S_RD_DONE = 1;
  // Every bit that invalidates a run (mirror of FATAL_BITS in software/riscq/ddr_regs.py). r13-#10:
  // byte equality alone must NOT certify a run that is carrying one of these.
  localparam [31:0] FATAL_MASK = (1<<3)  | (1<<4)  | (1<<9)  | (1<<10) | (1<<11) | (1<<13) | (1<<5)
                               | (1<<6)  | (1<<17) | (1<<21) | (1<<22) | (1<<23) | (1<<24);
  localparam O_REJECTED = 'h180;
  // AXI DMA (PG021) simple mode, S2MM half
  localparam O_S2MM_DMACR = 'h30, O_S2MM_DMASR = 'h34, O_S2MM_DA = 'h48, O_S2MM_LENGTH = 'h58;

  // ---- test geometry ----------------------------------------------------------------------------
  // r14-#8: deliberately NOT 0. `run_base`'s reset value is 0, so a base of 0 makes the post-DMA
  // "run_base unchanged" gate pass even if the whole control register file was reset underneath us.
  localparam WR_BASE  = 64'h0002_0000;      // in the MIG's own AXI space, 512-B aligned (see BD address map)
  localparam PS_DEST    = 64'h1000_0000;    // PS DDR destination for the DMA
  localparam N_INJ    = 12;                 // injected results (must fit one 32-B beat multiple)
  localparam [63:0] POISON = 64'hDEAD_BEEF_0000_0000;

  // ---- clocks -----------------------------------------------------------------------------------
  // BFM mode does not run the DDR PHY, so only the free-running board clocks matter.
  reg sysclk300 = 0, dspclk = 0, hostclk = 0, adcclk = 0, dacclk = 0, sysref = 0, usysref = 0;
  // 3.334 ns, exactly the MIG's CONFIG.C0.DDR4_InputClockPeriod (3334 ps, ddr-config.tcl). The old #1666
  // (3.332 ns) ran the memory clock 0.06 % fast: the controller's tRRD_S / tFAW counts, sized for 3.334 ns,
  // then fell 1-9 ps short and the DDR4 model reported them as VIOLATIONs (P3b r1).
`ifdef G4_SYSCLK_3333
  // margin probe (RISCQ_G4_SYSCLK_3333=1): the board oscillator's true 300.0 MHz, 3.333 ns, as
  // 1666 + 1667 ps half-periods (the TB precision is 1 ps)
  always begin #1666 sysclk300 = 1; #1667 sysclk300 = 0; end
`else
  always #1667 sysclk300 = ~sysclk300;   // 299.94 MHz = the configured 3334 ps
`endif
  always #1000 dspclk    = ~dspclk;      // 500.0 MHz
  always #4000 hostclk   = ~hostclk;     // 125.0 MHz
  always #1000 adcclk    = ~adcclk;      // placeholder: nothing in this test uses the RFDC
  always #1000 dacclk    = ~dacclk;
  always #50000 sysref   = ~sysref;
  always #50000 usysref  = ~usysref;

  wire [16:0] ddr4_adr; wire [1:0] ddr4_ba, ddr4_bg; wire ddr4_act_n, ddr4_ck_c, ddr4_ck_t;
  wire ddr4_cke, ddr4_odt, ddr4_reset_n; wire [1:0] ddr4_cs_n;
  wire [3:0] ddr4_dm_n, ddr4_dqs_c, ddr4_dqs_t; wire [31:0] ddr4_dq;

  riscq_bd_wrapper DUT (
    .default_sysclk_c0_300mhz_clk_p(sysclk300), .default_sysclk_c0_300mhz_clk_n(~sysclk300),
    .dspClk_clk_p(dspclk),   .dspClk_clk_n(~dspclk),
    .hostClk_clk_p(hostclk), .hostClk_clk_n(~hostclk),
    .adc_clk_clk_p(adcclk),  .adc_clk_clk_n(~adcclk),
    .dac_clk_clk_p(dacclk),  .dac_clk_clk_n(~dacclk),
    .sysref_in_diff_p(sysref), .sysref_in_diff_n(~sysref),
    .user_sysref_clk_p(usysref), .user_sysref_clk_n(~usysref),
    .ddr4_sdram_c0_act_n(ddr4_act_n), .ddr4_sdram_c0_adr(ddr4_adr), .ddr4_sdram_c0_ba(ddr4_ba),
    .ddr4_sdram_c0_bg(ddr4_bg), .ddr4_sdram_c0_ck_c(ddr4_ck_c), .ddr4_sdram_c0_ck_t(ddr4_ck_t),
    .ddr4_sdram_c0_cke(ddr4_cke), .ddr4_sdram_c0_cs_n(ddr4_cs_n), .ddr4_sdram_c0_dm_n(ddr4_dm_n),
    .ddr4_sdram_c0_dq(ddr4_dq), .ddr4_sdram_c0_dqs_c(ddr4_dqs_c), .ddr4_sdram_c0_dqs_t(ddr4_dqs_t),
    .ddr4_sdram_c0_odt(ddr4_odt), .ddr4_sdram_c0_reset_n(ddr4_reset_n)
    // every RFDC analog port is left unconnected: this test drives the uplink through the injector.
  );

  // The DDR4 device. Required in BOTH simulation modes: `Simulation_Mode = BFM` makes the XiPhy
  // primitives behavioural, it does NOT supply a memory -- the controller still drives a real device on
  // these pins. Leaving them unconnected (G4a's original premise) means writes go nowhere and reads
  // return zeros, which is exactly what run 6 showed: perfect framing, full strobes, BRESP=OKAY, and an
  // all-zero payload. G4b differs only in that the PHY itself is Unisim rather than BFM.
  ddr4_mem_c0 mem_c0 (
    .ck_t (ddr4_ck_t), .ck_c (ddr4_ck_c), .act_n(ddr4_act_n), .adr  (ddr4_adr),
    .ba   (ddr4_ba),   .bg   (ddr4_bg),   .cke  (ddr4_cke),   .cs_n (ddr4_cs_n),
    .odt  (ddr4_odt),  .reset_n(ddr4_reset_n),
    .dq   (ddr4_dq),   .dqs_t(ddr4_dqs_t), .dqs_c(ddr4_dqs_c), .dm_n(ddr4_dm_n)
  );

  // ---- datapath probes -------------------------------------------------------------------------
  // G4a run 2 showed the DMA reporting a clean completion (DMASR=0x1002, no error bits) while every
  // destination word still held its poison -- i.e. it wrote NOTHING. `Idle` alone cannot tell "finished"
  // from "never started", so both links between the uplink and PS memory are counted directly.
  int axis_beats = 0, axis_last = 0, aw_cnt = 0, w_cnt = 0, wlast_cnt = 0, b_cnt = 0;
  logic [31:0]  first_awaddr = 32'hFFFF_FFFF;
  logic [1:0]   last_bresp   = 2'b11;
  // r23-#5: counts + WLAST + BRESP do NOT prove the writes were correct -- all-zero WSTRB would produce
  // exactly "OKAY with unchanged poison". Capture the payload shape too.
  logic [7:0]   first_awlen   = 8'hFF;
  logic [2:0]   first_awsize  = 3'h7;
  logic [1:0]   first_awburst = 2'h3;                 // r25-#3b
  // r25-#3c: counting `last` is not enough -- one early assertion followed by more beats has the same
  // count as a correctly framed burst. Record the 1-based beat index it landed on.
  int axis_last_at = 0, wlast_at = 0, hp_wlast_at = 0;
  // r26-#2: recording only `=== 1` silently ignores an X/Z LAST on a non-final beat, after which a
  // correct final assertion satisfies both the count and the position check. Count the unknowns.
  int axis_last_x = 0, wlast_x = 0, hp_wlast_x = 0;
  logic [255:0] first_wdata  = 256'hX;
  logic [31:0]  first_wstrb  = 32'hX;
  int           zero_strb_beats = 0;
  logic [31:0]  strb_and = 32'hFFFF_FFFF, strb_or = 32'h0;
  // Every W beat is logged so the verdict can be taken at the LAST POINT THE DESIGN CONTROLS -- the
  // bytes it hands to the PS -- instead of depending on the VIP's memory model. See the note in step 10.
  localparam int MAXW = 16;
  logic [255:0] w_data_log [0:MAXW-1];
  logic [31:0]  w_strb_log [0:MAXW-1];
  // r26-#3: the AXIS leg was counted but never CHECKED. Log its payload too, so a corruption between
  // mmu2 and the DMA is caught at the point it happens instead of only at HP0.
  logic [255:0] axis_data_log [0:MAXW-1];
  // r24-#5: the DMA's own master port is UPSTREAM of smc_dma (256->128 downsize) and of the HP0 address
  // decode, so it cannot speak for either. Monitor the HP0 slave port too -- that is the last signal
  // boundary of the design, and the 128-bit beats there must reconstruct the same 12 words.
  localparam int MAXH = 32;
  logic [127:0] hp_data_log [0:MAXH-1];
  logic [15:0]  hp_strb_log [0:MAXH-1];
  int hp_aw = 0, hp_w = 0, hp_wlast = 0, hp_b = 0;
  logic [48:0] hp_first_awaddr = {49{1'b1}};
  logic [7:0]  hp_first_awlen  = 8'hFF;
  logic [2:0]  hp_first_awsize = 3'h7;
  logic [1:0]  hp_first_awburst= 2'h3;
  logic [1:0]  hp_last_bresp   = 2'b11;

  always @(posedge DUT.riscq_bd_i.ddr4_0.c0_ddr4_ui_clk) begin
    // uplink AXIS -> DMA
    if (DUT.riscq_bd_i.top_M_AXIS_RD_TVALID === 1'b1 && DUT.riscq_bd_i.top_M_AXIS_RD_TREADY === 1'b1) begin
      if (axis_beats < MAXW) axis_data_log[axis_beats] = DUT.riscq_bd_i.top_M_AXIS_RD_TDATA;
      axis_beats++;
      if (DUT.riscq_bd_i.top_M_AXIS_RD_TLAST !== 1'b0 && DUT.riscq_bd_i.top_M_AXIS_RD_TLAST !== 1'b1) axis_last_x++;
      if (DUT.riscq_bd_i.top_M_AXIS_RD_TLAST === 1'b1) begin axis_last++; axis_last_at = axis_beats; end
    end
    // DMA -> HP0 write channel
    if (DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_AWVALID === 1'b1 &&
        DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_AWREADY === 1'b1) begin
      if (aw_cnt == 0) begin
        first_awaddr = DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_AWADDR;
        first_awlen   = DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_AWLEN;
        first_awsize  = DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_AWSIZE;
        first_awburst = DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_AWBURST;
      end
      aw_cnt++;
    end
    if (DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WVALID === 1'b1 &&
        DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WREADY === 1'b1) begin
      if (w_cnt == 0) begin
        first_wdata = DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WDATA;
        first_wstrb = DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WSTRB;
      end
      if (w_cnt < MAXW) begin
        w_data_log[w_cnt] = DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WDATA;
        w_strb_log[w_cnt] = DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WSTRB;
      end
      strb_and &= DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WSTRB;
      strb_or  |= DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WSTRB;
      if (DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WSTRB === 32'h0) zero_strb_beats++;
      w_cnt++;
      if (DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WLAST !== 1'b0 && DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WLAST !== 1'b1) wlast_x++;
      if (DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_WLAST === 1'b1) begin wlast_cnt++; wlast_at = w_cnt; end
    end
    if (DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_BVALID === 1'b1 &&
        DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_BREADY === 1'b1) begin
      b_cnt++;
      last_bresp = DUT.riscq_bd_i.axi_dma_0_M_AXI_S2MM_BRESP;
    end
    // ---- HP0 slave port (smc_dma -> zynq_ps/S_AXI_HP0_FPD) ----
    if (DUT.riscq_bd_i.smc_dma_M00_AXI_AWVALID === 1'b1 &&
        DUT.riscq_bd_i.smc_dma_M00_AXI_AWREADY === 1'b1) begin
      if (hp_aw == 0) begin
        hp_first_awaddr  = DUT.riscq_bd_i.smc_dma_M00_AXI_AWADDR;
        hp_first_awlen   = DUT.riscq_bd_i.smc_dma_M00_AXI_AWLEN;
        hp_first_awsize  = DUT.riscq_bd_i.smc_dma_M00_AXI_AWSIZE;
        hp_first_awburst = DUT.riscq_bd_i.smc_dma_M00_AXI_AWBURST;
      end
      hp_aw++;
    end
    if (DUT.riscq_bd_i.smc_dma_M00_AXI_WVALID === 1'b1 &&
        DUT.riscq_bd_i.smc_dma_M00_AXI_WREADY === 1'b1) begin
      if (hp_w < MAXH) begin
        hp_data_log[hp_w] = DUT.riscq_bd_i.smc_dma_M00_AXI_WDATA;
        hp_strb_log[hp_w] = DUT.riscq_bd_i.smc_dma_M00_AXI_WSTRB;
      end
      hp_w++;
      if (DUT.riscq_bd_i.smc_dma_M00_AXI_WLAST !== 1'b0 && DUT.riscq_bd_i.smc_dma_M00_AXI_WLAST !== 1'b1) hp_wlast_x++;
      if (DUT.riscq_bd_i.smc_dma_M00_AXI_WLAST === 1'b1) begin hp_wlast++; hp_wlast_at = hp_w; end
    end
    if (DUT.riscq_bd_i.smc_dma_M00_AXI_BVALID === 1'b1 &&
        DUT.riscq_bd_i.smc_dma_M00_AXI_BREADY === 1'b1) begin
      hp_b++;
      hp_last_bresp = DUT.riscq_bd_i.smc_dma_M00_AXI_BRESP;
    end
  end

  int backdoor_disagreed = 0;

  task automatic report_links(input string whn);
    begin
      $display("[G4] links %0s: AXIS beats=%0d (tlast x%0d) | S2MM AW=%0d (first addr 0x%08h len=%0d size=%0d) W=%0d (wlast x%0d) B=%0d (bresp %b)",
               whn, axis_beats, axis_last, aw_cnt, first_awaddr, first_awlen, first_awsize,
               w_cnt, wlast_cnt, b_cnt, last_bresp);
      if (w_cnt > 0)
        $display("[G4]   first W: wstrb=0x%08h wdata[63:0]=0x%016h | strb AND=0x%08h OR=0x%08h, all-zero-strb beats=%0d",
                 first_wstrb, first_wdata[63:0], strb_and, strb_or, zero_strb_beats);
      $display("[G4]   HP0 slave: AW=%0d (addr 0x%011h len=%0d size=%0d burst=%0d) W=%0d (wlast x%0d) B=%0d (bresp %b)",
               hp_aw, hp_first_awaddr, hp_first_awlen, hp_first_awsize, hp_first_awburst,
               hp_w, hp_wlast, hp_b, hp_last_bresp);
    end
  endtask

  // ---- PS VIP access helpers --------------------------------------------------------------------
  reg [1023:0] wbuf, rbuf;
  reg [1:0]    rsp;

  // r13-#9: a failed test MUST fail the batch job. $finish leaves Vivado's exit status successful, so
  // every failure path goes through $fatal, and run-sim-bd.tcl additionally requires exactly one
  // "[G4] PASS:" line and zero "[G4] FAIL:" lines in the transcript.
  task automatic fail(input string why);
    begin $display("[G4] FAIL: %0s", why); $fatal(1, "[G4] FAIL"); end
  endtask

  task automatic ps_w32(input [63:0] addr, input [31:0] val);
    begin
      wbuf = 1024'd0; wbuf[31:0] = val;
      DUT.riscq_bd_i.zynq_ps.inst.write_data(addr, 4, wbuf, rsp);
      if (rsp !== 2'b00) fail($sformatf("write to 0x%0h returned BRESP=%0d", addr, rsp));
    end
  endtask

  // r15-#7: this MUST be a task, not a function -- it enables the VIP's `read_data` task and consumes
  // simulation time, neither of which a SystemVerilog function may do.
  // r15-#6: the value stays FOUR-STATE all the way out. Coercing to `bit` would turn an X status bit
  // into a 0, i.e. "the flag is clear", which is the most dangerous possible misreading here.
  task automatic ps_r32(input [63:0] addr, output logic [31:0] val);
    begin
      DUT.riscq_bd_i.zynq_ps.inst.read_data(addr, 4, rbuf, rsp);
      if (rsp !== 2'b00) fail($sformatf("read from 0x%0h returned RRESP=%0d", addr, rsp));
      val = rbuf[31:0];
      // r16-#2: reject X/Z HERE, once, for every register read. An ordinary `if (val & MASK)` treats an
      // X error bit as false, so an undriven status/error field would read as "nothing wrong" -- the
      // most dangerous possible misreading in a gate whose whole job is to reject bad runs.
      if ((val ^ val) !== 32'd0)
        fail($sformatf("read from 0x%0h returned X/Z: 0x%08h", addr, val));
    end
  endtask

  /** One STATUS bit, four-state. Callers compare with === so X/Z can never masquerade as 0. */
  task automatic stat(input int b, output logic v);
    logic [31:0] s; begin
      ps_r32(CTRL + O_STATUS, s);
      v = s[b];   // ps_r32 has already rejected any X/Z in the whole word
    end
  endtask

  // r13-#10: no run may be certified while ANY fatal sticky is set.
  task automatic check_clean(input string whn);
    logic [31:0] s; begin
      ps_r32(CTRL + O_STATUS, s);   // X/Z already rejected inside ps_r32
      if (s & FATAL_MASK) fail($sformatf("fatal status 0x%08h (mask 0x%08h) %0s", s, FATAL_MASK, whn));
    end
  endtask

  task automatic poll_clear(input int b, input string what);
    int n; logic v; begin
      n = 0; stat(b, v);
      while (v === 1'b1 && n < POLL_LIMIT) begin #1000; n++; stat(b, v); end
      if (v !== 1'b0) fail($sformatf("%0s never cleared", what));
    end
  endtask

  // ---- expected DDR image -----------------------------------------------------------------------
  // word = {tag[7:0], real[31:4], imag[31:4]} -- ground truth QubiC ddr_readout_data.py
  function automatic [63:0] tag_word(input [7:0] tag, input [31:0] re, input [31:0] im);
    begin tag_word = {tag, re[31:4], im[31:4]}; end
  endfunction

  // ======================= P3b: helpers for the extra phases ======================================
  localparam S_BRESP_ERR = 3, S_RRESP_ERR = 4;
  localparam [63:0] PS_DEST2 = 64'h1100_0000;
  localparam int    MAXP = 256;
  reg  [63:0] pexp [0:MAXP-1];
  logic [31:0] ps, pv;
  logic        pb;
  logic [1:0]  fcode;                 // the forced response code (a force RHS must be a static variable)

  task automatic reset_links();
    begin
      axis_beats = 0; axis_last = 0; aw_cnt = 0; w_cnt = 0; wlast_cnt = 0; b_cnt = 0;
      axis_last_at = 0; wlast_at = 0; hp_wlast_at = 0; axis_last_x = 0; wlast_x = 0; hp_wlast_x = 0;
      zero_strb_beats = 0; strb_and = 32'hFFFF_FFFF; strb_or = 32'h0;
      hp_aw = 0; hp_w = 0; hp_wlast = 0; hp_b = 0;
      first_awaddr = 32'hFFFF_FFFF; last_bresp = 2'b11; hp_last_bresp = 2'b11;
    end
  endtask

  task automatic p_start(input [63:0] base);
    int n; logic v; begin
      ps_w32(CTRL + O_WR_BASE, base);
      ps_w32(CTRL + O_BASE_RST, 1);
      n = 0; stat(S_RUN_ACTIVE, v);
      while (v !== 1'b1 && n < POLL_LIMIT) begin #1000; n++; stat(S_RUN_ACTIVE, v); end
      if (v !== 1'b1) begin ps_r32(CTRL + O_STATUS, ps); fail($sformatf("run at 0x%0h never became active (STATUS=0x%08h)", base, ps)); end
      ps_r32(CTRL + O_RUN_BASE, pv);
      if (pv !== base[31:0]) fail($sformatf("run_base latched 0x%0h, expected 0x%0h", pv, base[31:0]));
    end
  endtask

  task automatic p_inject(input int n, input int salt);
    begin
      if (n > MAXP) fail("p_inject: too many");
      for (int i = 0; i < n; i++) begin
        automatic int core = i % n_ch;
        automatic int re   = 32'h0100_0000 * (salt + 1) + (i << 8) + 16;
        automatic int im   = 32'h0050_0000 + (salt << 16) + (i << 8) + 32;
        ps_w32(CTRL + O_INJ_REAL, re); ps_w32(CTRL + O_INJ_IMAG, im);
        ps_w32(CTRL + O_INJ_CORE, core); ps_w32(CTRL + O_INJ_FIRE, 1);
        poll_clear(S_INJ_BUSY, "inj_busy");
        pexp[i] = tag_word(core[7:0], re, im);
      end
    end
  endtask

  task automatic p_flush(output logic [31:0] s);
    begin
      ps_w32(CTRL + O_FLUSH, 1);
      poll_clear(S_FLUSH_BUSY, "flush_busy");
      ps_r32(CTRL + O_STATUS, s);
    end
  endtask

  // the ddr_board.dma_recv_prepare order: RS, RS must take (not halted), DA, LENGTH (starts it)
  task automatic p_dma_arm(input [63:0] dest, input [31:0] n);
    int k2; begin
      ps_w32(DMA + O_S2MM_DMACR, 32'h1);
      k2 = 0; ps_r32(DMA + O_S2MM_DMASR, ps);
      while (ps[0] !== 1'b0 && k2 < POLL_LIMIT) begin #1000; k2++; ps_r32(DMA + O_S2MM_DMASR, ps); end
      if (ps[0] !== 1'b0) fail($sformatf("S2MM stayed halted after RS=1 (DMASR=0x%08h)", ps));
      ps_w32(DMA + O_S2MM_DA, dest[31:0]);
      ps_w32(DMA + O_S2MM_LENGTH, n);
    end
  endtask

  task automatic p_drain_go(input [63:0] base, input [31:0] n);
    begin
      ps_w32(CTRL + O_RD_BASE, base); ps_w32(CTRL + O_RD_SIZE, n); ps_w32(CTRL + O_RD_START, 1);
    end
  endtask

  // poll S2MM_DMASR for Idle with a bound (the driver's timeout); `idle` = completed
  task automatic p_dma_wait(input int limit_us, output logic [31:0] sr, output logic idle);
    int k2; begin
      k2 = 0; idle = 1'b0;
      do begin #1000; k2++; ps_r32(DMA + O_S2MM_DMASR, sr); end
      while (sr[1] !== 1'b1 && (sr & 32'h770) == 0 && k2 < limit_us);
      idle = (sr[1] === 1'b1);
    end
  endtask

  // ddr_board.dma_reset(): DMACR.Reset, wait for it to self-clear, the channel is halted again
  task automatic p_dma_soft_reset();
    int k2; begin
      ps_w32(DMA + O_S2MM_DMACR, 32'h4);
      k2 = 0; ps_r32(DMA + O_S2MM_DMACR, pv);
      while (pv[2] !== 1'b0 && k2 < POLL_LIMIT) begin #1000; k2++; ps_r32(DMA + O_S2MM_DMACR, pv); end
      if (pv[2] !== 1'b0) fail($sformatf("S2MM soft reset never cleared (DMACR=0x%08h)", pv));
      ps_r32(DMA + O_S2MM_DMASR, ps);
      if (ps[0] !== 1'b1) fail($sformatf("S2MM not halted after the soft reset (DMASR=0x%08h)", ps));
    end
  endtask

  // byte-exact check of an n-word drain at all three boundaries (uplink AXIS, DMA master, HP0 slave)
  task automatic p_verify(input int n, input [63:0] dest, input string what);
    int errs; begin
      errs = 0;
      if (axis_beats != (n * 8) / 32 || axis_last != 1 || axis_last_at != (n * 8) / 32)
        fail($sformatf("%0s: AXIS %0d beats, TLAST x%0d at %0d (expected %0d, x1)", what, axis_beats, axis_last, axis_last_at, (n*8)/32));
      if (aw_cnt != 1 || b_cnt != 1 || last_bresp !== 2'b00 || first_awaddr !== dest[31:0])
        fail($sformatf("%0s: S2MM AW=%0d B=%0d bresp=%b addr=0x%08h", what, aw_cnt, b_cnt, last_bresp, first_awaddr));
      if (hp_aw != 1 || hp_b != 1 || hp_last_bresp !== 2'b00 || hp_w != (n * 8) / 16 || hp_wlast != 1)
        fail($sformatf("%0s: HP0 AW=%0d B=%0d W=%0d wlast x%0d bresp=%b", what, hp_aw, hp_b, hp_w, hp_wlast, hp_last_bresp));
      if (zero_strb_beats != 0) fail($sformatf("%0s: %0d all-zero-strobe beats", what, zero_strb_beats));
      for (int i = 0; i < n; i++) begin
        if (axis_data_log[i / 4][64 * (i % 4) +: 64] !== pexp[i]) errs++;
        if (w_data_log[i / 4][64 * (i % 4) +: 64] !== pexp[i]) errs++;
        if (w_strb_log[i / 4][8 * (i % 4) +: 8] !== 8'hFF) errs++;
        if (hp_data_log[i / 2][64 * (i % 2) +: 64] !== pexp[i]) errs++;
        if (hp_strb_log[i / 2][8 * (i % 2) +: 8] !== 8'hFF) errs++;
      end
      if (errs != 0) fail($sformatf("%0s: %0d word/strobe mismatches", what, errs));
    end
  endtask

  reg [63:0]   expect_w [0:N_INJ-1];
  int          n_ch, k, errors;
  logic [31:0] final_addr, nbytes, dmasr, acc_total, rv, sreg;
  logic        sv;
  reg [1023:0] mem, pbuf;

  initial begin
    errors = 0;
    DUT.riscq_bd_i.zynq_ps.inst.set_stop_on_error(1);

    // 1) release the PS, then the PL
    DUT.riscq_bd_i.zynq_ps.inst.por_srstb_reset(1'b1);
    DUT.riscq_bd_i.zynq_ps.inst.fpga_soft_reset(32'h0);

    // r13-#11: wait for an EXPLICIT readiness condition, not a fixed delay. `c0_init_calib_complete`
    // is the MIG's own "the memory interface is usable" flag (asserted almost immediately in BFM mode,
    // after real calibration in a full-PHY run), and the ui_clk reset tree is released after it.
    k = 0;
    while (DUT.riscq_bd_i.ddr4_0.c0_init_calib_complete !== 1'b1 && k < CALIB_LIMIT) begin #1000; k++; end
    if (DUT.riscq_bd_i.ddr4_0.c0_init_calib_complete !== 1'b1)
      fail("c0_init_calib_complete never asserted -- the MIG never became ready");
    $display("[G4] MIG calibration complete at t=%0t", $time);

    // r14-#9: and the ui_clk reset tree must actually be RELEASED -- calibration complete only gates the
    // stretcher's input. Poll the proc_sys_reset output, then the uplink's own view of its reset.
    k = 0;
    while (DUT.riscq_bd_i.psr_ddr.peripheral_aresetn !== 1'b1 && k < CALIB_LIMIT) begin #1000; k++; end
    if (DUT.riscq_bd_i.psr_ddr.peripheral_aresetn !== 1'b1)
      fail("psr_ddr/peripheral_aresetn never released -- the ui_clk reset tree is stuck");
    k = 0; stat(S_DSP_IN_RESET, sv);
    while (sv === 1'b1 && k < POLL_LIMIT) begin #1000; k++; stat(S_DSP_IN_RESET, sv); end
    if (sv !== 1'b0) fail("the uplink still reports dsp_in_reset -- the DSP domain never woke");
    $display("[G4] ui_clk reset tree released at t=%0t", $time);

    // 2) the uplink must be alive and self-describing
    ps_r32(CTRL + O_NUM_CH, rv);   n_ch = rv;
    ps_r32(CTRL + O_STATUS, sreg);
    $display("[G4] NUM_CH = %0d, STATUS = 0x%08h", n_ch, sreg);
    if (n_ch == 0 || n_ch > 32)
      fail($sformatf("NUM_CH=%0d is not plausible -- the control slave is not responding", n_ch));

    // 3) start a run
    ps_w32(CTRL + O_WR_BASE, WR_BASE);
    ps_w32(CTRL + O_BASE_RST, 1);
    #20000;
    stat(S_RUN_ACTIVE, sv);
    if (sv !== 1'b1) begin
      ps_r32(CTRL + O_STATUS, sreg);
      fail($sformatf("run never became active (STATUS=0x%08h)", sreg));
    end
    ps_r32(CTRL + O_RUN_BASE, rv);
    if (rv !== WR_BASE[31:0]) fail($sformatf("run_base latched 0x%0h, expected 0x%0h", rv, WR_BASE[31:0]));

    // 4) inject N results, round-robin over the cores that exist
    for (k = 0; k < N_INJ; k++) begin
      automatic int core = k % n_ch;
      automatic int re   = 32'h0011_0000 + (k << 8);
      automatic int im   = 32'h0022_0000 + (k << 8);
      ps_w32(CTRL + O_INJ_REAL, re);
      ps_w32(CTRL + O_INJ_IMAG, im);
      ps_w32(CTRL + O_INJ_CORE, core);
      ps_w32(CTRL + O_INJ_FIRE, 1);
      poll_clear(S_INJ_BUSY, "inj_busy");
      expect_w[k] = tag_word(core[7:0], re, im);
    end

    // 5) flush and check the accounting
    ps_w32(CTRL + O_FLUSH, 1);
    poll_clear(S_FLUSH_BUSY, "flush_busy");
    stat(S_WRITE_DONE, sv);
    if (sv !== 1'b1) fail("write_done not set after the flush");
    check_clean("after the flush");
    acc_total = 0;
    for (k = 0; k < n_ch; k++) begin
      ps_r32(CTRL + O_ACCEPTED + 4*k, rv); acc_total += rv;
      ps_r32(CTRL + O_REJECTED + 4*k, rv);
      if (rv !== 32'd0) fail($sformatf("core %0d rejected %0d results", k, rv));
    end
    if (acc_total !== N_INJ) fail($sformatf("accepted total %0d != %0d injected", acc_total, N_INJ));
    ps_r32(CTRL + O_FINAL_ADDR, final_addr);
    nbytes = final_addr - WR_BASE[31:0];
    $display("[G4] flush done: accepted=%0d, final_addr=0x%08h (%0d B)", acc_total, final_addr, nbytes);
    if (nbytes % 32 != 0 || nbytes < N_INJ*8 || nbytes > N_INJ*8 + 24)
      fail($sformatf("final_addr implies %0d B, expected %0d..%0d and a multiple of 32",
                     nbytes, N_INJ*8, N_INJ*8+24));

    // 6) POISON the destination first. An all-X readback is ambiguous -- it could mean "the DMA never
    //    wrote here" OR "the VIP's memory model does not back this address at all". Writing a known
    //    pattern and reading it back disambiguates before the DMA runs, and afterwards any surviving
    //    poison word points at the DMA rather than at the model. (Found by the first G4a run, which
    //    returned 12/12 X with the DMA reporting a clean completion.)
    for (k = 0; k < N_INJ; k++) begin
      pbuf = 1024'd0; pbuf[63:0] = POISON + k;
      DUT.riscq_bd_i.zynq_ps.inst.write_mem(pbuf, PS_DEST + 8*k, 8);
    end
    for (k = 0; k < N_INJ; k++) begin
      DUT.riscq_bd_i.zynq_ps.inst.read_mem(PS_DEST + 8*k, 8, mem);
      if (mem[63:0] !== (POISON + k))
        fail($sformatf("the VIP memory model does not hold what write_mem put at 0x%0h: wrote 0x%016h, read 0x%016h",
                       PS_DEST + 8*k, POISON + k, mem[63:0]));
    end
    $display("[G4] destination 0x%0h..0x%0h poisoned and verified readable", PS_DEST, PS_DEST + 8*N_INJ - 1);

    // 7) arm the S2MM DMA, then start the drain. Order matters: mmu2 streams as soon as rd_start fires.
    ps_w32(DMA + O_S2MM_DMACR, 32'h1);                     // RS=1 (run)
    #20000;
    ps_w32(DMA + O_S2MM_DA,     PS_DEST[31:0]);
    ps_w32(DMA + O_S2MM_LENGTH, nbytes);                   // writing LENGTH starts the transfer
    ps_w32(CTRL + O_RD_BASE, WR_BASE);
    ps_w32(CTRL + O_RD_SIZE, nbytes);
    report_links("before rd_start");
    ps_w32(CTRL + O_RD_START, 1);

    // 8) wait for the DMA to go Idle (bit 1 of S2MM_DMASR); check its error bits on the way
    k = 0;
    do begin
      #10000; k++;
      ps_r32(DMA + O_S2MM_DMASR, dmasr);
      if (dmasr & 32'h770) fail($sformatf("S2MM_DMASR error 0x%08h", dmasr));
    end while (dmasr[1] !== 1'b1 && k < POLL_LIMIT);
    if (dmasr[1] !== 1'b1) fail($sformatf("DMA never went idle (DMASR=0x%08h)", dmasr));
    $display("[G4] DMA idle, DMASR=0x%08h after %0d polls", dmasr, k);
    ps_r32(DMA + O_S2MM_DMACR,  rv);  $display("[G4] S2MM_DMACR  = 0x%08h (bit0 RS)", rv);
    ps_r32(DMA + O_S2MM_LENGTH, rv);  $display("[G4] S2MM_LENGTH = %0d bytes actually transferred", rv);
    ps_r32(DMA + O_S2MM_DA,     rv);  $display("[G4] S2MM_DA     = 0x%08h", rv);
    report_links("after the drain");
    if (axis_beats == 0)
      fail("the uplink never streamed a beat: mmu2 -> S_AXIS_S2MM is dead (rd_start did not start a drain)");
    if (aw_cnt == 0)
      fail($sformatf("the DMA accepted %0d AXIS beats but issued NO write to HP0", axis_beats));
    // r23-#5 / r24-#4: a COMPLETE oracle on both boundaries. Counts alone let extra bursts or a wrong
    // address pass; payload alone lets a mis-framed burst pass. Everything is pinned.
    //   expected shape: N_INJ words = N_INJ*8 bytes, one burst, 256-bit beats at the DMA master and
    //   128-bit beats at HP0 after smc_dma's downsize.
    begin
      automatic int exp_w   = (N_INJ * 8) / 32;      // 256-bit beats
      automatic int exp_hpw = (N_INJ * 8) / 16;      // 128-bit beats
      // -- uplink AXIS --
      if (axis_beats != exp_w)
        fail($sformatf("uplink streamed %0d AXIS beats, expected %0d", axis_beats, exp_w));
      if (axis_last != 1)
        fail($sformatf("uplink asserted TLAST %0d times, expected exactly 1", axis_last));
      if (axis_last_at != exp_w)
        fail($sformatf("uplink asserted TLAST on beat %0d, expected the last one (%0d)", axis_last_at, exp_w));
      if (axis_last_x != 0)
        fail($sformatf("uplink drove TLAST X/Z on %0d accepted beat(s)", axis_last_x));
      if (axis_beats > MAXW) fail($sformatf("%0d AXIS beats exceeds the %0d-deep log", axis_beats, MAXW));
      // -- DMA master port --
      if (aw_cnt != 1) fail($sformatf("S2MM issued %0d AW bursts, expected exactly 1", aw_cnt));
      if (b_cnt  != 1) fail($sformatf("S2MM saw %0d write responses, expected exactly 1", b_cnt));
      if (last_bresp !== 2'b00) fail($sformatf("S2MM BRESP=%b, expected OKAY", last_bresp));
      if (first_awaddr !== PS_DEST[31:0])
        fail($sformatf("S2MM wrote to 0x%08h, expected the programmed destination 0x%08h",
                       first_awaddr, PS_DEST[31:0]));
      if (first_awsize !== 3'd5)
        fail($sformatf("S2MM AWSIZE=%0d, expected 5 (32 bytes = the 256-bit master)", first_awsize));
      if (first_awburst !== 2'b01)
        fail($sformatf("S2MM AWBURST=%0d, expected 1 (INCR)", first_awburst));
      if (w_cnt != exp_w)   fail($sformatf("S2MM sent %0d W beats, expected %0d", w_cnt, exp_w));
      if (wlast_cnt != 1)   fail($sformatf("S2MM asserted WLAST %0d times, expected exactly 1", wlast_cnt));
      if (wlast_at != exp_w)
        fail($sformatf("S2MM asserted WLAST on beat %0d, expected the last one (%0d)", wlast_at, exp_w));
      if (wlast_x != 0) fail($sformatf("S2MM drove WLAST X/Z on %0d accepted beat(s)", wlast_x));
      // r25-#3d: four-state-safe. `!=` on an X AWLEN yields X, which `if` treats as false -- the check
      // would silently not fire on exactly the corrupt value it is meant to catch.
      if (first_awlen !== (exp_w - 1))
        fail($sformatf("S2MM AWLEN=%0d (0x%02h), expected %0d for %0d beats",
                       first_awlen, first_awlen, exp_w - 1, exp_w));
      if (zero_strb_beats != 0)
        // NOTE: SystemVerilog has no C-style adjacent string-literal concatenation -- keep this on one
        // line (or use {"a","b"}). An earlier split here failed xvlog with "syntax error near ...".
        fail($sformatf("%0d of %0d S2MM write beats had WSTRB=0 (an all-zero-strobe burst is answered OKAY and writes nothing)", zero_strb_beats, w_cnt));
      // -- HP0 slave port: proves smc_dma's downsize AND the address decode, which the DMA port cannot --
      if (hp_aw != 1) fail($sformatf("HP0 saw %0d AW bursts, expected exactly 1", hp_aw));
      if (hp_b  != 1) fail($sformatf("HP0 saw %0d write responses, expected exactly 1", hp_b));
      if (hp_last_bresp !== 2'b00) fail($sformatf("HP0 BRESP=%b, expected OKAY", hp_last_bresp));
      // r25-#3a: the HP0 port is 49-bit -- compare the WHOLE address, or a non-zero upper field selects
      // a completely different location while the low 32 bits still look right.
      if (hp_first_awaddr !== {17'd0, PS_DEST[31:0]})
        fail($sformatf("HP0 received AWADDR 0x%013h, expected 0x%013h -- the address decode moved it",
                       hp_first_awaddr, {17'd0, PS_DEST[31:0]}));
      if (hp_first_awburst !== 2'b01)
        fail($sformatf("HP0 AWBURST=%0d, expected 1 (INCR)", hp_first_awburst));
      if (hp_first_awsize !== 3'd4)
        fail($sformatf("HP0 AWSIZE=%0d, expected 4 (16 bytes after smc_dma's 256->128 downsize)",
                       hp_first_awsize));
      if (hp_w != exp_hpw)  fail($sformatf("HP0 saw %0d W beats, expected %0d", hp_w, exp_hpw));
      if (hp_wlast != 1)    fail($sformatf("HP0 saw WLAST %0d times, expected exactly 1", hp_wlast));
      if (hp_wlast_at != exp_hpw)
        fail($sformatf("HP0 saw WLAST on beat %0d, expected the last one (%0d)", hp_wlast_at, exp_hpw));
      if (hp_wlast_x != 0) fail($sformatf("HP0 saw WLAST X/Z on %0d accepted beat(s)", hp_wlast_x));
      if (hp_first_awlen !== (exp_hpw - 1))
        fail($sformatf("HP0 AWLEN=%0d (0x%02h), expected %0d for %0d beats",
                       hp_first_awlen, hp_first_awlen, exp_hpw - 1, exp_hpw));
    end

    // NOTE: there is deliberately no PS "front-door" read of the destination here. The Zynq VIP's
    // `read_data`/`write_data` model PS->PL transactions only: `check_master_address()` accepts an
    // address only inside the M_AXI_GP0/1/2 apertures, and on anything else the task prints an
    // error and, with set_stop_on_error(1), calls $stop -- which hangs a batch xsim. (Cost one run.)
    // The backdoor write/read control above (the poison) is what proves the model can hold this
    // address at all.
    stat(S_RD_DONE, sv);
    if (sv !== 1'b1)
      $display("[G4] NOTE: rd_done not set (mmu2 `done` is a pulse; TLAST is the contract)");

    // 9) the run must STILL be clean after the drain (an RRESP raised while mmu2 was reading would
    //    otherwise be certified as good data -- same gate the software driver applies)
    check_clean("after the drain");
    // r14-#8: `write_done` must STILL be set. A control-domain reset during the DMA clears the whole
    // register file; with a non-zero WR_BASE the run_base check catches it too, but a vanished
    // write_done is the direct evidence (same gate as the software driver's post-DMA re-check).
    stat(S_WRITE_DONE, sv);
    if (sv !== 1'b1) fail("write_done vanished during the drain -- the uplink was reset");
    ps_r32(CTRL + O_RUN_BASE, rv);
    if (rv !== WR_BASE[31:0]) fail("run_base changed during the drain");

    // 10) THE VERDICT: byte-exact comparison of the bytes the design handed to the PS.
    //
    // The check is taken on the logged write beats at BOTH boundaries -- the DMA's `M_AXI_S2MM` port and
    // the `HP0` slave port after `smc_dma`'s 256->128 downsize and the address decode. HP0 is the last
    // signal boundary the design controls (r24-#5: the DMA port alone cannot speak for the downsizer or
    // the decode). This is COMPLEMENTARY to a memory image, not a substitute for one (r26-#4): it covers
    // every beat, lane and byte strobe rather than the end state, but it says nothing about persistence.
    // It is what is available here, because the VIP's memory model -- in
    // Vivado 2022.1, answers HP0 writes with BRESP=OKAY without updating the store that `read_mem()`
    // reads (proven in run 3: correct AW/W/WLAST/BRESP with the destination still holding poison).
    // `read_mem()` is still consulted, and any disagreement is printed, but it is not the verdict.
    if (w_cnt > MAXW) fail($sformatf("%0d DMA W beats exceeds the %0d-deep log", w_cnt, MAXW));
    if (hp_w  > MAXH) fail($sformatf("%0d HP0 W beats exceeds the %0d-deep log", hp_w, MAXH));
    for (k = 0; k < N_INJ; k++) begin
      automatic int beat = k / 4;              // 4 x 64-bit words per 256-bit beat
      automatic int lane = k % 4;
      automatic reg [63:0] sent = w_data_log[beat][64*lane +: 64];
      automatic reg [7:0]  strb = w_strb_log[beat][8*lane +: 8];
      automatic int hbeat = k / 2;                     // 2 x 64-bit words per 128-bit HP0 beat
      automatic int hlane = k % 2;
      automatic reg [63:0] streamed = axis_data_log[beat][64*lane +: 64];
      automatic reg [63:0] arrived = hp_data_log[hbeat][64*hlane +: 64];
      automatic reg [7:0]  hstrb   = hp_strb_log[hbeat][8*hlane +: 8];
      if (strb !== 8'hFF)
        fail($sformatf("word %0d left the DMA with WSTRB=0x%02h, not 0xFF -- those bytes never land", k, strb));
      if (hstrb !== 8'hFF)
        fail($sformatf("word %0d reached HP0 with WSTRB=0x%02h, not 0xFF", k, hstrb));
      if (streamed !== expect_w[k]) begin
        $display("[G4] MISMATCH word %0d ON THE UPLINK AXIS: 0x%016h, expected 0x%016h (mmu2/the drain)",
                 k, streamed, expect_w[k]);
        errors++;
      end
      if (sent !== expect_w[k]) begin
        $display("[G4] MISMATCH word %0d: the DMA wrote 0x%016h, expected 0x%016h", k, sent, expect_w[k]);
        errors++;
      end
      if (arrived !== expect_w[k]) begin
        $display("[G4] MISMATCH word %0d AT HP0: 0x%016h, expected 0x%016h (smc_dma corrupted it)",
                 k, arrived, expect_w[k]);
        errors++;
      end
      DUT.riscq_bd_i.zynq_ps.inst.read_mem(PS_DEST + 8*k, 8, mem);
      if (mem[63:0] !== sent) begin
        backdoor_disagreed++;
        if (backdoor_disagreed == 1)
          $display("[G4]   word %0d: read_mem backdoor 0x%016h != what the design wrote 0x%016h%0s",
                   k, mem[63:0], sent,
                   (mem[63:0] === (POISON + k)) ? "   (backdoor still holds poison)" : "");
      end
    end
    if (backdoor_disagreed != 0)
      $display("[G4] NOTE: the VIP read_mem() backdoor disagreed with the bytes the design wrote on %0d of %0d words -- known Vivado 2022.1 VIP behaviour, see sim/README.md",
               backdoor_disagreed, N_INJ);

    if (errors != 0) fail($sformatf("%0d/%0d words mismatched", errors, N_INJ));
    $display("[G4] ok-A: %0d injected results byte-exact at the AXIS, S2MM master and HP0 slave ports", N_INJ);

    // ============================ P3b phase B: BRESP errors =====================================
    for (int e = 0; e < 2; e++) begin
      automatic logic [1:0] code = e ? 2'b11 : 2'b10;    // SLVERR, then DECERR
      p_start(64'h0004_0000 + 64'h1000 * e);
      p_inject(8, 16 + e);
      fcode = code;
      force DUT.riscq_bd_i.smc_ddr_M00_AXI_BRESP = fcode;  // the MIG's write response, at its output
      p_flush(sreg);
      release DUT.riscq_bd_i.smc_ddr_M00_AXI_BRESP;
      ps_r32(CTRL + O_STATUS, sreg);
      if (sreg[S_BRESP_ERR] !== 1'b1)
        fail($sformatf("BRESP=%b on the write burst but bresp_err is clear (STATUS=0x%08h)", code, sreg));
      if ((sreg & FATAL_MASK) == 0) fail("bresp_err is not in the fatal mask");
      $display("[G4] ok-B%0d: BRESP=%b forced on the MIG write response -> STATUS=0x%08h, bresp_err=1: the run is refused",
               e, code, sreg);
    end

    // ============================ P3b phase R: RRESP errors =====================================
    for (int e = 0; e < 2; e++) begin
      automatic logic [1:0] code = e ? 2'b11 : 2'b10;
      automatic logic [63:0] base = 64'h0006_0000 + 64'h1000 * e;
      p_start(base);
      p_inject(8, 20 + e);
      p_flush(sreg);
      check_clean($sformatf("before the RRESP=%b drain", code));
      reset_links();
      p_dma_arm(PS_DEST2, 64);
      fcode = code;
      force DUT.riscq_bd_i.smc_ddr_M00_AXI_RRESP = fcode;  // the MIG's read response, during the drain
      p_drain_go(base, 64);
      p_dma_wait(POLL_LIMIT, dmasr, sv);
      release DUT.riscq_bd_i.smc_ddr_M00_AXI_RRESP;
      if (sv !== 1'b1) fail($sformatf("RRESP=%b drain: the DMA did not complete (DMASR=0x%08h)", code, dmasr));
      ps_r32(CTRL + O_STATUS, sreg);
      if (sreg[S_RRESP_ERR] !== 1'b1)
        fail($sformatf("RRESP=%b on the drain but rresp_err is clear (STATUS=0x%08h)", code, sreg));
      $display("[G4] ok-R%0d: RRESP=%b forced on the MIG read data -> the drain completed (%0d AXIS beats) but STATUS=0x%08h, rresp_err=1: refused",
               e, code, axis_beats, sreg);
    end

    // ============================ P3b phase C: reset rejection ==================================
    p_start(64'h0008_0000);
    p_inject(8, 30);
    force DUT.riscq_bd_i.dsp_rst_peripheral_reset = 1'b1;   // the DSP domain's reset, mid-run
    #1_000_000;
    release DUT.riscq_bd_i.dsp_rst_peripheral_reset;
    k = 0; stat(S_DSP_IN_RESET, sv);
    while (sv === 1'b1 && k < POLL_LIMIT) begin #1000; k++; stat(S_DSP_IN_RESET, sv); end
    #20000;
    ps_r32(CTRL + O_STATUS, sreg); ps_r32(CTRL + O_RUN_BASE, rv);
    acc_total = 0;
    for (k = 0; k < n_ch; k++) begin ps_r32(CTRL + O_ACCEPTED + 4*k, pv); acc_total += pv; end
    // the certification ddr.py applies: run active until the flush, write_done, run_base == wr_base, exact counts
    if (sreg[S_RUN_ACTIVE] === 1'b1) fail($sformatf("the run survived a DSP reset (STATUS=0x%08h)", sreg));
    if (sreg[S_WRITE_DONE] === 1'b1 && rv === 32'h0008_0000 && acc_total == 8)
      fail("a run interrupted by a DSP reset would still certify");
    $display("[G4] ok-C: DSP reset mid-run -> STATUS=0x%08h run_active=0, run_base=0x%08h, accepted=%0d: the run cannot be certified",
             sreg, rv, acc_total);
    p_start(64'h0009_0000);
    p_inject(12, 31);
    p_flush(sreg);
    check_clean("the run after the DSP reset");
    reset_links();
    p_dma_arm(PS_DEST2, 96);
    p_drain_go(64'h0009_0000, 96);
    p_dma_wait(POLL_LIMIT, dmasr, sv);
    if (sv !== 1'b1 || (dmasr & 32'h770)) fail($sformatf("post-reset drain: DMASR=0x%08h", dmasr));
    p_verify(12, PS_DEST2, "the run after the DSP reset");
    $display("[G4] ok-C: the next run is byte-exact at AXIS, S2MM and HP0");

    // ============================ P3b phase D: DMA truncation recovery ==========================
    // The drain is started with the S2MM channel still HALTED (a halted DMA holds TREADY low), so the
    // uplink's AXIS is valid and stalled, its FIFO full and R back-pressured at the MIG. The DMA is then
    // armed, and the DSP reset is forced at its first accepted beat: the reset hold discards the R beats
    // still owed, the DDR half resets, and the AXIS packet stops without TLAST, part-way through.
    // (Arming first and resetting at beat 4 was tried: the 32-beat drain finished before the reset landed.)
    p_start(64'h000A_0000);
    p_inject(256, 40);                                        // 2 KiB: a 64-beat drain
    p_flush(sreg);
    check_clean("before the truncated drain");
    reset_links();
    p_dma_soft_reset();                                      // the previous phases left it running: halt it
    ps_r32(DMA + O_S2MM_DMASR, dmasr);
    if (dmasr[0] !== 1'b1) fail($sformatf("the S2MM channel is not halted before the drain (DMASR=0x%08h)", dmasr));
    p_drain_go(64'h000A_0000, 2048);
    #2_000_000;                                              // the uplink fills its FIFO against TREADY low
    // (a halted S2MM still takes a few beats into its input buffer before TREADY drops: measured 4)
    k = axis_beats;
    $display("[G4] D: the halted S2MM took %0d beat(s) before holding TREADY low", k);
    if (k >= 16) fail($sformatf("%0d AXIS beats accepted by a halted DMA", k));
    fork
      begin
        wait (axis_beats >= k + 1);
        force DUT.riscq_bd_i.dsp_rst_peripheral_reset = 1'b1;   // DSP reset with the packet mid-flight
        #1_000_000;
        release DUT.riscq_bd_i.dsp_rst_peripheral_reset;
      end
    join_none
    p_dma_arm(PS_DEST2, 2048);
    #1_500_000;
    p_dma_wait(50, dmasr, sv);                               // 50 us, the stand-in for ddr_board's timeout
    $display("[G4] D: after the DSP reset: %0d of 64 AXIS beats, TLAST x%0d, DMASR=0x%08h (idle=%b)",
             axis_beats, axis_last, dmasr, sv);
    if (sv === 1'b1 || axis_last != 0 || axis_beats >= 64 || axis_beats == 0)
      fail($sformatf("the reset did not truncate the packet (%0d beats, TLAST x%0d, idle %b) -- nothing to recover from",
                     axis_beats, axis_last, sv));
    ps_r32(CTRL + O_STATUS, sreg);
    if (sreg[S_WRITE_DONE] === 1'b1) fail($sformatf("write_done survived the reset (STATUS=0x%08h): ddr.py would certify", sreg));
    p_dma_soft_reset();                                      // ddr_board.dma_reset()
    $display("[G4] ok-D: S2MM timed out on the truncated packet; soft reset cleared, channel halted; write_done=0 (STATUS=0x%08h): refused",
             sreg);
    k = 0; stat(S_DSP_IN_RESET, sv);
    while (sv === 1'b1 && k < POLL_LIMIT) begin #1000; k++; stat(S_DSP_IN_RESET, sv); end
    p_start(64'h000C_0000);
    p_inject(12, 41);
    p_flush(sreg);
    check_clean("the run after the truncation");
    reset_links();
    p_dma_arm(PS_DEST2, 96);
    p_drain_go(64'h000C_0000, 96);
    p_dma_wait(POLL_LIMIT, dmasr, sv);
    if (sv !== 1'b1 || (dmasr & 32'h770)) fail($sformatf("drain after the S2MM soft reset: DMASR=0x%08h", dmasr));
    p_verify(12, PS_DEST2, "the drain after the S2MM soft reset");
    $display("[G4] ok-D: the next drain on the reset channel is byte-exact at AXIS, S2MM and HP0");
    // r24-#5 / r25-#4: state exactly what was proven and what was not. The oracle sits at the HP0 slave
    // port -- the design's last signal boundary. Persistence INSIDE the PS memory is not claimed here,
    // and G4b does NOT close it either: G4b's Micron model terminates the PL MIG interface, while HP0
    // still terminates in the same Zynq VIP. PS-memory persistence is provable only on HARDWARE (G6).
    $display("[G4] PASS: %0d injected results travelled uplink -> smc_ddr -> MIG -> mmu2 -> AXIS -> axi_dma -> smc_dma -> HP0, byte-exact at BOTH the DMA master port and the HP0 slave port, single INCR burst to 0x%08h with full byte strobes; B (BRESP SLVERR/DECERR), R (RRESP SLVERR/DECERR), C (DSP reset mid-run) and D (DMA truncation + S2MM soft-reset recovery) passed%0s",
             N_INJ, PS_DEST[31:0],
             (backdoor_disagreed == 0) ? " (and the VIP memory image agrees)"
                                       : " (PS-memory persistence NOT claimed -- provable only on hardware, see the read_mem note above)");
    $finish;
  end

  // hard wall-clock guard: a hang must fail, not block the flow
  initial begin
    #WALL_PS;
    fail("timeout -- the test did not finish");
  end

endmodule
