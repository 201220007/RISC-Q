//no wait for switch
module circular_buffer3 #(
    parameter WR_DATA_WIDTH = 64,
    parameter RD_DATA_WIDTH = 256,
    parameter ADDR_WIDTH    = 4
)(
    input  wire                      wr_clk,
    input  wire                      wr_rst_n,
    input  wire                      rd_clk,
    input  wire                      rd_rst_n,
    input  wire                      wr_en,
    input  wire [WR_DATA_WIDTH-1:0]  wr_data,
    input  wire                      write_finished_ext,
    input  wire                      rd_en,
    input  wire [ADDR_WIDTH-1:0]     rd_addr,
    input  wire                      read_finished,
    output wire [RD_DATA_WIDTH-1:0]  rd_data,
    output wire                      wr_en_out,
    output wire                      write_finished_out,
    output wire                      write_almost_finished_out,
    output wire                      able_to_read_out,
    output wire [ADDR_WIDTH-1:0]     rd_addr_valid_out,
    output wire                      rd_empty,
    // [qubic3 Fork A] write-side acceptance: 0 while the last slot of the bank is reached and the reader
    // credit is absent (bank stall). The wrapper MUST NOT assert wr_en while wr_ready=0; internally every
    // write-side effect (RAM weA, metadata, switch, recovery test, wr_en_out) is gated by wr_accept.
    output wire                      wr_ready
);

    // ----------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------
    localparam integer RATIO         = RD_DATA_WIDTH / WR_DATA_WIDTH;
    localparam integer RATIO_LG2     = (RATIO > 1) ? $clog2(RATIO) : 0;
    localparam integer RD_DEPTH      = (1 << ADDR_WIDTH);
    localparam integer WR_DEPTH      = RD_DEPTH * RATIO;
    localparam integer WR_ADDR_WIDTH = ADDR_WIDTH + RATIO_LG2;

    // ----------------------------------------------------------------
    // Registers
    // ----------------------------------------------------------------
    reg bank_sel_wr = 1'b0;
    reg bank_sel_rd = 1'b0;
    reg [WR_ADDR_WIDTH-1:0] wr_addr;

    reg write_finished        = 1'b0;
    reg write_almost_finished = 1'b0;


    reg [ADDR_WIDTH-1:0] last_valid_addr [0:1];
    reg buffer_empty [0:1];
    // Power-up INIT (bitstream/initial values, NOT reset): both banks start EMPTY. Without this,
    // buffer_empty[] powers up 0 ("not empty") until the FIRST wr_clk reset — and on the board the
    // wr clock (dspclk = CLK104/LMK) may be dead for seconds after the rd domain (MIG ui_clk) wakes,
    // so the rd side syncs the bogus "not empty" and the AXI writer emits ONE garbage beat to DDR
    // at boot, advancing cur_axi (board-observed on 02317191; masked by the host's cid_reset
    // base-reset). Same for last_valid_addr. Guard: test_plsv_v2b_readout_bootskew pre-start check.
    initial begin
        buffer_empty[0]    = 1'b1;
        buffer_empty[1]    = 1'b1;
        last_valid_addr[0] = {ADDR_WIDTH{1'b0}};
        last_valid_addr[1] = {ADDR_WIDTH{1'b0}};
    end

    (* ASYNC_REG = "TRUE" *) reg read_finished_sync1, read_finished_sync2;
    reg read_finished_sync2_d;
    // Switch credit: "the reader has finished with its bank". Semantic initial state = 1 (the reader
    // starts idle), so the credit is BORN in the wr domain instead of depending on the reader's one-shot
    // post-reset read_finished pulse crossing the 2-FF sync. On the board the rd domain (MIG ui_clk)
    // wakes tens of ms before dspclk (CLK104/LMK, programmed later by software); that one-shot 1-rd-cycle
    // pulse then crosses while the wr domain is unclocked and is LOST — the first bank switch (seamless
    // or flush Recovery) then never fires and the readout writer deadlocks in ST_IDLE forever
    // (board bug: final_addr=0; sim repro: test_plsv_v2b_readout_bootskew).
    reg read_finished_reg_wr = 1'b1;

    reg read_finished_d_rd, read_finished_reg_rd;
    (* ASYNC_REG = "TRUE" *) reg rd_bank_sel_sync1, rd_bank_sel_sync2;
    reg rd_bank_sel_prev;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH-1:0] last_valid_addr_sync1 [0:1], last_valid_addr_sync2 [0:1];
    (* ASYNC_REG = "TRUE" *) reg buffer_empty_sync1 [0:1], buffer_empty_sync2 [0:1];

    // ----------------------------------------------------------------
    // Outputs
    // ----------------------------------------------------------------
    assign write_finished_out        = write_finished;
    assign write_almost_finished_out = write_almost_finished;
    // [qubic3 Fork A] centralized acceptance (Codex r02 s7.1)
    assign wr_ready   = !((wr_addr == (WR_DEPTH - 1)) && !read_finished_reg_wr);
    wire   wr_accept  = wr_en && wr_ready;
    assign wr_en_out                 = wr_accept;

    wire wr_bank_sel    = bank_sel_wr;
    wire rd_bank_sel_wr = ~bank_sel_rd;
    wire rd_bank_sel    = rd_bank_sel_sync2;

    assign rd_addr_valid_out = last_valid_addr_sync2[rd_bank_sel];
    assign rd_empty          = buffer_empty_sync2[rd_bank_sel];
    assign able_to_read_out  = !read_finished & !read_finished_d_rd & !read_finished_reg_rd;

    integer i, j;

    // ----------------------------------------------------------------
    // Write Domain Logic
    // ----------------------------------------------------------------
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_addr                        <= {WR_ADDR_WIDTH{1'b0}};
            bank_sel_wr                    <= 1'b0;
            bank_sel_rd                    <= 1'b0;
            write_finished                 <= 1'b0;
            write_almost_finished          <= 1'b0;
            
            read_finished_sync1            <= 1'b0;
            read_finished_sync2            <= 1'b0;
            read_finished_sync2_d          <= 1'b0;
            read_finished_reg_wr           <= 1'b1;   // reader idle at reset = credit granted (see note above)

            for (i = 0; i < 2; i = i + 1) begin
                last_valid_addr[i] <= {ADDR_WIDTH{1'b0}};
                buffer_empty[i]    <= 1'b1;
            end
        end else begin

            // 1. Sync read_finished
            read_finished_sync1   <= read_finished;
            read_finished_sync2   <= read_finished_sync1;
            read_finished_sync2_d <= read_finished_sync2;

            if (!read_finished_sync2_d && read_finished_sync2) begin
                read_finished_reg_wr <= 1'b1;
            end

            // 2. Write Logic  ([qubic3 Fork A] gated by wr_accept)
            if (wr_accept) begin
                
                buffer_empty[wr_bank_sel]    <= 1'b0;
                last_valid_addr[wr_bank_sel] <= wr_addr[WR_ADDR_WIDTH-1:RATIO_LG2];

                // --- Case A: End of Bank (Current addr is MAX) ---
                if (wr_addr == (WR_DEPTH - 1)) begin
                    if (read_finished_reg_wr) begin
                        // SEAMLESS SWITCH
                        bank_sel_wr <= ~bank_sel_wr;
                        bank_sel_rd <= ~bank_sel_rd;
                        wr_addr     <= {WR_ADDR_WIDTH{1'b0}};
                        
                        read_finished_reg_wr <= 1'b0;
                        
                        // [FIX 3] Clean pipeline usage: Update the shadows
                        write_finished       <= 1'b1;
                        write_almost_finished <= 1'b1;

                        buffer_empty[~bank_sel_wr]    <= 1'b1;
                        last_valid_addr[~bank_sel_wr] <= {ADDR_WIDTH{1'b0}};
                    end else begin
                        // STALL
                        // Do not change originals; they hold their '1' state
                    end
                end 
                // --- Case B: Second to last (Current addr is MAX-1) ---
                else if (wr_addr == (WR_DEPTH - 2)) begin
                    wr_addr <= wr_addr + 1'b1;
                    // Prepare 'finished' to be HIGH next cycle
                    write_almost_finished <= 1'b1;
                end 
                // --- Case C: Normal ---
                else begin
                    wr_addr <= wr_addr + 1'b1;
                    write_almost_finished <= 1'b0;
                    write_finished        <= 1'b0;
                end
            end

            // External force (override)
            // Note: We must force both the output and the shadow to prevent glitches
            if (write_finished_ext && !write_finished) begin
                write_finished          <= 1'b1;
            end

            // [FIXED] Recovery from Stall
            // Remove (wr_addr == WR_DEPTH - 1) check to support external force finish
            if (write_finished && read_finished_reg_wr) begin
                // Only act here if we aren't writing this cycle (wr_en=0).
                // If wr_en=1, the logic is handled in the main write block above.
                if (!wr_accept) begin   // [qubic3 Fork A]
                    bank_sel_wr <= ~bank_sel_wr;
                    bank_sel_rd <= ~bank_sel_rd;
                    wr_addr     <= {WR_ADDR_WIDTH{1'b0}};
                    
                    read_finished_reg_wr <= 1'b0;
                    
                    // Reset Output and Shadow
                    write_finished                 <= 1'b0;
                    write_almost_finished          <= 1'b0;

                    // Clear the NEW bank
                    buffer_empty[~bank_sel_wr]    <= 1'b1;
                    last_valid_addr[~bank_sel_wr] <= {ADDR_WIDTH{1'b0}};
                end
            end
        end
    end
    
    // ... Read Domain Logic and RAM Instantiation (Unchanged) ...
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            read_finished_d_rd   <= 1'b0;
            read_finished_reg_rd <= 1'b0;
            rd_bank_sel_sync1    <= 1'b0;
            rd_bank_sel_sync2    <= 1'b0;
            rd_bank_sel_prev     <= 1'b0;
            for (j = 0; j < 2; j = j + 1) begin
                last_valid_addr_sync1[j] <= {ADDR_WIDTH{1'b0}};
                last_valid_addr_sync2[j] <= {ADDR_WIDTH{1'b0}};
                buffer_empty_sync1[j]    <= 1'b1;
                buffer_empty_sync2[j]    <= 1'b1;
            end
        end else begin
            read_finished_d_rd <= read_finished;
            if (!read_finished_d_rd && read_finished) read_finished_reg_rd <= 1'b1;

            rd_bank_sel_sync1 <= rd_bank_sel_wr;
            rd_bank_sel_sync2 <= rd_bank_sel_sync1;

            if (rd_bank_sel_sync2 != rd_bank_sel_prev) begin
                rd_bank_sel_prev     <= rd_bank_sel_sync2;
                read_finished_reg_rd <= 1'b0;
            end

            for (j = 0; j < 2; j = j + 1) begin
                last_valid_addr_sync1[j] <= last_valid_addr[j];
                last_valid_addr_sync2[j] <= last_valid_addr_sync1[j];
                buffer_empty_sync1[j]    <= buffer_empty[j];
                buffer_empty_sync2[j]    <= buffer_empty_sync1[j];
            end
        end
    end

    /* verilator lint_off PINMISSING */
    // cbuf_ram_read_wider = DEDICATED 1-cycle-read copy of asym_ram_sdp_read_wider (see its header).
    // The shared asym_ram (basename-dedup -> common-hdl copy in the integrated build) reads in 2
    // cycles for the dsp env/freq timing; the axi writer's 1-beat rd_addr look-ahead needs 1-cycle
    // reads here, else beat N+1 ships beat N's data (board 02317191 beat-duplication, 2026-07-01).
    cbuf_ram_read_wider #(
        .DATAWIDTHA (WR_DATA_WIDTH),
        .DATAWIDTHB (RD_DATA_WIDTH),
        .SIZEA      (WR_DEPTH * 2),        // [qubic3 Fork B] two banks; default 16384 wasted 128 KiB BRAM
        .SIZEB      (RD_DEPTH * 2),        // [qubic3 Fork B]
        .ADDRWIDTHA (WR_ADDR_WIDTH + 1),
        .ADDRWIDTHB (ADDR_WIDTH + 1),
        .RAM_STYLE  ("block"),
        .INIT_FILE  ("")
    ) u_ram (
        .clkA  (wr_clk),
        .clkB  (rd_clk),
        .weA   (wr_accept),   // [qubic3 Fork A]
        .addrA ({bank_sel_wr, wr_addr}),
        .diA   (wr_data),
        .addrB ({rd_bank_sel, rd_addr}),
        .doB   (rd_data)
    );
    /* verilator lint_on PINMISSING */

endmodule
