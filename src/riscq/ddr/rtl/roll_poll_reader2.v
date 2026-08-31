`timescale 1ns/1ps
// Round-robin reader with conditional pipeline for timing closure.
// When NUM_CH <= PIPELINE_THRESHOLD, uses original single-cycle logic.
// When NUM_CH >  PIPELINE_THRESHOLD, uses a NON-overlapped 3-phase
// pipeline (BARREL -> ENCODE -> CONSUME), adopted 2026-06-11 from the
// board-proven QubiC gateware roll_poll_reader.v (closes 500 MHz).
// Throughput: 1 sub-word / 3 cycles; system constraint pgap >= 64
// (= 3*NUM_CH drain + pipeline margin, see plan.md Appendix C rev2).
// The previous overlapped 1-sub-word/cycle version computed the whole
// barrel-shift + priority-encode + wrap-add in one cycle inside a
// feedback loop - the structural root of the 500 MHz setup violations.
// Do not care write_finished
module roll_poll_reader2 #(
    parameter integer NUM_CH     = 4,
    parameter integer DATA_WIDTH = 32
) (
    input  wire                        clk,
    input  wire                        rst_n,

    // NUM_CH input data/valid signals
    input  wire [NUM_CH-1:0]            data_valid,
    input  wire [NUM_CH*DATA_WIDTH-1:0] data_in,

    // Control inputs
    input  wire                        write_almost_finished,
    input  wire                        N_shot_finished,

    // Output to downstream writer
    output reg                         wr_en,
    output reg [DATA_WIDTH-1:0]        wr_data,

    // Read enables back to the sources (one-hot)
    output reg [NUM_CH-1:0]            rd_en,

    // Pulse high for 1 cycle after all valid data has been sent out in flush mode
    output reg                         write_finished_external
);

    // ------------------------------------------------------------
    // clog2 function (pure Verilog)
    // ------------------------------------------------------------
    function integer clog2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            clog2 = 0;
            while (v > 0) begin
                v = v >> 1;
                clog2 = clog2 + 1;
            end
        end
    endfunction

    localparam integer IDX_W = clog2(NUM_CH);
    localparam [IDX_W-1:0] LAST_CH = NUM_CH - 1;

    // Number of cycles to wait after N_shot_finished
    localparam integer FLUSH_DELAY    = 250;
    localparam integer FLUSH_DELAY_W  = clog2(FLUSH_DELAY);

    // Pipeline threshold: enable pipeline for high channel counts
    localparam integer PIPELINE_THRESHOLD = 8;

    // Common internal state
    reg [IDX_W-1:0] cur_idx;
    reg             flushing;
    reg             N_shot_finished_q;
    reg [FLUSH_DELAY_W-1:0] flush_delay_cnt;
    reg                     flush_delay_active;

    wire N_shot_finished_rise = N_shot_finished & ~N_shot_finished_q;
    wire no_valid_any         = (data_valid == {NUM_CH{1'b0}});

    // Legacy (unused but kept for interface compatibility)
    reg write_almost_finished_g;
    reg sel_found_prev;

    // Channel selection outputs (driven by generate block)
    reg [IDX_W-1:0] sel_idx;
    reg             sel_found;

generate
// ================================================================
// HIGH CHANNEL COUNT: Non-overlapped 3-phase pipeline (1 sub-word/3 cycles)
// ================================================================
if (NUM_CH > PIPELINE_THRESHOLD) begin : g_pipe

    // ============================================================
    // NON-overlapped 3-phase pipeline (blueprint: gateware_data_comm
    // top/src/mmu/roll_poll_reader.v g_pipe, board-proven at 500 MHz).
    //   Phase 0 BARREL : register the barrel-rotated data_valid vector
    //   Phase 1 ENCODE : priority-encode the REGISTERED vector, register it
    //   Phase 2 CONSUME: rd_en from registered index; latch data SAME cycle
    // Per-cycle combinational depth = one barrel shift OR one priority
    // encode OR one data mux - never the whole scan.
    // data_buffer contract: data_valid clears the cycle AFTER rd_en;
    // data persists after rd_en until the next write; write priority
    // over clear. Same-cycle data latch (CONSUME) therefore reads the
    // OLD entry even if a new write lands in the same cycle.
    // ============================================================

    // Step 1: rotate data_valid so that cur_idx maps to bit 0 (combinational)
    wire [2*NUM_CH-1:0] dv_doubled   = {data_valid, data_valid};
    wire [IDX_W:0]      cur_idx_ext  = {1'b0, cur_idx};
    wire [NUM_CH-1:0]   dv_rot       = dv_doubled[cur_idx_ext +: NUM_CH];

    // Registered barrel-shift output (sampled in BARREL phase)
    reg [NUM_CH-1:0] dv_rot_r;

    // Step 2: priority encoder on REGISTERED rotated vector
    integer j;
    reg [IDX_W-1:0] first_offset;
    reg             found_any;
    always @* begin
        found_any    = 1'b0;
        first_offset = {IDX_W{1'b0}};
        for (j = 0; j < NUM_CH; j = j + 1) begin
            if (!found_any && dv_rot_r[j]) begin
                found_any    = 1'b1;
                first_offset = j[IDX_W-1:0];
            end
        end
    end

    // Step 3: convert offset back to absolute channel index
    localparam [IDX_W:0] NUM_CH_EXT = NUM_CH[IDX_W:0];
    wire [IDX_W:0] abs_sum = {1'b0, cur_idx} + {1'b0, first_offset};
    always @* begin
        sel_found = found_any;
        if (abs_sum >= NUM_CH_EXT)
            sel_idx = abs_sum[IDX_W-1:0] - NUM_CH[IDX_W-1:0];
        else
            sel_idx = abs_sum[IDX_W-1:0];
    end

    // Pipeline registers
    reg [1:0]       phase;     // 0=BARREL, 1=ENCODE, 2=CONSUME
    reg [IDX_W-1:0] pipe_idx;
    reg             pipe_found;

    // Read Enable: driven by pipeline registers (CONSUME phase only)
    always @* begin
        rd_en = {NUM_CH{1'b0}};
        if (phase == 2'd2 && pipe_found) begin
            if (flushing) begin
                if (!no_valid_any)
                    rd_en[pipe_idx] = 1'b1;
            end else begin
                rd_en[pipe_idx] = 1'b1;
            end
        end
    end

    // Sequential Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_idx                 <= {IDX_W{1'b0}};
            flushing                <= 1'b0;
            N_shot_finished_q       <= 1'b0;
            wr_en                   <= 1'b0;
            wr_data                 <= {DATA_WIDTH{1'b0}};
            write_finished_external <= 1'b0;
            flush_delay_cnt         <= {FLUSH_DELAY_W{1'b0}};
            flush_delay_active      <= 1'b0;
            sel_found_prev          <= 1'b0;
            phase                   <= 2'd0;
            pipe_idx                <= {IDX_W{1'b0}};
            pipe_found              <= 1'b0;
            dv_rot_r                <= {NUM_CH{1'b0}};
        end else begin
            wr_en                   <= 1'b0;
            wr_data                 <= {DATA_WIDTH{1'b0}};
            write_finished_external <= 1'b0;
            N_shot_finished_q       <= N_shot_finished;
            sel_found_prev          <= sel_found;

            // ---- Flush delay logic (identical to g_direct) ----
            if (N_shot_finished_rise && !flushing) begin
                flush_delay_active <= 1'b1;
                flush_delay_cnt    <= {FLUSH_DELAY_W{1'b0}};
            end
            if (flush_delay_active && !flushing) begin
                if (flush_delay_cnt == FLUSH_DELAY - 1) begin
                    flushing           <= 1'b1;
                    flush_delay_active <= 1'b0;
                end else begin
                    flush_delay_cnt <= flush_delay_cnt + {{(FLUSH_DELAY_W-1){1'b0}}, 1'b1};
                end
            end

            // ---- Three-phase pipeline ----
            case (phase)
                2'd0: begin  // BARREL: register the barrel-shift result
                    dv_rot_r <= dv_rot;
                    phase    <= 2'd1;
                end
                2'd1: begin  // ENCODE: priority encoder on registered dv_rot_r
                    pipe_idx   <= sel_idx;
                    pipe_found <= sel_found;
                    phase      <= 2'd2;
                end
                2'd2: begin  // CONSUME: rd_en comb-high this cycle; latch data
                    phase <= 2'd0;
                    if (rd_en[pipe_idx]) begin
                        wr_en   <= 1'b1;
                        wr_data <= data_in[pipe_idx*DATA_WIDTH +: DATA_WIDTH];
                        if (pipe_idx == LAST_CH)
                            cur_idx <= {IDX_W{1'b0}};
                        else
                            cur_idx <= pipe_idx + {{(IDX_W-1){1'b0}}, 1'b1};
                    end
                end
                default: phase <= 2'd0;
            endcase

            // ---- Flush Completion ----
            // At BARREL phase nothing is in flight (pipe regs are stale and
            // rd_en only fires at CONSUME), so empty data_valid here means
            // every sub-word has been drained.
            if (flushing && no_valid_any && phase == 2'd0) begin
                write_finished_external <= 1'b1;
                flushing                <= 1'b0;
            end
        end
    end

    // Legacy combo (dead logic, kept byte-identical to pre-change file)
    always @(*) begin
        if (sel_found_prev == 1'b1)
            write_almost_finished_g = write_almost_finished;
        else
            write_almost_finished_g = write_almost_finished_g;
    end

end else begin : g_direct
// ================================================================
// LOW CHANNEL COUNT: Original single-cycle logic (identical to unmodified)
// ================================================================

    integer k;
    reg [IDX_W-1:0] scan_idx;

    always @* begin
        sel_found = 1'b0;
        sel_idx   = cur_idx;
        for (k = 0; k < NUM_CH; k = k + 1) begin
            scan_idx = cur_idx + k[IDX_W-1:0];
            if (scan_idx >= NUM_CH)
                scan_idx = scan_idx - NUM_CH;
            if (!sel_found && data_valid[scan_idx]) begin
                sel_found = 1'b1;
                sel_idx   = scan_idx;
            end
        end
    end

    // Read Enable: single-cycle, directly from sel_idx
    always @* begin
        rd_en = {NUM_CH{1'b0}};
        if (sel_found) begin
            if (flushing) begin
                if (!no_valid_any)
                    rd_en[sel_idx] = 1'b1;
            end else begin
                rd_en[sel_idx] = 1'b1;
            end
        end
    end

    // Sequential Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_idx                 <= {IDX_W{1'b0}};
            flushing                <= 1'b0;
            N_shot_finished_q       <= 1'b0;
            wr_en                   <= 1'b0;
            wr_data                 <= {DATA_WIDTH{1'b0}};
            write_finished_external <= 1'b0;
            flush_delay_cnt         <= {FLUSH_DELAY_W{1'b0}};
            flush_delay_active      <= 1'b0;
            sel_found_prev          <= 1'b0;
        end else begin
            wr_en                   <= 1'b0;
            wr_data                 <= {DATA_WIDTH{1'b0}};
            write_finished_external <= 1'b0;
            N_shot_finished_q       <= N_shot_finished;
            sel_found_prev          <= sel_found;

            // Flush delay
            if (N_shot_finished_rise && !flushing) begin
                flush_delay_active <= 1'b1;
                flush_delay_cnt    <= {FLUSH_DELAY_W{1'b0}};
            end
            if (flush_delay_active && !flushing) begin
                if (flush_delay_cnt == FLUSH_DELAY - 1) begin
                    flushing           <= 1'b1;
                    flush_delay_active <= 1'b0;
                end else begin
                    flush_delay_cnt <= flush_delay_cnt + {{(FLUSH_DELAY_W-1){1'b0}}, 1'b1};
                end
            end

            // Data Latching
            if (rd_en[sel_idx]) begin
                wr_en   <= 1'b1;
                wr_data <= data_in[sel_idx*DATA_WIDTH +: DATA_WIDTH];
                if (sel_idx == LAST_CH)
                    cur_idx <= {IDX_W{1'b0}};
                else
                    cur_idx <= sel_idx + {{(IDX_W-1){1'b0}}, 1'b1};
            end

            // Flush Completion
            if (flushing && no_valid_any) begin
                write_finished_external <= 1'b1;
                flushing                <= 1'b0;
            end
        end
    end

    // Legacy combo
    always @(*) begin
        if (sel_found_prev == 1'b1)
            write_almost_finished_g = write_almost_finished;
        else
            write_almost_finished_g = write_almost_finished_g;
    end

end
endgenerate

endmodule
