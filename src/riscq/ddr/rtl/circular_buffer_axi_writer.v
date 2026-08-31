`timescale 1ns/1ps

module circular_buffer_axi_writer #(
    parameter RD_DATA_WIDTH   = 256,
    parameter ADDR_WIDTH      = 4,    // circular buffer depth per bank: 2^ADDR_WIDTH words
    parameter AXI_ADDR_WIDTH  = 32
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // runtime base address input
    input  wire [AXI_ADDR_WIDTH-1:0]     base_addr,

    // SOFTWARE base-reset strobe (rd_clk, round 10): the host pulses
    // circuit_id_reset at run setup (already CDC'd wr->rd in mmu_readout);
    // it deterministically returns cur_axi_addr to base_addr at the run
    // boundary. This REPLACES the phase-fragile hardware auto-reset as the
    // load-bearing mechanism: the production auto-reset sites (gated by the
    // suspect `able_to_read` signal — production source even warns about it)
    // were board-proven to miss systematically (round 8/9, run_base=0x2800).
    // Honored ONLY in the quiescent ST_IDLE state (never mid-burst).
    input  wire                          base_reset,

    // Interface to circular_buffer (read side)
    input  wire                          able_to_read,        // from circular_buffer
    input  wire                          rd_empty,            // from circular_buffer
    input  wire [ADDR_WIDTH-1:0]         rd_addr_valid,       // last valid address in bank
    output wire [ADDR_WIDTH-1:0]         rd_addr,             // address into circular_buffer
    output reg                           rd_en,               // read enable to circular_buffer
    output reg                           read_finished,       // pulse to circular_buffer
    input  wire                          write_finished_ext, // external force finish signal
    output reg  [AXI_ADDR_WIDTH-1:0]     final_addr,
    output wire [AXI_ADDR_WIDTH-1:0]     cur_axi_addr_out,       // running write address for AWADDR
    output wire                          writer_idle,            // [qubic3 Fork C2] state==ST_IDLE (base_reset admissible)
    output reg                           addr_fault,             // [qubic3 Fork C4] sticky: ring wrap OR burst end > WRAP_LIMIT; cleared by base_reset
    output reg                           current_user_done,
    input  wire [RD_DATA_WIDTH-1:0]      rd_data,

    // AXI4-Full write address channel
    output reg  [AXI_ADDR_WIDTH-1:0]     m_axi_awaddr,
    output reg  [7:0]                    m_axi_awlen,
    output reg  [2:0]                    m_axi_awsize,
    output reg  [1:0]                    m_axi_awburst,
    output reg                           m_axi_awvalid,
    input  wire                          m_axi_awready,

    // AXI4-Full write data channel
    output wire [RD_DATA_WIDTH-1:0]      m_axi_wdata,
    output reg  [RD_DATA_WIDTH/8-1:0]    m_axi_wstrb,
    output wire                          m_axi_wlast,
    output reg                           m_axi_wvalid,
    input  wire                          m_axi_wready,

    // AXI4-Full write response channel
    input  wire [1:0]                    m_axi_bresp,
    input  wire                          m_axi_bvalid,
    output reg                           m_axi_bready
);

    // Debug signals
    reg  [31:0]                   debug_counter;
    reg  [31:0]                   max_gap;
    // ----------------------------------------------------------------
    // Local parameters
    // ----------------------------------------------------------------
    localparam integer AXI_SIZE = $clog2(RD_DATA_WIDTH/8); // bytes per beat = 2^AXI_SIZE
    localparam integer GAP_THRESHOLD = 120; // Debug threshold
`ifdef SIM_WRAP_LIMIT
    // DIAGNOSTIC BUILD ONLY (Codex round-8 ruling #7): reduced ring limit
    // for the writer-level wrap repro TB. Production/Vivado builds never
    // define SIM_WRAP_LIMIT and use the real 2 GiB constants below.
    localparam [AXI_ADDR_WIDTH-1:0] WRAP_LIMIT = `SIM_WRAP_LIMIT;
    localparam [AXI_ADDR_WIDTH-1:0] WRAP_SIZE  = `SIM_WRAP_LIMIT + 1;
`else
    localparam [AXI_ADDR_WIDTH-1:0] WRAP_LIMIT = 32'h7FFFFFFF;
    localparam [AXI_ADDR_WIDTH-1:0] WRAP_SIZE  = 32'h80000000; // == WRAP_LIMIT+1
`endif

    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_AW         = 3'd1;
    localparam [2:0] ST_READ_WRITE = 3'd2;
    localparam [2:0] ST_WAIT_B     = 3'd3;

    // ----------------------------------------------------------------
    // Registers
    // ----------------------------------------------------------------
    reg [2:0]                state;

    reg [ADDR_WIDTH:0]       bank_len;        // number of beats in current bank (0..2^ADDR_WIDTH)
    reg [ADDR_WIDTH:0]       beat_count;      // beats sent on AXI in this bank

    reg [AXI_ADDR_WIDTH-1:0] cur_axi_addr;        // running write address for AWADDR
    reg [AXI_ADDR_WIDTH-1:0] cur_axi_addr_next;   // next value

    reg                      last_burst;
    reg                      write_finished_ext_d;

    // Debug registers
    reg [31:0]               gap_timer;

    // For computing next address at end of a bank
    wire [AXI_ADDR_WIDTH-1:0] bank_bytes;
    wire [AXI_ADDR_WIDTH-1:0] cur_axi_addr_plus_bank;

    assign bank_bytes             = bank_len * (RD_DATA_WIDTH/8);
    assign cur_axi_addr_plus_bank = cur_axi_addr + bank_bytes;
    assign cur_axi_addr_out        = cur_axi_addr;
    assign writer_idle             = (state == ST_IDLE);   // [qubic3 Fork C2]

    // ----------------------------------------------------------------
    // Combinational assignments
    // ----------------------------------------------------------------
    assign m_axi_wdata = rd_data;

    // wlast on final beat of a bank
    // [qubic3 Fork C] WLAST must be stable while WVALID && !WREADY (AXI A3.2.1); beat_count only
    // advances on a W handshake, so this is stable by construction.
    assign m_axi_wlast = m_axi_wvalid && (beat_count == bank_len - 1);

    // rd_addr driven from beat_count during READ_WRITE; 0 otherwise
    assign rd_addr =
        (state == ST_READ_WRITE)
            ? ((m_axi_wready && m_axi_wvalid) ? (beat_count + 1) : beat_count)
            : {ADDR_WIDTH{1'b0}};

    // ----------------------------------------------------------------
    // cur_axi_addr next-value logic (combinational)
    // ----------------------------------------------------------------
    always @* begin
        // default: hold current value
        cur_axi_addr_next = cur_axi_addr;

        // When we see a BVALID at end of a bank, advance address.
        // F4 (round 8, Codex-approved production-parity restoration): the
        // production writer (gateware_data_comm mmu) RESETS the pointer to
        // base_addr on the run's LAST bank — the para lineage deleted this
        // branch (its old comment here even said "never resets to
        // base_addr"), letting partial run tails misalign every later run;
        // a misaligned base makes some bank STRADDLE the 2 GiB ring seam
        // (this FSM issues one whole-bank burst with no boundary split),
        // writing the tail past the ring and leaving a never-written hole
        // at [0, eps) — board-proven (A/B campaign 2026-06-12). With the
        // reset restored, every run starts at the aligned base and full
        // fixed-size banks can never straddle the seam.
        if (state == ST_WAIT_B && m_axi_bvalid) begin
            if (last_burst) begin
                // On the last burst, reset pointer back to base_addr
                cur_axi_addr_next = base_addr;
            end else begin
                if (cur_axi_addr_plus_bank > WRAP_LIMIT) begin
                    cur_axi_addr_next = cur_axi_addr_plus_bank - WRAP_SIZE;
                end else begin
                    cur_axi_addr_next = cur_axi_addr_plus_bank;
                end
            end
        end
    end

    // ----------------------------------------------------------------
    // Sequential logic (synchronous reset)
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            read_finished <= 1'b0;

            bank_len     <= {ADDR_WIDTH+1{1'b0}};
            beat_count   <= {ADDR_WIDTH+1{1'b0}};

            cur_axi_addr <= {AXI_ADDR_WIDTH{1'b0}};
            rd_en        <= 1'b0;
            m_axi_awaddr <= {AXI_ADDR_WIDTH{1'b0}};
            m_axi_awlen  <= 8'd0;
            m_axi_awsize <= AXI_SIZE[2:0];
            m_axi_awburst<= 2'b01; // INCR
            m_axi_awvalid<= 1'b0;

            m_axi_wstrb  <= {RD_DATA_WIDTH/8{1'b0}};
            m_axi_wvalid <= 1'b0;

            m_axi_bready <= 1'b0;

            write_finished_ext_d <= 1'b0;
            last_burst           <= 1'b0;
            final_addr           <= {AXI_ADDR_WIDTH{1'b0}};
            current_user_done    <= 1'b0;
            addr_fault           <= 1'b0;   // [qubic3 Fork C4]

            // Reset debug outputs and timer
            debug_counter        <= 32'd0;
            max_gap              <= 32'd0;
            gap_timer            <= 32'd0;
        end else begin
            // Update cur_axi_addr from next-value logic
            cur_axi_addr <= cur_axi_addr_next;

            // Software base-reset (round 10): quiescent-only, takes priority
            // over the next-value update above (later NBA wins). In ST_IDLE
            // there is no AW/W in flight and no pending B wait, so resetting
            // the pointer cannot truncate a burst. A pulse arriving while
            // non-quiescent is IGNORED (Codex ruling); software catches that
            // via the CUR_ADDR!=0 readback + the [F4] sentinel.
            if (base_reset && state == ST_IDLE) begin
                cur_axi_addr         <= base_addr;
                write_finished_ext_d <= 1'b0;
                last_burst           <= 1'b0;
                addr_fault           <= 1'b0;   // [qubic3 Fork C4] per-run sticky
            end

            // [qubic3 Fork C4] address-fault detection (sticky until base_reset):
            //  (a) the ring-wrap branch fires on a non-final bank advance;
            //  (b) an issued burst ends past WRAP_LIMIT (the final-burst path bypasses the wrap branch).
            if (state == ST_WAIT_B && m_axi_bvalid && !last_burst && (cur_axi_addr_plus_bank > WRAP_LIMIT))
                addr_fault <= 1'b1;
            if (state == ST_IDLE && able_to_read && !rd_empty && !base_reset &&
                ((cur_axi_addr + (({1'b0, rd_addr_valid} + 1'b1) * (RD_DATA_WIDTH/8)) - 1) > WRAP_LIMIT))
                addr_fault <= 1'b1;

            // Defaults each cycle
            read_finished      <= 1'b0;
            m_axi_bready       <= 1'b1;
            current_user_done  <= 1'b0;
            rd_en              <= 1'b0;

            // Latch external finish flag
            if (write_finished_ext)
                write_finished_ext_d <= 1'b1;

            case (state)
                // ----------------------------------------------------
                // ST_IDLE: Wait for a non-empty ready bank
                // ----------------------------------------------------
                ST_IDLE: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    gap_timer     <= 32'd0; // Reset timer in IDLE
// warning: may have some logic problem for able_to_read signal
                    // [qubic3 Fork C3] base_reset has priority: never start a burst in the cycle the
                    // pointer is being re-based (awaddr would capture the OLD cur_axi_addr).
                    if (able_to_read && !base_reset) begin
                        if (!rd_empty) begin
                            // If this is the last burst in this user shot,
                            // mark it so we reset addr after it finishes.
                            if (write_finished_ext_d) begin
                                last_burst           <= 1'b1;
                                write_finished_ext_d <= 1'b0;
                            end

                            // Capture bank length = rd_addr_valid + 1
                            bank_len   <= {1'b0, rd_addr_valid} + 1'b1;
                            beat_count <= {ADDR_WIDTH+1{1'b0}};

                            // Start address for this burst
                            m_axi_awaddr  <= cur_axi_addr;
                            m_axi_awlen   <= ({1'b0, rd_addr_valid} + 1'b1) - 1; // AXI = beats-1
                            m_axi_awsize  <= AXI_SIZE[2:0];
                            m_axi_awburst <= 2'b01;  // INCR burst
                            m_axi_awvalid <= 1'b1;

                            state         <= ST_AW;
                        end else begin
                            // Buffer empty but external write-finish may be pending
                            if (write_finished_ext_d) begin
                                // No data to write, but last shot is done:
                                // report current address, reset pointer to base.
                                // (Round 9: second production reset site —
                                // para deleted BOTH resets; round 8 restored
                                // only the last_burst one. The board takes
                                // THIS path whenever the final bank beats
                                // the flag across the CDC, which is the
                                // systematic case for small runs — proven
                                // by the [F4] sentinel STOP, 2026-06-12.
                                // Direct NBA wins over the top-of-cycle
                                // cur_axi_addr <= cur_axi_addr_next, exactly
                                // as in the production source.)
                                current_user_done    <= 1'b1;
                                final_addr           <= cur_axi_addr;
                                cur_axi_addr         <= base_addr;
                                write_finished_ext_d <= 1'b0;
                                last_burst           <= 1'b0;
                            end
                            read_finished <= 1'b1;
                            state         <= ST_IDLE;
                        end
                    end
                end

                // ----------------------------------------------------
                // ST_AW: send AW for the entire bank
                // ----------------------------------------------------
                ST_AW: begin
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        // start reading data from circular_buffer
                        rd_en         <= 1'b1;
                        m_axi_wstrb   <= {RD_DATA_WIDTH/8{1'b1}};
                        m_axi_wvalid  <= 1'b1;
                        state         <= ST_READ_WRITE;
                    end
                end

                // ----------------------------------------------------
                // ST_READ_WRITE: issue reads to circular_buffer while
                // streaming data out on the AXI W channel.
                // ----------------------------------------------------
                ST_READ_WRITE: begin
                    // rd_en asserted when we want new data from circular buffer:
                    // here we keep it high while W is active.
                    if (m_axi_wvalid && m_axi_wready) begin
                        rd_en      <= 1'b1;
                        beat_count <= beat_count + 1'b1;

                        if (beat_count == bank_len - 1) begin
                            // Last beat of this bank
                            m_axi_wvalid <= 1'b0;
                            // Initialize gap timer to 0 immediately when last beat is accepted
                            gap_timer    <= 32'd0;
                            state        <= ST_WAIT_B;
                        end
                    end
                end

                // ----------------------------------------------------
                // ST_WAIT_B: wait for AXI write response
                // ----------------------------------------------------
                ST_WAIT_B: begin
                    m_axi_wvalid <= 1'b0;

                    if (m_axi_bvalid) begin
                        // Check if the accumulated gap exceeded threshold
                        if (gap_timer > GAP_THRESHOLD) begin
                            debug_counter <= debug_counter + 1;
                        end
                        // Update max_gap if the current gap is the new maximum
                        if (gap_timer > max_gap) begin
                            max_gap <= gap_timer;
                        end

                        // Normal logic below...
                        if (last_burst) begin
                            final_addr        <= cur_axi_addr_plus_bank;
                            current_user_done <= 1'b1;
                            last_burst        <= 1'b0;
                        end

                        read_finished <= 1'b1;
                        state         <= ST_IDLE;
                    end else begin
                        // Increment timer while waiting for BVALID
                        gap_timer <= gap_timer + 1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
