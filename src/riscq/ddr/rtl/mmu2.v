`timescale 1ns/1ps

module mmu2 #(
    parameter integer AXI_ADDR_WIDTH = 32,
    parameter integer AXI_DATA_WIDTH = 256,  // 32 bytes
    parameter integer AXI_ID_WIDTH   = 4,
    parameter integer FIFO_ADDR_BITS = 4     // FIFO depth = 2^N beats
)(
    input  wire                             clk,
    input  wire                             rst_n,

    // Control
    input  wire                             start,        // pulse to begin
    output reg                              busy,
    output reg                              done,         // 1-cycle pulse

    // Read range
    input  wire [AXI_ADDR_WIDTH-1:0]        base_addr,
    input  wire [AXI_ADDR_WIDTH:0]          size_bytes,

    // AXI Read Address
    output reg  [AXI_ID_WIDTH-1:0]          arid,
    output reg  [AXI_ADDR_WIDTH-1:0]        araddr,
    output reg  [7:0]                       arlen,
    output reg  [2:0]                       arsize,
    output reg  [1:0]                       arburst,
    output reg                              arvalid,
    input  wire                             arready,

    // AXI Read Data
    input  wire [AXI_ID_WIDTH-1:0]          rid,
    input  wire [AXI_DATA_WIDTH-1:0]        rdata,
    input  wire [1:0]                       rresp,
    input  wire                             rlast,
    input  wire                             rvalid,
    output wire                             rready,

    // AXI4-Stream output
    output wire [AXI_DATA_WIDTH-1:0]        m_axis_tdata,
    output wire                             m_axis_tvalid,
    input  wire                             m_axis_tready,
    output wire                             m_axis_tlast
);

    // ---------------- Parameters ----------------
    localparam integer BYTES_PER_BEAT = AXI_DATA_WIDTH/8;      // 32
    localparam integer ADDR_LSB       = $clog2(BYTES_PER_BEAT);
    localparam [1:0]   AXI_BURST_INCR = 2'b01;
    localparam integer WORDS_AW       = AXI_ADDR_WIDTH+1-ADDR_LSB;

    // Total beats expected = size_bytes / 32
    wire [WORDS_AW-1:0] total_words = size_bytes[AXI_ADDR_WIDTH:ADDR_LSB];

    // =====================================================================
    // FIFO (simple synchronous FIFO)
    // =====================================================================
    wire fifo_full, fifo_afull, fifo_empty;
    wire [AXI_DATA_WIDTH-1:0] fifo_rd_data;
    wire fifo_rd_en;

    // FIFO write = actual read-data handshake
    wire fifo_wr_en = (rvalid & rready);

    // Ensure async_fifo_same is available in your project
    async_fifo_same #(
        .DW    (AXI_DATA_WIDTH),
        .DEPTH (1 << FIFO_ADDR_BITS)
    ) u_fifo (
        .clk         (clk),
        .rstn        (rst_n),
        .wr_en       (fifo_wr_en),
        .wr_data     (rdata),
        .full        (fifo_full),
        .almost_full (fifo_afull),
        .rd_en       (fifo_rd_en),
        .rd_data     (fifo_rd_data),
        .empty       (fifo_empty)
    );

    // =====================================================================
    // AXIS Output
    // =====================================================================
    reg  [WORDS_AW-1:0] beats_sent;

    assign m_axis_tdata  = fifo_rd_data;
    assign m_axis_tvalid = !fifo_empty && (beats_sent < total_words);

    assign fifo_rd_en = m_axis_tvalid & m_axis_tready;

    wire axis_fire = fifo_rd_en;

    assign m_axis_tlast = 
        m_axis_tvalid && 
        (beats_sent + {{(WORDS_AW-1){1'b0}},1'b1} == total_words);

    // Beat counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            beats_sent <= 0;
        else if (start && !busy)
            beats_sent <= 0;
        else if (axis_fire)
            beats_sent <= beats_sent + 1;
    end

    // =====================================================================
    // AXI Read Logic (TIMING FIXED via Down-Counter)
    // =====================================================================
    reg [WORDS_AW-1:0] word_idx;        // Counts received data (into FIFO)
    
    // TIMING FIX: Replaced 'words_requested' accumulator with 'beats_remaining' down-counter
    // This simplifies the combinatorial path for next-state calculation.
    reg [WORDS_AW-1:0] beats_remaining; 
    
    reg                outstanding;     // 1 if a burst is currently active

    assign rready = outstanding && !fifo_afull;

    // ---------------------------------------------------------------------
    // Burst Size Calculation
    // ---------------------------------------------------------------------
    // Logic is simpler now: Min(256, beats_remaining)
    // If beats_remaining > 256, len=256. Else len=beats_remaining.
    wire [8:0] current_burst_len_comb = (beats_remaining > 256) ? 9'd256 : beats_remaining[8:0];
    
    // Calculate what remains AFTER this transaction
    // Synthesizer optimization: if (rem > 256) next = rem - 256; else next = 0;
    wire [WORDS_AW-1:0] next_remaining = beats_remaining - current_burst_len_comb;

    // Condition to issue AR
    wire can_issue_ar = busy && !outstanding && !fifo_afull && (beats_remaining > 0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arvalid         <= 0;
            araddr          <= 0;
            outstanding     <= 0;
            beats_remaining <= 0;

            // Registered Outputs (Reset)
            arid    <= {AXI_ID_WIDTH{1'b0}};
            arlen   <= 8'd0;
            arsize  <= ADDR_LSB[2:0];
            arburst <= AXI_BURST_INCR;
        end else begin
            
            // --------------------------------------------------------
            // 1. Transaction Start
            // --------------------------------------------------------
            if (start && !busy) begin
                araddr          <= base_addr;
                beats_remaining <= total_words; // Load down-counter
                
                // Pre-calculate arlen for the FIRST burst
                if (total_words > 256) 
                    arlen <= 8'd255;
                else 
                    arlen <= total_words[7:0] - 8'd1;
            end

            // --------------------------------------------------------
            // 2. Issue Read Request
            // --------------------------------------------------------
            if (can_issue_ar) begin
                arvalid <= 1'b1;
            end

            // --------------------------------------------------------
            // 3. AR Handshake & Update
            // --------------------------------------------------------
            if (arvalid && arready) begin
                
                // Debug Print
                // $display("[MMU2_AR] Time=%0t | Burst Issued | Addr=0x%h | Len=%0d", 
                //          $time, araddr, current_burst_len_comb);
                
                arvalid         <= 0;
                outstanding     <= 1'b1;
                
                // Update down-counter
                beats_remaining <= next_remaining;

                // Update address for the NEXT burst
                // Shift by ADDR_LSB (e.g. 5 for 32 bytes)
                araddr <= araddr + ({23'd0, current_burst_len_comb} << ADDR_LSB); 

                // Pre-calculate arlen for the NEXT burst
                // This removes the logic from the critical path of the next cycle
                if (next_remaining > 256)
                    arlen <= 8'd255;
                else if (next_remaining > 0)
                    arlen <= next_remaining[7:0] - 8'd1;
                else
                    arlen <= 8'd0; // Transaction will conclude
            end

            // --------------------------------------------------------
            // 4. Burst Completion
            // --------------------------------------------------------
            if (fifo_wr_en && rlast) begin
                outstanding <= 0;
            end
        end
    end

    // =====================================================================
    // Data Receive Counter & Done Flag
    // =====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy     <= 0;
            done     <= 0;
            word_idx <= 0;
        end else begin
            done <= 0;

            if (start && !busy) begin
                busy     <= 1;
                word_idx <= 0;
            end

            if (fifo_wr_en) begin
                // $display("[MMU2_R]  Time=%0t | Data=0x%h | Last=%b | Idx=%0d", 
                //          $time, rdata, rlast, word_idx);

                if (word_idx + 1 == total_words) begin
                    word_idx <= word_idx + 1;
                    busy     <= 0;
                    done     <= 1;
                end else begin
                    word_idx <= word_idx + 1;
                end
            end
        end
    end

endmodule
