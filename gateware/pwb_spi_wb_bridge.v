// Papilio Wishbone SPI-to-Wishbone Bridge with Multi-Width, Burst, and FIFO Support
//
// Protocol: [CMD][ADDR_HI][ADDR_LO][DATA...] (single-word)
//           [CMD][ADDR_HI][ADDR_LO][COUNT_H][COUNT_L][DATA1][DATA2]...[DATAn] (burst)
//
//   CMD byte encoding:
//     Bit 0:    0=Read, 1=Write
//     Bits 2:1: 00=8-bit, 01=16-bit, 10=24-bit, 11=32-bit data width
//     Bit 3:    0=Single-word, 1=Burst mode
//     Bits 7:4: Reserved (must be 0)
//
//   Single-word (bit 3=0):
//     8-bit:  [CMD][AH][AL] + 8-bit SPI transfer for data
//     16-bit: [CMD][AH][AL] + 16-bit SPI transfer for data
//     24-bit: [CMD][AH][AL] + 24-bit SPI transfer for data
//     32-bit: [CMD][AH][AL] + 32-bit SPI transfer for data
//
//   Burst mode (bit 3=1):
//     [CMD][AH][AL][COUNT_H][COUNT_L][DATA1][DATA2]...[DATAn]
//     COUNT = 16-bit word count (1-65535), big-endian (MSB first)
//     Address auto-increments after each word
//     FIFO buffered: SPI streams at full speed, Wishbone drains in background
//
// Backward compatible: CMD=0x00 (8-bit read), CMD=0x01 (8-bit write)
//
// SPI Mode 1: CPOL=0, CPHA=1
// Data sampled on falling edge, shifted out on rising edge
//
// FIFO Architecture (Phase 2):
//   - Burst writes: SPI → RX FIFO → Wishbone (decoupled)
//   - Burst reads: Wishbone pre-fetch → TX FIFO → SPI (no wait states)
//   - Single-word: Direct path (bypass FIFO for lowest latency)

module pwb_spi_wb_bridge #(
    parameter FIFO_DEPTH = 64,           // FIFO depth in 32-bit words
    parameter ALMOST_FULL_THRESHOLD = 4, // Words remaining before almost_full
    parameter ALMOST_EMPTY_THRESHOLD = 4 // Words remaining before almost_empty
) (
    input wire clk,
    input wire rst,
    
    // SPI Interface
    input wire spi_sclk,
    input wire spi_mosi,
    output wire spi_miso,
    input wire spi_cs_n,
    
    // Wishbone Master Interface (16-bit address, 32-bit data)
    output reg [15:0] wb_adr_o,
    output reg [31:0] wb_dat_o,
    input wire [31:0] wb_dat_i,
    output reg [3:0] wb_sel_o,      // Byte select for narrow transfers
    output reg wb_we_o,
    output reg wb_cyc_o,
    output reg wb_stb_o,
    input wire wb_ack_i,
    
    // FIFO Status Flags (for firmware monitoring)
    output wire fifo_rx_almost_full,
    output wire fifo_tx_almost_empty
);

    // =========================================================================
    // Command byte encoding
    // =========================================================================
    localparam CMD_RW_BIT = 0;           // Bit 0: 0=read, 1=write
    localparam CMD_WIDTH_LSB = 1;        // Bits 2:1: data width
    localparam CMD_WIDTH_MSB = 2;
    localparam CMD_BURST_BIT = 3;        // Bit 3: 0=single, 1=burst
    
    localparam WIDTH_8  = 2'b00;
    localparam WIDTH_16 = 2'b01;
    localparam WIDTH_24 = 2'b10;
    localparam WIDTH_32 = 2'b11;

    // =========================================================================
    // Synchronize SPI signals to system clock
    // =========================================================================
    reg spi_cs_n_d1, spi_cs_n_d2, spi_cs_n_d3;
    reg spi_sclk_d1, spi_sclk_d2, spi_sclk_d3;
    reg spi_mosi_d1, spi_mosi_d2;
    
    always @(posedge clk) begin
        spi_cs_n_d1 <= spi_cs_n;
        spi_cs_n_d2 <= spi_cs_n_d1;
        spi_cs_n_d3 <= spi_cs_n_d2;
        spi_sclk_d1 <= spi_sclk;
        spi_sclk_d2 <= spi_sclk_d1;
        spi_sclk_d3 <= spi_sclk_d2;
        spi_mosi_d1 <= spi_mosi;
        spi_mosi_d2 <= spi_mosi_d1;
    end
    
    wire spi_cs_active = !spi_cs_n_d2;
    wire spi_sclk_posedge = spi_sclk_d2 && !spi_sclk_d3;
    wire spi_sclk_negedge = !spi_sclk_d2 && spi_sclk_d3;
    
    // =========================================================================
    // SPI Receive registers
    // =========================================================================
    reg [5:0] bit_count;           // Expanded for 32-bit (up to 32 bits = 5 bits needed)
    reg [31:0] data_shift;         // 32-bit shift register for data phase
    reg [7:0] byte_shift;          // 8-bit shift register for header bytes
    reg [2:0] byte_count;          // Header byte counter (0-2 for CMD, AH, AL, or 0-3 for burst)
    reg [7:0] cmd;
    reg [7:0] addr_high;
    reg [7:0] addr_low;
    reg [15:0] addr;
    reg [31:0] data_in;
    reg transaction_complete;
    reg is_read;
    reg [1:0] data_width;          // 00=8, 01=16, 10=24, 11=32
    reg start_read;
    reg in_data_phase;             // True once header received, collecting data bits
    
    // =========================================================================
    // Burst mode registers
    // =========================================================================
    reg is_burst;                  // True when burst mode enabled (cmd[3]=1)
    reg [15:0] burst_count;        // Number of words in burst (16-bit COUNT, big-endian)
    reg [15:0] word_counter;       // Current word index in burst (0 to burst_count-1)
    reg in_count_phase;            // True when receiving COUNT bytes (burst only)
    reg count_byte_num;            // Which COUNT byte we're receiving (0=high, 1=low)
    reg [7:0] count_high;          // High byte of COUNT (received first)
    reg burst_word_done;           // Pulse when one burst word completes
    
    // =========================================================================
    // FIFO Instantiation (Phase 2: Burst buffering)
    // Uses 32-bit words to handle all widths (8/16/24/32-bit)
    // =========================================================================
    localparam FIFO_ADDR_WIDTH = $clog2(FIFO_DEPTH);
    
    // RX FIFO signals (burst writes: SPI → FIFO → Wishbone)
    reg [31:0] rx_fifo_wr_data;
    reg rx_fifo_wr_valid;
    wire rx_fifo_wr_ready;
    wire [31:0] rx_fifo_rd_data;
    wire rx_fifo_rd_valid;
    reg rx_fifo_rd_ready;
    wire rx_fifo_full, rx_fifo_empty;
    wire rx_fifo_almost_full, rx_fifo_almost_empty;
    
    // TX FIFO signals (burst reads: Wishbone → FIFO → SPI)
    reg [31:0] tx_fifo_wr_data;
    reg tx_fifo_wr_valid;
    wire tx_fifo_wr_ready;
    wire [31:0] tx_fifo_rd_data;
    wire tx_fifo_rd_valid;
    reg tx_fifo_rd_ready;
    wire tx_fifo_full, tx_fifo_empty;
    wire tx_fifo_almost_full, tx_fifo_almost_empty;
    
    // FIFO reset (active low for fifo_sync)
    wire fifo_rst_n = ~rst;
    
    // RX FIFO instance
    fifo_sync #(
        .DATA_WIDTH(32),
        .DEPTH(FIFO_DEPTH),
        .ALMOST_FULL_THRESHOLD(ALMOST_FULL_THRESHOLD),
        .ALMOST_EMPTY_THRESHOLD(ALMOST_EMPTY_THRESHOLD)
    ) rx_fifo (
        .clk(clk),
        .rst_n(fifo_rst_n),
        .wr_data(rx_fifo_wr_data),
        .wr_valid(rx_fifo_wr_valid),
        .wr_ready(rx_fifo_wr_ready),
        .rd_data(rx_fifo_rd_data),
        .rd_valid(rx_fifo_rd_valid),
        .rd_ready(rx_fifo_rd_ready),
        .full(rx_fifo_full),
        .empty(rx_fifo_empty),
        .almost_full(rx_fifo_almost_full),
        .almost_empty(rx_fifo_almost_empty),
        .count()  // Not used
    );
    
    // TX FIFO instance
    fifo_sync #(
        .DATA_WIDTH(32),
        .DEPTH(FIFO_DEPTH),
        .ALMOST_FULL_THRESHOLD(ALMOST_FULL_THRESHOLD),
        .ALMOST_EMPTY_THRESHOLD(ALMOST_EMPTY_THRESHOLD)
    ) tx_fifo (
        .clk(clk),
        .rst_n(fifo_rst_n),
        .wr_data(tx_fifo_wr_data),
        .wr_valid(tx_fifo_wr_valid),
        .wr_ready(tx_fifo_wr_ready),
        .rd_data(tx_fifo_rd_data),
        .rd_valid(tx_fifo_rd_valid),
        .rd_ready(tx_fifo_rd_ready),
        .full(tx_fifo_full),
        .empty(tx_fifo_empty),
        .almost_full(tx_fifo_almost_full),
        .almost_empty(tx_fifo_almost_empty),
        .count()  // Not used
    );
    
    // Expose status flags to firmware
    assign fifo_rx_almost_full = rx_fifo_almost_full;
    assign fifo_tx_almost_empty = tx_fifo_almost_empty;
    
    // =========================================================================
    // FIFO Drain/Fill State Machines
    // =========================================================================
    // RX Drain states (burst write: FIFO → Wishbone)
    localparam RX_DRAIN_IDLE = 2'd0;
    localparam RX_DRAIN_WB_START = 2'd1;
    localparam RX_DRAIN_WB_WAIT = 2'd2;
    localparam RX_DRAIN_DONE = 2'd3;
    
    reg [1:0] rx_drain_state;
    reg [15:0] rx_drain_count;     // Words remaining to drain (16-bit)
    reg [15:0] rx_drain_addr;      // Current Wishbone address for drain
    reg [1:0] rx_drain_width;      // Width for byte select
    reg rx_drain_active;           // True when draining burst write data

    // TX Pre-fetch states (burst read: Wishbone → FIFO)
    localparam TX_PREFETCH_IDLE = 2'd0;
    localparam TX_PREFETCH_WB_START = 2'd1;
    localparam TX_PREFETCH_WB_WAIT = 2'd2;
    localparam TX_PREFETCH_PUSH = 2'd3;

    reg [1:0] tx_prefetch_state;
    reg [15:0] tx_prefetch_count;  // Words remaining to prefetch (16-bit)
    reg [15:0] tx_prefetch_addr;   // Current Wishbone address for prefetch
    reg [1:0] tx_prefetch_width;   // Width for byte select
    reg tx_prefetch_active;        // True when prefetching burst read data
    reg tx_prefetch_done;          // All words prefetched into TX FIFO
    
    // =========================================================================
    // SPI Transmit registers
    // =========================================================================
    reg [31:0] tx_shift;
    reg [31:0] read_data;
    reg read_data_valid;
    reg tx_active;
    reg [5:0] tx_bit_count;
    reg tx_data_loaded;
    reg first_bit_sent;
    reg tx_word_done;          // Pulse when TX word transmission completes (for burst read)
    
    // =========================================================================
    // Wishbone done pulse (used by SPI receive to increment burst addr)
    // =========================================================================
    reg wb_done_pulse;         // Pulse from WB state machine when transaction completes
    
    // =========================================================================
    // MISO output - drive from transmit shift register MSB
    // Tri-state when CS is inactive (active low logic: cs_n=1 -> inactive)
    // =========================================================================
    assign spi_miso = (spi_cs_active) ? tx_shift[31] : 1'bz;
    
    // =========================================================================
    // Calculate expected data bits based on width
    // =========================================================================
    function [5:0] get_data_bits;
        input [1:0] width;
        begin
            case (width)
                WIDTH_8:  get_data_bits = 6'd8;
                WIDTH_16: get_data_bits = 6'd16;
                WIDTH_24: get_data_bits = 6'd24;
                WIDTH_32: get_data_bits = 6'd32;
                default:  get_data_bits = 6'd8;
            endcase
        end
    endfunction
    
    // =========================================================================
    // SPI Receive Logic (Mode 1: sample on falling edge of SCLK)
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_count <= 0;
            byte_shift <= 0;
            data_shift <= 0;
            byte_count <= 0;
            cmd <= 0;
            addr_high <= 0;
            addr_low <= 0;
            addr <= 0;
            data_in <= 0;
            transaction_complete <= 0;
            is_read <= 0;
            data_width <= WIDTH_8;
            start_read <= 0;
            in_data_phase <= 0;
            // Burst mode registers
            is_burst <= 0;
            burst_count <= 0;
            word_counter <= 0;
            in_count_phase <= 0;
            count_byte_num <= 0;
            count_high <= 0;
            burst_word_done <= 0;
        end else begin
            transaction_complete <= 0;
            start_read <= 0;
            burst_word_done <= 0;
            
            // Burst mode: increment address and word counter when WB transaction completes
            if (wb_done_pulse && is_burst) begin
                addr <= addr + 1;
                word_counter <= word_counter + 1;
            end
            
            if (!spi_cs_active) begin
                // Reset when CS goes inactive
                bit_count <= 0;
                byte_count <= 0;
                is_read <= 0;
                data_width <= WIDTH_8;
                in_data_phase <= 0;
                data_shift <= 0;
                // Burst mode reset
                is_burst <= 0;
                burst_count <= 0;
                word_counter <= 0;
                in_count_phase <= 0;
                count_byte_num <= 0;
                count_high <= 0;
            end else if (spi_sclk_negedge) begin
                if (!in_data_phase && !in_count_phase) begin
                    // Header phase: receive CMD, ADDR_HI, ADDR_LO as 8-bit bytes
                    byte_shift <= {byte_shift[6:0], spi_mosi_d2};
                    bit_count <= bit_count + 1;
                    
                    if (bit_count[2:0] == 3'd7) begin
                        // Completed receiving a header byte
                        bit_count <= 0;
                        
                        case (byte_count)
                            3'd0: begin
                                cmd <= {byte_shift[6:0], spi_mosi_d2};
                                // Bit 0 = 0 means read, = 1 means write
                                is_read <= ~spi_mosi_d2;  // LSB is the last bit shifted in
                                // Bits [2:1] encode width
                                data_width <= byte_shift[1:0];
                                // Bit 3 = burst enable
                                is_burst <= byte_shift[2];
                            end
                            3'd1: addr_high <= {byte_shift[6:0], spi_mosi_d2};
                            3'd2: begin
                                addr_low <= {byte_shift[6:0], spi_mosi_d2};
                                addr <= {addr_high, {byte_shift[6:0], spi_mosi_d2}};
                                
                                // Check if burst mode - need to receive COUNT byte
                                if (is_burst) begin
                                    in_count_phase <= 1;
                                    bit_count <= 0;
                                end else begin
                                    // Single-word mode - go directly to data phase
                                    in_data_phase <= 1;
                                    bit_count <= 0;
                                    word_counter <= 0;
                                    // For single-word reads, start Wishbone transaction now
                                    if (is_read) begin
                                        start_read <= 1;
                                    end
                                end
                            end
                        endcase
                        
                        if (byte_count < 3'd7)
                            byte_count <= byte_count + 1;
                    end
                end else if (in_count_phase) begin
                    // COUNT phase (burst mode only): receive 16-bit word count (big-endian)
                    byte_shift <= {byte_shift[6:0], spi_mosi_d2};
                    bit_count <= bit_count + 1;

                    if (bit_count[2:0] == 3'd7) begin
                        // Completed one COUNT byte
                        if (count_byte_num == 1'b0) begin
                            // First byte (high byte) received
                            count_high <= {byte_shift[6:0], spi_mosi_d2};
                            count_byte_num <= 1'b1;
                            bit_count <= 0;
                        end else begin
                            // Second byte (low byte) received - assemble 16-bit COUNT
                            burst_count <= {count_high, {byte_shift[6:0], spi_mosi_d2}};
                            in_count_phase <= 0;
                            in_data_phase <= 1;
                            bit_count <= 0;
                            word_counter <= 0;
                            count_byte_num <= 0;
                            // For burst reads, start first Wishbone read now
                            if (is_read) begin
                                start_read <= 1;
                            end
                        end
                    end
                end else begin
                    // Data phase: receive data bits based on width
                    data_shift <= {data_shift[30:0], spi_mosi_d2};
                    bit_count <= bit_count + 1;
                    
                    // Check if we've received all data bits for current word
                    if (bit_count == get_data_bits(data_width) - 1) begin
                        // Capture the complete data value
                        // MSB-first: first bit shifted goes to MSB, last bit is LSB
                        case (data_width)
                            WIDTH_8:  data_in <= {24'b0, data_shift[6:0], spi_mosi_d2};
                            WIDTH_16: data_in <= {16'b0, data_shift[14:0], spi_mosi_d2};
                            WIDTH_24: data_in <= {8'b0, data_shift[22:0], spi_mosi_d2};
                            WIDTH_32: data_in <= {data_shift[30:0], spi_mosi_d2};
                            default:  data_in <= {24'b0, data_shift[6:0], spi_mosi_d2};
                        endcase
                        
                        // For SINGLE-WORD writes (non-burst), trigger Wishbone directly
                        // For BURST writes, data goes to FIFO (handled separately)
                        if (!is_read && !is_burst) begin
                            transaction_complete <= 1;
                        end
                        
                        // Burst mode: signal word done for counter/address update
                        if (is_burst) begin
                            burst_word_done <= 1;
                        end
                        
                        // Reset bit counter for next word (burst) or end (single)
                        bit_count <= 0;
                    end
                end
            end
        end
    end
    
    // =========================================================================
    // SPI Transmit Logic (Mode 1: shift out on rising edge of SCLK)
    // For burst reads: pulls data from TX FIFO
    // For single-word reads: uses read_data directly (bypass FIFO)
    // =========================================================================
    
    // TX data source selection: FIFO for burst, direct for single-word
    wire [31:0] tx_data_source = (is_burst && is_read) ? tx_fifo_rd_data : read_data;
    wire tx_data_available = (is_burst && is_read) ? tx_fifo_rd_valid : read_data_valid;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx_shift <= 32'h00000000;
            tx_active <= 0;
            tx_bit_count <= 0;
            tx_data_loaded <= 0;
            first_bit_sent <= 0;
            tx_word_done <= 0;
            tx_fifo_rd_ready <= 0;
        end else begin
            tx_word_done <= 0;  // Clear pulse each cycle
            tx_fifo_rd_ready <= 0;  // Default: not consuming FIFO data
            
            if (!spi_cs_active) begin
                // CS inactive - reset transmitter
                tx_shift <= 32'h00000000;
                tx_active <= 0;
                tx_bit_count <= 0;
                tx_data_loaded <= 0;
                first_bit_sent <= 0;
            end else begin
                // Load read data as soon as it becomes valid
                if (tx_data_available && !tx_data_loaded) begin
                    // Left-align data in shift register based on width
                    case (data_width)
                        WIDTH_8:  tx_shift <= {tx_data_source[7:0], 24'hFFFFFF};
                        WIDTH_16: tx_shift <= {tx_data_source[15:0], 16'hFFFF};
                        WIDTH_24: tx_shift <= {tx_data_source[23:0], 8'hFF};
                        WIDTH_32: tx_shift <= tx_data_source;
                        default:  tx_shift <= {tx_data_source[7:0], 24'hFFFFFF};
                    endcase
                    tx_active <= 1;
                    tx_bit_count <= 0;
                    tx_data_loaded <= 1;
                    first_bit_sent <= 0;
                    // For burst mode: consume data from TX FIFO
                    if (is_burst && is_read) begin
                        tx_fifo_rd_ready <= 1;
                    end
                end else if (spi_sclk_negedge && tx_active && !first_bit_sent) begin
                    // First falling edge - master samples MSB
                    first_bit_sent <= 1;
                end else if (spi_sclk_posedge && tx_active && first_bit_sent) begin
                    // Shift out on rising edge (after first sample)
                    if (tx_bit_count < get_data_bits(data_width) - 1) begin
                        tx_shift <= {tx_shift[30:0], 1'b1};
                        tx_bit_count <= tx_bit_count + 1;
                    end else begin
                        // Word transmission complete
                        tx_word_done <= 1;
                        tx_active <= 0;
                        tx_data_loaded <= 0;  // Ready to accept next word for burst
                        first_bit_sent <= 0;
                    end
                end
            end
        end
    end
    
    // =========================================================================
    // Wishbone state machine
    // =========================================================================
    localparam WB_IDLE = 3'd0;
    localparam WB_WAIT_ACK = 3'd1;
    localparam WB_DONE = 3'd2;
    localparam WB_BURST_READ_PREFETCH = 3'd3;
    localparam WB_BURST_WRITE_DRAIN = 3'd4;
    
    reg [2:0] wb_state;
    
    // =========================================================================
    // Byte select based on data width
    // =========================================================================
    function [3:0] get_byte_sel;
        input [1:0] width;
        begin
            case (width)
                WIDTH_8:  get_byte_sel = 4'b0001;
                WIDTH_16: get_byte_sel = 4'b0011;
                WIDTH_24: get_byte_sel = 4'b0111;
                WIDTH_32: get_byte_sel = 4'b1111;
                default:  get_byte_sel = 4'b0001;
            endcase
        end
    endfunction
    
    // =========================================================================
    // RX FIFO Write Logic (Burst Writes: SPI data → FIFO)
    // Push each received burst word to FIFO immediately
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_fifo_wr_data <= 32'd0;
            rx_fifo_wr_valid <= 1'b0;
        end else begin
            rx_fifo_wr_valid <= 1'b0;  // Default: not writing
            
            // On burst_word_done pulse (write), push data to RX FIFO
            if (burst_word_done && is_burst && !is_read && rx_fifo_wr_ready) begin
                rx_fifo_wr_data <= data_in;
                rx_fifo_wr_valid <= 1'b1;
            end
        end
    end
    
    // =========================================================================
    // TX FIFO Write Logic (controlled by WB state machine during prefetch)
    // =========================================================================
    // tx_fifo_wr_data and tx_fifo_wr_valid are set by WB state machine
    
    // =========================================================================
    // Wishbone State Machine (with FIFO-buffered burst support)
    // 
    // Single-word: Direct SPI ↔ Wishbone (no FIFO, lowest latency)
    // Burst writes: SPI → RX FIFO, then drain FIFO → Wishbone in background
    // Burst reads: Pre-fetch all COUNT words to TX FIFO before SPI transmits
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wb_state <= WB_IDLE;
            wb_adr_o <= 0;
            wb_dat_o <= 0;
            wb_sel_o <= 4'b1111;
            wb_we_o <= 0;
            wb_cyc_o <= 0;
            wb_stb_o <= 0;
            read_data <= 32'hFFFFFFFF;
            read_data_valid <= 0;
            wb_done_pulse <= 0;
            // FIFO drain/prefetch registers
            rx_drain_state <= RX_DRAIN_IDLE;
            rx_drain_count <= 0;
            rx_drain_addr <= 0;
            rx_drain_width <= WIDTH_8;
            rx_drain_active <= 0;
            rx_fifo_rd_ready <= 0;
            tx_prefetch_state <= TX_PREFETCH_IDLE;
            tx_prefetch_count <= 0;
            tx_prefetch_addr <= 0;
            tx_prefetch_width <= WIDTH_8;
            tx_prefetch_active <= 0;
            tx_prefetch_done <= 0;
            tx_fifo_wr_data <= 0;
            tx_fifo_wr_valid <= 0;
        end else begin
            wb_done_pulse <= 0;  // Clear pulse each cycle
            rx_fifo_rd_ready <= 0;  // Default: not consuming from RX FIFO
            tx_fifo_wr_valid <= 0;  // Default: not writing to TX FIFO
            
            // =====================================================================
            // Reset FIFO state machines when CS goes inactive
            // =====================================================================
            if (!spi_cs_active) begin
                read_data_valid <= 0;
                rx_drain_state <= RX_DRAIN_IDLE;
                rx_drain_active <= 0;
                tx_prefetch_state <= TX_PREFETCH_IDLE;
                tx_prefetch_active <= 0;
                tx_prefetch_done <= 0;
            end
            
            case (wb_state)
                WB_IDLE: begin
                    wb_cyc_o <= 0;
                    wb_stb_o <= 0;
                    
                    // Clear read_data_valid when CS goes inactive
                    if (!spi_cs_active) begin
                        read_data_valid <= 0;
                    end
                    // Clear read_data_valid when TX consumes the data (single-word mode)
                    else if (read_data_valid && tx_data_loaded && !is_burst) begin
                        read_data_valid <= 0;
                    end
                    
                    // =========================================================
                    // SINGLE-WORD READ: Start Wishbone read directly
                    // =========================================================
                    if (start_read && !is_burst) begin
                        wb_adr_o <= addr;
                        wb_sel_o <= get_byte_sel(data_width);
                        wb_we_o <= 0;
                        wb_cyc_o <= 1;
                        wb_stb_o <= 1;
                        wb_state <= WB_WAIT_ACK;
                        read_data_valid <= 0;
                    end
                    // =========================================================
                    // BURST READ: Start pre-fetching all COUNT words to TX FIFO
                    // =========================================================
                    else if (start_read && is_burst && !tx_prefetch_active) begin
                        tx_prefetch_addr <= addr;
                        tx_prefetch_count <= burst_count;
                        tx_prefetch_width <= data_width;
                        tx_prefetch_active <= 1;
                        tx_prefetch_done <= 0;
                        wb_state <= WB_BURST_READ_PREFETCH;
                    end
                    // =========================================================
                    // SINGLE-WORD WRITE: Execute Wishbone write directly
                    // =========================================================
                    else if (transaction_complete && cmd[CMD_RW_BIT] && !is_burst) begin
                        wb_adr_o <= addr;
                        wb_dat_o <= data_in;
                        wb_sel_o <= get_byte_sel(data_width);
                        wb_we_o <= 1;
                        wb_cyc_o <= 1;
                        wb_stb_o <= 1;
                        wb_state <= WB_WAIT_ACK;
                    end
                    // =========================================================
                    // BURST WRITE: Start draining RX FIFO when data available
                    // =========================================================
                    else if (is_burst && !is_read && rx_fifo_rd_valid && !rx_drain_active) begin
                        // Initialize drain operation
                        rx_drain_addr <= addr;
                        rx_drain_count <= burst_count;
                        rx_drain_width <= data_width;
                        rx_drain_active <= 1;
                        wb_state <= WB_BURST_WRITE_DRAIN;
                    end
                    // Continue draining if active and more data in FIFO
                    else if (rx_drain_active && rx_fifo_rd_valid) begin
                        wb_state <= WB_BURST_WRITE_DRAIN;
                    end
                end
                
                // =============================================================
                // SINGLE-WORD: Wait for Wishbone ACK
                // =============================================================
                WB_WAIT_ACK: begin
                    if (wb_ack_i) begin
                        if (!wb_we_o) begin
                            // Read completed - capture data for single-word
                            read_data <= wb_dat_i;
                            read_data_valid <= 1;
                        end
                        wb_cyc_o <= 0;
                        wb_stb_o <= 0;
                        wb_state <= WB_DONE;
                    end
                end
                
                // =============================================================
                // SINGLE-WORD: Transaction complete
                // =============================================================
                WB_DONE: begin
                    wb_done_pulse <= 1;
                    wb_state <= WB_IDLE;
                end
                
                // =============================================================
                // BURST READ: Pre-fetch all words to TX FIFO
                // =============================================================
                WB_BURST_READ_PREFETCH: begin
                    case (tx_prefetch_state)
                        TX_PREFETCH_IDLE: begin
                            if (tx_prefetch_count > 0 && tx_fifo_wr_ready) begin
                                // Start Wishbone read
                                wb_adr_o <= tx_prefetch_addr;
                                wb_sel_o <= get_byte_sel(tx_prefetch_width);
                                wb_we_o <= 0;
                                wb_cyc_o <= 1;
                                wb_stb_o <= 1;
                                tx_prefetch_state <= TX_PREFETCH_WB_WAIT;
                            end else if (tx_prefetch_count == 0) begin
                                // All words pre-fetched
                                tx_prefetch_done <= 1;
                                tx_prefetch_active <= 0;
                                wb_state <= WB_IDLE;
                            end
                        end
                        
                        TX_PREFETCH_WB_WAIT: begin
                            if (wb_ack_i) begin
                                // Push read data to TX FIFO
                                tx_fifo_wr_data <= wb_dat_i;
                                tx_fifo_wr_valid <= 1;
                                wb_cyc_o <= 0;
                                wb_stb_o <= 0;
                                tx_prefetch_state <= TX_PREFETCH_PUSH;
                            end
                        end
                        
                        TX_PREFETCH_PUSH: begin
                            // Increment address and decrement count
                            tx_prefetch_addr <= tx_prefetch_addr + 1;
                            tx_prefetch_count <= tx_prefetch_count - 1;
                            tx_prefetch_state <= TX_PREFETCH_IDLE;
                        end
                        
                        default: tx_prefetch_state <= TX_PREFETCH_IDLE;
                    endcase
                end
                
                // =============================================================
                // BURST WRITE: Drain RX FIFO to Wishbone
                // =============================================================
                WB_BURST_WRITE_DRAIN: begin
                    case (rx_drain_state)
                        RX_DRAIN_IDLE: begin
                            if (rx_fifo_rd_valid && rx_drain_count > 0) begin
                                // Read from FIFO and start Wishbone write
                                rx_fifo_rd_ready <= 1;
                                wb_adr_o <= rx_drain_addr;
                                wb_dat_o <= rx_fifo_rd_data;
                                wb_sel_o <= get_byte_sel(rx_drain_width);
                                wb_we_o <= 1;
                                wb_cyc_o <= 1;
                                wb_stb_o <= 1;
                                rx_drain_state <= RX_DRAIN_WB_WAIT;
                            end else if (rx_drain_count == 0) begin
                                // All words drained
                                rx_drain_active <= 0;
                                wb_state <= WB_IDLE;
                            end else begin
                                // Wait for more data in FIFO
                                wb_state <= WB_IDLE;
                            end
                        end
                        
                        RX_DRAIN_WB_WAIT: begin
                            if (wb_ack_i) begin
                                wb_cyc_o <= 0;
                                wb_stb_o <= 0;
                                rx_drain_state <= RX_DRAIN_DONE;
                            end
                        end
                        
                        RX_DRAIN_DONE: begin
                            // Increment address and decrement count
                            rx_drain_addr <= rx_drain_addr + 1;
                            rx_drain_count <= rx_drain_count - 1;
                            rx_drain_state <= RX_DRAIN_IDLE;
                        end
                        
                        default: rx_drain_state <= RX_DRAIN_IDLE;
                    endcase
                end
                
                default: wb_state <= WB_IDLE;
            endcase
        end
    end

endmodule
