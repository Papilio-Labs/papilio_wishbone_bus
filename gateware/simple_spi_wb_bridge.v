// Simple SPI to Wishbone Bridge with Read/Write Support
// Protocol: [CMD][ADDR_HI][ADDR_LO][DATA]
//   CMD=0x00: Read  - returns data on MISO during 4th byte
//   CMD=0x01: Write - writes DATA to address
//
// SPI Mode 1: CPOL=0, CPHA=1
// Data sampled on falling edge, shifted out on rising edge (stable for master sample on falling)

module simple_spi_wb_bridge (
    input wire clk,
    input wire rst,
    
    // SPI Interface
    input wire spi_sclk,
    input wire spi_mosi,
    output wire spi_miso,
    input wire spi_cs_n,
    
    // Wishbone Master Interface (16-bit address, 8-bit data)
    output reg [15:0] wb_adr_o,
    output reg [7:0] wb_dat_o,
    input wire [7:0] wb_dat_i,
    output reg wb_we_o,
    output reg wb_cyc_o,
    output reg wb_stb_o,
    input wire wb_ack_i
);

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
    reg [2:0] bit_count;
    reg [7:0] byte_shift;
    reg [2:0] byte_count;
    reg [7:0] cmd;
    reg [7:0] addr_high;
    reg [7:0] addr_low;
    reg [15:0] addr;
    reg [7:0] data_in;
    reg transaction_complete;
    reg is_read;
    reg start_read;
    
    // =========================================================================
    // SPI Transmit registers
    // =========================================================================
    reg [7:0] tx_shift;
    reg [7:0] read_data;
    reg read_data_valid;
    reg tx_active;
    
    // =========================================================================
    // MISO output - drive from transmit shift register MSB
    // =========================================================================
    assign spi_miso = tx_shift[7];
    
    // =========================================================================
    // Wishbone state machine
    // =========================================================================
    localparam WB_IDLE = 2'd0;
    localparam WB_WAIT_ACK = 2'd1;
    localparam WB_DONE = 2'd2;
    
    reg [1:0] wb_state;
    
    // =========================================================================
    // SPI Receive Logic (Mode 1: sample on falling edge of SCLK)
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_count <= 0;
            byte_shift <= 0;
            byte_count <= 0;
            cmd <= 0;
            addr_high <= 0;
            addr_low <= 0;
            addr <= 0;
            data_in <= 0;
            transaction_complete <= 0;
            is_read <= 0;
            start_read <= 0;
        end else begin
            transaction_complete <= 0;
            start_read <= 0;
            
            if (!spi_cs_active) begin
                // Reset when CS goes inactive
                bit_count <= 0;
                byte_count <= 0;
                is_read <= 0;
            end else if (spi_sclk_negedge) begin
                // Shift in MOSI data on falling edge
                byte_shift <= {byte_shift[6:0], spi_mosi_d2};
                bit_count <= bit_count + 1;
                
                if (bit_count == 3'd7) begin
                    // Completed receiving a byte
                    bit_count <= 0;
                    
                    case (byte_count)
                        3'd0: begin
                            cmd <= {byte_shift[6:0], spi_mosi_d2};
                            is_read <= ({byte_shift[6:0], spi_mosi_d2} == 8'h00);
                        end
                        3'd1: addr_high <= {byte_shift[6:0], spi_mosi_d2};
                        3'd2: begin
                            addr_low <= {byte_shift[6:0], spi_mosi_d2};
                            addr <= {addr_high, {byte_shift[6:0], spi_mosi_d2}};
                            // For reads, start the Wishbone transaction now
                            if (is_read) begin
                                start_read <= 1;
                            end
                        end
                        3'd3: begin
                            data_in <= {byte_shift[6:0], spi_mosi_d2};
                            // For writes, trigger the transaction
                            if (!is_read) begin
                                transaction_complete <= 1;
                            end
                        end
                    endcase
                    
                    if (byte_count < 3'd7)
                        byte_count <= byte_count + 1;
                end
            end
        end
    end
    
    // =========================================================================
    // SPI Transmit Logic (Mode 1: shift out on rising edge of SCLK)
    // Data should be stable on falling edge (when master samples)
    // So we shift on rising edge to prepare for the next falling edge
    // =========================================================================
    reg [2:0] tx_bit_count;
    reg tx_data_loaded;
    reg first_bit_sent;  // Flag to skip first falling edge after load
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx_shift <= 8'hFF;
            tx_active <= 0;
            tx_bit_count <= 0;
            tx_data_loaded <= 0;
            first_bit_sent <= 0;
        end else begin
            if (!spi_cs_active) begin
                // CS inactive - reset transmitter
                tx_shift <= 8'hFF;
                tx_active <= 0;
                tx_bit_count <= 0;
                tx_data_loaded <= 0;
                first_bit_sent <= 0;
            end else begin
                // Load read data as soon as it becomes valid
                if (read_data_valid && !tx_data_loaded) begin
                    tx_shift <= read_data;
                    tx_active <= 1;
                    tx_bit_count <= 0;
                    tx_data_loaded <= 1;
                    first_bit_sent <= 0;  // Need to wait for first rising edge
                end else if (spi_sclk_negedge && tx_active && !first_bit_sent) begin
                    // First falling edge - master samples MSB
                    first_bit_sent <= 1;
                end else if (spi_sclk_posedge && tx_active && first_bit_sent && tx_bit_count < 3'd7) begin
                    // Shift out on rising edge (after first sample)
                    tx_shift <= {tx_shift[6:0], 1'b1};
                    tx_bit_count <= tx_bit_count + 1;
                end
            end
        end
    end
    
    // =========================================================================
    // Wishbone State Machine
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wb_state <= WB_IDLE;
            wb_adr_o <= 0;
            wb_dat_o <= 0;
            wb_we_o <= 0;
            wb_cyc_o <= 0;
            wb_stb_o <= 0;
            read_data <= 8'hFF;
            read_data_valid <= 0;
        end else begin
            case (wb_state)
                WB_IDLE: begin
                    wb_cyc_o <= 0;
                    wb_stb_o <= 0;
                    
                    // Clear read_data_valid when CS goes inactive
                    if (!spi_cs_active) begin
                        read_data_valid <= 0;
                    end
                    
                    // Start read when start_read pulse arrives
                    if (start_read) begin
                        wb_adr_o <= addr;
                        wb_we_o <= 0;
                        wb_cyc_o <= 1;
                        wb_stb_o <= 1;
                        wb_state <= WB_WAIT_ACK;
                        read_data_valid <= 0;
                    end
                    // Start write when transaction_complete for writes
                    else if (transaction_complete && cmd == 8'h01) begin
                        wb_adr_o <= addr;
                        wb_dat_o <= data_in;
                        wb_we_o <= 1;
                        wb_cyc_o <= 1;
                        wb_stb_o <= 1;
                        wb_state <= WB_WAIT_ACK;
                    end
                end
                
                WB_WAIT_ACK: begin
                    if (wb_ack_i) begin
                        if (!wb_we_o) begin
                            // Read completed - capture data
                            read_data <= wb_dat_i;
                            read_data_valid <= 1;
                        end
                        wb_cyc_o <= 0;
                        wb_stb_o <= 0;
                        wb_state <= WB_DONE;
                    end
                end
                
                WB_DONE: begin
                    wb_state <= WB_IDLE;
                end
                
                default: wb_state <= WB_IDLE;
            endcase
        end
    end

endmodule
