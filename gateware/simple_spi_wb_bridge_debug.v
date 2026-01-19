// Simple SPI to Wishbone Bridge with UART Debug
// Protocol: [CMD][ADDR][DATA] where CMD=0x01 for write

module simple_spi_wb_bridge_debug (
    input wire clk,
    input wire rst,
    
    // SPI Interface
    input wire spi_sclk,
    input wire spi_mosi,
    input wire spi_cs_n,
    
    // Wishbone Master Interface (16-bit address, 8-bit data)
    output reg [15:0] wb_adr_o,
    output reg [7:0] wb_dat_o,
    output reg wb_we_o,
    output reg wb_cyc_o,
    output reg wb_stb_o,
    input wire wb_ack_i,
    
    // UART Debug output
    output wire uart_tx
);

    // Synchronize SPI signals
    reg spi_cs_n_d1, spi_cs_n_d2;
    reg spi_sclk_d1, spi_sclk_d2;
    reg spi_mosi_d1, spi_mosi_d2;
    
    always @(posedge clk) begin
        spi_cs_n_d1 <= spi_cs_n;
        spi_cs_n_d2 <= spi_cs_n_d1;
        spi_sclk_d1 <= spi_sclk;
        spi_sclk_d2 <= spi_sclk_d1;
        spi_mosi_d1 <= spi_mosi;
        spi_mosi_d2 <= spi_mosi_d1;
    end
    
    wire spi_cs_active = !spi_cs_n_d2;
    wire spi_sclk_posedge = spi_sclk_d1 && !spi_sclk_d2;
    
    // Byte receiver - Extended to 4 bytes for 16-bit addressing
    reg [2:0] bit_count;
    reg [7:0] byte_shift;
    reg [2:0] byte_count;  // 4-byte command frame
    reg [7:0] cmd, addr_high, addr_low, data;
    reg [15:0] addr;  // 16-bit address
    reg transaction_complete;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_count <= 0;
            byte_shift <= 0;
            byte_count <= 0;
            cmd <= 0;
            addr_high <= 0;
            addr_low <= 0;
            addr <= 0;
            data <= 0;
            transaction_complete <= 0;
        end else begin
            transaction_complete <= 0;
            
            if (!spi_cs_active) begin
                bit_count <= 0;
                byte_count <= 0;
            end else if (spi_sclk_posedge) begin
                byte_shift <= {byte_shift[6:0], spi_mosi_d2};
                bit_count <= bit_count + 1;
                
                if (bit_count == 7) begin
                    bit_count <= 0;
                    
                    case (byte_count)
                        3'd0: cmd <= {byte_shift[6:0], spi_mosi_d2};
                        3'd1: addr_high <= {byte_shift[6:0], spi_mosi_d2};
                        3'd2: addr_low <= {byte_shift[6:0], spi_mosi_d2};
                        3'd3: begin
                            data <= {byte_shift[6:0], spi_mosi_d2};
                            addr <= {addr_high, addr_low};  // Combine address bytes
                            transaction_complete <= 1;
                        end
                    endcase
                    
                    byte_count <= byte_count + 1;
                end
            end
        end
    end
    
    // Wishbone state machine
    localparam WB_IDLE = 2'd0;
    localparam WB_WAIT_ACK = 2'd1;
    localparam WB_DONE = 2'd2;
    
    reg [1:0] wb_state;
    reg [7:0] last_cmd, last_data;
    reg [15:0] last_addr;  // Changed to 16-bit
    reg wb_started;  // Debug flag
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wb_state <= WB_IDLE;
            wb_adr_o <= 0;
            wb_dat_o <= 0;
            wb_we_o <= 0;
            wb_cyc_o <= 0;
            wb_stb_o <= 0;
            last_cmd <= 0;
            last_addr <= 0;
            last_data <= 0;
            wb_started <= 0;
        end else begin
            case (wb_state)
                WB_IDLE: begin
                    wb_cyc_o <= 0;
                    wb_stb_o <= 0;
                    wb_started <= 0;
                    
                    if (transaction_complete && cmd == 8'h01) begin
                        // Start Wishbone write transaction
                        wb_adr_o <= addr;  // Now 16-bit
                        wb_dat_o <= data;
                        wb_we_o <= 1;
                        wb_cyc_o <= 1;
                        wb_stb_o <= 1;
                        wb_state <= WB_WAIT_ACK;
                        last_cmd <= cmd;
                        last_addr <= addr;  // 16-bit
                        last_data <= data;
                        wb_started <= 1;
                    end
                end
                
                WB_WAIT_ACK: begin
                    if (wb_ack_i) begin
                        wb_cyc_o <= 0;
                        wb_stb_o <= 0;
                        wb_state <= WB_DONE;
                    end
                end
                
                WB_DONE: begin
                    wb_state <= WB_IDLE;
                end
            endcase
        end
    end
    
    // UART debug - send status every second
    reg [26:0] uart_timer;
    reg uart_send_trigger;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            uart_timer <= 0;
            uart_send_trigger <= 0;
        end else begin
            uart_send_trigger <= 0;
            uart_timer <= uart_timer + 1;
            if (uart_timer == 27'd27_000_000) begin
                uart_timer <= 0;
                uart_send_trigger <= 1;
            end
        end
    end
    
    // UART transmitter
    reg [7:0] uart_data;
    reg uart_send;
    wire uart_busy;
    
    uart_tx uart_inst (
        .clk(clk),
        .rst(rst),
        .data(uart_data),
        .send(uart_send),
        .tx(uart_tx),
        .busy(uart_busy)
    );
    
    // UART state machine - format: "C:xx A:xxxx D:xx S:x\n" (16-bit address now)
    reg [5:0] uart_state;  // Extended for more states
    localparam UART_IDLE = 0;
    localparam UART_C = 1;
    localparam UART_C_COLON = 2;
    localparam UART_C_HI = 3;
    localparam UART_C_LO = 4;
    localparam UART_SPACE1 = 5;
    localparam UART_A = 6;
    localparam UART_A_COLON = 7;
    localparam UART_A_HH = 8;   // Address high nibble of high byte
    localparam UART_A_HL = 9;   // Address low nibble of high byte
    localparam UART_A_LH = 10;  // Address high nibble of low byte
    localparam UART_A_LL = 11;  // Address low nibble of low byte
    localparam UART_SPACE2 = 12;
    localparam UART_D = 13;
    localparam UART_D_COLON = 14;
    localparam UART_D_HI = 15;
    localparam UART_D_LO = 16;
    localparam UART_SPACE3 = 17;
    localparam UART_S = 18;
    localparam UART_S_COLON = 19;
    localparam UART_S_VAL = 20;
    localparam UART_NL = 21;
    
    function [7:0] hex_to_ascii;
        input [3:0] hex;
        begin
            hex_to_ascii = (hex < 10) ? (8'd48 + hex) : (8'd55 + hex);
        end
    endfunction
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            uart_state <= UART_IDLE;
            uart_send <= 0;
            uart_data <= 0;
        end else begin
            uart_send <= 0;
            
            case (uart_state)
                UART_IDLE: if (uart_send_trigger) uart_state <= UART_C;
                
                UART_C: if (!uart_busy) begin
                    uart_data <= 8'd67; uart_send <= 1; uart_state <= UART_C_COLON;
                end
                
                UART_C_COLON: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd58; uart_send <= 1; uart_state <= UART_C_HI;
                end
                
                UART_C_HI: if (!uart_busy && !uart_send) begin
                    uart_data <= hex_to_ascii(last_cmd[7:4]); uart_send <= 1; uart_state <= UART_C_LO;
                end
                
                UART_C_LO: if (!uart_busy && !uart_send) begin
                    uart_data <= hex_to_ascii(last_cmd[3:0]); uart_send <= 1; uart_state <= UART_SPACE1;
                end
                
                UART_SPACE1: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd32; uart_send <= 1; uart_state <= UART_A;
                end
                
                UART_A: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd65; uart_send <= 1; uart_state <= UART_A_COLON;
                end
                
                UART_A_COLON: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd58; uart_send <= 1; uart_state <= UART_A_HH;
                end
                
                UART_A_HH: if (!uart_busy && !uart_send) begin
                    uart_data <= hex_to_ascii(last_addr[15:12]); uart_send <= 1; uart_state <= UART_A_HL;
                end
                
                UART_A_HL: if (!uart_busy && !uart_send) begin
                    uart_data <= hex_to_ascii(last_addr[11:8]); uart_send <= 1; uart_state <= UART_A_LH;
                end
                
                UART_A_LH: if (!uart_busy && !uart_send) begin
                    uart_data <= hex_to_ascii(last_addr[7:4]); uart_send <= 1; uart_state <= UART_A_LL;
                end
                
                UART_A_LL: if (!uart_busy && !uart_send) begin
                    uart_data <= hex_to_ascii(last_addr[3:0]); uart_send <= 1; uart_state <= UART_SPACE2;
                end
                
                UART_SPACE2: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd32; uart_send <= 1; uart_state <= UART_D;
                end
                
                UART_D: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd68; uart_send <= 1; uart_state <= UART_D_COLON;
                end
                
                UART_D_COLON: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd58; uart_send <= 1; uart_state <= UART_D_HI;
                end
                
                UART_D_HI: if (!uart_busy && !uart_send) begin
                    uart_data <= hex_to_ascii(last_data[7:4]); uart_send <= 1; uart_state <= UART_D_LO;
                end
                
                UART_D_LO: if (!uart_busy && !uart_send) begin
                    uart_data <= hex_to_ascii(last_data[3:0]); uart_send <= 1; uart_state <= UART_SPACE3;
                end
                
                UART_SPACE3: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd32; uart_send <= 1; uart_state <= UART_S;
                end
                
                UART_S: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd83; uart_send <= 1; uart_state <= UART_S_COLON;
                end
                
                UART_S_COLON: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd58; uart_send <= 1; uart_state <= UART_S_VAL;
                end
                
                UART_S_VAL: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd48 + wb_state; uart_send <= 1; uart_state <= UART_NL;
                end
                
                UART_NL: if (!uart_busy && !uart_send) begin
                    uart_data <= 8'd10; uart_send <= 1; uart_state <= UART_IDLE;
                end
                
                default: uart_state <= UART_IDLE;
            endcase
        end
    end

endmodule
