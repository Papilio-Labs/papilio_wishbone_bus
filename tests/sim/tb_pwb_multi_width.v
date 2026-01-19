`timescale 1ns / 1ps
// Papilio Wishbone Bridge Multi-Width and Burst Mode Testbench
// Tests pwb_spi_wb_bridge + pwb_register_block with 8/16/24/32-bit transfers
// Includes single-word and burst mode tests
// SPI Mode 1 (CPOL=0, CPHA=1)
//
// Command byte encoding:
//   Bit 0:    0=Read, 1=Write
//   Bits 2:1: 00=8-bit, 01=16-bit, 10=24-bit, 11=32-bit
//   Bit 3:    0=Single-word, 1=Burst mode
//
// Single-word commands: 8-bit=0x00/0x01, 16-bit=0x02/0x03, 24-bit=0x04/0x05, 32-bit=0x06/0x07
// Burst commands:       8-bit=0x08/0x09, 16-bit=0x0A/0x0B, 24-bit=0x0C/0x0D, 32-bit=0x0E/0x0F

module tb_pwb_multi_width;
    // Clocks and reset
    reg clk = 0;
    reg rst = 1;

    // SPI signals (Mode 1: CPOL=0, CPHA=1)
    reg spi_sclk = 0;
    reg spi_mosi = 0;
    wire spi_miso;
    reg spi_cs_n = 1;

    // Wishbone bus from bridge
    wire [15:0] wb_adr;
    wire [31:0] wb_dat_m2s;    // Master to slave
    wire [31:0] wb_dat_s2m;    // Slave to master
    wire [3:0]  wb_sel;
    wire        wb_we;
    wire        wb_cyc;
    wire        wb_stb;
    wire        wb_ack;

    // Individual slave outputs (directly exposed, no mux for simplicity)
    wire [7:0]  wb_dat_8;
    wire [15:0] wb_dat_16;
    wire [23:0] wb_dat_24;
    wire [31:0] wb_dat_32;
    wire        wb_ack_8, wb_ack_16, wb_ack_24, wb_ack_32;

    // Address decoding: 
    //   0x0000-0x00FF: 8-bit registers
    //   0x0100-0x017F: 16-bit registers
    //   0x0180-0x01FF: 24-bit registers  
    //   0x0200-0x02FF: 32-bit registers
    wire sel_8  = (wb_adr[15:8] == 8'h00);
    wire sel_16 = (wb_adr[15:8] == 8'h01) && !wb_adr[7];
    wire sel_24 = (wb_adr[15:8] == 8'h01) && wb_adr[7];
    wire sel_32 = (wb_adr[15:8] == 8'h02);

    // Mux ACK and data back to bridge
    assign wb_ack = (sel_8 & wb_ack_8) | (sel_16 & wb_ack_16) | (sel_24 & wb_ack_24) | (sel_32 & wb_ack_32);
    assign wb_dat_s2m = sel_8  ? {24'b0, wb_dat_8} :
                        sel_16 ? {16'b0, wb_dat_16} :
                        sel_24 ? {8'b0, wb_dat_24} :
                        sel_32 ? wb_dat_32 :
                        32'hDEADBEEF;

    // FIFO status signals (for monitoring)
    wire fifo_rx_almost_full;
    wire fifo_tx_almost_empty;

    // Forward declarations for RAM (defined after register blocks)
    wire wb_ack_with_ram;
    wire [31:0] wb_dat_s2m_with_ram;

    // Instantiate DUT: Multi-width SPI bridge with FIFO support
    pwb_spi_wb_bridge #(
        .FIFO_DEPTH(64),
        .ALMOST_FULL_THRESHOLD(4),
        .ALMOST_EMPTY_THRESHOLD(4)
    ) dut_bridge (
        .clk(clk),
        .rst(rst),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .wb_adr_o(wb_adr),
        .wb_dat_o(wb_dat_m2s),
        .wb_dat_i(wb_dat_s2m_with_ram),
        .wb_sel_o(wb_sel),
        .wb_we_o(wb_we),
        .wb_cyc_o(wb_cyc),
        .wb_stb_o(wb_stb),
        .wb_ack_i(wb_ack_with_ram),
        .fifo_rx_almost_full(fifo_rx_almost_full),
        .fifo_tx_almost_empty(fifo_tx_almost_empty)
    );

    // 8-bit register block at 0x0000
    wb_register_block #(
        .ADDR_WIDTH(4),
        .DATA_WIDTH(8),
        .RESET_VALUE(8'h00)
    ) regs_8 (
        .clk(clk),
        .rst(rst),
        .wb_adr_i(wb_adr[3:0]),
        .wb_dat_i(wb_dat_m2s[7:0]),
        .wb_dat_o(wb_dat_8),
        .wb_we_i(wb_we),
        .wb_cyc_i(wb_cyc & sel_8),
        .wb_stb_i(wb_stb & sel_8),
        .wb_ack_o(wb_ack_8)
    );

    // 16-bit register block at 0x0100
    wb_register_block #(
        .ADDR_WIDTH(4),
        .DATA_WIDTH(16),
        .RESET_VALUE(16'h0000)
    ) regs_16 (
        .clk(clk),
        .rst(rst),
        .wb_adr_i(wb_adr[3:0]),
        .wb_dat_i(wb_dat_m2s[15:0]),
        .wb_dat_o(wb_dat_16),
        .wb_we_i(wb_we),
        .wb_cyc_i(wb_cyc & sel_16),
        .wb_stb_i(wb_stb & sel_16),
        .wb_ack_o(wb_ack_16)
    );

    // 24-bit register block at 0x0180
    wb_register_block #(
        .ADDR_WIDTH(4),
        .DATA_WIDTH(24),
        .RESET_VALUE(24'h000000)
    ) regs_24 (
        .clk(clk),
        .rst(rst),
        .wb_adr_i(wb_adr[3:0]),
        .wb_dat_i(wb_dat_m2s[23:0]),
        .wb_dat_o(wb_dat_24),
        .wb_we_i(wb_we),
        .wb_cyc_i(wb_cyc & sel_24),
        .wb_stb_i(wb_stb & sel_24),
        .wb_ack_o(wb_ack_24)
    );

    // 32-bit register block at 0x0200
    wb_register_block #(
        .ADDR_WIDTH(4),
        .DATA_WIDTH(32),
        .RESET_VALUE(32'h00000000)
    ) regs_32 (
        .clk(clk),
        .rst(rst),
        .wb_adr_i(wb_adr[3:0]),
        .wb_dat_i(wb_dat_m2s[31:0]),
        .wb_dat_o(wb_dat_32),
        .wb_we_i(wb_we),
        .wb_cyc_i(wb_cyc & sel_32),
        .wb_stb_i(wb_stb & sel_32),
        .wb_ack_o(wb_ack_32)
    );

    // =========================================================================
    // Large RAM block at 0x1000 for testing burst counts > 255
    // 512 x 8-bit words = 512 bytes
    // =========================================================================
    wire sel_ram = (wb_adr[15:9] == 7'b0001000);  // 0x1000-0x11FF
    reg [7:0] ram_mem [0:511];
    reg [7:0] ram_dat_o;
    reg ram_ack;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ram_ack <= 0;
            ram_dat_o <= 8'h00;
        end else begin
            ram_ack <= 0;
            if (wb_cyc && wb_stb && sel_ram && !ram_ack) begin
                if (wb_we) begin
                    ram_mem[wb_adr[8:0]] <= wb_dat_m2s[7:0];
                end else begin
                    ram_dat_o <= ram_mem[wb_adr[8:0]];
                end
                ram_ack <= 1;
            end
        end
    end

    // Update mux to include RAM
    assign wb_ack_with_ram = wb_ack | (sel_ram & ram_ack);
    assign wb_dat_s2m_with_ram = sel_ram ? {24'b0, ram_dat_o} : wb_dat_s2m;

    // Generate 27 MHz system clock (~37 ns period)
    always #18.5 clk = ~clk;

    // =========================================================================
    // SPI helper tasks
    // =========================================================================
    
    // Send a single 8-bit byte over SPI (Mode 1)
    task spi_byte;
        input  [7:0] tx;
        output [7:0] rx;
        integer i;
        begin
            rx = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                spi_mosi = tx[i];
                #500 spi_sclk = 1;
                #500 spi_sclk = 0;
                rx[i] = spi_miso;
                #100;
            end
        end
    endtask

    // Send 16 bits over SPI (for 16-bit data phase)
    task spi_word16;
        input  [15:0] tx;
        output [15:0] rx;
        integer i;
        begin
            rx = 16'h0000;
            for (i = 15; i >= 0; i = i - 1) begin
                spi_mosi = tx[i];
                #500 spi_sclk = 1;
                #500 spi_sclk = 0;
                rx[i] = spi_miso;
                #100;
            end
        end
    endtask

    // Send 24 bits over SPI (for 24-bit data phase)
    task spi_word24;
        input  [23:0] tx;
        output [23:0] rx;
        integer i;
        begin
            rx = 24'h000000;
            for (i = 23; i >= 0; i = i - 1) begin
                spi_mosi = tx[i];
                #500 spi_sclk = 1;
                #500 spi_sclk = 0;
                rx[i] = spi_miso;
                #100;
            end
        end
    endtask

    // Send 32 bits over SPI (for 32-bit data phase)
    task spi_word32;
        input  [31:0] tx;
        output [31:0] rx;
        integer i;
        begin
            rx = 32'h00000000;
            for (i = 31; i >= 0; i = i - 1) begin
                spi_mosi = tx[i];
                #500 spi_sclk = 1;
                #500 spi_sclk = 0;
                rx[i] = spi_miso;
                #100;
            end
        end
    endtask

    // =========================================================================
    // 8-bit write/read (CMD=0x01/0x00, backward compatible)
    // =========================================================================
    task spi_write8;
        input [15:0] addr;
        input [7:0] data;
        reg [7:0] dummy;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h01, dummy);           // CMD: 8-bit write
            spi_byte(addr[15:8], dummy);
            spi_byte(addr[7:0], dummy);
            spi_byte(data, dummy);
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    task spi_read8;
        input  [15:0] addr;
        output [7:0] data;
        reg [7:0] rx;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h00, rx);              // CMD: 8-bit read
            spi_byte(addr[15:8], rx);
            spi_byte(addr[7:0], rx);
            spi_byte(8'hFF, data);            // Clock out data
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    // =========================================================================
    // 16-bit write/read (CMD=0x03/0x02)
    // Bits [2:1] = 01 for 16-bit, bit 0 = r/w
    // =========================================================================
    task spi_write16;
        input [15:0] addr;
        input [15:0] data;
        reg [7:0] dummy8;
        reg [15:0] dummy16;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h03, dummy8);          // CMD: 16-bit write (0b0000_0011)
            spi_byte(addr[15:8], dummy8);
            spi_byte(addr[7:0], dummy8);
            spi_word16(data, dummy16);        // 16-bit data transfer
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    task spi_read16;
        input  [15:0] addr;
        output [15:0] data;
        reg [7:0] rx8;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h02, rx8);             // CMD: 16-bit read (0b0000_0010)
            spi_byte(addr[15:8], rx8);
            spi_byte(addr[7:0], rx8);
            spi_word16(16'hFFFF, data);       // Clock out 16-bit data
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    // =========================================================================
    // 24-bit write/read (CMD=0x05/0x04)
    // Bits [2:1] = 10 for 24-bit, bit 0 = r/w
    // =========================================================================
    task spi_write24;
        input [15:0] addr;
        input [23:0] data;
        reg [7:0] dummy8;
        reg [23:0] dummy24;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h05, dummy8);          // CMD: 24-bit write (0b0000_0101)
            spi_byte(addr[15:8], dummy8);
            spi_byte(addr[7:0], dummy8);
            spi_word24(data, dummy24);        // 24-bit data transfer
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    task spi_read24;
        input  [15:0] addr;
        output [23:0] data;
        reg [7:0] rx8;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h04, rx8);             // CMD: 24-bit read (0b0000_0100)
            spi_byte(addr[15:8], rx8);
            spi_byte(addr[7:0], rx8);
            spi_word24(24'hFFFFFF, data);     // Clock out 24-bit data
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    // =========================================================================
    // 32-bit write/read (CMD=0x07/0x06)
    // Bits [2:1] = 11 for 32-bit, bit 0 = r/w
    // =========================================================================
    task spi_write32;
        input [15:0] addr;
        input [31:0] data;
        reg [7:0] dummy8;
        reg [31:0] dummy32;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h07, dummy8);          // CMD: 32-bit write (0b0000_0111)
            spi_byte(addr[15:8], dummy8);
            spi_byte(addr[7:0], dummy8);
            spi_word32(data, dummy32);        // 32-bit data transfer
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    task spi_read32;
        input  [15:0] addr;
        output [31:0] data;
        reg [7:0] rx8;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h06, rx8);             // CMD: 32-bit read (0b0000_0110)
            spi_byte(addr[15:8], rx8);
            spi_byte(addr[7:0], rx8);
            spi_word32(32'hFFFFFFFF, data);   // Clock out 32-bit data
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    // =========================================================================
    // Test infrastructure
    // =========================================================================
    integer pass_8, fail_8;
    integer pass_16, fail_16;
    integer pass_24, fail_24;
    integer pass_32, fail_32;

    task check_write_read8;
        input [15:0] addr;
        input [7:0] value;
        reg [7:0] rd;
        begin
            spi_write8(addr, value);
            spi_read8(addr, rd);
            if (rd === value) begin
                pass_8 = pass_8 + 1;
                $display("[PASS] 8-bit  Addr 0x%04h wrote 0x%02h read 0x%02h", addr, value, rd);
            end else begin
                fail_8 = fail_8 + 1;
                $display("[FAIL] 8-bit  Addr 0x%04h wrote 0x%02h read 0x%02h", addr, value, rd);
            end
        end
    endtask

    task check_write_read16;
        input [15:0] addr;
        input [15:0] value;
        reg [15:0] rd;
        begin
            spi_write16(addr, value);
            spi_read16(addr, rd);
            if (rd === value) begin
                pass_16 = pass_16 + 1;
                $display("[PASS] 16-bit Addr 0x%04h wrote 0x%04h read 0x%04h", addr, value, rd);
            end else begin
                fail_16 = fail_16 + 1;
                $display("[FAIL] 16-bit Addr 0x%04h wrote 0x%04h read 0x%04h", addr, value, rd);
            end
        end
    endtask

    task check_write_read24;
        input [15:0] addr;
        input [23:0] value;
        reg [23:0] rd;
        begin
            spi_write24(addr, value);
            spi_read24(addr, rd);
            if (rd === value) begin
                pass_24 = pass_24 + 1;
                $display("[PASS] 24-bit Addr 0x%04h wrote 0x%06h read 0x%06h", addr, value, rd);
            end else begin
                fail_24 = fail_24 + 1;
                $display("[FAIL] 24-bit Addr 0x%04h wrote 0x%06h read 0x%06h", addr, value, rd);
            end
        end
    endtask

    task check_write_read32;
        input [15:0] addr;
        input [31:0] value;
        reg [31:0] rd;
        begin
            spi_write32(addr, value);
            spi_read32(addr, rd);
            if (rd === value) begin
                pass_32 = pass_32 + 1;
                $display("[PASS] 32-bit Addr 0x%04h wrote 0x%08h read 0x%08h", addr, value, rd);
            end else begin
                fail_32 = fail_32 + 1;
                $display("[FAIL] 32-bit Addr 0x%04h wrote 0x%08h read 0x%08h", addr, value, rd);
            end
        end
    endtask

    // =========================================================================
    // Burst mode tasks - 8-bit burst write/read
    // COUNT is now 16-bit (big-endian): [COUNT_H][COUNT_L]
    // =========================================================================
    task spi_burst_write8;
        input [15:0] start_addr;
        input [15:0] count;
        input [7:0] data0, data1, data2, data3;  // Up to 4 words for simplicity
        reg [7:0] dummy;
        integer i;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h09, dummy);           // CMD: 8-bit burst write (0x09)
            spi_byte(start_addr[15:8], dummy);
            spi_byte(start_addr[7:0], dummy);
            spi_byte(count[15:8], dummy);     // COUNT high byte (MSB)
            spi_byte(count[7:0], dummy);      // COUNT low byte (LSB)
            // Send data words
            if (count >= 1) spi_byte(data0, dummy);
            if (count >= 2) spi_byte(data1, dummy);
            if (count >= 3) spi_byte(data2, dummy);
            if (count >= 4) spi_byte(data3, dummy);
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    // =========================================================================
    // Burst mode tasks - 32-bit burst write/read
    // COUNT is now 16-bit (big-endian): [COUNT_H][COUNT_L]
    // =========================================================================
    task spi_burst_write32;
        input [15:0] start_addr;
        input [15:0] count;
        input [31:0] data0, data1, data2, data3;
        reg [7:0] dummy8;
        reg [31:0] dummy32;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h0F, dummy8);          // CMD: 32-bit burst write (0x0F)
            spi_byte(start_addr[15:8], dummy8);
            spi_byte(start_addr[7:0], dummy8);
            spi_byte(count[15:8], dummy8);    // COUNT high byte (MSB)
            spi_byte(count[7:0], dummy8);     // COUNT low byte (LSB)
            // Send data words
            if (count >= 1) spi_word32(data0, dummy32);
            if (count >= 2) spi_word32(data1, dummy32);
            if (count >= 3) spi_word32(data2, dummy32);
            if (count >= 4) spi_word32(data3, dummy32);
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    task spi_burst_read32;
        input  [15:0] start_addr;
        input  [15:0] count;
        output [31:0] data0, data1, data2, data3;
        reg [7:0] rx8;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h0E, rx8);             // CMD: 32-bit burst read (0x0E)
            spi_byte(start_addr[15:8], rx8);
            spi_byte(start_addr[7:0], rx8);
            spi_byte(count[15:8], rx8);       // COUNT high byte (MSB)
            spi_byte(count[7:0], rx8);        // COUNT low byte (LSB)
            // Read data words
            data0 = 32'h0; data1 = 32'h0; data2 = 32'h0; data3 = 32'h0;
            if (count >= 1) spi_word32(32'hFFFFFFFF, data0);
            if (count >= 2) spi_word32(32'hFFFFFFFF, data1);
            if (count >= 3) spi_word32(32'hFFFFFFFF, data2);
            if (count >= 4) spi_word32(32'hFFFFFFFF, data3);
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    // Burst test counters
    integer pass_burst, fail_burst;

    task check_burst_write_read32;
        input [15:0] start_addr;
        input [31:0] val0, val1, val2, val3;
        reg [31:0] rd0, rd1, rd2, rd3;
        integer ok;
        begin
            // Write 4 words via burst (16-bit count)
            spi_burst_write32(start_addr, 16'd4, val0, val1, val2, val3);
            // Read back via single-word to verify
            spi_read32(start_addr, rd0);
            spi_read32(start_addr + 1, rd1);
            spi_read32(start_addr + 2, rd2);
            spi_read32(start_addr + 3, rd3);

            ok = (rd0 === val0) && (rd1 === val1) && (rd2 === val2) && (rd3 === val3);
            if (ok) begin
                pass_burst = pass_burst + 1;
                $display("[PASS] Burst32 Write Addr 0x%04h: 0x%08h 0x%08h 0x%08h 0x%08h",
                         start_addr, val0, val1, val2, val3);
            end else begin
                fail_burst = fail_burst + 1;
                $display("[FAIL] Burst32 Write Addr 0x%04h", start_addr);
                $display("       Expected: 0x%08h 0x%08h 0x%08h 0x%08h", val0, val1, val2, val3);
                $display("       Got:      0x%08h 0x%08h 0x%08h 0x%08h", rd0, rd1, rd2, rd3);
            end
        end
    endtask

    task check_burst_read32;
        input [15:0] start_addr;
        reg [31:0] wr0, wr1, wr2, wr3;
        reg [31:0] rd0, rd1, rd2, rd3;
        integer ok;
        begin
            // Write test values via single-word
            wr0 = 32'hDEADBEEF;
            wr1 = 32'hCAFEBABE;
            wr2 = 32'h12345678;
            wr3 = 32'h9ABCDEF0;
            spi_write32(start_addr, wr0);
            spi_write32(start_addr + 1, wr1);
            spi_write32(start_addr + 2, wr2);
            spi_write32(start_addr + 3, wr3);

            // Read back via burst (16-bit count)
            spi_burst_read32(start_addr, 16'd4, rd0, rd1, rd2, rd3);

            ok = (rd0 === wr0) && (rd1 === wr1) && (rd2 === wr2) && (rd3 === wr3);
            if (ok) begin
                pass_burst = pass_burst + 1;
                $display("[PASS] Burst32 Read Addr 0x%04h: 0x%08h 0x%08h 0x%08h 0x%08h",
                         start_addr, rd0, rd1, rd2, rd3);
            end else begin
                fail_burst = fail_burst + 1;
                $display("[FAIL] Burst32 Read Addr 0x%04h", start_addr);
                $display("       Expected: 0x%08h 0x%08h 0x%08h 0x%08h", wr0, wr1, wr2, wr3);
                $display("       Got:      0x%08h 0x%08h 0x%08h 0x%08h", rd0, rd1, rd2, rd3);
            end
        end
    endtask

    // =========================================================================
    // Large burst test task (8-bit, count > 255 to test 16-bit COUNT)
    // Uses RAM at 0x1000
    // =========================================================================
    task spi_burst_write8_large;
        input [15:0] start_addr;
        input [15:0] count;
        reg [7:0] dummy;
        integer i;
        begin
            spi_cs_n = 0;
            #500;
            spi_byte(8'h09, dummy);           // CMD: 8-bit burst write (0x09)
            spi_byte(start_addr[15:8], dummy);
            spi_byte(start_addr[7:0], dummy);
            spi_byte(count[15:8], dummy);     // COUNT high byte (MSB)
            spi_byte(count[7:0], dummy);      // COUNT low byte (LSB)
            // Send data words - pattern: i & 0xFF
            for (i = 0; i < count; i = i + 1) begin
                spi_byte(i[7:0], dummy);
            end
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    task spi_burst_read8_large;
        input  [15:0] start_addr;
        input  [15:0] count;
        output integer errors;
        reg [7:0] rx8;
        reg [7:0] expected;
        integer i;
        begin
            errors = 0;
            spi_cs_n = 0;
            #500;
            spi_byte(8'h08, rx8);             // CMD: 8-bit burst read (0x08)
            spi_byte(start_addr[15:8], rx8);
            spi_byte(start_addr[7:0], rx8);
            spi_byte(count[15:8], rx8);       // COUNT high byte (MSB)
            spi_byte(count[7:0], rx8);        // COUNT low byte (LSB)
            // Read data words and verify pattern
            for (i = 0; i < count; i = i + 1) begin
                spi_byte(8'hFF, rx8);
                expected = i[7:0];
                if (rx8 !== expected) begin
                    errors = errors + 1;
                    if (errors <= 5) begin  // Only print first 5 errors
                        $display("       Mismatch at offset %0d: expected 0x%02h, got 0x%02h", i, expected, rx8);
                    end
                end
            end
            #2000;
            spi_cs_n = 1;
            #1000;
        end
    endtask

    task check_large_burst8;
        input [15:0] count;
        integer write_errors, read_errors;
        begin
            $display("  Testing %0d-word burst (0x%04h words)...", count, count);
            // Write pattern to RAM via burst
            spi_burst_write8_large(16'h1000, count);
            // Read back and verify
            spi_burst_read8_large(16'h1000, count, read_errors);

            if (read_errors == 0) begin
                pass_burst = pass_burst + 1;
                $display("[PASS] Large Burst8: %0d words written and verified correctly", count);
            end else begin
                fail_burst = fail_burst + 1;
                $display("[FAIL] Large Burst8: %0d words, %0d mismatches", count, read_errors);
            end
        end
    endtask

    integer idx;

    initial begin
        $dumpfile("tb_pwb_multi_width.vcd");
        $dumpvars(0, tb_pwb_multi_width);

        pass_8 = 0; fail_8 = 0;
        pass_16 = 0; fail_16 = 0;
        pass_24 = 0; fail_24 = 0;
        pass_32 = 0; fail_32 = 0;
        pass_burst = 0; fail_burst = 0;

        // Reset pulse
        #200;
        rst = 0;
        #1000;

        // =====================================================================
        // 8-bit tests (address 0x0000-0x000F)
        // =====================================================================
        $display("");
        $display("=== 8-bit Transfer Tests ===");
        check_write_read8(16'h0000, 8'h12);
        check_write_read8(16'h0001, 8'h34);
        check_write_read8(16'h0002, 8'hAA);
        check_write_read8(16'h0003, 8'h55);
        check_write_read8(16'h0004, 8'h00);
        check_write_read8(16'h0005, 8'hFF);
        // Walking ones
        for (idx = 0; idx < 8; idx = idx + 1) begin
            check_write_read8(16'h0008 + idx[15:0], (8'h01 << idx));
        end

        // =====================================================================
        // 16-bit tests (address 0x0100-0x010F)
        // =====================================================================
        $display("");
        $display("=== 16-bit Transfer Tests ===");
        check_write_read16(16'h0100, 16'h1234);
        check_write_read16(16'h0101, 16'h5678);
        check_write_read16(16'h0102, 16'hAAAA);
        check_write_read16(16'h0103, 16'h5555);
        check_write_read16(16'h0104, 16'h0000);
        check_write_read16(16'h0105, 16'hFFFF);
        // Walking ones
        for (idx = 0; idx < 8; idx = idx + 1) begin
            check_write_read16(16'h0108 + idx[15:0], (16'h0001 << idx));
        end

        // =====================================================================
        // 24-bit tests (address 0x0180-0x018F)
        // =====================================================================
        $display("");
        $display("=== 24-bit Transfer Tests ===");
        check_write_read24(16'h0180, 24'h123456);
        check_write_read24(16'h0181, 24'h789ABC);
        check_write_read24(16'h0182, 24'hAAAAAA);
        check_write_read24(16'h0183, 24'h555555);
        check_write_read24(16'h0184, 24'h000000);
        check_write_read24(16'h0185, 24'hFFFFFF);
        // Walking ones
        for (idx = 0; idx < 8; idx = idx + 1) begin
            check_write_read24(16'h0188 + idx[15:0], (24'h000001 << idx));
        end

        // =====================================================================
        // 32-bit tests (address 0x0200-0x020F)
        // =====================================================================
        $display("");
        $display("=== 32-bit Transfer Tests ===");
        check_write_read32(16'h0200, 32'h12345678);
        check_write_read32(16'h0201, 32'h9ABCDEF0);
        check_write_read32(16'h0202, 32'hAAAAAAAA);
        check_write_read32(16'h0203, 32'h55555555);
        check_write_read32(16'h0204, 32'h00000000);
        check_write_read32(16'h0205, 32'hFFFFFFFF);
        // Walking ones
        for (idx = 0; idx < 8; idx = idx + 1) begin
            check_write_read32(16'h0208 + idx[15:0], (32'h00000001 << idx));
        end

        // =====================================================================
        // Burst mode tests (32-bit, address 0x0200-0x020F)
        // =====================================================================
        $display("");
        $display("=== Burst Mode Tests (32-bit) ===");
        
        // Test burst write: write 4 words, read back with single-word
        check_burst_write_read32(16'h0200, 32'hAABBCCDD, 32'h11223344, 32'h55667788, 32'h99AABBCC);
        
        // Test burst read: write with single-word, read back with burst
        check_burst_read32(16'h0204);

        // =====================================================================
        // Large Burst Tests (8-bit, count > 255 to validate 16-bit COUNT)
        // Uses RAM at 0x1000
        // =====================================================================
        $display("");
        $display("=== Large Burst Tests (16-bit COUNT validation) ===");

        // Test with count = 300 (0x012C) - greater than 255 to validate 16-bit COUNT
        check_large_burst8(16'd300);

        // Test with count = 512 (0x0200) - uses full RAM
        check_large_burst8(16'd512);

        // =====================================================================
        // Summary
        // =====================================================================
        $display("");
        $display("=== Suite Summary ===");
        $display("8-bit:  PASS=%0d FAIL=%0d", pass_8, fail_8);
        $display("16-bit: PASS=%0d FAIL=%0d", pass_16, fail_16);
        $display("24-bit: PASS=%0d FAIL=%0d", pass_24, fail_24);
        $display("32-bit: PASS=%0d FAIL=%0d", pass_32, fail_32);
        $display("Burst:  PASS=%0d FAIL=%0d", pass_burst, fail_burst);
        $display("TOTAL:  PASS=%0d FAIL=%0d", pass_8+pass_16+pass_24+pass_32+pass_burst, 
                                              fail_8+fail_16+fail_24+fail_32+fail_burst);
        
        #5000;
        $finish;
    end

    // Timeout protection
    initial begin
        #50000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
