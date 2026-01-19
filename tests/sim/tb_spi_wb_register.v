`timescale 1ns / 1ps
// Wishbone bridge simulation testbench
// Exercises simple_spi_wb_bridge + wb_register_block with SPI Mode 1 (CPOL=0, CPHA=1)
// Generates VCD for offline analysis and can be extended with further patterns.

module tb_spi_wb_register;
    // Clocks and reset
    reg clk = 0;
    reg rst = 1;

    // SPI signals (Mode 1: CPOL=0, CPHA=1)
    reg spi_sclk = 0;
    reg spi_mosi = 0;
    wire spi_miso;
    reg spi_cs_n = 1;

    // Wishbone wires
    wire [15:0] wb_adr;
    wire [7:0]  wb_dat_o;
    wire [7:0]  wb_dat_i;
    wire        wb_we;
    wire        wb_cyc;
    wire        wb_stb;
    wire        wb_ack;

    // Instantiate DUT: SPI bridge + register block
    simple_spi_wb_bridge dut_bridge (
        .clk(clk),
        .rst(rst),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .wb_adr_o(wb_adr),
        .wb_dat_o(wb_dat_o),
        .wb_dat_i(wb_dat_i),
        .wb_we_o(wb_we),
        .wb_cyc_o(wb_cyc),
        .wb_stb_o(wb_stb),
        .wb_ack_i(wb_ack)
    );

    wb_register_block #(
        .ADDR_WIDTH(5),  // 32 registers (matches walking patterns)
        .DATA_WIDTH(8),
        .RESET_VALUE(8'h00)
    ) dut_regs (
        .clk(clk),
        .rst(rst),
        .wb_adr_i(wb_adr[4:0]),
        .wb_dat_i(wb_dat_o),
        .wb_dat_o(wb_dat_i),
        .wb_we_i(wb_we),
        .wb_cyc_i(wb_cyc),
        .wb_stb_i(wb_stb),
        .wb_ack_o(wb_ack)
    );

    // Generate 27 MHz system clock (~37 ns period)
    always #18.5 clk = ~clk;

    // SPI helpers (1 MHz SCLK; Mode 1 timing: sample on falling edge)
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
                #100; // hold MOSI stable after sample to avoid edge-time jitter
            end
        end
    endtask

    task spi_write;
        input [15:0] addr;
        input [7:0] data;
        reg [7:0] dummy;
        begin
            spi_cs_n = 0;
            #500; // allow CS synchronizer to settle before first edge
            spi_byte(8'h01, dummy);           // CMD_WRITE
            spi_byte(addr[15:8], dummy);
            spi_byte(addr[7:0], dummy);
            spi_byte(data, dummy);
            #2000; // hold CS low after final edge so last bit propagates
            spi_cs_n = 1;
            #1000; // inter-transaction gap
        end
    endtask

    task spi_read;
        input  [15:0] addr;
        output [7:0] data;
        reg [7:0] rx;
        begin
            spi_cs_n = 0;
            #500; // allow CS synchronizer to settle before first edge
            spi_byte(8'h00, rx);              // CMD_READ
            spi_byte(addr[15:8], rx);
            spi_byte(addr[7:0], rx);
            spi_byte(8'hFF, data);            // dummy to clock data out
            #2000; // hold CS low after final edge so last bit propagates
            spi_cs_n = 1;
            #1000;
        end
    endtask

    integer pass_cnt;
    integer fail_cnt;
    integer idx;

    task check_write_read;
        input [15:0] addr;
        input [7:0] value;
        reg [7:0] rd;
        begin
            spi_write(addr, value);
            spi_read(addr, rd);
            if (rd === value) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] Addr 0x%04h wrote 0x%02h read 0x%02h", addr, value, rd);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] Addr 0x%04h wrote 0x%02h read 0x%02h", addr, value, rd);
            end
        end
    endtask

    initial begin
        $dumpfile("tb_spi_wb_register.vcd");
        $dumpvars(0, tb_spi_wb_register);

        pass_cnt = 0;
        fail_cnt = 0;
        idx = 0;

        // Reset pulse
        #200;
        rst = 0;

        // Smoke tests
        check_write_read(16'h0000, 8'h12);
        check_write_read(16'h0001, 8'h34);
        check_write_read(16'h0002, 8'h56);
        check_write_read(16'h0003, 8'h78);
        check_write_read(16'h0004, 8'hAA);
        check_write_read(16'h0005, 8'h55);
        check_write_read(16'h0006, 8'h00);
        check_write_read(16'h0007, 8'hFF);

        // Walking ones / zeros
        for (idx = 0; idx < 8; idx = idx + 1) begin
            check_write_read(16'h0010 + idx[15:0], (8'h01 << idx));
        end
        for (idx = 0; idx < 8; idx = idx + 1) begin
            check_write_read(16'h0020 + idx[15:0], ~(8'h01 << idx));
        end

        $display("-- Suite Summary --");
        $display("PASS=%0d FAIL=%0d", pass_cnt, fail_cnt);
        #5000;
        $finish;
    end

    // Timeout protection
    initial begin
        #5000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
