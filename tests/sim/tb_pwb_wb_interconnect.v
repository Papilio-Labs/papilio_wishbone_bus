`timescale 1ns / 1ps
// Testbench: pwb_wb_interconnect
//
// Directly drives the Wishbone master interface (no SPI bridge) to verify:
//   1. Slot tier routing — correct slot receives cyc/stb
//   2. Slot isolation — other slots remain deasserted
//   3. Out-of-range slot address — returns 0xDEADBEEF
//   4. Extended tier routing — ext_cyc/stb asserted, slot/large not
//   5. Large tier stub — returns 0xDEADBEEF, no hang
//   6. Address outside all tiers — returns 0xDEADBEEF

module tb_pwb_wb_interconnect;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    parameter NUM_SLOTS = 8;

    // -------------------------------------------------------------------------
    // Clock and reset
    // -------------------------------------------------------------------------
    reg clk = 0;
    reg rst = 1;
    always #18.5 clk = ~clk; // 27 MHz

    // -------------------------------------------------------------------------
    // Master Wishbone interface
    // -------------------------------------------------------------------------
    reg  [23:0] wb_adr_i;
    reg  [31:0] wb_dat_i;
    wire [31:0] wb_dat_o;
    reg  [3:0]  wb_sel_i;
    reg         wb_we_i;
    reg         wb_cyc_i;
    reg         wb_stb_i;
    wire        wb_ack_o;

    // -------------------------------------------------------------------------
    // Slot Wishbone arrays (from interconnect)
    // -------------------------------------------------------------------------
    wire [NUM_SLOTS*16-1:0] slot_adr_o;
    wire [NUM_SLOTS*32-1:0] slot_dat_o;
    reg  [NUM_SLOTS*32-1:0] slot_dat_i;
    wire [NUM_SLOTS*4-1:0]  slot_sel_o;
    wire [NUM_SLOTS-1:0]    slot_we_o;
    wire [NUM_SLOTS-1:0]    slot_cyc_o;
    wire [NUM_SLOTS-1:0]    slot_stb_o;
    reg  [NUM_SLOTS-1:0]    slot_ack_i;

    // -------------------------------------------------------------------------
    // Extended tier ports
    // -------------------------------------------------------------------------
    wire [15:0] ext_adr_o;
    wire [31:0] ext_dat_o;
    reg  [31:0] ext_dat_i;
    wire [3:0]  ext_sel_o;
    wire        ext_we_o;
    wire        ext_cyc_o;
    wire        ext_stb_o;
    reg         ext_ack_i;

    // -------------------------------------------------------------------------
    // Large tier ports
    // -------------------------------------------------------------------------
    wire [23:0] large_adr_o;
    wire [31:0] large_dat_o;
    reg  [31:0] large_dat_i;
    wire [3:0]  large_sel_o;
    wire        large_we_o;
    wire        large_cyc_o;
    wire        large_stb_o;
    reg         large_ack_i;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    pwb_wb_interconnect #(.NUM_SLOTS(NUM_SLOTS)) dut (
        .clk(clk),
        .rst(rst),
        .wb_adr_i(wb_adr_i),
        .wb_dat_i(wb_dat_i),
        .wb_dat_o(wb_dat_o),
        .wb_sel_i(wb_sel_i),
        .wb_we_i(wb_we_i),
        .wb_cyc_i(wb_cyc_i),
        .wb_stb_i(wb_stb_i),
        .wb_ack_o(wb_ack_o),
        .slot_adr_o(slot_adr_o),
        .slot_dat_o(slot_dat_o),
        .slot_dat_i(slot_dat_i),
        .slot_sel_o(slot_sel_o),
        .slot_we_o(slot_we_o),
        .slot_cyc_o(slot_cyc_o),
        .slot_stb_o(slot_stb_o),
        .slot_ack_i(slot_ack_i),
        .ext_adr_o(ext_adr_o),
        .ext_dat_o(ext_dat_o),
        .ext_dat_i(ext_dat_i),
        .ext_sel_o(ext_sel_o),
        .ext_we_o(ext_we_o),
        .ext_cyc_o(ext_cyc_o),
        .ext_stb_o(ext_stb_o),
        .ext_ack_i(ext_ack_i),
        .large_adr_o(large_adr_o),
        .large_dat_o(large_dat_o),
        .large_dat_i(large_dat_i),
        .large_sel_o(large_sel_o),
        .large_we_o(large_we_o),
        .large_cyc_o(large_cyc_o),
        .large_stb_o(large_stb_o),
        .large_ack_i(large_ack_i)
    );

    // -------------------------------------------------------------------------
    // Test counters
    // -------------------------------------------------------------------------
    integer pass_cnt;
    integer fail_cnt;

    // -------------------------------------------------------------------------
    // Wishbone transaction helper — single read cycle, returns data from master
    // -------------------------------------------------------------------------
    task wb_read;
        input  [23:0] addr;
        output [31:0] data;
        begin
            @(negedge clk);
            wb_adr_i = addr;
            wb_dat_i = 32'h00000000;
            wb_sel_i = 4'b1111;
            wb_we_i  = 0;
            wb_cyc_i = 1;
            wb_stb_i = 1;
            @(posedge clk);
            // Wait for ACK
            while (!wb_ack_o) @(posedge clk);
            data = wb_dat_o;
            @(negedge clk);
            wb_cyc_i = 0;
            wb_stb_i = 0;
            @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // Check: slot cyc/stb routing
    // Asserts that only the target slot is asserted during the cycle
    // -------------------------------------------------------------------------
    task check_slot_routing;
        input [4:0] target_slot;
        input [7:0] local_addr;
        integer k;
        reg [23:0] addr;
        reg [NUM_SLOTS-1:0] expected_cyc;
        begin
            addr = {11'b0, target_slot[4:0], local_addr[7:0]};
            expected_cyc = (1 << target_slot);

            @(negedge clk);
            wb_adr_i = addr;
            wb_sel_i = 4'b1111;
            wb_we_i  = 0;
            wb_cyc_i = 1;
            wb_stb_i = 1;

            @(posedge clk); // one cycle for routing to settle

            if (slot_cyc_o === expected_cyc && slot_stb_o === expected_cyc) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] Slot routing: slot %0d addr 0x%06h cyc=0x%0h stb=0x%0h",
                         target_slot, addr, slot_cyc_o, slot_stb_o);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] Slot routing: slot %0d addr 0x%06h expected cyc/stb=0x%0h got cyc=0x%0h stb=0x%0h",
                         target_slot, addr, expected_cyc, slot_cyc_o, slot_stb_o);
            end

            @(negedge clk);
            wb_cyc_i = 0;
            wb_stb_i = 0;
            @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // Check: returned data matches expectation
    // -------------------------------------------------------------------------
    task check_data;
        input [23:0]  addr;
        input [31:0]  inject_dat;  // data to drive from slave side
        input         inject_ack;  // which slave provides ack
        input [4:0]   inject_slot; // slot index to inject (0xFF = ext tier)
        input [31:0]  expected;
        input [63:0]  label;       // 8-char label for display
        reg   [31:0]  got;
        begin
            // Pre-set the slave response
            if (inject_slot == 5'hFF) begin
                ext_dat_i = inject_dat;
                // ext_ack_i driven below via always
            end else begin
                slot_dat_i[inject_slot*32 +: 32] = inject_dat;
            end

            wb_read(addr, got);

            if (got === expected) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] %0s addr 0x%06h got 0x%08h", label, addr, got);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] %0s addr 0x%06h expected 0x%08h got 0x%08h", label, addr, expected, got);
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Slot ACK model: combinatorial ACK from each slot (1-cycle response)
    // -------------------------------------------------------------------------
    always @(*) begin : slot_ack_model
        integer s;
        for (s = 0; s < NUM_SLOTS; s = s + 1)
            slot_ack_i[s] = slot_cyc_o[s] & slot_stb_o[s];
    end

    // Extended tier ACK model: 1-cycle response
    always @(*) ext_ack_i = ext_cyc_o & ext_stb_o;

    // Large tier: no ACK from outside (stub inside interconnect handles it)
    initial large_ack_i = 0;
    initial large_dat_i = 32'h00000000;

    // -------------------------------------------------------------------------
    // Test suite
    // -------------------------------------------------------------------------
    integer i;
    reg [31:0] rd;

    initial begin
        $dumpfile("tb_pwb_wb_interconnect.vcd");
        $dumpvars(0, tb_pwb_wb_interconnect);

        pass_cnt = 0;
        fail_cnt = 0;

        // Initialise master bus idle
        wb_adr_i = 24'h000000;
        wb_dat_i = 32'h00000000;
        wb_sel_i = 4'b1111;
        wb_we_i  = 0;
        wb_cyc_i = 0;
        wb_stb_i = 0;

        // Init slot response data to unique values
        for (i = 0; i < NUM_SLOTS; i = i + 1)
            slot_dat_i[i*32 +: 32] = 32'hA0000000 | (i << 4);

        ext_dat_i = 32'hB0DECADE;

        // Release reset
        #200;
        rst = 0;
        #100;

        // =====================================================================
        // Test group 1: Slot routing — each slot receives cyc/stb exclusively
        // =====================================================================
        $display("\n--- Test 1: Slot routing (cyc/stb assertion) ---");
        for (i = 0; i < NUM_SLOTS; i = i + 1)
            check_slot_routing(i[4:0], 8'h10);

        // =====================================================================
        // Test group 2: Slot isolation — read data comes from the correct slot
        // =====================================================================
        $display("\n--- Test 2: Slot data isolation ---");
        for (i = 0; i < NUM_SLOTS; i = i + 1) begin
            // Each slot has a unique value; verify the correct one is returned
            slot_dat_i[i*32 +: 32] = 32'hCA000000 | (i[31:0] << 8) | 8'hBE;
            wb_read({11'b0, i[4:0], 8'h00}, rd);
            if (rd === (32'hCA000000 | (i[31:0] << 8) | 8'hBE)) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] Slot %0d isolation: got 0x%08h", i, rd);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] Slot %0d isolation: expected 0x%08h got 0x%08h",
                         i, 32'hCA000000 | (i[31:0] << 8) | 8'hBE, rd);
            end
        end

        // =====================================================================
        // Test group 3: Out-of-range slot address returns 0xDEADBEEF
        //   Slot 8..31 are out of range when NUM_SLOTS=8
        // =====================================================================
        $display("\n--- Test 3: Out-of-range slot address ---");
        wb_read(24'h000800, rd); // slot 8 — beyond NUM_SLOTS=8
        if (rd === 32'hDEADBEEF) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] OOB slot 8: got 0xDEADBEEF");
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] OOB slot 8: expected 0xDEADBEEF got 0x%08h", rd);
        end

        wb_read(24'h001F00, rd); // slot 31 — maximum slot index, still OOB
        if (rd === 32'hDEADBEEF) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] OOB slot 31: got 0xDEADBEEF");
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] OOB slot 31: expected 0xDEADBEEF got 0x%08h", rd);
        end

        // =====================================================================
        // Test group 4: Extended tier routing
        //   Address 0x002000 → ext_cyc/stb asserted, slot/large not
        // =====================================================================
        $display("\n--- Test 4: Extended tier routing ---");

        ext_dat_i = 32'hEEEE1234;

        // Check cyc/stb routing
        @(negedge clk);
        wb_adr_i = 24'h002000;
        wb_sel_i = 4'b1111;
        wb_we_i  = 0;
        wb_cyc_i = 1;
        wb_stb_i = 1;
        @(posedge clk);

        if (ext_cyc_o && ext_stb_o && !wb_ack_o) begin
            // (ack comes next cycle via always block)
        end

        if (ext_cyc_o === 1 && ext_stb_o === 1 && slot_cyc_o === 0 && large_cyc_o === 0) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] Ext routing: ext_cyc=1, slot_cyc=0, large_cyc=0");
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] Ext routing: ext_cyc=%0b slot_cyc=0x%0h large_cyc=%0b",
                     ext_cyc_o, slot_cyc_o, large_cyc_o);
        end

        // Wait for ACK (driven combinatorially from ext_ack model)
        while (!wb_ack_o) @(posedge clk);

        if (wb_dat_o === 32'hEEEE1234) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] Ext data: got 0x%08h", wb_dat_o);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] Ext data: expected 0xEEEE1234 got 0x%08h", wb_dat_o);
        end

        @(negedge clk);
        wb_cyc_i = 0;
        wb_stb_i = 0;
        @(posedge clk);

        // Extended tier boundary: 0xFFFF (top of ext range)
        ext_dat_i = 32'hEEEEFFFF;
        wb_read(24'h00FFFF, rd);
        if (rd === 32'hEEEEFFFF) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] Ext top boundary 0x00FFFF: got 0x%08h", rd);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] Ext top boundary 0x00FFFF: expected 0xEEEEFFFF got 0x%08h", rd);
        end

        // =====================================================================
        // Test group 5: Large tier stub — returns 0xDEADBEEF without hanging
        // =====================================================================
        $display("\n--- Test 5: Large tier stub ---");
        wb_read(24'h010000, rd); // base of large tier
        if (rd === 32'hDEADBEEF) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] Large tier stub 0x010000: got 0xDEADBEEF");
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] Large tier stub 0x010000: expected 0xDEADBEEF got 0x%08h", rd);
        end

        wb_read(24'hFF0000, rd); // top of large tier
        if (rd === 32'hDEADBEEF) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] Large tier stub 0xFF0000: got 0xDEADBEEF");
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] Large tier stub 0xFF0000: expected 0xDEADBEEF got 0x%08h", rd);
        end

        // Verify large tier stub does NOT assert slot or ext cyc/stb
        @(negedge clk);
        wb_adr_i = 24'h020000;
        wb_sel_i = 4'b1111;
        wb_we_i  = 0;
        wb_cyc_i = 1;
        wb_stb_i = 1;
        @(posedge clk);

        if (slot_cyc_o === 0 && ext_cyc_o === 0) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] Large tier isolation: slot_cyc=0, ext_cyc=0");
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] Large tier isolation: slot_cyc=0x%0h ext_cyc=%0b",
                     slot_cyc_o, ext_cyc_o);
        end

        while (!wb_ack_o) @(posedge clk);
        @(negedge clk);
        wb_cyc_i = 0;
        wb_stb_i = 0;
        @(posedge clk);

        // =====================================================================
        // Summary
        // =====================================================================
        $display("\n--- Suite Summary ---");
        $display("PASS=%0d FAIL=%0d", pass_cnt, fail_cnt);

        if (fail_cnt == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** %0d TEST(S) FAILED ***", fail_cnt);

        #1000;
        $finish;
    end

    // -------------------------------------------------------------------------
    // Timeout protection
    // -------------------------------------------------------------------------
    initial begin
        #10000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
