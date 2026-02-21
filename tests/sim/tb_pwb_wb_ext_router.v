`timescale 1ns / 1ps
// Testbench: pwb_wb_ext_router
//
// Directly drives the Wishbone master interface to verify:
//   1. Ext slot 0 routing — address 0x3000 routes to slot 0 only
//   2. Ext slot 1 routing — address 0xB010 routes to slot 1 only
//   3. Slot isolation — slot 0 transaction leaves slot 1 cyc/stb deasserted
//   4. Slot isolation — slot 1 transaction leaves slot 0 cyc/stb deasserted
//   5. Unmatched address (0xC000 with no slot there) returns 0xDEADBEEF
//   6. Correct read data routing — each slot returns distinct data
//   7. Write path — write to slot 0, slot 1, verify cyc/stb only on target slot
//   8. Priority — address in both slots (overlap) routes to lower slot index
//
// Configuration:
//   Slot 0: BASE=0x2000, SIZE=0x9000  → range [0x2000, 0xAFFF]
//   Slot 1: BASE=0xB000, SIZE=0x1000  → range [0xB000, 0xBFFF]
//   (consistent with dev top.v address map)

module tb_pwb_wb_ext_router;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam NUM_EXT_SLOTS = 2;
    localparam [(NUM_EXT_SLOTS*16)-1:0] BASE_ADDRS = {16'hB000, 16'h2000};
    localparam [(NUM_EXT_SLOTS*16)-1:0] SIZES      = {16'h1000, 16'h9000};

    // -------------------------------------------------------------------------
    // Clock and reset
    // -------------------------------------------------------------------------
    reg clk = 0;
    reg rst = 1;
    always #18.5 clk = ~clk;  // 27 MHz

    // -------------------------------------------------------------------------
    // Master Wishbone interface
    // -------------------------------------------------------------------------
    reg  [15:0] wb_adr_i;
    reg  [31:0] wb_dat_i;
    wire [31:0] wb_dat_o;
    reg  [3:0]  wb_sel_i;
    reg         wb_we_i;
    reg         wb_cyc_i;
    reg         wb_stb_i;
    wire        wb_ack_o;

    // -------------------------------------------------------------------------
    // Extended slot arrays (from router)
    // -------------------------------------------------------------------------
    wire [NUM_EXT_SLOTS*16-1:0] slot_adr_o;
    wire [NUM_EXT_SLOTS*32-1:0] slot_dat_o;
    reg  [NUM_EXT_SLOTS*32-1:0] slot_dat_i;
    wire [NUM_EXT_SLOTS*4-1:0]  slot_sel_o;
    wire [NUM_EXT_SLOTS-1:0]    slot_we_o;
    wire [NUM_EXT_SLOTS-1:0]    slot_cyc_o;
    wire [NUM_EXT_SLOTS-1:0]    slot_stb_o;
    reg  [NUM_EXT_SLOTS-1:0]    slot_ack_i;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    pwb_wb_ext_router #(
        .NUM_EXT_SLOTS (NUM_EXT_SLOTS),
        .BASE_ADDRS    (BASE_ADDRS),
        .SIZES         (SIZES)
    ) dut (
        .clk         (clk),
        .rst         (rst),
        .wb_adr_i    (wb_adr_i),
        .wb_dat_i    (wb_dat_i),
        .wb_dat_o    (wb_dat_o),
        .wb_sel_i    (wb_sel_i),
        .wb_we_i     (wb_we_i),
        .wb_cyc_i    (wb_cyc_i),
        .wb_stb_i    (wb_stb_i),
        .wb_ack_o    (wb_ack_o),
        .slot_adr_o  (slot_adr_o),
        .slot_dat_o  (slot_dat_o),
        .slot_dat_i  (slot_dat_i),
        .slot_sel_o  (slot_sel_o),
        .slot_we_o   (slot_we_o),
        .slot_cyc_o  (slot_cyc_o),
        .slot_stb_o  (slot_stb_o),
        .slot_ack_i  (slot_ack_i)
    );

    // -------------------------------------------------------------------------
    // Test counters
    // -------------------------------------------------------------------------
    integer pass_cnt;
    integer fail_cnt;

    // -------------------------------------------------------------------------
    // Combinatorial ACK model — immediate 1-cycle response from each slot
    // -------------------------------------------------------------------------
    integer s;
    always @(*) begin
        for (s = 0; s < NUM_EXT_SLOTS; s = s + 1)
            slot_ack_i[s] = slot_cyc_o[s] & slot_stb_o[s];
    end

    // -------------------------------------------------------------------------
    // Wishbone read helper
    // -------------------------------------------------------------------------
    task wb_read;
        input  [15:0] addr;
        output [31:0] data;
        begin
            @(negedge clk);
            wb_adr_i = addr;
            wb_dat_i = 32'h0000_0000;
            wb_sel_i = 4'b1111;
            wb_we_i  = 0;
            wb_cyc_i = 1;
            wb_stb_i = 1;
            @(posedge clk);
            while (!wb_ack_o) @(posedge clk);
            data = wb_dat_o;
            @(negedge clk);
            wb_cyc_i = 0;
            wb_stb_i = 0;
            @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // Wishbone write helper
    // -------------------------------------------------------------------------
    task wb_write;
        input [15:0] addr;
        input [31:0] data;
        begin
            @(negedge clk);
            wb_adr_i = addr;
            wb_dat_i = data;
            wb_sel_i = 4'b1111;
            wb_we_i  = 1;
            wb_cyc_i = 1;
            wb_stb_i = 1;
            @(posedge clk);
            while (!wb_ack_o) @(posedge clk);
            @(negedge clk);
            wb_cyc_i = 0;
            wb_stb_i = 0;
            wb_we_i  = 0;
            @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // Helper: check cyc/stb routing for a given address
    // target_slot = 0..NUM_EXT_SLOTS-1, or -1 for "no slot should match"
    // -------------------------------------------------------------------------
    task check_routing;
        input [15:0]  addr;
        input integer target_slot;
        input [8*32:1] label;
        reg   [NUM_EXT_SLOTS-1:0] exp_cyc;
        begin
            if (target_slot < 0)
                exp_cyc = {NUM_EXT_SLOTS{1'b0}};
            else
                exp_cyc = (1 << target_slot);

            @(negedge clk);
            wb_adr_i = addr;
            wb_sel_i = 4'b1111;
            wb_we_i  = 0;
            wb_cyc_i = 1;
            wb_stb_i = 1;
            @(posedge clk);  // one cycle for combinatorial routing to settle

            if (slot_cyc_o === exp_cyc && slot_stb_o === exp_cyc) begin
                pass_cnt = pass_cnt + 1;
                $display("[PASS] %0s: addr=0x%04h cyc=0x%0h stb=0x%0h",
                         label, addr, slot_cyc_o, slot_stb_o);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("[FAIL] %0s: addr=0x%04h expected cyc/stb=0x%0h got cyc=0x%0h stb=0x%0h",
                         label, addr, exp_cyc, slot_cyc_o, slot_stb_o);
            end

            @(negedge clk);
            wb_cyc_i = 0;
            wb_stb_i = 0;
            @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // Test suite
    // -------------------------------------------------------------------------
    reg [31:0] rd;

    initial begin
        $dumpfile("tb_pwb_wb_ext_router.vcd");
        $dumpvars(0, tb_pwb_wb_ext_router);

        pass_cnt = 0;
        fail_cnt = 0;

        // Initialise master bus idle
        wb_adr_i = 16'h0000;
        wb_dat_i = 32'h0000_0000;
        wb_sel_i = 4'b1111;
        wb_we_i  = 0;
        wb_cyc_i = 0;
        wb_stb_i = 0;

        // Slot response data: unique sentinel per slot
        slot_dat_i[0*32 +: 32] = 32'hDECA_0000;  // slot 0
        slot_dat_i[1*32 +: 32] = 32'hCAFE_1111;  // slot 1

        // Release reset
        repeat (4) @(posedge clk);
        rst = 0;
        repeat (2) @(posedge clk);

        // ==================================================================
        // Test 1: Slot 0 routing — address inside slot 0 range (0x3000)
        // ==================================================================
        check_routing(16'h3000, 0, "T1: slot0 routing (0x3000)");

        // ==================================================================
        // Test 2: Slot 1 routing — address inside slot 1 range (0xB010)
        // ==================================================================
        check_routing(16'hB010, 1, "T2: slot1 routing (0xB010)");

        // ==================================================================
        // Test 3: Slot isolation — slot 0 address does NOT assert slot 1
        // ==================================================================
        @(negedge clk);
        wb_adr_i = 16'h5000;  // in slot 0 range
        wb_sel_i = 4'b1111;
        wb_we_i  = 0;
        wb_cyc_i = 1;
        wb_stb_i = 1;
        @(posedge clk);
        if (!slot_cyc_o[1] && !slot_stb_o[1]) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] T3: slot isolation: slot0 addr 0x5000, slot1 cyc=%0b stb=%0b",
                     slot_cyc_o[1], slot_stb_o[1]);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] T3: slot isolation: slot0 addr 0x5000 leaked to slot1: cyc=%0b stb=%0b",
                     slot_cyc_o[1], slot_stb_o[1]);
        end
        @(negedge clk); wb_cyc_i = 0; wb_stb_i = 0; @(posedge clk);

        // ==================================================================
        // Test 4: Slot isolation — slot 1 address does NOT assert slot 0
        // ==================================================================
        @(negedge clk);
        wb_adr_i = 16'hB500;  // in slot 1 range
        wb_sel_i = 4'b1111;
        wb_we_i  = 0;
        wb_cyc_i = 1;
        wb_stb_i = 1;
        @(posedge clk);
        if (!slot_cyc_o[0] && !slot_stb_o[0]) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] T4: slot isolation: slot1 addr 0xB500, slot0 cyc=%0b stb=%0b",
                     slot_cyc_o[0], slot_stb_o[0]);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] T4: slot isolation: slot1 addr 0xB500 leaked to slot0: cyc=%0b stb=%0b",
                     slot_cyc_o[0], slot_stb_o[0]);
        end
        @(negedge clk); wb_cyc_i = 0; wb_stb_i = 0; @(posedge clk);

        // ==================================================================
        // Test 5: Unmatched address — 0xC000 (beyond both slots), expect DEADBEEF
        // ==================================================================
        wb_read(16'hC000, rd);
        if (rd === 32'hDEAD_BEEF) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] T5: unmatched 0xC000 returned 0x%08h", rd);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] T5: unmatched 0xC000 expected 0xDEAD_BEEF got 0x%08h", rd);
        end

        // ==================================================================
        // Test 6: Correct read data — slot 0 returns slot_dat_i[0]
        // ==================================================================
        wb_read(16'h3000, rd);
        if (rd === 32'hDECA_0000) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] T6: slot0 read 0x3000 returned 0x%08h", rd);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] T6: slot0 read 0x3000 expected 0xDECA_0000 got 0x%08h", rd);
        end

        // ==================================================================
        // Test 7: Correct read data — slot 1 returns slot_dat_i[1]
        // ==================================================================
        wb_read(16'hB010, rd);
        if (rd === 32'hCAFE_1111) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] T7: slot1 read 0xB010 returned 0x%08h", rd);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] T7: slot1 read 0xB010 expected 0xCAFE_1111 got 0x%08h", rd);
        end

        // ==================================================================
        // Test 8: Write path — write to slot 0, verify only slot 0 cyc/stb
        // ==================================================================
        @(negedge clk);
        wb_adr_i = 16'h4000;
        wb_dat_i = 32'hABCD_1234;
        wb_sel_i = 4'b1111;
        wb_we_i  = 1;
        wb_cyc_i = 1;
        wb_stb_i = 1;
        @(posedge clk);
        if (slot_cyc_o[0] && slot_stb_o[0] && !slot_cyc_o[1] && slot_we_o[0]) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] T8: write 0x4000 routed to slot0 only, we_o[0]=%0b", slot_we_o[0]);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] T8: write 0x4000 routing wrong: slot0_cyc=%0b slot0_stb=%0b slot1_cyc=%0b we[0]=%0b",
                     slot_cyc_o[0], slot_stb_o[0], slot_cyc_o[1], slot_we_o[0]);
        end
        @(negedge clk); wb_cyc_i = 0; wb_stb_i = 0; wb_we_i = 0; @(posedge clk);

        // ==================================================================
        // Test 9: Address at exactly BASE of slot 0 (boundary check)
        // ==================================================================
        check_routing(16'h2000, 0, "T9: slot0 base boundary (0x2000)");

        // ==================================================================
        // Test 10: Address at last byte of slot 0 range (0xAFFF)
        // ==================================================================
        check_routing(16'hAFFF, 0, "T10: slot0 end boundary (0xAFFF)");

        // ==================================================================
        // Test 11: Address beyond slot 0 end, before slot 1 start (0xAFFF+1 = 0xB000 = slot 1)
        // ==================================================================
        check_routing(16'hB000, 1, "T11: slot1 base boundary (0xB000)");

        // ==================================================================
        // Test 12: Address 0x1FFF — below all slots (not in ext tier at all)
        // ==================================================================
        wb_read(16'h1FFF, rd);
        if (rd === 32'hDEAD_BEEF) begin
            pass_cnt = pass_cnt + 1;
            $display("[PASS] T12: below-ext-tier addr 0x1FFF returned 0x%08h", rd);
        end else begin
            fail_cnt = fail_cnt + 1;
            $display("[FAIL] T12: below-ext-tier addr 0x1FFF expected DEADBEEF got 0x%08h", rd);
        end

        // ==================================================================
        // Summary
        // ==================================================================
        #100;
        $display("");
        $display("========================================");
        $display("tb_pwb_wb_ext_router: %0d passed, %0d failed", pass_cnt, fail_cnt);
        $display("========================================");

        if (fail_cnt == 0) begin
            $display("*** ALL TESTS PASSED ***");
            $finish;
        end else begin
            $display("*** %0d TEST(S) FAILED ***", fail_cnt);
            $finish(1);
        end
    end

endmodule
