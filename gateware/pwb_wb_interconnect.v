// Papilio Wishbone Interconnect
// Three-tier address decoder and signal router for Wishbone peripherals
//
// Tiers:
//   Slot:     0x00_0000 - 0x00_1FFF (NUM_SLOTS × 256 bytes)
//   Extended: 0x00_2000 - 0x00_FFFF (56 KB, on-chip BRAM/memory)
//   Large:    0x01_0000 - 0xFF_FFFF (~16 MB, external DDR/SDRAM)
//
// Phase 1: Slot tier only (Extended/Large stubbed)
// Phase 2: Extended tier enabled
// Phase 3: Large tier enabled
//
// Slot addressing:
//   - Bits [12:8] select slot (0-31 when NUM_SLOTS=32)
//   - Bits [7:0] are local address within slot
//   - Each slot gets 256 bytes (0x00-0xFF)
//   - Slot 0 reserved for system control
//
// Usage in top.v:
//   pwb_wb_interconnect #(.NUM_SLOTS(8)) interconnect (
//       .clk(clk_27mhz),
//       .rst(rst),
//       .wb_adr_i(wb_adr_from_spi_bridge),
//       ...
//       .slot_adr_o(slot_adr),
//       .slot_dat_o(slot_dat_m2s),
//       .slot_dat_i(slot_dat_s2m),
//       ...
//   );
//
//   Connect slots using SLOT_CONNECT macro:
//   `SLOT_CONNECT(0, wb_register_block, slot0_sys);

module pwb_wb_interconnect #(
    parameter NUM_SLOTS = 32,          // Number of slot peripherals (1-32)
    parameter SLOT_ADDR_BITS = 8       // Address bits per slot (256 bytes)
) (
    input  wire        clk,
    input  wire        rst,

    // -------------------------------------------------------------------------
    // Master interface (from SPI bridge)
    // -------------------------------------------------------------------------
    input  wire [23:0] wb_adr_i,       // 24-bit address (full 3-tier range)
    input  wire [31:0] wb_dat_i,       // Write data from master
    output reg  [31:0] wb_dat_o,       // Read data to master
    input  wire [3:0]  wb_sel_i,       // Byte select
    input  wire        wb_we_i,        // Write enable
    input  wire        wb_cyc_i,       // Cycle active
    input  wire        wb_stb_i,       // Strobe
    output reg         wb_ack_o,       // Acknowledge to master

    // -------------------------------------------------------------------------
    // Slot interfaces (flattened arrays for NUM_SLOTS peripherals)
    // -------------------------------------------------------------------------
    // Access pattern: slot_adr_o[N*16 +: 16] for slot N's address
    output wire [NUM_SLOTS*16-1:0]  slot_adr_o,    // Address to each slot
    output wire [NUM_SLOTS*32-1:0]  slot_dat_o,    // Write data to each slot
    input  wire [NUM_SLOTS*32-1:0]  slot_dat_i,    // Read data from each slot
    output wire [NUM_SLOTS*4-1:0]   slot_sel_o,    // Byte select to each slot
    output wire [NUM_SLOTS-1:0]     slot_we_o,     // Write enable to each slot
    output wire [NUM_SLOTS-1:0]     slot_cyc_o,    // Cycle to each slot (only selected slot asserted)
    output wire [NUM_SLOTS-1:0]     slot_stb_o,    // Strobe to each slot (only selected slot asserted)
    input  wire [NUM_SLOTS-1:0]     slot_ack_i,    // Acknowledge from each slot

    // -------------------------------------------------------------------------
    // Extended tier interface (Phase 2 - currently stubbed)
    // -------------------------------------------------------------------------
    output wire [15:0] ext_adr_o,
    output wire [31:0] ext_dat_o,
    input  wire [31:0] ext_dat_i,
    output wire [3:0]  ext_sel_o,
    output wire        ext_we_o,
    output wire        ext_cyc_o,
    output wire        ext_stb_o,
    input  wire        ext_ack_i,

    // -------------------------------------------------------------------------
    // Large tier interface (Phase 3 - currently stubbed)
    // -------------------------------------------------------------------------
    output wire [23:0] large_adr_o,
    output wire [31:0] large_dat_o,
    input  wire [31:0] large_dat_i,
    output wire [3:0]  large_sel_o,
    output wire        large_we_o,
    output wire        large_cyc_o,
    output wire        large_stb_o,
    input  wire        large_ack_i
);

    // -------------------------------------------------------------------------
    // Tier decode
    // -------------------------------------------------------------------------
    // Phase 1: Only slot tier active
    wire in_slot_tier  = (wb_adr_i[23:13] == 11'b0);  // 0x00_0000 - 0x00_1FFF
    wire in_ext_tier   = 1'b0;  // Phase 2: (wb_adr_i[23:16] == 8'b0) && wb_adr_i[15:13] != 3'b0
    wire in_large_tier = 1'b0;  // Phase 3: wb_adr_i[23:16] != 8'b0

    // -------------------------------------------------------------------------
    // Slot tier decode
    // -------------------------------------------------------------------------
    // Slot number from address bits [12:8] (supports up to 32 slots)
    wire [4:0] slot_sel_raw = wb_adr_i[12:8];
    
    // Clamp to NUM_SLOTS range (addresses beyond NUM_SLOTS-1 return default)
    wire slot_sel_valid = (slot_sel_raw < NUM_SLOTS);
    wire [4:0] slot_sel = slot_sel_valid ? slot_sel_raw : 5'd0;

    // -------------------------------------------------------------------------
    // Generate-based slot signal routing
    // -------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < NUM_SLOTS; i = i + 1) begin : slot_gen
            // Each slot gets the full master signals
            assign slot_adr_o[i*16 +: 16] = wb_adr_i[15:0];
            assign slot_dat_o[i*32 +: 32] = wb_dat_i;
            assign slot_sel_o[i*4 +: 4]   = wb_sel_i;
            assign slot_we_o[i]           = wb_we_i;
            
            // But only the selected slot in slot tier gets cyc/stb
            assign slot_cyc_o[i] = wb_cyc_i & in_slot_tier & slot_sel_valid & (slot_sel == i);
            assign slot_stb_o[i] = wb_stb_i & in_slot_tier & slot_sel_valid & (slot_sel == i);
        end
    endgenerate

    // -------------------------------------------------------------------------
    // ACK and data multiplexing from selected slave
    // -------------------------------------------------------------------------
    reg [31:0] selected_slot_dat;
    reg        selected_slot_ack;
    
    integer j;
    always @(*) begin
        // Default values
        selected_slot_dat = 32'h00000000;
        selected_slot_ack = 1'b0;
        
        // Mux the selected slot's data and ack
        for (j = 0; j < NUM_SLOTS; j = j + 1) begin
            if (slot_sel == j) begin
                selected_slot_dat = slot_dat_i[j*32 +: 32];
                selected_slot_ack = slot_ack_i[j];
            end
        end
    end

    // -------------------------------------------------------------------------
    // Phase 2/3 tier stubs (return ACK + 0xDEADBEEF)
    // -------------------------------------------------------------------------
    assign ext_adr_o   = 16'h0000;
    assign ext_dat_o   = 32'h00000000;
    assign ext_sel_o   = 4'b0000;
    assign ext_we_o    = 1'b0;
    assign ext_cyc_o   = 1'b0;
    assign ext_stb_o   = 1'b0;
    
    assign large_adr_o = 24'h000000;
    assign large_dat_o = 32'h00000000;
    assign large_sel_o = 4'b0000;
    assign large_we_o  = 1'b0;
    assign large_cyc_o = 1'b0;
    assign large_stb_o = 1'b0;

    // Stubbed tiers always ACK with distinctive data
    wire ext_stub_ack   = in_ext_tier & wb_cyc_i & wb_stb_i;
    wire large_stub_ack = in_large_tier & wb_cyc_i & wb_stb_i;

    // -------------------------------------------------------------------------
    // Final ACK and data to master
    // -------------------------------------------------------------------------
    always @(*) begin
        if (in_slot_tier && slot_sel_valid) begin
            // Valid slot address
            wb_dat_o = selected_slot_dat;
            wb_ack_o = selected_slot_ack;
        end else if (in_slot_tier && !slot_sel_valid) begin
            // Slot address out of range
            wb_dat_o = 32'hDEADBEEF;
            wb_ack_o = wb_cyc_i & wb_stb_i;  // Immediate ACK
        end else if (in_ext_tier) begin
            // Extended tier stub (Phase 2)
            wb_dat_o = 32'hDEADBEEF;
            wb_ack_o = ext_stub_ack;
        end else if (in_large_tier) begin
            // Large tier stub (Phase 3)
            wb_dat_o = 32'hDEADBEEF;
            wb_ack_o = large_stub_ack;
        end else begin
            // Address outside all tiers
            wb_dat_o = 32'hDEADBEEF;
            wb_ack_o = wb_cyc_i & wb_stb_i;  // Immediate ACK
        end
    end

endmodule
