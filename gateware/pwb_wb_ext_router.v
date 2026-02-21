// Papilio Wishbone Bus — Extended Tier Router
//
// Routes Wishbone transactions within the extended address space (0x2000–0xFFFF)
// to one of NUM_EXT_SLOTS sub-peripherals based on programmed BASE_ADDRS/SIZES.
//
// Usage (via pwb_bus_wires.vh auto-instantiation):
//   - Define  `define NUM_EXT_SLOTS  before including pwb_bus_wires.vh
//   - Set     localparam NUM_EXT_SLOTS = N;
//   - Set     localparam [N*16-1:0] EXT_BASE_ADDRS = {slot_N-1_base, ..., slot_0_base};
//   - Set     localparam [N*16-1:0] EXT_SIZES      = {slot_N-1_size, ..., slot_0_size};
//   The include file auto-instantiates this module as u_pwb_ext_router.
//
// Address Decode:
//   Slot N is active when: wb_adr_i >= BASE_ADDRS[N*16 +: 16]
//                       && wb_adr_i <  BASE_ADDRS[N*16 +: 16] + SIZES[N*16 +: 16]
//   Priority: lowest slot index wins when address ranges overlap (slot 0 highest priority).
//   Each slot receives the raw wb_adr_i — peripherals perform their own BASE_ADDR subtraction.
//   Unmatched addresses return ACK=1, dat_o=32'hDEADBEEF (consistent with interconnect).
//
// Register Map:
//   This module has no registers; it is a pure combinational/routing module.
//
// Parameters:
//   NUM_EXT_SLOTS  - Number of extended-tier sub-slots (default 4, max 16)
//   BASE_ADDRS     - Flattened 16-bit base address per slot; slot 0 in bits [15:0]
//   SIZES          - Flattened 16-bit address range size per slot; slot 0 in bits [15:0]
//
// Default slot map (4-slot config):
//   Slot 0: 0x2000–0x7FFF (24 KB)
//   Slot 1: 0x8000–0x8FFF (4 KB — not mapped by default, SIZE=0 means no match)
//   Slot 2: 0x9000–0x9FFF (4 KB — not mapped by default)
//   Slot 3: 0xA000–0xAFFF (4 KB — not mapped by default)

`timescale 1ns / 1ps

module pwb_wb_ext_router #(
    parameter                        NUM_EXT_SLOTS = 4,
    parameter [(NUM_EXT_SLOTS*16)-1:0] BASE_ADDRS  = {16'hA000, 16'h9000, 16'h8000, 16'h2000},
    parameter [(NUM_EXT_SLOTS*16)-1:0] SIZES        = {16'h1000, 16'h1000, 16'h6000, 16'h6000}
) (
    input  wire        clk,
    input  wire        rst,

    // -------------------------------------------------------------------------
    // Master interface (from ext_* wires, driven by pwb_wb_interconnect)
    // -------------------------------------------------------------------------
    input  wire [15:0] wb_adr_i,
    input  wire [31:0] wb_dat_i,
    output reg  [31:0] wb_dat_o,
    input  wire [3:0]  wb_sel_i,
    input  wire        wb_we_i,
    input  wire        wb_cyc_i,
    input  wire        wb_stb_i,
    output reg         wb_ack_o,

    // -------------------------------------------------------------------------
    // Slot interfaces (flattened arrays for NUM_EXT_SLOTS sub-peripherals)
    // -------------------------------------------------------------------------
    // Access pattern: slot_adr_o[N*16 +: 16] for slot N's address
    output wire [NUM_EXT_SLOTS*16-1:0] slot_adr_o,
    output wire [NUM_EXT_SLOTS*32-1:0] slot_dat_o,
    input  wire [NUM_EXT_SLOTS*32-1:0] slot_dat_i,
    output wire [NUM_EXT_SLOTS*4-1:0]  slot_sel_o,
    output wire [NUM_EXT_SLOTS-1:0]    slot_we_o,
    output wire [NUM_EXT_SLOTS-1:0]    slot_cyc_o,
    output wire [NUM_EXT_SLOTS-1:0]    slot_stb_o,
    input  wire [NUM_EXT_SLOTS-1:0]    slot_ack_i
);

    // -------------------------------------------------------------------------
    // Address decode: determine which slot (if any) matches the incoming address
    // -------------------------------------------------------------------------
    // in_range[N] = 1 when wb_adr_i falls within slot N's programmed range
    wire [NUM_EXT_SLOTS-1:0] in_range;

    genvar i;
    generate
        for (i = 0; i < NUM_EXT_SLOTS; i = i + 1) begin : addr_decode
            wire [15:0] base = BASE_ADDRS[i*16 +: 16];
            wire [15:0] size = SIZES[i*16 +: 16];
            // Match iff size > 0 AND address is within [base, base+size)
            assign in_range[i] = (size != 16'h0000) &&
                                  (wb_adr_i >= base) &&
                                  (wb_adr_i <  base + size);
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Priority encode: lowest slot index wins (slot 0 = highest priority)
    // -------------------------------------------------------------------------
    // sel_slot[N] = 1 only when in_range[N] AND no lower-numbered slot also matches
    wire [NUM_EXT_SLOTS-1:0] sel_slot;

    generate
        for (i = 0; i < NUM_EXT_SLOTS; i = i + 1) begin : priority_enc
            if (i == 0) begin
                assign sel_slot[0] = in_range[0];
            end else begin
                // Slot i is selected only if it is in_range AND no lower-index slot is selected
                wire any_lower;
                assign any_lower = |sel_slot[i-1:0];
                assign sel_slot[i] = in_range[i] & ~any_lower;
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Slot signal routing
    // -------------------------------------------------------------------------
    // All slots receive the same address/data/sel/we — only cyc/stb are gated.
    generate
        for (i = 0; i < NUM_EXT_SLOTS; i = i + 1) begin : slot_route
            assign slot_adr_o[i*16 +: 16] = wb_adr_i;
            assign slot_dat_o[i*32 +: 32] = wb_dat_i;
            assign slot_sel_o[i*4  +:  4] = wb_sel_i;
            assign slot_we_o[i]            = wb_we_i;
            // cyc and stb only asserted to the winning slot
            assign slot_cyc_o[i] = wb_cyc_i & sel_slot[i];
            assign slot_stb_o[i] = wb_stb_i & sel_slot[i];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Response mux: return selected slot's data/ack to master
    // -------------------------------------------------------------------------
    // Combinational — registered ACK is handled inside each peripheral.
    wire        any_matched = |sel_slot;

    reg  [31:0] mux_dat;
    reg         mux_ack;
    integer     j;

    always @(*) begin
        mux_dat = 32'h0000_0000;
        mux_ack = 1'b0;
        for (j = 0; j < NUM_EXT_SLOTS; j = j + 1) begin
            if (sel_slot[j]) begin
                mux_dat = slot_dat_i[j*32 +: 32];
                mux_ack = slot_ack_i[j];
            end
        end
    end

    always @(*) begin
        if (any_matched) begin
            wb_dat_o = mux_dat;
            wb_ack_o = mux_ack;
        end else begin
            // Unmatched address: immediate ACK with sentinel data
            wb_dat_o = 32'hDEAD_BEEF;
            wb_ack_o = wb_cyc_i & wb_stb_i;
        end
    end

endmodule
