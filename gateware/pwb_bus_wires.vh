// Papilio Wishbone Bus - Wire Declarations and Helper Macros
// Include this file in your top.v after defining:
//   wire clk;                   (your system clock)
//   localparam NUM_SLOTS = 8;   (number of peripheral slots)
//
// You must also have a reset signal available as 'rst'. Two options:
//   Option A (end users): pwb_wb_system drives rst via .rst_o(rst)
//   Option B (advanced):  Define your own reset generator (reg rst = ...)
//
// This file provides:
//   1. slot_* wires       - Flattened arrays for slot Wishbone interfaces
//   2. SLOT_CONNECT macro - One-line peripheral wiring
//   3. PWB_SLOT_PORTS     - Port list macro for pwb_wb_system instantiation

// =========================================================================
// Slot Wishbone interface wires (flattened arrays)
// =========================================================================
wire [NUM_SLOTS*16-1:0]  slot_adr;
wire [NUM_SLOTS*32-1:0]  slot_dat_m2s;
wire [NUM_SLOTS*32-1:0]  slot_dat_s2m;
wire [NUM_SLOTS*4-1:0]   slot_sel;
wire [NUM_SLOTS-1:0]     slot_we;
wire [NUM_SLOTS-1:0]     slot_cyc;
wire [NUM_SLOTS-1:0]     slot_stb;
wire [NUM_SLOTS-1:0]     slot_ack;

// =========================================================================
// SLOT_CONNECT Macro - One-line peripheral wiring
// =========================================================================
// Usage: `SLOT_CONNECT(slot_number, module_with_params, instance_name)
//
// Wires all 8 standard Wishbone signals between the interconnect's
// flattened arrays and a peripheral module. Includes clk and rst.
//
// Examples:
//   `SLOT_CONNECT(0, wb_register_block #(.DATA_WIDTH(8)),  slot0_sys);
//   `SLOT_CONNECT(1, wb_rgb_led,                           slot1_rgb);
//
// For peripherals with extra I/O (e.g., LED output), wire those
// separately after the macro call:
//   `SLOT_CONNECT(5, wb_rgb_led, slot5_rgb);
//   assign rgb_led_out = slot5_rgb_extra_output;
//
`define SLOT_CONNECT(N, MODULE, INST) \
    MODULE INST ( \
        .clk(clk), \
        .rst(rst), \
        .wb_adr_i(slot_adr[(N)*16 +: 16]), \
        .wb_dat_i(slot_dat_m2s[(N)*32 +: 32]), \
        .wb_dat_o(slot_dat_s2m[(N)*32 +: 32]), \
        .wb_sel_i(slot_sel[(N)*4 +: 4]), \
        .wb_we_i(slot_we[(N)]), \
        .wb_cyc_i(slot_cyc[(N)]), \
        .wb_stb_i(slot_stb[(N)]), \
        .wb_ack_o(slot_ack[(N)]) \
    )

// =========================================================================
// PWB_SLOT_PORTS Macro - System module port wiring
// =========================================================================
// Use inside the pwb_wb_system instantiation to connect all slot ports:
//
//   pwb_wb_system #(.NUM_SLOTS(NUM_SLOTS)) bus (
//       .clk(clk),
//       .rst_o(rst),
//       .spi_sclk(spi_sclk),
//       .spi_mosi(spi_mosi),
//       .spi_miso(spi_miso),
//       .spi_cs_n(spi_cs_n),
//       `PWB_SLOT_PORTS
//   );
//
`define PWB_SLOT_PORTS \
    .slot_adr_o(slot_adr), \
    .slot_dat_o(slot_dat_m2s), \
    .slot_dat_i(slot_dat_s2m), \
    .slot_sel_o(slot_sel), \
    .slot_we_o(slot_we), \
    .slot_cyc_o(slot_cyc), \
    .slot_stb_o(slot_stb), \
    .slot_ack_i(slot_ack)
