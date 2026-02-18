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
//   2. ext_* wires        - Extended tier Wishbone interface wires
//   3. SLOT_CONNECT macro - One-line slot peripheral wiring
//   4. EXT_CONNECT macro  - One-line extended tier peripheral wiring
//   5. PWB_SLOT_PORTS     - Slot port list macro for pwb_wb_system
//   6. PWB_EXT_PORTS      - Extended tier port list macro for pwb_wb_system

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
// Wires clk, rst, and all 8 standard Wishbone signals between the
// interconnect's flattened arrays and a peripheral module.
//
// The macro leaves the port list OPEN — the caller closes with );
// This allows extra I/O ports to be added before closing.
//
// Simple peripheral (no extra ports):
//   `SLOT_CONNECT(0, wb_register_block #(.DATA_WIDTH(8)), slot0_sys);
//
// Peripheral with extra I/O:
//   `SLOT_CONNECT(5, wb_simple_rgb_led, slot5_rgb),
//       .led_out(led_out)
//   );
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
        .wb_ack_o(slot_ack[(N)])

// =========================================================================
// Extended Tier Wishbone interface wires
// =========================================================================
// These are always declared. If you don't use the extended tier,
// the synthesizer will optimize away the unused wires.
wire [15:0] ext_adr;
wire [31:0] ext_dat_m2s;
wire [31:0] ext_dat_s2m;
wire [3:0]  ext_sel;
wire        ext_we;
wire        ext_cyc;
wire        ext_stb;
wire        ext_ack;

// =========================================================================
// EXT_CONNECT Macro - One-line extended tier peripheral wiring
// =========================================================================
// Usage: `EXT_CONNECT(module_with_params, instance_name)
//
// Wires clk, rst, and all 8 standard Wishbone signals between the
// extended tier port and a peripheral module.
// Only one peripheral can be connected to the extended tier.
//
// The macro leaves the port list OPEN — the caller closes with );
//
// Simple peripheral (no extra ports):
//   `EXT_CONNECT(wb_bram #(.ADDR_WIDTH(10), .DATA_WIDTH(32)), ext_bram);
//
// Peripheral with extra I/O:
//   `EXT_CONNECT(my_peripheral, ext_dev),
//       .extra_out(some_wire)
//   );
//
`define EXT_CONNECT(MODULE, INST) \
    MODULE INST ( \
        .clk(clk), \
        .rst(rst), \
        .wb_adr_i(ext_adr), \
        .wb_dat_i(ext_dat_m2s), \
        .wb_dat_o(ext_dat_s2m), \
        .wb_sel_i(ext_sel), \
        .wb_we_i(ext_we), \
        .wb_cyc_i(ext_cyc), \
        .wb_stb_i(ext_stb), \
        .wb_ack_o(ext_ack)

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

// =========================================================================
// PWB_EXT_PORTS Macro - System module extended tier port wiring
// =========================================================================
// Use inside the pwb_wb_system instantiation to connect extended tier ports:
//
//   pwb_wb_system #(.NUM_SLOTS(NUM_SLOTS)) bus (
//       .clk(clk),
//       .rst_o(rst),
//       .spi_sclk(spi_sclk),
//       .spi_mosi(spi_mosi),
//       .spi_miso(spi_miso),
//       .spi_cs_n(spi_cs_n),
//       `PWB_SLOT_PORTS,
//       `PWB_EXT_PORTS
//   );
//
`define PWB_EXT_PORTS \
    .ext_adr_o(ext_adr), \
    .ext_dat_o(ext_dat_m2s), \
    .ext_dat_i(ext_dat_s2m), \
    .ext_sel_o(ext_sel), \
    .ext_we_o(ext_we), \
    .ext_cyc_o(ext_cyc), \
    .ext_stb_o(ext_stb), \
    .ext_ack_i(ext_ack)
