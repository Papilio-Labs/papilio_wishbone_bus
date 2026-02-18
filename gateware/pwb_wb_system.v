// Papilio Wishbone System
// High-level wrapper that encapsulates the complete bus infrastructure:
//   - Reset generation (16-cycle startup)
//   - SPI-to-Wishbone bridge (pwb_spi_wb_bridge)
//   - Three-tier Wishbone interconnect (pwb_wb_interconnect)
//
// End users instantiate this module once and connect peripherals via
// SLOT_CONNECT macro from pwb_bus_wires.vh.  For peripherals that need
// more address space (e.g., BRAM), wire the ext_* ports to the slave.
//
// Usage:
//   wire clk = clk_27mhz;
//   localparam NUM_SLOTS = 8;
//   wire rst;
//   `include "pwb_bus_wires.vh"
//
//   pwb_wb_system #(.NUM_SLOTS(NUM_SLOTS)) bus (
//       .clk(clk),
//       .rst_o(rst),
//       .spi_sclk(spi_sclk),
//       .spi_mosi(spi_mosi),
//       .spi_miso(spi_miso),
//       .spi_cs_n(spi_cs_n),
//       `PWB_SLOT_PORTS,
//       .ext_adr_o(ext_adr),   // Optional: extended tier for BRAM etc.
//       ...
//   );
//
//   `SLOT_CONNECT(0, wb_register_block #(.DATA_WIDTH(8)), slot0_sys);
//   `SLOT_CONNECT(1, wb_rgb_led,                          slot1_rgb);

module pwb_wb_system #(
    parameter NUM_SLOTS = 8            // Number of slot peripherals (1-32)
) (
    input  wire clk,                   // System clock
    output wire rst_o,                 // Reset output for user modules

    // SPI interface (directly from top-level pins)
    input  wire spi_sclk,
    input  wire spi_mosi,
    output wire spi_miso,
    input  wire spi_cs_n,

    // Slot Wishbone interfaces (flattened arrays — connect via PWB_SLOT_PORTS)
    output wire [NUM_SLOTS*16-1:0]  slot_adr_o,
    output wire [NUM_SLOTS*32-1:0]  slot_dat_o,
    input  wire [NUM_SLOTS*32-1:0]  slot_dat_i,
    output wire [NUM_SLOTS*4-1:0]   slot_sel_o,
    output wire [NUM_SLOTS-1:0]     slot_we_o,
    output wire [NUM_SLOTS-1:0]     slot_cyc_o,
    output wire [NUM_SLOTS-1:0]     slot_stb_o,
    input  wire [NUM_SLOTS-1:0]     slot_ack_i,

    // Extended tier interface (active after Phase 2, active low stub by default)
    output wire [15:0] ext_adr_o,
    output wire [31:0] ext_dat_o,
    input  wire [31:0] ext_dat_i,
    output wire [3:0]  ext_sel_o,
    output wire        ext_we_o,
    output wire        ext_cyc_o,
    output wire        ext_stb_o,
    input  wire        ext_ack_i
);

    // =========================================================================
    // Reset generation — hold reset high for 16 clock cycles on startup
    // =========================================================================
    reg [3:0] reset_counter = 4'b0000;
    reg       rst = 1'b1;

    always @(posedge clk) begin
        if (reset_counter != 4'b1111) begin
            reset_counter <= reset_counter + 1'b1;
            rst <= 1'b1;
        end else begin
            rst <= 1'b0;
        end
    end

    assign rst_o = rst;

    // =========================================================================
    // SPI-to-Wishbone bridge
    // =========================================================================
    wire [15:0] wb_adr_16;
    wire [31:0] wb_dat_m2s;
    wire [31:0] wb_dat_s2m;
    wire [3:0]  wb_sel;
    wire        wb_we;
    wire        wb_cyc;
    wire        wb_stb;
    wire        wb_ack;

    pwb_spi_wb_bridge bridge (
        .clk(clk),
        .rst(rst),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .wb_adr_o(wb_adr_16),
        .wb_dat_o(wb_dat_m2s),
        .wb_dat_i(wb_dat_s2m),
        .wb_sel_o(wb_sel),
        .wb_we_o(wb_we),
        .wb_cyc_o(wb_cyc),
        .wb_stb_o(wb_stb),
        .wb_ack_i(wb_ack)
    );

    // =========================================================================
    // Address extension: 16-bit from bridge → 24-bit for interconnect
    // (Phase 3 will upgrade bridge to native 24-bit, this becomes a pass-through)
    // =========================================================================
    wire [23:0] wb_adr = {8'b0, wb_adr_16};

    // =========================================================================
    // Three-tier Wishbone interconnect
    // =========================================================================
    pwb_wb_interconnect #(
        .NUM_SLOTS(NUM_SLOTS)
    ) interconnect (
        .clk(clk),
        .rst(rst),

        // Master interface (from SPI bridge)
        .wb_adr_i(wb_adr),
        .wb_dat_i(wb_dat_m2s),
        .wb_dat_o(wb_dat_s2m),
        .wb_sel_i(wb_sel),
        .wb_we_i(wb_we),
        .wb_cyc_i(wb_cyc),
        .wb_stb_i(wb_stb),
        .wb_ack_o(wb_ack),

        // Slot interfaces (pass through to top-level)
        .slot_adr_o(slot_adr_o),
        .slot_dat_o(slot_dat_o),
        .slot_dat_i(slot_dat_i),
        .slot_sel_o(slot_sel_o),
        .slot_we_o(slot_we_o),
        .slot_cyc_o(slot_cyc_o),
        .slot_stb_o(slot_stb_o),
        .slot_ack_i(slot_ack_i),

        // Extended tier (pass through to top-level)
        .ext_adr_o(ext_adr_o),
        .ext_dat_o(ext_dat_o),
        .ext_dat_i(ext_dat_i),
        .ext_sel_o(ext_sel_o),
        .ext_we_o(ext_we_o),
        .ext_cyc_o(ext_cyc_o),
        .ext_stb_o(ext_stb_o),
        .ext_ack_i(ext_ack_i),

        // Large tier (Phase 3 — stubbed inside this wrapper)
        .large_adr_o(),
        .large_dat_o(),
        .large_dat_i(32'h00000000),
        .large_sel_o(),
        .large_we_o(),
        .large_cyc_o(),
        .large_stb_o(),
        .large_ack_i(1'b0)
    );

endmodule
