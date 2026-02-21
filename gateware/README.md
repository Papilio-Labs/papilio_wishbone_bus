# Gateware for papilio_wishbone_bus

This directory contains the FPGA-side Verilog modules for the `papilio_wishbone_bus` library. The SPI front end relies on the validated `papilio_spi_slave` core; drive it with an SPI Mode 1 master (CPOL=0, CPHA=1) using 8-bit transfers at up to 4 MHz.

## Modules

### `pwb_wb_system.v` — High-level system wrapper (recommended for end users)

Encapsulates the complete bus infrastructure in a single module:
- 16-cycle startup reset generator (drives `rst_o`)
- SPI-to-Wishbone bridge (`pwb_spi_wb_bridge`)
- Three-tier Wishbone interconnect (`pwb_wb_interconnect`)

**Parameters:**

| Parameter   | Default | Description                          |
|-------------|---------|--------------------------------------|
| `NUM_SLOTS` | `8`     | Number of slot peripherals (1–32)    |

**Ports:**

| Port           | Direction | Width           | Description                              |
|----------------|-----------|-----------------|------------------------------------------|
| `clk`          | in        | 1               | System clock                             |
| `rst_o`        | out       | 1               | Reset output for user modules            |
| `spi_sclk`     | in        | 1               | SPI clock from ESP32                     |
| `spi_mosi`     | in        | 1               | SPI MOSI from ESP32                      |
| `spi_miso`     | out       | 1               | SPI MISO to ESP32                        |
| `spi_cs_n`     | in        | 1               | SPI chip select (active low)             |
| `slot_adr_o`   | out       | NUM_SLOTS×16    | Address to each slot (flattened)         |
| `slot_dat_o`   | out       | NUM_SLOTS×32    | Write data to each slot (flattened)      |
| `slot_dat_i`   | in        | NUM_SLOTS×32    | Read data from each slot (flattened)     |
| `slot_sel_o`   | out       | NUM_SLOTS×4     | Byte select to each slot (flattened)     |
| `slot_we_o`    | out       | NUM_SLOTS       | Write enable per slot                    |
| `slot_cyc_o`   | out       | NUM_SLOTS       | Cycle per slot (only active slot set)    |
| `slot_stb_o`   | out       | NUM_SLOTS       | Strobe per slot (only active slot set)   |
| `slot_ack_i`   | in        | NUM_SLOTS       | Acknowledge per slot                     |
| `ext_adr_o`    | out       | 16              | Extended tier address (0x2000–0xFFFF)    |
| `ext_dat_o`    | out       | 32              | Extended tier write data                 |
| `ext_dat_i`    | in        | 32              | Extended tier read data                  |
| `ext_sel_o`    | out       | 4               | Extended tier byte select                |
| `ext_we_o`     | out       | 1               | Extended tier write enable               |
| `ext_cyc_o`    | out       | 1               | Extended tier cycle                      |
| `ext_stb_o`    | out       | 1               | Extended tier strobe                     |
| `ext_ack_i`    | in        | 1               | Extended tier acknowledge                |

**Typical instantiation** (using `pwb_bus_wires.vh`):

```verilog
wire clk = clk_27mhz;
localparam NUM_SLOTS = 8;
wire rst;
`include "pwb_bus_wires.vh"

pwb_wb_system #(.NUM_SLOTS(NUM_SLOTS)) bus (
    .clk(clk),
    .rst_o(rst),
    .spi_sclk(spi_sclk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_cs_n(spi_cs_n),
    `PWB_SLOT_PORTS
    // Uncomment to enable extended tier (BRAM at 0x2000+):
    // ,`PWB_EXT_PORTS
);
```

---

### `pwb_wb_interconnect.v` — Three-tier address decoder (advanced/dev)

Decodes the 24-bit Wishbone address and routes transactions to the correct tier. End users should use `pwb_wb_system` instead; this module is for advanced integrations where custom reset generation or bridge selection is needed.

**Address map:**

| Tier      | Range                   | Active? | Notes                          |
|-----------|-------------------------|---------|--------------------------------|
| Slot      | `0x00_0000–0x00_1FFF`   | ✅ Yes  | Up to 32 slots × 256 bytes     |
| Extended  | `0x00_2000–0x00_FFFF`   | ✅ Yes  | Single port (e.g., BRAM)       |
| Large     | `0x01_0000–0xFF_FFFF`   | ❌ Stub | Returns `0xDEADBEEF` (Phase 3) |

**Slot addressing** (within `0x00_0000–0x00_1FFF`):
- `addr[12:8]` — slot number (0–31)
- `addr[7:0]` — local address within the slot (256 bytes per slot)
- Slot 0: `0x0000–0x00FF`, Slot 1: `0x0100–0x01FF`, …, Slot N: `0xN*0x100–0xN*0x100+0xFF`

**Parameters:**

| Parameter        | Default | Description                         |
|------------------|---------|-------------------------------------|
| `NUM_SLOTS`      | `32`    | Number of slot peripherals (1–32)   |
| `SLOT_ADDR_BITS` | `8`     | Address bits per slot (256 bytes)   |

**Out-of-range behavior:** Accesses to a slot number ≥ `NUM_SLOTS`, or to the large tier stub, immediately ACK with data `0xDEADBEEF`.

**Expanding slot count:** Change `NUM_SLOTS` parameter only — the slot wiring is fully generated (`genvar` loop). Also update the ESP32 firmware constant `PWB_NUM_SLOTS` to match.

**Extended tier — single port, no sub-decode:** The extended tier (`0x2000–0xFFFF`) routes to a single `ext_*` port. For a single slave use `EXT_CONNECT`. For multiple slaves use `pwb_wb_ext_router` via the `NUM_EXT_SLOTS` + `EXT_SLOT_CONNECT` pattern.

---

### `pwb_wb_ext_router.v` — Extended Tier Router

Routes Wishbone transactions within the extended address space (0x2000– 0xFFFF) to one of **N sub-peripherals** based on programmed `BASE_ADDRS` and `SIZES`.

**Parameters:**

| Parameter       | Default         | Description                                                  |
|-----------------|-----------------|--------------------------------------------------------------|
| `NUM_EXT_SLOTS` | `4`             | Number of extended-tier sub-slots (max 16)                   |
| `BASE_ADDRS`    | see below       | Flattened 16-bit base address per slot; slot 0 at bits [15:0]|
| `SIZES`         | see below       | Flattened 16-bit range size per slot; slot 0 at bits [15:0]  |

**Address decode:**
- Slot N matches when `wb_adr_i >= BASE_ADDRS[N*16+:16]` AND `wb_adr_i < BASE_ADDRS[N*16+:16] + SIZES[N*16+:16]`
- Lowest slot index wins when ranges overlap (slot 0 = highest priority)
- Raw address passed to each slot (no BASE subtraction — peripheral handles it)
- Unmatched address returns `ACK=1, dat_o=32'hDEADBEEF`

**Auto-instantiated via `pwb_bus_wires.vh`** when `` `define NUM_EXT_SLOTS `` is set — users do not instantiate this module directly. See `pwb_bus_wires.vh` docs below.

**Direct instantiation (advanced):**
```verilog
pwb_wb_ext_router #(
    .NUM_EXT_SLOTS (2),
    .BASE_ADDRS    ({16'hB000, 16'h2000}),   // slot 1 at [31:16], slot 0 at [15:0]
    .SIZES         ({16'h1000, 16'h9000})
) u_ext_router (
    .clk(clk), .rst(rst),
    .wb_adr_i(ext_adr),     .wb_dat_i(ext_dat_m2s),
    .wb_dat_o(ext_dat_s2m), .wb_sel_i(ext_sel),
    .wb_we_i(ext_we),       .wb_cyc_i(ext_cyc),
    .wb_stb_i(ext_stb),     .wb_ack_o(ext_ack),
    .slot_adr_o(ext_slot_adr),   .slot_dat_o(ext_slot_dat_m2s),
    .slot_dat_i(ext_slot_dat_s2m), .slot_sel_o(ext_slot_sel),
    .slot_we_o(ext_slot_we), .slot_cyc_o(ext_slot_cyc),
    .slot_stb_o(ext_slot_stb), .slot_ack_i(ext_slot_ack)
);
```

---

### `pwb_bus_wires.vh` — Wire declarations and helper macros

Include this file inside your top module body after defining `wire clk` and `localparam NUM_SLOTS`. It declares all the flattened slot and extended tier wires, and provides four macros:

**Prerequisites in your module:**
```verilog
wire clk;               // your system clock wire
localparam NUM_SLOTS = 8;
wire rst;               // driven by pwb_wb_system.rst_o (or your own generator)
`include "pwb_bus_wires.vh"
```

**Macros:**

| Macro                  | Usage                                                | Description                                              |
|------------------------|------------------------------------------------------|----------------------------------------------------------|
| `` `SLOT_CONNECT``     | `` `SLOT_CONNECT(N, MODULE #(.P(V)), INST))``         | Wire slot N to a peripheral (close with `));`)           |
| `` `EXT_CONNECT``      | `` `EXT_CONNECT(MODULE #(.P(V)), INST))``            | Wire extended tier to single peripheral (no router)      |
| `` `EXT_SLOT_CONNECT`` | `` `EXT_SLOT_CONNECT(N, MODULE #(.P(V)), INST))``    | Wire ext router slot N to a peripheral (requires router) |
| `` `PWB_SLOT_PORTS``   | (in port list)                                       | Connect all slot wires to `pwb_wb_system`                |
| `` `PWB_EXT_PORTS``    | (in port list)                                       | Connect extended tier wires to `pwb_wb_system`           |

Both `SLOT_CONNECT` and `EXT_CONNECT` leave the module port list **open** — the caller closes with `);`. This lets you add extra I/O ports for peripherals that have them.

**`SLOT_CONNECT` usage:**
```verilog
// Simple peripheral — close immediately
`SLOT_CONNECT(0, wb_register_block #(.ADDR_WIDTH(4), .DATA_WIDTH(8)), slot0_reg));

// Peripheral with extra I/O — add ports before closing
`SLOT_CONNECT(1, wb_simple_rgb_led, slot1_led),
    .led_out(rgb_data)
);

// Swap a peripheral: just change the module name, instance name
`SLOT_CONNECT(2, wb_new_device, slot2_x));
```

**`EXT_SLOT_CONNECT` usage (multi-slot router) — add `define NUM_EXT_SLOTS` first:**
```verilog
// BEFORE `include pwb_bus_wires.vh:
`define    NUM_EXT_SLOTS
localparam NUM_EXT_SLOTS = 2;
localparam [NUM_EXT_SLOTS*16-1:0] EXT_BASE_ADDRS = {16'hB000, 16'h2000};
localparam [NUM_EXT_SLOTS*16-1:0] EXT_SIZES      = {16'h1000, 16'h9000};
// `include "pwb_bus_wires.vh" auto-instantiates the router

// Then wire each sub-peripheral:
`EXT_SLOT_CONNECT(0, papilio_hdmi_wb #(.BASE_ADDR(16'h2000)), u_hdmi),
    .O_tmds_clk_p(O_tmds_clk_p)
);
`EXT_SLOT_CONNECT(1, wb_bram #(.ADDR_WIDTH(10)), u_bram));

// PWB_EXT_PORTS in pwb_wb_system is UNCHANGED
```

**`EXT_CONNECT` usage (one peripheral only):**
```verilog
// Simple — no extra ports
`EXT_CONNECT(wb_bram #(.ADDR_WIDTH(10), .DATA_WIDTH(32)), ext_bram));

// With extra I/O
`EXT_CONNECT(my_sdram_ctrl, ext_sdram),
    .sdram_clk(sdram_clk),
    .sdram_dq(sdram_dq)
);
// Also add ,`PWB_EXT_PORTS to the pwb_wb_system port list
```

---

### `pwb_spi_wb_bridge.v` — Multi-width SPI-to-Wishbone bridge

See the main library `AI_SKILL.md` for full protocol and port documentation.

---

### `pwb_wb_system` vs `pwb_wb_interconnect` — Which to use?

| Situation                                    | Use                    |
|----------------------------------------------|------------------------|
| Building a new Papilio project               | `pwb_wb_system`        |
| Using `papilio_project_template`             | `pwb_wb_system`        |
| Custom reset generator or multi-clock design | `pwb_wb_interconnect`  |
| Testing the interconnect in isolation        | `pwb_wb_interconnect`  |
| Integrating into an existing top-level       | `pwb_wb_interconnect`  |

---

### Legacy Modules (deprecated)

- `simple_spi_wb_bridge_debug.v` — 8-bit only bridge with UART debug output
- `wb_address_decoder.v` — Flat address decoder (superseded by interconnect)
- `uart_tx.v` — Minimal UART transmitter (used by legacy bridge)

---

## Build

Build using the Gowin vendor flow (Gowin IDE or `gw_sh.exe build.tcl`). Include:
- `pwb_wb_system.v`
- `pwb_wb_interconnect.v`
- `pwb_spi_wb_bridge.v`
- `pwb_bus_wires.vh`
- `pwb_wb_ext_router.v` (include when using multi-slot extended tier)
- `libs/papilio_spi_slave/gateware/spi_slave.v` (from its library location — do not copy)
