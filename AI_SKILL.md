# papilio_wishbone_bus - AI Skill

This file provides machine-consumable details about the `papilio_wishbone_bus` library for AI assistants.

## Purpose
Multi-width SPI-based Wishbone bus master for ESP32-to-FPGA communication. Supports 8/16/24/32-bit data transfers with single-word and burst modes.

## Protocol Overview

### Command Byte Encoding
- **Bit 0**: 0=read, 1=write
- **Bits 2:1**: 00=8-bit, 01=16-bit, 10=24-bit, 11=32-bit
- **Bit 3**: 0=single-word, 1=burst mode
- **Bits 7:4**: Reserved (0)

### Command Reference Table

#### Single Word Commands (Bit 3 = 0)

| Width  | Read CMD | Write CMD | Format                                    |
|--------|----------|-----------|-------------------------------------------|
| 8-bit  | 0x00     | 0x01      | [CMD][ADDR_H][ADDR_L][DATA]               |
| 16-bit | 0x02     | 0x03      | [CMD][ADDR_H][ADDR_L][DATA_H][DATA_L]     |
| 24-bit | 0x04     | 0x05      | [CMD][ADDR_H][ADDR_L][D2][D1][D0]         |
| 32-bit | 0x06     | 0x07      | [CMD][ADDR_H][ADDR_L][D3][D2][D1][D0]     |

#### Burst Commands (Bit 3 = 1)

Burst commands include a 16-bit COUNT field (big-endian) after the address.
COUNT specifies the number of words (1-65535).

| Width  | Read CMD | Write CMD | Format                                              |
|--------|----------|-----------|-----------------------------------------------------|
| 8-bit  | 0x08     | 0x09      | [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L] + N×8-bit   |
| 16-bit | 0x0A     | 0x0B      | [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L] + N×16-bit  |
| 24-bit | 0x0C     | 0x0D      | [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L] + N×24-bit  |
| 32-bit | 0x0E     | 0x0F      | [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L] + N×32-bit  |

**Burst Addressing**: Address auto-increments by data width after each word:
- 8-bit: address += 1
- 16-bit: address += 2
- 24-bit: address += 3
- 32-bit: address += 4

## Address Map (Slot Tier)

The slot tier covers `0x0000–0x1FFF`. Each slot gets 256 bytes.

| Slot | Address Range       | Firmware constant example    |
|------|---------------------|------------------------------|
| 0    | `0x0000–0x00FF`     | `PWB_SLOT0_BASE = 0x0000`    |
| 1    | `0x0100–0x01FF`     | `PWB_SLOT1_BASE = 0x0100`    |
| 2    | `0x0200–0x02FF`     | `PWB_SLOT2_BASE = 0x0200`    |
| …    | …                   | …                            |
| N    | `N×0x100–N×0x100+0xFF` | `PWB_SLOTN_BASE = N*0x100` |

## Address Map (Extended Tier)

The extended tier covers `0x2000–0xFFFF`. Default: single slave via `ext_*` port (use `EXT_CONNECT`). With `pwb_wb_ext_router`: up to 16 sub-slots, each with configurable base + size.

**Dev project extended slot map (`top.v`):**

| Ext Slot | Address Range     | Size    | Peripheral           |
|----------|-------------------|---------|----------------------|
| 0        | `0x2000–0xAFFF`  | ~36 KB  | `papilio_hdmi_wb`    |
| 1        | `0xB000–0xBFFF`  | 4 KB    | `wb_bram`            |

Large tier: `0x010000–0xFFFFFF` — **stubbed**, returns `0xDEADBEEF` (Phase 3, not yet implemented).

## Gateware Modules

### pwb_wb_system.v
High-level system wrapper — **recommended for end users**. Encapsulates reset generator, SPI bridge, and interconnect. Parameters: `NUM_SLOTS` (default 8). Exposes slot and extended tier ports; use with `pwb_bus_wires.vh` macros.

### pwb_wb_ext_router.v
Optional extended-tier router. Auto-instantiated by `pwb_bus_wires.vh` when `` `define NUM_EXT_SLOTS `` is set. Routes ext master bus to one of N sub-slots based on BASE_ADDRS/SIZES flattened vectors. Priority-first (slot 0 wins on overlap). Unmatched returns `0xDEADBEEF`. Passes raw address to peripheral (no subtraction). See gateware/README.md for parameter and usage details.

### pwb_wb_interconnect.v
Three-tier address decoder/router. Use directly for advanced integrations. Slot tier active (0x00_0000–0x00_1FFF), extended tier active (0x00_2000–0x00_FFFF), large tier stubbed. Address bits `[12:8]` select the slot.

### pwb_bus_wires.vh
Include file that declares all slot/extended wire arrays and macros:
- `` `SLOT_CONNECT(N, MODULE, INST)`` — wire slot N to peripheral
- `` `EXT_CONNECT(MODULE, INST)`` — wire extended tier to single peripheral (no router)
- `` `EXT_SLOT_CONNECT(N, MODULE, INST)`` — wire ext router slot N to peripheral (requires `define NUM_EXT_SLOTS)
- `` `PWB_SLOT_PORTS`` — port list for `pwb_wb_system` slot connections
- `` `PWB_EXT_PORTS`` — port list for `pwb_wb_system` extended tier connection (unchanged when using router)

**Multi-slot router pattern:**
```verilog
`define    NUM_EXT_SLOTS
localparam NUM_EXT_SLOTS = 2;
localparam [NUM_EXT_SLOTS*16-1:0] EXT_BASE_ADDRS = {16'hB000, 16'h2000};
localparam [NUM_EXT_SLOTS*16-1:0] EXT_SIZES      = {16'h1000, 16'h9000};
wire rst;
`include "pwb_bus_wires.vh"   // auto-instantiates pwb_wb_ext_router

// Then use EXT_SLOT_CONNECT for each sub-peripheral
`EXT_SLOT_CONNECT(0, my_hdmi #(.BASE_ADDR(16'h2000)), u_hdmi));
`EXT_SLOT_CONNECT(1, wb_bram #(.ADDR_WIDTH(10)), u_bram));
```

### pwb_spi_wb_bridge.v
Multi-width SPI-to-Wishbone bridge with FIFO buffering for burst transfers.

**Parameters:**
- `FIFO_DEPTH` (default 64): FIFO depth in 32-bit words
- `ALMOST_FULL_THRESHOLD` (default 4): Words remaining before almost_full flag
- `ALMOST_EMPTY_THRESHOLD` (default 4): Words remaining before almost_empty flag

**Ports:**
```verilog
module pwb_spi_wb_bridge #(
    parameter FIFO_DEPTH = 64,
    parameter ALMOST_FULL_THRESHOLD = 4,
    parameter ALMOST_EMPTY_THRESHOLD = 4
) (
    input wire clk,
    input wire rst,
    
    // SPI Interface
    input wire spi_sclk,
    input wire spi_mosi,
    output wire spi_miso,
    input wire spi_cs_n,
    
    // Wishbone Master Interface (16-bit address, 32-bit data)
    output reg [15:0] wb_adr_o,
    output reg [31:0] wb_dat_o,
    input wire [31:0] wb_dat_i,
    output reg [3:0] wb_sel_o,      // Byte select for narrow transfers
    output reg wb_we_o,
    output reg wb_cyc_o,
    output reg wb_stb_o,
    input wire wb_ack_i,
    
    // FIFO Status Flags (for firmware monitoring)
    output wire fifo_rx_almost_full,
    output wire fifo_tx_almost_empty
);
```

**Features:**
- Single-word and burst transfer modes
- 8/16/24/32-bit data widths
- FIFO buffering for high-throughput burst transfers
- Automatic address increment in burst mode
- Byte lane selection via wb_sel_o

**State Machine:**
1. IDLE: Waiting for CS assertion
2. RECEIVE_CMD: Capture command byte
3. RECEIVE_ADDR: Capture 16-bit address
4. RECEIVE_COUNT: Capture burst count (burst mode only)
5. WISHBONE_CYCLE: Execute Wishbone transaction
6. SEND_DATA: Transmit read data (read operations)

**Timing:**
- SPI Mode 1: CPOL=0, CPHA=1
- Wishbone transactions can be decoupled via FIFOs in burst mode
- Supports back-to-back burst transfers with auto-increment

**Dependencies:**
- Requires `fifo_sync.v` from `papilio_spi_slave` library

### pwb_register_block.v
Parameterized Wishbone register block for testing.

**Parameters:**
- `ADDR_WIDTH` (default 4): Register address bits
- `DATA_WIDTH` (default 8): Register data width
- `RESET_VALUE` (default 0): Initial register value

**Ports:**
- Standard Wishbone classic slave interface
- Synchronous reset clears all registers to RESET_VALUE

### Legacy Modules (deprecated)
- `simple_spi_wb_bridge.v`: 8-bit only bridge (use `pwb_spi_wb_bridge` instead)
- `wb_address_decoder*.v`: Flat address decoders (superseded by `pwb_wb_interconnect`)

## Firmware API

### Class: WishboneSPI

**Single Word Operations:**
```cpp
// 8-bit transfers
void wishboneWrite8(uint16_t address, uint8_t data);
uint8_t wishboneRead8(uint16_t address);

// 16-bit transfers
void wishboneWrite16(uint16_t address, uint16_t data);
uint16_t wishboneRead16(uint16_t address);

// 24-bit transfers
void wishboneWrite24(uint16_t address, uint32_t data24);
uint32_t wishboneRead24(uint16_t address);

// 32-bit transfers
void wishboneWrite32(uint16_t address, uint32_t data);
uint32_t wishboneRead32(uint16_t address);
```

**Burst Operations:**
```cpp
// 8-bit burst
void wishboneWriteBurst8(uint16_t startAddr, const uint8_t* data, uint16_t count);
void wishboneReadBurst8(uint16_t startAddr, uint8_t* data, uint16_t count);

// 16-bit burst
void wishboneWriteBurst16(uint16_t startAddr, const uint16_t* data, uint16_t count);
void wishboneReadBurst16(uint16_t startAddr, uint16_t* data, uint16_t count);

// 24-bit burst
void wishboneWriteBurst24(uint16_t startAddr, const uint32_t* data, uint16_t count);
void wishboneReadBurst24(uint16_t startAddr, uint32_t* data, uint16_t count);

// 32-bit burst
void wishboneWriteBurst32(uint16_t startAddr, const uint32_t* data, uint16_t count);
void wishboneReadBurst32(uint16_t startAddr, uint32_t* data, uint16_t count);
```

**Legacy Aliases:**
```cpp
void wishboneWrite(uint32_t address, uint32_t data);   // Calls wishboneWrite32
uint32_t wishboneRead(uint32_t address);                // Calls wishboneRead32
```

## Pin Assignments

No fixed pin assignments - configure in top-level constraints:
- `spi_sclk`: SPI clock to FPGA
- `spi_cs_n`: Chip select (active low)
- `spi_mosi`: Master Out (ESP32), Slave In (FPGA)
- `spi_miso`: Master In (ESP32), Slave Out (FPGA)

## Testing

### Simulation Tests
Location: `tests/sim/`
- `tb_pwb_wb_interconnect.v`: Interconnect routing (24 tests — slot routing, isolation, OOB, extended tier, large stub)
- `tb_pwb_multi_width.v`: Multi-width bridge validation
- Generates VCD files for waveform analysis

Run simulations:
```powershell
cd tests/sim
python run_all_sims.py
```

### Hardware Tests
Location: `tests/hw/`
- PlatformIO-based ESP32 test harness
- Validates all transfer widths and burst modes
- Tests with `pwb_register_block` targets

Run hardware tests:
```powershell
cd tests/hw
pio test
```

### Top-Level Test Runner
```powershell
python run_all_tests.py  # Runs both sim and hw tests
```

## Common Operations

### Instantiate Bridge in Top Module
```verilog
pwb_spi_wb_bridge #(
    .FIFO_DEPTH(64),
    .ALMOST_FULL_THRESHOLD(4),
    .ALMOST_EMPTY_THRESHOLD(4)
) bridge_inst (
    .clk(sys_clk),
    .rst(reset),
    
    // SPI pins (connect to top-level ports)
    .spi_sclk(spi_sclk),
    .spi_cs_n(spi_cs_n),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    
    // Wishbone master
    .wb_adr_o(wb_adr),
    .wb_dat_o(wb_dat_m2s),
    .wb_dat_i(wb_dat_s2m),
    .wb_sel_o(wb_sel),
    .wb_we_o(wb_we),
    .wb_cyc_o(wb_cyc),
    .wb_stb_o(wb_stb),
    .wb_ack_i(wb_ack),
    
    // FIFO status (optional monitoring)
    .fifo_rx_almost_full(),
    .fifo_tx_almost_empty()
);
```

### ESP32 Basic Usage
```cpp
#include "WishboneSPI.h"

WishboneSPI wb;

void setup() {
    wb.begin();
    
    // 8-bit register access
    wb.wishboneWrite8(0x0000, 0x42);
    uint8_t val = wb.wishboneRead8(0x0000);
    
    // 32-bit burst write
    uint32_t data[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    wb.wishboneWriteBurst32(0x1000, data, 10);
}
```

### Connect Slot Peripherals
Macros leave the port list open — close with `);`. Extra I/O goes before the close.
```verilog
// Simple peripheral
`SLOT_CONNECT(0, wb_register_block #(.ADDR_WIDTH(4), .DATA_WIDTH(8)), slot0_reg));

// Peripheral with extra I/O
`SLOT_CONNECT(1, wb_simple_rgb_led, slot1_led),
    .led_out(rgb_data)
);
```

### Connect Extended Tier Peripheral (BRAM)
```verilog
// Uncomment ,`PWB_EXT_PORTS in pwb_wb_system port list, then:
`EXT_CONNECT(wb_bram #(.ADDR_WIDTH(10), .DATA_WIDTH(32)), ext_bram));
// Access from ESP32: wb.wishboneRead32(0x2000);
```

### Swap a Peripheral
```verilog
// Change the module name and instance name — no other wiring needed:
// Before: `SLOT_CONNECT(2, wb_old_device, slot2_old));
`SLOT_CONNECT(2, wb_new_device, slot2_new));
```

## Notes for AI Assistants

### When Modifying This Library
- Maintain Papilio Library Standards compliance
- Update both firmware API and gateware in sync
- Extend protocol using reserved command bits (0x10-0xFF)
- Add testbenches for new features
- Document register maps completely

### Troubleshooting Patterns
- **No Wishbone ACK**: Check slave response timing, verify wb_cyc_o/wb_stb_o signals
- **Burst address errors**: Verify auto-increment matches data width (1/2/3/4 bytes)
- **Data corruption**: Check SPI clock speed ≤ 4 MHz, system clock ≥ 27 MHz
- **Wrong data width**: Bridge reads `wb_data_width` output, ensure slave handles correctly

### Adding Features
- **New transfer widths**: Update command encoding (bits 2:1), add API methods
- **Enhanced burst modes**: Use reserved command bits (0x10+)
- **Status registers**: Add to bridge state machine, expose via Wishbone
- **DMA support**: Integrate with `papilio_spi_slave` FIFO variants

### Integration with Other Libraries
- **papilio_spi_slave**: Uses same SPI timing and CDC patterns
- **papilio_wb_bram**: Connect as Wishbone slave for memory testing
- **papilio_wishbone_register**: Connect as Wishbone slave for simple register access
- **papilio_os**: Optional CLI integration (not yet implemented)

## Example Projects

See `examples/basic_wishbone_example/` for complete ESP32 + FPGA project demonstrating:
- Multi-width register access
- Burst transfers
- Top module instantiation
- Constraint file setup

## Repository
https://github.com/Papilio-Labs/papilio_wishbone_bus

## Dependencies
- Uses `papilio_spi_slave` design patterns (not a formal library dependency)
- Testing uses `papilio_dev_tools` infrastructure
