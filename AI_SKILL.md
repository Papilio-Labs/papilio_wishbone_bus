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

## Gateware Modules

### pwb_spi_wb_bridge.v
Multi-width SPI-to-Wishbone bridge with dynamic width handling.

**Parameters:**
- `ADDR_WIDTH` (default 16): Wishbone address width
- `MAX_DATA_WIDTH` (default 32): Maximum data width (must be ≥ all transfer widths)

**Ports:**
```verilog
module pwb_spi_wb_bridge #(
    parameter ADDR_WIDTH = 16,
    parameter MAX_DATA_WIDTH = 32
) (
    input wire clk,
    input wire rst,
    
    // SPI Interface
    input wire spi_sck,
    input wire spi_cs_n,
    input wire spi_mosi,
    output wire spi_miso,
    
    // Wishbone Master Interface
    output reg [ADDR_WIDTH-1:0] wb_adr_o,
    output reg [MAX_DATA_WIDTH-1:0] wb_dat_o,
    input wire [MAX_DATA_WIDTH-1:0] wb_dat_i,
    output reg wb_we_o,
    output reg wb_cyc_o,
    output reg wb_stb_o,
    input wire wb_ack_i,
    output reg [2:0] wb_data_width  // 0=8-bit, 1=16-bit, 2=24-bit, 3=32-bit
);
```

**State Machine:**
1. IDLE: Waiting for CS assertion
2. RECEIVE_CMD: Capture command byte
3. RECEIVE_ADDR: Capture 16-bit address
4. RECEIVE_COUNT: Capture burst count (burst mode only)
5. WISHBONE_CYCLE: Execute Wishbone transaction
6. SEND_DATA: Transmit read data (read operations)

**Timing:**
- Uses `papilio_spi_slave` core patterns
- Wishbone transactions complete during CS active time
- Supports back-to-back burst transfers with auto-increment

### pwb_register_block.v
Parameterized Wishbone register block for testing.

**Parameters:**
- `ADDR_WIDTH` (default 4): Register address bits
- `DATA_WIDTH` (default 8): Register data width
- `RESET_VALUE` (default 0): Initial register value

**Ports:**
- Standard Wishbone classic slave interface
- Synchronous reset clears all registers to RESET_VALUE

### Legacy Modules
- `simple_spi_wb_bridge.v`: 8-bit only bridge (deprecated, use pwb_spi_wb_bridge)
- `wb_address_decoder*.v`: Address fanout helpers for multiple Wishbone slaves

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
- `spi_sck`: SPI clock to FPGA
- `spi_cs_n`: Chip select (active low)
- `spi_mosi`: Master Out (ESP32), Slave In (FPGA)
- `spi_miso`: Master In (ESP32), Slave Out (FPGA)

## Testing

### Simulation Tests
Location: `tests/sim/`
- `tb_pwb_multi_width.v`: Multi-width bridge validation
- Generates VCD files for waveform analysis
- Tests all command types (0x00-0x0F)

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
    .ADDR_WIDTH(16),
    .MAX_DATA_WIDTH(32)
) bridge_inst (
    .clk(sys_clk),
    .rst(reset),
    
    // SPI pins (connect to top-level ports)
    .spi_sck(spi_sck),
    .spi_cs_n(spi_cs_n),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    
    // Wishbone master
    .wb_adr_o(wb_adr),
    .wb_dat_o(wb_dat_m2s),
    .wb_dat_i(wb_dat_s2m),
    .wb_we_o(wb_we),
    .wb_cyc_o(wb_cyc),
    .wb_stb_o(wb_stb),
    .wb_ack_i(wb_ack),
    .wb_data_width(wb_width)
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

### Connect Multiple Wishbone Slaves
Use `wb_address_decoder` modules to fan out to multiple slaves:
```verilog
// Slave 0: 0x0000-0x0FFF (registers)
// Slave 1: 0x1000-0x1FFF (BRAM)
// Add address decoding logic or use decoder modules
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
