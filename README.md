# Papilio Wishbone Bus — `libs/papilio_wishbone_bus`

This library provides a multi-width SPI-based master for talking to an FPGA-side Wishbone bridge, using the validated `papilio_spi_slave` core. Supports 8-bit, 16-bit, and 32-bit data transfers.

## Files
- `src/WishboneSPI.h` — Public header with API and protocol constants.
- `src/WishboneSPI.cpp` — PapilioSPI-based implementation (SPI Mode 1, MSB-first, <=4 MHz).
- `gateware/pwb_spi_wb_bridge.v` — Multi-width SPI-to-Wishbone bridge (dynamically handles 8/16/32-bit based on command byte).
- `gateware/pwb_register_block.v` — Parameterized Wishbone register block for testing.
- `gateware/simple_spi_wb_bridge.v` — Legacy 8-bit only bridge (deprecated).
- `gateware/wb_address_decoder*.v` — Wishbone address fanout helpers.
- `tests/sim/` — Simulation testbenches with VCD for multi-width bridge testing.
- `tests/hw/` — ESP32 hardware regression tests.
- `run_all_tests.py` — Top-level test runner (uses papilio_dev_tools infrastructure).

## API

### 8-bit transfers (CMD 0x00/0x01)
```cpp
void wishboneWrite8(uint16_t address, uint8_t data);
uint8_t wishboneRead8(uint16_t address);
```

### 16-bit transfers (CMD 0x02/0x03)
```cpp
void wishboneWrite16(uint16_t address, uint16_t data);
uint16_t wishboneRead16(uint16_t address);
```

### 24-bit transfers (CMD 0x04/0x05)
```cpp
void wishboneWrite24(uint16_t address, uint32_t data24);
uint32_t wishboneRead24(uint16_t address);
```

### 32-bit transfers (CMD 0x06/0x07)
```cpp
void wishboneWrite32(uint16_t address, uint32_t data);
uint32_t wishboneRead32(uint16_t address);

// Legacy aliases
void wishboneWrite(uint32_t address, uint32_t data);
uint32_t wishboneRead(uint32_t address);
```

### Burst Mode (Sequential Transfers)
Burst commands allow reading/writing multiple words with auto-incrementing address.
The `count` parameter is 16-bit, allowing up to 65,535 words per burst.

```cpp
// 8-bit Burst
void wishboneWriteBurst8(uint16_t startAddr, const uint8_t* data, uint16_t count);
void wishboneReadBurst8(uint16_t startAddr, uint8_t* data, uint16_t count);

// 16-bit Burst
void wishboneWriteBurst16(uint16_t startAddr, const uint16_t* data, uint16_t count);
void wishboneReadBurst16(uint16_t startAddr, uint16_t* data, uint16_t count);

// 24-bit Burst
void wishboneWriteBurst24(uint16_t startAddr, const uint32_t* data, uint16_t count);
void wishboneReadBurst24(uint16_t startAddr, uint32_t* data, uint16_t count);

// 32-bit Burst
void wishboneWriteBurst32(uint16_t startAddr, const uint32_t* data, uint16_t count);
void wishboneReadBurst32(uint16_t startAddr, uint32_t* data, uint16_t count);
```

## Protocol

Command byte encoding:
- Bit 0: 0=read, 1=write
- Bits 2:1: 00=8-bit, 01=16-bit, 10=24-bit, 11=32-bit
- Bit 3: 0=single-word, 1=burst mode

### Single Word Commands (Bit 3 = 0)

| Width | Read CMD | Write CMD | Format |
|-------|----------|-----------|--------|
| 8-bit | 0x00 | 0x01 | [CMD][ADDR_H][ADDR_L] + 8-bit data |
| 16-bit | 0x02 | 0x03 | [CMD][ADDR_H][ADDR_L] + 16-bit data |
| 24-bit | 0x04 | 0x05 | [CMD][ADDR_H][ADDR_L] + 24-bit data |
| 32-bit | 0x06 | 0x07 | [CMD][ADDR_H][ADDR_L] + 32-bit data |

### Burst Commands (Bit 3 = 1)

Burst commands include a 16-bit `COUNT` field (big-endian, MSB first) after the address.
COUNT specifies the number of words (1-65535).

| Width | Read CMD | Write CMD | Format |
|-------|----------|-----------|--------|
| 8-bit | 0x08 | 0x09 | [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L] + N * 8-bit data |
| 16-bit | 0x0A | 0x0B | [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L] + N * 16-bit data |
| 24-bit | 0x0C | 0x0D | [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L] + N * 24-bit data |
| 32-bit | 0x0E | 0x0F | [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L] + N * 32-bit data |

## Usage

```cpp
#include "WishboneSPI.h"

SPIClass fpgaSPI;

void setup() {
  fpgaSPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);
  wishboneInit(&fpgaSPI, SPI_CS);
  
  // Single Access
  wishboneWrite32(0x0200, 0xDEADBEEF);
  
  // Burst Write (4 words)
  uint32_t data[] = {0x11111111, 0x22222222, 0x33333333, 0x44444444};
  wishboneWriteBurst32(0x0210, data, 4);
}
```

## Testing

### Running All Tests
```bash
# Run simulation and hardware tests
python run_all_tests.py

# Run simulation tests only
python run_all_tests.py --sim-only

# Run hardware tests only (requires connected hardware)
python run_all_tests.py --hw-only
```

### Simulation Tests
Testbenches in `tests/sim/` exercise all three data widths (8/16/32-bit):
- 8-bit registers at 0x0000
- 16-bit registers at 0x0100  
- 32-bit registers at 0x0200

Run simulations with:
```bash
cd tests/sim
python run_all_sims.py
```

Uses cross-platform simulation infrastructure from `papilio_dev_tools`.

### Hardware Tests
The hardware test suite in `tests/hw/` validates all functionality against real FPGA hardware with `top_pwb_multi_width.v`.

See `tests/hw/README.md` for hardware test details.
