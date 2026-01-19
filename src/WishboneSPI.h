#ifndef WISHBONESPI_H
#define WISHBONESPI_H

#include <Arduino.h>
#include <PapilioSPI.h>

// Initialize the Wishbone SPI helper with a PapilioSPI-compatible SPIClass instance and CS pin
// This waits for the FPGA bootloader to complete before returning
void wishboneInit(SPIClass* spi, int cs_pin);

// Wait for FPGA to be ready (called automatically by wishboneInit)
// Returns true if FPGA responded, false if timeout
// bootloaderDelayMs: time to wait for bootloader (default 4000ms for 3s bootloader + margin)
// timeoutMs: additional polling timeout (default 5000ms)
bool wishboneWaitForFPGA(unsigned long bootloaderDelayMs = 4000, unsigned long timeoutMs = 5000);

// Check if FPGA is responding by reading video mode register
bool wishboneIsReady();

// 8-bit data access (CMD 0x00/0x01)
void wishboneWrite8(uint16_t address, uint8_t data);
uint8_t wishboneRead8(uint16_t address);

// 16-bit data access (CMD 0x04/0x05)
void wishboneWrite16(uint16_t address, uint16_t data);
uint16_t wishboneRead16(uint16_t address);

// 24-bit data access (CMD 0x08/0x09) - for RGB888 video
void wishboneWrite24(uint16_t address, uint32_t data24);
uint32_t wishboneRead24(uint16_t address);

// 32-bit data access (CMD 0x0C/0x0D)
void wishboneWrite32(uint16_t address, uint32_t data);
uint32_t wishboneRead32(uint16_t address);

// Legacy 32-bit access (aliases to native 32-bit functions)
void wishboneWrite(uint32_t address, uint32_t data);
uint32_t wishboneRead(uint32_t address);

// ============================================================================
// Burst mode API - sequential multi-word transfers with auto-increment
// COUNT is now 16-bit (1-65535 words per burst)
// ============================================================================

// Burst write: write 'count' sequential words starting at 'startAddr'
// Address auto-increments after each word
void wishboneWriteBurst8(uint16_t startAddr, const uint8_t* data, uint16_t count);
void wishboneWriteBurst16(uint16_t startAddr, const uint16_t* data, uint16_t count);
void wishboneWriteBurst24(uint16_t startAddr, const uint32_t* data, uint16_t count);
void wishboneWriteBurst32(uint16_t startAddr, const uint32_t* data, uint16_t count);

// Burst read: read 'count' sequential words starting at 'startAddr'
// Address auto-increments after each word
void wishboneReadBurst8(uint16_t startAddr, uint8_t* data, uint16_t count);
void wishboneReadBurst16(uint16_t startAddr, uint16_t* data, uint16_t count);
void wishboneReadBurst24(uint16_t startAddr, uint32_t* data, uint16_t count);
void wishboneReadBurst32(uint16_t startAddr, uint32_t* data, uint16_t count);

// ============================================================================
// DMA mode API - zero-CPU burst transfers using ESP32 GDMA
// ============================================================================
// NOTE: DMA mode is currently STUB/NON-FUNCTIONAL
// The API exists but falls back to CPU transfer.
// Full GDMA+SPI driver integration is required for true hardware DMA.
// ============================================================================

// DMA burst write callback - called when DMA transfer completes
// Parameters: user_data (passed to wishboneWriteBurstDMA), success (true/false)
typedef void (*WishboneDMACallback)(void* user_data, bool success);

// DMA burst write: write 'count' sequential 32-bit words using GDMA
// Buffer must remain valid until callback is called
// Returns false if DMA setup fails, true if transfer started
// count: number of 32-bit words (1-65535)
// callback: optional completion callback (can be nullptr for polling mode)
// user_data: optional user data passed to callback
bool wishboneWriteBurstDMA32(uint16_t startAddr, const uint32_t* data, uint16_t count,
                              WishboneDMACallback callback = nullptr, void* user_data = nullptr);

// Check if DMA transfer is in progress
bool wishboneDMABusy();

// Wait for DMA transfer to complete (blocking)
// Returns true if successful, false if timeout
bool wishboneDMAWait(unsigned long timeoutMs = 5000);

// ============================================================================
// Wishbone SPI protocol command values (multi-width bridge)
// ============================================================================
// Command byte encoding:
//   Bit 0:   0=read, 1=write
//   Bits 2:1: 00=8-bit, 01=16-bit, 10=24-bit, 11=32-bit
//   Bit 3:   0=single-word, 1=burst mode
//   Bits 7:4: Reserved (must be 0)

// Single-word commands (bit 3 = 0)
#define CMD_READ_8   0x00
#define CMD_WRITE_8  0x01
#define CMD_READ_16  0x02
#define CMD_WRITE_16 0x03
#define CMD_READ_24  0x04
#define CMD_WRITE_24 0x05
#define CMD_READ_32  0x06
#define CMD_WRITE_32 0x07

// Burst commands (bit 3 = 1)
#define CMD_BURST_READ_8   0x08
#define CMD_BURST_WRITE_8  0x09
#define CMD_BURST_READ_16  0x0A
#define CMD_BURST_WRITE_16 0x0B
#define CMD_BURST_READ_24  0x0C
#define CMD_BURST_WRITE_24 0x0D
#define CMD_BURST_READ_32  0x0E
#define CMD_BURST_WRITE_32 0x0F

// Backward compatibility
#define CMD_READ  CMD_READ_8
#define CMD_WRITE CMD_WRITE_8

#endif // WISHBONESPI_H
