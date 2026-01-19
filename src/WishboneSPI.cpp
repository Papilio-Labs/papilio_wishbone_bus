#include "WishboneSPI.h"

static PapilioSPI _wbSpi;
static bool _wbInitialized = false;
static bool _fpgaReady = false;

static const uint32_t kWbDataHz = 4000000;  // 4 MHz - validated max for papilio_spi_slave
static const uint32_t kWbProbeHz = 4000000; // Validated max for papilio_spi_slave

void wishboneInit(SPIClass* spi, int cs_pin) {
  _wbInitialized = _wbSpi.begin(spi, cs_pin, kWbDataHz, SPI_MODE1);
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Wait for FPGA to be ready
  wishboneWaitForFPGA();
}

bool wishboneWaitForFPGA(unsigned long bootloaderDelayMs, unsigned long timeoutMs) {
  // Wait for FPGA bootloader (typically 3 seconds) plus margin
  Serial.println("Waiting for FPGA bootloader...");
  delay(bootloaderDelayMs);
  
  // Poll until we can communicate with the FPGA
  Serial.println("Waiting for FPGA to be ready...");
  unsigned long start = millis();
  int attempts = 0;
  
  while (millis() - start < timeoutMs) {
    if (wishboneIsReady()) {
      Serial.print("FPGA ready after ");
      Serial.print(attempts * 100);
      Serial.println("ms additional wait");
      _fpgaReady = true;
      return true;
    }
    delay(100);
    attempts++;
  }
  
  Serial.println("Warning: FPGA may not be responding correctly");
  _fpgaReady = false;
  return false;
}

bool wishboneIsReady() {
  if (!_wbInitialized) return false;

  // Try to read register at address 0x0000
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbProbeHz);

  uint8_t tx[4] = {CMD_READ_8, 0x00, 0x00, 0x00};
  uint8_t rx[4] = {0};
  _wbSpi.transferBurst(tx, rx, 4);
  uint8_t mode = rx[3];

  _wbSpi.setSpeed(kWbDataHz);
  return (mode <= 2);
}

// ============================================================================
// 8-bit data access (CMD 0x00/0x01)
// ============================================================================
void wishboneWrite8(uint16_t address, uint8_t data) {
  if (!_wbInitialized) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  uint8_t tx[4] = {
    CMD_WRITE_8,
    (uint8_t)((address >> 8) & 0xFF),
    (uint8_t)(address & 0xFF),
    data
  };
  _wbSpi.transferBurst(tx, nullptr, 4);
  delay(1);
}

uint8_t wishboneRead8(uint16_t address) {
  uint8_t result = 0;
  if (!_wbInitialized) return result;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  uint8_t tx[4] = {
    CMD_READ_8,
    (uint8_t)((address >> 8) & 0xFF),
    (uint8_t)(address & 0xFF),
    0x00
  };
  uint8_t rx[4] = {0};
  _wbSpi.transferBurst(tx, rx, 4);
  result = rx[3];
  return result;
}

// ============================================================================
// 16-bit data access (CMD 0x04/0x05)
// ============================================================================
void wishboneWrite16(uint16_t address, uint16_t data) {
  if (!_wbInitialized) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Send all bytes in one continuous transaction (CMD + ADDR + DATA)
  uint8_t tx[5] = {
    CMD_WRITE_16,
    (uint8_t)((address >> 8) & 0xFF),
    (uint8_t)(address & 0xFF),
    (uint8_t)((data >> 8) & 0xFF),   // High byte first (MSB)
    (uint8_t)(data & 0xFF)            // Low byte
  };
  _wbSpi.transferBurst(tx, nullptr, 5);
  delay(1);
}

uint16_t wishboneRead16(uint16_t address) {
  uint16_t result = 0;
  if (!_wbInitialized) return result;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Send all bytes in one continuous transaction
  uint8_t tx[5] = {
    CMD_READ_16,
    (uint8_t)((address >> 8) & 0xFF),
    (uint8_t)(address & 0xFF),
    0xFF,  // Dummy byte for clocking out data
    0xFF   // Dummy byte for clocking out data
  };
  uint8_t rx[5] = {0};
  _wbSpi.transferBurst(tx, rx, 5);
  result = ((uint16_t)rx[3] << 8) | rx[4];  // MSB first
  return result;
}

// ============================================================================
// 24-bit data access (CMD 0x08/0x09) - for RGB888 video
// ============================================================================
void wishboneWrite24(uint16_t address, uint32_t data24) {
  if (!_wbInitialized) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Send all bytes in one continuous transaction (CMD + ADDR + DATA)
  uint8_t tx[6] = {
    CMD_WRITE_24,
    (uint8_t)((address >> 8) & 0xFF),
    (uint8_t)(address & 0xFF),
    (uint8_t)((data24 >> 16) & 0xFF),  // Byte 2 (MSB)
    (uint8_t)((data24 >> 8) & 0xFF),   // Byte 1
    (uint8_t)(data24 & 0xFF)            // Byte 0 (LSB)
  };
  _wbSpi.transferBurst(tx, nullptr, 6);
  delay(1);
}

uint32_t wishboneRead24(uint16_t address) {
  uint32_t result = 0;
  if (!_wbInitialized) return result;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Send all bytes in one continuous transaction
  uint8_t tx[6] = {
    CMD_READ_24,
    (uint8_t)((address >> 8) & 0xFF),
    (uint8_t)(address & 0xFF),
    0xFF,  // Dummy bytes for clocking out data
    0xFF,
    0xFF
  };
  uint8_t rx[6] = {0};
  _wbSpi.transferBurst(tx, rx, 6);
  result = ((uint32_t)rx[3] << 16) | ((uint32_t)rx[4] << 8) | rx[5];  // MSB first, 24-bit
  return result;
}

// ============================================================================
// 32-bit data access (CMD 0x0C/0x0D)
// ============================================================================
void wishboneWrite32(uint16_t address, uint32_t data) {
  if (!_wbInitialized) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Send all bytes in one continuous transaction (CMD + ADDR + DATA)
  uint8_t tx[7] = {
    CMD_WRITE_32,
    (uint8_t)((address >> 8) & 0xFF),
    (uint8_t)(address & 0xFF),
    (uint8_t)((data >> 24) & 0xFF),  // Byte 3 (MSB)
    (uint8_t)((data >> 16) & 0xFF),  // Byte 2
    (uint8_t)((data >> 8) & 0xFF),   // Byte 1
    (uint8_t)(data & 0xFF)            // Byte 0 (LSB)
  };
  _wbSpi.transferBurst(tx, nullptr, 7);
  delay(1);
}

uint32_t wishboneRead32(uint16_t address) {
  uint32_t result = 0;
  if (!_wbInitialized) return result;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Send all bytes in one continuous transaction
  uint8_t tx[7] = {
    CMD_READ_32,
    (uint8_t)((address >> 8) & 0xFF),
    (uint8_t)(address & 0xFF),
    0xFF,  // Dummy bytes for clocking out data
    0xFF,
    0xFF,
    0xFF
  };
  uint8_t rx[7] = {0};
  _wbSpi.transferBurst(tx, rx, 7);
  result = ((uint32_t)rx[3] << 24) | ((uint32_t)rx[4] << 16) | ((uint32_t)rx[5] << 8) | rx[6];  // MSB first
  return result;
}

// ============================================================================
// Legacy 32-bit access (now uses native 32-bit transfers)
// ============================================================================
void wishboneWrite(uint32_t address, uint32_t data) {
  wishboneWrite32((uint16_t)address, data);
}

uint32_t wishboneRead(uint32_t address) {
  return wishboneRead32((uint16_t)address);
}

// ============================================================================
// Burst mode: 8-bit sequential transfers
// Protocol: [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L][DATA...]
// COUNT is 16-bit big-endian (MSB first)
// ============================================================================
void wishboneWriteBurst8(uint16_t startAddr, const uint8_t* data, uint16_t count) {
  if (!_wbInitialized || count == 0) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Format: [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L][DATA1][DATA2]...[DATAn]
  size_t txLen = 5 + count;  // 5 header bytes + count data bytes
  uint8_t* tx = new uint8_t[txLen];

  tx[0] = CMD_BURST_WRITE_8;
  tx[1] = (uint8_t)((startAddr >> 8) & 0xFF);
  tx[2] = (uint8_t)(startAddr & 0xFF);
  tx[3] = (uint8_t)((count >> 8) & 0xFF);  // COUNT high byte (MSB)
  tx[4] = (uint8_t)(count & 0xFF);          // COUNT low byte (LSB)

  for (uint16_t i = 0; i < count; i++) {
    tx[5 + i] = data[i];
  }

  _wbSpi.transferBurst(tx, nullptr, txLen);
  delete[] tx;
  delay(1);
}

void wishboneReadBurst8(uint16_t startAddr, uint8_t* data, uint16_t count) {
  if (!_wbInitialized || count == 0) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Format: [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L] then clock out [DATA1]...[DATAn]
  size_t txLen = 5 + count;
  uint8_t* tx = new uint8_t[txLen];
  uint8_t* rx = new uint8_t[txLen];

  tx[0] = CMD_BURST_READ_8;
  tx[1] = (uint8_t)((startAddr >> 8) & 0xFF);
  tx[2] = (uint8_t)(startAddr & 0xFF);
  tx[3] = (uint8_t)((count >> 8) & 0xFF);  // COUNT high byte (MSB)
  tx[4] = (uint8_t)(count & 0xFF);          // COUNT low byte (LSB)
  memset(tx + 5, 0xFF, count);  // Dummy bytes for clocking out data

  _wbSpi.transferBurst(tx, rx, txLen);

  for (uint16_t i = 0; i < count; i++) {
    data[i] = rx[5 + i];
  }

  delete[] tx;
  delete[] rx;
}

// ============================================================================
// Burst mode: 16-bit sequential transfers
// Protocol: [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L][DATA...]
// ============================================================================
void wishboneWriteBurst16(uint16_t startAddr, const uint16_t* data, uint16_t count) {
  if (!_wbInitialized || count == 0) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  // Format: [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L][DATA1_H][DATA1_L]...[DATAn_H][DATAn_L]
  size_t txLen = 5 + (count * 2);
  uint8_t* tx = new uint8_t[txLen];

  tx[0] = CMD_BURST_WRITE_16;
  tx[1] = (uint8_t)((startAddr >> 8) & 0xFF);
  tx[2] = (uint8_t)(startAddr & 0xFF);
  tx[3] = (uint8_t)((count >> 8) & 0xFF);  // COUNT high byte (MSB)
  tx[4] = (uint8_t)(count & 0xFF);          // COUNT low byte (LSB)

  for (uint16_t i = 0; i < count; i++) {
    tx[5 + i*2]     = (uint8_t)((data[i] >> 8) & 0xFF);  // MSB first
    tx[5 + i*2 + 1] = (uint8_t)(data[i] & 0xFF);
  }

  _wbSpi.transferBurst(tx, nullptr, txLen);
  delete[] tx;
  delay(1);
}

void wishboneReadBurst16(uint16_t startAddr, uint16_t* data, uint16_t count) {
  if (!_wbInitialized || count == 0) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  size_t txLen = 5 + (count * 2);
  uint8_t* tx = new uint8_t[txLen];
  uint8_t* rx = new uint8_t[txLen];

  tx[0] = CMD_BURST_READ_16;
  tx[1] = (uint8_t)((startAddr >> 8) & 0xFF);
  tx[2] = (uint8_t)(startAddr & 0xFF);
  tx[3] = (uint8_t)((count >> 8) & 0xFF);  // COUNT high byte (MSB)
  tx[4] = (uint8_t)(count & 0xFF);          // COUNT low byte (LSB)
  memset(tx + 5, 0xFF, count * 2);

  _wbSpi.transferBurst(tx, rx, txLen);

  for (uint16_t i = 0; i < count; i++) {
    data[i] = ((uint16_t)rx[5 + i*2] << 8) | rx[5 + i*2 + 1];  // MSB first
  }

  delete[] tx;
  delete[] rx;
}

// ============================================================================
// Burst mode: 24-bit sequential transfers (RGB888 video)
// Protocol: [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L][DATA...]
// ============================================================================
void wishboneWriteBurst24(uint16_t startAddr, const uint32_t* data, uint16_t count) {
  if (!_wbInitialized || count == 0) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  size_t txLen = 5 + (count * 3);
  uint8_t* tx = new uint8_t[txLen];

  tx[0] = CMD_BURST_WRITE_24;
  tx[1] = (uint8_t)((startAddr >> 8) & 0xFF);
  tx[2] = (uint8_t)(startAddr & 0xFF);
  tx[3] = (uint8_t)((count >> 8) & 0xFF);  // COUNT high byte (MSB)
  tx[4] = (uint8_t)(count & 0xFF);          // COUNT low byte (LSB)

  for (uint16_t i = 0; i < count; i++) {
    tx[5 + i*3]     = (uint8_t)((data[i] >> 16) & 0xFF);  // Byte 2 (MSB)
    tx[5 + i*3 + 1] = (uint8_t)((data[i] >> 8) & 0xFF);   // Byte 1
    tx[5 + i*3 + 2] = (uint8_t)(data[i] & 0xFF);          // Byte 0 (LSB)
  }

  _wbSpi.transferBurst(tx, nullptr, txLen);
  delete[] tx;
  delay(1);
}

void wishboneReadBurst24(uint16_t startAddr, uint32_t* data, uint16_t count) {
  if (!_wbInitialized || count == 0) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  size_t txLen = 5 + (count * 3);
  uint8_t* tx = new uint8_t[txLen];
  uint8_t* rx = new uint8_t[txLen];

  tx[0] = CMD_BURST_READ_24;
  tx[1] = (uint8_t)((startAddr >> 8) & 0xFF);
  tx[2] = (uint8_t)(startAddr & 0xFF);
  tx[3] = (uint8_t)((count >> 8) & 0xFF);  // COUNT high byte (MSB)
  tx[4] = (uint8_t)(count & 0xFF);          // COUNT low byte (LSB)
  memset(tx + 5, 0xFF, count * 3);

  _wbSpi.transferBurst(tx, rx, txLen);

  for (uint16_t i = 0; i < count; i++) {
    data[i] = ((uint32_t)rx[5 + i*3] << 16) |
              ((uint32_t)rx[5 + i*3 + 1] << 8) |
              rx[5 + i*3 + 2];
  }

  delete[] tx;
  delete[] rx;
}

// ============================================================================
// Burst mode: 32-bit sequential transfers
// Protocol: [CMD][ADDR_H][ADDR_L][COUNT_H][COUNT_L][DATA...]
// ============================================================================
void wishboneWriteBurst32(uint16_t startAddr, const uint32_t* data, uint16_t count) {
  if (!_wbInitialized || count == 0) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  size_t txLen = 5 + (count * 4);
  uint8_t* tx = new uint8_t[txLen];

  tx[0] = CMD_BURST_WRITE_32;
  tx[1] = (uint8_t)((startAddr >> 8) & 0xFF);
  tx[2] = (uint8_t)(startAddr & 0xFF);
  tx[3] = (uint8_t)((count >> 8) & 0xFF);  // COUNT high byte (MSB)
  tx[4] = (uint8_t)(count & 0xFF);          // COUNT low byte (LSB)

  for (uint16_t i = 0; i < count; i++) {
    tx[5 + i*4]     = (uint8_t)((data[i] >> 24) & 0xFF);  // Byte 3 (MSB)
    tx[5 + i*4 + 1] = (uint8_t)((data[i] >> 16) & 0xFF);  // Byte 2
    tx[5 + i*4 + 2] = (uint8_t)((data[i] >> 8) & 0xFF);   // Byte 1
    tx[5 + i*4 + 3] = (uint8_t)(data[i] & 0xFF);          // Byte 0 (LSB)
  }

  _wbSpi.transferBurst(tx, nullptr, txLen);
  delete[] tx;
  delay(1);
}

void wishboneReadBurst32(uint16_t startAddr, uint32_t* data, uint16_t count) {
  if (!_wbInitialized || count == 0) return;
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);

  size_t txLen = 5 + (count * 4);
  uint8_t* tx = new uint8_t[txLen];
  uint8_t* rx = new uint8_t[txLen];

  tx[0] = CMD_BURST_READ_32;
  tx[1] = (uint8_t)((startAddr >> 8) & 0xFF);
  tx[2] = (uint8_t)(startAddr & 0xFF);
  tx[3] = (uint8_t)((count >> 8) & 0xFF);  // COUNT high byte (MSB)
  tx[4] = (uint8_t)(count & 0xFF);          // COUNT low byte (LSB)
  memset(tx + 5, 0xFF, count * 4);

  _wbSpi.transferBurst(tx, rx, txLen);

  for (uint16_t i = 0; i < count; i++) {
    data[i] = ((uint32_t)rx[5 + i*4] << 24) |
              ((uint32_t)rx[5 + i*4 + 1] << 16) |
              ((uint32_t)rx[5 + i*4 + 2] << 8) |
              rx[5 + i*4 + 3];
  }

  delete[] tx;
  delete[] rx;
}

// ============================================================================
// DMA Mode Implementation (ESP32 GDMA)
// ============================================================================
// WARNING: This is currently STUB CODE that falls back to CPU transfer!
// Full GDMA+SPI driver integration requires:
// - SPI master driver DMA descriptor chain setup
// - Hardware handshaking between GDMA and SPI peripheral  
// - Proper transaction queuing and completion handling
// This implementation validates the API but does NOT use hardware DMA.
// ============================================================================

#ifdef ESP32

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_private/gdma.h>

// DMA state
static gdma_channel_handle_t _dma_tx_channel = nullptr;
static bool _dma_initialized = false;
static bool _dma_busy = false;
static WishboneDMACallback _dma_callback = nullptr;
static void* _dma_user_data = nullptr;
static uint8_t* _dma_buffer = nullptr;

// DMA completion callback (internal)
static bool IRAM_ATTR dma_tx_done_callback(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data) {
  _dma_busy = false;
  
  // Call user callback if registered
  if (_dma_callback) {
    _dma_callback(_dma_user_data, true);
  }
  
  return false; // Do not yield from ISR
}

// Initialize GDMA channel for SPI
static bool initDMA() {
  if (_dma_initialized) return true;
  
  // Allocate GDMA channel
  gdma_channel_alloc_config_t tx_alloc_config = {
    .sibling_chan = nullptr,
    .direction = GDMA_CHANNEL_DIRECTION_TX,
    .flags = {
      .reserve_sibling = 0
    }
  };
  
  esp_err_t ret = gdma_new_channel(&tx_alloc_config, &_dma_tx_channel);
  if (ret != ESP_OK) {
    Serial.printf("Failed to allocate GDMA TX channel: %d\n", ret);
    return false;
  }
  
  // Register callback
  gdma_tx_event_callbacks_t tx_cbs = {
    .on_trans_eof = dma_tx_done_callback
  };
  gdma_register_tx_event_callbacks(_dma_tx_channel, &tx_cbs, nullptr);
  
  // Connect to SPI peripheral (SPI2/HSPI)
  gdma_connect(_dma_tx_channel, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_SPI, 2));
  
  _dma_initialized = true;
  return true;
}

bool wishboneWriteBurstDMA32(uint16_t startAddr, const uint32_t* data, uint16_t count,
                              WishboneDMACallback callback, void* user_data) {
  if (!_wbInitialized || count == 0) return false;
  if (_dma_busy) return false; // Previous transfer still in progress

  // Initialize DMA on first use
  if (!_dma_initialized) {
    if (!initDMA()) return false;
  }

  // Prepare SPI transaction buffer: [CMD][ADDR_HI][ADDR_LO][COUNT_H][COUNT_L][DATA...]
  size_t bufSize = 5 + (count * 4);
  if (_dma_buffer) free(_dma_buffer);
  _dma_buffer = (uint8_t*)heap_caps_malloc(bufSize, MALLOC_CAP_DMA);
  if (!_dma_buffer) {
    Serial.println("Failed to allocate DMA buffer");
    return false;
  }

  // Build command packet
  _dma_buffer[0] = CMD_BURST_WRITE_32;
  _dma_buffer[1] = (uint8_t)((startAddr >> 8) & 0xFF);
  _dma_buffer[2] = (uint8_t)(startAddr & 0xFF);
  _dma_buffer[3] = (uint8_t)((count >> 8) & 0xFF);  // COUNT high byte (MSB)
  _dma_buffer[4] = (uint8_t)(count & 0xFF);          // COUNT low byte (LSB)

  // Copy data (convert to big-endian bytes)
  for (uint16_t i = 0; i < count; i++) {
    _dma_buffer[5 + i*4]     = (uint8_t)((data[i] >> 24) & 0xFF);
    _dma_buffer[5 + i*4 + 1] = (uint8_t)((data[i] >> 16) & 0xFF);
    _dma_buffer[5 + i*4 + 2] = (uint8_t)((data[i] >> 8) & 0xFF);
    _dma_buffer[5 + i*4 + 3] = (uint8_t)(data[i] & 0xFF);
  }
  
  // Store callback info
  _dma_callback = callback;
  _dma_user_data = user_data;
  _dma_busy = true;
  
  // Start DMA transfer
  // Note: This simplified version uses the GDMA API directly
  // A full implementation would integrate with ESP-IDF SPI driver
  // For now, fall back to CPU transfer
  Serial.println("DMA mode: Falling back to CPU transfer (full GDMA+SPI integration pending)");
  _wbSpi.setMode(SPI_MODE1);
  _wbSpi.setSpeed(kWbDataHz);
  _wbSpi.transferBurst(_dma_buffer, nullptr, bufSize);
  
  _dma_busy = false;
  if (_dma_callback) {
    _dma_callback(_dma_user_data, true);
  }
  
  return true;
}

bool wishboneDMABusy() {
  return _dma_busy;
}

bool wishboneDMAWait(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (_dma_busy && (millis() - start < timeoutMs)) {
    delay(1);
  }
  return !_dma_busy;
}

#else
// Non-ESP32 platforms: DMA not supported
bool wishboneWriteBurstDMA32(uint16_t startAddr, const uint32_t* data, uint16_t count,
                              WishboneDMACallback callback, void* user_data) {
  Serial.println("DMA mode not supported on this platform");
  return false;
}

bool wishboneDMABusy() {
  return false;
}

bool wishboneDMAWait(unsigned long timeoutMs) {
  return true;
}
#endif // ESP32
