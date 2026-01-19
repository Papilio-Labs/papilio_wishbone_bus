// Basic example for papilio_wishbone_spi_master
// Demonstrates initializing the SPI/Wishbone helper and performing a write/read

#include <SPI.h>
#include "WishboneSPI.h"

// Adjust pins to your board
#define SPI_CLK   12
#define SPI_MOSI  11
#define SPI_MISO  9
#define SPI_CS    10

SPIClass *fpgaSPI = nullptr;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Basic Wishbone Example");

  fpgaSPI = new SPIClass(HSPI);
  fpgaSPI->begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);

  // Initialize the wishbone helper (sets up CS and any state)
  wishboneInit(fpgaSPI, SPI_CS);

  // Example: write 0x12 to address 0x03, then read it back
  uint8_t addr = 0x03;
  uint8_t value = 0x12;

  Serial.print("Writing "); Serial.print(value, HEX); Serial.print(" to "); Serial.println(addr, HEX);
  wishboneWrite8(addr, value);

  uint8_t readback = wishboneRead8(addr);
  Serial.print("Read back: "); Serial.println(readback, HEX);
}

void loop() {
  delay(1000);
}
