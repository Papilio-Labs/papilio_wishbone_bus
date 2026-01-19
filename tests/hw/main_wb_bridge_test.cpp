#include <Arduino.h>
#include "WishboneSPI.h"

// SPI pin configuration (ESP32-S3)
#define SPI_CLK   1
#define SPI_MOSI  2
#define SPI_MISO  4
#define SPI_CS    3

SPIClass fpgaSPI;

struct TestCase {
  uint16_t addr;
  uint8_t value;
};

static TestCase kSmokeTests[] = {
  {0x0000, 0x12},
  {0x0001, 0x34},
  {0x0002, 0x56},
  {0x0003, 0x78},
  {0x0004, 0xAA},
  {0x0005, 0x55},
  {0x0006, 0x00},
  {0x0007, 0xFF},
};

struct TestResult {
  uint32_t passed = 0;
  uint32_t failed = 0;
};

static TestResult runTestCase(const TestCase& t) {
  TestResult r;
  wishboneWrite8(t.addr, t.value);
  uint8_t rd = wishboneRead8(t.addr);
  if (rd == t.value) {
    r.passed++;
    Serial.printf("[PASS] Addr 0x%04X wrote 0x%02X read 0x%02X\n", t.addr, t.value, rd);
  } else {
    r.failed++;
    Serial.printf("[FAIL] Addr 0x%04X wrote 0x%02X read 0x%02X\n", t.addr, t.value, rd);
  }
  return r;
}

static TestResult runSmokeSuite() {
  Serial.println("Running smoke tests (static patterns)...");
  TestResult accum;
  for (auto& t : kSmokeTests) {
    TestResult r = runTestCase(t);
    accum.passed += r.passed;
    accum.failed += r.failed;
  }
  return accum;
}

static TestResult runWalkingOnes(uint16_t baseAddr) {
  Serial.println("Running walking-ones (0x01 << n) across 8 bits)...");
  TestResult accum;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    uint8_t val = (1u << bit);
    TestResult r = runTestCase({(uint16_t)(baseAddr + bit), val});
    accum.passed += r.passed;
    accum.failed += r.failed;
  }
  return accum;
}

static TestResult runWalkingZeros(uint16_t baseAddr) {
  Serial.println("Running walking-zeros (~(0x01 << n)) across 8 bits)...");
  TestResult accum;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    uint8_t val = (uint8_t)~(1u << bit);
    TestResult r = runTestCase({(uint16_t)(baseAddr + bit), val});
    accum.passed += r.passed;
    accum.failed += r.failed;
  }
  return accum;
}

static void printSummary(const char* label, const TestResult& r) {
  Serial.printf("%s: passed=%lu failed=%lu\n", label, (unsigned long)r.passed, (unsigned long)r.failed);
}

static void runTestsOnce() {
  TestResult total;

  TestResult smoke = runSmokeSuite();
  total.passed += smoke.passed;
  total.failed += smoke.failed;

  TestResult ones = runWalkingOnes(0x0010);
  total.passed += ones.passed;
  total.failed += ones.failed;

  TestResult zeros = runWalkingZeros(0x0020);
  total.passed += zeros.passed;
  total.failed += zeros.failed;

  Serial.println("-- Suite Summary --");
  printSummary("Smoke", smoke);
  printSummary("Walking Ones", ones);
  printSummary("Walking Zeros", zeros);
  printSummary("Total", total);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("====================================");
  Serial.println("Wishbone SPI Bridge Test (Mode 1, 8-bit, <=4 MHz)");
  Serial.println("====================================");

  fpgaSPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);
  wishboneInit(&fpgaSPI, SPI_CS);

  runTestsOnce();
}

void loop() {
  Serial.println("==== Restarting test suite in 4s ====");
  delay(4000);
  runTestsOnce();
}
