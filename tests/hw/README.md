# Hardware Regression (ESP32 + FPGA)

This folder holds the ESP32 hardware regression sketch for the Wishbone bridge bring-up. It mirrors the root `src/main_wb_bridge_test.cpp` used by PlatformIO and keeps the smoke + walking suites together with the library assets.

To run on hardware:
1) Ensure FPGA bitstream with `simple_spi_wb_bridge` + `wb_register_block` is loaded.
2) In repo root: `pio run -e esp32 -t upload`.
3) Open serial monitor to view PASS/FAIL and the 4s cadence banner.

Keep this copy in sync with the PlatformIO `src/main_wb_bridge_test.cpp` when evolving the tests.
