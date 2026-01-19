# Simulation and VCD Analysis

Artifacts for sim-based Wishbone bridge testing live here. The testbench wires `simple_spi_wb_bridge` to `wb_register_block`, drives Mode 1 SPI stimuli, and emits a VCD for inspection and scripted checks.

## Run

```
run_sim.bat
```

Requirements: OSS CAD Suite installed at `C:\oss-cad-suite` (for `iverilog`, `vvp`, `gtkwave`) and Python 3 on PATH.

Outputs:
- `sim.vvp` compiled simulation
- `tb_spi_wb_register.vcd` waveform
- `parse_vcd.py` summary (ack pulses, first few reads/writes)

Extend the testbench or parser to add new scenarios; both are intentionally minimal and readable.
