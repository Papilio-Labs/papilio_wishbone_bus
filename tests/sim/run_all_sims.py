#!/usr/bin/env python3
"""
Run all simulation tests for papilio_wishbone_bus

Uses run_sim module from papilio_dev_tools for cross-platform simulation.

Usage:
    python run_all_sims.py
"""

import sys
import os
import glob
from pathlib import Path

# Import run_sim from papilio_dev_tools
dev_tools_scripts = Path(__file__).parent.parent.parent.parent / "papilio_dev_tools" / "scripts"
sys.path.insert(0, str(dev_tools_scripts))

try:
    import run_sim
except ImportError:
    print("Error: Could not import run_sim from papilio_dev_tools", file=sys.stderr)
    print(f"Looking in: {dev_tools_scripts}", file=sys.stderr)
    print("Please ensure papilio_dev_tools is installed.", file=sys.stderr)
    sys.exit(1)


def find_testbenches():
    """Find all testbench files (tb_*.v)"""
    testbenches = glob.glob("tb_*.v")
    return sorted(testbenches)


def run_testbench(testbench, oss_cad_path):
    """Run a single testbench"""
    print(f"\n{'='*60}")
    print(f"Running: {testbench}")
    print('='*60)
    
    # Determine required source files based on testbench
    gateware_dir = Path("../../gateware")
    wb_reg_dir = Path("../../../papilio_wb_register/gateware")
    spi_slave_dir = Path("../../../papilio_spi_slave/gateware")
    
    sources = [testbench]
    
    # Add gateware modules
    if "pwb_multi_width" in testbench:
        sources.extend([
            str(gateware_dir / "pwb_spi_wb_bridge.v"),
            str(wb_reg_dir / "wb_register_block.v"),
            str(spi_slave_dir / "fifo_sync.v")  # Required by pwb_spi_wb_bridge
        ])
    elif "spi_wb_register" in testbench:
        sources.extend([
            str(gateware_dir / "simple_spi_wb_bridge.v"),
            str(wb_reg_dir / "wb_register_block.v")
        ])
    
    # Compile
    output = testbench.replace('.v', '.vvp')
    success = run_sim.compile_verilog(
        sources=sources,
        output=output,
        include_dirs=None,
        standard="2012",
        oss_cad_path=oss_cad_path
    )
    
    if not success:
        return False
    
    # Run
    success = run_sim.run_simulation(output, oss_cad_path=oss_cad_path)
    
    return success


def main():
    print("="*60)
    print("Papilio Wishbone Bus - Simulation Test Runner")
    print("="*60)
    
    # Set up environment
    oss_cad_path = run_sim.setup_environment()
    
    # Find testbenches
    testbenches = find_testbenches()
    
    if not testbenches:
        print("No testbenches found (no tb_*.v files)")
        return 0
    
    print(f"\nFound {len(testbenches)} testbench(es):")
    for tb in testbenches:
        print(f"  - {tb}")
    
    # Run each testbench
    results = []
    for tb in testbenches:
        success = run_testbench(tb, oss_cad_path)
        results.append((tb, success))
    
    # Summary
    print(f"\n{'='*60}")
    print("Test Summary")
    print('='*60)
    
    passed = sum(1 for _, success in results if success)
    failed = len(results) - passed
    
    for tb, success in results:
        status = "[PASS]" if success else "[FAIL]"
        print(f"{status}: {tb}")
    
    print(f"\nTotal: {len(results)} | Passed: {passed} | Failed: {failed}")
    
    if failed == 0:
        print("\n*** All simulation tests passed! ***")
        return 0
    else:
        print(f"\n*** {failed} test(s) failed! ***")
        return 1


if __name__ == "__main__":
    sys.exit(main())
