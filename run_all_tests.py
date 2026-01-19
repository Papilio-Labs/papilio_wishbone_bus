#!/usr/bin/env python3
"""
Top-level test runner for papilio_wishbone_bus

This uses the test infrastructure from papilio_dev_tools.

Usage:
    python run_all_tests.py
    python run_all_tests.py --sim-only
    python run_all_tests.py --hw-only

Author: Papilio Project
License: MIT
"""

import sys
import subprocess
from pathlib import Path

# Use the regression test runner from papilio_dev_tools
dev_tools_path = Path(__file__).parent.parent / "papilio_dev_tools"
script_path = dev_tools_path / "scripts" / "run_regression_tests.py"

if not script_path.exists():
    print(f"ERROR: Cannot find papilio_dev_tools at {dev_tools_path}")
    print("Please ensure papilio_dev_tools library is installed.")
    sys.exit(1)

if __name__ == "__main__":
    # Pass all arguments to the regression test runner
    # Run from the library root directory
    lib_root = Path(__file__).parent
    result = subprocess.run(
        [sys.executable, str(script_path)] + sys.argv[1:],
        cwd=str(lib_root)
    )
    sys.exit(result.returncode)
