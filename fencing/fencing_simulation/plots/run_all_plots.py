#!/usr/bin/env python3
"""
Run all plots for the fencing simulation.
Usage: python plots/run_all_plots.py
"""

import subprocess
import sys
from pathlib import Path

plots_dir = Path(__file__).parent

plots = [
    "plot_1d.py",
    "plot_2d.py",
    "plot_2d_breathing.py",
    "plot_3d.py",
    "plot_3d_planar.py",
]

for plot in plots:
    print(f"\n{'='*60}")
    print(f"Running {plot}...")
    print(f"{'='*60}")
    result = subprocess.run(
        [sys.executable, str(plots_dir / plot)],
        cwd=str(plots_dir.parent),
        capture_output=True,
        text=True
    )
    print(result.stdout)
    if result.stderr:
        print("STDERR:", result.stderr)
    if result.returncode != 0:
        print(f"ERROR: {plot} failed with return code {result.returncode}")

print("\n" + "="*60)
print("All plots completed!")
print("="*60)