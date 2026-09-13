"""
Plotting script for Case B — Jammed Boundary.

Loads data/case_b.npz and produces:
    figures/case_b_trajectory.pdf
    figures/case_b_distances.pdf
    figures/case_b_angular_velocities.pdf

Run from the repository root:
    python simulation/plot_case_b.py
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_base import (
    figure, save_figure,
    plot_trajectories, plot_distances, plot_angular_velocities,
    TH,
)

# ── Load data ────────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
data_dir = os.path.join(repo_root, "data")
fig_dir = os.path.join(repo_root, "figures")
os.makedirs(fig_dir, exist_ok=True)

data = np.load(os.path.join(data_dir, "case_b.npz"))
pos_all = data["pos_all"]
vel_all = data["vel_all"]
t_eval = data["t_eval"]
d = float(data["d"])
mu = float(data["mu"])
k1 = float(data["k1"])
k2 = float(data["k2"])
N = int(data["N"])
description = str(data["description"])

print(f"Loaded {description}  (N={N}, {len(t_eval)} frames)")

# ── 1. Trajectory plot ──────────────────────────────────────────────────────
fig, ax = figure(figsize=(8, 7))
plot_trajectories(ax, pos_all, t_eval, d, mu, N,
                  title=f"Case B — {description}")
save_figure(fig, os.path.join(fig_dir, "case_b_trajectory.pdf"))

# ── 2. Distance plot ────────────────────────────────────────────────────────
fig, ax = figure(figsize=(10, 4.5))
plot_distances(ax, pos_all, t_eval, d, N,
               title=f"Case B — pairwise distances")
save_figure(fig, os.path.join(fig_dir, "case_b_distances.pdf"))

# ── 3. Angular velocity plot ────────────────────────────────────────────────
fig, ax = figure(figsize=(10, 4.5))
plot_angular_velocities(ax, pos_all, vel_all, t_eval, k2, N,
                        title=f"Case B — angular velocities")
save_figure(fig, os.path.join(fig_dir, "case_b_angular_velocities.pdf"))

# ── 4. Zoomed trajectory (early stage) ──────────────────────────────────────
# Focus on the first third to see the collision dynamics clearly
cut = len(t_eval) // 3
fig, ax = figure(figsize=(7, 6))
plot_trajectories(ax, pos_all[:cut], t_eval[:cut], d, mu, N,
                  title=f"Case B — trajectory (early, $t \\leq {t_eval[cut-1]:.0f}$)")
save_figure(fig, os.path.join(fig_dir, "case_b_trajectory_zoom.pdf"))

print("All Case B plots generated.")