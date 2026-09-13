"""
Plotting script for Case C — Breathing Limit Cycle.

Loads data/case_c.npz and produces:
    figures/case_c_trajectory.pdf
    figures/case_c_distances.pdf
    figures/case_c_angular_velocities.pdf
    figures/case_c_phase_portrait.pdf

Run from the repository root:
    python simulation/plot_case_c.py
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_base import (
    figure, style_axes, save_figure,
    plot_trajectories, plot_distances, plot_angular_velocities,
    TH, AGENT_COLORS,
)

# ── Load data ────────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
data_dir = os.path.join(repo_root, "data")
fig_dir = os.path.join(repo_root, "figures")
os.makedirs(fig_dir, exist_ok=True)

data = np.load(os.path.join(data_dir, "case_c.npz"))
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
                  title=f"Case C — {description}")
save_figure(fig, os.path.join(fig_dir, "case_c_trajectory.pdf"))

# ── 2. Distance plot ────────────────────────────────────────────────────────
fig, ax = figure(figsize=(10, 4.5))
plot_distances(ax, pos_all, t_eval, d, N,
               title=f"Case C — pairwise distances (periodic oscillation)")
save_figure(fig, os.path.join(fig_dir, "case_c_distances.pdf"))

# ── 3. Angular velocity plot ────────────────────────────────────────────────
fig, ax = figure(figsize=(10, 4.5))
plot_angular_velocities(ax, pos_all, vel_all, t_eval, k2, N,
                        title=f"Case C — angular velocities")
save_figure(fig, os.path.join(fig_dir, "case_c_angular_velocities.pdf"))

# ── 4. Phase portrait (radius vs. radial velocity for agent 0) ──────────────
fig, ax = figure(figsize=(6, 5))
xi0 = pos_all[:, 0]          # position of agent 0 over time
dxi0 = vel_all[:, 0]         # velocity of agent 0 over time
r0 = np.linalg.norm(xi0, axis=1)
r_dot0 = np.sum(xi0 * dxi0, axis=1) / np.maximum(r0, 1e-12)

ax.plot(r0, r_dot0, color=AGENT_COLORS[0], lw=0.8, alpha=0.7)
ax.scatter(r0[0], r_dot0[0], s=40, color=AGENT_COLORS[0], zorder=5,
           edgecolors=TH["surface"], linewidths=0.8, label="start")
ax.scatter(r0[-1], r_dot0[-1], s=40, color=AGENT_COLORS[0], marker="s",
           zorder=5, edgecolors=TH["surface"], linewidths=0.8, label="end")
ax.set_xlabel("$r_0 = \\|\\tilde{x}_0\\|$", color=TH["secondary"], fontsize=9)
ax.set_ylabel("$\\dot{r}_0$", color=TH["secondary"], fontsize=9)
ax.set_title(f"Case C — phase portrait (Agent 0)", fontsize=10,
             color=TH["primary"], fontweight="bold", loc="left", pad=8)
ax.legend(frameon=False, fontsize=7, labelcolor=TH["secondary"])
save_figure(fig, os.path.join(fig_dir, "case_c_phase_portrait.pdf"))

# ── 5. Mean radius over time ────────────────────────────────────────────────
fig, ax = figure(figsize=(10, 4))
r_mean = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
r_std = np.std(np.linalg.norm(pos_all, axis=2), axis=1)
ax.plot(t_eval, r_mean, color=TH["primary"], lw=1.0, label="mean radius")
ax.fill_between(t_eval, r_mean - r_std, r_mean + r_std,
                color=TH["primary"], alpha=0.15, label="$\\pm 1\\sigma$")
ax.axhline(d, color=TH["muted"], lw=1.0, ls="--", alpha=0.5,
           label=f"$d={d}$")
ax.set_xlabel("$t$", color=TH["secondary"], fontsize=9)
ax.set_ylabel("$\\|\\tilde{x}\\|$", color=TH["secondary"], fontsize=9)
ax.set_title(f"Case C — mean agent radius (periodic breathing)", fontsize=10,
             color=TH["primary"], fontweight="bold", loc="left", pad=8)
ax.legend(frameon=False, fontsize=7, labelcolor=TH["secondary"])
save_figure(fig, os.path.join(fig_dir, "case_c_radius.pdf"))

print("All Case C plots generated.")