#!/usr/bin/env python3
"""
Plotting script for 1D simulation.
Reads data from data/simulate_1d.npz and generates figures.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path


# Global font setting
plt.rc('font', family='Times New Roman')

# Load data
data_dir = Path(__file__).parent.parent / "data"
fig_dir = Path(__file__).parent.parent / "figures"
fig_dir.mkdir(exist_ok=True)

data = np.load(data_dir / 'simulate_1d.npz', allow_pickle=True)
t = data['t']
x_traj = data['x_traj']
x_target_traj = data['x_target_traj']
pairwise_dists = data['pairwise_dists']
vel_error = data['vel_error']
params = data['params'].item()

N = params['N']
d_col = params['d_col']
mu = params['mu']
k1 = params['k1']
v0 = params['v0']

print("Generating 1D plots...")

# ============================================================
# Plot 1: Pairwise distances
# ============================================================
fig, ax = plt.subplots(figsize=(10, 5))
for idx in range(pairwise_dists.shape[0]):
    ax.plot(t, pairwise_dists[idx], alpha=0.7, linewidth=0.8)
ax.axhline(y=d_col, color='r', linestyle='--', alpha=0.5, label=f'Collision dist d={d_col}')
ax.axhline(y=mu, color='g', linestyle='--', alpha=0.5, label=f'Sensing radius μ={mu}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Distance')
ax.set_title('1D Fencing Controller (no observer): Pairwise Vehicle Distances', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'distances_1d.png', dpi=150)
plt.close()
print("Saved distances_1d.png")

# ============================================================
# Plot 2: Velocity error norm
# ============================================================
fig, ax = plt.subplots(figsize=(10, 5))
for i in range(N):
    ax.plot(t, vel_error[i], alpha=0.7, linewidth=0.8, label=f'Agent {i+1}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('|ẋ_i - v₀|')
ax.set_title('1D Fencing Controller (no observer): Velocity Error Norm per Agent', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'vel_error_1d.png', dpi=150)
plt.close()
print("Saved vel_error_1d.png")

# ============================================================
# Plot 3: Positions x-t heatmap (replaces GIF)
# ============================================================
fig, ax = plt.subplots(figsize=(14, 6))

# Use target-relative positions for better visualization
rel_traj = x_traj - x_target_traj  # (N, len(t))

# Plot each agent's trajectory as a line
for i in range(N):
    ax.plot(t, rel_traj[i], linewidth=1.0, alpha=0.8, label=f'Agent {i+1}')

# Target line at x=0
ax.axhline(y=0, color='red', linestyle='-', alpha=0.3, linewidth=2, label='Target')

ax.set_xlabel('Time [s]')
ax.set_ylabel('Position (target-relative)')
ax.set_title('1D Fencing Controller (no observer): Agent Positions vs Time', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'positions_1d.png', dpi=150)
plt.close()
print("Saved positions_1d.png")

print("\nAll 1D plots generated successfully.")