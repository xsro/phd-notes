#!/usr/bin/env python3
"""
Plotting script for the 3D planar test.
Reads data from data/test_3d_planar.npz and generates figures.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path


# Global font setting
plt.rc('font', family='Times New Roman')

# Load data
data_dir = Path(__file__).parent.parent / "data"
fig_dir = Path(__file__).parent.parent / "figures"
fig_dir.mkdir(exist_ok=True)

data = np.load(data_dir / 'test_3d_planar.npz', allow_pickle=True)
t = data['t']
x_traj = data['x_traj']
v_traj = data['v_traj']
x_target_traj = data['x_target_traj']
params = data['params'].item()

N = params['N']
d_col = params['d_col']
mu = params['mu']
k1 = params['k1']
k2 = params['k2']
v0 = params['v0']

print("Generating 3D planar test plots...")

# ============================================================
# Plot 1: Pairwise distances
# ============================================================
pairwise_dists = np.zeros((N * (N - 1) // 2, len(t)))
idx = 0
for i in range(N):
    for j in range(i + 1, N):
        pairwise_dists[idx] = np.linalg.norm(x_traj[i] - x_traj[j], axis=0)
        idx += 1

fig, ax = plt.subplots(figsize=(10, 5))
for idx in range(pairwise_dists.shape[0]):
    ax.plot(t, pairwise_dists[idx], alpha=0.7, linewidth=0.8)
ax.axhline(y=d_col, color='r', linestyle='--', alpha=0.5, label=f'd={d_col}')
ax.axhline(y=mu, color='g', linestyle='--', alpha=0.5, label=f'μ={mu}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Distance')
ax.set_title('3D Planar Test: Pairwise Vehicle Distances', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'distances_3d_planar.png', dpi=150)
plt.close()
print("Saved distances_3d_planar.png")

# ============================================================
# Plot 2: Velocity error norm
# ============================================================
vel_error = np.linalg.norm(v_traj - v0.reshape(1, 3, 1), axis=1)

fig, ax = plt.subplots(figsize=(10, 5))
for i in range(N):
    ax.plot(t, vel_error[i], alpha=0.7, linewidth=0.8, label=f'Agent {i+1}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('|v_i - v₀|')
ax.set_title('3D Planar Test: Velocity Error Norm per Agent', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'vel_error_3d_planar.png', dpi=150)
plt.close()
print("Saved vel_error_3d_planar.png")

# ============================================================
# Plot 3: z-coordinate (should stay near zero)
# ============================================================
xi_traj = x_traj - x_target_traj.reshape(1, 3, -1)

fig, ax = plt.subplots(figsize=(10, 5))
for i in range(N):
    ax.plot(t, xi_traj[i, 2, :], alpha=0.7, linewidth=0.8, label=f'Agent {i+1}')
ax.axhline(y=0, color='r', linestyle='--', alpha=0.5)
ax.set_xlabel('Time [s]')
ax.set_ylabel('z (target-relative)')
ax.set_title('3D Planar Test: z-coordinate vs Time (should stay ~0)', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'z_coord_3d_planar.png', dpi=150)
plt.close()
print("Saved z_coord_3d_planar.png")

# ============================================================
# Plot 4: Angular velocity
# ============================================================
vell_traj = v_traj - v0.reshape(1, 3, 1)
omega_z = np.zeros(len(t))
for k in range(len(t)):
    total = 0.0
    total_weight = 0.0
    for i in range(N):
        xi, yi = xi_traj[i, 0, k], xi_traj[i, 1, k]
        vxi, vyi = vell_traj[i, 0, k], vell_traj[i, 1, k]
        r2 = xi**2 + yi**2
        if r2 > 0.01:
            total += (xi * vyi - yi * vxi) / r2
            total_weight += 1
    if total_weight > 0:
        omega_z[k] = total / total_weight

fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(t, omega_z, 'b-', linewidth=1.0)
ax.axhline(y=np.sqrt(k2), color='r', linestyle='--', alpha=0.7,
           label=f'√k₂ = {np.sqrt(k2):.4f}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('ω_z')
ax.set_title('3D Planar Test: Angular Velocity ω_z vs Time', fontsize=14)
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'omega_3d_planar.png', dpi=150)
plt.close()
print("Saved omega_3d_planar.png")

print("\nAll 3D planar test plots generated successfully.")