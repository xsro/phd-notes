#!/usr/bin/env python3
"""
Plotting script for 3D simulation with N=30.
Reads data from data/simulate_3d_n30.npz and generates figures + GIF.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path

# Load data
data_dir = Path(__file__).parent.parent / "data"
fig_dir = Path(__file__).parent.parent / "figures_n30"
fig_dir.mkdir(exist_ok=True)

data = np.load(data_dir / 'simulate_3d_n30.npz', allow_pickle=True)
t = data['t']
x_traj = data['x_traj']
v_traj = data['v_traj']
x_target_traj = data['x_target_traj']
pairwise_dists = data['pairwise_dists']
vel_error = data['vel_error']
T_ANIM = float(data['T_ANIM'])
params = data['params'].item()

N = params['N']
d_col = params['d_col']
mu = params['mu']
k1 = params['k1']
k2 = params['k2']
v0 = params['v0']

print(f"Generating 3D plots for N={N}...")

# ============================================================
# Plot 1: Pairwise distances
# ============================================================
fig, ax = plt.subplots(figsize=(12, 6))
# Plot mean and std band
mean_pd = np.mean(pairwise_dists, axis=0)
std_pd = np.std(pairwise_dists, axis=0)
ax.plot(t, mean_pd, 'b-', linewidth=1.5, label='Mean distance')
ax.fill_between(t, mean_pd - std_pd, mean_pd + std_pd, alpha=0.3, color='blue')
ax.axhline(y=d_col, color='r', linestyle='--', alpha=0.5, label=f'd={d_col}')
ax.axhline(y=mu, color='g', linestyle='--', alpha=0.5, label=f'μ={mu}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Distance')
ax.set_title(f'3D Fencing Controller (N={N}, μ={mu}): Pairwise Vehicle Distances', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'distances_3d_n30.png', dpi=150)
plt.close()
print("Saved distances_3d_n30.png")

# ============================================================
# Plot 2: Velocity error norm
# ============================================================
fig, ax = plt.subplots(figsize=(12, 6))
mean_ve = np.mean(vel_error, axis=0)
ax.plot(t, mean_ve, 'r-', linewidth=1.5, label='Mean velocity error')
ax.set_xlabel('Time [s]')
ax.set_ylabel('|v_i - v₀|')
ax.set_title(f'3D Fencing Controller (N={N}, μ={mu}): Mean Velocity Error', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'vel_error_3d_n30.png', dpi=150)
plt.close()
print("Saved vel_error_3d_n30.png")

# ============================================================
# Animation: Positions over time (3D) — steady state only
# ============================================================
def make_animation():
    # Show only steady state: start from T_MAX * 0.5
    T_MAX_val = float(params['T_MAX'])
    t_start = T_MAX_val * 0.5
    n_frames = 80
    t_anim = np.linspace(t_start, T_ANIM, n_frames)
    t_indices = [np.argmin(np.abs(t - tt)) for tt in t_anim]

    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(111, projection='3d')

    margin = 15.0
    ax.set_xlim(-margin, margin)
    ax.set_ylim(-margin, margin)
    ax.set_zlim(-margin, margin)
    ax.set_xlabel('x (target-relative)')
    ax.set_ylabel('y (target-relative)')
    ax.set_zlabel('z (target-relative)')
    ax.set_title(f'3D Fencing Controller (N={N}, μ={mu}): Steady-State Agent Positions', fontsize=14)
    ax.view_init(elev=25, azim=45)

    target_dot = ax.plot([0], [0], [0], 'ro', markersize=12, label='Target')[0]
    scatter = ax.scatter([], [], [], s=40, c='blue', alpha=0.6, edgecolors='darkblue')
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=14)

    def init():
        scatter._offsets3d = ([], [], [])
        target_dot.set_data([0], [0])
        target_dot.set_3d_properties([0])
        time_text.set_text('')
        return scatter, target_dot, time_text

    def update(frame):
        idx = t_indices[frame]
        rel_positions = x_traj[:, :, idx] - x_target_traj[:, idx].reshape(1, 3)
        scatter._offsets3d = (rel_positions[:, 0], rel_positions[:, 1], rel_positions[:, 2])
        target_dot.set_data([0], [0])
        target_dot.set_3d_properties([0])
        time_text.set_text(f't = {t_anim[frame]:.1f}s')
        return scatter, target_dot, time_text

    ani = animation.FuncAnimation(
        fig, update, frames=len(t_anim),
        init_func=init, blit=True, interval=50
    )

    writer = animation.PillowWriter(fps=20)
    ani.save(fig_dir / 'positions_3d_n30_steady.gif', writer=writer, dpi=80)
    plt.close()
    print("Saved positions_3d_n30_steady.gif")

make_animation()

print("\nAll 3D N=30 plots generated successfully.")