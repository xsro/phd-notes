#!/usr/bin/env python3
"""
Plotting script for 3D simulation.
Reads data from data/simulate_3d.npz and generates figures.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path

# Load data
data_dir = Path(__file__).parent.parent / "data"
fig_dir = Path(__file__).parent.parent / "figures"
fig_dir.mkdir(exist_ok=True)

data = np.load(data_dir / 'simulate_3d.npz', allow_pickle=True)
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

print("Generating 3D plots...")

# ============================================================
# Plot 1: Pairwise distances
# ============================================================
fig, ax = plt.subplots(figsize=(10, 5))
for idx in range(pairwise_dists.shape[0]):
    ax.plot(t, pairwise_dists[idx], alpha=0.7, linewidth=0.8)
ax.axhline(y=d_col, color='r', linestyle='--', alpha=0.5, label=f'd={d_col}')
ax.axhline(y=mu, color='g', linestyle='--', alpha=0.5, label=f'μ={mu}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Distance')
ax.set_title('3D Fencing Controller: Pairwise Vehicle Distances', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'distances_3d.png', dpi=150)
plt.close()
print("Saved distances_3d.png")

# ============================================================
# Plot 2: Velocity error norm
# ============================================================
fig, ax = plt.subplots(figsize=(10, 5))
for i in range(N):
    ax.plot(t, vel_error[i], alpha=0.7, linewidth=0.8, label=f'Agent {i+1}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('|v_i - v₀|')
ax.set_title('3D Fencing Controller: Velocity Error Norm per Agent', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'vel_error_3d.png', dpi=150)
plt.close()
print("Saved vel_error_3d.png")

# ============================================================
# Animation: Positions over time (3D)
# ============================================================
def make_animation():
    t_anim = np.arange(0, T_ANIM + 0.05, 0.05)
    t_indices = [np.argmin(np.abs(t - tt)) for tt in t_anim]

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Camera follows target — use target-relative frame
    margin = 12.0
    ax.set_xlim(-margin, margin)
    ax.set_ylim(-margin, margin)
    ax.set_zlim(-margin, margin)
    ax.set_xlabel('x (target-relative)')
    ax.set_ylabel('y (target-relative)')
    ax.set_zlabel('z (target-relative)')
    ax.set_title('3D Fencing Controller: Agent Positions', fontsize=14)
    ax.view_init(elev=20, azim=45)

    # Target marker (always at origin)
    target_dot = ax.plot([0], [0], [0], 'ro', markersize=10, label='Target')[0]
    scatter = ax.scatter([], [], [], s=150, c='blue', alpha=0.7, edgecolors='darkblue')
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=14)

    def init():
        scatter._offsets3d = ([], [], [])
        target_dot.set_data([0], [0])
        target_dot.set_3d_properties([0])
        time_text.set_text('')
        return scatter, target_dot, time_text

    def update(frame):
        idx = t_indices[frame]
        # Use target-relative positions so target is always at origin
        rel_positions = x_traj[:, :, idx] - x_target_traj[:, idx].reshape(1, 3)
        scatter._offsets3d = (rel_positions[:, 0], rel_positions[:, 1], rel_positions[:, 2])
        # Target stays at origin
        target_dot.set_data([0], [0])
        target_dot.set_3d_properties([0])
        time_text.set_text(f't = {t_anim[frame]:.1f}s')
        return scatter, target_dot, time_text

    ani = animation.FuncAnimation(
        fig, update, frames=len(t_anim),
        init_func=init, blit=True, interval=50
    )

    writer = animation.PillowWriter(fps=20)
    ani.save(fig_dir / 'positions_3d.gif', writer=writer, dpi=100)
    plt.close()
    print("Saved positions_3d.gif")

make_animation()

print("\nAll 3D plots generated successfully.")