#!/usr/bin/env python3
"""
Plotting script for 2D breathing limit cycle simulation.
Reads data from data/simulate_2d_breathing.npz and generates figures.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path
from scipy.spatial import ConvexHull

# Global font setting
plt.rc('font', family='Times New Roman')

# Load data
data_dir = Path(__file__).parent.parent / "data"
fig_dir = Path(__file__).parent.parent / "figures"
fig_dir.mkdir(exist_ok=True)

data = np.load(data_dir / 'simulate_2d_breathing.npz', allow_pickle=True)
t = data['t']
x_traj = data['x_traj']
v_traj = data['v_traj']
x_target_traj = data['x_target_traj']
pairwise_dists = data['pairwise_dists']
vel_error = data['vel_error']
mean_dist = data['mean_dist']
T_ANIM = float(data['T_ANIM'])
T_breathing = float(data['T_breathing'])
f_dominant = float(data['f_dominant'])
params = data['params'].item()

N = params['N']
d_col = params['d_col']
mu = params['mu']
k1 = params['k1']
k2 = params['k2']
v0 = params['v0']
T_MAX = params['T_MAX']

print("Generating 2D breathing plots...")

# ============================================================
# Plot 1: Pairwise distances (focus on late-time behavior)
# ============================================================
fig, ax = plt.subplots(figsize=(12, 5))
mask_detail = t > (T_MAX - 50)
for idx in range(pairwise_dists.shape[0]):
    ax.plot(t[mask_detail], pairwise_dists[idx, mask_detail],
            alpha=0.7, linewidth=0.8, label=f'd_{idx+1}')
ax.axhline(y=d_col, color='r', linestyle='--', alpha=0.5, label=f'd={d_col}')
ax.axhline(y=mu, color='g', linestyle='--', alpha=0.5, label=f'μ={mu}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Distance')
ax.set_title('2D Breathing Limit Cycle: Pairwise Vehicle Distances (last 50s)', fontsize=14)
ax.legend(loc='upper right', fontsize=8, ncol=3)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'distances_2d_breathing.png', dpi=150)
plt.close()
print("Saved distances_2d_breathing.png")

# ============================================================
# Plot 2: Breathing signal (mean distance oscillation)
# ============================================================
mask_lc = t > (T_MAX - 500)
t_lc = t[mask_lc]
mean_dist_lc = mean_dist[mask_lc]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# Mean distance
ax1.plot(t_lc, mean_dist_lc, 'b-', linewidth=1.0)
ax1.axhline(y=np.mean(mean_dist_lc), color='k', linestyle='--', alpha=0.5,
            label=f'mean = {np.mean(mean_dist_lc):.3f}')
ax1.set_ylabel('Mean distance')
ax1.set_title('2D Breathing Limit Cycle: Breathing Signal (Mean Pairwise Distance)', fontsize=14)
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

# Amplitude (deviation from mean)
amp = mean_dist_lc - np.mean(mean_dist_lc)
ax2.plot(t_lc, amp, 'r-', linewidth=1.0)
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Deviation from mean')
ax2.set_title(f'Breathing Amplitude (peak-to-peak = {2*np.std(amp)*np.sqrt(2):.4f})', fontsize=14)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
fig.savefig(fig_dir / 'breathing_signal.png', dpi=150)
plt.close()
print("Saved breathing_signal.png")

# ============================================================
# Plot 3: Velocity error norm
# ============================================================
fig, ax = plt.subplots(figsize=(10, 5))
for i in range(N):
    ax.plot(t[mask_detail], vel_error[i, mask_detail],
            alpha=0.7, linewidth=0.8, label=f'Agent {i+1}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('|v_i - v₀|')
ax.set_title('2D Breathing Limit Cycle: Velocity Error Norm per Agent (last 50s)', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'vel_error_2d_breathing.png', dpi=150)
plt.close()
print("Saved vel_error_2d_breathing.png")

# ============================================================
# Animation: Positions over time
# ============================================================
def make_animation():
    t_anim = np.arange(0, T_ANIM + 0.05, 0.05)
    t_indices = [np.argmin(np.abs(t - tt)) for tt in t_anim]

    fig, ax = plt.subplots(figsize=(10, 10))

    margin = 12.0
    ax.set_xlim(-margin, margin)
    ax.set_ylim(-margin, margin)
    ax.set_aspect('equal')
    ax.set_xlabel('x (target-relative)')
    ax.set_ylabel('y (target-relative)')
    ax.set_title('2D Breathing Limit Cycle: Agent Positions', fontsize=14)
    ax.grid(True, alpha=0.3)

    # Target marker (always at origin)
    target_dot = ax.plot([], [], 'ro', markersize=10, label='Target')[0]

    # Agent scatter
    scatter = ax.scatter([], [], s=150, c='blue', alpha=0.7, edgecolors='darkblue')

    # Formation center trail (shows the breathing oscillation)
    center_trail, = ax.plot([], [], 'g-', alpha=0.4, linewidth=1.5,
                            label='Formation center')

    time_text = ax.text(0.98, 0.95, '', transform=ax.transAxes,
                        ha='right', fontsize=14)

    trail_x, trail_y = [], []

    def init():
        scatter.set_offsets(np.empty((0, 2)))
        target_dot.set_data([0], [0])
        center_trail.set_data([], [])
        time_text.set_text('')
        return scatter, target_dot, center_trail, time_text

    def update(frame):
        idx = t_indices[frame]
        rel_positions = x_traj[:, :, idx] - x_target_traj[:, idx].reshape(1, 2)
        scatter.set_offsets(rel_positions)
        target_dot.set_data([0], [0])

        center = np.mean(rel_positions, axis=0)
        trail_x.append(center[0])
        trail_y.append(center[1])
        center_trail.set_data(trail_x, trail_y)

        time_text.set_text(f't = {t_anim[frame]:.1f}s')
        return scatter, target_dot, center_trail, time_text

    ani = animation.FuncAnimation(
        fig, update, frames=len(t_anim),
        init_func=init, blit=True, interval=50
    )

    writer = animation.PillowWriter(fps=20)
    ani.save(fig_dir / 'positions_2d_breathing.gif', writer=writer, dpi=100)
    plt.close()
    print("Saved positions_2d_breathing.gif")

make_animation()

# ============================================================
# Plot 4: Steady-state snapshot grid (t = 30, 31, 32 s)
# ============================================================
snapshot_times = [30, 31, 32]
snapshot_indices = [np.argmin(np.abs(t - t_target)) for t_target in snapshot_times]

fig, axes = plt.subplots(1, 3, figsize=(18, 6))

for k, (t_target, idx) in enumerate(zip(snapshot_times, snapshot_indices)):
    ax = axes[k]
    rel_pos = x_traj[:, :, idx] - x_target_traj[:, idx].reshape(1, 2)

    # Target at origin
    ax.plot(0, 0, 'ro', markersize=12, label='Target', zorder=5)

    # Vehicles
    for i in range(N):
        ax.plot(rel_pos[i, 0], rel_pos[i, 1], 'o', markersize=10,
                color='blue', alpha=0.8, markeredgecolor='darkblue', markeredgewidth=1.5)
        ax.annotate(f'{i+1}', (rel_pos[i, 0], rel_pos[i, 1]),
                   textcoords="offset points", xytext=(6, 6), fontsize=9)

    # Connect vehicles to form polygon (in order)
    try:
        hull = ConvexHull(rel_pos.T)
        hull_pts = rel_pos[hull.vertices]
        hull_pts = np.vstack([hull_pts, hull_pts[0]])
        ax.plot(hull_pts[:, 0], hull_pts[:, 1], 'g-', alpha=0.4, linewidth=1.5)
    except Exception:
        ax.plot(rel_pos[:, 0], rel_pos[:, 1], 'g-', alpha=0.4, linewidth=1.5)
        ax.plot([rel_pos[-1, 0], rel_pos[0, 0]], [rel_pos[-1, 1], rel_pos[0, 1]],
                'g-', alpha=0.4, linewidth=1.5)

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (target-relative)')
    ax.set_ylabel('y (target-relative)')
    ax.set_title(f't = {t_target}s', fontsize=26)
    ax.set_xlim(-8, 8)
    ax.set_ylim(-8, 8)

plt.tight_layout()
fig.savefig(fig_dir / 'snapshots_2d_breathing.png', dpi=150)
fig.savefig(fig_dir / 'snapshots_2d_breathing.pdf')
plt.close()
print("Saved snapshots_2d_breathing.png and snapshots_2d_breathing.pdf")

print("\nAll 2D breathing plots generated successfully.")