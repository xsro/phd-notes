#!/usr/bin/env python3
"""
Plotting script for 2D simulation (rigid rotation).
Reads data from data/simulate_2d.npz and generates figures.
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

data = np.load(data_dir / 'simulate_2d.npz', allow_pickle=True)
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

print("Generating 2D plots...")

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
ax.set_title('2D Fencing Controller: Pairwise Vehicle Distances', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'distances_2d.png', dpi=150)
plt.close()
print("Saved distances_2d.png")

# ============================================================
# Plot 2: Velocity error norm
# ============================================================
fig, ax = plt.subplots(figsize=(10, 5))
for i in range(N):
    ax.plot(t, vel_error[i], alpha=0.7, linewidth=0.8, label=f'Agent {i+1}')
ax.set_xlabel('Time [s]')
ax.set_ylabel('|v_i - v₀|')
ax.set_title('2D Fencing Controller: Velocity Error Norm per Agent', fontsize=14)
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)
plt.tight_layout()
fig.savefig(fig_dir / 'vel_error_2d.png', dpi=150)
plt.close()
print("Saved vel_error_2d.png")

# ============================================================
# Plot 3: Steady-state snapshot grid (t = 30, 31, 32 s)
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
        hull_pts = np.vstack([hull_pts, hull_pts[0]])  # close polygon
        ax.plot(hull_pts[:, 0], hull_pts[:, 1], 'g-', alpha=0.4, linewidth=1.5)
    except Exception:
        # Collinear points — draw line only
        ax.plot(rel_pos[:, 0], rel_pos[:, 1], 'g-', alpha=0.4, linewidth=1.5)
        ax.plot([rel_pos[-1, 0], rel_pos[0, 0]], [rel_pos[-1, 1], rel_pos[0, 1]],
                'g-', alpha=0.4, linewidth=1.5)

    # Equal aspect, grid
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (target-relative)')
    ax.set_ylabel('y (target-relative)')
    ax.set_title(f't = {t_target}s', fontsize=26)
    ax.set_xlim(-8, 8)
    ax.set_ylim(-8, 8)

plt.tight_layout()
fig.savefig(fig_dir / 'snapshots_2d.png', dpi=150)
fig.savefig(fig_dir / 'snapshots_2d.pdf')
plt.close()
print("Saved snapshots_2d.png and snapshots_2d.pdf")

# ============================================================
# Animation: Positions over time
# ============================================================
def make_animation():
    """Create GIF animation of vehicle positions in 2D."""
    t_anim = np.arange(0, T_ANIM + 0.05, 0.05)
    t_indices = [np.argmin(np.abs(t - tt)) for tt in t_anim]

    fig, ax = plt.subplots(figsize=(10, 10))

    # Set up the plot — camera follows target
    margin = 10.0
    ax.set_xlim(-margin, margin)
    ax.set_ylim(-margin, margin)
    ax.set_aspect('equal')
    ax.set_xlabel('x (target-relative)')
    ax.set_ylabel('y (target-relative)')
    ax.set_title('2D Fencing Controller: Agent Positions', fontsize=14)
    ax.grid(True, alpha=0.3)

    # Target marker (always at origin, fixed)
    target_dot = ax.plot([], [], 'ro', markersize=10, label='Target')[0]

    # Agent scatter
    scatter = ax.scatter([], [], s=150, c='blue', alpha=0.7, edgecolors='darkblue')

    # Time text
    time_text = ax.text(0.98, 0.95, '', transform=ax.transAxes, ha='right', fontsize=14)

    # Target trail (in target-relative frame, shows rotation history)
    trail_x, trail_y = [], []
    target_trail, = ax.plot([], [], 'r-', alpha=0.3, linewidth=1)

    def init():
        scatter.set_offsets(np.empty((0, 2)))
        target_dot.set_data([0], [0])
        target_trail.set_data([], [])
        time_text.set_text('')
        return scatter, target_dot, target_trail, time_text

    def update(frame):
        idx = t_indices[frame]
        # Use target-relative positions so target is always at origin
        rel_positions = x_traj[:, :, idx] - x_target_traj[:, idx].reshape(1, 2)
        scatter.set_offsets(rel_positions)

        # Target is always at (0,0) — draw a marker
        target_dot.set_data([0], [0])

        # Record the formation center in target-relative frame
        center = np.mean(rel_positions, axis=0)
        trail_x.append(center[0])
        trail_y.append(center[1])
        target_trail.set_data(trail_x, trail_y)

        time_text.set_text(f't = {t_anim[frame]:.1f}s')
        return scatter, target_dot, target_trail, time_text

    ani = animation.FuncAnimation(
        fig, update, frames=len(t_anim),
        init_func=init, blit=True, interval=50
    )

    writer = animation.PillowWriter(fps=20)
    ani.save(fig_dir / 'positions_2d.gif', writer=writer, dpi=100)
    plt.close()
    print("Saved positions_2d.gif")

make_animation()

print("\nAll 2D plots generated successfully.")