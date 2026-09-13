#!/usr/bin/env python3
"""
2D simulation of the first Kou-Chen-Xiang fencing controller.

This script ONLY runs the simulation and saves data to .npz files.
Plotting is handled by plot_2d.py.

Controller (Eq. 7 in the paper):
    u_i = φ_i + k₁(x₀ - x_i) + v_i
    v̇_i = k₂(x₀ - x_i)

In 2D, vehicles move in the plane. The target moves at constant velocity.
Repulsive forces are pairwise central. This is the classic case where
the controller induces rigid rotation about the target.

Outputs:
    - data/simulate_2d.npz : simulation data
"""

import numpy as np
from scipy.integrate import solve_ivp
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

# ============================================================
# Parameters (from the paper)
# ============================================================
N = 6                   # number of vehicles
d_col = 5.0             # collision distance
mu = 9.0                # sensing radius
k1 = 0.5                # attractive gain
k2 = 0.5                # observer gain
v0 = np.array([1.0, 0.0])  # target velocity vector

# Initial conditions: regular hexagon around origin
np.random.seed(42)
theta = np.linspace(0, 2 * np.pi, N, endpoint=False)
R_init = 8.0
x0_init = np.column_stack([R_init * np.cos(theta), R_init * np.sin(theta)])
v_init = np.zeros((N, 2))

x_target0 = np.array([0.0, 0.0])

# Simulation settings
T_MAX = 80.0
DT = 0.05
T_ANIM = 40.0

# Output directory
OUT_DIR = Path(__file__).parent / "data"
OUT_DIR.mkdir(exist_ok=True)

# ============================================================
# Repulsion function
# ============================================================
def alpha(s):
    """Repulsion strength for distance s."""
    if s > mu:
        return 0.0
    if s <= d_col:
        warnings.warn(f"Collision risk: s={s:.4f} <= d={d_col}")
        return 1e6
    return 1.0 / (s - d_col) - 1.0 / (mu - d_col)

def compute_phi(x):
    """Compute φ_i for all vehicles in 2D. x shape: (N, 2). Returns (N, 2)."""
    phi = np.zeros((N, 2))
    for i in range(N):
        for j in range(N):
            if i == j:
                continue
            xij = x[i] - x[j]
            dist = np.linalg.norm(xij)
            if d_col < dist <= mu:
                phi[i] += alpha(dist) * xij / dist
    return phi

# ============================================================
# Dynamics
# ============================================================
def dynamics(t, y):
    """
    State vector y: [x_0, ..., x_{N-1}, v_0, ..., v_{N-1}]
    Each x_i and v_i is 2D. Total length = 4N.
    """
    x = y[:2*N].reshape(N, 2)
    v = y[2*N:].reshape(N, 2)

    x_t = x_target0 + v0 * t

    phi = compute_phi(x)

    u = phi + k1 * (x_t - x) + v
    v_dot = k2 * (x_t - x)

    dy = np.zeros(4 * N)
    dy[:2*N] = u.flatten()
    dy[2*N:] = v_dot.flatten()
    return dy

# ============================================================
# Integrate
# ============================================================
print("Integrating 2D system...")
t_eval = np.arange(0, T_MAX + DT, DT)

y0 = np.zeros(4 * N)
y0[:2*N] = x0_init.flatten()
y0[2*N:] = v_init.flatten()

sol = solve_ivp(
    dynamics, [0, T_MAX], y0,
    method='LSODA',
    t_eval=t_eval,
    rtol=1e-9,
    atol=1e-11,
    max_step=DT
)

t = sol.t
x_traj = sol.y[:2*N, :].reshape(N, 2, -1)   # (N, 2, len(t))
v_traj = sol.y[2*N:, :].reshape(N, 2, -1)   # (N, 2, len(t))
x_target_traj = x_target0.reshape(2, 1) + np.outer(v0, t)  # (2, len(t))

print(f"Integrated {len(t)} time steps.")

# ============================================================
# Compute metrics
# ============================================================

# Pairwise distances
pairwise_dists = np.zeros((N * (N - 1) // 2, len(t)))
idx = 0
for i in range(N):
    for j in range(i + 1, N):
        pairwise_dists[idx] = np.linalg.norm(
            x_traj[i] - x_traj[j], axis=0
        )
        idx += 1

# Velocity error: ||v_i - v0||
vel_error = np.linalg.norm(v_traj - v0.reshape(1, 2, 1), axis=1)  # (N, len(t))

# ============================================================
# Save data
# ============================================================
np.savez(
    OUT_DIR / 'simulate_2d.npz',
    t=t,
    x_traj=x_traj,
    v_traj=v_traj,
    x_target_traj=x_target_traj,
    pairwise_dists=pairwise_dists,
    vel_error=vel_error,
    T_ANIM=T_ANIM,
    params=dict(N=N, d_col=d_col, mu=mu, k1=k1, k2=k2, v0=v0, T_MAX=T_MAX, DT=DT)
)
print(f"Saved data to {OUT_DIR / 'simulate_2d.npz'}")

# ============================================================
# Print summary
# ============================================================
print("\n=== 2D Simulation Summary ===")
print(f"Initial positions shape: {x0_init.shape}")
print(f"Final positions (t={t[-1]:.1f}s):")
for i in range(N):
    print(f"  Agent {i+1}: ({x_traj[i, 0, -1]:.3f}, {x_traj[i, 1, -1]:.3f})")
print(f"Final pairwise distances: {pairwise_dists[:, -1]}")
print(f"Final velocity errors: {vel_error[:, -1]}")
print(f"Mean final velocity error: {np.mean(vel_error[:, -1]):.6f}")

# Check if target is fenced (inside convex hull)
from scipy.spatial import ConvexHull
try:
    hull = ConvexHull(x_traj[:, :, -1].T)
    from matplotlib.path import Path
    hull_path = Path(x_traj[:, :, -1].T[hull.vertices])
    target_final = x_target_traj[:, -1]
    fenced = hull_path.contains_point(target_final)
    print(f"Target fenced: {fenced} (target=({target_final[0]:.3f}, {target_final[1]:.3f}))")
except Exception as e:
    print(f"Target fenced: could not compute ({e})")