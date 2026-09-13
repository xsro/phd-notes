#!/usr/bin/env python3
"""
3D simulation of the first Kou-Chen-Xiang fencing controller.

This script ONLY runs the simulation and saves data to .npz files.
Plotting is handled by plot_3d.py.

Controller (Eq. 7 in the paper):
    u_i = φ_i + k₁(x₀ - x_i) + v_i
    v̇_i = k₂(x₀ - x_i)

In 3D, vehicles move in three-dimensional space. The target moves at constant
velocity. Repulsive forces are pairwise central.

Outputs:
    - data/simulate_3d.npz : simulation data
"""

import numpy as np
from scipy.integrate import solve_ivp
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

# ============================================================
# Parameters
# ============================================================
N = 6                   # number of vehicles
d_col = 5.0             # collision distance
mu = 9.0                # sensing radius
k1 = 0.5                # attractive gain
k2 = 0.5                # observer gain
v0 = np.array([1.0, 0.0, 0.0])  # target velocity vector

# Initial conditions: random positions in 3D (spherical distribution)
np.random.seed(42)
x0_init = np.random.randn(N, 3) * 5.0
# Ensure initial pairwise distances > d_col
for i in range(N):
    for j in range(i+1, N):
        while np.linalg.norm(x0_init[i] - x0_init[j]) <= d_col:
            x0_init[j] = np.random.randn(3) * 5.0
v_init = np.zeros((N, 3))

x_target0 = np.array([0.0, 0.0, 0.0])

# Simulation settings
T_MAX = 200.0
DT = 0.05
T_ANIM = 80.0

# Output directory
OUT_DIR = Path(__file__).parent / "data"
OUT_DIR.mkdir(exist_ok=True)

# ============================================================
# Repulsion function
# ============================================================
def alpha(s):
    if s > mu:
        return 0.0
    if s <= d_col:
        warnings.warn(f"Collision risk: s={s:.4f} <= d={d_col}")
        return 1e6
    return 1.0 / (s - d_col) - 1.0 / (mu - d_col)

def compute_phi(x):
    phi = np.zeros((N, 3))
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
    x = y[:3*N].reshape(N, 3)
    v = y[3*N:].reshape(N, 3)
    x_t = x_target0 + v0 * t
    phi = compute_phi(x)
    u = phi + k1 * (x_t - x) + v
    v_dot = k2 * (x_t - x)
    dy = np.zeros(6 * N)
    dy[:3*N] = u.flatten()
    dy[3*N:] = v_dot.flatten()
    return dy

# ============================================================
# Integrate
# ============================================================
print("Integrating 3D system...")
t_eval = np.arange(0, T_MAX + DT, DT)

y0 = np.zeros(6 * N)
y0[:3*N] = x0_init.flatten()
y0[3*N:] = v_init.flatten()

sol = solve_ivp(
    dynamics, [0, T_MAX], y0,
    method='LSODA',
    t_eval=t_eval,
    rtol=1e-9,
    atol=1e-11,
    max_step=DT
)

t = sol.t
x_traj = sol.y[:3*N, :].reshape(N, 3, -1)
v_traj = sol.y[3*N:, :].reshape(N, 3, -1)
x_target_traj = x_target0.reshape(3, 1) + np.outer(v0, t)

print(f"Integrated {len(t)} time steps.")

# ============================================================
# Compute metrics
# ============================================================
pairwise_dists = np.zeros((N * (N - 1) // 2, len(t)))
idx = 0
for i in range(N):
    for j in range(i + 1, N):
        pairwise_dists[idx] = np.linalg.norm(x_traj[i] - x_traj[j], axis=0)
        idx += 1

vel_error = np.linalg.norm(v_traj - v0.reshape(1, 3, 1), axis=1)  # (N, len(t))

# ============================================================
# Save data
# ============================================================
np.savez(
    OUT_DIR / 'simulate_3d.npz',
    t=t,
    x_traj=x_traj,
    v_traj=v_traj,
    x_target_traj=x_target_traj,
    pairwise_dists=pairwise_dists,
    vel_error=vel_error,
    T_ANIM=T_ANIM,
    params=dict(N=N, d_col=d_col, mu=mu, k1=k1, k2=k2, v0=v0, T_MAX=T_MAX, DT=DT)
)
print(f"Saved data to {OUT_DIR / 'simulate_3d.npz'}")

# ============================================================
# Print summary
# ============================================================
print("\n=== 3D Simulation Summary ===")
print(f"Initial positions shape: {x0_init.shape}")
print(f"Final positions (t={t[-1]:.1f}s):")
for i in range(N):
    print(f"  Agent {i+1}: ({x_traj[i, 0, -1]:.3f}, {x_traj[i, 1, -1]:.3f}, {x_traj[i, 2, -1]:.3f})")
print(f"Final pairwise distances: {pairwise_dists[:, -1]}")
print(f"Final velocity errors: {vel_error[:, -1]}")
print(f"Mean final velocity error: {np.mean(vel_error[:, -1]):.6f}")

print(f"\nPlanarity analysis (SVD of final target-relative positions):")
xi_traj = x_traj - x_target_traj.reshape(1, 3, -1)
final_rel = xi_traj[:, :, -1]
centered = final_rel - np.mean(final_rel, axis=0)
U, S, Vt = np.linalg.svd(centered, full_matrices=False)
print(f"  Singular values: {S}")
print(f"  σ₃/σ₁ = {S[2]/S[0]:.6f}")
if S[2] / S[0] < 0.01:
    print("  → Formation is planar (as predicted)")
elif S[2] / S[0] < 0.1:
    print("  → Formation is nearly planar")
else:
    print("  → Formation is not strongly planar yet")

# Check if vehicles track the target
mask = t > (T_MAX - 50)
print(f"\nTarget tracking (last 50s):")
for i in range(N):
    avg = np.mean(xi_traj[i, :, mask], axis=0)
    print(f"  Agent {i+1}: <ξ> = ({avg[0]:.4f}, {avg[1]:.4f}, {avg[2]:.4f}), ||<ξ>|| = {np.linalg.norm(avg):.4f}")