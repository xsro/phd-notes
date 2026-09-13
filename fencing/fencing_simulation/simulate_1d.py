#!/usr/bin/env python3
"""
1D simulation of the first Kou-Chen-Xiang fencing controller (no observer).

This script ONLY runs the simulation and saves data to .npz files.
Plotting is handled by plot_1d.py.

Simplified controller:
    u_i = φ_i + k₁(x₀ - x_i)

In 1D, vehicles are constrained to a line. The target moves at constant velocity.
Repulsive forces are pairwise central (along the line).

Outputs:
    - data/simulate_1d.npz : simulation data
"""

import numpy as np
from scipy.integrate import solve_ivp
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

# ============================================================
# Parameters
# ============================================================
N = 5                   # number of vehicles
d_col = 0.5             # collision distance
mu = 2.0                # sensing radius
k1 = 1.0                # attractive gain
k2 = 1.0
v0 = 1.0                # target velocity (scalar)

# Initial conditions
np.random.seed(42)
x_init = np.random.uniform(-3, 3, N)
# Ensure initial pairwise distances > d_col
for i in range(N):
    for j in range(i+1, N):
        while abs(x_init[i] - x_init[j]) <= d_col:
            x_init[j] = np.random.uniform(-3, 3)

x_target0 = 0.0 
v_init = np.zeros((N,))                         # target initial position

# Simulation settings
T_MAX = 60.0
DT = 0.05

# Output directory
OUT_DIR = Path(__file__).parent / "data"
OUT_DIR.mkdir(exist_ok=True)

# ============================================================
# Repulsion function
# ============================================================
def alpha(s):
    """Repulsion strength for distance s in 1D."""
    if s > mu:
        return 0.0
    if s <= d_col:
        warnings.warn(f"Collision risk: s={s:.4f} <= d={d_col}")
        return 1e6
    return 1.0 / (s - d_col) - 1.0 / (mu - d_col)

def compute_phi(x):
    """Compute φ_i for all vehicles in 1D. x shape: (N,). Returns (N,)."""
    phi = np.zeros(N)
    for i in range(N):
        for j in range(N):
            if i == j:
                continue
            xij = x[i] - x[j]
            s = abs(xij)
            if d_col < s <= mu:
                phi[i] += alpha(s) * np.sign(xij)
    return phi

# ============================================================
# Dynamics (no observer — v ≡ 0)
# ============================================================
def dynamics(t, y):
    """
    State vector y: [x_0, ..., x_{N-1}]
    Dynamics: ẋ_i = φ_i + k₁(x₀ - x_i)
    """
    x = y[0:N]
    v = y[N:]
    x_t = x_target0 + v0 * t
    phi = compute_phi(x)
    u = phi + k1 * (x_t - x) + v
    dot_v = k2 * (x_t - x)
    return np.concatenate([u,dot_v])

# ============================================================
# Integrate
# ============================================================
print("Integrating 1D system...")
t_eval = np.arange(0, T_MAX + DT, DT)

x0_init=np.concatenate([x_init,v_init])
sol = solve_ivp(
    dynamics, [0, T_MAX], x0_init,
    t_eval=t_eval,
    rtol=1e-9,
    atol=1e-11,
    max_step=DT
)

t = sol.t
x_traj = sol.y[0:N,:]          # (N, len(t))
x_target_traj = x_target0 + v0 * t

print(f"Integrated {len(t)} time steps.")

# ============================================================
# Compute metrics
# ============================================================
# Pairwise distances
pairwise_dists = np.zeros((N * (N - 1) // 2, len(t)))
idx = 0
for i in range(N):
    for j in range(i + 1, N):
        pairwise_dists[idx] = np.abs(x_traj[i] - x_traj[j])
        idx += 1

# Velocity: ẋ_i = φ_i + k₁(x₀ - x_i)
vel_traj = np.zeros((N, len(t)))
for k in range(len(t)):
    vel_traj[:, k] = compute_phi(x_traj[:, k]) + k1 * (x_target_traj[k] - x_traj[:, k])
vel_error = np.abs(vel_traj - v0)  # (N, len(t))

# ============================================================
# Save data
# ============================================================
np.savez(
    OUT_DIR / 'simulate_1d.npz',
    t=t,
    x_traj=x_traj,
    x_target_traj=x_target_traj,
    pairwise_dists=pairwise_dists,
    vel_error=vel_error,
    vel_traj=vel_traj,
    params=dict(N=N, d_col=d_col, mu=mu, k1=k1, v0=v0, T_MAX=T_MAX, DT=DT)
)
print(f"Saved data to {OUT_DIR / 'simulate_1d.npz'}")

# ============================================================
# Print summary
# ============================================================
print("\n=== 1D Simulation Summary (no observer) ===")
print(f"Parameters: N={N}, d={d_col}, μ={mu}, k₁={k1}, v₀={v0}")
print(f"Initial positions: {x0_init}")
print(f"Final positions (t={t[-1]:.1f}s): {x_traj[:, -1]}")
print(f"Final pairwise distances: {pairwise_dists[:, -1]}")
print(f"Final velocity errors: {vel_error[:, -1]}")
print(f"Mean final velocity error: {np.mean(vel_error[:, -1]):.6f}")

# Check if target is fenced
x_min_final = np.min(x_traj[:, -1])
x_max_final = np.max(x_traj[:, -1])
target_final = x_target_traj[-1]
fenced = x_min_final <= target_final <= x_max_final
print(f"Target fenced: {fenced} (target={target_final:.3f}, range=[{x_min_final:.3f}, {x_max_final:.3f}])")