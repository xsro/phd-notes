#!/usr/bin/env python3
"""
3D simulation of the first Kou-Chen-Xiang fencing controller with N=30 vehicles.

Optimized version with vectorized repulsion computation for speed.

This script ONLY runs the simulation and saves data to .npz files.
Plotting is handled by plot_3d_n30.py.

Controller (Eq. 7 in the paper):
    u_i = φ_i + k₁(x₀ - x_i) + v_i
    v̇_i = k₂(x₀ - x_i)

In 3D, vehicles move in three-dimensional space. The target moves at constant
velocity. Repulsive forces are pairwise central.

Outputs:
    - data/simulate_3d_n30.npz : simulation data
"""

import numpy as np
from scipy.integrate import solve_ivp
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

# ============================================================
# Parameters
# ============================================================
N = 30                  # number of vehicles
d_col = 5.0             # collision distance
mu = 15.0               # sensing radius (larger for N=30)
k1 = 0.5                # attractive gain
k2 = 0.5                # observer gain
v0 = np.array([1.0, 0.0, 0.0])  # target velocity vector

# Initial conditions: random positions in 3D (spherical distribution)
np.random.seed(42)
x0_init = np.random.randn(N, 3) * 15.0
# Ensure initial pairwise distances > d_col
for i in range(N):
    for j in range(i+1, N):
        while np.linalg.norm(x0_init[i] - x0_init[j]) <= d_col:
            x0_init[j] = np.random.randn(3) * 15.0
v_init = np.zeros((N, 3))

x_target0 = np.array([0.0, 0.0, 0.0])

# Simulation settings
T_MAX = 50.0
DT = 0.2
T_ANIM = 40.0

# Output directory
OUT_DIR = Path(__file__).parent / "data"
OUT_DIR.mkdir(exist_ok=True)

# ============================================================
# Vectorized repulsion function
# ============================================================
def alpha(s):
    """Vectorized alpha function."""
    result = np.zeros_like(s)
    mask = (s > d_col) & (s <= mu)
    result[mask] = 1.0 / (s[mask] - d_col) - 1.0 / (mu - d_col)
    # Collision check
    collision_mask = s <= d_col
    if np.any(collision_mask):
        warnings.warn(f"Collision risk: s <= d={d_col}")
        result[collision_mask] = 1e6
    return result

def compute_phi_vectorized(x):
    """Vectorized computation of phi_i for all vehicles."""
    # x: (N, 3)
    # Compute all pairwise differences: x[i] - x[j]
    diff = x[:, np.newaxis, :] - x[np.newaxis, :, :]  # (N, N, 3)
    dists = np.linalg.norm(diff, axis=2)  # (N, N)
    
    # Mask out self-pairs and pairs outside sensing range
    mask = (dists > d_col) & (dists <= mu)  # (N, N)
    np.fill_diagonal(mask, False)
    
    # Compute alpha values
    alphas = alpha(dists)  # (N, N)
    alphas[~mask] = 0.0
    
    # Compute unit vectors and sum
    # diff / dists gives unit vectors, but need to handle dists=0
    safe_dists = np.where(dists > 0, dists, 1.0)
    unit_vectors = diff / safe_dists[:, :, np.newaxis]  # (N, N, 3)
    
    # phi[i] = sum_j alpha_ij * unit_vector_ij
    phi = np.sum(alphas[:, :, np.newaxis] * unit_vectors, axis=1)  # (N, 3)
    return phi

# ============================================================
# Dynamics
# ============================================================
def dynamics(t, y):
    x = y[:3*N].reshape(N, 3)
    v = y[3*N:].reshape(N, 3)
    x_t = x_target0 + v0 * t
    phi = compute_phi_vectorized(x)
    u = phi + k1 * (x_t - x) + v
    v_dot = k2 * (x_t - x)
    dy = np.zeros(6 * N)
    dy[:3*N] = u.flatten()
    dy[3*N:] = v_dot.flatten()
    return dy

# ============================================================
# Integrate
# ============================================================
print("Integrating 3D system (N=30, vectorized)...")
t_eval = np.arange(0, T_MAX + DT, DT)

y0 = np.zeros(6 * N)
y0[:3*N] = x0_init.flatten()
y0[3*N:] = v_init.flatten()

sol = solve_ivp(
    dynamics, [0, T_MAX], y0,
    method='BDF',
    t_eval=t_eval,
    rtol=1e-6,
    atol=1e-8,
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
print("Computing pairwise distances...")
pairwise_dists = np.zeros((N * (N - 1) // 2, len(t)))
idx = 0
for i in range(N):
    for j in range(i + 1, N):
        pairwise_dists[idx] = np.linalg.norm(x_traj[i] - x_traj[j], axis=0)
        idx += 1

vel_error = np.linalg.norm(v_traj - v0.reshape(1, 3, 1), axis=1)  # (N, len(t))

# ============================================================
# FFT analysis of pairwise distances (steady state)
# ============================================================
print("Performing FFT analysis...")
mask_ss = t > (T_MAX * 0.3)  # last 70% as steady state
t_ss = t[mask_ss]
pd_ss = pairwise_dists[:, mask_ss]
T_ss = len(t_ss)

# Average FFT across all pairwise distances
fft_freqs = np.fft.rfftfreq(T_ss, d=DT)
avg_power = np.zeros(len(fft_freqs))
for pd in pd_ss:
    pd_detrend = pd - np.mean(pd)
    fft = np.fft.rfft(pd_detrend)
    avg_power += np.abs(fft) ** 2
avg_power[0] = 0  # kill DC
dominant_idx = np.argmax(avg_power)
dominant_freq = fft_freqs[dominant_idx]
dominant_period = 1.0 / dominant_freq if dominant_freq > 0.001 else None

# Peak-to-total ratio (how much of the power is in the dominant peak)
total_power = np.sum(avg_power)
peak_ratio = avg_power[dominant_idx] / total_power if total_power > 0 else 0

# ============================================================
# Planarity analysis
# ============================================================
xi_traj = x_traj - x_target_traj.reshape(1, 3, -1)
final_rel = xi_traj[:, :, -1]
centered = final_rel - np.mean(final_rel, axis=0)
U, S, Vt = np.linalg.svd(centered, full_matrices=False)
sigma_ratio = S[2] / S[0] if len(S) >= 3 else 1.0

# ============================================================
# Save data
# ============================================================
np.savez(
    OUT_DIR / 'simulate_3d_n30.npz',
    t=t,
    x_traj=x_traj,
    v_traj=v_traj,
    x_target_traj=x_target_traj,
    pairwise_dists=pairwise_dists,
    vel_error=vel_error,
    T_ANIM=T_ANIM,
    params=dict(N=N, d_col=d_col, mu=mu, k1=k1, k2=k2, v0=v0, T_MAX=T_MAX, DT=DT)
)
print(f"Saved data to {OUT_DIR / 'simulate_3d_n30.npz'}")

# ============================================================
# Print summary
# ============================================================
print("\n=== 3D Simulation Summary (N=30) ===")
print(f"Initial positions shape: {x0_init.shape}")
print(f"Final positions (t={t[-1]:.1f}s):")
for i in range(min(N, 10)):  # print first 10
    print(f"  Agent {i+1}: ({x_traj[i, 0, -1]:.3f}, {x_traj[i, 1, -1]:.3f}, {x_traj[i, 2, -1]:.3f})")
if N > 10:
    print(f"  ... ({N-10} more agents)")
print(f"Final pairwise distances: min={pairwise_dists[:, -1].min():.3f}, max={pairwise_dists[:, -1].max():.3f}, mean={pairwise_dists[:, -1].mean():.3f}")
print(f"Final velocity errors: mean={vel_error[:, -1].mean():.6f}")

print(f"\nPlanarity analysis (SVD of final target-relative positions):")
print(f"  Singular values: {S}")
print(f"  σ₃/σ₁ = {sigma_ratio:.6f}")
if S[2] / S[0] < 0.01:
    print("  → Formation is planar (as predicted)")
elif S[2] / S[0] < 0.1:
    print("  → Formation is nearly planar")
else:
    print("  → Formation is non-planar (3D breathing)")

# Check if vehicles track the target
mask = t > (T_MAX - 50)
print(f"\nTarget tracking (last 50s):")
for i in range(min(N, 5)):
    avg = np.mean(xi_traj[i, :, mask], axis=0)
    print(f"  Agent {i+1}: <ξ> = ({avg[0]:.4f}, {avg[1]:.4f}, {avg[2]:.4f}), ||<ξ>|| = {np.linalg.norm(avg):.4f}")

# Breathing analysis
print(f"\nBreathing analysis (FFT of pairwise distances, steady state):")
print(f"  Dominant frequency: {dominant_freq:.4f} Hz")
print(f"  Dominant period:   {dominant_period:.3f} s")
print(f"  Peak power ratio:  {peak_ratio:.4f}")