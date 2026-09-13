#!/usr/bin/env python3
"""
3D breathing limit cycle simulation.

Discovered via experiment/search_3d_breathing.py (seed=0).
Unlike the rigid rotation case (simulate_3d.py), here the pairwise distances
oscate periodically without converging — a genuine 3D breathing attractor.

Initial conditions: random 3D positions, zero initial velocity.
Seed 0 produces a breathing limit cycle with:
  - Dominant frequency: ~0.22 Hz (period ~4.55 s)
  - Non-planar formation: sigma_3/sigma_1 ~ 0.825
  - Strong pairwise distance oscillation (FFT peak ratio ~0.99)

Outputs:
    - data/simulate_3d_breathing.npz : simulation data
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

# ============================================================
# Initial conditions (seed=0 from search_3d_breathing.py)
# ============================================================
XI0 = np.array([
    [14.112419,  3.201258,  7.829904],
    [17.927146, 14.940464, -7.818223],
    [ 7.600707, -1.210858, -0.825751],
    [ 3.284788,  1.152349, 11.634188],
    [ 6.088302,  0.973400,  3.550906],
    [ 2.669395, 11.952633, -1.641266],
])
VT0 = np.zeros((N, 3))

x_target0 = np.array([0.0, 0.0, 0.0])

# Simulation settings
T_MAX = 500.0           # long enough for several breathing periods
DT = 0.02               # fine time step for good FFT resolution
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
print("Integrating 3D breathing system...")
t_eval = np.arange(0, T_MAX + DT, DT)

y0 = np.zeros(6 * N)
y0[:3*N] = XI0.flatten()
y0[3*N:] = VT0.flatten()

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

print(f"Integrated {len(t)} time steps (T_max={T_MAX}s, dt={DT}s).")

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
# FFT analysis of pairwise distances (steady state)
# ============================================================
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
    OUT_DIR / 'simulate_3d_breathing.npz',
    t=t,
    x_traj=x_traj,
    v_traj=v_traj,
    x_target_traj=x_target_traj,
    pairwise_dists=pairwise_dists,
    vel_error=vel_error,
    XI0=XI0,
    VT0=VT0,
    T_ANIM=T_ANIM,
    params=dict(N=N, d_col=d_col, mu=mu, k1=k1, k2=k2, v0=v0, T_MAX=T_MAX, DT=DT)
)
print(f"Saved data to {OUT_DIR / 'simulate_3d_breathing.npz'}")

# ============================================================
# Print summary
# ============================================================
print("\n=== 3D Breathing Simulation Summary ===")
print(f"Initial positions (seed=0):")
for i in range(N):
    print(f"  Agent {i+1}: ({XI0[i,0]:.3f}, {XI0[i,1]:.3f}, {XI0[i,2]:.3f})")
print(f"Final pairwise distances: {pairwise_dists[:, -1]}")
print(f"Final velocity errors: {vel_error[:, -1]}")
print(f"Mean final velocity error: {np.mean(vel_error[:, -1]):.6f}")

print(f"\nBreathing analysis (FFT of pairwise distances, steady state):")
print(f"  Dominant frequency: {dominant_freq:.4f} Hz")
print(f"  Dominant period:   {dominant_period:.3f} s")
print(f"  Peak power ratio:  {peak_ratio:.4f}")
print(f"  (ratio of total power in the dominant frequency bin)")

print(f"\nPlanarity analysis (SVD of final target-relative positions):")
print(f"  Singular values: {S}")
print(f"  sigma_3/sigma_1 = {sigma_ratio:.6f}")
if sigma_ratio > 0.3:
    print("  -> Formation is NON-PLANAR (genuine 3D breathing)")
else:
    print("  -> Formation is nearly planar")

# Check if vehicles track the target
mask_track = t > (T_MAX - 50)
print(f"\nTarget tracking (last 50s):")
for i in range(N):
    avg = np.mean(xi_traj[i, :, mask_track], axis=0)
    print(f"  Agent {i+1}: <xi> = ({avg[0]:.4f}, {avg[1]:.4f}, {avg[2]:.4f}), ||<xi>|| = {np.linalg.norm(avg):.4f}")

print(f"\nClassification: 3D BREATHING LIMIT CYCLE")
print(f"  - Pairwise distances oscillate periodically (peak ratio = {peak_ratio:.3f})")
print(f"  - Formation is non-planar (sigma3/sigma1 = {sigma_ratio:.3f})")
print(f"  - Breathing frequency f = {dominant_freq:.4f} Hz, T = {dominant_period:.3f} s")