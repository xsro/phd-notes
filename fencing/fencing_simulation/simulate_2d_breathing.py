#!/usr/bin/env python3
"""
2D Breathing Limit Cycle simulation of the first Kou-Chen-Xiang fencing controller.

This script ONLY runs the simulation and saves data to .npz files.
Plotting is handled by plot_2d_breathing.py.

This is Case C from the trichotomy conjecture: the formation exhibits a
breathing limit cycle where pairwise distances oscillate periodically
without converging to a fixed radius.

Controller (Eq. 7 in the paper):
    u_i = φ_i + k₁(x₀ - x_i) + v_i
    v̇_i = k₂(x₀ - x_i)

Outputs:
    - data/simulate_2d_breathing.npz : simulation data
"""

import numpy as np
from scipy.integrate import solve_ivp
from pathlib import Path
from scipy.fft import fft, fftfreq
from scipy.signal import find_peaks
import warnings
warnings.filterwarnings('ignore')

# ============================================================
# Parameters — baseline breathing case (Case C)
# ============================================================
N = 5                   # number of vehicles
d_col = 5.0             # collision distance
mu = 9.0                # sensing radius
k1 = 0.5                # attractive gain
k2 = 0.5                # observer gain
v0 = np.array([1.0, 0.0])  # target velocity vector

# Initial conditions from case3.py (the counterexample that excites breathing)
XI0 = np.array([
    [-10.584563, -12.532987],
    [ -5.382648,   5.904905],
    [ -0.177347,   9.175920],
    [ -7.352506,  -4.805249],
    [ -6.288338,  12.435830]
])
VT0 = np.array([
    [ 3.528048, -1.274511],
    [-0.511988, -1.485443],
    [ 1.972071, -3.679895],
    [-3.460489, -0.767699],
    [-2.039248,  2.761800]
])

x0_init = XI0.copy()
# VT0 is in target frame (observer state minus target velocity)
# In world frame: v_i(0) = VT0_i + v0
v_init = VT0.copy() + v0

x_target0 = np.array([0.0, 0.0])

# Simulation settings
T_MAX = 3000.0          # Long enough to reach the limit cycle (matching breathing analysis)
DT = 0.05
T_ANIM = 40.0           # Show several breathing periods (~2.5s each)

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
print("Integrating 2D breathing limit cycle...")
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
x_traj = sol.y[:2*N, :].reshape(N, 2, -1)
v_traj = sol.y[2*N:, :].reshape(N, 2, -1)
x_target_traj = x_target0.reshape(2, 1) + np.outer(v0, t)

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

vel_error = np.linalg.norm(v_traj - v0.reshape(1, 2, 1), axis=1)

# Mean distance (breathing signal)
mean_dist = np.mean(pairwise_dists, axis=0)

# ============================================================
# FFT analysis for breathing period (done once, saved with data)
# ============================================================
mask_lc = t > (T_MAX - 500)
t_lc = t[mask_lc]
mean_dist_lc = mean_dist[mask_lc]
mean_dist_lc_detrend = mean_dist_lc - np.mean(mean_dist_lc)

n_samples = len(t_lc)
dt_sample = t_lc[1] - t_lc[0]
yf = fft(mean_dist_lc_detrend)
xf = fftfreq(n_samples, dt_sample)[:n_samples//2]
power = np.abs(yf[:n_samples//2])
dominant_idx = np.argmax(power[1:]) + 1
f_dominant = xf[dominant_idx]
T_breathing = 1.0 / f_dominant if f_dominant > 0 else float('inf')
print(f"FFT dominant frequency: {f_dominant:.4f} Hz")
print(f"FFT dominant period: {T_breathing:.4f} s")

# Also try find_peaks for comparison
peaks, _ = find_peaks(mean_dist_lc, prominence=0.001, distance=20)
if len(peaks) >= 2:
    T_peaks = np.mean(np.diff(t_lc[peaks]))
    print(f"Peak-based period: {T_peaks:.4f} s")
else:
    T_peaks = None

# ============================================================
# Save data
# ============================================================
np.savez(
    OUT_DIR / 'simulate_2d_breathing.npz',
    t=t,
    x_traj=x_traj,
    v_traj=v_traj,
    x_target_traj=x_target_traj,
    pairwise_dists=pairwise_dists,
    vel_error=vel_error,
    mean_dist=mean_dist,
    T_ANIM=T_ANIM,
    T_breathing=T_breathing,
    f_dominant=f_dominant,
    params=dict(N=N, d_col=d_col, mu=mu, k1=k1, k2=k2, v0=v0, T_MAX=T_MAX, DT=DT)
)
print(f"Saved data to {OUT_DIR / 'simulate_2d_breathing.npz'}")

# ============================================================
# Print summary
# ============================================================
print("\n=== 2D Breathing Limit Cycle Summary ===")
print(f"Parameters: N={N}, d={d_col}, μ={mu}, k₁={k1}, k₂={k2}")
print(f"Initial positions (case3.py counterexample):")
for i in range(N):
    print(f"  Agent {i+1}: ({x0_init[i,0]:.6f}, {x0_init[i,1]:.6f})")
print(f"\nFinal positions (t={t[-1]:.1f}s):")
for i in range(N):
    print(f"  Agent {i+1}: ({x_traj[i, 0, -1]:.3f}, {x_traj[i, 1, -1]:.3f})")
print(f"\nFinal pairwise distances: {pairwise_dists[:, -1]}")
print(f"Mean final pairwise distance: {np.mean(pairwise_dists[:, -1]):.4f}")
print(f"Final velocity errors: {vel_error[:, -1]}")
print(f"Mean final velocity error: {np.mean(vel_error[:, -1]):.6f}")

# Breathing analysis
mask_detail = t > (T_MAX - 50)
print(f"\nBreathing analysis (last 50s):")
print(f"  Mean distance: {np.mean(mean_dist[mask_detail]):.4f}")
print(f"  Std of mean distance: {np.std(mean_dist[mask_detail]):.4f}")
print(f"  Relative amplitude: {np.std(mean_dist[mask_detail])/np.mean(mean_dist[mask_detail])*100:.2f}%")
if T_breathing:
    print(f"  Breathing period: {T_breathing:.4f} s")
    print(f"  ω_eff/√k₂: {2*np.pi/T_breathing/np.sqrt(k2):.4f}")