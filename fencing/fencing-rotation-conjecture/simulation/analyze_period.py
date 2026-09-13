"""
Deeper analysis of the Breathing Limit Cycle period.

Explores:
1. The range of the breathing motion (min/max radius)
2. Relationship with the effective stiffness of the repulsive potential
3. Whether the period is determined by √k₂ or by the repulsion geometry
4. Parameter scan: vary k₁, k₂, d, μ to see how period scales
"""

import os, sys
import numpy as np
from scipy.signal import find_peaks
from scipy.optimize import brentq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fencing_ode import integrate, evaluate, alpha

# ── Parameters (Case C) ──────────────────────────────────────────────────────
d, mu, k1, k2 = 5.0, 9.0, 0.5, 0.5
N = 5

XI0 = np.array([[-10.584563, -12.532987],
                [-5.382648,   5.904905],
                [-0.177347,   9.17592 ],
                [-7.352506,  -4.805249],
                [-6.288338,  12.43583 ]])

VT0 = np.array([[ 3.528048, -1.274511],
                [-0.511988, -1.485443],
                [ 1.972071, -3.679895],
                [-3.460489, -0.767699],
                [-2.039248,  2.7618  ]])

# ── Integrate ────────────────────────────────────────────────────────────────
tf = 3000.0
print(f"Integrating to t={tf} ...")
rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, tf)

# ── High-resolution evaluation ───────────────────────────────────────────────
t_start = 500.0
t_end = 3000.0
dt = 0.1
t_eval = np.arange(t_start, t_end + dt, dt)
pos_all, vel_all = evaluate(sol, N, t_eval)

# ── Mean radius analysis ─────────────────────────────────────────────────────
radii = np.linalg.norm(pos_all, axis=2)
mean_r = np.mean(radii, axis=1)
r_min = np.min(mean_r)
r_max = np.max(mean_r)
r_avg = np.mean(mean_r)
print(f"\n── Radius statistics (t ≥ {t_start}) ──")
print(f"  Mean radius: min = {r_min:.4f}, max = {r_max:.4f}, avg = {r_avg:.4f}")
print(f"  Breathing amplitude: {(r_max - r_min):.4f}  ({(r_max - r_min)/r_avg*100:.1f}%)")

# ── Individual agent radii ───────────────────────────────────────────────────
print(f"\n── Individual agent radii (t ≥ {t_start}) ──")
for i in range(N):
    r_i = radii[:, i]
    print(f"  Agent {i}: min = {np.min(r_i):.4f}, max = {np.max(r_i):.4f}, "
          f"mean = {np.mean(r_i):.4f}, std = {np.std(r_i):.4f}")

# ── Effective stiffness of the repulsive potential ───────────────────────────
# The radial force on one agent (from the repulsion) is roughly:
#   F_rep(r) ≈ α(r)  (for a regular polygon with nearest-neighbor distance s ≈ r)
# The effective spring constant at the mean radius: k_eff = dα/ds * ds/dr
# But the actual motion is more complex due to many-body interactions.
# Let's compute the mean distance between agents and the repulsion at that distance

dists = np.zeros((len(t_eval), N, N))
for i in range(N):
    for j in range(N):
        if i != j:
            dij = np.linalg.norm(pos_all[:, i, :] - pos_all[:, j, :], axis=1)
            dists[:, i, j] = dij

mean_gap = np.mean(dists[:, 0, 1])  # approximate
min_gap = np.min(dists)
print(f"\n── Inter-agent distances ──")
print(f"  Mean nearest gap: {mean_gap:.4f}")
print(f"  Min gap: {min_gap:.4f}")

# Compute α(s) and α'(s) at the mean gap
s_mean = mean_gap
alpha_val = alpha(s_mean, d, mu)
print(f"  α({s_mean:.4f}) = {alpha_val:.4f}")

# Derivative: α'(s) = -1/(s-d)^2 for s ≤ μ
if s_mean <= mu:
    alpha_prime = -1.0 / (s_mean - d)**2
    print(f"  α'({s_mean:.4f}) = {alpha_prime:.4f}")
    # Effective frequency from radial stiffness
    # For a single particle in the repulsive potential: ω_eff² ≈ -α'(s_mean) 
    # (minus because α' is negative, giving positive stiffness)
    omega_stiff = np.sqrt(-alpha_prime)
    print(f"  ω_stiff = √(-α') = {omega_stiff:.4f}")

# ── Period detection ─────────────────────────────────────────────────────────
peaks, _ = find_peaks(mean_r, prominence=0.01, distance=10)
peak_times = t_eval[peaks]
periods = np.diff(peak_times)
T_mean = np.mean(periods)
T_std = np.std(periods)
T_natural = 2*np.pi/np.sqrt(k2)

omega_eff = 2*np.pi / T_mean

print(f"\n── Period analysis ──")
print(f"  T (observed) = {T_mean:.6f} ± {T_std:.6f}")
print(f"  T_n = 2π/√k₂ = {T_natural:.6f}")
print(f"  ω_eff = 2π/T = {omega_eff:.6f}")
print(f"  √k₂ = {np.sqrt(k2):.6f}")
print(f"  ω_eff / √k₂ = {omega_eff/np.sqrt(k2):.6f}")
print(f"  T_n / T = {T_natural/T_mean:.6f}")

# Check if ω_eff is related to a harmonic of √k₂
for n in range(1, 10):
    ratio = omega_eff / (np.sqrt(k2) * n)
    if abs(ratio - 1) < 0.05:
        print(f"  ω_eff ≈ {n}·√k₂ (harmonic {n})")
    ratio2 = omega_eff * n / np.sqrt(k2)
    if abs(ratio2 - 1) < 0.05:
        print(f"  ω_eff ≈ √k₂/{n} (subharmonic 1/{n})")

# Check if ω_eff relates to the repulsive stiffness
if s_mean <= mu:
    omega_stiff = np.sqrt(-alpha_prime)
    print(f"\n  ω_stiff = √(-α'(s)) = {omega_stiff:.6f}")
    print(f"  ω_eff / ω_stiff = {omega_eff/omega_stiff:.6f}")

# ── Fourier analysis ─────────────────────────────────────────────────────────
from scipy.fft import rfft, rfftfreq
n_fft = len(mean_r)
freqs = rfftfreq(n_fft, d=dt)
spectrum = np.abs(rfft(mean_r - np.mean(mean_r)))
# Find dominant frequency (excluding DC)
peak_idx = np.argmax(spectrum[1:]) + 1
f_dom = freqs[peak_idx]
T_dom = 1.0 / f_dom
print(f"\n── Fourier analysis ──")
print(f"  Dominant frequency: f = {f_dom:.6f} Hz")
print(f"  Dominant period: T = {T_dom:.6f}")
print(f"  ω_dom = 2πf = {2*np.pi*f_dom:.6f}")

# Show top 5 peaks
top5 = np.argsort(spectrum[1:])[-5:] + 1
print(f"  Top 5 frequencies (Hz): {freqs[top5]}")
print(f"  Corresponding periods: {1.0/freqs[top5]}")

# ── Save results ─────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
out_path = os.path.join(repo_root, "data", "period_analysis.txt")
with open(out_path, "w") as f:
    f.write(f"Breathing Limit Cycle — Period Analysis\n")
    f.write(f"======================================\n\n")
    f.write(f"Parameters: k1={k1}, k2={k2}, d={d}, mu={mu}, N={N}\n\n")
    f.write(f"Radius stats:\n")
    f.write(f"  min mean r = {r_min:.4f}, max mean r = {r_max:.4f}, avg = {r_avg:.4f}\n")
    f.write(f"  breathing amplitude = {r_max-r_min:.4f} ({100*(r_max-r_min)/r_avg:.1f}%)\n\n")
    f.write(f"Period:\n")
    f.write(f"  T = {T_mean:.6f} ± {T_std:.6f}\n")
    f.write(f"  T_n = 2π/√k2 = {T_natural:.6f}\n")
    f.write(f"  ω_eff = {omega_eff:.6f}\n")
    f.write(f"  ω_eff / √k2 = {omega_eff/np.sqrt(k2):.6f}\n")
    f.write(f"  T_n / T = {T_natural/T_mean:.6f}\n\n")
    f.write(f"Fourier:\n")
    f.write(f"  Dominant f = {f_dom:.6f} Hz, T = {T_dom:.6f}\n")
    f.write(f"  Top 5 f: {freqs[top5]}\n")
    f.write(f"  Top 5 T: {1.0/freqs[top5]}\n")
print(f"\nResults saved → {out_path}")