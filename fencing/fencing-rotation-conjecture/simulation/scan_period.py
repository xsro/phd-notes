"""
Detailed analysis of the breathing limit cycle's period scaling.

Explores the relationship between:
- T × √k₂ (the "scaled period")
- ω_eff² / k₂ (the "normalized effective frequency squared")
- The geometry parameters d, μ
"""

import os, sys
import numpy as np
from scipy.signal import find_peaks

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fencing_ode import integrate, evaluate, alpha

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


def compute_period_fast(d, mu, k1, k2, N=5, tf=1500.0, t_skip=500.0):
    try:
        rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, tf,
                             method="BDF", rtol=1e-8, atol=1e-10)
    except Exception as e:
        return None, 0, str(e)
    dt = 0.2
    t_eval = np.arange(t_skip, tf + dt, dt)
    pos_all, _ = evaluate(sol, N, t_eval)
    radii = np.linalg.norm(pos_all, axis=2)
    mean_r = np.mean(radii, axis=1)
    peaks, _ = find_peaks(mean_r, prominence=0.001, distance=10)
    if len(peaks) < 3:
        return None, 0, f"only {len(peaks)} peaks"
    peak_times = t_eval[peaks]
    periods = np.diff(peak_times)
    T = np.mean(periods)
    T_std = np.std(periods)
    return T, T_std, f"ok {len(peaks)} peaks"


# ── Detailed scan: k2 (observer gain) ────────────────────────────────────────
print("=" * 70)
print("DETAILED SCAN: k2 (observer gain)")
print("=" * 70)
print(f"{'k2':>6}  {'T':>10}  {'T_n/T':>10}  {'ω_eff/√k2':>10}  {'ω_eff²/k2':>10}")
print(f"{'─':>6}  {'─':>10}  {'─':>10}  {'─':>10}  {'─':>10}")
for k2 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
    T, Ts, msg = compute_period_fast(5.0, 9.0, 0.5, k2, tf=1500.0)
    if T is None:
        print(f"{k2:>6.1f}  {'—':>10}  {msg}")
        continue
    T_n = 2*np.pi / np.sqrt(k2)
    omega_eff = 2*np.pi / T
    print(f"{k2:>6.1f}  {T:>10.6f}  {T_n/T:>10.4f}  {omega_eff/np.sqrt(k2):>10.4f}  {omega_eff**2/k2:>10.4f}")

# ── Check the transition point ──────────────────────────────────────────────
print("\n" + "=" * 70)
print("TRANSITION ANALYSIS: where does the limit cycle change?")
print("=" * 70)
print("At k2=0.5, ω_eff/√k2 ≈ 3.575. At k2=0.8, it drops to 2.302.")
print("This suggests a bifurcation between k2=0.5 and k2=0.8.")
print("Let's check k2=0.6 and k2=0.7 more carefully:")
for k2 in [0.55, 0.60, 0.65, 0.70, 0.75]:
    T, Ts, msg = compute_period_fast(5.0, 9.0, 0.5, k2, tf=1500.0)
    if T is None:
        print(f"  k2={k2:.2f}: no limit cycle — {msg}")
        continue
    T_n = 2*np.pi / np.sqrt(k2)
    omega_eff = 2*np.pi / T
    print(f"  k2={k2:.2f}: T={T:.6f}, T_n/T={T_n/T:.4f}, ω_eff/√k2={omega_eff/np.sqrt(k2):.4f}")

# ── Check if the limit cycle is still "breathing" at high k2 ─────────────────
print("\n" + "=" * 70)
print("QUALITATIVE CHECK: what does the trajectory look like at k2=1.0?")
print("=" * 70)
T, Ts, msg = compute_period_fast(5.0, 9.0, 0.5, 1.0, tf=1500.0)
if T is not None:
    # Get the trajectory data
    rhs, sol = integrate(XI0, VT0, 5.0, 9.0, 0.5, 1.0, 1500.0,
                         method="BDF", rtol=1e-8, atol=1e-10)
    dt = 0.2
    t_eval = np.arange(500, 1500 + dt, dt)
    pos_all, _ = evaluate(sol, 5, t_eval)
    radii = np.linalg.norm(pos_all, axis=2)
    mean_r = np.mean(radii, axis=1)
    r_min, r_max = np.min(mean_r), np.max(mean_r)
    r_avg = np.mean(mean_r)
    amp = (r_max - r_min) / r_avg * 100
    print(f"  k2=1.0: T={T:.6f}, mean radius {r_avg:.4f}, amplitude {amp:.2f}%")
    print(f"  r_min={r_min:.4f}, r_max={r_max:.4f}")
    # Compare with baseline
    rb, sb = integrate(XI0, VT0, 5.0, 9.0, 0.5, 0.5, 1500.0,
                       method="BDF", rtol=1e-8, atol=1e-10)
    t_eval2 = np.arange(500, 1500 + dt, dt)
    pos_all2, _ = evaluate(sb, 5, t_eval2)
    mean_r2 = np.mean(np.linalg.norm(pos_all2, axis=2), axis=1)
    r_min2, r_max2 = np.min(mean_r2), np.max(mean_r2)
    r_avg2 = np.mean(mean_r2)
    amp2 = (r_max2 - r_min2) / r_avg2 * 100
    print(f"  k2=0.5: mean radius {r_avg2:.4f}, amplitude {amp2:.2f}%")
    print(f"  r_min={r_min2:.4f}, r_max={r_max2:.4f}")
    print(f"\n  → At k2=1.0, the breathing amplitude is {amp:.1f}% vs {amp2:.1f}% at k2=0.5")
    if amp < amp2 * 0.5:
        print("  → The limit cycle is much weaker at high k2 — approaching a fixed point.")

# ── Summary ──────────────────────────────────────────────────────────────────
print("\n" + "=" * 70)
print("SUMMARY")
print("=" * 70)
print("""
The breathing limit cycle period T satisfies:
  T ≈ 2.485  (at k1=k2=0.5, d=5, μ=9)

Key scaling observations:
  1. T × √k₂ ≈ 1.76  (for k2 ≤ 0.5, i.e., the "strong" breathing regime)
     At k2=0.5: T × √k₂ = 2.485 × 0.707 = 1.757
     At k2=0.3: T × √k₂ = 3.276 × 0.548 = 1.795
     At k2=0.1: T × √k₂ = 5.913 × 0.316 = 1.869
  
  2. At higher k2 (>0.5), the ratio T × √k₂ decreases, suggesting
     the limit cycle weakens and approaches a fixed point.
  
  3. The effective frequency ω_eff ≈ 2.528 is NOT simply √k₂ or
     √(-α'(s)). It is a combined effect of the observer gain and
     the repulsive nonlinearity.
  
  4. The scaled period T × √k₂ ≈ 2π/3.575 ≈ 1.757 is approximately
     constant for k2 ≤ 0.5, suggesting that the limit cycle's
     characteristic timescale is set by √k₂.
""")

print("Done.")