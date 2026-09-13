"""
Comprehensive analysis: fill in gaps in the period-parameter relationship.

Scans:
  1. k₁ = k₂ (symmetric case) — verify the 3.58×√k₂ formula
  2. 2D grid (k₁, k₂) — map limit cycle region
  3. R_0 scan (initial radius) — test attractor robustness
  4. Gap-frequency relationship — is ω_eff² = k₂ + C/(s₁-d)²?
  5. Effective potential analysis — derive T from V_eff(r)
"""

import sys, numpy as np
from scipy.signal import find_peaks
from scipy.interpolate import interp1d
from scipy.integrate import trapezoid

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate, alpha

# ── Geometry constants for N=5 ───────────────────────────────────────────────
sin_pi_5 = np.sin(np.pi/5)
sin_2pi_5 = np.sin(2*np.pi/5)
c1 = 0.5877852522924731  # (1 - cos(72°))/(2·sin(36°))
c2 = 0.9510565162951535  # (1 - cos(144°))/(2·sin(72°))

# ── Initial conditions from case3.py ─────────────────────────────────────────
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


def analyze_trajectory(k1, k2, d=5.0, mu=9.0, tf=1500.0, t_skip=500.0):
    """Integrate and analyze the trajectory. Returns dict of results."""
    rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, tf,
                         method="BDF", rtol=1e-8, atol=1e-10)
    
    dt = 0.1
    t_eval = np.arange(t_skip, tf + dt, dt)
    n_frames = len(t_eval)
    if n_frames < 10:
        return None
    
    pos_all, vel_all = evaluate(sol, 5, t_eval)
    
    # Mean radius
    r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    r_avg = np.mean(r)
    r_min = np.min(r)
    r_max = np.max(r)
    amp = (r_max - r_min) / r_avg * 100 if r_avg > 0 else 0
    
    # Detect peaks for period
    peaks, _ = find_peaks(r, prominence=0.001, distance=10)
    if len(peaks) < 3:
        return {
            'has_lc': False,
            'r_avg': r_avg,
            'amp': amp,
            'r_min': r_min, 'r_max': r_max,
            'T': None, 'omega_eff': None, 'ratio': None,
        }
    
    periods = np.diff(t_eval[peaks])
    T = np.mean(periods)
    T_std = np.std(periods)
    omega_eff = 2*np.pi / T
    ratio = omega_eff / np.sqrt(k2) if k2 > 0 else 0
    
    # Gap analysis
    s1 = 2 * r * sin_pi_5
    s1_avg = np.mean(s1)
    s1_min = 2 * r_min * sin_pi_5
    s1_max = 2 * r_max * sin_pi_5
    gap_avg = s1_avg - d
    gap_min = s1_min - d
    gap_max = s1_max - d
    
    # Linearized stiffness at mean radius
    a1p = -1.0/(s1_avg - d)**2 if s1_avg > d and s1_avg <= mu else 0.0
    a2p = -1.0/(2*r_avg*sin_2pi_5 - d)**2 if 2*r_avg*sin_2pi_5 > d and 2*r_avg*sin_2pi_5 <= mu else 0.0
    fp = 4*a1p*sin_pi_5*c1 + 4*a2p*sin_2pi_5*c2
    omega_lin = np.sqrt(k2 - fp) if k2 - fp > 0 else 0
    
    # Stiffness at inner and outer turning points
    a1p_min = -1.0/(s1_min - d)**2 if s1_min > d and s1_min <= mu else 0.0
    a1p_max = -1.0/(s1_max - d)**2 if s1_max > d and s1_max <= mu else 0.0
    fp_min = 4*a1p_min*sin_pi_5*c1
    fp_max = 4*a1p_max*sin_pi_5*c1
    omega_min = np.sqrt(k2 - fp_min) if k2 - fp_min > 0 else 0
    omega_max = np.sqrt(k2 - fp_max) if k2 - fp_max > 0 else 0
    omega_avg_tp = (omega_min + omega_max) / 2
    
    # Minimum distance between any pair (for detecting jamming)
    all_dists = []
    for idx in range(min(50, n_frames)):
        xi = pos_all[-1-idx]  # near end of simulation
        for i in range(5):
            for j in range(i+1, 5):
                all_dists.append(np.linalg.norm(xi[i] - xi[j]))
    min_dist = min(all_dists)
    
    return {
        'has_lc': True,
        'r_avg': r_avg,
        'amp': amp,
        'r_min': r_min, 'r_max': r_max,
        'T': T,
        'T_std': T_std,
        'omega_eff': omega_eff,
        'omega2': omega_eff**2,
        'ratio': ratio,
        'gap_avg': gap_avg,
        'gap_min': gap_min,
        'gap_max': gap_max,
        's1_avg': s1_avg,
        'omega_lin': omega_lin,
        'omega2_lin': omega_lin**2,
        'omega_min': omega_min,
        'omega_max': omega_max,
        'omega_avg_tp': omega_avg_tp,
        'min_dist': min_dist,
        'r': r, 't_eval': t_eval,
    }


# ═══════════════════════════════════════════════════════════════════════════
# SCAN 1: k₁ = k₂ (symmetric case)
# ═══════════════════════════════════════════════════════════════════════════
print("=" * 80)
print("SCAN 1: k₁ = k₂ (symmetric case)")
print("=" * 80)
print(f"\n{'k':>6}  {'T':>8}  {'ω_eff':>8}  {'ω_eff/√k':>10}  {'r_avg':>8}  "
      f"{'amp%':>6}  {'gap':>6}  {'min_dist':>8}  {'notes':>12}")

for k in [0.05, 0.08, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65]:
    res = analyze_trajectory(k, k)
    if res is None:
        print(f"{k:>6.2f}  {'---':>8}  {'---':>8}  {'---':>10}  {'---':>8}  {'---':>6}  {'---':>6}  {'---':>8}  {'fail':>12}")
        continue
    
    if not res['has_lc']:
        notes = 'no LC' if res['min_dist'] > 5.01 else 'jammed'
        print(f"{k:>6.2f}  {'---':>8}  {'---':>8}  {'---':>10}  {res['r_avg']:>8.4f}  "
              f"{res['amp']:>6.2f}  {res['gap_avg']:>6.3f}  {res['min_dist']:>8.4f}  {notes:>12}")
        continue
    
    formula_T = 2*np.pi / (3.575 * np.sqrt(k))
    formula_err = (res['T'] - formula_T) / formula_T * 100
    
    notes = f"err={formula_err:+.1f}%"
    print(f"{k:>6.2f}  {res['T']:>8.4f}  {res['omega_eff']:>8.4f}  {res['ratio']:>10.4f}  "
          f"{res['r_avg']:>8.4f}  {res['amp']:>6.2f}  {res['gap_avg']:>6.3f}  "
          f"{res['min_dist']:>8.4f}  {notes:>12}")


# ═══════════════════════════════════════════════════════════════════════════
# SCAN 2: 2D grid (k₁, k₂) — map limit cycle region
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("SCAN 2: 2D grid (k₁, k₂) — limit cycle region")
print("=" * 80)

k1_grid = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
k2_grid = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]

print(f"\n{'k₁\\k₂':>6}", end='')
for k2 in k2_grid:
    print(f"{k2:>8.1f}", end='')
print()

for k1 in k1_grid:
    print(f"{k1:>6.1f}", end='')
    for k2 in k2_grid:
        res = analyze_trajectory(k1, k2, tf=1200.0, t_skip=400.0)
        if res is None or not res['has_lc']:
            notes = '—' if res is None or res['min_dist'] > 5.01 else 'J'
            print(f"{notes:>8}", end='')
        else:
            T = res['T']
            ratio = res['ratio']
            print(f"{T:>8.3f}", end='')
    print()


# ═══════════════════════════════════════════════════════════════════════════
# SCAN 3: R₀ scan (initial radius)
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("SCAN 3: Initial radius R₀ (k₁=0.5, k₂=0.5)")
print("=" * 80)

def make_initial_config(R0):
    """Create a regular pentagon with radius R0."""
    angles = np.linspace(0, 2*np.pi, 5, endpoint=False)
    XI0 = R0 * np.column_stack([np.cos(angles), np.sin(angles)])
    VT0 = np.sqrt(0.5) * np.column_stack([-XI0[:, 1], XI0[:, 0]])
    return XI0, VT0

print(f"\n{'R₀':>6}  {'T':>8}  {'ω_eff':>8}  {'r_avg':>8}  {'amp%':>6}  {'gap':>6}  {'notes':>12}")

for R0 in [3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0, 15.0, 20.0]:
    x0, v0 = make_initial_config(R0)
    rhs, sol = integrate(x0, v0, 5.0, 9.0, 0.5, 0.5, 1500.0,
                         method="BDF", rtol=1e-8, atol=1e-10)
    dt = 0.1
    t_eval = np.arange(500, 1500 + dt, dt)
    pos_all, vel_all = evaluate(sol, 5, t_eval)
    r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    r_avg = np.mean(r)
    amp = (np.max(r) - np.min(r)) / r_avg * 100
    
    peaks, _ = find_peaks(r, prominence=0.001, distance=10)
    if len(peaks) < 3:
        print(f"{R0:>6.1f}  {'---':>8}  {'---':>8}  {r_avg:>8.4f}  {amp:>6.2f}  "
              f"{np.mean(2*r*np.sin(np.pi/5)-5.0):>6.3f}  {'no LC':>12}")
        continue
    
    periods = np.diff(t_eval[peaks])
    T = np.mean(periods)
    omega_eff = 2*np.pi / T
    gap = np.mean(2*r*np.sin(np.pi/5) - 5.0)
    print(f"{R0:>6.1f}  {T:>8.4f}  {omega_eff:>8.4f}  {r_avg:>8.4f}  {amp:>6.2f}  "
          f"{gap:>6.3f}  {'':>12}")


# ═══════════════════════════════════════════════════════════════════════════
# SCAN 4: Gap-frequency relationship
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("SCAN 4: Gap-frequency relationship")
print("=" * 80)
print("\nTesting: ω_eff² = k₂ + C/(s₁-d)²")

# Collect data from various k₁, k₂ combinations
print(f"\n{'k₁':>6}  {'k₂':>6}  {'gap':>6}  {'ω²_obs':>8}  {'ω²_lin':>8}  {'C/(gap)²':>10}  {'C/(gap)':>10}  {'C':>10}")

for k1, k2 in [(0.35, 0.5), (0.4, 0.5), (0.45, 0.5), (0.5, 0.5), (0.55, 0.5), (0.6, 0.5),
               (0.5, 0.1), (0.5, 0.3), (0.5, 0.4), (0.5, 0.45)]:
    res = analyze_trajectory(k1, k2)
    if res is None or not res['has_lc']:
        continue
    
    g = res['gap_avg']
    omega2_obs = res['omega2']
    omega2_lin = k2 + 1.382 / g**2  # 1.382 = 4·sin(π/5)·c₁
    C_inv_sq = 1.382 / g**2
    C_inv = 2.0 / g
    C = omega2_obs - k2
    
    print(f"{k1:>6.2f}  {k2:>6.2f}  {g:>6.3f}  {omega2_obs:>8.4f}  {omega2_lin:>8.4f}  "
          f"{C_inv_sq:>10.2f}  {C_inv:>10.2f}  {C:>10.4f}")


# ═══════════════════════════════════════════════════════════════════════════
# SCAN 5: Effective potential — derive T from V_eff(r)
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("SCAN 5: Effective potential derivation of T")
print("=" * 80)
print("\nComparing the observed period with the period derived from the")
print("effective potential V_eff(r) = ∫(k₂·r - F_rep(r)) dr")

for k1 in [0.35, 0.40, 0.45, 0.50, 0.55, 0.60]:
    res = analyze_trajectory(k1, 0.5)
    if res is None or not res['has_lc']:
        continue
    
    r = res['r']
    t = res['t_eval']
    T_obs = res['T']
    
    # Compute radial velocity dr/dt
    dr = np.gradient(r, t)
    
    # Compute the effective repulsive force from the trajectory
    # r̈ + k₁·ṙ + k₂·r = F_rep_eff(r)
    d2r = np.gradient(dr, t)
    F_rep = d2r + k1 * dr + 0.5 * r
    
    # Effective conservative force: F_cons = F_rep - k₂·r
    F_cons = F_rep - 0.5 * r  # = r̈ + k₁·ṙ
    
    # Sort by r for integration
    idx = np.argsort(r)
    r_sorted = r[idx]
    F_cons_sorted = F_cons[idx]
    
    # Remove duplicates and downsample
    _, unique_idx = np.unique(np.round(r_sorted, 4), return_index=True)
    r_u = r_sorted[np.sort(unique_idx)]
    F_u = F_cons_sorted[np.sort(unique_idx)]
    
    if len(r_u) < 10:
        continue
    
    # Effective potential: V_eff(r) = -∫ F_cons dr
    # Manual cumulative trapezoidal integration
    V_eff = -np.concatenate([[0], np.cumsum((F_u[1:] + F_u[:-1]) / 2 * np.diff(r_u))])
    V_eff -= np.min(V_eff)  # shift so minimum is at 0
    
    # Interpolate V_eff
    V_interp = interp1d(r_u, V_eff, bounds_error=False, fill_value='extrapolate')
    
    # Compute total energy of the trajectory
    KE = 0.5 * dr**2  # kinetic energy of the radial mode
    PE = V_interp(r)
    E_total = KE + PE
    E_mean = np.mean(E_total)
    
    # Find turning points
    r_low = np.min(r)
    r_high = np.max(r)
    
    # Try to find where V_eff(r) = E_mean
    # Use the trajectory itself to find the turning points
    # (the points where dr ≈ 0)
    cross_low = np.argmin(np.abs(r - r_low))
    cross_high = np.argmin(np.abs(r - r_high))
    
    r_left = r_low
    r_right = r_high
    
    # Period from the effective potential
    r_grid = np.linspace(r_left, r_right, 2000)
    V_grid = V_interp(r_grid)
    integrand = 1.0 / np.sqrt(2 * np.maximum(E_mean - V_grid, 1e-16))
    T_from_V = 2 * np.trapz(integrand, r_grid)
    
    # Compare
    err = (T_from_V - T_obs) / T_obs * 100
    print(f"  k₁={k1:.2f}: T_obs={T_obs:.4f}, T_from_V={T_from_V:.4f}, err={err:+.2f}%")

print("\nDone.")