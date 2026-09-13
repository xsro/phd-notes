"""
Focused high-precision 2D scan for clean period-parameter relationships.

Uses LSODA with rtol=1e-8, atol=1e-10, t=800, t_skip=300 for reliable
limit cycle detection. Scans a focused grid of (k₁, k₂) values.
"""

import sys, numpy as np, time
from scipy.signal import find_peaks

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate

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


def scan(k1, k2, d=5.0, mu=9.0):
    """High-precision scan. Returns dict or None."""
    rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, 800.0,
                         method='LSODA', rtol=1e-8, atol=1e-10)
    t_eval = np.arange(300, 800.1, 0.1)
    pos_all, vel_all = evaluate(sol, 5, t_eval)
    r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    r_avg = np.mean(r)
    amp = (np.max(r) - np.min(r)) / r_avg * 100 if r_avg > 0 else 0
    peaks, _ = find_peaks(r, prominence=0.001, distance=10)
    if len(peaks) < 3:
        return None
    periods = np.diff(t_eval[peaks])
    T = np.mean(periods)
    T_std = np.std(periods)
    omega = 2 * np.pi / T
    s1 = 2 * r * np.sin(np.pi / 5)
    gap = np.mean(s1) - d
    return {
        'T': T, 'T_std': T_std, 'omega': omega,
        'omega2': omega**2, 'ratio': omega / np.sqrt(k2),
        'r_avg': r_avg, 'amp': amp, 'gap': gap,
        'min_dist': min(np.linalg.norm(pos_all[-1, i] - pos_all[-1, j])
                        for i in range(5) for j in range(i+1, 5))
    }


# ═══════════════════════════════════════════════════════════════════════════
# FOCUSED GRID: high precision, small set
# ═══════════════════════════════════════════════════════════════════════════
print("=" * 80)
print("FOCUSED HIGH-PRECISION 2D SCAN")
print("=" * 80)

# Key data points: cover the clean region
cases = [
    # k1, k2
    (0.3, 0.3), (0.3, 0.5), (0.3, 0.7),
    (0.4, 0.3), (0.4, 0.5), (0.4, 0.7),
    (0.5, 0.1), (0.5, 0.2), (0.5, 0.3), (0.5, 0.4), (0.5, 0.5), (0.5, 0.6),
    (0.6, 0.3), (0.6, 0.5), (0.6, 0.7),
]

t0 = time.time()
results = {}
for k1, k2 in cases:
    res = scan(k1, k2)
    results[(k1, k2)] = res
    if res:
        print(f"k1={k1:.1f}, k2={k2:.1f}: T={res['T']:.4f} +/-{res['T_std']:.4f}, "
              f"ratio={res['ratio']:.4f}, r_avg={res['r_avg']:.4f}, "
              f"amp={res['amp']:.2f}%, gap={res['gap']:.3f}")
    else:
        print(f"k1={k1:.1f}, k2={k2:.1f}: NO LC")
print(f"Total time: {time.time()-t0:.0f}s")

# ═══════════════════════════════════════════════════════════════════════════
# ANALYSIS: find patterns
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("PATTERN ANALYSIS")
print("=" * 80)

# Filter out anomalous data (large T_std indicates non-stable limit cycle)
clean_results = {k: v for k, v in results.items() if v is not None and v['T_std']/v['T'] < 0.1}

# 1. Ratio vs k1/k2
print("\n--- Ratio omega_eff/sqrt(k2) vs k1/k2 (clean data) ---")
print(f"{'k1':>5} {'k2':>5} {'k1/k2':>7} {'ratio':>8} {'T':>8}")
for (k1, k2), res in sorted(clean_results.items()):
    print(f"{k1:>5.1f} {k2:>5.1f} {k1/k2:>7.2f} {res['ratio']:>8.4f} {res['T']:>8.4f}")

# 2. Linear fit: ratio = a + b*k1 + c*k2
print("\n--- Linear fit: ratio = a + b*k1 + c*k2 (clean data) ---")
k1s = np.array([k[0] for k in clean_results.keys()])
k2s = np.array([k[1] for k in clean_results.keys()])
ratios = np.array([res['ratio'] for res in clean_results.values()])

A = np.column_stack([np.ones_like(k1s), k1s, k2s])
coeffs = np.linalg.lstsq(A, ratios, rcond=None)[0]
print(f"  ratio = {coeffs[0]:.4f} + {coeffs[1]:.4f}*k1 + {coeffs[2]:.4f}*k2")
for (k1, k2), res in clean_results.items():
    pred = coeffs[0] + coeffs[1]*k1 + coeffs[2]*k2
    err = (pred - res['ratio'])/res['ratio']*100
    print(f"  k1={k1:.1f},k2={k2:.1f}: pred={pred:.4f}, obs={res['ratio']:.4f}, err={err:+.2f}%")

# 3. Alternative: ratio = a + b*(k1/k2) + c*k2
print("\n--- Alternative: ratio = a + b*(k1/k2) + c*k2 ---")
k1k2 = k1s / k2s
A = np.column_stack([np.ones_like(k1k2), k1k2, k2s])
coeffs = np.linalg.lstsq(A, ratios, rcond=None)[0]
print(f"  ratio = {coeffs[0]:.4f} + {coeffs[1]:.4f}*(k1/k2) + {coeffs[2]:.4f}*k2")
for (k1, k2), res in clean_results.items():
    pred = coeffs[0] + coeffs[1]*(k1/k2) + coeffs[2]*k2
    err = (pred - res['ratio'])/res['ratio']*100
    print(f"  k1={k1:.1f},k2={k2:.1f}: pred={pred:.4f}, obs={res['ratio']:.4f}, err={err:+.2f}%")

# 4. Check if ratio is approximately constant
mean_ratio = np.mean(ratios)
std_ratio = np.std(ratios)
print(f"\n--- Ratio statistics (clean data) ---")
print(f"  mean ratio = {mean_ratio:.4f} +/- {std_ratio:.4f} ({std_ratio/mean_ratio*100:.1f}%)")
print(f"  range: [{np.min(ratios):.4f}, {np.max(ratios):.4f}]")

# 5. Gap-frequency relationship (clean data)
print("\n--- Gap-frequency (clean data) ---")
print(f"{'k1':>5} {'k2':>5} {'gap':>7} {'omega^2':>8} {'(omega^2-k2)/gap':>10}")
for (k1, k2), res in sorted(clean_results.items()):
    C = res['omega2'] - k2
    g = res['gap']
    print(f"{k1:>5.1f} {k2:>5.1f} {g:>7.4f} {res['omega2']:>8.4f} {C/g:>10.4f}")

# 6. Simple formula: omega_eff^2 = A * k2 (for k1=0.5)
print("\n--- Testing omega_eff^2 = A*k2 (k1=0.5) ---")
k1_05 = clean_results.get((0.5, 0.5), None)
if k1_05:
    A_val = k1_05['omega2'] / 0.5
    print(f"  A = {A_val:.4f} (at k1=0.5, k2=0.5)")
    for (k1, k2), res in clean_results.items():
        if abs(k1 - 0.5) > 0.01: continue
        pred = A_val * k2
        err = (pred - res['omega2'])/res['omega2']*100
        print(f"  k2={k2:.1f}: pred={pred:.4f}, obs={res['omega2']:.4f}, err={err:+.2f}%")

# 7. Simple formula: omega_eff^2 = A * k2 + B * k1 * k2
print("\n--- Testing omega_eff^2 = A*k2 + B*k1*k2 ---")
omega2s = np.array([res['omega2'] for res in clean_results.values()])
A_mat = np.column_stack([k2s, k1s * k2s])
coeffs = np.linalg.lstsq(A_mat, omega2s, rcond=None)[0]
print(f"  omega^2 = {coeffs[0]:.4f}*k2 + {coeffs[1]:.4f}*k1*k2")
for (k1, k2), res in clean_results.items():
    pred = coeffs[0]*k2 + coeffs[1]*k1*k2
    err = (pred - res['omega2'])/res['omega2']*100
    print(f"  k1={k1:.1f},k2={k2:.1f}: pred={pred:.4f}, obs={res['omega2']:.4f}, err={err:+.2f}%")

print("\nDone.")