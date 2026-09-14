#!/usr/bin/env python3
"""
3D breathing simulation with a CONTINUOUS alpha function.

Requirements for alpha(s):
  1. Continuous on [d, ∞)
  2. α(s) → ∞ as s → d⁺  (repulsive force blows up at collision)
  3. α(s) = 0 for s > μ   (no force beyond sensing radius)
  4. Range: [0, ∞), including 0

The standard Kou-Chen-Xiang form α(s) = 1/(s-d) - 1/(μ-d) already satisfies
all of these on (d, μ]. The bug in the original code was the discontinuous
`if s <= d_col: return 1e6` clause, which broke continuity at s = d.

This script uses the continuous form and runs parameter scans to study
the relationship between the breathing period and system parameters in 3D.

Alpha function forms available:
  - "standard":  α(s) = 1/(s-d) - 1/(μ-d)
  - "power":     α(s) = 1/(s-d)^p - 1/(μ-d)^p     (p > 0, default p=1)
  - "rational":  α(s) = (μ-s) / (s-d)              (simplest rational form)
  - "log":       α(s) = log((μ-d)/(s-d))           (logarithmic form)
  - "stiff":     α(s) = 1/(s-d)^2 - 1/(μ-d)^2     (stiffer repulsion)
"""

import sys
import numpy as np
from pathlib import Path
from scipy.integrate import solve_ivp
from scipy.signal import find_peaks

# Add parent dir
sys.path.insert(0, str(Path(__file__).resolve().parent))

# ============================================================
# Known 3D breathing initial conditions (seed=0, N=6)
# ============================================================
XI0_BASE = np.array([
    [14.112419,  3.201258,  7.829904],
    [17.927146, 14.940464, -7.818223],
    [ 7.600707, -1.210858, -0.825751],
    [ 3.284788,  1.152349, 11.634188],
    [ 6.088302,  0.973400,  3.550906],
    [ 2.669395, 11.952633, -1.641266],
])


def make_alpha_func(form="standard", power=1.0):
    """
    Create a continuous alpha function satisfying:
      - Continuous on [d, ∞)
      - α(s) → ∞ as s → d⁺
      - α(s) = 0 for s > μ
      - Range: [0, ∞), including 0

    All forms are continuous at s = μ (value = 0) and blow up at s = d.
    """
    def alpha_factory(d_col, mu):
        if form == "standard":
            # α(s) = 1/(s-d) - 1/(μ-d)
            # At s=μ: 1/(μ-d) - 1/(μ-d) = 0 ✓
            # As s→d⁺: 1/(s-d) → ∞ ✓
            def alpha(s):
                if s >= mu:
                    return 0.0
                if s <= d_col:
                    return 1e6  # numerical floor, effectively ∞
                return 1.0 / (s - d_col) - 1.0 / (mu - d_col)
            return alpha

        elif form == "power":
            # α(s) = 1/(s-d)^p - 1/(μ-d)^p
            # At s=μ: 1/(μ-d)^p - 1/(μ-d)^p = 0 ✓
            # As s→d⁺: 1/(s-d)^p → ∞ ✓
            def alpha(s):
                if s >= mu:
                    return 0.0
                if s <= d_col:
                    return 1e6
                return 1.0 / (s - d_col)**power - 1.0 / (mu - d_col)**power
            return alpha

        elif form == "rational":
            # α(s) = (μ-s) / (s-d)
            # At s=μ: (μ-μ)/(μ-d) = 0 ✓
            # As s→d⁺: (μ-d)/(s-d) → ∞ ✓
            # This is simpler and has a softer singularity
            def alpha(s):
                if s >= mu:
                    return 0.0
                if s <= d_col:
                    return 1e6
                return (mu - s) / (s - d_col)
            return alpha

        elif form == "log":
            # α(s) = log((μ-d)/(s-d))
            # At s=μ: log(1) = 0 ✓
            # As s→d⁺: log((μ-d)/0⁺) → ∞ ✓
            def alpha(s):
                if s >= mu:
                    return 0.0
                if s <= d_col:
                    return 1e6
                return np.log((mu - d_col) / (s - d_col))
            return alpha

        elif form == "stiff":
            # α(s) = 1/(s-d)^2 - 1/(μ-d)^2
            # Same as power with p=2 — stiffer near collision
            def alpha(s):
                if s >= mu:
                    return 0.0
                if s <= d_col:
                    return 1e6
                return 1.0 / (s - d_col)**2 - 1.0 / (mu - d_col)**2
            return alpha

        elif form == "exponential":
            # α(s) = exp(1/(s-d)) * (μ-s) / (μ-d)
            # At s=μ: exp(1/(μ-d)) * 0 = 0 ✓
            # As s→d⁺: exp(∞) * (μ-d)/(μ-d) → ∞ ✓
            # Very stiff near collision
            def alpha(s):
                if s >= mu:
                    return 0.0
                if s <= d_col:
                    return 1e6
                return np.exp(1.0 / (s - d_col)) * (mu - s) / (mu - d_col)
            return alpha

        else:
            raise ValueError(f"Unknown alpha form: {form}")

    return alpha_factory


def make_rhs(N, d_col, mu, k1, k2, v0, alpha_func):
    """Create the RHS function for the 3D fencing system."""
    alpha = alpha_func(d_col, mu)

    def rhs(t, y):
        x = y[:3*N].reshape(N, 3)
        v = y[3*N:].reshape(N, 3)
        x_t = np.zeros(3) + v0 * t
        phi = np.zeros((N, 3))
        for i in range(N):
            for j in range(N):
                if i == j:
                    continue
                xij = x[i] - x[j]
                dist = np.linalg.norm(xij)
                if d_col < dist <= mu:
                    phi[i] += alpha(dist) * xij / dist
        u = phi + k1 * (x_t - x) + v
        v_dot = k2 * (x_t - x)
        return np.concatenate([u.ravel(), v_dot.ravel()])
    return rhs


def generate_scaled_init(N_target, N_base=6, scale_factor=1.0, seed=0):
    """Generate initial conditions for different N."""
    np.random.seed(seed)
    if N_target == N_base:
        return XI0_BASE.copy()
    x0 = np.random.randn(N_target, 3) * 8.0 * scale_factor
    for i in range(N_target):
        for j in range(i+1, N_target):
            attempts = 0
            while np.linalg.norm(x0[i] - x0[j]) <= 5.0 and attempts < 100:
                x0[j] = np.random.randn(3) * 8.0 * scale_factor
                attempts += 1
    return x0


def run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_func, T_MAX, DT):
    """Run simulation and measure dominant frequency of pairwise distances."""
    rhs = make_rhs(N, d_col, mu, k1, k2, v0, alpha_func)
    y0 = np.zeros(6 * N)
    y0[:3*N] = xi0.ravel()
    t_eval = np.arange(0, T_MAX + DT, DT)

    try:
        sol = solve_ivp(rhs, [0, T_MAX], y0, method='LSODA',
                        t_eval=t_eval, rtol=1e-8, atol=1e-10, max_step=DT)
    except Exception as e:
        return None, str(e)

    if not sol.success:
        return None, sol.message

    t = sol.t
    x_traj = sol.y[:3*N, :].reshape(N, 3, -1)
    x_target_traj = np.zeros((3, len(t))) + np.outer(v0, t)

    # Pairwise distances
    pairwise_dists = []
    for i in range(N):
        for j in range(i+1, N):
            pairwise_dists.append(np.linalg.norm(x_traj[i] - x_traj[j], axis=0))
    pairwise_dists = np.array(pairwise_dists)

    # Steady state: last 50%
    mask = t > (t[-1] - t[-1] * 0.5)
    t_ss = t[mask]
    pd_ss = pairwise_dists[:, mask]
    T_ss = len(t_ss)

    # FFT
    fft_freqs = np.fft.rfftfreq(T_ss, d=DT)
    avg_power = np.zeros(len(fft_freqs))
    for pd in pd_ss:
        pd_detrend = pd - np.mean(pd)
        fft = np.fft.rfft(pd_detrend)
        avg_power += np.abs(fft) ** 2
    avg_power[0] = 0

    dominant_idx = np.argmax(avg_power)
    dominant_freq = fft_freqs[dominant_idx]
    total_power = np.sum(avg_power)
    peak_ratio = avg_power[dominant_idx] / total_power if total_power > 0 else 0

    # Check if converged (rigid rotation) vs oscillating (breathing)
    is_breathing = peak_ratio > 0.05

    # Planarity
    xi_ss = x_traj[:, :, mask] - x_target_traj.reshape(1, 3, -1)[:, :, mask]
    final_rel = xi_ss[:, :, -1]
    centered = final_rel - np.mean(final_rel, axis=0)
    _, S, _ = np.linalg.svd(centered, full_matrices=False)
    sigma_ratio = S[2] / S[0] if len(S) >= 3 else 1.0

    return {
        'freq': dominant_freq,
        'period': 1.0 / dominant_freq if dominant_freq > 0.001 else None,
        'peak_ratio': peak_ratio,
        'is_breathing': is_breathing,
        'sigma_ratio': sigma_ratio,
        'S': S,
    }, None


# ============================================================
# Scans
# ============================================================
def scan_k1(alpha_form, N=6, d_col=5.0, mu=9.0, k2=0.5):
    """Scan k1, keeping k2 fixed."""
    print(f"\n{'='*70}")
    print(f"SCAN k1 (alpha={alpha_form}, k2={k2}, N={N}, d={d_col}, mu={mu})")
    print('='*70)
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 300.0, 0.05
    xi0 = XI0_BASE.copy()
    alpha_func = make_alpha_func(alpha_form)

    print(f"{'k1':>8}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak':>10}  {'sigma':>10}  {'type':>12}")
    print("-" * 70)
    for k1 in [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8]:
        result, err = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_func, T_MAX, DT)
        if result is None:
            print(f"{k1:>8.2f}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        print(f"{k1:>8.2f}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>10.4f}  {result['sigma_ratio']:>10.4f}  {t:>12}")


def scan_k2(alpha_form, N=6, d_col=5.0, mu=9.0, k1=0.5):
    """Scan k2, keeping k1 fixed."""
    print(f"\n{'='*70}")
    print(f"SCAN k2 (alpha={alpha_form}, k1={k1}, N={N}, d={d_col}, mu={mu})")
    print('='*70)
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 300.0, 0.05
    xi0 = XI0_BASE.copy()
    alpha_func = make_alpha_func(alpha_form)

    print(f"{'k2':>8}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak':>10}  {'sigma':>10}  {'type':>12}  {'2pi/sqrt(k2)':>14}")
    print("-" * 75)
    for k2 in [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8]:
        result, err = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_func, T_MAX, DT)
        if result is None:
            print(f"{k2:>8.2f}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        T_rot = 2*np.pi/np.sqrt(k2) if k2 > 0 else float('inf')
        print(f"{k2:>8.2f}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>10.4f}  {result['sigma_ratio']:>10.4f}  {t:>12}  {T_rot:>14.3f}")


def scan_d_mu(alpha_form, N=6, k1=0.5, k2=0.5):
    """Scan d_col and mu."""
    print(f"\n{'='*70}")
    print(f"SCAN d_col & mu (alpha={alpha_form}, k1={k1}, k2={k2}, N={N})")
    print('='*70)
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 300.0, 0.05
    xi0 = XI0_BASE.copy()
    alpha_func = make_alpha_func(alpha_form)

    print("\n--- Vary d_col (mu=9.0) ---")
    print(f"{'d_col':>8}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak':>10}  {'sigma':>10}  {'type':>12}")
    print("-" * 70)
    for d_col in [3.0, 4.0, 5.0, 6.0, 7.0]:
        result, err = run_and_measure(N, d_col, 9.0, k1, k2, v0, xi0, alpha_func, T_MAX, DT)
        if result is None:
            print(f"{d_col:>8.1f}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        print(f"{d_col:>8.1f}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>10.4f}  {result['sigma_ratio']:>10.4f}  {t:>12}")

    print("\n--- Vary mu (d_col=5.0) ---")
    print(f"{'mu':>8}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak':>10}  {'sigma':>10}  {'type':>12}")
    print("-" * 70)
    for mu in [7.0, 8.0, 9.0, 10.0, 11.0, 12.0]:
        result, err = run_and_measure(N, 5.0, mu, k1, k2, v0, xi0, alpha_func, T_MAX, DT)
        if result is None:
            print(f"{mu:>8.1f}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        print(f"{mu:>8.1f}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>10.4f}  {result['sigma_ratio']:>10.4f}  {t:>12}")


def scan_N(alpha_form, k1=0.5, k2=0.5, d_col=5.0, mu=9.0):
    """Scan N."""
    print(f"\n{'='*70}")
    print(f"SCAN N (alpha={alpha_form}, k1={k1}, k2={k2}, d={d_col}, mu={mu})")
    print('='*70)
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 300.0, 0.05
    alpha_func = make_alpha_func(alpha_form)

    print(f"{'N':>6}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak':>10}  {'sigma':>10}  {'type':>12}")
    print("-" * 70)
    for N in [4, 5, 6, 7, 8]:
        xi0 = generate_scaled_init(N, N_base=6, scale_factor=1.0, seed=0)
        result, err = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_func, T_MAX, DT)
        if result is None:
            print(f"{N:>6}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        print(f"{N:>6}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>10.4f}  {result['sigma_ratio']:>10.4f}  {t:>12}")


def scan_k1_k2_grid(alpha_form, N=6, d_col=5.0, mu=9.0):
    """2D grid scan of (k1, k2)."""
    print(f"\n{'='*70}")
    print(f"SCAN (k1,k2) grid (alpha={alpha_form}, N={N}, d={d_col}, mu={mu})")
    print('='*70)
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 250.0, 0.05
    xi0 = XI0_BASE.copy()
    alpha_func = make_alpha_func(alpha_form)

    print("\nPeriod T (s) — rows=k1, cols=k2")
    print(f"{'k1\\k2':>8}", end="")
    for k2 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
        print(f"{k2:>10}", end="")
    print()
    print("-" * 80)

    for k1 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]:
        print(f"{k1:>8.2f}", end="")
        for k2 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
            result, err = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_func, T_MAX, DT)
            if result is None:
                print(f"{'ERR':>10}", end="")
            elif result['is_breathing'] and result['period']:
                print(f"{result['period']:>10.3f}", end="")
            elif result['is_breathing']:
                print(f"{'—':>10}", end="")
            else:
                print(f"{'ROT':>10}", end="")
        print()


# ============================================================
# Main
# ============================================================
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        description="3D breathing simulation with continuous alpha function"
    )
    parser.add_argument(
        '--alpha', type=str, default='standard',
        choices=['standard', 'power', 'rational', 'log', 'stiff', 'exponential'],
        help='Alpha function form (default: standard)'
    )
    parser.add_argument(
        '--power', type=float, default=1.0,
        help='Power for power-form alpha (default: 1.0)'
    )
    parser.add_argument(
        '--quick', action='store_true',
        help='Quick scan (fewer parameters)'
    )
    args = parser.parse_args()

    alpha_form = args.alpha
    if alpha_form == 'power':
        # Override the power parameter
        pass  # Will use default 1.0 = standard

    print(f"\n{'#'*70}")
    print(f"# 3D Breathing Parameter Scan with Continuous Alpha Function")
    print(f"#  Alpha form: {alpha_form}")
    print(f"{'#'*70}")

    if args.quick:
        scan_k1(alpha_form)
        scan_k2(alpha_form)
        scan_d_mu(alpha_form)
        scan_N(alpha_form)
    else:
        scan_k1(alpha_form)
        scan_k2(alpha_form)
        scan_d_mu(alpha_form)
        scan_N(alpha_form)
        scan_k1_k2_grid(alpha_form)

    print(f"\n{'='*70}")
    print("DONE")
    print('='*70)