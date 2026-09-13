#!/usr/bin/env python3
"""
Scan 3D breathing period vs. system parameters.

Uses the known 3D breathing initial conditions (seed=0) and varies each
parameter systematically to discover the relationship between the
steady-state breathing period and the parameters.

Parameters scanned:
  - k1 (attractive gain): 0.1 ~ 0.8
  - k2 (observer gain):   0.1 ~ 0.8
  - N (vehicles):         4, 5, 6, 7, 8
  - d_col (collision):    3.0, 5.0, 7.0
  - mu (sensing):         7.0, 9.0, 11.0

For each parameter set, runs a moderate-length simulation and measures
the dominant FFT frequency of pairwise distances in steady state.
"""

import sys
import numpy as np
from pathlib import Path
from scipy.integrate import solve_ivp
from scipy.signal import find_peaks

# Add parent dir for fencing_ode (not used here but kept for consistency)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# ============================================================
# Known 3D breathing initial conditions (seed=0)
# ============================================================
XI0_BASE = np.array([
    [14.112419,  3.201258,  7.829904],
    [17.927146, 14.940464, -7.818223],
    [ 7.600707, -1.210858, -0.825751],
    [ 3.284788,  1.152349, 11.634188],
    [ 6.088302,  0.973400,  3.550906],
    [ 2.669395, 11.952633, -1.641266],
])


def make_3d_rhs(N, d_col, mu, k1, k2, x_target0, v0):
    """Return rhs(t, y) for 3D fencing system."""
    def alpha(s):
        if s > mu:
            return 0.0
        if s <= d_col:
            return 1e6
        return 1.0 / (s - d_col) - 1.0 / (mu - d_col)

    def rhs(t, y):
        x = y[:3*N].reshape(N, 3)
        v = y[3*N:].reshape(N, 3)
        x_t = x_target0 + v0 * t
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
    """Generate initial conditions for different N by scaling the base IC."""
    np.random.seed(seed)
    if N_target == N_base:
        return XI0_BASE.copy()

    # For different N, generate random positions scaled to similar density
    x0 = np.random.randn(N_target, 3) * 8.0 * scale_factor
    # Ensure pairwise distances > d_col
    for i in range(N_target):
        for j in range(i+1, N_target):
            attempts = 0
            while np.linalg.norm(x0[i] - x0[j]) <= 5.0 and attempts < 100:
                x0[j] = np.random.randn(3) * 8.0 * scale_factor
                attempts += 1
    return x0


def run_and_measure(N, d_col, mu, k1, k2, v0, xi0, T_MAX, DT):
    """Run simulation and measure dominant frequency of pairwise distances."""
    x_target0 = np.array([0.0, 0.0, 0.0])
    rhs = make_3d_rhs(N, d_col, mu, k1, k2, x_target0, v0)

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
    x_target_traj = x_target0.reshape(3, 1) + np.outer(v0, t)

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
    # If peak_ratio is very small, it's rigid rotation
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
    }, None


# ============================================================
# Parameter scans
# ============================================================
def scan_k1():
    """Scan k1, keeping k2=0.5 fixed."""
    print("=" * 70)
    print("SCAN 1: k1 (k2=0.5 fixed, N=6, d=5.0, mu=9.0)")
    print("=" * 70)
    N, d_col, mu = 6, 5.0, 9.0
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 300.0, 0.05
    xi0 = XI0_BASE.copy()

    print(f"{'k1':>8}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak_ratio':>12}  {'sigma3/s1':>10}  {'type':>15}")
    print("-" * 70)
    for k1 in [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8]:
        result, err = run_and_measure(N, d_col, mu, k1, 0.5, v0, xi0, T_MAX, DT)
        if result is None:
            print(f"{k1:>8.2f}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        print(f"{k1:>8.2f}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>12.4f}  {result['sigma_ratio']:>10.4f}  {t:>15}")


def scan_k2():
    """Scan k2, keeping k1=0.5 fixed."""
    print("\n" + "=" * 70)
    print("SCAN 2: k2 (k1=0.5 fixed, N=6, d=5.0, mu=9.0)")
    print("=" * 70)
    N, d_col, mu = 6, 5.0, 9.0
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 300.0, 0.05
    xi0 = XI0_BASE.copy()

    print(f"{'k2':>8}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak_ratio':>12}  {'sigma3/s1':>10}  {'type':>15}  {'2pi/sqrt(k2)':>14}")
    print("-" * 70)
    for k2 in [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8]:
        result, err = run_and_measure(N, d_col, mu, 0.5, k2, v0, xi0, T_MAX, DT)
        if result is None:
            print(f"{k2:>8.2f}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        T_rot = 2*np.pi/np.sqrt(k2) if k2 > 0 else float('inf')
        print(f"{k2:>8.2f}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>12.4f}  {result['sigma_ratio']:>10.4f}  {t:>15}  {T_rot:>14.3f}")


def scan_N():
    """Scan N, keeping k1=0.5, k2=0.5 fixed."""
    print("\n" + "=" * 70)
    print("SCAN 3: N (k1=0.5, k2=0.5, d=5.0, mu=9.0)")
    print("=" * 70)
    d_col, mu = 5.0, 9.0
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 300.0, 0.05

    print(f"{'N':>6}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak_ratio':>12}  {'sigma3/s1':>10}  {'type':>15}")
    print("-" * 70)
    for N in [4, 5, 6, 7, 8]:
        xi0 = generate_scaled_init(N, N_base=6, scale_factor=1.0, seed=0)
        result, err = run_and_measure(N, d_col, mu, 0.5, 0.5, v0, xi0, T_MAX, DT)
        if result is None:
            print(f"{N:>6}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        print(f"{N:>6}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>12.4f}  {result['sigma_ratio']:>10.4f}  {t:>15}")


def scan_d_mu():
    """Scan d_col and mu, keeping k1=0.5, k2=0.5, N=6 fixed."""
    print("\n" + "=" * 70)
    print("SCAN 4: d_col and mu (k1=0.5, k2=0.5, N=6)")
    print("=" * 70)
    N = 6
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 300.0, 0.05
    xi0 = XI0_BASE.copy()

    print("\n--- Vary d_col (mu=9.0) ---")
    print(f"{'d_col':>8}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak_ratio':>12}  {'sigma3/s1':>10}  {'type':>15}")
    print("-" * 70)
    for d_col in [3.0, 4.0, 5.0, 6.0, 7.0]:
        result, err = run_and_measure(N, d_col, 9.0, 0.5, 0.5, v0, xi0, T_MAX, DT)
        if result is None:
            print(f"{d_col:>8.1f}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        print(f"{d_col:>8.1f}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>12.4f}  {result['sigma_ratio']:>10.4f}  {t:>15}")

    print("\n--- Vary mu (d_col=5.0) ---")
    print(f"{'mu':>8}  {'f(Hz)':>10}  {'T(s)':>10}  {'peak_ratio':>12}  {'sigma3/s1':>10}  {'type':>15}")
    print("-" * 70)
    for mu in [7.0, 8.0, 9.0, 10.0, 11.0, 12.0]:
        result, err = run_and_measure(N, 5.0, mu, 0.5, 0.5, v0, xi0, T_MAX, DT)
        if result is None:
            print(f"{mu:>8.1f}  {'ERROR':>10}  {err}")
            continue
        t = 'BREATHING' if result['is_breathing'] else 'ROTATION'
        T_str = f"{result['period']:.3f}" if result['period'] else "—"
        print(f"{mu:>8.1f}  {result['freq']:>10.4f}  {T_str:>10}  {result['peak_ratio']:>12.4f}  {result['sigma_ratio']:>10.4f}  {t:>15}")


def scan_k1_k2_grid():
    """2D grid scan of (k1, k2) to map the breathing region."""
    print("\n" + "=" * 70)
    print("SCAN 5: (k1, k2) grid (N=6, d=5.0, mu=9.0)")
    print("=" * 70)
    N, d_col, mu = 6, 5.0, 9.0
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX, DT = 250.0, 0.05
    xi0 = XI0_BASE.copy()

    print("\nPeriod T (s) — rows=k1, cols=k2")
    print(f"{'k1\\k2':>8}", end="")
    for k2 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
        print(f"{k2:>10}", end="")
    print()
    print("-" * 80)

    for k1 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]:
        print(f"{k1:>8.2f}", end="")
        for k2 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
            result, err = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, T_MAX, DT)
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
    scan_k1()
    scan_k2()
    scan_N()
    scan_d_mu()
    scan_k1_k2_grid()

    print("\n" + "=" * 70)
    print("DONE")
    print("=" * 70)