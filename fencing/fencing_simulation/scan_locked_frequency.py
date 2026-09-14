#!/usr/bin/env python3
"""
Analyze the LOCKED frequency (after frequency locking occurs) and its
relationship with system parameters: k1, k2, d, mu, N.

In the locked regime, f_rad = f_tan = f_self. We scan parameters to find
what determines this common frequency.
"""

import numpy as np
from scipy.integrate import solve_ivp
import json
import os
import warnings
warnings.filterwarnings('ignore')

# ============================================================
# Parameters
# ============================================================
D_COL = 5.0
K1 = 0.5
K2 = 0.5
V0 = np.array([1.0, 0.0, 0.0])
SEED = 0
T_MAX = 150.0
DT = 0.02
T_STEADY = 80.0

# ============================================================
# Repulsion and dynamics
# ============================================================
def make_rhs(N, d, mu, k1, k2):
    def alpha(s):
        if s > mu:
            return 0.0
        if s <= d:
            return 1e6
        return 1.0 / (s - d) - 1.0 / (mu - d)

    def rhs(t, y):
        x = y[:3*N].reshape(N, 3)
        v = y[3*N:].reshape(N, 3)
        xt = V0 * t
        phi = np.zeros((N, 3))
        for i in range(N):
            for j in range(N):
                if i == j:
                    continue
                e = x[i] - x[j]
                s = np.linalg.norm(e)
                if d < s <= mu:
                    phi[i] += alpha(s) * e / s
        u = phi + k1 * (xt - x) + v
        vd = k2 * (xt - x)
        return np.concatenate([u.ravel(), vd.ravel()])
    return rhs


def generate_initial_conditions(N, d, mu, seed=SEED):
    np.random.seed(seed)
    spread = max(15.0, 10.0 * np.sqrt(N))
    x0 = np.random.randn(N, 3) * spread
    for i in range(N):
        for j in range(i+1, N):
            attempts = 0
            while np.linalg.norm(x0[i] - x0[j]) <= d * 1.2 and attempts < 100:
                x0[j] = np.random.randn(3) * spread
                attempts += 1
    return x0


def simulate_and_measure(N, d, mu, k1, k2, seed=SEED):
    """Simulate and return frequencies. Returns None if simulation fails."""
    xi0 = generate_initial_conditions(N, d, mu, seed)
    rhs = make_rhs(N, d, mu, k1, k2)

    y0 = np.zeros(6 * N)
    y0[:3*N] = xi0.ravel()
    t_eval = np.arange(0, T_MAX + DT, DT)

    try:
        sol = solve_ivp(rhs, [0, T_MAX], y0, method='LSODA',
                        t_eval=t_eval, rtol=1e-8, atol=1e-10, max_step=DT)
    except Exception:
        return None

    if not sol.success:
        return None

    t = sol.t
    x = sol.y[:3*N, :].reshape(N, 3, -1)
    v = sol.y[3*N:, :].reshape(N, 3, -1)
    xt = np.outer(V0, t)

    x_cm = np.mean(x - xt.reshape(1, 3, -1), axis=0)
    x_rel = x - xt.reshape(1, 3, -1) - x_cm.reshape(1, 3, -1)

    r = np.linalg.norm(x_rel, axis=1)
    r_mag = np.linalg.norm(x_rel, axis=1)
    r_hat = x_rel / (r_mag[:, np.newaxis, :] + 1e-10)
    v_radial = np.sum(v * r_hat, axis=1)
    v_tan = v - v_radial[:, np.newaxis, :] * r_hat
    v_tan_mag = np.linalg.norm(v_tan, axis=1)
    v_self = np.linalg.norm(v, axis=1)

    mask = t > T_STEADY
    if np.sum(mask) < 100:
        mask = t > t[-1] * 0.5

    r_ss = r[:, mask]
    v_tan_ss = v_tan_mag[:, mask]
    v_self_ss = v_self[:, mask]

    dt = t[1] - t[0]
    n_ss = len(t[mask])
    freqs = np.fft.rfftfreq(n_ss, d=dt)

    def find_peak(signal, freqs):
        fft_power = np.zeros(len(freqs))
        for s in signal:
            s_detrend = s - np.mean(s)
            fft_power += np.abs(np.fft.rfft(s_detrend))**2
        fft_power[0] = 0
        idx = np.argmax(fft_power)
        return freqs[idx], idx

    f_rad, _ = find_peak(r_ss, freqs)
    f_tan, _ = find_peak(v_tan_ss, freqs)
    f_self, _ = find_peak(v_self_ss, freqs)

    # Pairwise distance
    pd = np.array([np.linalg.norm(x[i] - x[j], axis=0)
                   for i in range(N) for j in range(i+1, N)])
    pd_ss = pd[:, mask]
    f_pd, _ = find_peak(pd_ss, freqs)

    # Planarity
    xi_final = x[:, :, -1] - xt.reshape(1, 3, -1)[:, :, -1]
    centered = xi_final - np.mean(xi_final, axis=0)
    _, S, _ = np.linalg.svd(centered, full_matrices=False)
    sigma_ratio = S[2] / S[0] if len(S) >= 3 else 1.0

    min_pairwise = np.min(pd_ss)

    f_rot = np.sqrt(k2) / (2 * np.pi)
    T_rot = 2 * np.pi / np.sqrt(k2)

    # Determine regime
    if f_rad > 0.001 and f_tan > 0.001:
        ratio = f_rad / f_tan
        if abs(ratio - 1.0) < 0.15:
            regime = "LOCKED"
        elif ratio > 1.3:
            regime = "BREATHING"
        else:
            regime = "OTHER"
    else:
        regime = "ANOMALOUS"

    return {
        'N': N, 'd': d, 'mu': mu, 'k1': k1, 'k2': k2,
        'f_rad': f_rad, 'f_tan': f_tan, 'f_self': f_self, 'f_pd': f_pd,
        'f_rot': f_rot, 'T_rot': T_rot,
        'regime': regime,
        'f_ratio': f_rad / f_tan if f_tan > 0.001 else 0,
        'sigma_ratio': sigma_ratio,
        'min_pairwise': min_pairwise,
    }


def scan_locked_frequency():
    """
    Scan parameters to find what determines the locked frequency.
    We focus on parameter combinations that produce LOCKED regime.
    """
    print("=" * 100)
    print("LOCKED FREQUENCY vs PARAMETERS")
    print("=" * 100)
    print()

    # ----------------------------------------------------------
    # 1. k2 scan in locked regime (N=15, which is locked at k1=k2=0.5)
    # ----------------------------------------------------------
    print("-" * 80)
    print("1. k2 SCAN (N=15, k1=0.5, d=5, mu scaled) — Locked frequency vs k2")
    print("-" * 80)
    print(f"{'k2':>6s} {'f_locked':>10s} {'T_locked':>10s} {'f_rot':>8s} {'f_locked/f_rot':>14s} {'T_locked/T_rot':>15s} {'sigma':>8s}")
    print("-" * 80)

    for k2 in [0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85]:
        N = 15
        mu = 9.0 * np.sqrt(N / 6.0)
        r = simulate_and_measure(N, D_COL, mu, K1, k2)
        if r and r['regime'] == 'LOCKED':
            f_lock = r['f_rad']  # In locked regime, f_rad = f_tan
            T_lock = 1.0 / f_lock if f_lock > 0.001 else 0
            f_rot = r['f_rot']
            T_rot = r['T_rot']
            print(f"{k2:6.2f} {f_lock:10.4f} {T_lock:10.2f} {f_rot:8.4f} {f_lock/f_rot:14.3f} {T_lock/T_rot:15.3f} {r['sigma_ratio']:8.3f}")
        elif r:
            print(f"{k2:6.2f} → regime={r['regime']}, f_rad={r['f_rad']:.4f}, f_tan={r['f_tan']:.4f}")
        else:
            print(f"{k2:6.2f} → FAILED")

    print()

    # ----------------------------------------------------------
    # 2. k1 scan in locked regime (N=15, k2=0.5)
    # ----------------------------------------------------------
    print("-" * 80)
    print("2. k1 SCAN (N=15, k2=0.5, d=5, mu scaled) — Locked frequency vs k1")
    print("-" * 80)
    print(f"{'k1':>6s} {'f_locked':>10s} {'T_locked':>10s} {'f_rot':>8s} {'f_locked/f_rot':>14s} {'T_locked/T_rot':>15s} {'sigma':>8s} {'regime':>10s}")
    print("-" * 80)

    for k1 in [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
        N = 15
        mu = 9.0 * np.sqrt(N / 6.0)
        r = simulate_and_measure(N, D_COL, mu, k1, K2)
        if r:
            f_lock = r['f_rad']
            T_lock = 1.0 / f_lock if f_lock > 0.001 else 0
            f_rot = r['f_rot']
            T_rot = r['T_rot']
            print(f"{k1:6.2f} {f_lock:10.4f} {T_lock:10.2f} {f_rot:8.4f} {f_lock/f_rot:14.3f} {T_lock/T_rot:15.3f} {r['sigma_ratio']:8.3f} {r['regime']:>10s}")
        else:
            print(f"{k1:6.2f} → FAILED")

    print()

    # ----------------------------------------------------------
    # 3. N scan with fixed k1=k2=0.5 (varying mu)
    # ----------------------------------------------------------
    print("-" * 80)
    print("3. N SCAN (k1=k2=0.5, d=5, mu scaled) — Locked frequency vs N")
    print("-" * 80)
    print(f"{'N':>4s} {'mu':>6s} {'f_locked':>10s} {'T_locked':>10s} {'f_rot':>8s} {'f_locked/f_rot':>14s} {'sigma':>8s} {'regime':>10s}")
    print("-" * 80)

    for N in [5, 8, 10, 12, 15, 20, 25]:
        mu = 9.0 * np.sqrt(N / 6.0)
        r = simulate_and_measure(N, D_COL, mu, K1, K2)
        if r:
            f_lock = r['f_rad']
            T_lock = 1.0 / f_lock if f_lock > 0.001 else 0
            f_rot = r['f_rot']
            print(f"{N:4d} {mu:6.1f} {f_lock:10.4f} {T_lock:10.2f} {f_rot:8.4f} {f_lock/f_rot:14.3f} {r['sigma_ratio']:8.3f} {r['regime']:>10s}")
        else:
            print(f"{N:4d} {mu:6.1f} → FAILED")

    print()

    # ----------------------------------------------------------
    # 4. d scan in locked regime (N=15, k1=k2=0.5)
    # ----------------------------------------------------------
    print("-" * 80)
    print("4. d SCAN (N=15, k1=k2=0.5, mu scaled) — Locked frequency vs d")
    print("-" * 80)
    print(f"{'d':>6s} {'f_locked':>10s} {'T_locked':>10s} {'f_rot':>8s} {'f_locked/f_rot':>14s} {'sigma':>8s} {'regime':>10s}")
    print("-" * 80)

    for d in [3.0, 4.0, 5.0, 6.0, 7.0, 8.0]:
        N = 15
        mu = 9.0 * np.sqrt(N / 6.0)
        r = simulate_and_measure(N, d, mu, K1, K2)
        if r:
            f_lock = r['f_rad']
            T_lock = 1.0 / f_lock if f_lock > 0.001 else 0
            f_rot = r['f_rot']
            print(f"{d:6.1f} {f_lock:10.4f} {T_lock:10.2f} {f_rot:8.4f} {f_lock/f_rot:14.3f} {r['sigma_ratio']:8.3f} {r['regime']:>10s}")
        else:
            print(f"{d:6.1f} → FAILED")

    print()

    # ----------------------------------------------------------
    # 5. mu scan in locked regime (N=15, k1=k2=0.5, d=5)
    # ----------------------------------------------------------
    print("-" * 80)
    print("5. mu SCAN (N=15, k1=k2=0.5, d=5) — Locked frequency vs mu")
    print("-" * 80)
    print(f"{'mu':>6s} {'f_locked':>10s} {'T_locked':>10s} {'f_rot':>8s} {'f_locked/f_rot':>14s} {'sigma':>8s} {'regime':>10s}")
    print("-" * 80)

    for mu in [8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0]:
        N = 15
        r = simulate_and_measure(N, D_COL, mu, K1, K2)
        if r:
            f_lock = r['f_rad']
            T_lock = 1.0 / f_lock if f_lock > 0.001 else 0
            f_rot = r['f_rot']
            print(f"{mu:6.1f} {f_lock:10.4f} {T_lock:10.2f} {f_rot:8.4f} {f_lock/f_rot:14.3f} {r['sigma_ratio']:8.3f} {r['regime']:>10s}")
        else:
            print(f"{mu:6.1f} → FAILED")

    print()


if __name__ == '__main__':
    scan_locked_frequency()