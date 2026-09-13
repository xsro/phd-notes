#!/usr/bin/env python3
"""
Search for 3D breathing limit cycles.

"Breathing" in 3D means: pairwise distances oscillate periodically without
converging to a fixed value, while the formation may or may not rotate.

Strategy:
  1. Generate random 3D initial conditions (spherical distribution)
  2. Integrate to steady state (t > T_max * 0.5)
  3. Analyze pairwise distances via FFT
  4. Classify:
     - Rigid rotation: pairwise distances → constant (no FFT peaks)
     - Breathing: pairwise distances oscillate (significant FFT peaks)
     - chaotic/other: broadband spectrum

Also check planarity (SVD) to distinguish planar rotation from 3D breathing.
"""

import sys
import numpy as np
from pathlib import Path
from scipy.integrate import solve_ivp
from scipy.signal import find_peaks

# Add parent dir to path for fencing_ode (2D shared code)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "fencing-rotation-conjecture" / "simulation"))

# ============================================================
# 3D dynamics (same as simulate_3d.py)
# ============================================================
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


def classify_3d(t, x_traj, x_target_traj, d_col, mu, dt):
    """Classify the 3D steady-state behavior.

    Returns dict with:
      - type: 'rigid_rotation', 'breathing', 'planar_rotation', 'chaotic', 'unknown'
      - dominant_freq: float (Hz), or None
      - period: float (s), or None
      - planarity: sigma_3/sigma_1
      - pairwise_oscillation: bool
      - details: str
    """
    N = x_traj.shape[0]
    # Steady state: last 50% of time
    mask = t > (t[-1] - t[-1] * 0.5)
    t_ss = t[mask]
    xi_ss = x_traj[:, :, mask] - x_target_traj.reshape(1, 3, -1)[:, :, mask]
    T_ss = len(t_ss)

    # Pairwise distances in steady state
    pairwise_dists = []
    for i in range(N):
        for j in range(i+1, N):
            pairwise_dists.append(np.linalg.norm(xi_ss[i] - xi_ss[j], axis=0))
    pairwise_dists = np.array(pairwise_dists)  # (N_pairs, T_ss)

    # FFT of pairwise distances (detrended)
    fft_freqs = np.fft.rfftfreq(T_ss, d=dt)
    peak_power_ratio = []
    for pd in pairwise_dists:
        pd_detrend = pd - np.mean(pd)
        fft = np.fft.rfft(pd_detrend)
        power = np.abs(fft) ** 2
        power[0] = 0  # kill DC
        total_power = np.sum(power)
        if total_power < 1e-20:
            peak_power_ratio.append(0.0)
            continue
        max_power = np.max(power)
        peak_power_ratio.append(max_power / total_power)

    max_peak_ratio = max(peak_power_ratio) if peak_power_ratio else 0.0

    # Find dominant frequency
    avg_power = np.zeros(len(fft_freqs))
    for pd in pairwise_dists:
        pd_detrend = pd - np.mean(pd)
        fft = np.fft.rfft(pd_detrend)
        avg_power += np.abs(fft) ** 2
    avg_power[0] = 0
    dominant_idx = np.argmax(avg_power)
    dominant_freq = fft_freqs[dominant_idx]
    dominant_power_ratio = avg_power[dominant_idx] / (np.sum(avg_power) + 1e-20)

    # Planarity via SVD
    final_rel = xi_ss[:, :, -1]
    centered = final_rel - np.mean(final_rel, axis=0)
    _, S, _ = np.linalg.svd(centered, full_matrices=False)
    sigma_ratio = S[2] / S[0] if len(S) >= 3 else 1.0

    # Classification
    if max_peak_ratio < 0.05:
        # No significant oscillation in pairwise distances
        if sigma_ratio < 0.1:
            result_type = 'planar_rotation'
        else:
            result_type = 'rigid_rotation'
        details = f"pairwise distances converge (max FFT peak ratio={max_peak_ratio:.4f})"
    elif dominant_freq < 0.01:
        result_type = 'unknown'
        details = f"very low frequency oscillation, f={dominant_freq:.5f} Hz"
    else:
        # Significant oscillation
        if sigma_ratio > 0.3:
            result_type = 'breathing_3d'
        else:
            result_type = 'breathing_planar'
        details = (f"pairwise oscillate: f={dominant_freq:.4f} Hz, "
                   f"T={1/dominant_freq:.2f}s, "
                   f"sigma3/sigma1={sigma_ratio:.3f}, "
                   f"max_peak_ratio={max_peak_ratio:.3f}")

    period = 1.0 / dominant_freq if dominant_freq > 0.001 else None

    return {
        'type': result_type,
        'dominant_freq': dominant_freq,
        'period': period,
        'planarity': sigma_ratio,
        'pairwise_oscillation': max_peak_ratio >= 0.05,
        'peak_ratio': max_peak_ratio,
        'details': details
    }


def run_single_3d(N, d_col, mu, k1, k2, v0, x0_init, T_MAX, DT, seed):
    """Run one 3D simulation and classify."""
    x_target0 = np.array([0.0, 0.0, 0.0])
    rhs = make_3d_rhs(N, d_col, mu, k1, k2, x_target0, v0)

    y0 = np.zeros(6 * N)
    y0[:3*N] = x0_init.ravel()
    # v_init = 0

    t_eval = np.arange(0, T_MAX + DT, DT)
    sol = solve_ivp(rhs, [0, T_MAX], y0, method='LSODA',
                    t_eval=t_eval, rtol=1e-9, atol=1e-11, max_step=DT)

    if not sol.success:
        return None

    t = sol.t
    x_traj = sol.y[:3*N, :].reshape(N, 3, -1)
    x_target_traj = x_target0.reshape(3, 1) + np.outer(v0, t)

    result = classify_3d(t, x_traj, x_target_traj, d_col, mu, DT)
    result['seed'] = seed
    result['x0_init'] = x0_init.copy()
    return result


def generate_random_3d_init(N, d_col, seed):
    """Generate random 3D initial positions with pairwise distances > d_col."""
    np.random.seed(seed)
    x0 = np.random.randn(N, 3) * 8.0
    # Ensure pairwise distances > d_col
    for i in range(N):
        for j in range(i+1, N):
            attempts = 0
            while np.linalg.norm(x0[i] - x0[j]) <= d_col and attempts < 100:
                x0[j] = np.random.randn(3) * 8.0
                attempts += 1
    return x0


# ============================================================
# Main search
# ============================================================
def main():
    N = 6
    d_col = 5.0
    mu = 9.0
    k1 = 0.5
    k2 = 0.5
    v0 = np.array([1.0, 0.0, 0.0])
    T_MAX = 200.0
    DT = 0.05
    N_SEEDS = 50

    print(f"Searching for 3D breathing: N={N}, d={d_col}, mu={mu}, k1={k1}, k2={k2}")
    print(f"T_MAX={T_MAX}, {N_SEEDS} random seeds")
    print("=" * 70)

    results = []
    for seed in range(N_SEEDS):
        x0 = generate_random_3d_init(N, d_col, seed)
        result = run_single_3d(N, d_col, mu, k1, k2, v0, x0, T_MAX, DT, seed)
        if result is None:
            print(f"  seed {seed:3d}: integration FAILED")
            continue
        results.append(result)
        type_str = result['type']
        if type_str == 'breathing_3d':
            print(f"  seed {seed:3d}: ★ BREATHING 3D ★  {result['details']}")
        elif type_str == 'breathing_planar':
            print(f"  seed {seed:3d}: BREATHING planar  {result['details']}")
        elif type_str == 'rigid_rotation':
            print(f"  seed {seed:3d}: rigid rotation  {result['details']}")
        elif type_str == 'planar_rotation':
            print(f"  seed {seed:3d}: planar rotation  {result['details']}")
        else:
            print(f"  seed {seed:3d}: {type_str}  {result['details']}")

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    type_counts = {}
    for r in results:
        t = r['type']
        type_counts[t] = type_counts.get(t, 0) + 1
    for t, c in sorted(type_counts.items()):
        print(f"  {t}: {c}")

    breathing_3d = [r for r in results if r['type'] == 'breathing_3d']
    breathing_any = [r for r in results if 'breathing' in r['type']]

    if breathing_3d:
        print(f"\n★ Found {len(breathing_3d)} genuine 3D breathing case(s)!")
        for r in breathing_3d:
            print(f"  seed={r['seed']}: f={r['dominant_freq']:.4f} Hz, T={r['period']:.2f}s, sigma3/sigma1={r['planarity']:.3f}")
    elif breathing_any:
        print(f"\nFound {len(breathing_any)} breathing case(s) but all are planar (not genuine 3D).")
    else:
        print("\nNo 3D breathing found in this batch.")


if __name__ == '__main__':
    main()