#!/usr/bin/env python3
"""
Search for new 2D breathing limit cycles.

The known breathing case (Case C) uses specific initial conditions from case3.py.
This script searches for other breathing cases by:
  1. Random initial conditions
  2. Perturbations of the known case3 conditions
  3. Systematic variation of N and initial formation shapes

"Breathing" = pairwise distances oscillate periodically without converging.
"""

import sys
import numpy as np
from pathlib import Path
from scipy.signal import find_peaks

# Add fencing_ode to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "fencing-rotation-conjecture" / "simulation"))
from fencing_ode import integrate, evaluate


# ============================================================
# Known case3 initial conditions
# ============================================================
XI0_CASE3 = np.array([[-10.584563, -12.532987],
                      [-5.382648,   5.904905],
                      [-0.177347,   9.175920],
                      [-7.352506,  -4.805249],
                      [-6.288338,  12.435830]])
VT0_CASE3 = np.array([[ 3.528048, -1.274511],
                      [-0.511988, -1.485443],
                      [ 1.972071, -3.679895],
                      [-3.460489, -0.767699],
                      [-2.039248,  2.761800]])


def classify_2d(t_eval, pos_all, d, mu, k1, k2):
    """Classify 2D steady-state behavior.

    Returns dict with type, period, oscillation metrics.
    """
    n = pos_all.shape[1]
    # Steady state: last 50%
    mask = t_eval > (t_eval[-1] - t_eval[-1] * 0.5)
    t_ss = t_eval[mask]
    pos_ss = pos_all[mask]  # (T, n, 2)

    # Pairwise distances
    pairwise_dists = []
    for i in range(n):
        for j in range(i+1, n):
            d_ij = np.linalg.norm(pos_ss[:, i] - pos_ss[:, j], axis=1)
            pairwise_dists.append(d_ij)
    pairwise_dists = np.array(pairwise_dists)  # (n_pairs, T_ss)

    dt = t_eval[1] - t_eval[0]
    T_ss = len(t_ss)

    # FFT analysis
    peak_ratios = []
    for pd in pairwise_dists:
        pd_detrend = pd - np.mean(pd)
        fft = np.fft.rfft(pd_detrend)
        power = np.abs(fft) ** 2
        power[0] = 0
        total = np.sum(power)
        if total < 1e-20:
            peak_ratios.append(0.0)
            continue
        peak_ratios.append(np.max(power) / total)

    max_peak_ratio = max(peak_ratios) if peak_ratios else 0.0

    # Average spectrum
    avg_power = np.zeros(len(np.fft.rfftfreq(T_ss, d=dt)))
    freqs = np.fft.rfftfreq(T_ss, d=dt)
    for pd in pairwise_dists:
        pd_detrend = pd - np.mean(pd)
        fft = np.fft.rfft(pd_detrend)
        avg_power += np.abs(fft) ** 2
    avg_power[0] = 0
    dominant_idx = np.argmax(avg_power)
    dominant_freq = freqs[dominant_idx]
    dominant_ratio = avg_power[dominant_idx] / (np.sum(avg_power) + 1e-20)

    # Also check if mean pairwise distance is changing (not converged)
    mean_dist = np.mean(pairwise_dists, axis=0)
    dist_variation = np.std(mean_dist) / (np.mean(mean_dist) + 1e-10)

    # Classification
    if max_peak_ratio < 0.02:
        result_type = 'rigid_rotation'
        details = f"pairwise distances converge (peak_ratio={max_peak_ratio:.4f}, dist_cv={dist_variation:.4f})"
    elif dominant_freq < 0.005:
        result_type = 'slow_oscillation'
        details = f"very slow oscillation, f={dominant_freq:.5f} Hz"
    else:
        result_type = 'breathing'
        period = 1.0 / dominant_freq if dominant_freq > 0.001 else None
        details = (f"pairwise oscillate: f={dominant_freq:.4f} Hz, "
                   f"T={period:.2f}s, peak_ratio={max_peak_ratio:.3f}, "
                   f"dist_cv={dist_variation:.4f}")

    period = 1.0 / dominant_freq if dominant_freq > 0.001 and max_peak_ratio >= 0.02 else None

    return {
        'type': result_type,
        'dominant_freq': dominant_freq,
        'period': period,
        'peak_ratio': max_peak_ratio,
        'dist_cv': dist_variation,
        'details': details
    }


def run_single_2d(n, xi0, vt0, d, mu, k1, k2, tf, label):
    """Run one 2D simulation and classify."""
    try:
        rhs, sol = integrate(xi0, vt0, d, mu, k1, k2, tf,
                             method="BDF", rtol=1e-10, atol=1e-12)
    except Exception as e:
        return None

    t_eval = np.arange(500, tf + 0.1, 0.5)  # steady state only
    pos_all, vel_all = evaluate(sol, n, t_eval)

    result = classify_2d(t_eval, pos_all, d, mu, k1, k2)
    result['label'] = label
    result['xi0'] = xi0.copy()
    result['vt0'] = vt0.copy()
    return result


def perturb_case3(n, scale, seed):
    """Perturb the case3 initial conditions."""
    np.random.seed(seed)
    xi0 = XI0_CASE3[:n].copy() + np.random.randn(n, 2) * scale
    vt0 = VT0_CASE3[:n].copy() + np.random.randn(n, 2) * scale * 0.5
    return xi0, vt0


def random_init(n, scale, seed):
    """Generate random initial conditions."""
    np.random.seed(seed)
    xi0 = np.random.randn(n, 2) * scale
    vt0 = np.random.randn(n, 2) * scale * 0.3
    return xi0, vt0


def polygon_init(n, radius, seed):
    """Regular polygon initial positions, small perturbation."""
    np.random.seed(seed)
    angles = np.linspace(0, 2*np.pi, n, endpoint=False)
    xi0 = np.column_stack([radius * np.cos(angles), radius * np.sin(angles)])
    xi0 += np.random.randn(n, 2) * 0.1
    vt0 = np.random.randn(n, 2) * 0.5
    return xi0, vt0


# ============================================================
# Main search
# ============================================================
def main():
    d = 5.0
    mu = 9.0
    k1 = 0.5
    k2 = 0.5
    tf = 1500.0

    print("2D Breathing Search")
    print(f"Parameters: d={d}, mu={mu}, k1={k1}, k2={k2}, tf={tf}")
    print("=" * 70)

    all_results = []

    # ---- Part 1: Perturb known case3 ----
    print("\n--- Part 1: Perturb case3 (N=5) ---")
    n = 5
    for scale in [0.01, 0.05, 0.1, 0.5, 1.0, 2.0]:
        for seed in range(10):
            xi0, vt0 = perturb_case3(n, scale, seed)
            label = f"case3_perturb_s{scale}_seed{seed}"
            result = run_single_2d(n, xi0, vt0, d, mu, k1, k2, tf, label)
            if result is None:
                continue
            all_results.append(result)
            if result['type'] == 'breathing':
                print(f"  ★ BREATHING: {label} → {result['details']}")
            elif result['type'] == 'rigid_rotation':
                pass  # expected, skip
            else:
                print(f"  {result['type']}: {label} → {result['details']}")

    # ---- Part 2: Random init, N=5 ----
    print("\n--- Part 2: Random init (N=5) ---")
    n = 5
    for scale in [5.0, 10.0, 15.0, 20.0]:
        for seed in range(20):
            xi0, vt0 = random_init(n, scale, seed)
            label = f"random_N5_s{scale}_seed{seed}"
            result = run_single_2d(n, xi0, vt0, d, mu, k1, k2, tf, label)
            if result is None:
                continue
            all_results.append(result)
            if result['type'] == 'breathing':
                print(f"  ★ BREATHING: {label} → {result['details']}")

    # ---- Part 3: Polygon init, N=5 ----
    print("\n--- Part 3: Polygon init (N=5) ---")
    n = 5
    for radius in [4.0, 4.5, 5.0, 5.5, 6.0, 7.0, 8.0]:
        for seed in range(5):
            xi0, vt0 = polygon_init(n, radius, seed)
            label = f"poly_N5_r{radius}_seed{seed}"
            result = run_single_2d(n, xi0, vt0, d, mu, k1, k2, tf, label)
            if result is None:
                continue
            all_results.append(result)
            if result['type'] == 'breathing':
                print(f"  ★ BREATHING: {label} → {result['details']}")

    # ---- Part 4: Different N values ----
    print("\n--- Part 4: Different N ---")
    for n in [4, 6, 7, 8]:
        for seed in range(10):
            xi0, vt0 = random_init(n, 10.0, seed)
            label = f"random_N{n}_seed{seed}"
            result = run_single_2d(n, xi0, vt0, d, mu, k1, k2, tf, label)
            if result is None:
                continue
            all_results.append(result)
            if result['type'] == 'breathing':
                print(f"  ★ BREATHING: {label} → {result['details']}")

    # ---- Part 5: Different k1 values with case3 init ----
    print("\n--- Part 5: Vary k1 with case3 init (N=5) ---")
    n = 5
    for k1_val in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
        xi0 = XI0_CASE3.copy()
        vt0 = VT0_CASE3.copy()
        label = f"case3_k1_{k1_val}"
        result = run_single_2d(n, xi0, vt0, d, mu, k1_val, k2, tf, label)
        if result is None:
            continue
        all_results.append(result)
        if result['type'] == 'breathing':
            print(f"  ★ BREATHING: {label} → {result['details']}")
        else:
            print(f"  {result['type']}: {label} → {result['details']}")

    # ---- Summary ----
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    type_counts = {}
    for r in all_results:
        t = r['type']
        type_counts[t] = type_counts.get(t, 0) + 1
    for t, c in sorted(type_counts.items()):
        print(f"  {t}: {c}")

    breathing_cases = [r for r in all_results if r['type'] == 'breathing']
    if breathing_cases:
        print(f"\n★ Found {len(breathing_cases)} breathing case(s):")
        for r in breathing_cases:
            print(f"  {r['label']}: f={r['dominant_freq']:.4f} Hz, T={r['period']:.2f}s, peak_ratio={r['peak_ratio']:.3f}")
    else:
        print("\nNo new breathing cases found.")


if __name__ == '__main__':
    main()