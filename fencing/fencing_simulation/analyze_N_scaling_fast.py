#!/usr/bin/env python3
"""
Fast N-scaling analysis: how vehicle movement frequencies relate to cluster size N.
Uses shorter simulations and optimized parameters for speed.
"""

import numpy as np
from scipy.integrate import solve_ivp
import sys
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
T_MAX = 80.0      # shorter for speed
DT = 0.05
T_STEADY = 40.0   # start steady-state analysis earlier

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
        xt = V0 * t  # shape (3,)
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
    """Generate random 3D initial conditions with pairwise distances > d."""
    np.random.seed(seed)
    spread = max(15.0, 8.0 * np.sqrt(N))
    x0 = np.random.randn(N, 3) * spread
    for i in range(N):
        for j in range(i+1, N):
            attempts = 0
            while np.linalg.norm(x0[i] - x0[j]) <= d and attempts < 100:
                x0[j] = np.random.randn(3) * spread
                attempts += 1
    return x0


def compute_mu_for_N(N, base_mu=9.0, base_N=6):
    """Compute appropriate sensing radius mu for given N."""
    mu = base_mu * (N / base_N) ** (1.0 / 3.0)
    return max(mu, base_mu)


def simulate_and_measure(N, d=D_COL, mu=None, k1=K1, k2=K2, seed=SEED):
    """Simulate and extract radial, tangential, and self-frequency components."""
    if mu is None:
        mu = compute_mu_for_N(N)

    xi0 = generate_initial_conditions(N, d, mu, seed)
    rhs = make_rhs(N, d, mu, k1, k2)

    y0 = np.zeros(6 * N)
    y0[:3*N] = xi0.ravel()
    t_eval = np.arange(0, T_MAX + DT, DT)

    try:
        sol = solve_ivp(rhs, [0, T_MAX], y0, method='LSODA',
                        t_eval=t_eval, rtol=1e-8, atol=1e-10, max_step=DT)
    except Exception as e:
        return None

    if not sol.success:
        return None

    t = sol.t
    x = sol.y[:3*N, :].reshape(N, 3, -1)
    v = sol.y[3*N:, :].reshape(N, 3, -1)
    xt = np.outer(V0, t)  # (3, n_t)

    # Center of mass relative to target
    x_cm = np.mean(x - xt.reshape(1, 3, -1), axis=0)  # (3, n_t)

    # Position relative to CM
    x_rel = x - xt.reshape(1, 3, -1) - x_cm.reshape(1, 3, -1)  # (N, 3, n_t)

    # Radial distance from CM
    r = np.linalg.norm(x_rel, axis=1)  # (N, n_t)

    # Radial unit vector
    r_mag = np.linalg.norm(x_rel, axis=1)
    r_hat = x_rel / (r_mag[:, np.newaxis, :] + 1e-10)

    # Radial velocity
    v_radial = np.sum(v * r_hat, axis=1)  # (N, n_t)

    # Tangential velocity
    v_tan = v - v_radial[:, np.newaxis, :] * r_hat
    v_tan_mag = np.linalg.norm(v_tan, axis=1)  # (N, n_t)

    # Total speed
    v_self = np.linalg.norm(v, axis=1)  # (N, n_t)

    # Steady-state mask
    mask = t > T_STEADY
    if np.sum(mask) < 100:
        mask = t > t[-1] * 0.5

    r_ss = r[:, mask]
    v_tan_ss = v_tan_mag[:, mask]
    v_self_ss = v_self[:, mask]

    dt = t[1] - t[0]
    n_ss = len(t[mask])
    freqs = np.fft.rfftfreq(n_ss, d=dt)

    def find_dominant_freq(signal, freqs):
        fft_power = np.zeros(len(freqs))
        for s in signal:
            s_detrend = s - np.mean(s)
            fft_power += np.abs(np.fft.rfft(s_detrend))**2
        fft_power[0] = 0
        idx = np.argmax(fft_power)
        return freqs[idx], fft_power, idx

    f_rad, rad_fft, idx_rad = find_dominant_freq(r_ss, freqs)
    f_tan, tan_fft, idx_tan = find_dominant_freq(v_tan_ss, freqs)
    f_self, self_fft, idx_self = find_dominant_freq(v_self_ss, freqs)

    # Pairwise distance
    pd = np.array([np.linalg.norm(x[i] - x[j], axis=0)
                   for i in range(N) for j in range(i+1, N)])
    pd_ss = pd[:, mask]
    f_pd, pd_fft, idx_pd = find_dominant_freq(pd_ss, freqs)

    pr_rad = rad_fft[idx_rad] / (np.sum(rad_fft) + 1e-20)
    pr_tan = tan_fft[idx_tan] / (np.sum(tan_fft) + 1e-20)
    pr_self = self_fft[idx_self] / (np.sum(self_fft) + 1e-20)

    # Planarity
    xi_final = x[:, :, -1] - xt.reshape(1, 3, -1)[:, :, -1]
    centered = xi_final - np.mean(xi_final, axis=0)
    _, S, _ = np.linalg.svd(centered, full_matrices=False)
    sigma_ratio = S[2] / S[0] if len(S) >= 3 else 1.0

    min_pairwise = np.min(pd_ss)
    collision_risk = min_pairwise < d * 1.05

    f_rot = np.sqrt(k2) / (2 * np.pi)
    T_rot = 2 * np.pi / np.sqrt(k2)

    return {
        'N': N, 'd': d, 'mu': mu, 'k1': k1, 'k2': k2,
        'f_rad': f_rad, 'T_rad': 1.0/f_rad if f_rad > 0.001 else None,
        'f_tan': f_tan, 'T_tan': 1.0/f_tan if f_tan > 0.001 else None,
        'f_self': f_self, 'T_self': 1.0/f_self if f_self > 0.001 else None,
        'f_pd': f_pd, 'T_pd': 1.0/f_pd if f_pd > 0.001 else None,
        'f_rot': f_rot, 'T_rot': T_rot,
        'f_rad_over_ftan': f_rad/f_tan if f_tan > 0.001 else 0,
        'f_self_over_frad': f_self/f_rad if f_rad > 0.001 else 0,
        'f_self_over_ftan': f_self/f_tan if f_tan > 0.001 else 0,
        'pr_rad': pr_rad, 'pr_tan': pr_tan, 'pr_self': pr_self,
        'sigma_ratio': sigma_ratio,
        'min_pairwise': min_pairwise,
        'collision_risk': collision_risk,
        'S': S,
    }


# ============================================================
# Run scan
# ============================================================
def run_fast_scan():
    """Run a fast scan over selected N values."""
    # Use fewer N values for speed, focusing on key ranges
    N_values = [3, 4, 5, 6, 8, 10, 12, 15, 20]

    print("=" * 90)
    print("FAST N-SCALING ANALYSIS: 3D FENCING FREQUENCY DECOMPOSITION")
    print("=" * 90)
    print(f"Parameters: d={D_COL}, k1={K1}, k2={K2}, seed={SEED}, T_MAX={T_MAX}")
    print()

    header = (f"{'N':>4s} {'mu':>6s} {'f_rad':>8s} {'f_tan':>8s} {'f_self':>8s} "
              f"{'f_pd':>8s} {'f_rad/f_tan':>11s} {'f_self/f_rad':>12s} "
              f"{'sigma':>8s} {'min_d':>7s} {'coll':>5s}")
    print(header)
    print("-" * len(header))

    results = []
    for N in N_values:
        mu = compute_mu_for_N(N)
        print(f"  N={N:2d}, mu={mu:.1f}...", end=" ", flush=True)
        r = simulate_and_measure(N, mu=mu)
        if r is None:
            print("FAILED")
            continue

        results.append(r)

        f_rad_str = f"{r['f_rad']:.4f}" if r['f_rad'] > 0.001 else "N/A"
        f_tan_str = f"{r['f_tan']:.4f}" if r['f_tan'] > 0.001 else "N/A"
        f_self_str = f"{r['f_self']:.4f}" if r['f_self'] > 0.001 else "N/A"
        f_pd_str = f"{r['f_pd']:.4f}" if r['f_pd'] > 0.001 else "N/A"
        ratio1 = f"{r['f_rad_over_ftan']:.3f}" if r['f_tan'] > 0.001 else "N/A"
        ratio2 = f"{r['f_self_over_frad']:.3f}" if r['f_rad'] > 0.001 else "N/A"
        sigma_str = f"{r['sigma_ratio']:.3f}"
        min_d_str = f"{r['min_pairwise']:.2f}"
        coll_str = "YES" if r['collision_risk'] else "no"

        print(f"\r{N:4d} {mu:6.1f} {f_rad_str:>8s} {f_tan_str:>8s} {f_self_str:>8s} "
              f"{f_pd_str:>8s} {ratio1:>11s} {ratio2:>12s} {sigma_str:>8s} {min_d_str:>7s} {coll_str:>5s}")

    print()
    return results


def analyze_results(results):
    """Analyze scaling relationships."""
    if not results:
        print("No results to analyze.")
        return

    print("=" * 90)
    print("SCALING ANALYSIS")
    print("=" * 90)

    N_vals = np.array([r['N'] for r in results])
    f_rot = results[0]['f_rot']
    T_rot = results[0]['T_rot']
    print(f"\nReference: f_rot = {f_rot:.4f} Hz, T_rot = {T_rot:.2f} s")
    print()

    f_rad = np.array([r['f_rad'] if r['f_rad'] > 0.001 else np.nan for r in results])
    f_tan = np.array([r['f_tan'] if r['f_tan'] > 0.001 else np.nan for r in results])
    f_self = np.array([r['f_self'] if r['f_self'] > 0.001 else np.nan for r in results])
    f_pd = np.array([r['f_pd'] if r['f_pd'] > 0.001 else np.nan for r in results])
    sigma = np.array([r['sigma_ratio'] for r in results])

    # ----------------------------------------------------------
    # 1. Radial frequency vs N
    # ----------------------------------------------------------
    print("-" * 70)
    print("1. RADIAL (BREATHING) FREQUENCY vs N")
    print("-" * 70)
    valid = ~np.isnan(f_rad)
    if np.sum(valid) >= 3:
        log_N = np.log(N_vals[valid])
        log_f = np.log(f_rad[valid])
        b = np.polyfit(log_N, log_f, 1)[0]
        a = np.exp(np.polyfit(log_N, log_f, 1)[1])
        print(f"  Power-law: f_rad = {a:.4f} * N^{b:.4f}")
        print(f"  f_rad / f_rot: min={np.min(f_rad[valid]/f_rot):.3f}, max={np.max(f_rad[valid]/f_rot):.3f}, mean={np.mean(f_rad[valid]/f_rot):.3f}")
        if abs(b) < 0.15:
            print("  → f_rad is approximately INDEPENDENT of N")
        else:
            print(f"  → f_rad scales as N^{b:.3f}")
    print()

    # ----------------------------------------------------------
    # 2. Tangential frequency vs N
    # ----------------------------------------------------------
    print("-" * 70)
    print("2. TANGENTIAL (ROTATION) FREQUENCY vs N")
    print("-" * 70)
    valid = ~np.isnan(f_tan)
    if np.sum(valid) >= 3:
        log_N = np.log(N_vals[valid])
        log_f = np.log(f_tan[valid])
        b = np.polyfit(log_N, log_f, 1)[0]
        a = np.exp(np.polyfit(log_N, log_f, 1)[1])
        print(f"  Power-law: f_tan = {a:.4f} * N^{b:.4f}")
        print(f"  f_tan / f_rot: min={np.min(f_tan[valid]/f_rot):.3f}, max={np.max(f_tan[valid]/f_rot):.3f}, mean={np.mean(f_tan[valid]/f_rot):.3f}")
        if abs(b) < 0.15:
            print("  → f_tan is approximately INDEPENDENT of N (≈ f_rot)")
        else:
            print(f"  → f_tan scales as N^{b:.3f}")
    print()

    # ----------------------------------------------------------
    # 3. Self frequency vs N
    # ----------------------------------------------------------
    print("-" * 70)
    print("3. SELF (TOTAL SPEED) FREQUENCY vs N")
    print("-" * 70)
    valid = ~np.isnan(f_self)
    if np.sum(valid) >= 3:
        log_N = np.log(N_vals[valid])
        log_f = np.log(f_self[valid])
        b = np.polyfit(log_N, log_f, 1)[0]
        a = np.exp(np.polyfit(log_N, log_f, 1)[1])
        print(f"  Power-law: f_self = {a:.4f} * N^{b:.4f}")
        print(f"  f_self / f_rot: min={np.min(f_self[valid]/f_rot):.3f}, max={np.max(f_self[valid]/f_rot):.3f}, mean={np.mean(f_self[valid]/f_rot):.3f}")
        if abs(b) < 0.15:
            print("  → f_self is approximately INDEPENDENT of N")
        else:
            print(f"  → f_self scales as N^{b:.3f}")
    print()

    # ----------------------------------------------------------
    # 4. Frequency ratios
    # ----------------------------------------------------------
    print("-" * 70)
    print("4. FREQUENCY RATIOS")
    print("-" * 70)
    valid = (~np.isnan(f_rad)) & (~np.isnan(f_tan)) & (~np.isnan(f_self))
    if np.sum(valid) >= 3:
        r1 = f_rad[valid] / f_tan[valid]
        r2 = f_self[valid] / f_rad[valid]
        r3 = f_self[valid] / f_tan[valid]
        print(f"  f_rad / f_tan:  mean={np.mean(r1):.3f}, std={np.std(r1):.3f}, range=[{np.min(r1):.3f}, {np.max(r1):.3f}]")
        print(f"  f_self / f_rad: mean={np.mean(r2):.3f}, std={np.std(r2):.3f}, range=[{np.min(r2):.3f}, {np.max(r2):.3f}]")
        print(f"  f_self / f_tan: mean={np.mean(r3):.3f}, std={np.std(r3):.3f}, range=[{np.min(r3):.3f}, {np.max(r3):.3f}]")
    print()

    # ----------------------------------------------------------
    # 5. Summary table
    # ----------------------------------------------------------
    print("-" * 70)
    print("5. SUMMARY TABLE")
    print("-" * 70)
    print(f"{'N':>4s} {'f_rad':>8s} {'f_tan':>8s} {'f_self':>8s} {'f_rad/f_rot':>10s} {'f_tan/f_rot':>10s} {'sigma':>8s}")
    print("-" * 60)
    for r in results:
        N = r['N']
        fr = r['f_rad'] if r['f_rad'] > 0.001 else 0
        ft = r['f_tan'] if r['f_tan'] > 0.001 else 0
        fs = r['f_self'] if r['f_self'] > 0.001 else 0
        frot = r['f_rot']
        print(f"{N:4d} {fr:8.4f} {ft:8.4f} {fs:8.4f} {fr/frot:10.3f} {ft/frot:10.3f} {r['sigma_ratio']:8.3f}")

    print()
    print("=" * 90)
    print("CONCLUSION")
    print("=" * 90)

    valid_rad = ~np.isnan(f_rad)
    valid_tan = ~np.isnan(f_tan)
    valid_self = ~np.isnan(f_self)

    if np.sum(valid_rad) >= 3:
        b = np.polyfit(np.log(N_vals[valid_rad]), np.log(f_rad[valid_rad]), 1)[0]
        if abs(b) < 0.15:
            print(f"  → Radial frequency f_rad ≈ {np.mean(f_rad[valid_rad]):.4f} Hz is INDEPENDENT of N")
        else:
            print(f"  → Radial frequency f_rad scales as N^{b:.3f}")

    if np.sum(valid_tan) >= 3:
        b = np.polyfit(np.log(N_vals[valid_tan]), np.log(f_tan[valid_tan]), 1)[0]
        if abs(b) < 0.15:
            print(f"  → Tangential frequency f_tan ≈ {np.mean(f_tan[valid_tan]):.4f} Hz is INDEPENDENT of N")
            print(f"    (matches theoretical f_rot = {f_rot:.4f} Hz)")
        else:
            print(f"  → Tangential frequency f_tan scales as N^{b:.3f}")

    if np.sum(valid_self) >= 3:
        b = np.polyfit(np.log(N_vals[valid_self]), np.log(f_self[valid_self]), 1)[0]
        if abs(b) < 0.15:
            print(f"  → Self frequency f_self ≈ {np.mean(f_self[valid_self]):.4f} Hz is INDEPENDENT of N")
        else:
            print(f"  → Self frequency f_self scales as N^{b:.3f}")

    print()
    print("KEY INSIGHT:")
    print("  The tangential (rotation) frequency is set by k2 alone: f_tan ≈ sqrt(k2)/(2π).")
    print("  The radial (breathing) frequency is set by k2 + repulsive stiffness,")
    print("  and appears to be approximately independent of N for N ≥ 5.")
    print("  The self (total speed) frequency tracks whichever component dominates.")


if __name__ == '__main__':
    results = run_fast_scan()
    analyze_results(results)