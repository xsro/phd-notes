#!/usr/bin/env python3
"""
Analyze how vehicle movement frequencies (radial/breathing, tangential/rotation, self)
scale with cluster size N in the 3D Kou-Chen-Xiang fencing controller.

For each N, we:
  1. Simulate the system to steady state
  2. Decompose each vehicle's motion into:
     - radial: distance from formation center (breathing mode)
     - tangential: velocity perpendicular to radial direction (rotation mode)
     - self: total speed of each vehicle (combined motion)
  3. FFT each component to find dominant frequencies
  4. Report f_rad, f_tan, f_self and their ratios to f_rot = sqrt(k2)/(2π)

We scan N from 3 to ~20, adjusting mu to maintain collision-free formations.
"""

import numpy as np
from scipy.integrate import solve_ivp
import sys
import warnings
warnings.filterwarnings('ignore')

# ============================================================
# Parameters (default values from README)
# ============================================================
D_COL = 5.0      # collision distance
K1 = 0.5          # attractive gain
K2 = 0.5          # observer gain
V0 = np.array([1.0, 0.0, 0.0])  # target velocity
SEED = 0
T_MAX = 120.0     # simulation time
DT = 0.05         # time step
T_STEADY = 60.0   # time to start steady-state analysis

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
    # Scale initial spread with N to help with convergence
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
    """
    Compute appropriate sensing radius mu for given N.
    For larger N, we need larger mu to avoid overcrowding.
    Scaling: mu ~ N^(1/3) to maintain similar density.
    """
    # Empirical scaling: mu needs to grow with N to avoid collisions
    # For N=6, mu=9 works. For N=30, mu=15 was needed.
    # Use a smooth scaling
    mu = base_mu * (N / base_N) ** (1.0 / 3.0)
    # Ensure mu is at least base_mu
    return max(mu, base_mu)


def simulate_and_measure(N, d=D_COL, mu=None, k1=K1, k2=K2, seed=SEED):
    """
    Simulate the 3D fencing system for given parameters and extract
    radial, tangential, and self-frequency components.

    Returns dict with frequencies, periods, and planarity metrics.
    """
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
        print(f"    Simulation failed: {e}")
        return None

    if not sol.success:
        print(f"    Solver did not converge: {sol.message}")
        return None

    t = sol.t
    x = sol.y[:3*N, :].reshape(N, 3, -1)
    v = sol.y[3*N:, :].reshape(N, 3, -1)
    xt = np.zeros((3, len(t))) + np.outer(V0, t)

    # ============================================================
    # Motion decomposition
    # ============================================================
    # Center of mass relative to target
    x_cm = np.mean(x - xt.reshape(1, 3, -1), axis=0)  # (3, n_t)

    # Position relative to CM
    x_rel = x - xt.reshape(1, 3, -1) - x_cm.reshape(1, 3, -1)  # (N, 3, n_t)

    # Radial distance from CM for each vehicle
    r = np.linalg.norm(x_rel, axis=1)  # (N, n_t)

    # Radial unit vector
    r_mag = np.linalg.norm(x_rel, axis=1)  # (N, n_t)
    r_hat = x_rel / (r_mag[:, np.newaxis, :] + 1e-10)  # (N, 3, n_t)

    # Radial velocity: v · r_hat
    v_radial = np.sum(v * r_hat, axis=1)  # (N, n_t)

    # Tangential velocity: v - (v·r_hat)*r_hat
    v_tan = v - v_radial[:, np.newaxis, :] * r_hat  # (N, 3, n_t)
    v_tan_mag = np.linalg.norm(v_tan, axis=1)  # (N, n_t)

    # Total speed (self)
    v_self = np.linalg.norm(v, axis=1)  # (N, n_t)

    # ============================================================
    # Steady-state analysis
    # ============================================================
    mask = t > T_STEADY
    if np.sum(mask) < 100:
        print(f"    Warning: only {np.sum(mask)} steady-state points")
        mask = t > t[-1] * 0.5

    r_ss = r[:, mask]
    v_rad_ss = v_radial[:, mask]
    v_tan_ss = v_tan_mag[:, mask]
    v_self_ss = v_self[:, mask]

    dt = t[1] - t[0]
    n_ss = len(t[mask])
    freqs = np.fft.rfftfreq(n_ss, d=dt)

    # ----------------------------------------------------------
    # FFT analysis: find dominant frequency for each component
    # ----------------------------------------------------------
    def find_dominant_freq(signal, freqs):
        """Find dominant frequency in signal (averaged over vehicles)."""
        fft_power = np.zeros(len(freqs))
        for s in signal:
            s_detrend = s - np.mean(s)
            fft_power += np.abs(np.fft.rfft(s_detrend))**2
        fft_power[0] = 0  # ignore DC
        idx = np.argmax(fft_power)
        return freqs[idx], fft_power, idx

    # Radial (breathing) frequency
    f_rad, rad_fft, idx_rad = find_dominant_freq(r_ss, freqs)
    # Tangential frequency
    f_tan, tan_fft, idx_tan = find_dominant_freq(v_tan_ss, freqs)
    # Self (total speed) frequency
    f_self, self_fft, idx_self = find_dominant_freq(v_self_ss, freqs)
    # Pairwise distance frequency (for comparison)
    pd = np.array([np.linalg.norm(x[i] - x[j], axis=0)
                   for i in range(N) for j in range(i+1, N)])
    pd_ss = pd[:, mask]
    f_pd, pd_fft, idx_pd = find_dominant_freq(pd_ss, freqs)

    # ----------------------------------------------------------
    # Peak ratios (how dominant the peak is)
    # ----------------------------------------------------------
    pr_rad = rad_fft[idx_rad] / (np.sum(rad_fft) + 1e-20)
    pr_tan = tan_fft[idx_tan] / (np.sum(tan_fft) + 1e-20)
    pr_self = self_fft[idx_self] / (np.sum(self_fft) + 1e-20)

    # ----------------------------------------------------------
    # Planarity (SVD of final positions relative to target)
    # ----------------------------------------------------------
    xi_final = x[:, :, -1] - xt.reshape(1, 3, -1)[:, :, -1]
    centered = xi_final - np.mean(xi_final, axis=0)
    _, S, _ = np.linalg.svd(centered, full_matrices=False)
    sigma_ratio = S[2] / S[0] if len(S) >= 3 else 1.0

    # ----------------------------------------------------------
    # Check for collisions
    # ----------------------------------------------------------
    min_pairwise = np.min(pd_ss)
    collision_risk = min_pairwise < d * 1.05

    # ----------------------------------------------------------
    # Rigid rotation reference frequency
    # ----------------------------------------------------------
    f_rot = np.sqrt(k2) / (2 * np.pi)
    T_rot = 2 * np.pi / np.sqrt(k2)

    return {
        'N': N,
        'd': d,
        'mu': mu,
        'k1': k1,
        'k2': k2,
        'f_rad': f_rad,
        'T_rad': 1.0 / f_rad if f_rad > 0.001 else None,
        'f_tan': f_tan,
        'T_tan': 1.0 / f_tan if f_tan > 0.001 else None,
        'f_self': f_self,
        'T_self': 1.0 / f_self if f_self > 0.001 else None,
        'f_pd': f_pd,
        'T_pd': 1.0 / f_pd if f_pd > 0.001 else None,
        'f_rot': f_rot,
        'T_rot': T_rot,
        'f_rad_over_ftan': f_rad / f_tan if f_tan > 0.001 else 0,
        'f_self_over_frad': f_self / f_rad if f_rad > 0.001 else 0,
        'f_self_over_ftan': f_self / f_tan if f_tan > 0.001 else 0,
        'pr_rad': pr_rad,
        'pr_tan': pr_tan,
        'pr_self': pr_self,
        'sigma_ratio': sigma_ratio,
        'min_pairwise': min_pairwise,
        'collision_risk': collision_risk,
        'S': S,
        'freqs': freqs,
        'rad_fft': rad_fft,
        'tan_fft': tan_fft,
        'self_fft': self_fft,
        'r_ss': r_ss,
        'v_tan_ss': v_tan_ss,
        'v_self_ss': v_self_ss,
    }


# ============================================================
# Main N-scan
# ============================================================
def run_N_scan(N_values=None):
    """Scan N values and report frequency scaling."""
    if N_values is None:
        # Scan from 3 to 20
        N_values = list(range(3, 21))

    print("=" * 100)
    print("N-SCALING ANALYSIS: 3D FENCING CONTROLLER FREQUENCY DECOMPOSITION")
    print("=" * 100)
    print(f"Parameters: d={D_COL}, k1={K1}, k2={K2}, seed={SEED}, T_MAX={T_MAX}")
    print()

    # Header
    header = (f"{'N':>4s} {'mu':>6s} {'f_rad':>8s} {'T_rad':>8s} {'f_tan':>8s} {'T_tan':>8s} "
              f"{'f_self':>8s} {'T_self':>8s} {'f_pd':>8s} {'f_rad/f_tan':>11s} "
              f"{'f_self/f_rad':>12s} {'f_self/f_tan':>12s} {'sigma':>8s} {'min_d':>8s} {'coll':>5s}")
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

        # Format output
        f_rad_str = f"{r['f_rad']:.4f}" if r['f_rad'] > 0.001 else "N/A"
        T_rad_str = f"{r['T_rad']:.2f}" if r['T_rad'] else "N/A"
        f_tan_str = f"{r['f_tan']:.4f}" if r['f_tan'] > 0.001 else "N/A"
        T_tan_str = f"{r['T_tan']:.2f}" if r['T_tan'] else "N/A"
        f_self_str = f"{r['f_self']:.4f}" if r['f_self'] > 0.001 else "N/A"
        T_self_str = f"{r['T_self']:.2f}" if r['T_self'] else "N/A"
        f_pd_str = f"{r['f_pd']:.4f}" if r['f_pd'] > 0.001 else "N/A"
        ratio1 = f"{r['f_rad_over_ftan']:.3f}" if r['f_tan'] > 0.001 else "N/A"
        ratio2 = f"{r['f_self_over_frad']:.3f}" if r['f_rad'] > 0.001 else "N/A"
        ratio3 = f"{r['f_self_over_ftan']:.3f}" if r['f_tan'] > 0.001 else "N/A"
        sigma_str = f"{r['sigma_ratio']:.3f}"
        min_d_str = f"{r['min_pairwise']:.2f}"
        coll_str = "YES" if r['collision_risk'] else "no"

        print(f"\r{N:4d} {mu:6.1f} {f_rad_str:>8s} {T_rad_str:>8s} {f_tan_str:>8s} {T_tan_str:>8s} "
              f"{f_self_str:>8s} {T_self_str:>8s} {f_pd_str:>8s} {ratio1:>11s} "
              f"{ratio2:>12s} {ratio3:>12s} {sigma_str:>8s} {min_d_str:>8s} {coll_str:>5s}")

    print()
    return results


def analyze_scaling(results):
    """Analyze how frequencies scale with N."""
    print("=" * 100)
    print("SCALING ANALYSIS")
    print("=" * 100)

    if not results:
        print("No results to analyze.")
        return

    N_vals = np.array([r['N'] for r in results])
    f_rot = results[0]['f_rot']
    T_rot = results[0]['T_rot']
    print(f"\nReference: f_rot = {f_rot:.4f} Hz, T_rot = {T_rot:.2f} s (from k2={results[0]['k2']})")
    print()

    # Extract arrays
    f_rad = np.array([r['f_rad'] if r['f_rad'] > 0.001 else np.nan for r in results])
    f_tan = np.array([r['f_tan'] if r['f_tan'] > 0.001 else np.nan for r in results])
    f_self = np.array([r['f_self'] if r['f_self'] > 0.001 else np.nan for r in results])
    f_pd = np.array([r['f_pd'] if r['f_pd'] > 0.001 else np.nan for r in results])
    sigma = np.array([r['sigma_ratio'] for r in results])
    min_d = np.array([r['min_pairwise'] for r in results])

    # ----------------------------------------------------------
    # 1. Does f_rad scale with N?
    # ----------------------------------------------------------
    print("-" * 80)
    print("1. RADIAL (BREATHING) FREQUENCY vs N")
    print("-" * 80)
    valid = ~np.isnan(f_rad)
    if np.sum(valid) >= 3:
        # Fit power law: f_rad = a * N^b
        log_N = np.log(N_vals[valid])
        log_f = np.log(f_rad[valid])
        coeffs = np.polyfit(log_N, log_f, 1)
        b_rad = coeffs[0]
        a_rad = np.exp(coeffs[1])
        print(f"  Power-law fit: f_rad = {a_rad:.4f} * N^{b_rad:.4f}")
        print(f"  Scaling exponent b = {b_rad:.4f}")

        if abs(b_rad) < 0.1:
            print("  → f_rad is approximately CONSTANT with N (no strong N-dependence)")
        elif b_rad > 0:
            print(f"  → f_rad INCREASES with N (sub-linear, b < 1)")
        else:
            print(f"  → f_rad DECREASES with N")

        # Check ratio to f_rot
        ratio_to_rot = f_rad[valid] / f_rot
        print(f"  f_rad / f_rot range: {np.min(ratio_to_rot):.3f} – {np.max(ratio_to_rot):.3f}")
        print(f"  Mean f_rad / f_rot: {np.mean(ratio_to_rot):.3f}")
    print()

    # ----------------------------------------------------------
    # 2. Does f_tan scale with N?
    # ----------------------------------------------------------
    print("-" * 80)
    print("2. TANGENTIAL (ROTATION) FREQUENCY vs N")
    print("-" * 80)
    valid = ~np.isnan(f_tan)
    if np.sum(valid) >= 3:
        log_N = np.log(N_vals[valid])
        log_f = np.log(f_tan[valid])
        coeffs = np.polyfit(log_N, log_f, 1)
        b_tan = coeffs[0]
        a_tan = np.exp(coeffs[1])
        print(f"  Power-law fit: f_tan = {a_tan:.4f} * N^{b_tan:.4f}")
        print(f"  Scaling exponent b = {b_tan:.4f}")

        if abs(b_tan) < 0.1:
            print("  → f_tan is approximately CONSTANT with N")
        elif b_tan > 0:
            print(f"  → f_tan INCREASES with N")
        else:
            print(f"  → f_tan DECREASES with N")

        ratio_to_rot = f_tan[valid] / f_rot
        print(f"  f_tan / f_rot range: {np.min(ratio_to_rot):.3f} – {np.max(ratio_to_rot):.3f}")
        print(f"  Mean f_tan / f_rot: {np.mean(ratio_to_rot):.3f}")
        print(f"  (Theoretical f_tan = f_rot = {f_rot:.4f} Hz for rigid rotation)")
    print()

    # ----------------------------------------------------------
    # 3. Does f_self scale with N?
    # ----------------------------------------------------------
    print("-" * 80)
    print("3. SELF (TOTAL SPEED) FREQUENCY vs N")
    print("-" * 80)
    valid = ~np.isnan(f_self)
    if np.sum(valid) >= 3:
        log_N = np.log(N_vals[valid])
        log_f = np.log(f_self[valid])
        coeffs = np.polyfit(log_N, log_f, 1)
        b_self = coeffs[0]
        a_self = np.exp(coeffs[1])
        print(f"  Power-law fit: f_self = {a_self:.4f} * N^{b_self:.4f}")
        print(f"  Scaling exponent b = {b_self:.4f}")

        if abs(b_self) < 0.1:
            print("  → f_self is approximately CONSTANT with N")
        elif b_self > 0:
            print(f"  → f_self INCREASES with N")
        else:
            print(f"  → f_self DECREASES with N")

        ratio_to_rot = f_self[valid] / f_rot
        print(f"  f_self / f_rot range: {np.min(ratio_to_rot):.3f} – {np.max(ratio_to_rot):.3f}")
        print(f"  Mean f_self / f_rot: {np.mean(ratio_to_rot):.3f}")
    print()

    # ----------------------------------------------------------
    # 4. Frequency ratios
    # ----------------------------------------------------------
    print("-" * 80)
    print("4. FREQUENCY RATIOS vs N")
    print("-" * 80)
    valid = (~np.isnan(f_rad)) & (~np.isnan(f_tan)) & (~np.isnan(f_self))
    if np.sum(valid) >= 3:
        ratio_rad_tan = f_rad[valid] / f_tan[valid]
        ratio_self_rad = f_self[valid] / f_rad[valid]
        ratio_self_tan = f_self[valid] / f_tan[valid]

        print(f"  f_rad / f_tan:  mean={np.mean(ratio_rad_tan):.3f}, std={np.std(ratio_rad_tan):.3f}, "
              f"range=[{np.min(ratio_rad_tan):.3f}, {np.max(ratio_rad_tan):.3f}]")
        print(f"  f_self / f_rad: mean={np.mean(ratio_self_rad):.3f}, std={np.std(ratio_self_rad):.3f}, "
              f"range=[{np.min(ratio_self_rad):.3f}, {np.max(ratio_self_rad):.3f}]")
        print(f"  f_self / f_tan: mean={np.mean(ratio_self_tan):.3f}, std={np.std(ratio_self_tan):.3f}, "
              f"range=[{np.min(ratio_self_tan):.3f}, {np.max(ratio_self_tan):.3f}]")

        # Check if ratios are constant
        if np.std(ratio_rad_tan) / np.mean(ratio_rad_tan) < 0.1:
            print("  → f_rad/f_tan is approximately CONSTANT (frequency locking)")
        else:
            print("  → f_rad/f_tan VARIES with N (no locking)")
    print()

    # ----------------------------------------------------------
    # 5. Planarity vs N
    # ----------------------------------------------------------
    print("-" * 80)
    print("5. PLANARITY (sigma_3/sigma_1) vs N")
    print("-" * 80)
    print(f"  Range: {np.min(sigma):.3f} – {np.max(sigma):.3f}")
    print(f"  Mean: {np.mean(sigma):.3f}")
    if np.std(sigma) / np.mean(sigma) < 0.2:
        print("  → Planarity is approximately CONSTANT with N")
    else:
        print("  → Planarity VARIES with N")
        # Check trend
        if np.polyfit(N_vals, sigma, 1)[0] > 0:
            print("  → Formation becomes MORE non-planar as N increases")
        else:
            print("  → Formation becomes LESS non-planar as N increases")
    print()

    # ----------------------------------------------------------
    # 6. Summary table
    # ----------------------------------------------------------
    print("-" * 80)
    print("6. SUMMARY: FREQUENCY SCALING WITH N")
    print("-" * 80)
    print(f"{'N':>4s} {'f_rad (Hz)':>10s} {'f_tan (Hz)':>10s} {'f_self (Hz)':>10s} "
          f"{'f_rad/f_rot':>10s} {'f_tan/f_rot':>10s} {'f_self/f_rot':>10s} {'sigma':>8s}")
    print("-" * 70)
    for r in results:
        N = r['N']
        f_rad = r['f_rad'] if r['f_rad'] > 0.001 else 0
        f_tan = r['f_tan'] if r['f_tan'] > 0.001 else 0
        f_self = r['f_self'] if r['f_self'] > 0.001 else 0
        f_rot = r['f_rot']
        print(f"{N:4d} {f_rad:10.4f} {f_tan:10.4f} {f_self:10.4f} "
              f"{f_rad/f_rot:10.3f} {f_tan/f_rot:10.3f} {f_self/f_rot:10.3f} "
              f"{r['sigma_ratio']:8.3f}")

    print()
    print("=" * 100)
    print("CONCLUSION")
    print("=" * 100)

    # Determine if frequencies depend on N
    valid_rad = ~np.isnan(f_rad)
    valid_tan = ~np.isnan(f_tan)
    valid_self = ~np.isnan(f_self)

    if np.sum(valid_rad) >= 3:
        b = np.polyfit(np.log(N_vals[valid_rad]), np.log(f_rad[valid_rad]), 1)[0]
        if abs(b) < 0.15:
            print("  → Radial frequency f_rad is INDEPENDENT of cluster size N")
            print(f"    (f_rad ≈ {np.mean(f_rad[valid_rad]):.4f} Hz, scaling as N^{b:.3f})")
        else:
            print(f"  → Radial frequency f_rad DEPENDS on N (scaling as N^{b:.3f})")

    if np.sum(valid_tan) >= 3:
        b = np.polyfit(np.log(N_vals[valid_tan]), np.log(f_tan[valid_tan]), 1)[0]
        if abs(b) < 0.15:
            print("  → Tangential frequency f_tan is INDEPENDENT of cluster size N")
            print(f"    (f_tan ≈ {np.mean(f_tan[valid_tan]):.4f} Hz, scaling as N^{b:.3f})")
        else:
            print(f"  → Tangential frequency f_tan DEPENDS on N (scaling as N^{b:.3f})")

    if np.sum(valid_self) >= 3:
        b = np.polyfit(np.log(N_vals[valid_self]), np.log(f_self[valid_self]), 1)[0]
        if abs(b) < 0.15:
            print("  → Self frequency f_self is INDEPENDENT of cluster size N")
            print(f"    (f_self ≈ {np.mean(f_self[valid_self]):.4f} Hz, scaling as N^{b:.3f})")
        else:
            print(f"  → Self frequency f_self DEPENDS on N (scaling as N^{b:.3f})")

    print()


if __name__ == '__main__':
    results = run_N_scan()
    analyze_scaling(results)