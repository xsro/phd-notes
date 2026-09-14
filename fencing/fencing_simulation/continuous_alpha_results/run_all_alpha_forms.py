#!/usr/bin/env python3
"""
Run parameter scans for ALL 6 continuous alpha function forms and save results.

Forms:
  1. standard:    α(s) = 1/(s-d) - 1/(μ-d)
  2. power:       α(s) = 1/(s-d)^p - 1/(μ-d)^p   (p=1.5)
  3. rational:    α(s) = (μ-s)/(s-d)
  4. log:         α(s) = ln((μ-d)/(s-d))
  5. stiff:       α(s) = 1/(s-d)^2 - 1/(μ-d)^2
  6. exponential: α(s) = exp(1/(s-d)) * (μ-s)/(μ-d)

For each form, scans:
  - k1 (k2=0.5 fixed)
  - k2 (k1=0.5 fixed)
  - d_col (mu=9.0)
  - mu (d=5.0)
  - N (k1=k2=0.5)
  - (k1, k2) grid

Results saved to continuous_alpha_results/ as CSV and JSON.
"""

import sys
import json
import csv
import time
import numpy as np
from pathlib import Path
from scipy.integrate import solve_ivp

sys.path.insert(0, str(Path(__file__).resolve().parent))

# ============================================================
# Output directory
# ============================================================
OUT_DIR = Path(__file__).parent / "continuous_alpha_results"
OUT_DIR.mkdir(exist_ok=True)

# ============================================================
# Known 3D breathing ICs (seed=0, N=6)
# ============================================================
XI0_BASE = np.array([
    [14.112419,  3.201258,  7.829904],
    [17.927146, 14.940464, -7.818223],
    [ 7.600707, -1.210858, -0.825751],
    [ 3.284788,  1.152349, 11.634188],
    [ 6.088302,  0.973400,  3.550906],
    [ 2.669395, 11.952633, -1.641266],
])


# ============================================================
# Alpha function definitions
# ============================================================
def make_alpha(form, d_col, mu, power=1.5):
    """Create a continuous alpha function."""
    if form == "standard":
        def alpha(s):
            if s >= mu: return 0.0
            if s <= d_col: return 1e6
            return 1.0/(s - d_col) - 1.0/(mu - d_col)
        return alpha

    elif form == "power":
        def alpha(s):
            if s >= mu: return 0.0
            if s <= d_col: return 1e6
            return 1.0/(s - d_col)**power - 1.0/(mu - d_col)**power
        return alpha

    elif form == "rational":
        def alpha(s):
            if s >= mu: return 0.0
            if s <= d_col: return 1e6
            return (mu - s) / (s - d_col)
        return alpha

    elif form == "log":
        def alpha(s):
            if s >= mu: return 0.0
            if s <= d_col: return 1e6
            return np.log((mu - d_col) / (s - d_col))
        return alpha

    elif form == "stiff":
        def alpha(s):
            if s >= mu: return 0.0
            if s <= d_col: return 1e6
            return 1.0/(s - d_col)**2 - 1.0/(mu - d_col)**2
        return alpha

    elif form == "exponential":
        def alpha(s):
            if s >= mu: return 0.0
            if s <= d_col: return 1e6
            return np.exp(1.0/(s - d_col)) * (mu - s) / (mu - d_col)
        return alpha

    else:
        raise ValueError(f"Unknown form: {form}")


# ============================================================
# Simulation and measurement
# ============================================================
def make_rhs(N, d_col, mu, k1, k2, v0, alpha_func):
    # alpha_func is already the alpha(s) function from make_alpha
    alpha = alpha_func
    def rhs(t, y):
        x = y[:3*N].reshape(N, 3)
        v = y[3*N:].reshape(N, 3)
        x_t = np.zeros(3) + v0 * t
        phi = np.zeros((N, 3))
        for i in range(N):
            for j in range(N):
                if i == j: continue
                xij = x[i] - x[j]
                dist = np.linalg.norm(xij)
                if d_col < dist <= mu:
                    phi[i] += alpha(dist) * xij / dist
        u = phi + k1 * (x_t - x) + v
        v_dot = k2 * (x_t - x)
        return np.concatenate([u.ravel(), v_dot.ravel()])
    return rhs


def generate_scaled_init(N_target, N_base=6, scale_factor=1.0, seed=0):
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


def run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_form, T_MAX, DT, alpha_power=1.5):
    """Run simulation and measure metrics."""
    alpha_func = make_alpha(alpha_form, d_col, mu, power=alpha_power)
    rhs = make_rhs(N, d_col, mu, k1, k2, v0, alpha_func)
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
    x_traj = sol.y[:3*N, :].reshape(N, 3, -1)
    x_target_traj = np.zeros((3, len(t))) + np.outer(v0, t)

    # Pairwise distances
    pd = []
    for i in range(N):
        for j in range(i+1, N):
            pd.append(np.linalg.norm(x_traj[i] - x_traj[j], axis=0))
    pd = np.array(pd)

    # Steady state: last 50%
    mask = t > (t[-1] - t[-1] * 0.5)
    pd_ss = pd[:, mask]
    T_ss = len(t[mask])

    # FFT
    freqs = np.fft.rfftfreq(T_ss, d=DT)
    ap = np.zeros(len(freqs))
    for p in pd_ss:
        p_detrend = p - np.mean(p)
        ap += np.abs(np.fft.rfft(p_detrend))**2
    ap[0] = 0
    idx = np.argmax(ap)
    f = freqs[idx]
    pr = ap[idx] / (np.sum(ap) + 1e-20)

    # Planarity
    xi_ss = x_traj[:, :, mask] - x_target_traj.reshape(1, 3, -1)[:, :, mask]
    final_rel = xi_ss[:, :, -1]
    centered = final_rel - np.mean(final_rel, axis=0)
    _, S, _ = np.linalg.svd(centered, full_matrices=False)
    sig = S[2]/S[0] if len(S) >= 3 else 1.0

    return {
        'freq': f,
        'period': 1.0/f if f > 0.001 else None,
        'peak_ratio': pr,
        'is_breathing': pr > 0.05,
        'sigma_ratio': sig,
    }


# ============================================================
# Scan functions
# ============================================================
def scan_k1(alpha_form, N=6, d_col=5.0, mu=9.0, k2=0.5, T_MAX=100.0, DT=0.1):
    v0 = np.array([1.0, 0.0, 0.0])
    xi0 = XI0_BASE.copy()
    results = []
    for k1 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
        r = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_form, T_MAX, DT)
        if r:
            r['k1'] = k1
            r['k2'] = k2
            results.append(r)
            t = 'BR' if r['is_breathing'] else 'RO'
            T_str = f"{r['period']:.3f}" if r['period'] else "—"
            print(f"  k1={k1:.2f}: f={r['freq']:.4f} T={T_str} peak={r['peak_ratio']:.4f} sigma={r['sigma_ratio']:.4f} {t}")
    return results


def scan_k2(alpha_form, N=6, d_col=5.0, mu=9.0, k1=0.5, T_MAX=100.0, DT=0.1):
    v0 = np.array([1.0, 0.0, 0.0])
    xi0 = XI0_BASE.copy()
    results = []
    for k2 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]:
        r = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_form, T_MAX, DT)
        if r:
            r['k1'] = k1
            r['k2'] = k2
            results.append(r)
            t = 'BR' if r['is_breathing'] else 'RO'
            T_str = f"{r['period']:.3f}" if r['period'] else "—"
            T_rot = 2*np.pi/np.sqrt(k2) if k2 > 0 else float('inf')
            print(f"  k2={k2:.2f}: f={r['freq']:.4f} T={T_str} T_rot={T_rot:.3f} ratio={r['period']/T_rot:.3f} peak={r['peak_ratio']:.4f} sigma={r['sigma_ratio']:.4f} {t}")
    return results


def scan_d(alpha_form, N=6, k1=0.5, k2=0.5, mu=9.0, T_MAX=100.0, DT=0.1):
    v0 = np.array([1.0, 0.0, 0.0])
    xi0 = XI0_BASE.copy()
    results = []
    for d_col in [4.0, 5.0, 6.0, 7.0]:
        r = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_form, T_MAX, DT)
        if r:
            r['k1'] = k1; r['k2'] = k2
            r['d_col'] = d_col; r['mu'] = mu
            results.append(r)
            t = 'BR' if r['is_breathing'] else 'RO'
            T_str = f"{r['period']:.3f}" if r['period'] else "—"
            print(f"  d={d_col:.0f}: f={r['freq']:.4f} T={T_str} peak={r['peak_ratio']:.4f} sigma={r['sigma_ratio']:.4f} {t}")
    return results


def scan_mu(alpha_form, N=6, k1=0.5, k2=0.5, d_col=5.0, T_MAX=100.0, DT=0.1):
    v0 = np.array([1.0, 0.0, 0.0])
    xi0 = XI0_BASE.copy()
    results = []
    for mu in [7.0, 9.0, 11.0, 13.0]:
        r = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_form, T_MAX, DT)
        if r:
            r['k1'] = k1; r['k2'] = k2
            r['d_col'] = d_col; r['mu'] = mu
            results.append(r)
            t = 'BR' if r['is_breathing'] else 'RO'
            T_str = f"{r['period']:.3f}" if r['period'] else "—"
            print(f"  mu={mu:.0f}: f={r['freq']:.4f} T={T_str} peak={r['peak_ratio']:.4f} sigma={r['sigma_ratio']:.4f} {t}")
    return results


def scan_N(alpha_form, k1=0.5, k2=0.5, d_col=5.0, mu=9.0, T_MAX=100.0, DT=0.1):
    v0 = np.array([1.0, 0.0, 0.0])
    results = []
    for N in [4, 5, 6, 7, 8]:
        xi0 = generate_scaled_init(N, N_base=6, scale_factor=1.0, seed=0)
        r = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_form, T_MAX, DT)
        if r:
            r['k1'] = k1; r['k2'] = k2
            r['d_col'] = d_col; r['mu'] = mu
            r['N'] = N
            results.append(r)
            t = 'BR' if r['is_breathing'] else 'RO'
            T_str = f"{r['period']:.3f}" if r['period'] else "—"
            print(f"  N={N}: f={r['freq']:.4f} T={T_str} peak={r['peak_ratio']:.4f} sigma={r['sigma_ratio']:.4f} {t}")
    return results


def scan_k1k2_grid(alpha_form, N=6, d_col=5.0, mu=9.0, T_MAX=100.0, DT=0.1):
    v0 = np.array([1.0, 0.0, 0.0])
    xi0 = XI0_BASE.copy()
    results = []
    k1_vals = [0.2, 0.4, 0.6, 0.8]
    k2_vals = [0.2, 0.4, 0.6, 0.8]
    for k1 in k1_vals:
        for k2 in k2_vals:
            r = run_and_measure(N, d_col, mu, k1, k2, v0, xi0, alpha_form, T_MAX, DT)
            if r:
                r['k1'] = k1; r['k2'] = k2
                results.append(r)
    return results


# ============================================================
# Save results
# ============================================================
def save_results(all_results, form_name):
    """Save scan results to CSV."""
    # Flatten all results
    flat = []
    for scan_name, results in all_results.items():
        for r in results:
            row = {'form': form_name, 'scan': scan_name}
            row.update(r)
            flat.append(row)

    # Save CSV
    csv_path = OUT_DIR / f"results_{form_name}.csv"
    if flat:
        keys = flat[0].keys()
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(flat)
    print(f"  Saved {len(flat)} results to {csv_path}")

    # Save JSON
    json_path = OUT_DIR / f"results_{form_name}.json"
    with open(json_path, 'w') as f:
        json.dump(all_results, f, indent=2, default=str)
    print(f"  Saved JSON to {json_path}")


# ============================================================
# Main
# ============================================================
FORMS = ["standard", "power", "rational", "log", "stiff", "exponential"]

if __name__ == '__main__':
    print("=" * 70)
    print("ALL ALPHA FORMS — 3D BREATHING PARAMETER SCAN")
    print("=" * 70)

    for form in FORMS:
        print(f"\n{'='*70}")
        print(f"FORM: {form}")
        print(f"{'='*70}")

        all_results = {}

        # k1 scan
        print("\n--- k1 scan (k2=0.5) ---")
        all_results['k1'] = scan_k1(form)

        # k2 scan
        print("\n--- k2 scan (k1=0.5) ---")
        all_results['k2'] = scan_k2(form)

        # d_col scan
        print("\n--- d_col scan ---")
        all_results['d'] = scan_d(form)

        # mu scan
        print("\n--- mu scan ---")
        all_results['mu'] = scan_mu(form)

        # N scan
        print("\n--- N scan ---")
        all_results['N'] = scan_N(form)

        # (k1,k2) grid
        print("\n--- (k1,k2) grid ---")
        all_results['grid'] = scan_k1k2_grid(form)

        # Save
        save_results(all_results, form)

    print("\n" + "=" * 70)
    print("ALL FORMS COMPLETE")
    print("=" * 70)