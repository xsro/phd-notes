"""
Analyze search results: investigate Q (quasi-periodic) candidates found in the
parameter scan, and run targeted longer simulations for verification.

The search found several interesting cases at k₁=0.30 with random_4 initial
condition, labeled 'Q' (quasi-periodic). This script:

1. Loads partial search results
2. Re-runs key cases with longer integration (tf=5000) and more frames
3. Produces diagnostic plots (distance time series, FFT, phase portrait)
4. Classifies definitively

Run from the repository root:
    ../.venv/bin/python simulation/analyze_q_candidates.py
"""

import os
import sys
import numpy as np
from scipy.signal import find_peaks
from scipy.fft import rfft, rfftfreq

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fencing_ode import integrate, evaluate
from plot_base import TH, AGENT_COLORS, style_axes, save_figure

# ── Fixed parameters ─────────────────────────────────────────────────────────
d, mu = 5.0, 9.0
N = 5
TF_LONG = 5000.0   # longer integration for definitive classification
N_FRAMES = 900     # more frames for better FFT resolution
T_SKIP = 0.5


# ── The random_4 initial condition (seed=4) ──────────────────────────────────
def get_random_4_ic():
    """Recreate the random_4 initial condition from seed=4."""
    rng = np.random.default_rng(4)
    xi0 = rng.uniform(-8.0, 8.0, (N, 2))
    # Ensure no collisions
    for _ in range(100):
        ok = True
        for i in range(N):
            for j in range(i + 1, N):
                if np.linalg.norm(xi0[i] - xi0[j]) < d + 0.5:
                    ok = False
                    xi0[j] = rng.uniform(-8.0, 8.0, 2)
                    break
            if not ok:
                break
        if ok:
            break
    vt0 = rng.uniform(-2.0, 2.0, (N, 2))
    return xi0, vt0


# ── FFT-based frequency analysis ─────────────────────────────────────────────
def analyze_spectrum(signal, fs):
    """Compute the power spectrum and find dominant frequencies."""
    n = len(signal)
    window = np.hanning(n)
    sig_win = (signal - np.mean(signal)) * window

    freqs = rfftfreq(n, d=1.0/fs)
    spectrum = np.abs(rfft(sig_win))

    # Find peaks
    peak_idx, props = find_peaks(spectrum, height=0.03 * spectrum.max(),
                                  distance=5)
    peak_freqs = freqs[peak_idx]
    peak_heights = spectrum[peak_idx]

    # Sort by height descending
    order = np.argsort(-peak_heights)
    peak_freqs = peak_freqs[order]
    peak_heights = peak_heights[order]

    return freqs, spectrum, peak_freqs, peak_heights


def frequency_ratio_classification(freqs, heights, tol=0.02):
    """Classify as periodic, quasi-periodic, or unknown based on frequency ratio."""
    if len(freqs) < 2:
        return 'C' if len(freqs) == 1 else '?', {}

    f0, f1 = freqs[0], freqs[1]
    if f0 <= 0 or f1 <= 0:
        return '?', {}

    ratio = f1 / f0
    # Check for rational relationship
    best_err = 1.0
    best_rat = None
    for num in range(1, 15):
        for den in range(1, 15):
            rat = num / den
            err = abs(ratio - rat)
            if err < best_err:
                best_err = err
                best_rat = (num, den)

    diag = {
        'freqs': freqs.tolist()[:5],
        'heights': heights.tolist()[:5],
        'ratio': float(ratio),
        'best_rational': f"{best_rat[0]}:{best_rat[1]}",
        'rational_err': float(best_err),
    }

    if best_err < tol:
        return 'C', diag  # frequency-locked → periodic
    else:
        return 'Q', diag  # incommensurate → quasi-periodic


# ── Diagnostic plots ─────────────────────────────────────────────────────────
def plot_diagnostics(k1, k2, label, t_eval, pos_all, vel_all, fig_dir):
    """Generate a comprehensive diagnostic figure for a case."""
    pairs = [(i, j) for i in range(N) for j in range(i + 1, N)]
    n_pairs = len(pairs)

    # Compute pairwise distances
    dists = np.zeros((len(t_eval), n_pairs))
    for b, (i, j) in enumerate(pairs):
        dists[:, b] = np.linalg.norm(pos_all[:, i] - pos_all[:, j], axis=1)

    # Angular velocities
    omega_sqrt = np.sqrt(k2)
    omega = np.zeros((len(t_eval), N))
    for i in range(N):
        xi = pos_all[:, i]
        dxi = vel_all[:, i]
        cross = xi[:, 0] * dxi[:, 1] - xi[:, 1] * dxi[:, 0]
        norm2 = np.sum(xi ** 2, axis=1)
        omega[:, i] = np.where(norm2 > 1e-12, cross / norm2, 0.0)

    # Mean radius
    r_mean = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)

    # FFT on the first pair distance (late window)
    late = len(t_eval) // 3
    signal_late = dists[late:, 0] - np.mean(dists[late:, 0])
    fs = 1.0 / (t_eval[1] - t_eval[0])
    freqs, spectrum, peak_freqs, peak_heights = analyze_spectrum(signal_late, fs)
    cls, diag = frequency_ratio_classification(peak_freqs, peak_heights)

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.patch.set_facecolor(TH["surface"])
    for ax in axes.flat:
        style_axes(ax)

    # 1. Trajectory
    ax = axes[0, 0]
    theta = np.linspace(0, 2 * np.pi, 200)
    ax.plot(d * np.cos(theta), d * np.sin(theta),
            color=TH["muted"], lw=0.8, ls="--", alpha=0.4)
    for i in range(N):
        ax.plot(pos_all[:, i, 0], pos_all[:, i, 1],
                color=AGENT_COLORS[i], lw=0.6, alpha=0.5)
        ax.scatter(pos_all[-1, i, 0], pos_all[-1, i, 1],
                   s=30, color=AGENT_COLORS[i], zorder=5)
    ax.set_aspect("equal")
    ax.set_title(f"Trajectories  (k₁={k1}, k₂={k2})", fontsize=9,
                 color=TH["primary"], fontweight="bold", loc="left", pad=6)
    ax.set_xlabel("x", color=TH["secondary"], fontsize=8)
    ax.set_ylabel("y", color=TH["secondary"], fontsize=8)

    # 2. Distances
    ax = axes[0, 1]
    colors = plt.cm.viridis(np.linspace(0.2, 0.9, n_pairs))
    for b in range(n_pairs):
        ax.plot(t_eval, dists[:, b], color=colors[b], lw=0.7, alpha=0.6)
    ax.axhline(d, color=TH["muted"], lw=0.8, ls="--", alpha=0.5)
    ax.set_title(f"Pairwise distances  [{label}]", fontsize=9,
                 color=TH["primary"], fontweight="bold", loc="left", pad=6)
    ax.set_xlabel("t", color=TH["secondary"], fontsize=8)
    ax.set_ylabel("distance", color=TH["secondary"], fontsize=8)

    # 3. Angular velocities
    ax = axes[0, 2]
    for i in range(N):
        ax.plot(t_eval, omega[:, i], color=AGENT_COLORS[i], lw=0.7, alpha=0.6)
    ax.axhline(omega_sqrt, color=TH["muted"], lw=0.8, ls="--", alpha=0.5)
    ax.set_title("Angular velocities", fontsize=9,
                 color=TH["primary"], fontweight="bold", loc="left", pad=6)
    ax.set_xlabel("t", color=TH["secondary"], fontsize=8)
    ax.set_ylabel("ω", color=TH["secondary"], fontsize=8)

    # 4. Mean radius
    ax = axes[1, 0]
    ax.plot(t_eval, r_mean, color=TH["primary"], lw=0.8)
    ax.axhline(d, color=TH["muted"], lw=0.8, ls="--", alpha=0.5)
    ax.set_title("Mean radius", fontsize=9,
                 color=TH["primary"], fontweight="bold", loc="left", pad=6)
    ax.set_xlabel("t", color=TH["secondary"], fontsize=8)
    ax.set_ylabel("⟨r⟩", color=TH["secondary"], fontsize=8)

    # 5. FFT spectrum (late window)
    ax = axes[1, 1]
    ax.plot(freqs, spectrum, color=TH["primary"], lw=0.8)
    for pf, ph in zip(peak_freqs[:5], peak_heights[:5]):
        ax.axvline(pf, color=AGENT_COLORS[1], lw=0.5, ls=":", alpha=0.5)
        ax.text(pf, ph * 1.1, f"{pf:.4f}", fontsize=6,
                ha="center", color=TH["secondary"])
    ax.set_xlim(0, min(freqs[-1], 0.5))
    ax.set_title(f"FFT spectrum  [{cls}]  ratio={diag.get('ratio','?'):.3f}",
                 fontsize=9, color=TH["primary"], fontweight="bold",
                 loc="left", pad=6)
    ax.set_xlabel("freq (Hz)", color=TH["secondary"], fontsize=8)
    ax.set_ylabel("|FFT|", color=TH["secondary"], fontsize=8)

    # 6. Phase portrait (agent 0)
    ax = axes[1, 2]
    xi0 = pos_all[-500:, 0] if len(pos_all) > 500 else pos_all[:, 0]
    dxi0 = vel_all[-500:, 0] if len(vel_all) > 500 else vel_all[:, 0]
    r0 = np.linalg.norm(xi0, axis=1)
    r_dot0 = np.sum(xi0 * dxi0, axis=1) / np.maximum(r0, 1e-12)
    ax.plot(r0, r_dot0, color=AGENT_COLORS[0], lw=0.6, alpha=0.7)
    ax.set_title("Phase portrait (agent 0)", fontsize=9,
                 color=TH["primary"], fontweight="bold", loc="left", pad=6)
    ax.set_xlabel("r₀", color=TH["secondary"], fontsize=8)
    ax.set_ylabel("ṙ₀", color=TH["secondary"], fontsize=8)

    fig.tight_layout()
    out_path = os.path.join(fig_dir, f"q_diagnostic_k1_{k1}_k2_{k2}.pdf")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"  → {out_path}")
    plt.close(fig)

    return cls, diag


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(script_dir)
    fig_dir = os.path.join(repo_root, "figures")
    data_dir = os.path.join(repo_root, "data")
    os.makedirs(fig_dir, exist_ok=True)
    os.makedirs(data_dir, exist_ok=True)

    # The Q candidates from the search (k₁, k₂) with random_4
    # and the known C case for comparison
    candidates = [
        (0.30, 0.30, "random_4"),
        (0.30, 0.80, "random_4"),
        (0.40, 0.30, "polygon"),
        (0.40, 0.80, "polygon"),
        (0.50, 0.30, "random_4"),
        (0.50, 0.50, "random_4"),   # known case C
    ]

    # Add the known case C initial condition
    xi0_case_c = np.array([[-10.584563, -12.532987],
                           [-5.382648,   5.904905],
                           [-0.177347,   9.17592 ],
                           [-7.352506,  -4.805249],
                           [-6.288338,  12.43583 ]])
    vt0_case_c = np.array([[ 3.528048, -1.274511],
                           [-0.511988, -1.485443],
                           [ 1.972071, -3.679895],
                           [-3.460489, -0.767699],
                           [-2.039248,  2.7618  ]])

    results_summary = []

    for k1, k2, init_name in candidates:
        print(f"\n{'='*60}")
        print(f"  k₁={k1:.2f}  k₂={k2:.2f}  init={init_name}  tf={TF_LONG}")
        print(f"{'='*60}")

        # Get initial condition
        if init_name == "random_4":
            xi0, vt0 = get_random_4_ic()
        elif init_name == "polygon":
            angles = np.linspace(0, 2 * np.pi, N, endpoint=False)
            xi0 = 6.0 * np.column_stack([np.cos(angles), np.sin(angles)])
            vt0 = np.sqrt(k2) * np.column_stack([-xi0[:, 1], xi0[:, 0]])
        else:
            xi0, vt0 = xi0_case_c, vt0_case_c

        print(f"  Integrating to t={TF_LONG} ...", flush=True)

        try:
            rhs, sol = integrate(xi0, vt0, d, mu, k1, k2, TF_LONG)
            t_eval = np.linspace(T_SKIP, TF_LONG, N_FRAMES)
            pos_all, vel_all = evaluate(sol, N, t_eval)

            cls, diag = plot_diagnostics(k1, k2, init_name,
                                          t_eval, pos_all, vel_all, fig_dir)

            print(f"  Classification: {cls}  "
                  f"freqs={[f'{f:.4f}' for f in diag.get('freqs', [])[:3]]}  "
                  f"ratio={diag.get('ratio', '?'):.4f}  "
                  f"rational_err={diag.get('rational_err', '?'):.4f}")

            results_summary.append({
                'k1': k1, 'k2': k2, 'init': init_name,
                'class': cls, 'diag': diag,
            })

        except Exception as e:
            print(f"  ERROR: {e}")
            results_summary.append({
                'k1': k1, 'k2': k2, 'init': init_name,
                'class': 'E', 'diag': {'error': str(e)},
            })

    # ── Summary ──
    print(f"\n{'='*60}")
    print("  SUMMARY")
    print(f"{'='*60}")
    for r in results_summary:
        cls = r['class']
        marker = ' ★ COUNTEREXAMPLE!' if cls == 'Q' else ''
        print(f"  k₁={r['k1']:.2f}  k₂={r['k2']:.2f}  {r['init']:10s}  → {cls}{marker}")

    # Save results
    out_path = os.path.join(data_dir, "q_analysis.npz")
    np.savez(out_path, results=results_summary)
    print(f"\nResults saved → {out_path}")


if __name__ == "__main__":
    main()