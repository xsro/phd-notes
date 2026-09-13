"""
Quick targeted analysis of the most interesting cases from the search.

Runs fast simulations (tf=2000, n_frames=600) for the Q candidates
and produces diagnostic plots.

Run:  ../.venv/bin/python simulation/quick_analyze.py
"""

import os, sys
import numpy as np
from scipy.signal import find_peaks
from scipy.fft import rfft, rfftfreq

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fencing_ode import integrate, evaluate
from plot_base import TH, AGENT_COLORS, style_axes

d, mu = 5.0, 9.0
N = 5

def get_init(name, k2=0.5):
    if name == "random_4":
        rng = np.random.default_rng(4)
        xi0 = rng.uniform(-8.0, 8.0, (N, 2))
        for _ in range(100):
            ok = True
            for i in range(N):
                for j in range(i+1, N):
                    if np.linalg.norm(xi0[i]-xi0[j]) < d+0.5:
                        ok = False
                        xi0[j] = rng.uniform(-8.0, 8.0, 2)
                        break
                    if not ok: break
        vt0 = rng.uniform(-2.0, 2.0, (N, 2))
        return xi0, vt0
    elif name == "polygon":
        ang = np.linspace(0, 2*np.pi, N, endpoint=False)
        xi0 = 6.0 * np.column_stack([np.cos(ang), np.sin(ang)])
        vt0 = np.sqrt(k2) * np.column_stack([-xi0[:,1], xi0[:,0]])
        return xi0, vt0
    elif name == "case_c":
        xi0 = np.array([[-10.584563, -12.532987],
                        [-5.382648,   5.904905],
                        [-0.177347,   9.17592 ],
                        [-7.352506,  -4.805249],
                        [-6.288338,  12.43583 ]])
        vt0 = np.array([[ 3.528048, -1.274511],
                        [-0.511988, -1.485443],
                        [ 1.972071, -3.679895],
                        [-3.460489, -0.767699],
                        [-2.039248,  2.7618  ]])
        return xi0, vt0
    else:
        raise ValueError(f"Unknown init: {name}")

# Cases to analyze: (k1, k2, init, label)
cases = [
    (0.30, 0.30, "random_4", "Q candidate"),
    (0.30, 0.80, "random_4", "Q candidate"),
    (0.40, 0.30, "polygon",  "C (limit cycle)"),
    (0.40, 0.80, "polygon",  "C (limit cycle)"),
    (0.50, 0.50, "random_4", "C (limit cycle)"),
    (0.50, 0.50, "case_c",   "C (known)"),
]

tf = 2000.0
n_frames = 600
t_skip = 0.5

script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
fig_dir = os.path.join(repo_root, "figures")
os.makedirs(fig_dir, exist_ok=True)

for k1, k2, init_name, label in cases:
    print(f"\nk₁={k1:.2f}  k₂={k2:.2f}  {init_name:10s}  [{label}]", flush=True)

    xi0, vt0 = get_init(init_name, k2)
    rhs, sol = integrate(xi0, vt0, d, mu, k1, k2, tf)
    t_eval = np.linspace(t_skip, tf, n_frames)
    pos_all, vel_all = evaluate(sol, N, t_eval)

    # ── Compute diagnostics ──
    # Pairwise distances
    pairs = [(i,j) for i in range(N) for j in range(i+1, N)]
    dists = np.zeros((n_frames, len(pairs)))
    for b, (i,j) in enumerate(pairs):
        dists[:,b] = np.linalg.norm(pos_all[:,i] - pos_all[:,j], axis=1)

    # Angular velocities
    omega = np.zeros((n_frames, N))
    for i in range(N):
        xi = pos_all[:,i]; dxi = vel_all[:,i]
        cross = xi[:,0]*dxi[:,1] - xi[:,1]*dxi[:,0]
        norm2 = np.sum(xi**2, axis=1)
        omega[:,i] = np.where(norm2 > 1e-12, cross/norm2, 0.0)

    # Mean radius
    r_mean = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)

    # FFT on late window (last half)
    late = n_frames // 2
    signal = dists[late:, 0] - np.mean(dists[late:, 0])
    fs = 1.0/(t_eval[1]-t_eval[0])
    n_fft = len(signal)
    window = np.hanning(n_fft)
    sig_win = signal * window
    freqs = rfftfreq(n_fft, d=1.0/fs)
    spectrum = np.abs(rfft(sig_win))
    peak_idx, _ = find_peaks(spectrum, height=0.03*spectrum.max(), distance=5)
    peak_freqs = freqs[peak_idx]
    peak_heights = spectrum[peak_idx]
    order = np.argsort(-peak_heights)
    peak_freqs = peak_freqs[order]
    peak_heights = peak_heights[order]

    # Frequency ratio check
    n_peaks = len(peak_freqs)
    if n_peaks >= 2:
        ratio = peak_freqs[1] / peak_freqs[0] if peak_freqs[0] > 0 else 0
        # nearest rational
        best_err = 1.0
        for num in range(1, 15):
            for den in range(1, 15):
                err = abs(ratio - num/den)
                if err < best_err: best_err = err
        is_q = best_err >= 0.02
    else:
        ratio = 0
        best_err = 1.0
        is_q = False

    cls = 'Q' if is_q else ('C' if n_peaks >= 1 else 'A')
    print(f"  → {cls}  peaks={n_peaks}  f0={peak_freqs[0] if n_peaks>=1 else 0:.4f}  "
          f"ratio={ratio:.4f}  rat_err={best_err:.4f}  "
          + ("★ QUASI-PERIODIC?" if is_q else ""))

    # ── Figure ──
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.patch.set_facecolor(TH["surface"])
    for ax in axes.flat: style_axes(ax)

    # 1. Trajectory
    ax = axes[0,0]
    theta = np.linspace(0, 2*np.pi, 200)
    ax.plot(d*np.cos(theta), d*np.sin(theta), color=TH["muted"], lw=0.8, ls="--", alpha=0.4)
    skip = n_frames // 100
    for i in range(N):
        ax.plot(pos_all[::skip, i, 0], pos_all[::skip, i, 1],
                color=AGENT_COLORS[i], lw=0.6, alpha=0.5)
        ax.scatter(pos_all[-1, i, 0], pos_all[-1, i, 1], s=30, color=AGENT_COLORS[i], zorder=5)
    ax.set_aspect("equal")
    ax.set_title(f"Trajectories  k₁={k1}, k₂={k2}", fontsize=9, color=TH["primary"], fontweight="bold", loc="left")

    # 2. Distances (late window)
    ax = axes[0,1]
    late_start = max(0, n_frames - 200)
    late_t = t_eval[late_start:]
    colors = plt.cm.viridis(np.linspace(0.2, 0.9, len(pairs)))
    for b in range(len(pairs)):
        ax.plot(late_t, dists[late_start:, b], color=colors[b], lw=0.7, alpha=0.7)
    ax.axhline(d, color=TH["muted"], lw=0.8, ls="--", alpha=0.5)
    ax.set_title(f"Distances (late)  [{cls}]", fontsize=9, color=TH["primary"], fontweight="bold", loc="left")

    # 3. Angular velocities (late)
    ax = axes[0,2]
    for i in range(N):
        ax.plot(late_t, omega[late_start:, i], color=AGENT_COLORS[i], lw=0.7, alpha=0.7)
    ax.axhline(np.sqrt(k2), color=TH["muted"], lw=0.8, ls="--", alpha=0.5)
    ax.set_title("Angular velocities (late)", fontsize=9, color=TH["primary"], fontweight="bold", loc="left")

    # 4. Mean radius
    ax = axes[1,0]
    ax.plot(t_eval, r_mean, color=TH["primary"], lw=0.8)
    ax.axhline(d, color=TH["muted"], lw=0.8, ls="--", alpha=0.5)
    ax.set_title("Mean radius", fontsize=9, color=TH["primary"], fontweight="bold", loc="left")

    # 5. FFT
    ax = axes[1,1]
    ax.plot(freqs, spectrum, color=TH["primary"], lw=0.8)
    for pf, ph in zip(peak_freqs[:5], peak_heights[:5]):
        ax.axvline(pf, color=AGENT_COLORS[3], lw=0.5, ls=":", alpha=0.5)
        ax.text(pf, ph*1.1, f"{pf:.4f}", fontsize=6, ha="center", color=TH["secondary"])
    ax.set_xlim(0, min(freqs[-1], 0.3))
    ax.set_title(f"FFT  peaks={n_peaks}  ratio={ratio:.4f}", fontsize=9, color=TH["primary"], fontweight="bold", loc="left")

    # 6. Phase portrait (agent 0)
    ax = axes[1,2]
    xi0_plot = pos_all[late_start:, 0]
    dxi0_plot = vel_all[late_start:, 0]
    r0 = np.linalg.norm(xi0_plot, axis=1)
    r_dot0 = np.sum(xi0_plot * dxi0_plot, axis=1) / np.maximum(r0, 1e-12)
    ax.plot(r0, r_dot0, color=AGENT_COLORS[0], lw=0.6, alpha=0.7)
    ax.set_title("Phase portrait (agent 0)", fontsize=9, color=TH["primary"], fontweight="bold", loc="left")

    fig.tight_layout()
    out_path = os.path.join(fig_dir, f"diag_k1_{k1:.2f}_k2_{k2:.2f}_{init_name}.pdf")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"  → {out_path}")
    plt.close(fig)

print("\n✓ Done")