#!/usr/bin/env python3
"""Generate every figure of fencing-rotation.tex.

Usage
-----
    uv run --no-project --with numpy,scipy,matplotlib python make_figures.py all
    uv run --no-project --with numpy,scipy,matplotlib python make_figures.py fig2d-rotation fig-jam

Each figure is written to ../figures/<name>.pdf.  Expensive integrations are
cached in ./cache/*.npz, so figures can be regenerated without re-integrating.
"""

import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from numpy.fft import rfft, rfftfreq
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

from fencing_lib import (distances, integrate_double, integrate_single,
                         repulsion, repulsion_hessian, style, velocity)

FIG = os.path.join(HERE, "..", "figures")
CACHE = os.path.join(HERE, "cache")
DATA = os.path.join(HERE, "..", "..")          # .../fencing
os.makedirs(FIG, exist_ok=True)
os.makedirs(CACHE, exist_ok=True)

# --------------------------------------------------------------------------- #
# baseline parameters (the paper's example)
# --------------------------------------------------------------------------- #
D, MU = 5.0, 9.0
K1, K2 = 0.5, 0.5

HEX_R = 8.0


def hexagon(n=6, r=HEX_R):
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([r * np.cos(th), r * np.sin(th)])

# the "Case C" breathing initial condition (N=5), from the conjecture notes
CASE_C_XI = np.array([[-10.584563, -12.532987],
                      [-5.382648, 5.904905],
                      [-0.177347, 9.175920],
                      [-7.352506, -4.805249],
                      [-6.288338, 12.435830]])
CASE_C_VT = np.array([[3.528048, -1.274511],
                      [-0.511988, -1.485443],
                      [1.972071, -3.679895],
                      [-3.460489, -0.767699],
                      [-2.039248, 2.761800]])

C_J = {"jam": "tab:red", "rot": "tab:blue", "breath": "tab:green"}


# --------------------------------------------------------------------------- #
# cached integration helpers
# --------------------------------------------------------------------------- #
def cached_run(name, build, force=False):
    """build() -> dict of arrays; cached as cache/<name>.npz."""
    path = os.path.join(CACHE, name + ".npz")
    if os.path.exists(path) and not force:
        z = np.load(path, allow_pickle=True)
        return {k: z[k] for k in z.files}
    t0 = time.time()
    out = build()
    np.savez(path, **out)
    print(f"  [integr] {name}: {time.time() - t0:.1f}s")
    return out


def rot_run(k2, force=False, tf=None, n_eval=1600, extra=""):
    if tf is None:
        tf = 400.0 if k2 >= 0.2 else 1200.0
    def build():
        r = integrate_single(hexagon(), np.zeros((6, 2)), D, MU, K1, k2, tf,
                             n_eval, method="LSODA", rtol=1e-8, atol=1e-10)
        return {"t": r["t"], "xi": r["xi"], "vt": r["vt"]}
    return cached_run(f"rot_k2-{k2:g}{extra}", build, force)


def breathing_run(k1=K1, k2=K2, tf=600.0, n_eval=2400, tag="cc", force=False):
    def build():
        r = integrate_single(CASE_C_XI, CASE_C_VT, D, MU, k1, k2, tf,
                             n_eval, method="LSODA", rtol=1e-9, atol=1e-11)
        return {"t": r["t"], "xi": r["xi"], "vt": r["vt"]}
    return cached_run(f"breath_{tag}_k1-{k1:g}_k2-{k2:g}", build, force)


# --------------------------------------------------------------------------- #
# small numerical utilities
# --------------------------------------------------------------------------- #
def omega_trace(xi, vt, k1):
    vx = velocity(xi, vt, k1, D, MU)
    cross = xi[..., 0] * vx[..., 1] - xi[..., 1] * vx[..., 0]
    return cross / np.sum(xi ** 2, axis=-1)


def tail_slice(t, frac=0.25):
    return slice(int((1.0 - frac) * len(t)), len(t))


def dominant_freq(t, sig):
    sig = np.asarray(sig, float) - np.mean(sig)
    dt = t[1] - t[0]
    f = rfftfreq(len(sig), dt)
    P = np.abs(rfft(sig))
    i = np.argmax(P[1:]) + 1
    return float(f[i]), float(P[i] / np.max(P)), f, P


def pairdist_trace(run):
    return np.array([distances(x) for x in run["xi"]])


def circle_root(k1, k2, k3=0.0):
    """Root r of alpha(r) = (k1 - k3/k2) r (regular-hexagon radial balance)."""
    f = lambda r: 1.0 / (r - D) - 1.0 / (MU - D) - (k1 - k3 / k2) * r
    lo, hi = D + 1e-6, MU - 1e-6
    try:
        return brentq(f, lo, hi)
    except ValueError:
        return np.nan


def plane_basis(n, dim=2):
    """Orthonormal basis of the zero-mean subspace of R^{n*dim}."""
    Q = np.kron(np.eye(n) - np.ones((n, n)) / n, np.eye(dim))
    U, s, _ = np.linalg.svd(Q)
    return U[:, : n * dim - dim]


def corotation_spectrum(run, k1, k2, omega):
    """Eigenvalues of the co-rotating linearisation DF (shape subspace)."""
    n = run["xi"].shape[1]
    xi = run["xi"][-1]
    vt = run["vt"][-1]
    th = -omega * run["t"][-1]
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    xs = R @ xi.T                       # co-rotating positions, (2, n)
    # rotate back for the Hessian (P is rotation invariant, so use lab frame)
    H = repulsion_hessian(xi, D, MU) - k1 * np.eye(2 * n)
    Jn = np.kron(np.eye(n), np.array([[0.0, -1.0], [1.0, 0.0]]))
    B = plane_basis(n)
    m = B.shape[1]
    Hr = B.T @ H @ B
    Jr = B.T @ Jn @ B
    DF = np.block([[Hr - omega * Jr, np.eye(m)],
                   [-k2 * np.eye(m), -omega * Jr]])
    ev = np.linalg.eigvals(DF)
    # neutral eigenvector test: tangent to the rotation family,
    #   (delta, epsilon) = (J xi*, -omega xi*) in the zero-mean subspace
    v0 = np.concatenate([B.T @ (Jn @ xi.reshape(-1)),
                         B.T @ (-omega * xi.reshape(-1))])
    w, V = np.linalg.eig(DF)
    idx = np.argmin(np.abs(w))
    align = abs(np.vdot(v0, V[:, idx])) / (np.linalg.norm(v0) * np.linalg.norm(V[:, idx]))
    return ev, float(abs(w[idx])), float(align)


# --------------------------------------------------------------------------- #
# figures
# --------------------------------------------------------------------------- #
def fig1d(force=False):
    """1D benchmark: no observer, static formation, target fenced."""
    def build():
        n, k1, mu, d = 5, 1.0, 2.0, 0.5
        xi = np.array([-0.75, 2.70, 1.39, 0.59, -2.06])
        x = xi.reshape(n, 1)
        tf, ne = 30.0, 600
        z0 = x.ravel()
        from scipy.integrate import solve_ivp

        def rhs(t, z):
            X = z.reshape(n, 1)
            phi = repulsion(X, d, mu)
            return (phi - k1 * X).ravel()
        tt = np.linspace(0, tf, ne)
        sol = solve_ivp(rhs, (0, tf), z0, method="LSODA", t_eval=tt,
                        rtol=1e-9, atol=1e-11)
        return {"t": sol.t, "xi": sol.y.T.reshape(-1, n, 1)}
    r = cached_run("1d", build, force)

    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    tt = r["t"]
    lab = r["xi"][:, :, 0] + 1.0 * tt[:, None]      # v0 = 1
    for i in range(lab.shape[1]):
        ax[0].plot(tt, lab[:, i], lw=1.0, label=f"$i={i + 1}$")
    ax[0].plot(tt, 1.0 * tt, "k--", lw=1.2, label="target")
    ax[0].set_xlabel("$t$")
    ax[0].set_ylabel("$x_i(t)$")
    ax[0].set_title("(a) 1D: positions and target")
    ax[0].legend(ncol=2, fontsize=6)

    # target-frame gaps
    g = np.array([distances(z) for z in r["xi"]])
    ax[1].plot(tt, g.min(axis=1), label="min gap")
    ax[1].plot(tt, g.max(axis=1), label="max gap")
    ax[1].axhline(0.5, color="k", ls=":", lw=0.9)
    ax[1].text(1.0, 0.62, "$d$", fontsize=8)
    ax[1].set_xlabel("$t$")
    ax[1].set_ylabel("gap")
    ax[1].set_title("(b) gaps settle, target stays inside")
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig1d.pdf"))
    plt.close(fig)


def fig2d_rotation(force=False):
    """Rigid rotation: trajectory, constant gaps, average vs individual error."""
    r = rot_run(K2, force, tf=400.0)
    t, xi, vt = r["t"], r["xi"], r["vt"]
    w_trace = omega_trace(xi, vt, K1)

    fig, ax = plt.subplots(1, 3, figsize=(7.2, 2.5))
    sl = (t > t[-1] - 30.0)
    for i in range(xi.shape[1]):
        ax[0].plot(xi[sl, i, 0], xi[sl, i, 1], lw=1.0)
        ax[0].plot(xi[-1, i, 0], xi[-1, i, 1], "o", ms=3,
                   color=f"C{i}")
    ax[0].plot(0, 0, "k*", ms=9)
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("$\\tilde x_1$")
    ax[0].set_ylabel("$\\tilde x_2$")
    ax[0].set_title("(a) target frame, last 30 s")

    g = pairdist_trace(r)
    ax[1].plot(t, g, lw=0.9)
    ax[1].set_xlabel("$t$")
    ax[1].set_ylabel("$\\|x_{ij}\\|$")
    ax[1].set_title("(b) gaps converge")
    late = g[t > t[-1] * 0.1]
    ax[1].set_ylim(D * 0.92, late.max() * 1.15)
    ax[1].set_xlim(0, t[-1])

    ind = np.linalg.norm(vt, axis=2).mean(axis=1)          # mean of norms
    avg = np.linalg.norm(vt.mean(axis=1), axis=1)          # norm of mean
    ax[2].semilogy(t, ind, label="mean $\\|\\tilde v_i\\|$")
    ax[2].semilogy(t, avg, label="$\\|\\overline{\\tilde v}\\|$")
    r_end = np.linalg.norm(xi[-1], axis=1)
    ax[2].axhline(np.sqrt(K2) * np.sqrt(np.mean(r_end ** 2)), ls=":", lw=0.8,
                  color="gray")
    ax[2].set_xlabel("$t$")
    ax[2].set_ylabel("velocity error")
    ax[2].set_title("(c) average decays, individuals do not")
    ax[2].legend(fontsize=6.5, loc="lower left")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig2d-rotation.pdf"))
    plt.close(fig)


K2_SWEEP = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0]


def fig2d_omega_sweep(force=False):
    """|omega| = sqrt(k2): sweep over a factor 40 in k2."""
    res = []
    for k2 in K2_SWEEP:
        r = rot_run(k2, force)
        w = omega_trace(r["xi"], r["vt"], K1)[tail_slice(r["t"], 0.25)]
        res.append((k2, float(np.nanmean(w)), float(np.nanmax(w) - np.nanmin(w))))
    res = np.array(res)
    pred = np.sqrt(res[:, 0])

    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    ax[0].plot(res[:, 0], pred, "k-", lw=1.0, label="$\\sqrt{k_2}$")
    ax[0].plot(res[:, 0], np.abs(res[:, 1]), "o", ms=4, label="measured")
    ax[0].set_xscale("log")
    ax[0].set_yscale("log")
    ax[0].set_xlabel("$k_2$")
    ax[0].set_ylabel("$|\\omega|$")
    ax[0].set_title("(a) angular speed")
    ax[0].legend(fontsize=7)

    rel = np.abs(np.abs(res[:, 1]) - pred) / pred
    ax[1].loglog(res[:, 0], np.maximum(rel, 1e-12), "o-", ms=4)
    ax[1].axhline(1e-3, ls=":", color="gray", lw=0.8)
    ax[1].text(0.06, 1.6e-3, "0.1\%", fontsize=7, color="gray")
    ax[1].set_xlabel("$k_2$")
    ax[1].set_ylabel("relative error")
    ax[1].set_title("(b) relative error")
    ax[1].set_ylim(1e-10, 1e-1)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig2d-omega-sweep.pdf"))
    plt.close(fig)
    np.save(os.path.join(CACHE, "k2_sweep.npy"), res)
    return res


def fig2d_shape_branch(force=False):
    """The rotating shape is an interleaved two-triangle, not a regular hexagon."""
    r = rot_run(K2, force, tf=400.0)
    rr = np.sort(np.linalg.norm(r["xi"][-1], axis=1))
    r_star = circle_root(K1, K2)

    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    ang = np.sort(np.degrees(np.arctan2(*r["xi"][-1].T[::-1])))
    ax[0].plot(range(1, 7), rr, "o", ms=5, label="steady radii")
    ax[0].axhline(r_star, ls="--", color="k", lw=1.0,
                  label=f"hexagon branch $r^*={r_star:.2f}$")
    ax[0].set_xlabel("vehicle (sorted)")
    ax[0].set_ylabel("$\\|\\tilde x_i\\|$")
    ax[0].set_title("(a) radii: two interleaved triangles")
    ax[0].legend(fontsize=7)

    th = np.linspace(0, 2 * np.pi, 400)
    ax[1].plot(r_star * np.cos(th), r_star * np.sin(th), "k--", lw=0.8)
    ax[1].plot(r["xi"][-1, :, 0], r["xi"][-1, :, 1], "o-", ms=4, lw=0.7)
    ax[1].plot(0, 0, "k*", ms=8)
    ax[1].set_aspect("equal")
    ax[1].set_xlabel("$\\tilde x_1$")
    ax[1].set_ylabel("$\\tilde x_2$")
    ax[1].set_title("(b) steady configuration vs $r^*$ circle")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig2d-shape-branch.pdf"))
    plt.close(fig)


def fig2d_degenerate(force=False):
    """Only a genuinely invariant subspace (collinear) stays rotation free."""
    def build(xi0, tf=400.0):
        r = integrate_single(xi0, np.zeros_like(xi0), D, MU, K1, K2, tf, 800,
                             method="BDF", rtol=1e-8, atol=1e-10)
        return {"t": r["t"], "xi": r["xi"], "vt": r["vt"]}

    coll = cached_run("deg_collinear", lambda: build(
        np.column_stack([np.array([-15., -9., -3., 3., 9., 15.]),
                         np.zeros(6)])), force)
    sq = cached_run("deg_square", lambda: build(
        np.array([[4.0, 4.0], [-4.0, 4.0], [-4.0, -4.0], [4.0, -4.0]])), force)
    gen = rot_run(K2, force, tf=400.0)

    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    for run, lab, ls in [(gen, "generic hexagon", "-"),
                         (coll, "collinear (invariant)", "--"),
                         (sq, "square $L(0)=0$ (not invariant)", ":")]:
        xi, vt = run["xi"], run["vt"]
        L = np.sum(xi[..., 0] * vt[..., 1] - xi[..., 1] * vt[..., 0], axis=1)
        sel = run["t"] < 200
        ax[0].plot(run["t"][sel], L[sel], ls, lw=1.1, label=lab)
    ax[0].axhline(0.0, color="k", lw=0.7)
    ax[0].set_xlabel("$t$")
    ax[0].set_ylabel(r"$L(t)=\sum_i \tilde x_i \times \tilde v_i$")
    ax[0].set_title("(a) directed circulation")
    ax[0].legend(fontsize=6.2)

    for run, lab, ls in [(coll, "collinear", "--"), (sq, "square", ":")]:
        g = pairdist_trace(run).min(axis=1)
        ax[1].semilogy(run["t"], np.abs(g - D), ls, lw=1.0, label=lab)
    ax[1].set_xlabel("$t$")
    ax[1].set_ylabel("$\min\|x_{ij}\|-d$")
    ax[1].set_title("(b) jam vs. rigid rotation")
    ax[1].legend(fontsize=6.5)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig2d-degenerate.pdf"))
    plt.close(fig)


def fig2d_spectrum(force=False):
    """Co-rotating linearisation: one neutral phase mode, rest strictly stable."""
    k2s = [0.1, 0.2, 0.5, 1.0]
    data = []
    for k2 in k2s:
        r = rot_run(k2, force)

        w = omega_trace(r["xi"], r["vt"], K1)[tail_slice(r["t"], 0.25)]
        omega = float(np.nanmean(w))
        ev, zero_res, align = corotation_spectrum(r, K1, k2, omega)
        mask = np.abs(ev) > 1e-4
        absc = float(np.max(ev.real[mask]))
        data.append((k2, omega, ev, absc, align))
        print(f"  k2={k2}: zero|.|={zero_res:.2e} alignment={align:.6f} "
              f"abscissa={absc:.4f}")

    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    k2, omega, ev, absc, align = data[2]
    ax[0].plot(ev.real, ev.imag, "o", ms=3.5)
    ax[0].axvline(0, color="k", lw=0.7)
    ax[0].set_xlabel("Re $\\lambda$")
    ax[0].set_ylabel("Im $\\lambda$")
    ax[0].set_title(f"(a) spectrum of $DF$, $k_2={k2}$")
    ax[0].annotate("neutral phase mode", xy=(0, 0), xytext=(-0.55, 0.28),
                   fontsize=7,
                   arrowprops=dict(arrowstyle="->", lw=0.7))

    ax[1].plot([d[0] for d in data], [d[3] for d in data], "o-", ms=4)
    ax[1].set_xscale("log")
    ax[1].set_xlabel("$k_2$")
    ax[1].set_ylabel("spectral abscissa")
    ax[1].set_title("(b) decay rate (zero mode excluded)")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig2d-spectrum.pdf"))
    plt.close(fig)


EPS = [-1e-2, -1e-6, -1e-12, 1e-12, 1e-6, 1e-2]


def fig2d_direction(force=False):
    """Direction = sign of the initial circulation; symmetric case is degenerate."""
    rows = []
    for eps in EPS:
        def build(eps=eps):
            xi0 = hexagon()
            vt0 = eps * np.column_stack([-xi0[:, 1], xi0[:, 0]])   # eps J x
            r = integrate_single(xi0, vt0, D, MU, K1, K2, 400.0, 600,
                                 method="LSODA", rtol=1e-8, atol=1e-10)
            return {"t": r["t"], "xi": r["xi"], "vt": r["vt"]}
        r = cached_run(f"dir_eps-{eps:g}", build, force)
        sl = tail_slice(r["t"], 0.25)
        w = float(np.nanmean(omega_trace(r["xi"][sl], r["vt"][sl], K1)))
        # lock time: first t after which |omega - mean| < 1e-3
        trace = omega_trace(r["xi"], r["vt"], K1).mean(axis=1)
        locked = np.where(np.abs(trace - w) > 1e-3)[0]
        t_lock = r["t"][locked[-1] + 1] if len(locked) and locked[-1] + 1 < len(trace) else 0.0
        rows.append((eps, w, t_lock))

    # exactly symmetric initial condition
    def build_sym():
        r = integrate_single(hexagon(), np.zeros((6, 2)), D, MU, K1, K2,
                             1500.0, 3000, method="LSODA", rtol=1e-10,
                             atol=1e-12)
        return {"t": r["t"], "xi": r["xi"], "vt": r["vt"]}
    sym = cached_run("dir_symmetric", build_sym, force)

    fig, ax = plt.subplots(1, 2, figsize=(6.8, 2.5))
    eps = np.array([r[0] for r in rows])
    w = np.array([r[1] for r in rows])
    ax[0].plot(eps, w, "o", ms=5)
    ax[0].axhline(np.sqrt(K2), ls=":", lw=0.8, color="gray")
    ax[0].axhline(-np.sqrt(K2), ls=":", lw=0.8, color="gray")
    ax[0].axvline(0, color="k", lw=0.7)
    ax[0].set_xscale("symlog", linthresh=1e-12)
    ticks = [-1e-2, -1e-6, -1e-12, 0.0, 1e-12, 1e-6, 1e-2]
    ax[0].set_xticks(ticks)
    ax[0].set_xticklabels(["", "$-10^{-6}$", "$-10^{-12}$", "$0$",
                           "$10^{-12}$", "$10^{-6}$", ""], fontsize=6.5)
    ax[0].set_xlabel("$\\varepsilon$")
    ax[0].set_ylabel("steady $\\omega$")
    ax[0].set_title("(a) direction follows $\\mathrm{sgn}\\,\\varepsilon$")
    ax[0].set_ylim(-1.0, 1.0)

    # panel (b): exponential growth of the directed circulation |L(t)|
    ax[1].set_yscale("log")
    for eps, c, ls in [(1e-12, "tab:blue", "-"), (-1e-12, "tab:red", "-")]:
        r = cached_run(f"dir_eps-{eps:g}", lambda: None, False)
        xi, vt, t = r["xi"], r["vt"], r["t"]
        L = np.abs(np.sum(xi[..., 0] * vt[..., 1] - xi[..., 1] * vt[..., 0], axis=1))
        ax[1].plot(t, np.maximum(L, 1e-17), ls, color=c, lw=1.0,
                   label=f"$\\varepsilon={eps:g}$")
    xi, vt, t = sym["xi"], sym["vt"], sym["t"]
    L = np.abs(np.sum(xi[..., 0] * vt[..., 1] - xi[..., 1] * vt[..., 0], axis=1))
    ax[1].plot(t, np.maximum(L, 1e-17), "k--", lw=1.0,
               label="symmetric ($L(0)$ from round-off)")
    L_inf = np.sqrt(K2) * np.sum(xi[-1] ** 2)
    ax[1].axhline(L_inf, ls=":", lw=0.9, color="gray")
    ax[1].text(60, L_inf * 1.4, "$\\sqrt{k_2}\\sum_i\\|\\tilde x_i\\|^2$", fontsize=7)
    ax[1].set_xlim(0, 300)
    ax[1].set_xlabel("$t$")
    ax[1].set_ylabel("$|L(t)|$")
    ax[1].set_title("(b) exponential symmetry breaking")
    ax[1].legend(fontsize=6.4, loc="lower right")

    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig2d-direction.pdf"))
    plt.close(fig)


def fig2d_breathing(force=False):
    """Case-C breathing limit cycle: periodic gaps, no fixed shape."""
    r = breathing_run(force=force)
    t, xi = r["t"], r["xi"]
    g = pairdist_trace(r)

    fig = plt.figure(figsize=(7.2, 2.5))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.15, 1, 1])
    ax0 = fig.add_subplot(gs[0])
    sl = t > t[-1] - 20.0
    for k in range(g.shape[1]):
        ax0.plot(t[sl], g[sl, k], lw=0.8)
    f, rel, ff, P = dominant_freq(t[tail_slice(t, 0.5)], g[tail_slice(t, 0.5), 0])
    ax0.set_xlabel("$t$ (last 20 s)")
    ax0.set_ylabel("$\\|x_{ij}\\|$")
    ax0.set_title(f"(a) gaps oscillate, $T\\approx{1 / f:.2f}$ s")
    ax1 = fig.add_subplot(gs[1])
    ax1.semilogy(ff[1:], P[1:] / P.max(), lw=0.8)
    ax1.axvline(f, ls=":", color="k", lw=0.8)
    ax1.set_xlim(0, 1.2)
    ax1.set_xlabel("frequency (Hz)")
    ax1.set_title(f"(b) dominant $f={f:.3f}$ Hz")
    ax1.set_ylabel("$|\\hat g|$")

    ax2 = fig.add_subplot(gs[2])
    s2 = t > t[-1] - 40.0
    for i in range(xi.shape[1]):
        ax2.plot(xi[s2, i, 0], xi[s2, i, 1], lw=0.8)
    ax2.plot(0, 0, "k*", ms=8)
    ax2.set_aspect("equal")
    ax2.set_xlabel("$\\tilde x_1$")
    ax2.set_ylabel("$\\tilde x_2$")
    ax2.set_title("(c) trajectory (last 40 s)")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig2d-breathing.pdf"))
    plt.close(fig)

    w = omega_trace(xi[tail_slice(t, 0.5)], r["vt"][tail_slice(t, 0.5)], K1)
    print(f"  breathing: f={f:.4f} Hz, T={1/f:.3f}s, "
          f"omega_mean={np.nanmean(w):.3f}, spread={np.nanmax(w) - np.nanmin(w):.3f}")


def fig2d_bistability(force=False):
    """Same gains, two initial conditions, two attractors; k1 removes the cycle."""
    rot = rot_run(K2, force, tf=400.0)
    br = breathing_run(force=force)
    br_hi = breathing_run(k1=0.7, tf=900.0, n_eval=3600, tag="cc_hi", force=force)
    br_lo = breathing_run(k1=0.3, tf=500.0, n_eval=2000, tag="cc_lo", force=force)

    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    for run, lab, c in [(rot, "regular hexagon $\\to$ rotation", "tab:blue"),
                        (br, "Case C $\\to$ breathing", "tab:green")]:
        g = pairdist_trace(run)
        sl = run["t"] > run["t"][-1] - 40.0
        ax[0].plot(run["t"][sl] - run["t"][-1], g[sl, 0], color=c, lw=1.0,
                   label=lab)
    ax[0].axhline(D, ls=":", color="k", lw=0.8)
    ax[0].set_xlabel("$t-t_{\\rm end}$ (s)")
    ax[0].set_ylabel("one gap")
    ax[0].set_title("(a) bistability at $k_1=k_2=0.5$")
    ax[0].legend(fontsize=6.5)

    for run, lab, c in [(br_lo, "$k_1=0.3$", "tab:green"),
                        (br, "$k_1=0.5$", "tab:olive"),
                        (br_hi, "$k_1=0.7$", "tab:blue")]:
        g = pairdist_trace(run)
        sl = run["t"] > 0.5 * run["t"][-1]
        ax[1].plot(run["t"][sl], g[sl, 0], color=c, lw=1.0, label=lab)
    ax[1].set_xlabel("$t$ (second half)")
    ax[1].set_ylabel("smallest gap")
    ax[1].set_title("(b) Case C: cycle dies as $k_1$ grows")
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig2d-bistability.pdf"))
    plt.close(fig)


def fig3d_breathing(force=False):
    """3D: random ICs breathe, planar ICs rotate rigidly."""
    def rand3d(seed=0, n=6):
        rng = np.random.default_rng(seed)
        pts = []
        while len(pts) < n:
            p = rng.uniform(-15, 18, 3)
            if all(np.linalg.norm(p - q) > 8.0 for q in pts):
                pts.append(p)
        return np.array(pts)

    def build():
        r = integrate_single(rand3d(), np.zeros((6, 3)), D, MU, K1, K2, 400.0,
                             800, method="LSODA", rtol=1e-8, atol=1e-10)
        return {"t": r["t"], "xi": r["xi"], "vt": r["vt"]}
    rnd = cached_run("3d_random", build, force)

    def build_planar():
        x0 = np.column_stack([hexagon(), np.zeros(6)])
        r = integrate_single(x0, np.zeros((6, 3)), D, MU, K1, K2, 300.0, 600,
                             method="LSODA", rtol=1e-8, atol=1e-10)
        return {"t": r["t"], "xi": r["xi"], "vt": r["vt"]}
    pl = cached_run("3d_planar", build_planar, force)

    fig = plt.figure(figsize=(7.2, 2.5))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.25, 1, 1])
    ax0 = fig.add_subplot(gs[0], projection="3d")
    s = rnd["t"] > rnd["t"][-1] - 25.0
    for i in range(6):
        ax0.plot(rnd["xi"][s, i, 0], rnd["xi"][s, i, 1], rnd["xi"][s, i, 2],
                 lw=0.8)
    ax0.plot([0], [0], [0], "k*", ms=6)
    ax0.set_xlabel("")
    ax0.set_ylabel("")
    ax0.set_zlabel("")
    ax0.tick_params(labelsize=5.5, pad=-2)
    ax0.set_xticks([-4, 0, 4])
    ax0.set_yticks([-4, 0, 4])
    ax0.set_zticks([-4, 0, 4])
    ax0.set_title("(a) 3D breathing", fontsize=9)

    ax1 = fig.add_subplot(gs[1])
    g = pairdist_trace(rnd)
    sl = rnd["t"] > rnd["t"][-1] - 20.0
    ax1.plot(rnd["t"][sl], g[sl, 0], lw=0.9, label="3D, random IC")
    gp = pairdist_trace(pl)
    ax1.plot(pl["t"][pl["t"] > pl["t"][-1] - 20.0],
             gp[pl["t"] > pl["t"][-1] - 20.0, 0], lw=0.9,
             label="3D, planar IC")
    ax1.set_xlabel("$t$ (last 20 s)")
    ax1.set_ylabel("$\\|x_{ij}\\|$")
    ax1.set_title("(b) one gap")
    ax1.legend(fontsize=6.5)

    f_r, _, ff, P = dominant_freq(rnd["t"][tail_slice(rnd["t"], 0.5)],
                                  g[tail_slice(rnd["t"], 0.5), 0])
    f_p, _, ff2, P2 = dominant_freq(pl["t"][tail_slice(pl["t"], 0.5)],
                                    gp[tail_slice(pl["t"], 0.5), 0])
    ax2 = fig.add_subplot(gs[2])
    ax2.semilogy(ff[1:], P[1:] / P.max(), lw=0.9,
                 label=f"breathing, $f={f_r:.3f}$ Hz")
    ax2.semilogy(ff2[1:], np.maximum(P2[1:] / P2.max(), 1e-6), lw=0.9,
                 alpha=0.8, label="planar: no peak")
    ax2.set_xlim(0, 1.2)
    ax2.set_xlabel("frequency (Hz)")
    ax2.set_ylabel("$|\\hat g|/\\max$")
    ax2.set_title("(c) spectra")
    ax2.legend(fontsize=6.5)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig3d-breathing.pdf"))
    plt.close(fig)


def fig_nscaling(force=False):
    """N-scaling from the stored scan data: breathing vs locked regime."""
    path = os.path.join(HERE, "..", "data", "n_scaling_results.json")
    recs = json.load(open(path))
    N = np.array([r["N"] for r in recs], float)
    fr = np.array([r["f_rot"] for r in recs])
    frad = np.array([r["f_rad"] for r in recs])
    ftan = np.array([r["f_tan"] for r in recs])
    sr = np.array([r["sigma_ratio"] for r in recs])
    order = np.argsort(N)

    # the near-DC FFT artefacts reported in the source notes are masked
    r_rad = (frad / fr)[order].copy()
    r_tan = (ftan / fr)[order].copy()
    # (the source notes flag N=10 radial and N=25 tangential as near-DC artefacts)
    r_rad[N[order] == 10] = np.nan
    r_tan[N[order] == 25] = np.nan

    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    ax[0].plot(N[order], r_rad, "o-", ms=4, label="$f_{\\rm rad}/f_{\\rm rot}$")
    ax[0].plot(N[order], r_tan, "s-", ms=4, label="$f_{\\rm tan}/f_{\\rm rot}$")
    ax[0].axhline(1.0, ls=":", color="k", lw=0.8)
    ax[0].set_xlabel("$N$")
    ax[0].set_ylabel("frequency ratio")
    ax[0].set_title("(a) breathing vs locked regime")
    ax[0].set_ylim(0.80, 2.02)
    ax[0].legend(fontsize=6.6, loc="lower center", ncol=2,
                 framealpha=0.9)

    ax[1].plot(N[order], sr[order], "o-", ms=4, color="tab:purple")
    ax[1].set_xlabel("$N$")
    ax[1].set_ylabel("$\\sigma_3/\\sigma_1$")
    ax[1].set_title("(b) non-planarity")
    ax[1].set_ylim(-0.05, 1.05)
    for a in ax:
        a.set_xscale("log")
        a.set_xticks([3, 4, 5, 6, 8, 10, 20, 35])
        a.set_xticklabels(["3", "4", "5", "6", "8", "10", "20", "35"],
                          fontsize=7)
        a.minorticks_off()
        a.set_xlim(2.6, 42)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig-nscaling.pdf"))
    plt.close(fig)


ALPHA_FORMS = ["standard", "power", "rational", "log", "stiff", "exponential"]


def fig_alpha(force=False):
    """Six continuous alpha forms: period is form independent."""
    import csv as _csv
    base = os.path.join(HERE, "..", "data", "alpha")
    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    for form in ALPHA_FORMS:
        path = os.path.join(base, f"results_{form}.csv")
        if not os.path.exists(path):
            continue
        rows = [r for r in _csv.DictReader(open(path)) if r["scan"] == "k2"]
        if not rows:
            continue
        k2 = np.array([float(r["k2"]) for r in rows])
        T = np.array([float(r["period"]) for r in rows])
        pr = np.array([float(r["peak_ratio"]) for r in rows])
        o = np.argsort(k2)
        ax[0].plot(k2[o], T[o], "o-", ms=3.2, lw=0.9, label=form)
        ax[1].plot(k2[o], pr[o], "o-", ms=3.2, lw=0.9, label=form)
    kk = np.linspace(0.15, 0.65, 120)
    ax[0].plot(kk, 0.52 * 2 * np.pi / np.sqrt(kk), "k--", lw=1.1,
               label="$0.52\\cdot 2\\pi/\\sqrt{k_2}$")
    ax[0].set_xlabel("$k_2$")
    ax[0].set_ylabel("$T_{\\rm breath}$ (s)")
    ax[0].set_title("(a) period: form independent")
    ax[0].legend(fontsize=5.4, ncol=2, loc="upper right")
    ax[1].set_xlabel("$k_2$")
    ax[1].set_ylabel("peak ratio")
    ax[1].set_title("(b) breathing strength: form dependent")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig-alpha.pdf"))
    plt.close(fig)


K2_DBL = [0.5, 0.6, 0.8, 1.0, 1.5]
K3_DBL = 0.2
J2 = np.array([[0.0, -1.0], [1.0, 0.0]])


def fig_double(force=False):
    """Double integrator: |omega| = sqrt(k3/k2) and the hexagon -> two triangles.

    The runs start *on* the regular-hexagon radial balance, i.e. they test the
    rate law and self-consistency rather than attraction (see the report).
    """
    rows = []
    traces = {}
    for k2 in K2_DBL:
        r0 = circle_root(K1, k2, K3_DBL)

        def build(k2=k2, r0=r0):
            om = np.sqrt(K3_DBL / k2)
            th = np.linspace(0, 2 * np.pi, 6, endpoint=False)
            e0 = np.column_stack([r0 * np.cos(th), r0 * np.sin(th)])
            v0 = e0 @ (om * J2).T
            w0 = e0 @ ((K3_DBL / om) * J2).T
            r = integrate_double(e0, v0, w0, D, MU, K1, k2, K3_DBL, 120.0, 400,
                                 method="LSODA", rtol=1e-9, atol=1e-11)
            return {"t": r["t"], "e": r["e"], "v": r["v"], "w": r["w"]}
        r = cached_run(f"double3_k2-{k2:g}", build, force)
        e, v = r["e"], r["v"]
        w = (e[..., 0] * v[..., 1] - e[..., 1] * v[..., 0]) / np.sum(e ** 2, -1)
        sl = tail_slice(r["t"], 0.5)
        rows.append((k2, float(np.nanmean(w[sl]))))
        traces[k2] = (r, np.linalg.norm(e, axis=-1), r0)
    rows = np.array(rows)

    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    pred = np.sqrt(K3_DBL / rows[:, 0])
    ax[0].plot(pred, pred, "k-", lw=1.0, label="$\sqrt{k_3/k_2}$")
    ax[0].plot(pred, np.abs(rows[:, 1]), "o", ms=5, label="measured")
    ax[0].set_xlabel("$\sqrt{k_3/k_2}$")
    ax[0].set_ylabel("measured $|\omega|$")
    ax[0].set_title("(a) angular speed")
    ax[0].legend(fontsize=7)

    r, rad, r0 = traces[0.8]
    for i in range(6):
        ax[1].plot(r["t"], rad[:, i], lw=0.9)
    ax[1].axhline(r0, ls="--", color="k", lw=1.0,
                  label=f"hexagon balance $r^*={r0:.2f}$")
    ax[1].set_xlabel("$t$")
    ax[1].set_ylabel("$\|e_i\|$")
    ax[1].set_title("(b) $k_2=0.8$: hexagon splits in two")
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig-double.pdf"))
    plt.close(fig)


def fig_jam(force=False):
    """Jammed branch: gap closes like 1/t while the observer diverges linearly."""
    def build():
        x = np.array([-15., -9., -3., 3., 9., 15.])
        xi0 = np.column_stack([x, np.zeros(6)])
        r = integrate_single(xi0, np.zeros((6, 2)), D, MU, K1, K2, 400.0, 2000,
                             method="BDF", rtol=1e-8, atol=1e-10)
        return {"t": r["t"], "xi": r["xi"], "vt": r["vt"]}
    r = cached_run("jam_collinear", build, force)
    g = pairdist_trace(r).min(axis=1)
    vn = np.linalg.norm(r["vt"], axis=(1, 2))

    fig, ax = plt.subplots(1, 2, figsize=(6.6, 2.5))
    c_end = (g[-1] - D) * r["t"][-1]
    ax[0].loglog(r["t"], g - D, lw=1.1, label="$\\min\\|x_{ij}\\|-d$")
    ax[0].loglog(r["t"], c_end / r["t"], "k--", lw=0.9, label="$c^*/t$")
    ax[0].set_xlabel("$t$")
    ax[0].set_ylabel("gap $-\\,d$")
    ax[0].set_title("(a) gap closes like $1/t$")
    ax[0].legend(fontsize=7)

    ax[1].plot(r["t"], vn / 6, lw=1.1, label="$\\frac{1}{N}\\sum_i\\|\\tilde v_i\\|$")
    ax[1].set_xlabel("$t$")
    ax[1].set_ylabel("observer")
    ax[1].set_title("(b) observer diverges linearly")
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig-jam.pdf"))
    plt.close(fig)



# --------------------------------------------------------------------------- #
# literature mechanisms: displacement consensus & distance gradient
# --------------------------------------------------------------------------- #
def _rot(th):
    return np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])


def _neighbours(n, edges):
    nbr = [[] for _ in range(n)]
    for i, j in edges:
        nbr[i].append(j)
        nbr[j].append(i)
    return nbr


def displacement_run(pstar, edges, vstar=None, kappa=0.0, scale=None,
                     misalign=None, tf=40.0, n_eval=1200, p0=None):
    """TAC-2021 displacement-consensus loop with modified weights / bad sensing.

    mu_ij (m x m) is built so that  kappa * sum_j mu_ij z*_ij = kappa * v*  when
    a common desired velocity v* is requested (Sec. IV-C of the paper).
    """
    m = len(pstar)
    nbr = _neighbours(m, edges)
    mu = {}
    if vstar is not None:
        for i in range(m):
            for j in nbr[i]:
                zs = pstar[i] - pstar[j]
                mu[(i, j)] = np.outer(vstar, zs) / (len(nbr[i]) * (zs @ zs))
    a = np.ones(m) if scale is None else np.asarray(scale, float)
    R = [np.eye(2)] * m if misalign is None else misalign

    def rhs(t, y):
        p = y.reshape(m, 2)
        u = np.zeros_like(p)
        for i in range(m):
            for j in nbr[i]:
                z = p[i] - p[j]
                zs = pstar[i] - pstar[j]
                if vstar is None:                     # bad sensing only
                    u[i] -= a[i] * (R[i] @ z) - zs
                else:                                # designed maneuvering
                    u[i] -= (a[i] * (R[i] @ z) - zs) + kappa * (mu[(i, j)] @ z)
        return u.ravel()
    if p0 is None:
        p0 = pstar + np.array([[0.5, -0.3], [-0.4, 0.5], [0.3, 0.4],
                               [-0.5, -0.6], [0.6, 0.2], [0.1, -0.5]])[:m]
    tt = np.linspace(0.0, tf, n_eval)
    sol = solve_ivp(rhs, (0.0, tf), p0.ravel(), t_eval=tt, method="LSODA",
                    rtol=1e-10, atol=1e-12)
    return {"t": sol.t, "p": sol.y.T.reshape(-1, m, 2)}


def distance_run(xstar, edges, dmap, tf=600.0, n_eval=6000):
    """TRO-2015 distance-gradient loop with per-agent perceived distances."""
    m = len(xstar)

    def rhs(t, y):
        x = y.reshape(m, 2)
        u = np.zeros_like(x)
        for i, j in edges:
            z = x[i] - x[j]
            nz = np.linalg.norm(z)
            u[i] -= z * (nz ** 2 - dmap[(i, j)] ** 2)
            u[j] += z * (nz ** 2 - dmap[(j, i)] ** 2)
        return u.ravel()
    tt = np.linspace(0.0, tf, n_eval)
    sol = solve_ivp(rhs, (0.0, tf), np.asarray(xstar, float).ravel(), t_eval=tt,
                    method="LSODA", rtol=1e-10, atol=1e-12)
    return {"t": sol.t, "x": sol.y.T.reshape(-1, m, 2)}


SQUARE = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
SQUARE_CYC = [(0, 1), (1, 2), (2, 3), (3, 0)]
SQUARE_TREE = [(0, 1), (1, 2), (2, 3)]
VSTAR = np.array([0.5, -0.3])


def fig_rotation_methods(force=False):
    """Reproduce the two literature mechanisms and their residual-motion boundary."""
    # (b) designed translational maneuvering, small kappa / (c) too large kappa
    man = {}
    for kap in [0.0, 1.0, 5.0]:
        man[kap] = cached_run(
            f"method_man_k{kap:g}",
            lambda kap=kap: displacement_run(SQUARE, SQUARE_TREE, VSTAR, kap,
                                             tf=40.0, n_eval=1200), force)
    # (d) one agent off, (e) all agents identically off
    bad1 = cached_run("method_bad_one", lambda: displacement_run(
        SQUARE, SQUARE_CYC, None, scale=[1.1, 1, 1, 1], tf=200.0, n_eval=2000),
        force)
    badall = cached_run("method_bad_all", lambda: displacement_run(
        SQUARE, SQUARE_CYC, None, scale=[1.1] * 4, tf=200.0, n_eval=2000),
        force)
    # (f) distance gradient with an inconsistent desired distance
    tri = np.array([[0.0, 0.0], [1.0, 0.0], [0.5, np.sqrt(3) / 2]])
    e3 = [(0, 1), (1, 2), (2, 0)]
    dmap = {}
    for i, j in e3:
        dmap[(i, j)] = dmap[(j, i)] = 1.0
    dmap[(1, 0)] = 1.06
    dg = cached_run("method_distgrad", lambda: distance_run(tri, e3, dmap),
                    force)

    fig, ax = plt.subplots(2, 3, figsize=(7.2, 4.5))

    def draw(a, run, key, label, lw=0.9):
        p = run[key]
        for i in range(p.shape[1]):
            a.plot(p[:, i, 0], p[:, i, 1], lw=lw)
            a.plot(p[-1, i, 0], p[-1, i, 1], "o", ms=3)
        a.set_aspect("equal")
        a.set_title(label, fontsize=9)

    # (a) pure displacement consensus: static
    draw(ax[0, 0], man[0.0], "p", "(a) $\\mu_{ij}=0$: static shape")
    ax[0, 0].plot(SQUARE[:, 0], SQUARE[:, 1], "k*", ms=4)

    draw(ax[0, 1], man[1.0], "p", "(b) designed $v^*$, small $\\kappa$")
    draw(ax[0, 2], man[5.0], "p", "(c) same $\\mu_{ij}$, $\\kappa$ too large")

    draw(ax[1, 0], bad1, "p", "(d) one agent with $a_i=1.1$")
    draw(ax[1, 1], badall, "p", "(e) every agent $a_i=1.1$")

    x = dg["x"]
    for i in range(x.shape[1]):
        ax[1, 2].plot(x[:, i, 0], x[:, i, 1], lw=0.5)
    ax[1, 2].set_aspect("equal")
    ax[1, 2].set_title("(f) inconsistent $d_{21}$: closed orbit", fontsize=9)
    for a in ax.ravel():
        a.set_xlabel("$p_1$", fontsize=8)
        a.set_ylabel("$p_2$", fontsize=8)
        a.tick_params(labelsize=6.5)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig-methods.pdf"))
    plt.close(fig)

    # numbers quoted in the text
    for kap in [0.0, 1.0, 5.0]:
        r = man[kap]
        v = (r["p"][-1] - r["p"][-101]) / (r["t"][-1] - r["t"][-101])
        g = np.linalg.norm(r["p"][:, 0] - r["p"][:, 1], axis=1)
        print(f"  kappa={kap}: mean v={np.round(v.mean(axis=0), 4)} "
              f"(pred {np.round(-kap * VSTAR, 4)}), side {g[0]:.3f}->{g[-1]:.3f}")
    for name, r in [("one agent off", bad1), ("all agents off", badall)]:
        v = (r["p"][-1] - r["p"][-501]) / (r["t"][-1] - r["t"][-501])
        c = r["p"].mean(axis=1, keepdims=True)
        rad = np.linalg.norm(r["p"][-1] - c[-1], axis=1)
        print(f"  {name}: |v|={np.linalg.norm(v.mean(axis=0)):.4f} "
              f"v-spread={np.linalg.norm(v - v.mean(axis=0), axis=1).max():.1e} "
              f"radii={np.round(rad, 3)}")
    g = np.linalg.norm(dg["x"][:, 0] - dg["x"][:, 1], axis=1)
    c = dg["x"].mean(axis=1, keepdims=True)
    rad = np.linalg.norm(dg["x"] - c, axis=-1)[:, 0]
    vv = np.linalg.norm(np.gradient(dg["x"], axis=0) /
                        (dg["t"][1] - dg["t"][0]), axis=-1)[:, 0]
    print(f"  distance mismatch: gap {g[0]:.3f}->{g[-1]:.4f}, radius "
          f"{rad[-1]:.4f}, speed {vv[-1]:.4f} (constant: "
          f"{vv[len(vv) // 2:].std():.1e})")

# --------------------------------------------------------------------------- #
# driver
# --------------------------------------------------------------------------- #
ORDER = ["fig1d", "fig2d-rotation", "fig2d-omega-sweep", "fig2d-shape-branch",
         "fig2d-degenerate", "fig2d-spectrum", "fig2d-direction",
         "fig2d-breathing", "fig2d-bistability", "fig3d-breathing",
         "fig-nscaling", "fig-alpha", "fig-double", "fig-jam",
         "fig-rotation-methods"]

FUNCS = {name: globals()[name.replace("-", "_")] for name in ORDER}

if __name__ == "__main__":
    style()
    args = sys.argv[1:]
    force = "--force" in args
    args = [a for a in args if not a.startswith("--")]
    targets = ORDER if (not args or args[0] == "all") else args
    for name in targets:
        t0 = time.time()
        print(f"[fig] {name}")
        FUNCS[name](force=force)
        print(f"       done in {time.time() - t0:.1f}s -> figures/{name}.pdf")
