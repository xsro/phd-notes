# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "matplotlib"]
# ///
"""
result4 : output-feedback multi-agent fencing WITHOUT any velocity observer.

Agent dynamics (per component l of agent i):
    p_i0_dot = v_i0
    v_i0_dot = b_il(t) * u_il + w_il          (b_il: unknown sign, |b| in [bmin,bmax])

Controller (per agent i, component l):
    eta_dot = k1*p_i0 - phi_i ,   eta(0) = 0
    y       = eta + k2*p_i0
    e1      = phi_t(t)*y
    alpha1  = c1 * tan(pi*e1/(2*kappa1)) * cos( tan(pi*e1/(2*kappa1)) )
    xi1_dot = -a1*xi1 + xi2 ,  xi2_dot = -a2*xi1 + u     (input-driven filter)
    e2      = xi2 - alpha1
    u       = -c2*tan(pi*e2/(2*kappa2))

The control law uses neither v_i0, nor neighbour velocities, nor b_il, nor d_bar.
Run:  uv run --no-project sim_result4.py
"""

import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

rng = np.random.default_rng(0)

# ----------------------------------------------------------------- parameters
N, m = 4, 2
T, dt, stride = 20.0, 1e-4, 25

# control gains (positive constants, independent of the plant)
k1, k2 = 1.0, 1.0
kappa1, kappa2 = 1.00, 2.00
c1, c2 = 20.0, 20.0
a1, a2 = 1.0, 1.0
tau = 8.0

# plant facts (NOT used by the controller)
b_min = 0.80 * np.ones((N, m))
b_max = 2.00
b_switch = np.array([10.0, 30.0])   # sign switches inside the tested window
d_bar = 0.20 * np.ones(N)
u0_bar = 0.15

# pairwise potential (an instance of (A5))
d_safe, mu, l0, c_a, c_b = 0.45, 1.20, 0.75, 0.30, 0.02

# numeric safety clamps (only diagnostics: never active if the funnel holds)
TAN_CLIP, U_MAX = 0.9999, 1e4


def bmat(t):
    """Unknown sign-switching diagonal gain, piecewise constant in t."""
    B = np.zeros((N, m))
    for i in range(N):
        for l in range(m):
            mag = b_min[i, l] + 0.3 * ((2 * i + 3 * l) % 3)
            mag = min(mag, b_max)
            sw = b_switch + 3 * i + 2 * l
            ns = int(np.sum(t >= sw))
            B[i, l] = (-1) ** ns * mag
    return B


def dist_agent(t, i):
    amp = d_bar[i]
    return np.array([0.8 * amp * np.sin(1.7 * t + 0.4 * i),
                     0.7 * amp * np.cos(1.3 * t + 0.6 * i)])


def target_acc(t):
    return np.array([0.8 * u0_bar * np.sin(0.6 * t),
                     0.8 * u0_bar * np.cos(0.5 * t)])


def coeff(s):
    """pairwise coefficient: collision barrier + linear restoring + barrier at mu."""
    if s < 1e-9:
        return 0.0
    rho = max(1.0 / (s - d_safe) - 1.0 / (mu - d_safe), 0.0) if s < mu else 0.0
    attr = -c_a * max(s - l0, 0.0)
    barr = -c_b * max(1.0 / (mu - s) - 1.0 / (mu - l0), 0.0) if s < mu else 0.0
    return rho + attr + barr


def field(p):
    """phi_i : purely relative-position based, pairwise antisymmetric -> sum_i phi_i = 0."""
    phi = np.zeros((N, m))
    for i in range(N):
        for j in range(i + 1, N):
            pij = p[i] - p[j]
            s = np.linalg.norm(pij)
            if s < 1e-9:
                continue
            c = coeff(s)
            phi[i] += c * pij / s
            phi[j] -= c * pij / s
    return phi


def phi_t(t):
    return np.sin(np.pi * t / (2 * tau)) if t < tau else 1.0


# ------------------------------------------------------------------ initial data
p = np.array([[2.00, 0.40], [0.40, 1.80], [-1.80, -0.30], [-0.20, -1.90]])
v = np.array([[0.00, 0.10], [-0.10, 0.00], [0.00, -0.10], [0.10, 0.00]])
eta = np.zeros((N, m))
xi1 = np.zeros((N, m))
xi2 = np.zeros((N, m))
p0 = np.array([0.0, 0.0])
v0 = np.array([0.15, 0.0])

Nt = int(np.floor(T / dt)) + 1
rec = {k: [] for k in ("t", "p", "p0", "u", "e1r", "e2r", "y", "dist", "sep", "vrel", "phi", "B")}
clips = np.zeros(3, dtype=int)

for k in range(Nt):
    t = k * dt

    phi_i = field(p)
    y = eta + k2 * (p - p0)
    e1 = phi_t(t) * y

    r1 = e1 / kappa1
    clips[0] += int(np.sum(np.abs(r1) > TAN_CLIP))
    r1 = np.clip(r1, -TAN_CLIP, TAN_CLIP)
    eta1 = np.tan(0.5 * np.pi * r1)
    alpha1 = c1 * eta1 * np.cos(eta1)

    e2 = xi2 - alpha1
    r2 = e2 / kappa2
    clips[1] += int(np.sum(np.abs(r2) > TAN_CLIP))
    r2 = np.clip(r2, -TAN_CLIP, TAN_CLIP)
    u = -c2 * np.tan(0.5 * np.pi * r2)
    clips[2] += int(np.sum(np.abs(u) > U_MAX))
    u = np.clip(u, -U_MAX, U_MAX)

    if k % stride == 0:
        rec["t"].append(t)
        rec["p"].append(p.copy())
        rec["p0"].append(p0.copy())
        rec["u"].append(u.copy())
        rec["e1r"].append(r1.copy())
        rec["e2r"].append(r2.copy())
        rec["y"].append(y.copy())
        rec["phi"].append(phi_i.copy())
        rec["B"].append(bmat(t))
        rec["dist"].append(np.linalg.norm(p.mean(axis=0) - p0))
        s_min = min(np.linalg.norm(p[i] - p[j]) for i in range(N) for j in range(i + 1, N))
        rec["sep"].append(s_min)
        rec["vrel"].append(np.linalg.norm(v - v0, axis=1))

    if k == Nt - 1:
        break

    d_i = np.array([dist_agent(t, i) for i in range(N)])
    B = bmat(t)
    w = d_i                      # absolute velocity dynamics: v_i_dot = B u + d_i

    p_new = p + dt * v
    v_new = v + dt * (B * u + w)
    eta = eta + dt * (k1 * (p - p0) - phi_i)
    xi1, xi2 = xi1 + dt * (-a1 * xi1 + xi2), xi2 + dt * (-a2 * xi1 + u)
    p, v = p_new, v_new
    p0 = p0 + dt * v0
    v0 = v0 + dt * target_acc(t)

H = {key: np.array(val) for key, val in rec.items()}
tt = H["t"]

# -------------------------------------------------------------------- metrics
m1 = float(np.max(np.abs(H["e1r"])))
m2 = float(np.max(np.abs(H["e2r"])))
maxu = float(np.max(np.abs(H["u"])))
dist_all = float(np.max(H["dist"]))
dist_tail = float(np.max(H["dist"][tt >= T - 10]))
minsep = float(np.min(H["sep"]))
maxvrel = float(np.max(H["vrel"]))
Bmax = float(np.max(np.abs(H["B"])))

ec = H["e1r"].mean(axis=1)                       # centroid-level e_1 (per component)
Dec = np.zeros(len(tt))
Dec[1:] = np.max(np.abs(np.diff(ec, axis=0)) / np.diff(tt)[:, None], axis=1)
kappa_meas = float(np.max(Dec) / k1)
Vbar_meas = float(np.max(H["vrel"]))
rho2_meas = maxu

os.makedirs("figures", exist_ok=True)
lines = [
    "=== result4 simulation metrics (Python / uv) ===",
    f"N={N} m={m} T={T} dt={dt}",
    f"k1={k1} k2={k2} kappa1={kappa1} kappa2={kappa2} c1={c1} c2={c2} a1={a1} a2={a2} tau={tau}",
    f"max |e1|/kappa1         = {m1:.4f}",
    f"max |e2|/kappa2         = {m2:.4f}",
    f"max |u|                 = {maxu:.4f}",
    f"max |b|                 = {Bmax:.4f}",
    f"max dist(p0,co(p)) all  = {dist_all:.4f}",
    f"max dist(p0,co(p)) tail = {dist_tail:.4f}",
    f"min pairwise distance   = {minsep:.4f}   (d = {d_safe})",
    f"max ||v_i - v_0||       = {maxvrel:.4f}",
    f"kappa_meas = sup|d e_c/dt|/k1 = {kappa_meas:.4f}",
    f"clip events (e1,e2,u)   = {clips[0]}, {clips[1]}, {clips[2]}",
]
open("metrics.txt", "w").write("\n".join(lines) + "\n")
print("\n".join(lines))

# ---------------------------------------------------------------------- plots
plt.rcParams.update({"font.size": 10, "axes.grid": True, "figure.dpi": 110})
cols = plt.cm.tab10(np.arange(N))

# Fig 1: trajectories + final hull
fig, ax = plt.subplots(figsize=(5.4, 4.8))
ax.plot(H["p0"][:, 0], H["p0"][:, 1], "k-", lw=1.6, label="target")
for i in range(N):
    ax.plot(H["p"][:, i, 0], H["p"][:, i, 1], color=cols[i], lw=1.2, label=f"agent {i+1}")
    ax.plot(H["p"][-1, i, 0], H["p"][-1, i, 1], "o", color=cols[i], ms=6)
Pf = H["p"][-1]
ang = np.arctan2(Pf[:, 1], Pf[:, 0])
order = np.argsort(ang)
ax.fill(Pf[order, 0], Pf[order, 1], color="0.75", alpha=0.35, ec="0.4", label="final hull")
ax.plot(H["p0"][-1, 0], H["p0"][-1, 1], "p", color="goldenrod", ms=12, label="target (final)")
ax.set_xlabel("$x$"), ax.set_ylabel("$y$"), ax.set_aspect("equal")
ax.set_title("result4: trajectories and final convex hull")
ax.legend(loc="best", fontsize=8, ncol=2)
fig.tight_layout()
fig.savefig("figures/fig4_traj.png")
plt.close(fig)

# Fig 2: funnel ratios
fig, axs = plt.subplots(2, 1, figsize=(6.6, 4.4), sharex=True)
axs[0].plot(tt, np.max(np.abs(H["e1r"]), axis=(1, 2)), lw=1.3)
axs[0].axhline(1.0, color="r", ls="--", lw=1.1)
axs[0].set_ylabel(r"$\max_{i,l}|e_{i1,l}|/\kappa_1$")
axs[0].set_title(r"level-1 funnel on $y_i=\eta_i+k_2p_{i0}$")
axs[1].plot(tt, np.max(np.abs(H["e2r"]), axis=(1, 2)), lw=1.3, color="C1")
axs[1].axhline(1.0, color="r", ls="--", lw=1.1)
axs[1].set_ylabel(r"$\max_{i,l}|e_{i2,l}|/\kappa_2$")
axs[1].set_xlabel("$t$")
axs[1].set_title(r"level-2 funnel on $\xi_{i2}-\alpha_{i1}$")
fig.tight_layout()
fig.savefig("figures/fig4_funnel.png")
plt.close(fig)

# Fig 3: centroid error + predicted envelope
fig, ax = plt.subplots(figsize=(6.6, 3.9))
ax.semilogy(tt, np.maximum(H["dist"], 1e-8), lw=1.4, label="simulation")
i0 = int(np.searchsorted(tt, tau))
env = H["dist"][i0] * np.exp(-(k1 / k2) * (tt[tt >= tau] - tau)) + kappa_meas
ax.semilogy(tt[tt >= tau], env, "r--", lw=1.3,
            label=r"$\varrho(\tau)e^{-(k_1/k_2)(t-\tau)}+\kappa$")
ax.axhline(max(kappa_meas, 1e-8), color="k", ls=":", lw=1.1,
           label=r"$\kappa=\sup|\dot{\bar e}|/k_1$ (measured)")
ax.set_xlabel("$t$"), ax.set_ylabel(r"$\mathrm{dist}(p_0,\mathrm{co}(p))$")
ax.set_title("centroid: exponential entry into the prescribed ball")
ax.legend(loc="upper right", fontsize=8)
fig.tight_layout()
fig.savefig("figures/fig4_dist.png")
plt.close(fig)

# Fig 4: input + separation
fig, axs = plt.subplots(2, 1, figsize=(6.6, 4.6), sharex=True)
axs[0].plot(tt, np.max(np.abs(H["u"]), axis=(1, 2)), lw=1.2)
axs[0].axvline(25, color="0.6", ls="--", lw=1)
axs[0].axvline(45, color="0.6", ls="--", lw=1)
axs[0].set_ylabel(r"$\max_{i,l}|u_{i,l}|$")
axs[0].set_title(r"control effort (dashed: sign switches of $b_{il}(t)$)")
axs[1].plot(tt, H["sep"], lw=1.2, color="C2")
axs[1].axhline(d_safe, color="r", ls="--", lw=1.2, label="$d$")
axs[1].axhline(mu, color="k", ls=":", lw=1.1, label=r"$\mu$")
axs[1].set_ylabel(r"$\min_{i<j}\|p_{ij}\|$"), axs[1].set_xlabel("$t$")
axs[1].set_title("collision avoidance / separation")
axs[1].legend(loc="upper right", fontsize=8)
fig.tight_layout()
fig.savefig("figures/fig4_u_sep.png")
plt.close(fig)

print("figures written to figures/")
