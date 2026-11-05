# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "matplotlib"]
# ///
"""
result4' : the *corrected* output-feedback fencing law.

Diagnosis (see also the LaTeX note):  the TF+BF+OF transplant of the TAC-2022 paper
    - has a destabilising orientation term  (eta*cos(eta) > 0 for |eta| < pi/2),
    - keeps the physical velocity error only *bounded* (input-driven filter error),
    - lives on the (infinitely steep) tan wall -> chattering + input spikes.
Fix used here: keep the integral sliding variable of the note's method 2, but replace
the unavailable v_i0 by a HIGH-GAIN observer driven by the MEASURED relative position.
Because the periodic law gives |u| <= R with R *a priori computable*, the unknown
input seen by the observer (b*u + w) is bounded by a known constant  b_max*R + w_bar,
so no time-varying / unbounded observer gain is needed (this is exactly what made
observer.tex awkward).

    eta_dot = k1*p - phi
    s       = eta + k2*p + v_hat
    Delta   = k1*p - phi + k2*v_hat
    u       = R * tanh( sin(pi*s/eps)/delta )            (smoothed periodic law)
    R       = ( |Delta| + w_bar + eps/T_r ) / b_min
    observer:  p_hat_dot = v_hat + (2/eps_o)*(p - p_hat)
               v_hat_dot =        (1/eps_o^2)*(p - p_hat)

Run:  uv run --no-project sim_result4_fixed.py
"""

import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ------------------------------------------------------------------ parameters
N, m = 4, 2
T, dt, stride = 60.0, 2e-4, 20

k1, k2 = 1.2, 2.0                 # same as the note's result2
eps_s = 0.25                      # periodic-law cell  (eps_i,l)
T_r = 2.0                         # desired arrival time per cell
lam = eps_s / T_r
delta_s = 0.05                    # smoothing width of sgn(sin(.))
eps_o = 0.05                      # high-gain observer parameter

b_min, b_max = 0.80, 1.40
w_bar = 0.35                      # |d_i - u_0| bound  (d_bar + u0_bar)
b_switch = np.array([25.0, 45.0])

d_safe, mu, l0, c_a, c_b = 0.45, 1.20, 0.75, 0.30, 0.02
clip_u = 60.0


def bmat(t):
    B = np.zeros((N, m))
    for i in range(N):
        for l in range(m):
            mag = min(b_min + 0.3 * ((2 * i + 3 * l) % 3), b_max)
            B[i, l] = (-1) ** int(np.sum(t >= b_switch + 3 * i + 2 * l)) * mag
    return B


def dist_agent(t, i):
    a = 0.20
    return np.array([0.8 * a * np.sin(1.7 * t + 0.4 * i), 0.7 * a * np.cos(1.3 * t + 0.6 * i)])


def target_acc(t):
    return np.array([0.8 * 0.15 * np.sin(0.6 * t), 0.8 * 0.15 * np.cos(0.5 * t)])


def coeff(s):
    if s < 1e-9:
        return 0.0
    rho = max(1.0 / (s - d_safe) - 1.0 / (mu - d_safe), 0.0) if s < mu else 0.0
    return rho - c_a * max(s - l0, 0.0) - (c_b * max(1.0 / (mu - s) - 1.0 / (mu - l0), 0.0) if s < mu else 0.0)


def field(p):
    phi = np.zeros((N, m))
    for i in range(N):
        for j in range(i + 1, N):
            pij, s = p[i] - p[j], np.linalg.norm(p[i] - p[j])
            if s < 1e-9:
                continue
            c = coeff(s)
            phi[i] += c * pij / s
            phi[j] -= c * pij / s
    return phi


# ------------------------------------------------------------------ initial data
p = np.array([[2.00, 0.40], [0.40, 1.80], [-1.80, -0.30], [-0.20, -1.90]])
v = np.array([[0.00, 0.10], [-0.10, 0.00], [0.00, -0.10], [0.10, 0.00]])
eta = np.zeros((N, m))
p0 = np.zeros(2)
v0 = np.array([0.15, 0.0])

p_hat = p.copy()
v_hat = np.zeros((N, m))

Nt = int(T / dt) + 1
rec = {k: [] for k in ("t", "p", "p0", "u", "s", "dist", "sep", "verr", "vhat_err", "R")}

for k in range(Nt):
    t = k * dt

    ph = field(p)
    p_rel = p - p0

    eta = eta + dt * (k1 * p_rel - ph)                 # integral state (local)
    s = eta + k2 * p_rel + v_hat
    Delta = k1 * p_rel - ph + k2 * v_hat
    R = (np.abs(Delta) + w_bar + lam) / b_min
    u = R * np.tanh(np.sin(np.pi * s / eps_s) / delta_s)
    u = np.clip(u, -clip_u, clip_u)

    if k % stride == 0:
        rec["t"].append(t)
        rec["p"].append(p.copy())
        rec["p0"].append(p0.copy())
        rec["u"].append(u.copy())
        rec["s"].append(s.copy())
        rec["R"].append(R.copy())
        rec["dist"].append(np.linalg.norm(p.mean(0) - p0))
        rec["sep"].append(min(np.linalg.norm(p[i] - p[j]) for i in range(N) for j in range(i + 1, N)))
        rec["verr"].append(np.max(np.linalg.norm(v - v0, axis=1)))
        rec["vhat_err"].append(np.max(np.abs(v_hat - v)))

    if k == Nt - 1:
        break

    # plant
    B = bmat(t)
    d_i = np.array([dist_agent(t, i) for i in range(N)])
    u0 = target_acc(t)
    p_new = p + dt * v
    v_new = v + dt * (B * u + d_i)
    p0 = p0 + dt * v0
    v0 = v0 + dt * u0
    p, v = p_new, v_new

    # high-gain observer on the measured relative position  p - p0
    ymeas = p - p0
    e = ymeas - p_hat
    p_hat = p_hat + dt * (v_hat + (2.0 / eps_o) * e)
    v_hat = v_hat + dt * ((1.0 / eps_o ** 2) * e)

H = {k: np.array(vv) for k, vv in rec.items()}
tt = H["t"]
ss = np.abs(H["s"])
tail = tt >= T - 10

# |s| relative to the cell, and the residual slip
s_ratio = float(np.max(ss[:, :, :]) / eps_s)
s_tail = float(np.max(ss[tail]) / eps_s)
dist_tail = float(np.max(H["dist"][tail]))
dist_all = float(np.max(H["dist"]))
minsep = float(np.min(H["sep"]))
maxu = float(np.max(np.abs(H["u"])))
maxR = float(np.max(H["R"]))
verr_tail = float(np.max(H["verr"][tail]))
verr_all = float(np.max(H["verr"]))
obs_err = float(np.max(H["vhat_err"][tt > 5 * eps_o]))

lines = [
    "=== result4' (corrected) simulation metrics ===",
    f"N={N} m={m} T={T} dt={dt}",
    f"k1={k1} k2={k2} eps={eps_s} T_r={T_r} delta={delta_s} eps_o={eps_o}",
    f"max |s|/eps            = {s_ratio:.3f}   (tail: {s_tail:.3f})",
    f"max |u|                = {maxu:.3f}   (max R = {maxR:.3f})",
    f"max dist all / tail    = {dist_all:.4f} / {dist_tail:.4f}",
    f"min pairwise distance  = {minsep:.4f}   (d = {d_safe})",
    f"max |v_i - v_0| all/tail = {verr_all:.4f} / {verr_tail:.4f}",
    f"observer err max |v_hat-v| (t>5eps_o) = {obs_err:.4f}",
]
os.makedirs("figures", exist_ok=True)
open("metrics_fixed.txt", "w").write("\n".join(lines) + "\n")
print("\n".join(lines))

# ---------------------------------------------------------------------- figures
plt.rcParams.update({"font.size": 10, "axes.grid": True, "figure.dpi": 110})
cols = plt.cm.tab10(np.arange(N))

fig, axs = plt.subplots(1, 2, figsize=(9.4, 4.0))
ax = axs[0]
ax.plot(H["p0"][:, 0], H["p0"][:, 1], "k-", lw=1.4, label="target")
for i in range(N):
    ax.plot(H["p"][:, i, 0], H["p"][:, i, 1], color=cols[i], lw=1.0, label=f"agent {i+1}")
Pf = H["p"][-1]
ang = np.arctan2(Pf[:, 1], Pf[:, 0])
o = np.argsort(ang)
ax.fill(Pf[o, 0], Pf[o, 1], color="0.8", alpha=0.4, ec="0.4", label="final hull")
ax.set_aspect("equal"), ax.set_xlabel("$x$"), ax.set_ylabel("$y$")
ax.set_title("trajectories and final hull")
ax.legend(fontsize=7, ncol=2, loc="best")

ax = axs[1]
ax.plot(tt, H["dist"], lw=1.3, label=r"$\mathrm{dist}(p_0,\mathrm{co}(p))$")
ax.set_xlabel("$t$"), ax.set_ylabel("centroid error")
ax.set_yscale("log")
ax.set_title("fencing error of the centroid")
ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig("figures/fig4b_traj_dist.png")
plt.close(fig)

fig, axs = plt.subplots(2, 1, figsize=(7.0, 4.6), sharex=True)
axs[0].plot(tt, np.max(np.abs(H["s"]), axis=(1, 2)) / eps_s, lw=1.1)
axs[0].axhline(1, color="r", ls="--", lw=1)
axs[0].set_ylabel(r"$\max|s_{i,l}|/\varepsilon$")
axs[0].set_title(r"periodic-sliding variable (dashed: $\varepsilon$)")
axs[1].plot(tt, np.max(np.abs(H["u"]), axis=(1, 2)), lw=1.1)
axs[1].plot(tt, np.max(H["R"], axis=(1, 2)), "r--", lw=1.0, label="$R$")
axs[1].set_ylabel(r"$\max|u_{i,l}|$"), axs[1].set_xlabel("$t$")
axs[1].legend(fontsize=8)
axs[1].set_title("control and the a-priori gain bound $R$")
fig.tight_layout()
fig.savefig("figures/fig4b_s_u.png")
plt.close(fig)

print("figures written: figures/fig4b_traj_dist.png, figures/fig4b_s_u.png")
