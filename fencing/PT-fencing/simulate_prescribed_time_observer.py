#!/usr/bin/env python3
"""
Numerical verification: prescribed-time decentralized target estimator
(upgrade of the Remark-6 estimator (25) in 1111_observer.md).

Compares:
  (A) original constant-gain estimator (25)   -> exponential (asymptotic)
  (B) time-varying-gain estimator (PT)        -> prescribed-time (exact at t = T)

PT estimator:
    d eps_i / dt = rho_i + k1 * mu(t)   * psi_i
    d rho_i / dt =           k2 * mu(t)^2 * psi_i
    psi_i = sum_{j in N_i} (eps_j - eps_i) + g_i(t) (x0 - eps_i)
    mu(t) = h / (T - t)   for t in [0, T)   (blow-up gain)

Theory: with  k1 > 1/(h * lambda_min(M))  and  k2 > 0, the estimation error
converges to zero exactly at the prescribed time t = T (temporal scaling proof).
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from pathlib import Path

# ---------------- topology ----------------
N = 5
# path graph 1-2-3-4-5, only agent 1 detects the target
edges = [(0, 1), (1, 2), (2, 3), (3, 4)]
L = np.zeros((N, N))
for i, j in edges:
    L[i, i] += 1; L[j, j] += 1
    L[i, j] -= 1; L[j, i] -= 1
G = np.diag([1.0, 0.0, 0.0, 0.0, 0.0])   # g_i = 1 only for agent 1
M = L + G
lam_min = np.linalg.eigvalsh(M).min()
print(f"lambda_min(M) = {lam_min:.6f}")

# ---------------- target ----------------
x0_0 = 2.0      # initial target position
nu0 = 0.5       # constant target velocity  (x0(t) = x0_0 + nu0 * t)

# ---------------- common params ----------------
np.random.seed(7)
eps0 = np.random.uniform(-4, 4, N)     # initial estimates
rho0 = np.zeros(N)

# ---------------- (A) original estimator ----------------
kappa = 1.0
gamma = 0.5

def dynA(t, y):
    eps, rho = y[:N], y[N:]
    x0 = x0_0 + nu0 * t
    psi = np.zeros(N)
    for i in range(N):
        psi[i] = np.sum(eps[list_of_neighbors[i]] - eps[i]) + G[i, i] * (x0 - eps[i])
    deps = kappa * psi + rho
    drho = gamma * kappa * psi
    return np.concatenate([deps, drho])

# ---------------- (B) prescribed-time estimator ----------------
T = 3.0          # prescribed convergence time
h = 2.0          # exponent of the blow-up function
k1 = 2.0 / (h * lam_min)   # satisfy k1 > 1/(h * lam_min)
k2 = 1.0

def mu(t):
    # cap gain for numerical robustness near t = T
    m = h / (T - t)
    return min(m, 1e6)

def dynB(t, y):
    eps, rho = y[:N], y[N:]
    x0 = x0_0 + nu0 * t
    psi = np.zeros(N)
    for i in range(N):
        psi[i] = np.sum(eps[list_of_neighbors[i]] - eps[i]) + G[i, i] * (x0 - eps[i])
    m = mu(t)
    deps = rho + k1 * m * psi
    drho = k2 * m * m * psi
    return np.concatenate([deps, drho])

list_of_neighbors = {i: [] for i in range(N)}
for i, j in edges:
    list_of_neighbors[i].append(j)
    list_of_neighbors[j].append(i)

# ---------------- simulate ----------------
t_end = T * 0.9995  # stop just before blow-up
t_eval = np.linspace(0, t_end, 2000)

yA0 = np.concatenate([eps0, rho0])
solA = solve_ivp(dynA, (0, t_end), yA0, t_eval=t_eval, method="BDF",
                 rtol=1e-8, atol=1e-10)

yB0 = np.concatenate([eps0, rho0])
solB = solve_ivp(dynB, (0, t_end), yB0, t_eval=t_eval, method="BDF",
                 rtol=1e-8, atol=1e-10, max_step=0.01)

# ---------------- errors ----------------
def errs(sol):
    eps = sol.y[:N, :]
    rho = sol.y[N:, :]
    x0 = x0_0 + nu0 * sol.t
    nu = np.full_like(sol.t, nu0)
    e_eps = np.abs(eps - x0[None, :])
    e_rho = np.abs(rho - nu[None, :])
    return e_eps, e_rho

eA_eps, eA_rho = errs(solA)
eB_eps, eB_rho = errs(solB)

# ---------------- plots ----------------
out = Path(__file__).parent
fig, axes = plt.subplots(2, 2, figsize=(12, 8))

for ax, e, label in [(axes[0, 0], eA_eps, "original (exponential)"),
                     (axes[0, 1], eB_eps, "prescribed-time"),
                     (axes[1, 0], eA_rho, "original (exponential)"),
                     (axes[1, 1], eB_rho, "prescribed-time")]:
    for i in range(N):
        ax.plot(solA.t if e is eA_eps or e is eA_rho else solB.t, e[i], lw=1.2)
    ax.set_yscale("log")
    ax.set_title(label)
    ax.set_xlabel("t")
    ax.grid(True, which="both", alpha=0.3)

axes[0, 0].set_ylabel("|eps_i - x0|")
axes[1, 0].set_ylabel("|rho_i - nu0|")

# mark T
for ax in axes.flat:
    ax.axvline(T, color="r", ls="--", lw=1, label="t = T")
axes[0, 1].legend()

fig.suptitle(f"Decentralized target estimator: exponential vs prescribed-time (T={T})")
fig.tight_layout()
fig.savefig(out / "prescribed_time_observer.png", dpi=130)
print("saved:", out / "prescribed_time_observer.png")

# ---------------- summary ----------------
print(f"\nFinal (t={t_end:.3f}) max |eps-x0|  :")
print(f"  original       : {eA_eps[:, -1].max():.3e}")
print(f"  prescribed-time: {eB_eps[:, -1].max():.3e}")
print(f"Final max |rho-nu0|  :")
print(f"  original       : {eA_rho[:, -1].max():.3e}")
print(f"  prescribed-time: {eB_rho[:, -1].max():.3e}")
