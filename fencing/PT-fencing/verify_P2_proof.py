#!/usr/bin/env python3
"""
Verify P2 theorem (1111_observer_P2_proof.md):
  target with BOUNDED acceleration a0(t) (non-constant velocity),
  observer PT-25' (time-varying gain + sliding term).

Two-stage integration to avoid the blow-up singularity at t=T:
  Stage 1 [0, 0.9995*T]: blow-up gain (capped)  -> error shrinks to ~0  (Part I)
  Stage 2 [0.9995*T, 1.6*T]: mu=0 (frozen), compare sigma>0 vs sigma=0 (Part II)
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from pathlib import Path

# ---- topology (path 1-2-3-4-5, only agent 1 detects target) ----
N = 5
edges = [(0,1),(1,2),(2,3),(3,4)]
L = np.zeros((N,N))
for i,j in edges:
    L[i,i]+=1; L[j,j]+=1; L[i,j]-=1; L[j,i]-=1
G = np.diag([1.0,0,0,0,0])
M = L + G
lam_min = np.linalg.eigvalsh(M).min()
print(f"lambda_min(M) = {lam_min:.6f}")

nb = {i:[] for i in range(N)}
for i,j in edges:
    nb[i].append(j); nb[j].append(i)

# ---- target: bounded acceleration ----
def x0(t): return 2.0 + 0.5*t + 0.3*np.sin(1.2*t)
def v0(t): return 0.5 + 0.3*1.2*np.cos(1.2*t)
def a0(t): return -0.3*1.2**2*np.sin(1.2*t)
a_bar = 0.3*1.2**2

# ---- observer params ----
T = 3.0; h = 2.0
k1 = 5.0/(h*lam_min)   # larger k1 -> faster rho convergence (see P2 proof note 2)
k2 = 1.0
sigma = 1.2*a_bar
MU_CAP = 1e4

def mu(t):
    if t >= T: return 0.0
    return min(h/(T-t), MU_CAP)

def sign_smooth(x, s=1000.0):
    return np.tanh(s*x)

def dyn(t, y, sig):
    eps, rho = y[:N], y[N:]
    psi = np.zeros(N)
    for i in range(N):
        psi[i] = sum(eps[i]-eps[j] for j in nb[i]) + G[i,i]*(eps[i]-x0(t))
    deps = rho - k1*mu(t)*psi
    drho = -k2*mu(t)**2*psi - sig*sign_smooth(psi)
    return np.concatenate([deps, drho])

np.random.seed(3)
eps0 = np.random.uniform(-4,4,N)
rho0 = np.zeros(N)
y0 = np.concatenate([eps0, rho0])

t_switch = 0.9995*T
# ---- stage 1: reaching [0, t_switch] ----
t1 = np.linspace(0, t_switch, 3000)
sol1 = solve_ivp(lambda t,y: dyn(t,y,sigma), (0,t_switch), y0, t_eval=t1,
                 method="RK45", rtol=1e-8, atol=1e-10, max_step=0.002)
y_switch = sol1.y[:,-1]

# ---- stage 2: frozen (mu=0) [t_switch, 1.6T], sigma vs 0 ----
t2 = np.linspace(t_switch, 1.6*T, 3000)
sol_sm = solve_ivp(lambda t,y: dyn(t,y,sigma), (t_switch,1.6*T), y_switch, t_eval=t2,
                   method="RK45", rtol=1e-8, atol=1e-10, max_step=0.002)
sol_ns = solve_ivp(lambda t,y: dyn(t,y,0.0),  (t_switch,1.6*T), y_switch, t_eval=t2,
                   method="RK45", rtol=1e-8, atol=1e-10, max_step=0.002)

def concat(sol1, sol2):
    t = np.concatenate([sol1.t, sol2.t])
    y = np.concatenate([sol1.y, sol2.y], axis=1)
    return t, y

t_sm, y_sm = concat(sol1, sol_sm)
t_ns, y_ns = concat(sol1, sol_ns)

def err(t, y):
    e = np.abs(y[:N,:] - x0(t)[None,:])
    r = np.abs(y[N:,:] - v0(t)[None,:])
    return e, r

e_sm, r_sm = err(t_sm, y_sm)
e_ns, r_ns = err(t_ns, y_ns)

# ---- summary ----
iT = np.argmin(np.abs(t_sm - T))
print(f"\nat t=T:     max|eps-x0| = {e_sm[:,iT].max():.3e}   (both sigma cases, same stage-1)")
print(f"             max|rho-v0| = {r_sm[:,iT].max():.3e}")
print(f"at t=1.6T:  max|eps-x0| sliding={e_sm[:,-1].max():.3e}   no-sliding={e_ns[:,-1].max():.3e}")
print(f"             max|rho-v0| sliding={r_sm[:,-1].max():.3e}   no-sliding={r_ns[:,-1].max():.3e}")

# ---- plots ----
out = Path(__file__).parent
fig, axes = plt.subplots(2,2, figsize=(12,8))
for ax,e,lab in [(axes[0,0],e_sm,"with sliding (sigma>0)"),
                 (axes[0,1],e_ns,"without sliding (sigma=0)"),
                 (axes[1,0],r_sm,"with sliding"),
                 (axes[1,1],r_ns,"without sliding")]:
    for i in range(N):
        ax.plot(t_sm, e[i], lw=1.1)
    ax.set_yscale("log"); ax.set_title(lab); ax.grid(True, which="both", alpha=0.3)
    ax.axvline(T, color="r", ls="--", lw=1)
axes[0,0].set_ylabel("|eps_i - x0|"); axes[1,0].set_ylabel("|rho_i - v0|")
for ax in axes[1]: ax.set_xlabel("t")
fig.suptitle("PT-25' with bounded acceleration a0(t)=-0.432 sin(1.2t): Part I reaching + Part II sliding")
fig.tight_layout()
fig.savefig(out/"P2_proof_verification.png", dpi=130)
print("saved:", out/"P2_proof_verification.png")
