"""
Simulation for Case A — Rotating Equilibrium Orbit.

Produces:  data/case_a.npz

Run from the repository root:
    python simulation/sim_case_a.py
"""

import os
import sys
import numpy as np

# Add parent to path so fencing_ode can be imported
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fencing_ode import integrate, evaluate

# ── Parameters ───────────────────────────────────────────────────────────────
d, mu, k1, k2 = 5.0, 9.0, 0.8, 0.5
N = 5

# ── Initial conditions: regular pentagon ─────────────────────────────────────
# Place agents at vertices of a regular pentagon with radius R.
# The interaction forces keep agents apart, and the damping + tangential
# observer velocity drives the system toward a rigid-body rotation.
R0 = 6.0
angles = np.linspace(0, 2 * np.pi, N, endpoint=False)
XI0 = R0 * np.column_stack([np.cos(angles), np.sin(angles)])

# Initialise observer velocity to match the expected rotating equilibrium:
#   vt_i = sqrt(k2) · J · xi_i  +  k1·xi_i  -  phi_i
# so that the RHS is close to a pure rotation.
# For a regular pentagon, phi_i is radial (by symmetry).  We compute it
# numerically so the initial condition is as close to equilibrium as possible.
omega = np.sqrt(k2)
VT0 = omega * np.column_stack([-XI0[:, 1], XI0[:, 0]])  # tangential component
# NOTE: the radial correction (k1·xi - phi) is omitted intentionally;
#       the system will self-adjust to the true equilibrium.

# ── Integration ──────────────────────────────────────────────────────────────
tf = 800.0
n_frames = 300
t_skip = 0.0
t_eval = np.linspace(t_skip, tf, n_frames)

print(f"[Case A] Integrating to t={tf}  (N={N}, k1={k1}, k2={k2}, d={d}, mu={mu})")
print(f"         Initial radius R0 = {R0}")
rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, tf)
pos_all, vel_all = evaluate(sol, N, t_eval)
print(f"         Integration complete.  pos_all shape = {pos_all.shape}")

# ── Save data ────────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
data_dir = os.path.join(repo_root, "data")
os.makedirs(data_dir, exist_ok=True)

out_path = os.path.join(data_dir, "case_a.npz")
np.savez(out_path,
         pos_all=pos_all, vel_all=vel_all, t_eval=t_eval,
         d=d, mu=mu, k1=k1, k2=k2, N=N,
         XI0=XI0, VT0=VT0,
         description="Case A: Rotating Equilibrium Orbit")
print(f"         Data saved → {out_path}")