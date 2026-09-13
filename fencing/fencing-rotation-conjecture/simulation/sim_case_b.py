"""
Simulation for Case B — Jammed Boundary.

At least one pair of agents approaches the collision distance d,
position converges, and the observer diverges.

Produces:  data/case_b.npz

Run from the repository root:
    python simulation/sim_case_b.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fencing_ode import integrate, evaluate

# ── Parameters ───────────────────────────────────────────────────────────────
d, mu, k1, k2 = 5.0, 9.0, 0.5, 0.5
N = 5

# ── Initial conditions: two agents near collision ────────────────────────────
# Place agents 0 and 1 very close together (distance ≈ d + ε) so the
# strong repulsive interaction, combined with the damping, causes them
# to converge to the collision boundary.
epsilon = 0.25
XI0 = np.array([
    [ d + epsilon,  0.0        ],   # agent 0 — just outside collision radius
    [-d - epsilon,  0.0        ],   # agent 1 — just outside, opposite side
    [ 0.0,          8.0        ],   # agent 2 — far away
    [-7.0,         -5.0        ],   # agent 3
    [ 7.0,         -5.0        ],   # agent 4
])

# Small observer velocities pointing toward the origin to encourage collapse
VT0 = np.array([
    [-0.1,  0.0],
    [ 0.1,  0.0],
    [ 0.0, -0.05],
    [ 0.05, 0.05],
    [-0.05, 0.05],
])

# ── Integration ──────────────────────────────────────────────────────────────
tf = 300.0
n_frames = 250
t_skip = 0.0
t_eval = np.linspace(t_skip, tf, n_frames)

print(f"[Case B] Integrating to t={tf}  (N={N}, k1={k1}, k2={k2}, d={d}, mu={mu})")
print(f"         Initial min distance ≈ {d + epsilon}")
rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, tf)
pos_all, vel_all = evaluate(sol, N, t_eval)
print(f"         Integration complete.  pos_all shape = {pos_all.shape}")

# Check final minimum distance
final_dists = []
for i in range(N):
    for j in range(i + 1, N):
        s = np.linalg.norm(pos_all[-1, i] - pos_all[-1, j])
        final_dists.append(s)
print(f"         Final min distance = {min(final_dists):.4f}  (d = {d})")

# ── Save data ────────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
data_dir = os.path.join(repo_root, "data")
os.makedirs(data_dir, exist_ok=True)

out_path = os.path.join(data_dir, "case_b.npz")
np.savez(out_path,
         pos_all=pos_all, vel_all=vel_all, t_eval=t_eval,
         d=d, mu=mu, k1=k1, k2=k2, N=N,
         XI0=XI0, VT0=VT0,
         description="Case B: Jammed Boundary")
print(f"         Data saved → {out_path}")