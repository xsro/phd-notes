"""
Simulation for Case C — Breathing Limit Cycle.

Uses the exact initial conditions and parameters from the original
counterexample (case3.py / animation) to produce a persistent periodic
shape oscillation.

Produces:  data/case_c.npz

Run from the repository root:
    python simulation/sim_case_c.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fencing_ode import integrate, evaluate

# ── Parameters (identical to case3.py) ───────────────────────────────────────
d, mu, k1, k2 = 5.0, 9.0, 0.5, 0.5
N = 5

# ── Initial conditions (identical to case3.py) ───────────────────────────────
XI0 = np.array([[-10.584563, -12.532987],
                [-5.382648,   5.904905],
                [-0.177347,   9.17592 ],
                [-7.352506,  -4.805249],
                [-6.288338,  12.43583 ]])

VT0 = np.array([[ 3.528048, -1.274511],
                [-0.511988, -1.485443],
                [ 1.972071, -3.679895],
                [-3.460489, -0.767699],
                [-2.039248,  2.7618  ]])

# Reference: original case3.py at
#   fencing-rotation-conjecture/simulation/case3.py

# ── Integration ──────────────────────────────────────────────────────────────
tf = 3000.0
n_frames = 300
t_skip = 0.5
t_eval = np.linspace(t_skip, tf, n_frames)

print(f"[Case C] Integrating to t={tf}  (N={N}, k1={k1}, k2={k2}, d={d}, mu={mu})")
print(f"         Initial conditions from case3.py (counterexample)")
rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, tf)
pos_all, vel_all = evaluate(sol, N, t_eval)
print(f"         Integration complete.  pos_all shape = {pos_all.shape}")

# ── Save data ────────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
data_dir = os.path.join(repo_root, "data")
os.makedirs(data_dir, exist_ok=True)

out_path = os.path.join(data_dir, "case_c.npz")
np.savez(out_path,
         pos_all=pos_all, vel_all=vel_all, t_eval=t_eval,
         d=d, mu=mu, k1=k1, k2=k2, N=N,
         XI0=XI0, VT0=VT0,
         description="Case C: Breathing Limit Cycle")
print(f"         Data saved → {out_path}")