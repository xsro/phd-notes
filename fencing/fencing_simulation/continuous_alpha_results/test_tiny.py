#!/usr/bin/env python3
"""Tiny test: shortest possible simulation."""
import time
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp

XI0 = np.array([
    [14.112419,  3.201258,  7.829904],
    [17.927146, 14.940464, -7.818223],
    [ 7.600707, -1.210858, -0.825751],
    [ 3.284788,  1.152349, 11.634188],
    [ 6.088302,  0.973400,  3.550906],
    [ 2.669395, 11.952633, -1.641266],
])
N = 6
d_col, mu = 5.0, 9.0
v0 = np.array([1.0, 0.0, 0.0])

def alpha(s):
    if s >= mu: return 0.0
    if s <= d_col: return 1e6
    return 1.0/(s - d_col) - 1.0/(mu - d_col)

def rhs(t, y):
    x = y[:3*N].reshape(N, 3)
    v = y[3*N:].reshape(N, 3)
    x_t = np.zeros(3) + v0 * t
    phi = np.zeros((N, 3))
    for i in range(N):
        for j in range(N):
            if i == j: continue
            xij = x[i] - x[j]
            dist = np.linalg.norm(xij)
            if d_col < dist <= mu:
                phi[i] += alpha(dist) * xij / dist
    u = phi + 0.5 * (x_t - x) + v
    v_dot = 0.5 * (x_t - x)
    return np.concatenate([u.ravel(), v_dot.ravel()])

y0 = np.zeros(6*N)
y0[:3*N] = XI0.ravel()

for T_MAX, DT in [(10, 0.1), (20, 0.1), (50, 0.2)]:
    t_eval = np.arange(0, T_MAX + DT, DT)
    t0 = time.time()
    sol = solve_ivp(rhs, [0, T_MAX], y0, method='LSODA',
                    t_eval=t_eval, rtol=1e-5, atol=1e-7, max_step=DT)
    elapsed = time.time() - t0
    print(f"T_MAX={T_MAX:.0f} DT={DT} → {elapsed:.2f}s, {len(sol.t)} steps, success={sol.success}")
    if not sol.success:
        print(f"  Message: {sol.message}")