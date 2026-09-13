#!/usr/bin/env python3
"""Test 3D with planar initial conditions. Saves data for plotting."""
import numpy as np
import warnings
from scipy.integrate import solve_ivp
from pathlib import Path

N = 6
d_col = 5.0
mu = 9.0
k1 = 0.5
k2 = 0.5
v0 = np.array([1.0, 0.0, 0.0])

# Planar initial conditions: regular hexagon in xy-plane
theta = np.linspace(0, 2*np.pi, N, endpoint=False)
R_init = 8.0
x0_init = np.column_stack([R_init * np.cos(theta), R_init * np.sin(theta), np.zeros(N)])
v_init = np.zeros((N, 3))
x_target0 = np.array([0.0, 0.0, 0.0])

print(f"Initial positions (planar):")
for i in range(N):
    print(f"  Agent {i+1}: ({x0_init[i,0]:.3f}, {x0_init[i,1]:.3f}, {x0_init[i,2]:.3f})")

def alpha(s):
    if s > mu:
        return 0.0
    if s <= d_col:
        warnings.warn(f"Collision risk: s={s:.4f} <= d={d_col}")
        return 1e6
    return 1.0 / (s - d_col) - 1.0 / (mu - d_col)

def compute_phi(x):
    phi = np.zeros((N, 3))
    for i in range(N):
        for j in range(N):
            if i == j:
                continue
            xij = x[i] - x[j]
            dist = np.linalg.norm(xij)
            if d_col < dist <= mu:
                phi[i] += alpha(dist) * xij / dist
    return phi

def dynamics(t, y):
    x = y[:3*N].reshape(N, 3)
    v = y[3*N:].reshape(N, 3)
    x_t = x_target0 + v0 * t
    phi = compute_phi(x)
    u = phi + k1 * (x_t - x) + v
    v_dot = k2 * (x_t - x)
    dy = np.zeros(6 * N)
    dy[:3*N] = u.flatten()
    dy[3*N:] = v_dot.flatten()
    return dy

T_MAX = 100.0
DT = 0.1
t_eval = np.arange(0, T_MAX + DT, DT)

y0 = np.zeros(6 * N)
y0[:3*N] = x0_init.flatten()
y0[3*N:] = v_init.flatten()

sol = solve_ivp(dynamics, [0, T_MAX], y0, method='LSODA',
                t_eval=t_eval, rtol=1e-9, atol=1e-11, max_step=DT)

t = sol.t
x_traj = sol.y[:3*N, :].reshape(N, 3, -1)
v_traj = sol.y[3*N:, :].reshape(N, 3, -1)
x_target_traj = x_target0.reshape(3, 1) + np.outer(v0, t)

# Save data
data_dir = Path(__file__).parent / "data"
data_dir.mkdir(exist_ok=True)
np.savez(
    data_dir / 'test_3d_planar.npz',
    t=t,
    x_traj=x_traj,
    v_traj=v_traj,
    x_target_traj=x_target_traj,
    params=dict(N=N, d_col=d_col, mu=mu, k1=k1, k2=k2, v0=v0, T_MAX=T_MAX, DT=DT)
)
print(f"Saved data to {data_dir / 'test_3d_planar.npz'}")

# Target-relative positions
xi_traj = x_traj - x_target_traj.reshape(1, 3, -1)

# Check planarity at the end
final_rel = xi_traj[:, :, -1]
centered = final_rel - np.mean(final_rel, axis=0)
U, S, Vt = np.linalg.svd(centered, full_matrices=False)
print(f"\n=== Planarity at t={T_MAX} ===")
print(f"  Singular values: {S}")
print(f"  sigma3/sigma1 = {S[2]/S[0]:.6f}")

# Check if z stays zero
print(f"\n=== z-coordinates (should stay ~0) ===")
print(f"  Final z: {xi_traj[:, 2, -1]}")
print(f"  Max |z| over time: {np.max(np.abs(xi_traj[:, 2, :])):.6f}")

# Check angular velocity
vell_traj = v_traj - v0.reshape(1, 3, 1)
omega_z = np.zeros(len(t))
for k in range(len(t)):
    total = 0.0
    total_weight = 0.0
    for i in range(N):
        xi, yi = xi_traj[i, 0, k], xi_traj[i, 1, k]
        vxi, vyi = vell_traj[i, 0, k], vell_traj[i, 1, k]
        r2 = xi**2 + yi**2
        if r2 > 0.01:
            total += (xi * vyi - yi * vxi) / r2
            total_weight += 1
    if total_weight > 0:
        omega_z[k] = total / total_weight

print(f"\n=== Angular velocity (last 10s) ===")
mask = t > (T_MAX - 10)
print(f"  Mean omega_z: {np.mean(omega_z[mask]):.6f}")
print(f"  Expected: sqrt(k2) = {np.sqrt(k2):.6f}")
print(f"  Std: {np.std(omega_z[mask]):.6f}")

# Check if vehicles track the target
print(f"\n=== Target tracking (last 10s) ===")
for i in range(N):
    avg_x = np.mean(xi_traj[i, 0, mask])
    avg_y = np.mean(xi_traj[i, 1, mask])
    avg_z = np.mean(xi_traj[i, 2, mask])
    print(f"  Agent {i+1}: <xi> = ({avg_x:.4f}, {avg_y:.4f}, {avg_z:.4f})")