#!/usr/bin/env python3
"""Quick diagnostic: check 3D system behavior with correct indexing. Saves data."""
import numpy as np
from scipy.integrate import solve_ivp
from pathlib import Path

N = 6
d_col = 5.0
mu = 9.0
k1 = 0.5
k2 = 0.5
v0 = np.array([1.0, 0.0, 0.0])

np.random.seed(42)
x0_init = np.random.randn(N, 3) * 5.0
for i in range(N):
    for j in range(i+1, N):
        while np.linalg.norm(x0_init[i] - x0_init[j]) <= d_col:
            x0_init[j] = np.random.randn(3) * 5.0
v_init = np.zeros((N, 3))
x_target0 = np.array([0.0, 0.0, 0.0])

def alpha(s):
    if s <= d_col or s > mu:
        return 0.0
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

T_MAX = 200.0
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
    data_dir / 'diagnose_3d.npz',
    t=t,
    x_traj=x_traj,
    v_traj=v_traj,
    x_target_traj=x_target_traj,
    params=dict(N=N, d_col=d_col, mu=mu, k1=k1, k2=k2, v0=v0, T_MAX=T_MAX, DT=DT)
)
print(f"Saved data to {data_dir / 'diagnose_3d.npz'}")

# Target-relative positions
xi_traj = x_traj - x_target_traj.reshape(1, 3, -1)
vell_traj = v_traj - v0.reshape(1, 3, 1)

# Angular momentum
L = np.zeros((len(t), 3))
for k in range(len(t)):
    for i in range(N):
        L[k] += np.cross(xi_traj[i, :, k], vell_traj[i, :, k])

# Time-averaged position (last 50s)
mask = t > (T_MAX - 50)
print("=== Time-averaged target-relative positions (last 50s) ===")
for i in range(N):
    avg = np.mean(xi_traj[i, :, mask], axis=0)
    print(f"  Agent {i+1}: <ξ> = ({avg[0]:.4f}, {avg[1]:.4f}, {avg[2]:.4f}), ||<ξ>|| = {np.linalg.norm(avg):.4f}")

print(f"\n=== Angular momentum (last 10s) ===")
mask2 = t > (T_MAX - 10)
L_last = np.mean(L[mask2], axis=0)
L_std = np.std(L[mask2], axis=0)
print(f"  <L> = ({L_last[0]:.4f}, {L_last[1]:.4f}, {L_last[2]:.4f})")
print(f"  ||<L>|| = {np.linalg.norm(L_last):.4f}")
print(f"  sigma(L) = ({L_std[0]:.4f}, {L_std[1]:.4f}, {L_std[2]:.4f})")

# Planarity
final_rel = xi_traj[:, :, -1]
centered = final_rel - np.mean(final_rel, axis=0)
U, S, Vt = np.linalg.svd(centered, full_matrices=False)
print(f"\n=== Planarity ===")
print(f"  Singular values: {S}")
print(f"  σ₃/σ₁ = {S[2]/S[0]:.6f}")

# Check if L is aligned with principal directions
for k in range(3):
    cos_angle = abs(np.dot(L_last, Vt[k]) / (np.linalg.norm(L_last) * np.linalg.norm(Vt[k])))
    print(f"  |cos(L, V{k+1})| = {cos_angle:.6f}")

# Check target tracking
print(f"\n=== Target tracking (last 50s) ===")
for i in range(N):
    avg = np.mean(xi_traj[i, :, mask], axis=0)
    print(f"  Agent {i+1}: <ξ> = ({avg[0]:.4f}, {avg[1]:.4f}, {avg[2]:.4f})")

# Check if the formation is rotating
center_traj = np.mean(xi_traj, axis=0)
omega = np.zeros(len(t))
for k in range(len(t)):
    total = 0.0
    total_weight = 0.0
    for i in range(N):
        r = xi_traj[i, :, k] - center_traj[:, k]
        v = vell_traj[i, :, k]
        r2 = np.dot(r, r)
        if r2 > 0.01:
            total += np.dot(np.cross(r, v), L_last) / (np.linalg.norm(L_last) * r2)
            total_weight += 1
    if total_weight > 0:
        omega[k] = total / total_weight

print(f"\n=== Angular velocity (last 10s) ===")
print(f"  Mean omega: {np.mean(omega[mask2]):.6f}")
print(f"  Expected sqrt(k2): {np.sqrt(k2):.6f}")
print(f"  Std: {np.std(omega[mask2]):.6f}")