"""
Quick 2D tangential period analysis using inline RHS (fast LSODA).
Uses known case3.py initial conditions for N=5 breathing.
"""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.fft import fft, fftfreq

# ── Known case3 ICs ─────────────────────────────────────────────────────────
N = 5
XI0 = np.array([
    [-10.584563, -12.532987],
    [ -5.382648,   5.904905],
    [ -0.177347,   9.175920],
    [ -7.352506,  -4.805249],
    [ -6.288338,  12.435830]
])
VT0 = np.array([
    [ 3.528048, -1.274511],
    [-0.511988, -1.485443],
    [ 1.972071, -3.679895],
    [-3.460489, -0.767699],
    [-2.039248,  2.761800]
])
v0 = np.array([1.0, 0.0])
v_init = VT0 + v0

d_col = 5.0
mu = 9.0
k1 = 0.5
k2 = 0.5

# ── Inline RHS ──────────────────────────────────────────────────────────────
def alpha(s):
    if s > mu: return 0.0
    if s <= d_col: return 1e6
    return 1.0/(s-d_col) - 1.0/(mu-d_col)

def rhs(t, y):
    x = y[:2*N].reshape(N, 2)
    v = y[2*N:].reshape(N, 2)
    xt = np.array([0.,0.]) + v0 * t
    phi = np.zeros((N, 2))
    for i in range(N):
        for j in range(N):
            if i == j: continue
            e = x[i] - x[j]
            s = np.linalg.norm(e)
            if d_col < s <= mu:
                phi[i] += alpha(s) * e / s
    u = phi + k1 * (xt - x) + v
    vd = k2 * (xt - x)
    return np.concatenate([u.ravel(), vd.ravel()])

# ── Integrate ───────────────────────────────────────────────────────────────
T_MAX = 300.0
DT = 0.02
t_eval = np.arange(0, T_MAX + DT, DT)

y0 = np.zeros(4*N)
y0[:2*N] = XI0.ravel()
y0[2*N:] = v_init.ravel()

print(f"Integrating 2D (N={N}, T_MAX={T_MAX}s, DT={DT}s)...")
sol = solve_ivp(rhs, [0, T_MAX], y0, method='LSODA',
                t_eval=t_eval, rtol=1e-8, atol=1e-10, max_step=DT)
print(f"Done. {len(sol.t)} steps, success={sol.success}")

t = sol.t
x = sol.y[:2*N, :].reshape(N, 2, -1)
v = sol.y[2*N:, :].reshape(N, 2, -1)

# ── Steady state ────────────────────────────────────────────────────────────
mask = t > (t[-1] - 200)
t_ss = t[mask]
dt_ss = t_ss[1] - t_ss[0]
n_ss = len(t_ss)

x_target = np.outer(v0, t_ss)  # (2, n_ss)

# CM relative to target
x_cm = np.mean(x[:, :, mask] - x_target.reshape(1, 2, -1), axis=0)  # (2, n_ss)

# Radial
x_rel = x[:, :, mask] - x_target.reshape(1, 2, -1) - x_cm.reshape(1, 2, -1)
r = np.linalg.norm(x_rel, axis=1)  # (N, n_ss)

# Tangential velocity
v_ss = v[:, :, mask]
r_hat = x_rel / (r[:, np.newaxis, :] + 1e-10)
v_rad = np.sum(v_ss * r_hat, axis=1)
v_tan = v_ss - v_rad[:, np.newaxis, :] * r_hat
v_tan_mag = np.linalg.norm(v_tan, axis=1)

# Angular position
theta = np.arctan2(x_rel[:, 1, :], x_rel[:, 0, :])
theta_unw = np.unwrap(theta, axis=1)
theta_dot = np.gradient(theta_unw, dt_ss, axis=1)
theta_dot_mean = np.mean(np.abs(theta_dot), axis=0)

# Pairwise distances
pd = np.array([np.linalg.norm(x[i, :, mask] - x[j, :, mask], axis=0)
               for i in range(N) for j in range(i+1, N)])

# ── FFT ─────────────────────────────────────────────────────────────────────
freqs = fftfreq(n_ss, d=dt_ss)

def dominant_freq(signal):
    s = signal - np.mean(signal)
    sp = np.abs(fft(s))**2
    sp[0] = 0
    idx = np.argmax(sp)
    return freqs[idx], sp, idx

f_rad, rad_pow, _ = dominant_freq(np.mean(r, axis=0))
f_tan, tan_pow, _ = dominant_freq(np.mean(v_tan_mag, axis=0))
f_theta, theta_pow, _ = dominant_freq(theta_dot_mean)
f_pd, pd_pow, _ = dominant_freq(np.mean(pd, axis=0))

T_rot = 2*np.pi/np.sqrt(k2)

print(f"\n{'='*60}")
print(f"2D Breathing Tangential Analysis")
print(f"{'='*60}")
print(f"Parameters: N={N}, k1={k1}, k2={k2}, d={d_col}, mu={mu}")
print(f"T_rot = 2*pi/sqrt(k2) = {T_rot:.4f} s")
print()
print(f"{'Component':<20s} {'f (Hz)':>10s} {'T (s)':>10s} {'T/T_rot':>10s}")
print(f"{'-'*50}")
print(f"{'radial (r)':<20s} {f_rad:10.4f} {1/f_rad:10.4f} {1/f_rad/T_rot:10.4f}")
print(f"{'tangential (v_tan)':<20s} {f_tan:10.4f} {1/f_tan:10.4f} {1/f_tan/T_rot:10.4f}")
print(f"{'angular (theta_dot)':<20s} {f_theta:10.4f} {1/f_theta:10.4f} {1/f_theta/T_rot:10.4f}")
print(f"{'pairwise dist':<20s} {f_pd:10.4f} {1/f_pd:10.4f} {1/f_pd/T_rot:10.4f}")
print(f"{'rigid rotation':<20s} {'':>10s} {T_rot:10.4f} {1.0:10.4f}")

print(f"\nf_rad / f_tan = {f_rad/f_tan:.4f}")
print(f"f_theta / f_tan = {f_theta/f_tan:.4f}")

# Top 3 peaks
print(f"\n--- Top 3 FFT peaks ---")
for name, pow_data in [('radial', rad_pow), ('tangential', tan_pow),
                        ('angular', theta_pow), ('pairwise', pd_pow)]:
    idx_top = np.argsort(pow_data[1:])[-3:] + 1
    peaks = [(freqs[i], 1/freqs[i] if freqs[i] > 0.001 else float('inf'), pow_data[i]) for i in idx_top]
    print(f"  {name}:")
    for f, T, p in peaks:
        print(f"    f={f:.4f} Hz, T={T:.4f}s, power={p:.2e}")