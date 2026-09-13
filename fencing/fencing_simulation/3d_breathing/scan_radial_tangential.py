"""
Study axial (radial) vs tangential motion in the 3D breathing limit cycle.

For each vehicle, decompose motion into:
  - radial: r_i(t) = |x_i(t) - x_cm(t)|  (breathing mode)
  - tangential: theta_i(t) = angular position relative to CM (rotation mode)

FFT each component separately to find f_radial and f_tangential.
Scan across k1, k2, d, mu, N.
"""

import sys
import numpy as np
from scipy.integrate import solve_ivp

# ── Inline RHS ──────────────────────────────────────────────────────────────

def make_rhs(N, d, mu, k1, k2):
    def alpha(s):
        if s > mu: return 0.0
        if s <= d: return 1e6
        return 1.0/(s-d) - 1.0/(mu-d)
    def rhs(t, y):
        x = y[:3*N].reshape(N,3)
        v = y[3*N:].reshape(N,3)
        xt = np.array([0.,0.,0.]) + np.array([1.,0.,0.])*t
        phi = np.zeros((N,3))
        for i in range(N):
            for j in range(N):
                if i==j: continue
                e = x[i]-x[j]
                s = np.linalg.norm(e)
                if d < s <= mu:
                    phi[i] += alpha(s)*e/s
        u = phi + k1*(xt-x) + v
        vd = k2*(xt-x)
        return np.concatenate([u.ravel(), vd.ravel()])
    return rhs

def measure(N, d, mu, k1, k2, xi0, T_MAX=120.0, DT=0.05):
    rhs = make_rhs(N, d, mu, k1, k2)
    y0 = np.zeros(6*N)
    y0[:3*N] = xi0.ravel()
    t_eval = np.arange(0, T_MAX+DT, DT)
    try:
        sol = solve_ivp(rhs, [0, T_MAX], y0, method='LSODA',
                        t_eval=t_eval, rtol=1e-8, atol=1e-10, max_step=DT)
    except:
        return None
    if not sol.success:
        return None

    t = sol.t
    x = sol.y[:3*N,:].reshape(N,3,-1)
    xt = np.zeros((3,len(t))) + np.outer([1.,0.,0.], t)

    # Center of mass (relative to target)
    x_cm = np.mean(x - xt.reshape(1,3,-1), axis=0)  # (3, n_t)

    # Radial distances from CM
    r = np.linalg.norm(x - xt.reshape(1,3,-1) - x_cm.reshape(1,3,-1), axis=1)  # (N, n_t)

    # Tangential: angular velocity = d(theta)/dt
    # For each vehicle, compute angle relative to CM
    # Use projection onto plane perpendicular to angular momentum axis
    # Simpler: compute velocity component perpendicular to radial direction
    v = sol.y[3*N:,:].reshape(N,3,-1)
    # Radial unit vector
    x_rel = x - xt.reshape(1,3,-1) - x_cm.reshape(1,3,-1)  # (N, 3, n_t)
    r_mag = np.linalg.norm(x_rel, axis=1)  # (N, n_t)
    r_hat = x_rel / (r_mag[:, np.newaxis, :] + 1e-10)  # (N, 3, n_t)
    # Tangential velocity: v - (v·r_hat)*r_hat
    v_radial = np.sum(v * r_hat, axis=1)  # (N, n_t) radial speed
    v_tan = v - v_radial[:, np.newaxis, :] * r_hat  # (N, 3, n_t) tangential velocity
    v_tan_mag = np.linalg.norm(v_tan, axis=1)  # (N, n_t) tangential speed

    # Steady state
    mask = t > t[-1]*0.5
    r_ss = r[:, mask]
    v_tan_ss = v_tan_mag[:, mask]
    dt = t[1] - t[0]
    n_ss = len(t[mask])

    freqs = np.fft.rfftfreq(n_ss, d=dt)

    # FFT of radial component (average over vehicles)
    rad_fft = np.zeros(len(freqs))
    for ri in r_ss:
        ri = ri - np.mean(ri)
        rad_fft += np.abs(np.fft.rfft(ri))**2
    rad_fft[0] = 0
    idx_rad = np.argmax(rad_fft)
    f_rad = freqs[idx_rad]

    # FFT of tangential component
    tan_fft = np.zeros(len(freqs))
    for vt in v_tan_ss:
        vt = vt - np.mean(vt)
        tan_fft += np.abs(np.fft.rfft(vt))**2
    tan_fft[0] = 0
    idx_tan = np.argmax(tan_fft)
    f_tan = freqs[idx_tan]

    # Also: pairwise distance FFT (the original breathing measure)
    pd = np.array([np.linalg.norm(x[i]-x[j],axis=0)
                   for i in range(N) for j in range(i+1,N)])
    pd_ss = pd[:, mask]
    pd_fft = np.zeros(len(freqs))
    for p in pd_ss:
        p = p - np.mean(p)
        pd_fft += np.abs(np.fft.rfft(p))**2
    pd_fft[0] = 0
    idx_pd = np.argmax(pd_fft)
    f_pd = freqs[idx_pd]

    # Peak ratios
    pr_rad = rad_fft[idx_rad] / (np.sum(rad_fft) + 1e-20)
    pr_tan = tan_fft[idx_tan] / (np.sum(tan_fft) + 1e-20)

    # Sigma (planarity)
    xi = x - xt.reshape(1,3,-1)
    fr = xi[:,:,-1]
    c = fr - np.mean(fr,axis=0)
    _,S,_ = np.linalg.svd(c, full_matrices=False)
    sig = S[2]/S[0] if len(S)>=3 else 1.0

    T_rot = 2*np.pi/np.sqrt(k2)

    return {
        'f_rad': f_rad, 'T_rad': 1.0/f_rad if f_rad > 0.001 else None,
        'f_tan': f_tan, 'T_tan': 1.0/f_tan if f_tan > 0.001 else None,
        'f_pd': f_pd, 'T_pd': 1.0/f_pd if f_pd > 0.001 else None,
        'pr_rad': pr_rad, 'pr_tan': pr_tan,
        'sigma': sig,
        'T_rot': T_rot,
        'rad_fft': rad_fft, 'tan_fft': tan_fft, 'freqs': freqs
    }

def gen_xi0(N, d, seed=0):
    np.random.seed(seed)
    return np.random.randn(N,3)*10

# ── Scans ───────────────────────────────────────────────────────────────────

print("=== k2 scan: radial vs tangential (k1=0.5, N=4, d=5, mu=9) ===")
print(f"{'k2':>6s}  {'T_rad':>7s}  {'T_tan':>7s}  {'T_pd':>7s}  {'T_rot':>7s}  {'f_rad/f_tan':>10s}  {'pr_rad':>7s}  {'pr_tan':>7s}  {'sigma':>7s}")
for k2 in [0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70]:
    r = measure(4, 5.0, 9.0, 0.5, k2, gen_xi0(4, 5.0, 0))
    if r:
        ratio = r['f_rad']/r['f_tan'] if r['f_tan'] > 0.001 else 0
        print(f"{k2:6.2f}  {r['T_rad']:7.3f}  {r['T_tan']:7.3f}  {r['T_pd']:7.3f}  {r['T_rot']:7.3f}  {ratio:10.3f}  {r['pr_rad']:7.3f}  {r['pr_tan']:7.3f}  {r['sigma']:7.3f}")

print("\n=== k1 scan: radial vs tangential (k2=0.5, N=4, d=5, mu=9) ===")
print(f"{'k1':>6s}  {'T_rad':>7s}  {'T_tan':>7s}  {'T_pd':>7s}  {'T_rot':>7s}  {'f_rad/f_tan':>10s}  {'pr_rad':>7s}  {'pr_tan':>7s}  {'sigma':>7s}")
for k1 in [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75]:
    r = measure(4, 5.0, 9.0, k1, 0.5, gen_xi0(4, 5.0, 0))
    if r:
        ratio = r['f_rad']/r['f_tan'] if r['f_tan'] > 0.001 else 0
        print(f"{k1:6.2f}  {r['T_rad']:7.3f}  {r['T_tan']:7.3f}  {r['T_pd']:7.3f}  {r['T_rot']:7.3f}  {ratio:10.3f}  {r['pr_rad']:7.3f}  {r['pr_tan']:7.3f}  {r['sigma']:7.3f}")

print("\n=== d scan: radial vs tangential (k1=k2=0.5, N=4, mu=9) ===")
print(f"{'d':>6s}  {'T_rad':>7s}  {'T_tan':>7s}  {'T_pd':>7s}  {'T_rot':>7s}  {'f_rad/f_tan':>10s}  {'pr_rad':>7s}  {'pr_tan':>7s}  {'sigma':>7s}")
for d in [3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0]:
    r = measure(4, d, 9.0, 0.5, 0.5, gen_xi0(4, d, 0))
    if r:
        ratio = r['f_rad']/r['f_tan'] if r['f_tan'] > 0.001 else 0
        print(f"{d:6.1f}  {r['T_rad']:7.3f}  {r['T_tan']:7.3f}  {r['T_pd']:7.3f}  {r['T_rot']:7.3f}  {ratio:10.3f}  {r['pr_rad']:7.3f}  {r['pr_tan']:7.3f}  {r['sigma']:7.3f}")

print("\n=== mu scan: radial vs tangential (k1=k2=0.5, N=4, d=5) ===")
print(f"{'mu':>6s}  {'T_rad':>7s}  {'T_tan':>7s}  {'T_pd':>7s}  {'T_rot':>7s}  {'f_rad/f_tan':>10s}  {'pr_rad':>7s}  {'pr_tan':>7s}  {'sigma':>7s}")
for mu in [6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0]:
    r = measure(4, 5.0, mu, 0.5, 0.5, gen_xi0(4, 5.0, 0))
    if r:
        ratio = r['f_rad']/r['f_tan'] if r['f_tan'] > 0.001 else 0
        print(f"{mu:6.1f}  {r['T_rad']:7.3f}  {r['T_tan']:7.3f}  {r['T_pd']:7.3f}  {r['T_rot']:7.3f}  {ratio:10.3f}  {r['pr_rad']:7.3f}  {r['pr_tan']:7.3f}  {r['sigma']:7.3f}")

print("\n=== N scan: radial vs tangential (k1=k2=0.5, d=5, mu=9) ===")
print(f"{'N':>6s}  {'T_rad':>7s}  {'T_tan':>7s}  {'T_pd':>7s}  {'T_rot':>7s}  {'f_rad/f_tan':>10s}  {'pr_rad':>7s}  {'pr_tan':>7s}  {'sigma':>7s}")
for n in [3, 4, 5, 6]:
    r = measure(n, 5.0, 9.0, 0.5, 0.5, gen_xi0(n, 5.0, 0))
    if r:
        ratio = r['f_rad']/r['f_tan'] if r['f_tan'] > 0.001 else 0
        print(f"{n:6d}  {r['T_rad']:7.3f}  {r['T_tan']:7.3f}  {r['T_pd']:7.3f}  {r['T_rot']:7.3f}  {ratio:10.3f}  {r['pr_rad']:7.3f}  {r['pr_tan']:7.3f}  {r['sigma']:7.3f}")

print("\nDone.")