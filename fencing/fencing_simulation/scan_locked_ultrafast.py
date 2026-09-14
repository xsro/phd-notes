#!/usr/bin/env python3
"""
Ultra-fast locked frequency scan. Minimal simulations, just enough to see trends.
"""
import numpy as np
from scipy.integrate import solve_ivp
import warnings
warnings.filterwarnings('ignore')

V0 = np.array([1.0, 0.0, 0.0])
T_MAX = 60.0
DT = 0.05
T_STEADY = 35.0

def make_rhs(N, d, mu, k1, k2):
    def alpha(s):
        if s > mu: return 0.0
        if s <= d: return 1e6
        return 1.0/(s-d) - 1.0/(mu-d)
    def rhs(t, y):
        x = y[:3*N].reshape(N,3)
        v = y[3*N:].reshape(N,3)
        xt = V0 * t
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

def gen_xi0(N, d, mu, seed=0):
    np.random.seed(seed)
    spread = max(15.0, 10.0*np.sqrt(N))
    x0 = np.random.randn(N,3)*spread
    for i in range(N):
        for j in range(i+1,N):
            att=0
            while np.linalg.norm(x0[i]-x0[j])<=d*1.2 and att<100:
                x0[j]=np.random.randn(3)*spread
                att+=1
    return x0

def measure(N, d, mu, k1, k2):
    xi0 = gen_xi0(N, d, mu)
    rhs = make_rhs(N, d, mu, k1, k2)
    y0 = np.zeros(6*N)
    y0[:3*N] = xi0.ravel()
    t_eval = np.arange(0, T_MAX+DT, DT)
    try:
        sol = solve_ivp(rhs, [0, T_MAX], y0, method='LSODA',
                        t_eval=t_eval, rtol=1e-7, atol=1e-9, max_step=DT)
    except:
        return None
    if not sol.success:
        return None
    t = sol.t
    x = sol.y[:3*N,:].reshape(N,3,-1)
    v = sol.y[3*N:,:].reshape(N,3,-1)
    xt = np.outer(V0, t)
    x_cm = np.mean(x - xt.reshape(1,3,-1), axis=0)
    x_rel = x - xt.reshape(1,3,-1) - x_cm.reshape(1,3,-1)
    r = np.linalg.norm(x_rel, axis=1)
    r_mag = np.linalg.norm(x_rel, axis=1)
    r_hat = x_rel / (r_mag[:,np.newaxis,:] + 1e-10)
    v_radial = np.sum(v * r_hat, axis=1)
    v_tan = v - v_radial[:,np.newaxis,:] * r_hat
    v_tan_mag = np.linalg.norm(v_tan, axis=1)
    mask = t > T_STEADY
    if np.sum(mask) < 50:
        mask = t > t[-1]*0.5
    r_ss = r[:,mask]
    v_tan_ss = v_tan_mag[:,mask]
    dt = t[1]-t[0]
    n_ss = len(t[mask])
    freqs = np.fft.rfftfreq(n_ss, d=dt)
    def find_peak(signal):
        fft_power = np.zeros(len(freqs))
        for s in signal:
            s_detrend = s - np.mean(s)
            fft_power += np.abs(np.fft.rfft(s_detrend))**2
        fft_power[0] = 0
        return freqs[np.argmax(fft_power)]
    f_rad = find_peak(r_ss)
    f_tan = find_peak(v_tan_ss)
    f_rot = np.sqrt(k2)/(2*np.pi)
    T_rot = 2*np.pi/np.sqrt(k2)
    ratio = f_rad/f_tan if f_tan > 0.001 else 0
    if abs(ratio-1.0) < 0.2:
        regime = "LOCKED"
    elif ratio > 1.3:
        regime = "BREATHING"
    else:
        regime = "OTHER"
    return {'f_rad':f_rad,'f_tan':f_tan,'f_rot':f_rot,'T_rot':T_rot,
            'regime':regime,'ratio':ratio}

# Quick test: just a few key points
print("LOCKED FREQUENCY vs PARAMETERS (quick)")
print(f"T_MAX={T_MAX}, freq_res≈{1/(T_MAX-T_STEADY):.4f} Hz")
print()

# 1. k2 scan at N=15 (locked regime)
print("1. k2 scan (N=15, k1=0.5, d=5):")
print(f"{'k2':>6s} {'f_lock':>9s} {'f_rot':>8s} {'f_lock/f_rot':>13s} {'regime':>9s}")
for k2 in [0.3, 0.4, 0.5, 0.6, 0.7]:
    N=15; mu=9.0*np.sqrt(N/6.0)
    r = measure(N, 5.0, mu, 0.5, k2)
    if r:
        print(f"{k2:6.2f} {r['f_rad']:9.4f} {r['f_rot']:8.4f} {r['f_rad']/r['f_rot']:13.3f} {r['regime']:>9s}")
    else:
        print(f"{k2:6.2f} {'FAIL':>9s}")

print()

# 2. k1 scan at N=15, k2=0.5
print("2. k1 scan (N=15, k2=0.5, d=5):")
print(f"{'k1':>6s} {'f_lock':>9s} {'f_rot':>8s} {'f_lock/f_rot':>13s} {'regime':>9s}")
for k1 in [0.3, 0.5, 0.7, 1.0]:
    N=15; mu=9.0*np.sqrt(N/6.0)
    r = measure(N, 5.0, mu, k1, 0.5)
    if r:
        print(f"{k1:6.2f} {r['f_rad']:9.4f} {r['f_rot']:8.4f} {r['f_rad']/r['f_rot']:13.3f} {r['regime']:>9s}")
    else:
        print(f"{k1:6.2f} {'FAIL':>9s}")

print()

# 3. N scan at k1=k2=0.5
print("3. N scan (k1=k2=0.5, d=5):")
print(f"{'N':>4s} {'mu':>6s} {'f_lock':>9s} {'f_rot':>8s} {'f_lock/f_rot':>13s} {'regime':>9s}")
for N in [5, 8, 12, 15, 20]:
    mu=9.0*np.sqrt(N/6.0)
    r = measure(N, 5.0, mu, 0.5, 0.5)
    if r:
        print(f"{N:4d} {mu:6.1f} {r['f_rad']:9.4f} {r['f_rot']:8.4f} {r['f_rad']/r['f_rot']:13.3f} {r['regime']:>9s}")
    else:
        print(f"{N:4d} {mu:6.1f} {'FAIL':>9s}")

print()

# 4. d scan at N=15, k1=k2=0.5
print("4. d scan (N=15, k1=k2=0.5):")
print(f"{'d':>6s} {'f_lock':>9s} {'f_rot':>8s} {'f_lock/f_rot':>13s} {'regime':>9s}")
for d in [3.0, 5.0, 7.0]:
    N=15; mu=9.0*np.sqrt(N/6.0)
    r = measure(N, d, mu, 0.5, 0.5)
    if r:
        print(f"{d:6.1f} {r['f_rad']:9.4f} {r['f_rot']:8.4f} {r['f_rad']/r['f_rot']:13.3f} {r['regime']:>9s}")
    else:
        print(f"{d:6.1f} {'FAIL':>9s}")

print()

# 5. mu scan at N=15, k1=k2=0.5, d=5
print("5. mu scan (N=15, k1=k2=0.5, d=5):")
print(f"{'mu':>6s} {'f_lock':>9s} {'f_rot':>8s} {'f_lock/f_rot':>13s} {'regime':>9s}")
for mu in [8.0, 12.0, 16.0, 20.0]:
    N=15
    r = measure(N, 5.0, mu, 0.5, 0.5)
    if r:
        print(f"{mu:6.1f} {r['f_rad']:9.4f} {r['f_rot']:8.4f} {r['f_rad']/r['f_rot']:13.3f} {r['regime']:>9s}")
    else:
        print(f"{mu:6.1f} {'FAIL':>9s}")

print()
print("Done.")