#!/usr/bin/env python3
"""
Quick locked-frequency scan using N=4 (fast simulations).
"""
import numpy as np
from scipy.integrate import solve_ivp
import warnings
warnings.filterwarnings('ignore')

V0 = np.array([1.0, 0.0, 0.0])
T_MAX = 80.0
DT = 0.05
T_STEADY = 45.0

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
    ratio = f_rad/f_tan if f_tan > 0.001 else 0
    if abs(ratio-1.0) < 0.2:
        regime = "LOCKED"
    elif ratio > 1.3:
        regime = "BREATHING"
    else:
        regime = "OTHER"
    return {'f_rad':f_rad,'f_tan':f_tan,'f_rot':f_rot,
            'regime':regime,'ratio':ratio}

print("LOCKED FREQUENCY vs PARAMETERS (N=4, fast)")
print(f"T_MAX={T_MAX}, freq_res≈{1/(T_MAX-T_STEADY):.4f} Hz")
print()

# 1. k1 scan (k2=0.5, N=4, d=5, mu=9)
print("1. k1 scan (N=4, k2=0.5, d=5, mu=9):")
print(f"{'k1':>6s} {'f_rad':>8s} {'f_tan':>8s} {'f_rot':>8s} {'f_rad/f_rot':>11s} {'f_rad/f_tan':>11s} {'regime':>9s}")
for k1 in [0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.90, 1.00]:
    r = measure(4, 5.0, 9.0, k1, 0.5)
    if r:
        print(f"{k1:6.2f} {r['f_rad']:8.4f} {r['f_tan']:8.4f} {r['f_rot']:8.4f} {r['f_rad']/r['f_rot']:11.3f} {r['ratio']:11.3f} {r['regime']:>9s}")
    else:
        print(f"{k1:6.2f} {'FAIL':>8s}")

print()

# 2. d scan (k1=k2=0.5, N=4, mu=9)
print("2. d scan (N=4, k1=k2=0.5, mu=9):")
print(f"{'d':>6s} {'f_rad':>8s} {'f_tan':>8s} {'f_rot':>8s} {'f_rad/f_rot':>11s} {'f_rad/f_tan':>11s} {'regime':>9s}")
for d in [5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0]:
    r = measure(4, d, 9.0, 0.5, 0.5)
    if r:
        print(f"{d:6.1f} {r['f_rad']:8.4f} {r['f_tan']:8.4f} {r['f_rot']:8.4f} {r['f_rad']/r['f_rot']:11.3f} {r['ratio']:11.3f} {r['regime']:>9s}")
    else:
        print(f"{d:6.1f} {'FAIL':>8s}")

print()

# 3. mu scan (k1=k2=0.5, N=4, d=5)
print("3. mu scan (N=4, k1=k2=0.5, d=5):")
print(f"{'mu':>6s} {'f_rad':>8s} {'f_tan':>8s} {'f_rot':>8s} {'f_rad/f_rot':>11s} {'f_rad/f_tan':>11s} {'regime':>9s}")
for mu in [5.5, 6.0, 6.5, 7.0, 9.0]:
    r = measure(4, 5.0, mu, 0.5, 0.5)
    if r:
        print(f"{mu:6.1f} {r['f_rad']:8.4f} {r['f_tan']:8.4f} {r['f_rot']:8.4f} {r['f_rad']/r['f_rot']:11.3f} {r['ratio']:11.3f} {r['regime']:>9s}")
    else:
        print(f"{mu:6.1f} {'FAIL':>8s}")

print()

# 4. k2 scan in locked regime (k1=0.75, N=4, d=5, mu=9)
print("4. k2 scan (N=4, k1=0.75, d=5, mu=9):")
print(f"{'k2':>6s} {'f_rad':>8s} {'f_tan':>8s} {'f_rot':>8s} {'f_rad/f_rot':>11s} {'f_rad/f_tan':>11s} {'regime':>9s}")
for k2 in [0.3, 0.4, 0.5, 0.6, 0.7]:
    r = measure(4, 5.0, 9.0, 0.75, k2)
    if r:
        print(f"{k2:6.2f} {r['f_rad']:8.4f} {r['f_tan']:8.4f} {r['f_rot']:8.4f} {r['f_rad']/r['f_rot']:11.3f} {r['ratio']:11.3f} {r['regime']:>9s}")
    else:
        print(f"{k2:6.2f} {'FAIL':>8s}")

print()
print("Done.")