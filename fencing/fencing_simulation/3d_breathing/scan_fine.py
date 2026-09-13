"""
Fine scan for C_rep distribution: more data points, longer T_MAX for finer resolution.
Uses inline RHS for speed (same as scan_3d_final.py).
"""

import sys
import numpy as np
from scipy.integrate import solve_ivp

# ── Inline RHS (same as scan_3d_final.py) ───────────────────────────────────

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

    pd = np.array([np.linalg.norm(x[i]-x[j],axis=0)
                   for i in range(N) for j in range(i+1,N)])
    mask = t > t[-1]*0.5
    pd_ss = pd[:,mask]
    T_ss = len(t[mask])

    freqs = np.fft.rfftfreq(T_ss, d=DT)
    ap = np.zeros(len(freqs))
    for p in pd_ss:
        p = p - np.mean(p)
        ap += np.abs(np.fft.rfft(p))**2
    ap[0] = 0
    idx = np.argmax(ap)
    f = freqs[idx]
    pr = ap[idx]/(np.sum(ap)+1e-20)

    xi = x - xt.reshape(1,3,-1)
    fr = xi[:,:,-1]
    c = fr - np.mean(fr,axis=0)
    _,S,_ = np.linalg.svd(c, full_matrices=False)
    sig = S[2]/S[0] if len(S)>=3 else 1.0

    return {
        'freq': f, 'period': 1.0/f if f>0.001 else None,
        'peak_ratio': pr, 'sigma': sig,
        'T_rot': 2*np.pi/np.sqrt(k2)
    }

def gen_xi0(N, d, seed=0):
    np.random.seed(seed)
    return np.random.randn(N,3)*10

# ── Fine scans ──────────────────────────────────────────────────────────────

results = {}

print("=== Fine k2 scan (k1=0.5, N=4, d=5, mu=9) ===")
k2_fine = np.arange(0.25, 0.75, 0.05)
for k2 in k2_fine:
    r = measure(4, 5.0, 9.0, 0.5, k2, gen_xi0(4, 5.0, 0), T_MAX=120.0, DT=0.05)
    if r:
        Crep = k2 * ((r['T_rot']/r['period'])**2 - 1)
        results.setdefault('k2', []).append((k2, r['period'], r['T_rot'], Crep/k2, r['peak_ratio'], r['sigma']))
        print(f"  k2={k2:.2f}  T={r['period']:.3f}s  Trot={r['T_rot']:.3f}s  C_rep/k2={Crep/k2:.3f}  peak={r['peak_ratio']:.3f}  sigma={r['sigma']:.3f}")

print("\n=== Fine k1 scan (k2=0.5, N=4, d=5, mu=9) ===")
k1_fine = np.arange(0.15, 0.75, 0.05)
for k1 in k1_fine:
    r = measure(4, 5.0, 9.0, k1, 0.5, gen_xi0(4, 5.0, 0), T_MAX=120.0, DT=0.05)
    if r:
        Crep = 0.5 * ((r['T_rot']/r['period'])**2 - 1)
        results.setdefault('k1', []).append((k1, r['period'], r['T_rot'], Crep/0.5, r['peak_ratio'], r['sigma']))
        print(f"  k1={k1:.2f}  T={r['period']:.3f}s  Trot={r['T_rot']:.3f}s  C_rep/k2={Crep/0.5:.3f}  peak={r['peak_ratio']:.3f}  sigma={r['sigma']:.3f}")

print("\n=== Fine d_col scan (k1=k2=0.5, N=4, mu=9) ===")
d_fine = np.arange(3.0, 7.5, 0.5)
for d in d_fine:
    r = measure(4, d, 9.0, 0.5, 0.5, gen_xi0(4, d, 0), T_MAX=120.0, DT=0.05)
    if r:
        Crep = 0.5 * ((r['T_rot']/r['period'])**2 - 1)
        results.setdefault('d', []).append((d, r['period'], r['T_rot'], Crep/0.5, r['peak_ratio'], r['sigma']))
        print(f"  d={d:.1f}  T={r['period']:.3f}s  Trot={r['T_rot']:.3f}s  C_rep/k2={Crep/0.5:.3f}  peak={r['peak_ratio']:.3f}  sigma={r['sigma']:.3f}")

print("\n=== Fine mu scan (k1=k2=0.5, N=4, d=5) ===")
mu_fine = np.arange(6.0, 12.0, 0.5)
for mu in mu_fine:
    r = measure(4, 5.0, mu, 0.5, 0.5, gen_xi0(4, 5.0, 0), T_MAX=120.0, DT=0.05)
    if r:
        Crep = 0.5 * ((r['T_rot']/r['period'])**2 - 1)
        results.setdefault('mu', []).append((mu, r['period'], r['T_rot'], Crep/0.5, r['peak_ratio'], r['sigma']))
        print(f"  mu={mu:.1f}  T={r['period']:.3f}s  Trot={r['T_rot']:.3f}s  C_rep/k2={Crep/0.5:.3f}  peak={r['peak_ratio']:.3f}  sigma={r['sigma']:.3f}")

print("\n=== N scan (k1=k2=0.5, d=5, mu=9) ===")
for n in [3, 4, 5, 6]:
    r = measure(n, 5.0, 9.0, 0.5, 0.5, gen_xi0(n, 5.0, 0), T_MAX=120.0, DT=0.05)
    if r:
        Crep = 0.5 * ((r['T_rot']/r['period'])**2 - 1)
        results.setdefault('N', []).append((n, r['period'], r['T_rot'], Crep/0.5, r['peak_ratio'], r['sigma']))
        print(f"  N={n}  T={r['period']:.3f}s  Trot={r['T_rot']:.3f}s  C_rep/k2={Crep/0.5:.3f}  peak={r['peak_ratio']:.3f}  sigma={r['sigma']:.3f}")

print("\n=== Seed scan (N=4, k1=k2=0.5, d=5, mu=9) ===")
for seed in range(10):
    r = measure(4, 5.0, 9.0, 0.5, 0.5, gen_xi0(4, 5.0, seed), T_MAX=120.0, DT=0.05)
    if r:
        Crep = 0.5 * ((r['T_rot']/r['period'])**2 - 1)
        results.setdefault('seed', []).append((seed, r['period'], r['T_rot'], Crep/0.5, r['peak_ratio'], r['sigma']))
        print(f"  seed={seed}  T={r['period']:.3f}s  Trot={r['T_rot']:.3f}s  C_rep/k2={Crep/0.5:.3f}  peak={r['peak_ratio']:.3f}  sigma={r['sigma']:.3f}")

# Save results
np.savez('/home/orangepi/repo/phd-notes/fencing/fencing_simulation/3d_breathing/data/crep_fine.npz',
         **{k: np.array(v) for k, v in results.items()})
print("\nSaved to data/crep_fine.npz")