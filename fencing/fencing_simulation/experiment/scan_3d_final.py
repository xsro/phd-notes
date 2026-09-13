#!/usr/bin/env python3
"""
3D breathing parameter scan — optimized for speed.
N=4, T_MAX=60, DT=0.1 (~5-7s per run).
Uses np.random.randn(N,3)*10 for initial conditions (proven fast).
"""

import sys
import numpy as np
from pathlib import Path
from scipy.integrate import solve_ivp

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


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


def measure(N, d, mu, k1, k2, xi0, T_MAX=60.0, DT=0.1):
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
        'is_breathing': pr > 0.05,
    }


def fmt(r, k1, k2):
    if r is None: return "ERR"
    t = 'BR' if r['is_breathing'] else 'RO'
    T_str = f"{r['period']:.3f}" if r['period'] else "—"
    T_rot = 2*np.pi/np.sqrt(k2) if k2>0 else float('inf')
    ratio = r['period']/T_rot if r['period'] else 0
    return (f"k1={k1:.2f} k2={k2:.2f}: f={r['freq']:.4f} T={T_str} "
            f"ratio={ratio:.3f} peak={r['peak_ratio']:.4f} sigma={r['sigma']:.4f} {t}")


if __name__ == '__main__':
    # ============================================================
    N, d, mu = 4, 5.0, 9.0

    print("="*65)
    print("SCAN 1: k1-k2 grid (N=4, seed=0)")
    print("="*65)
    np.random.seed(0)
    xi0 = np.random.randn(N,3)*10
    for k1 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]:
        for k2 in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]:
            r = measure(N, d, mu, k1, k2, xi0)
            print(fmt(r, k1, k2))

    print()
    print("="*65)
    print("SCAN 2: d_col scan (N=4, k1=k2=0.5, mu=9.0)")
    print("="*65)
    for d_col in [3.0, 4.0, 5.0, 6.0, 7.0]:
        np.random.seed(0)
        xi0 = np.random.randn(N,3)*10
        r = measure(N, d_col, 9.0, 0.5, 0.5, xi0)
        print(fmt(r, 0.5, 0.5) + f"  d={d_col:.0f}")

    print()
    print("="*65)
    print("SCAN 3: mu scan (N=4, k1=k2=0.5, d=5.0)")
    print("="*65)
    for mu_val in [6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]:
        np.random.seed(0)
        xi0 = np.random.randn(N,3)*10
        r = measure(N, 5.0, mu_val, 0.5, 0.5, xi0)
        print(fmt(r, 0.5, 0.5) + f"  mu={mu_val:.0f}")

    print()
    print("="*65)
    print("SCAN 4: Seed scan (N=4, k1=k2=0.5, d=5.0, mu=9.0)")
    print("="*65)
    for seed in range(8):
        np.random.seed(seed)
        xi0 = np.random.randn(N,3)*10
        r = measure(N, 5.0, 9.0, 0.5, 0.5, xi0)
        print(fmt(r, 0.5, 0.5) + f"  seed={seed}")

    print()
    print("="*65)
    print("SCAN 5: N scan (k1=k2=0.5, d=5.0, mu=9.0)")
    print("="*65)
    for Nv in [3, 4, 5]:
        np.random.seed(0)
        xi0 = np.random.randn(Nv,3)*10
        r = measure(Nv, 5.0, 9.0, 0.5, 0.5, xi0)
        if r:
            t = 'BR' if r['is_breathing'] else 'RO'
            T_str = f"{r['period']:.3f}" if r['period'] else "—"
            T_rot = 2*np.pi/np.sqrt(0.5)
            print(f"N={Nv}: f={r['freq']:.4f} T={T_str} T_rot={T_rot:.3f} ratio={r['period']/T_rot:.3f} peak={r['peak_ratio']:.4f} sigma={r['sigma']:.4f} {t}")
        else:
            print(f"N={Nv}: ERR")

    print()
    print("="*65)
    print("SCAN 6: Wider k2 (N=4, k1=0.5, d=5.0, mu=9.0)")
    print("="*65)
    np.random.seed(0)
    xi0 = np.random.randn(N,3)*10
    for k2 in [0.05, 0.1, 0.15, 0.2, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.9]:
        r = measure(N, 5.0, 9.0, 0.5, k2, xi0)
        print(fmt(r, 0.5, k2))

    print()
    print("="*65)
    print("SCAN 7: Wider k1 (N=4, k2=0.5, d=5.0, mu=9.0)")
    print("="*65)
    np.random.seed(0)
    xi0 = np.random.randn(N,3)*10
    for k1 in [0.05, 0.1, 0.15, 0.7, 0.75, 0.8, 0.85, 0.9]:
        r = measure(N, 5.0, 9.0, k1, 0.5, xi0)
        print(fmt(r, k1, 0.5))

    print("\nDONE")