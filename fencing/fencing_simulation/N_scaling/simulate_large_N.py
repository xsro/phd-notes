#!/usr/bin/env python3
"""
Simulate N=25, 30, 35 for the 3D fencing controller.
Appends results to n_scaling_results.json.
"""
import numpy as np
from scipy.integrate import solve_ivp
import json
import os
import warnings
warnings.filterwarnings('ignore')

V0 = np.array([1.0, 0.0, 0.0])
T_MAX = 150.0
DT = 0.02
T_STEADY = 80.0
D_COL = 5.0
K1 = 0.5
K2 = 0.5
SEED = 0

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

def gen_xi0(N, d, mu, seed=SEED):
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

def compute_mu(N, base_mu=9.0, base_N=6):
    return max(base_mu * np.sqrt(N/base_N), base_mu)

def simulate(N):
    mu = compute_mu(N)
    print(f"  N={N}, mu={mu:.1f}...", end=" ", flush=True)

    xi0 = gen_xi0(N, D_COL, mu)
    rhs = make_rhs(N, D_COL, mu, K1, K2)
    y0 = np.zeros(6*N)
    y0[:3*N] = xi0.ravel()
    t_eval = np.arange(0, T_MAX+DT, DT)

    try:
        sol = solve_ivp(rhs, [0, T_MAX], y0, method='LSODA',
                        t_eval=t_eval, rtol=1e-8, atol=1e-10, max_step=DT)
    except Exception as e:
        print(f"FAILED: {e}")
        return None

    if not sol.success:
        print(f"FAILED: {sol.message}")
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
    v_self = np.linalg.norm(v, axis=1)

    mask = t > T_STEADY
    if np.sum(mask) < 100:
        mask = t > t[-1]*0.5

    r_ss = r[:,mask]
    v_tan_ss = v_tan_mag[:,mask]
    v_self_ss = v_self[:,mask]

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
    f_self = find_peak(v_self_ss)

    pd = np.array([np.linalg.norm(x[i]-x[j],axis=0) for i in range(N) for j in range(i+1,N)])
    pd_ss = pd[:,mask]
    f_pd = find_peak(pd_ss)

    xi_final = x[:,:,-1] - xt.reshape(1,3,-1)[:,:,-1]
    centered = xi_final - np.mean(xi_final, axis=0)
    _,S,_ = np.linalg.svd(centered, full_matrices=False)
    sigma = S[2]/S[0] if len(S)>=3 else 1.0

    min_pairwise = np.min(pd_ss)
    f_rot = np.sqrt(K2)/(2*np.pi)
    T_rot = 2*np.pi/np.sqrt(K2)

    if f_rad > 0.001 and f_tan > 0.001:
        ratio = f_rad/f_tan
        if abs(ratio-1.0) < 0.15:
            regime = "LOCKED"
        elif ratio > 1.3:
            regime = "BREATHING"
        else:
            regime = "OTHER"
    else:
        regime = "ANOMALOUS"

    result = {
        'N': N, 'd': D_COL, 'mu': mu, 'k1': K1, 'k2': K2,
        'f_rad': f_rad, 'T_rad': 1.0/f_rad if f_rad > 0.001 else None,
        'f_tan': f_tan, 'T_tan': 1.0/f_tan if f_tan > 0.001 else None,
        'f_self': f_self, 'T_self': 1.0/f_self if f_self > 0.001 else None,
        'f_pd': f_pd, 'T_pd': 1.0/f_pd if f_pd > 0.001 else None,
        'f_rot': f_rot, 'T_rot': T_rot,
        'f_rad_over_ftan': f_rad/f_tan if f_tan > 0.001 else 0,
        'f_self_over_frad': f_self/f_rad if f_rad > 0.001 else 0,
        'f_self_over_ftan': f_self/f_tan if f_tan > 0.001 else 0,
        'sigma_ratio': sigma,
        'min_pairwise': min_pairwise,
        'collision_risk': str(min_pairwise < D_COL * 1.1),
        'regime': regime,
        'freq_resolution': freqs[1]-freqs[0],
    }

    print(f"f_rad={f_rad:.4f}, f_tan={f_tan:.4f}, f_self={f_self:.4f}, "
          f"f_rad/f_tan={result['f_rad_over_ftan']:.3f}, sigma={sigma:.3f}, regime={regime}")
    return result


def main():
    json_path = os.path.join(os.path.dirname(__file__), 'n_scaling_results.json')

    # Load existing results
    if os.path.exists(json_path):
        with open(json_path) as f:
            results = json.load(f)
        print(f"Loaded {len(results)} existing results")
    else:
        results = []

    # Simulate N=25, 30, 35
    for N in [25, 30, 35]:
        # Skip if already done
        if any(r['N'] == N for r in results):
            print(f"N={N} already done, skipping")
            continue
        r = simulate(N)
        if r:
            results.append(r)
            # Save incrementally
            with open(json_path, 'w') as f:
                json.dump(results, f, indent=2, default=str)
            print(f"  Saved to {json_path}")

    # Print summary
    print("\n" + "="*90)
    print("UPDATED SUMMARY: ALL N VALUES")
    print("="*90)
    print(f"{'N':>4s} {'mu':>6s} {'f_rad':>9s} {'f_tan':>9s} {'f_self':>9s} "
          f"{'f_pd':>9s} {'f_rad/f_tan':>12s} {'f_rad/f_rot':>11s} {'sigma':>8s} {'regime':>12s}")
    print("-"*95)
    for r in sorted(results, key=lambda x: x['N']):
        N = r['N']
        fr = r['f_rad']
        ft = r['f_tan']
        fs = r['f_self']
        fp = r['f_pd']
        frot = r['f_rot']
        ratio = r['f_rad_over_ftan']
        sigma = r['sigma_ratio']
        regime = r.get('regime', '?')
        print(f"{N:4d} {r['mu']:6.1f} {fr:9.4f} {ft:9.4f} {fs:9.4f} {fp:9.4f} "
              f"{ratio:12.3f} {fr/frot:11.3f} {sigma:8.3f} {regime:>12s}")

    print(f"\nTotal: {len(results)} results saved to {json_path}")


if __name__ == '__main__':
    main()