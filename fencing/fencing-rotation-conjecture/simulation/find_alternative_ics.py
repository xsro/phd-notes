"""
Phase 2: Compare period-parameter relationships for 3 alternative ICs.
"""

import sys, numpy as np, time
from scipy.signal import find_peaks

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate

XI0_REF = np.array([[-10.584563, -12.532987],
                    [-5.382648,   5.904905],
                    [-0.177347,   9.17592 ],
                    [-7.352506,  -4.805249],
                    [-6.288338,  12.43583 ]])
VT0_REF = np.array([[ 3.528048, -1.274511],
                    [-0.511988, -1.485443],
                    [ 1.972071, -3.679895],
                    [-3.460489, -0.767699],
                    [-2.039248,  2.7618  ]])


def quick_test(x0, v0, k1=0.5, k2=0.5, d=5.0, mu=9.0, t_end=150.0):
    """Quick test with short integration."""
    rhs, sol = integrate(x0, v0, d, mu, k1, k2, t_end,
                         method='LSODA', rtol=1e-5, atol=1e-7)
    t_eval = np.arange(10, t_end, 0.2)
    pos_all, vel_all = evaluate(sol, 5, t_eval)
    r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    r_avg = np.mean(r)
    if r_avg == 0:
        return None
    amp = (np.max(r) - np.min(r)) / r_avg * 100
    peaks, _ = find_peaks(r, prominence=0.001, distance=10)
    if len(peaks) < 3:
        return None
    periods = np.diff(t_eval[peaks])
    T = np.mean(periods)
    T_std = np.std(periods)
    omega = 2 * np.pi / T
    s1 = 2 * r * np.sin(np.pi / 5)
    gap = np.mean(s1) - d
    if amp < 0.5 and abs(r_avg - 4.64) < 0.1:
        return None  # rotating equilibrium
    if T_std / T > 0.3:
        return None
    return {'T': T, 'T_std': T_std, 'omega': omega,
            'omega2': omega**2, 'ratio': omega / np.sqrt(k2),
            'r_avg': r_avg, 'amp': amp, 'gap': gap,
            'x0': x0, 'v0': v0}


# Define the 3 working ICs
np.random.seed(42)
x0_1 = XI0_REF + 0.3 * np.random.randn(5, 2)
v0_1 = VT0_REF + 0.3 * np.random.randn(5, 2)

np.random.seed(7)
x0_2 = XI0_REF + 0.5 * np.random.randn(5, 2)
v0_2 = VT0_REF + 0.5 * np.random.randn(5, 2)

np.random.seed(99)
angles = np.linspace(0, 2*np.pi, 5, endpoint=False)
x0_3 = 6.0 * np.column_stack([np.cos(angles), np.sin(angles)])
x0_3 += 2.0 * np.random.randn(5, 2)
v0_3 = np.sqrt(0.5) * np.column_stack([-x0_3[:, 1], x0_3[:, 0]])
v0_3 += 0.5 * np.random.randn(5, 2)

working_ics = [
    ('IC1_case3_pert0.3', x0_1, v0_1),
    ('IC2_case3_pert0.5', x0_2, v0_2),
    ('IC3_pentagon_pert2.0', x0_3, v0_3),
]

# Phase 1: Baseline
print("=" * 60)
print("Phase 1: Baseline (k1=0.5, k2=0.5)")
print("=" * 60)
baselines = {}
for name, x0, v0 in working_ics:
    res = quick_test(x0, v0, k1=0.5, k2=0.5)
    baselines[name] = res
    if res:
        print(f"  {name}: T={res['T']:.4f}, ratio={res['ratio']:.4f}, amp={res['amp']:.2f}%, r_avg={res['r_avg']:.4f}")

# Phase 2: Parameter scan
print("\n" + "=" * 60)
print("Phase 2: Parameter scan")
print("=" * 60)

test_cases = [(0.3, 0.5), (0.5, 0.3), (0.5, 0.7), (0.6, 0.5)]

for name, x0, v0 in working_ics:
    baseline = baselines[name]
    if baseline is None:
        continue
    print(f"\n--- {name} ---")
    print(f"  Baseline: T={baseline['T']:.4f}, ratio={baseline['ratio']:.4f}")
    for k1, k2 in test_cases:
        print(f"    k1={k1:.1f},k2={k2:.1f}...", end=' ', flush=True)
        res = quick_test(x0, v0, k1=k1, k2=k2)
        if res is not None:
            print(f"T={res['T']:.4f}, ratio={res['ratio']:.4f}, amp={res['amp']:.2f}%, gap={res['gap']:.3f}")
        else:
            print("NO LC")

# Phase 3: Compare with reference formula
print("\n" + "=" * 60)
print("Phase 3: Comparison with reference formula")
print("  Reference: ratio = 3.78 - 0.81*k1 + 0.39*k2")
print("=" * 60)

for name, x0, v0 in working_ics:
    baseline = baselines[name]
    if baseline is None:
        continue
    print(f"\n{name}:")
    # Test a few more points
    for k1, k2 in [(0.4, 0.5), (0.5, 0.4), (0.5, 0.6), (0.6, 0.5)]:
        res = quick_test(x0, v0, k1=k1, k2=k2)
        if res is not None:
            pred = 3.78 - 0.81*k1 + 0.39*k2
            err = (pred - res['ratio'])/res['ratio']*100
            print(f"  k1={k1:.1f},k2={k2:.1f}: obs={res['ratio']:.4f}, pred={pred:.4f}, err={err:+.1f}%")
        else:
            print(f"  k1={k1:.1f},k2={k2:.1f}: NO LC")

print("\nDone.")