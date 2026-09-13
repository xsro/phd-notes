"""
Derivation of an approximate analytical expression for the
breathing limit cycle's effective frequency.
"""

import sys, numpy as np
from scipy.signal import find_peaks

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate, alpha

XI0 = np.array([[-10.584563, -12.532987],
                [-5.382648,   5.904905],
                [-0.177347,   9.17592 ],
                [-7.352506,  -4.805249],
                [-6.288338,  12.43583 ]])
VT0 = np.array([[ 3.528048, -1.274511],
                [-0.511988, -1.485443],
                [ 1.972071, -3.679895],
                [-3.460489, -0.767699],
                [-2.039248,  2.7618  ]])


def compute_lc(k1, k2=0.5, d=5.0, mu=9.0, tf=1500.0):
    rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, tf,
                         method="BDF", rtol=1e-8, atol=1e-10)
    t_eval = np.arange(500, tf + 0.1, 0.2)
    pos_all, _ = evaluate(sol, 5, t_eval)
    mean_r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    r_avg = np.mean(mean_r)
    r_min, r_max = np.min(mean_r), np.max(mean_r)
    amp_frac = (r_max - r_min) / r_avg
    peaks, _ = find_peaks(mean_r, prominence=0.001, distance=10)
    if len(peaks) < 3:
        return None, None, r_avg, amp_frac, r_min, r_max
    periods = np.diff(t_eval[peaks])
    T = np.mean(periods)
    omega_eff = 2*np.pi / T
    ratio = omega_eff / np.sqrt(k2)
    return T, ratio, r_avg, amp_frac, r_min, r_max


# ── Collect all data points ──────────────────────────────────────────────────
print("=" * 75)
print("DATA COLLECTION  (k2=0.5, d=5.0, mu=9.0, N=5)")
print("=" * 75)
header = f"{'k1':>6}  {'T':>8}  {'we/sqrt(k2)':>11}  {'r_avg':>8}  {'amp(%)':>8}  {'r_min':>8}  {'r_max':>8}"
print(header)

data = []
for k1 in [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60]:
    T, ratio, r_avg, amp, r_min, r_max = compute_lc(k1)
    if T is None:
        print(f"{k1:>6.2f}  {'---':>8}  {'---':>11}  {r_avg:>8.4f}  {amp*100:>8.2f}  {'---':>8}  {'---':>8}  (no LC)")
    else:
        data.append((k1, T, ratio, r_avg, amp, r_min, r_max))
        print(f"{k1:>6.2f}  {T:>8.4f}  {ratio:>11.4f}  {r_avg:>8.4f}  {amp*100:>8.2f}  {r_min:>8.4f}  {r_max:>8.4f}")

# ── Geometry constants for N=5 regular pentagon ──────────────────────────────
N = 5
d_val, mu_val = 5.0, 9.0
sin_pi_N = np.sin(np.pi/N)
sin_2pi_N = np.sin(2*np.pi/N)
c1 = (1 - np.cos(2*np.pi/N)) / (2 * sin_pi_N)
c2 = (1 - np.cos(4*np.pi/N)) / (2 * sin_2pi_N)

print(f"\nN={N} regular pentagon geometry:")
print(f"  sin(pi/5) = {sin_pi_N:.4f}")
print(f"  sin(2pi/5) = {sin_2pi_N:.4f}")
print(f"  c1 (nearest-neighbor radial coeff) = {c1:.4f}")
print(f"  c2 (next-nearest radial coeff) = {c2:.4f}")

# ── Stiffness analysis ──────────────────────────────────────────────────────
print('\n' + f"{'k1':>6}  {'r_avg':>8}  {'s1':>8}  {'s1-d':>8}  {'-alpha_p(s1)':>14}  {'-f_p(r)':>10}  {'we2(obs)':>10}  {'we2(lin)':>10}")
for k1, T, ratio, r_avg, amp, r_min, r_max in data:
    s1 = 2 * r_avg * sin_pi_N
    s2 = 2 * r_avg * sin_2pi_N
    a1p = -1.0/(s1-d_val)**2 if s1 <= mu_val else 0.0
    a2p = -1.0/(s2-d_val)**2 if s2 <= mu_val else 0.0
    fp = 4*a1p*sin_pi_N*c1 + 4*a2p*sin_2pi_N*c2
    omega2_lin = 0.5 - fp
    omega2_obs = (ratio * np.sqrt(0.5))**2
    print(f"{k1:>6.2f}  {r_avg:>8.4f}  {s1:>8.4f}  {s1-5.0:>8.4f}  {-a1p:>14.2f}  {-fp:>10.2f}  {omega2_obs:>10.4f}  {omega2_lin:>10.4f}")

# ── Fit C = omega_eff^2 - k2 ─────────────────────────────────────────────────
print("\n" + "=" * 75)
print("POWER-LAW FIT:  C = omega_eff^2 - k2 = a * k1^b")
print("=" * 75)
k1_vals = np.array([d[0] for d in data])
ratio_vals = np.array([d[2] for d in data])
omega2_vals = (ratio_vals * np.sqrt(0.5))**2
C_vals = omega2_vals - 0.5

log_k1 = np.log(k1_vals)
log_C = np.log(C_vals)
A = np.column_stack([np.ones_like(log_k1), log_k1])
coeffs = np.linalg.lstsq(A, log_C, rcond=None)[0]
log_a, b = coeffs
a = np.exp(log_a)

print(f"\n  C = {a:.4f} * k1^{b:.4f}")
print(f"  -> omega_eff^2 = k2 + {a:.4f} * k1^{b:.4f}")
print(f"  -> omega_eff / sqrt(k2) = sqrt(1 + {a:.4f}/0.5 * k1^{b:.4f})")

# Also linear fit
c_lin = np.mean(C_vals / k1_vals)
print(f"\n  Linear fit:  C = {c_lin:.4f} * k1")
print(f"  -> omega_eff^2 = k2 + {c_lin:.4f} * k1")

# ── Amplitude-stiffness relationship ─────────────────────────────────────────
print("\n" + "=" * 75)
print("AMPLITUDE-AVERAGED STIFFNESS")
print("=" * 75)
print(f"\n{'k1':>6}  {'amp(%)':>8}  {'we^2(obs)':>10}  {'we^2(avg)':>10}  {'we^2(pow)':>10}")
for k1, T, ratio, r_avg, amp, r_min, r_max in data:
    omega2_obs = (ratio * np.sqrt(0.5))**2
    # Average stiffness at r_min and r_max
    s1_min = 2*r_min*sin_pi_N
    s1_max = 2*r_max*sin_pi_N
    a1p_min = -1.0/(s1_min-d_val)**2 if s1_min > d_val and s1_min <= mu_val else 0.0
    a1p_max = -1.0/(s1_max-d_val)**2 if s1_max > d_val and s1_max <= mu_val else 0.0
    fp_min = 4*a1p_min*sin_pi_N*c1
    fp_max = 4*a1p_max*sin_pi_N*c1
    omega2_min = 0.5 - fp_min
    omega2_max = 0.5 - fp_max
    omega2_avg = (omega2_min + omega2_max) / 2
    omega2_pow = 0.5 + a * k1**b
    print(f"{k1:>6.2f}  {amp*100:>8.2f}  {omega2_obs:>10.4f}  {omega2_avg:>10.4f}  {omega2_pow:>10.4f}")

# ── Final formula ────────────────────────────────────────────────────────────
print("\n" + "=" * 75)
print("PROPOSED APPROXIMATE FORMULA")
print("=" * 75)
print('\n' + '=' * 75)
print('PROPOSED APPROXIMATE FORMULA')
print('=' * 75)
print(f'''
For the breathing limit cycle (N=5, k1 <= k2, k2=0.5):

    omega_eff^2  =  k2  +  {a:.4f} * k1^{b:.4f}

Equivalently:

    T  =  2*pi / sqrt(k2 + {a:.4f} * k1^{b:.4f})

Physical interpretation:
  - The term k2 comes from the observer integral (the "spring")
  - The term {a:.4f} * k1^{b:.4f} comes from the repulsive force
    gradient averaged over the oscillation cycle
  - The exponent b = {b:.4f} reflects that the amplitude A ~ 1/k1^alpha
    and the stiffness ~ 1/(s-d)^2 is a steep function of amplitude

The limit cycle exists only when k1 < k1_crit ~ 0.65 (for k2=0.5).
''')

print("Done.")