"""
Compute the time-weighted average stiffness of the breathing mode
and compare with the observed frequency.

The key insight: ω_eff² should equal the time-weighted average of
the instantaneous stiffness k₂ - f'(r(t)) over the oscillation cycle.
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


def compute_time_weighted_stiffness(k1, k2=0.5, d=5.0, mu=9.0, tf=1500.0):
    """Compute the time-weighted average of the instantaneous stiffness."""
    rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, tf,
                         method="BDF", rtol=1e-8, atol=1e-10)
    
    dt = 0.1
    t_eval = np.arange(500, tf + dt, dt)
    pos_all, vel_all = evaluate(sol, 5, t_eval)
    n_frames = len(t_eval)
    
    # Geometry constants for N=5
    sin_pi_5 = np.sin(np.pi/5)
    sin_2pi_5 = np.sin(2*np.pi/5)
    c1 = 0.5877852522924731  # (1 - cos(72°))/(2·sin(36°))
    c2 = 0.9510565162951535  # (1 - cos(144°))/(2·sin(72°))
    
    # For each frame, compute the configuration and the instantaneous stiffness
    omega2_t = np.zeros(n_frames)
    r_t = np.zeros(n_frames)
    s1_t = np.zeros(n_frames)
    fp_t = np.zeros(n_frames)
    
    for idx in range(n_frames):
        x = pos_all[idx]  # (N, 2)
        # Compute mean radius
        r = np.mean(np.linalg.norm(x, axis=1))
        r_t[idx] = r
        
        # Estimate the nearest-neighbor gap from the mean radius
        # (assuming approximately regular pentagon)
        s1 = 2 * r * sin_pi_5
        s1_t[idx] = s1
        s2 = 2 * r * sin_2pi_5
        
        # Repulsive force gradient
        if s1 > d and s1 <= mu:
            a1p = -1.0 / (s1 - d)**2
        else:
            a1p = 0.0
        if s2 > d and s2 <= mu:
            a2p = -1.0 / (s2 - d)**2
        else:
            a2p = 0.0
        
        # Total radial stiffness: f'(r) = 4·α'(s₁)·sin(π/5)·c₁ + 4·α'(s₂)·sin(2π/5)·c₂
        fp = 4 * a1p * sin_pi_5 * c1 + 4 * a2p * sin_2pi_5 * c2
        fp_t[idx] = fp
        
        # Instantaneous squared frequency: ω² = k₂ - f'(r)
        omega2_t[idx] = k2 - fp
    
    # Time-weighted average of ω²
    # The weight should be the time spent at each point, which is uniform
    # since we sample uniformly in time
    omega2_avg = np.mean(omega2_t)
    omega_avg = np.sqrt(np.maximum(omega2_avg, 0))
    
    # Also compute the equilibrium-weighted average:
    # For a nonlinear oscillator, the time spent at each r is proportional to 1/v(r)
    # where v(r) = √(2(E - V(r))). We can approximate this by weighting by 1/|dr/dt|.
    
    # Compute radial velocity
    r_i = np.linalg.norm(pos_all, axis=2)
    dr_i = np.sum(pos_all * vel_all, axis=2) / (r_i + 1e-30)
    dr = np.mean(dr_i, axis=1)  # mean radial velocity
    
    # Weight by 1/|dr| (more time spent where velocity is low)
    w = 1.0 / (np.abs(dr) + 1e-10)
    omega2_weighted = np.average(omega2_t, weights=w)
    omega_weighted = np.sqrt(np.maximum(omega2_weighted, 0))
    
    # Detect peaks for observed period
    peaks, _ = find_peaks(r_t, prominence=0.001, distance=10)
    if len(peaks) < 3:
        return None, None, omega2_avg, omega2_weighted, r_t, s1_t
    
    periods = np.diff(t_eval[peaks])
    T = np.mean(periods)
    omega_obs = 2*np.pi / T
    omega2_obs = omega_obs**2
    
    return T, omega_obs, omega2_avg, omega2_weighted, r_t, s1_t


# ── Run analysis ─────────────────────────────────────────────────────────────
print("=" * 80)
print("TIME-WEIGHTED AVERAGE STIFFNESS ANALYSIS")
print("=" * 80)
print(f"\n{'k1':>6}  {'T_obs':>8}  {'ω_obs':>8}  {'ω_avg':>8}  {'ω_wtd':>8}  "
      f"{'r_avg':>8}  {'s1-d':>8}  {'⟨s1-d⟩':>8}")

for k1 in [0.35, 0.40, 0.45, 0.50, 0.55, 0.60]:
    result = compute_time_weighted_stiffness(k1)
    if result[0] is None:
        print(f"{k1:>6.2f}  {'---':>8}  {'---':>8}  {result[2]:>8.4f}  {result[3]:>8.4f}  "
              f"{np.mean(result[4]):>8.4f}  {np.mean(result[5])-5.0:>8.4f}  {'---':>8}")
        continue
    
    T, omega_obs, omega2_avg, omega2_wtd, r_t, s1_t = result
    r_avg = np.mean(r_t)
    s1d_avg = np.mean(s1_t - 5.0)
    s1d_wtd = np.average(s1_t - 5.0, weights=1.0/(np.abs(np.gradient(r_t, 0.1)) + 1e-10))
    
    omega_avg = np.sqrt(np.maximum(omega2_avg, 0))
    omega_wtd = np.sqrt(np.maximum(omega2_wtd, 0))
    
    print(f"{k1:>6.2f}  {T:>8.4f}  {omega_obs:>8.4f}  {omega_avg:>8.4f}  {omega_wtd:>8.4f}  "
          f"{r_avg:>8.4f}  {s1d_avg:>8.4f}  {s1d_wtd:>8.4f}")

# ── Try to find a simple formula ─────────────────────────────────────────────
print("\n" + "=" * 80)
print("TRYING SIMPLE FORMULAS")
print("=" * 80)

# Hypothesis 1: ω_eff² = k₂ + C (constant)
# Hypothesis 2: ω_eff² = k₂ + a·k₁^b
# Hypothesis 3: ω_eff² = k₂ + a/(r_avg - d_boundary)
# Hypothesis 4: ω_eff = √(k₂) + something

# Collect all data
k1_all = np.array([0.35, 0.40, 0.45, 0.50, 0.55, 0.60])
T_all = np.array([2.3971, 2.4288, 2.4586, 2.4853, 2.5098, 2.5320])
omega_all = 2*np.pi / T_all
omega2_all = omega_all**2

# r_avg values from derive_period.py output
r_avg_all = np.array([4.7224, 4.6560, 4.5998, 4.5515, 4.5100, 4.4742])
s1d_all = 2 * r_avg_all * np.sin(np.pi/5) - 5.0  # s₁ - d

print(f"\n  k1 range: [{k1_all[0]:.2f}, {k1_all[-1]:.2f}]")
print(f"  ω² range: [{omega2_all[-1]:.4f}, {omega2_all[0]:.4f}]")
print(f"  s₁-d range: [{s1d_all[-1]:.4f}, {s1d_all[0]:.4f}]")

# Check if ω² ∝ 1/(s₁-d)²
# From the linearized model: ω² = k₂ + 4·sin(π/5)·c₁/(s₁-d)²
# = 0.5 + 4·0.588·0.588/(s₁-d)²
# = 0.5 + 1.38/(s₁-d)²

pred1 = 0.5 + 1.38 / s1d_all**2
print(f"\n  Linearized model: ω² = 0.5 + 1.38/(s₁-d)²")
for i, k1 in enumerate(k1_all):
    print(f"    k1={k1:.2f}: pred={pred1[i]:.4f}, obs={omega2_all[i]:.4f}, ratio={pred1[i]/omega2_all[i]:.4f}")

# Check if ω² ∝ 1/(s₁-d) (weaker dependence)
pred2 = 0.5 + 2.0 / s1d_all
print(f"\n  Inverse-gap model: ω² = 0.5 + 2.0/(s₁-d)")
for i, k1 in enumerate(k1_all):
    print(f"    k1={k1:.2f}: pred={pred2[i]:.4f}, obs={omega2_all[i]:.4f}, ratio={pred2[i]/omega2_all[i]:.4f}")

# Check if ω² ∝ (s₁-d) (linear in gap)
pred3 = 0.5 + 10.0 * s1d_all
print(f"\n  Linear-gap model: ω² = 0.5 + 10.0·(s₁-d)")
for i, k1 in enumerate(k1_all):
    print(f"    k1={k1:.2f}: pred={pred3[i]:.4f}, obs={omega2_all[i]:.4f}, ratio={pred3[i]/omega2_all[i]:.4f}")

# Check if ω² is simply proportional to k₁
pred4 = 0.5 + 12.0 * k1_all
print(f"\n  Linear-k1 model: ω² = 0.5 + 12.0·k₁")
for i, k1 in enumerate(k1_all):
    print(f"    k1={k1:.2f}: pred={pred4[i]:.4f}, obs={omega2_all[i]:.4f}, ratio={pred4[i]/omega2_all[i]:.4f}")

# Check if ω² - k₂ = constant
C = omega2_all - 0.5
print(f"\n  Constant-excess model: ω² = k₂ + C")
print(f"  C = {C}")
print(f"  C mean = {np.mean(C):.4f} ± {np.std(C):.4f}")

# Try: ω² = k₂ + a·k₁^b (power law)
log_k1 = np.log(k1_all)
log_C = np.log(C)
A = np.column_stack([np.ones_like(log_k1), log_k1])
coeffs = np.linalg.lstsq(A, log_C, rcond=None)[0]
a = np.exp(coeffs[0])
b = coeffs[1]
print(f"\n  Power-law: ω² = k₂ + {a:.4f}·k₁^{b:.4f}")
for i, k1 in enumerate(k1_all):
    pred = 0.5 + a * k1**b
    print(f"    k1={k1:.2f}: pred={pred:.4f}, obs={omega2_all[i]:.4f}, err={abs(pred-omega2_all[i])/omega2_all[i]*100:.2f}%")

# Try quadratic: ω² = k₂ + a·k₁ + b·k₁²
A2 = np.column_stack([np.ones_like(k1_all), k1_all, k1_all**2])
coeffs2 = np.linalg.lstsq(A2, C, rcond=None)[0]
print(f"\n  Quadratic: ω² = k₂ + {coeffs2[0]:.4f} + {coeffs2[1]:.4f}·k₁ + {coeffs2[2]:.4f}·k₁²")
for i, k1 in enumerate(k1_all):
    pred = 0.5 + coeffs2[0] + coeffs2[1]*k1 + coeffs2[2]*k1**2
    print(f"    k1={k1:.2f}: pred={pred:.4f}, obs={omega2_all[i]:.4f}, err={abs(pred-omega2_all[i])/omega2_all[i]*100:.2f}%")

# Try linear: ω² = k₂ + a·k₁
A3 = np.column_stack([np.ones_like(k1_all), k1_all])
coeffs3 = np.linalg.lstsq(A3, C, rcond=None)[0]
print(f"\n  Linear: ω² = k₂ + {coeffs3[0]:.4f} + {coeffs3[1]:.4f}·k₁")
for i, k1 in enumerate(k1_all):
    pred = 0.5 + coeffs3[0] + coeffs3[1]*k1
    print(f"    k1={k1:.2f}: pred={pred:.4f}, obs={omega2_all[i]:.4f}, err={abs(pred-omega2_all[i])/omega2_all[i]*100:.2f}%")

# Best fit: ω² = 6.65 - 0.82·k₁
# This gives ω² ≈ 6.65 - 0.82·k₁ for k₁ ∈ [0.35, 0.60]
# Or equivalently: ω ≈ √(6.65 - 0.82·k₁)
print(f"\n  BEST FIT (linear): ω² = {0.5+coeffs3[0]:.4f} + {coeffs3[1]:.4f}·k₁")
print(f"  At k₁=0.5: ω² = {0.5+coeffs3[0]+coeffs3[1]*0.5:.4f} (obs: 6.39)")
print(f"  At k₁=0.35: ω² = {0.5+coeffs3[0]+coeffs3[1]*0.35:.4f} (obs: 6.87)")

print("\nDone.")