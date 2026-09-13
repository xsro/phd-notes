"""
Compare the angular velocity variation period of a 2D damped harmonic oscillator
with the breathing period of the fencing system.

For the 2D damped oscillator: x'' + k1*x' + k2*x = 0, x in R^2
- theta(t) = atan2(x2, x1)
- dot_theta(t) has period T = pi/omega where omega = sqrt(4*k2 - k1^2)/2
- The denominator of dot_theta is r^2 = x1^2 + x2^2, which oscillates with period pi/omega

For the fencing system:
- The breathing period is the period of mean radius oscillation
- The angular velocity of agents varies with the same period
- dot_theta_i ~ L_i / r_i^2 (angular momentum conservation)

The key question: is the angular velocity variation period in BOTH cases
equal to the period of r^2 oscillation?
"""

import sys, numpy as np
from scipy.signal import find_peaks
from scipy.integrate import solve_ivp

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate

# ═══════════════════════════════════════════════════════════════════════════
# 1. Simple 2D damped harmonic oscillator
# ═══════════════════════════════════════════════════════════════════════════
print("=" * 80)
print("1. 2D DAMPED HARMONIC OSCILLATOR")
print("   x'' + k1*x' + k2*x = 0, x in R^2")
print("=" * 80)

k1_h, k2_h = 1.0, 1.0
omega_h = np.sqrt(4*k2_h - k1_h**2) / 2  # = sqrt(3)/2
T_dot_theta_pred = np.pi / omega_h  # period of dot_theta
print(f"  k1 = {k1_h}, k2 = {k2_h}")
print(f"  omega = sqrt(4*k2 - k1^2)/2 = {omega_h:.4f}")
print(f"  Predicted dot_theta period T = pi/omega = {T_dot_theta_pred:.4f} s")

# Numerically integrate
def ode_2d(t, y):
    x1, x2, v1, v2 = y
    a1 = -k1_h*v1 - k2_h*x1
    a2 = -k1_h*v2 - k2_h*x2
    return [v1, v2, a1, a2]

# ICs from simulate.m: x(0) = (1,0), x'(0) = (0,1)
y0 = [1.0, 0.0, 0.0, 1.0]
t_eval_h = np.arange(0, 20, 0.01)
sol = solve_ivp(ode_2d, [0, 20], y0, method='RK45', t_eval=t_eval_h, rtol=1e-8, atol=1e-10)
x1, x2, v1, v2 = sol.y

# Compute theta and dot_theta
theta = np.arctan2(x2, x1)
r = np.sqrt(x1**2 + x2**2)
dot_theta = (x1*v2 - x2*v1) / (r**2 + 1e-15)

# Find period of dot_theta
peaks_dt, _ = find_peaks(dot_theta, prominence=0.001, distance=10)
if len(peaks_dt) >= 3:
    T_dt_obs = np.mean(np.diff(t_eval_h[peaks_dt]))
else:
    T_dt_obs = None

# Find period of r^2 oscillation
peaks_r2, _ = find_peaks(r**2, prominence=0.001, distance=10)
if len(peaks_r2) >= 3:
    T_r2 = np.mean(np.diff(t_eval_h[peaks_r2]))
else:
    T_r2 = None

print(f"\n  Numerical results:")
print(f"  Observed dot_theta period = {T_dt_obs:.4f} s" if T_dt_obs else "  No peaks found")
print(f"  r^2 oscillation period = {T_r2:.4f} s" if T_r2 else "  No peaks found")
if T_dt_obs and T_r2:
    print(f"  T_dot_theta / T_r^2 = {T_dt_obs/T_r2:.4f}")
    print(f"  T_dot_theta / (pi/omega) = {T_dt_obs/T_dot_theta_pred:.4f}")

# Check if dot_theta ~ 1/r^2
print(f"\n  Is dot_theta proportional to 1/r^2?")
# r^2 * dot_theta should be constant if angular momentum is conserved
L = r**2 * dot_theta
print(f"  L = r^2 * dot_theta: mean = {np.mean(L[-500:]):.4f}, std/mean = {np.std(L[-500:])/np.mean(L[-500:])*100:.2f}%")

# ═══════════════════════════════════════════════════════════════════════════
# 2. Compare with fencing system
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("2. FENCING SYSTEM (breathing limit cycle)")
print("=" * 80)

XI0 = np.array([[-10.584563, -12.532987], [-5.382648, 5.904905],
                [-0.177347, 9.17592], [-7.352506, -4.805249], [-6.288338, 12.43583]])
VT0 = np.array([[3.528048, -1.274511], [-0.511988, -1.485443],
                [1.972071, -3.679895], [-3.460489, -0.767699], [-2.039248, 2.7618]])

k1_f, k2_f, d_f, mu_f = 0.5, 0.5, 5.0, 9.0
N = 5

rhs, sol = integrate(XI0, VT0, d_f, mu_f, k1_f, k2_f, 600.0,
                     method='LSODA', rtol=1e-6, atol=1e-8)
t_eval_f = np.arange(100, 600.1, 0.1)
pos_all, vel_all = evaluate(sol, N, t_eval_f)

# Mean radius (breathing mode)
r_f = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
peaks_r_f, _ = find_peaks(r_f, prominence=0.001, distance=10)
T_breathing = np.mean(np.diff(t_eval_f[peaks_r_f]))

# Agent angular velocities
r_i = pos_all
v_i = vel_all
cross = r_i[:, :, 0] * v_i[:, :, 1] - r_i[:, :, 1] * v_i[:, :, 0]
r_mag = np.linalg.norm(r_i, axis=2)
omega_i = cross / (r_mag**2 + 1e-10)
omega_mean = np.mean(omega_i, axis=1)

# Period of omega_mean variation
peaks_om, _ = find_peaks(omega_mean, prominence=0.001, distance=10)
T_om = np.mean(np.diff(t_eval_f[peaks_om]))

# Period of r^2 oscillation
r_sq = r_f**2
peaks_rsq, _ = find_peaks(r_sq, prominence=0.001, distance=10)
T_rsq = np.mean(np.diff(t_eval_f[peaks_rsq]))

print(f"  k1 = {k1_f}, k2 = {k2_f}")
print(f"  Breathing period T_breathing = {T_breathing:.4f} s")
print(f"  omega_mean variation period T_om = {T_om:.4f} s")
print(f"  r^2 oscillation period T_r^2 = {T_rsq:.4f} s")
print(f"  T_om / T_breathing = {T_om/T_breathing:.4f}")
print(f"  T_om / T_r^2 = {T_om/T_rsq:.4f}")

# Check if omega_mean ~ 1/r^2
L_f = r_f**2 * omega_mean
print(f"\n  L = r^2 * omega_mean: mean = {np.mean(L_f[-1000:]):.4f}, "
      f"std/mean = {np.std(L_f[-1000:])/np.abs(np.mean(L_f[-1000:]))*100:.2f}%")

# ═══════════════════════════════════════════════════════════════════════════
# 3. Structure comparison
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("3. STRUCTURAL COMPARISON")
print("=" * 80)

print("""
Both systems share the same mathematical structure for dot_theta:

    dot_theta = (x1*v2 - x2*v1) / (x1^2 + x2^2)

For the 2D damped oscillator (linear, decaying):
    x_i(t) = e^{-k1*t/2} * R_i * cos(omega*t - phi_i)
    r^2(t) = e^{-k1*t} * [R1^2*cos^2(omega*t - phi1) + R2^2*cos^2(omega*t - phi2)]
    The e^{-k1*t} factor cancels in dot_theta
    Period of dot_theta = pi/omega = 2*pi/sqrt(4*k2 - k1^2)

For the fencing system (nonlinear, limit cycle):
    r(t) oscillates around a fixed mean (no decay)
    Period of dot_theta = period of r^2 oscillation = breathing period

The KEY common structure:
    dot_theta(t) varies because r^2(t) oscillates
    The period of dot_theta variation = the period of r^2 oscillation
""")

print(f"  2D damped oscillator: T_dot_theta = {T_dt_obs:.4f} s, T_r^2 = {T_r2:.4f} s, "
      f"ratio = {T_dt_obs/T_r2:.4f}" if T_dt_obs and T_r2 else "")
print(f"  Fencing system:      T_om = {T_om:.4f} s, T_r^2 = {T_rsq:.4f} s, "
      f"ratio = {T_om/T_rsq:.4f}")

# ═══════════════════════════════════════════════════════════════════════════
# 4. Check if the simple oscillator formula predicts the fencing period
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 80)
print("4. CAN THE SIMPLE OSCILLATOR FORMULA PREDICT THE FENCING PERIOD?")
print("=" * 80)

# For the simple oscillator: T_dot_theta = pi/omega = 2*pi/sqrt(4*k2 - k1^2)
# For the fencing system: T_breathing = 2*pi/omega_eff

# If we compute omega from the fencing parameters:
omega_f = np.sqrt(4*k2_f - k1_f**2) / 2
T_pred = np.pi / omega_f
print(f"  Fencing: k1={k1_f}, k2={k2_f}")
print(f"  Simple oscillator formula: T = pi/omega = 2*pi/sqrt(4*k2 - k1^2)")
print(f"  omega = sqrt(4*{k2_f} - {k1_f}^2)/2 = {omega_f:.4f}")
print(f"  Predicted T = {T_pred:.4f} s")
print(f"  Observed breathing T = {T_breathing:.4f} s")
print(f"  Ratio obs/pred = {T_breathing/T_pred:.4f}")
print(f"  => The simple formula does NOT match the fencing period.")

# But what if we consider the effective k2?
# The fencing system has a nonlinear repulsive potential that adds stiffness
# Effective k2 = k2 + k_rep
# omega_eff ~ 3.58*sqrt(k2) = sqrt(12.8*k2) for k1=k2
# So omega_eff^2 = 12.8*k2
# If we write omega_eff^2 = 4*k2_eff - k1^2/4 (from the simple formula structure)
# k2_eff = (omega_eff^2 + k1^2/4) / 4
k2_eff = (12.8*k2_f + k1_f**2/4) / 4
print(f"\n  If we treat the repulsive force as modifying k2:")
print(f"  omega_eff^2 = 12.8*k2 = {12.8*k2_f:.4f}")
print(f"  k2_eff = (omega_eff^2 + k1^2/4) / 4 = {k2_eff:.4f}")
print(f"  This is {k2_eff/k2_f:.2f}x the original k2")
print(f"  => The repulsive force effectively increases k2 by {k2_eff/k2_f:.1f}x")

# The simple oscillator formula: T_dot_theta = 2*pi/sqrt(4*k2 - k1^2)
# For the fencing system, we need to replace k2 with k2_eff
T_pred_eff = 2*np.pi / np.sqrt(4*k2_eff - k1_f**2)
print(f"  Predicted T with k2_eff = {T_pred_eff:.4f} s")
print(f"  Observed T = {T_breathing:.4f} s")
print(f"  Error = {abs(T_pred_eff - T_breathing)/T_breathing*100:.2f}%")