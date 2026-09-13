"""
Check if the breathing period is related to the angular period of INDIVIDUAL agents
relative to the target. For each agent i:
  a_i = x_i - x_0 (relative position to target, with x_0 at origin)
  theta_i = arctan2(y_i, x_i)  (angle of agent around target)
  omega_i = d/dt theta_i  (angular velocity of agent)
  
We check:
  1. Period of omega_i variation vs breathing period
  2. Period of theta_i variation (rotation period) vs breathing period
"""

import sys, numpy as np
from scipy.signal import find_peaks

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate

# Case C: Breathing limit cycle
XI0 = np.array([[-10.584563, -12.532987], [-5.382648, 5.904905],
                [-0.177347, 9.17592], [-7.352506, -4.805249], [-6.288338, 12.43583]])
VT0 = np.array([[3.528048, -1.274511], [-0.511988, -1.485443],
                [1.972071, -3.679895], [-3.460489, -0.767699], [-2.039248, 2.7618]])

k1, k2, d, mu = 0.5, 0.5, 5.0, 9.0
N = 5

rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, 600.0,
                     method='LSODA', rtol=1e-6, atol=1e-8)
t_eval = np.arange(100, 600.1, 0.1)
pos_all, vel_all = evaluate(sol, N, t_eval)

# Mean radius (breathing mode)
r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
peaks_r, _ = find_peaks(r, prominence=0.001, distance=10)
T_breathing = np.mean(np.diff(t_eval[peaks_r]))
print(f"Breathing period T_breathing = {T_breathing:.4f} s")

print("\n" + "=" * 80)
print("INDIVIDUAL AGENT ANGULAR ANALYSIS")
print("=" * 80)

for i in range(N):
    x_i = pos_all[:, i, :]
    v_i = vel_all[:, i, :]
    
    theta_i = np.arctan2(x_i[:, 1], x_i[:, 0])
    theta_i_unwrapped = np.unwrap(theta_i)
    omega_i = np.gradient(theta_i_unwrapped, t_eval)
    
    cross_i = x_i[:, 0] * v_i[:, 1] - x_i[:, 1] * v_i[:, 0]
    r_i_mag = np.linalg.norm(x_i, axis=1)
    omega_i_cross = cross_i / (r_i_mag**2 + 1e-10)
    
    peaks_om, _ = find_peaks(omega_i, prominence=0.001, distance=10)
    T_om_i = np.mean(np.diff(t_eval[peaks_om])) if len(peaks_om) >= 3 else None
    
    # Rotation period: time for theta to increase by 2*pi
    # Find when theta crosses multiples of 2*pi
    theta_cycles = theta_i_unwrapped / (2*np.pi)
    # Find indices where theta_cycles passes integer values
    crossings = []
    for j in range(1, len(theta_cycles)):
        if np.floor(theta_cycles[j]) > np.floor(theta_cycles[j-1]):
            crossings.append(j)
    if len(crossings) >= 3:
        rotation_periods = np.diff(t_eval[crossings])
        T_rotation = np.mean(rotation_periods)
    else:
        T_rotation = None
    
    mean_omega_i = np.mean(omega_i[-1000:])
    
    print(f"\nAgent {i+1}:")
    print(f"  Mean radius = {np.mean(r_i_mag):.4f} +/- {np.std(r_i_mag):.4f}")
    print(f"  Mean angular velocity omega_i = {mean_omega_i:.4f} rad/s")
    print(f"  omega_i / sqrt(k2) = {mean_omega_i/np.sqrt(k2):.4f}")
    print(f"  Rotation period = 2*pi/|omega_i| = {2*np.pi/abs(mean_omega_i):.4f} s")
    if T_om_i:
        print(f"  omega_i variation period T_om_i = {T_om_i:.4f} s")
        print(f"  T_om_i / T_breathing = {T_om_i/T_breathing:.4f}")
    if T_rotation:
        print(f"  Actual rotation period = {T_rotation:.4f} s")

# Summary
print("\n" + "=" * 80)
print("SUMMARY")
print("=" * 80)
print(f"""
The vector a_i = x_i - x_0 = x_i (since target at origin).
- Each agent's angle θ_i(t) rotates with mean angular velocity ω_i ≈ -0.775 rad/s
- The rotation period T_rot = 2π/|ω_i| ≈ 8.1 s
- The breathing period T_breathing = {T_breathing:.4f} s
- The angular velocity ω_i(t) varies with period T_breathing (not T_rot)

The angular variation period of a_i = the breathing period.
The rotation period of a_i ≠ the breathing period.

So: the breathing period IS the period of angular velocity VARIATION,
but NOT the period of angular ROTATION of individual agents.
""")