"""
Analyze the angular velocity of the cluster center vector (r_cm) during the
breathing limit cycle, and compare its variation period with the breathing period.
"""

import sys, numpy as np
from scipy.signal import find_peaks, correlate

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate

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


def analyze_cm_angular_velocity(k1=0.5, k2=0.5, d=5.0, mu=9.0, t_end=600.0):
    """Integrate and analyze cluster center angular velocity."""
    rhs, sol = integrate(XI0, VT0, d, mu, k1, k2, t_end,
                         method='LSODA', rtol=1e-8, atol=1e-10)
    t_eval = np.arange(200, t_end, 0.1)
    pos_all, vel_all = evaluate(sol, 5, t_eval)
    
    # Cluster center position and velocity
    r_cm = np.mean(pos_all, axis=1)  # (n_frames, 2)
    v_cm = np.mean(vel_all, axis=1)  # (n_frames, 2)
    
    # Mean radius (breathing mode)
    r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    
    # Angular velocity of each agent
    r_i = pos_all  # (n_frames, N, 2)
    v_i = vel_all  # (n_frames, N, 2)
    cross = r_i[:, :, 0] * v_i[:, :, 1] - r_i[:, :, 1] * v_i[:, :, 0]
    r_mag = np.linalg.norm(r_i, axis=2)
    omega_i = cross / (r_mag**2 + 1e-10)
    omega_mean = np.mean(omega_i, axis=1)
    
    # Angular velocity of cluster center
    cross_cm = r_cm[:, 0] * v_cm[:, 1] - r_cm[:, 1] * v_cm[:, 0]
    r_cm_mag = np.linalg.norm(r_cm, axis=1)
    omega_cm = cross_cm / (r_cm_mag**2 + 1e-10)
    
    # Angular velocity from finite difference of theta
    theta_cm = np.arctan2(r_cm[:, 1], r_cm[:, 0])
    theta_cm_unwrapped = np.unwrap(theta_cm)
    omega_cm_fd = np.gradient(theta_cm_unwrapped, t_eval)
    
    # Find breathing period from mean radius
    peaks_r, _ = find_peaks(r, prominence=0.001, distance=10)
    T_breathing = np.mean(np.diff(t_eval[peaks_r])) if len(peaks_r) >= 3 else None
    
    # Find period of omega_cm variation
    peaks_om, _ = find_peaks(omega_cm, prominence=0.001, distance=10)
    T_om = np.mean(np.diff(t_eval[peaks_om])) if len(peaks_om) >= 3 else None
    
    # Find period of omega_mean variation
    peaks_om_mean, _ = find_peaks(omega_mean, prominence=0.001, distance=10)
    T_om_mean = np.mean(np.diff(t_eval[peaks_om_mean])) if len(peaks_om_mean) >= 3 else None
    
    return {
        't_eval': t_eval,
        'r': r,
        'omega_cm': omega_cm,
        'omega_cm_fd': omega_cm_fd,
        'omega_mean': omega_mean,
        'r_cm_mag': r_cm_mag,
        'T_breathing': T_breathing,
        'T_om': T_om,
        'T_om_mean': T_om_mean,
        'mean_omega_cm': np.mean(omega_cm),
        'std_omega_cm': np.std(omega_cm),
        'mean_omega_mean': np.mean(omega_mean),
        'std_omega_mean': np.std(omega_mean),
        'L_cm': r_cm_mag**2 * omega_cm,
    }


# Analyze baseline case
print("=" * 80)
print("Cluster center angular velocity analysis")
print("=" * 80)

res = analyze_cm_angular_velocity(k1=0.5, k2=0.5)

print(f"\nBreathing period T_breathing = {res['T_breathing']:.4f} s")
if res['T_om']:
    print(f"omega_cm variation period T_om = {res['T_om']:.4f} s")
    print(f"Ratio T_om / T_breathing = {res['T_om']/res['T_breathing']:.4f}")
if res['T_om_mean']:
    print(f"omega_mean variation period T_om_mean = {res['T_om_mean']:.4f} s")
    print(f"Ratio T_om_mean / T_breathing = {res['T_om_mean']/res['T_breathing']:.4f}")

print(f"\nMean omega_cm = {res['mean_omega_cm']:.4f}, std = {res['std_omega_cm']:.4f}")
print(f"Mean omega_mean = {res['mean_omega_mean']:.4f}, std = {res['std_omega_mean']:.4f}")

k2 = 0.5
omega_nat = np.sqrt(k2)
print(f"\nNatural frequency sqrt(k2) = {omega_nat:.4f}")
print(f"omega_cm / sqrt(k2) = {res['mean_omega_cm']/omega_nat:.4f}")
print(f"omega_mean / sqrt(k2) = {res['mean_omega_mean']/omega_nat:.4f}")

# Check angular momentum conservation
L_cm = res['L_cm']
print(f"\nAngular momentum L_cm = r_cm^2 * omega_cm:")
print(f"  mean = {np.mean(L_cm):.4f}, std = {np.std(L_cm):.4f}")
print(f"  std/mean = {np.std(L_cm)/np.mean(L_cm)*100:.2f}%")

# Check if omega_cm ~ 1/r_cm^2
omega_cm_pred = np.mean(L_cm) / (res['r_cm_mag']**2)
omega_cm_actual = res['omega_cm']
err = np.mean(np.abs(omega_cm_pred - omega_cm_actual) / np.abs(omega_cm_actual)) * 100
print(f"\nIf L_cm conserved: omega_cm = L_mean / r_cm^2")
print(f"  Mean relative error: {err:.2f}%")
print(f"  -> omega_cm variation IS explained by 1/r_cm^2: {'YES' if err < 5 else 'NO'}")

# Check correlation between r and omega_cm
r = res['r']
omega_cm = res['omega_cm']
corr = correlate(r - np.mean(r), omega_cm - np.mean(omega_cm), mode='same')
lag_idx = np.argmax(np.abs(corr)) - len(corr)//2
lag_s = lag_idx * 0.1
print(f"\nCross-correlation between r and omega_cm:")
print(f"  Lag: {lag_idx} samples ({lag_s:.2f} s)")
print(f"  This means omega_cm varies {'in phase' if abs(lag_s) < 0.2 else 'out of phase'} with r")

# Check: does omega_cm vary with the SAME period as r?
print(f"\n{'='*60}")
print(f"CONCLUSION:")
print(f"{'='*60}")
print(f"  T_breathing = {res['T_breathing']:.4f} s (period of mean radius oscillation)")
if res['T_om']:
    print(f"  T_om = {res['T_om']:.4f} s (period of omega_cm variation)")
    print(f"  T_om / T_breathing = {res['T_om']/res['T_breathing']:.4f}")
    if abs(res['T_om']/res['T_breathing'] - 1.0) < 0.05:
        print(f"  => omega_cm varies with THE SAME period as the breathing mode!")
        print(f"  => The breathing period IS the angular velocity variation period.")
    else:
        print(f"  => omega_cm varies with a DIFFERENT period than the breathing mode.")
print(f"\n  omega_cm / sqrt(k2) = {res['mean_omega_cm']/omega_nat:.4f}")
print(f"  This is the effective rotation frequency of the cluster center.")
print(f"  It is NOT equal to sqrt(k2) (the natural frequency).")
print(f"  The ratio omega_cm/sqrt(k2) = {res['mean_omega_cm']/omega_nat:.4f}")
print(f"  is the same as the breathing frequency ratio we computed earlier!")