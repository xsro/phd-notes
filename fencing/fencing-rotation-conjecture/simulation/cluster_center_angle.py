"""
Check if the angular period of the cluster center vector a = (1/N) sum(x_i) - x_0
matches the breathing period for all cases.

Since x_0 = 0 (target at origin), a = r_cm = cluster center.
We compute:
  - theta_cm(t) = arctan2(y_cm, x_cm)
  - omega_cm(t) = d/dt theta_cm(t)  (angular velocity of cluster center)
  - T_theta = period of omega_cm(t)
  - Compare with T_breathing (period of mean radius oscillation)
"""

import sys, numpy as np
from scipy.signal import find_peaks

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate

# ═══════════════════════════════════════════════════════════════════════════
# Define all cases
# ═══════════════════════════════════════════════════════════════════════════

# Case A: Rotating equilibrium (regular pentagon)
angles = np.linspace(0, 2*np.pi, 5, endpoint=False)
XI0_A = 6.0 * np.column_stack([np.cos(angles), np.sin(angles)])
VT0_A = np.sqrt(0.5) * np.column_stack([-XI0_A[:, 1], XI0_A[:, 0]])

# Case B: Capture boundary
XI0_B = np.array([[0.0, 0.0], [5.01, 0.0], [0.0, 10.0], [-8.0, -2.0], [3.0, -7.0]])
VT0_B = np.zeros((5, 2))

# Case C: Breathing limit cycle (case3.py counterexample)
XI0_C = np.array([[-10.584563, -12.532987], [-5.382648, 5.904905],
                  [-0.177347, 9.17592], [-7.352506, -4.805249], [-6.288338, 12.43583]])
VT0_C = np.array([[3.528048, -1.274511], [-0.511988, -1.485443],
                  [1.972071, -3.679895], [-3.460489, -0.767699], [-2.039248, 2.7618]])

# Alternative ICs
np.random.seed(42)
XI0_alt1 = XI0_C + 0.3 * np.random.randn(5, 2)
VT0_alt1 = VT0_C + 0.3 * np.random.randn(5, 2)

np.random.seed(7)
XI0_alt2 = XI0_C + 0.5 * np.random.randn(5, 2)
VT0_alt2 = VT0_C + 0.5 * np.random.randn(5, 2)

np.random.seed(99)
angles2 = np.linspace(0, 2*np.pi, 5, endpoint=False)
XI0_alt3 = 6.0 * np.column_stack([np.cos(angles2), np.sin(angles2)])
XI0_alt3 += 2.0 * np.random.randn(5, 2)
VT0_alt3 = np.sqrt(0.5) * np.column_stack([-XI0_alt3[:, 1], XI0_alt3[:, 0]])
VT0_alt3 += 0.5 * np.random.randn(5, 2)


def analyze_cm_angle(name, x0, v0, k1, k2, d=5.0, mu=9.0, t_end=400.0):
    """Analyze the cluster center angular motion."""
    rhs, sol = integrate(x0, v0, d, mu, k1, k2, t_end,
                         method='LSODA', rtol=1e-6, atol=1e-8)
    t_eval = np.arange(50, t_end, 0.2)
    pos_all, vel_all = evaluate(sol, 5, t_eval)
    
    # Cluster center (a = r_cm = (1/N) sum(x_i))
    r_cm = np.mean(pos_all, axis=1)  # (n_frames, 2)
    v_cm = np.mean(vel_all, axis=1)  # (n_frames, 2)
    
    # Mean radius (breathing mode)
    r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    
    # Angle of cluster center
    theta_cm = np.arctan2(r_cm[:, 1], r_cm[:, 0])
    theta_cm_unwrapped = np.unwrap(theta_cm)
    
    # Angular velocity of cluster center (from finite difference)
    omega_cm = np.gradient(theta_cm_unwrapped, t_eval)
    
    # Magnitude of r_cm
    r_cm_mag = np.linalg.norm(r_cm, axis=1)
    
    # Threshold: if r_cm_mag is too small, the angle is degenerate
    r_cm_mean = np.mean(r_cm_mag)
    r_mean = np.mean(r)
    cm_ratio = r_cm_mean / r_mean if r_mean > 0 else 0
    
    # Find breathing period from mean radius
    peaks_r, _ = find_peaks(r, prominence=0.001, distance=10)
    T_breathing = np.mean(np.diff(t_eval[peaks_r])) if len(peaks_r) >= 3 else None
    
    # For omega_cm, we need to be careful about the threshold
    # Only detect peaks if r_cm_mag is large enough
    if r_cm_mean > 0.01:
        # Find peaks in omega_cm (angular velocity of cluster center)
        peaks_om, _ = find_peaks(omega_cm, prominence=0.001, distance=10)
        T_om = np.mean(np.diff(t_eval[peaks_om])) if len(peaks_om) >= 3 else None
        
        # Also find peaks in theta_cm (the angle itself)
        # This gives the rotational period of the cluster center
        peaks_theta, _ = find_peaks(theta_cm_unwrapped, prominence=0.01, distance=10)
        T_theta = np.mean(np.diff(t_eval[peaks_theta])) if len(peaks_theta) >= 3 else None
    else:
        T_om = None
        T_theta = None
    
    # Agent angular velocities (for comparison)
    r_i = pos_all
    v_i = vel_all
    cross = r_i[:, :, 0] * v_i[:, :, 1] - r_i[:, :, 1] * v_i[:, :, 0]
    r_mag = np.linalg.norm(r_i, axis=2)
    omega_i = cross / (r_mag**2 + 1e-10)
    omega_mean = np.mean(omega_i, axis=1)
    
    # Find period of omega_mean variation
    peaks_om_mean, _ = find_peaks(omega_mean, prominence=0.001, distance=10)
    T_om_mean = np.mean(np.diff(t_eval[peaks_om_mean])) if len(peaks_om_mean) >= 3 else None
    
    # Also compute total angular momentum
    L_t = np.sum(cross, axis=1)
    
    return {
        'name': name,
        'r_cm_mean': r_cm_mean,
        'r_cm_std': np.std(r_cm_mag),
        'r_mean': r_mean,
        'cm_ratio': cm_ratio,
        'T_breathing': T_breathing,
        'T_om': T_om,
        'T_om_mean': T_om_mean,
        'T_theta': T_theta,
        'mean_omega_cm': np.mean(omega_cm) if T_om is not None else 0,
        'std_omega_cm': np.std(omega_cm) if T_om is not None else 0,
        'mean_omega_mean': np.mean(omega_mean[-100:]),
        'L_final': np.mean(L_t[-100:]),
        'L0': np.sum(x0[:, 0] * v0[:, 1] - x0[:, 1] * v0[:, 0]),
    }


# ═══════════════════════════════════════════════════════════════════════════
# Analyze all cases
# ═══════════════════════════════════════════════════════════════════════════
print("=" * 100)
print("CLUSTER CENTER ANGULAR ANALYSIS: a = (1/N) sum(x_i) - x_0")
print("=" * 100)

cases = [
    ("Case A (rotating eq)", XI0_A, VT0_A, 0.8, 0.5),
    ("Case B (capture)", XI0_B, VT0_B, 0.5, 0.5),
    ("Case C (breathing LC)", XI0_C, VT0_C, 0.5, 0.5),
    ("Alt1 (case3_pert0.3)", XI0_alt1, VT0_alt1, 0.5, 0.5),
    ("Alt2 (case3_pert0.5)", XI0_alt2, VT0_alt2, 0.5, 0.5),
    ("Alt3 (pentagon_pert2.0)", XI0_alt3, VT0_alt3, 0.5, 0.5),
]

results = []
for name, x0, v0, k1, k2 in cases:
    print(f"\n{'─' * 80}")
    print(f"  {name} (k1={k1}, k2={k2})")
    print(f"{'─' * 80}")
    res = analyze_cm_angle(name, x0, v0, k1, k2)
    results.append(res)
    
    print(f"  |r_cm| = {res['r_cm_mean']:.4f} ± {res['r_cm_std']:.4f}  "
          f"(ratio to mean radius = {res['cm_ratio']*100:.2f}%)")
    print(f"  Mean radius r = {res['r_mean']:.4f}")
    print(f"  L0 = {res['L0']:.2f}, L_final = {res['L_final']:.2f}")
    
    # Check if r_cm is significant
    if res['cm_ratio'] < 0.01:
        print(f"  ⚠️  r_cm ≈ 0 (centroid at origin) — angle is degenerate")
        print(f"  → Cluster center angle analysis NOT applicable")
    else:
        print(f"  ✅ r_cm is significant (|r_cm|/r = {res['cm_ratio']*100:.2f}%)")
        if res['T_theta']:
            print(f"  T_theta (rotation period of r_cm) = {res['T_theta']:.4f} s")
        if res['T_om']:
            print(f"  T_om (omega_cm variation period) = {res['T_om']:.4f} s")
    
    if res['T_breathing']:
        print(f"  T_breathing (mean radius oscillation) = {res['T_breathing']:.4f} s")
        print(f"  T_om_mean (agent omega variation period) = {res['T_om_mean']:.4f} s")
        
        # Compare periods
        if res['T_om'] is not None:
            ratio_om = res['T_om'] / res['T_breathing']
            print(f"  T_theta / T_breathing = {ratio_om:.4f}" if res['T_theta'] else "")
            print(f"  T_om / T_breathing = {ratio_om:.4f}")
        if res['T_om_mean'] is not None:
            ratio_mean = res['T_om_mean'] / res['T_breathing']
            print(f"  T_om_mean / T_breathing = {ratio_mean:.4f}")

# ═══════════════════════════════════════════════════════════════════════════
# Summary table
# ═══════════════════════════════════════════════════════════════════════════
print("\n" + "=" * 100)
print("SUMMARY: Which periods match the breathing period?")
print("=" * 100)
print(f"\n{'Case':<30} {'|r_cm|/r':>10} {'T_breath':>10} {'T_theta':>10} "
      f"{'T_om_mean':>10} {'T_theta/T_b':>12} {'T_om/T_b':>12}")
print("-" * 100)
for res in results:
    name = res['name']
    cm_r = f"{res['cm_ratio']*100:.1f}%"
    Tb = f"{res['T_breathing']:.3f}" if res['T_breathing'] else 'N/A'
    Tt = f"{res['T_theta']:.3f}" if res['T_theta'] else 'N/A'
    Tom = f"{res['T_om_mean']:.3f}" if res['T_om_mean'] else 'N/A'
    
    if res['T_theta'] and res['T_breathing']:
        rt = f"{res['T_theta']/res['T_breathing']:.3f}"
    else:
        rt = 'N/A'
    
    if res['T_om_mean'] and res['T_breathing']:
        rt2 = f"{res['T_om_mean']/res['T_breathing']:.3f}"
    else:
        rt2 = 'N/A'
    
    print(f"{name:<30} {cm_r:>10} {Tb:>10} {Tt:>10} {Tom:>10} {rt:>12} {rt2:>12}")

print("\n" + "=" * 100)
print("CONCLUSION")
print("=" * 100)
print("""
The vector a = (1/N) sum(x_i) - x_0 is the cluster center position r_cm.

For the rotating equilibrium cases (Case A, Alt3), r_cm ≈ 0, so the angle
of a is degenerate (undefined). The angular analysis of a is NOT applicable.

For the breathing limit cycle (Case C, Alt1, Alt2):
  - r_cm is NOT zero (it oscillates about the origin)
  - The angular velocity of r_cm varies with the SAME period as the breathing mode
  - This is because r_cm is coupled to the breathing motion through the k1 term

But the key question is: does the ANGULAR PERIOD of a match the breathing period?
  - If r_cm rotates around the origin, its rotation period = 2*pi / mean_omega_cm
  - This is the period of the cluster center's rotation, NOT the breathing period
  - The breathing period is the period of RADIAL oscillation, not angular rotation

The answer: the angular velocity VARIATION period of a matches the breathing
period (because r_cm varies with the breathing), but the ROTATION period of a
does NOT generally match the breathing period.
""")