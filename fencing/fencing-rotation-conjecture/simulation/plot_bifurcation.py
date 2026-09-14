"""
Bifurcation diagram using pre-computed data from breathing.md + light verification.

Uses documented period values from the parameter scan and supplements with
short verification runs to classify branch type (high-freq LC, low-freq LC, rot eq).

Usage:
    python simulation/plot_bifurcation.py
"""

import sys, numpy as np
from scipy.signal import find_peaks
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate

K2 = 0.5
D  = 5.0
MU = 9.0
N  = 5

# ── Pre-computed data from breathing.md (k1 scan, k2=0.5, case3 IC) ─────────
# Format: (k1, T, amplitude, branch)
# Branch: 'high' = high-frequency LC, 'low' = low-frequency LC, 'rot' = rotating eq
DOC_DATA_HIGH = [
    (0.15, 2.172, 0.067, 'high'),
    (0.25, 2.342, 0.062, 'high'),
    (0.35, 2.397, 0.059, 'high'),
    (0.40, 2.429, 0.059, 'high'),
    (0.45, 2.459, 0.058, 'high'),
    (0.50, 2.485, 0.058, 'high'),
    (0.55, 2.510, 0.058, 'high'),
    (0.60, 2.532, 0.057, 'high'),
    (0.62, 2.540, 0.057, 'high'),
]

DOC_DATA_LOW = [
    (0.05, 4.485, 0.129, 'low'),
    (0.10, 4.589, 0.189, 'low'),
    (0.20, 4.750, 0.094, 'low'),
]

DOC_DATA_ROT = [
    (0.65, None, 0.0, 'rot'),
]

# ── Initial conditions ───────────────────────────────────────────────────────
XI0_high = np.array([[-10.584563, -12.532987],
                     [-5.382648,   5.904905],
                     [-0.177347,   9.17592 ],
                     [-7.352506,  -4.805249],
                     [-6.288338,  12.43583 ]])
VT0_high = np.array([[ 3.528048, -1.274511],
                     [-0.511988, -1.485443],
                     [ 1.972071, -3.679895],
                     [-3.460489, -0.767699],
                     [-2.039248,  2.7618  ]])

np.random.seed(99)
angles = np.linspace(0, 2*np.pi, N, endpoint=False)
XI0_low = 6.0 * np.column_stack([np.cos(angles), np.sin(angles)])
XI0_low += 2.0 * np.random.randn(N, 2)
VT0_low = np.sqrt(K2) * np.column_stack([-XI0_low[:, 1], XI0_low[:, 0]])
VT0_low += 0.5 * np.random.randn(N, 2)

angles2 = np.linspace(0, 2*np.pi, N, endpoint=False)
XI0_rot = 6.0 * np.column_stack([np.cos(angles2), np.sin(angles2)])
VT0_rot = np.sqrt(K2) * np.column_stack([-XI0_rot[:, 1], XI0_rot[:, 0]])


def quick_classify(k1, xi0, vt0, t_end=120.0, t_skip=60.0):
    """Quick classification: integrate briefly and classify the attractor."""
    rhs, sol = integrate(xi0, vt0, D, MU, k1, K2, t_end,
                         method='BDF', rtol=1e-4, atol=1e-6)
    t_eval = np.arange(t_skip, t_end + 0.1, 0.5)
    pos_all, vel_all = evaluate(sol, N, t_eval)

    r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    r_mean = np.mean(r)
    r_std  = np.std(r)
    amp_rel = r_std / r_mean if r_mean > 0 else 0

    # Angular velocity
    r_i = pos_all
    v_i = vel_all
    cross = r_i[:, :, 0] * v_i[:, :, 1] - r_i[:, :, 1] * v_i[:, :, 0]
    r_mag = np.linalg.norm(r_i, axis=2)
    omega_i = cross / (r_mag**2 + 1e-10)
    omega_mean = np.mean(omega_i, axis=1)
    omega_final = np.mean(omega_mean[-100:])

    # Classify
    if amp_rel < 0.005:
        return 'rot', None, amp_rel, r_mean, omega_final

    prominence = max(0.005, amp_rel * 0.15)
    peaks, _ = find_peaks(r, prominence=prominence, distance=5)

    if len(peaks) >= 3:
        T = np.mean(np.diff(t_eval[peaks]))
        T_std = np.std(np.diff(t_eval[peaks]))
        cv = T_std / T if T > 0 else 999
        if cv < 0.15:
            # Check if high or low frequency
            ratio = 2*np.pi / (T * np.sqrt(K2))
            if ratio > 2.5:
                return 'high', T, amp_rel, r_mean, omega_final
            else:
                return 'low', T, amp_rel, r_mean, omega_final
        else:
            return 'transient', T, amp_rel, r_mean, omega_final
    else:
        return 'unknown', None, amp_rel, r_mean, omega_final


# ── Quick verification sweep ─────────────────────────────────────────────────
# Only verify a few key points to confirm the branches exist
VERIFY_K1 = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60]

print("=" * 80)
print("QUICK VERIFICATION SWEEP")
print("=" * 80)

verify_results = {}
for k1 in VERIFY_K1:
    print(f"\nk1 = {k1:.2f}")
    for name, xi0, vt0 in [("case3", XI0_high, VT0_high),
                            ("pent2.0", XI0_low, VT0_low),
                            ("pentagon", XI0_rot, VT0_rot)]:
        status, T, amp, r_mean, omega = quick_classify(k1, xi0, vt0)
        T_str = f"{T:.4f}" if T else "N/A"
        print(f"  {name:<12} → {status:>10}, T={T_str:>8}, amp={amp*100:5.1f}%, ω={omega:.4f}")
        if name not in verify_results:
            verify_results[name] = {'k1': [], 'status': [], 'T': [], 'amp': [], 'omega': []}
        verify_results[name]['k1'].append(k1)
        verify_results[name]['status'].append(status)
        verify_results[name]['T'].append(T)
        verify_results[name]['amp'].append(amp)
        verify_results[name]['omega'].append(omega)

# ── Build combined dataset ───────────────────────────────────────────────────
# High-frequency branch (case3 IC)
high_k1 = [d[0] for d in DOC_DATA_HIGH]
high_T  = [d[1] for d in DOC_DATA_HIGH]

# Low-frequency branch (pentagon pert 2.0 IC)
low_k1 = [d[0] for d in DOC_DATA_LOW]
low_T  = [d[1] for d in DOC_DATA_LOW]

# Rotating equilibrium
rot_k1 = [d[0] for d in DOC_DATA_ROT]
rot_T_val = 2*np.pi / (1.0 * np.sqrt(K2))  # T = 2π/√k₂ for ω/√k₂=1
rot_T = [rot_T_val] * len(rot_k1)
rot_omega = [1.0] * len(rot_k1)

# ── Plot ─────────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Panel 1: Period vs k1
ax1.plot(high_k1, high_T, 'o-', color='#1f77b4', linewidth=2, markersize=10,
         label='High-freq LC (case3 IC)', zorder=5)
ax1.plot(low_k1, low_T, 's-', color='#d62728', linewidth=2, markersize=10,
         label='Low-freq LC (pentagon pert 2.0)', zorder=5)
ax1.plot(rot_k1, rot_T, 'v--', color='#2ca02c', linewidth=2, markersize=10,
         label='Rotating equilibrium', zorder=4)

# Add verification points
for name, res in verify_results.items():
    for i, k1 in enumerate(res['k1']):
        if res['status'][i] == 'high':
            ax1.plot(k1, res['T'][i], 'o', color='#1f77b4', markersize=6,
                     markerfacecolor='white', markeredgewidth=2, zorder=6)
        elif res['status'][i] == 'low':
            ax1.plot(k1, res['T'][i], 's', color='#d62728', markersize=6,
                     markerfacecolor='white', markeredgewidth=2, zorder=6)
        elif res['status'][i] == 'rot':
            T_rot = 2*np.pi / (abs(res['omega'][i]) * np.sqrt(K2))
            ax1.plot(k1, T_rot, 'v', color='#2ca02c', markersize=6,
                     markerfacecolor='white', markeredgewidth=2, zorder=6)
        elif res['status'][i] == 'transient' and res['T'][i]:
            ax1.plot(k1, res['T'][i], 'x', color='#7f7f7f', markersize=6,
                     markeredgewidth=2, zorder=3)

# Shade coexistence region
ax1.axvspan(0.15, 0.30, alpha=0.1, color='gray', label='Coexistence region')
ax1.axvline(x=0.65, color='red', linestyle='--', alpha=0.8, label='k₁_crit ≈ 0.65')

ax1.set_xlabel('k₁ (damping)', fontsize=13)
ax1.set_ylabel('Period T (s)', fontsize=13)
ax1.set_title('Bifurcation Diagram: Period vs k₁\n(k₂=0.5, d=5, μ=9, N=5)', fontsize=14)
ax1.grid(True, alpha=0.3)
ax1.legend(fontsize=9, loc='upper left')

# Panel 2: Frequency ratio ω_eff/√k₂ vs k1
high_ratio = [2*np.pi / (t * np.sqrt(K2)) for t in high_T]
low_ratio  = [2*np.pi / (t * np.sqrt(K2)) for t in low_T]

ax2.plot(high_k1, high_ratio, 'o-', color='#1f77b4', linewidth=2, markersize=10,
         label='High-freq LC', zorder=5)
ax2.plot(low_k1, low_ratio, 's-', color='#d62728', linewidth=2, markersize=10,
         label='Low-freq LC', zorder=5)
ax2.plot(rot_k1, rot_omega, 'v--', color='#2ca02c', linewidth=2, markersize=10,
         label='Rotating eq (ω/√k₂=1)', zorder=4)

# Verification points
for name, res in verify_results.items():
    for i, k1 in enumerate(res['k1']):
        if res['status'][i] == 'high' and res['T'][i]:
            ratio = 2*np.pi / (res['T'][i] * np.sqrt(K2))
            ax2.plot(k1, ratio, 'o', color='#1f77b4', markersize=6,
                     markerfacecolor='white', markeredgewidth=2, zorder=6)
        elif res['status'][i] == 'low' and res['T'][i]:
            ratio = 2*np.pi / (res['T'][i] * np.sqrt(K2))
            ax2.plot(k1, ratio, 's', color='#d62728', markersize=6,
                     markerfacecolor='white', markeredgewidth=2, zorder=6)
        elif res['status'][i] == 'rot':
            ax2.plot(k1, abs(res['omega'][i])/np.sqrt(K2), 'v', color='#2ca02c',
                     markersize=6, markerfacecolor='white', markeredgewidth=2, zorder=6)
        elif res['status'][i] == 'transient' and res['T'][i]:
            ratio = 2*np.pi / (res['T'][i] * np.sqrt(K2))
            ax2.plot(k1, ratio, 'x', color='#7f7f7f', markersize=6,
                     markeredgewidth=2, zorder=3)

ax2.axvspan(0.15, 0.30, alpha=0.1, color='gray', label='Coexistence region')
ax2.axhline(y=3.5, color='blue', linestyle=':', alpha=0.5, label='high ≈ 3.5')
ax2.axhline(y=1.42, color='red', linestyle=':', alpha=0.5, label='low ≈ 1.42')
ax2.axhline(y=1.0, color='gray', linestyle=':', alpha=0.5, label='rigid = 1.0')

ax2.set_xlabel('k₁ (damping)', fontsize=13)
ax2.set_ylabel('ω_eff / √k₂', fontsize=13)
ax2.set_title('Frequency Ratio vs k₁\n(k₂=0.5, d=5, μ=9, N=5)', fontsize=14)
ax2.grid(True, alpha=0.3)
ax2.legend(fontsize=9, loc='upper right')

plt.tight_layout()
out_path = 'data/bifurcation_diagram.png'
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"\n✓ Bifurcation diagram saved to {out_path}")