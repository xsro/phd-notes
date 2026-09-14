"""
Enhanced bifurcation diagram with 3 panels:
  1. Period T vs k1
  2. Frequency ratio ω_eff/√k2 vs k1
  3. Breathing amplitude vs k1

Uses documented data as backbone + targeted verification in transition regions.

Usage:
    python simulation/plot_bifurcation_enhanced.py
"""

import sys, numpy as np
from scipy.signal import find_peaks
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

sys.path.insert(0, 'simulation')
from fencing_ode import integrate, evaluate

K2 = 0.5
D  = 5.0
MU = 9.0
N  = 5

# ── Documented data (from breathing.md parameter scans) ─────────────────────
# High-frequency branch (case3 IC, k2=0.5)
HIGH = [
    (0.15, 2.172, 0.067),
    (0.25, 2.342, 0.062),
    (0.35, 2.397, 0.059),
    (0.40, 2.429, 0.059),
    (0.45, 2.459, 0.058),
    (0.50, 2.485, 0.058),
    (0.55, 2.510, 0.058),
    (0.60, 2.532, 0.057),
    (0.62, 2.540, 0.057),
]

# Low-frequency branch (perturbed pentagon IC, k2=0.5)
LOW = [
    (0.05, 4.485, 0.129),
    (0.10, 4.589, 0.189),
    (0.20, 4.750, 0.094),
    (0.50, 6.24, 0.056),   # from low-freq branch scan
    (0.60, 6.26, 0.062),   # from low-freq branch scan
]

# Rotating equilibrium
ROT_K1 = [0.65]
ROT_T = [2*np.pi/np.sqrt(K2)]  # 8.886 s
ROT_AMP = [0.0]

# ── Initial conditions ─────────────────────────────────────────────────────
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

# Multiple ICs for basin mapping (reduced set for speed)
np.random.seed(99)
angles = np.linspace(0, 2*np.pi, N, endpoint=False)
XI0_pent = 6.0 * np.column_stack([np.cos(angles), np.sin(angles)])
VT0_pent = np.sqrt(K2) * np.column_stack([-XI0_pent[:, 1], XI0_pent[:, 0]])

ICS = []
for pert in [0.5, 2.0]:
    np.random.seed(99 + int(pert*100))
    xi = XI0_pent + pert * np.random.randn(N, 2)
    vt = VT0_pent + 0.5 * np.random.randn(N, 2)
    ICS.append((f"pent_pert{pert:.1f}", xi, vt))

for pert in [0.1, 0.5]:
    np.random.seed(42 + int(pert*100))
    xi = XI0_high + pert * np.random.randn(N, 2)
    vt = VT0_high + pert * np.random.randn(N, 2)
    ICS.append((f"case3_pert{pert:.1f}", xi, vt))


def quick_classify(k1, xi0, vt0, t_end=100.0, t_skip=50.0):
    """Quick classification with period and amplitude estimation."""
    rhs, sol = integrate(xi0, vt0, D, MU, k1, K2, t_end,
                         method='BDF', rtol=1e-4, atol=1e-6)
    t_eval = np.arange(t_skip, t_end + 0.1, 0.3)
    pos_all, vel_all = evaluate(sol, N, t_eval)

    r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    r_mean = np.mean(r)
    r_std  = np.std(r)
    amp_rel = r_std / r_mean if r_mean > 0 else 0

    r_i = pos_all
    v_i = vel_all
    cross = r_i[:, :, 0] * v_i[:, :, 1] - r_i[:, :, 1] * v_i[:, :, 0]
    r_mag = np.linalg.norm(r_i, axis=2)
    omega_i = cross / (r_mag**2 + 1e-10)
    omega_mean = np.mean(omega_i, axis=1)
    omega_final = np.mean(omega_mean[-150:])

    if amp_rel < 0.008:
        return 'rot', None, amp_rel, omega_final

    prominence = max(0.005, amp_rel * 0.15)
    peaks, _ = find_peaks(r, prominence=prominence, distance=5)

    if len(peaks) >= 3:
        T = np.mean(np.diff(t_eval[peaks]))
        T_std = np.std(np.diff(t_eval[peaks]))
        cv = T_std / T if T > 0 else 999
        if cv < 0.15:
            ratio = 2*np.pi / (T * np.sqrt(K2))
            if ratio > 2.5:
                return 'high', T, amp_rel, omega_final
            elif ratio < 2.0:
                return 'low', T, amp_rel, omega_final
            else:
                return 'mid', T, amp_rel, omega_final
        else:
            return 'transient', T, amp_rel, omega_final
    else:
        return 'unknown', None, amp_rel, omega_final


# ── Targeted verification in key regions ────────────────────────────────────
# Focus on: coexistence region (0.10-0.35) and near bifurcation (0.60-0.70)
VERIFY_K1 = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.65]

print("=" * 80)
print("ENHANCED BIFURCATION SWEEP")
print(f"Verifying {len(VERIFY_K1)} k1 values × {len(ICS)} ICs = {len(VERIFY_K1)*len(ICS)} runs")
print("=" * 80)

basin_map = {}  # (k1, ic_name) -> branch

for k1 in VERIFY_K1:
    print(f"\nk1 = {k1:.2f}")
    for name, xi0, vt0 in ICS:
        status, T, amp, omega = quick_classify(k1, xi0, vt0)
        basin_map[(k1, name)] = (status, T, amp, omega)
        T_str = f"{T:.3f}" if T else "N/A"
        print(f"  {name:<18} → {status:>10}, T={T_str:>8}, amp={amp*100:5.1f}%, ω={omega:.4f}")

# ── Plot ─────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 12))

# Use gridspec for custom layout
from matplotlib.gridspec import GridSpec
gs = GridSpec(2, 2, figure=fig, width_ratios=[2, 1], height_ratios=[1, 1],
              wspace=0.35, hspace=0.35, left=0.06, right=0.96, top=0.92, bottom=0.08)

ax_main = fig.add_subplot(gs[0, :])  # Main bifurcation: T vs k1
ax_ratio = fig.add_subplot(gs[1, 0])  # Frequency ratio
ax_amp = fig.add_subplot(gs[1, 1])   # Amplitude

# ── Color palette ───────────────────────────────────────────────────────────
c_high = '#1f77b4'   # blue
c_low  = '#d62728'   # red
c_rot  = '#2ca02c'   # green
c_mid  = '#ff7f0e'   # orange
c_trans = '#7f7f7f'  # gray

# ═══════════════════════════════════════════════════════════════════════════
# Panel 1: Period T vs k1 (main)
# ═══════════════════════════════════════════════════════════════════════════

# Plot documented data as large filled markers
high_k1 = [d[0] for d in HIGH]
high_T  = [d[1] for d in HIGH]
low_k1  = [d[0] for d in LOW]
low_T   = [d[1] for d in LOW]

ax_main.plot(high_k1, high_T, 'o-', color=c_high, linewidth=2.5, markersize=12,
             label='High-freq LC (case3 IC)', zorder=10)
ax_main.plot(low_k1, low_T, 's-', color=c_low, linewidth=2.5, markersize=12,
             label='Low-freq LC (perturbed pentagon)', zorder=10)
ax_main.plot(ROT_K1, ROT_T, 'v--', color=c_rot, linewidth=2.5, markersize=12,
             label='Rotating equilibrium', zorder=10)

# Plot verification points as smaller markers
for (k1, name), (status, T, amp, omega) in basin_map.items():
    if T is None:
        continue
    if status == 'high':
        ax_main.plot(k1, T, 'o', color=c_high, markersize=8,
                     markerfacecolor='white', markeredgewidth=2, zorder=8)
    elif status == 'low':
        ax_main.plot(k1, T, 's', color=c_low, markersize=8,
                     markerfacecolor='white', markeredgewidth=2, zorder=8)
    elif status == 'mid':
        ax_main.plot(k1, T, '^', color=c_mid, markersize=8,
                     markerfacecolor='white', markeredgewidth=2, zorder=8)
    elif status == 'rot':
        T_rot = 2*np.pi/(abs(omega)*np.sqrt(K2))
        ax_main.plot(k1, T_rot, 'v', color=c_rot, markersize=8,
                     markerfacecolor='white', markeredgewidth=2, zorder=8)
    elif status == 'transient':
        ax_main.plot(k1, T, 'x', color=c_trans, markersize=8,
                     markeredgewidth=2, zorder=6)

# Shade coexistence region
ax_main.axvspan(0.10, 0.35, alpha=0.08, color='gray')
ax_main.axvspan(0.60, 0.70, alpha=0.05, color='red')

# Annotations
ax_main.annotate('Multi-attractor\ncoexistence', xy=(0.22, 5.5), fontsize=9,
                 ha='center', color='gray',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
ax_main.annotate('Saddle-node\nbifurcation', xy=(0.65, 7.5), fontsize=9,
                 ha='center', color='red',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))

ax_main.set_xlabel('k₁ (damping)', fontsize=13, fontweight='bold')
ax_main.set_ylabel('Period T (s)', fontsize=13, fontweight='bold')
ax_main.set_title('Bifurcation Diagram: Breathing Limit Cycle vs Damping\n(k₂=0.5, d=5, μ=9, N=5)', fontsize=14, fontweight='bold')
ax_main.grid(True, alpha=0.3, linestyle='--')
ax_main.legend(fontsize=9, loc='upper left', ncol=2, framealpha=0.9)

# ═══════════════════════════════════════════════════════════════════════════
# Panel 2: Frequency ratio ω_eff/√k2 vs k1
# ═══════════════════════════════════════════════════════════════════════════

high_ratio = [2*np.pi/(t*np.sqrt(K2)) for t in high_T]
low_ratio  = [2*np.pi/(t*np.sqrt(K2)) for t in low_T]

ax_ratio.plot(high_k1, high_ratio, 'o-', color=c_high, linewidth=2.5, markersize=10,
              label='High-freq LC', zorder=10)
ax_ratio.plot(low_k1, low_ratio, 's-', color=c_low, linewidth=2.5, markersize=10,
              label='Low-freq LC', zorder=10)
ax_ratio.plot(ROT_K1, [1.0]*len(ROT_K1), 'v--', color=c_rot, linewidth=2.5, markersize=10,
              label='Rotating eq', zorder=10)

for (k1, name), (status, T, amp, omega) in basin_map.items():
    if T is None:
        if status == 'rot':
            ax_ratio.plot(k1, abs(omega)/np.sqrt(K2), 'v', color=c_rot, markersize=6,
                         markerfacecolor='white', markeredgewidth=1.5, zorder=8)
        continue
    ratio = 2*np.pi/(T*np.sqrt(K2))
    if status == 'high':
        ax_ratio.plot(k1, ratio, 'o', color=c_high, markersize=6,
                     markerfacecolor='white', markeredgewidth=1.5, zorder=8)
    elif status == 'low':
        ax_ratio.plot(k1, ratio, 's', color=c_low, markersize=6,
                     markerfacecolor='white', markeredgewidth=1.5, zorder=8)
    elif status == 'mid':
        ax_ratio.plot(k1, ratio, '^', color=c_mid, markersize=6,
                     markerfacecolor='white', markeredgewidth=1.5, zorder=8)

# Reference lines
ax_ratio.axhline(y=3.5, color=c_high, linestyle=':', alpha=0.4, label='high ≈ 3.5')
ax_ratio.axhline(y=1.42, color=c_low, linestyle=':', alpha=0.4, label='low ≈ 1.42')
ax_ratio.axhline(y=1.0, color=c_rot, linestyle=':', alpha=0.4, label='rigid = 1.0')
ax_ratio.axvspan(0.10, 0.35, alpha=0.08, color='gray')
ax_ratio.axvspan(0.60, 0.70, alpha=0.05, color='red')

ax_ratio.set_xlabel('k₁ (damping)', fontsize=12, fontweight='bold')
ax_ratio.set_ylabel('ω_eff / √k₂', fontsize=12, fontweight='bold')
ax_ratio.set_title('Frequency Ratio vs k₁', fontsize=13, fontweight='bold')
ax_ratio.grid(True, alpha=0.3, linestyle='--')
ax_ratio.legend(fontsize=8, loc='upper right')

# ═══════════════════════════════════════════════════════════════════════════
# Panel 3: Amplitude vs k1
# ═══════════════════════════════════════════════════════════════════════════

high_amp = [d[2] for d in HIGH]
low_amp  = [d[2] for d in LOW]

ax_amp.plot(high_k1, high_amp, 'o-', color=c_high, linewidth=2.5, markersize=10,
            label='High-freq LC', zorder=10)
ax_amp.plot(low_k1, low_amp, 's-', color=c_low, linewidth=2.5, markersize=10,
            label='Low-freq LC', zorder=10)
ax_amp.plot(ROT_K1, ROT_AMP, 'v--', color=c_rot, linewidth=2.5, markersize=10,
            label='Rotating eq', zorder=10)

for (k1, name), (status, T, amp, omega) in basin_map.items():
    if amp is None:
        continue
    if status == 'high':
        ax_amp.plot(k1, amp, 'o', color=c_high, markersize=6,
                   markerfacecolor='white', markeredgewidth=1.5, zorder=8)
    elif status == 'low':
        ax_amp.plot(k1, amp, 's', color=c_low, markersize=6,
                   markerfacecolor='white', markeredgewidth=1.5, zorder=8)
    elif status == 'mid':
        ax_amp.plot(k1, amp, '^', color=c_mid, markersize=6,
                   markerfacecolor='white', markeredgewidth=1.5, zorder=8)
    elif status == 'rot':
        ax_amp.plot(k1, 0.0, 'v', color=c_rot, markersize=6,
                   markerfacecolor='white', markeredgewidth=1.5, zorder=8)

ax_amp.axvspan(0.10, 0.35, alpha=0.08, color='gray')
ax_amp.axvspan(0.60, 0.70, alpha=0.05, color='red')
ax_amp.axhline(y=0.05, color='black', linestyle='--', alpha=0.3, label='5% amplitude')

ax_amp.set_xlabel('k₁ (damping)', fontsize=12, fontweight='bold')
ax_amp.set_ylabel('Breathing amplitude', fontsize=12, fontweight='bold')
ax_amp.set_title('Breathing Amplitude vs k₁', fontsize=13, fontweight='bold')
ax_amp.grid(True, alpha=0.3, linestyle='--')
ax_amp.legend(fontsize=8, loc='upper right')

# ═══════════════════════════════════════════════════════════════════════════
# Legend for marker types
# ═══════════════════════════════════════════════════════════════════════════
legend_elements = [
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=c_high, markersize=10, markeredgewidth=2, label='Documented data'),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='white', markersize=8, markeredgewidth=2, label='Verification (hollow)'),
    plt.Line2D([0], [0], marker='x', color=c_trans, markersize=8, markeredgewidth=2, label='Transient / uncertain'),
]
ax_main.legend(handles=legend_elements, fontsize=9, loc='upper left', ncol=3, framealpha=0.9)

plt.suptitle('Fencing Rotation Conjecture — Bifurcation Analysis', fontsize=15, fontweight='bold')
# plt.tight_layout(rect=[0, 0, 1, 0.96])  # GridSpec handles layout

out_path = 'data/bifurcation_diagram_enhanced.png'
plt.savefig(out_path, dpi=200, bbox_inches='tight', facecolor='white')
print(f"\n✓ Enhanced bifurcation diagram saved to {out_path}")