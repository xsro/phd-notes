"""
Plot C_rep vs each parameter from the scan data in SCAN_RESULTS.md.

C_rep = k2 * ( (T_rot / T_breath)^2 - 1 )

where T_rot = 2*pi / sqrt(k2).
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Extract data from SCAN_RESULTS.md ──────────────────────────────────────

# k2 scan (k1=0.5, N=4, d=5, mu=9)
k2_scan = [0.05, 0.10, 0.15, 0.20, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.90]
T_k2    = [30.0, 30.0, 15.0, 15.0, 10.0, 6.0, 5.0, 4.29, 4.29, 3.75, 3.33, 3.33]
Trot_k2 = [28.10, 19.87, 16.22, 14.05, 12.57, 10.68, 9.40, 8.57, 7.85, 7.26, 6.78, 6.62]

# k1 scan (k2=0.5, N=4, d=5, mu=9)
k1_scan = [0.05, 0.10, 0.15, 0.70, 0.75, 0.80, 0.85, 0.90]
T_k1    = [4.29, 4.29, 4.29, 7.50, 7.50, 7.50, 7.50, 7.50]
Trot_k1 = [8.89]*8  # k2=0.5 fixed

# d_col scan (k1=k2=0.5, N=4, mu=9)
d_scan  = [3.0, 4.0, 5.0, 6.0, 7.0]
T_d     = [4.29, 7.50, 5.00, 6.00, 7.50]
Trot_d  = [8.89]*5  # k2=0.5 fixed

# mu scan (k1=k2=0.5, N=4, d=5)
mu_scan = [6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]
T_mu    = [7.50, 6.00, 5.00, 5.00, 5.00, 5.00, 5.00]
Trot_mu = [8.89]*7  # k2=0.5 fixed

# N scan (k1=k2=0.5, d=5, mu=9)
N_scan  = [3, 4, 5]
T_N     = [6.00, 5.00, 7.50]
Trot_N  = [8.89]*3  # k2=0.5 fixed

# seed scan (N=4, k1=k2=0.5, d=5, mu=9)
seed_scan = [0, 1, 2, 3, 4, 5]
T_seed    = [5.00, 5.00, 5.00, 4.29, 5.00, 5.00]
Trot_seed = [8.89]*6  # k2=0.5 fixed

# ── Compute C_rep ──────────────────────────────────────────────────────────

def compute_Crep(T, Trot, k2):
    """C_rep = k2 * ( (Trot/T)^2 - 1 )"""
    ratio = Trot / T
    return k2 * (ratio**2 - 1)

# For scans where k2 varies (k2 scan), compute C_rep with the corresponding k2
Crep_k2 = [compute_Crep(T, Trot, k2) for T, Trot, k2 in zip(T_k2, Trot_k2, k2_scan)]

# For scans where k2 is fixed at 0.5
k2_fixed = 0.5
Crep_k1 = [compute_Crep(T, Trot, k2_fixed) for T, Trot in zip(T_k1, Trot_k1)]
Crep_d  = [compute_Crep(T, Trot, k2_fixed) for T, Trot in zip(T_d, Trot_d)]
Crep_mu = [compute_Crep(T, Trot, k2_fixed) for T, Trot in zip(T_mu, Trot_mu)]
Crep_N  = [compute_Crep(T, Trot, k2_fixed) for T, Trot in zip(T_N, Trot_N)]
Crep_seed = [compute_Crep(T, Trot, k2_fixed) for T, Trot in zip(T_seed, Trot_seed)]

# ── Plot ───────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(2, 3, figsize=(15, 10))
axes = axes.flatten()

# 1. C_rep vs k2
ax = axes[0]
ax.plot(k2_scan, Crep_k2, 'o-', color='C0', markersize=6)
ax.axhline(0, color='gray', lw=0.5)
ax.set_xlabel('$k_2$')
ax.set_ylabel('$C_{\\mathrm{rep}}$')
ax.set_title('$C_{\\mathrm{rep}}$ vs $k_2$ ($k_1=0.5$, $N=4$)')
ax.grid(True, alpha=0.3)

# 2. C_rep vs k1
ax = axes[1]
ax.plot(k1_scan, Crep_k1, 'o-', color='C1', markersize=6)
ax.axhline(0, color='gray', lw=0.5)
ax.set_xlabel('$k_1$')
ax.set_ylabel('$C_{\\mathrm{rep}}$')
ax.set_title('$C_{\\mathrm{rep}}$ vs $k_1$ ($k_2=0.5$, $N=4$)')
ax.grid(True, alpha=0.3)

# 3. C_rep vs d_col
ax = axes[2]
ax.plot(d_scan, Crep_d, 'o-', color='C2', markersize=6)
ax.axhline(0, color='gray', lw=0.5)
ax.set_xlabel('$d_{\\mathrm{col}}$')
ax.set_ylabel('$C_{\\mathrm{rep}}$')
ax.set_title('$C_{\\mathrm{rep}}$ vs $d_{\\mathrm{col}}$ ($k_1=k_2=0.5$, $N=4$)')
ax.grid(True, alpha=0.3)

# 4. C_rep vs mu
ax = axes[3]
ax.plot(mu_scan, Crep_mu, 'o-', color='C3', markersize=6)
ax.axhline(0, color='gray', lw=0.5)
ax.set_xlabel('$\\mu$')
ax.set_ylabel('$C_{\\mathrm{rep}}$')
ax.set_title('$C_{\\mathrm{rep}}$ vs $\\mu$ ($k_1=k_2=0.5$, $N=4$)')
ax.grid(True, alpha=0.3)

# 5. C_rep vs N
ax = axes[4]
ax.plot(N_scan, Crep_N, 'o-', color='C4', markersize=6)
ax.axhline(0, color='gray', lw=0.5)
ax.set_xlabel('$N$')
ax.set_ylabel('$C_{\\mathrm{rep}}$')
ax.set_title('$C_{\\mathrm{rep}}$ vs $N$ ($k_1=k_2=0.5$)')
ax.grid(True, alpha=0.3)

# 6. C_rep vs seed
ax = axes[5]
ax.plot(seed_scan, Crep_seed, 'o-', color='C5', markersize=6)
ax.axhline(0, color='gray', lw=0.5)
ax.set_xlabel('seed')
ax.set_ylabel('$C_{\\mathrm{rep}}$')
ax.set_title('$C_{\\mathrm{rep}}$ vs seed ($N=4$, $k_1=k_2=0.5$)')
ax.grid(True, alpha=0.3)

fig.suptitle('$C_{\\mathrm{rep}} = k_2\\left(\\left(\\frac{T_{\\mathrm{rot}}}{T_{\\mathrm{breath}}}\\right)^2 - 1\\right)$ — Repulsive stiffness contribution to breathing frequency', fontsize=14)
fig.tight_layout(rect=[0, 0, 1, 0.95])

out = '/home/orangepi/repo/phd-notes/fencing/fencing_simulation/3d_breathing/crep_scan.png'
fig.savefig(out, dpi=150, bbox_inches='tight')
print(f'Saved to {out}')

# ── Print values ────────────────────────────────────────────────────────────

print("\n=== C_rep values ===")
print(f"\nk2 scan (k1=0.5):")
for k2, T, Trot, C in zip(k2_scan, T_k2, Trot_k2, Crep_k2):
    print(f"  k2={k2:.2f}  T={T:.2f}s  Trot={Trot:.2f}s  C_rep={C:.3f}  C_rep/k2={C/k2:.3f}")

print(f"\nk1 scan (k2=0.5):")
for k1, T, Trot, C in zip(k1_scan, T_k1, Trot_k1, Crep_k1):
    print(f"  k1={k1:.2f}  T={T:.2f}s  Trot={Trot:.2f}s  C_rep={C:.3f}  C_rep/k2={C/k2_fixed:.3f}")

print(f"\nd_col scan (k1=k2=0.5):")
for d, T, Trot, C in zip(d_scan, T_d, Trot_d, Crep_d):
    print(f"  d={d:.0f}  T={T:.2f}s  Trot={Trot:.2f}s  C_rep={C:.3f}  C_rep/k2={C/k2_fixed:.3f}")

print(f"\nmu scan (k1=k2=0.5):")
for mu, T, Trot, C in zip(mu_scan, T_mu, Trot_mu, Crep_mu):
    print(f"  mu={mu:.0f}  T={T:.2f}s  Trot={Trot:.2f}s  C_rep={C:.3f}  C_rep/k2={C/k2_fixed:.3f}")

print(f"\nN scan (k1=k2=0.5):")
for N, T, Trot, C in zip(N_scan, T_N, Trot_N, Crep_N):
    print(f"  N={N}  T={T:.2f}s  Trot={Trot:.2f}s  C_rep={C:.3f}  C_rep/k2={C/k2_fixed:.3f}")

print(f"\nseed scan (N=4, k1=k2=0.5):")
for s, T, Trot, C in zip(seed_scan, T_seed, Trot_seed, Crep_seed):
    print(f"  seed={s}  T={T:.2f}s  Trot={Trot:.2f}s  C_rep={C:.3f}  C_rep/k2={C/k2_fixed:.3f}")