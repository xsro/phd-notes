"""
Plot C_rep from fine scan data with more data points and finer resolution.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Load fine scan data
d = np.load('/home/orangepi/repo/phd-notes/fencing/fencing_simulation/3d_breathing/data/crep_fine.npz')

# Extract
k2_data = d['k2']    # (k2, T, Trot, Crep/k2, peak, sigma)
k1_data = d['k1']
d_data  = d['d']
mu_data = d['mu']
N_data  = d['N']
seed_data = d['seed']

fig, axes = plt.subplots(2, 3, figsize=(16, 10))
axes = axes.flatten()

# 1. C_rep vs k2
ax = axes[0]
ax.plot(k2_data[:,0], k2_data[:,3], 'o-', color='C0', markersize=7, linewidth=1.5)
ax.axhline(0, color='gray', lw=0.5, linestyle='--')
ax.set_xlabel('$k_2$', fontsize=12)
ax.set_ylabel('$C_{\\mathrm{rep}}/k_2$', fontsize=12)
ax.set_title('(a) $C_{\\mathrm{rep}}/k_2$ vs $k_2$\n($k_1=0.5$, $N=4$, $d=5$, $\\mu=9$)', fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(0.2, 0.75)

# 2. C_rep vs k1
ax = axes[1]
ax.plot(k1_data[:,0], k1_data[:,3], 'o-', color='C1', markersize=7, linewidth=1.5)
ax.axhline(0, color='gray', lw=0.5, linestyle='--')
ax.set_xlabel('$k_1$', fontsize=12)
ax.set_ylabel('$C_{\\mathrm{rep}}/k_2$', fontsize=12)
ax.set_title('(b) $C_{\\mathrm{rep}}/k_2$ vs $k_1$\n($k_2=0.5$, $N=4$, $d=5$, $\\mu=9$)', fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(0.1, 0.75)

# 3. C_rep vs d_col
ax = axes[2]
ax.plot(d_data[:,0], d_data[:,3], 'o-', color='C2', markersize=7, linewidth=1.5)
ax.axhline(0, color='gray', lw=0.5, linestyle='--')
ax.set_xlabel('$d_{\\mathrm{col}}$', fontsize=12)
ax.set_ylabel('$C_{\\mathrm{rep}}/k_2$', fontsize=12)
ax.set_title('(c) $C_{\\mathrm{rep}}/k_2$ vs $d_{\\mathrm{col}}$\n($k_1=k_2=0.5$, $N=4$, $\\mu=9$)', fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(2.8, 7.2)

# 4. C_rep vs mu
ax = axes[3]
ax.plot(mu_data[:,0], mu_data[:,3], 'o-', color='C3', markersize=7, linewidth=1.5)
ax.axhline(0, color='gray', lw=0.5, linestyle='--')
ax.set_xlabel('$\\mu$', fontsize=12)
ax.set_ylabel('$C_{\\mathrm{rep}}/k_2$', fontsize=12)
ax.set_title('(d) $C_{\\mathrm{rep}}/k_2$ vs $\\mu$\n($k_1=k_2=0.5$, $N=4$, $d=5$)', fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(5.8, 11.8)

# 5. C_rep vs N
ax = axes[4]
ax.plot(N_data[:,0], N_data[:,3], 'o-', color='C4', markersize=7, linewidth=1.5)
ax.axhline(0, color='gray', lw=0.5, linestyle='--')
ax.set_xlabel('$N$', fontsize=12)
ax.set_ylabel('$C_{\\mathrm{rep}}/k_2$', fontsize=12)
ax.set_title('(e) $C_{\\mathrm{rep}}/k_2$ vs $N$\n($k_1=k_2=0.5$, $d=5$, $\\mu=9$)', fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(2.5, 6.5)

# 6. C_rep vs seed
ax = axes[5]
ax.plot(seed_data[:,0], seed_data[:,3], 'o-', color='C5', markersize=7, linewidth=1.5)
ax.axhline(0, color='gray', lw=0.5, linestyle='--')
ax.set_xlabel('seed', fontsize=12)
ax.set_ylabel('$C_{\\mathrm{rep}}/k_2$', fontsize=12)
ax.set_title('(f) $C_{\\mathrm{rep}}/k_2$ vs seed\n($N=4$, $k_1=k_2=0.5$, $d=5$, $\\mu=9$)', fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(-0.5, 9.5)

fig.suptitle('$C_{\\mathrm{rep}}/k_2 = \\left(\\frac{T_{\\mathrm{rot}}}{T_{\\mathrm{breath}}}\\right)^2 - 1$  —  Fine scan (T_MAX=120s, DT=0.05s, resolution=0.0083Hz)',
             fontsize=13, y=1.01)
fig.tight_layout(rect=[0, 0, 1, 0.95])

out = '/home/orangepi/repo/phd-notes/fencing/fencing_simulation/3d_breathing/crep_scan_fine.png'
fig.savefig(out, dpi=150, bbox_inches='tight')
print(f'Saved to {out}')

# ── Print summary statistics ────────────────────────────────────────────────
print("\n=== Summary ===")
print(f"k2 scan: C_rep/k2 range [{k2_data[:,3].min():.3f}, {k2_data[:,3].max():.3f}], mean={k2_data[:,3].mean():.3f}")
print(f"k1 scan: C_rep/k2 range [{k1_data[:,3].min():.3f}, {k1_data[:,3].max():.3f}]")
print(f"d scan:  C_rep/k2 range [{d_data[:,3].min():.3f}, {d_data[:,3].max():.3f}]")
print(f"mu scan: C_rep/k2 range [{mu_data[:,3].min():.3f}, {mu_data[:,3].max():.3f}]")
print(f"N scan:  C_rep/k2 range [{N_data[:,3].min():.3f}, {N_data[:,3].max():.3f}]")
print(f"seed scan: C_rep/k2 range [{seed_data[:,3].min():.3f}, {seed_data[:,3].max():.3f}], unique values: {np.unique(np.round(seed_data[:,3],3))}")