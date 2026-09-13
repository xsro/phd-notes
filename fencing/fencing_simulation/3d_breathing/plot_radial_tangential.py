"""
Plot radial vs tangential frequency decomposition from scan_radial_tangential.py.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Data from the scan output
# Format: (param, T_rad, T_tan, T_pd, T_rot, f_rad/f_tan, pr_rad, pr_tan, sigma)

# k2 scan
k2_data = [
    (0.25, 10.000, 12.000, 10.000, 12.566, 1.200, 0.259, 0.351, 0.605),
    (0.30,  6.000, 12.000,  6.000, 11.471, 2.000, 0.602, 0.839, 0.840),
    (0.35,  5.455, 10.000,  5.455, 10.621, 1.833, 0.639, 0.472, 0.760),
    (0.40,  5.000, 10.000,  5.000,  9.935, 2.000, 0.428, 0.922, 0.886),
    (0.45,  5.000, 10.000,  5.000,  9.366, 2.000, 0.523, 0.667, 0.508),
    (0.50,  4.615,  8.571,  4.615,  8.886, 1.857, 0.824, 0.679, 0.640),
    (0.55,  4.286,  8.571,  4.286,  8.472, 2.000, 0.628, 0.948, 0.396),
    (0.60,  4.286,  8.571,  4.286,  8.112, 2.000, 0.505, 0.592, 0.011),
    (0.65,  4.000,  7.500,  4.000,  7.793, 1.875, 0.838, 0.676, 0.324),
    (0.70,  3.750,  7.500,  3.750,  7.510, 2.000, 0.470, 0.962, 0.879),
]

k1_data = [
    (0.15, 4.615, 8.571, 4.615, 8.886, 1.857, 0.446, 0.545, 0.846),
    (0.20, 4.615, 8.571, 4.615, 8.886, 1.857, 0.290, 0.754, 0.862),
    (0.25, 4.615, 8.571, 4.615, 8.886, 1.857, 0.463, 0.800, 0.705),
    (0.30, 4.615, 8.571, 4.615, 8.886, 1.857, 0.566, 0.793, 0.038),
    (0.35, 4.615, 8.571, 4.615, 8.886, 1.857, 0.649, 0.790, 0.907),
    (0.40, 4.615, 8.571, 4.615, 8.886, 1.857, 0.732, 0.780, 0.178),
    (0.45, 4.615, 8.571, 4.615, 8.886, 1.857, 0.751, 0.768, 0.966),
    (0.50, 4.615, 8.571, 4.615, 8.886, 1.857, 0.824, 0.679, 0.640),
    (0.55, 4.615, 8.571, 4.615, 8.886, 1.857, 0.818, 0.635, 0.098),
    (0.60, 4.615, 8.571, 4.615, 8.886, 1.857, 0.765, 0.574, 0.707),
    (0.65, 4.615, 8.571, 4.615, 8.886, 1.857, 0.598, 0.519, 0.460),
    (0.70, 7.500, 7.500, 7.500, 8.886, 1.000, 0.359, 0.484, 0.627),
    (0.75, 8.571, 8.571, 8.571, 8.886, 1.000, 0.227, 0.431, 0.633),
]

d_data = [
    (3.0, 4.615, 8.571, 4.615, 8.886, 1.857, 0.579, 0.793, 0.175),
    (3.5, 4.615, 8.571, 4.615, 8.886, 1.857, 0.657, 0.774, 0.490),
    (4.0, 4.615, 8.571, 4.615, 8.886, 1.857, 0.577, 0.725, 0.767),
    (4.5, 4.615, 8.571, 4.615, 8.886, 1.857, 0.777, 0.707, 0.915),
    (5.0, 4.615, 8.571, 4.615, 8.886, 1.857, 0.824, 0.679, 0.640),
    (5.5, 6.000, 8.571, 4.615, 8.886, 1.429, 0.122, 0.444, 0.272),
    (6.0, 4.615, 8.571, 4.615, 8.886, 1.857, 0.656, 0.670, 0.640),
    (6.5, 4.615, 8.571, 4.615, 8.886, 1.857, 0.424, 0.540, 0.283),
    (7.0, 7.500, 7.500, 7.500, 8.886, 1.000, 0.269, 0.461, 0.826),
]

mu_data = [
    (6.0,  7.500, 7.500, 7.500, 8.886, 1.000, 0.324, 0.526, 0.696),
    (6.5,  7.500, 7.500, 7.500, 8.886, 1.000, 0.290, 0.407, 0.527),
    (7.0,  4.615, 8.571, 4.615, 8.886, 1.857, 0.540, 0.628, 0.929),
    (7.5,  4.615, 8.571, 4.615, 8.886, 1.857, 0.273, 0.447, 0.833),
    (8.0,  4.615, 8.571, 4.615, 8.886, 1.857, 0.818, 0.681, 0.580),
    (8.5,  4.615, 8.571, 4.615, 8.886, 1.857, 0.826, 0.676, 0.075),
    (9.0,  4.615, 8.571, 4.615, 8.886, 1.857, 0.824, 0.679, 0.640),
    (9.5,  4.615, 8.571, 4.615, 8.886, 1.857, 0.812, 0.684, 0.990),
    (10.0, 4.615, 8.571, 4.615, 8.886, 1.857, 0.799, 0.688, 0.883),
]

N_data = [
    (3, 6.000, 8.571, 6.000, 8.886, 1.429, 0.299, 0.815, 0.000),
    (4, 4.615, 8.571, 4.615, 8.886, 1.857, 0.824, 0.679, 0.640),
    (5, 7.500, 7.500, 7.500, 8.886, 1.000, 0.284, 0.293, 0.582),
    (6, 4.615, 8.571, 4.615, 8.886, 1.857, 0.666, 0.589, 0.892),
]

fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# ── Panel 1: T_rad and T_tan vs k2 ──
ax = axes[0, 0]
k2 = [d[0] for d in k2_data]
T_rad = [d[1] for d in k2_data]
T_tan = [d[2] for d in k2_data]
T_rot = [d[4] for d in k2_data]
ax.plot(k2, T_rad, 'o-', color='C0', label='$T_{\\mathrm{rad}}$ (breathing)', markersize=6)
ax.plot(k2, T_tan, 's-', color='C1', label='$T_{\\mathrm{tan}}$ (tangential)', markersize=6)
ax.plot(k2, T_rot, '--', color='C2', label='$T_{\\mathrm{rot}} = 2\\pi/\\sqrt{k_2}$', alpha=0.7)
ax.set_xlabel('$k_2$')
ax.set_ylabel('Period (s)')
ax.set_title('(a) Radial vs Tangential Period vs $k_2$')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# ── Panel 2: f_rad/f_tan vs each parameter ──
ax = axes[0, 1]
ax.plot([d[0] for d in k2_data], [d[5] for d in k2_data], 'o-', color='C0', label='$k_2$', markersize=5)
ax.plot([d[0] for d in k1_data], [d[5] for d in k1_data], 's-', color='C1', label='$k_1$', markersize=5)
ax.plot([d[0] for d in d_data], [d[5] for d in d_data], '^-', color='C2', label='$d_{\\mathrm{col}}$', markersize=5)
ax.plot([d[0] for d in mu_data], [d[5] for d in mu_data], 'v-', color='C3', label='$\\mu$', markersize=5)
ax.plot([d[0] for d in N_data], [d[5] for d in N_data], 'D-', color='C4', label='$N$', markersize=5)
ax.axhline(1.857, color='gray', lw=0.8, linestyle=':', alpha=0.7)
ax.axhline(1.0, color='red', lw=0.8, linestyle=':', alpha=0.7, label='locked ($f_{\\mathrm{rad}}=f_{\\mathrm{tan}}$)')
ax.set_xlabel('Parameter value')
ax.set_ylabel('$f_{\\mathrm{rad}} / f_{\\mathrm{tan}}$')
ax.set_title('(b) Frequency Ratio vs All Parameters')
ax.legend(fontsize=7, loc='best')
ax.grid(True, alpha=0.3)

# ── Panel 3: T_rad/T_rot and T_tan/T_rot vs k2 ──
ax = axes[1, 0]
ax.plot(k2, [d[1]/d[4] for d in k2_data], 'o-', color='C0', label='$T_{\\mathrm{rad}}/T_{\\mathrm{rot}}$', markersize=6)
ax.plot(k2, [d[2]/d[4] for d in k2_data], 's-', color='C1', label='$T_{\\mathrm{tan}}/T_{\\mathrm{rot}}$', markersize=6)
ax.axhline(0.54, color='C0', lw=0.5, linestyle='--', alpha=0.5)
ax.axhline(0.96, color='C1', lw=0.5, linestyle='--', alpha=0.5)
ax.set_xlabel('$k_2$')
ax.set_ylabel('Ratio to $T_{\\mathrm{rot}}$')
ax.set_title('(c) Normalized Periods vs $k_2$')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# ── Panel 4: sigma vs f_rad/f_tan ──
ax = axes[1, 1]
all_ratio = [d[5] for d in k2_data + k1_data + d_data + mu_data + N_data]
all_sigma = [d[8] for d in k2_data + k1_data + d_data + mu_data + N_data]
all_params = (['$k_2$']*len(k2_data) + ['$k_1$']*len(k1_data) +
              ['$d$']*len(d_data) + ['$\\mu$']*len(mu_data) + ['$N$']*len(N_data))
colors = {'$k_2$': 'C0', '$k_1$': 'C1', '$d$': 'C2', '$\\mu$': 'C3', '$N$': 'C4'}
for p, r, s in zip(all_params, all_ratio, all_sigma):
    ax.plot(r, s, 'o', color=colors[p], markersize=6, alpha=0.7)
ax.axvline(1.857, color='gray', lw=0.8, linestyle=':', alpha=0.7, label='breathing ($f_r/f_t \\approx 1.86$)')
ax.axvline(1.0, color='red', lw=0.8, linestyle=':', alpha=0.7, label='locked ($f_r/f_t = 1.0$)')
ax.set_xlabel('$f_{\\mathrm{rad}} / f_{\\mathrm{tan}}$')
ax.set_ylabel('$\\sigma_3/\\sigma_1$ (planarity)')
ax.set_title('(d) Planarity vs Frequency Ratio')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

fig.suptitle('Radial (breathing) vs Tangential (rotation) Motion Decomposition',
             fontsize=14, y=1.01)
fig.tight_layout(rect=[0, 0, 1, 0.95])

out = '/home/orangepi/repo/phd-notes/fencing/fencing_simulation/3d_breathing/radial_tangential.png'
fig.savefig(out, dpi=150, bbox_inches='tight')
print(f'Saved to {out}')