"""Double integrator SMC — quantizer on s vs quantizer on u.

Approach A: quantizer replaces sign(s)   — 量化器作用于滑模面
  A1 — Ideal sign (tanh):     u = -c·x₂ - k·tanh(μs)
  A2 — Log quantizer on s:    u = -c·x₂ - k·q_log(s)
  A3 — Ceil quantizer on s:   u = -c·x₂ - k·sgn(s)·ceil(|s|)
  A4 — Floor quantizer on s:  u = -c·x₂ - k·sgn(s)·floor(|s|)

Approach B: quantizer scales the control — 量化器作用于控制信号
  B1 — Ideal (no quantizer):  u = -c·x₂ - k·tanh(μs)
  B2 — Log quantizer on u:    u = q_log(-c·x₂ - k·tanh(μs))
  B3 — Floor quantizer on u:  u = sgn(u)·floor(|u|) where u = -c·x₂ - k·tanh(μs)
  B4 — Ceil quantizer on u:   u = sgn(u)·ceil(|u|) where u = -c·x₂ - k·tanh(μs)

System:  ẋ₁ = x₂,  ẋ₂ = u + d(t),  d(t) = A·sin(ωt)
Sliding surface:  s = c·x₁ + x₂
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ============================================================
# Quantizers
# ============================================================
def log_quantizer(v, rho=0.5, u0=1.0):
    if v == 0: return 0.0
    alpha = 2 * abs(v) / ((1 + rho) * u0)
    i = int(np.ceil(np.log(alpha) / np.log(rho)))
    return np.sign(v) * u0 * rho**i

def ceil_quantizer(v):
    if v == 0: return 0.0
    return np.sign(v) * np.ceil(np.abs(v))

def floor_quantizer(v):
    """sgn(v)·floor(|v|) — rounds toward zero; has deadzone for |v|<1."""
    if v == 0: return 0.0
    f = np.floor(np.abs(v))
    if f == 0: return 0.0  # deadzone
    return np.sign(v) * f

def disturbance(t, A, omega):
    return A * np.sin(omega * t)

# ============================================================
# Simulation
# ============================================================
def simulate_approach(x0, approach, c=1.0, k=2.0, rho=0.5, mu=50.0,
                      T=12.0, dt=2e-5, dist_params=None):
    steps = int(T / dt)
    t_arr = np.linspace(0, T, steps)

    algos = {
        'A': ['A1_ideal', 'A2_log', 'A3_ceil', 'A4_floor'],
        'B': ['B1_ideal', 'B2_log', 'B3_floor', 'B4_ceil'],
    }[approach]

    res = {n: {'x': np.zeros((steps,2)), 's': np.zeros(steps), 'u': np.zeros(steps)}
           for n in algos}
    for n in algos: res[n]['x'][0] = x0

    for i in range(steps - 1):
        d = disturbance(t_arr[i], **dist_params) if dist_params else 0.0
        for name in algos:
            x1, x2 = res[name]['x'][i]
            s = c * x1 + x2

            if name in ['A1_ideal', 'B1_ideal']:
                u_val = -c * x2 - k * np.tanh(mu * s)
            elif name == 'A2_log':
                u_val = -c * x2 - k * log_quantizer(s, rho)
            elif name == 'A3_ceil':
                u_val = -c * x2 - k * ceil_quantizer(s)
            elif name == 'A4_floor':
                u_val = -c * x2 - k * floor_quantizer(s)
            elif name == 'B2_log':
                u_val = log_quantizer(-c * x2 - k * np.tanh(mu * s), rho)
            elif name == 'B3_floor':
                u_raw = -c * x2 - k * np.tanh(mu * s)
                u_val = floor_quantizer(u_raw)
            else:  # B4_ceil
                u_raw = -c * x2 - k * np.tanh(mu * s)
                u_val = ceil_quantizer(u_raw)

            res[name]['x'][i+1] = [x1 + dt * x2, x2 + dt * (u_val + d)]
            res[name]['s'][i] = s
            res[name]['u'][i] = u_val

    for n in algos:
        res[n]['s'][-1] = res[n]['s'][-2]
        res[n]['u'][-1] = res[n]['u'][-2]
    return t_arr, res


# ============================================================
# Plot
# ============================================================
def plot_approach(t, res, algos, labels, c, title, filename):
    n = len(algos)
    sample = 50
    idx = slice(0, len(t), sample)
    t_p = t[idx]

    fig, axes = plt.subplots(3, n, figsize=(5*n+1, 10))
    if n <= 2:
        axes = axes.reshape(3, -1)

    for col, name in enumerate(algos):
        lbl, clr, ls, lw = labels[name]
        d = res[name]
        x_p, s_p, u_p = d['x'][idx], d['s'][idx], d['u'][idx]

        ax = axes[0, col]
        ax.plot(t_p, x_p[:,0], color=clr, ls=ls, lw=lw, label='x₁')
        ax.plot(t_p, x_p[:,1], color=clr, ls=':', lw=lw*0.8, alpha=0.6, label='x₂')
        ax.set_title(lbl, fontsize=11); ax.set_ylabel('State')
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

        ax = axes[1, col]
        ax.plot(t_p, s_p, color=clr, ls=ls, lw=lw)
        ax.axhline(y=0, color='k', ls='--', alpha=0.3)
        ax.set_ylabel('s'); ax.grid(True, alpha=0.3)

        ax = axes[2, col]
        ax.plot(t_p, u_p, color=clr, ls=ls, lw=lw*0.6)
        ax.set_xlabel('Time'); ax.set_ylabel('u'); ax.grid(True, alpha=0.3)

    fig.suptitle(title, fontsize=13, y=1.01)
    plt.tight_layout()
    fig.savefig(os.path.join(_SCRIPT_DIR, filename), dpi=150, bbox_inches='tight')
    print(f'Saved: {filename}')
    plt.close(fig)


# ============================================================
# Main
# ============================================================
if __name__ == '__main__':
    c = 1.0; rho = 0.5
    x0 = np.array([3.0, 0.0]); T = 12.0; dt = 2e-5

    labels = {
        'A1_ideal':  ('Ideal tanh',        '#0072bd', '-',  1.2),
        'A2_log':    ('Log on s',          '#77ac30', '-',  1.2),
        'A3_ceil':   ('Ceil on s',         '#d95319', '-',  2.2),
        'A4_floor':  ('Floor on s',        '#7f2d9e', '--', 2.2),
        'B1_ideal':  ('Ideal tanh',        '#0072bd', '-',  1.2),
        'B2_log':    ('Log on u',          '#edb120', '-',  2.2),
        'B3_floor':  ('Floor on u',        '#7f2d9e', '--', 2.2),
        'B4_ceil':   ('Ceil on u',         '#d95319', '-',  2.2),
    }

    dist_weak = {'A':0.3, 'omega':2.0}
    dist_strong = {'A':0.8, 'omega':3.0}
    scenarios = [
        (None, 'none'),
        (dist_weak, 'weak'),
        (dist_strong, 'strong'),
    ]
    dist_labels = {
        'none': 'No disturbance',
        'weak': 'd(t) = 0.3·sin(2t)',
        'strong': 'd(t) = 0.8·sin(3t)',
    }

    for approach, algos, prefix in [('A', ['A1_ideal','A2_log','A3_ceil','A4_floor'], 'appA'),
                                     ('B', ['B1_ideal','B2_log','B3_floor','B4_ceil'], 'appB')]:
        print(f'\n=== Approach {approach} ===')
        for dist_params, key in scenarios:
            for k in [1.5, 3.0]:
                t, res = simulate_approach(x0, approach, c=c, k=k, rho=rho, T=T, dt=dt,
                                           dist_params=dist_params)
                plot_approach(t, res, algos, labels, c,
                              title=f'Approach {approach} — {dist_labels[key]} (k={k})',
                              filename=f'{prefix}_{key}_k{k:.1f}.png')

    # Comparison table
    print('\n=== Steady-state |s| ===')
    header = f'{"Appr":<5} {"Scenario":<10} {"k":<6}'
    for name in ['A1_ideal','A2_log','A3_ceil','A4_floor','B1_ideal','B2_log','B3_floor','B4_ceil']:
        header += f' {labels[name][0]:<14}'
    print(header)
    print('-' * len(header))

    for approach, algos, label in [('A', ['A1_ideal','A2_log','A3_ceil','A4_floor'], 'On s'),
                                    ('B', ['B1_ideal','B2_log','B3_floor','B4_ceil'], 'On u')]:
        for dist_params, key in scenarios:
            for k in [1.5, 3.0]:
                t, res = simulate_approach(x0, approach, c=c, k=k, rho=rho, T=T, dt=dt,
                                           dist_params=dist_params)
                vals = [f'{np.abs(res[n]["s"][-int(len(t)/4):]).mean():.3e}' for n in algos]
                print(f' {label:<4} {key:<10} {k:<6}', *[f'{v:<14}' for v in vals])

    print('\nDone.')