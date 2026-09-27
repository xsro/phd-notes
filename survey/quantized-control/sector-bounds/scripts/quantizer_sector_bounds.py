"""Plot sector bounds for logarithmic and general quantizers.

Based on Fu & Xie, "The sector bound approach to quantized feedback control," IEEE TAC 2005.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import os
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# ============================================================
# Helper: logarithmic quantizer
# ============================================================
def log_quantizer(v, rho, u0=1.0):
    """Logarithmic quantizer with density rho in (0,1).

    Uses Fu & Xie (2005) exact decision boundaries (no deadzone):
      For v > 0:  f(v) = u_i  if  v ∈ [u_i·(1+ρ)/2,  u_i·(1+ρ)/(2ρ))
      where  u_i = ρ^i · u0,  i = 0, ±1, ±2, ...
      i → +∞ covers v → 0⁺,  i → −∞ covers v → +∞.

    Returns f(v) and the sector parameter Δ(v) = (f(v)-v)/v.
    """
    if v == 0:
        return 0.0, 0.0
    # Normalized input: α = 2|v| / ((1+ρ)·u₀) ∈ [ρ^i, ρ^{i-1})
    # Since ρ<1 ⇒ log(ρ)<0, we have:  i = ceil(log(α)/log(ρ))
    abs_v = abs(v)
    alpha = 2 * abs_v / ((1 + rho) * u0)
    i = int(np.ceil(np.log(alpha) / np.log(rho)))
    f_v = np.sign(v) * u0 * rho**i
    delta_v = (f_v - v) / v
    return f_v, delta_v


def log_quantizer_vectorized(v_vals, rho, u0=1.0):
    """Vectorized version."""
    f_vals = np.zeros_like(v_vals)
    delta_vals = np.zeros_like(v_vals)
    for idx, v in enumerate(v_vals):
        f_vals[idx], delta_vals[idx] = log_quantizer(v, rho, u0)
    return f_vals, delta_vals


# ============================================================
# Helper: uniform quantizer (for §3 general quantizer)
# ============================================================
def uniform_quantizer(v, step=0.5):
    """Uniform quantizer with given step size."""
    if v == 0:
        return 0.0, 0.0
    f_v = step * np.round(v / step)
    delta_v = (f_v - v) / v if v != 0 else 0.0
    return f_v, delta_v


# ============================================================
# Figure 1: Logarithmic Quantizer — input-output characteristic
# ============================================================
def plot_log_quantizer_characteristic():
    rho = 0.5  # quantization density
    u0 = 1.0
    v_vals = np.linspace(-10, 10, 2000)
    f_vals, delta_vals = log_quantizer_vectorized(v_vals, rho, u0)
    delta_bound = (1 - rho) / (1 + rho)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # ========= Left: input-output =========
    ax1.plot(v_vals, v_vals, 'k--', alpha=0.3, label='y = v (ideal)')
    ax1.plot(v_vals, f_vals, 'b-', linewidth=1.2, label=f'f(v) (ρ={rho})')
    ax1.set_xlabel('v')
    ax1.set_ylabel('f(v)')
    ax1.set_title('Logarithmic Quantizer: Input-Output Characteristic')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([-2, 8])
    ax1.set_ylim([-8, 8])

    # ========= Right: sector parameter Δ(v) =========
    # Fill the sector [-δ, δ] for all v ≠ 0 (no deadzone)
    ax2.fill_between(v_vals, -delta_bound, delta_bound,
                     alpha=0.12, color='green',
                     label=f'Sector [−δ, δ] = [{-delta_bound:.3f}, {delta_bound:.3f}]')
    ax2.plot(v_vals, delta_vals, 'r-', linewidth=1.2)
    # Sector boundary lines
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.axhline(y=delta_bound, color='g', linestyle=':', alpha=0.7)
    ax2.axhline(y=-delta_bound, color='g', linestyle=':', alpha=0.7)
    ax2.set_xlabel('v')
    ax2.set_ylabel('Δ(v) = (f(v)−v)/v')
    ax2.set_title(f'Sector Parameter Δ(v): globally bounded by δ = {delta_bound:.3f}')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([-2, 8])
    ax2.set_ylim([-1.5 * delta_bound, 1.5 * delta_bound])

    plt.tight_layout()
    return fig


# ============================================================
# Figure 2: General Quantizer — asymmetric sector bound
# ============================================================
def plot_general_quantizer_sector():
    """Plot a general (non-logarithmic) quantizer with asymmetric sector."""
    step = 1.0
    saturation = 5.0

    def general_quantizer(v):
        if v > saturation:
            return saturation, (saturation - v) / v
        elif v < -saturation:
            return -saturation, (-saturation - v) / v
        else:
            f_v = step * np.round(v / step)
            if v != 0:
                return f_v, (f_v - v) / v
            else:
                return 0.0, 0.0

    v_vals = np.linspace(-8, 8, 2000)
    f_vals = np.array([general_quantizer(v)[0] for v in v_vals])
    delta_vals = np.array([general_quantizer(v)[1] for v in v_vals])

    nonzero_mask = np.abs(v_vals) > 0.01
    delta_nonzero = delta_vals[nonzero_mask]
    delta_minus = np.min(delta_nonzero)
    delta_plus = np.max(delta_nonzero)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # Left: input-output characteristic
    ax1.plot(v_vals, v_vals, 'k--', alpha=0.3, label='y = v (ideal)')
    ax1.plot(v_vals, f_vals, 'b-', linewidth=1.2, label='f(v) (uniform)')
    ax1.set_xlabel('v')
    ax1.set_ylabel('f(v)')
    ax1.set_title('General (Uniform) Quantizer: Input-Output')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([-8, 8])
    ax1.set_ylim([-8, 8])

    # Right: sector parameter
    ax2.plot(v_vals, delta_vals, 'r-', linewidth=1.2)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.axhline(y=delta_plus, color='g', linestyle=':', alpha=0.7,
                label=f'δ⁺ = {delta_plus:.3f}')
    ax2.axhline(y=delta_minus, color='g', linestyle=':', alpha=0.7,
                label=f'δ⁻ = {delta_minus:.3f}')
    ax2.fill_between(v_vals, delta_minus, delta_plus, alpha=0.1, color='green',
                     label=f'Sector [δ⁻, δ⁺] = [{delta_minus:.3f}, {delta_plus:.3f}]')
    ax2.set_xlabel('v')
    ax2.set_ylabel('Δ(v)')
    ax2.set_title('Sector Parameter for General Quantizer')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([-8, 8])
    ax2.set_ylim([delta_minus - 0.5, delta_plus + 0.5])

    plt.tight_layout()
    return fig


# ============================================================
# Main
# ============================================================
if __name__ == '__main__':
    figs = [
        ('log_quantizer_characteristic', plot_log_quantizer_characteristic),
        ('general_quantizer_sector', plot_general_quantizer_sector),
    ]

    for name, func in figs:
        fig = func()
        path = os.path.join(_SCRIPT_DIR, f'{name}.png')
        fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f'Saved: {path}')
        plt.close(fig)

    print('\nAll figures generated successfully.')