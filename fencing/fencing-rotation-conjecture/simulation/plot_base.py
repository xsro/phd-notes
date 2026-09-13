"""
Shared plotting utilities for the fencing rotation conjecture.

Provides common style helpers, colour palettes, and plot types so that
case-specific plotting scripts can focus on layout and annotation.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# ── Colour palette (Tol, colourblind-friendly) ──────────────────────────────
AGENT_COLORS = ["#4477AA", "#EE6677", "#228833", "#CCBB44", "#66CCEE"]

# ── Light theme ──────────────────────────────────────────────────────────────
TH = {
    "surface": "#fcfcfb",
    "primary": "#0b0b0b",
    "secondary": "#52514e",
    "muted": "#8a8880",
    "grid": "#e4e3df",
}


def style_axes(ax):
    """Apply the shared light theme to an axes."""
    ax.set_facecolor(TH["surface"])
    ax.grid(True, color=TH["grid"], lw=0.4, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(TH["grid"])
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=TH["secondary"], labelsize=7.5, length=3, width=0.6)


def figure(figsize=(10, 6)):
    """Create a themed figure with a single axes."""
    fig, ax = plt.subplots(figsize=figsize)
    fig.patch.set_facecolor(TH["surface"])
    style_axes(ax)
    return fig, ax


# ── Plot helpers ─────────────────────────────────────────────────────────────
def plot_trajectories(ax, pos_all, t_eval, d, mu, N, title="",
                      skip=5, show_collision=True, show_mu=True):
    """Plot agent trajectories in the x-y plane.

    Parameters
    ----------
    pos_all : ndarray, shape (n_frames, N, 2)
    t_eval : ndarray
    d, mu : float — collision and interaction radii
    N : int — number of agents
    skip : int — plot every `skip`-th position marker
    show_collision, show_mu : bool — draw reference circles
    """
    # Reference circles
    theta = np.linspace(0, 2 * np.pi, 200)
    if show_collision:
        ax.plot(d * np.cos(theta), d * np.sin(theta),
                color=TH["muted"], lw=1.0, ls="--", alpha=0.5,
                label=f"collision bound $d={d}$", zorder=1)
    if show_mu:
        ax.plot(mu * np.cos(theta), mu * np.sin(theta),
                color=TH["muted"], lw=0.6, ls=":", alpha=0.3,
                label=f"interaction range $\\mu={mu}$", zorder=1)

    # Trajectory lines
    for i in range(N):
        ax.plot(pos_all[:, i, 0], pos_all[:, i, 1],
                color=AGENT_COLORS[i], lw=0.8, alpha=0.5, zorder=2)

    # Position markers (every `skip` frames)
    for i in range(N):
        ax.scatter(pos_all[::skip, i, 0], pos_all[::skip, i, 1],
                   s=6, color=AGENT_COLORS[i], alpha=0.4, zorder=3,
                   edgecolors="none")

    # Final positions (larger, filled)
    for i in range(N):
        ax.scatter(pos_all[-1, i, 0], pos_all[-1, i, 1],
                   s=40, color=AGENT_COLORS[i], zorder=5,
                   edgecolors=TH["surface"], linewidths=0.8,
                   label=f"Agent {i} (final)")

    ax.set_aspect("equal")
    ax.set_xlabel("$x$", color=TH["secondary"], fontsize=9)
    ax.set_ylabel("$y$", color=TH["secondary"], fontsize=9)
    if title:
        ax.set_title(title, fontsize=10, color=TH["primary"],
                     fontweight="bold", loc="left", pad=8)

    # Legend (only agent labels)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(frameon=False, fontsize=7, loc="lower left",
                  labelcolor=TH["secondary"])


def plot_distances(ax, pos_all, t_eval, d, N, title=""):
    """Plot all pairwise distances over time."""
    pairs = [(i, j) for i in range(N) for j in range(i + 1, N)]
    colors = plt.cm.viridis(np.linspace(0.2, 0.9, len(pairs)))

    for idx, (i, j) in enumerate(pairs):
        dists = np.linalg.norm(pos_all[:, i] - pos_all[:, j], axis=1)
        ax.plot(t_eval, dists, color=colors[idx], lw=1.0, alpha=0.7,
                label=f"$d_{{{i}{j}}}$")

    ax.axhline(d, color=TH["muted"], lw=1.0, ls="--", alpha=0.6,
               label=f"$d = {d}$")
    ax.set_xlabel("$t$", color=TH["secondary"], fontsize=9)
    ax.set_ylabel("distance", color=TH["secondary"], fontsize=9)
    if title:
        ax.set_title(title, fontsize=10, color=TH["primary"],
                     fontweight="bold", loc="left", pad=8)
    ax.legend(frameon=False, fontsize=6.5, ncol=3,
              labelcolor=TH["secondary"])


def plot_angular_velocities(ax, pos_all, vel_all, t_eval, k2, N, title=""):
    """Plot angular velocity ω_i = (xi × dxi/dt) / ||xi||² for each agent."""
    omega_sqrt = np.sqrt(k2)
    for i in range(N):
        xi = pos_all[:, i]          # (n_frames, 2)
        dxi = vel_all[:, i]         # (n_frames, 2)
        # ω_i = (xi_x * dxi_y - xi_y * dxi_x) / ||xi||²
        cross = xi[:, 0] * dxi[:, 1] - xi[:, 1] * dxi[:, 0]
        norm2 = np.sum(xi ** 2, axis=1)
        omega_i = np.where(norm2 > 1e-12, cross / norm2, 0.0)
        ax.plot(t_eval, omega_i, color=AGENT_COLORS[i], lw=1.0, alpha=0.7,
                label=f"$\\omega_{i}$")

    ax.axhline(omega_sqrt, color=TH["muted"], lw=1.0, ls="--", alpha=0.6,
               label=f"$\\sqrt{{k_2}} = {omega_sqrt:.3f}$")
    ax.set_xlabel("$t$", color=TH["secondary"], fontsize=9)
    ax.set_ylabel("$\\omega_i$", color=TH["secondary"], fontsize=9)
    if title:
        ax.set_title(title, fontsize=10, color=TH["primary"],
                     fontweight="bold", loc="left", pad=8)
    ax.legend(frameon=False, fontsize=6.5, ncol=3,
              labelcolor=TH["secondary"])


def save_figure(fig, path):
    """Save figure with tight layout and 150 dpi."""
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"  → {path}")
    plt.close(fig)