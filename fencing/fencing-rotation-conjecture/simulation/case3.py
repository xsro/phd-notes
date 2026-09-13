"""
Animate 3 periods of the breathing limit cycle (steady state only, no trails).

Produces:
    figures/breathing_cycle_agents.gif   — looping GIF

Run from the repository root:
    ../.venv/bin/python simulation/case3.py
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from scipy.integrate import solve_ivp

# ── Simulation parameters ────────────────────────────────────────────────────
d, mu, k1, k2 = 5.0, 9.0, 0.5, 0.5

XI0 = np.array([[-10.584563, -12.532987],
                [-5.382648,   5.904905],
                [-0.177347,   9.17592],
                [-7.352506,  -4.805249],
                [-6.288338,  12.43583]])
VT0 = np.array([[3.528048, -1.274511],
                [-0.511988, -1.485443],
                [1.972071, -3.679895],
                [-3.460489, -0.767699],
                [-2.039248,  2.7618]])

N = len(XI0)

# ── ODE machinery ───────────────────────────────────────────────────────────
def alpha(s):
    return 1.0 / (s - d) - 1.0 / (mu - d) if s <= mu else 0.0


def make_rhs(n):
    def rhs(t, z):
        xi = z[:2 * n].reshape(n, 2)
        vt = z[2 * n:].reshape(n, 2)
        phi = np.zeros_like(xi)
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                e = xi[i] - xi[j]
                s = np.linalg.norm(e)
                if s <= mu:
                    phi[i] += alpha(s) * e / s
        return np.concatenate([(phi - k1 * xi + vt).ravel(), (-k2 * xi).ravel()])
    return rhs


def integrate(xi0, vt0, tf):
    rhs = make_rhs(len(xi0))
    z0 = np.concatenate([xi0.ravel(), vt0.ravel()])
    sol = solve_ivp(rhs, [0, tf], z0, method="BDF",
                    rtol=1e-10, atol=1e-12, dense_output=True)
    assert sol.success, sol.message
    return rhs, sol


# ── Colours ──────────────────────────────────────────────────────────────────
AGENT_COLORS = ["#4477AA", "#EE6677", "#228833", "#CCBB44", "#66CCEE"]

THEME = dict(surface="#fcfcfb", primary="#0b0b0b", secondary="#52514e",
             muted="#8a8880", grid="#e4e3df")


def style_axes(ax):
    ax.set_facecolor(THEME["surface"])
    ax.set_aspect("equal")
    ax.grid(True, color=THEME["grid"], lw=0.4, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(THEME["grid"])
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=THEME["secondary"], labelsize=7.5, length=3, width=0.6)


# ── Main ─────────────────────────────────────────────────────────────────────
def main():
    T_BREATH = 2.485          # observed period (s)
    N_PERIODS = 3             # show 3 full periods
    T_SKIP = 500.0            # skip transient (s)
    T_END = T_SKIP + N_PERIODS * T_BREATH

    print(f"Integrating to t={T_END:.1f}s (skip {T_SKIP}s transient) ...", flush=True)
    rhs, sol = integrate(XI0, VT0, T_END)
    print("  ✓ integration complete", flush=True)

    # Sample exactly 3 periods at high resolution
    n_frames = 180
    t_eval = np.linspace(T_SKIP, T_END, n_frames)
    print(f"Evaluating {n_frames} frames over {N_PERIODS} periods ...", flush=True)

    z_all = sol.sol(t_eval)
    pos_all = z_all[:2 * N].T.reshape(n_frames, N, 2)
    vel_all = z_all[2 * N:].T.reshape(n_frames, N, 2)
    print("  ✓ precomputed", flush=True)

    # ── Animation setup ──────────────────────────────────────────────────
    n = N
    fps = 20

    fig, ax = plt.subplots(figsize=(8, 7))
    fig.patch.set_facecolor(THEME["surface"])
    style_axes(ax)

    # Data bounds
    all_xy = pos_all.reshape(-1, 2)
    margin = 3.0
    ax.set_xlim(all_xy[:, 0].min() - margin, all_xy[:, 0].max() + margin)
    ax.set_ylim(all_xy[:, 1].min() - margin, all_xy[:, 1].max() + margin)
    ax.set_xlabel("x", color=THEME["secondary"], fontsize=9)
    ax.set_ylabel("y", color=THEME["secondary"], fontsize=9)

    # Static: collision boundary circle
    theta = np.linspace(0, 2 * np.pi, 200)
    coll_circle = ax.plot(d * np.cos(theta), d * np.sin(theta),
                          color=THEME["muted"], lw=1.0, ls="--", alpha=0.6,
                          label=f"collision bound $d={d}$")[0]
    mu_circle = ax.plot(mu * np.cos(theta), mu * np.sin(theta),
                        color=THEME["muted"], lw=0.6, ls=":", alpha=0.4,
                        label=f"interaction range $\\mu={mu}$")[0]

    # Agent scatter (no trails)
    agent_scatter = ax.scatter(
        np.zeros(n), np.zeros(n), s=80, zorder=5,
        edgecolors=THEME["surface"], linewidths=0.8, alpha=0.0
    )

    # Velocity arrows
    quiver = ax.quiver(np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n),
                       color=THEME["primary"], alpha=0.7,
                       scale=1.0, scale_units="xy", width=0.008, zorder=6)

    # Agent index labels
    labels = []
    for i in range(n):
        lbl = ax.text(0, 0, str(i), fontsize=8, fontweight="bold",
                      color=THEME["surface"], ha="center", va="center", zorder=7)
        labels.append(lbl)

    # Time display
    time_text = ax.text(0.02, 0.96, "", transform=ax.transAxes,
                        fontsize=11, color=THEME["primary"], fontweight="bold",
                        va="top", ha="left",
                        bbox=dict(facecolor=THEME["surface"], edgecolor=THEME["grid"],
                                  boxstyle="round,pad=0.3", alpha=0.85))

    leg = ax.legend(frameon=False, fontsize=7.5, loc="lower left",
                    labelcolor=THEME["secondary"])
    for txt in leg.get_texts():
        txt.set_color(THEME["secondary"])

    title = ax.set_title("", fontsize=10, color=THEME["primary"],
                         fontweight="bold", loc="left", pad=8)

    fig.tight_layout()

    # ── Animation functions ──────────────────────────────────────────────
    def init():
        agent_scatter.set_offsets(np.zeros((n, 2)))
        agent_scatter.set_alpha(0.0)
        quiver.set_offsets(np.zeros((n, 2)))
        quiver.set_UVC(np.zeros(n), np.zeros(n))
        for lbl in labels:
            lbl.set_position((0, 0))
        time_text.set_text("")
        title.set_text("")
        return [agent_scatter, coll_circle, mu_circle, time_text, title] + labels

    def update(frame):
        pos = pos_all[frame]
        vel = vel_all[frame]

        agent_scatter.set_offsets(pos)
        agent_scatter.set_alpha(1.0)
        agent_scatter.set_color([AGENT_COLORS[i] for i in range(n)])
        agent_scatter.set_edgecolors([THEME["surface"]] * n)

        quiver.set_offsets(pos)
        quiver.set_UVC(vel[:, 0], vel[:, 1])

        for i in range(n):
            labels[i].set_position(pos[i])

        time_text.set_text(f"$t = {t_eval[frame] - T_SKIP:.2f}$ s / $T = {T_BREATH:.3f}$ s")
        title.set_text(
            f"Breathing limit cycle — $N={n}$, $k_1={k1}$, $k_2={k2}$, "
            f"$d={d}$, $\\mu={mu}$"
        )

        return [agent_scatter, coll_circle, mu_circle, time_text, title] + labels

    ani = animation.FuncAnimation(fig, update, frames=n_frames,
                                  init_func=init, blit=True, interval=1000 / fps)

    # ── Save ─────────────────────────────────────────────────────────────
    script_dir = os.path.dirname(os.path.abspath(__file__))
    figures_dir = os.path.join(os.path.dirname(script_dir), "figures")
    os.makedirs(figures_dir, exist_ok=True)

    env_out = os.environ.get("OUT", "")
    if env_out:
        out_gif = env_out
    else:
        out_gif = os.path.join(figures_dir, "breathing_cycle_agents.gif")

    print(f"Writing GIF → {out_gif}  ({n_frames} frames, {fps} fps) ...", flush=True)
    ani.save(out_gif, writer=animation.PillowWriter(fps=fps), dpi=80)
    print("  ✓ done", flush=True)

    plt.close(fig)
    print("\nAll done.", flush=True)


if __name__ == "__main__":
    main()