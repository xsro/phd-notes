"""
Figure 2: Obstacle collision avoidance & singleton formation (with scipy simulation).

Scenario: 4 vehicles aligned in a straight line, obstacle at one end, target at the other.
  (a) rotation term disabled  -> vehicles stay collinear, blocked by obstacle
  (b) rotation term enabled   -> vehicles break out of singleton, fence target & avoid obstacle
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from matplotlib.path import Path
import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial import ConvexHull
from pathlib import Path as PPath

plt.rcParams.update({
    "font.size": 11,
    "axes.titlesize": 13,
    "axes.labelsize": 12,
    "figure.dpi": 150,
})

OUT_DIR = PPath(__file__).parent / "images"

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
N = 4
d = 0.5
mu = 1.5
k = 1.0
eps = 0.8
delta = np.pi / 4

# Obstacle and target at the same end of the line
obstacle = np.array([2.6, 0.0])
target = np.array([3.4, 0.0])

init_pos = np.array([
    [-2.2, 0.0],
    [-0.8, 0.0],
    [ 0.8, 0.0],
    [ 2.2, 0.0],
])

R = np.array([[0, -1],
              [1,  0]])

# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------
def alpha(s, d, mu):
    if s <= d:
        return 1e6
    if s >= mu:
        return 0.0
    return 1.0 / (s - d) - 1.0 / (mu - d)


def angle_between(v1, v2):
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-12 or n2 < 1e-12:
        return np.pi
    cos_ang = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return np.arccos(cos_ang)


def beta(xi, xj, xo, eps, delta):
    ang = angle_between(xi - xo, xj - xo)
    return eps * max(0.0, delta - ang)


def compute_control(x_vec, use_rotation=True):
    x = x_vec.reshape(N, 2)
    u = np.zeros((N, 2))
    for i in range(N):
        xi = x[i]

        # Repulsive: from neighbors + obstacle
        rep = np.zeros(2)
        for j in range(N):
            if j == i:
                continue
            xij = xi - x[j]
            dist = np.linalg.norm(xij)
            if dist < mu:
                rep += alpha(dist, d, mu) * xij / dist

        # Obstacle repulsion
        xio = xi - obstacle
        dist_o = np.linalg.norm(xio)
        if dist_o < mu:
            rep += alpha(dist_o, d, mu) * xio / dist_o

        # Rotation: from neighbors only
        rot = np.zeros(2)
        if use_rotation:
            for j in range(N):
                if j == i:
                    continue
                xij = xi - x[j]
                dist = np.linalg.norm(xij)
                if dist < mu:
                    b = beta(xi, x[j], target, eps, delta)
                    rot += b * (R @ xij) / dist

        # Attractive: toward target
        att = k * (target - xi)

        u[i] = rep + rot + att
    return u.flatten()


def ode_rhs(t, x_vec, use_rotation=True):
    return compute_control(x_vec, use_rotation)


# ---------------------------------------------------------------------------
# Simulate
# ---------------------------------------------------------------------------
t_span = (0, 200)
t_eval = np.linspace(0, 200, 1000)

print("Simulating case (a): rotation term disabled...")
sol_a = solve_ivp(ode_rhs, t_span, init_pos.flatten(),
                  args=(False,), method='RK45',
                  t_eval=t_eval, rtol=1e-6, atol=1e-8)

print("Simulating case (b): rotation term enabled...")
sol_b = solve_ivp(ode_rhs, t_span, init_pos.flatten(),
                  args=(True,), method='RK45',
                  t_eval=t_eval, rtol=1e-6, atol=1e-8)

final_a = sol_a.y[:, -1].reshape(N, 2)
final_b = sol_b.y[:, -1].reshape(N, 2)

print("\nCase (a) final positions:")
for i in range(N):
    print(f"  x_{i+1} = ({final_a[i,0]:.3f}, {final_a[i,1]:.3f})")
print(f"  Obstacle at ({obstacle[0]}, {obstacle[1]})")
print(f"  Target at ({target[0]}, {target[1]})")

print("\nCase (b) final positions:")
for i in range(N):
    print(f"  x_{i+1} = ({final_b[i,0]:.3f}, {final_b[i,1]:.3f})")
print(f"  Obstacle at ({obstacle[0]}, {obstacle[1]})")
print(f"  Target at ({target[0]}, {target[1]})")


def draw_convex_hull(ax, pts, **kwargs):
    try:
        hull = ConvexHull(pts)
        hull_pts = pts[hull.vertices]
        ax.add_patch(plt.Polygon(hull_pts, **kwargs))
        return hull_pts
    except Exception:
        idx = np.argsort(pts[:, 0])
        ax.plot([pts[idx[0], 0], pts[idx[-1], 0]],
                [pts[idx[0], 1], pts[idx[-1], 1]],
                color=kwargs.get("edgecolor", "green"),
                linestyle=kwargs.get("linestyle", "--"),
                linewidth=kwargs.get("linewidth", 1.8))
        return pts[idx[[0, -1]]]


def target_inside_hull(pts, target):
    try:
        hull = ConvexHull(pts)
        return Path(pts[hull.vertices]).contains_points([target])[0]
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.8))

init_handle = mpatches.Patch(facecolor="#B0B0B0", edgecolor="black", alpha=0.5,
                             label="initial position")
final_handle = mpatches.Patch(facecolor="#4472C4", edgecolor="black",
                              label="final steady state")
obstacle_handle = mpatches.Patch(facecolor="#808080", edgecolor="black",
                                 label="obstacle")
target_handle = Line2D([0], [0], marker="*", markerfacecolor="#C00000",
                       markeredgecolor="black", markersize=12, linestyle="None",
                       label="target")

# --- Panel (a): rotation disabled ---
for i in range(N):
    ax1.plot(sol_a.y[2*i, ::60], sol_a.y[2*i+1, ::60],
             color="#4472C4", alpha=0.2, linewidth=0.7)

for pos in init_pos:
    ax1.scatter(pos[0], pos[1], s=140, zorder=4, edgecolors="black",
                linewidths=1.0, color="#B0B0B0", alpha=0.5)

hull_pts_a = draw_convex_hull(
    ax1, final_a,
    fill=True, facecolor="#E2EFDA",
    edgecolor="green", linewidth=1.8, linestyle="--", alpha=0.5)

for i, pos in enumerate(final_a):
    ax1.scatter(pos[0], pos[1], s=200, zorder=5, edgecolors="black",
                linewidths=1.2, color="#4472C4")
    ax1.annotate(f"$x_{i+1}$", pos, textcoords="offset points",
                 xytext=(0, 15), ha="center", fontsize=9, fontweight="bold")

# Obstacle
ax1.scatter(obstacle[0], obstacle[1], s=220, zorder=6, marker="s",
            color="#808080", edgecolors="black", linewidths=1.2)
ax1.annotate("obstacle", obstacle, textcoords="offset points",
             xytext=(10, 14), fontsize=9, fontweight="bold", color="#606060")

# Target
ax1.scatter(target[0], target[1], s=280, zorder=6, marker="*",
            color="#C00000", edgecolors="black", linewidths=0.8)
ax1.annotate("$x_o$", target, textcoords="offset points",
             xytext=(10, 14), fontsize=10, fontweight="bold", color="#C00000")

inside_a = target_inside_hull(final_a, target)
ax1.text(0.0, -1.1, f"blocked by obstacle", ha="center", fontsize=10,
         fontweight="bold", color="#C00000")
ax1.text(0.0, -1.6, "rotation term disabled", ha="center", fontsize=10,
         fontweight="bold", color="#808080")
ax1.text(0.0, -2.05, "vehicles remain collinear (singleton)", ha="center", fontsize=9,
         fontstyle="italic", color="#808080")

ax1.set_xlim(-3.2, 4.2)
ax1.set_ylim(-2.4, 1.8)
ax1.set_aspect("equal")
ax1.set_title("(a) Rotation term disabled\nFormation blocked by obstacle", fontsize=12)
ax1.legend(handles=[init_handle, final_handle, obstacle_handle, target_handle],
            loc="upper left", fontsize=8, framealpha=0.9)
ax1.grid(True, alpha=0.3)

# --- Panel (b): rotation enabled ---
for i in range(N):
    ax2.plot(sol_b.y[2*i, ::60], sol_b.y[2*i+1, ::60],
             color="#2E7D32", alpha=0.2, linewidth=0.7)

for pos in init_pos:
    ax2.scatter(pos[0], pos[1], s=140, zorder=4, edgecolors="black",
                linewidths=1.0, color="#B0B0B0", alpha=0.5)

hull_pts_b = draw_convex_hull(
    ax2, final_b,
    fill=True, facecolor="#E2EFDA",
    edgecolor="green", linewidth=1.8, linestyle="--", alpha=0.5)

for i, pos in enumerate(final_b):
    ax2.scatter(pos[0], pos[1], s=200, zorder=5, edgecolors="black",
                linewidths=1.2, color="#4472C4")
    ax2.annotate(f"$x_{i+1}$", pos, textcoords="offset points",
                 xytext=(0, 15), ha="center", fontsize=9, fontweight="bold")

ax2.scatter(obstacle[0], obstacle[1], s=220, zorder=6, marker="s",
            color="#808080", edgecolors="black", linewidths=1.2)
ax2.annotate("obstacle", obstacle, textcoords="offset points",
             xytext=(10, 14), fontsize=9, fontweight="bold", color="#606060")

ax2.scatter(target[0], target[1], s=280, zorder=6, marker="*",
            color="#C00000", edgecolors="black", linewidths=0.8)
ax2.annotate("$x_o$", target, textcoords="offset points",
             xytext=(10, 14), fontsize=10, fontweight="bold", color="#C00000")

inside_b = target_inside_hull(final_b, target)
ax2.text(0.0, -1.5, f"target fenced ✓ (inside={inside_b})", ha="center", fontsize=10,
         fontweight="bold", color="#2E7D32")
ax2.text(0.0, -2.0, "rotation term enabled", ha="center", fontsize=10,
         fontweight="bold", color="#808080")
ax2.text(0.0, -2.45, "vehicles break out of singleton", ha="center", fontsize=9,
         fontstyle="italic", color="#808080")

ax2.set_xlim(-3.2, 5.2)
ax2.set_ylim(-2.8, 2.2)
ax2.set_aspect("equal")
ax2.set_title("(b) Rotation term enabled\nTarget is fenced ✓", fontsize=12)
ax2.legend(handles=[init_handle, final_handle, obstacle_handle, target_handle],
            loc="upper left", fontsize=8, framealpha=0.9)
ax2.grid(True, alpha=0.3)

fig.suptitle("Obstacle Collision Avoidance & Singleton Formation\n"
             "(initial: vehicles aligned in a line, obstacle at one end, target at the other)",
             fontsize=13, fontweight="bold", y=1.02)
plt.tight_layout()
fig.savefig(f"{OUT_DIR}/fig2_obstacle_singleton.png", bbox_inches="tight", dpi=200)
print("Saved images/fig2_obstacle_singleton.png")