"""
Figure 2: Obstacle collision avoidance & singleton formation.

Conceptual illustration (no dynamical simulation needed).
Shows how singleton (collinear) formation blocks all vehicles when an
obstacle appears, while a non-collinear formation can bypass it.
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

plt.rcParams.update({
    "font.size": 11,
    "axes.titlesize": 13,
    "axes.labelsize": 12,
    "figure.dpi": 150,
})

OUT_DIR = "/Users/a1/repo/phd-notes/fencing/review/chen2019/images"

init_handle = mpatches.Patch(facecolor="#B0B0B0", edgecolor="black", alpha=0.5,
                             label="initial position")
final_handle = mpatches.Patch(facecolor="#4472C4", edgecolor="black",
                              label="final steady state")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.8))

# ---------------------------------------------------------------------------
# Panel (a): singleton formation blocked by obstacle
# ---------------------------------------------------------------------------
vehicles_line = np.array([
    [-2.0, 0.0],
    [-0.5, 0.0],
    [ 0.5, 0.0],
    [ 2.0, 0.0],
])

for i, pos in enumerate(vehicles_line):
    ax1.scatter(pos[0], pos[1], s=140, zorder=4, edgecolors="black",
                linewidths=1.0, color="#B0B0B0", alpha=0.5)
    ax1.scatter(pos[0], pos[1], s=200, zorder=5, edgecolors="black",
                linewidths=1.2, color="#4472C4")
    ax1.annotate(f"$x_{i+1}$", pos, textcoords="offset points",
                 xytext=(0, 15), ha="center", fontsize=9, fontweight="bold")

# Obstacle
obstacle = mpatches.FancyBboxPatch(
    (-0.3, -0.3), 0.6, 0.6, boxstyle="round,pad=0.05",
    facecolor="#B0B0B0", edgecolor="black", linewidth=1.5, alpha=0.9)
ax1.add_patch(obstacle)
ax1.text(0.0, 0.0, "OBSTACLE", ha="center", va="center",
         fontsize=8, fontweight="bold", color="white")

ax1.plot([-2.5, 2.5], [0, 0], "k:", linewidth=1, alpha=0.4)

for x_pos in [-1.2, 1.2]:
    ax1.annotate("", xy=(x_pos, 0.0), xytext=(x_pos, 0.6),
                 arrowprops=dict(arrowstyle="->", lw=1.5, color="#C00000", alpha=0.6))
    ax1.annotate("", xy=(x_pos, 0.0), xytext=(x_pos, -0.6),
                 arrowprops=dict(arrowstyle="->", lw=1.5, color="#C00000", alpha=0.6))

ax1.text(0.0, -1.3, "ALL vehicles blocked", ha="center", fontsize=10,
         fontweight="bold", color="#C00000")
ax1.text(0.0, -1.8, "singleton (collinear) formation", ha="center", fontsize=10,
         fontweight="bold", color="#808080")

ax1.set_xlim(-3.0, 3.0)
ax1.set_ylim(-2.2, 1.8)
ax1.set_aspect("equal")
ax1.set_title("(a) Singleton formation\nAll vehicles blocked", fontsize=12)
ax1.legend(handles=[init_handle, final_handle], loc="upper left", fontsize=8, framealpha=0.9)
ax1.grid(True, alpha=0.3)

# ---------------------------------------------------------------------------
# Panel (b): non-collinear formation, can bypass obstacle
# ---------------------------------------------------------------------------
vehicles_arc = np.array([
    [-1.8, -0.3],
    [ 1.8, -0.3],
    [ 0.0,  1.0],
])

for i, pos in enumerate(vehicles_arc):
    ax2.scatter(pos[0], pos[1], s=140, zorder=4, edgecolors="black",
                linewidths=1.0, color="#B0B0B0", alpha=0.5)
    ax2.scatter(pos[0], pos[1], s=200, zorder=5, edgecolors="black",
                linewidths=1.2, color="#4472C4")
    ax2.annotate(f"$x_{i+1}$", pos, textcoords="offset points",
                 xytext=(0, 15), ha="center", fontsize=9, fontweight="bold")

obstacle2 = mpatches.FancyBboxPatch(
    (-0.3, -1.0), 0.6, 0.6, boxstyle="round,pad=0.05",
    facecolor="#B0B0B0", edgecolor="black", linewidth=1.5, alpha=0.9)
ax2.add_patch(obstacle2)
ax2.text(0.0, -0.7, "OBSTACLE", ha="center", va="center",
         fontsize=7, fontweight="bold", color="white")

ax2.annotate("", xy=(-1.0, -0.1), xytext=(-1.0, -0.8),
             arrowprops=dict(arrowstyle="->", lw=1.5, color="#2E7D32", alpha=0.7))
ax2.annotate("", xy=(1.0, -0.1), xytext=(1.0, -0.8),
             arrowprops=dict(arrowstyle="->", lw=1.5, color="#2E7D32", alpha=0.7))
ax2.annotate("", xy=(0.0, 0.4), xytext=(0.0, -0.2),
             arrowprops=dict(arrowstyle="->", lw=1.5, color="#2E7D32", alpha=0.7))

ax2.text(0.0, -1.9, "vehicles can bypass ✓", ha="center", fontsize=10,
         fontweight="bold", color="#2E7D32")
ax2.text(0.0, -2.4, "non-collinear formation", ha="center", fontsize=10,
         fontweight="bold", color="#808080")

ax2.set_xlim(-3.0, 3.0)
ax2.set_ylim(-2.8, 2.2)
ax2.set_aspect("equal")
ax2.set_title("(b) Non-collinear formation\nVehicles can bypass obstacle ✓", fontsize=12)
ax2.legend(handles=[init_handle, final_handle], loc="upper left", fontsize=8, framealpha=0.9)
ax2.grid(True, alpha=0.3)

fig.suptitle("Obstacle Collision Avoidance & Singleton Formation", fontsize=14, fontweight="bold")
plt.tight_layout()
fig.savefig(f"{OUT_DIR}/fig2_obstacle_singleton.png", bbox_inches="tight", dpi=200)
print("Saved images/fig2_obstacle_singleton.png")