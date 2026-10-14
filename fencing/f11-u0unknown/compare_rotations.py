#!/usr/bin/env python3
"""
Standalone script: compare rotation algorithms for breaking collinear agent formation.

Four agents start at (0,0), (3,0), (6,0), (9,0) — all on the x-axis — and must
surround a target at (10,0), also on the x-axis.  This is a degenerate collinear
configuration.  Different rotation strategies are compared against a baseline
with no rotation.

Run with:
    uv run --with numpy --with matplotlib python compare_rotations.py
    # or directly:
    python compare_rotations.py
"""

from __future__ import annotations

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from itertools import combinations
from pathlib import Path

# ═══════════════════════════════════════════════════════════════════════
#  Utility functions
# ═══════════════════════════════════════════════════════════════════════

def sgn(x: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(x)
    return np.zeros_like(x) if n == 0 else x / n


def compute_angle(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na == 0 or nb == 0:
        return np.pi
    return np.arccos(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))


def rot_matrix(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


# ═══════════════════════════════════════════════════════════════════════
#  Simple graph (adjacency list)
# ═══════════════════════════════════════════════════════════════════════

class Graph:
    def __init__(self):
        self._edges: dict[tuple, float] = {}

    def add_edge(self, edge):
        i, j, *rest = edge
        w = rest[0] if rest else True
        self._edges[(i, j)] = w
        self._edges[(j, i)] = w

    def neighbors(self, i: int) -> list[int]:
        return [j for (a, j) in self._edges if a == i]


# ═══════════════════════════════════════════════════════════════════════
#  Repulsive force (Alpha)
# ═══════════════════════════════════════════════════════════════════════

class Alpha:
    def __init__(self, params: dict):
        self.mu = params["mu"]
        self.d = params["d"]
        self.AMAX = params.get("ALPHA_MAX", 1e5)

    def _alpha_r(self, s: float) -> float:
        if s > self.d and s < self.mu:
            return min(1 / (s - self.d) - 1 / (self.mu - self.d), self.AMAX)
        return self.AMAX if s < self.d else 0.0

    def __call__(self, positions: list[np.ndarray]):
        reps, g_conn, g_dist = [], Graph(), Graph()
        for i, pos in enumerate(positions):
            f = np.zeros_like(pos)
            for j, posj in enumerate(positions):
                if j == i:
                    continue
                rij = pos - posj
                dij = float(np.linalg.norm(rij))
                g_dist.add_edge((i, j, dij))
                if dij > self.mu:
                    continue
                g_conn.add_edge((i, j))
                f += self._alpha_r(dij) * rij
            reps.append(f)
        return reps, g_conn, g_dist


# ═══════════════════════════════════════════════════════════════════════
#  Rotation strategies
# ═══════════════════════════════════════════════════════════════════════

class RotationChen19:
    """Chen et al. 2019 — the original rotation impulse."""

    def __init__(self, params: dict):
        self.eps = params["epsilon"]
        self.delta = params["delta"]
        self.R = rot_matrix(np.pi / 2)

    def __call__(self, positions, target, g: Graph):
        out = []
        for i, pos in enumerate(positions):
            psi = np.zeros(2)
            ri0 = pos - target
            for j in g.neighbors(i):
                posj = positions[j]
                rij = pos - posj
                rij = sgn(rij)
                rj0 = posj - target
                angle = compute_angle(ri0, rj0)
                if abs(angle) < self.eps:
                    beta = self.eps * max(0.0, self.delta - angle)
                    psi += beta * (self.R @ rij)
            out.append(psi)
        return out


class RotationOur1:
    """Our 2025 rotation with four strategies."""

    def __init__(self, params: dict):
        self.eps = params["epsilon"]
        self.delta = params["delta"]
        self.R = rot_matrix(np.pi / 2)
        self.strategy = params.get("strategy", "psi")

    def __call__(self, positions, g: Graph, target, psis):
        if self.strategy == "chen19":
            return self._chen19(positions, target, g)
        if self.strategy == "opposite":
            return self._chen19_trimed(positions, target, g)
        if self.strategy == "distance":
            return self._distance(positions, target, g)
        if self.strategy == "psi":
            return self._psi(positions, psis, g)
        return [np.zeros(2) for _ in positions]

    # ── chen19 ────────────────────────────────────────────────────────
    def _chen19(self, positions, target, g: Graph):
        out = []
        for i, pos in enumerate(positions):
            psi = np.zeros(2)
            ri0 = pos - target
            for j in g.neighbors(i):
                posj = positions[j]
                rij = sgn(pos - posj)
                rj0 = posj - target
                angle = compute_angle(ri0, rj0)
                if abs(angle) < self.delta:
                    beta = self.eps * max(0.0, self.delta - angle)
                    psi += beta * (self.R @ rij)
            out.append(psi)
        return out

    # ── opposite (chen19-trimed) ──────────────────────────────────────
    def _chen19_trimed(self, positions, target, g: Graph):
        out = []
        for i, pos in enumerate(positions):
            psi = np.zeros(2)
            ri0 = pos - target
            ri0n = float(np.linalg.norm(ri0))
            angles, rije = [], []
            for j in g.neighbors(i):
                posj = positions[j]
                rij = pos - posj
                rije.append(sgn(rij))
                rj0 = posj - target
                rj0n = float(np.linalg.norm(rj0))
                if ri0n < 1e-9 or rj0n < 1e-9:
                    angles.append(np.pi)
                else:
                    angles.append(compute_angle(ri0, rj0))
            uncol = [a for a in angles if self.delta <= a <= np.pi - self.delta]
            if uncol:
                for k, angle in enumerate(angles):
                    a = angle - (np.pi - self.delta)
                    if a > 0:
                        psi += self.eps * a * (self.R @ rije[k])
            out.append(psi)
        return out

    # ── distance ──────────────────────────────────────────────────────
    def _distance(self, positions, target, g: Graph):
        out = []
        for i, posi in enumerate(positions):
            imp = np.zeros(2)
            Ni = g.neighbors(i)
            if len(Ni) < 2:
                out.append(imp)
                continue
            beta = lambda a, b: max(0.0, compute_angle(a, b) - (np.pi - self.delta))
            for j in Ni:
                posj = positions[j]
                near = [beta(positions[k] - posi, posj - posi) for k in Ni if k != j]
                imp += max(near) * (self.R @ sgn(posj - posi))
            out.append(imp)
        return out

    # ── psi ───────────────────────────────────────────────────────────
    def _psi(self, positions, psis, g: Graph):
        out = []
        for i, posi in enumerate(positions):
            imp = np.zeros(2)
            Ni = g.neighbors(i)
            betas = []
            for j, k in combinations(Ni, 2):
                angle = compute_angle(posi - positions[j], posi - positions[k])
                if angle > np.pi / 2 + self.delta:
                    betas.append(angle - (np.pi / 2 - self.delta))
                elif angle < np.pi / 2 - self.delta:
                    betas.append((np.pi / 2 - self.delta) - angle)
                else:
                    betas.append(0.0)
            beta3 = min(betas) if betas else 0.0
            if beta3 > 0:
                imp = self.eps * beta3 * (self.R @ sgn(psis[i]))
            out.append(imp)
        return out


# ═══════════════════════════════════════════════════════════════════════
#  Modified opposite: only triggers on [π-δ, π]
# ═══════════════════════════════════════════════════════════════════════

class RotationOppositePiOnly:
    """Opposite-side judgment: triggers only when angle ∈ [π-δ, π]."""

    def __init__(self, params: dict):
        self.eps = params["epsilon"]
        self.delta = params["delta"]
        self.R = rot_matrix(np.pi / 2)

    def __call__(self, positions, target, g: Graph):
        out = []
        for i, pos in enumerate(positions):
            psi = np.zeros(2)
            ri0 = pos - target
            ri0n = float(np.linalg.norm(ri0))
            for j in g.neighbors(i):
                posj = positions[j]
                rij = sgn(pos - posj)
                rj0 = posj - target
                rj0n = float(np.linalg.norm(rj0))
                if ri0n < 1e-9 or rj0n < 1e-9:
                    angle = np.pi
                else:
                    angle = compute_angle(ri0, rj0)
                if angle > np.pi - self.delta:
                    a = angle - (np.pi - self.delta)
                    beta = self.eps * a
                    psi += beta * (self.R @ rij)
            out.append(psi)
        return out


# ═══════════════════════════════════════════════════════════════════════
#  Simulation
# ═══════════════════════════════════════════════════════════════════════

def simulate(positions_init, target, alpha_params, rot_params,
             attraction_k, rotation_type="none",
             t_end=30.0, dt=0.01):
    """
    Run a single-integrator simulation.

    rotation_type ∈ {"none", "chen19", "our1opposite", "our1oppositepi",
                     "our1distance", "our1psi"}
    """
    positions = [np.array(p, dtype=float) for p in positions_init]
    target = np.array(target, dtype=float)
    alpha = Alpha(alpha_params)

    # Build rotation object
    if rotation_type == "none":
        rot = None
    elif rotation_type == "chen19":
        rot = RotationChen19(rot_params)
    elif rotation_type == "our1oppositepi":
        rot = RotationOppositePiOnly(rot_params)
    elif rotation_type.startswith("our1"):
        rp = dict(rot_params)
        rp["strategy"] = rotation_type[4:]  # skip "our1"
        rot = RotationOur1(rp)
    else:
        raise ValueError(f"Unknown rotation: {rotation_type}")

    n_steps = int(t_end / dt)
    traj = np.zeros((n_steps + 1, len(positions), 2))
    traj[0] = [p.copy() for p in positions]

    for step in range(n_steps):
        reps, g_conn, _ = alpha(positions)

        if rot is None:
            rots = [np.zeros(2) for _ in positions]
        elif isinstance(rot, RotationChen19):
            rots = rot(positions, target, g_conn)
        elif isinstance(rot, RotationOppositePiOnly):
            rots = rot(positions, target, g_conn)
        else:  # RotationOur1
            psis = [-attraction_k * (ai - target) + reps[i]
                    for i, ai in enumerate(positions)]
            rots = rot(positions, g_conn, target, psis)

        for i, ai in enumerate(positions):
            attraction = -attraction_k * (ai - target)
            dx = attraction + reps[i] + rots[i]
            positions[i] = ai + dx * dt

        traj[step + 1] = [p.copy() for p in positions]

    return traj


# ═══════════════════════════════════════════════════════════════════════
#  Convex hull (Graham scan)
# ═══════════════════════════════════════════════════════════════════════

def convex_hull(points):
    pts = sorted(set(map(tuple, points)))
    if len(pts) <= 1:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


# ═══════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════

def main():
    positions_init = [(0.0, 0.0), (3.0, 0.0), (6.0, 0.0), (9.0, 0.0)]
    target = (10.0, 0.0)

    alpha_params = {"d": 0.5, "mu": 3.0, "ALPHA_MAX": 1e5}
    rot_params = {"epsilon": 0.05, "delta": np.deg2rad(10)}
    attraction_k = 2.0
    t_end, dt = 30.0, 0.01

    algorithms = [
        ("无严格凸包保持\n(无旋转基线)",              "none",        2),
        ("同侧判断方案\n(Chen19, angle<ε)",          "chen19",      1),
        ("异侧判断方案\n(angle∈[π-δ,π])",            "our1oppositepi", 1),
        ("基于距离的方案\n(Our1 distance)",          "our1distance",1),
        ("基于排斥项的方案\n(Our1 psi)",             "our1psi",     1),
    ]

    print("Running simulations …")
    results = {}
    for name, rot_type, _ in algorithms:
        print(f"  {name.replace(chr(10), ' ')} …")
        results[name] = simulate(
            positions_init, target, alpha_params, rot_params,
            attraction_k, rotation_type=rot_type,
            t_end=t_end, dt=dt,
        )

    # ── Plot ──────────────────────────────────────────────────────────
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    markers = ["o", "s", "^", "D"]

    n_plots = len(algorithms)
    n_cols = 2
    n_rows = 3
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(6 * n_cols, 5 * n_rows))
    axes = np.atleast_1d(axes).flatten()

    # First row spans both columns — remove the duplicate cell
    fig.delaxes(axes[1])
    axes[0].set_position([0.12, 0.68, 0.78, 0.28])  # [left, bottom, width, height]

    plot_axes = [axes[0], axes[2], axes[3], axes[4], axes[5]]

    for idx, (name, rot_type, _) in enumerate(algorithms):
        ax = plot_axes[idx]
        traj = results[name]

        # Trajectories
        for i in range(len(positions_init)):
            ax.plot(traj[:, i, 0], traj[:, i, 1],
                    color=colors[i], linewidth=1.2, alpha=0.6)

        # Final positions
        final = [traj[-1, i] for i in range(len(positions_init))]
        for i, fp in enumerate(final):
            ax.plot(fp[0], fp[1], marker=markers[i], color=colors[i],
                    markersize=9, markeredgecolor="black", markeredgewidth=1)

        # Target
        ax.plot(target[0], target[1], marker="*", color="black",
                markersize=18, markeredgecolor="black")

        # Convex hull of final positions
        hull = convex_hull(final)
        if len(hull) >= 3:
            ax.add_patch(Polygon(hull, closed=True, facecolor="gray",
                                 alpha=0.15, edgecolor="gray", linestyle="--"))

        # Initial positions (small dots)
        for i, ip in enumerate(positions_init):
            ax.plot(ip[0], ip[1], marker=".", color=colors[i],
                    markersize=4, alpha=0.3)

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title(name, fontsize=11, fontweight="bold")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)

    # ── Legend (bottom) ───────────────────────────────────────────────
    legend_elements = []
    for i in range(len(positions_init)):
        legend_elements.append(
            plt.Line2D([0], [0], marker=markers[i], color="w",
                       markerfacecolor=colors[i], markersize=10,
                       markeredgecolor="black", markeredgewidth=1,
                       label=f"Agent {i+1}")
        )
    legend_elements.append(
        plt.Line2D([0], [0], marker="*", color="w",
                   markerfacecolor="black", markersize=15,
                   markeredgecolor="black", label="Target")
    )
    legend_elements.append(
        plt.Line2D([0], [0], linestyle="--", color="gray",
                   label="Convex Hull")
    )

    fig.legend(handles=legend_elements, loc="lower center", ncol=4,
              fontsize=10, frameon=True)

    fig.suptitle(
        "规避直线队形的旋转算法对比\n"
        "智能体: (0,0) (3,0) (6,0) (9,0)  →  目标: (10,0)",
        fontsize=14, fontweight="bold"
    )

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.10)

    out = Path(__file__).resolve().parent / "rotation_comparison.png"
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"\nPlot saved to: {out}")
    plt.close()


if __name__ == "__main__":
    main()