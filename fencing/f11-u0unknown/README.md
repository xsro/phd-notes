# Rotation Algorithm Comparison for Collinear Formation Avoidance

Standalone Python script that simulates four agents in a collinear (straight-line)
formation trying to surround a target, and compares different rotation algorithms
extracted from the [fencer](https://github.com/earendil-works/fencer) project.

## Scenario

- **Agents**: (0, 0), (3, 0), (6, 0), (9, 0) — all on the x-axis
- **Target**: (10, 0) — also on the x-axis
- This is a **degenerate collinear configuration** — agents cannot break out of
  the line without a rotation impulse.

## Algorithms Compared

| # | Algorithm | Type |
|---|-----------|------|
| 1 | No Rotation (Baseline) | Attraction + Repulsion only |
| 2 | Same-side Rotation (Chen19) | `RotationChen19` |
| 3 | Opposite-side Rotation (angle ∈ [π−δ, π]) | `RotationOppositePiOnly` |
| 4 | Distance-based Rotation | `RotationOur1` → `distance` |
| 5 | Repulsion-based Rotation | `RotationOur1` → `psi` |

## Quick Start

```bash
# Directly (if numpy + matplotlib are already installed)
python3 compare_rotations.py

# With uv — ignore the parent project's dependencies:
uv run --no-project --with numpy --with matplotlib \
    python compare_rotations.py
```

## Output

Generates `rotation_comparison.png` in the compact layout of `target.png`: one full-width baseline panel above a 2×2 grid of rotation strategies. The canvas is transparent and the plot panels are white.
Each subplot shows:

- Coloured lines: agent trajectories
- Blue, orange, yellow, and purple circles: final agent positions
- Small dark cross: target position
- Grey dashed polygon with a pale fill: convex hull of final positions

A single-row legend sits at the bottom. Chinese labels use an available Chinese font; English labels are used if none is installed. Simulation parameters and trajectories are unchanged.

## Parameters

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `d` | 0.5 | Collision avoidance distance |
| `mu` | 3.0 | Repulsion range / communication radius |
| `epsilon` | 0.05 | Rotation gain |
| `delta` | 10° | Rotation angular threshold |
| `k` | 2.0 | Attraction gain |
| `t_end` | 30.0 s | Simulation duration |
| `dt` | 0.01 s | Integration step |

## Source

Algorithms reimplemented from the `fencer` project's `mas/fence/` module:

- `RotationChen19` → `mas/fence/rotation.py`
- `RotationOur1` (4 strategies) → `mas/fence/rotation.py`
- `Alpha` repulsion → `mas/fence/alpha.py`