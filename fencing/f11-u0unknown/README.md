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

| # | Algorithm | Type | Breaks Collinearity? |
|---|-----------|------|---------------------|
| 1 | No Rotation (Baseline) | Attraction + Repulsion only | ❌ |
| 2 | Chen19 Rotation | `RotationChen19` | ✅ |
| 3 | Our1 (chen19 strategy) | `RotationOur1` → `chen19` | ✅ |
| 4 | Our1 (opposite strategy) | `RotationOur1` → `opposite` | ❌ |
| 5 | Our1 (distance strategy) | `RotationOur1` → `distance` | ✅ |
| 6 | Our1 (psi strategy) | `RotationOur1` → `psi` | ✅ |

## Quick Start

```bash
# Directly (if numpy + matplotlib are already installed)
python3 compare_rotations.py

# With uv — run from outside the project directory to avoid
# pulling in the parent project's dependencies:
cd /tmp && uv run --with numpy --with matplotlib \
    python compare_rotations.py
```

## Output

Generates `rotation_comparison.png` (output of `compare_rotations.py`) — a 2×3 grid of subplots, one per algorithm.
Each subplot shows:

- Coloured lines: agent trajectories
- Coloured markers (circle, square, triangle, diamond): final agent positions
- Black star: target position
- Grey dashed polygon: convex hull of final positions
- Small dots: initial positions

Legend at the bottom.

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