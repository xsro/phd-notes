# 3D Breathing Limit Cycle — Results Summary

## Overview

The 3D fencing system with the Kou-Chen-Xiang controller supports a **non-planar
3D breathing limit cycle** — pairwise distances oscillate periodically without
converging, while the formation rotates in 3D. This is the first discovered 3D
breathing attractor.

## Key Files

| File | Purpose |
|------|---------|
| `simulate_3d_breathing.py` | Standalone simulation of the known 3D breathing case (seed=0, N=6) |
| `scan_3d_final.py` | Parameter scan: 6×6 k₁-k₂ grid, d_col, μ, N, seeds (~70 runs) |
| `data/simulate_3d_breathing.npz` | Saved simulation data (25,001 time steps, 500s) |

## Known 3D Breathing Case (N=6, seed=0)

**Parameters**: N=6, d=5.0, μ=9.0, k₁=0.5, k₂=0.5, v₀=(1,0,0)

**Initial conditions** (from `generate_random_3d_init(N=6, d_col=5.0, seed=0)`):
```
XI0 = [[14.112419,  3.201258,  7.829904],
       [17.927146, 14.940464, -7.818223],
       [ 7.600707, -1.210858, -0.825751],
       [ 3.284788,  1.152349, 11.634188],
       [ 6.088302,  0.973400,  3.550906],
       [ 2.669395, 11.952633, -1.641266]]
VT0 = [[0.0, 0.0, 0.0]]  (zero initial velocity)
```

**Steady-state properties**:
- Breathing frequency: f ≈ 0.22 Hz (T ≈ 4.55 s)
- Peak FFT power ratio: ~0.76 (strong oscillation)
- σ₃/σ₁ ≈ 0.83 (non-planar)
- Mean final velocity error: 3.14 (vehicles don't converge to target velocity)

## Parameter Dependence (N=4 scan)

### Primary scaling with k₂

For k₂ ≥ 0.35:

$$T_{\text{breath}} \approx 0.5 \times \frac{2\pi}{\sqrt{k_2}}, \quad f_{\text{breath}} \approx 2 \times \frac{\sqrt{k_2}}{2\pi}$$

The breathing frequency scales as √k₂ (same as rigid rotation) but is ~2× higher.

### k₁ modulation

k₁ has a weaker but noticeable effect, especially at low k₂:

| k₁ range | Effect on T (at k₂=0.5) |
|----------|--------------------------|
| k₁ ≤ 0.15 | T ≈ 4.29 s (faster breathing) |
| k₁ ≥ 0.70 | T ≈ 7.50 s (slower breathing) |
| 0.2–0.6 | T ≈ 4.29–5.0 s (transition) |

For k₂ ≥ 0.35, the k₁ effect diminishes; T/T_rot ≈ 0.5 holds broadly.

### d_col and μ

- **μ ≥ 8**: period stable at T = 5.0 s (for k₁=k₂=0.5)
- **μ < 8**: period increases as sensing range shrinks
- **d_col**: non-monotonic effect — d=3: T=4.29s, d=4: T=7.5s, d=5: T=5.0s, d=6: T=6.0s, d=7: T=7.5s

### N dependence

Non-monotonic — N=3 is planar (σ=0, T=6.0s), N=4 has shortest period (T=5.0s),
N=5 is slower (T=7.5s) at k₁=k₂=0.5.

### Robustness

Breathing period is robust across random seeds. For k₁=k₂=0.5, N=4:
5 of 6 seeds give T=5.0s, 1 seed gives T=4.29s (within frequency resolution).

## Full k₁-k₂ Grid (N=4, seed=0)

All 36 combinations (k₁=0.1–0.6, k₂=0.1–0.6) converge to breathing.
Period T (s) as function of (k₁, k₂):

```
k₁\k₂  0.1   0.2   0.3   0.4   0.5   0.6
0.1   10.0  7.5   6.0   5.0   4.29  4.29
0.2   15.0  7.5   6.0   5.0   4.29  4.29
0.3   15.0  10.0  6.0   5.0   4.29  4.29
0.4   30.0  10.0  6.0   7.5   4.29  4.29
0.5   30.0  15.0  7.5   5.0   5.0   4.29
0.6   30.0  15.0  10.0  6.0   5.0   4.29
```

Frequency resolution: 0.033 Hz (T_MAX=60s, DT=0.1s, 50% steady-state window).

## Comparison with 2D Breathing

| Property | 2D Breathing | 3D Breathing |
|----------|-------------|-------------|
| T/T_rot | ~0.28 | ~0.5 |
| f_breath/f_rot | ~3.57 | ~2.0 |
| Planarity | Planar (σ₃/σ₁=0) | Non-planar (σ₃/σ₁≈0.4–0.8) |
| N dependence | N=5: T≈2.49s | N=4: T≈5.0s, N=6: T≈4.55s |

3D breathing is closer to rigid rotation than 2D breathing, consistent with
3D having more degrees of freedom and thus a less stiff breathing mode.

## Discovery

The 3D breathing case was discovered by `experiment/search_3d_breathing.py`,
which scans random 3D initial conditions and classifies steady-state behavior
via FFT of pairwise distances. The seed=0 case (N=6) was the first genuine
3D breathing attractor found.