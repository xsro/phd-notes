# 3D Breathing Limit Cycle — Parameter Scan Results

## Overview

The 3D fencing system with the Kou-Chen-Xiang controller supports a **non-planar
3D breathing limit cycle** — pairwise distances oscillate periodically without
converging, while the formation rotates in 3D. This is the first discovered 3D
breathing attractor.

This document consolidates all parameter scan results from
`scan_3d_final.py`.

## Scan Configuration

- **N** = 4 vehicles (fast scan; N=6 case documented separately)
- **d** = 5.0 (collision distance)
- **μ** = 9.0 (sensing radius)
- **v₀** = (1, 0, 0) (target velocity)
- **T_MAX** = 60 s, **DT** = 0.1 s
- **Frequency resolution**: 1/(60×0.5) = 0.033 Hz
- **Initial conditions**: `np.random.randn(N, 3) * 10` with `np.random.seed(0)`
- **Classification**: `peak_ratio > 0.05` → breathing; otherwise rotation

## Primary Scaling: k₂

For k₂ ≥ 0.35:

$$T_{\text{breath}} \approx 0.5 \times \frac{2\pi}{\sqrt{k_2}}, \quad f_{\text{breath}} \approx 2 \times \frac{\sqrt{k_2}}{2\pi}$$

The breathing frequency scales as √k₂ (same as rigid rotation) but is ~2× higher.

### k₂ scan (k₁ = 0.5 fixed)

| k₂ | f (Hz) | T (s) | T_rot (s) | T/T_rot | peak_ratio | Type |
|----|--------|-------|-----------|---------|------------|------|
| 0.05 | 0.033 | 30.00 | 28.10 | 1.07 | 0.708 | BR |
| 0.10 | 0.033 | 30.00 | 19.87 | 1.51 | 0.404 | BR |
| 0.15 | 0.067 | 15.00 | 16.22 | 0.93 | 0.566 | BR |
| 0.20 | 0.067 | 15.00 | 14.05 | 1.07 | 0.435 | BR |
| 0.25 | 0.100 | 10.00 | 12.57 | 0.80 | 0.344 | BR |
| 0.35 | 0.167 | 6.00 | 10.68 | 0.56 | 0.484 | BR |
| 0.45 | 0.200 | 5.00 | 9.40 | 0.53 | 0.599 | BR |
| 0.55 | 0.233 | 4.29 | 8.57 | 0.51 | 0.249 | BR |
| 0.65 | 0.233 | 4.29 | 7.85 | 0.55 | 0.221 | BR |
| 0.75 | 0.267 | 3.75 | 7.26 | 0.52 | 0.904 | BR |
| 0.85 | 0.300 | 3.33 | 6.78 | 0.49 | 0.566 | BR |
| 0.90 | 0.300 | 3.33 | 6.62 | 0.50 | 0.894 | BR |

For k₂ ≥ 0.35, T/T_rot ≈ 0.5 (stable). For k₂ < 0.35, the ratio varies
and can exceed 1.0, indicating the breathing mode becomes softer.

## k₁ Modulation

k₁ has a weaker but noticeable effect, especially at low k₂.

### k₁ scan (k₂ = 0.5 fixed)

| k₁ | f (Hz) | T (s) | T_rot (s) | T/T_rot | peak_ratio | Type |
|----|--------|-------|-----------|---------|------------|------|
| 0.05 | 0.233 | 4.29 | 8.89 | 0.48 | 0.655 | BR |
| 0.10 | 0.233 | 4.29 | 8.89 | 0.48 | 0.690 | BR |
| 0.15 | 0.233 | 4.29 | 8.89 | 0.48 | 0.587 | BR |
| 0.70 | 0.133 | 7.50 | 8.89 | 0.84 | 0.520 | BR |
| 0.75 | 0.133 | 7.50 | 8.89 | 0.84 | 0.477 | BR |
| 0.80 | 0.133 | 7.50 | 8.89 | 0.84 | 0.447 | BR |
| 0.85 | 0.133 | 7.50 | 8.89 | 0.84 | 0.340 | BR |
| 0.90 | 0.133 | 7.50 | 8.89 | 0.84 | 0.313 | BR |

For k₁ ≤ 0.15, T ≈ 4.29 s. For k₁ ≥ 0.70, T ≈ 7.50 s. The transition
occurs in the range k₁ = 0.2–0.6 (see grid scan below).

## Full k₁-k₂ Grid (N=4, seed=0)

All 36 combinations (k₁ = 0.1–0.6, k₂ = 0.1–0.6) converge to breathing.
Period T (s):

```
k₁\k₂  0.1   0.2   0.3   0.4   0.5   0.6
0.1   10.0  7.5   6.0   5.0   4.29  4.29
0.2   15.0  7.5   6.0   5.0   4.29  4.29
0.3   15.0  10.0  6.0   5.0   4.29  4.29
0.4   30.0  10.0  6.0   7.5   4.29  4.29
0.5   30.0  15.0  7.5   5.0   5.0   4.29
0.6   30.0  15.0  10.0  6.0   5.0   4.29
```

Key observations:
- For k₂ ≥ 0.3, T is mostly determined by k₂ (rows are similar)
- For k₂ = 0.1–0.2, increasing k₁ strongly increases T
- The k₁ effect diminishes as k₂ increases

## d_col Scan (k₁ = k₂ = 0.5, μ = 9.0)

| d_col | f (Hz) | T (s) | T/T_rot | peak_ratio | σ₃/σ₁ | Type |
|-------|--------|-------|---------|------------|-------|------|
| 3.0 | 0.233 | 4.29 | 0.48 | 0.585 | 0.624 | BR |
| 4.0 | 0.133 | 7.50 | 0.84 | 0.300 | 0.113 | BR |
| 5.0 | 0.200 | 5.00 | 0.56 | 0.412 | 0.444 | BR |
| 6.0 | 0.167 | 6.00 | 0.68 | 0.247 | 0.392 | BR |
| 7.0 | 0.133 | 7.50 | 0.84 | 0.458 | 0.028 | BR |

Non-monotonic effect. d = 5 gives the shortest period. At d = 7, the
formation becomes nearly planar (σ₃/σ₁ = 0.028).

## μ Scan (k₁ = k₂ = 0.5, d = 5.0)

| μ | f (Hz) | T (s) | T/T_rot | peak_ratio | σ₃/σ₁ | Type |
|---|--------|-------|---------|------------|-------|------|
| 6.0 | 0.133 | 7.50 | 0.84 | 0.399 | 0.714 | BR |
| 7.0 | 0.167 | 6.00 | 0.68 | 0.288 | 0.611 | BR |
| 8.0 | 0.200 | 5.00 | 0.56 | 0.438 | 0.476 | BR |
| 9.0 | 0.200 | 5.00 | 0.56 | 0.412 | 0.444 | BR |
| 10.0 | 0.200 | 5.00 | 0.56 | 0.380 | 0.783 | BR |
| 11.0 | 0.200 | 5.00 | 0.56 | 0.363 | 0.520 | BR |
| 12.0 | 0.200 | 5.00 | 0.56 | 0.298 | 0.256 | BR |

For μ ≥ 8, the period stabilizes at T = 5.0 s. For smaller μ, the period
increases (breathing slows down) as the sensing range shrinks.

## Seed Scan (N=4, k₁ = k₂ = 0.5, d = 5.0, μ = 9.0)

| seed | f (Hz) | T (s) | T/T_rot | peak_ratio | σ₃/σ₁ | Type |
|------|--------|-------|---------|------------|-------|------|
| 0 | 0.200 | 5.00 | 0.56 | 0.412 | 0.444 | BR |
| 1 | 0.200 | 5.00 | 0.56 | 0.386 | 0.540 | BR |
| 2 | 0.200 | 5.00 | 0.56 | 0.428 | 0.832 | BR |
| 3 | 0.233 | 4.29 | 0.48 | 0.421 | 0.978 | BR |
| 4 | 0.200 | 5.00 | 0.56 | 0.389 | 0.705 | BR |
| 5 | 0.200 | 5.00 | 0.56 | 0.764 | 0.502 | BR |

The breathing period is robust: 5 of 6 seeds give T = 5.0 s.
Seed 3 gives T = 4.29 s, which is within the frequency resolution (0.033 Hz).

## N Scan (k₁ = k₂ = 0.5, d = 5.0, μ = 9.0)

| N | f (Hz) | T (s) | T_rot (s) | T/T_rot | peak_ratio | σ₃/σ₁ | Type |
|---|--------|-------|-----------|---------|------------|-------|------|
| 3 | 0.167 | 6.00 | 8.89 | 0.68 | 0.575 | 0.000 | BR (planar) |
| 4 | 0.200 | 5.00 | 8.89 | 0.56 | 0.412 | 0.444 | BR |
| 5 | 0.133 | 7.50 | 8.89 | 0.84 | 0.536 | 0.637 | BR |

Non-monotonic N dependence. N = 4 has the shortest period.
N = 3 is planar (σ₃/σ₁ = 0), meaning the formation collapses to a plane.

## Wider k₂ Scan (k₁ = 0.5, N=4, d=5.0, μ=9.0)

| k₂ | f (Hz) | T (s) | T_rot (s) | T/T_rot | peak_ratio | Type |
|----|--------|-------|-----------|---------|------------|------|
| 0.05 | 0.033 | 30.00 | 28.10 | 1.07 | 0.708 | BR |
| 0.10 | 0.033 | 30.00 | 19.87 | 1.51 | 0.404 | BR |
| 0.15 | 0.067 | 15.00 | 16.22 | 0.93 | 0.566 | BR |
| 0.20 | 0.067 | 15.00 | 14.05 | 1.07 | 0.435 | BR |
| 0.25 | 0.100 | 10.00 | 12.57 | 0.80 | 0.344 | BR |
| 0.35 | 0.167 | 6.00 | 10.68 | 0.56 | 0.484 | BR |
| 0.45 | 0.200 | 5.00 | 9.40 | 0.53 | 0.599 | BR |
| 0.55 | 0.233 | 4.29 | 8.57 | 0.51 | 0.249 | BR |
| 0.65 | 0.233 | 4.29 | 7.85 | 0.55 | 0.221 | BR |
| 0.75 | 0.267 | 3.75 | 7.26 | 0.52 | 0.904 | BR |
| 0.85 | 0.300 | 3.33 | 6.78 | 0.49 | 0.566 | BR |
| 0.90 | 0.300 | 3.33 | 6.62 | 0.50 | 0.894 | BR |

## Wider k₁ Scan (k₂ = 0.5, N=4, d=5.0, μ=9.0)

| k₁ | f (Hz) | T (s) | T_rot (s) | T/T_rot | peak_ratio | Type |
|----|--------|-------|-----------|---------|------------|------|
| 0.05 | 0.233 | 4.29 | 8.89 | 0.48 | 0.655 | BR |
| 0.10 | 0.233 | 4.29 | 8.89 | 0.48 | 0.690 | BR |
| 0.15 | 0.233 | 4.29 | 8.89 | 0.48 | 0.587 | BR |
| 0.70 | 0.133 | 7.50 | 8.89 | 0.84 | 0.520 | BR |
| 0.75 | 0.133 | 7.50 | 8.89 | 0.84 | 0.477 | BR |
| 0.80 | 0.133 | 7.50 | 8.89 | 0.84 | 0.447 | BR |
| 0.85 | 0.133 | 7.50 | 8.89 | 0.84 | 0.340 | BR |
| 0.90 | 0.133 | 7.50 | 8.89 | 0.84 | 0.313 | BR |

## Summary of Parameter Dependence

1. **k₂ is the primary determinant** of breathing period. For k₂ ≥ 0.35:
   $$T_{\text{breath}} \approx 0.5 \times 2\pi/\sqrt{k_2}$$

2. **k₁ modulates** the period, especially at low k₂:
   - k₁ ≤ 0.15: faster breathing (T ≈ 4.29 s at k₂=0.5)
   - k₁ ≥ 0.70: slower breathing (T ≈ 7.50 s at k₂=0.5)
   - Effect diminishes as k₂ increases

3. **d_col** has a non-monotonic effect, with d = 5 giving the shortest period.

4. **μ** stabilizes for μ ≥ 8 (T = 5.0 s at k₁=k₂=0.5). Smaller μ slows breathing.

5. **N** has a non-monotonic effect: N=4 has the shortest period, N=3 is planar.

6. **Random seeds**: The breathing period is robust; most seeds give the same T
   within frequency resolution.

## Comparison with 2D Breathing

| Property | 2D Breathing | 3D Breathing |
|----------|-------------|-------------|
| T/T_rot | ~0.28 | ~0.5 |
| f_breath/f_rot | ~3.57 | ~2.0 |
| Planarity | Planar (σ₃/σ₁ = 0) | Non-planar (σ₃/σ₁ ≈ 0.4–0.8) |
| N dependence | N=5: T ≈ 2.49 s | N=4: T ≈ 5.0 s, N=6: T ≈ 4.55 s |

3D breathing is closer to rigid rotation than 2D breathing, consistent with
3D having more degrees of freedom and thus a less stiff breathing mode.

## Known 3D Breathing Case (N=6, seed=0)

For comparison with the N=4 scan results above:

- **Parameters**: N=6, d=5.0, μ=9.0, k₁=0.5, k₂=0.5, v₀=(1,0,0)
- **Frequency**: f ≈ 0.22 Hz (T ≈ 4.55 s)
- **T/T_rot**: 4.55/8.89 ≈ 0.51
- **σ₃/σ₁**: ≈ 0.83 (non-planar)
- **Initial conditions**: seed=0 from `generate_random_3d_init(N=6, d_col=5.0, seed=0)`