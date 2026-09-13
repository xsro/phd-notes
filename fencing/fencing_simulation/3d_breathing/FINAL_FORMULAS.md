# 3D Breathing Period — Final Conjecture

## Primary Scaling (k₂)

For k₂ ≥ 0.30, the breathing period is set primarily by k₂:

$$T_{\text{breath}} \approx \alpha \cdot \frac{2\pi}{\sqrt{k_2}}, \qquad \alpha \approx 0.52$$

Equivalently:

$$f_{\text{breath}} \approx 1.9 \cdot f_{\text{rot}} = 1.9 \cdot \frac{\sqrt{k_2}}{2\pi}$$

## C_rep: Repulsive Stiffness Contribution

Define:

$$C_{\text{rep}} \equiv k_2 \left[ \left( \frac{T_{\text{rot}}}{T_{\text{breath}}} \right)^2 - 1 \right]$$

so that:

$$\omega_{\text{eff}}^2 = k_2 + C_{\text{rep}}, \qquad
T_{\text{breath}} = \frac{2\pi}{\sqrt{k_2 + C_{\text{rep}}}}$$

C_rep quantifies how much the central repulsive force stiffens the radial mode
beyond the observer gain k₂ alone. It depends on N, d_col, μ, and k₁.

### Fine Scan Data (T_MAX=120s, DT=0.05s, resolution=0.0083 Hz)

#### C_rep/k₂ vs k₂ (k₁=0.5, N=4, d=5, μ=9)

| k₂ | T_breath (s) | T_rot (s) | C_rep/k₂ |
|----|-------------|-----------|----------|
| 0.25 | 10.000 | 12.566 | 0.58 |
| 0.30 | 6.000 | 11.471 | 2.66 |
| 0.35 | 5.455 | 10.621 | 2.79 |
| 0.40 | 5.000 | 9.935 | 2.95 |
| 0.45 | 5.000 | 9.366 | 2.51 |
| 0.50 | 4.615 | 8.886 | 2.71 |
| 0.55 | 4.286 | 8.472 | 2.91 |
| 0.60 | 4.286 | 8.112 | 2.58 |
| 0.65 | 4.000 | 7.793 | 2.80 |
| 0.70 | 3.750 | 7.510 | 3.01 |

For k₂ ≥ 0.30: C_rep/k₂ ≈ 2.5–3.0 (mean = 2.67, std = 0.15).
Sharp transition at k₂ ≈ 0.25–0.30.

#### C_rep/k₂ vs k₁ (k₂=0.5, N=4, d=5, μ=9)

| k₁ | T_breath (s) | C_rep/k₂ |
|----|-------------|----------|
| 0.15–0.65 | 4.615 | 2.71 |
| 0.70 | 7.500 | 0.40 |

Sharp transition between k₁ = 0.65 and 0.70. For k₁ ≤ 0.65, C_rep/k₂ = 2.71.
For k₁ ≥ 0.70, C_rep/k₂ = 0.40.

#### C_rep/k₂ vs d_col (k₁=k₂=0.5, N=4, μ=9)

| d_col | T_breath (s) | C_rep/k₂ |
|-------|-------------|----------|
| 3.0–6.5 | 4.615 | 2.71 |
| 7.0 | 7.500 | 0.40 |

Sharp transition between d = 6.5 and 7.0.

#### C_rep/k₂ vs μ (k₁=k₂=0.5, N=4, d=5)

| μ | T_breath (s) | C_rep/k₂ |
|---|-------------|----------|
| 6.0–6.5 | 7.500 | 0.40 |
| 7.0–11.5 | 4.615 | 2.71 |

Sharp transition between μ = 6.5 and 7.0.

#### C_rep/k₂ vs N (k₁=k₂=0.5, d=5, μ=9)

| N | T_breath (s) | C_rep/k₂ | σ₃/σ₁ |
|---|-------------|----------|-------|
| 3 | 6.000 | 1.19 | 0.000 (planar) |
| 4 | 4.615 | 2.71 | 0.640 |
| 5 | 7.500 | 0.40 | 0.582 |
| 6 | 4.615 | 2.71 | 0.892 |

Non-monotonic: N=4 and N=6 give high C_rep, N=3 gives intermediate (planar),
N=5 gives low C_rep.

#### C_rep/k₂ vs seed (N=4, k₁=k₂=0.5, d=5, μ=9)

| seed | T_breath (s) | C_rep/k₂ |
|------|-------------|----------|
| 0–4 | 4.615 | 2.71 |
| 5–6 | 5.000 | 2.16 |
| 7–9 | 4.615 | 2.71 |

8/10 seeds give C_rep/k₂ = 2.71, 2/10 give 2.16. Robust.

## Key Insight: Quantization

The C_rep/k₂ values cluster around discrete levels (0.40, 1.19, 2.16, 2.71)
because the frequency resolution (0.0083 Hz with T_MAX=120s) still quantizes
T_breath to a few values (3.75, 4.0, 4.29, 4.62, 5.0, 5.45, 6.0, 7.5, 10.0 s).
The true C_rep is likely a smooth function of parameters.

## Physical Interpretation

C_rep originates from the Jacobian of the central repulsive force:

$$\phi_i = \sum_{j \neq i} \alpha(|x_i - x_j|) \frac{x_i - x_j}{|x_i - x_j|}$$

Linearizing around the equilibrium configuration and projecting onto the radial
direction gives:

$$C_{\text{rep}} \propto N \cdot [-\alpha'(s_{\text{eq}})] \cdot \text{geom}(N, \sigma_3/\sigma_1)$$

where α'(s) = −1/(s−d)². Thus:

- **d_col ↓** → s_eq − d ↓ → α'(s_eq) ↑ → C_rep ↑ (strongly, as 1/(s−d)²)
- **N** → more pairwise interactions → C_rep ↑ (but geometric factor can reduce it)
- **μ** → affects s_eq → C_rep changes (saturates for μ ≥ 7)
- **k₁ ↑** → attractive force dominates → equilibrium spacing changes → C_rep ↓

The sharp transitions suggest that C_rep is not a smooth function but rather
depends on whether the equilibrium configuration supports a breathing mode
with sufficient radial stiffness. The transition boundaries in parameter space
are likely bifurcation surfaces.

## Comparison with 2D Breathing

| | 2D Breathing | 3D Breathing |
|---|---|---|
| T/T_rot | ~0.28 | ~0.52 |
| C_rep/k₂ | ~11.7 | ~2.7 |
| Planarity | Planar (σ₃/σ₁ = 0) | Non-planar (σ₃/σ₁ ≈ 0.4–0.9) |

3D breathing has lower C_rep (softer radial mode) because the extra spatial
dimension provides more degrees of freedom, reducing the effective stiffness.

## Known 3D Breathing Case (N=6, seed=0)

- **Parameters**: N=6, d=5.0, μ=9.0, k₁=0.5, k₂=0.5
- **T_breath** ≈ 4.55 s, **T_rot** = 8.89 s
- **T/T_rot** ≈ 0.51 → **C_rep/k₂** ≈ 2.84
- **σ₃/σ₁** ≈ 0.83 (strongly non-planar)

Consistent with the fine scan: N=6 gives C_rep/k₂ ≈ 2.71, matching the
N=4 result within resolution.