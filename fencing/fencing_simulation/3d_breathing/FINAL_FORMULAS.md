# 3D Breathing Period — Final Conjecture

## Primary Scaling (k₂)

For k₂ ≥ 0.35, the breathing period is set primarily by k₂:

$$T_{\text{breath}} \approx \alpha \cdot \frac{2\pi}{\sqrt{k_2}}, \qquad \alpha \approx 0.5$$

Equivalently, the breathing frequency is ~2× the rigid rotation frequency:

$$f_{\text{breath}} \approx 2 \cdot f_{\text{rot}} = 2 \cdot \frac{\sqrt{k_2}}{2\pi}$$

The √k₂ scaling is the same as rigid rotation — the observer gain k₂ sets the
overall timescale — but the breathing mode is stiffer, giving a shorter period.

## C_rep: Repulsive Stiffness Contribution

Define the repulsive stiffness contribution as:

$$C_{\text{rep}} \equiv k_2 \left[ \left( \frac{T_{\text{rot}}}{T_{\text{breath}}} \right)^2 - 1 \right]$$

so that the effective breathing frequency is:

$$\omega_{\text{eff}}^2 = k_2 + C_{\text{rep}}, \qquad
T_{\text{breath}} = \frac{2\pi}{\sqrt{k_2 + C_{\text{rep}}}}$$

C_rep quantifies how much the central repulsive force stiffens the radial mode
beyond the observer gain k₂ alone. It depends on N, d_col, μ, and k₁.

### C_rep vs k₂ (k₁=0.5, N=4, d=5, μ=9)

| k₂ | T_breath (s) | T_rot (s) | C_rep | C_rep/k₂ |
|----|-------------|-----------|-------|----------|
| 0.05 | 30.00 | 28.10 | −0.01 | −0.12 |
| 0.10 | 30.00 | 19.87 | −0.06 | −0.56 |
| 0.15 | 15.00 | 16.22 | 0.03 | 0.17 |
| 0.20 | 15.00 | 14.05 | −0.03 | −0.12 |
| 0.25 | 10.00 | 12.57 | 0.15 | 0.58 |
| 0.35 | 6.00 | 10.68 | 0.76 | 2.17 |
| 0.45 | 5.00 | 9.40 | 1.14 | 2.53 |
| 0.55 | 4.29 | 8.57 | 1.65 | 2.99 |
| 0.65 | 4.29 | 7.85 | 1.53 | 2.35 |
| 0.75 | 3.75 | 7.26 | 2.06 | 2.75 |
| 0.85 | 3.33 | 6.78 | 2.67 | 3.15 |
| 0.90 | 3.33 | 6.62 | 2.66 | 2.95 |

For k₂ ≥ 0.35, C_rep/k₂ ≈ 2.2–3.2 (mean ≈ 2.6). For k₂ < 0.35, C_rep
vanishes or becomes negative — the breathing mode softens and T_breath ≥ T_rot.

### C_rep vs k₁ (k₂=0.5, N=4, d=5, μ=9)

| k₁ | T_breath (s) | C_rep | C_rep/k₂ |
|----|-------------|-------|----------|
| 0.05 | 4.29 | 1.65 | 3.29 |
| 0.10 | 4.29 | 1.65 | 3.29 |
| 0.15 | 4.29 | 1.65 | 3.29 |
| 0.70 | 7.50 | 0.20 | 0.41 |
| 0.75 | 7.50 | 0.20 | 0.41 |
| 0.80 | 7.50 | 0.20 | 0.41 |
| 0.85 | 7.50 | 0.20 | 0.41 |
| 0.90 | 7.50 | 0.20 | 0.41 |

Two regimes separated by a sharp transition in k₁ ∈ (0.15, 0.70):
- k₁ ≤ 0.15: C_rep/k₂ ≈ 3.3 (stiff breathing)
- k₁ ≥ 0.70: C_rep/k₂ ≈ 0.4 (soft breathing)

### C_rep vs d_col (k₁=k₂=0.5, N=4, μ=9)

| d_col | T_breath (s) | C_rep | C_rep/k₂ |
|-------|-------------|-------|----------|
| 3 | 4.29 | 1.65 | 3.29 |
| 4 | 7.50 | 0.20 | 0.41 |
| 5 | 5.00 | 1.08 | 2.16 |
| 6 | 6.00 | 0.60 | 1.20 |
| 7 | 7.50 | 0.20 | 0.41 |

Non-monotonic. d=3 gives the highest C_rep (vehicles closest to collision
boundary → α'(s) = −1/(s−d)² is largest). d=4 and d=7 give the lowest C_rep.

### C_rep vs μ (k₁=k₂=0.5, N=4, d=5)

| μ | T_breath (s) | C_rep | C_rep/k₂ |
|---|-------------|-------|----------|
| 6 | 7.50 | 0.20 | 0.41 |
| 7 | 6.00 | 0.60 | 1.20 |
| 8 | 5.00 | 1.08 | 2.16 |
| 9 | 5.00 | 1.08 | 2.16 |
| 10 | 5.00 | 1.08 | 2.16 |
| 11 | 5.00 | 1.08 | 2.16 |
| 12 | 5.00 | 1.08 | 2.16 |

Monotonic increase for μ < 8, then saturation at C_rep/k₂ ≈ 2.2 for μ ≥ 8.

### C_rep vs N (k₁=k₂=0.5, d=5, μ=9)

| N | T_breath (s) | C_rep | C_rep/k₂ |
|---|-------------|-------|----------|
| 3 | 6.00 | 0.60 | 1.20 |
| 4 | 5.00 | 1.08 | 2.16 |
| 5 | 7.50 | 0.20 | 0.41 |

Non-monotonic. N=4 maximizes C_rep. N=3 is planar (σ₃/σ₁=0), N=5 is softer.

### C_rep vs seed (N=4, k₁=k₂=0.5, d=5, μ=9)

| seed | T_breath (s) | C_rep | C_rep/k₂ |
|------|-------------|-------|----------|
| 0 | 5.00 | 1.08 | 2.16 |
| 1 | 5.00 | 1.08 | 2.16 |
| 2 | 5.00 | 1.08 | 2.16 |
| 3 | 4.29 | 1.65 | 3.29 |
| 4 | 5.00 | 1.08 | 2.16 |
| 5 | 5.00 | 1.08 | 2.16 |

Robust: 5/6 seeds give C_rep/k₂ = 2.16. One outlier (seed=3) gives 3.29.

## Physical Interpretation

C_rep originates from the Jacobian of the central repulsive force:

$$\phi_i = \sum_{j \neq i} \alpha(|x_i - x_j|) \frac{x_i - x_j}{|x_i - x_j|}$$

Linearizing around the equilibrium configuration and projecting onto the radial
direction gives:

$$C_{\text{rep}} \propto N \cdot [-\alpha'(s_{\text{eq}})] \cdot \text{geom}(N, \sigma_3/\sigma_1)$$

where α'(s) = −1/(s−d)². Thus:

- **d_col ↓** → s_eq − d ↓ → α'(s_eq) ↑ → C_rep ↑ (strongly, as 1/(s−d)²)
- **N ↑** → more pairwise interactions → C_rep ↑ (but geometric factor can reduce it)
- **μ ↑** → s_eq can increase → α'(s_eq) ↓ → C_rep ↓ (until μ ≥ 8 where s_eq saturates)
- **k₁ ↑** → attractive force dominates → equilibrium spacing changes → C_rep ↓

The observed quantization of C_rep into discrete levels (0.4, 1.2, 2.2, 3.3)
is an artifact of the frequency resolution (0.033 Hz, T quantized to 4.29, 5.0,
6.0, 7.5 s). The true C_rep is likely smoother.

## Comparison with 2D Breathing

| | 2D Breathing | 3D Breathing |
|---|---|---|
| T/T_rot | ~0.28 | ~0.5 |
| C_rep/k₂ | ~11.7 | ~2.2 |
| Planarity | Planar (σ₃/σ₁ = 0) | Non-planar (σ₃/σ₁ ≈ 0.4–0.8) |

3D breathing has lower C_rep (softer radial mode) because the extra spatial
dimension provides more degrees of freedom, reducing the effective stiffness.

## Known 3D Breathing Case (N=6, seed=0)

- **Parameters**: N=6, d=5.0, μ=9.0, k₁=0.5, k₂=0.5
- **T_breath** ≈ 4.55 s, **T_rot** = 8.89 s
- **T/T_rot** ≈ 0.51 → **C_rep/k₂** ≈ 2.84
- **σ₃/σ₁** ≈ 0.83 (strongly non-planar)

Consistent with the N=4 scan: N=6 gives slightly higher C_rep than N=4,
following the non-monotonic N dependence.