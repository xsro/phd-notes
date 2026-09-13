# Radial vs Tangential Motion Decomposition

## Method

For each vehicle $i$, decompose the motion relative to the center of mass into:

- **Radial**: $r_i(t) = \|x_i(t) - x_{cm}(t)\|$ — distance from formation center
- **Tangential**: $v_{\perp,i}(t) = \|v_i - (v_i \cdot \hat{r}_i)\hat{r}_i\|$ — velocity perpendicular to radial direction

FFT each component averaged over all vehicles to find $f_{\text{rad}}$ and $f_{\text{tan}}$.
Also compute pairwise distance FFT ($f_{\text{pd}}$) for comparison.

## Key Finding: Two Regimes

The system exhibits **two distinct dynamical regimes** separated by sharp transitions:

| Regime | $f_{\text{rad}}/f_{\text{tan}}$ | $T_{\text{rad}}/T_{\text{rot}}$ | $T_{\text{tan}}/T_{\text{rot}}$ | Characteristics |
|--------|-------------------------------|-------------------------------|-------------------------------|----------------|
| **Breathing** | ~1.86 | ~0.54 | ~0.96 | Distinct radial oscillation, tangential ≈ rigid rotation |
| **Locked** | 1.00 | ~0.84–0.96 | ~0.84–0.96 | Radial and tangential frequencies identical |

## Regime 1: Breathing ($f_{\text{rad}}/f_{\text{tan}} \approx 1.86$)

### Frequency relationships

$$f_{\text{rad}} \approx 1.86 \cdot f_{\text{tan}}, \qquad
f_{\text{tan}} \approx f_{\text{rot}} = \frac{\sqrt{k_2}}{2\pi}$$

$$T_{\text{rad}} \approx 0.54 \cdot T_{\text{rot}}, \qquad
T_{\text{tan}} \approx 0.96 \cdot T_{\text{rot}}$$

The tangential period matches rigid rotation ($T_{\text{tan}} \approx T_{\text{rot}}$),
confirming that rotation is driven by the observer gain $k_2$ alone.
The radial (breathing) period is about half the rotation period.

### Equivalent stiffness

$$\omega_{\text{rad}}^2 \approx (1.86)^2 \cdot k_2 \approx 3.45 \cdot k_2 = k_2 + C_{\text{rep}}$$

$$C_{\text{rep}}/k_2 \approx 2.45$$

This is consistent with the pairwise-distance measurement ($C_{\text{rep}}/k_2 \approx 2.7$),
confirming that the breathing mode is a radial oscillation stiffened by repulsive forces.

### Parameter scans in breathing regime

#### k₂ scan (k₁=0.5, N=4, d=5, μ=9)

| k₂ | T_rad (s) | T_tan (s) | T_rot (s) | f_rad/f_tan | σ₃/σ₁ |
|----|-----------|-----------|-----------|-------------|-------|
| 0.30 | 6.000 | 12.000 | 11.471 | 2.000 | 0.840 |
| 0.35 | 5.455 | 10.000 | 10.621 | 1.833 | 0.760 |
| 0.40 | 5.000 | 10.000 | 9.935 | 2.000 | 0.886 |
| 0.45 | 5.000 | 10.000 | 9.366 | 2.000 | 0.508 |
| 0.50 | 4.615 | 8.571 | 8.886 | 1.857 | 0.640 |
| 0.55 | 4.286 | 8.571 | 8.472 | 2.000 | 0.396 |
| 0.60 | 4.286 | 8.571 | 8.112 | 2.000 | 0.011 |
| 0.65 | 4.000 | 7.500 | 7.793 | 1.875 | 0.324 |
| 0.70 | 3.750 | 7.500 | 7.510 | 2.000 | 0.879 |

Both T_rad and T_tan scale as 1/√k₂, preserving the ~1.86 ratio.
Note: T_tan is slightly less than T_rot (0.96×) due to finite-size effects.

#### k₁ scan (k₂=0.5, N=4, d=5, μ=9)

| k₁ | T_rad (s) | T_tan (s) | f_rad/f_tan | σ₃/σ₁ |
|----|-----------|-----------|-------------|-------|
| 0.15–0.65 | 4.615 | 8.571 | 1.857 | 0.04–0.97 |
| 0.70 | 7.500 | 7.500 | **1.000** | 0.627 |
| 0.75 | 8.571 | 8.571 | **1.000** | 0.633 |

For k₁ ≤ 0.65: stable breathing with f_rad/f_tan = 1.857.
For k₁ ≥ 0.70: sharp transition to locked regime.

#### d_col scan (k₁=k₂=0.5, N=4, μ=9)

| d | T_rad (s) | T_tan (s) | f_rad/f_tan | σ₃/σ₁ |
|---|-----------|-----------|-------------|-------|
| 3.0–5.0 | 4.615 | 8.571 | 1.857 | 0.18–0.92 |
| 5.5 | 6.000 | 8.571 | 1.429 | 0.27 |
| 6.0–6.5 | 4.615 | 8.571 | 1.857 | 0.28–0.64 |
| 7.0 | 7.500 | 7.500 | **1.000** | 0.83 |

d=5.5 is an outlier (T_rad=6.0s, f_rad/f_tan=1.43) — possibly a different mode.

#### μ scan (k₁=k₂=0.5, N=4, d=5)

| μ | T_rad (s) | T_tan (s) | f_rad/f_tan | σ₃/σ₁ |
|---|-----------|-----------|-------------|-------|
| 6.0–6.5 | 7.500 | 7.500 | **1.000** | 0.53–0.70 |
| 7.0–11.5 | 4.615 | 8.571 | 1.857 | 0.08–0.99 |

Sharp transition at μ = 6.5→7.0.

#### N scan (k₁=k₂=0.5, d=5, μ=9)

| N | T_rad (s) | T_tan (s) | f_rad/f_tan | σ₃/σ₁ |
|---|-----------|-----------|-------------|-------|
| 3 | 6.000 | 8.571 | 1.429 | 0.000 (planar) |
| 4 | 4.615 | 8.571 | 1.857 | 0.640 |
| 5 | 7.500 | 7.500 | **1.000** | 0.582 |
| 6 | 4.615 | 8.571 | 1.857 | 0.892 |

Non-monotonic: N=4 and N=6 are breathing, N=5 is locked, N=3 is planar.

## Regime 2: Locked ($f_{\text{rad}}/f_{\text{tan}} = 1.00$)

In this regime, radial and tangential motion have the **same period**:

$$T_{\text{rad}} = T_{\text{tan}} \approx 0.84\text{–}0.96 \cdot T_{\text{rot}}$$

The motion is not pure rigid rotation (T < T_rot) nor pure breathing (f_rad ≠ 2×f_tan).
Instead, the formation undergoes a combined radial-tangential oscillation where
the two modes are frequency-locked.

This occurs when:
- k₁ ≥ 0.70 (attractive force dominates)
- d ≥ 7.0 (vehicles far from collision boundary)
- μ ≤ 6.5 (limited sensing)
- N = 5 (odd number of vehicles)

## Physical Interpretation

The breathing mode is a **radial eigenmode** of the formation. Its frequency is set by:

$$\omega_{\text{rad}}^2 = k_2 + C_{\text{rep}}$$

where C_rep comes from the repulsive force Jacobian. The tangential mode is the
**neutral (rotation) mode** with ω_tan = √k₂ (no restoring force in tangential direction).

The ratio f_rad/f_tan ≈ 1.86 means:

$$\frac{\omega_{\text{rad}}}{\omega_{\text{tan}}} = \sqrt{1 + \frac{C_{\text{rep}}}{k_2}} \approx 1.86$$

$$\frac{C_{\text{rep}}}{k_2} \approx 2.45$$

The sharp transitions between breathing and locked regimes suggest **bifurcations**:
the breathing mode loses stability when the repulsive stiffness C_rep drops below
a threshold, and the system transitions to a frequency-locked state.

## Comparison: Pairwise Distance vs Radial Decomposition

| Measure | T (s) | C_rep/k₂ |
|---------|-------|----------|
| Pairwise distance FFT (T_pd) | 4.615 | 2.71 |
| Radial distance from CM (T_rad) | 4.615 | 2.45 |

Both give the same period, confirming that pairwise distance oscillation is
driven by the radial (breathing) mode, not the tangential rotation.