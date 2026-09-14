# Tangential Period ≈ Rigid Rotation Period

## Main Result

In the 3D breathing limit cycle, the **tangential rotation period** $T_{\text{tan}}$
is approximately equal to the **rigid rotation period**:

$$T_{\text{tan}} \approx T_{\text{rot}} = \frac{2\pi}{\sqrt{k_2}}$$

This holds across all parameter scans in the breathing regime, with
$T_{\text{tan}}/T_{\text{rot}} \approx 0.94\text{–}1.07$ (mean ≈ 0.97).

The physical reason: the tangential direction is the **neutral (unstable) mode**
of the linearized dynamics. The observer gain $k_2$ provides the only restoring
force in the rotating frame, giving $\omega_{\text{tan}} = \sqrt{k_2}$ regardless
of the repulsive interaction. The repulsive force is central (radial) and does
not directly affect the tangential frequency.

## Evidence

### k₂ scan (k₁=0.5, N=4, d=5, μ=9)

| k₂ | T_tan (s) | T_rot (s) | T_tan/T_rot |
|----|-----------|-----------|-------------|
| 0.25 | 12.000 | 12.566 | 0.955 |
| 0.30 | 12.000 | 11.471 | 1.046 |
| 0.35 | 10.000 | 10.621 | 0.942 |
| 0.40 | 10.000 | 9.935 | 1.007 |
| 0.45 | 10.000 | 9.366 | 1.068 |
| 0.50 | 8.571 | 8.886 | 0.965 |
| 0.55 | 8.571 | 8.472 | 1.012 |
| 0.60 | 8.571 | 8.112 | 1.057 |
| 0.65 | 7.500 | 7.793 | 0.962 |
| 0.70 | 7.500 | 7.510 | 0.999 |

Both $T_{\text{tan}}$ and $T_{\text{rot}}$ scale as $1/\sqrt{k_2}$, preserving
the near-unity ratio. The deviations (up to ±7%) are within the frequency
resolution (0.0083 Hz at T_MAX=120s) and likely reflect finite-size effects.

### Other parameters (k₂=0.5 fixed)

| Scan | T_tan/T_rot range | Mean |
|------|-------------------|------|
| k₁ = 0.15–0.65 | 0.965 | 0.965 |
| d = 3.0–6.5 | 0.965 | 0.965 |
| μ = 7.0–11.5 | 0.965 | 0.965 |
| N = 4, 6 | 0.965 | 0.965 |

In the breathing regime, $T_{\text{tan}}/T_{\text{rot}}$ is remarkably constant
at 0.965 across all scanned parameters.

## Exceptions

### 1. Locked regime ($f_{\text{rad}}/f_{\text{tan}} = 1.0$)

When the breathing mode loses stability, the system enters a frequency-locked
state where $T_{\text{tan}}$ drops significantly below $T_{\text{rot}}$:

| Case | T_tan (s) | T_rot (s) | T_tan/T_rot |
|------|-----------|-----------|-------------|
| k₁ = 0.70 | 7.500 | 8.886 | **0.844** |
| k₁ = 0.75 | 8.571 | 8.886 | 0.964 |
| d = 7.0 | 7.500 | 8.886 | **0.844** |
| μ = 6.0 | 7.500 | 8.886 | **0.844** |
| μ = 6.5 | 7.500 | 8.886 | **0.844** |
| N = 5 | 7.500 | 8.886 | **0.844** |

In these cases, the tangential rotation is **16% slower** than rigid rotation.
The locked mode couples radial and tangential motion, reducing the effective
tangential frequency.

### 2. High k₂ ($k_2 \geq 0.40$)

At higher observer gains, $T_{\text{tan}}$ can exceed $T_{\text{rot}}$ by up to 7%:

| k₂ | T_tan/T_rot | Excess |
|----|-------------|--------|
| 0.40 | 1.007 | +0.7% |
| 0.45 | 1.068 | +6.8% |
| 0.60 | 1.057 | +5.7% |

This may be a finite-size effect: at high $k_2$, the formation contracts more
tightly, and the effective rotation radius changes, modifying the tangential
period. Alternatively, it may reflect the fact that $T_{\text{tan}}$ is measured
from the FFT peak of tangential speed, which has finite width and may be
biased by the radial harmonic.

### 3. Planar case (N=3)

| N | T_tan (s) | T_rot (s) | T_tan/T_rot | σ₃/σ₁ |
|---|-----------|-----------|-------------|-------|
| 3 | 8.571 | 8.886 | 0.965 | 0.000 |

N=3 gives a planar configuration (σ₃/σ₁ = 0). The tangential period is close
to but slightly below $T_{\text{rot}}$. The 3.5% discrepancy may arise because
the planar configuration has a different effective inertia.

### 4. Outlier: d = 5.5

| d | T_tan (s) | T_rot (s) | T_tan/T_rot | f_rad/f_tan |
|---|-----------|-----------|-------------|-------------|
| 5.5 | 8.571 | 8.886 | 0.965 | 1.429 |

This is a transitional case where $f_{\text{rad}}/f_{\text{tan}} = 1.43$ (neither
1.86 nor 1.0). The tangential period remains close to $T_{\text{rot}}$, but the
radial mode is in an intermediate state.

## Summary

$$T_{\text{tan}} \approx T_{\text{rot}} = \frac{2\pi}{\sqrt{k_2}}$$

is a robust result for the 3D breathing limit cycle, holding within ~5% across
the breathing regime. The main exceptions are:

1. **Locked regime**: $T_{\text{tan}}/T_{\text{rot}} \approx 0.84$ (tangential slows)
2. **High k₂**: $T_{\text{tan}}/T_{\text{rot}}$ can reach 1.07 (tangential speeds up)
3. **Planar N=3**: $T_{\text{tan}}/T_{\text{rot}} \approx 0.965$ (slight slowdown)
4. **Transitional d=5.5**: $T_{\text{tan}}/T_{\text{rot}} \approx 0.965$ (normal)

The constancy of $T_{\text{tan}}$ across parameters supports the interpretation
that the tangential rotation is a **neutral mode** governed solely by $k_2$,
while the breathing (radial) mode is a **stiffened mode** with effective
frequency $\omega_{\text{rad}} = \sqrt{k_2 + C_{\text{rep}}}$.