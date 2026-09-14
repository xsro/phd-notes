# 2D Breathing: Tangential Period vs Rigid Rotation

## Result: Does NOT satisfy $T_{\text{tan}} \approx T_{\text{rot}}$

The 2D breathing limit cycle **does not** satisfy $T_{\text{tan}} \approx T_{\text{rot}}$.
Instead, the tangential period is much shorter:

$$T_{\text{tan}} \approx 0.28 \times T_{\text{rot}}$$

## Data

For the known 2D breathing case (case3.py ICs, N=5, k₁=k₂=0.5, d=5, μ=9):

| Component | Period (s) | T/T_rot |
|-----------|-----------|---------|
| $T_{\text{rad}}$ (radial) | 2.488 | 0.280 |
| $T_{\text{tan}}$ (tangential) | 2.488 | 0.280 |
| $T_{\text{rot}} = 2\pi/\sqrt{k_2}$ | 8.886 | 1.000 |

**Key difference from 3D**: In 2D, $f_{\text{rad}} = f_{\text{tan}}$ (frequency-locked),
not $f_{\text{rad}} \approx 1.86 \cdot f_{\text{tan}}$ as in 3D breathing.

## Physical Interpretation

The 2D breathing mode is a **strongly stiffened radial-tangential oscillation**:

$$\omega_{\text{eff}}^2 = k_2 + C_{\text{rep}} \approx 12.7 \cdot k_2$$

$$C_{\text{rep}}/k_2 \approx 11.7$$

The repulsive stiffness is ~4× larger in 2D than in 3D (11.7 vs 2.7).
This makes the breathing frequency ~3.6× higher than the rigid rotation frequency,
and the tangential mode is **not neutral** — it is strongly coupled to the radial mode.

### Why the difference?

| | 2D Breathing | 3D Breathing |
|---|---|---|
| $C_{\text{rep}}/k_2$ | ~11.7 | ~2.7 |
| $T_{\text{tan}}/T_{\text{rot}}$ | 0.28 | 0.96 |
| $f_{\text{rad}}/f_{\text{tan}}$ | 1.0 (locked) | 1.86 |
| Planarity σ₃/σ₁ | 0 (planar) | 0.4–0.9 |

In 2D, all motion is confined to a plane. The radial and tangential directions
are tightly coupled through the stiff repulsive force, causing them to oscillate
at the same frequency. The effective stiffness is very high because vehicles
in 2D have fewer degrees of freedom to avoid each other, leading to stronger
repulsive interactions.

In 3D, the extra spatial dimension allows vehicles to "dodge" each other more
easily, reducing the effective repulsive stiffness. The tangential mode remains
approximately neutral (close to rigid rotation), while the radial mode oscillates
at ~2× the tangential frequency.

## Exceptions and Special Cases

### 1. 2D rigid rotation (not breathing)

For most 2D initial conditions, the system converges to **rigid rotation**:
pairwise distances are constant, and the formation rotates at exactly
$\omega = \sqrt{k_2}$. In this case:

$$T_{\text{tan}} = T_{\text{rot}} \quad \text{(exactly)}$$

This is the "trivial" case where the conclusion holds, but there is no breathing.

### 2. 2D breathing with different N

The known 2D breathing case uses N=5. For other N values:
- **N=3**: May converge to rigid rotation or a different attractor
- **N=4**: Limited data; behavior depends on initial conditions
- **N≥6**: Not systematically studied; 2D simulation is very slow

### 3. 2D breathing with different parameters

The case3.py breathing case uses k₁=k₂=0.5. Varying these parameters:
- **k₁ → 0**: Attractive force weakens; breathing may become unstable
- **k₁ → large**: System may converge to rigid rotation
- **k₂ → 0**: Observer dynamics slow down; period increases
- **k₂ → large**: System may converge to rigid rotation

The 2D breathing attractor appears to be **isolated** — it exists only for
specific initial conditions (case3.py ICs) and a narrow parameter range.

## Summary

| | 2D | 3D |
|---|---|---|
| $T_{\text{tan}} \approx T_{\text{rot}}$? | **No** (T_tan = 0.28×T_rot) | **Yes** (T_tan = 0.96×T_rot) |
| $f_{\text{rad}}/f_{\text{tan}}$ | 1.0 (locked) | 1.86 (breathing) |
| $C_{\text{rep}}/k_2$ | ~11.7 | ~2.7 |
| Regime | Strongly coupled oscillation | Radial breathing + neutral rotation |

The conclusion $T_{\text{tan}} \approx T_{\text{rot}}$ is a **3D-specific result**.
In 2D, the tangential mode is strongly coupled to the radial mode, and both
oscillate at a frequency much higher than $\sqrt{k_2}$.