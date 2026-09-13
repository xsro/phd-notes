# 3D Breathing Period — Final Conjecture

## Primary Scaling (k₂)

For k₂ ≥ 0.35, the breathing period is set primarily by k₂:

$$T_{\text{breath}} \approx \alpha \cdot \frac{2\pi}{\sqrt{k_2}}, \qquad \alpha \approx 0.5$$

Equivalently, the breathing frequency is ~2× the rigid rotation frequency:

$$f_{\text{breath}} \approx 2 \cdot f_{\text{rot}} = 2 \cdot \frac{\sqrt{k_2}}{2\pi}$$

The √k₂ scaling is the same as rigid rotation — the observer gain k₂ sets the
overall timescale — but the breathing mode is stiffer, giving a shorter period.

## k₁ Modulation

α depends weakly on k₁:

$$\alpha(k_1) \approx \begin{cases}
0.48 & k_1 \leq 0.15 \\
0.50\text{–}0.56 & 0.2 \leq k_1 \leq 0.6 \\
0.84 & k_1 \geq 0.70
\end{cases}$$

k₁ has negligible effect for k₂ ≥ 0.35 in the range k₁ = 0.2–0.6.
At low k₂, increasing k₁ strongly increases α (slows breathing).

## μ Cutoff

For μ ≥ 8, the period is stable (μ does not affect T).
For μ < 8, T increases as μ decreases:

$$T \propto \frac{1}{\mu_{\text{eff}}^2}, \quad \mu_{\text{eff}} < 8$$

## Full Empirical Formula

Combining all effects (for the scanned parameter range):

$$T_{\text{breath}} \approx \alpha(k_1) \cdot \frac{2\pi}{\sqrt{k_2}} \cdot \beta(\mu) \cdot \gamma(d_{\text{col}}, N)$$

where:
- α(k₁) ≈ 0.5 (primary, weak k₁ dependence)
- β(μ) ≈ 1 for μ ≥ 8, β(μ) > 1 for μ < 8
- γ(d_col, N) is a non-monotonic correction of order ~1 (N=4 minimizes T)

## Physical Interpretation

The breathing mode is a **radial oscillation** of the formation. Its frequency
is set by the effective stiffness:

$$\omega_{\text{eff}}^2 = k_2 + C_{\text{rep}}(k_1, d_{\text{col}}, \mu, N)$$

where C_rep comes from the stiff repulsive gradient α'(s) = −1/(s−d)².
Since C_rep > 0, we have ω_eff > √k₂, hence T_breath < T_rot.
The factor α ≈ 0.5 corresponds to C_rep ≈ 3k₂ (since ω_eff² = 5k₂ → ω_eff = √5·√k₂ → T = 2π/(√5·√k₂) ≈ 0.45·2π/√k₂).

## Comparison with 2D

| | 2D Breathing | 3D Breathing |
|---|---|---|
| T/T_rot | ~0.28 | ~0.5 |
| ω_eff/√k₂ | ~3.57 | ~2.0 |
| C_rep/k₂ | ~11.7 | ~3.0 |

3D breathing is softer (closer to rigid rotation) because the extra spatial
dimension provides more degrees of freedom, reducing the effective stiffness
of the radial mode.