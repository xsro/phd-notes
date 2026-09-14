# Alpha Function Comparison for 3D Fencing Simulation

## Requirements

The repulsive force function α(s) must satisfy:

| # | Requirement | Mathematical Statement |
|---|-------------|----------------------|
| 1 | **Continuous** | α(s) is continuous on [d, ∞) |
| 2 | **Collision singularity** | α(s) → ∞ as s → d⁺ (integral diverges at collision distance) |
| 3 | **Finite sensing range** | α(s) = 0 for s > μ |
| 4 | **Domain** | s ∈ [d, ∞) |
| 5 | **Range** | α(s) ∈ [0, ∞), including 0 |

### Why the original code fails

The original `simulate_3d.py` uses:

```python
def alpha(s):
    if s > mu:
        return 0.0
    if s <= d_col:
        warnings.warn(...)
        return 1e6          # ← DISCONTINUOUS jump!
    return 1.0/(s - d_col) - 1.0/(mu - d_col)
```

At s = d_col, the function jumps from ∞ (theoretical) to 1e6 (numerical floor), breaking continuity. The fix is to remove the special-case branch and let the formula handle the singularity naturally.

---

## Candidate Alpha Functions

All forms below are **continuous** on [d, ∞), satisfy α(s) → ∞ as s → d⁺, and α(s) = 0 for s > μ.

### 1. Standard (Kou-Chen-Xiang)

$$\alpha(s) = \frac{1}{s-d} - \frac{1}{\mu-d} \quad \text{for } d < s \leq \mu$$

| Property | Value |
|----------|-------|
| Singularity type | $1/(s-d)$ — algebraic, order 1 |
| At s = μ | $1/(\mu-d) - 1/(\mu-d) = 0$ ✓ |
| As s → d⁺ | $1/(s-d) \to \infty$ ✓ |
| Derivative | $\alpha'(s) = -1/(s-d)^2$ |

**Notes:** The original form from the paper. Simplest and most well-studied.

---

### 2. Power Law

$$\alpha(s) = \frac{1}{(s-d)^p} - \frac{1}{(\mu-d)^p} \quad \text{for } d < s \leq \mu$$

| Property | Value |
|----------|-------|
| Singularity type | $1/(s-d)^p$ — tunable algebraic order |
| At s = μ | $1/(\mu-d)^p - 1/(\mu-d)^p = 0$ ✓ |
| As s → d⁺ | $1/(s-d)^p \to \infty$ ✓ |
| Derivative | $\alpha'(s) = -p/(s-d)^{p+1}$ |

**Notes:** p = 1 recovers the standard form. p > 1 makes the force stiffer near collision; p < 1 makes it softer.

---

### 3. Rational (Simplest Form)

$$\alpha(s) = \frac{\mu - s}{s - d} \quad \text{for } d < s \leq \mu$$

| Property | Value |
|----------|-------|
| Singularity type | $(\mu-d)/(s-d)$ — algebraic, order 1 |
| At s = μ | $(\mu-\mu)/(\mu-d) = 0$ ✓ |
| As s → d⁺ | $(\mu-d)/(s-d) \to \infty$ ✓ |
| Derivative | $\alpha'(s) = -(\mu-d)/(s-d)^2$ |

**Notes:** The simplest possible rational function satisfying all requirements. No constant offset term. Note that $\alpha(s) = \frac{\mu-s}{s-d} = \frac{\mu-d}{s-d} - 1$, which differs from the standard form by a constant offset of 1.

---

### 4. Logarithmic

$$\alpha(s) = \ln\left(\frac{\mu-d}{s-d}\right) \quad \text{for } d < s \leq \mu$$

| Property | Value |
|----------|-------|
| Singularity type | $-\ln(s-d)$ — logarithmic (weakest) |
| At s = μ | $\ln(1) = 0$ ✓ |
| As s → d⁺ | $\ln(\infty) \to \infty$ ✓ |
| Derivative | $\alpha'(s) = -1/(s-d)$ |

**Notes:** The softest possible singularity. Force grows very slowly near collision. May allow closer approaches before strong repulsion kicks in.

---

### 5. Stiff (Quadratic Singularity)

$$\alpha(s) = \frac{1}{(s-d)^2} - \frac{1}{(\mu-d)^2} \quad \text{for } d < s \leq \mu$$

| Property | Value |
|----------|-------|
| Singularity type | $1/(s-d)^2$ — algebraic, order 2 |
| At s = μ | $1/(\mu-d)^2 - 1/(\mu-d)^2 = 0$ ✓ |
| As s → d⁺ | $1/(s-d)^2 \to \infty$ ✓ |
| Derivative | $\alpha'(s) = -2/(s-d)^3$ |

**Notes:** Twice as stiff as standard near collision. Vehicles will maintain larger pairwise distances.

---

### 6. Exponential (Super-Stiff)

$$\alpha(s) = e^{1/(s-d)} \cdot \frac{\mu-s}{\mu-d} \quad \text{for } d < s \leq \mu$$

| Property | Value |
|----------|-------|
| Singularity type | $\exp(1/(s-d))$ — essential singularity |
| At s = μ | $\exp(1/(\mu-d)) \cdot 0 = 0$ ✓ |
| As s → d⁺ | $\exp(\infty) \cdot 1 \to \infty$ ✓ |
| Derivative | Very steep, grows faster than any power law |

**Notes:** Extremely stiff near collision. Essentially infinite repulsion even for small approach. May be numerically stiff.

---

## Comparison Table

| Form | α(s) formula | Singularity | Stiffness | Numerical stiffness |
|------|-------------|-------------|-----------|-------------------|
| **standard** | $1/(s-d) - 1/(\mu-d)$ | $1/(s-d)$ | moderate | low |
| **power (p=1.5)** | $1/(s-d)^{1.5} - 1/(\mu-d)^{1.5}$ | $1/(s-d)^{1.5}$ | moderate+ | low-moderate |
| **rational** | $(\mu-s)/(s-d)$ | $(\mu-d)/(s-d)$ | moderate | low |
| **log** | $\ln((\mu-d)/(s-d))$ | $-\ln(s-d)$ | weak | very low |
| **stiff** | $1/(s-d)^2 - 1/(\mu-d)^2$ | $1/(s-d)^2$ | strong | moderate |
| **exponential** | $e^{1/(s-d)} \cdot (\mu-s)/(\mu-d)$ | $\exp(1/(s-d))$ | extreme | high |

---

## Key Differences in Dynamics

The singularity type affects:

1. **Effective stiffness**: How strongly vehicles resist close approach. Stronger singularity → larger steady-state pairwise distances.

2. **Breathing amplitude**: Softer singularities may allow larger breathing oscillations since the restoring force is weaker near the equilibrium separation.

3. **Breathing frequency**: The effective spring constant from the repulsive force depends on α'(s) evaluated at the equilibrium separation. Different singularity types give different effective stiffness, hence different breathing frequencies.

4. **Numerical stiffness**: Stronger singularities require smaller integration steps and/or looser tolerances, increasing computation time.

---

## How to Run

```bash
# Run comparison for all alpha forms
python3 compare_alpha_forms.py

# Or run individual forms
python3 simulate_3d_continuous_alpha.py --alpha standard
python3 simulate_3d_continuous_alpha.py --alpha rational
python3 simulate_3d_continuous_alpha.py --alpha stiff
# ... etc
```