# 3D Breathing Parameter Scan — All Alpha Forms: Results Summary

## Overview

All 6 continuous alpha function forms were scanned across k1, k2, d_col, mu, N, and a (k1,k2) grid.
Every single run produced **breathing behavior** (is_breathing=True), confirming that the 3D breathing
attractor is robust across alpha function choices.

**Key finding**: The breathing period T shows nearly **identical dependence on k2** across all forms,
while the breathing **strength** (peak_ratio) and **non-planarity** (sigma_ratio) differ.

---

## 1. Period vs k2 (k1=0.5, N=6, d=5, mu=9)

The most important scan — how the breathing period scales with observer gain.

| k2 | standard | power | rational | log | stiff | exponential | 2π/√k2 |
|----|----------|-------|----------|-----|-------|-------------|--------|
| 0.1 | 10.0s | 20.0s | 10.0s | 10.0s | 20.0s | 10.0s | 19.87s |
| 0.2 | 10.0s | 10.0s | 10.0s | 10.0s | 10.0s | 10.0s | 14.05s |
| 0.3 | 6.67s | 6.67s | 6.67s | 6.67s | 6.67s | 6.67s | 11.47s |
| 0.4 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 9.94s |
| 0.5 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 8.89s |
| 0.6 | 4.0s | 4.0s | 4.0s | 4.0s | 4.0s | 4.0s | 8.11s |
| 0.7 | 5.0s | 4.0s | 4.0s | 4.0s | 4.0s | 4.0s | 7.51s |
| 0.8 | 3.33s | 3.33s | 3.33s | 4.0s | 4.0s | 4.0s | 7.03s |

**Observations:**
- All forms show T decreasing with increasing k2 (as expected — stronger observer → faster convergence)
- T/T_rot ratio ranges from ~0.47 to ~1.0, with most forms clustering around 0.5-0.7
- The **power** and **stiff** forms show T = 20s at k2=0.1 (matching T_rot), while others show 10s
- At k2 ≥ 0.3, all forms agree within FFT frequency resolution (0.05-0.1 Hz)
- The period is set primarily by k2, not by the alpha function form

---

## 2. Period vs k1 (k2=0.5, N=6, d=5, mu=9)

| k1 | standard | power | rational | log | stiff | exponential |
|----|----------|-------|----------|-----|-------|-------------|
| 0.1 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 0.2 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 0.3 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 0.4 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 0.5 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 0.6 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 0.7 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 0.8 | 5.0s | 10.0s | 5.0s | 5.0s | 10.0s | 5.0s |

**Observations:**
- Period is **nearly independent of k1** for all forms (T ≈ 5s for k1 = 0.1-0.7)
- At k1=0.8, power and stiff forms show T=10s (possibly a bifurcation or mode change)
- This confirms that k1 (attractive gain) does not significantly affect the breathing frequency

---

## 3. Breathing Strength (peak_ratio) Comparison

The peak_ratio measures how much of the pairwise distance power is in the dominant frequency.
Higher = stronger, cleaner breathing oscillation.

### k2 scan (k1=0.5)

| k2 | standard | power | rational | log | stiff | exponential |
|----|----------|-------|----------|-----|-------|-------------|
| 0.1 | 0.459 | 0.612 | 0.522 | 0.503 | 0.600 | 0.535 |
| 0.2 | 0.515 | 0.526 | 0.531 | 0.446 | 0.503 | 0.423 |
| 0.3 | 0.651 | 0.654 | 0.615 | 0.510 | 0.645 | 0.551 |
| 0.4 | 0.742 | 0.687 | 0.877 | 0.951 | 0.637 | 0.921 |
| 0.5 | 0.574 | 0.638 | 0.556 | 0.741 | 0.686 | 0.504 |
| 0.6 | 0.516 | 0.621 | 0.897 | 0.669 | 0.512 | 0.577 |
| 0.7 | 0.373 | 0.548 | 0.825 | 0.919 | 0.825 | 0.737 |
| 0.8 | 0.527 | 0.444 | 0.479 | 0.435 | 0.456 | 0.570 |

**Observations:**
- **log** and **exponential** forms show the highest peak ratios (up to 0.95), indicating very clean, strong breathing
- **standard** form has moderate peak ratios (0.37-0.74)
- **rational** form shows strong breathing at higher k2 (0.82-0.90 at k2=0.6,0.7)
- No single form is uniformly strongest — the optimal form depends on k2

---

## 4. Non-Planarity (sigma_ratio = σ₃/σ₁) Comparison

Higher = more non-planar (genuine 3D behavior).

### k2 scan (k1=0.5)

| k2 | standard | power | rational | log | stiff | exponential |
|----|----------|-------|----------|-----|-------|-------------|
| 0.1 | 0.942 | 0.916 | 0.866 | 0.794 | 0.940 | 0.826 |
| 0.2 | 0.846 | 0.746 | 0.841 | 0.889 | 0.673 | 0.861 |
| 0.3 | 0.807 | 0.740 | 0.848 | 0.901 | 0.793 | 0.913 |
| 0.4 | 0.848 | 0.853 | 0.859 | 0.880 | 0.837 | 0.854 |
| 0.5 | 0.886 | 0.861 | 0.847 | 0.739 | 0.904 | 0.859 |
| 0.6 | 0.495 | 0.907 | 0.909 | 0.693 | 0.823 | 0.859 |
| 0.7 | 0.768 | 0.720 | 0.863 | 0.797 | 0.871 | 0.904 |
| 0.8 | 0.895 | 0.868 | 0.906 | 0.760 | 0.697 | 0.831 |

**Observations:**
- All forms maintain non-planar formations (σ₃/σ₁ > 0.4 in all cases)
- **standard** and **stiff** forms show the highest non-planarity at low k2 (σ ≈ 0.94)
- **power** form shows a dip at k2=0.2 (σ=0.746) but recovers
- The exponential form is consistently non-planar (σ > 0.82 across all k2)

---

## 5. d_col Scan (k1=k2=0.5, mu=9)

| d_col | standard | power | rational | log | stiff | exponential |
|-------|----------|-------|----------|-----|-------|-------------|
| 4 | 5.0s | 5.0s | 5.0s | 5.0s | 6.67s | 5.0s |
| 5 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 6 | 6.67s | 6.67s | 6.67s | 6.67s | 10.0s | 6.67s |
| 7 | 6.67s | 6.67s | 6.67s | 6.67s | 6.67s | 6.67s |

**Observations:**
- Period increases slightly with d_col (more space → slower breathing)
- The **stiff** form is most sensitive to d_col (T=10s at d=6)
- At d=5 (default), all forms agree on T=5s

---

## 6. mu Scan (k1=k2=0.5, d=5)

| mu | standard | power | rational | log | stiff | exponential |
|----|----------|-------|----------|-----|-------|-------------|
| 7 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 9 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 11 | 6.67s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 13 | 5.0s | 6.67s | 5.0s | 5.0s | 5.0s | 5.0s |

**Observations:**
- Period is **very weakly dependent on mu** for all forms
- Most forms show T=5s across all mu values
- This makes physical sense: mu only affects the cutoff, not the equilibrium

---

## 7. N Scan (k1=k2=0.5, d=5, mu=9)

| N | standard | power | rational | log | stiff | exponential |
|---|----------|-------|----------|-----|-------|-------------|
| 4 | 6.67s | 6.67s | 5.0s | 5.0s | 6.67s | 6.67s |
| 5 | 6.67s | 6.67s | 5.0s | 5.0s | 6.67s | 5.0s |
| 6 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 7 | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s | 5.0s |
| 8 | 5.0s | 6.67s | 5.0s | 5.0s | 5.0s | 5.0s |

**Observations:**
- Period decreases slightly with N (more vehicles → faster breathing)
- N=4,5 show T≈6.67s; N≥6 show T≈5.0s
- The rational and log forms are the most consistent across N

---

## 8. (k1, k2) Grid Summary

For the 3×3 grid (k1, k2 ∈ {0.2, 0.4, 0.6}):

| Form | Breathing region | T range | Notes |
|------|-----------------|---------|-------|
| standard | All 9 points | 5.0-10.0s | Consistent breathing |
| power | All 9 points | 4.0-10.0s | T=10s at (0.6,0.2) |
| rational | All 9 points | 4.0-10.0s | Strong breathing at high k2 |
| log | All 9 points | 4.0-10.0s | Highest peak ratios |
| stiff | All 9 points | 4.0-10.0s | T=5s at (0.4,0.6) despite k2=0.6 |
| exponential | All 9 points | 4.0-10.0s | Consistent non-planarity |

---

## Key Takeaways

1. **Period is primarily determined by k2**, with T ≈ 2π/(√k₂) × (0.5-0.7) across all forms
2. **k1 has negligible effect** on period (except at very high k1=0.8 for some forms)
3. **d_col has a weak effect** — period increases slightly with larger collision distance
4. **mu has almost no effect** on period
5. **N has a weak effect** — period decreases slightly with more vehicles
6. **The alpha function form matters most for breathing strength (peak_ratio) and non-planarity (sigma_ratio)**, not for the period itself
7. **All 6 forms produce robust 3D breathing** — the continuous alpha function requirement is sufficient to preserve the breathing attractor
8. **The log and exponential forms** produce the cleanest, strongest breathing oscillations (highest peak_ratio)
9. **The standard form** (original Kou-Chen-Xiang) works just as well as any other form for producing 3D breathing

---

## Recommendation

If the goal is to study the **period-parameter relationship**, the **standard form** is sufficient —
the period scaling with k2 is essentially identical across all forms.

If the goal is to maximize **breathing signal strength** (easiest to detect/measure), the **log** or **exponential** forms are preferred.

If the goal is to study how **alpha function form affects dynamics**, the **rational** form (simplest) and **stiff** form (quadratic singularity) provide the clearest contrast with the standard form.

---

## Files

All raw data saved in `continuous_alpha_results/`:
- `results_<form>.csv` — scan results for each form
- `results_<form>.json` — full result data
- `run.log` — complete simulation output
- `alpha_function_comparison.md` — alpha function definitions and properties