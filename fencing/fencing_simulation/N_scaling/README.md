# N-Scaling Analysis: 3D Fencing Controller

Analysis of how vehicle movement frequencies depend on cluster size $N$ in the
3D Kou-Chen-Xiang fencing controller.

## Folder Structure

```
N_scaling/
├── README.md                  # This file
├── analyze_N_targeted.py      # Main analysis: N-scan with frequency decomposition
├── n_scaling_results.json     # Raw results for N=3,4,5,6,8,10,15,20
├── scan_locked_frequency.py   # Full locked-frequency parameter scan (slow)
├── scan_locked_fast.py        # Fast locked-frequency scan
├── scan_locked_n4.py          # N=4 locked-frequency scan
├── scan_locked_ultrafast.py   # Ultra-fast locked-frequency scan
└── quick_locked_test.py       # Quick verification tests
```

## How to Run

```bash
# Main N-scaling analysis (high resolution, ~5-10 min per N)
uv run --no-project --with numpy,scipy python analyze_N_targeted.py

# Quick locked-frequency tests (fast, N=4)
uv run --no-project --with numpy,scipy python quick_locked_test.py

# Full parameter scan (slow, many simulations)
uv run --no-project --with numpy,scipy python scan_locked_frequency.py
```

## Quick Reference

**Parameters**: $d=5.0$, $k_1=0.5$, $k_2=0.5$, $\mu$ scaled with $N$, seed=0

**Reference**: $f_{\text{rot}} = \sqrt{k_2}/(2\pi) = 0.1125$ Hz, $T_{\text{rot}} = 8.89$ s

### Key Results

Two dynamical regimes are observed:

**Breathing regime** ($f_{\text{rad}} \neq f_{\text{tan}}$):
| $N$ | $f_{\text{rad}}$ (Hz) | $f_{\text{tan}}$ (Hz) | $f_{\text{rad}}/f_{\text{tan}}$ | $\sigma_3/\sigma_1$ |
|-----|----------------------|----------------------|-------------------------------|--------------------|
| 3   | 0.157                | 0.114                | 1.375                         | 0.000              |
| 4   | 0.214                | 0.114                | 1.875                         | 0.068              |
| 6   | 0.214                | 0.114                | 1.875                         | 0.827              |
| 8   | 0.214                | 0.114                | 1.875                         | 0.991              |
| 10  | 0.014*               | 0.114                | 0.125                         | 0.847              |

$^*$N=10 radial frequency is near-DC (anomalous regime).
In this regime: $f_{\text{tan}} \approx f_{\text{rot}} = 0.1125$ Hz (within 0.002 Hz).

**Locked regime** ($f_{\text{rad}} = f_{\text{tan}} = f_{\text{self}}$):
| $N$ | $f_{\text{lock}}$ (Hz) | $f_{\text{lock}}/f_{\text{rot}}$ | $\sigma_3/\sigma_1$ |
|-----|----------------------|-------------------------------|--------------------|
| 5   | 0.171                | 1.52                          | 0.734              |
| 15  | 0.200                | 1.78                          | 0.884              |
| 20  | 0.200                | 1.78                          | 0.803              |

In this regime: $f_{\text{tan}}$ is **not** $f_{\text{rot}}$ — the tangential frequency
shifts significantly due to mode coupling.

### Three Frequency Components

For each vehicle, motion relative to the formation center of mass is decomposed into:

1. **Radial** ($f_{\text{rad}}$): distance from CM — the *breathing* mode
2. **Tangential** ($f_{\text{tan}}$): velocity perpendicular to radial — the *rotation* mode
3. **Self** ($f_{\text{self}}$): total speed — combined motion

### Core Findings

#### Two Dynamical Regimes

The system exhibits two distinct regimes separated by sharp transitions:

| Regime | $f_{\text{rad}}/f_{\text{tan}}$ | $f_{\text{tan}}$ vs $f_{\text{rot}}$ | Occurs when |
|--------|-------------------------------|--------------------------------------|------------|
| **Breathing** | $> 1.3$ | $f_{\text{tan}} \approx f_{\text{rot}}$ (within freq. resolution) | Low $k_1$, low $d$, high $\mu$, small $N$ |
| **Locked** | $= 1.0$ | $f_{\text{tan}} = f_{\text{lock}} \neq f_{\text{rot}}$ (significant shift) | High $k_1$, high $d$, low $\mu$, large $N$ |

**In the breathing regime**, the tangential direction is a neutral mode (no restoring force),
so $f_{\text{tan}} \approx f_{\text{rot}} = \sqrt{k_2}/(2\pi)$. The deviation is $< 0.002$ Hz,
well within the frequency resolution of $0.0143$ Hz.

**In the locked regime**, radial and tangential modes couple into a single oscillation.
The common frequency $f_{\text{lock}}$ is shifted away from $f_{\text{rot}}$ by the
repulsive stiffness. The shift is real and significant (up to 78% for $N=15,20$).

#### Radial Frequency → Weakly dependent on $N$, non-monotonic
$$\omega_{\text{rad}}^2 = k_2 + C_{\text{rep}}(N)$$
The breathing mode is stiffened by repulsive forces. As $N$ grows, $C_{\text{rep}}$
decreases (forces shared among more neighbors), but the relationship is non-monotonic
due to formation geometry effects. Power-law fit: $f_{\text{rad}} \propto N^{-0.20}$.

#### Frequency Locking for Large $N$
For $N \geq 15$: $f_{\text{rad}} = f_{\text{tan}} = f_{\text{self}} = 0.200$ Hz.
The radial and tangential modes couple into a single oscillation. The locked
frequency is **1.78× higher** than $f_{\text{rot}}$, indicating that both $k_2$ and
the residual repulsive stiffness contribute to the effective frequency.

#### Locked Frequency vs. Parameter Path
The locked frequency depends on *which* parameter triggers locking:

| Trigger | $f_{\text{lock}}/f_{\text{rot}}$ | Example |
|---------|-------------------------------|---------|
| High $k_1$ | ~1.04–1.18 | $k_1 \geq 0.70$, $k_2=0.5$ |
| High $d$ | ~1.18 | $d \geq 7.0$, $k_1=k_2=0.5$ |
| Low $\mu$ | ~1.18 | $\mu \leq 6.5$, $k_1=k_2=0.5$ |
| High $N$ | ~1.52–1.78 | $N \geq 15$, $k_1=k_2=0.5$ |

High-$N$ locking produces the highest locked frequency because the formation radius
grows with $N$, increasing the effective attractive/observer force magnitude.

#### Self Frequency → Always tracks tangential
$f_{\text{self}} = f_{\text{tan}}$ in both regimes. Total speed is dominated by the
tangential (rotational) component, not the radial (breathing) oscillation.

#### Planarity
- $N=3$: planar ($\sigma_3/\sigma_1 = 0$)
- $N=4$: nearly planar ($\sigma_3/\sigma_1 = 0.068$)
- $N \geq 5$: non-planar ($\sigma_3/\sigma_1 \approx 0.73$–$0.99$), approximately independent of $N$

### Physical Mechanism

The controller constrains only the average position error and average velocity,
leaving $2(N-1)$ circulation degrees of freedom unconstrained.

**Breathing regime**: The tangential mode is neutral ($\omega_{\text{tan}} = \sqrt{k_2}$),
while the radial mode is stiffened by the repulsive force gradient
$\alpha'(s) = -1/(s-d)^2$:

$$\omega_{\text{rad}}^2 = k_2 + C_{\text{rep}}, \quad \omega_{\text{tan}}^2 = k_2$$

Since $C_{\text{rep}} > 0$, we have $f_{\text{rad}} > f_{\text{tan}} \approx f_{\text{rot}}$.

**Locked regime**: As $N$ increases, $C_{\text{rep}}$ decreases (repulsive forces
distributed over more neighbors), causing $f_{\text{rad}}$ to soften toward $f_{\text{tan}}$.
When $C_{\text{rep}}$ drops below a threshold, the two modes couple and lock at a
common frequency $f_{\text{lock}}$ that lies **between** $f_{\text{rot}}$ and the original
$f_{\text{rad}}$. The exact value depends on which parameter triggered the locking:

$$\omega_{\text{lock}}^2 = k_2 + \beta \cdot C_{\text{rep}}, \quad \beta \in [0, 1]$$

- $k_1$-triggered locking: $\beta \approx 0$ → $f_{\text{lock}} \approx f_{\text{rot}}$
- $d$/$\mu$-triggered locking: $\beta \approx 0.2$–$0.3$ → $f_{\text{lock}} \approx 1.18 \times f_{\text{rot}}$
- $N$-triggered locking: $\beta \approx 0.5$–$0.6$ → $f_{\text{lock}} \approx 1.5$–$1.8 \times f_{\text{rot}}$

The higher locked frequency for large $N$ occurs because the formation radius grows
with $N$, increasing the effective attractive/observer force magnitude.

### Comparison with Previous Results

| Property | 2D Breathing ($N=5$) | 3D Breathing ($N=4,6$) | This Work ($N=15,20$) |
|----------|---------------------|----------------------|----------------------|
| $f_{\text{rad}}/f_{\text{tan}}$ | ~3.57 | ~1.86 | **1.00** (locked) |
| Planarity | Planar | Non-planar | Non-planar |

3D breathing is closer to rigid rotation than 2D breathing, consistent with 3D
having more degrees of freedom. For very large $N$, the system enters a
frequency-locked regime.

## Data

`n_scaling_results.json` contains the raw simulation results for $N = 3, 4, 5, 6, 8, 10, 15, 20$,
including all frequency components, planarity metrics, and collision checks.