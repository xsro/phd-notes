# Kou-Chen-Xiang Fencing Controller: Multi-Dimensional Simulation

This project simulates the **first cooperative controller** of Kou, Chen, and Xiang
(2022) for the moving-target-fencing (MTF) problem in one, two, and three dimensions,
plus 2D and 3D breathing limit cycles, and a comparison of continuous alpha repulsion forms.

## Controller

$$
\begin{aligned}
u_i &= \phi_i + k_1(x_0 - x_i) + v_i \\
\dot{v}_i &= k_2(x_0 - x_i)
\end{aligned}
$$

where:
- $\phi_i$ is pairwise central repulsion: $\alpha(s) = \frac{1}{s-d} - \frac{1}{\mu-d}$ for $s \in (d, \mu]$
- When $s \leq d$ (collision), $\alpha$ returns $10^6$ with a warning to prevent vehicle overlap
- $k_1$ is the attractive gain (pulls vehicles toward target)
- $k_2$ is the observer gain (integrates position error to estimate target velocity)
- $d$ is the collision distance, $\mu$ is the sensing radius

## Directory Structure

```
fencing_simulation/
├── README.md
├── .gitignore
│
├── simulate_1d.py              # 1D simulation → saves data/*.npz
├── simulate_2d.py              # 2D rigid rotation → saves data/*.npz
├── simulate_2d_breathing.py    # 2D breathing limit cycle → saves data/*.npz
├── simulate_3d.py              # 3D breathing (seed=42) → saves data/*.npz
│
├── experiment/                 # Search & diagnostic scripts
│   ├── search_2d_breathing.py  # 2D breathing search
│   ├── search_3d_breathing.py  # 3D breathing search
│   ├── check_periodicity.py    # 3D steady-state periodicity analysis
│   ├── diagnose_3d.py          # 3D diagnostic tool
│   └── test_3d_planar.py       # Planar initial condition test
│
├── continuous_alpha_results/   # Continuous alpha function comparison
│   ├── alpha_function_comparison.md   # Alpha function definitions & properties
│   ├── summary.md                     # Scan results summary
│   ├── compare_alpha_forms.py         # Run all 6 alpha forms
│   ├── run_all_alpha_forms.py         # Superseded by compare_alpha_forms.py
│   ├── simulate_3d_continuous_alpha.py # Standalone continuous alpha sim
│   ├── test_tiny.py                   # Timing benchmark
│   ├── run.log                        # Full simulation log
│   └── results_<form>.csv / .json     # Per-form scan data
│
├── 3d_breathing/               # 3D breathing limit cycle research
│   ├── simulate_3d_breathing.py # Standalone simulation (N=6, seed=0)
│   ├── scan_3d_final.py         # Parameter scan (k1,k2,d,mu,N,seeds)
│   ├── scan_fine.py             # Fine-grained scan
│   ├── scan_radial_tangential.py # Radial/tangential decomposition
│   ├── plot_crep.py             # CREP (coupled repulsion) scan plot
│   ├── plot_crep_fine.py        # Fine CREP scan plot
│   ├── plot_radial_tangential.py # Radial/tangential plot
│   ├── check_2d_tangential.py   # 2D tangential analysis
│   ├── RESULTS.md               # 3D breathing results
│   ├── SCAN_RESULTS.md          # Complete scan results
│   ├── FINAL_FORMULAS.md        # Final period-parameter formulas
│   ├── RADIAL_TANGENTIAL.md     # Radial/tangential analysis
│   ├── TANGENTIAL_PERIOD.md     # Tangential period analysis
│   ├── TANGENTIAL_PERIOD_2D.md  # 2D tangential period analysis
│   ├── crep_scan.png            # CREP scan figure
│   ├── crep_scan_fine.png       # Fine CREP scan figure
│   ├── radial_tangential.png    # Radial/tangential figure
│   └── data/                    # Simulation data (.npz)
│
├── plots/                      # Plotting scripts
│   ├── plot_1d.py
│   ├── plot_2d.py
│   ├── plot_2d_breathing.py
│   ├── plot_3d.py
│   ├── plot_3d_planar.py
│   └── run_all_plots.py
│
└── data/                       # Simulation data (.npz files)
```

## Simulation → Plotting Separation

Each simulation script only integrates the ODE and saves data to `data/`.
Plotting scripts read the `.npz` files and generate figures.

**Run simulation only:**
```bash
uv run --no-project --with numpy,scipy,matplotlib,Pillow python simulate_1d.py
uv run --no-project --with numpy,scipy,matplotlib,Pillow python simulate_2d.py
uv run --no-project --with numpy,scipy,matplotlib,Pillow python simulate_2d_breathing.py
uv run --no-project --with numpy,scipy,matplotlib,Pillow python simulate_3d.py
```

**3D breathing scan** (in `3d_breathing/`):
```bash
uv run --no-project --with numpy,scipy,matplotlib,Pillow python 3d_breathing/scan_3d_final.py
```

**3D breathing standalone simulation** (in `3d_breathing/`):
```bash
uv run --no-project --with numpy,scipy,matplotlib,Pillow python 3d_breathing/simulate_3d_breathing.py
```

**Continuous alpha comparison** (in `continuous_alpha_results/`):
```bash
uv run --no-project --with numpy,scipy,matplotlib,Pillow python continuous_alpha_results/compare_alpha_forms.py
```

**Run plotting only:**
```bash
# Individual plots
uv run --no-project --with numpy,scipy,matplotlib,Pillow python plots/plot_1d.py
uv run --no-project --with numpy,scipy,matplotlib,Pillow python plots/plot_2d.py
# ...

# All plots at once
uv run --no-project --with numpy,scipy,matplotlib,Pillow python plots/run_all_plots.py
```

## Results by Dimension

### 1D Simulation

**File**: `simulate_1d.py` / `plots/plot_1d.py`

**Behavior**: Vehicles form a linear formation that fences the target (target is inside
the convex hull of vehicle positions). **No observer** — the simplified controller
$u_i = \phi_i + k_1(x_0 - x_i)$ is used, so vehicles converge to a static equilibrium
in the target frame.

**Key observations**:
- Target is fenced: $x_{\text{min}} \leq x_{\text{target}} \leq x_{\text{max}}$ ✓
- Pairwise distances stay above collision threshold $d$ ✓
- Velocity errors converge to ~0 (vehicles match target velocity exactly)
- Formation center tracks the target position
- **No GIF** — positions are shown as an x-t line plot (`positions_1d.png`)

### 2D Simulation (Rigid Rotation — Case A)

**File**: `simulate_2d.py` / `plots/plot_2d.py`

**Behavior**: Vehicles converge to a **planar rigid rotation** about the target.
This is the classic case analyzed in `fencing_rotation.tex`.

**Key observations**:
- Formation converges to a regular hexagon (or similar polygon)
- Angular velocity: $|\omega| = \sqrt{k_2} = 0.707$ (theoretically predicted and numerically verified)
- Pairwise distances converge to constant values > d
- Velocity errors remain non-zero (each vehicle has tangential velocity component)
- Target is fenced inside the convex hull of vehicles

**Theoretical foundation** (from the research document):
- The controller constrains only the average position error and average velocity
- This leaves $2(N-1)$ circulation degrees of freedom unconstrained
- At radial balance, the tangential stiffness of repulsion cancels $k_1$, leaving one neutral direction
- The observer integrates the rotating position error into tangential velocity
- Frequency selection: $\omega^2 = k_2$ (forced by zero net torque of central repulsive forces)

### 2D Breathing Limit Cycle (Case C)

**File**: `simulate_2d_breathing.py` / `plots/plot_2d_breathing.py`

**Behavior**: With the `case3.py` counterexample initial conditions, the formation
exhibits a **breathing limit cycle** — pairwise distances oscillate periodically
without converging to a fixed radius. This is Case C from the trichotomy conjecture.

**Key observations**:
- Breathing period: $T \approx 2.488\,\text{s}$ (FFT-verified; matches `fencing-rotation-conjecture/doc/breathing.md`)
- Effective frequency ratio: $\omega_{\text{eff}}/\sqrt{k_2} \approx 3.57$ (matches the theoretical prediction of ~3.58)
- **Why not $2\pi/\sqrt{k_2}$?** The breathing mode is a radial oscillation driven by the stiff repulsive gradient $\alpha'(s) = -1/(s-d)^2$, not the neutral tangential mode of rigid rotation. Effective frequency: $\omega_{\text{eff}}^2 = k_2 + C(k_1)$ where $C(k_1)$ comes from repulsive stiffness.
- Mean pairwise distance: ~7.2 (average formation radius ~4.6, close to expected 4.55)
- Breathing amplitude: ~0.57% of mean distance (weak but persistent oscillation)
- Pairwise distances oscillate periodically, never converging
- Velocity errors remain non-zero

**Initial conditions** (from `case3.py`):
```
XI0 = [[-10.584563, -12.532987],
       [ -5.382648,   5.904905],
       [ -0.177347,   9.175920],
       [ -7.352506,  -4.805249],
       [ -6.288338,  12.435830]]
VT0 = [[ 3.528048, -1.274511],
       [-0.511988, -1.485443],
       [ 1.972071, -3.679895],
       [-3.460489, -0.767699],
       [-2.039248,  2.761800]]
```

**Key difference from rigid rotation**: The breathing limit cycle requires
non-symmetric initial conditions (the case3.py counterexample). Symmetric
initial conditions (e.g., regular pentagon) converge to rigid rotation instead.
This demonstrates the **bistability** of the system: both attractors coexist
for the same parameters, and the initial conditions determine which one is reached.

### 3D Simulation (Breathing Limit Cycle, N=30)

**File**: `simulate_3d_n30.py` / `plots/plot_3d_n30.py`

**Behavior**: With random 3D initial conditions and N=30 vehicles, the system converges to a **non-planar 3D breathing limit cycle** — a 3D wrapping formation that fences the target.

**Key observations** (N=30, μ=15.0, seed=42):
- Vehicles form a 3D enclosure around the target (target is inside convex hull ✓)
- 19 of 30 vehicles are on the convex hull boundary (good wrapping)
- Formation is NON-PLANAR (σ₃/σ₁ ≈ 0.836)
- Pairwise distances oscillate: dominant frequency f ≈ 0.17 Hz (T ≈ 5.83 s)
- No collisions: min pairwise distance stays above d=5.0 ✓
- Target tracking: vehicles maintain mean distance ~7.85 from target
- Velocity errors remain non-zero (breathing/rotating formation)

**Tangential vs. Rigid Rotation Frequency** (steady-state analysis):

| Frequency | Value | Ratio to f_rot |
|-----------|-------|----------------|
| Rigid rotation f_rot = √k₂/(2π) | 0.1125 Hz (T=8.89s) | 1.00 |
| Formation centroid tangential | 0.1052 Hz | **0.935** ≈ 1.00 |
| Breathing (radial) f_breath | 0.1714 Hz (T=5.83s) | **1.52** |

- The formation centroid rotates at ≈ the rigid rotation frequency √k₂, but individual vehicles have varying tangential frequencies (not pure rotation)
- Breathing frequency is ~1.5× higher than the rotation frequency
- This differs from pure rigid rotation where all vehicles rotate at exactly √k₂

**Parameter note**: With the original μ=9.0, N=30 leads to overcrowding and collisions.
Increasing μ to 15.0 provides adequate sensing range for 30 vehicles to form a stable
collision-free wrapping formation.

**Files**:
- Simulation: `simulate_3d_n30.py` → `data/simulate_3d_n30.npz`
- Plotting: `plots/plot_3d_n30.py` → `figures_n30/`
  - `distances_3d_n30.png` — pairwise distances over time
  - `vel_error_3d_n30.png` — mean velocity error
  - `positions_3d_n30.gif` — 3D animation of agent positions

### 3D Simulation (Breathing Limit Cycle)

**File**: `simulate_3d.py` / `plots/plot_3d.py`

**Behavior**: With random 3D initial conditions (seed=42), vehicles converge to a
**non-planar 3D breathing limit cycle** — pairwise distances oscillate periodically
without converging. This is the same attractor as `3d_breathing/simulate_3d_breathing.py`
(seed=0), confirming that 3D breathing is the generic attractor for random initial conditions.

**Key observations**:
- Vehicles are distributed in 3D (not confined to a plane)
- Time-averaged positions $\langle\xi_i\rangle \approx 0$ (vehicles track the target well)
- Pairwise distances stay above collision threshold
- The formation is NON-PLANAR ($\sigma_3/\sigma_1 \approx 0.91$)
- **Pairwise distances oscillate**: dominant frequency $f \approx 0.22\,\text{Hz}$ ($T \approx 4.55\,\text{s}$)
- Breathing amplitude: pairwise distances vary by up to ~1.4 units (mean ~6.6-7.3)

**Planar test** (`experiment/test_3d_planar.py` / `plots/plot_3d_planar.py`): If initialized in a plane (e.g., regular hexagon
in the xy-plane), the formation stays perfectly planar and rotates at $|\omega| = \sqrt{k_2}$.
This is the **3D rigid rotation** case — but it requires planar initial conditions.
With random 3D initial conditions, the system generically converges to the 3D breathing
attractor instead. This confirms the simulation code is correct and demonstrates that
the 3D system has at least two attractors: planar rigid rotation (for planar ICs) and
non-planar 3D breathing (for generic random ICs).

**Theoretical interpretation**: In 3D, the breathing attractor is a non-planar
oscillating formation where each vehicle moves on a closed 3D orbit. The pairwise
distances oscillate because the repulsive force gradient creates a radial restoring
mechanism (analogous to the 2D breathing case), but in 3D the additional degrees of
freedom allow the oscillation to be non-planar.

### 3D Breathing Limit Cycle

**Folder**: `3d_breathing/`

**Behavior**: With random 3D initial conditions, vehicles converge to a **non-planar
3D breathing limit cycle** — pairwise distances oscillate periodically without
converging, while the formation as a whole rotates in 3D. This is the first discovered
3D breathing attractor, and it is the **generic attractor** for random 3D initial conditions.

**Key observations** (N=6, seed=0):
- Breathing frequency: $f \approx 0.22\,\text{Hz}$ ($T \approx 4.55\,\text{s}$)
- Pairwise distances oscillate with peak FFT power ratio ~0.76 (strong oscillation)
- Formation is NON-PLANAR ($\sigma_3/\sigma_1 \approx 0.83$)
- Time-averaged positions $\langle\xi_i\rangle \approx 0$ (vehicles track target well)
- Velocity errors remain non-zero

**Key observations** (N=6, seed=42, `simulate_3d.py`):
- Same breathing frequency: $f \approx 0.22\,\text{Hz}$ ($T \approx 4.55\,\text{s}$)
- $\sigma_3/\sigma_1 \approx 0.91$ (even more non-planar than seed=0)
- Confirms 3D breathing is the generic attractor for random ICs

**Initial conditions** (seed=0, N=6):
```
XI0 = [[14.112419,  3.201258,  7.829904],
       [17.927146, 14.940464, -7.818223],
       [ 7.600707, -1.210858, -0.825751],
       [ 3.284788,  1.152349, 11.634188],
       [ 6.088302,  0.973400,  3.550906],
       [ 2.669395, 11.952633, -1.641266]]
VT0 = [[0.0, 0.0, 0.0]]  (zero initial velocity)
```

**Discovery**: Found by `experiment/search_3d_breathing.py` which scans random
3D initial conditions and classifies steady-state behavior via FFT of pairwise
distances. The seed=0 case (N=6) was the first genuine 3D breathing attractor.

**Key difference from 3D rigid rotation**: In rigid rotation, pairwise distances
converge to constants. In 3D breathing, they oscillate periodically. Both are
non-planar, but only breathing exhibits persistent pairwise distance oscillations.

---

## 3D Breathing Parameter Scan Results

Full scan data from `3d_breathing/scan_3d_final.py` (N=4, seed=0, T_MAX=60s).
Complete tables in `3d_breathing/SCAN_RESULTS.md`.

### Primary Scaling with $k_2$ ($k_1 = 0.5$ fixed)

For $k_2 \geq 0.35$:
$$T_{\text{breath}} \approx 0.5 \times \frac{2\pi}{\sqrt{k_2}}, \quad f_{\text{breath}} \approx 2 \times \frac{\sqrt{k_2}}{2\pi}$$
The breathing frequency scales as $\sqrt{k_2}$ (same as rigid rotation) but is $\sim 2\times$ higher.

| $k_2$ | $f$ (Hz) | $T$ (s) | $T_{\text{rot}}$ (s) | $T/T_{\text{rot}}$ | peak_ratio | Type |
|--------|----------|---------|---------------------|--------------------|------------|------|
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

For $k_2 \geq 0.35$, $T/T_{\text{rot}} \approx 0.5$ (stable). For $k_2 < 0.35$, the ratio varies
and can exceed 1.0, indicating the breathing mode becomes softer.

### $k_1$ Modulation ($k_2 = 0.5$ fixed)

$k_1$ has a weaker but noticeable effect, especially at low $k_2$:

| $k_1$ | $f$ (Hz) | $T$ (s) | $T/T_{\text{rot}}$ | peak_ratio | Type |
|--------|----------|---------|--------------------|------------|------|
| 0.05 | 0.233 | 4.29 | 0.48 | 0.655 | BR |
| 0.10 | 0.233 | 4.29 | 0.48 | 0.690 | BR |
| 0.15 | 0.233 | 4.29 | 0.48 | 0.587 | BR |
| 0.70 | 0.133 | 7.50 | 0.84 | 0.520 | BR |
| 0.75 | 0.133 | 7.50 | 0.84 | 0.477 | BR |
| 0.80 | 0.133 | 7.50 | 0.84 | 0.447 | BR |
| 0.85 | 0.133 | 7.50 | 0.84 | 0.340 | BR |
| 0.90 | 0.133 | 7.50 | 0.84 | 0.313 | BR |

For $k_1 \leq 0.15$, $T \approx 4.29$ s. For $k_1 \geq 0.70$, $T \approx 7.50$ s.
The transition occurs in $k_1 = 0.2$–$0.6$ (see grid scan).

### Full $k_1$-$k_2$ Grid (N=4, seed=0)

All 36 combinations ($k_1 = 0.1$–$0.6$, $k_2 = 0.1$–$0.6$) converge to breathing.
Period $T$ (s):

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
- For $k_2 \geq 0.3$, $T$ is mostly determined by $k_2$ (rows are similar)
- For $k_2 = 0.1$–$0.2$, increasing $k_1$ strongly increases $T$
- The $k_1$ effect diminishes as $k_2$ increases

### $d_{\text{col}}$ Scan ($k_1 = k_2 = 0.5$, $\mu = 9.0$)

| $d_{\text{col}}$ | $f$ (Hz) | $T$ (s) | $T/T_{\text{rot}}$ | peak_ratio | $\sigma_3/\sigma_1$ | Type |
|-------------------|----------|---------|--------------------|------------|---------------------|------|
| 3.0 | 0.233 | 4.29 | 0.48 | 0.585 | 0.624 | BR |
| 4.0 | 0.133 | 7.50 | 0.84 | 0.300 | 0.113 | BR |
| 5.0 | 0.200 | 5.00 | 0.56 | 0.412 | 0.444 | BR |
| 6.0 | 0.167 | 6.00 | 0.68 | 0.247 | 0.392 | BR |
| 7.0 | 0.133 | 7.50 | 0.84 | 0.458 | 0.028 | BR |

Non-monotonic effect. $d = 5$ gives the shortest period. At $d = 7$, the
formation becomes nearly planar ($\sigma_3/\sigma_1 = 0.028$).

### $\mu$ Scan ($k_1 = k_2 = 0.5$, $d = 5.0$)

| $\mu$ | $f$ (Hz) | $T$ (s) | $T/T_{\text{rot}}$ | peak_ratio | $\sigma_3/\sigma_1$ | Type |
|--------|----------|---------|--------------------|------------|---------------------|------|
| 6.0 | 0.133 | 7.50 | 0.84 | 0.399 | 0.714 | BR |
| 7.0 | 0.167 | 6.00 | 0.68 | 0.288 | 0.611 | BR |
| 8.0 | 0.200 | 5.00 | 0.56 | 0.438 | 0.476 | BR |
| 9.0 | 0.200 | 5.00 | 0.56 | 0.412 | 0.444 | BR |
| 10.0 | 0.200 | 5.00 | 0.56 | 0.380 | 0.783 | BR |
| 11.0 | 0.200 | 5.00 | 0.56 | 0.363 | 0.520 | BR |
| 12.0 | 0.200 | 5.00 | 0.56 | 0.298 | 0.256 | BR |

For $\mu \geq 8$, the period stabilizes at $T = 5.0$ s. For smaller $\mu$, the period
increases (breathing slows down) as the sensing range shrinks.

### Seed Scan (N=4, $k_1 = k_2 = 0.5$, $d = 5.0$, $\mu = 9.0$)

| seed | $f$ (Hz) | $T$ (s) | $T/T_{\text{rot}}$ | peak_ratio | $\sigma_3/\sigma_1$ | Type |
|------|----------|---------|--------------------|------------|---------------------|------|
| 0 | 0.200 | 5.00 | 0.56 | 0.412 | 0.444 | BR |
| 1 | 0.200 | 5.00 | 0.56 | 0.386 | 0.540 | BR |
| 2 | 0.200 | 5.00 | 0.56 | 0.428 | 0.832 | BR |
| 3 | 0.233 | 4.29 | 0.48 | 0.421 | 0.978 | BR |
| 4 | 0.200 | 5.00 | 0.56 | 0.389 | 0.705 | BR |
| 5 | 0.200 | 5.00 | 0.56 | 0.764 | 0.502 | BR |

The breathing period is robust: 5 of 6 seeds give $T = 5.0$ s.
Seed 3 gives $T = 4.29$ s, which is within the frequency resolution (0.033 Hz).

### $N$ Scan ($k_1 = k_2 = 0.5$, $d = 5.0$, $\mu = 9.0$)

| $N$ | $f$ (Hz) | $T$ (s) | $T_{\text{rot}}$ (s) | $T/T_{\text{rot}}$ | peak_ratio | $\sigma_3/\sigma_1$ | Type |
|-----|----------|---------|---------------------|--------------------|------------|---------------------|------|
| 3 | 0.167 | 6.00 | 8.89 | 0.68 | 0.575 | 0.000 | BR (planar) |
| 4 | 0.200 | 5.00 | 8.89 | 0.56 | 0.412 | 0.444 | BR |
| 5 | 0.133 | 7.50 | 8.89 | 0.84 | 0.536 | 0.637 | BR |

Non-monotonic $N$ dependence. $N = 4$ has the shortest period.
$N = 3$ is planar ($\sigma_3/\sigma_1 = 0$), meaning the formation collapses to a plane.

### Summary of Parameter Dependence

1. **$k_2$ is the primary determinant** of breathing period. For $k_2 \geq 0.35$:
   $$T_{\text{breath}} \approx 0.5 \times 2\pi/\sqrt{k_2}$$
2. **$k_1$ modulates** the period, especially at low $k_2$:
   - $k_1 \leq 0.15$: faster breathing ($T \approx 4.29$ s at $k_2=0.5$)
   - $k_1 \geq 0.70$: slower breathing ($T \approx 7.50$ s at $k_2=0.5$)
   - Effect diminishes as $k_2$ increases
3. **$d_{\text{col}}$** has a non-monotonic effect, with $d = 5$ giving the shortest period.
4. **$\mu$** stabilizes for $\mu \geq 8$ ($T = 5.0$ s at $k_1=k_2=0.5$). Smaller $\mu$ slows breathing.
5. **$N$** has a non-monotonic effect: $N=4$ has the shortest period, $N=3$ is planar.
6. **Random seeds**: The breathing period is robust; most seeds give the same $T$ within frequency resolution.

### Comparison with 2D Breathing

| Property | 2D Breathing | 3D Breathing |
|----------|-------------|-------------|
| $T/T_{\text{rot}}$ | ~0.28 | ~0.5 |
| $f_{\text{breath}}/f_{\text{rot}}$ | ~3.57 | ~2.0 |
| Planarity | Planar ($\sigma_3/\sigma_1 = 0$) | Non-planar ($\sigma_3/\sigma_1 \approx 0.4$–$0.8$) |
| $N$ dependence | $N=5$: $T \approx 2.49$ s | $N=4$: $T \approx 5.0$ s, $N=6$: $T \approx 4.55$ s |

3D breathing is closer to rigid rotation than 2D breathing, consistent with
3D having more degrees of freedom and thus a less stiff breathing mode.

### Known 3D Breathing Cases

**N=6, seed=0** (`3d_breathing/simulate_3d_breathing.py`):
- Frequency: f ≈ 0.22 Hz (T ≈ 4.55 s), T/T_rot ≈ 0.51, σ₃/σ₁ ≈ 0.83
- First discovered 3D breathing attractor

**N=6, seed=42** (`simulate_3d.py`):
- Frequency: f ≈ 0.22 Hz (T ≈ 4.55 s), T/T_rot ≈ 0.51, σ₃/σ₁ ≈ 0.91
- Confirms 3D breathing is the generic attractor for random ICs

**N=4, seed=0** (scan reference case):
- Frequency: f = 0.20 Hz (T = 5.00 s), T/T_rot = 0.56, σ₃/σ₁ = 0.444
- Used for all parameter scans in this section

### Continuous Alpha Function Comparison

**Folder**: `continuous_alpha_results/`

**Purpose**: The original alpha function has a discontinuity at $s=d$ (it jumps to $1e6$).
This project replaces it with 6 continuous alternatives and compares their effect
on the 3D breathing limit cycle.

**6 alpha forms compared**:

| Form | $\alpha(s)$ | Singularity | Stiffness |
|------|------------|-------------|-----------|
| standard | $1/(s-d) - 1/(\mu-d)$ | $1/(s-d)$ | moderate |
| power (p=1.5) | $1/(s-d)^{1.5} - 1/(\mu-d)^{1.5}$ | $1/(s-d)^{1.5}$ | moderate+ |
| rational | $(\mu-s)/(s-d)$ | $(\mu-d)/(s-d)$ | moderate |
| log | $\ln((\mu-d)/(s-d))$ | $-\ln(s-d)$ | weak |
| stiff | $1/(s-d)^2 - 1/(\mu-d)^2$ | $1/(s-d)^2$ | strong |
| exponential | $e^{1/(s-d)} \cdot (\mu-s)/(\mu-d)$ | $\exp(1/(s-d))$ | extreme |

All forms satisfy: continuous on $[d,\infty)$, $\alpha(s)\to\infty$ as $s\to d^+$, $\alpha(s)=0$ for $s>\mu$.

**Key finding**: The breathing period $T$ is **nearly identical** across all 6 forms
for $k_2 \geq 0.3$. The alpha function form affects breathing **strength** (peak_ratio)
and **non-planarity** (sigma_ratio) more than the period itself.

**Results**: `continuous_alpha_results/summary.md` — full comparison tables.
`continuous_alpha_results/alpha_function_comparison.md` — alpha function definitions.

## Parameters

| Parameter | 1D | 2D (rotation) | 2D (breathing) | 3D (rotation) | 3D (breathing) |
|-----------|----|---------------|----------------|---------------|----------------|
| N (vehicles) | 5 | 6 | 5 | 6 (seed=42) | 6 (seed=0) |
| d (collision) | 0.5 | 5.0 | 5.0 | 5.0 | 5.0 |
| μ (sensing) | 2.0 | 9.0 | 9.0 | 9.0 | 9.0 |
| k₁ (attractive) | 1.0 | 0.5 | 0.5 | 0.5 | 0.5 |
| k₂ (observer) | — (none) | 0.5 | 0.5 | 0.5 | 0.5 |
| v₀ (target vel) | 1.0 | (1, 0) | (1, 0) | (1, 0, 0) | (1, 0, 0) |
| T_max | 60s | 80s | 3000s | 200s | 500s |

**Note**: The 3D rigid rotation case (planar ICs, `experiment/test_3d_planar.py`) uses
the same parameters (N=6, d=5, μ=9, k₁=k₂=0.5, v₀=(1,0,0)) but with planar initial
conditions (regular hexagon in xy-plane).

## Dependencies

- numpy
- scipy
- matplotlib
- Pillow (for GIF saving)

Install with: `uv pip install numpy scipy matplotlib Pillow`

## Key Findings

1. **1D**: Linear formation fences the target; vehicles converge to static equilibrium in target frame (no observer, velocity error → 0). Positions shown as x-t line plot.
2. **2D (rotation)**: Planar rigid rotation at $|\omega| = \sqrt{k_2}$; matches the theoretical analysis in `fencing_rotation.tex`.
3. **2D (breathing)**: Breathing limit cycle with $T \approx 2.488\,\text{s}$ and $\omega_{\text{eff}}/\sqrt{k_2} \approx 3.57$; matches the breathing analysis in `fencing-rotation-conjecture/doc/breathing.md`. Demonstrates bistability with rigid rotation.
4. **3D (rotation)**: Non-planar 3D rotating formation with random initial conditions; fundamentally richer than 2D.
   - If initialized in a plane, stays planar (confirms code correctness).
   - With random 3D initial conditions, converges to a 3D non-planar rotating state.
   - **Steady-state motion is periodic**: FFT/autocorrelation analysis confirms a dominant frequency $f \approx 0.11\,\text{Hz}$ ($T \approx 9.0\,\text{s}$), matching $2\pi/\sqrt{k_2}$.
5. **3D (breathing, seed=42)**: `simulate_3d.py` — non-planar 3D breathing with generic random ICs. Pairwise distances oscillate at $f \approx 0.22\,\text{Hz}$ ($T \approx 4.55\,\text{s}$), $\sigma_3/\sigma_1 \approx 0.91$. Shows that 3D breathing is the generic attractor for random initial conditions.
5b. **3D (breathing, seed=0)**: `3d_breathing/simulate_3d_breathing.py` — the first discovered 3D breathing attractor. Same frequency and non-planarity as seed=42. Discovered by `experiment/search_3d_breathing.py`.
5c. **3D (rigid rotation)**: `experiment/test_3d_planar.py` — planar rigid rotation at $|\omega| = \sqrt{k_2}$, but requires planar initial conditions (regular hexagon in xy-plane). Demonstrates bistability: planar ICs → rotation, random 3D ICs → breathing.
6. **Continuous alpha comparison**: 6 continuous alpha forms all produce robust 3D breathing. Period scaling with $k_2$ is form-independent; breathing strength and non-planarity vary by form. Log and exponential forms give the strongest, cleanest breathing signal.

## References

- Kou, Chen, Xiang (2022): "Cooperative Fencing of a Moving Target"
- `fencing_rotation.tex`: Analysis of why the first controller induces rotation
- `fencing-rotation-conjecture/`: Trichotomy conjecture (rotation / jammed / breathing)
- `fencing-rotation-conjecture/doc/breathing.md`: Detailed breathing limit cycle analysis