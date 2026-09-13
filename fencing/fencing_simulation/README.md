# Kou-Chen-Xiang Fencing Controller: Multi-Dimensional Simulation

This project simulates the **first cooperative controller** of Kou, Chen, and Xiang
(2022) for the moving-target-fencing (MTF) problem in one, two, and three dimensions,
plus a 2D breathing limit cycle case.

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
├── check_periodicity.py         # 3D steady-state periodicity analysis
├── simulate_1d.py              # 1D simulation → saves data/*.npz
├── simulate_2d.py              # 2D rigid rotation simulation → saves data/*.npz
├── simulate_2d_breathing.py    # 2D breathing limit cycle simulation → saves data/*.npz
├── simulate_3d.py              # 3D rigid rotation → saves data/*.npz
├── 3d_breathing/               # 3D breathing limit cycle (consolidated)
│   ├── simulate_3d_breathing.py # Standalone simulation (N=6, seed=0)
│   ├── scan_3d_final.py         # Parameter scan (k1,k2,d,mu,N,seeds)
│   ├── SCAN_RESULTS.md          # Complete scan results
│   └── data/                    # Simulation data (.npz)
├── experiment/                  # Search scripts for new cases
│   ├── search_2d_breathing.py   # 2D breathing search
│   └── search_3d_breathing.py   # 3D breathing search
├── data/                       # Simulation data (.npz files)
├── plots/                      # Plotting scripts
│   ├── plot_1d.py
│   ├── plot_2d.py
│   ├── plot_2d_breathing.py
│   ├── plot_3d.py
│   ├── plot_3d_planar.py
│   └── run_all_plots.py
└── figures/                    # Generated figures (PNG + PDF)
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

**3D breathing scan** (in `3d_breathing/`):
```bash
uv run --no-project --with numpy,scipy,matplotlib,Pillow python 3d_breathing/scan_3d_final.py
```
uv run --no-project --with numpy,scipy,matplotlib,Pillow python simulate_3d_breathing.py

**3D breathing scan** (in `3d_breathing/`):
```bash
uv run --no-project --with numpy,scipy,matplotlib,Pillow python 3d_breathing/scan_3d_final.py
```
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

### 3D Simulation

**File**: `simulate_3d.py` / `plots/plot_3d.py`

**Behavior**: With random 3D initial conditions, vehicles converge to a **non-planar
3D rotating formation**. This is a surprising result that differs from the 2D case.

**Key observations**:
- Vehicles are distributed in 3D (not confined to a plane)
- Angular momentum $L = \sum \xi_i \times \dot{v}_i$ is well-defined and aligned with one principal axis
- Time-averaged positions $\langle\xi_i\rangle \approx 0$ (vehicles track the target well)
- Pairwise distances stay above collision threshold
- The formation is NOT planar ($\sigma_3/\sigma_1 \approx 0.91$)
- **Motion is periodic**: FFT and autocorrelation analysis of steady-state trajectories show a dominant frequency $f \approx 0.11\,\text{Hz}$ ($T \approx 9.0\,\text{s}$), consistent with $2\pi/\sqrt{k_2} \approx 8.89\,\text{s}$

**Planar test** (`test_3d_planar.py` / `plots/plot_3d_planar.py`): If initialized in a plane (e.g., regular hexagon
in the xy-plane), the formation stays perfectly planar and rotates at $|\omega| = \sqrt{k_2}$.
This confirms the simulation code is correct and the non-planar behavior with random
initial conditions is a genuine 3D phenomenon.

**Theoretical interpretation**: In 3D, the system supports non-planar rotating
formations where each vehicle moves on a closed 3D orbit that is symmetric about the
origin (so $\langle\xi_i\rangle = 0$ and the observer does not wind up). This is fundamentally
different from the 2D case where rigid rotation forces all vehicles into a plane
perpendicular to the rotation axis.

### 3D Breathing Limit Cycle

**File**: `3d_breathing/simulate_3d_breathing.py`

**Behavior**: With specific random 3D initial conditions (seed=0 from the breathing
search), vehicles converge to a **non-planar 3D breathing limit cycle** — pairwise
distances oscillate periodically without converging, while the formation as a whole
rotates in 3D. This is the first discovered 3D breathing attractor.

**Key observations**:
- Breathing frequency: $f \approx 0.22\,\text{Hz}$ ($T \approx 4.55\,\text{s}$)
- Pairwise distances oscillate with peak FFT power ratio ~0.76 (strong oscillation)
- Formation is NON-PLANAR ($\sigma_3/\sigma_1 \approx 0.83$)
- Time-averaged positions $\langle\xi_i\rangle \approx 0$ (vehicles track target well)
- Velocity errors remain non-zero

**Initial conditions** (seed=0):
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
distances. The seed=0 case is the first genuine 3D breathing attractor discovered.
Full parameter scan and analysis in `3d_breathing/SCAN_RESULTS.md`.

**Key difference from 3D rigid rotation**: In rigid rotation, pairwise distances
converge to constants. In 3D breathing, they oscillate periodically. Both are
non-planar, but only breathing exhibits persistent pairwise distance oscillations.

**Parameter dependence (experimental scan, N=4)**:

**Primary scaling with $k_2$**: For $k_2 \geq 0.35$:
$$T_{\text{breath}} \approx 0.5 \times \frac{2\pi}{\sqrt{k_2}}, \quad f_{\text{breath}} \approx 2 \times \frac{\sqrt{k_2}}{2\pi}$$
The breathing frequency scales as $\sqrt{k_2}$ (same as rigid rotation) but is $\sim 2\times$ higher.

**$k_1$ modulation**: $k_1$ has a weaker but noticeable effect, especially at low $k_2$:
- $k_1 \leq 0.15$: breathing is faster ($T \approx 4.29$ s at $k_2=0.5$)
- $k_1 \geq 0.7$: breathing is slower ($T \approx 7.5$ s at $k_2=0.5$)
- For $k_2 \geq 0.35$, the $k_1$ effect diminishes; $T/T_{\text{rot}} \approx 0.5$ holds broadly

**$d_{\text{col}}$ and $\mu$**: Non-monotonic but weak for $\mu \geq 8$:
- $\mu \geq 8$: period stable at $T=5.0$ s (for $k_1=k_2=0.5$)
- $\mu < 8$: period increases as sensing range shrinks
- $d_{\text{col}}$ affects the breathing amplitude and period in a complex way (likely via how close vehicles approach the collision boundary)

**$N$ dependence**: Non-monotonic — $N=4$ has the shortest period ($T=5.0$ s), $N=3$ is planar ($T=6.0$ s), $N=5$ is slower ($T=7.5$ s) at $k_1=k_2=0.5$.

**Robustness**: Breathing period is robust across random seeds (most seeds give $T=5.0$ s for $k_1=k_2=0.5$, $N=4$).

**Full scan data**: `3d_breathing/scan_3d_final.py` — 6×6 $k_1$-$k_2$ grid, $d_{\text{col}}$ scan, $\mu$ scan, seed scan, $N$ scan, wider $k_1$/$k_2$ ranges. Complete results in `3d_breathing/SCAN_RESULTS.md`.

**Comparison with 2D**: 3D breathing is closer to rigid rotation ($T/T_{\text{rot}} \approx 0.5$) than 2D breathing ($T/T_{\text{rot}} \approx 0.28$), consistent with 3D having more degrees of freedom and thus a less stiff breathing mode.

## Parameters

| Parameter | 1D | 2D (rotation) | 2D (breathing) | 3D (rotation) | 3D (breathing) |
|-----------|----|---------------|----------------|---------------|----------------|
| N (vehicles) | 5 | 6 | 5 | 6 | 6 |
| d (collision) | 0.5 | 5.0 | 5.0 | 5.0 | 5.0 |
| μ (sensing) | 2.0 | 9.0 | 9.0 | 9.0 | 9.0 |
| k₁ (attractive) | 1.0 | 0.5 | 0.5 | 0.5 | 0.5 |
| k₂ (observer) | — (none) | 0.5 | 0.5 | 0.5 | 0.5 |
| v₀ (target vel) | 1.0 | (1, 0) | (1, 0) | (1, 0, 0) | (1, 0, 0) |
| T_max | 60s | 80s | 3000s | 200s | 500s |

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
5. **3D (breathing)**: Non-planar 3D breathing limit cycle with specific random initial conditions (seed=0). Pairwise distances oscillate at $f \approx 0.22\,\text{Hz}$ ($T \approx 4.55\,\text{s}$), $\sigma_3/\sigma_1 \approx 0.83$. Discovered by `experiment/search_3d_breathing.py`. Demonstrates that the 3D system supports attractors beyond rigid rotation.

## References

- Kou, Chen, Xiang (2022): "Cooperative Fencing of a Moving Target"
- `fencing_rotation.tex`: Analysis of why the first controller induces rotation
- `fencing-rotation-conjecture/`: Trichotomy conjecture (rotation / jammed / breathing)
- `fencing-rotation-conjecture/doc/breathing.md`: Detailed breathing limit cycle analysis