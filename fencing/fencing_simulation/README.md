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
├── simulate_3d.py              # 3D simulation → saves data/*.npz
├── test_3d_planar.py           # 3D planar test → saves data/*.npz
├── diagnose_3d.py              # 3D diagnostic → saves data/*.npz
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

## Parameters

| Parameter | 1D | 2D (rotation) | 2D (breathing) | 3D |
|-----------|----|---------------|----------------|-----|
| N (vehicles) | 5 | 6 | 5 | 6 |
| d (collision) | 0.5 | 5.0 | 5.0 | 5.0 |
| μ (sensing) | 2.0 | 9.0 | 9.0 | 9.0 |
| k₁ (attractive) | 1.0 | 0.5 | 0.5 | 0.5 |
| k₂ (observer) | — (none) | 0.5 | 0.5 | 0.5 |
| v₀ (target vel) | 1.0 | (1, 0) | (1, 0) | (1, 0, 0) |
| T_max | 60s | 80s | 3000s | 200s |

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
4. **3D**: Non-planar 3D rotating formation with random initial conditions; fundamentally richer than 2D.
   - If initialized in a plane, stays planar (confirms code correctness).
   - With random 3D initial conditions, converges to a 3D non-planar rotating state.
   - **Steady-state motion is periodic**: FFT/autocorrelation analysis confirms a dominant frequency $f \approx 0.11\,\text{Hz}$ ($T \approx 9.0\,\text{s}$), matching $2\pi/\sqrt{k_2}$.
   - This suggests the 3D system has attractors that are not simple embeddings of 2D rotations.

## References

- Kou, Chen, Xiang (2022): "Cooperative Fencing of a Moving Target"
- `fencing_rotation.tex`: Analysis of why the first controller induces rotation
- `fencing-rotation-conjecture/`: Trichotomy conjecture (rotation / jammed / breathing)
- `fencing-rotation-conjecture/doc/breathing.md`: Detailed breathing limit cycle analysis