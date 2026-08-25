# Kou et al. (2022) — Cooperative Fencing Control Simulation

**Paper:** Liwei Kou, Zhiyong Chen, Ji Xiang, "Cooperative Fencing Control of Multiple Vehicles for a Moving Target With an Unknown Velocity," *IEEE Transactions on Automatic Control*, vol. 67, no. 8, pp. 4218-4225, 2022. DOI: 10.1109/TAC.2021.3075320

## Problem

A group of \(N\) vehicles (kinematic: \(\dot{x}_i = u_i\)) must fence a moving target with **unknown constant velocity** \(v_0\), driving it into their convex hull without collision and without a predefined stand-off distance or formation.

## Simulated Algorithm: Controller (a) — Average Velocity Tracking

From Section III-A of the paper.

### Controller

\[
\begin{aligned}
u_i^a &= \phi_i + k_1 (x_0 - x_i) + v_i \\
\dot{v}_i &= k_2 (x_0 - x_i) \\
\phi_i &= \sum_{j \in \mathcal{N}_i} \alpha(\|x_{ij}\|) \frac{x_{ij}}{\|x_{ij}\|} \\
\alpha(r) &= \frac{1}{r - d} - \frac{1}{\mu - d}, \quad r \in (d, \mu]
\end{aligned}
\]

- \(\phi_i\): repulsive force from neighbors (collision avoidance)
- \(k_1(x_0 - x_i)\): attractive force toward the target
- \(v_i\): adaptive estimate of the unknown target velocity \(v_0\)
- The average velocity \(\frac{1}{N}\sum_i v_i \to v_0\), but individual \(v_i\) need not converge to \(v_0\) (vehicles may rotate around the target)

### Key Properties (Theorem 1)

- (P1) Target is exponentially fenced: \(\lim_{t\to\infty} P_{x_0(t)}(x(t)) = 0\)
- (P2) Collision avoidance: \(\|x_{ij}(t)\| > d\) for all \(t\)
- Average velocity converges: \(\frac{1}{N}\sum_i u_i(t) \to v_0\)

## Simulation Parameters (Section IV)

| Parameter | Value |
|-----------|-------|
| \(N\) | 6 vehicles |
| \(\mu\) (neighbor radius) | 9 |
| \(d\) (collision threshold) | 5 |
| \(k_1, k_2\) | 0.5 |
| \(x_0(0)\) | \([0, 20]^T\) |
| \(v_0\) | \([3, 1]^T\) (unknown to vehicles) |
| \(x_i(0)\) | \(10[\cos((i-1)\pi/3), \sin((i-1)\pi/3)]^T\) |

## Files

- `simulate_kou2021.m` — MATLAB script implementing the above. Plots:
  1. Vehicle and target trajectories
  2. Average position error \(\hat{x}(t)\)
  3. Minimum inter-vehicle distance (collision check)
  4. Vehicle-to-target distances
  5. Average velocity estimation error
  6. Velocity estimates for first 3 vehicles
  7. Animation of the fencing process

## Running

```matlab
cd simu/kou2021
simulate_kou2021
```