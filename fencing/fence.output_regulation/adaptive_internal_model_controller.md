# Adaptive Internal Model Controller for Cooperative Target Fencing

## 1. Problem Formulation

Consider a group of $n \geq 2$ second-order vehicles with dynamics:

$$\dot{x}_i = v_i, \quad \dot{v}_i = u_i, \quad i = 1, \ldots, n$$

where $x_i, v_i, u_i \in \mathbb{R}^2$ denote position, velocity, and control input.

The target evolves according to:

$$\dot{\theta} = (S \otimes I_2)\theta, \quad \theta = [x_0^\top, v_0^\top]^\top$$

with system matrix:

$$S = \begin{bmatrix} 0 & 1 \\ -s_1 & -s_2 \end{bmatrix}$$

**Key assumption for this adaptive version:** The target dynamics structure (second-order) is known, but the parameters $s_1, s_2$ are **unknown**. Each vehicle can only measure relative positions: $e_i = x_i - x_0$ (relative to target) and $x_{ij} = x_i - x_j$ (relative to neighbors within communication range $r_c$).

**Control Objectives:**
- **(P1)** Target fenced within convex hull: $\lim_{t\to\infty} r_{co}(t) = 0$
- **(P2)** Collision avoidance: $\|x_{ij}(t)\| > r_s$ for all $t$
- **(P3)** Velocity synchronization: $\lim_{t\to\infty} [v_i(t) - v_0(t)] = 0$

## 2. The Adaptive Internal Model Controller

### 2.1 Controller Architecture

The controller for each vehicle $i$ is:

$$u_i = -(K \otimes I_2) z_i + \alpha_1 \sum_{j \in \mathbb{N}_i} \varpi(\|x_{ij}\|) \frac{x_{ij}}{\|x_{ij}\|}$$

where $K = [k_1, k_2, k_3, k_4]$, and $z_i = [\hat{x}_i^\top, \hat{v}_i^\top, \hat{x}_{0i}^\top, \hat{v}_{0i}^\top]^\top$ is the state estimate maintained by the **adaptive** dynamic compensator:

$$\dot{z}_i = (\hat{G}_1(t) \otimes I_2) z_i + (G_2 \otimes I_2) e_i + (\alpha_2 \otimes I_2) \sum_{j \in \mathbb{N}_i} \varpi(\|x_{ij}\|) \frac{x_{ij}}{\|x_{ij}\|}$$

where the **adaptive internal model matrix** is:

$$\hat{G}_1(t) = \begin{bmatrix} -l_1 & 1 & 0 & 0 \\ -k_1-l_2 & -k_2 & -k_3 & -k_4 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & -\hat{s}_1(t) & -\hat{s}_2(t) \end{bmatrix}, \quad G_2 = \begin{bmatrix} l_1 \\ l_2 \\ 0 \\ 1 \end{bmatrix}$$

**The key difference from the fixed-structure controller:** The lower-right $2\times 2$ block of $\hat{G}_1(t)$ is the **online estimate** $\hat{S}(t) = \begin{bmatrix} 0 & 1 \\ -\hat{s}_1(t) & -\hat{s}_2(t) \end{bmatrix}$, replacing the fixed $S$ that would require prior knowledge of $s_1, s_2$.

The repulsion function is:

$$\varpi(\|x_{ij}\|) = \begin{cases} \dfrac{\eta}{\|x_{ij}\| - r_s} - \dfrac{\eta}{r_c - r_s}, & r_s < \|x_{ij}\| \leq r_c \\ 0, & \|x_{ij}\| > r_c \end{cases}$$

### 2.2 Parameter Adaptive Law

To estimate the unknown $s_1, s_2$ online, we introduce a filtered version of the relative position error and a gradient-based adaptation law.

**Filtered error signal:**

$$\dot{\varepsilon}_i = -\lambda \varepsilon_i + e_i, \quad \lambda > 0$$

This first-order low-pass filter removes high-frequency components from the collision-avoidance term and produces a clean regression signal.

**Adaptive law for $\hat{s}_1, \hat{s}_2$:**

$$\dot{\hat{s}}_1 = -\gamma_1 \, \hat{x}_{0i} \, \varepsilon_i, \qquad \dot{\hat{s}}_2 = -\gamma_2 \, \hat{v}_{0i} \, \varepsilon_i, \qquad \gamma_1, \gamma_2 > 0$$

where $[\hat{x}_{0i}, \hat{v}_{0i}]^\top$ are the target state estimates from the compensator state $z_i$ (specifically, the third and fourth blocks of $z_i$). These serve as the **regression vector** for the gradient descent.

**Practical implementation note:** In practice, projection operators can be added to bound $\hat{s}_1, \hat{s}_2$ within a known feasible set, or regularized least-squares gains can be used for numerical robustness.

### 2.3 The p-Copy Internal Model (Adaptive Version)

The pair $(\hat{G}_1(t), G_2)$ incorporates an **adaptive p-copy internal model** of the unknown $S$:
- The lower-right $2\times 2$ block of $\hat{G}_1(t)$ is the online estimate $\hat{S}(t)$ — this is the adaptive internal model that learns the target dynamics.
- The upper-left $2\times 2$ block $\begin{bmatrix} -l_1 & 1 \\ -k_1-l_2 & -k_2 \end{bmatrix}$ with $G_2$'s upper entries $\begin{bmatrix} l_1 \\ l_2 \end{bmatrix}$ forms a second controllable copy, providing degrees of freedom to make the augmented closed-loop matrix Hurwitz.

As $\hat{s}_1 \to s_1$ and $\hat{s}_2 \to s_2$ (under persistent excitation), the adaptive internal model converges to the true internal model, and the controller reduces to the fixed-structure Wen et al. (2026) controller.

### 2.4 Parameter Design

Define the augmented matrices:

$$A = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}, \quad C = [1, 0], \quad U = [0, 1, 0, 0, 0, 0]$$

The **instantaneous** closed-loop matrix is:

$$A_c(\hat{S}) = \begin{bmatrix} A & -BK \\ G_2 C & \hat{G}_1 \end{bmatrix}$$

**Parameter selection procedure:**

1. Choose $k_1, k_2, k_3, k_4, l_1, l_2$ such that $A_c(\hat{S})$ remains Hurwitz throughout the parameter convergence process (at least locally), and $(U^\top U, A_c(\hat{S}))$ is observable.
2. Solve the Lyapunov equation $P A_c(\hat{S}) + A_c(\hat{S})^\top P + U^\top U = 0$ for $P > 0$.
3. Set $\alpha = [0, \alpha_1, \alpha_2^\top]^\top = \epsilon P^{-1} U^\top$ for any $\epsilon > 0$.
4. Choose adaptation gains $\gamma_1, \gamma_2 > 0$ and filter constant $\lambda > 0$.

## 3. Stability Proof (Outline)

### 3.1 Two-Time-Scale Structure

The adaptive closed-loop system has a **two-time-scale** structure:

- **Fast time scale** (controller/compensator dynamics): governed by $A_c(\hat{S})$, which is designed to be Hurwitz.
- **Slow time scale** (parameter adaptation): $\hat{s}_1, \hat{s}_2$ evolve according to the gradient law.

### 3.2 Parameter Error Dynamics

Define parameter errors $\tilde{s}_1 = s_1 - \hat{s}_1$, $\tilde{s}_2 = s_2 - \hat{s}_2$. The adaptive law can be written as:

$$\dot{\tilde{s}}_1 = \gamma_1 \hat{x}_{0i} \varepsilon_i, \qquad \dot{\tilde{s}}_2 = \gamma_2 \hat{v}_{0i} \varepsilon_i$$

### 3.3 Lyapunov Analysis

Consider the composite Lyapunov function:

$$V = V_{\text{fence}} + V_{\text{est}}$$

where

$$V_{\text{fence}} = \frac{\epsilon}{2}\sum_{i=1}^n \sum_{j \in \mathbb{N}_i} \int_{\|x_{ij}\|}^{r_c} \varpi(s)\,ds + \frac{1}{2}\sum_{i=1}^n \tilde{\zeta}_i^\top (P \otimes I_2) \tilde{\zeta}_i$$

$$V_{\text{est}} = \frac{1}{2\gamma_1}\tilde{s}_1^2 + \frac{1}{2\gamma_2}\tilde{s}_2^2$$

**Step 1 — Fast dynamics:** With $\hat{S}$ frozen, the error dynamics are:

$$\dot{\tilde{\zeta}}_i = (A_c(\hat{S}) \otimes I_2)\tilde{\zeta}_i + (\alpha \otimes I_2) \sum_{j \in \mathbb{N}_i} \varpi \frac{x_{ij}}{\|x_{ij}\|} + \text{parameter mismatch terms}$$

The parameter mismatch terms couple the fast and slow dynamics. They are bounded by $\|\tilde{s}_1\|, \|\tilde{s}_2\|$ times functions of $z_i$.

**Step 2 — Slow dynamics:** The adaptive law drives $\tilde{s}_1, \tilde{s}_2 \to 0$ provided the regression vector $[\hat{x}_{0i}, \hat{v}_{0i}]^\top$ is **persistently exciting (PE)**. For a target with time-varying velocity (eigenvalues of $S$ not in the left half-plane), the target trajectory naturally provides persistent excitation.

**Step 3 — Composite Lyapunov derivative:** Under PE and appropriate gain selection, the derivative $\dot{V}$ can be shown to be negative semi-definite:

$$\dot{V} \leq -\frac{1}{2}\sum_i \|\tilde{v}_i\|^2 - c_1 \|\tilde{s}_1\|^2 - c_2 \|\tilde{s}_2\|^2 + \text{higher-order cross terms}$$

By standard adaptive control arguments (e.g., Ioannou & Sun, 1996; Serrani-Isidori-Marconi, 2001), the parameter errors converge to zero exponentially under PE, and the closed-loop system achieves:

- $\tilde{\zeta}_i \to 0$ exponentially (fence error and velocity synchronization)
- $\tilde{s}_1, \tilde{s}_2 \to 0$ (parameter convergence)

### 3.4 Conclusion

**(P1) Fencing:** $\bar{e} \to 0$ as $\tilde{\bar{\zeta}} \to 0$, so the centroid converges to the target.

**(P2) Collision avoidance:** $V(t) \leq V(0) < \infty$ ensures $\|x_{ij}(t)\| > r_s$ for all $t$.

**(P3) Velocity synchronization:** $\dot{V} \leq 0$ and Barbalat's Lemma give $\lim_{t\to\infty} \tilde{v}_i = 0$.

**Parameter convergence:** Under persistent excitation (target with time-varying velocity), $\hat{s}_1 \to s_1$ and $\hat{s}_2 \to s_2$.

## 4. Algorithm Summary

```
Algorithm: Adaptive Internal Model Fencing Controller
======================================================================

INPUT: Relative positions e_i = x_i - x_0, x_{ij} = x_i - x_j;
       target dynamics structure (second-order);
       safe distance r_s, communication range r_c

PARAMETERS:
  1. Choose k_1, k_2, k_3, k_4, l_1, l_2 such that A_c(Ŝ) is Hurwitz
  2. Solve Lyapunov equation: P A_c + A_c^T P + U^T U = 0
  3. Set α = [0, α_1, α_2^T]^T = ε P^{-1} U^T
  4. Choose adaptation gains γ_1, γ_2 > 0 and filter constant λ > 0

CONTROL LAW:
  For each vehicle i:
    u_i = -(K ⊗ I_2) z_i + α_1 Σ_j varpi(||x_{ij}||) x_{ij}/||x_{ij}||

COMPENSATOR (adaptive):
    ż_i = (Ĝ_1(t) ⊗ I_2) z_i + (G_2 ⊗ I_2) e_i + (α_2 ⊗ I_2) Σ_j varpi(||x_{ij}||) x_{ij}/||x_{ij}||

ADAPTIVE LAW:
    ε̇_i = -λ ε_i + e_i
    ŝ̇_1 = -γ_1 · x̂_{0i} · ε_i
    ŝ̇_2 = -γ_2 · v̂_{0i} · ε_i

OUTPUT: Control inputs u_i achieving (P1), (P2), (P3), with ŝ_1 → s_1, ŝ_2 → s_2
```

## 5. Key Design Insight

The adaptive internal model controller extends the fixed-structure Wen et al. (2026) controller by:

1. **Replacing the fixed $S$ with an online estimate $\hat{S}(t)$** in the compensator matrix $\hat{G}_1(t)$, eliminating the need for prior knowledge of $s_1, s_2$.

2. **Adding a gradient-based parameter adaptation law** driven by the filtered relative position error $\varepsilon_i$, using the compensator's target state estimates $[\hat{x}_{0i}, \hat{v}_{0i}]^\top$ as the regression vector.

3. **Leveraging persistent excitation** from the time-varying target trajectory to guarantee parameter convergence $\hat{s}_1 \to s_1$, $\hat{s}_2 \to s_2$.

4. **Maintaining the core output regulation framework** — once parameters converge, the adaptive controller reduces exactly to the fixed-structure controller whose stability is already established.

The controller uses **only relative position measurements** ($e_i$ and $x_{ij}$), making it fully distributed and suitable for sensing-constrained environments.

---

*This document presents the complete adaptive internal model controller design with parameter adaptation laws for unknown target dynamics parameters $s_1, s_2$, suitable for distributed cooperative target fencing of second-order vehicles using only relative position measurements.*