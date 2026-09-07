# Task: Asymptotic Fencing Control for Heterogeneous EL Systems with Uncertain Parameters

## Problem Statement

### System Model

Consider a swarm of $N$ ($N \geq 3$) heterogeneous Euler-Lagrange systems (HELS):

$$
M_i(q_i)\ddot{q}_i + C_i(q_i,\dot{q}_i)\dot{q}_i + g_i(q_i) = \tau_i, \quad i \in \mathcal{N} = \{1,\dots,N\},
$$

where $q_i \in \mathbb{R}^p$ is the generalized coordinate, $M_i(q_i)$ is the inertia matrix, $C_i(q_i,\dot{q}_i)$ is the Coriolis/centrifugal term, $g_i(q_i)$ is the gravity vector, and $\tau_i$ is the control input. The target (index 0) has state $(q_0, \dot{q}_0)$ with bounded acceleration $\ddot{q}_0$.

### Standard Assumptions

- **A1 (Boundedness):** $0 < k_{\underline{m}}I_p \leq M_i(q_i) \leq k_{\overline{m}}I_p$, $\|C_i(x,y)z\| \leq k_C\|y\|\|z\|$, $\|g_i(q_i)\| \leq k_{g_i}$.
- **A2 (Skew-symmetry):** $\dot{M}_i(q_i) - 2C_i(q_i,\dot{q}_i)$ is skew-symmetric.
- **A3 (Parameter linearization):** $M_i(q_i)x + C_i(q_i,\dot{q})y + g_i(q_i) = Y_i(q_i,\dot{q}_i,x,y)\Theta_i$, where $\Theta_i$ is the unknown constant parameter vector.
- **A4 (Target):** $\ddot{q}_0$ is bounded and $\dot{q}_0$ is continuous.

### Control Objectives

- **P1 (Convex-hull fencing):** $\lim_{t\to\infty} q_0(t) \in \lim_{t\to\infty} \operatorname{co}(q(t))$ — the target asymptotically enters the convex hull of the agents.
- **P2 (Collision avoidance):** $\|q_i(t) - q_j(t)\| > d$ for all $i \neq j$, $t \geq 0$.
- **P3 (Velocity matching):** $\lim_{t\to\infty} \|\dot{q}_i(t) - \dot{q}_0(t)\| = 0$ for all $i$.

### The APF-Based Fencing Framework

The artificial potential function (APF) for collision avoidance is:

$$
\phi_i = \sum_{j \in \mathcal{N}_i} \alpha(\|q_{ij}\|)\frac{q_{ij}}{\|q_{ij}\|},
$$

where $\alpha(s) \to \infty$ as $s \to d^+$ (repulsion near safety distance $d$), $\alpha(s) = 0$ for $s \geq \mu$ (communication radius), and $\sum_i \phi_i = 0$ (action-reaction cancellation).

## The Core Challenge

### The Triple Constraint

We need a controller that simultaneously satisfies:

| # | Constraint | Why it is hard |
|---|-----------|----------------|
| **C1** | **No PE condition** | The target's motion may not be persistently exciting (e.g., constant velocity or stationary). Standard adaptive controllers require PE for parameter convergence and asymptotic tracking. |
| **C2** | **No $\dot{\phi}_i$ in the control law** | $\phi_i$ depends on neighbor sets $\mathcal{N}_i$ and can be discontinuous when agents enter/leave communication range. Computing $\dot{\phi}_i$ requires differentiating $\alpha(\|q_{ij}\|)$, which is noisy and practically undesirable. |
| **C3** | **Asymptotic convergence** (P1, P3) | Practical (ultimately bounded) convergence is easier but weaker. We want the target to be *exactly* fenced in the limit. |

### Why Existing Approaches Fail

| Approach | Fails C1 (No PE) | Fails C2 (No $\dot{\phi}_i$) | Fails C3 (Asymptotic) |
|----------|:---:|:---:|:---:|
| **Controller 1** (Integral APF) | ✅ | ✅ | ❌ (practical only) |
| **Controller 2** (Differential APF) | ✅ | ❌ (needs $\dot{\phi}_i$) | ✅ |
| **Controller 3** (Filtered APF) | ✅ | ✅ | ❌ (proof flawed) |
| **Controller 4** (DSC) | ✅ | ✅ | ❌ (practical only) |
| **Controller 5-1** (Robust + SM) | ✅ | ✅ | ✅, but needs a global $\|Y_i\tilde{\Theta}_i\|$ bound (unavailable for integral APF) |
| **Controller 5-2** (Adaptive gain) | ✅ | ✅ | ❌ (practical only) |
| **Controller 5-3** (Projection + SM, state-dep. gain) | ✅ | ✅ | ✅ (best candidate) |

### Controller 5-3: The Best Candidate

**Idea:** Combine the integral APF (to avoid $\dot{\phi}_i$) with a sliding-mode robust term (to force $s_i \to 0$ in finite time, making the residual $\dot{s}_i$ vanish exactly), and a projection operator on the parameter estimates. The robust gain is made **state-dependent** to avoid requiring a global bound on $\|Y_i\|$ (which is problematic because the integral term in $\zeta_i$ can make $\|Y_i\|$ unbounded).

**Mechanism:**

1. **Integral APF** (same as Controller 1):
   $$ \zeta_i = \dot{q}_0 - k_\alpha q_{i0} + \int_0^t (-k_\alpha q_{i0} + \phi_i(\tau))\,d\tau $$
   $$ s_i = \dot{q}_i - \zeta_i $$
   This gives $\dot{s}_i = \ddot{q}_{i0} + k_\alpha\dot{q}_{i0} + k_\alpha q_{i0} - \phi_i$ with no $\dot{\phi}_i$.

2. **Sliding-mode robust term**:
   $$ \tau_i = -k s_i - \rho_i(t)\,\text{sgn}(s_i) + Y_i\hat{\Theta}_i $$
   Drives $s_i \to 0$ in finite time, so $\dot{s}_i \equiv 0$ thereafter.

3. **Projection adaptation**:
   $$ \dot{\hat{\Theta}}_i = \text{Proj}_{\hat{\Theta}_i}(-\Lambda^{-1}Y_i^T s_i) $$
   Keeps $\|\hat{\Theta}_i\| \leq \Theta_{i,\max}$, so $\|\tilde{\Theta}_i\| \leq 2\Theta_{i,\max}$.

4. **State-dependent robust gain** (key innovation):
   $$ \rho_i(t) = \|Y_i\hat{\Theta}_i\| + \rho_{i0} + \rho_{i1}\|\dot{q}_{i0}\| + \rho_{i2}\|q_{i0}\| + \rho_{i3}\|\dot{q}_i\|^2 + \rho_{i4}\|\dot{q}_i\|\|s_i\| + \eta_i $$
   
   The coefficients $\rho_{i0},\dots,\rho_{i4}$ are chosen based on the **known structural bounds** from A1 ($k_{\overline m}$, $k_C$, $k_{g_i}$), A4 ($\bar{a}_0$), and the APF bound $\bar{\phi}$. This guarantees $\rho_i(t) \ge \|Y_i\tilde{\Theta}_i\| + \eta_i$ pointwise, without any global bound on $\|Y_i\|$.

**Why $\|Y_i\| \le k_Y$ was problematic:** The integral term in $\zeta_i$ can accumulate over time, making $\zeta_i$ and hence $Y_i$ potentially unbounded, even on collision-free trajectories. Unlike the standard Slotine-Li controller (where $\dot{q}_r = \dot{q}_d - \Lambda\tilde{q}$ contains no integral term and is a priori bounded), the integral APF loses this property.

**How the state-dependent gain fixes it:** From $Y_i\tilde{\Theta}_i = Y_i\hat{\Theta}_i - (M_i\dot{\zeta}_i + C_i\zeta_i + g_i)$, we use A1 to bound $\|M_i\dot{\zeta}_i + C_i\zeta_i + g_i\|$ by a function of $\|\dot{q}_i\|$, $\|s_i\|$, $\|\dot{q}_{i0}\|$, $\|q_{i0}\|$, and known constants. The term $\|Y_i\hat{\Theta}_i\|$ is measured directly in the controller. The resulting bound is a **known function of measurable signals**, so the gain can be computed online.

**After sliding ($t \geq T^*$):** $s_i \equiv 0$, $\dot{s}_i \equiv 0$, so

$$ \ddot{q}_{i0} = -k_\alpha\dot{q}_{i0} - k_\alpha q_{i0} + \phi_i, $$

which is the clean APF-driven dynamics of Controller 2, yielding asymptotic fencing (P1, P3) and collision avoidance (P2).

### What Has Been Done

- ✅ **Five controllers designed** and analyzed (C1–C5-3).
- ✅ **Controller 5-3** identified as the best candidate.
- ✅ **Complete proof written** for Controller 5-3 (8 steps, 5 pages).
- ✅ **Circularity issue fixed**: replaced constant $\|Y_i\| \le k_Y$ assumption with state-dependent gain using known structural bounds from A1.
- ✅ **LaTeX compiles** without errors (0 errors, 0 undefined references).
- ❌ **Simulation** (`body/simulation.tex`) is currently empty — needs implementation.

### What Remains

1. **Simulations:** Implement numerical simulations for Controller 5-3 (and possibly C1 as baseline) to validate the theoretical results.
2. **Comparison:** Show the performance gap between practical convergence (C1) and asymptotic convergence (C5-3).
3. **Self-organizing formation:** Demonstrate the ability to handle agents joining/leaving.
4. **Chattering mitigation:** Implement $\text{sat}(s_i/\delta_i)$ instead of $\text{sgn}(s_i)$ and analyze the trade-off.

### Key References

- Slotine & Li, *On the Adaptive Control of Robot Manipulators*, 1987. (Slotine-Li adaptive framework)
- Ioannou & Sun, *Robust Adaptive Control*, 1996. (Projection operator)
- Khalil, *Nonlinear Systems*, 3rd ed., 2002. (Finite-time convergence, comparison lemma)
- Chen, *A Cooperative Target-Fencing Protocol of Multiple Vehicles*, 2019. (Original fencing problem)