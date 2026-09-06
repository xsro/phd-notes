# Fencing Control for Euler-Lagrange Swarms

## Problem Description

Consider $N$ ($N\geq 3$) agents modeled by heterogeneous Euler-Lagrange systems (HELS):

$$
M_i(q_i)\ddot{q}_i + C_i(q_i,\dot{q}_i)\dot{q}_i + g_i(q_i) = \tau_i,\quad i \in \mathcal{N} = \{1,\dots,N\},
$$

where $q_i\in\mathbb{R}^p$ is the generalized coordinate, $M_i$ is the inertia matrix, $C_i$ represents Coriolis/centrifugal forces, $g_i$ is gravity, and $\tau_i$ is the control input. The target is denoted by index 0 with state $(q_0,\dot{q}_0)$.

### Assumptions

- **A1 (Boundedness):** $0 < k_{\underline{m}}I_p \leq M_i(q_i) \leq k_{\overline{m}}I_p$, $\|C_i(x,y)z\| \leq k_C\|y\|\|z\|$, $\|g_i(q_i)\| \leq k_{g_i}$.
- **A2 (Skew-symmetry):** $\dot{M}_i(q_i) - 2C_i(q_i,\dot{q}_i)$ is skew-symmetric.
- **A3 (Linearization):** $M_i(q_i)x + C_i(q_i,\dot{q})y + g_i(q_i) = Y_i(q_i,\dot{q}_i,x,y)\Theta_i$ with regressor $Y_i$ and unknown constant parameter vector $\Theta_i$.
- **A4 (Maneuvering target):** $\ddot{q}_0$ or $\dot{q}_0$ is not a constant value.

### Control Objectives

- **P1 (Convex hull fencing):** $\lim_{t\to\infty} q_0(t) \in \lim_{t\to\infty} \operatorname{co}(q(t))$.
- **P2 (Collision avoidance):** $\|q_i(t) - q_j(t)\| > d$, $\forall i\neq j$, $\forall t\geq 0$.
- **P3 (Velocity matching):** $\lim_{t\to\infty} \|\dot{q}_i(t) - \dot{q}_0(t)\| = 0$, $\forall i$.

---

## Controller 1: Integral APF

**Requires PE condition:** Yes — the ISS argument for (P1) needs $\dot{s}_i\to0$, which requires $Y_i\tilde{\Theta}_i\to0$; without PE the parameter error does not vanish.

### Error Variables

$$
q_{i0} = q_i - q_0,\qquad \dot{\tilde{q}}_i = \dot{q}_i - \dot{q}_0.
$$

### Sliding Variable

$$
s_i = \dot{q}_i - \zeta_i,\qquad
\zeta_i = \dot{q}_0 - k_\alpha q_{i0} + \int_0^t \bigl(-k_\alpha q_{i0} + \phi_i(\tau)\bigr) d\tau,
$$

where $\phi_i = \sum_{j\in\mathcal{N}_i} \alpha(\|q_{ij}\|)\frac{q_{ij}}{\|q_{ij}\|}$ is the artificial potential force, $\mathcal{N}_i = \{j : \|q_i - q_j\| \leq \mu\}$ is the neighbor set, and $\alpha(\cdot)$ is a potential function satisfying:

- $\lim_{s\to d^+} \alpha(s) = +\infty$ (repulsion near safety distance $d$),
- $\alpha(s) = 0$ for $s \geq \mu$ (finite communication range).

### Control Law

$$
\tau_i = -k s_i + Y_i(q_i,\dot{q}_i,\dot{\zeta}_i,\zeta_i) \hat{\Theta}_i,
$$

where $k > 0$ and $Y_i$ satisfies the regression equation:

$$
M_i(q_i)\dot{\zeta}_i + C_i(q_i,\dot{q}_i)\zeta_i + g_i(q_i) = Y_i(q_i,\dot{q}_i,\dot{\zeta}_i,\zeta_i)\Theta_i.
$$

### Adaptation Law

$$
\dot{\hat{\Theta}}_i = -\Lambda^{-1} Y_i^T(q_i,\dot{q}_i,\dot{\zeta}_i,\zeta_i) s_i,
$$

where $\Lambda > 0$ is the adaptation gain matrix.

### Theorem

> Under Assumptions A1–A3 and the PE condition (the signal $[\dot{q}_0^T,\ddot{q}_0^T]^T$ is sufficiently rich such that $\int_t^{t+T} Y_i^T Y_i\,d\tau \geq \alpha I$ for some $T>0$, $\alpha>0$), the HELS governed by the control law and adaptation law above achieves objectives P1–P3.

---

## Controller 2: Differential APF

**Requires PE condition:** No — the proof uses direct completion of squares on a combined Lyapunov function $V = V_1/(4k) + V_2$, yielding $\dot{V} = -\sum_i\|{-k_\alpha q_{i0} + \phi_i + s_i/2}\|^2 \leq 0$. Barbalat's lemma then gives $q_{i0}\to0$, $\phi_i\to0$, $s_i\to0$, and $\dot{q}_{i0}\to0$ without requiring parameter convergence.

### Error Variables

Same as Controller 1: $q_{i0} = q_i - q_0$, $\dot{\tilde{q}}_i = \dot{q}_i - \dot{q}_0$.

### Sliding Variable

$$
s_i = \dot{q}_i - \zeta_i = \dot{q}_{i0} + k_\alpha q_{i0} - \phi_i,
$$

where $\zeta_i = \dot{q}_0 - k_\alpha q_{i0} + \phi_i$ and $\phi_i = \sum_{j\in\mathcal{N}_i} \alpha(\|q_{ij}\|)\frac{q_{ij}}{\|q_{ij}\|}$.

### Control Law

$$
\tau_i = -k s_i + Y_i(q_i,\dot{q}_i,\dot{\zeta}_i,\zeta_i) \hat{\Theta}_i,
$$

with $k > 0$ and the regression equation:

$$
M_i(q_i)\dot{\zeta}_i + C_i(q_i,\dot{q}_i)\zeta_i + g_i(q_i) = Y_i(q_i,\dot{q}_i,\dot{\zeta}_i,\zeta_i)\Theta_i.
$$

### Adaptation Law

$$
\dot{\hat{\Theta}}_i = -\Lambda^{-1} Y_i^T(q_i,\dot{q}_i,\dot{\zeta}_i,\zeta_i) s_i,
$$

where $\Lambda > 0$ is the adaptation gain matrix.

### Theorem

> Under Assumptions A1–A3, the HELS governed by the control law and adaptation law above achieves objectives P1–P3. The PE condition is **not required**; parameter estimates need not converge to their true values.

---

## Key Difference Between Controllers

| Aspect | Controller 1 (Integral APF) | Controller 2 (Differential APF) |
|--------|----------------------------|--------------------------------|
| $\zeta_i$ | Contains integral of $-k_\alpha q_{i0} + \phi_i$ | Direct algebraic expression |
| Sliding var | $s_i = \dot{q}_i - \zeta_i$ | $s_i = \dot{q}_{i0} + k_\alpha q_{i0} - \phi_i$ |
| Lyapunov $V_2$ | Includes $\frac12\sum_i\|\dot{q}_{i0}\|^2$ | Potential-only (no velocity term) |
| Proof technique | $W = V_2 - \int h\,dt$ with Barbalat + ISS | Direct completion of squares |
| **PE required?** | **Yes** — needed for $\dot{s}_i\to0$ in ISS argument | **No** — Barbalat on $\dot{V}$ gives $q_{i0}\to0$ directly |