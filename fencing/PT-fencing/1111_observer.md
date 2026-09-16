# Decentralized Target Position Estimator (Remark 6)

## Background

The controllers (6) and (20) are derived under the assumption that the target's position $x_0$ is available for all vehicles. From a practical perspective, however, only a few vehicles can access $x_0$ considering undesired physical constraints such as distance limitation and obstacles. This observation motivates the design of a decentralized estimator for $x_0$.

## Estimator Design

The decentralized estimator is given by:

$$
\begin{aligned}
\dot{\varepsilon}_i &= \kappa \left( \sum_{j \in \mathcal{N}_i} (\varepsilon_j - \varepsilon_i) + g_i(t) (x_0 - \varepsilon_i) \right) + \rho_i \\
\dot{\rho}_i &= \gamma \kappa \left( \sum_{j \in \mathcal{N}_i} (\varepsilon_j - \varepsilon_i) + g_i(t) (x_0 - \varepsilon_i) \right)
\end{aligned}
\tag{25}
$$

where:
- $\varepsilon_i$ is vehicle $i$'s estimation of $x_0$
- $\rho_i$ is an intermediate state
- $\gamma > 0$, $\kappa > 0$ are design parameters
- $g_i(t)$ is a time-varying indicator function:

$$
g_i(t) = \begin{cases}
1 & \text{if vehicle } i \text{ can detect } x_0 \text{ at } t, \\
0 & \text{otherwise.}
\end{cases}
$$

**Assumption:** The target $x_0$ can be detected by at least one vehicle, i.e., for any $t$, there exists at least one $i \in \mathbb{N}$ such that $g_i(t) = 1$.

## Convergence Analysis

Let $\{ (L_1, G_1), (L_2, G_2), \dots, (L_m, G_m) \}$ be the finite number of possible network topologies at every instant of time. For a piecewise-constant time function $\sigma(t): [0, \infty) \mapsto \{1, 2, \ldots, m\}$:
- $L_{\sigma(t)}$ is the Laplacian matrix for the $n$ vehicles according to $\mathcal{N}_i(t)$, $i \in \mathbb{N}$
- $G_{\sigma(t)} = \operatorname{diag}\{g_1(t), \ldots, g_n(t)\}$

Define the estimation errors:

$$
\boldsymbol{\varepsilon} = [\varepsilon_1^T, \ \dots \ , \varepsilon_n^T]^T - \mathbf{1} \otimes x_0, \quad
\boldsymbol{\rho} = [\rho_1^T, \ldots, \rho_n^T]^T - \mathbf{1} \otimes \nu_0
$$

where $\mathbf{1} = [1, \ . . . \ , 1]^T \in \mathbb{R}^n$ and $\otimes$ is the Kronecker product.

According to (2) and (25), the error dynamics are:

$$
\begin{aligned}
\dot{\boldsymbol{\varepsilon}} &= -\kappa (L_{\sigma} + G_{\sigma}) \otimes I_2 \, \boldsymbol{\varepsilon} + I_n \otimes I_2 \, \boldsymbol{\rho} \\
\dot{\boldsymbol{\rho}} &= -\gamma \kappa (L_{\sigma} + G_{\sigma}) \otimes I_2 \, \boldsymbol{\varepsilon}
\end{aligned}
\tag{26}
$$

Let $y = [\boldsymbol{\varepsilon}^T, \boldsymbol{\rho}^T]^T$ and $M_{\sigma} = L_{\sigma} + G_{\sigma}$. Equation (26) can be written compactly as:

$$
\dot{y} = A_{\sigma} y, \quad A_{\sigma} = \begin{pmatrix} -\kappa M_{\sigma} & I_n \\ -\gamma \kappa M_{\sigma} & 0 \end{pmatrix} \otimes I_2
$$

## Lyapunov Stability Proof

Select a Lyapunov function $V(y) = y^T P y$ with a symmetric positive definite matrix:

$$
P = \begin{pmatrix} \gamma_1 I_n & -\gamma I_n \\ -\gamma I_n & 2\gamma I_n \end{pmatrix} \otimes I_2, \quad \gamma_1 > \gamma^2
$$

The time derivative of $V(y)$ along (26) is:

$$
\dot{V}(y) = y^T (A_{\sigma}^T P + P A_{\sigma}) y = -y^T Q_{\sigma} y
$$

where

$$
Q_{\sigma} = -(A_{\sigma}^T P + P A_{\sigma}) = \begin{pmatrix} 2\kappa(\gamma_1 - \gamma^2) M_{\sigma} & -\gamma_1 I_n \\ -\gamma_1 I_n & 2\gamma I_n \end{pmatrix} \otimes I_2
$$

**Key result:** If the network of the $n$ vehicles is connected, then the matrix $M_{\sigma}$ is positive definite, meaning its minimum eigenvalue $\bar{\lambda} = \lambda_{\min}(M_{\sigma}) > 0$.

If $\kappa$ is selected as:

$$
\kappa > \frac{\gamma_1^2}{4\gamma(\gamma_1 - \gamma^2)\bar{\lambda}}
$$

then $2\gamma I_n - \frac{\gamma_1^2}{2\kappa(\gamma_1 - \gamma^2)} M_{\sigma}^{-1}$ is positive definite, which further implies that $Q_{\sigma}$ is positive definite by Schur complement.

Thus:

$$
\dot{V}(y) \leq -\beta V(y), \quad 0 < \beta \leq \frac{\lambda_{\min}(Q_{\sigma})}{\lambda_{\max}(P)}
$$

Consequently:

$$
\lim_{t \to \infty} V(y(t)) = 0 \quad \text{and} \quad \lim_{t \to \infty} \varepsilon_i(t) = x_0, \quad \forall i \in \mathbb{N}
$$

exponentially.

**Remark:** An auxiliary parameter $\gamma_1 > \gamma^2$ is introduced to relax the requirement of $\gamma \in (0, 1)$, making the proposed decentralized estimator more general compared to Reference 43.

## Modified Controller with Estimator

Using the estimated target position $\varepsilon_i$ instead of the true $x_0$, the controller (6) is modified as:

$$
\begin{aligned}
u_i &= \phi_i + k_2 (\varepsilon_i - x_i) + k_1 (\hat{\nu}_i - \nu_i) \\
\dot{\hat{\nu}}_i &= \frac{k_3}{k_1 k_2} \left( \phi_i + k_2 (\varepsilon_i - x_i) \right) \\
\phi_i &= \sum_{j \in \mathcal{N}_i} \alpha(\|x_{ij}\|) \frac{x_{ij}}{\|x_{ij}\|}
\end{aligned}
\tag{27}
$$

The analysis similar to Theorem 1 can be used to prove that the controller (27) ensures properties (P1)–(P3). The modification for the controller (20) is similar and thus omitted.