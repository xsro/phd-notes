# 异构 Euler-Lagrange 群体的围栏控制（Fencing Control）

## 问题描述

考虑 $N$（$N \geq 3$）个异构 Euler-Lagrange 系统（HELS）智能体：

$$
M_i(q_i)\ddot{q}_i + C_i(q_i,\dot{q}_i)\dot{q}_i + g_i(q_i) = \tau_i,\quad i \in \mathcal{N} = \{1,\dots,N\},
$$

其中 $q_i\in\mathbb{R}^p$ 是广义坐标，$M_i$ 是惯量矩阵，$C_i$ 是科氏/离心力项，$g_i$ 是重力项，$\tau_i$ 是控制输入。目标（Target）用下标 $0$ 表示，状态为 $(q_0,\dot{q}_0)$。

### 假设条件

- **A1（有界性）：** $0 < k_{\underline{m}}I_p \leq M_i(q_i) \leq k_{\overline{m}}I_p$，$\|C_i(x,y)z\| \leq k_C\|y\|\|z\|$，$\|g_i(q_i)\| \leq k_{g_i}$。
- **A2（反对称性）：** $\dot{M}_i(q_i) - 2C_i(q_i,\dot{q}_i)$ 是斜对称的。
- **A3（参数线性化）：** $M_i(q_i)x + C_i(q_i,\dot{q})y + g_i(q_i) = Y_i(q_i,\dot{q}_i,x,y)\Theta_i$，其中 $Y_i$ 是回归矩阵，$\Theta_i$ 是未知常参数向量。
- **A4（机动目标）：** $\ddot{q}_0$ 或 $\dot{q}_0$ 不是常值。

### 控制目标

- **P1（凸包围栏）：** $\lim_{t\to\infty} q_0(t) \in \lim_{t\to\infty} \operatorname{co}(q(t))$，即目标最终进入群体凸包内部。
- **P2（碰撞避免）：** $\|q_i(t) - q_j(t)\| > d$，$\forall i\neq j$，$\forall t\geq 0$。
- **P3（速度匹配）：** $\lim_{t\to\infty} \|\dot{q}_i(t) - \dot{q}_0(t)\| = 0$，$\forall i$。

---

## 人工势场（APF）基础

每个智能体 $i$ 对邻居 $j\in\mathcal{N}_i = \{j : \|q_i - q_j\| \leq \mu\}$ 施加排斥力：

$$
\phi_i = \sum_{j\in\mathcal{N}_i} \alpha(\|q_{ij}\|)\frac{q_{ij}}{\|q_{ij}\|},
$$

其中势函数 $\alpha(\cdot)$ 满足：

- $\lim_{s\to d^+} \alpha(s) = +\infty$（在安全距离 $d$ 附近趋于无穷，保证 P2），
- $\alpha(s) = 0$ for $s \geq \mu$（超出通信范围 $\mu$ 后无作用），
- $\sum_i \phi_i = 0$（作用力与反作用力成对抵消）。

---

## 五种控制器总览

本项目共设计了 **五种** 控制器，按"是否需要 PE 条件"和"是否需要对 $\phi_i$ 求导 $\dot{\phi}_i$"两个维度分类。

| 控制器 | 文件 | 核心机制 | 需要 PE？ | 需要 $\dot{\phi}_i$？ | 收敛类型 |
|--------|------|----------|-----------|---------------------|----------|
| **Controller 1** | `body/result.tex` | 积分 APF（$\zeta_i$ 含积分项） | 否 | 否 | 实用收敛（practical） |
| **Controller 2** | `body/result2.tex` | 微分 APF（$\zeta_i$ 直接代数式） | 否 | **是**（控制律 + 证明） | 渐近收敛（asymptotic） |
| **Controller 3** | `body/result3.tex` | 滤波 APF（$\dot\Phi_i = -k_\phi\Phi_i+\phi_i$） | 否 | 否 | 实用收敛（证明为 sketch） |
| **Controller 4** | `body/result4.tex` | 命令滤波反步 / DSC | 否 | 否（控制律） | 实用收敛 |
| **Controller 5** | `body/result5.tex` | 鲁棒自适应 + 积分 APF + 滑模 | 否 | 否 | **渐近收敛** |

---

## 各控制器要点

### Controller 1：积分 APF（`result.tex`）

滑模面定义为

$$
s_i = \dot{q}_i - \zeta_i,\qquad
\zeta_i = \dot{q}_0 - k_\alpha q_{i0} + \int_0^t \bigl(-k_\alpha q_{i0} + \phi_i(\tau)\bigr) d\tau,
$$

因此 $\dot{\zeta}_i = \ddot{q}_0 - k_\alpha\dot{q}_{i0} - k_\alpha q_{i0} + \phi_i$ **含 $\phi_i$ 但不含 $\dot{\phi}_i$**。控制律 $\tau_i = -k s_i + Y_i\hat{\Theta}_i$，自适应律 $\dot{\hat{\Theta}}_i = -\Lambda^{-1}Y_i^T s_i$。

- **PE：不需要**（自适应律无 PE 要求；$s_i\to0$ 通过 Barbalat）。
- **$\dot{\phi}_i$：不需要**（$\zeta_i$ 用积分构造）。
- **代价**：$\dot{s}_i$ 残余项不消失，$\dot{V}_2$ 中有 $\sum\dot{q}_{i0}^T\dot{s}_i$ 交叉项，只能得到实用收敛，速度误差最终有界 $\sigma\sqrt{N}/k_\alpha$。

### Controller 2：微分 APF（`result2.tex`）

滑模面 $s_i = \dot{q}_{i0} + k_\alpha q_{i0} - \phi_i$，$\zeta_i = \dot{q}_0 - k_\alpha q_{i0} + \phi_i$。

- **PE：不需要**（证明用配平方 $\dot{V} = -\sum_i\|{-k_\alpha q_{i0} + \phi_i + s_i/2}\|^2 \leq 0$）。
- **$\dot{\phi}_i$：需要**（控制律中的回归 $Y_i$ 需要 $\dot{\zeta}_i$，而 $\dot{\zeta}_i$ 含 $\dot{\phi}_i$）。
- **优点**：渐近收敛，是五种里收敛最强的。

### Controller 3：滤波 APF（`result3.tex`）

对 $\phi_i$ 本身加一阶滤波 $\dot{\Phi}_i = -k_\phi\Phi_i + \phi_i$，用 $\Phi_i$ 替代 $\phi_i$。

- **PE：不需要**；**$\dot{\phi}_i$：不需要**。
- **代价**：证明为 sketch，交叉项 $\sum\dot{q}_{i0}^T(\phi_i-\Phi_i)$ 不消失，需要高增益压制。

### Controller 4：命令滤波反步 / DSC（`result4.tex`）

虚拟控制 $\alpha_i = -k_\alpha q_{i0} + \phi_i$，过一阶命令滤波器 $\varepsilon\dot{\alpha}_{if} = -\alpha_{if} + \alpha_i$。滑模面 $s_i = \dot{q}_{i0} - \alpha_{if}$，$\zeta_i = \dot{q}_0 + \alpha_{if}$。

- **PE：不需要**；**$\dot{\phi}_i$：控制律不需要**（$\dot{\alpha}_{if} = (\alpha_i-\alpha_{if})/\varepsilon$），但分析中仍要求 $\alpha(\cdot)\in C^1$。
- **代价**：滤波误差 $y_i = \alpha_{if}-\alpha_i$ 导致实用收敛，误差 $\propto B = \max_i\sup_t\|\dot{\alpha}_i(t)\|$。
- **优点**：实现最省心，$\varepsilon$ 调小即可逼近 Controller 2 的性能。

### Controller 5：鲁棒自适应 + 积分 APF + 滑模（`result5.tex`）

沿用 Controller 1 的积分 APF，控制律加滑模鲁棒项：

$$
\tau_i = -k s_i - \rho_i\,\mathrm{sgn}(s_i) + Y_i\hat{\Theta}_i,\qquad \rho_i \geq \|Y_i\tilde{\Theta}_i\| + \eta_i.
$$

- **PE：不需要**；**$\dot{\phi}_i$：不需要**。
- **核心机制**：滑模项使 $s_i$ 在**有限时间**内收敛到 $0$（$\dot{V}_{si} \leq -c_i\sqrt{V}_{si}$），之后 $\dot{s}_i \equiv 0$，残余项精确消失，恢复 Controller 2 的干净动力学 $\ddot{q}_{i0} = -k_\alpha\dot{q}_{i0} - k_\alpha q_{i0} + \phi_i$。
- **优点**：**渐近收敛**（P1、P3），同时不需要 $\dot{\phi}_i$ 和 PE 条件。
- **代价**：理想 $\mathrm{sgn}$ 有颤振（实际用 $\mathrm{sat}(s_i/\delta_i)$ 边界层近似后退化为实用收敛）；需要已知 $\|Y_i\tilde{\Theta}_i\|$ 的界。

---

## 核心设计权衡（Summary）

本项目要同时满足两个约束：**(1) 不需要 PE 条件**，**(2) 不需要对 $\phi_i$ 求导 $\dot{\phi}_i$**。五种控制器的取舍如下：

1. **Controller 1** 是最直接的方案——积分 APF 天然避开 $\dot{\phi}_i$，自适应律不需要 PE，代价是只能实用收敛。
2. **Controller 2** 想要渐近收敛就必须引入 $\dot{\phi}_i$（控制律和证明都需要），是"收敛最强但假设最重"的方案。
3. **Controller 3** 用滤波 $\Phi_i$ 绕开 $\dot{\phi}_i$，但证明不完整。
4. **Controller 4（DSC）** 是标准工程解法——用命令滤波器代替求导，控制律完全不含 $\dot{\phi}_i$，实用收敛，实现最省心。
5. **Controller 5（鲁棒自适应 + 滑模）** 是理论最优解——有限时间滑模使残余项精确归零，同时达成渐近收敛、无 PE、无 $\dot{\phi}_i$ 三项目标，代价是需要鲁棒增益的界和边界层近似。

**结论**：两个约束可以同时满足。若追求**工程实用**，选 Controller 4（DSC）；若追求**理论上的渐近收敛**且不想引入 $\dot{\phi}_i$，选 Controller 5（滑模鲁棒自适应）。

---

## 编译说明

- 使用 **pdflatex** 编译（`fontspec`/`xeCJK` 已注释，本项目无中文字符）。
- 编译流程：`pdflatex → bibtex → pdflatex → pdflatex`。
- 参考文献文件：`body/reference.bib`。