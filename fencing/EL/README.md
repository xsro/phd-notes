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
- **A4（目标轨迹）：** $\ddot{q}_0$ 有界，$\dot{q}_0$ 连续。

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

## 七大控制器总览

本项目共设计了 **七种** 控制器，按三个维度分类：是否需要 PE 条件、是否需要对 $\phi_i$ 求导 $\dot{\phi}_i$、收敛类型。

| 控制器 | 文件 | 核心机制 | 需要 PE？ | 需要 $\dot{\phi}_i$？ | 收敛类型 |
|--------|------|----------|-----------|---------------------|----------|
| **Controller 1** | `body/result.tex` | 积分 APF（$\zeta_i$ 含积分项） | 否 | 否 | 实用收敛（practical） |
| **Controller 2** | `body/result2.tex` | 微分 APF（$\zeta_i$ 直接代数式） | 否 | **是**（控制律 + 证明） | 渐近收敛（asymptotic） |
| **Controller 3** | `body/result3.tex` | 滤波 APF（$\dot\Phi_i = -k_\phi\Phi_i+\phi_i$） | 否 | 否 | 实用收敛（证明为 sketch） |
| **Controller 4** | `body/result4.tex` | 命令滤波反步 / DSC | 否 | 否（控制律） | 实用收敛 |
| **Controller 5** | `body/result5.tex` | 鲁棒自适应 + 积分 APF + 滑模（常值增益） | 否 | 否 | **渐近收敛**（需 $\|Y_i\tilde{\Theta}_i\|$ 的全局界） |
| **Controller 6** | `body/result6.tex` | 自适应鲁棒增益（无需先验界） | 否 | 否 | 实用收敛（渐近 $s_i\to0$，但 $\dot{s}_i$ 残余不消失） |
| **Controller 7** | `body/result7.tex` | 投影自适应 + 状态相关鲁棒增益（有限时间滑模） | 否 | 否 | **渐近收敛**（无需全局 $\|Y_i\|$ 界） |

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
- **优点**：渐近收敛，是收敛性最强的方案。

### Controller 3：滤波 APF（`result3.tex`）

对 $\phi_i$ 本身加一阶滤波 $\dot{\Phi}_i = -k_\phi\Phi_i + \phi_i$，用 $\Phi_i$ 替代 $\phi_i$。

- **PE：不需要**；**$\dot{\phi}_i$：不需要**。
- **代价**：证明为 sketch，交叉项 $\sum\dot{q}_{i0}^T(\phi_i-\Phi_i)$ 不消失，需要高增益压制。

### Controller 4：命令滤波反步 / DSC（`result4.tex`）

虚拟控制 $\alpha_i = -k_\alpha q_{i0} + \phi_i$，过一阶命令滤波器 $\varepsilon\dot{\alpha}_{if} = -\alpha_{if} + \alpha_i$。滑模面 $s_i = \dot{q}_{i0} - \alpha_{if}$，$\zeta_i = \dot{q}_0 + \alpha_{if}$。

- **PE：不需要**；**$\dot{\phi}_i$：控制律不需要**（$\dot{\alpha}_{if} = (\alpha_i-\alpha_{if})/\varepsilon$），但分析中仍要求 $\alpha(\cdot)\in C^1$。
- **代价**：滤波误差 $y_i = \alpha_{if}-\alpha_i$ 导致实用收敛，误差 $\propto B = \max_i\sup_t\|\dot{\alpha}_i(t)\|$。
- **优点**：实现最省心，$\varepsilon$ 调小即可逼近 Controller 2 的性能。

### Controller 5：鲁棒自适应 + 积分 APF + 滑模（常值增益）（`result5.tex`）

沿用 Controller 1 的积分 APF，控制律加滑模鲁棒项：

$$
\tau_i = -k s_i - \rho_i\,\mathrm{sgn}(s_i) + Y_i\hat{\Theta}_i,\qquad \rho_i \geq \|Y_i\tilde{\Theta}_i\| + \eta_i.
$$

- **PE：不需要**；**$\dot{\phi}_i$：不需要**。
- **核心机制**：滑模项使 $s_i$ 在**有限时间**内收敛到 $0$（$\dot{V}_{si} \leq -c_i\sqrt{V}_{si}$），之后 $\dot{s}_i \equiv 0$，残余项精确消失，恢复 Controller 2 的干净动力学 $\ddot{q}_{i0} = -k_\alpha\dot{q}_{i0} - k_\alpha q_{i0} + \phi_i$。
- **优点**：**渐近收敛**（P1、P3），同时不需要 $\dot{\phi}_i$ 和 PE 条件。
- **代价**：理想 $\mathrm{sgn}$ 有颤振（实际用 $\mathrm{sat}(s_i/\delta_i)$ 边界层近似后退化为实用收敛）；需要已知 $\|Y_i\tilde{\Theta}_i\|$ 的**全局界**，这在积分 APF 框架下难以保证（因为 $\zeta_i$ 的积分项可能使 $Y_i$ 无界）。

### Controller 6：自适应鲁棒增益（`result6.tex`）

Controller 5 需要知道 $\|Y_i\tilde{\Theta}_i\|$ 的全局界才能设定 $\rho_i$。Controller 6 通过在线自适应避开了这个先验要求：

$$
\tau_i = -k s_i - \hat{\rho}_i\,\mathrm{sgn}(s_i) + Y_i\hat{\Theta}_i,\qquad
\dot{\hat{\rho}}_i = \gamma\|s_i\|_1.
$$

- **PE：不需要**；**$\dot{\phi}_i$：不需要**。
- **核心机制**：$\hat{\rho}_i$ 单调增长直到足以抑制 $Y_i\tilde{\Theta}_i$。Lyapunov 分析给出 $\dot{V} = -k\|s_i\|^2 - \rho_i^*\|s_i\|_1 \leq 0$，其中 $\rho_i^* = \sup_t\|Y_i\tilde{\Theta}_i\|$ 是未知的真值界。
- **优点**：**无需任何先验界**（对 $\|Y_i\|$ 或 $\|Y_i\tilde{\Theta}_i\|$ 均不需要）。
- **代价**：$\hat{\rho}_i$ 自适应是渐近的（不是有限时间），所以 $s_i$ 只渐近趋于零，$\dot{s}_i$ 残余项不精确消失，围栏目标变为**实用收敛**。

### Controller 7：投影自适应 + 状态相关鲁棒增益（有限时间滑模）（`result7.tex`）

Controller 6 丢失了有限时间收敛性质。Controller 7 通过**投影算子** + **状态相关增益**同时恢复有限时间滑模且避免全局 $\|Y_i\|$ 界假设。

**核心机制：**

1. **投影算子**：$\dot{\hat{\Theta}}_i = \mathrm{Proj}_{\hat{\Theta}_i}(-\Lambda^{-1}Y_i^T s_i)$，保证 $\|\hat{\Theta}_i\| \leq \Theta_{i,\max}$，从而 $\|\tilde{\Theta}_i\| \leq 2\Theta_{i,\max}$。
2. **状态相关鲁棒增益**（区别于 Controller 5 的常值增益）：

$$
\rho_i(t) = \|Y_i\hat{\Theta}_i\| + \rho_{i0} + \rho_{i1}\|\dot{q}_{i0}\| + \rho_{i2}\|q_{i0}\| + \rho_{i3}\|\dot{q}_i\|^2 + \rho_{i4}\|\dot{q}_i\|\|s_i\| + \eta_i.
$$

系数 $\rho_{i0},\dots,\rho_{i4}$ 由 A1 的结构常数（$k_{\overline m}, k_C, k_{g_i}$）和 A4 的目标加速度界 $\bar{a}_0$ 确定，无需假设 $\|Y_i\|$ 全局有界。

**为什么不需要 $\|Y_i\| \leq k_Y$：**

利用 $Y_i\tilde{\Theta}_i = Y_i\hat{\Theta}_i - (M_i\dot{\zeta}_i + C_i\zeta_i + g_i)$，通过 A1 将 $\|M_i\dot{\zeta}_i + C_i\zeta_i + g_i\|$ 上界表示为 $\|\dot{q}_i\|,\|s_i\|,\|\dot{q}_{i0}\|,\|q_{i0}\|$ 的已知函数。$\|Y_i\hat{\Theta}_i\|$ 由控制器直接计算。因此 $\rho_i(t)$ 是**可测信号的已知函数**，点态满足 $\rho_i(t) \geq \|Y_i\tilde{\Theta}_i\| + \eta_i$。

- **PE：不需要**；**$\dot{\phi}_i$：不需要**。
- **优点**：有限时间 $s_i\to0$ 恢复**渐近收敛**，且无需全局 $\|Y_i\|$ 界假设。证明是自洽的（无循环论证）。
- **代价**：状态相关增益表达式较复杂；理想 $\mathrm{sgn}$ 仍有颤振（实际用 $\mathrm{sat}$ 近似）。

---

## 核心设计权衡（Summary）

本项目要同时满足三个约束：**(1) 不需要 PE 条件**，**(2) 不需要对 $\phi_i$ 求导 $\dot{\phi}_i$**，**(3) 渐近收敛**。七种控制器的取舍如下：

| # | 方案 | 避 PE | 避 $\dot{\phi}_i$ | 渐近收敛 | 先验信息需求 |
|---|------|:-----:|:-----------------:|:---------:|:------------:|
| 1 | 积分 APF | ✅ | ✅ | ❌（实用） | 只需 $\Theta_i$ 参数化 |
| 2 | 微分 APF | ✅ | ❌（需要 $\dot{\phi}_i$） | ✅ | 只需 $\Theta_i$ 参数化 |
| 3 | 滤波 APF | ✅ | ✅ | ❌（证明不完整） | 只需 $\Theta_i$ 参数化 |
| 4 | DSC | ✅ | ✅ | ❌（实用） | 只需 $\Theta_i$ 参数化 |
| 5 | 鲁棒 + 滑模（常值增益） | ✅ | ✅ | ✅ | 需 $\|Y_i\tilde{\Theta}_i\|$ 全局界（难以保证） |
| 6 | 自适应鲁棒增益 | ✅ | ✅ | ❌（实用） | **无需任何界** |
| 7 | 投影 + 状态相关增益 | ✅ | ✅ | ✅ | 需 $\|\Theta_i\| \leq \Theta_{i,\max}$ 和 A1 结构常数 |

**选择建议：**

- **理论最优**（用于论文核心贡献）：**Controller 7**（投影自适应 + 状态相关增益），同时满足三项约束，证明自洽。
- **工程实现最简单**：**Controller 4（DSC）**，控制律不含 $\dot{\phi}_i$，$\varepsilon$ 调小即有良好性能。
- **基线对比**：**Controller 1**（积分 APF，实用收敛）作为性能下界；**Controller 2**（微分 APF，渐近收敛）作为性能上界。

---

## 文件结构

```
fenceEL.tex              # 主文件
body/
├── introduction.tex     # 引言
├── problem.tex          # 问题描述
├── result.tex           # Controller 1: 积分 APF
├── result2.tex          # Controller 2: 微分 APF
├── result3.tex          # Controller 3: 滤波 APF
├── result4.tex          # Controller 4: DSC
├── result5.tex          # Controller 5: 鲁棒自适应 + 滑模（常值增益）
├── result6.tex          # Controller 6: 自适应鲁棒增益
├── result7.tex          # Controller 7: 投影自适应 + 状态相关增益
├── simulation.tex       # 仿真（待实现）
├── conclusion.tex       # 结论
└── reference.bib        # 参考文献
```

---

## 编译说明

- 使用 **pdflatex** 编译（`fontspec`/`xeCJK` 已注释，本项目无中文字符）。
- 编译流程：`pdflatex → bibtex → pdflatex → pdflatex`。
- 参考文献文件：`body/reference.bib`。