# 需求文档：非匀速运动目标的预设时间去中心化观测器
## —— 目标本身或其导数满足有界性条件的情形

> 本文是 `1111_observer.md` 与 `1111_observer_prescribed_time.md` 的进一步推广：
> 把"目标匀速（$\dot x_0$ 为常数）"这一假设放宽为"目标**非匀速**，但 $x_0$ 本身或其（若干阶）导数**有界**"。
>
> 核心参考文献：Gong et al. (2022)（有界未知输入的级联预设时间分布式观测器）、Holloway & Krstic (2019)（时变增益）、
> Aldana-López et al. (2023) / Li et al. (2025)（有界时变增益）。

---

## 1. 问题背景

在围捕/ fencing 场景中，目标 $x_0(t)$ 通常是非合作的，其速度 $\dot x_0(t)$ **不是常数**（存在机动）。此时：

- 原估计器 (25)（常值增益 PI 结构）只能**精确**跟踪匀速目标，目标机动时存在**无法消除的稳态误差**；
- 上一文档的预设时间估计器 (PT-25) 同样假设 $\ddot x_0 \equiv 0$，仅对匀速目标保证 $t = T$ 时**精确到零**。

本需求的目标：在目标**机动（非匀速）**、但**其本身或导数满足有界性条件**的前提下，设计去中心化的**预设时间**观测器，
使每个智能体对 $x_0$（及其若干阶导数）的估计误差在用户预设时刻 $T$ **精确收敛到零**，收敛时间与初值、网络参数无关。

---

## 2. 符号与图模型

沿用原文档记号：$n$ 个智能体，通信图为无向连通图（可切换），Laplacian $L_\sigma$；$g_i(t)\in\{0,1\}$ 为"智能体 $i$ 在 $t$ 时刻能否检测目标"的指示函数，
$G_\sigma = \operatorname{diag}\{g_1,\dots,g_n\}$，$M_\sigma = L_\sigma + G_\sigma$。设

$$
\boldsymbol{1} = [1,\dots,1]^T\in\mathbb R^n,\qquad
\bar\lambda := \inf_{\sigma}\lambda_{\min}(M_\sigma) > 0 .
$$

（$\bar\lambda > 0$ 由"网络连通 + 任一时刻至少一个智能体能检测目标"保证，见原文档。）

局部量测误差。**符号约定**：本文统一采用与 Gong et al. (2022) 相同的约定（注意与 `1111_observer.md` 的 $\psi_i$ **差一个负号**：原文档的 $\psi_i^{\rm old} = -\psi_i^{\rm new}$，即原式 $\sum_j(\varepsilon_j-\varepsilon_i)+g_i(x_0-\varepsilon_i)$ 对应本文的 $-\psi_i$）：

$$
\psi_k^i(t) = \sum_{j\in\mathcal N_i}\big(\hat x_{0,k}^i - \hat x_{0,k}^j\big) + g_i(t)\big(\hat x_{0,k}^i - x_{0,k}\big),
\qquad
\boldsymbol\psi_k = M_\sigma\,\tilde{\boldsymbol x}_{0,k},
\tag{1}
$$

其中 $\hat x_{0,k}^i$ 是智能体 $i$ 对目标第 $k$ 阶状态 $x_{0,k}$ 的估计，$\tilde x_{0,k}^i = \hat x_{0,k}^i - x_{0,k}$。

---

## 3. 目标运动模型与有界性假设

### 3.1 目标模型（积分器链）

设目标位置 $x_0(t)$ 由如下 $n$ 阶积分器链描述（各坐标独立，$\mathbb R^d$ 情形按分量/$\otimes I_d$ 解耦）：

$$
x_{0,1} = x_0, \qquad
\dot x_{0,k} = x_{0,k+1}\ \ (k=1,\dots,n-1), \qquad
\dot x_{0,n} = f_0(t),
\tag{2}
$$

即 $x_0^{(n)}(t) = f_0(t)$，其中 $f_0(t)$ 未知。

### 3.2 有界性假设（核心需求）

**假设 A2（有界性层次，从弱到强）。** 目标满足以下**任一**条件：

| 编号 | 条件 | 等价链阶数 | 可精确估计的量 |
|---|---|---|---|
| A2-0 | 位置有界：$\sup_t\|x_0(t)\| \le \bar x_0$ | ——（无导数信息） | 只能给出误差界，**不能**预设时间精确到零 |
| A2-1 | 速度有界：$\sup_t\|\dot x_0(t)\| \le \bar v_0$ | $n=1$（$f_0 = \dot x_0$） | $x_0$ |
| A2-2 | 加速度有界：$\sup_t\|\ddot x_0(t)\| \le \bar a_0$ | $n=2$（$f_0 = \ddot x_0$） | $x_0,\ \dot x_0$ |
| $\cdots$ | $\cdots$ | $\cdots$ | $\cdots$ |
| A2-$n$ | $n$ 阶导数有界：$\sup_t\|x_0^{(n)}(t)\| \le \bar f_n$ | 阶数 $n$（$f_0 = x_0^{(n)}$） | $x_0,\dot x_0,\dots,x_0^{(n-1)}$ |

**关键结论：** 若只假设 $x_0^{(n)}$ 有界（A2-$n$），则可以用一个 $n$ 阶级联/并行观测器，
在预设时间内**精确**重构 $x_0$ 及其前 $n-1$ 阶导数（$n \ge 1$）；而仅有 A2-0（位置有界）时，
对任意有界速度目标一般**无法**做到预设时间精确收敛（只能保证误差有界）。

**说明：** 原文档的匀速假设对应 $n=2$ 且 $\bar f_2 = 0$（即 $f_0 \equiv 0$）。本需求放宽为 $\bar f_n \ge 0$。

### 3.3 量测假设（需明确选择）

- **M1（位置量测，非合作目标）**：智能体 $i$ 仅能测得目标位置 $x_0 = x_{0,1}$（当 $g_i=1$）。**fencing 场景对应此情形**。
- **M2（全状态量测，合作目标）**：智能体 $i$ 可测得 $x_{0,1},\dots,x_{0,n}$ 全部状态（Gong et al. 2022 的标准假设）。

---

## 4. 观测需求（Requirements）

针对上述模型，观测器需满足：

- **R1（去中心化）**：每个智能体只用自身估计 $\hat x_{0,k}^i$、邻居估计 $\hat x_{0,k}^j$（$j\in\mathcal N_i$）与自身量测 $g_i x_{0,k}$；不使用全局信息。
- **R2（预设时间精确收敛）**：存在用户预设时刻 $T>0$，使 $\forall i,k$：
  $$\hat x_{0,k}^i(t) = x_{0,k}(t),\qquad \forall t \ge T,$$
  且 $T$ 与初值、网络代数连通度无关。
- **R3（非匀速/机动目标）**：在 A2-$n$（$x_0^{(n)}$ 有界、未知）下仍满足 R2，不要求 $\dot x_0$ 为常数。
- **R4（有界性条件利用）**：$f_0$ 的上界 $\bar f_n$ 作为设计参数进入观测器（用于精确抵消未知输入）。
- **R5（切换拓扑与可见性）**：对分段常值切换的连通拓扑 $\sigma(t)$ 与时变可见性 $g_i(t)$ 均成立。
- **R6（导数估计，可选）**：除 $x_0$ 外，还能在预设时间内精确给出 $\dot x_0,\dots,x_0^{(n-1)}$（供控制器 (20)/(27) 的阻尼/前馈项使用）。
- **R7（可实现性）**：时变增益在 $[0,T)$ 内尽可能有界或给出明确的封顶/冻结策略，避免无穷增益与噪声放大。

---

## 5. 提出的算法

### 5.1 二阶情形（A2-2：加速度有界，位置量测 M1）—— 直接推广 PT-25

这是 fencing 场景最常用、且直接承接 `1111_observer_prescribed_time.md` 的版本。对每个智能体 $i$：

$$
\boxed{
\begin{aligned}
\dot{\varepsilon}_i &= \rho_i - k_1\,\mu(t)\,\psi_i(t) \\
\dot{\rho}_i      &= -k_2\,\mu^2(t)\,\psi_i(t) - \sigma\,\operatorname{sign}\!\big(\psi_i(t)\big)
\end{aligned}
}
\tag{PT-25'}
$$

$$
\psi_i(t) = \sum_{j\in\mathcal N_i}(\varepsilon_i - \varepsilon_j) + g_i(t)\big(\varepsilon_i - x_0\big),\qquad
\mu(t)=\frac{h}{T-t}\ (t<T),\ \ \mu(t)=0\ (t\ge T).
\tag{3}
$$

（其中 $\boldsymbol\psi = M_\sigma\tilde\varepsilon$，见式 (1)。）

其中新增的**滑模项** $-\sigma\,\mathrm{sign}(\psi_i)$ 用于**精确抵消未知有界加速度** $\ddot x_0$（要求 $\sigma \ge \bar a_0$）。
它与 PT-25 的唯一区别就是 $\dot\rho_i$ 中多出这一项；当 $\ddot x_0 \equiv 0$（匀速）时取 $\sigma = 0$ 即退化为 PT-25（符号约定差一负号，见 §2）。

**定理（已证明，见 `1111_observer_P2_proof.md`）。** 设 A2-2 成立（$\|\ddot x_0\| \le \bar a_0$），网络连通且至少一个智能体检测目标，取

$$
\sigma \ge \bar a_0,\qquad k_1 > \frac{1}{h\,\lambda_{\min}(M_\sigma)},\qquad k_2 > 0 .
\tag{4}
$$

则 $\varepsilon_i(t) = x_0(t)$、$\rho_i(t) = \dot x_0(t)$ 对 $\forall t \ge T$、$\forall i$ 成立，且与初值无关。

### 5.2 一般 $n$ 阶级联算法（A2-$n$，全状态量测 M2，Gong et al. 2022）

当需要同时估计 $x_0,\dot x_0,\dots,x_0^{(n-1)}$（$n \ge 2$）且目标状态可测（M2）时，采用**级联（cascaded）**结构：
**先估计最高阶 $x_{0,n}$，再逐级向下估计 $x_{0,n-1},\dots,x_{0,1}$**。对智能体 $i$：

$$
\begin{aligned}
\dot{\hat x}_{0,k}^i &= \hat x_{0,k+1}^i - \Big(\alpha_k + \beta_k\,\mu_k(t)\Big)\,\psi_k^i(t),\qquad k=1,\dots,n-1,\\[4pt]
\dot{\hat x}_{0,n}^i &= -\sigma_n\,\operatorname{sign}\!\big(\psi_n^i(t)\big) - \Big(\alpha_n + \beta_n\,\mu_n(t)\Big)\,\psi_n^i(t),
\end{aligned}
\tag{5}
$$

其中 $\psi_k^i$ 如 (1)，各阶增益在**错开的时窗**内激活：

$$
\mu_k(t) = \frac{\dot\varsigma(t_{k-1},\,T_k)}{\varsigma(t_{k-1},\,T_k)},\qquad
\varsigma(t_0,T_k) = \begin{cases}\big(\tfrac{T_k}{t_0+T_k-t}\big)^{h}, & t\in[t_0,t_0+T_k),\\[2pt] 1, & \text{其它},\end{cases}
$$

$T_k>0$ 为估计第 $k$ 阶状态所用的时间，总时间 $T_{\mathrm{obs}} = \sum_{k=1}^{n} T_k$；第 $n$ 阶先估计（$t_0$ 起），完成后第 $n-1$ 阶在其基础上估计，依此类推。
滑模增益 $\sigma_n \ge \bar f_n$ 精确抵消顶层未知输入 $f_0$。

**定理（Gong et al. 2022, Th. 1 的对称图版本）。** 设 A2-$n$ 成立、网络含以目标为根的生成树、$M_\sigma \succ 0$。若

$$
\alpha_k > 0,\qquad
\beta_k \ge \frac{\max_i\{\rho_i\}}{\lambda_{\min}(M(L_0))},\qquad
\sigma_n \ge \bar f_n,
\tag{6}
$$

则 $\hat x_{0,k}^i(t) = x_{0,k}(t)$，$\forall t \ge t_0 + T_{\mathrm{obs}}$，$k=1,\dots,n$，$\forall i$。切换拓扑情形由 Corollary 1（其假设 A3：存在公共对角 $H>0$ 使 $\tfrac12(HL_j + L_j^T H) > 0$）保证。

### 5.3 位置量测（M1）+ 高阶（$n\ge 3$）的混合结构

若仅测位置（M1）但需估计 $x_0,\dot x_0,\ddot x_0$（$n=3$，A2-3：jerk 有界），则 §5.2 的 $\psi_k^i$（$k\ge2$）中的量测项
$g_i(\hat x_{0,k}^i - x_{0,k})$ **不可用**，需改为：

$$
\psi_1^i = \sum_{j\in\mathcal N_i}\big(\hat x_{0,1}^i - \hat x_{0,1}^j\big) + g_i\big(\hat x_{0,1}^i - x_0\big),
\qquad
\psi_k^i = \sum_{j\in\mathcal N_i}\big(\hat x_{0,k}^i - \hat x_{0,k}^j\big),\ k=2,\dots,n,
\tag{7}
$$

即高阶状态的"量测校正"只能通过**邻居一致性 + 级联前馈**传播。这正是**分布式预设时间微分器（distributed prescribed-time differentiator）**问题。
此情形是本需求中**尚未完全解决**的部分，列为待办（§8）。

---

## 6. 收敛性证明思路（Proof Sketch）

**（i）时变增益部分（预设时间）。** 与 `1111_observer_prescribed_time.md` 相同：对时变增益 $\mu,\mu^2$ 作时间尺度变换
$s = h\ln\tfrac{T}{T-t}$（$\tfrac{ds}{dt}=\mu$）并定义 $\eta = \tilde\rho/\mu$，把时变系统化为 $s$ 上的 LTI 系统
$\tfrac{d}{ds}[\xi;\eta] = \bar A[\xi;\eta]$，$\bar A$ 为 Hurwitz（条件 $k_1 > \tfrac{1}{h\lambda_{\min}(M)}$、$k_2>0$）。于是
$t\to T^-$ 对应 $s\to\infty$，误差指数收敛到零，即预设时间收敛。

**（ii）滑模项部分（精确抵消 $f_0$）。** 对 A2-$n$，未知输入 $f_0$（$\|f_0\|\le\bar f_n$）出现在最高阶误差方程中。
取 $\sigma_n \ge \bar f_n$，滑模项 $\sigma_n\,\mathrm{sign}(\psi_n^i)$ 在一致性流形 $\boldsymbol\psi_n = 0$ 上建立滑动模，
逐点支配 $f_0$ 使 $\psi_n^i \to 0$（从而 $\tilde x_{0,n}\to 0$，因 $M_\sigma\succ0$）；随后由级联结构，下层状态以其精确估计
$\hat x_{0,k+1}^i = x_{0,k+1}$ 作前馈，$\dot V_k^b = 0$，逐级把误差压到零。该部分严格证明见 Gong et al. (2022) Th. 1（式 (5)–(8)）。

**（iii）§5.1 的二阶并行结构（PT-25'）** 是 §5.2 在 $n=2$ 的并行化变体：滑模项 $-\sigma\,\mathrm{sign}(\psi_i)$ 与
$\psi_i = M_\sigma\tilde\varepsilon$ 组合成二阶滑动模，在 $t=T$ 前把 $\boldsymbol\psi \to 0$（即 $\tilde\varepsilon\to 0$、$\tilde\rho\to0$）。
**该并行结构的完整严格证明尚需完成**（见 §8 待办 P2），单智能体版本即标准"预设时间超螺旋型微分器"（Holloway & Krstic 2019；Ding et al. 2026），多智能体版本有待把 Gong 的级联证明平移到并行结构。

---

## 7. 特例回退

| 情形 | 假设 | 算法 | 结果 |
|---|---|---|---|
| 匀速目标 | $\dot x_0$ 常数（$n=2,\ \bar f_2=0$） | PT-25（$\sigma=0$） | `1111_observer_prescribed_time.md` |
| 有界加速度 | A2-2（$n=2$） | PT-25'（$\sigma\ge\bar a_0$） | §5.1 |
| 有界 $n$ 阶导数（全状态） | A2-$n$，M2 | 级联 (5) | §5.2（Gong 已证） |
| 有界 $n$ 阶导数（仅位置） | A2-$n$，M1 | 混合结构 (5)+(7) | 待解决 |

---

## 8. 实现要点与待办/开放问题

**实现要点**

1. **增益发散**：$\mu(t)=\tfrac{h}{T-t}$ 在 $t\to T^-$ 发散。工程处理：增益封顶/提前冻结、有界时变增益 TBG（Li et al. 2025）、或隐式离散化（Efimov & Orlov 2026）。
2. **滑模抖振**：$\operatorname{sign}(\cdot)$ 可换成饱和函数/连续化，代价是收敛到"误差界"而非精确零；或改用超螺旋（super-twisting）抑制抖振（Ding et al. 2026）。
3. **$\bar\lambda$ 是全局信息**：增益条件 (4)(6) 用到 $\lambda_{\min}(M_\sigma)$，与"去中心化"（R1）冲突。可研究自适应/全分布式增益（如 Mao et al. 2026 的时变缩放函数方法）来消除该依赖。

**待办 / 待用户确认的决策点**

- **P1**：目标的机动阶数——按 A2-1（仅速度有界）、A2-2（加速度有界）还是 A2-$n$？建议先从 **A2-2（$n=2$）** 入手。
- **P2**：~~§5.1 并行结构（PT-25'）的完整严格证明~~ **已完成**，见 `1111_observer_P2_proof.md`（时间尺度变换 + 滑模等价控制，含切换拓扑推论）。
- **P3**：量测假设确认为 **M1（仅位置）** 还是 M2（全状态）？fencing 场景应为 M1。
- **P4**：是否要求估计速度 $\dot x_0$（R6）？控制器 (20)/(27) 若用目标速度前馈则需要。
- **P5**：滑模（精确、有抖振）与连续平滑（近似、无抖振）之间如何取舍。

---

## 9. 参考文献

- X. Gong, Y. Cui, T. Wang, J. Shen, and T. Huang, "Distributed prescribed-time consensus observer for high-order integrator multi-agent systems on directed graphs," *IEEE Trans. Circuits Syst. II*, 2022.（有界未知输入的级联 DPTO，本文 §5.2 直接来源）
- J. Holloway and M. Krstic, "Prescribed-time observers for linear systems in observer canonical form," *IEEE Trans. Autom. Control*, 64(9): 3905–3912, 2019.（时变增益/积分器链）
- R. Aldana-López, R. Seeber, D. Gómez-Gutiérrez, M. T. Angulo, and M. Defoort, "A redesign methodology generating predefined-time differentiators with bounded time-varying gains," *Int. J. Robust Nonlinear Control*, 2023.
- H. Li, X. Jia, S. Duan, and X. Chi, "Accurate prescribed-time output consensus of heterogeneous multi-agent systems: a bounded time-varying gain approach," *Automatica*, 182: 112546, 2025.（TBG 有界增益）
- Y. Ding, B. Zhou, and Y. Shi, "Prescribed-time control via periodic delayed feedback-based super-twisting sliding mode algorithm," *IEEE Trans. Autom. Control*, 2026.（连续滑模、无无穷增益、抑制抖振）
- N. Mao, S. Liu, and Y. Yuan, "Prescribed-time fully distributed optimization for time-varying costs: zero-gradient-sum scheme," *ISA Transactions*, 2026.（消除对 Laplacian 特征值依赖的时变缩放函数）
- D. Efimov and Y. Orlov, "On discretization and sampled-time implementation of prescribed-time stabilizing controls," *Automatica*, 2026.
