# 预设时间 (Prescribed-Time) 收敛的去中心化目标位置估计器
## —— 基于时变增益 (Time-Varying Gain) 的改进

> 本文是对 `1111_observer.md` 中 Remark 6 去中心化估计器 (25) 的改进：
> 把常值增益改为**时变增益 (time-varying gain)**，使估计误差在**用户预设的时刻 $T$** 精确收敛到零，
> 收敛时间与初值、网络代数连通度无关（即预设时间 / prescribed-time 收敛）。
>
> 参考文献：Holloway & Krstic (2019), Gong et al. (2022), Aldana-López et al. (2023), Li et al. (2025), Zheng et al. (2025)。

---

## 1. 背景与思路

原估计器 (25)：

$$
\begin{aligned}
\dot{\varepsilon}_i &= \kappa \Big( \sum_{j \in \mathcal{N}_i} (\varepsilon_j - \varepsilon_i) + g_i(t)(x_0 - \varepsilon_i) \Big) + \rho_i \\
\dot{\rho}_i   &= \gamma \kappa \Big( \sum_{j \in \mathcal{N}_i} (\varepsilon_j - \varepsilon_i) + g_i(t)(x_0 - \varepsilon_i) \Big)
\end{aligned}
$$

其中 $\rho_i$ 是积分项，用于估计目标的常值速度 $\nu_0 = \dot{x}_0$（即 $\rho_i \to \nu_0$）。误差动力学 (26) 是
$\dot y = A_\sigma y$，$A_\sigma$ 为 Hurwitz，因此只得到**指数（渐近）收敛**，收敛到零需要 $t \to \infty$。

预设时间收敛的核心思想（Holloway & Krstic, 2019）：把常值观测器增益换成**随时间发散 (blow-up) 的时变增益**
$\mu(t) = \tfrac{h}{T-t}$（$t \to T^-$ 时 $\mu \to \infty$）。增益的"发散"把无穷时间轴上的指数收敛
**压缩**到有限区间 $[0, T)$ 上，使误差恰好在 $t = T$ 达到零。

关键约束：时变增益只能作用在**本地可测的量**上。这里本地可测量是局部误差

$$
\psi_i(t) = \sum_{j \in \mathcal{N}_i} (\varepsilon_j - \varepsilon_i) + g_i(t)\big(x_0 - \varepsilon_i\big),
$$

它只含邻居估计差与可测目标量，不含未知的 $x_0$ 偏移（见式 (7)）。因此时变增益作用在 $\psi_i$ 上即可实现，
这也是去中心化分布式观测器的标准做法（Gong et al., 2022）。

---

## 2. 预设时间估计器设计

对每个智能体 $i$，用两个**不同幂次**的时变增益替换原式中的 $\kappa$、$\gamma\kappa$：

$$
\boxed{
\begin{aligned}
\dot{\varepsilon}_i &= \rho_i + k_1\,\mu(t)\,\psi_i(t) \\
\dot{\rho}_i      &= k_2\,\mu^2(t)\,\psi_i(t)
\end{aligned}
}
\tag{PT-25}
$$

$$
\psi_i(t) = \sum_{j \in \mathcal{N}_i} (\varepsilon_j - \varepsilon_i) + g_i(t)\big(x_0 - \varepsilon_i\big),
\qquad
\mu(t) = \begin{cases} \dfrac{h}{T-t}, & t \in [0, T), \\[4pt] 0, & t \ge T, \end{cases}
\quad h > 0.
\tag{6}
$$

其中 $T > 0$ 是**用户预设的收敛时刻**，$k_1, k_2$ 是常值设计参数，$h > 0$ 是时变函数的幂次。

**结构说明：**

- $\dot\varepsilon_i$ 中保留 $+\rho_i$ 项（速度估计前馈），因此对匀速运动目标 $x_0(t) = x_0(0) + \nu_0 t$ 仍无稳态误差；
- 增益 $k_1\mu(t)$、$k_2\mu^2(t)$ 是 Holloway–Krstic 观测器标准型中 $\tfrac{1}{T-t}$、$\tfrac{1}{(T-t)^2}$ 幂次结构的推广；
- $t \ge T$ 时 $\mu \equiv 0$，估计器退化为 $\dot\varepsilon_i = \rho_i$、$\dot\rho_i = 0$，见第 5 节。

---

## 3. 误差动力学

设目标匀速运动 $\dot{x}_0 = \nu_0$（常数）、$\dot{\nu}_0 = 0$，定义估计误差

$$
\tilde{\boldsymbol{\varepsilon}} = [\varepsilon_1^T,\dots,\varepsilon_n^T]^T - \mathbf{1}\otimes x_0,
\qquad
\tilde{\boldsymbol{\rho}} = [\rho_1^T,\dots,\rho_n^T]^T - \mathbf{1}\otimes \nu_0 .
$$

由 $\psi_i$ 的定义（各坐标独立），局部误差向量满足

$$
\boldsymbol{\psi} = -M_\sigma \tilde{\boldsymbol{\varepsilon}},
\qquad
M_\sigma = L_\sigma + G_\sigma,
\tag{7}
$$

（与原文 (26) 的推导一致）。于是 (PT-25) 的误差动力学为

$$
\begin{aligned}
\dot{\tilde{\boldsymbol{\varepsilon}}} &= -k_1\mu(t)\, M_\sigma \tilde{\boldsymbol{\varepsilon}} + \tilde{\boldsymbol{\rho}} \\
\dot{\tilde{\boldsymbol{\rho}}}        &= -k_2\mu^2(t)\, M_\sigma \tilde{\boldsymbol{\varepsilon}}
\end{aligned}
\tag{PT-26}
$$

（各分量由 $\otimes I_d$ 解耦，$d$ 为位置维数；原文 $d=2$，下同，省略 $\otimes I_d$。）

---

## 4. 预设时间收敛性证明（时间尺度变换 / Temporal Scaling）

**定理（预设时间收敛）。** 假设网络连通、$M_\sigma \succ 0$，且至少一个智能体能检测目标（原假设）。取

$$
k_1 > \frac{1}{h\,\lambda_{\min}(M_\sigma)}, \qquad k_2 > 0 .
\tag{8}
$$

则估计误差在预设时刻 $T$ 精确收敛到零：

$$
\lim_{t\to T^-} \tilde{\boldsymbol{\varepsilon}}(t) = 0, \qquad
\lim_{t\to T^-} \tilde{\boldsymbol{\rho}}(t) = 0,
$$

且对 $t \ge T$ 恒有 $\tilde{\boldsymbol{\varepsilon}}(t) \equiv 0$、$\tilde{\boldsymbol{\rho}}(t) \equiv 0$。收敛时间 $T$ 与初值和网络参数无关。

**证明。** 引入"拉伸时间" $s$ 与尺度化误差

$$
s = h \ln\frac{T}{T-t} \in [0, \infty),
\qquad
\frac{ds}{dt} = \frac{h}{T-t} = \mu(t),
\tag{9}
$$

$$
\xi = \tilde{\boldsymbol{\varepsilon}}, \qquad \eta = \frac{\tilde{\boldsymbol{\rho}}}{\mu(t)} .
\tag{10}
$$

由 $\dot\mu = \tfrac{h}{(T-t)^2} = \tfrac{\mu^2}{h}$，对 (PT-26) 求导并把 $\tfrac{d}{dt} = \mu \tfrac{d}{ds}$ 代入，得

$$
\begin{aligned}
\frac{d\xi}{ds} &= -k_1 M_\sigma \xi + \eta, \\
\frac{d\eta}{ds} &= -k_2 M_\sigma \xi - \frac{1}{h}\eta,
\end{aligned}
\qquad\Longleftrightarrow\qquad
\frac{d}{ds}\begin{pmatrix}\xi\\ \eta\end{pmatrix}
= \underbrace{\begin{pmatrix} -k_1 M_\sigma & I_n \\ -k_2 M_\sigma & -\frac{1}{h} I_n \end{pmatrix}}_{\bar A_\sigma}
\begin{pmatrix}\xi\\ \eta\end{pmatrix} .
\tag{11}
$$

**关键一步**：时变系统 (PT-26) 在拉伸时间 $s$ 下变成了**时不变 LTI 系统** (11)。$t: 0\to T^-$ 对应 $s: 0\to\infty$，
因此"$t$ 上预设时间收敛"等价于"$s$ 上指数收敛"。

$\bar A_\sigma$ 的 Hurwitz 性：$M_\sigma \succ 0$ 对称，可对角化 $M_\sigma = U\Lambda U^T$，$\Lambda=\operatorname{diag}(\lambda_j)$，$\lambda_j > 0$。
系统在 $U$ 下按 $\lambda_j$ 解耦，每个块

$$
\bar A_j = \begin{pmatrix} -k_1\lambda_j & 1 \\ -k_2\lambda_j & -\frac{1}{h} \end{pmatrix}
$$

的特征多项式为 $s^2 + (k_1\lambda_j + \tfrac1h)s + \big(\tfrac{k_1}{h} + k_2\big)\lambda_j$，当 $k_1,k_2,h>0$ 时两根实部均为负，故 $\bar A_\sigma$ Hurwitz。

进一步要求 $s$ 时间上的衰减率**严格大于 $1/h$**（保证 $\tilde{\boldsymbol{\rho}} = \mu\eta \to 0$）。对 $\bar A_j$ 作平移 $w = s + \tfrac1h$：

$$
w^2 + \Big(k_1\lambda_j - \tfrac1h\Big) w + k_2\lambda_j = 0 .
$$

由 Routh–Hurwitz，两根 $w$ 的实部均为负当且仅当

$$
k_1\lambda_j - \frac1h > 0 \ \Longleftrightarrow\ k_1 \lambda_j > \frac1h,
\qquad k_2\lambda_j > 0 .
$$

对所有 $j$ 成立即式 (8)：$k_1 > \tfrac{1}{h\,\lambda_{\min}(M_\sigma)}$，$k_2 > 0$。此时存在常数 $\lambda > 1/h$、$c>0$ 使

$$
\|\xi(s)\|,\ \|\eta(s)\| \le c\, e^{-\lambda s} \|\xi(0)\| .
$$

**回代到原时间轴。** 由 $e^s = \big(\tfrac{T}{T-t}\big)^h$：

$$
\|\tilde{\boldsymbol{\varepsilon}}(t)\| = \|\xi\| \le c\Big(\tfrac{T-t}{T}\Big)^{\lambda h} \xrightarrow{t\to T^-} 0,
$$

$$
\|\tilde{\boldsymbol{\rho}}(t)\| = \mu\|\eta\| \le c\,\frac{h}{T-t}\Big(\tfrac{T-t}{T}\Big)^{\lambda h}
= c\,\frac{h}{T}\Big(\tfrac{T-t}{T}\Big)^{\lambda h - 1} \xrightarrow{t\to T^-} 0,
\tag{12}
$$

（因 $\lambda h - 1 > 0$）。故两个误差都在 $t = T$ 达到零，且**与初值无关**。$\blacksquare$

**等价 Lyapunov 形式。** 取 $\bar A_\sigma$ 的 Lyapunov 解 $P_s \succ 0$（$\bar A_\sigma^T P_s + P_s\bar A_\sigma = -Q_s$，$Q_s \succ 0$），
令 $V = [\xi^T,\eta^T]P_s[\xi;\eta]$，则 $\tfrac{dV}{ds} \le -\beta_s V$。回到原时间

$$
\dot V = \mu \frac{dV}{ds} \le -\beta_s \mu V = -\frac{\beta_s h}{T-t}\, V
\ \Longrightarrow\
V(t) \le V(0)\Big(\tfrac{T-t}{T}\Big)^{\beta_s h} \xrightarrow{t\to T^-} 0 .
\tag{13}
$$

这与 Gong et al. (2022, Lemma 1) 的判据一致：$\dot V = -\big(c + 2\tfrac{\dot\varsigma}{\varsigma}\big)V$，$\varsigma = \big(\tfrac{T}{T-t}\big)^h$，$\tfrac{\dot\varsigma}{\varsigma} = \tfrac{h}{T-t}$。

---

## 5. $t \ge T$ 的行为与工程实现

**$t \ge T$ 的自洽性。** 在 $t = T$ 时 $\tilde{\boldsymbol{\varepsilon}}(T) = 0$、$\tilde{\boldsymbol{\rho}}(T) = 0$，即
$\varepsilon_i(T) = x_0(T)$、$\rho_i(T) = \nu_0$。$t \ge T$ 时取 $\mu \equiv 0$，估计器退化为

$$
\dot\varepsilon_i = \rho_i = \nu_0, \qquad \dot\rho_i = 0 ,
$$

恰好使 $\varepsilon_i(t) = x_0(t)$（目标匀速），误差恒为零。因此**无需额外切换逻辑**，估计器自然保持零误差。

**增益发散问题（实用要点）。** $\mu(t) = \tfrac{h}{T-t}$ 在 $t \to T^-$ 时发散，实际数值实现与噪声下会放大测量噪声、
产生奇异性。工程上常用三种处理（详见 Orlov (2026) 综述、Li et al. (2025)）：

1. **增益封顶 / 提前冻结**：在 $t = T - \Delta$（$\Delta$ 很小）处把 $\mu$ 冻结为 $\tfrac{h}{\Delta}$，损失可任意小的精度；
2. **有界时变增益 (bounded time-varying gain / TBG)**：构造在 $[0,T]$ 内有界的增益函数（Li et al., 2025; Aldana-López et al., 2023），避免无穷增益与数值奇异；
3. **隐式离散化**：对发散增益闭环做隐式 Euler 离散（Efimov & Orlov, 2026），在保持预设时间收敛的同时抑制噪声。

**切换拓扑。** 若 $M_{\sigma(t)}$ 分段常数且一致连通（$\inf_\sigma \lambda_{\min}(M_\sigma) = \bar\lambda > 0$），
只需把式 (8) 换成 $k_1 > \tfrac{1}{h\bar\lambda}$，结论不变——因为时变增益 $\mu(t)$ 与拓扑无关，(11) 变为分段常值
LTI 系统，可用公共 Lyapunov 函数证明 $s$ 时间上的指数稳定性，从而同样得到预设时间收敛（与原文对切换拓扑的处理一致）。

---

## 6. 与原估计器的对比

| | 原估计器 (25) | 预设时间估计器 (PT-25) |
|---|---|---|
| 增益 | 常值 $\kappa,\gamma\kappa$ | 时变 $k_1\mu(t),\ k_2\mu^2(t)$，$\mu=\tfrac{h}{T-t}$ |
| 收敛 | 指数（$t\to\infty$） | **恰在 $t=T$ 精确到零** |
| 收敛时间 | 依赖初值与 $\lambda_{\min}(M_\sigma)$ | 用户预设 $T$，与初值/网络无关 |
| 目标速度 | 估计 $\nu_0$（$\rho_i\to\nu_0$） | 同左，且 $\rho_i(T)=\nu_0$ 精确 |
| 代价 | —— | 增益在 $t\to T^-$ 发散，需封顶/有界化处理 |

---

## 7. 数值验证

脚本 `simulate_prescribed_time_observer.py`（$N=5$ 路径图，仅智能体 1 能检测目标，目标匀速 $x_0 = 2 + 0.5t$，
$T = 3$，$h = 2$，$k_1 = \tfrac{2}{h\lambda_{\min}(M)} = 12.34$，$k_2 = 1$）：

- 原估计器在 $t = 2.998$ 时最大位置误差仍为 $2.27$（未收敛）；
- 预设时间估计器在 $t = 2.998$ 时最大位置误差 $\approx 5.3\times 10^{-5}$，速度误差 $\approx 5.7\times 10^{-2}$（按式 (12) 以 $(T-t)^{\lambda h-1}$ 速率趋零），误差在 $t\to T$ 时趋零，且与初值无关。

---

## 附注：关于原式 $Q_\sigma$ 的一个小问题

对原式 (26) 的 $A_\sigma$、$P$ 直接计算得

$$
A_\sigma^T P + P A_\sigma =
\begin{pmatrix}
-2\kappa(\gamma_1-\gamma^2)M_\sigma & \gamma_1 I_n - \gamma\kappa(2\gamma-1)M_\sigma \\
\gamma_1 I_n - \gamma\kappa(2\gamma-1)M_\sigma & -2\gamma I_n
\end{pmatrix}\otimes I_2 .
$$

因此原文给出的 $Q_\sigma$ 中副对角块 $-\gamma_1 I_n$ 仅在 $\gamma = \tfrac12$ 时严格成立（此时 $2\gamma-1=0$ 消去
$\gamma\kappa(2\gamma-1)M_\sigma$ 项）。若 $\gamma \ne \tfrac12$，副对角块多出一项，Schur 补条件需相应调整
（取 $\kappa$ 足够大仍可保证 $Q_\sigma \succ 0$）。这是原推导中一个值得注意的小细节，不影响本文第 2–5 节的结论。

---

## 参考文献

- J. Holloway and M. Krstic, "Prescribed-time observers for linear systems in observer canonical form," *IEEE Trans. Autom. Control*, 64(9): 3905–3912, 2019.
- X. Gong, Y. Cui, T. Wang, J. Shen, and T. Huang, "Distributed prescribed-time consensus observer for high-order integrator multi-agent systems on directed graphs," *IEEE Trans. Circuits Syst. II*, 2022.
- R. Aldana-López, R. Seeber, D. Gómez-Gutiérrez, M. T. Angulo, and M. Defoort, "A redesign methodology generating predefined-time differentiators with bounded time-varying gains," *Int. J. Robust Nonlinear Control*, 2023.
- H. Li, X. Jia, S. Duan, and X. Chi, "Accurate prescribed-time output consensus of heterogeneous multi-agent systems: a bounded time-varying gain approach," *Automatica*, 182: 112546, 2025.
- J. Zheng, S. Zhao, and X. Wang, "Prescribed-time target enclosing and tracking with motion and visibility constraints," *IEEE Systems Journal*, 2025.
- D. Efimov and Y. Orlov, "On discretization and sampled-time implementation of prescribed-time stabilizing controls," *Automatica*, 2026.
