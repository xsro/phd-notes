# leader 系统矩阵不可得时的估计器：五条路线与公式对照

> 整理日期：2026-10-18
> 语料来源：本地知识库（OKB）文献库，doc id 见各节标题与末节参考文献
> 关键词：自适应分布式观测器、仅输出观测、有限时间/固定时间、初始激励、数据驱动、协同输出调节

---

## 0. 问题设定与统一记号

leader（外系统）+ 异构 follower：

$$
\begin{cases}
\dot v = S v, & y_0 = -F v \\[2pt]
\dot x_i = A_i x_i + B_i\big(f_i(x_i) + u_i\big),\quad i=1,\dots,N \\
y_i = C_i x_i
\end{cases}
$$

追踪目标（TVFT / COR）：$e_i = y_i - y_0 - h_i(t) \to 0$，其中 $h_i(t)$ 为期望时变编队。

**核心矛盾**：输出调节的求解依赖调节器方程

$$
\begin{cases}
X_i S = A_i X_i + B_i U_i\\
0 = C_i X_i - F\\
0 = B_i R_i - X_i E
\end{cases}
$$

上式中 $S$（即文献 [1848] 的 $D$）、$E$、$F$ 只有少数 follower 掌握，甚至谁都不掌握。于是问题分解为两个子问题：

| 子问题 | 要估什么 | 典型工具 |
|---|---|---|
| **P1：leader 侧** | $v$（状态）、$S$、$F$、支配输入界 $d$ | 分布式观测器 / 自适应律 / 数据滤波 |
| **P2：follower 侧** | $X_i, U_i, R_i$（调节器方程解） | 自适应梯度（在线解线性方程组）/ 内模 / 数据 |
| **P3（可选）** | 连 $A_i,B_i,C_i$ 都不知 | 数据驱动（Willems）、RL/ADP |

本文只整理 **P1 + P2 在“leader 矩阵不可得”前提下的五类算法**，并给出空白点分析。

---

## 1. 基准：自适应分布式观测器族（积分型自适应 + 邻接一致性）

### 1.1 原始结果（doc 3343，Cai–Lewis–Hu–Huang, Automatica 2017）

leader $\dot v = S v$ 中 $S$ **对 follower 未知**，通过邻接一致性把它当未知常值估计：

$$
\dot S_i(t) = \mu \sum_{j=0}^{N} a_{ij}(t)\big( S_j(t) - S_i(t)\big),\qquad S_0 = S
$$

$$
\dot v_i(t) = S_i(t) v_i(t) + \mu' \sum_{j=0}^{N} a_{ij}(t)\big( v_j(t) - v_i(t)\big)
$$

- 前提：**至少一个 follower 知道 $S$**（$a_{i0}>0$ 的边上传递 $S$ 与 $v$）。
- 收敛：指数；$S$ 需临界稳定（或已知虚轴特征值），后续才推广到一般线性 leader。
- 调节器方程解 $X_i,U_i$ 同样用自适应律在线逼近（这就是 [1848] 中 Lemma 2 的前身，见 §2.4）。

### 1.2 未知 leader 的输出包含（doc 1300，Zuo–Song–Lewis–Davoudi, Automatica 2018）

多 leader、动态只有邻居跟随者知道，每个 follower 配局部自适应观测器估 leader 动态，再配 **自适应调参律在线解输出调节器方程（不求 $X_i,U_i$ 的显式解）**，给出 state-feedback 与 dynamic output-feedback 两种协议。

### 1.3 有限时间版（doc 1848，Wang–Hua–Dong–Shu–Lü–Ren, TCYB 2023）—— 本案基准

distributed finite-time observer：

$$
\dot \eta_i = \bar D_i(t)\eta_i - d_i(t)\bar E_i(t)\,\mathrm{sign}\!\big(\bar E_i^{\top}(t)\xi_i\big) - \gamma_i \xi_i - \mathrm{sign}(\xi_i)|\xi_i|^{\beta}
$$

$$
\dot D_i = -\kappa_1 \mathrm{sign}(\varsigma_{1i})|\varsigma_{1i}|^{\beta_1},\quad
\dot E_i = -\kappa_2 \mathrm{sign}(\varsigma_{2i})|\varsigma_{2i}|^{\beta_2},
$$
$$
\dot F_i = -\kappa_3 \mathrm{sign}(\varsigma_{3i})|\varsigma_{3i}|^{\beta_3},\quad
\dot d_i = -\kappa_4 \mathrm{sign}(\varsigma_{4i})|\varsigma_{4i}|^{\beta_4}
$$

$$
\xi_i = w_{i0}\big(\eta_i - x_0\big) + \sum_{j} w_{ij}\big(\eta_i - \eta_j\big),\qquad
\varsigma_{1i} = w_{i0}(D_i - D_0) + \sum_j w_{ij}(D_i - D_j)
$$

其中 $\bar D_i = \mathrm{vec}^{-1}(D_i)$，$0<\beta<1$。误差系统

$$
\dot{\tilde D} = -\kappa_1 \mathrm{sign}\big((L_2\otimes I_{n^2})\tilde D\big)\,\big|(L_2\otimes I_{n^2})\tilde D\big|^{\beta_1},
\qquad
V_1 = \tfrac12 \tilde D^{\top}(L_2\otimes I_{n^2})\tilde D
$$
$$
\dot V_1 \le -k_5 V_1^{(\beta_1+1)/2}
\;\Rightarrow\;
T_D \le \frac{2}{k_5(1-\beta_1)},\quad
T_1 = \max\{T_D,T_E,T_F,T_d\}
$$

- 优点：**不需要所有 follower 知道 $D,E,F$**；对未知输入用 $-\,d_i\bar E_i\,\mathrm{sign}(\bar E_i^{\top}\xi_i)$ 抑制。
- 缺点：**每个 follower 都要跑 4 个 $n^2$ 维积分器**；$D,E,F,d$ 的收敛依赖“$\varsigma$ 的符号一致 + cooperative 可辨识”，本质是自适应型收敛；有限时间结论靠对 Lyapunov 函数强行套幂次（引 Tang 1998 的终端滑模判据），而不是真正常规意义下的有限时间可辨识。

### 1.4 调节器方程解的自适应有限时间求解（doc 1848, Lemma 2）

把 (23)(24) 写成线性方程组 $\Pi_i\theta_i = b_i$，$\theta_i = \mathrm{vec}([X_i;U_i;R_i])$：

$$
\dot{\hat\theta}_i = -\kappa \hat\Pi_i^{\top}(t)\,\mathrm{sig}^{\phi}\!\big(\hat\Pi_i(t)\hat\theta_i(t) - \hat b_i(t)\big),\qquad 0<\phi<1
$$

$$
\hat\Pi_i(t) = \bar D_i^{\top}(t)\otimes \begin{bmatrix} I_{n_i}&0&0\\0&0&0\\0&0&0\end{bmatrix}
- I_n \otimes \begin{bmatrix} A_i & B_i & 0\\ C_i & 0 & 0\\ 0&0& B_i\end{bmatrix}
$$

来源：Cai et al. [50, Lemma 8] 的指数收敛自适应律 + $\mathrm{sig}^{\phi}$ 提升为有限时间。

---

## 2. 路线 A：有限时间 + leader 矩阵对所有 follower 未知（doc 2832）

> Q. Wang, X. Dong, Z. Chen, Z. Lian, J. Lü, *Finite-time formation tracking for heterogeneous Euler–Lagrange systems with an uncertain leader*, Automatica 2025.
> 与 [1848] 同一作者线的直接后续，**目前最接近“无 leader 矩阵 + 有限时间”的公开结果**。

### 2.1 设定

leader：$\dot q_0 = E q_0$，$q_0$ 有界；**$E$ 对任何 follower 都不可得**。

$$
\text{Assumption 2.1: } E \text{ 反对称，特征值全为单重纯虚数（多正弦外系统）}
$$

follower：Euler–Lagrange 系统（$M_i,\ C_i,\ G_i$ 已知结构，参数化性质 3）。只有组 $\Pi_1=\{1,\dots,m\}$ 能测到 $q_0$；其余为 $\Pi_2=\{m+1,\dots,N\}$。

### 2.2 关键技巧 1：参数化 + 稳定化滤波（concurrent-learning 风格）

$$
\theta = \mathrm{vec}(E^{\top})\in\mathbb R^{n^2},\qquad
\bar S(t) = I_n \otimes q_0^{\top}(t) \;\Rightarrow\; \dot q_0 = \bar S(t)\theta
$$

$$
\dot N(t) = -k N(t) + \bar S(t),\quad N(0)=0;\qquad
\dot g(t) = -k g(t) + \dot q_0(t),\quad g(0)=0
\;\Rightarrow\; g = N\theta
$$

**消掉 $\dot q_0$ 的分部积分技巧（只需 leader 状态/输出本身）**：

$$
g(t) = q_0(t) - e^{-kt}q_0(0) - k f(t),\qquad \dot f = -kf + q_0,\quad f(0)=0
$$

$$
\dot{\hat M} = N^{\top}N,\quad \dot G = N^{\top}g
\;\Rightarrow\; G(t) = \hat M(t)\theta,\qquad \hat M(t)=\int_0^t N^{\top}(\tau)N(\tau)d\tau \succeq 0
$$

### 2.3 关键技巧 2：有限时间学习律 + 分布式状态观测器

$$
\xi_i(t) = q_0(t),\quad i\in\Pi_1
$$

$$
\dot\theta_i = -\hat M(t)\,\mathrm{sig}^{\beta}\!\big(\hat M(t)\theta_i - G(t)\big),\qquad i\in\Pi_1
$$

$$
\dot\xi_i = \hat\Xi_i(t)\theta_i - \mu_1\eta_i - \mathrm{sig}^{\beta}(\eta_i),\qquad i\in\Pi_2
$$
$$
\dot\theta_i = -\hat\Xi_i^{\top}(t)\eta_i - \mathrm{sig}^{\beta}(\hat\delta_i(t)),\qquad i\in\Pi_2
$$

$$
\hat\Xi_i = I_n\otimes\xi_i^{\top},\qquad
\eta_i = \sum_j a_{ij}(\xi_i - \xi_j),\qquad
\hat\delta_i = \sum_j a_{ij}(\theta_i - \theta_j)
$$

误差系统：

$$
\dot{\tilde\zeta}_1 = -\mathbf M(t)\,\mathrm{sig}^{\beta}\big(\mathbf M(t)\tilde\zeta_1\big),\qquad \mathbf M = I_m\otimes\hat M
$$
$$
\dot{\tilde\xi} = \big(I_{N-m}\otimes E - \mu_1 \mathcal H_2\otimes I_n\big)\tilde\xi - \mathrm{sig}^{\beta}(\eta)
$$

### 2.4 收敛条件：**初始激励**（弱于 PE / cPE / FTE）

$$
\exists\, t_0>0,\ d_1>0:\qquad
\int_0^{t_0} N^{\top}(t)N(t)\,dt \succeq d_1 I_{n^2}
$$

即只要求**前 $t_0$ 时间内的数据足够丰富**，之后无需持续激励。

**Theorem 3.1**：$\theta_i(t)$ 有限时间收敛到 $\theta$；$\xi_i(t)$（$i\in\Pi_2$）有限时间收敛到 $q_0(t)$。
**Remark 2**：控制器不需要 leader 的系统矩阵、输出矩阵、状态导数信息，也不需要全局图信息。

### 2.5 与 [1848] 的差别（可直接对照）

| | [1848] TCYB'23 | [2832] Auto'25 |
|---|---|---|
| leader 矩阵未知范围 | 某邻居知道 $D,E,F$ | **谁都不知道 $E$** |
| 参数更新方式 | 积分型自适应 $\dot D_i=-\kappa_1\mathrm{sign}(\varsigma)|\varsigma|^{\beta_1}$ | **数据滤波 $\hat M, G$ + 幂次梯度** |
| 收敛条件 | cooperative 可辨识条件 | **初始激励**（更弱） |
| 需要 leader 状态？ | 是（$w_{i0}=1$ 的 follower 测 $x_0$） | 是（$\Pi_1$ 测 $q_0$） |
| 结论强度 | 有限时间（Lyapunov 幂次） | 有限时间（参数真值 + 状态） |

---

## 3. 路线 B：仅用 leader 输出的自适应分布式观测器（doc 3041）

> C. He, J. Huang, *Output-based adaptive distributed observer for general linear leader systems over periodic switching digraphs*, Autonomous Intelligent Systems, 2023.

### 3.1 设定

$$
\dot\nu = S\nu,\qquad y = C\nu
$$

- $S$ 为**一般**线性矩阵（不要求临界稳定）；有向周期切换图。
- **leader 状态 $\nu$ 对任何 follower 都不可得，只能拿到 $y$。**

### 3.2 观测器（关键：外部信息只有“估计的输出”）

$$
\dot S_i = \mu_1 \sum_{j=0}^{N} a_{ij}(t)(S_j - S_i)
$$
$$
\dot C_i = \mu_2 \sum_{j=0}^{N} a_{ij}(t)(C_j - C_i)
$$
$$
\dot L_i = \mu_3 \sum_{j=0}^{N} a_{ij}(t)(L_j - L_i)
$$
$$
\dot\xi_i = S_i\xi_i + \mu_4 L_i \sum_{j=0}^{N} a_{ij}(t)\big(C_j\xi_j - C_i\xi_i\big)
$$

其中 $S_0=S,\ C_0=C,\ L_0=L=PC^{\top},\ \xi_0=\nu$。$P$ 由代数 Riccati 方程

$$
SP + PS^{\top} - PC^{\top}CP + I_q = 0
$$

唯一正定解给出。**注意 $P$（从而 $L$）follower 也不知道，所以 $L_i$ 也要分布式地估**——这是该文相对早期“$L=PC^{\top}$ 由 node 0 统一发”的关键推进。

### 3.3 ★ 还需要 leader 的系统矩阵吗？—— 需要，但只在“局部”需要

式 (10) 里写得很明确：

$$
S_0 = S,\qquad C_0 = C,\qquad L_0 = L,\qquad \xi_0 = \nu
$$

即 **node 0（leader 侧）把 $S$, $C$, $L$ 广播给它自己的出邻居**；只有满足 $a_{i0}(t)>0$（能收到 leader 信息）的 follower 才真正拿到 $S$，其余 follower 靠一致性把 $S$ 沿网络搬运过去：

$$
\dot S_i = \mu_1\sum_{j=0}^{N}a_{ij}(t)(S_j - S_i)
\;\Longrightarrow\;
S_i(t)\to S
$$

所以结论要分三层说清：

| 层次 | 是否需要 leader 的 $S$ |
|---|---|
| **leader 的出邻居** | **需要**，直接收到 $S$（且要能收到） |
| **普通 follower** | **不需要**，$S_i(0)$ 可任意，靠一致性估计得到 |
| **$L=PC^{\top}$** | **谁都不需要**，由 (10c) 分布式估出 |

- 相对 **doc 3343** 的推进：3343 要求**所有** follower 都知道 $S$；3041 把这条降成“leader 的邻居知道”，同时把“leader 状态可得”降成“只用 $y$”。
- **实质仍是“某个节点持有 $S$ 并向外扩散”**，物理上等价于“leader 愿意公开自己的系统矩阵”，对被跟踪目标是非合作方（如逃逸目标、非己方飞行器）的场景并不成立。
- 若真要**谁都不知道 leader 矩阵**：看 **doc 2832**（$E$ 全员未知，靠数据滤波 + 初始激励有限时间辨识）或 **doc 3743**（只给 $y$，$f$ 用参数化/NN 学）。

### 3.4 条件与结论

$$
\text{A1：联合连通（每个区间存在从 node 0 到所有点的路径）};\qquad
\text{A2：切换信号周期 } T;\qquad
\text{A3：}(C,S)\text{ 可检测}
$$

$$
\mu_2,\ \mu_3 > \bar\mu_2 = \frac{\lambda}{\mathrm{Re}\,\bar\lambda_{\min}\big(\sum_{p}\omega_pH_p\big)},\qquad
\mu_4 \ge \bar\mu_3 = \frac{1}{2\,\mathrm{Re}\,\bar\lambda_{\min}\big(\sum_p \omega_p H_p\big)},\qquad
0<T<\bar T_2
$$

结论（Theorem 3.1，全部**指数**收敛）：

$$
S_i\to S,\qquad C_i\to C,\qquad L_i\to L,\qquad \xi_i\to \nu
$$

- 贡献：去掉“所有 follower 知道 $S$”与“leader 状态可得”两个假设，且从“临界稳定 + 无向图”放宽到“一般线性 + 有向周期切换图”。
- 局限：**指数而非有限时间**；仍需某邻居知道 $(S,C)$。

---

## 4. 路线 C：仅输出 + 非线性 leader + 神经网络（doc 3743）

> J. Lv, C. Wang, L. Xie, *Adaptive distributed observer design for nonlinear multiagent systems*, Automatica 2025.

### 4.1 问题转换（可观测标准型）

非线性自治 leader，经 $n$-微分可观的可观测映射 $x = O_n(z)$ 转成标准型：

$$
\dot x_1 = x_2,\ \dots,\ \dot x_{n-1} = x_n,\qquad
\dot x_n = f(x) = W^{\top}\phi(x)
$$

其中 $W$ 为未知常向量，$\phi(\cdot)$ 已知基函数（不必可微）。外部只有 $y$，即 $x_1$。

### 4.2 参数化情形的自适应 DO

$$
e_{1i} = \sum_{j=0}^{N} a_{ij}\big(\hat x_{1j} - \hat x_{1i}\big),\qquad \hat x_{10} = x_1
$$

$$
\dot{\hat x}_i = A\hat x_i + K e_{1i} + B\hat W_i^{\top}\hat\phi_i
$$
$$
\dot{\hat W}_i = \gamma\, e_{1i}\,\hat\phi_i,\qquad \hat\phi_i = \phi(\hat x_i)
$$

$$
K = \Big[\tfrac{l_1}{\tau},\ \tfrac{l_2}{\tau^2},\ \dots,\ \tfrac{l_n}{\tau^n}\Big]^{\top},\qquad \tau>0,\ \gamma>0
$$

误差（含线性化余项 $R_j$，$\tilde\phi_j = R_j\varepsilon_j$，$\|R_j\|\le r_m$）：

$$
\dot{\bar e} = \tau^{-1}\big(I_N\otimes A - L\otimes GC\big)\bar e
+ \tau^{n-1}\big[L\otimes(1_m^{\top}\otimes B)\big]\Phi\tilde W
+ \big(L\otimes\bar\tau\bar BF_W\big)R\big(L^{-1}\otimes\bar\tau^{-1}\big)\bar e
$$

$$
\bar\tau = \mathrm{diag}\{1,\tau,\dots,\tau^{n-1}\},\qquad G = \delta P^{-1}C^{\top}
$$

**Lemma 4**：若

$$
\alpha_1 I_{nN} \le \frac{1}{T_1}\int_{t_0}^{t_0+T_1}\phi_f(\sigma)\phi_f^{\top}(\sigma)\,d\sigma \le \alpha_2 I_{nN}
\quad(\text{PE 条件})
$$

则 $\varepsilon\to 0,\ \tilde W_i\to 0$（渐近）。

### 4.3 非参数情形的 NN DO

$$
f(x) = W^{\top}\phi(x) + \nu,\qquad
\phi_i(x) = \exp\!\Big[-\frac{(x-\rho_i)^{\top}(x-\rho_i)}{\mu_i^2}\Big]
$$

$$
\dot{\hat x}_i = A\hat x_i + K e_{1i} + B\hat W_i^{\top}\hat\phi_i
$$
$$
\dot{\hat W}_i = \mathrm{Pr}\big(\gamma e_{1i}\hat\phi_i\big)
$$

**Theorem 2**：存在 $\tau, G, T^{*}$ 使 $\|\varepsilon\|\le \varepsilon^{*}\ \forall t\ge T^{*}$（实用一致最终有界，非精确收敛）。

- 贡献：**真正只依赖 leader 输出**；follower 间只交换“估计的输出” $\hat x_{1i}$；follower 只需 $f$ 连续（去掉 DESO 的可微要求）。
- 局限：需要可观测标准型；参数化要 PE，NN 只给 UUB；**无有限时间结论**。

---

## 5. 路线 D：数据驱动协同输出调节（doc 3622）

> Y. Li, W. Liu, G. Wang, L. Xie, J. Sun, J. Chen, *Data-Driven Cooperative Output Regulation via Output Feedback Control*, 2026.

### 5.1 设定（离散、线性、全未知）

$$
x_i^{+} = A_i x_i + B_i u_i + E_i v,\qquad y_i = C_i x_i
$$
$$
v^{+} = Sv,\qquad y_0 = -Fv,\qquad e_i = C_i x_i + Fv
$$

$A_i,B_i,C_i,E_i,F$ **全部未知**。仅 $S$ 的**极小多项式**用于内模（Definition 1）。

### 5.2 历史 I/O 替代不可测状态

$$
h_i(t) = \mathrm{col}\big(y_i(t-\ell_i),\dots,y_i(t-1),\ u_i(t-\ell_i),\dots,u_i(t-1)\big)
$$

$$
h_i^{+} = \mathbf A_i h_i + \mathbf B_i u_i + \mathbf L_i\Gamma_{1i}\mathbf v_i,\qquad
y_i = \Pi_i h_i + \Gamma_{1i}\mathbf v_i
$$

### 5.3 内模 + 分布式输出反馈控制器

$$
u_i = K_{hi}h_i + K_{zi}z_i,\qquad
z_i^{+} = Mz_i + \Phi e_{vi},\qquad
e_{vi} = \sum_j a_{ij}(y_i-y_j) + a_{i0}(y_i - y_0)
$$

$(M,\Phi)$ 为 $S$ 的 $p$-copy 内模：$M=\mathrm{blkdiag}(\beta,\dots,\beta)$，$\Phi=\mathrm{blkdiag}(\sigma,\dots,\sigma)$，$\beta$ 的特征多项式 $=$ $S$ 的极小多项式，$(\beta,\sigma)$ 可控。

增广系统 $\xi=\mathrm{col}(h,z)$：

$$
\xi^{+} = \mathbf A_\xi \xi + \mathbf B_\xi u + \mathbf E_\xi\mathbf v,\qquad
e_v = (H\otimes I_p)(\Pi_\xi \xi + \Gamma_2 \mathbf v)
$$

### 5.4 一次离线数据 → agent-wise SDP

采集 $\{u_i,y_i,v,z_i^{m}\}$，构造 $U_{i+},\ \Xi_{i-},\ \Xi_{i+},\ V_i$（式 (18)）。求解

$$
P_i \succ 0,\qquad
\begin{bmatrix} P_i & \Xi_{i+}Y_i\\ (\Xi_{i+}Y_i)^{\top} & P_i\end{bmatrix} \succ 0,\qquad
\begin{bmatrix} P_i\\0\end{bmatrix} = \begin{bmatrix}\Xi_{i-}\\ V_i\end{bmatrix} Y_i
$$

则

$$
K_{\xi i} = U_{i+}Y_i P_i^{-1}
$$

- **不辨识系统、不解输出调节器方程、不构造任何观测器**（Remark 3）。
- 数据条件：$D_i = [U_{i+};\Xi_{i-};V_i]$ 满行秩（数据丰富性，替代 Willems 的 PE）。
- 结论：Schur 稳定 + 精确追踪，但**仅渐近**（离散、线性）。

> 注意：OKB doc 3621 的 markdown 与标题不符（实际内容是 MIMO 正弦扰动抑制一文），引用请用 **3622**。

---

## 6. 路线 E：RL / ADP 数据驱动（leader 模型信息不进控制律）

**doc 3643**（Cheng–Liu–Wen–Lü–Lewis, Automatica 2023，异构空地车 TVFCC，active leader）：

- 切换拓扑下的分布式观测器：$\hat\varsigma_{qi} = [(\hat p_{qi})^{\top},(\dot{\hat p}_{qi})^{\top}]^{\top}$，局部误差
  $$
  \bar\delta_{qi}^{\varrho(t)} = \rho_{q,i}^{\varrho(t)}\big(z_{p0} - \hat\varsigma_{qi} + \bar\sigma_i\big)
  + \sum_{j} w_{ij}^{\varrho(t)}\big(\hat\varsigma_{qj} - \hat\varsigma_{qi} + \bar\sigma_{ij}\big)
  $$
  其中 $\rho_{q,i}^{\varrho(t)}>0$ 表示四旋翼 $i$ 能看到虚拟 leader，$\bar\sigma_i=[\sigma_i^{\top},0_{3\times1}^{\top}]^{\top}$ 为待估的编队偏差。
- 控制律用 RL 求最优（**不需要精确模型参数**），是“数据驱动 + TVFT”的代表。

**doc 3817 / 3818**：事件触发 + 零和博弈/off-policy RL；[3818] 用 **predefined-time 分布式观测器**给 leader 状态估计，再用纯数据驱动 off-policy 学习最优编队控制器（避开 Kronecker 积）。

**要点**：这类方法把“未知”放在 **follower 侧动力学**，leader 侧仍由观测器（多半需要 leader 模型或直接测 leader 状态）处理；且收敛多为渐近/实用。

---

## 7. 路线 F：完全不给 leader 建模（把 $y_0$ 当信号）

用高阶滑模**鲁棒精确微分器**（Levant）在线取 $y_0$ 及其各阶导数，再配 terminal sliding mode 项抑制未知输入。三阶形式：

$$
\dot z_0 = z_1 - \lambda_0 |z_0 - y_0|^{3/4}\mathrm{sign}(z_0-y_0)
$$
$$
\dot z_1 = z_2 - \lambda_1 |z_1 - \dot z_0|^{1/2}\mathrm{sign}(z_1-\dot z_0)
$$
$$
\dot z_2 = -\lambda_2 \mathrm{sign}(z_2 - \dot z_1)
$$

- 优点：**有限时间**天然成立（与 §1.3 的 $\mathrm{sign}(\cdot)|\cdot|^{\beta}$ 同源）；不需要任何 leader 矩阵。
- 缺点：无法复现内模结构，一般只能得到实用/包含性结论；导数噪声敏感。

---

## 8. 横向对比

| 文献 | leader 哪些量未知 | 需要谁掌握什么 | 额外条件 | 收敛 | 解调节器方程 |
|---|---|---|---|---|---|
| **[1848]** TCYB'23 | $D,E,F,d$（某邻居知） | 邻接 follower 知 $D,E,F$ | cooperative 可辨识 | **有限时间** | 是（$\mathrm{sig}^{\phi}$ 自适应） |
| **[3343]** Auto'17 | $S$ | 某邻居知 $S$ | 临界稳定 | 指数 | 自适应律 |
| **[1300]** Auto'18 | 多 leader 动态 | 邻居知 leader 动态 | — | 渐近 | 自适应调参 |
| **[2832]** Auto'25 | $E$（**全员未知**） | $\Pi_1$ 测 leader **状态** | **初始激励** | **有限时间**（参数+状态） | 否（直接学 $E$） |
| **[3041]** 2023 | 仅 $y$（$\nu$ 不可得） | 某邻居知 $(S,C)$ | $(C,S)$ 可检测 + 周期切换 + 联合连通 | 指数 | 否（在观测层） |
| **[3743]** Auto'25 | 仅 $y$ + $f$ 未知 | 至少一个 follower 连 leader 输出 | 可观测标准型 + PE / NN 近似 | 渐近 / UUB | 否 |
| **[3622]** 2026 | $A_i,B_i,C_i,E_i,F$ 全未知 | 需 $S$ 的极小多项式 | 数据满行秩（SDP 可行） | 渐近（离散线性） | **完全绕过** |
| **[3643]/[3817]/[3818]** | follower 参数未知 | 观测器给 reference | — | 渐近/最优 | 否 |
| **[F]** 微分器 | 全部（不建模） | 仅 $y_0$ | 无 | 有限时间（实用） | 否 |

---

## 9. 三个判断与空白点

1. **“有限时间 + leader 矩阵对所有人未知”已有**：doc 2832。代价是 **$\Pi_1$ 必须直接测 leader 状态**，并需要**初始激励**。
2. **“仅 leader 输出 + 矩阵未知”已有**：doc 3041（线性）、doc 3743（非线性/NN）。但**只有指数/实用收敛**。
3. ⇒ **空白点（可做）**：**“仅用 leader 输出的有限时间观测器 + leader 矩阵对所有 follower 未知”**。
4. **第二个空白点**：**数据驱动（SDP/基本引理型）× 有限时间**。障碍在于 $\mathrm{sig}^{\beta}$ 是非多项式/非凸项，会破坏 LMI/SDP 的数据表示；目前 [3622] 类方法只能是 Schur 稳定（渐近）。

### 为什么“有限时间 + 无激励”本质上难

自适应/学习型参数辨识依赖“激励 $\Leftrightarrow$ 回归量矩阵可逆”：

$$
\hat M(t) = \int_0^t N^{\top}(\tau)N(\tau)d\tau \succeq d_1 I
\ \Longleftrightarrow\ \theta \text{ 可唯一辨识}
$$

无激励时 $\theta$ 不可辨识，只能：
- (i) 放宽到**初始激励**（doc 2832 的做法）；
- (ii) **放弃参数收敛**，只保证**跟踪误差有限时间收敛**（doc 1848 的 $\mathrm{sign}$ 项 + 微分器路线）；
- (iii) 用**集合成员/区间估计**只保证 $\theta$ 属于不变集（尚无人做 COR）。

---

## 10. 融合方案草案：仅输出 + 有限时间 + 无 leader 矩阵

思路：**doc 3743 的可观测映射**（把 $y_0$ 提升为伪状态）+ **doc 2832 的滤波–$\mathrm{sig}^{\beta}$ 学习**（只用 $q_0$ 不用 $\dot q_0$）。

**Step 1（结构）** 对 leader 取可观测映射 $x = O_n(z)$，得标准型

$$
\dot x_1 = x_2,\ \dots,\ \dot x_n = W^{\top}\phi(x) = \bar S(x)\,W,\qquad y_0 = x_1
$$

**Step 2（数据滤波，仅用 $x_1$ 与 $\hat x$）**

$$
\dot N = -kN + \bar S(\hat x),\quad
\dot f = -kf + x_1,\quad
g = x_1 - e^{-kt}x_1(0) - kf,\quad
\dot{\hat M} = N^{\top}N,\quad \dot G = N^{\top}g
$$

**Step 3（有限时间参数学习 + 状态观测）**

$$
\dot\theta_i = -\hat M\,\mathrm{sig}^{\beta}\big(\hat M\theta_i - G\big)
$$
$$
\dot{\hat x}_i = A\hat x_i + K e_{1i} + B\,\theta_i^{\top}\phi(\hat x_i) - \mu_1\eta_i - \mathrm{sig}^{\beta}(\eta_i)
$$
$$
e_{1i} = \sum_j a_{ij}(\hat x_{1j} - \hat x_{1i}) + a_{i0}(\hat x_{1i} - y_0)
$$

**Step 4（控制器）** 沿用 [1848] 的坐标变换 + $\zeta_i$ 技巧，把 $X_i,U_i,R_i$ 用 $\theta_i$ 的在线解代入；用 $\mathrm{sign}$ 项补偿残余未知输入。

**风险/要处理的技术点**

1. $\phi(\hat x_i)$ 依赖估计值 → 产生 doc 3743 的 $\psi_1+\psi_2$ 耦合项，在 $\mathrm{sig}^{\beta}$ 下需要重建齐次性（可用 $\mathrm{sig}^{\beta}(a+b)$ 的界 + Hölder 不等式）。
2. 可观测映射 $O_n$ 通常含参数；参数在 $O_n$ 里与 $\bar S$ 里可能耦合，需先证“映射的有限时间收敛”还是“参数直接辨识”二者取一。
3. 仅 $y_0$ 可得时 $x_2,\dots,x_n$ 不可测 → 滤波器里的 $\bar S(\hat x)$ 必须用估计，PE/初始激励条件要改成“估计量版本”的激励条件（这是新的理论贡献点）。
4. 若只要 TVFT 有限时间、不要参数真值 → 退回 §7 的微分器 + sign 项组合，结论更容易写（实用有限时间）。

---

## 11. 参考文献（含 OKB doc id）

- **[1848]** Q. Wang, Y. Hua, X. Dong, P. Shu, J. Lü, Z. Ren, "Finite-time time-varying formation tracking for heterogeneous nonlinear multiagent systems using adaptive output regulation," *IEEE Trans. Cybern.*, 2023. doi:10.1109/TCYB.2023.3245139
- **[2832]** Q. Wang, X. Dong, Z. Chen, Z. Lian, J. Lü, "Finite-time formation tracking for heterogeneous Euler–Lagrange systems with an uncertain leader," *Automatica*, vol. 177, 2025.
- **[3041]** C. He, J. Huang, "Output-based adaptive distributed observer for general linear leader systems over periodic switching digraphs," *Autonomous Intelligent Systems*, 2023.
- **[3743]** J. Lv, C. Wang, L. Xie, "Adaptive distributed observer design for nonlinear multiagent systems," *Automatica*, 2025.
- **[3622]** Y. Li, W. Liu, G. Wang, L. Xie, J. Sun, J. Chen, "Data-Driven Cooperative Output Regulation via Output Feedback Control," 2026. （注意：**3621 的正文与标题不符**）
- **[3343]** H. Cai, F. L. Lewis, G. Hu, J. Huang, "The adaptive distributed observer approach to the cooperative output regulation of linear multi-agent systems," *Automatica*, vol. 75, pp. 299–305, 2017.
- **[1300]** S. Zuo, Y. Song, F. L. Lewis, A. Davoudi, "Adaptive output containment control of heterogeneous multi-agent systems with unknown leaders," *Automatica*, vol. 93, 2018.
- **[3643]** M. Cheng, H. Liu, G. Wen, J. Lü, F. L. Lewis, "Data-driven time-varying formation-containment control for a heterogeneous air–ground vehicle team subject to active leaders and switching topologies," *Automatica*, vol. 154, 2023.
- **[3817]** Z. Ren, H. Liu, M. V. Basin, Q. Gao, "Event-triggered data-driven formation control for heterogeneous multiagent systems subject to DoS attacks," *IEEE TCNS*, 2025.
- **[3818]** J.-X. Yang, Y. Xu, Y. Li, "Distributed estimation and data-driven formation control of air–ground systems via efficient reinforcement learning," *IEEE TAES*, 2025.
- **[53]** B. Dong, L. Yan, C. Yan, Y. Xia, "Output formation tracking for heterogeneous MASs with incomplete leader measurements: a hybrid-triggering observer approach," *IEEE TASE*, 2026.

### 库外经典（需另找原文）

- J. Huang, *Nonlinear Output Regulation: Theory and Applications*, SIAM, 2004.（调节器方程可解性充要条件）
- Y. Tang, "Terminal sliding mode control for rigid robots," *Automatica*, vol. 34, no. 1, pp. 51–56, 1998.（**Lemma 1 有限时间判据来源**）
- J. C. Willems et al., "A note on persistency of excitation," *Syst. Control Lett.*, 2005.（数据驱动秩条件）
- A. Levant, "Higher-order sliding modes, differentiation and output-feedback control," *Int. J. Control*, 2003.（鲁棒精确微分器）
- S. Roy, et al., 2018 / C. Yuan et al., 2021（doc 2832 的滤波–concurrent learning 思想来源）

---

## 12. 待办

- [ ] 取 doc 2832 §3.1 的完整证明（(13)–(20)），确认 $\mathrm{sig}^{\beta}$ 与滤波器 $\hat M$ 耦合时的齐次度。
- [ ] 取 doc 3041 的 (15)–(20) 证明，确认 $L_i$ 的估计误差与 $\xi_i$ 的耦合处理方式。
- [ ] 取 doc 3622 的 (18) 数据矩阵完整定义，评估“加 $\mathrm{sig}^{\beta}$ 后如何保住数据表示”。
- [ ] 检索库外：Cai & Huang 系列（adaptive DO with $S$ unknown to all followers）、Willems 基本引理型 COR、Levant 微分器型 leader 跟踪。
- [ ] 核实 doc 3621 的元数据错误（标题与正文不符），修正知识库条目。
