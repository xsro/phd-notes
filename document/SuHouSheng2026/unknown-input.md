# 论文分析：Completely Distributed Joint State–Unknown Input Estimation via Interval Observer

**元数据**：Miaohong Luo, Housheng Su, Zhigang Zeng (Fellow, IEEE)；*IEEE Transactions on Automatic Control*, Vol. 71, No. 8, August 2026, pp. 4986–4993；DOI 10.1109/TAC.2026.3662304。

> 说明：本文把“未知输入”记为 $$\vartheta(t)\in\mathbb{R}^m$$（下文为便于阅读亦记为 $$d(t)$$）。匹配条件命名为 **OMC**（observer matching condition，即 Assumption 2），最小相位条件命名为 **扩展 MPC**（extensive minimum phase condition，即 Definition 2）。区间观测器定理为 **Theorem 1**，完全分布式未知输入观测器结论为 **Theorem 2**。

---

## 一、问题描述

- **系统模型**（式 (1)(2)）：

```text
\dot{x}(t) = A x(t) + M \vartheta(t)          (1)
y(t)      = C x(t)                            (2)
```

其中 $$x(t)\in\mathbb{R}^n$$ 为状态，$$y(t)\in\mathbb{R}^q$$ 为测量输出，$$\vartheta(t)\in\mathbb{R}^m$$ 为未知输入（UI），矩阵 $$A,M,C$$ 维数适当。由于目标系统高维且空间分布，部署 $$N$$ 个 agent 的传感器网络协同测量，每个 agent $$i$$ 仅测量输出的一个子集：

```text
y_i(t) = C_i x(t)                            (5)
```

聚合后满足 $$\mathrm{col}\{y_i\} = Cx$$，即式 (6)。

- **研究问题**：在**无全局拓扑信息**的多智能体/分布式条件下，设计由 $$N$$ 个局部 UIO 构成的完全分布式未知输入观测器（DUIO），**同时估计状态 $$x(t)$$ 与未知输入 $$\vartheta(t)$$**，即要求

```text
lim_{t→∞} (x(t) - \hat{x}_i(t)) = 0,   lim_{t→∞} (\vartheta(t) - \hat{\vartheta}_i(t)) = 0.
```

- **关键难点**：
  1. UI 经**已知矩阵 $$M$$** 进入系统动力学；
  2. 传感器输出 $$y_i=C_i x$$ **仅含状态**、不直接含 $$\vartheta$$，且每个 agent 只有局部输出；
  3. 要求**完全分布式**（不依赖任何全局网络信息，如拉普拉斯矩阵特征值），须用**自适应耦合增益**取代静态/集中式增益。

- **对 UI 与初值的假设**（式 (3)(4)）：UI 为 Lipschitz 连续函数，被两个已知 Lipschitz 连续函数夹逼

```text
\underline{\vartheta}(t) \le \vartheta(t) \le \bar{\vartheta}(t)          (3)
\underline{x}(0)     \le x(0)     \le \bar{x}(0)            (4)
```

即 UI 与初值均有界。

- **匹配条件 OMC**（Assumption 2）：

```text
rank(C_i M) = rank(M) = m.                     (Assumption 2)
```

该条件（Remark 1 指明即 observer matching condition, OMC）保证每个 agent 的局部输出完整捕获 UI 影响状态的所有方向，是经典 UIO 抑制 UI 影响的基本前提（隐含 $$q_i\ge m$$）。

- **扩展最小相位条件 MPC**（Definition 2）：系统 (1) 满足 extensive MPC，当且仅当对全部 $$\mathrm{Re}(\varsigma)\ge0$$ 有

```text
rank [ \varsigma I_{\mu_i} - A_{id}    T_{id}^T M ;
        C_{id}               0        ] = \mu_i + m.      (Definition 2)
```

它排除了可检测子空间内位于闭右半平面的不稳定零动态，是从 Step 1 到 Theorem 2 证明所需的结构性条件。

---

## 二、算法设计

整体由“自适应分布式状态观测器 + UI 重构模块”两部分构成，互相依赖、协同工作（Remark 8 概括为四步）。

- **步骤 1：可检测性分解**。对可能不可检测的矩阵对 $$(A,C_i)$$ 采用可检测性分解（基于 [10]）：取 $$U_i=U_D(C_i,A)$$，其正交补为 $$U_i^\perp$$；令 $$T_{id}\in\mathbb{R}^{n\times\mu_i}$$、$$T_{iu}\in\mathbb{R}^{n\times(n-\mu_i)}$$ 分别张成 $$U_i^\perp$$ 与 $$U_i$$ 的正交基，构造正交阵 $$T_i=[T_{id}\ T_{iu}]$$，则

```text
T_i^T A T_i = [ A_{id}  0 ;  A_{ir}  A_{iu} ],    C_i T_i = [ C_{id}  0 ].
```

其中 $$(C_{id},A_{id})$$ 可检测，故存在 $$K_{id}$$ 使 $$A_{id}+K_{id}C_{id}$$ Hurwitz。

- **步骤 2：区间观测器（Theorem 1）**。针对可检测部分 $$\delta_{id}=T_{id}^T x$$，构造时变坐标变换

```text
\xi_{id}(t) = \Gamma_{id}(t) S_{id} \delta_{id}(t) \equiv \Xi_{id}(t) \delta_{id}(t),
```

使矩阵 $$\Omega_{id}$$ **既 Hurwitz 又 Metzler**（时不变）。区间观测器动态（式 (11) 上/下界）写为

```text
\dot{\xi}_{id} = \Omega_{id} \xi_{id} - \Xi_{id} K_{id} y_i
              + (\Xi_{id} T_{id}^T M)^+ \vartheta - (\Xi_{id} T_{id}^T M)^- \vartheta.
```

由引理 4（Metzler 矩阵+非负初值/输入→状态非负），误差 $$e_{\xi_{id}},e_{\bar{\xi}_{id}}$$ 非负，从而得到

```text
\underline{\delta}_{id} \le \delta_{id} \le \bar{\delta}_{id},
```

即 $$\delta_{id}$$ 的上下界（Theorem 1 结论）。

- **步骤 3：代数法重构 UI（式 (19)）**。由 $$y_i=C_i x=C_{id}\Xi_{id}^{-1}\xi_{id}=F_1(t)\xi_{id}$$ 得到输出上下界，引入凸组合系数 $$\sigma_i(t)\in[0,1]$$（式 (14)–(15)）。对比 $$y_i$$ 的两种表达式并比较 (17) 与 (18)，因 $$\mathrm{rank}(C_i M)=m$$ 左可逆（Moore–Penrose 逆存在），给出代数 UI 表达式：

```text
\vartheta = (C_i M)^\dagger \big\{ \mathrm{diag}\{f_{i,2}(t)\}\sigma_i
          + \mathrm{diag}\{f_{i,1}(t)\}\dot{\sigma}_i + f_{i,3}(t)
          - C_{id}(A_{id}+K_{id}C_{id})\delta_{id} + C_{id}K_{id}y_i \big\}.   (19)
```

其中 $$\sigma_i$$ 由区间观测器边界按式 (20) 给出；$$\dot{\sigma}_i$$ 由滑模微分器（式 (21)）**有限时间**重构：

```text
\dot{\alpha}_{i,s} = \beta_{i,s},\quad
\dot{\beta}_{i,s} = -\gamma_{i,s}^{(1)}|\alpha_{i,s}-\sigma_{i,s}|^{1/2}\mathrm{sign}(\cdot)+\alpha_{i,s},\quad
\dot{\alpha}_{i,s} = -\gamma_{i,s}^{(2)}\mathrm{sign}(\alpha_{i,s}-\beta_{i,s}).          (21)
```

适当调参后 $$\alpha_{i,s}$$ 在有限时间内精确估计 $$\dot{\sigma}_{i,s}$$（Remark 6 给出调参规则 $$\gamma_{i,s}^{(1)}>1/\bar{h}^2,\ \gamma_{i,s}^{(2)}>\bar{h},\ \bar{h}\ge|\ddot{\sigma}_{i,s}|$$）。

- **步骤 4：自适应分布式状态观测器（式 (22)）及动态增益 $$\gamma_i,\rho_i$$（式 (23)–(24)）**。agent $$i$$ 的更新律：

```text
\dot{\hat{x}}_i = (A+K_i C_i)\hat{x}_i - (\gamma_i+\rho_i)T_{iu}T_{iu}^T\!\sum_{j=1}^N l_{ij}\hat{x}_j
              - K_i y_i + M\hat{\vartheta}_i - L_i(y_i-C_i\hat{x}_i),                       (22)
\hat{\vartheta}_i = (C_i M)^\dagger\{\dots +(C_i M)\text{项用 }\dot{\hat{\sigma}}_i\text{ 替代}\dots\},
\rho_i = \Big\|T_{iu}^T\!\sum_{j=1}^N l_{ij}\hat{x}_j\Big\|,   \dot{\gamma}_i=\rho_i,\ \gamma_i(0)>0.   (23)(24)
```

增益 $$\gamma_i,\rho_i$$ 仅依赖局部/邻居信息，无需全局拓扑参数。

---

## 三、稳定性分析

- **Theorem 2 结论**：若 **Assumption 1**（通信拓扑 $$G$$ 为任意有向图，含 $$\phi$$ 个 iSCC）、**Assumption 2（OMC）** 成立，系统满足 **extensive MPC（Definition 2）**，且对每个 iSCC 有 $$(\mathrm{col}\{C_i\}_{i\in V_s^{sub}},A)$$ 可检测，则更新律 (22) 连同自适应律 (23)(24) 构成一个**完全 DUIO**，即**不依赖任何全局信息、同时渐近估计目标状态与 UI**。

- **收敛性证明方法**：基于**LaSalle 不变原理**。定义状态估计误差 $$\zeta_i=x-\hat{x}_i$$、UI 估计误差 $$\eta_i=\vartheta-\hat{\vartheta}_i$$；由于 $$\dot{\sigma}_i-\hat{\dot{\sigma}}_i=0$$ 在有限时间成立，可得代数关系 $$\eta_i=-(C_i M)^\dagger C_{id}(A_{id}+K_{id}C_{id})T_{id}^T\zeta_i$$（式 (26)）。经可检测性分解与坐标变换 (31)–(34)，构造 Lyapunov 函数 (式 964)，证明 $$\dot{V}\le0$$；由 $$\dot{V}=0\Rightarrow z_{0d}=z^d=z_{0u}=z^u=0$$，依 LaSalle 不变原理得

```text
lim_{t→∞} \zeta_i = lim_{t→∞} (x - \hat{x}_i) = 0,
lim_{t→∞} \eta_i = lim_{t→∞} (\vartheta - \hat{\vartheta}_i) = 0.        (Theorem 2 末尾)
```

- **所需稳定条件**：
  1. 误差极点 Hurwitz 配置：$$A_{id}+K_{id}C_{id}$$ 与 $$\tilde{A}_{id}=A_{id}+K_{id}C_{id}-T_{id}^T M(C_i M)^\dagger C_{id}(A_{id}+K_{id}C_{id})+L_{id}C_{id}$$ 均 Hurwitz；
  2. **OMC**（Assumption 2）；
  3. **extensive MPC**（Definition 2）；
  4. 滑模微分器可达性（式 (21) 调参条件，使 $$\dot{\sigma}_i$$ 有限时间精确估计）。

- **说明**：状态误差 $$\zeta_i$$ 为**渐近**衰减；UI 导数 $$\dot{\sigma}_i$$ 为**有限时间**估计，故随状态估计收敛，$$\hat{\vartheta}_i$$ 整体达到**渐近精确**（$$\eta_i\to0$$）。

---

## 四、回答两个问题

### 1. 文章中的 unknown input 是什么？

未知输入 $$\vartheta(t)\in\mathbb{R}^m$$ 是经**已知矩阵 $$M$$** 进入系统动力学 $$\dot{x}=Ax+M\vartheta$$ 的外生信号，代表外部扰动、执行器故障或参数不确定性等有界干扰（式 (3) 假设其被 $$\underline{\vartheta}(t)\le\vartheta(t)\le\bar{\vartheta}(t)$$ 夹逼且 Lipschitz 连续）。其可估计性依赖**匹配条件 OMC**（$$\mathrm{rank}(C_i M)=\mathrm{rank}(M)=m$$，保证局部输出完整捕获 UI 影响）及**有界性假设**。

### 2. 估计 input 的结果是渐近的吗？

**是，整体为渐近估计。** 需区分三层收敛速度：① 状态估计误差 $$\zeta_i=x-\hat{x}_i$$ 依 **Theorem 2** 与 **LaSalle 不变原理** **渐近**收敛到 0；② UI 导数 $$\dot{\sigma}_i$$ 由滑模微分器（式 (21)）**有限时间**精确重构；③ 由于代数 UI 表达式 (19) 中除 $$\dot{\sigma}_i$$ 外的状态量均渐近收敛，整体 UI 估计误差 $$\eta_i=\vartheta-\hat{\vartheta}_i$$ 随之**渐近精确**（$$\lim_{t\to\infty}\eta_i=0$$）。因此，文章结论“同时渐近估计状态与 UI”中的 UI 估计确为渐近收敛。
