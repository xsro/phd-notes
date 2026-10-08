# Forward Completeness 概念调研报告

> **调研日期**: 2025 年 10 月 8 日
> **数据来源**: OKB-Assist 知识库 (Zotero 文献管理)
> **关键词**: forward completeness, unboundedness observability, Lyapunov characterization, finite escape time

---

## 1. 定义与基本概念

### 1.1 Forward Completeness (前向完备性)

考虑一般非线性系统：

$$
\dot{x} = f(x, u), \qquad y = h(x)
$$

其中 $x \in \mathbb{R}^n$ 为状态，$u \in \mathbb{R}^m$ 为输入，$y \in \mathbb{R}^p$ 为输出。假设 $f$ 和 $h$ 为局部 Lipschitz 连续。对任意初始条件 $\xi$ 和输入信号 $u(\cdot)$，存在唯一的最大解 $x(\cdot, \xi, u)$，定义在区间 $(\sigma_{\xi,u}^{\min}, \sigma_{\xi,u}^{\max})$ 上。

**Forward Completeness 定义**：系统是前向完备的（forward complete），当且仅当对任意初始条件 $\xi$ 和任意输入信号 $u$，相应解对所有 $t \geq 0$ 有定义，即 $\sigma_{\xi,u}^{\max} = +\infty$。

换句话说，系统不会在有限时间内发生状态逃逸（finite escape time）。

### 1.2 Unboundedness Observability (无界可观性)

这是一个比 forward completeness 更弱的性质。系统具有无界可观性（UO），如果对每个 $\xi$ 和控制 $u$，当最大存在时间 $\sigma = \sigma_{\xi,u}^{\max} < \infty$ 时，必然有：

$$
\lim_{t \nearrow \sigma} \sup |y(t, \xi, u)| = +\infty
$$

也就是说，状态的任何无界性都可以通过输出观测到。其逆否命题是：如果输出在 $[0, T)$ 上有界，则 $x(T)$ 一定存在。

---

## 2. 核心参考文献

### 2.1 奠基性论文

| 文献 | 作者 | 年份 | 期刊 |
|------|------|------|------|
| **Forward completeness, unboundedness observability, and their Lyapunov characterizations** | David Angeli, Eduardo D. Sontag | 1999 | Systems & Control Letters, Vol.38, No.4, pp.209-217 |

**ID: 3464 | DOI: 10.1016/S0167-6911(99)00055-9**

这是 forward completeness 理论的奠基之作。主要贡献包括：

1. **Lyapunov 充要条件**：证明 forward completeness 可以通过光滑标量增长不等式来刻画。
2. **有输入系统的推广**：将结果推广到带输入的系统。
3. **带输出系统推广**：引入 unboundedness observability 作为相对完备性的概念。
4. **可达集边界**：用输入能量的类 $L^1$ 估计给出可达状态的界。

### 2.2 重要相关文献

| 文献 | 作者 | 年份 | 来源 |
|------|------|------|------|
| Delay Compensation for Nonlinear, Adaptive, and PDE Systems (Chapter 11: Forward-Complete Systems) | Miroslav Krstic | 2009 | 专著 |
| Further results on the use of Nussbaum gains in adaptive neural network control | Haris E. Psillakis | 2010 | IEEE TAC |
| Output-feedback multivariable global variable gain super-twisting algorithm | Vidal, Nunes, Hsu | 2017 | IEEE TAC |
| Integral Characterizations of Uniform Asymptotic and Exponential Stability with Applications | Andrew Teel | 2002 | Math. Control Signals Systems |
| High-Order Barrier Functions: Robustness, Safety, and Performance-Critical Control | Tan, Cortez, Dimarogonas | 2022 | IEEE TAC |
| Event-triggered sliding mode control using new triggering rules | Malisoff, Selivanov | 2026 | Automatica |
| Forward completeness in open sets and applications to control of automated vehicles | Karafyllis, Theodosis, Papageorgiou | 2025 | IEEE TAC |
| Robust Neural IDA-PBC: Passivity-based stabilization under uncertainty | Sanchez-Escalonilla Plaza, Zoboli, Jayawardhana | 2026 | Automatica |
| Low-complexity tracking control for p-normal form systems using a novel Nussbaum function | Ding, Wei | 2022 | IEEE TAC |
| Sufficient Conditions for UGAS and ISS Using High-Order Control Barrier Functions | Marley, Skjetne, Teel | 2023 | IEEE TAC |

---

## 3. Lyapunov 刻画理论

### 3.1 Forward Completeness 的 Lyapunov 充要条件

**定理 2 (Angeli & Sontag, 1999)**：系统 (5)（输入取值于紧致集 $\mathcal{D}$）是 forward complete 的，当且仅当存在一个 proper 且光滑的函数 $V: \mathbb{R}^n \to \mathbb{R}_{\geq 0}$，满足以下指数增长条件：

$$
DV(x) g(x, d) \leq V(x), \quad \forall x \in \mathbb{R}^n, \forall d \in \mathcal{D}
$$

这个结果的一个推论（Corollary 2.11）是：系统 (1) 是 forward complete 的，当且仅当存在光滑 proper 函数 $V$ 和 $\mathcal{K}_\infty$ 类函数 $\sigma$，使得：

$$
DV(x) f(x, u) \leq V(x) + \sigma(|u|), \quad \forall x \in \mathbb{R}^n, \forall u \in \mathbb{R}^m
$$

### 3.2 Unboundedness Observability 的 Lyapunov 充要条件

**定理 1**：系统 (1) 具有 UO 性质，当且仅当存在 proper 光滑函数 $V$ 和 $\mathcal{K}_\infty$ 类函数 $\sigma_1, \sigma_2$，使得：

$$
DV(x) f(x, u) \leq V(x) + \sigma_1(|u|) + \sigma_2(|h(x)|), \quad \forall x \in \mathbb{R}^n, \forall u \in \mathbb{R}^m
$$

### 3.3 等价的积分形式

**推论 2.12**：系统 (1) 是 forward complete 的，当且仅当存在光滑 proper 函数 $W$ 和 $\mathcal{K}_\infty$ 类函数 $\sigma$，使得：

$$
DW(x) f(x, u) \leq 1 + \sigma(|u|), \quad \forall x \in \mathbb{R}^n, \forall u \in \mathbb{R}^m
$$

由此可得状态的显式上界估计：

$$
\alpha(|x(t, \xi, u)|) \leq W(\xi) + t + \int_0^t \sigma(|u(s)|) ds
$$

其中 $\alpha \in \mathcal{K}_\infty$。

### 3.4 可达集的能量估计

**推论 2.13**：系统 (1) 是 forward complete 的，当且仅当存在 $\mathcal{K}_\infty$ 类函数 $\chi_1, \chi_2, \chi_3, \sigma$ 和常数 $c \geq 0$，使得：

$$
|x(t, \xi, u)| \leq \chi_1(t) + \chi_2(|\xi|) + \chi_3\left(\int_0^t \sigma(|u(s)|) ds\right) + c
$$

这个结果本质上给出了**用输入能量 $L^1$ 范数来界定状态**的方法，但 Remark 2.16 指出不能一般性地取 $\sigma = \text{Id}$。

---

## 4. 核心证明思路

### 4.1 有界可达集

引理 2.1 证明：如果系统具有 UO 性质，则从任意紧致集出发、在有界时间内、使用有界控制、且输出保持有界条件下的可达集是有界的。

证明技巧：通过"输出注入"（output injection）构造辅助系统，将原系统的输出约束嵌入动力学中，然后利用 forward complete 系统的标准结论。

### 4.2 Lyapunov 函数构造

构造分几个步骤：
1. **定义 cost 函数**：$W(t, \xi) = \inf\{|x(-t, \zeta, d)| : \zeta \in \mathbb{R}^n, d \in \mathcal{M}_{\mathcal{D}}, x(0, \zeta, d) = \xi\}$ 
2. **证明局部 Lipschitz 性**（引理 2.9）
3. **通过 $U(\xi) = \inf_{t \geq 0} \alpha(W(t, \xi)) e^t$ 构造径向无界函数**
4. **光滑化**得到最终的 Lyapunov 函数

---

## 5. 与其他概念的关系

### 5.1 与 ISS 的关系

Forward completeness 是输入-状态稳定性 (ISS) 的基础假设之一。ISS 要求系统不仅要 forward complete，还要具有对输入幅度的渐近增益性质。在 ISS 理论中，forward completeness 往往作为隐式假设。

### 5.2 与控制屏障函数 (CBF) 的关系

在高阶屏障函数（HOBF）理论中（Tan et al., 2022），forward completeness 假设不再必要——该方法通过直接检查高阶导数保证前向不变性，而不需要系统本身是 forward complete 的。

### 5.3 与时滞系统预测反馈的关系

在 Krstic (2009) 的时滞补偿理论中，forward completeness 是**非线性预测反馈设计的前提条件**——如果系统不是 forward complete 的，在执行器延迟期间状态可能发生有限时间逃逸，导致控制信号永远无法作用于系统。

### 5.4 与滑模控制的关系

Malisoff & Selivanov (2026) 指出，在事件触发的滑模控制中，控制输入的设计必须确保闭环系统的 forward completeness，即对每个初始条件和所有时间 $t \geq 0$，解都存在。

---

## 6. 主要应用领域

| 应用领域 | 具体问题 | 代表性工作 |
|----------|----------|-----------|
| 自适应控制 | Nussbaum 增益在神经网络控制中的有界性 | Psillakis (2010) |
| 时滞系统 | 非线性预测反馈设计的前提条件 | Krstic (2009) |
| 安全关键控制 | 高阶控制屏障函数，前向不变性 | Tan et al. (2022), Marley et al. (2023) |
| 事件触发控制 | 滑模控制中的有限逃逸预防 | Malisoff & Selivanov (2026) |
| 协同控制 | 多智能体分布式观测器设计 | Zuo et al. (2020) |
| 机器人控制 | 空间机器人操作的协调控制 | Bruschi et al. (2026) |
| 自动化车辆 | 开放集上的前向完备性 | Karafyllis et al. (2025) |
| 输出反馈 | 多变量全局变增益超螺旋算法 | Vidal et al. (2017) |

---

## 7. 前沿进展

2025-2026 年的最新文献显示 forward completeness 概念正在向以下方向扩展：

1. **开放集上的前向完备性**（Karafyllis et al., 2025）：将经典定义从 $\mathbb{R}^n$ 推广到任意开集，应用于自动驾驶车辆的路径约束控制。

2. **神经网络 IDA-PBC 的鲁棒性**（Sanchez-Escalonilla Plaza et al., 2026）：在基于无源性的控制中，forward completeness 用于保证能量函数的良好性。

3. **指定时间控制**（Ding et al., 2026）：利用 Lyapunov 方法验证不满足线性增长条件的系统的 forward completeness。

4. **多层预测反馈**（Li, Shang, Diagne, 2026）：在非线性积分-微分方程中，通过设计源项的结构确保 forward completeness 不被破坏。

---

## 8. 关键术语对照

| 英文 | 中文 |
|------|------|
| forward completeness | 前向完备性 |
| finite escape time | 有限逃逸时间 |
| unboundedness observability (UO) | 无界可观性 |
| Lyapunov characterization | Lyapunov 刻画 |
| input-to-state stability (ISS) | 输入-状态稳定性 |
| control barrier function (CBF) | 控制屏障函数 |
| predictor feedback | 预测反馈 |
| backward completeness | 后向完备性 |
| reachable set | 可达集 |
| proper function | 径向无界函数（proper 函数） |

---

## 9. 非光滑系统的 Forward Completeness

经典 Angeli & Sontag (1999) 理论假设 $f$ 和 $h$ 为局部 Lipschitz 连续。但很多实际系统（滑模控制、变结构系统、含摩擦/碰撞的力学系统、混杂系统）具有**不连续的右端向量场**，必须用非光滑分析框架处理。

### 9.1 微分包含框架下的定义

对非光滑系统，通常用 **Filippov 意义下的微分包含** 描述动力学：

$$
\dot{x} \in F(x), \quad \text{where} \quad F(x) = K[f](x) = \bigcap_{\delta>0} \bigcap_{\mu(N)=0} \overline{	ext{co}}\{f(B(x,\delta)\setminus N)\}
$$

其中 $K[f]$ 是 Filippov 集值映射，$\overline{\text{co}}$ 为凸闭包。

**Forward Completeness 定义（非光滑系统）**：系统 $\dot{x} \in F(x)$ 是前向完备的，如果对每一个初始条件 $x(0) = \xi$，每一个**最大 Filippov 解**（maximal Filippov solution）$x(\cdot, \xi)$ 都定义在所有 $t \geq 0$ 上，即不存在有限逃逸时间。

这里的关键区别：
- **解的概念**：Filippov 解是绝对连续函数，几乎处处满足微分包含
- **最大解**：不能继续扩展的解称为最大解，其存在区间为 $[0, \sigma_{\max})$，若 $\sigma_{\max} < \infty$ 则发生有限逃逸
- **存在性**：在 $F$ 上半连续、非空紧凸、局部有界的标准假设下，Filippov 解一定局部存在

### 9.2 关键理论工具

#### (a) Filippov 解的存在性
**标准假设** (Filippov, 1988; Cortés, 2008)：若集值映射 $F: \mathbb{R}^n 
ightrightarrows \mathbb{R}^n$ 满足：
1. **上半连续** (upper semicontinuous)
2. **非空、紧致、凸值** (nonempty, compact, convex values)
3. **局部有界** (locally bounded)

则对任意初始条件，存在至少一个 Filippov 解 $x(\cdot)$ 定义在某个最大区间 $[0, \sigma_{\max})$ 上。

#### (b) 非光滑 Lyapunov 函数
对于非光滑系统，Lyapunov 函数 $V$ 通常取为：
- **局部 Lipschitz 连续** (locally Lipschitz continuous)
- **正则** (regular) 函数

其沿解的方向导数由 **Clarke 广义梯度** (Clarke's generalized gradient) $\partial V(x)$ 给出：

$$
\dot{V}(x) \in igcap_{\xi \in \partial V(x)} \xi^	op F(x)
$$

**链式法则** (Shevitz & Paden, 1994)：若 $x(\cdot)$ 是 Filippov 解，$V$ 是局部 Lipschitz 且正则，则 $V(x(t))$ 绝对连续且：

$$
rac{d}{dt} V(x(t)) \in \dot{	ilde{V}}(x(t)) \quad 	ext{a.e.}
$$

其中 $\dot{	ilde{V}}(x) = igcap_{\zeta \in \partial V(x)} \zeta^	op K[f](x)$。

#### (c) Lyapunov 刻画（非光滑版本）
**类比光滑情形**：对微分包含 $\dot{x} \in F(x)$，forward completeness 的 Lyapunov 充要条件为：

存在 proper、局部 Lipschitz、正则的函数 $V: \mathbb{R}^n 	o \mathbb{R}_{\geq 0}$，使得：

$$
\max_{\xi \in \partial V(x)} \max_{v \in F(x)} \xi^	op v \leq V(x), \quad orall x \in \mathbb{R}^n
$$

或等价地，沿所有 Filippov 解有 $D^+ V(x(t)) \leq V(x(t))$。

### 9.3 相关文献

#### 基础理论文献

| 文献 | 作者 | 年份 | 来源 | 核心贡献 |
|------|------|------|------|----------|
| Lyapunov stability theory of nonsmooth systems | **Shevitz, Paden** | 1994 | IEEE TAC | 建立非光滑 Lyapunov 稳定性理论，Clarke 梯度 + Filippov 包含框架 |
| Stability and stabilization of discontinuous systems and nonsmooth Lyapunov functions | **Bacciotti, Ceragioli** | 1999 | ESAIM: COCV | 局部 Lipschitz 正则 Lyapunov 函数的稳定性定理，微分包含一般框架 |
| Discontinuous dynamical systems: A tutorial | **Cortés** | 2008 | IEEE Control Systems Mag. | 综合性教程：Filippov 解、非光滑分析、稳定性 |
| Nonlinear differential equations with discontinuous right-hand sides | **Haddad** | 2014 | Comm. Applied Analysis | Filippov 解的非光滑稳定性、耗散性、最优控制理论 |
| On reduction of differential inclusions and Lyapunov stability | **Kamalapurkar, Dixon, Teel** | 2020 | ESAIM: COCV | 微分包含的 Lyapunov 稳定性降阶方法 |

#### 涉及 Forward Completeness 的应用文献

| 文献 | 作者 | 年份 | 来源 | 与非光滑 FC 的关系 |
|------|------|------|------|-------------------|
| **Sufficient Conditions for UGAS and ISS Using HOCBFs** | Marley, Skjetne, **Teel** | 2023 | IEEE TAC | **最直接相关**：定理面向微分包含，无需控制输入的 Lipschitz 假设；隐式假设 forward completeness |
| Event-triggered sliding mode control using new triggering rules | Malisoff, Selivanov | 2026 | Automatica | 仅用局部 Lipschitz 不能保证全局解存在，需 forward completeness 假设 |
| A unifying point of view on output feedback designs | Andrieu, Praly | 2009 | Automatica | 讨论了有限逃逸时间前的估计任务，涉及非光滑观测器 |
| High-Order Barrier Functions: Robustness, Safety, and Performance-Critical Control | Tan, Cortez, Dimarogonas | 2022 | IEEE TAC | **不需要** forward completeness 假设，通过 HOBF 直接保证前向不变性 |
| Robust finite-time stabilisation of an arbitrary-order nonholonomic system | Rocha, Castaños, Moreno | 2022 | Automatica | Filippov 微分包含解的定义在非连续向量场中 |
| Saturated Lipschitz Continuous Sliding Mode Controller | Martinez-Fuentes et al. | 2021 | IEEE TAC | 滑模控制中的微分包含分析 |

### 9.4 非光滑 FC 的 Lyapunov 刻画现状

目前**已知**的理论结果：

1. **充分条件** (充分性方向)：
   - 如果存在 proper 光滑 $V$ 满足 $DV(x)v \leq V(x)$ 对所有 $v \in F(x)$ 成立，则微分包含是 forward complete 的
   - 如果 $V$ 是局部 Lipschitz 且 $\max_{\xi\in\partial V(x)}\max_{v\in F(x)}\xi^	op v \leq V(x)$，则 forward complete

2. **必要条件** (必要性方向)：
   - **尚未有与 Angeli-Sontag 等价的完全充要性证明**公开发表
   - Teel et al. (2002) 对微分包含给出了 UGAS/UGES 的积分刻画，但未直接处理 forward completeness 的必要性 Lyapunov 构造
   - 非光滑情形下的 converse Lyapunov 定理（由 Lyapunov 函数存在性反推稳定性）仍然是一个开放或部分开放的问题

3. **特殊情形**：
   - 对于 **Filippov 意义下**满足标准上半连续、紧凸值假设的微分包含，forward completeness 的**定义本身**是清晰的
   - **混杂系统** (hybrid systems, Goebel-Sanfelice-Teer 框架) 有完整的存在性理论，forward completeness 定义为解对所有混合时间域 $(t,j)$ 存在

### 9.5 关键区别总结

| 方面 | 光滑系统 (Angeli-Sontag) | 非光滑系统 (Filippov) |
|------|------------------------|---------------------|
| 右端项 | $f$ 局部 Lipschitz | $F$ 上半连续、紧凸值 |
| 解的概念 | 经典 ODE 解 | Filippov 解（绝对连续） |
| Lyapunov 函数 | 光滑 $C^\infty$ | 局部 Lipschitz + 正则 |
| 导数工具 | 标准梯度 $DV$ | Clarke 广义梯度 $\partial V$ |
| FC 充要条件 | 完全建立 (Thm 1 & 2) | **充分性部分建立，必要性部分开放** |
| 主要文献 | Angeli-Sontag (1999) | Shevitz-Paden (1994), Bacciotti-Ceragioli (1999) |

---

## 10. 总结

Forward completeness 是连续时间非线性动力系统的一个基本性质，确保系统解对所有 $t \geq 0$ 存在。Angeli & Sontag (1999) 给出了其完整的 Lyapunov 充要条件刻画，这一结果已成为现代控制理论的重要基石。该概念在时滞系统、自适应控制、安全关键控制、事件触发控制等众多领域中发挥着关键作用，且近年来仍在持续向更复杂的系统类型（如开放集约束系统、PDE-ODE 耦合系统等）扩展。

对于**非光滑系统**，forward completeness 的定义已通过 Filippov 微分包含框架得以清晰表述，但其 Lyapunov 充要条件刻画（尤其是必要性方向）尚未完全建立，是一个仍有研究空间的方向。目前已有充分性方向的非光滑 Lyapunov 条件，以及部分特殊情形下的完整结果。