# 规定时间 / 精确时间控制及其相关概念的定义汇编

> 资料来源：控制理论文献知识库（okb-assist MCP）。
> 说明：下列定义均标注具体出处（文献、作者、年份、知识库文档 ID）。文末专门列出**不同文献间定义存在差异/命名冲突**的地方，请特别留意。

---

## 1. 时间收敛控制家族总览

控制理论中围绕"收敛时间可设计"形成了一族概念，按"收敛时刻与设定时间 T 的关系"可排序为：

| 概念 | 设定时间 T 的含义 | 是否依赖初值 | 代表出处 |
|------|---------------------|--------------|----------|
| 有限时间 (Finite-time, FT) | 收敛发生在某个有限时刻，**依赖初值** | 是 | Bhat & Bernstein; 见 Liu et al. 2022 (doc 2508) |
| 固定时间 (Fixed-time, FxT) | 收敛时间的**上界**与初值无关 | 否（仅与设计参数有关） | Polyakov 2011; 见 Liu et al. 2022 (doc 2508) |
| 预定/规定时间 (Predefined/Prescribed-time, PdT/PT) | 收敛时间的**最小上界**可由设计者**任意预设**，且与初值无关 | 否 | Sánchez-Torres et al. 2018（PdT）/ Song et al. 2017-2023（PT） |
| 精确时间 (Exact-time) | 系统**恰好在**设定的 T 时刻收敛（等于 T，而非仅以 T 为上界） | 否 | Chen, Sun & Hua 2023（仅被引用，知识库无全文） |
| 约定时间 (Appointed-time) | 可由设计者**离线预先指定**收敛时刻 T；部分文献坚称其等价于 FT/FxT，但强调"时间可预先给定" | 视具体定义 | Wei et al. 2018 (doc 266) |
| 任意时间 (Arbitrary-time, AT) | 收敛时间上界 Ta 可通过**调节设计参数 ϕ**任意整定（Ta **依赖参数**，需调参实现） | 否（但 Ta 依赖参数） | Pal, Kamal, Nagar, Bandyopadhyay & Fridman 2020（Automatica 112:108710） |
| 自由意志任意时间 (Free-will Arbitrary-time, FWAT) | 收敛时间 Ta **独立于任何系统参数与初值**，可由设计者**自由预先指定**；分弱型（Ta ≥ 真实收敛时间）与强型（Ta = 真实收敛时间，恰在 Ta 命中） | 否 | 同上（Pal et al. 2020，定义原文见第 2.9 节） |

> 综述性出处：Song, Ye & Lewis, **"Prescribed-Time Control and Its Latest Developments,"** IEEE TSMC, 2023（知识库 doc **2315**）。
> 该综述将发展脉络明确为：FT（收敛依赖初值）→ FxT（上界与初值无关但难以直接整定）→ PdT/PT（最小上界可由用户预设、与初值及参数均无关）。


---

## 2. 各概念的具体定义与出处

### 2.1 有限时间稳定 (Finite-time stable, FT)

**出处**：Liu, Li, Zuo, Li & Lu, **"An Overview of Finite/Fixed-Time Control and Its Application in Engineering Systems,"** IEEE/CAA J. Autom. Sinica, 2022（知识库 doc **2508**, Definition 1，引自 Bhat & Bernstein [10]）。

**定义（Definition 1）**：自治系统 dot chi = f(t, chi) 的平衡点 chi=0 称为**有限时间稳定**，若存在原点的邻域 U 与函数 T(chi0): U\{0} -> (0,inf) 使：

1. **有限时间收敛**：lim_{t->T(chi0)} chi(t,chi0)=0，且对任意 t>T(chi0) 有 chi(t,chi0)=0；
2. **Lyapunov 稳定**。

其中 T(chi0) 称为收敛时间/镇定时间（settling time），**显式依赖初值** chi0。当 U=D=R^n 时称为**全局有限时间稳定**。

**出处原文要点**（doc 2508, p.2-3）：
> "the settling time will become long if the system initial value chi0 is large … T(chi0) may not be computed when [the initial value] is unknown."

---

### 2.2 固定时间稳定 (Fixed-time stable, FxT)

**出处**：同上（Liu et al. 2022, doc **2508**, Definition 2，引自 Polyakov [21]/[41]）。

**定义（Definition 2）**：原点称为**固定时间稳定**，若它**全局有限时间稳定**，且收敛时间存在上界，即存在 T_max>0 使得对任意 chi0 in R^n 有
T(chi0) <= T_max.

即收敛时间**上界与初值无关**，只与设计参数有关（可通过形如 Vdot <= -[gamma1 V^p + gamma2 V^q]^k, pk<1, qk>1 的 Lyapunov 条件保证，doc 2508 Lemma 5）。

**出处原文要点**（doc 2508, p.3）：
> "the settling time has an upper bound T_max such that T(chi0) <= T_max for any chi0 in R^n."

> 综述 Song et al. 2023（doc 2315）补充：FxT 的收敛时间上界虽与初值无关，但"没有简单直接的关系把控制参数与期望的上界联系起来"，且上界常被**严重高估**（数百乃至数千倍）。

---

### 2.3 规定时间 / 预定时间稳定 (Prescribed-time / Predefined-time, PT / PdT)

这两个名称在文献中并存，核心思想一致，但**命名与实现机制存在分歧**（详见第 3 节）。

**(a) Prescribed-time（规定时间，Song 一派）**

**出处**：Song, Ye & Lewis, **"Prescribed-Time Control and Its Latest Developments,"** IEEE TSMC, 2023（知识库 doc **2315**，综述，源自 Song et al. 2017 原创工作）。

**定义要点**（doc 2315 摘要与引言）：
> "The salient feature of PT control lies in its ability to achieve system stability within a finite settling time **user-assignable in advance irrespective of initial conditions**."

即：系统在**用户预先规定、且与初值无关**的有限镇定时间内达到稳定。实现上几乎必须采用**时变反馈增益**（随时间趋于设定时间 T 而趋于无穷），故常称 time-varying high-gain / time-varying feedback 方法（doc 2315, 1777, 2821 等多篇均强调此点）。

**(b) Predefined-time（预定时间，Sánchez-Torres 一派）**

**出处**：Sánchez-Torres, Gómez-Gutiérrez, López & Loukianov, **"A class of predefined-time stable dynamical systems,"** IMA J. Math. Control Inf., 35(1):1-29, 2018（知识库内**仅有引用**，见于 doc 954 / 1120 / 1494 / 2534 / 3026 的参考文献；并见 Jiménez-Rodríguez et al., *"A Lyapunov-like characterization of predefined-time stability,"* IEEE TAC 2020，doc 1120 引 [23]）。

**定义要点**（据引用文献描述）：给定系统可在**用户预设的收敛时间上界**内稳定，且上界可由设计参数直接给定、与初值无关。该派常借助 **Time Base Generator (TBG)** 与特定 Lyapunov 函数类来实现（如 doc 703、2534 所述"predefined convergence time bound"）。

> 注：在多智能体/观测器/微分器文献中，"predefined-time" 有时被定义为**"具有用户预设上界的固定时间稳定系统"**（Aldana-López et al., *"Generating new classes of fixed-time stable systems with predefined upper bound for the settling time,"* Int. J. Control, 2022，doc 2534 引）。即"predefined"在此修饰的是**固定时间系统的上界**，而非独立的新收敛类——这是另一处命名歧义。

---

### 2.4 实用规定时间稳定 (Practical Prescribed-time, PPT)

**出处**：Cao, Cao & Song, **"Practical prescribed time tracking control over infinite time interval…,"** Automatica, 2022（知识库 doc **2845**）；Shi, Keliris, Hou, Duan & Polycarpou, IEEE TAC, 2025（doc **1224**）；Jia, Liu, Jia & He, *"Prescribed-time nonsingular sliding mode control based on neural network…,"* Information Sciences, 2025（doc **954**，含"practical prescribed time stability"引理）。

**定义要点**（doc 2845 摘要；doc 1224 引言）：
> "the system state/tracking error can be shown to converge to a **prescribed set** [残差集/给定小邻域] within a **user-assignable settling time**."

即：不是收敛到**精确零**，而是收敛到**用户规定的集合（小残差区域）**，且到达该集合的时间可由用户预设。与"精确收敛到零"相对（见 2.6、2.7）。

---

### 2.5 约定时间（Appointed-time）

**出处**：Wei, Luo, Yin & Yuan, **"Leader-following consensus of second-order multi-agent systems with arbitrarily appointed-time prescribed performance,"** IET Control Theory & Applications, 2018（知识库 doc **266**, Definition 1；并见 Remark 1）。

**定义（Definition 1, doc 266, p.3）**：对多智能体系统，若对**任意预先指定的设定时间** T1，位置/速度跟踪误差满足
lim_{t->T1} ||x_i(t) - q_r(t)|| = 0,   lim_{t->T1} ||nu_i(t) - qdot_r(t)|| = 0,
且对一切 t>=T1 保持恒等，则称实现了 **leader-following appointed-time consensus**。

**同一文献的 Remark 1（doc 266）给出另一处表述**，注意其口吻不同：
> "rigorously speaking, **appointed-time control can be also referred to as finite-time or fixed-time control**. The difference … is that in the appointed-time control, the settling time can be **pre-assigned offline** based on the actual requirements. While in the traditional finite-time or fixed-time control … the setting time can be only estimated based on several design parameters."

即：该文献内部将"appointed-time"同时描述为 (i) "可在任意预设时刻 T1 精确达到"（Definition 1）与 (ii) "本质上属于 FT/FxT，只是时间可离线预先给定"（Remark 1）。这两种表述的强弱程度并不完全一致，是需注意的**同一文献内部的措辞差异**。

---

### 2.6 精确时间控制 (Exact-time control)

**出处**：Chen, Sun & Hua, **"Finite/fixed/predefined/exact time control: a unified framework,"** Int. J. Systems Science, 54(5):977-990, 2023（知识库内**仅有引用**，见于 doc **3247** 参考文献 [139]；知识库**未收录全文**）。

**定义要点（据统一框架的分类，非原文逐字）**：在有限/固定/预定/精确四类"时间可控"框架中，**exact time** 指系统收敛发生的实际时刻**恰好等于**用户设定的 T——即收敛时刻被**精确钉在** T，而非仅以 T 为上界（区别于 predefined-time 的"最小上界为 T"）。

> 提醒：因知识库无此文全文，"exact-time"的严格数学定义（与各概念的边界）应以原文 Chen et al. 2023 为准；本表据此文在统一框架中的定位作解释性描述。

---

### 2.7 规定时间精确跟踪 (Prescribed-Time Exact Tracking, PTeT)

**出处**：Wang & Liu, **"Prescribed-time exact tracking for a class of nonlinear systems,"** IEEE Control Systems Letters, 2023（知识库 doc **2316**）。

**定义要点**（doc 2316 引言）：与"规定时间镇定 (PTS，目标为 0)"和"规定时间实用跟踪 (PTpT，误差进入给定带状区域)"不同，PTeT 要求**跟踪误差在规定的 T 时刻恰好归零**（目标为时变信号），且一旦在 T 时刻误差归零，控制任务即完成、控制系统随之结束。

> doc 2316 明确指出 PTeT 比 PTpT **更强**：PTpT 只要求误差进入预设条纹带，PTeT 要求误差在 T 时刻**命中零**。

---

### 2.8 预先给定时间 / 预先指定时间 (Preassigned-time / Pregiven-time)

> ✅ **检索复核说明（2026-08-02 通过 okb-assist MCP 全文检索复核）**：本节关于 "preassigned / pregiven / pre-assigned / pre-given" 的结论，已用 `grep_search` 对全库做完整检索复核（说明：组合正则 `preassigned|pregiven|pre-assigned|pre-given` 在本接口返回 0 命中，疑为 alternation 不被支持；故改用单关键词 `pregiven`、`pre-assigned`、`preassigned` 分次检索，分别得到 10 / 17 / 若干条命中）。结果**支持**原判断，并补充若干原未检出的片段（见下）。

**核心判断**：在本知识库中，**"preassigned-time" 与 "pregiven-time" 并非像 prescribed/predefined/appointed/exact 那样被当作一个形式化、独立命名的收敛类**，而是作为**描述性措辞**（"预先给定/预先指定"的形容词或动词）零散出现，其含义通常与 appointed/prescribed/predefined 时间**重叠、可互换**。

**已检索到的具体出处片段**：

- **doc 2508**（Liu, Li, Zuo, Li & Lu, 2022, *An Overview of Finite/Fixed-Time Control…*，固定时间部分）在固定时间引理讨论中写道：
  > "Therefore, the settling time can be **pregiven**."
  
  即在**固定时间 (FxT)** 框架下，由于收敛时间上界只取决于设计参数，故该上界可以"预先给定"。此处 "pregiven" 仅是说明 FxT 上界**可预先设定**，并未引入新的控制类。

- **doc 266**（Wei, Luo, Yin & Yuan, 2018, *Appointed-time consensus…*，Remark 1）在约定时间讨论中写道：
  > "the setting time can be **pre-assigned offline** based on the actual requirements."
  
  即在 **appointed-time** 语境下，设定时间可"离线预先指定"。此处 "pre-assigned" 是描述 appointed-time 的特征，而非独立概念。

- **doc 2289**（Zou, Deng, Dong, Ding & Lu, 2022, *Distributed output feedback consensus tracking…*，参考文献 [1]）的引文标题为：
  > "Neural network-based distributed adaptive **pre-assigned finite-time** consensus of multiple TCP/AQM networks"

  即 "pre-assigned" 在此作为**修饰语**修饰 **finite-time consensus**（预设有限时间一致性），表示一致性可在预先指定的时间内达成；它并未引入区别于 FT/FxT 的独立收敛类，而是与 finite-time 组合使用。

- **doc 391**（Xiong & Zhang, 2024, *Time-varying formation-surrounding control…*）在 appointed-fixed-time 语境中写道：
  > "T denotes the **pre-assigned** settling time … the system is **appointed-fixed-time** stable"

  即 "pre-assigned settling time" 是 **appointed-fixed-time**（约定-固定时间）框架下的"预先给定镇定时间"描述语，仍非独立类别。

- **doc 2817**（Cui, Wang, Liu & Xia, 2023, *Sliding mode based prescribed-time consensus…*，Automatica）在 prescribed/predefined-time 综述中写道：
  > "the convergence time is irrespective of initial conditions and any design parameters, thus can be **pre-assigned**."

  即 prescribed-/predefined-time 的收敛时间"可预先指定"，此处 "pre-assigned" 与 prescribed/predefined 同义混用。

- **doc 3279 / 1224**（Liu et al. 2024 / Shi et al. 2025）在 prescribed performance control (PPC) 语境中使用 "**preassigned** performance metrics / preassigned performance control"——此处 "preassigned" 修饰的是**性能界（performance）**而非收敛时间，属另一概念分支，需注意与"时间可预设"区分。

**结论**：preassigned / pregiven 在库中主要是**修饰语**，用来表达"时间（或参数/性能）可由设计者事先设定"这一共性；它们**没有**像 exact-time 那样被赋予"收敛时刻恰好等于 T"的独特数学定义。若某篇文献将其用作正式类别名（如 "pre-assigned finite-time consensus"），通常与 finite-time / fixed-time / appointed-time / prescribed-time 同义混用，而非一个单独的收敛类。全库检索未见将其定义为**独立收敛类**的原文，故不单列形式化定义，仅作说明。

---

### 2.9 任意时间 / 自由意志任意时间稳定 (Arbitrary-time / Free-will Arbitrary-time Stable, AT / FWAT)

**出处（本节定义原文）**：Pal, Kamal, Nagar, Bandyopadhyay & Fridman, **"Design of controllers with arbitrary convergence time,"** *Automatica*, 112 (2020) 108710（用户提供 PDF 全文；知识库 doc 2508 的综述亦以 "Free-Will Arbitrary Time Control" 为题引述了同一非自治微分方程）。

**定义 3（Arbitrary-time Stable，任意时间稳定）**：系统原点称为**任意时间稳定**，若
1. 它是固定时间稳定；
2. 存在 Ta > 0，它**依赖于已知系统参数 ϕ** 且对给定 ϕ 可预先估计；
3. 可通过改变设计参数 ϕ（在设计允许范围内）**任意整定** Ta；
4. 对给定 ϕ，下列之一成立：
   - (a) Ta ≥ Ttf（**弱**任意时间稳定，weak arbitrary-time）；
   - (b) Ta = Ttf（**强**任意时间稳定，strong arbitrary-time）；

其中 Ttf 为**真实固定收敛时间**（true fixed time，轨迹实际收敛到原点的精确时刻）。

**定义 4（Free-will Arbitrary-time Stable，自由意志任意时间稳定）**：系统原点称为**自由意志任意时间稳定**，若
1. 它是固定时间稳定；
2. 存在 Ta > 0，它**独立于任何系统参数与初值**，且可由设计者**任意预先指定**；
3. 下列之一成立：
   - (a) Ta ≥ Ttf（Free-will **weak** arbitrary-time stable）；
   - (b) Ta = Ttf（Free-will **strong** arbitrary-time stable）。

> **Remark 1（为何称 "free-will"）**：自由意志是一个"强"概念，它让设计者对系统拥有**终极控制**——轨迹在**预先给定的时刻**收敛到平衡点，"随心所欲"（as per our own will），**无需顾及初值与系统参数**。这正是 "free-will" 命名的由来。
> **Remark 2**：True fixed time（真实固定时间）是轨迹实际收敛到原点的**真实/精确**时刻。

**Lyapunov 刻画（Theorem 1）**：对有限区间 I = [t0, tf]，若存在 C¹ 函数 V : I×D → R≥0 与 η ≥ 1 使
- α₁(x) ≤ V(t,x) ≤ α₂(x)，且 V(t,0)=0；
- V̇ ≤ −η(e^V − 1) / [e^V (tf − t)]  （∀V ≠ 0, ∀t ∈ I），

则原点是 **free-will weak arbitrary-time stable**，且 Ta = tf − t0 ≥ Ttf。若等号成立 V̇ = −η(e^V − 1)/[e^V (tf − t)]，则原点是 **free-will strong arbitrary-time stable**，且 Ta = tf − t0 = Ttf。

**典型非自治示例（论文 Eq.(1)）**：
ẋ = −η(e^x − 1) / [e^x (tf − t)]（t0 ≤ t < tf），否则 0，η ≥ 1。
其解 x = ln(C (tf − t)^η + 1)，C = (e^{x(t0)} − 1)/(tf − t0)^η。易见当 t → tf 时 x, ẋ → 0，且对一切 t ≥ tf 有 x = 0——即系统**在时刻 tf 之前或之时**收敛并从此保持于原点。

**与本文其他概念的对照**：
- FWAT 的收敛时间 Ta 既**与初值无关**、又**与系统参数无关**——这是它区别于普通 arbitrary-time（AT，Ta 依赖参数、需调参整定）的关键，也是它区别于 fixed-time（上界依赖参数、难以直接整定）之处。
- FWAT **强型**的 Ta = Ttf，即收敛**恰好发生在**预设时刻 tf，这与第 4 节讨论的 **exact-time / PTeT** "恰在 T 时刻命中"语义一致；而 FWAT **弱型**仅保证"在 tf 之内"收敛（上界保证），与 **prescribed-time (PT)** 的"在 T 内"语义一致。
- 实现机制上，FWAT 与 PT 同源：均在分母引入 (tf − t) 的时变增益，使增益在 t → tf 时趋于无界（time-varying high-gain），从而把收敛"钉"在预设时刻附近。区别在于 PT（Song 一派）强调 T 与初值无关、由用户 assign；FWAT 进一步强调 Ta 还**与系统参数无关、可自由指定**，并按 Ta 是否等于真实收敛时间区分强/弱型。

---

## 3. 不同文献间定义的差异与命名冲突（请特别留意）

整理过程中发现以下**并非完全一致**的定义/命名，使用时需对照原文：

1. **"prescribed-time" 与 "predefined-time" 是否为同一概念？**
   - **Song 一派（PT, doc 2315 等）** 与 **Sánchez-Torres 一派（PdT, 2018）** 在综述中常被当作同一思想（用户预设、与初值无关的收敛时间）混用。
   - 但**机制不同**：PT 主打 time-varying high-gain 反馈（增益在 t->T 时趋于无穷）；PdT 主打 TBG 与特定 Lyapunov 函数类。
   - 另有文献（如 Aldana-López et al. 2022, doc 2534 引）把 "predefined-time" 定义为**"上界被用户预设的固定时间稳定系统"**——此时 predefined 修饰的是 FxT 的上界，而非与 PT 并列的新类别。
   - **结论**：两词高度相关但不可无条件互换；跨文献引用时应核查作者所指机制。

2. **"appointed-time" 的强弱表述不一致**
   - Wei et al. 2018（doc 266）的 Definition 1 给出**强定义**：对任意预设 T1 误差**精确**在 T1 归零。
   - 同一文献 Remark 1 又将其弱化为"本质上属 FT/FxT，只是时间可离线预先给定"。
   - 这与 "prescribed/exact time" 的"精确在 T 收敛"在语义上易混淆，需区分"时间可预设"与"收敛时刻恰好等于 T"。

3. **"exact time" 与 "prescribed-time exact tracking (PTeT)" 易混**
   - "exact time"（Chen et al. 2023）是**统一框架中的一个收敛类**，强调收敛时刻**等于** T。
   - "PTeT"（Wang & Liu 2023, doc 2316）是**跟踪问题**中的目标（误差在 T 精确命中零），属应用场景而非独立的收敛类定义。
   - 二者都含"精确"二字，但层级不同。

4. **"practical" 类（PPT / PTpT / practically appointed-time）均放宽到残差集**
   - 凡带 "practical" 的定义（doc 2845/1224/954，及 doc 266 Definition 2）都是收敛到**规定集合/精度**而非精确零，与严格（exact/精确）定义形成对照。

5. **关键原文的可用性**
   - 本知识库中 **"exact-time" 统一框架原文（Chen et al. 2023）与 Sánchez-Torres 2018 PdT 原文均只有引用、无全文**。上述相关定义系依据引用文献的描述与定位归纳，正式引用请以原文为准。

6. **"preassigned / pregiven time" 不是独立命名的收敛类（2026-08-02 已用 okb-assist MCP `grep_search` 全库复核）**
   - 在本知识库中，这两个词**主要作为描述性措辞**出现：doc 2508 用 "pregiven" 说明**固定时间上界**可由设计参数预先给定；doc 266 用 "pre-assigned offline" 描述 **appointed-time** 的特征；doc 2817 在 prescribed-/predefined-time 综述中称收敛时间 "can be pre-assigned"，与 prescribed/predefined 同义。三者都未给出区别于 FT/FxT/PT/PdT 的独立数学定义。
   - 另有组合用法（均非独立类）：doc 2289 的 "**pre-assigned finite-time** consensus" 表示"预先指定的有限时间一致性"；doc 391 的 "appointed-fixed-time ... pre-assigned settling time" 表示约定-固定时间下的预先给定镇定时间；doc 3279 / 1224 的 "**preassigned** performance" 则修饰**性能界**而非收敛时间（属 PPC 分支）。
   - 因此，遇到 "preassigned-time / pregiven-time" 时，应理解为**"时间（或参数/性能）可由设计者事先设定"的共性描述**，通常与 appointed / prescribed / predefined 时间**同义混用**，而非一个单独的类别。全库检索未见将其定义为独立收敛类的原文。

7. **"Free-will Arbitrary-time (FWAT)" 与 "prescribed-time / exact-time" 易混（详见 2.9 节，原文 Pal et al. 2020, Automatica 112:108710）**
   - FWAT 与 PT 机制同源（都在分母引入 (tf − t) 的时变高增益，使增益在 t→tf 时趋于无界），且都让收敛时间**与初值无关**、可由用户指定。二者最易混淆。
   - 关键区别在**参数独立性**：FWAT 的预设时间 Ta **同时独立于初值与系统参数**（"free-will" 之名的由来）；而 PT（Song 一派）的 T 与初值无关，但其实现仍依赖设计参数。普通 arbitrary-time（AT，非 free-will）的 Ta **依赖参数**、需调参整定，弱于 FWAT。
   - 强弱层级上：FWAT **强型**（Ta = Ttf）与 exact-time / PTeT 同为"恰在 T 时刻命中"语义；FWAT **弱型**（Ta ≥ Ttf）与 PT 同为"在 T 之内（上界）"语义。跨文献引用时应分清所用的是强型还是弱型，以及是指 free-will 还是普通 arbitrary-time。

---

## 4. 关键澄清：prescribed-time 是"在 T 内收敛"还是"在 T 时刻收敛"？

> 这是初学者最容易混淆的一点，单列说明。

**结论**：prescribed-time (PT) control 的语义是**"在用户设定的时间 T 之内（不晚于 T）收敛"**——即 T 是镇定时间（settling time）的**用户可预设的上界 / 截止时刻**，而非"系统恰好在 T 这一瞬间完成收敛"。它与 exact-time / PTeT 的"恰在 T 时刻命中"是不同强弱层级的概念。

**为什么是"T 之内"而非"恰在 T 时刻"：**

1. **标准定义以上界形式给出。** PT 的源头工作——Song, Wang, Holloway & Krstić (2017) [doc 2359 引]——以"在 prescribed finite time 内实现调节（regulation in prescribed finite time）"立论，其方法通过对状态乘以一个"在终端时刻 T 趋于无界"的时变函数，使系统在指定的终端时刻 T **之前或之时**到达平衡点；此处 T 即用户指定的截止时刻。随后 Song, Ye & Lewis (2023) 综述 [doc 2315] 将 PT 界定为：系统"在用户预先规定、且与初值无关的有限镇定时间**内**达到稳定"（原文："achieve system stability **within** a finite settling time user-assignable in advance irrespective of initial conditions"）。即对任意初值，镇定时间满足 T(χ₀) ≤ T，且这个上界 T 可由设计者**任意指定**。T 是**最坏情况时限（deadline）**：实际收敛可能更早发生，但保证不晚于 T。
2. **PT 的核心卖点是"可用户指定的截止时间 T"。** 它区别于 fixed-time 的关键在于"上界可被用户直接给定"（FxT 上界虽与初值无关，但难以直接整定、常被严重高估）。PT 承诺的是"在 T **之前或之时**一定收敛"，而非"恰在 T 命中"。

**与"恰在 T 时刻"的区别（关键，勿混用）：**

| 概念 | 收敛语义 | 出处 |
|------|----------|------|
| PT / PTS（prescribed-time stabilization，目标 0） | 在 T **之内**（上界 T）收敛到零 | doc 2315 / 2316 |
| PTpT（prescribed-time practical tracking） | 在 T **之内**进入给定条纹带（残差集） | doc 2316 |
| **PTeT**（prescribed-time exact tracking） | 误差**恰在 T 时刻**归零（更强） | Wang & Liu 2023, doc 2316 |
| **exact-time**（统一框架中的收敛类） | 收敛发生的**实际时刻恰好等于** T（更强） | Chen et al. 2023, doc 3247 引 |

- doc 2316 明确把二者区分开：对 PTpT，"误差在 prescribed time **内**进入给定条纹带"就够了；而对 PTeT，"误差需要 **hit zero at the prescribed time**（在 T 时刻命中零）"，这是"a more demanding objective"。即"在 T 内"是 PTS/PTpT 的语义，"在 T 时刻命中"是 PTeT 才要求的更强语义。
- exact-time（Chen, Sun & Hua 2023，见 doc 3247 参考文献 [139]）在统一框架中强调收敛时刻**等于** T，同样是比 PT 更强的"时刻"语义。

**佐证文献（Literature Evidence）：**

1. **Song, Wang, Holloway & Krstić (2017)** — *"Time-varying feedback for … regulation in prescribed finite time"*（PT 的奠基性系统方法）。知识库 doc **2359**（Zhou 2020, Automatica）转引："A systematic approach by using time-varying feedback laws was originally established in Song, Wang, Holloway, and Krstic (2017) … to achieve regulation in prescribed finite time … via employing a scaling of the state by a function of time that grows unbounded towards the terminal time." → 说明 T 是用户指定的**终端/截止时刻**，收敛发生在该时刻之前或之时。
2. **Song, Ye & Lewis (2023)** — *"Prescribed-Time Control and Its Latest Developments,"* IEEE TSMC（PT 权威综述）。知识库 doc **2315**，摘要原文："achieve system stability **within** a finite settling time user-assignable in advance irrespective of initial conditions." → 直接佐证 PT 是"在 T **之内**"语义，且 T 由用户提前指定、与初值无关。
3. **Wang & Liu (2023)** — *"Prescribed-time exact tracking for a class of nonlinear systems,"* IEEE Control Systems Letters。知识库 doc **2316**，原文对比："It is not enough for the tracking error to enter a pregiven strip **within** prescribed time [PTpT]; instead, the error needs to **hit zero at the prescribed time** [PTeT]." → 直接佐证"在 T 内"与"恰在 T 时刻"是强弱不同的两层语义，前者（PT/PTpT）是上界保证，后者（PTeT）才要求命中 T。
4. **Chen, Sun & Hua (2023)** — *"Finite/fixed/predefined/exact time control: a unified framework,"* Int. J. Systems Science, 54(5):977-990（统一框架原文，知识库仅引用）。知识库 doc **3247** 参考文献 [139]。→ 在统一框架中把 "exact time" 定义为收敛时刻**等于** T，作为比 prescribed-time 更强的"时刻"类，佐证二者层级差异。

**一句话总结**：**prescribed-time = 保证在用户给定的截止时间 T 之前或之时收敛（T 是上界/时限）；"恰在 T 这一时刻收敛"是 exact-time / PTeT 的更强语义，二者不要混用。**（佐证见上列 4 篇文献。）

---

## 5. 关键文献索引（知识库 doc ID）

| doc ID | 文献 | 提供的定义/内容 |
|--------|------|----------------|
| 2315 | Song, Ye & Lewis, 2023, Prescribed-Time Control and Its Latest Developments (IEEE TSMC) | PT 综述、FT/FxT/PdT 脉络 |
| 2508 | Liu, Li, Zuo, Li & Lu, 2022, An Overview of Finite/Fixed-Time Control… (IEEE/CAA JAS) | FT (Definition 1)、FxT (Definition 2) 形式化定义与引理 |
| 266 | Wei, Luo, Yin & Yuan, 2018, Appointed-time consensus… (IET CTA) | appointed-time 定义（Definition 1 + Remark 1，注意内部措辞差异） |
| 2845 | Cao, Cao & Song, 2022, Practical prescribed time tracking… (Automatica) | PPT 定义 |
| 1224 | Shi et al., 2025, Preset-Trajectory-Based Tracking… (IEEE TAC) | PPT / 规定集合收敛 |
| 954 | Jia, Liu, Jia & He, 2025, Prescribed-time nonsingular SMC… (Information Sciences) | practical prescribed time stability 引理 |
| 2316 | Wang & Liu, 2023, Prescribed-time exact tracking… (IEEE CSL) | PTeT 定义（区别于 PTS/PTpT） |
| 2508（片段） | Liu et al. 2022 | "pregiven"：固定时间上界可由设计参数预先给定（描述性措辞，非独立类） |
| 266（片段） | Wei et al. 2018 | "pre-assigned offline"：appointed-time 特征描述（描述性措辞，非独立类） |
| 703 | Li, Jia, Duan & Chi, 2025, Accurate prescribed-time output consensus… (Automatica) | 有界时变增益 PT 方法 |
| 1777 / 2821 | Zhou et al. 2023 / Zuo et al. 2025 | PT 时变高增益方法 |
| 1120 / 1494 / 2534 / 3026 | 多篇 | Sánchez-Torres 2018 PdT 及后续（仅引用） |
| 3247 | 蔡中泽、曾庆双, 2024 学位论文 | Chen, Sun & Hua 2023 "exact time" 统一框架（仅引用 [139]） |
| （用户提供 PDF） | Pal, Kamal, Nagar, Bandyopadhyay & Fridman, 2020, *Design of controllers with arbitrary convergence time*, Automatica 112:108710 | **AT / FWAT 定义原文（Definition 3 & 4、Theorem 1、Lyapunov 刻画）**；知识库 doc 2508 综述亦以 "Free-Will Arbitrary Time Control" 引述 |
