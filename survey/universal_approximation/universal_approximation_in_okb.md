# 万能逼近定理（Universal Approximation Theorem）在 okb-assist 知识库中的介绍方式

> 调研对象：MCP 服务器 **okb-assist**（全文 grep + 章节抽取 + 目录树）
> 语料规模：`total_documents = 3994`，已索引 `3120`，全部为 `markdown_done`
> 检索工具：`mcp__okb_assist__grep_search` / `search_info` / `read_markdown` / `get_toc` / `get_stats`
> 去重命中：**79 篇**文献提到 UAT 相关表述（英文 76 + 中文 5，交集 2；见附录 A）

---

## 0. TL;DR

1. **知识库里没有"一篇讲 UAT 的专题文章"**，UAT 是作为**工具性引理**散落在控制论文、教材、PINN 综述里的。
2. 精确中文术语"万能逼近定理"只命中**3 篇**（3423/3773/3804），"通用逼近定理"只命中 **1 篇**（3423）；英文 `universal approximat*` 命中 **76 篇**（不含中文命中）。**中英术语严重不对称**，只用中文检索会漏掉 95% 的语料。
3. 最常见的介绍形式是**一句话合法性声明 + 立刻转入 RBF/模糊系统的参数化公式**（`f = W*ᵀh(x) + ε`），几乎不陈述定理条件。
4. 真正给出**ε-δ 形式严格陈述**的只有少数：3761（Theorem 5.1，模糊系统 + Stone–Weierstrass）、3298（Cybenko/Hornik，`sup|f−N|<ε`）、988（Park & Sandberg）、3792/254（**函数与梯度同时逼近**的 Property 2.3）、1869（`sup_{x∈K}|ε(x)|≤e` 且对梯度成立）。
5. 教材层面写得最"像教材"的是 **3423《理解深度学习》**：先用 ReLU 分段线性直观解释"隐藏单元数 = 线性区域数"，再补一段注释给出 Cybenko(1989)/Hornik(1991) 的谱系。
6. 中文控制教材传统里 UAT 的身份是"**万能逼近器**"（universal approximator）认证机构：3773 明确把模糊系统说成"除多项式函数逼近器、神经网络逼近之外的一个新的万能逼近器"，并称"万能逼近器是自适应模糊控制的理论基础"。
7. PINN 一脉把它挪用到**算子/导数层面**（函数 + 导数、DeepONet 的算子 UAT），这是知识库里唯一成体系的一"族"介绍。
8. 知识库同时收录了**反方证据**：Barron 下界 `(1/N)^{2/n}`（2105、1674）、深度效率与 COD（3423 §4.5、3298 §2.2）、KAN 用 Kolmogorov–Arnold 定理与 UAT 正面对照（3710）、approximation-free 控制路线（478、3499）、有限时间控制因 ε 而失效的 remark（1965）。

---

## 1. 检索方法与命中概览

```text
万能逼近定理        3 篇    (3773, 3804, 3423)
万能逼近            5 篇
万能逼近器          1 篇    (3773)
通用逼近定理        1 篇    (3423)
universal approximation theorem   30 篇
universal approximator            57 篇
────────────────────────────────────────
去重合计                          79 篇
```

补充检索词（用于抓"变体表述"）：

| 检索词 | 命中 | 说明 |
|---|---|---|
| `Barron` | 15 | 其中真正相关：2105、1674、2518、3215（Barron 1993 下界） |
| `Cybenko` | 15 | 3298、3304、3423、609、3205、3415 等给出正统谱系 |
| `Stone-Weierstrass` | 5 | 3761（模糊系统证明依据）、2449（Remark 1）、3812（数学分析原典）、254/3792 |
| `Kolmogorov-Arnold` | 6 | 3710（KAN 原文对照）、3302/3304（PI-KAN） |
| `Funahashi` | 4 | 3296 用它作为 UAT 的出处 |
| `Park and Sandberg` | 2 | 775、988（RBF 版 UAT 的原始出处） |
| `逼近任意连续函数` | 1 | 3771（RBF 单隐层三段前馈网络） |
| `万能逼近原理` | 0 | 该说法在库中不存在 |

**结论：术语分布本身就是一个调研结果**——中文语料偏好"万能逼近（特性/器）"，英文语料偏好 "universal approximator"（指物）而非 "theorem"（指定理）。写综述时必须两套词都检索。

---

## 2. 六种介绍范式

### 2.1 严格陈述型（少数派，但最可引用）

**【3761】Methods of Developing Sliding Mode Controllers, §5.2 Theorem 5.1**
少见地以"定理环境"（Theorem 5.1）正式给出，且把逼近器明确指定为模糊系统：

> Consider U as a close set in Rⁿ. Then for every continuous function H(x) in U and an arbitrary ε > 0 there exists a fuzzy system M(x) such that
> `sup_{x∈U} |M(x) − H(x)| < ε`
> It means that fuzzy systems including singleton fuzzifiers, center average de-fuzzifier, product inference engine, and Gaussian membership functions are universal approximators, and the proof is proposed in [1] based on **Stone–Weierstrass Theorem**.

特征：给出定义域（闭集）、量词顺序（∀ε>0 ∃M）、上确界范数，并交代**证明依据**（Stone–Weierstrass），还顺带固定了模糊系统的四件套配置。这是库里"信息最完整的一次介绍"。

**【3298】PINNs for differential equation solutions: A comprehensive review, §2.2**
最标准的"Cybenko/Hornik 版"：

> Classical results by Cybenko [35] and Hornik et al. [61] demonstrated that a feedforward neural network with a single hidden layer and finitely many neurons can approximate any continuous function on a compact subset of Rⁿ to arbitrary accuracy, provided the activation function is **non-constant, bounded, and monotonically increasing**. More precisely, for any continuous function f: [0,1]ⁿ → R and any ε > 0, there exists a neural network N(x) such that `sup_{x∈[0,1]ⁿ} |f(x) − N(x)| < ε`.
> For PINNs, this theorem is particularly relevant because solutions to well-posed differential equations are typically smooth functions that satisfy the continuity assumptions... **In practice, however, shallow networks often require an impractically large number of neurons**, motivating the use of deeper architectures.

特征：条件（激活函数性质）+ 结论（sup 范数）+ **立刻自我拆台**（浅层不够用）三步走。

**【988】Congealed neural network design for uncertain nonlinear spatiotemporal control systems**
用 Park & Sandberg (1991) 的版本，把量化误差写进 proof：

> According to the universal approximation theorem (Park & Sandberg, 1991), for any continuous function ψₖ: Rᵐ → R and any desired approximation accuracy ε > 0, there exists an NN `ψ̂ₖ(x) = W̄ₖᵀ S(x)` on a compact set Ωₓ with weight wₖ ∈ Rᴸ, such that `sup_{x∈Ωₓ} |ψₖ(x) − ψ̂ₖ(x)| ≤ ε`.

**【3792 / 254】Reinforcement Learning for Optimal Feedback Control, Property 2.3**
库里给出"**函数 + 梯度同时逼近**"的**正式命题**（仅此一处，对自适应动态规划/actor-critic 至关重要）：

> **Property 2.3** Let V ∈ C¹(Rⁿ,R), χ ⊂ Rⁿ be compact, ε̄ > 0 be a constant, and let {σᵢ ∈ C¹(Rⁿ,R)} be a set of countably many uniformly bounded basis functions. Then there exists L ∈ N, basis functions and weights such that
> `sup_{x∈χ} ( |V(x) − Wᵀσ(x)| + ‖∇ₓV(x) − Wᵀ∇ₓσ(x)‖ ) ≤ ε̄`

并解释为："also known as the Universal Approximation Theorem, states that a single layer neural network can simultaneously approximate a function and its derivative given a sufficiently large number of basis functions."

**【1869】Handbook of Reinforcement Learning and Control**
同样的"函数 + 梯度"诉求，用非正式写法：

> By the universal approximation theorem [60], any smooth function V can be approximated as `V = Wᵀφ(x) + ε(x)` on compact sets K and with any e-precision, i.e., there exists a sufficiently large p such that `sup_{x∈K} |ε(x)| ≤ e`. Naturally, this approximation property **extends to the gradient of V**.

---

### 2.2 一句话引用型（主流，占 60% 以上）

绝大多数控制论文只写半句就转入设计，典型三段式：**"因为 NN/FLS 是万能逼近器 → 所以 f(x) 可写成 Wᵀφ(x)+ε → 所以 ‖ε‖ ≤ ε̄"**。

**【609】Adaptive Output Feedback Tracking Control for Nonaffine Nonlinear Systems**

> Based on the universal approximation theorem of NNs, any continuous nonlinear function can be approximated with arbitrary degree of accuracy using a two-layer feedforward NN with sufficient hidden neurons [9], [25].

**【1360】Secure consensus control of multiagent systems with Byzantine attacks**
介绍得较"啰嗦"，同时把 NN 与 FLS 并列并解释两者关系（库中少见的"选型说明"）：

> Since the exact mathematical models of the multiagent systems are unknown, an adaptive approximation scheme is essential... In this work, RBF neural networks are used as the approximation tools, which are a class of feedforward networks utilizing RBFs as activation units to approximate any continuous function over **a compact set** with a simple linear structure. Similarly, fuzzy logic systems are also effective universal approximators... **By introducing proper fuzzy basis functions, fuzzy logic systems can be theoretically equivalent to RBF neural networks.** However, RBF neural networks are selected primarily for their concise algebraic structure and computational efficiency.

**【129】Adaptive Safety-Certified RL for Constrained Optimal Control**

> Thus, using the universal approximation theorem, the value function V*(η) can be approximated via **a single-layer NN over a compact domain** Ω ∈ Rⁿ as follows:

**【3208 / 3346】Adaptive Q-Learning for Data-Based Optimal Output Regulation**

> It is known that NN is an universal approximator [62], which can be employed to approximate any continuous function on a compact set.

**【3963】Adaptive NN Distributed Control for Time-Varying Networks**

> The usage of NNs as universal approximators of smooth nonlinear functions makes it possible to cope with the unknown nonlinear dynamics.

**【1543】Neurodynamic formation maneuvering control（合围/编队语境）**

> by virtue of the universal approximation theorem, NN is extensively exploited in consensus control applications because of the superior capabilities in unknown function approximations [22], including second-order nonlinear MASs, AUVs, multiple USVs, spacecraft...

> ⚠️ 观察：这一类引用的**引用链高度同质**——Hornik/Stinchcombe/White (1989)、Cybenko (1989)、Park & Sandberg (1991)、Wang (1992, fuzzy)、Ge & Wang、Lewis 等几篇被反复引，形成"引文回声室"。库中出现次数最高的三条：
> - Hornik, Stinchcombe, White, *Multilayer Feedforward Networks Are Universal Approximators*, Neural Networks 2(5):359–366, 1989（943、1170、1417、2816、3294、3298、3299、3305、3415、3742、3844、3205…）
> - Cybenko, *Approximation by superpositions of a sigmoidal function*, MCSS 2(4):303–314, 1989（276、609、1417、1682、3205、3298、3304）
> - Wang L.X., *Fuzzy Systems are Universal Approximators*, IEEE FUZZ 1992（3771、3773）

---

### 2.3 RBF / FLS 参数化落地型（控制向语料的"标准插件"）

定理一出现，紧接着必然是"理想权重 + 高斯基函数 + 有界逼近误差"三件套。

**【3804】《一类严格反馈系统可预设时间的保性能控制》(重庆大学博士论文) §2.5**
中文论文里最规范的一段（可作为写作模板）：

> 根据万能逼近定理可知，神经网络对连续函数具有固有的逼近能力。径向基函数神经网络以其简单的参数化结构，被广泛用于逼近未知连续的非线性函数 [72]–[74]。对一个在紧集 Ω_z ⊂ R^q 上的函数 f(Z): R^q → R^l，用 RBFNN 逼近函数 f(Z) 可得：
> `f(Z) = wᵀφ(Z) + δ(Z)`，其中 w 为未知理想权重矩阵，δ(Z) 表示 **固有的有界逼近误差**，满足 `‖δ(Z)‖ ≤ δ̄ < ∞`；φᵢ(Z) = exp[−(Z−χᵢ)ᵀ(Z−χᵢ)/(2ςᵢ²)]。

**【3773】《滑模变结构控制MATLAB仿真（第3版）》§9.1.2 RBF 网络原理**

> 由于 RBF 网络具有万能逼近特性 [1]，采用 RBF 神经网络逼近 f(x)，网络算法为
> `h_j = exp(‖x − c_j‖²/(2b_j²))`，`f = W*ᵀh(x) + ε`
> 其中 W* 为网络的**理想权值**；ε 为网络的逼近误差，`ε ≤ ε_N`。

（同一作者 2009 版【3771】写法更"科普"：RBF 是"具有单隐层的三层前馈网络……**已证明它能任意精度逼近任意连续函数**"。）

**【3773】§8.3.2 模糊逼近原理** 给出最优参数的定义式（很多论文直接照抄）：

> 由于模糊系统具有万能逼近特性 [2]，以 f̂(x|θ) 来逼近 f(x)。
> `θ* = arg min_{θ∈Ω} [ sup_{x∈R²} |f̂(x|θ) − f(x)| ]`，于是 `f(x) = θ*ᵀ ξ(x) + ε`，ε 为模糊系统的逼近误差。

**【775】Reactionless control of free-floating space manipulators**（与 `space_manipulator/` 笔记直接相关）
直接引 Park & Sandberg 的 RBF 版："According to Park and Sandberg [40], the continuous unknown function f̂(θ) approximately..."

> 📌 **关键差别（值得在综述里点明）**：这一范式里 UAT 的作用**不是**"保证一定能逼近"，而是**授权把未知函数参数化成 `W*ᵀφ+ε` 并假设 ε 有界**。定理的"存在性"被压缩成一个**建模假设**（Assumption: 存在理想权重 W\* 使 ‖ε‖ ≤ ε̄）。1965 的 Remark 4 承认了这一后果：
> > When universal approximators (e.g., fuzzy logic systems, neural networks) are used to approximate system nonlinearities and the corresponding approximation errors will **generate a positive constant appearing in the right side** of the inequality, which makes classical finite-time stability criteria fail.

---

### 2.4 教材化解说型（"为什么可以"的直观 + 历史注释）

**【3423】《理解深度学习》(Simon J. D. Prince 中译本) §3.2 通用逼近定理**
库里唯一"教学式"的讲解，走的是 **ReLU 分段线性 + 线性区域计数** 路线：

> 浅层网络中隐藏单元的数量是网络容量的衡量标准。使用 ReLU 激活函数，具有 D 个隐藏单元的网络的输出最多有 D 个关节，因此它是一个最多有 D+1 个线性区域的分段线性函数。随着我们添加更多的隐藏单元，模型可以近似更复杂的函数。
> 事实上，具有足够容量（隐藏单元）的浅层网络可以任意精度描述任何定义在实数线上紧凑子集的连续一维函数。……随着这些线性区域逐渐增多，它们代表的函数部分不断缩短，就可用线段越来越逼近目标曲线（图 3.5）。神经网络逼近任何连续函数的能力可以通过数学推理来证明，这称为通用逼近定理。

§3.7 注释给出**谱系与版本差异**（这正好解释库里为什么"通用逼近定理/万能逼近定理"两种译名并存）：

> 这个定理的通用版本指出，一个包含有限数量隐藏单元和一个激活函数的单隐藏层网络，可在 Rⁿ 的紧凑子集上以任意精度逼近任何连续函数。这一点已由 **Cybenko (1989) 针对 Sigmoid 激活函数**进行了证明，后来 **Hornik (1991)** 证明这一结论同样适用于更大类别的非线性激活函数。

并在 §4.5 立刻转向"**那还要深度干什么**"：

> 深层和浅层网络都可以模拟任意函数，但某些函数可用深层网络更高效地逼近。研究发现，对于某些函数，浅层网络需要**指数级**的隐藏单元才能达到与深层网络等效的函数。这种现象被称为神经网络的深度效率。

---

### 2.5 领域迁移型：UAT 被改写成别的定理

这是知识库里唯一"有体系"的一族（10 篇 PINN 综述 + KAN），UAT 在此**换了对象**：

| 迁移方向 | 文献 | 改写成什么 |
|---|---|---|
| 函数 → 函数+导数 | 3298、3302、1869、3792 | "guarantees that neural networks can approximate any smooth function **and crucially, its derivatives**, on compact domains"（3302） |
| 函数 → 算子 | 1364、3294、3300、3301 | DeepONet 基于"**universal approximation theorem of operators**"（Lu et al., Nat. Mach. Intell. 2021） |
| 函数 → 概率分布 | 3301 | "A universal approximation theorem of deep neural networks for **expressing probability distributions**" |
| 函数 → 结构选择 | 3747 / 3227 (Encyclopedia of Systems and Control) | 不是定理而是**选型准则**："The model structure should thus be **a universal approximator**." |
| MLP → KAN | 3710 | 用 **Kolmogorov–Arnold representation theorem** 替代 UAT 作为架构合法性来源 |

**【3299】PINNs: A didactic derivation** 提供了一条很实用的"讲必要性"的论证（激活函数非线性的必要性）：

> Without a non-linear activation, every layer would compute an affine transformation... The network would be limited to representing straight lines... The non-linear activation is precisely what breaks this limitation: the Universal Approximation Theorem [8] guarantees that a feedforward network with at least one hidden layer and a **non-constant, bounded, continuous activation function** can approximate any continuous function on a compact set to arbitrary accuracy, provided sufficiently many neurons. **Without the non-linearity, this result fails entirely.**

**【3710】KAN: Kolmogorov–Arnold Networks** 是库里唯一把 UAT 当"**靶子**"的文献：

> MLPs ... are the default models in machine learning for approximating nonlinear functions, due to their expressive power guaranteed by the universal approximation theorem [3]. **However, are MLPs the best nonlinear regressors we can build?**
> Whereas MLPs are inspired by the universal approximation theorem, KANs are inspired by the Kolmogorov–Arnold representation theorem.

并在正文表格里做正面对照（Model / Theorem / Formula）：

| | MLP | KAN |
|---|---|---|
| Theorem | Universal Approximation Theorem | Kolmogorov–Arnold Representation Theorem |
| 浅层公式 | `f(x) ≈ Σ_{i=1}^{N(ε)} aᵢ σ(wᵢ·x + bᵢ)` | `f(x) = Σ_{q=1}^{2n+1} Φ_q(Σ_{p=1}^{n} φ_{q,p}(x_p))` |

（隐含批评：UAT 只说"存在 N(ε)"，**不给 N 与 ε 的速率**；KAN 论文因此改用 B-spline 的 `CG^{−k−1+m}` 收敛率来谈 scaling law。）

---

### 2.6 历史/谱系型（"为什么是单隐层线性参数化"）

**【2518】The Control Handbook, 2nd ed.** 把 UAT 放进控制史叙事，解释了整个领域为何偏爱 **LIP（linear-in-the-parameters）单隐层网络**：

> In the late 1980s to early 1990s, the use of neural networks as universal approximators of unknown nonlinear functions led to the use of on-line parameter estimators to "train" or update the weights of the neural networks. **Difficulties in establishing global convergence results soon arose since in multilayer neural networks the weights appear in a nonlinear fashion**, leading to "nonlinear in the parameters" parameterizations for which globally stable on-line parameter estimators cannot be developed. This led to the consideration of single-layer neural networks where the weights can be expressed in certain parametric models that are convenient for estimation [20,21].

**【3773】第8章引言**：给出中文教材对"万能逼近器"的**分类学**（这句话几乎是国内自适应模糊控制教材的模板句）：

> 万能逼近定理表明模糊系统是**除多项式函数逼近器、神经网络逼近之外的一个新的万能逼近器**。模糊逼近器与其他逼近器相比，其优势在于能够有效地利用模糊语言的能力。**万能逼近器是自适应模糊控制的理论基础。**

**【3455】Robust Adaptive Control（书评）** 则把 UAT 当成"跨学科桥梁"：

> The tools for design and analysis presented in the book are very valuable in understanding and analyzing similar parameter estimation problems that appear in **neural networks, fuzzy systems, and other universal approximators**.

---

## 3. 三种"标配后续动作"（定理之后必然出现的东西）

| 动作 | 表述模板 | 代表文献 |
|---|---|---|
| ① 定义理想权重/最优参数 | `W* = arg min_{W∈Ω} sup |f − Ŵ|`、`θ* = arg min sup` | 3773 §8.3.3、3804 §2.5 |
| ② 假设误差有界 | `‖ε‖ ≤ ε̄`、`|ε| ≤ ε_max`，并在 Lyapunov 里取 `η > |ε|_max` | 3773 §8.3.3、2449 Remark 1、988 |
| ③ 用自适应律 + 鲁棒项吸收 ε | `θ̇̂ = γ s ξ(x)` + `sgn(s)` 项 | 3773 §8.3.3、3773 §9.1.3 |

**【2449】Remark 1** 是库里对"为什么可以假设 ε 有界"写得最清楚的一段：

> If ε = 0, then f(x) is in the functional range of the NN. In general for any positive constant real number ε_N > 0, f(x) is within ε_N of the NN range if there exist **finite hidden neurons N₂ and constant weights** so that for all inputs in the compact set, the approximation holds with ‖ε‖ < ε_N. For various activation functions, results such as the **Stone–Weierstrass theorem** indicate that any sufficiently smooth function can be approximated by a suitable large network. Therefore, the fact that the approximation error ε is bounded **follows from the Universal Approximation Property** of the NNs [1].

**【1766】Three-dimensional prescribed-time cooperative guidance** 把两个误差源拆开（很值得学的分析口径）：

> the size of this region is primarily determined by two factors: (1) the upper bound ε* dictated by the universal approximation theorem, and (2) the estimation error bound introduced by the NN when approximating the disturbance function. Here, **ε\* reflects the approximation capability of the NN structure, while the latter is determined by the deviation between the trained parameters and their optimal values**. ... the network's capacity can be enhanced by increasing the number of neurons in the hidden layer.

---

## 4. 边界条件与反方证据（知识库里"唱反调"的部分）

| 论点 | 出处 | 原文要点 |
|---|---|---|
| **非构造性**：只保证存在，不说怎么找 | 3710 | UAT 给出 `N(ε)` 的存在性，但不给 N–ε 速率，故转向 scaling law 讨论 |
| **浅层要爆炸的神经元数** | 3298 §2.2 | "shallow networks often require an **impractically large number of neurons**, motivating the use of deeper architectures" |
| **深度效率** | 3423 §4.5.3 / §4.5.2 | 某些函数浅层需指数级隐藏单元；深度网络每参数线性区域数更多 |
| **Barron 下界** | 2105、1674 | `(1/N)^{2/n}` 的逼近误差下界；NN 比多项式/三角/样条"更 efficient"，但下界仍在 |
| **逼近 ≠ 收敛** | 1201 (Swarm stability and optimization) | 用多项式或 NN 任意逼近 πᵢ(sᵢ)、cᵢ(sᵢ) 后，"convergence of the output errors to zero **is not guaranteed anymore**" |
| **破坏有限时间性** | 1965 Remark 4 | ε 在不等式右侧产生正常数，classical finite-time criteria fail |
| **UAT 不是万能药 → approximation-free 路线** | 478、3499 | "It does not require separate adaptive schemes or **universal approximators** such as neural networks or fuzzy logic systems adaptively tuned online"；把 UAT 方法列为"传统两类路线之一"再提出替代 |
| **维度灾难** | 3710、3296 | spline 的 COD；MLP 靠特征学习缓解但不消除 |

---

## 5. 术语与引文速查

**中文译名**（同一文献内也不统一）：

| 译法 | 出处 |
|---|---|
| 万能逼近定理 | 3773、3804 |
| 万能逼近（特性/性质） | 1161、1916、3773、3771 |
| 万能逼近器 | 3773（"新的万能逼近器"） |
| 通用逼近定理 | 3423（§3.2 标题）、3423 §3.7 |
| 万能近似器 | 3423（用于 normalizing flow："自回归流是万能近似器"） |
| 神经网络逼近定理 / 任意精度逼近 | 3771、1916（无专名） |

**英文变体**：`universal approximation theorem` / `universal approximator` / `universal approximation property` / `universal approximation theorem of operators`（DeepONet）/ `are universal approximators`（作谓语，Hornik 标题式）。

**被引最多的 5 条**（跨 79 篇统计）：

1. Hornik, Stinchcombe & White (1989), *Neural Networks* 2(5):359–366 — 至少 12 篇
2. Cybenko (1989), *Math. Control Signals Syst.* 2(4):303–314 — 至少 7 篇
3. Park & Sandberg (1991), *Neural Computation* 3(2):246–257（RBF 版）— 775、988
4. Wang L.X. (1992), *Fuzzy Systems are Universal Approximators* — 3771、3773（模糊路线源头）
5. Barron (1993), *IEEE TIT* 39(3):930–945（**带速率**的版本）— 1674、2105、2518、3215

---

## 6. 若要把这段写进自己的论文/综述（建议）

1. **引用要分层**，不要只引 Hornik：
   - 存在性 + 无速率 → Hornik 1989 / Cybenko 1989
   - RBF 专用 → Park & Sandberg 1991
   - 带速率/复杂度 → Barron 1993
   - 模糊系统 → Wang 1992 + Stone–Weierstrass
2. **必须写明 4 个条件**（库里多数论文漏掉的）：紧集、连续性、激活函数类型、隐藏单元有限但足够多。缺一条就等于在引用一个不成立的定理。
3. **区分"定理"与"假设"**：定理只给"∃W\*，ε 可任意小"；控制器设计里真正用的是"∃W\* 使 ‖ε‖ ≤ ε̄（ε̄ 未知常数）"这一**假设**。3804 §2.5、2449 Remark 1 是正确写法。
4. **梯度/导数场合要单独引**（ADP、自适应评判、PINN）：3792 Property 2.3 或 Han et al.；只引 Hornik 1989 不够。
5. **中文写作建议统一为"万能逼近定理（universal approximation theorem, UAT）"**，首次出现给英文，避免"通用/万能/万能近似"混用（库里 3423 一本书内就混用了三种）。
6. **不要用它论证"深度"或"精度"**：这两件事 UAT 都不管（3298 与 3423 §4.5 都明确说明），需要 scaling law / 深度效率的文献。

**对本仓库选题的可用结论**（合围、预设时间、space manipulator）：

- 合围/编队论文（1543、1965、2071、2816、3017、3282）里的标准写法：UAT → RBF/FLS → `‖ε‖≤ε̄` → 用鲁棒项或 `η > ε_max` 吸收。
- 若目标是**预设时间/有限时间**收敛，1965 的 Remark 4 是必须引用的"坑"：逼近误差会破坏经典有限时间判据，需要 PI/预设时间变换或自适应增益补偿。
- 若目标是**渐近**收敛（如 RISE、command-filter 类），2449、478、3499 提供了"NN 前馈 + 不连续反馈"与"approximation-free"两条对照路线，可用于引出贡献。

---

## 附录 A：全部命中文档（79 篇）

**A1. 严格陈述 / 教材型（7）**
`3761` Methods of Developing SMC（Thm 5.1, Stone–Weierstrass）· `3423` 理解深度学习（§3.2/§3.7/§4.5）· `3298` PINN review（§2.2）· `3299` PINN didactic · `3302` PINN survey · `3304` PINN for PDE review · `988` Congealed NN（Park & Sandberg）

**A2. 函数+梯度 / 值函数逼近（5）**
`3792`、`254`（Property 2.3）· `1869` Handbook of RL & Control · `129` Safety-Certified RL · `3205` actor–critic–identifier

**A3. 中文控制教材/学位论文（4）**
`3773` 滑模变结构控制MATLAB仿真(2016) · `3771` 同书(2009) · `3804` 重大博士论文 · `1161` 多无人系统追逃对策

**A4. RL / ADP / Q-learning（7）**
`3208`、`3346`（Adaptive Q-learning）· `3215` adaptive critic · `1417` Deep RL survey · `2105` NN optimal control of manipulators（Barron）· `2479`· `361`（RVFL, `O(C/√n)` 速率）

**A5. 模糊系统作为万能逼近器（11）**
`424` · `710` · `1248` · `1714` · `1847` · `1965` · `2071` · `3017` · `3761` · `3773` · `3822`

**A6. 控制论文"一句话引用"（约 30）**
`341` `478` `609` `943` `983` `1072` `1114` `1170` `1201` `1223` `1360` `1442` `1543` `1682` `1766` `2449` `2816` `3111` `3143` `3213` `3280` `3282` `3326` `3360` `3415` `3445` `3455` `3499` `3750` `3844` `3956` `3963` `3742` `2198` `3227/3747`

**A7. PINN / 算子 / KAN（10）**
`3294` `3296` `3300` `3301` `3305` `1364` `3710` `3695`·`3699`（KAN 应用，仅关键词命中）

**A8. 历史 / 数学原典 / 其他（5）**
`2518` Control Handbook · `3812` Principles of Mathematical Analysis（Stone–Weierstrass 原典）· `3455` Robust Adaptive Control（书评）· `3747/3227` Encyclopedia of Systems and Control · `1201` Swarm stability and optimization

---

## 附录 B：复现方式

```js
// grep_search: 中英双查（regex 默认开启）
grep_search({query: "万能逼近定理|通用逼近定理|万能逼近器", limit: 50})
grep_search({query: "universal approximat", limit: 50})        // 覆盖 theorem/approximator/property
grep_search({query: "Stone-Weierstrass|Cybenko|Hornik|Park and Sandberg|Barron", limit: 20})

// 定位章节后按标题抽取（避免被分页截断）
get_toc({id: 3423})
read_markdown({id: 3423, section: "通用逼近定理"})
read_markdown({id: 3773, line_start: 13155, line_count: 130})

// 只看某几篇文档内部（逗号/区间）
grep_search({query: "universal approximat", doc_ids: "2449,775,3769", max_context_chars: 1200})
```

> 备注：`grep_search` 结果按 doc id 升序、每篇只返回一个最佳片段，因此"命中篇数"不等于"出现次数"；同一文档内多次出现需用 `doc_ids` 限定 + `read_markdown` 逐段读。
