# 只用位置测量的 Euler–Lagrange 系统：文献调研

> **数据来源**：OKB-Assist 知识库（MCP 服务 `okb`）
> **检索方式**：`grep_search`（全文正则：`Euler-?Lagrange` / `仅位置测量` / `without velocity measurements` / `velocity-free` / `position-only` / `tachometer` 等 20+ 组模式）、`search_info`（题录）、`get_document_abstract`（摘要核校）、`get_document_info`（元数据）、`read_markdown`（方法精读）
> **整理日期**：2026-11-14
> 文中 `[OKB #id]` 为该文献在知识库中的文档 ID，可直接 `read_markdown(id=...)` 取原文。

---

## 0. 结论先行

一句话：

> **EL 系统"只用位置测量"的本质不是"少测一个状态"，而是把速度信息变成必须自行构造的估计量；因此所有方法的分歧点只有一个——速度从哪里来，以及这个估计是否进入稳定性证明。**

由此形成 5 条互不等价的技术路线（详见 §2），知识库中被明确证实"仅位置"的 EL 论文共 **19 篇**（含 1 本专著），分布如下：

| 层级 | 篇数 | 代表 |
|---|---|---|
| 多 EL 智能体一致性 / 编队 / 包含 | 6 | [OKB #173]、[OKB #3879]、[OKB #2636]、[OKB #1978]、[OKB #2815]、[OKB #2695] |
| EL 合围 / fencing | 2 | [OKB #883]、[OKB #2695] |
| 单机械臂 / 机械系统输出反馈 | 9 | [OKB #180]、[OKB #532]、[OKB #831]、[OKB #3238]、[OKB #2422]、[OKB #1911]、[OKB #3241]、[OKB #1844]、[OKB #3740] |
| 空间机械臂 / 非完整机器人（同族） | 4 | [OKB #151]、[OKB #1933]、[OKB #2391]、[OKB #75] |

**最重要的三个发现**：

1. **"异构 EL + 仅位置测量"在知识库中是空的**。异构 EL 的论文（[OKB #2023]、[OKB #2384]、[OKB #2832]、[OKB #2870]、[OKB #2411]、[OKB #3906]）全部默认速度可测或全状态反馈；仅位置测量的 EL 论文（[OKB #173]、[OKB #3879]、[OKB #1978]、[OKB #2636]）全部默认**同构**（同自由度、同结构，最多参数不同）。这两条线的交叉点即为可写的空白（§7）。
2. **仅位置测量 + 合围只有 2 篇**：[OKB #883]（AJC 2026，EL + 移动目标 fencing，只用相对位置）与 [OKB #2695]（黑龙江大学硕士论文，输入受限 EL 包含控制，不依赖相对速度）。这两篇是"位置-only 合围"最接近的先例。
3. **位置-only 的收敛性普遍弱一档**：全状态反馈下常见的"渐近/有限时间"在位置-only 下退化为"半全局 / 半全局预设时间 / UUB"。原因见 §2.4 的"半全局性来源"。

---

## 1. 问题界定：什么才算"只用位置测量"

EL 模型：

$$
M_i(q_i)\ddot q_i + C_i(q_i,\dot q_i)\dot q_i + G_i(q_i) = \tau_i
$$

"仅位置"在文献中有 **5 种强度不同**的表述，写作时必须明确选哪一种，否则对比会失真：

| 等级 | 测量假设 | 知识库中的典型表述 | 难度 |
|---|---|---|---|
| L0 | 位置 + 速度全测 | "full state feedback" | 基准 |
| L1 | 位置可测，速度**可由差分/已知模型**得到 | "without velocity sensors but with known dynamics" | 低 |
| L2 | 位置可测，速度**不可测且模型参数未知** | "using only position measurements" | 中 |
| L3 | **只有相对位置**（无全局坐标、无邻居速度） | "only relative position information is exchanged" | 高 |
| L4 | 位置本身含噪 / 只有部分分量 / 采样保持 | "limited model information", "sampled-data", "pose-only" | 最高 |

知识库明确落在 **L2/L3** 的 EL 论文：`[#173] #3879 #2636 #1978 #2815 #883 #2695 #180 #532 #831 #3238 #2422 #3241`；落在 **L4**：`[#151] #1844`；`[#1933] #2391` 属非完整机器人（同族但非标准 EL）。

> 注意：`[OKB #38] #232 #1955 #63 #3906 #2642 #624 #3232 #3236 #2814` 等 EL 论文虽频繁出现"无速度测量"字样，但其**上下文均为参考文献列表**，正文采用全状态或对领导者做状态观测，不属于本次范围（已逐篇核校摘要）。

---

## 2. 五条技术路线

### 2.1 路线 A：显式速度估计器 + 位置反馈控制器（"观测-控制分离"）

先构造 $\hat{\dot q}_i$，再代入常规协议。子类：

- **线性/高阶滤波（velocity filter）**：`[#173]` 用速度滤波器 + 三阶线性参考模型，把一致性转成轨迹跟踪；`[#3879]` 为每个智能体配一个速度估计滤波器，再叠加时变增益矩阵实现预设时间。
- **高增益观测器**：`[#2815]` 对领导者/跟随者分别配高增益观测器，外层配有限时间滑模估计器 + 自适应 NN 处理模型不确定性，收敛性为 UUB；`[#151]` 用高增益观测器估计空间机械臂速度，配 BLF + RBFNN。
- **二阶滑模 / 有限时间微分器**：`[#2636]` 先用"相对位置 + 相对速度"设计有限时间协议，再用二阶滑模观测器把相对速度替换为仅相对位置的版本。
- **降阶自适应速度观测器**：`[#3238]`（T-RA 1995）降阶 + 对未知参数自适应；`[#831]`（Automatica 2004）不连续速度观测器 + 不连续输出反馈，半全局渐近。
- **齐次性有限时间速度观测器**：`[#75]` 固定时间速度观测器 + 固定时间加权质心估计器 + 滑模面。

- **优点**：与既有全状态协议兼容，工程上可"打补丁"；观测器误差可单独分析。
- **代价**：**分离原理在非线性 EL 下不成立**，必须做级联/Lyapunov 联合证明；高增益带来峰值现象（peaking）与噪声放大，因此普遍只能得到半全局结论。

### 2.2 路线 B：能量整形 / 无源性（passivity-based）——不显式估计速度

控制器本身被设计成另一个 EL 系统，与对象通过势函数梯度互联，用阻尼注入代替速度反馈：

- `[#1978]`（IJRNC 2019，Cruz-Zavala / Nuño / Moreno）：势能与耗散函数取齐次形式 ⇒ **有限时间**收敛，无速度测量，leaderless + leader-follower 一致。
- `[#3740]`（专著）：Passivity-based Control of EL Systems，是这一路线的理论母本。
- `[#1933]`（TAC 2023）：port-Hamiltonian 框架 + **动态扩张（dynamic extension）**，把速度项消掉，仅位置测量下做轨迹跟踪与编队；可处理环状（ring）非无圈子图。

- **优点**：不需要精确模型，鲁棒性好；证明中速度误差**不出现**，避免级联问题；物理意义清晰（能量）。
- **代价**：需要 EL 的结构性质（反对称性 $\dot M-2C$、无源性）；对**有向图**和**输入饱和**较难扩展（`[#3879]` 在引言中明确指出该路线"主要限于无向网络"）。

### 2.3 路线 C：模型无关 / 自适应（model-independent）

不辨识模型、不估计参数，靠自适应增益 + 滑模/PI 吃掉未知项：

- `[#532]`（ICRA 2007，Su/Müller/Zheng）：**仅位置反馈的 PID**，全局渐近稳定（Lyapunov + LaSalle），显式给出增益条件，控制器不含模型信息。
- `[#2422]`（AJC 2015，Driessen）：观察器/控制器联合设计，全局**指数实用**稳定（GEPS），观测器更新不连续但力矩连续。
- `[#3241]`（TMech 2020）：把"关节速度测量不确定"当成待重构的量，自适应律只用关节位置 + 重构信息，含执行器故障，实验验证。
- `[#2695]`（2021 硕士论文）：EL 多机械系统在**输入受限**下的分布式包含控制，不依赖相对速度；后续章节用自适应神经网络做非线性包含与容错。

- **优点**：最贴近工程（PID 级实现复杂度）；对参数不确定/故障最鲁棒。
- **代价**：多为**调节/跟踪**问题而非协同问题；协同版本往往只能得到 UUB 或实用稳定，且增益整定靠试。

### 2.4 路线 D：预设/固定时间 + 位置反馈（时间规格化）

把"速度不可测"与"时间可指定"两个难点叠加：

- `[#3879]`（IEEE TIE 2026，Shi/Lin/Feng/Wang/Dong/Ren）：**预设时间时变编队 + 仅位置测量**，速度估计滤波器 + 时变增益矩阵，有向拓扑，**半全局**预设时间，含实验验证。
- `[#75]`（Nonlinear Dynamics 2024）：固定时间加权质心编队跟踪，无速度测量，含质心估计器。
- `[#2870]`：固定时间 EL 同步（全状态），可作为"若要位置-only 需补哪一环"的对照。

- **要点**：位置-only 下"半全局"几乎不可避免——因为高增益/滤波器的初始误差必须被先验界压住，才能保证不越界。

### 2.5 路线 E：安全/约束下的位置-only（最前沿、最薄弱）

- `[#1844]`（CDC 2023，Wang & Xu）：EL 系统 + 有限模型信息 + 测量不确定。把 EL 分解为 **proxy 子系统** 与 **virtual tracking 子系统**：前者用 CBF 保证受控不变性，后者用 BLF 保证安全速度跟踪误差有界。
- `[#151]`（Acta Astronautica 2021）：自由漂浮空间机械臂，BLF 保证预设性能，输出反馈用高增益观测器。

- **意义**：这条线把"位置-only"和"避碰/安全"接起来，正是**合围**所必需的两件事（包围 + 不撞），但知识库中 **"EL + 位置-only + 合围 + 安全"四者同时出现的论文为 0 篇**。

---

## 3. 多 EL 智能体：一致性 / 编队 / 包含（核心表）

| OKB | 文献 | 年 / 期刊 | 测量假设 | 关键技术 | 任务与收敛性 |
|---|---|---|---|---|---|
| [#173] | Meng, Mei 梅杰 哈工大, Miao, Wu, Ma — *Fully Distributed Consensus of Multiple Euler–Lagrange Systems Under Switching Directed Graphs Using Only Position Measurements* | 2023, **TAC** | L3（仅位置，且只交换相对位置） | 模型参考自适应一致性 + **速度滤波器** + 三阶线性参考模型 | 无领导一致；切换有向图、联合连通；**全分布式**（无需全局增益）；位置一致 |
| [#3879] | Shi, Lin, Feng, Wang 王庆 北航, Dong, Ren — *Prescribed-Time Time-Varying Formation for Multiple Euler-Lagrange Systems Using Only Position Measurements* | 2026, **IEEE TIE** | L2 | 虚拟系统 TVF + **速度估计滤波器** + 时变增益矩阵 | 半全局**预设时间**时变编队；有向拓扑；仿真 + **实验** |
| [#2636] | Zhao, Duan, Wen — *Distributed finite-time tracking of multiple Euler–Lagrange systems without velocity measurements* | 2015, IJRNC | L3 | 二阶滑模观测器替换相对速度；齐次性分析 | 有限时间跟踪；控制输入受限；动态领导者用分布式估计器 |
| [#1978] | Cruz-Zavala, Nuño, Moreno — *Finite-time consensus of Euler–Lagrange agents without velocity measurements via energy shaping* | 2019, IJRNC | L2 | **能量整形 + 阻尼注入**，齐次势能/耗散 | leaderless + leader-follower **有限时间**一致；控制器本身是 EL 系统 |
| [#2815] | Li, Zhang, He, Li, Ge — *Two-layer distributed formation-containment control of multiple Euler–Lagrange systems by output feedback* | 2019, **TCyb** | L2（无速度传感器） | 有限时间滑模估计器 + **高增益观测器** + 自适应 NN | 两层**编队-包含**；常值/时变编队；UUB |
| [#2695] | 徐廷铷 — *多智能体系统的分布式合围控制*（硕士论文） | 2021, 黑龙江大学 | L2/L3 | 输入受限下的分布式协议（不依赖相对速度）+ 观测器型自适应 NN | 多机械系统**包含（合围）** + 执行器偏移故障容错 |

**读法**：#173 解决"无全局增益 + 切换有向图"，#3879 解决"时间规格化"，#2636 解决"输入受限 + 动态领导者"，#1978 解决"不估计速度的有限时间"，#2815 解决"分层包含"，#2695 是唯一把三者（EL + 位置-only + 合围/包含）放在一起的中文工作。

---

## 4. 单机械臂 / 机械系统：输出反馈与速度观测器（方法源头）

| OKB | 文献 | 年 / 期刊 | 测量假设 | 关键技术 | 结果 |
|---|---|---|---|---|---|
| [#180] | Børhaug, Pettersen — *Global Output Feedback PID Control for n-DOF Euler-Lagrange Systems* | ACC 2006 | L2（仅位置） | 非线性观测器（**UGES** 误差动态）+ 反步 + 积分作用 | 用**级联系统理论**证明整体 UGES；"观测-控制分离"在 EL 下的经典反例与正解 |
| [#532] | Su, Müller, Zheng — *A Global Asymptotic Stable Output Feedback PID Regulator for Robot Manipulators* | ICRA 2007 | L2（仅位置反馈） | 纯 PID 结构 + Lyapunov + LaSalle 不变原理 | **全局渐近**；显式增益条件；控制器**不含模型信息** |
| [#2422] | Driessen — *Observer/controller with global practical stability for tracking in robots without velocity measurement* | 2015, AJC | L2 | 观测器/控制器联合；观测器更新不连续、力矩连续 | 全局**指数实用**稳定（GEPS），无需参数知识、扰动全局有界 |
| [#831] | Xian, de Queiroz, Dawson, McIntyre — *A discontinuous output feedback controller and velocity observer for nonlinear mechanical systems* | 2004, **Automatica** | L2 | 新的滤波器设计 + 不连续速度观测器 | 半全局渐近跟踪；观测器全局渐近 |
| [#3238] | Erlic, Lu — *A Reduced-Order Adaptive Velocity Observer for Manipulator Control* | 1995, **T-RA** | L2 | **降阶** + 参数自适应速度观测器 | 局部渐近（观测误差与跟踪误差）；PUMA-560 仿真，优于数值微分 |
| [#1911] | Hong, Xu, Huang — *Finite-time control for robot manipulators* | 2002, **S&CL** | L2（动态输出反馈） | 状态反馈 + **动态输出反馈**（位置驱动） | 有限时间调节/跟踪；把有限时间结论从二阶推广到高阶非线性 |
| [#3241] | Xiao, Cao, Xu, Liu — *Robust Tracking Control of Robot Manipulators With Actuator Faults and Joint Velocity Measurement Uncertainty* | 2020, **TMech** | L4（速度测量有不确定度） | 速度测量不确定度**重构律**（有限时间）+ 自适应 | 实际关节速度与故障先验知识均不需要；**真机实验** |
| [#151] | Yao — *Adaptive trajectory tracking control of a free-flying space manipulator with guaranteed prescribed performance and actuator saturation* | 2021, Acta Astronautica | L2/L4 | BLF + RBFNN + **高增益观测器** | 输出反馈下位置/速度误差收敛到小邻域，且满足预设性能与饱和 |
| [#1844] | Wang, Xu — *Safe Control of Euler-Lagrange Systems with Limited Model Information* | 2023, CDC | L4（测量不确定） | **proxy 子系统（CBF）+ virtual tracking 子系统（BLF）** | 有限模型信息下保证安全速度跟踪误差有界与受控不变性 |
| [#3740] | *Passivity-based Control of Euler-Lagrange Systems*（专著） | — | 理论 | 无源性/能量整形 + 阻尼注入总框架 | 路线 B 的母本，含"速度不可测时如何用位置构造阻尼"的一般论述 |

> **写作提示**：#180 的 UGES 观测器与 #831 的不连续观测器是"位置-only EL" 的两个标准答案（连续 vs 不连续）；#532 证明最朴素的 PID 也能全局渐近，因为 EL 的结构（$M\succ0$、$\dot M-2C$ 反对称、重力势有界）本身提供了足够阻尼；#2422 说明"观测器不必连续，但力矩最好连续"这一工程折衷。

---

## 5. 合围 / fencing / 包围：EL + 位置-only 的两篇半

合围类论文在知识库中很多（[OKB #2278] #231 #563 #773 #779 #852 #1111 #1976 等），但**绝大多数是二阶积分器或线性模型**；真正用 EL 动力学且只用位置测量的只有：

| OKB | 文献 | 年 / 期刊 | 模型 | 测量假设 | 关键技术 | 结论 |
|---|---|---|---|---|---|---|
| [#883] | Zhao, Guo, Yue, Tian, Sun, Sun — *Flexible distributed fencing control for Euler–Lagrange multi-agent systems with moving target* | 2026, **AJC** | EL（含未知动态参数） | **仅相对位置信息**（目标与邻居的相对位置） | 凸包内柔性 fencing + 速度匹配 + 避碰 + 稳定编队 + **无标签（label-free）** | 目标速度未知但**常值**；智能体与目标的速度匹配得以保证 |
| [#2695] | 徐廷铷，同 §3 | 2021，硕士论文 | EL 多机械系统 | 不依赖相对速度 | 输入受限 + 观测器型自适应 NN | 跟随者收敛到领导者凸包（**包含**语义的合围） |

**对照（非 EL，但同一问题结构，可作为技术借用来源）**：
- 二阶积分器 + 位置-only 合围/包围：[OKB #1111]（有/无速度测量两版协议）、[OKB #231]、[OKB #563]、[OKB #2278]（label-free 刚性编队）、[OKB #773]（label-free fencing）、[OKB #2860]（未知机动目标 + 相对位置估计器）。
- 双层 UAV 目标 fencing：[OKB #149]（相对测量）。
- 综述与专著：[OKB #1976]（*Multi-robot systems: Coordinated fencing control and applications*）。

> **差距点**：#883 只处理**常值速度**目标，且未考虑输入饱和与通信中断；#2695 是包含而非"抗逃逸"围捕。**"EL + 未知时变机动目标 + 仅位置 + 输入饱和 + 避碰"仍是空位**——这正是 `survey/heterogeneous/` 这条线可以推进的位置。

---

## 6. 与"异构"的交叉：为什么这是一个空位（也是本文放在此目录的原因）

### 6.1 知识库事实

| 论文族 | 异构性 | 是否位置-only |
|---|---|---|
| [#2023] 多异构 EL 任务空间预设时间编队跟踪 | 参数 + 任务空间 | ❌ 全状态 |
| [#2384] 分层预设时间异构 EL 编队跟踪 | 参数不确定 + 扰动 | ❌ 全状态 |
| [#2832] 异构 EL 有限时间编队跟踪（未知领导者） | 参数/动力学 + 未知领导者 | ❌ 全状态 |
| [#2870] 网络化不确定 EL 固定时间同步 | 参数不确定 | ❌ 全状态 |
| [#2411] 异构不确定 EL 领导者-跟随同步 | 异构 + 扰动 | ❌ 全状态（RISE） |
| [#3906] 异构 EL 采样数据保连通一致 | **结构异构**（heterogeneous dynamics） | ❌（保连通 + 采样，非位置-only） |
| [#173] [#3879] [#2636] [#1978] | **同构**（参数可不同） | ✅ |

即：**异构 ≠ 位置-only，两条线在知识库中没有交集**。

### 6.2 三条真正会"打架"的地方（写作时的贡献点）

1. **速度估计器要为不同自由度/结构各设计一次**：同构时一个滤波器模板可复用（[#173]）；异构时 $n_i$ 不同、$M_i$ 结构不同，滤波器阶数与增益无法统一 ⇒ "全分布式 + 无全局增益"更难。
2. **位置-only 把"输出"降到位置**，而异构下的凸包/编队参考本身只能定义在**输出空间**（参见 `heterogeneous_models_survey.md` §5.4 与 `heterogeneous_swarm_encirclement_challenges.md` §2.1）。于是"输出凸包 + 位置反馈"是唯一自洽的表述，但其**充要条件在知识库中尚属空白**。
3. **半全局性 vs 异构性**：位置-only 的证明普遍依赖"初始误差有界"（半全局）；异构初值分布更散，半全局前提更难满足 ⇒ 需要**预设时间/预设性能**把瞬态钉住（[#3879]、[#151] 是这个思路的两个样本）。

### 6.3 可直接复用的四个技术模块

| 需求 | 可借用 | 出处 |
|---|---|---|
| 无模型速度/阻尼构造 | 能量整形 + 阻尼注入（无需估计速度） | [#1978]、[#3740] |
| 有向图 + 全分布式 | 模型参考自适应 + 速度滤波器 | [#173] |
| 时间规格化 | 时变增益矩阵 + 预设时间滤波器 | [#3879] |
| 输入受限下的包含/合围 | 受限协议（不依赖相对速度） | [#2695] |
| 安全（避碰 + 速度约束） | proxy（CBF）+ virtual tracking（BLF）双层 | [#1844]、[#151] |
| 合围 + 无标签 + 未知参数 | 柔性 fencing + 自适应 | [#883] |

---

## 7. 选型建议（若要做"异构 EL + 仅位置 + 合围"）

1. **模型层**：EL 写 $M_i(q_i)\ddot q_i+C_i\dot q_i+G_i=\tau_i$，异构体现在 $(M_i,C_i,G_i)$ 与 $n_i$；若 $n_i$ 不同，合围只能定义在**位置输出空间**，不要试图做状态凸包。
2. **速度层**：优先 **能量整形**（[#1978]/[#3740]）——证明中根本不出现速度误差，天然适配异构（每个智能体用自身 EL 结构做整形）；若必须有向图与全分布式，退回 [#173] 的滤波 + 自适应参考模型。
3. **时间层**：用 [#3879] 的时变增益把收敛压到预设时间，否则半全局结论在异构初值下不可用。
4. **约束层**：用 [#1844] 的双子系统把避碰（CBF）与速度跟踪（BLF）解耦；合围的凸包约束可用 [#883] 的柔性 fencing 形式。
5. **可发表的空白**：把 2–4 组合，给出**异构 EL（$n_i$ 可不同）在仅位置测量下的输出凸包含/合围**判据，并处理未知时变机动目标 + 输入饱和。目前知识库无此类结果。

---

## 8. 检索方法与未覆盖项（可复核）

**用过的检索模式（`grep_search`，正则）**：
`Euler-?Lagrange`、`Euler.{0,3}Lagrange`、`only position measurements?`、`using only position measurements?`、`without velocity measurements?`、`without velocity information`、`no velocity measurement`、`velocity-free`、`velocity (measure|information|sensor|feedback)`、`position-only`、`position feedback`、`tachometers?`、`velocities are (not|un)`、`only positions are`、`仅位置测量`、`仅利用位置`、`无需速度测量`、`无速度测量`、`速度不可测`、`位置反馈`、`位置量测`、`欧拉-拉格朗日`、`heterogeneous Euler.Lagrange`、`Euler.Lagrange.{0,60}heterogeneous`；并配合 `search_info` 的题录短语检索（如 `finite-time tracking of multiple Euler-Lagrange systems without velocity` → 命中 [#2636]）。

**核校方式**：所有入选文献均读过 `get_document_abstract`；`#38 #232 #1955 #63 #3906 #2642 #624 #3232 #3236 #2814` 等因"无速度测量"字样仅出现在参考文献列表、正文采用全状态反馈或仅对领导者做状态观测而被排除。

**已知未覆盖 / 待补**：
- 教科书级结果：Berghuis & Nijmeijer *Global regulation of robots using only position measurements*（S&CL 1993，知识库仅在 [#532] 的参考文献中可见，未入库）。
- Immersion-and-invariance 速度观测器族（[#3879] 引言中的 [16]），知识库无全文。
- Na et al. 的微分器法、Yang et al. 的复合估计-控制（[#3879] 引言 [20][21]），未入库。
- 四旋翼/无人艇等"位置-only 输出反馈"文献（[#771] #557 #3348 #1135 #382 #3465 等）属二阶/欠驱动模型，未纳入本表，仅作对照。
- 未检索非英文（德/法）与 2026 年之后新上线预印本。
