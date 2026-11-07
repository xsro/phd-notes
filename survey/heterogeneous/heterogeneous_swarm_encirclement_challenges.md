# 异构集群合围（Heterogeneous Swarm Encirclement）难点调研

> **数据来源**：OKB-Assist 知识库（MCP 服务 `okb`，总文档 5533 条 / 已索引 3114 条）
> **检索方式**：`search_info`（题录/关键词）、`grep_search`（全文正则）、`get_toc` / `read_markdown`（章节精读）、`get_document_abstract`（摘要核校）
> **覆盖术语**：enclosing / encirclement / fencing / containment / formation-containment / 围捕 / 合围 / 包围 / 异构 / mixed-order / cross-domain
> **整理日期**：2026-11-07
> 文中 `[OKB #id]` 为该文献在知识库中的文档 ID，可直接用 `read_markdown(id=...)` 取原文。

---

## 0. 结论先行

只记一句话：

> **异构集群合围的难点不在"合围"本身，而在"异构"把每一个经典假设都打破了：状态维数不同使状态同步失去意义，动力学阶次不同使凸包/编队参考无法统一构造，平台能力不同使"谁来围、围多近、围多久"必须动态决策，而跨域介质又使通信与时间尺度不可比。**

由此派生出 9 类相互耦合的难点（详见 §2）：

| # | 难点 | 本质矛盾 | 对应理论缺口 |
|---|------|----------|--------------|
| 1 | 异构性建模与"合围"定义 | 状态空间不可比 → 只能谈输出合围 | 高阶/奇异/混合阶的输出合围充要条件 [OKB #3437] |
| 2 | 目标状态不可全测 | 只有部分智能体/部分状态可观测 | 预设时间/固定时间分布式观测器、二阶中心状态估计器 [OKB #2974] |
| 3 | 凸包与输出调节 | 领导者矩阵时变 → 调节器方程不可直接用 | 全分布式自适应观测器 [OKB #2941] |
| 4 | 约束同时在线 | 饱和/避碰/连通/视野 ⇒ 可行集可能为空 | 约束可行性与协议保守性 [OKB #3319, #1247, #2344] |
| 5 | 通信受限与对抗 | 时延、丢包、间歇、切换、DoS/Byzantine | 无界时延下输出合围 [OKB #2293] |
| 6 | 未知扰动/故障/饱和 | 异构平台扰动界不可比 | practical prescribed-time FTC for mixed-order [OKB #2847] |
| 7 | 收敛时间与瞬态性能 | 异构初值 ⇒ 固定时间上界难统一 | 预设时间/预设性能下的异构一致性 |
| 8 | 自组织与角色/构型分配 | 无预分配角度槽、成员可进出、多目标 | 自组织合围 + 动态重组 [OKB #1760, #2896] |
| 9 | 跨域协同与决策-控制耦合 | 速度/时间尺度差 2–3 个数量级；逃逸博弈 | 分阶段规划 + 分层博弈 [OKB #239, #3449, #92] |

---

## 1. 术语与问题界定（写作前必须统一）

知识库中同一件事至少有 6 个英文名、5 个中文名，综述若不先统一会重复计数。

| 术语 | 含义 | 关键区别 | 知识库代表 |
|------|------|----------|-----------|
| **enclosing / encirclement / circumnavigation** | 智能体绕目标形成闭合环（多为圆/椭圆），常含"旋转跟踪" | 强调几何环 + 角速度，可单目标 | [OKB #156, #1098, #2599, #3889] |
| **fencing（合围）** | 在目标周围形成刚性/半刚性凸包，即使目标逃逸也保持包围 | **凸包约束** + 目标未知速度估计 | [OKB #1976, #1247, #1760, #2846] |
| **containment（包含/包围）** | 跟随者进入由**多个领导者**状态张成的凸包内部 | 领导者是被围者（虚拟/真实），无"逃逸"概念 | [OKB #1593, #3319, #2231, #2728] |
| **formation-containment（编队-合围）** | 领导者先形成编队，跟随者再进入领导者凸包 | 分层：上层编队 + 下层包含 | [OKB #1725, #1737, #2941, #2559] |
| **formation tracking（编队跟踪）** | 形成队形并跟踪动态领导者轨迹 | 不含"包围"语义，但合围常以其为内环 | [OKB #2364, #3351, #4013] |
| **hunting / rounding-up / 围捕** | 任务级：搜索 → 分配 → 围捕 → 捕获 | 含任务分配与决策，控制只是其中一环 | [OKB #295, #92, #1161, #128] |

**异构**（heterogeneous）在知识库中出现 4 种互不等价的用法，合围问题里必须显式声明是哪一种：

1. **模型类别异构**：线性/非线性、Euler–Lagrange、欠驱动（UAV vs USV vs UUV）[OKB #2974, #3443]
2. **阶次异构（mixed-order）**：一阶 + 二阶 + 高阶混合 [OKB #3319, #2848, #1593]
3. **相对阶/状态维数异构**：\(n_i\) 不同 ⇒ 状态同步无意义，只能输出同步 [OKB #1231 *Output consensus of heterogeneous uncertain linear MAS*；参见 `survey/heterogeneous/heterogeneous_models_survey.md`]
4. **连续/离散时间异构（混杂）**：连续跟随者 + 离散领导者 [OKB #2728]
5. **能力异构**：感知范围、速度、续航、通信半径不同（跨域集群）[OKB #239, #2930]

---

## 2. 难点逐条拆解

### 2.1 难点一：异构使"合围"的定义本身需要重建

- 状态维数不同 ⇒ 无法直接定义状态凸包包含，必须退到**输出合围**。知识库综述明确指出这是该领域**尚未解决**的问题：
  > "如果高阶集群系统中智能体只能获得自身及邻居的输出信息并且只需要跟随者的输出进入到领导者输出形成的凸包内……高阶集群系统输出合围的分析和设计问题都尚未解决。" [OKB #3437，§1.3 "合围控制研究中存在的问题"]
- 奇异（广义）集群系统、带时延的奇异集群系统的合围分析"尚处于空白阶段" [OKB #3437]。
- 混合阶/混杂动态（连续体 + 离散体）的包含控制只有少量充要条件结果，且大多限定一阶/二阶 [OKB #2728, #3018, #2848]。
- **口径不统一**：fencing 用"刚性凸包"，containment 用"领导者凸包"，enclosing 用"半径/角度槽"，三者数学对象不同但文献常混用 ⇒ 综述对比时须逐篇归一。

| 文献 | 年 | 期刊 | 与本难点的关系 |
|------|----|------|----------------|
| *Saturated formation containment control for a heterogeneous multi-agent system with unknown perturbations* [OKB #3319] | 2024 | Automatica | 一阶+二阶智能体**均可为领导者或跟随者**，用误差型 GPIO 估扰 + 饱和函数 |
| *Cooperative target tracking-fencing control for heterogeneous MAS* [OKB #2974] | 2025 | Syst. Control Lett. | 显式提出"二阶中心状态估计器"以**解决异构平台状态维数不一致**，为低维智能体提供制导信息 |
| 异质多智能体系统的合围控制 [OKB #1593] | 2023 | 控制与决策 | 一阶/二阶异质，分两种"领航-跟随阶次组合"给出 4 类协议与判据 |
| 混杂多智能体系统的包围控制 [OKB #2728] | 2022 | 西安电子科技大学（博士） | 连续-离散混杂、仅位置测量的包含控制充要条件 |
| *Cooperative tracking control of heterogeneous mixed-order MAS with higher-order nonlinear dynamics* [OKB #2848] | 2022 | IEEE Trans. Cybern. | 阶次异构（1/2/高阶非线性），指出生成树须按**状态**而非按**智能体**定义 |

### 2.2 难点二：目标状态不可全测 ⇒ 观测器成为合围的前置条件

合围的本质是"用估计出的目标运动去闭合包围圈"，因此**观测器误差直接变成队形几何误差**。

- 分布式观测器需在**固定时间/预设时间**内收敛，否则合围初期队形破裂 [OKB #2846, #1760]。
- 目标加速度/控制输入未知 ⇒ 需 differential state observer / 扩张状态观测器 / 内模观测器 [OKB #1247, #67]。
- 只有**部分**智能体能直接测量目标（感知范围/视野限制）⇒ 需协同观测（cooperative target motion observer）[OKB #1760, #843]。
- 仅有方位（bearing-only）测量时的可观测性弱、需附加机动或通信 [OKB #3889, #2101, #3892]。
- 水下/跨域的"弱测量"场景中，测量野值、丢包与时延叠加，传统定位与跟踪方法难以持续稳定 [OKB #239，§2 "面临的难点与挑战"]

### 2.3 难点三：凸包/编队参考的构造 —— 输出调节框架的"卡点"

- 异构领导者矩阵不同且**被在线估计（时变）** ⇒ 经典调节器方程（regulator equations）无法直接用于控制器综合 [OKB #2941 摘要原文]。
- 全分布式要求：不能依赖拉普拉斯矩阵特征值、领导者输入上界、编队函数等全局信息 [OKB #2364, #2941]。
- 多领导者：部分跟随者无法直接访问任何领导者信息，须先重构领导者系统矩阵与状态 [OKB #2941, #339, #3316]。
- 时变编队的**可行性条件**（feasible formation set）在高阶异构下缺少显式表达 [OKB #3437]。

### 2.4 难点四：多重约束同时在线 ⇒ 可行性与保守性之争

这是工程落地最硬的难点，四类约束常常互相排斥：

| 约束 | 与合围的冲突点 | 代表文献 |
|------|----------------|----------|
| 执行器饱和 | 角度排斥项在接近目标时被饱和截断 ⇒ 收敛域缩小 | [OKB #3319, #2185, #1720, #2846] |
| 避碰（机间/机-目标） | 包围圈半径被安全距离下界顶住，目标可能"挤出"缺口 | [OKB #2846, #2165, #332, #2843] |
| 连通性保持 | 分散包围会拉长通信链路，与"均匀分布角度"目标冲突 | [OKB #1247, #1051, #1635, #800] |
| 视野/能见度 | 相机 FOV 限制 ⇒ 参考轨迹必须永久可见，与最小半径冲突 | [OKB #2344, #2758] |

要点：
- 现有做法多为"势函数负梯度 + 一致性项"的加权叠加（APF-style），**权重整定靠经验**，缺乏统一的可行性判据 [OKB #1247, #843]。
- NMPC/CBF 类方法能显式处理约束，但计算量与分布式实时性矛盾 [OKB #2165, #71, #2269, #3265]。
- 知识库综述把"考虑机器人位置不确定性和外部扰动的多机协同围捕"列为尚未形成统一研究框架的方向 [OKB #295，§5 展望 5)]。

### 2.5 难点五：通信受限与通信对抗

- 无界分布式传输时延下的输出包含控制 [OKB #2293]；间歇通信下的移动目标合围 [OKB #67]；非均匀水声时延下的自组织合围（见 §4 表）。
- 切换/有向拓扑下的时变编队跟踪：二阶系统的充分条件已有，高阶异构仍缺 [OKB #3437, #339, #3349]。
- 事件触发/量化通信：在合围场景下"触发阈值 ↔ 队形精度"缺少定量映射 [OKB #1857, #2185, #3817]。
- 恶意攻击：DoS 下的异构 UAV-UGV 编队安全 MPC [OKB #3646]；Byzantine 攻击下的异构编队最优跟踪 [OKB #3351]；图时序逻辑约束下的异构集群概率控制 [OKB #4376]。
- **知识库原文挑战**："海上通信组网能力弱，艇间信息交互困难……如何在高动态、不可靠的通信条件下构建强适应性的交互机制，是制约水面无人艇集群应用的首要挑战。" [OKB #3449，§1.3]

### 2.6 难点六：未知扰动、故障与参数不确定

- 异构意味着扰动/不确定的**界不可比**，统一增益往往被最差平台拖累。
- 主流补偿手段：GPIO（广义比例积分观测器）[OKB #3319]、扩张状态观测器 [OKB #2290]、模糊逻辑系统逼近未建模动态 [OKB #2846]、神经网络 [OKB #3349]。
- 执行器故障：混合阶异构的 practical prescribed-time 主动容错 [OKB #2847]；异构多智能体执行器故障下的攻防博弈 [OKB #3354]；异构船舶容错编队 + 避障 [OKB #1805]。
- 数据驱动路线可绕开模型异构：完全数据驱动的异构多领导者输出编队跟踪（含执行器故障）[OKB #2887]；事件触发数据驱动异构编队抗 DoS [OKB #3817]。

### 2.7 难点七：收敛时间与瞬态性能在异构下的"不可统一"

- finite / fixed / prescribed-time 三类，只有 prescribed-time 的收敛时间可**显式设为设计参数且与初值无关** [OKB #1570, #2836, #3441]。
- 异构系统初值不同、控制方向可能未知、执行器饱和 ⇒ 固定时间上界估计被放大，甚至不可行 [OKB #2847, #2849]。
- 预设性能（prescribed performance）与避碰/饱和叠加时，误差变换函数的可行性条件容易违反 [OKB #4, #3351]。
- 合围任务对"进入包围圈的时刻"敏感（早进入 = 被反制，晚进入 = 目标逃逸），因此**收敛时间可编程性**在合围中比在编队中更关键，但相关文献极少 [OKB #2758, #2974]。

### 2.8 难点八：自组织、角色分配与动态重构

- **无需预分配角度槽/虚拟领导者**：label-free / self-organized 设计 [OKB #1760, #2806, #1569, #3102（*Label-Free Formation Control of MAS with Bearing-Based SOM Neural Network*）]。
- **成员可进出**：合围过程中 UAV 可 join/exit 而队形不破 [OKB #1760]；plug-and-play 合围 [OKB #3116, #1857]。
- **多目标与动态半径**：多机动目标 + 动态半径的 plug-and-play 合围 [OKB #3116]；自适应协同合围 + 动态重组 [OKB #2896]；多目标安全合围 [OKB #3113]。
- **角色异构**：按动态能力为不同类型机器人分配不同角色（asymmetric self-play）[OKB #3807]；异构无人系统协同捕获中的 UAV/USV/UUV 分工 [OKB #2930]。
- 难点本质：**角色分配（离散决策）与合围控制（连续控制）的耦合**，目前多为分层但缺少稳定性保证的桥接。

### 2.9 难点九：跨域异构 —— 时间尺度、介质与决策-控制耦合

- 速度/续航差 2–3 个数量级：UAV 快速展开、USV 中继、UUV 抵近 ⇒ 路径规划呈**分阶段特性**，需无缝衔接 [OKB #239，§2 "未知海洋环境下动态协同规划难"]。
- 跨介质通信：水声信道"时-空-频"变、多普勒扩展、带宽受限 ⇒ 组网稳定性弱 [OKB #239，§2 第 3) 条]。
- 跨域分层控制架构：上层位置编队（UAV）+ 下层方位编队（USV），以 UAV 作移动领导者 [OKB #3443]；引导向量场法 [OKB #1946]；跨域受限通信下多目标跟踪 [OKB #663]。
- **对抗性任务中个体理性与群体一致性的冲突**：
  > "当无人艇集群执行拦截、围捕和资源竞争等强对抗性任务时……个体决策所对应的局部目标、行为偏好及风险权衡往往与系统整体目标存在冲突……如何在保持个体自主性的同时实现群体协同，已成为……关键问题。" [OKB #3449，§1.3 第 3) 条]
- 逃逸方的智能性被长期简化：现有研究多假设目标逃逸策略简单，缺少 intelligent escape strategy 的对手建模 [OKB #92，§6 (2)]。

### 2.10 难点十：从理论到实验（规模 × 实时性 × 异构硬件）

- 知识库综述明确点名："如何研发复杂协同场景下的异构多机器人围捕系统，包括高效的通讯方式、机器人异构性、协同手段等，有望成为未来的重点研究方向。" [OKB #295，§5 展望 2)]
- 大规模、异构、通信复杂、任务不确定下的**任务分配**仍缺有效算法；群体智能方法易陷局部最优 [OKB #295，§5 展望 4)]。
- 实验验证稀缺：多数异构合围结果止于数值仿真；带实物实验的仅少数（异构 UAV+UGV 编队 [OKB #3351]、多旋翼编队-合围 [OKB #1725]、UAV-USV 集群 [OKB #3443]）。

---

## 3. 难点 ↔ 文献映射速查

| 难点 | 首选精读（OKB ID） | 补充 |
|------|-------------------|------|
| 异构合围定义/建模 | #3437, #2974, #3319 | #1593, #2728, #2848, #3018 |
| 观测器/目标估计 | #2846, #1760, #1247 | #67, #2101, #3889, #3892 |
| 输出调节/凸包 | #2941, #2231, #2559 | #339, #3316, #2293 |
| 约束（饱和/避碰/连通/视野） | #1247, #2165, #1720 | #3319, #2344, #2758, #1051, #800 |
| 通信受限/攻击 | #2293, #3646, #3351 | #67, #3817, #4376, #3349 |
| 扰动/故障 | #3319, #2847, #2887 | #3354, #1805, #3349 |
| 收敛时间/性能 | #1570, #2836, #3441 | #2847, #4, #2758 |
| 自组织/重构 | #1760, #2896, #3116 | #2806, #1569, #1857, #3113 |
| 跨域异构 | #239, #3449, #92 | #3443, #1946, #663, #2930, #3163, #133 |
| 综述入口 | #295, #3437, #3449, #1976 | #92, #3838, #1570, #1142 |

---

## 4. 相关论文清单（按主题分表）

### 4.1 异构编队 / 编队跟踪（formation & formation tracking）

| 文献 | 年 | 期刊 | 贡献要点 | OKB |
|------|----|------|----------|-----|
| 异构集群系统分布式自适应输出时变编队跟踪控制 | 2021 | 自动化学报 | 领导/跟随动力学可完全不同；已知或未知领导输入、未知扰动、**有向拓扑切换**并存；摆脱对拉氏矩阵特征值与编队函数全局信息的依赖 | #2364 |
| *Resilient time-varying formation optimal tracking for heterogeneous MAS with coupled constraints* | 2025 | IEEE T-ASE | Byzantine 攻击下诚实智能体的**约束最优**编队跟踪；preview control + primal–dual；给出动态遗憾与约束违反上界；1 UAV + 4 UGV 实验 | #3351 |
| *Reinforcement learning-based optimized formation tracking control for heterogeneous AAV and ASV swarms* | 2025 | IEEE T-ASE | 双层架构：上层位置编队（虚拟领导者），下层**方位编队**（以 AAV 为移动领导者）；backstepping + actor-critic | #3443 |
| *Fully distributed output formation-containment control of heterogeneous MAS via adaptive observer approach* | 2026 | ISA Trans. | 部分跟随者无法访问领导者 ⇒ 自适应增益观测器重构领导者**系统矩阵与状态**；解决时变估计矩阵下调节器方程失效 | #2941 |
| *Distributed adaptive fixed-time formation tracking for heterogeneous MAS* | 2026 | Automatica | 异构固定时间编队跟踪 | #4013 |
| *Fully distributed finite-time formation tracking of heterogeneous MAS under directed networks* | 2025 | IEEE T-ASE | 领导输入未知、完全分布式有限时间 | #1920 |
| *Fixed-time formation tracking for heterogeneous MAS under actuator faults and directed topologies* | 2022 | IEEE TAES | 故障 + 有向拓扑下固定时间 | #524 |
| *Time-varying output formation tracking of heterogeneous linear MAS with multiple leaders and switching topologies* | 2019 | J. Franklin Inst. | 多领导者 + 切换拓扑 | #339 |
| *Time-varying formation tracking control of multi-leader multiagent systems with sampled-data* | 2024 | IEEE T-ASE | 采样数据下的多领导者编队跟踪 | #3316 |
| *Fully data-driven robust output formation tracking control for heterogeneous MAS with multiple leaders and actuator faults* | 2024 | IEEE Trans. Cybern. | 完全数据驱动，规避模型异构 | #2887 |
| 面向空地协同作战的无人机-无人车异构时变编队跟踪控制 | 2019 | 航空兵器 | UAV-UGV 跨域异构时变编队 | #1474 |
| 基于事件触发的异构无人机集群分布式编队控制研究 | 2021 | — | 事件触发 + 异构 UAV | #3803 |
| 异构多无人机自主集群算法研究 | 2023 | 沈阳航空航天大学 | 异构 UAV 集群算法 | #2490 |
| *Robust formation control for nonlinear heterogeneous MAS based on adaptive event-triggered strategy* | 2022 | IEEE T-ASE | 自适应事件触发 | #2300 |
| *Event-triggered data-driven formation control for heterogeneous MAS subject to DoS attacks* | 2025 | IEEE TCNS | 数据驱动 + DoS | #3817 |
| *Secure distributed MPC for heterogeneous UAV-UGV formation under DoS attacks* | 2025 | IEEE T-IV | 跨域异构 + 攻击 | #3646 |
| *Mutual-rendezvous control and feature-compatible landing optimization for heterogeneous UAV–USV fleets* | 2026 | IEEE TII | 异构跨域会合与降落 | #3086 |
| *Fault-tolerant cooperative formation control for heterogeneous ships: obstacle avoidance maneuvering* | 2026 | IEEE T-ASE | 异构船舶容错编队 + 避障 | #1805 |
| *Probabilistic control of heterogeneous swarms subject to graph temporal logic specifications* | 2023 | IEEE TAC | 图时序逻辑任务下的异构集群，去中心化可扩展 | #4376 |
| *Practical prescribed-time active fault-tolerant control for mixed-order heterogeneous MAS* | 2024 | Automatica | 混合阶 + 预设时间 + 主动容错（全驱动系统方法） | #2847 |

### 4.2 编队-合围 / 包含控制（formation-containment & containment）

| 文献 | 年 | 期刊 | 贡献要点 | OKB |
|------|----|------|----------|-----|
| *Saturated formation containment control for a heterogeneous MAS with unknown perturbations* | 2024 | Automatica | 一/二阶智能体**均为领导者或跟随者**；误差型 GPIO 估扰 + 饱和函数；领导者先成编队再带跟随者入凸包 | #3319 |
| *Formation-containment tracking intelligent control for heterogeneous swarm systems under unknown input* | 2023 | Sci. Sinica Tech. | 未知输入下异构集群编队-合围跟踪 | #2559 |
| *Distributed robust containment control of linear heterogeneous MAS: an output regulation approach* | 2022 | IEEE/CAA JAS | 输出调节视角的异构鲁棒包含 | #2231 |
| *Output containment control of heterogeneous linear MAS with unbounded distributed transmission delays* | 2022 | IEEE Trans. Cybern. | **无界**分布式传输时延 | #2293 |
| *Theory and experiment on formation-containment control of multiple multirotor UAV systems* | 2019 | IEEE T-ASE | 多旋翼编队-合围的**实物实验** | #1725 |
| *Model predictive formation tracking-containment control for multi-UAVs with obstacle avoidance* | 2024 | IEEE TSMC | MPC + 障碍规避的编队跟踪-包含 | #1737 |
| *Multi-agent path planning in complex multi-obstacle environment: RL-based formation containment* | 2026 | IEEE T-ASE | 强化学习 + 编队包含路径规划 | #2153 |
| *Prescribed-time disturbance observer-based fully distributed prescribed-time containment control* | 2024 | IEEE TCAS-II | 预设时间包含 | #64 |
| 离散多自主体系统输出时变编队-合围控制 | 2021 | 控制工程 | 离散时间输出编队-合围 | #2362 |
| *Finite-time containment control without velocity and acceleration measurements* | 2015 | Nonlinear Dyn. | 仅位置测量的有限时间包含 | #2421 |
| *Distributed containment control for multiple autonomous vehicles with double-integrator dynamics: algorithms and experiments* | 2011 | IEEE TCST | 双积分器包含 + 实验 | #2463 |
| 编队飞行航天器合围控制方法研究 | 2022 | 哈尔滨工业大学 | 航天器编队合围 | #2484 |

### 4.3 合围 / 围捕（fencing / encircling / hunting）

| 文献 | 年 | 期刊 | 贡献要点 | OKB |
|------|----|------|----------|-----|
| *Multi-robot systems: coordinated fencing control and applications*（专著） | 2025 | Springer | 合围控制系统化整理：邻域角度排斥、ISS、分布式目标估计 | #1976 |
| *Cooperative target tracking-fencing control for heterogeneous MAS* | 2025 | Syst. Control Lett. | 异构 MAS 的**跟踪-合围**；预设时间分布式观测器 + 未知输入补偿 + 二阶中心状态估计器解决**维数不一致** | #2974 |
| *Distributed observer-based formation-fencing control for multi-UAVs* | 2025 | IEEE TVT | 参数不确定/外扰/输入饱和；固定时间观测器；**连续切换函数**实现编队→合围的平滑过渡 | #2846 |
| *Cooperative target fencing control for UAV swarm with collision, obstacle avoidance, and connectivity maintenance* | 2024 | Drones | 无预定义精确队形；微分状态观测器；一套势函数同时兼顾**飞行安全与通信连通** | #1247 |
| *Self-organizing distributed target fencing of UAV swarm for an uncooperative target* | 2026 | Aerosp. Sci. Technol. | 目标运动估计 + 协同观测 + 自组织合围；**成员可 join/exit** | #1760 |
| *Distributed self-organizing fencing strategy with UAV swarm under incomplete information* | 2025 | Adv. Eng. Inform. | 不完全信息下的自组织合围 | #1569 |
| *Adaptive cooperative fencing and dynamic reorganization control of UAV swarm for multiple maneuvering targets* | 2026 | Complex Intell. Syst. | 多机动目标 + **动态重组** | #2896 |
| *Self-organizing fencing formation control of quadrotor swarms under actuator saturation* | 2026 | Aerosp. Sci. Technol. | 执行器饱和 + 有限感知；三层全分布式层次架构 | #1720 |
| *Target-fencing control of MAS with input saturation* | 2023 | ICoIAS | 输入饱和下的合围 | #2185 |
| *Cooperative target fencing safety control for multiple AAVs via NMPC* | 2026 | IEEE TCNS | 吸引-排斥机制锁住目标于**刚性凸包**；NLMPC-CBF 处理时延/扰动/障碍 | #2165 |
| *Plug-and-play distributed fencing of multi-vehicle systems for multiple uncooperative targets with dynamic radius* | 2026 | IEEE TVT | 多目标 + **动态半径** + plug-and-play | #3116 |
| *Plug-and-play distributed fencing of MAS for an uncooperative target without velocity measurements* | 2025 | IEEE T-ASE | 无速度测量的即插即用合围 | #1857 |
| *Adaptive distributed label-free fencing control of disturbed second-order MAS* | 2025 | ICRAS | **label-free**（无标签/无角色预分配） | #2806 |
| *Bearing-based moving target fencing control for multiple underactuated ASVs with fixed-time differentiators* | 2025 | J. Field Robot. | 方位测量 + 欠驱动 ASV | #2854 |
| *Internal model observer-based distributed moving-target-fencing control of multiple underactuated MSVs with intermittent communications* | 2024 | Ocean Eng. | 内模观测器 + **间歇通信** | #67 |
| *Cooperative moving-target enclosing of networked vehicles with constant linear velocities* | 2020 | IEEE Trans. Cybern. | 匀速运动目标的协同合围（经典 baseline） | #156 |
| *Theory and experiments on enclosing control of MAS* | 2021 | IEEE/CAA JAS | 合围控制理论与**实验** | #2088 |
| *Formation control for enclosing and tracking via relative localization* | 2025 | IEEE TAES | **相对定位**下的合围 + 跟踪 | #2842 |
| *Adaptive multirobot formation planning to enclose and track a target with motion and visibility constraints* | 2020 | IEEE T-RO | 非完整约束 + **相机视野**约束下的合围编队规划 | #2344 |
| *Prescribed-time target enclosing and tracking with motion and visibility constraints* | 2025 | IEEE Syst. J. | 预设时间 + 运动/视野约束 | #2758 |
| *Two-layer optimal collision-avoidance enclosing control for multiple satellites against an escaping target* | 2026 | Aerosp. Sci. Technol. | 弱机动卫星围捕**逃逸**目标；传感器故障 + 不完全信息；两层预设性能最优控制 | #3703 |
| *Three-dimensional cooperative enclosing guidance law with circling plane constraint* | 2025 | JGCD | 三维合围制导 + 环绕平面约束 | #1957 |
| *Bearing-only based enclosing control of nonholonomic MASs with a moving target of unknown velocity* | 2026 | Asian J. Control | 纯方位 + 未知速度目标 | #3889 |
| *Target enclosing control for nonholonomic MAS with connectivity maintenance and collision avoidance* | 2025 | arXiv | 连通性 + 避碰 | #2843 |
| *Decentralized multi-robot encirclement of a 3D target with guaranteed collision avoidance* | 2016 | Auton. Robots | 三维目标 + 避碰保证 | #332 |
| *Target encirclement with any smooth pattern using range-based measurements* | 2020 | Automatica | 任意光滑构型 + 距离测量 | #2599 |
| *MPC-based cooperative enclosing for nonholonomic mobile agents under input constraint and unknown disturbance* | 2023 | IEEE Trans. Cybern. | 输入约束 + 未知扰动 | #3265 |
| *Robust moving target enclosing control of MAS* | 2025 | IJRNC | 鲁棒合围 | #2255 |
| *Bioinspired bearing-based target enclosing control for UAV swarm* | 2025 | IEEE/ASME TMech | 仿生 + 方位 | #3891 |
| *Leader–follower interactive potential for target enclosing of perception-limited UAV groups* | 2022 | IEEE Syst. J. | 感知受限 UAV 群 | #843 |
| *Circular formation control of target enclosing for fixed-wing UAVs in 3D* | 2024 | IEEE T-IV | 固定翼三维圆编队合围 | #1098 |
| *Self-organizing multiagent target enclosing under limited information and safety guarantees* | 2024 | IEEE TAES | 有限信息 + 安全保证 | #1269 / #2604 |
| *Safe and adaptive multi-target encirclement with socially aware multi-robot control* | 2026 | IEEE RA-L | 多目标 + 社会性安全 | #3113 |
| *Cooperative encirclement hunting for space non-cooperative targets based on real-time reachable area prediction* | 2026 | Aerosp. Sci. Technol. | 可达域预测 + 航天器合围 | #3717 |
| *Encircling hostile spacecraft through constellation collaboration: a differential game approach* | 2024 | IEEE TAES | 微分博弈围捕 | #3733 |
| *Distributed entrapping control of MAS using bearing measurements* | 2021 | IEEE TAC | 方位测量的"困住"控制（entrapping） | #2101 |
| 多机器人协同围捕方法综述 | 2024 | 自动化学报 | 搜索 / 分配 / 规划 / 控制全链路综述 + 5 条展望 | #295 |
| 水面无人艇集群控制与博弈技术：研究进展与挑战 | 2026 | 中国舰船研究 | 通信 / 组织 / 博弈三大挑战 | #3449 |
| 海上跨域集群水下目标协同追踪：关键技术与展望 | 2026 | 自动化学报 | 弱测量定位、非合作目标跟踪、水声组网、跨域规划四难点 | #239 |
| 集群系统编队合围控制理论及应用（专著） | 2025 | — | 一致性/编队/编队跟踪/合围/编队-合围五类问题 + 各方向"有待解决的问题" | #3437 |
| Multi-UAV cooperative hunting with external disturbance | 2023 | Adv. Guid. Navig. Control | 全分布式多 UAV 围捕 + 扰动 | #2493 |
| 多约束条件下多无人系统追逃对策的编队与围捕控制方法研究 | 2025 | — | 多约束追逃 + 编队围捕 | #1161 |
| 动态多目标完全分布式定时定点包围编队控制 | 2026 | 国防科技大学学报 | 定时定点包围编队 | #2958 |
| 基于覆盖理论的高速强机动目标协同围捕策略 | 2024 | 工程科学学报 | 高速强机动目标 + 覆盖理论 | #128 |
| 基于社会化协同的无人机集群目标合围控制 | 2026 | 自动化学报 | 社会化协同机制 + 合围 | #224 |
| 基于覆盖控制的多机动目标合围拦截制导方法 | — | 飞行力学 | 覆盖控制 + 合围拦截制导 | #2059 |
| 拦截机动目标的分布式协同围捕制导方法 | 2022 | 航空学报 | 分布式协同围捕制导 | #2719 |
| 基于群体意志统一的无人机协同围捕策略 | 2022 | 北京航空航天大学学报 | 群体意志统一 | #3660 |
| 3 维未知复杂环境下无人机群协同围捕 | 2025 | — | 三维未知环境围捕 | #3722 |
| 基于狼群优化的卫星集群对空间目标围捕方法 | 2024 | — | 仿狼群优化 | #3663 |
| 无人艇集群协同合围理论与应用研究 | 2022 | 华中科技大学（博士） | USV 集群合围理论与实践 | #1897 |
| 多四旋翼无人机的分布式合围控制研究 | 2023 | 江苏科技大学 | 四旋翼分布式合围 | #626 |
| 多智能体对非合作目标的合围控制 | 2022 | 南京邮电大学 | 非合作目标合围 | #2727 |
| 多智能体系统的分布式合围控制 | 2021 | 黑龙江大学 | 分布式合围 | #2695 |
| 基于覆盖理论的高速强机动目标协同围捕策略 | 2024 | 工程科学学报 | 覆盖理论围捕 | #128 |
| 多无人艇协同围捕智能逃跑目标方法研究 | 2023 | 中国舰船研究 | 智能逃逸目标 | #1438 |
| 无人艇编队预设时间多目标协同包围跟踪控制 | 2025 | 中国舰船研究 | 预设时间 + 多目标 | #284 |

### 4.4 异构跨域捕获 / 围捕（cross-domain capture）

| 文献 | 年 | 期刊 | 贡献要点 | OKB |
|------|----|------|----------|-----|
| *Underwater target capture based on heterogeneous unmanned system collaboration* | 2024 | IEEE T-IV | UAV+USV+UUV 异构系统；通信与能量约束下的**全覆盖路径规划** | #2930 |
| *Cooperation detection and tracking of underwater target via aerial–surface–underwater vehicles* | 2025 | IEEE TAC | 测度论构造异构探测模式，通过调整 USV-UUV 队形最大化探测概率；DDPG 跟踪 | #3163 |
| *Asymmetric self-play-enabled intelligent heterogeneous multirobot catching system* | 2023 | IEEE T-RO | 按动态能力分配角色 + 非对称自博弈训练多类型机器人协同抓捕 | #3807 |
| *Cooperative multi-agent path planning for heterogeneous UAVs in contested environments* | 2026 | JGCD | 低优先级 UAV 探测/定位威胁，高优先级 UAV 突防；加权与广义 Voronoi | #3651 |
| *Cooperative path planning for heterogeneous UAV swarms: a Stackelberg game approach* | 2025 | IEEE T-ASE | 异构 UAV 集群路径规划的 Stackelberg 博弈 | #1467 |
| *Distributed task allocation with minimum makespan for heterogeneous multiplayer pursuit-evasion games* | 2024 | IEEE TAC | 异构追逃的分布式任务分配 | #2725 |
| *An isochron-based solution to pursuit-evasion games of two heterogeneous players* | 2024 | IEEE TAC | 异构双人追逃等时线解 | #2724 |
| *Robust circumnavigation of a heterogeneous multi-agent system* | 2021 | Auton. Robots | 异构系统的鲁棒环绕 | #2370 |
| *A remark on collective circular motion of heterogeneous multi-agents* | 2013 | Automatica | 异构集群集体圆周运动（合围的几何基础） | #2802 |
| *Distributed self-organizing control for cross-domain unmanned swarm multi-target tracking under constrained communication* | 2026 | Aerosp. Sci. Technol. | 跨域集群多目标跟踪 + 受限通信 | #663 |
| *Coordinated navigation control of cross-domain unmanned systems via guiding vector fields* | 2024 | IEEE TCST | 引导向量场的跨域协同导航 | #1946 |
| *A survey of cross-domain robotics: classification, perception, planning and control* | 2026 | IEEE/ASME TMech | 跨域机器人综述（分类/感知/规划/控制） | #133 |
| 空地跨域无人系统低空协同立体监测与控制 | 2026 | 控制与决策 | 空地跨域协同监测 | #347 |
| 基于空-海-潜跨域无人平台协同的海上目标探测追踪策略 | 2026 | 自动化学报 | 空-海-潜协同探测追踪 | #17 |

### 4.5 方法学支撑（理论/工具类，非合围专属）

| 文献 | 年 | 期刊 | 用途 | OKB |
|------|----|------|------|-----|
| *Fixed-time and prescribed-time consensus control of MAS and its applications: a survey* | 2023 | IEEE TII | 收敛时间理论的综述入口 | #1570 |
| *Bearing-based multi-agent formation control: a survey and taxonomy* | 2026 | Annu. Rev. Control | 方位编队分类学（合围常用） | #3838 |
| *Zero-sum pursuit-evasion differential games with many objects: survey* | 2017 | Dyn. Games Appl. | 多人追逃博弈综述 | #1142 |
| *A survey on collaborative hunting with robotic swarm* | 2024 | Neurocomputing | 搜索/分配/路径规划 + 4 条未来方向 | #92 |
| *A survey on techniques in the circular formation of MAS* | 2021 | Electronics | 圆周编队技术综述 | #2584 |
| *Passivity-based decentralized strategy for generalized connectivity maintenance* | 2013 | IJRR | 连通性保持 | #837 |
| *Distance-based adaptive formation control with guaranteed collision avoidance and connectivity maintenance* | 2024 | IEEE TCNS | 避碰 + 连通性同时保证 | #1146 |
| *Distributed control barrier functions for global connectivity maintenance* | 2024 | ICRA | CBF 全局连通 | #800 |
| *Nonsmooth CBF design of continuous constraints for network connectivity maintenance* | 2023 | Automatica | 非光滑 CBF | #1531 |
| *Connectivity preservation control for multiple UAVs in the presence of bounded actuation* | 2024 | ISA Trans. | 有界执行下的连通保持 | #1051 |
| *Prescribed-time formation tracking of second-order multi-agent networks with directed graphs* | 2023 | Automatica | 预设时间编队跟踪基准 | #2836 |
| *Adaptive fuzzy prescribed-time formation control for nonlinear MAS* | 2025 | IEEE T-ASE | 模糊 + 预设时间 | #3441 |
| *Finite-time consensus for heterogeneous MAS with mixed-order agents* | 2015 | Int. J. Syst. Sci. | 混合阶一致性 | #3018 |
| *Hierarchical consensus of constrained second-order MAS with application to formation of multiple mobile robots* | 2026 | IEEE TAC | 分层一致性 + 约束 | #41 |
| *A three-layer formation maneuvering control for MAS with dynamic collision avoidance* | 2026 | IEEE TAC | 三层编队机动 + 动态避碰 | #3075 |
| *Safe reinforcement learning based agile satellite attitude control … actuator faults* | 2026 | Aerosp. Sci. Technol. | 安全 RL + 非合作目标跟踪 + 故障 | #1791 |
| *Control barrier function-based collision avoidance guidance for multi-fixed-wing UAV pursuit-evasion* | 2024 | Drones | CBF + 追逃避碰 | #1810 |
| *Safety-aware pursuit-evasion game based on CBF and RL* | 2025 | IEEE TSMC | CBF+RL 安全追逃 | #2269 |

---

## 5. 知识库原文摘录：现存"未解决问题"清单

以下为知识库中**综述/专著自述的开放问题**，可作为选题直接引用（均已核校原文）。

### 5.1 来自《集群系统编队合围控制理论及应用》[OKB #3437，§1.3]

1. **一致性**：非一致时变延迟 + 拓扑不确定性 + 外部扰动**同时存在**时的高阶一致性问题未深入研究；一致函数的显式表达式未给出。
2. **编队**：时变编队研究少；高阶线性定常系统的可行时变编队条件、可行集扩展、编队参考显式表达、运动模态配置均未解决；带延迟时变编队的可行性条件未知；自适应协议摆脱全局信息依赖仍是难题。
3. **编队跟踪**：多领导者情形比单领导者复杂得多，缺少能"既保队形又跟多领导者"的分布式协议。
4. **合围**：高阶多可配置模态领导者的**充要条件**未知；**输出合围**的分析与设计"尚未解决"；**奇异（广义）集群系统**的合围研究"鲜有报道"，带延迟的奇异系统合围"尚处于空白阶段"。
5. **编队-合围**：现有成果主要针对低阶系统；高阶线性定常系统的状态/输出编队-合围条件与控制器设计"还未被充分研究"，**缺少统一定义框架与分层耦合协同关系的深入分析**。

### 5.2 来自《多机器人协同围捕方法综述》[OKB #295，§5]

1. 通讯约束下的多机器人协同围捕（中继节点方案）
2. **异构多机器人协同围捕**（子任务分类 → 不同机器人类型；"如何研发复杂协同场景下的异构多机器人围捕系统，包括高效的通讯方式、机器人异构性、协同手段等"）
3. 基于学习的目标搜索与任务分配（计算量大是主要障碍）
4. 复杂环境下基于深度学习的大规模围捕任务分配（群体智能易陷局部最优）
5. 考虑**机器人位置不确定性和外部扰动**的协同围捕（"尚未形成统一的研究框架"）

### 5.3 来自《水面无人艇集群控制与博弈技术》[OKB #3449，§1.3]

1. 海上通信组网弱、艇间交互困难（时延/丢包/干扰/信道攻击 + 静态拓扑不适应）
2. 复杂海况 + 欠驱动 + 多重约束、**规模扩大 + 平台异构 + 突发故障**共同削弱传统控制策略适用性
3. 强对抗任务中**个体理性与群体一致性冲突**，一致性误差框架难以刻画个体间相互影响

### 5.4 来自《海上跨域集群水下目标协同追踪》[OKB #239，§2]

1. 弱测量场景下高精度实时定位难（GNSS 不可用、测量野值、参考受限、惯性累积误差、能耗受限）
2. 非合作目标跟踪精度与稳定性难以兼顾（高机动、无主动反馈、误差恶性循环）
3. 恶劣水声信道跨介质组网稳定性弱（1500 m/s、指数衰减、多径、"时-空-频"变、多普勒）
4. 未知海洋环境下动态协同规划难（跨介质性能差异 → 分阶段规划；模糊信息 + 逃逸博弈）

### 5.5 来自《A survey on collaborative hunting with robotic swarm》[OKB #92，§6]

1. Heterogeneous collaborative distributed hunting（跨介质目标 ⇒ 需异构分布式围捕体系）
2. Intelligent escape strategy（现有研究的目标逃逸策略过于简单）
3. Multi-agent multi-task deep RL（搜索/分配/追踪多子任务导致训练难度剧增）
4. Brain-like intelligence（RL 的维度灾难与算力/能耗问题）

---

## 6. 可切入的研究空白（归纳，非知识库原文）

按"理论新颖性 × 可验证性"排序，供后续选题：

| 空白 | 为什么是空白 | 可能的切口 |
|------|--------------|-----------|
| **混合阶异构系统的输出合围充要条件** | 专著明确称"尚未解决" [OKB #3437]；现有结果多为充分条件或限定一阶/二阶 | 用输出调节 + 混合阶状态变换，构造统一凸包刻画 |
| **编队-合围的统一分层框架** | "缺少统一定义框架以及对分层耦合协同关系的深入分析" [OKB #3437] | 上层编队流形 + 下层凸包收缩率，给出耦合稳定性条件 |
| **饱和/避碰/连通/视野四约束的联合可行性** | 现有做法靠势函数加权整定，无可行性判据 [OKB #1247, #295] | 用 CBF 的可容许控制集交集空/非空作为可行性判据 |
| **合围进入时刻的可编程性** | 预设时间理论成熟但合围中几乎未用；[OKB #2974, #2758] 刚起步 | prescribed-time 观测器 + prescribed-time 凸包收缩 |
| **异构平台的维数桥接** | 仅 [OKB #2974] 提出"二阶中心状态估计器"解决维数差异，属孤例 | 推广为一般维数桥接算子，给出桥接误差界 |
| **角色/构型分配与控制的一体化稳定性** | 分层但无稳定性桥接；成员 join/exit 仅仿真 [OKB #1760, #3807] | 切换系统 + 混合自动机建模，给出驻留时间条件 |
| **有逃逸智能的对手建模** | "现有逃逸策略过于简单" [OKB #92] | 博弈 + 学习型对手，评估围捕策略的最坏情形 |
| **跨域分阶段规划的衔接时刻优化** | 只在综述里被点名 [OKB #239] | 切换时刻作为决策变量，最小化暴露时间 |
| **规模扩展性（>20 平台）的分布式合围** | 实验多为 3–10 平台 [OKB #3351, #1725] | 分层聚类 + 局部凸包嵌套 |

---

## 7. 检索方法与可复现记录

### 7.1 用到的 okb 工具

```text
search_info(query, fields=[id,title,year,journal], limit)   # 题录检索（中英双语，标题/作者/期刊/关键词/摘要）
grep_search(query, regex=true, limit, context)              # 全文正则（可限定 year_start/year_end、doc_ids）
get_toc(id)                                                 # 章节目录
read_markdown(id, section=..., line_start=..., line_count=...)  # 精读某节
get_document_abstract(id)                                   # 摘要核校
```
> 注意：`search_info` 是**元数据匹配**，命中数偏少（很多论文标题不含目标词但正文强相关），因此本调研对关键结论均用 `grep_search` 或 `read_markdown` 回查过正文。

### 7.2 主要检索式

| 中文 | 英文 |
|------|------|
| 合围、围捕、包围、异构 | heterogeneous, heterogeneous swarm/formation/multi-agent |
| — | enclosing, encirclement, fencing, containment, surrounding, entrapping |
| — | formation containment, formation tracking, mixed-order, cross-domain unmanned |
| — | bearing-only, connectivity maintenance, collision avoidance, obstacle avoidance |
| — | self-organizing, pursuit-evasion, target enclosing, multiple leaders |
| — | actuator fault, DoS attack, switching topology, input saturation |

### 7.3 已精读全文/章节的文档

| OKB ID | 文档 | 精读位置 |
|--------|------|----------|
| #3437 | 集群系统编队合围控制理论及应用 | §1.2.3–§1.4（含 §1.3 五类"有待解决的问题"） |
| #295 | 多机器人协同围捕方法综述 | §4 其他关键技术、§5 总结与展望 |
| #92 | A survey on collaborative hunting with robotic swarm | §6 Future works |
| #3449 | 水面无人艇集群控制与博弈技术：研究进展与挑战 | §1.3 面临的挑战 |
| #239 | 海上跨域集群水下目标协同追踪 | §2 面临的难点与挑战 |
| #1976 | Multi-robot systems: Coordinated fencing control and applications | 目录 + Ch.1–3 结构 |

### 7.4 局限

- `search_info` 的召回受题录质量影响；okb 中约 1483 条文档仍为 `uploaded` 状态（未索引），可能遗漏。
- 部分中文会议/学位论文无摘要，贡献描述取自标题与目录，已在上表以"要点"而非"贡献"措辞区分。
- 同一文献在库中可能重复（如 #1269 / #2604、#3707 / #3721 / #3713、#3714 / #2308），引用时建议以 ISSN/DOI 去重后再定稿。

---

*文件：`survey/heterogeneous/heterogeneous_swarm_encirclement_challenges.md`*
