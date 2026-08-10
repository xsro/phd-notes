# 编队控制 / 编队跟踪控制中的碰撞避免方法调研

> 调研目的：梳理 formation control（编队控制）与 formation tracking control（编队跟踪控制）中
> 如何实现碰撞避免，并**严格区分三类场景**：
> 1. **Agent 间互碰（inter-agent collision）**——智能体互相躲避；
> 2. **静态障碍物（static obstacles）**——位置固定的障碍物；
> 3. **动态障碍物（dynamic obstacles / 移动障碍）**——位置随时间变化的障碍物。
>
> 资料来源：控制理论文献知识库 `okb-assist`（MCP），辅以经典文献与综述。括号内 `doc:` 为该文献在
> 知识库中的文档编号，可通过 MCP 的 `read_markdown` 进一步查阅。

---

## 1. 一个重要的术语区分

在很多论文中，"碰撞避免"与"避障"被不加区分地使用，但严格的文献会明确区分两者。
最典型的界定来自 Wang 等（2007）《Cooperative UAV Formation Flying With Obstacle/Collision Avoidance》：

> "The phrase 'collision avoidance' is used in this brief to refer to a UAV trying to avoid another UAV,
> and the expression 'obstacle avoidance' represents the scenario where the UAVs try to avoid obstacles."
> （`doc:3311`）

即：
- **collision avoidance = agent ↔ agent**（多机互碰）；
- **obstacle avoidance = agent ↔ 障碍物**（再细分为 static / dynamic）。

这一区分是本调研的主线。三者面临的**信息假设、预测需求、实时性要求**都不同：

| 维度 | Agent 间互碰 | 静态障碍物 | 动态障碍物 |
|---|---|---|---|
| 障碍物是否运动 | 是（均为主动体） | 否 | 是 |
| 是否需要预测 | 需预测对方运动（互协商） | 不需要 | 需预测其未来轨迹 |
| 交互性 | 双向/互惠（reciprocal） | 单向 | 多单向（障碍不配合） |
| 典型难点 | 振荡、deadlock | 局部极小、狭窄通道 | 实时反应、轨迹预测 |
| 常用方法 | APF、ORCA/RVO、CBF、NavFn | APF、流场、NavFn、MPC | VO/ORCA、CBF、序贯凸规划、增强APF |

---

## 2. 主流方法分类与三类场景的覆盖能力

下面按"方法"组织，并标注其天然适合处理哪类碰撞。一张总览表先行：

| 方法 | Agent 间 | 静态障碍 | 动态障碍 | 安全性保证 | 实时性 | 备注 |
|---|:---:|:---:|:---:|:---:|:---:|---|
| 人工势场 APF | ✅ | ✅ | ✅(需预测) | 启发式 | 高 | 易局部极小 |
| 速度障碍 VO / RVO / ORCA | ✅✅ | ✅ | ✅✅ | 几何充分 | 高 | 专为互碰与动障设计 |
| 控制障碍函数 CBF / CLF-CBF-QP | ✅ | ✅ | ✅ | 硬约束(前向不变) | 高 | 现代主流 |
| 导航函数 Navigation Function | ✅ | ✅ | △ | 近全局收敛 | 高 | 易陷局部极小时更优 |
| 模型预测控制 MPC | ✅ | ✅ | ✅ | 约束内最优 | 中~低 | 计算随规模增大 |
| 流场 / 数值流场 | △ | ✅ | △ | 启发式 | 高 | 适合静态密集障碍 |
| 序贯凸规划 (SCP) | ✅ | ✅ | ✅ | 局部最优 | 中 | 整体编队参数优化 |
| 学习型 (DRL) | ✅ | ✅ | ✅ | 经验性 | 高(推理) | 需训练/泛化 |

---

## 3. 各类方法详解

### 3.1 人工势场法（Artificial Potential Field, APF）

**核心思想**：把"吸引力"指向目标/期望编队位置，"排斥力"来自障碍物与其它 agent；控制量取合势场的
负梯度。APF 的突出优点是**一套框架同时覆盖三类场景**——只需把"其它 agent"和"障碍物"都建模成
排斥势场源（`doc:930`, `doc:3207`, `doc:1547`）。

- **Agent 间互碰**：agent 之间施加相互排斥势，靠近时斥力剧增（`doc:1575` Adaptive formation control using artificial potentials）。
- **静态障碍**：障碍物视为高势点，进入预设范围即触发斥力绕行（`doc:3207` Wen 等 2018，将 APF 用于一类随机多智能体系统的编队+避障）。
- **动态障碍**：把移动障碍的当前/预测位置当作时变斥力源；增强型 APF（Enhanced PF）专门处理动态环境
  （`doc:1213` Choi 等，dynamic environment 下 UAV 避碰；`doc:943` 用 APF 专家策略做模仿学习）。

**局限**：局部极小、势场参数整定困难、狭窄通道易卡死。文献中常以**导航函数**或**流场**缓解局部极小。

### 3.2 速度障碍族：VO / RVO / ORCA

这类方法本质上**同时解决 agent 间互碰与动态障碍**，是在速度空间中做几何约束的经典方案。

- **Velocity Obstacle (VO)**：Fiorini & Shiller 提出，最初用于"在（被动）移动障碍中导航"
  （`doc:507`）。它把"未来会碰撞的相对速度集合"定义为速度障碍锥。
- **Reciprocal VO (RVO) / Optimal RVO (ORCA)**：van den Berg 等（2007）指出，多主动体若各自只把对方当
  被动障碍会产生**振荡**；RVO/ORCA 让每个 agent 承担"一半"避让责任，从而**保证无振荡、可实时、可扩展**
  地协调数百个 agent，且同时处理静态与移动障碍（`doc:507`, `doc:2079`, `doc:1074`）。
- 在编队中，ORCA 常作为局部运动规划层：在 2D 速度空间为每个邻居构造半平面约束并求解低维 LP，
  复杂度仅随局部邻居数增长（`doc:1324`）。ORCA-DD 还把差速机器人的非完整约束纳入（`doc:1159`）。

**适用**：agent 间互碰 ✅✅、动态障碍 ✅✅、静态障碍 ✅（视作静止障碍）。是"反应式、分布式、无需通信"
  场景的首选。

### 3.3 控制障碍函数（CBF）/ CLF-CBF-QP

CBF 把"安全"编码为**硬约束**，使安全集前向不变（forward invariant），是近年 safety-critical 控制的主流。

- **统一框架**：将编队目标编码为 CLF（可松弛的稳定性约束），将安全要求（含 agent 间与障碍物）编码为
  CBF（硬约束），合成一个 QP 实时求解。相比 MPC 有更清晰的**可行性保证**与更好实时性
  （`doc:3084` Liu 等 2026，Distributed CLF-CBF Certificates for Reactive Formation）。
- **反应式编队（reactive formation）**：允许在危险时**临时偏离**编队形状以避障、事后自动恢复
  （`doc:3084`）。这正是处理动态障碍的关键思想。
- **鲁棒 CBF**：在速度/输入饱和、扰动下仍能保证避碰（`doc:1740` Fu 等 2024，Robust collision-avoidance
  formation navigation，结合预定义时间观测器 + 局部二次优化安全控制器）。
- 高阶 CBF（HOCBF）、嵌入式/动态 CBF 也被用于微型机器人群与无人机（`doc:3258` Liu 等 2026；
  `doc:701` Palani 等 2025，用 collision cones + CBF 做多 UAV 避碰）。
- APF-CBF 混合：用 APF 生成参考修正、CBF 保证安全（`doc:1273` Tang 等 2025，多机避碰）。

**覆盖**：agent 间 ✅、静态 ✅、动态 ✅（需把障碍预测轨迹喂入 CBF 约束）。安全性理论最扎实。

### 3.4 导航函数（Navigation Function, NavFn）

- Tanner & Kumar 的 APF 策略可保证编队形状的**近全局渐近收敛**同时保证过程无碰（`doc:930`）。
- Hong 等（2018）用导航函数做"圆形编队环绕跟踪"，兼顾互碰避免与目标环绕（`doc:1523`）。
- 多用于 **agent 间 + 静态障碍**；在动态场景下相对笨重（缺乏预测机制），故动态环境较少单独使用。

### 3.5 模型预测控制（MPC）

- MPC 在**一个优化框架内同时处理**编队、互碰、静态/动态障碍与状态/输入约束，是约束最丰富的方案
  （`doc:3084`, `doc:1737` Du 等 2024，Lyapunov-based MPC 做 multi-UAV formation tracking-containment + 避障）。
- 局限：严格初始可行条件、随 agent 数量**计算负担激增**，实时性受限（`doc:3075`, `doc:3084`）。
- Wang 等（2007）把 MPC 用作**跟踪层**，上层用 Grossberg 神经网络的 dual-mode 策略生成参考轨迹，
  分别处理 safe mode（无碍）与 danger mode（有碰撞/障碍）（`doc:3311`）。

### 3.6 流场 / 数值流场

- Shao 等（2006）用**数值流场**实现柔性编队避障，适合静态、密集障碍环境（`doc:2770`）。
- 本质是预计算一个"无旋流场"引导 agent 绕障，静态场景高效，动态场景适应性弱。

### 3.7 序贯凸规划 / 编队参数优化（动态环境专用）

- **Alonso-Mora 等（2017）**：在**位置-时间空间**中先生长出一个无碰撞凸区域，再用约束优化（序贯凸规划）
  优化编队的"位置、朝向、尺寸"等参数，使编队整体在静态与**动态**障碍中保持无碰并朝目标推进
  （`doc:966`，IJRR 2017，含实地实验与最多 16 架 MAV 仿真）。
- **Zhao 等（2026）三层编队机动控制**：把动态障碍建模为**虚拟领导者层**，通过领导者-跟随者协同，
  将编队机动与动障避让统一；引入归一化符号函数（NSF）保证构型切换平滑、控制输入无突变，
  可**同时避静态与动态障碍**（`doc:3075`, IEEE TAC 2026）。

### 3.8 学习型方法（DRL）

- Sui 等（2021）用**深度强化学习**解决 leader-follower 编队的碰撞避免：两阶段训练（模仿学习 + RL），
  模仿阶段用"一致性编队控制器 + ORCA"作专家示范，RL 阶段用复合奖励同时兼顾编队保持与避碰，
  并用 LSTM 感知不定数量障碍（`doc:1859`, IEEE TNNLS）。
- 适合**不确定动态环境**，推理实时，但安全性为经验性、依赖训练分布与泛化。

---

## 4. 按场景归纳的"该用什么"

### 4.1 只关心 Agent 间互碰
- 首选 **ORCA/RVO**（分布式、实时、无振荡，`doc:507`）。
- 或 **CBF/CLF-CBF-QP**（带安全证明，`doc:3084`）、**APF 互排斥**（`doc:1575`）、**导航函数**（近全局，
  `doc:930`）。

### 4.2 只关心静态障碍物
- **APF / 增强 APF**（`doc:3207`）、**数值流场**（`doc:2770`）、**导航函数**（`doc:1523`）、
  **MPC**（约束多时，`doc:1737`）。
- 狭窄通道可配合 **formation scaling（编队缩放）**：把编队尺寸作为可调变量，过不去就缩小
  （`doc:1547` Liu 等 2019 事件触发编队跟踪；`doc:3075` 自适应缩放）。

### 4.3 动态障碍物（难点所在）
- **反应式框架**：VO/RVO/ORCA（将障碍视为移动 VO，`doc:507`）、**CBF + 障碍预测轨迹**（reactive formation，
  `doc:3084`）、**增强 APF**（时变斥力，`doc:1213`）。
- **编队整体优化**：Alonso-Mora 的 position-time 凸规划（`doc:966`）、Zhao 的三层虚拟领导者法（`doc:3075`）。
- **学习/混合**：DRL + ORCA 示范（`doc:1859`）。
- 关键共性：**必须预测障碍未来轨迹**，并把预测融入避让约束。

### 4.4 三者都要兼顾（最一般情形）
- **统一优化/安全框架**：CLF-CBF-QP（`doc:3084`）、鲁棒 CBF（`doc:1740`）、MPC（`doc:1737`）、
  APF-CBF 混合（`doc:1273`）。
- **分层架构**：上层（APF / ORCA / 凸规划 / GNN）生成安全参考，下层（MPC / 跟踪控制器）跟踪
  （`doc:3311` Wang 2007 的 dual-mode + MPC 跟踪）。

---

## 5. 关键论文索引（可在 okb-assist 中按 doc 编号读取）

| doc | 标题 | 年份 / 出处 | 主要贡献 |
|---|---|---|---|
| 3311 | Cooperative UAV Formation Flying With Obstacle/Collision Avoidance | 2007, IEEE TCST | 明确区分 collision(agent间) 与 obstacle；dual-mode + GNN + MPC |
| 930 | A survey and analysis of multi-robot coordination | 2013, IJARS | 综述；Tanner&Kumar 近全局 APF 编队无碰 |
| 507 | Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation | 2007, ICRA | RVO/ORCA 基础，解决互碰振荡，兼顾静/动障碍 |
| 2079 | Generalized reciprocal collision avoidance | 2015, IJRR | RVO 推广 |
| 1074 | Cooperative collision avoidance for nonholonomic robots | 2018, IEEE T-RO | ORCA 用于非完整机器人 |
| 3207 | Formation Control With Obstacle Avoidance for Stochastic MASs | 2018, IEEE TIE | APF 用于随机系统编队+静态避障 |
| 1547 | Event-Triggered Coordination for Formation Tracking in Constrained Space | 2019, IEEE TCYB | 受限空间编队跟踪、编队缩放、事件触发 |
| 2770 | Flexible formation control for obstacle avoidance based on numerical flow field | 2006, CDC | 数值流场静态避障 |
| 1523 | Cooperative circular pattern target tracking using navigation function | 2018, AST | 导航函数环绕跟踪+互碰 |
| 966 | Multi-robot formation control and object transport in dynamic environments via constrained optimization | 2017, IJRR | position-time 凸规划，静/动障碍统一 |
| 3075 | A three-layer formation maneuvering control with dynamic collision avoidance | 2026, IEEE TAC | 动态障碍建模为虚拟领导者层，静/动兼顾 |
| 3084 | Distributed CLF-CBF Certificates for Reactive Formation | 2026, IEEE TAC | CLF-CBF-QP 反应式编队，硬安全约束 |
| 1740 | Robust collision-avoidance formation navigation of VIC multirobot systems | 2024, IEEE TCYB | 鲁棒 CBF + 输入/速度饱和约束 |
| 701 | Collision avoidance control of multiple UAVs using collision cones and CBF | 2025 | collision cone + CBF |
| 1273 | Dynamic Event-Triggered Multi-Aircraft Collision Avoidance (APF-CBF) | 2025, Aerospace | APF-CBF 混合参考修正 |
| 3258 | Safety-Critical Fixed-Time Distributed Formation Control (Embedded DCBF) | 2026, IEEE/ASME TMECH | 嵌入式动态 CBF 微型机器人群 |
| 1737 | Model Predictive Formation Tracking-Containment Control for Multi-UAVs With Obstacle Avoidance | 2024, IEEE TSMC | Lyapunov-MPC 编队跟踪+避障 |
| 1859 | Formation control with collision avoidance through deep RL using model-guided demonstration | 2021, IEEE TNNLS | DRL + ORCA 示范，处理不确定动态环境 |
| 1213 | Enhanced potential field-based collision avoidance for UAVs in a dynamic environment | 2020, AIAA Scitech | 动态环境增强 APF |
| 943 | Delay-aware UAV swarm formation control via imitation learning from ARD-PF expert policies | 2026, Drones | APF 专家策略模仿学习 |
| 1324 | Distributed task planning … uncertain environments (ORCA) | 2026, IEEE TASE | ORCA 局部运动规划复杂度分析 |
| 1159 | Distributed multi-robot collision avoidance via deep RL | 2020, IJRR | 复杂场景 DRL 避碰（含 ORCA-DD） |

---

## 6. 趋势与建议

1. **从"分开处理"走向"统一安全框架"**：早期多为"编队 + 单独避障"的拼凑（如 `doc:1859` 指出的问题），
   现在更倾向用 CLF-CBF-QP、鲁棒 CBF、MPC 在一个优化/QP 中统一目标与安全（`doc:3084`, `doc:1740`）。
2. **反应式编队（reactive formation）**成为共识：允许临时偏离编队以保安全、事后恢复（`doc:3084`）。
3. **动态障碍需要预测**：VO/ORCA、CBF、序贯凸规划都依赖对障碍未来轨迹的预测；预测越准，避让越稳。
4. **计算可扩展性**：MPC 随规模性能下降，APF/ORCA/CBF-QP 因局部、分布式更利于大规模（`doc:3075`）。
5. **学习 + 模型混合**：用模型方法（ORCA/APF/一致性）作专家示范训练 DRL，兼顾性能与泛化（`doc:1859`, `doc:943`）。
6. **工程落地注意点**：非完整约束（ORCA-DD）、输入/速度饱和（鲁棒 CBF）、通信受限（事件触发 `doc:1547`）、
   拒绝服务攻击下的弹性无碰编队（`doc:3079` Zhang 等 2026, Automatica）。

---

### 一句话总结
- **Agent 间互碰** → ORCA/RVO、CBF、互排斥 APF、导航函数；
- **静态障碍** → APF、流场、导航函数、MPC、编队缩放；
- **动态障碍** → VO/RVO/ORCA、CBF（含预测）、序贯凸规划、增强 APF、反应式框架；
- **三者通吃** → CLF-CBF-QP / 鲁棒 CBF / MPC / 分层（规划+跟踪）架构。
