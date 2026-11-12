# 跨域集群（Cross-Domain Swarm）文献梳理：研究了哪些问题

> **范围**：OKB-Assist 知识库（MCP 服务 `okb`，总文档 5533 / 已索引 3114）中 `cross-domain` 相关且属于**多平台集群/协同**的文献
> **检索方式**：`search_info`（题录）、`grep_search`（全文正则）、`get_document_abstract`（摘要核校）、`get_toc` / `read_markdown`（章节精读）
> **检索式**：`cross-domain` / `跨域` / `cross-domain unmanned` / `aerial-surface-underwater` / `UAV-USV` / `跨介质` / `trans-medium` / `amphibious` / `两栖` / `空地协同` / `UAV-UGV`
> **整理日期**：2026-11-08　　文中 `[OKB #id]` 可直接 `read_markdown(id=...)` 取原文
> **姊妹文档**：`heterogeneous_models_survey.md`（§2.5 平台级模型）、`heterogeneous_swarm_encirclement_challenges.md`（§2.9 难点九、§4.4 跨域捕获）

---

## 0. 先分清三类"cross-domain"（否则会重复计数）

知识库中 `cross-domain` 至少三义，只有第一类属于本文件夹关心的"跨域集群"：

| 含义 | 载体 | 代表 | 是否本报告范围 |
|---|---|---|---|
| **① 跨域集群/跨域协同**（multi-domain swarm） | 多平台（UAV/USV/UUV/UGV）分工协同 | #137, #239, #3312, #663, #17 | ✅ 是 |
| **② 跨介质单平台**（cross-domain robot, CDR） | 一台机器人切换水/陆/空介质 | #133（117 个原型综述）、#2114、#1138 | ⚠️ 只作背景（异构的极端形态） |
| **③ 机器学习迁移的 cross-domain** | 域适应/迁移学习 | #407, #884, #3324, #5283, #489 | ❌ 无关 |

> **写作口径建议**：中文文献里"跨域"= 跨物理域（海/陆/空/潜/临空/天）；英文 `cross-domain` 在控制期刊中通常就是"UAV+USV(+UUV)"这类异构多平台系统。若审稿人按 ②/③ 理解，需在引言第一次出现时定义一次。

另一条重要界线：**"跨域" ≠ "异构"**。跨域一定是异构（模型类别、维数、时间尺度都不同），但异构可以发生在同一域内（如 mixed-order、相对阶不同）。本文件夹三篇文档的关系是：
`异构模型（建模层） ⊃ 平台级异构 ≡ 跨域（物理层）`，合围/围捕是其中一类任务。

---

## 1. 文献清单（按域组合分组）

### 1.1 空–海（UAV–USV，库中数量最多）

| 文献 | 年 | 期刊 | 研究的问题 | OKB |
|---|---|---|---|---|
| *Aerial-Marine Cross-Domain Uncrewed Systems: An Overview of Cyberphysical Coordination Frameworks* | 2025 | IEEE Control Systems | **AMCDUS 分层信息物理协同架构**：物理空间三级（规划路径 → 多 UAV-USV 协同指令 → 单机能力）+ 信息空间三级（云/网络/终端）；明确把 UAV-USV 视为**双时间尺度系统**（快 UAV 子系统 + 慢 USV 子系统）；跨域导航与降落真机实验 | #3312 |
| *Coordinated Navigation Control of Cross-Domain Unmanned Systems via Guiding Vector Fields* | 2024 | IEEE TCST | 异构 UAV+USV 沿**各自规定路径**协同导航；DGVF 分两层（上层异构引导速度 + 下层信号跟踪），**仅靠标量（附加虚拟坐标）通信**完成异构协同；千岛湖 3×M100 + 3×HUSTER 真机 | #1946 |
| *Hierarchical event-triggered predictive control for cross-domain unmanned systems with mixed constraints* | 2025 | IEEE/CAA JAS | **多维度编队跟踪（MDFT）**：把 USV 的 2D 工作空间扩维成 3D 虚拟状态，才能与 UAV 组成同一几何队形；速度/控制输入/欧拉角**混合约束**下的事件触发预测控制（合作层 + 局部层） | #265 |
| *Dual-stage coordinated landing control for cross-domain UAV-USV fleets* | 2026 | IEEE T-IE | **多 UAV 对多 USV 的分布式着舰/降落**：阶段一 CBAA 一致性拍卖 + 最小能耗任务执行实现配对与跟踪，阶段二视觉导航降落；给出有限时间稳定性条件；3×M100 + 3×HUSTER-37 湖试 | #3083 |
| *A cooperative control framework for path following of UAV-USV cooperative system in maritime scenarios* | 2025 | Ocean Eng. | 海上搜救的**协同路径跟踪**：UAV-USV 耦合运动学-动力学建模 + 微分平坦的协调轨迹生成；USV 耐波性（heave/roll/pitch）多浪向预测 | #3090 |
| *A simplified task-assisted constrained multi objective genetic algorithm for USV-UAV collaborative maritime search and rescue* | 2025 | IEEE TVT | **带时间窗约束的 USV-UAV 协同搜救任务分配**：SOM 时空聚类压缩搜索空间定 USV 位置 + 三任务辅助约束多目标遗传算法（CMOGE-STC） | #3091 |
| *A novel cooperative design for USV-UAV systems: 3-D mapping guidance and adaptive fuzzy control* | 2023 | IEEE TCNS | USV-UAV 的 **3D 制导映射** + 自适应模糊控制 | #99 |
| *NMPC-based UAV-USV cooperative tracking and landing* | 2023 | J. Franklin Inst. | NMPC 下的 UAV 跟踪 USV 并降落 | #846 |
| *A separation and rendezvous control method for the UAV-USV system based on distributed NMPC* | 2024 | IEEE T-IV | **分离–会合（separation & rendezvous）**：分布式 NMPC 处理跨域交接过程 | #3089 |
| *Performance-Optimize Adaptive Robust Tracking Control for USV-UAV Heterogeneous Systems With Uncertainty* | 2025 | IEEE TVT | 不确定动态下 USV-UAV **海空 3D 协同目标跟踪**；预设性能 + 最优虚拟控制 + 饱和处理 UAV 位置环逆解奇异 | #997 |

### 1.2 空–海–潜（UAV–USV–UUV，"3U"网络）

| 文献 | 年 | 期刊 | 研究的问题 | OKB |
|---|---|---|---|---|
| *海上跨域集群水下目标协同追踪: 关键技术与展望* | 2026 | 自动化学报 | **综述**：按定位、跟踪、通信、规划四模块列难点；指出感知能力不足、跨域协同机制不完善、环境适应性难兼顾 | #239 |
| *基于空-海-潜跨域无人平台协同的海上目标探测追踪策略* | 2026 | 自动化学报 | 测度论求 **UAV-USV-AUV 最优探测队形**（最大化探测概率）+ 逆强化学习编队控制器 + 动态避障追踪；含实验 | #17 |
| *Cooperation Detection and Tracking of Underwater Target via Aerial–Surface–Underwater Vehicles* | 2025 | IEEE TAC | 同上思路的期刊版：**测度论构造异构探测模式**（调 USV-UUV 队形最大化探测概率）+ DDPG 跟踪 + 多步位置预测保连通 | #3163 |
| *Underwater target capture based on heterogeneous unmanned system collaboration* | 2024 | IEEE T-IV | **水下目标捕获**：通信与能量约束下 UAV-USV 全覆盖路径规划（扩大单位时间观测范围）+ UUV 用自适应灰狼算法围捕 | #2930 |
| *3U: joint design of UAV-USV-UUV networks for cooperative target hunting* | 2023 | IEEE TVT | 3U 网络**联合设计**：联合优化 UAV 位置、UUV 轨迹与互连通性，DQN 求解，能耗-连通性折中 | #3085 |
| *Age of information-aware multi-objective optimization for heterogeneous UAV-USV-UUV networks in underwater target hunting* | 2025 | IEEE TMC | 引入 **AoI（信息年龄）** 度量探测时效；约束多目标优化（能耗 + 任务时长）优化三域轨迹，AE-MVTD3 求解；UAV 找目标、USV 中继、UUV 群围捕的角色分工 | #3087 |

### 1.3 空–地（UAV–UGV）

| 文献 | 年 | 期刊 | 研究的问题 | OKB |
|---|---|---|---|---|
| *面向空地协同作战的无人机-无人车异构时变编队跟踪控制* | 2019 | 航空兵器 | 麦克纳姆轮 UGV + 四旋翼 UAV 的**异构时变编队跟踪**分布式控制器与参数选取 | #1474 |
| *空地跨域无人系统低空协同立体监测与控制* | 2026 | 控制与决策 | 3-DOF 欠驱动 UGV 与 6-DOF 全驱动 UAV 的**组合动力学建模**；卡尔曼滤波预测 UGV 未来位置作为 UAV 监测目标；Backstepping + RBF-NN + SGUUB 证明 | #347 |
| *Multimodal cross-domain motion of terrestrial-aerial amphibious robots* | 2026 | IEEE/ASME TMech | 陆空两栖**单平台**的跨域运动规划与控制（CDR 路线，非集群） | #2114 |

### 1.4 综述 / 战略 / 支撑技术

| 文献 | 年 | 期刊 | 作用 | OKB |
|---|---|---|---|---|
| *Cross-Domain Cooperative Technology of Intelligent Unmanned Swarm Systems: Current Status and Prospects* | 2024 | 中国工程科学 | **中文最全的跨域协同战略综述**：按海空 / 空地 / 海陆(海陆空)三场景对比五类技术（跨域通信、导航定位、路径规划与跟踪、自主着舰/陆、任务规划与异构协同控制），给出 5 个重点方向 | #137 |
| *A survey of cross-domain robotics: classification, perception, planning and control* | 2026 | IEEE/ASME TMech | **CDR（跨介质单平台）**分类学 + 117 原型库；提供符号体系 | #133 |
| *海上无人系统集群发展现状及关键技术研究进展* | 2021 | 中国舰船研究 | §1.4 跨域集群发展概况（美英法演示项目表：UUV-UAV 跨域通信、AMMCS/ACER 指控、I4Drones）；关键技术：通信自组网、协同态势感知、任务分配 | #2261 |
| *A survey of maritime unmanned search system* | 2023 | Ocean Eng. | 海上无人搜救理论/应用/展望 | #1726 |
| *A survey of underwater multi-robot systems* | 2022 | IEEE/CAA JAS | 水下多机器人（跨域的水下侧） | #1689 |
| *面向跨域协同导航的历史测距容错滤波方法* | 2025 | 中国惯性技术学报 | 跨域集群**协同导航**中传感器量测故障 → 相对定位误差发散；历史测距基线 + 误差感知序列故障检测 + CP-EKF 策略性解算 | #3139 |
| *复杂约束条件下无人系统跨域协同作战任务规划方法* | 2026 | 系统仿真学报 | 战前**任务规划**：性能/弹药/任务时序/时间窗/航路多约束，多策略增强灰狼算法（MSEGWO）最小化综合代价 | #1612 |
| *基于MLP的海上无人跨域协同效能评估系统的设计与实现* | 2024 | 学位/会议 | **效能评估**：探测能力指标体系 + ADC 法生成样本 + MLP 评价模型 | #3686 |
| *Fully distributed fixed-time control of cross-domain swarm robots for non-cooperative target tracking* | 2023 | IEEE TCAS-II | 唯一把"cross-domain swarm"直接写进控制问题的：**非合作目标**（用网络模型构造轨迹，避免系统矩阵观测器的自环问题）+ 完全分布式固定时间跟踪 | #196 |
| *Asymmetric self-play-enabled intelligent heterogeneous multirobot catching system* | 2023 | IEEE T-RO | 按**动态能力分配角色** + 非对称自博弈训练多类型机器人协同抓捕 | #3807 |
| *Distributed task allocation with minimum makespan for heterogeneous multiplayer pursuit-evasion games* | 2024 | IEEE TAC | 异构追逃的分布式任务分配（最小完工时间） | #2725 |

---

## 2. 这些论文到底研究了哪些问题（按问题域归并）

跨域集群的文献可以归到 8 个问题域。**注意它们的耦合顺序**：A（建模）→ B/C（通信与感知）→ D（控制）→ E（交接动作）→ F/G（决策与自组织）→ H（验证）。

### 问题 A：异构动力学的统一建模 —— "维数与时间尺度不匹配"

这是跨域区别于普通异构 MAS 的**根本难点**，也是所有后续设计的入口。

- **维数不匹配**：USV 是 2D 平面运动（3-DOF 欠驱动），UAV 是 3D（6-DOF 全驱动）。解决套路有三种：
 1. **升维**：把 USV 状态扩成 3D 虚拟状态，才能在 3D 中与 UAV 组同一几何队形 [OKB #265]；
 2. **组合动力学模型**：把 UGV/UAV 两套方程写成统一组合形式再设计协同律 [OKB #347]、UAV-USV 耦合运动学-动力学框架 [OKB #3090]；
 3. **分层/分级引导**：不给统一状态空间，而用"上层引导量 + 下层执行"两级结构绕开维数问题 [OKB #1946, #3312]。
- **时间尺度不匹配**：UAV-USV 被显式建模为**快-慢双时间尺度系统**（fast UAV / slow USV）[OKB #3312]；#239 指出跨介质性能差异导致规划必须**分阶段**。
- **模型类别不匹配**：欠驱动 vs 全驱动、水声/气动/耐波性差异 → 同一 Lyapunov 函数难以覆盖 [OKB #137 §四(三)]。
- 战略层综述直接把这条列为重点方向：**"具有强异质动力学特性的协同控制"**，要求"动力学模型的统一与适配" [OKB #137]。

> 与本文件夹 `heterogeneous_models_survey.md` §2.5 的"平台级模型"完全对应；理论上仍是**输出级/几何级**协同，而非状态同步。

### 问题 B：跨介质通信与信息时效

- **信道差异**：水声（1500 m/s、指数衰减、多径、"时-空-频"变、多普勒）+ 无线电 + 5G/WiFi 混合组网；海上组网弱、"艇间信息交互困难" [OKB #239 §2, #3449 §1.3]。
- **时延与陈旧信息**：跨域集群在受限通信下的**时间戳更新协议 + 时延补偿预测**，保证决策一致性 [OKB #663]。
- **时效性指标化**：把 **AoI（信息年龄）** 写进 UAV 搜索策略，与能耗一起构成多目标 [OKB #3087]。
- **通信量的极简化**：异构协同**只传一个标量**（附加虚拟坐标）[OKB #1946] —— 这是跨域通信受限下很有代表性的设计哲学。
- **连通性约束**：多步位置预测保证 UAV-USV-UUV 连通 [OKB #3163]；连通性与能耗联合优化 [OKB #3085]。
- **导航信息容错**：测距故障导致相对定位误差放大，需历史测距基线 + 故障检测 + CP-EKF [OKB #3139]。
- 战略层：融合组网通信被视为"核心技术之一"；"空天地海"跨域组网要求**可弹性重构、多维、可扩展**的网络架构 [OKB #137 §三、§四(五)]。

### 问题 C：跨域感知 / 探测 / 定位（GNSS 拒止、弱测量）

- **探测概率最大化**：用**测度理论**构造异构探测模式，通过调整 USV-UUV 队形最大化对高机动目标的探测概率 [OKB #3163, #17]。
- **观测范围扩展**：通信与能量约束下 UAV-USV **全覆盖路径规划**，提高单位时间观测范围，为 UUV 捕获创造条件 [OKB #2930]。
- **弱测量定位**：GNSS 不可用、测量野值、惯性累积误差、能耗受限 → 高精度实时定位难 [OKB #239 §2 第1条]。
- **多源融合**：水声 + 雷达 + 视觉 + 空中平台传感器联合定位 [OKB #137 §三(一)①]。

### 问题 D：跨域协同控制（队形/编队/跟踪的几何一致性）

- **异构引导向量场（DGVF）**：上层异构引导速度控制器 + 下层信号跟踪，沿各自规定路径机动并同时成队形；渐近收敛的充分条件在"指数衰减跟踪误差"下给出 [OKB #1946]。
- **多维度编队跟踪（MDFT）**：USV 虚拟状态 + UAV 在 3D 局部坐标成几何队形 + 几何中心在地球坐标跟踪参考轨迹；混合约束下的事件触发预测控制 [OKB #265]。
- **异构编队跟踪**：UAV-UGV 时变编队跟踪分布式控制器 [OKB #1474]；USV-UAV 预设性能 + 最优控制的 3D 协同跟踪（含 UAV 位置环逆解奇异性处理）[OKB #997]。
- **完全分布式 + 固定时间**：跨域集群对非合作目标的固定时间跟踪，只依赖自身与邻居输出，且不需要目标系统矩阵 [OKB #196]。
- **协同捕获/围捕**：UUV 群用自适应 GWO 捕获，解决早熟收敛 [OKB #2930]。
- 战略层：**"异构协同控制需要设计灵活、智能的协同控制算法，结合实时反馈与调整机制，实现动力学模型的统一与适配"** [OKB #137 §三(二)④]。

### 问题 E：跨介质交接动作（着舰/降落/会合/分离/回收）

这是跨域**独有**、单域集群没有的问题域，也是实验论文的主战场：

- **多对多配对降落**：CBAA 一致性拍卖 + 最小能耗匹配 + 视觉导航降落，有限时间稳定 [OKB #3083]。
- **分离–会合**：分布式 NMPC 处理 UAV 起飞/返航与 USV 的交接 [OKB #3089]；NMPC 跟踪降落 [OKB #846]。
- **自主着舰/着陆技术**被 #137 单列为五类跨域技术之一，且**海空与空地的部署差异明显**（波浪 vs 建筑/道路/人流）。
- **跨介质飞行浮标**（coaxial cross-domain flying buoy）在未知海况下的跟踪控制 [OKB #1138]。
- **实验验证**：真机跨域降落实验是 #3312 的核心贡献之一。

### 问题 F：跨域任务规划与任务分配

- **带时间窗的多目标搜救分配**：SOM 时空聚类 + 三任务辅助约束多目标遗传算法 [OKB #3091]。
- **战前任务规划**：性能差异 + 弹药 + 任务时序 + 时间窗 + 航路多约束的 MSEGWO [OKB #1612]。
- **分布式任务分配 + 冲突消解**：在目标分裂、信息不完整/延迟下实时重分组 [OKB #663]。
- **异构追逃任务分配**（最小完工时间）[OKB #2725]；**按能力分配角色**的非对称自博弈抓捕 [OKB #3807]。
- 战略层：**"多域耦合约束下的弹性决策"**——宏观全局约束（任务目标、时敏完成时间）+ 微观局部约束（各域环境）维度/重要性均不同，需弹性权衡 [OKB #137 §四(二)]。

### 问题 G：自组织 / 动态重构 / 能量与角色分工

- **能力互补的分工范式**：UAV 高机动强感知、USV 长续航中继、UUV 抵近作业 [OKB #3087, #663, #239]。
- **自主状态机**：SECA（State-Event-Condition-Action）模型让 UAV 在 Tracking / Returning / Energy Replenishment / Standby 间自主切换，支撑全任务周期连续作业 [OKB #663]。
- **自组织重组**：目标分裂时实时重构子群（subgroup reorganization），优于集中式与固定编队 [OKB #663]。
- 战略层：跨域协同决策需结合"智能辅助决策 + **动态编组**" [OKB #137 §三(二)③]。

### 问题 H：实验验证与效能评估

- **真机跨域平台**：3×DJI M100 + 3×HUSTER-37 USV 湖试（降落）[OKB #3083]；3×M100 + 2×HUSTER-16 + 1×HUSTER-12C + 5G 基站（导航）[OKB #1946]；AMCDUS 导航与降落实验 [OKB #3312]；空-海-潜探测追踪含实验 [OKB #17, #3163]。
- **仿真为主**：#663, #3085, #3087, #2930, #196, #347 等仍以数值仿真为主。
- **效能评估**：探测能力指标体系 + ADC 样本生成 + MLP 评价 [OKB #3686]。
- **作战概念验证（国外）**：UUV-UAV 跨域通信、AMMCS/ACER 单系统对多类无人平台指控、法国 I4Drones 三域拦截演示 [OKB #2261 §1.4 表1]。

---

## 3. 与"合围/围捕"的交集：跨域合围几乎是空白

现有跨域文献的**任务**几乎都是：探测 → 跟踪 → 捕获/降落/搜救/中继，**没有一篇做"跨域异构合围（fencing/enclosing）"的严格理论**。可对接的只有：

| 可复用要素 | 来源 | 移植到跨域合围的问题 |
|---|---|---|
| 测度论探测队形优化 | #3163, #17 | 把"探测概率最大"换成"合围成功率/封口概率最大" |
| 异构角色分工（UAV 找、USV 中继、UUV 围） | #3087, #3085 | 谁围、围多近、围多久的**角色-构型-控制一体化** |
| 双时间尺度建模 | #3312 | 快慢子系统下的合围收敛性（多时间尺度 Lyapunov） |
| 维数桥接（USV 2D → 3D 虚拟状态） | #265 | 凸包/合围几何在高维与低维平台间如何定义 |
| 固定时间完全分布式跟踪（无目标系统矩阵） | #196 | 非合作逃逸目标下的固定时间合围 |
| 分阶段规划 | #239 | 跨域合围的阶段切换时刻优化（本文件夹另一篇 §6 已列为空白） |

---

## 4. 尚未解决的问题（可作选题）

1. **跨介质时间尺度差的严格稳定性理论**：现有分层/分级结构多给充分条件，且依赖"跟踪误差指数衰减"等假设 [OKB #1946, #265]；快慢子系统的耦合稳定性、以及"规划层周期 ≫ 控制层周期"的量化条件缺失。
2. **跨域合围/包围的定义与判据**：维数不同 ⇒ 只能谈输出合围，而高阶/混合阶输出合围本身"尚未解决" [OKB #3437]；跨域版本更无定义。
3. **多域耦合约束的可行性判据**：环境约束 + 任务约束 + 平台间耦合约束同时在线时的可行集刻画与弹性决策 [OKB #137 §四(二)]；现有做法仍是加权势函数 + 经验整定。
4. **跨域通信的时效-能耗-连通性三目标统一理论**：AoI 只被当作优化项 [OKB #3087]，缺少"通信指标 → 队形精度/合围闭合度"的定量映射。
5. **规模扩展**：真机实验普遍 3–6 平台（#3083, #1946）；#3083 仅用仿真说明"对更大编队的鲁棒性"。
6. **标准化与互操作**：通信协议不兼容、时延与传输不稳定被列为第一位的现实障碍 [OKB #137 §五(一)]。
7. **临近空间平台入队**："天临空地海"一体化仅有概念，协同作业研究"并不多见" [OKB #137 §四(四)]。
8. **跨域集群的信息安全**：#137 提出安全与隐私保护需求，但库中跨域文献几乎不涉及攻击下的跨域协同（对比单域已有 DoS/Byzantine 结果 [OKB #3646, #3351]）。

---

## 5. 检索记录与局限

### 5.1 工具与检索式

```text
search_info(query, fields=[id,title,year,journal,doc_type], limit)
grep_search(query, regex=true, limit, context, max_context_chars)
get_document_abstract(id)   get_toc(id)   read_markdown(id, line_start, line_count)
```

| 命中量 | 检索式 | 备注 |
|---|---|---|
| 23 | `cross-domain`（题录） | 含大量 ML 域适应噪声，需按标题筛 |
| 11 | `跨域`（题录） | 中文命中率高 |
| 34+ | `cross-domain\|cross domain`（全文） | 召回更全，含仅正文提及者 |
| 30+ | `跨域`（全文） | 含作战效能评估类无关文献 |
| 8 | `UAV-USV`（题录） | 空–海子域核心 |
| 0 | `cross-medium` / `跨介质` / `空海潜`（题录） | 说明单纯靠题录会漏；改用全文 `trans-medium` 才找回 #239/#17/#1474 |
| 20 | `aerial.?surface.?underwater\|air-sea`（全文） | 3U 网络主要入口 |

### 5.2 已核校原文的文档

| OKB ID | 精读位置 | 用途 |
|---|---|---|
| #137 | 目录 + §三现状与需求 + §四重点方向 | 跨域技术分类与 5 大方向的原文表述 |
| #239 | §2 难点（结合本文件夹合围文档已引） | 四模块难点 |
| #2261 | §1.4 跨域集群发展概况 + 表1 | 国外演示项目清单 |
| #3312 | 首页 + INTRODUCTION | AMCDUS 分层架构、双时间尺度表述 |
| #133 | 摘要（CDR 定义与 117 原型库） | 区分"跨介质单平台" |

### 5.3 局限

- `search_info` 是题录匹配，跨域论文常不在标题含 `cross-domain`；本表已用全文检索补检，但仍可能漏掉仅正文一句提及的工作。
- 库中约 1483 条文档为 `uploaded`（未索引），全文检索不可达。
- #3312 的 `abstract` 字段为空，本表结论取自正文 INTRODUCTION 与 Summary。
- 部分中文会议/学位论文无摘要，贡献要点取自标题与目录，措辞已降级为"要点"。
- 重复条目需去重后引用（如 #3139 / #130、#2930 / #3087 系列同一课题组的连续工作）。

---

*文件：`survey/heterogeneous/cross_domain_swarm_survey.md`*
