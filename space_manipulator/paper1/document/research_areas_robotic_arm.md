# 机械臂研究方向调研报告

> 基于 MCP 知识库（okb-assist）中 3814 篇文献，检索得到 221 篇机械臂/空间机械臂相关论文
> 数据来源：`document/` 文件夹下 MCP 索引的学术文献（含 article、thesis、journalArticle、conferencePaper 等类型）

---

## 一、运动规划与控制 (Motion Planning & Control)

### 核心问题
机械臂末端位姿跟踪、动态避障、路径优化、时间最优轨迹规划。

### 关键技术
- 零空间投影动态层级避障
- 凸优化时间最优路径跟踪
- 差分进化算法最优轨迹规划
- 非线性模型预测控制 (NMPC)

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *面向在轨捕获的空间机械臂运动规划与控制方法研究* (严宇新) | thesis | 2025 | **id: 2093** |
| 2 | *Time-optimal path tracking for dual-arm free-floating space manipulator system using convex programming* (An et al.) | journalArticle | 2023 | **id: 113** |
| 3 | *Optimal trajectory planning of free-floating space manipulator using differential evolution algorithm* (Wang et al.) | article | 2018 | **id: 2080** |
| 4 | *在轨服务空间机械臂运动及任务规划方法研究* (曾岑) | thesis | 2013 | **id: 1356** |
| 5 | *Online Trajectory Generation for Space Manipulator via Nonlinear Model Predictive Control* (Giordano et al.) | article | 2026 | **id: 995** |
| 6 | *Trajectory optimization and control of a free-floating two-arm humanoid robot* (Ramon et al.) | journalArticle | 2022 | **id: 210** |

---

## 二、振动抑制与柔性结构控制 (Vibration Suppression & Flexible Structure Control)

### 核心问题
空间机械臂臂杆柔性、关节柔性导致的末端振动，以及挠性附件（太阳翼等）的耦合振动问题。

### 关键技术
- 分段约束阻尼层（SCLD）
- 六维加速度传感器反馈抑振
- 奇异摄动法刚柔解耦
- 变转速控制力矩陀螺（VSCMG）主动抑制

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *分段约束阻尼层结构及其在空间机械臂减振中的应用* (田士涛) | thesis | 2016 | **id: 91** |
| 2 | *空间机械臂传感系统及关节振动抑制研究* (邹添) | thesis | 2017 | **id: 905** |
| 3 | *Maneuver and active vibration suppression of free-flying space robot* (Jia et al.) | article | 2018 | **id: 3149** |
| 4 | *带挠性附件的浮动基座空间机械臂系统跟踪控制研究* (范一迪) | thesis | 2022 | **id: 157** |

---

## 三、力控制与柔顺交互 (Force Control & Compliant Interaction)

### 核心问题
机械臂与环境不确定接触下的力/位混合控制、阻抗控制、无传感器力估计。

### 关键技术
- 串联弹性执行器 (SEA)
- 扩张状态观测器 (ESO)
- 非光滑接触力观测器
- 干扰观测器 (DOB)

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *机械臂不确定接触环境下的运动/力/阻抗统一控制方法研究* (林银洁) | thesis | 2022 | **id: 426** |
| 2 | *机械臂系统的建模与抗干扰控制方法研究* (韩林言) | thesis | 2022 | **id: 1603** |
| 3 | *On position/force tracking control problem of cooperative robot manipulators using adaptive fuzzy backstepping approach* (Baigzadehnoe et al.) | journalArticle | 2017 | **id: 341** |
| 4 | *配置柔顺机构空间机器人双臂捕获卫星操作力学模拟及基于神经网络的全阶滑模避撞柔顺控制* (朱安) | article | 2019 | **id: 3034** |

---

## 四、空间目标捕获与碰撞动力学 (Space Target Capture & Impact Dynamics)

### 核心问题
捕获过程中的接触碰撞建模、碰撞力预测、捕获后复合体消旋与稳定控制。

### 关键技术
- 圈套式绳索捕获机构
- 加速度势场法避障轨迹规划
- 径向基函数神经网络 (RBFNN) 不确定性补偿
- 反步自适应控制

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *空间机械臂在轨捕获碰撞动力学及控制研究* (张龙) | thesis | 2017 | **id: 1010** |
| 2 | *空间机械臂抓捕目标后的复合体稳定策略研究* (詹博文) | thesis | 2022 | **id: 72** |
| 3 | *空间机械臂抓取目标的碰撞前构型规划与控制问题研究* (丛佩超) | thesis | 2009 | **id: 177** |
| 4 | *Impact modeling and reactionless control for post-capturing and maneuvering of orbiting objects using a multi-arm space robot* (Raina et al.) | article | 2021 | **id: 473** |
| 5 | *具有多维可控阻尼关节的空间捕获机械臂镇定控制研究* (常睿) | thesis | — | **id: 1684** |
| 6 | *Free-floating space manipulator impacting a floating object: Modeling and output SDRE controller design* (Nekoo et al.) | article | 2024 | **id: 2050** |

---

## 五、智能控制方法 (Intelligent Control / AI-based Methods)

### 核心问题
利用强化学习、神经网络等数据驱动方法解决机械臂非线性动力学建模与控制难题。

### 关键技术
- 模型预测强化学习 (MPC-RL)
- Actor-Critic 框架
- 张神经动力学 (Zhang Neural Dynamics, ZNN)
- 预定义时间收敛循环神经网络

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *基于强化学习的空间连续型机械臂碎片跟踪的动力学控制* (江达) | thesis | 2023 | **id: 472** |
| 2 | *Event-triggered image-space tracking control of space manipulators using off-policy reinforcement learning with disturbance observers* (Zhuang et al.) | article | 2025 | **id: 3151** |
| 3 | *A flexible-predefined-time convergence and noise-suppression ZNN for solving time-variant Sylvester equation and its application to robotic arm* (Zheng et al.) | journalArticle | 2024 | **id: 3541** |
| 4 | *An Arbitrarily Predefined-Time Convergent RNN for Dynamic LMVE With Its Applications in UR3 Robotic Arm Control and Multiagent Systems* (Zheng et al.) | article | 2025 | **id: 1538** |
| 5 | *Simplified Zhang-Gradient Neurodynamics Handling Robotic Arm End-Effector Tracking Problems* (Zhang & Zhang) | article | 2024 | **id: 287** |
| 6 | *Model-Free and Pseudoinverse-Free Zhang Neurodynamics Scheme for Robotic Arms' Path Tracking Control* (Chen et al.) | article | 2025 | **id: 2170** |

---

## 六、抗饱和与约束控制 (Anti-Saturation & Constrained Control)

### 核心问题
输入饱和、状态约束（位置/速度边界）、暂态/稳态性能约束下的控制。

### 关键技术
- 正切双曲饱和函数
- 预设性能控制 (Prescribed Performance Control)
- 固定时间终端滑模控制
- 屏障 Lyapunov 函数

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *空间机械臂输出反馈抗饱和 PID 控制算法设计* (张晓东等) | article | 2011 | **id: 154** |
| 2 | *Constrained fixed-time terminal sliding-mode control with prescribed performance for space manipulator system* (Hu et al.) | journalArticle | 2024 | **id: 1072** |
| 3 | *Saturated output feedback control of free-floating space manipulator with fragility-avoidance prescribed performance* (Zhou et al.) | article | 2025 | **id: 3143** |

---

## 七、无扰控制与基座扰动抑制 (Reactionless Control)

### 核心问题
自由漂浮/自由飞行模式下，机械臂运动对基座（卫星）姿态的最小扰动。

### 关键技术
- 虚拟基座动力学
- 零空间自运动
- 状态相关黎卡提方程 (SDRE)

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *Reactionless control of free-floating space manipulators* (Zong et al.) | journalArticle | 2020 | **id: 797** |
| 2 | *Finite-time state-dependent Riccati equation regulation of anthropomorphic dual-arm space manipulator system in free-flying conditions* (Scalvini et al.) | article | 2024 | **id: 103** |
| 3 | *浮动基座空间机械臂系统的动力学建模与惯性轨迹跟踪的滑模控制* (陈力、刘延柱) | article | 2000 | **id: 1428** |

---

## 八、连续型/软体机械臂 (Continuum Manipulator)

### 核心问题
线驱动连续型机械臂的刚柔耦合动力学建模与智能控制，适用于复杂空间环境中的碎片清除。

### 关键技术
- 模块化臂节线驱动
- 多智能体深度强化学习
- 博弈论双臂协同防碰撞

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *基于强化学习的空间连续型机械臂碎片跟踪的动力学控制* (江达) | thesis | 2023 | **id: 472** |

---

## 九、容错控制与运动可靠性 (Fault-tolerant Control & Motion Reliability)

### 核心问题
空间机械臂长寿命在轨服役中，关节/部件失效后的任务完成能力。

### 关键技术
- 参数突变抑制
- 容错轨迹优化
- 概率任务规划

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *基于运动可靠性的空间机械臂优化控制研究* (李彤) | thesis | 2016 | **id: 717** |

---

## 十、传感系统与标定 (Sensor Systems & Calibration)

### 核心问题
六维力/力矩传感器的设计与在线标定、多感知关节架构。

### 关键技术
- CPU+FPGA 冗余架构
- 自标定补偿算法
- 漂移补偿

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *空间机械臂六维力/力矩传感器及其在线标定的研究* (孙永军) | thesis | 2016 | **id: 1108** |
| 2 | *空间机械臂传感系统及关节振动抑制研究* (邹添) | thesis | 2017 | **id: 905** |

---

## 十一、视觉感知与位姿测量 (Visual Perception & Pose Measurement)

### 核心问题
空间非合作目标的相对位姿测量，支持接近、跟踪与捕获操作。

### 关键技术
- 最大外轮廓识别
- 图像增强抗模糊
- 特征提取与匹配

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *A pose measurement method of a space noncooperative target based on maximum outer contour recognition* (Peng et al.) | journalArticle | 2020 | **id: 211** |
| 2 | *空间机械臂技术综述及展望* (刘宏等) — 综述文章中提及视觉感知方向 | article | 2021 | **id: 262** |

---

## 十二、高精度轨迹跟踪控制 (High-Precision Trajectory Tracking)

### 核心问题
模型不确定性、多约束、外部扰动下的跟踪精度保障。

### 关键技术
- 全驱动系统方法 (FASA)
- 分布鲁棒模型预测控制 (DR-MPC)
- 分数阶滑模控制
- 预测误差自适应 Jacobian

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *High-precision trajectory tracking control for free-flying space manipulators with multiple constraints and system uncertainties* (Tian et al.) | article | 2024 | **id: 3150** |
| 2 | *Distributionally robust model predictive control for trajectory tracking of space manipulator based on fully actuated system approach* (Yang et al.) | article | 2025 | **id: 2085** |
| 3 | *Disturbance observer-based fractional-order sliding mode control for free-floating space manipulator with disturbance* (Dou & Yue) | article | 2023 | **id: 2984** |
| 4 | *Prediction error based adaptive jacobian tracking for free-floating space manipulators* (Wang & Xie) | journalArticle | 2012 | **id: 1318** |
| 5 | *Fractional-order resolved acceleration control for free-floating space manipulator with system uncertainty* (Shao et al.) | article | 2021 | **id: 2143** |

---

## 十三、事件触发与资源优化控制 (Event-Triggered Control)

### 核心问题
在保证控制性能的同时，降低通信与计算资源消耗。

### 关键技术
- 事件触发机制
- 离策略强化学习

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *Event-triggered image-space tracking control of space manipulators using off-policy reinforcement learning with disturbance observers* (Zhuang et al.) | article | 2025 | **id: 3151** |

---

## 十四、线驱动/索驱冗余机械臂 (Cable-Driven Redundant Manipulator)

### 核心问题
轻质、低惯性、本征柔顺的索驱机械臂对翻滚目标的快速捕获。

### 关键技术
- 时变约束自适应规划
- 协同控制

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *Adaptive and rapid capture of tumbling targets by a cable-driven redundant space manipulator under time-varying constraints* (Yan et al.) | journalArticle | 2025 | **id: 179** |

---

## 十五、综述文章 (Survey Papers)

### 代表文献

| # | 文献标题 | 类型 | 年份 | MCP ID |
|---|---------|------|------|--------|
| 1 | *空间机械臂技术综述及展望* (刘宏、刘冬雨、蒋再男) — 航空学报 | article | 2021 | **id: 262** |

---

## 综合热度分析

| 研究方向 | 文献密度 | 增长趋势 | 备注 |
|---------|---------|---------|------|
| 🥇 运动规划与控制 | ★★★★★ | → 持续热点 | 基础方向，不断结合新方法 |
| 🥇 智能控制 (RL/NN) | ★★★★☆ | ↑ **增长最快** | 近年热度飙升 |
| 🥇 空间捕获与复合体稳定 | ★★★★★ | → **空间机械臂核心** | 中国空间站建设驱动 |
| 🥈 振动抑制/柔性结构 | ★★★★☆ | → 稳步推进 | 与柔性关节、臂杆发展相关 |
| 🥈 力控制/柔顺交互 | ★★★★☆ | ↑ 增长快 | SEA、无传感器力估计算法 |
| 🥈 高精度轨迹跟踪 | ★★★★☆ | → 持续热点 | FASA、DR-MPC 等新方法 |
| 🥉 约束控制 | ★★★☆☆ | → 稳步发展 | 预设性能控制成为趋势 |
| 🥉 无扰控制 | ★★★☆☆ | → 空间特有 | 自由漂浮基座必需 |
| 🥉 连续型/软体臂 | ★★☆☆☆ | ↑ 新兴方向 | 碎片清除新方案 |
| 🥉 容错/可靠性 | ★★★☆☆ | ↑ 新兴方向 | 长寿命任务需求驱动 |

---

## 与本研究方向的关联

> 本课题（空间机械臂论文）的核心方向涉及 **1️⃣ 运动规划与控制**、**4️⃣ 空间目标捕获**、**2️⃣ 振动抑制**、**12️⃣ 高精度轨迹跟踪控制** 等多个方向，重点关注捕获过程中的碰撞动力学建模、捕获后复合体稳定策略、以及非合作目标的接近与跟踪控制。上述文献中：
> - **id: 2093**（严宇新，2025）直接研究"面向在轨捕获的空间机械臂运动规划与控制"，最为贴近
> - **id: 72**（詹博文，2022）全面覆盖捕获后复合体稳定策略，含避障、振动抑制、基座无扰
> - **id: 1010**（张龙，2017）专注于捕获碰撞动力学建模与控制
> - **id: 3150**（Tian et al., 2024）涉及多约束高精度跟踪，可用于捕获阶段的接近控制

---

*报告生成时间: 2025年*  
*数据源: MCP okb-assist 知识库（3814 篇文献索引）*