# 多智能体预设时间编队/合围/跟踪控制文献综述

> 基于 okb-assist 知识库检索整理，涵盖 **预设时间编队控制**、**编队合围/围捕 (fencing)**、**编队跟踪 (formation tracking)**、**编队-合围 (formation-containment)** 及相关异构/混阶系统研究。

---

## 1. 预设时间编队控制 (Prescribed-Time Formation Control)

预设时间控制的核心特征是收敛时间可作为用户设计的参数显式给定，且与系统初态无关。该类研究通常引入时变缩放函数（time-varying function）将原系统映射到新的时间尺度，从而实现“预设时间”收敛。

| 文献 | 年份 | 期刊 | 主要贡献 |
|------|------|------|----------|
| Ding T-F, Ge M-F, Xiong C, et al. *Prescribed-time formation tracking of second-order multi-agent networks with directed graphs* | 2023 | Automatica | 针对二阶多智能体网络，在有向图含生成树的条件下，提出基于时变函数的预设时间观测器算法，使跟随者在预设时间内跟踪动态领导者并形成编队。 |
| Shi Z, Lin Z, Feng Z, et al. *Prescribed-Time Time-Varying Formation for Multiple Euler-Lagrange Systems Using Only Position Measurements* | 2026 | IEEE Trans. Ind. Electron. | 仅利用位置测量实现多 Euler-Lagrange 系统的预设时间时变编队。 |
| *Adaptive fuzzy prescribed-time formation control for nonlinear multi-agent systems* | 2025 | IEEE Trans. Autom. Sci. Eng. | 针对有向拓扑下的非线性多智能体系统，提出基于模糊逻辑系统（FLS）逼近未知非线性的预设时间编队协议，并结合自适应技术消除不利影响。 |
| *Prescribed-time formation tracking control against corrupted outputs: a hierarchical event-triggered approach* | 2026 | Nonlinear Dynamics | 研究受污染输出测量下的领导者-跟随者时变输出编队跟踪问题，提出分层事件触发机制，兼顾预设时间与通信效率。 |
| *Reinforcement learning-based prescribed-time optimal formation control for multiagent systems* | 2025 | IEEE Trans. Autom. Sci. Eng. | 将预设收敛时间性能嵌入最优控制框架，基于 Actor-Critic 结构设计强化学习最优编队控制器。 |
| *Neural network-based reinforcement learning for prescribed-time formation tracking of hybrid networked marine systems under attacks* | 2025 | ISA Trans. | 针对水面-水下混合网络海洋系统（ASV/AUV），在 DoS 与物理攻击下提出预设时间分层控制框架实现时变编队跟踪。 |

### 关键技术点
- **时变缩放函数**：通过引入随时间趋于无穷的时变增益，将有限时间区间映射到无限时间区间。
- **分布式观测器**：估计领导者状态/输入，通常在预设时间内收敛。
- **自适应 / 智能方法**：模糊逻辑系统、神经网络、强化学习用于处理未知非线性与最优性。

---

## 2. 编队跟踪控制 (Formation Tracking Control)

编队跟踪要求跟随者不仅形成期望几何构型，同时跟踪一个动态领导者或参考轨迹。按收敛特性可分为有限时间、固定时间和预设时间等。

### 2.1 有限时间编队跟踪

| 文献 | 年份 | 期刊 | 主要贡献 |
|------|------|------|----------|
| *Finite-time formation tracking for heterogeneous euler–lagrange systems with an uncertain leader* | 2025 | Automatica | 针对具有不确定领导者的异构 Euler-Lagrange 系统，提出无需领导者动力学知识的分布式观测器有限时间编队跟踪框架。 |
| *Robust decentralized formation tracking control of complex multiagent systems* | 2024 | Complexity | 在输出约束与外部扰动下，利用障碍 Lyapunov 函数保证输出不越界，实现有限时间分散时变编队跟踪。 |
| *Fully distributed finite-time formation tracking of heterogeneous multi-agent systems under directed networks* | 2025 | IEEE Trans. Autom. Sci. Eng. | 针对高阶异构系统，在领导者输入未知情况下提出完全分布式有限时间控制策略，无需全局拓扑信息。 |
| *Command filtered finite-time formation tracking control of AUVs with unknown control directions* | 2020 | IET Control Theory Appl. | 针对 AUV 的有限时间编队跟踪，处理未知控制方向问题。 |

### 2.2 固定时间编队跟踪

| 文献 | 年份 | 期刊 | 主要贡献 |
|------|------|------|----------|
| *Fixed-time formation tracking for multiple nonholonomic wheeled mobile robots based on distributed observer* | 2021 | Nonlinear Dynamics | 有向固定/切换拓扑下，通过非线性变换将编队控制转化为一致性问题，基于分布式观测器设计固定时间编队跟踪协议。 |
| Cheng W, Zhang K, Jiang B, Ding S X. *Fixed-time formation tracking for heterogeneous multiagent systems under actuator faults and directed topologies* | 2022 | IEEE Trans. Aerosp. Electron. Syst. | 在执行器故障与有向拓扑下实现异构多智能体系统的固定时间编队跟踪。 |
| *Distributed Fixed-Time Leader-Following Formation Control for Multiquadrotors With Prescribed Performance and Collision Avoidance* | 2023 | IEEE Trans. Aerosp. Electron. Syst. | 针对多四旋翼系统设计固定时间估计器，结合预定性能控制与避碰。 |

### 2.3 输出编队跟踪与事件触发

| 文献 | 年份 | 期刊 | 主要贡献 |
|------|------|------|----------|
| *Output formation tracking for heterogeneous mass with incomplete leader measurements: a hybrid-triggering observer approach* | 2026 | IEEE Trans. Autom. Sci. Eng. | 在联合可观测条件（JOC）下，针对领导者测量信息不完整的异构系统，提出混合触发观测器实现时变输出编队跟踪。 |
| *Distributed extended state observer-based formation control of flight vehicles subject to constraints on speed and acceleration* | 2025 | IEEE Trans. Cybern. | 基于分布式扩张状态观测器，在速度与加速度约束下实现飞行器编队控制。 |

---

## 3. 编队合围 / 围捕控制 (Formation Fencing / Encirclement)

合围/围捕控制要求多智能体围绕一个或多个机动目标形成包围构型，常用于无人机/无人车/无人艇集群对非合作目标的跟踪与拦截。

| 文献 | 年份 | 期刊 | 主要贡献 |
|------|------|------|----------|
| *Cooperative target tracking-fencing control for heterogeneous MAS* | 2025 | Syst. Control Lett. | 提出分层协同控制方案，实现状态估计、跟踪-合围与导引的多目标协同；设计预设时间分布式观测器估计未知目标运动。 |
| *Self-Organized Fencing Control of Multi-AUV Systems Under Limited Sensing and Nonuniform Acoustic Communication Delays* | 2026 | J. Mar. Sci. Eng. | 在有限感知范围与非均匀水声通信时延下，提出自组织动态合围控制，无需固定角度槽、虚拟领导者或持久角色分配。 |
| *Cooperative fencing control for uncertain multiple nonholonomic vehicles with self-organized formation* | 2026 | Nonlinear Dyn. | 针对不确定非完整多车系统，对变速运动目标进行自组织编队合围；设计分布式观测器使部分车辆可恢复目标状态。 |
| *Adaptive cooperative fencing and dynamic reorganization control of UAV swarm for multiple maneuvering targets* | 2026 | Complex Intell. Syst. | 针对多机动目标的无人机集群合围，提出自组织协同合围策略，具备自适应任务调度能力；设计分布式预设时间目标状态观测器。 |
| *Learning-based target fencing control for delay-tolerant unmanned aerial vehicle swarm* | 2026 | Eng. Appl. Artif. Intell. | 在通信时延下，基于 RBF-BELBIC 提出自适应自组织控制框架；设计固定时间收敛观测器估计目标多状态。 |
| *Cooperative moving-target fencing control of multi-mobile-manipulator vehicles with a dual-layer architecture* | 2026 | Int. J. Robust Nonlinear Control | 针对三维空间中恒定水平速度移动目标，基于移动操作臂双层协同控制框架实现合围。 |
| *Distributed observer-based formation-fencing control for multi-unmanned aerial vehicles* | 2025 | IEEE Trans. Veh. Technol. | 针对参数不确定、外部扰动与输入饱和下的多 UAV，设计固定时间分布式观测器，实现从编队控制到合围控制的过渡。 |

### 关键技术点
- **目标状态估计**：分布式/集中式观测器，在预设时间或固定时间内估计目标位置、速度、加速度。
- **自组织 (self-organized)**：无需预先分配角度、角色或虚拟领导者。
- **吸引-排斥机制**：目标-智能体径向吸引/排斥 + 智能体间分布式排斥，形成稳定包围圈。

---

## 4. 编队-合围控制 (Formation-Containment Control)

编队-合围控制是编队控制与包含控制（containment control）的结合：一部分跟随者形成指定编队，另一部分跟随者进入由多个领导者形成的凸包内。

| 文献 | 年份 | 期刊 | 主要贡献 |
|------|------|------|----------|
| Cheng M, Liu H, Wen G, Lü J, Lewis F L. *Data-driven time-varying formation-containment control for a heterogeneous air-ground vehicle team subject to active leaders and switching topologies* | 2023 | Automatica | 针对异构空地车队的数据驱动时变编队-合围控制，处理主动领导者与切换拓扑。 |
| Wang Q, Ge Z, Jin S, Hou Z. *Model-Free Adaptive Formation-Containment Control for UAV–UGV Systems Under Fading Channels* | 2026 | IEEE Trans. Cybern. | 针对 UAV-UGV 系统，在衰落信道下实现无模型自适应编队-合围控制。 |
| *Fully distributed output formation-containment control of heterogeneous multi-agent systems via adaptive observer approach* | 2026 | ISA Trans. | 将输出编队-合围问题转化为协同输出调节问题，基于完全分布式自适应观测器实现。 |
| *Two-layer distributed formation-containment control of multiple euler–lagrange systems by output feedback* | 2019 | IEEE Trans. Cybern. | 针对多 Euler-Lagrange 系统，提出两层分布式编队-合围框架，仅利用输出反馈。 |
| Wang L B, et al. *Formation-containment tracking intelligent control for heterogeneous swarm systems under unknown input* | 2023 | SCIENTIA SINICA Technologica | 针对异构集群系统，在领导者输入未知情况下研究编队-合围跟踪智能控制。 |

---

## 5. 异构 / 混阶多智能体系统 (Heterogeneous / Mixed-Order MAS)

实际集群中智能体往往具有不同阶次或不同动力学模型（异构），近年来混阶系统研究受到关注。

| 文献 | 年份 | 期刊 | 主要贡献 |
|------|------|------|----------|
| *Cooperative tracking control of heterogeneous mixed-order multiagent systems with higher-order nonlinear dynamics* | 2022 | IEEE Trans. Cybern. | 研究具有不同一阶、二阶甚至高阶非线性的异构混阶多智能体系统的有限时间协同跟踪。 |
| *Event-triggered observation-based control of nonlinear mixed-order multiagent systems under input saturation* | 2024 | IEEE Syst. J. | 研究输入饱和下非线性混阶多智能体系统的事件触发观测器协同跟踪控制。 |
| 赵阳. *基于事件触发的异构无人机集群分布式编队控制研究* | 2021 | 博士论文 | 针对异构无人机集群的事件触发分布式编队控制。 |

---

## 6. 有限/固定时间编队控制 (Finite/Fixed-Time Formation)

| 文献 | 年份 | 期刊 | 主要贡献 |
|------|------|------|----------|
| *Finite-time output-feedback formation control for high-order nonlinear multiagent systems with obstacle avoidance* | 2024 | IEEE Trans. Autom. Sci. Eng. | 针对高阶非线性多智能体系统，考虑障碍规避、不可测状态与死区输入，设计神经网络 k-滤波观测器与积分-乘法正切 Lyapunov-Barrier 函数。 |
| *Fixed-time formation control of multirobot systems: design and experiments* | 2019 | IEEE Trans. Ind. Electron. | 考虑视觉多机器人系统中的时延约束，采用预测器状态变换处理输入时延，实现固定时间编队控制并实验验证。 |
| *Distributed finite-time tracking control for multi-agent systems: an observer-based approach* | 2013 | Syst. Control Lett. | 针对二阶多智能体系统，提出基于观测器的有限时间跟踪控制算法，仅需共享位置信息。 |

---

## 7. 专著/综述资源

| 文献 | 年份 | 类型 | 说明 |
|------|------|------|------|
| 《集群系统编队合围控制理论及应用》 | 2025 | 专著 | 系统介绍一致性控制、编队控制、编队跟踪控制、合围控制及编队-合围控制五方面内容，并在无人机集群上应用验证。 |

---

## 8. 研究趋势与展望

1. **时间约束精细化**：从有限时间、固定时间向预设时间/预定义时间发展，收敛时间成为可设计参数。
2. **异构与混阶**：实际集群智能体动力学差异显著，异构/混阶系统控制是重要方向。
3. **智能方法融合**：神经网络、模糊系统、强化学习与预设/固定时间控制结合，处理未知非线性与最优性。
4. **通信与计算效率**：事件触发、动态事件触发、混合触发机制减少通信负担与执行器磨损。
5. **安全性与鲁棒性**：输出约束、输入饱和、执行器故障、DoS 攻击、通信时延/衰落、避碰等实际问题日益受到重视。
6. **合围任务扩展**：从单目标合围到多目标合围，从固定角色到自组织动态重组。

---

## 附录：按主题快速索引

### 预设时间编队/跟踪
- [1] Ding et al., Automatica, 2023
- [2] Shi et al., IEEE TIE, 2026
- [3] Adaptive fuzzy PTF, IEEE TASE, 2025
- [4] Hierarchical event-triggered PT formation tracking, Nonlinear Dyn., 2026
- [5] RL-based PT optimal formation, IEEE TASE, 2025
- [6] NN-based RL for PT formation tracking of marine systems, ISA Trans., 2025

### 编队合围/围捕
- [7] Cooperative target tracking-fencing, Syst. Control Lett., 2025
- [8] Self-organized fencing of multi-AUV, JMSE, 2026
- [9] Cooperative fencing for nonholonomic vehicles, Nonlinear Dyn., 2026
- [10] Adaptive cooperative fencing for UAV swarm, Complex Intell. Syst., 2026
- [11] Learning-based target fencing for UAV swarm, Eng. Appl. AI, 2026
- [12] Fencing of mobile manipulators, IJRNC, 2026
- [13] Formation-fencing for UAVs, IEEE TVT, 2025

### 编队-合围
- [14] Data-driven TV formation-containment, Automatica, 2023
- [15] Model-free adaptive formation-containment, IEEE TCYB, 2026
- [16] Fully distributed output formation-containment, ISA Trans., 2026
- [17] Two-layer DFC for Euler-Lagrange systems, IEEE TCYB, 2019

### 异构/混阶系统
- [18] Mixed-order MAS cooperative tracking, IEEE TCYB, 2022
- [19] Event-triggered mixed-order MAS with input saturation, IEEE Syst. J., 2024
- [20] 赵阳，异构无人机集群分布式编队控制，博士论文，2021

---

*整理时间：2026-11-02*  
*来源：okb-assist 知识库*
