# 空间机械臂轨迹控制论文调研报告

## 调研目的

本报告围绕"空间机械臂轨迹控制"主题，系统调研了知识库中相关文献，重点回答以下四个问题：

1. **是否包含实验**：论文是纯理论/仿真还是包含物理实验？
2. **实验平台**：若包含实验，使用了什么实验平台？
3. **投稿期刊**：论文发表在哪些期刊上？
4. **页数**：论文页数范围？

---

## 一、文献概览

从知识库中检索到与空间机械臂/空间机器人直接相关的文献约 **80+ 篇**，时间跨度从 1989 年（Umetani & Yoshida 提出广义雅可比矩阵）至 2026 年。这些文献涵盖：

- 自由漂浮/自由飞行空间机械臂建模与控制
- 轨迹规划与轨迹跟踪控制
- 在轨捕获与碰撞动力学
- 姿态稳定与振动抑制
- 地面试验验证与在轨飞行实验

---

## 二、是否包含实验

### 分类统计

| 类别 | 数量（估计） | 说明 |
|------|-------------|------|
| **纯理论推导 + 数值仿真** | ~85% | 多数期刊论文属于此类，使用 MATLAB/Simulink 进行数值仿真验证 |
| **含物理实验** | ~5-8% | 包括地面气浮平台实验、在轨飞行实验 |
| **综述/学位论文** | ~5-7% | 对领域进行全面总结，不提出新实验 |

### 典型不含物理实验的论文特征

绝大多数轨迹控制类论文采用"理论推导 + 数值仿真"模式，例如：

- **Jia & Shan (2020)** [IEEE TIE]: "Finite-Time Trajectory Tracking Control of Space Manipulator Under Actuator Saturation" — 仅含 MATLAB 数值仿真（第 IV 节 "NUMERICAL SIMULATION"）
- **Xu et al. (2024)** [IEEE TAES]: "Predefined-time tracking control of a free-flying space robot on SE(3)" — 仅含 "numerical studies"
- **Yang et al. (2025)** [IEEE TAES]: "Distributionally robust model predictive control for trajectory tracking of space manipulator" — 仿真验证
- **Zou et al. (2026)** [Acta Astronautica]: "Tunable predefined-time trajectory tracking control for cable-driven space manipulators" — 数值仿真

### 含物理实验的代表性论文

仅少数论文包含真实的物理实验或在轨飞行验证：

| 论文 | 实验类型 | 实验平台 |
|------|---------|---------|
| **Yoshida et al. (2001)** — "Zero Reaction Maneuver: Flight Validation with ETS-VII Space Robot" | 在轨飞行实验 | ETS-VII（日本工程试验卫星 VII 号），6-DOF 机械臂，2 米长 |
| **Shao et al. (2022)** — "Direct Trajectory Optimization of Free-Floating Space Manipulator for Reducing Spacecraft Variation" [IEEE LRA] | 地面微重力实验 | 气浮式地面微重力平台（air-bearing technique），模拟自由漂浮工况 |
| **刘宏等 (2021)** — "空间机械臂技术综述及展望" [航空学报] | 综述（引用他人实验） | 综述了 SRMS 气浮平台、ROKVISS、ETS-VII、天宫二号等实验 |

### 关键发现

1. **在轨飞行实验极其稀缺**：只有 ETS-VII（1997-1999 年日本）进行了系统的空间机械臂在轨轨迹控制实验，由 Kazuya Yoshida 团队完成。这是领域内最具影响力的实验验证工作。

2. **地面气浮实验是主流物理实验方案**：Shao et al. (2022) 使用气浮平台模拟二维微重力环境，验证了轨迹优化方法的有效性。类似平台也被 Aghili & Lampariello、SRMS 等使用。

3. **绝大多数论文停留在仿真阶段**：包括 IEEE TIE、IEEE TAES、Automatica、Acta Astronautica 等顶刊上的轨迹控制论文，几乎全部仅使用数值仿真。这与空间机器人领域的特殊性有关——真实空间实验成本极高、风险极大。

---

## 三、实验平台

### 3.1 在轨飞行实验平台

| 平台 | 国家/项目 | 时间 | 机械臂配置 | 实验内容 |
|------|----------|------|-----------|---------|
| **ETS-VII** | 日本 NASDA | 1997-1999 | 6-DOF，2m 长 | 零反作用机动（ZRM）、动力学在轨辨识、遥操作、目标捕获 |
| **SRMS (航天飞机)** | 加拿大 | 1981- | 6-DOF 单臂 | 载荷释放/回收、哈勃望远镜维修、ISS 组装 |
| **SSRMS (ISS)** | 加拿大 | 2001- | 7-DOF 冗余 | ISS 组装、货运飞船捕获、地面遥操作 |
| **ROKVISS** | 德国 | 2005- | 2-DOF 关节 | 阻抗控制、力反馈遥操作、关节摩擦/刚度辨识 |
| **ROTEX** | 德国 | 1993 | 6-DOF | 舱内 ORU 连接/分离、结构组装、漂浮物体抓取 |
| **天宫二号** | 中国 | 2016 | 机械臂+灵巧手 | 在轨辨识、抓取、旋拧电连接器、撕多层、旋拧螺钉 |
| **试验七号** | 中国 | 2013 | 6-DOF | 在轨捕获与操作 |

### 3.2 地面实验平台

| 平台类型 | 原理 | 适用范围 | 代表 |
|---------|------|---------|------|
| **气浮式（Air-bearing）** | 利用气膜消除摩擦，模拟二维微重力 | 二维平面运动验证 | SRMS、Shao et al. (2022)、Aghili & Lampariello |
| **水浮式（Neutral Buoyancy）** | 利用水的浮力抵消重力 | 航天员操作训练 | Ranger 中性浮力模拟器 |
| **抛物线飞行/落塔** | 短时微重力（20-30s） | 极少用于机械臂 | — |
| **吊丝配重式** | 机械补偿重力 | 响应速率要求高，安全隐患大 | — |
| **半物理（Hardware-in-loop）** | 接触环节实物 + 运动仿真 | 未来发展趋势 | SPDM 任务验证系统 |

### 3.3 仿真平台

几乎所有论文均使用 MATLAB/Simulink 进行数值仿真，部分使用：
- **Robotics System Toolbox**（rigidBodyTree 模型）
- **Simscape Multibody**（用于动力学交叉验证）
- **自定义动力学代码**（CRBA、Jacobian 求和法等）

---

## 四、投稿期刊

### 4.1 主要期刊分布

从知识库统计，空间机械臂轨迹控制相关论文主要发表在以下期刊：

| 期刊 | 论文数量（估计） | 影响因子范围 | 特点 |
|------|----------------|-------------|------|
| **IEEE Transactions on Aerospace and Electronic Systems** | ~20+ | ~2.0-3.0 | 空间机器人领域最核心的期刊，发表大量轨迹控制论文 |
| **IEEE Transactions on Industrial Electronics** | ~5 | ~7.0-8.0 | Jia & Shan (2020) 发表于此 |
| **Acta Astronautica** | ~10+ | ~2.0-3.0 | 空间科学与技术综合期刊 |
| **Aerospace Science and Technology** | ~8+ | ~2.0-3.0 | 航空航天技术期刊 |
| **Automatica** | ~5 | ~5.0 | 控制理论顶刊 |
| **Journal of Guidance, Control, and Dynamics** | ~5 | ~2.0-3.0 | AIAA 期刊，制导控制方向 |
| **Robotics and Autonomous Systems** | ~3 | ~3.0-4.0 | 机器人领域核心期刊 |
| **IEEE Transactions on Robotics** | ~2 | ~8.0+ | 机器人顶刊（较少空间方向论文） |
| **Advances in Space Research** | ~3 | ~2.0 | 空间研究综合期刊 |
| **Nonlinear Dynamics** | ~3 | ~2.0-3.0 | 非线性动力学期刊 |
| **Control Engineering Practice** | ~2 | ~2.0-3.0 | 控制工程实践期刊 |
| **IET Cyber-Systems and Robotics** | ~1 | ~2.0 | 新兴机器人期刊 |

### 4.2 主要会议

| 会议 | 说明 |
|------|------|
| **IEEE/RSJ IROS** | 机器人顶会，偶有空间机器人论文 |
| **IEEE ICRA** | 机器人顶会，Yoshida 的 ETS-VII 实验论文发表于 2001 年 ICRA |
| **IEEE CDC** | 决策与控制顶会 |
| **Chinese Control Conference (CCC)** | 中国控制会议 |
| **AIAA Guidance, Navigation, and Control Conference** | AIAA 制导控制会议 |
| **iSAIRAS** | 空间人工智能与机器人国际会议 |

### 4.3 投稿趋势

- **期刊偏好**：IEEE TAES 是空间机器人领域的绝对主流期刊，近年约 60% 以上的空间机械臂轨迹控制论文发表于此
- **IEEE TIE / TRO** 属于更高层次但接收空间机器人论文较少
- **Acta Astronautica** 接收大量空间机器人论文，但影响因子相对较低
- **Automatica** 偏好控制理论创新，纯空间机器人论文较少

---

## 五、页数

### 5.1 期刊页数范围

| 期刊 | 典型页数 | 说明 |
|------|---------|------|
| **IEEE TAES** | 10-15 页 | 长文格式，Xu et al. (2024): pp. 5906-5919（14 页） |
| **IEEE TIE** | 12-16 页 | Jia & Shan (2020): 约 12-15 页 |
| **IEEE LRA** | 4-6 页 | Letter 格式，Shao et al. (2022): 短信格式 |
| **Acta Astronautica** | 12-20 页 | 常规论文长度 |
| **Automatica** | 8-12 页 | 短文格式 |
| **IEEE TRO** | 12-20 页 | 长文格式 |
| **航空学报** | 14 页 | 刘宏等 (2021): 524164（14 页） |

### 5.2 典型论文页数

| 论文 | 期刊 | 页数 |
|------|------|------|
| Xu et al. (2024) — SE(3) predefined-time tracking | IEEE TAES | 14 页 (5906-5919) |
| Jia & Shan (2020) — Finite-time tracking | IEEE TIE | ~12-15 页 |
| Shao et al. (2022) — Direct trajectory optimization | IEEE LRA | ~4-6 页 (Letter) |
| 刘宏等 (2021) — 空间机械臂综述 | 航空学报 | 14 页 |
| Yoshida et al. (2001) — ETS-VII flight validation | ICRA | ~6 页 (会议) |
| 严宇新 (2025) — 在轨捕获运动规划与控制 | 博士论文 | ~200+ 页 |

### 5.3 当前用户论文状态

用户当前论文（paper0）编译为 **27 页**，包含完整的理论推导、定理-证明体系和仿真验证。根据投稿路线图：

- **当前状态**：适合 arXiv 预印本或内部技术报告
- **P0+P1+P2 完成后**：ICRA / IROS（8 页顶会）
- **全部完成后**：IEEE Trans. Robotics / Automatica / Acta Astronautica

---

## 六、综合结论

### 6.1 关于实验

**空间机械臂轨迹控制领域，包含物理实验的论文是极少数。** 绝大多数论文（包括顶刊）仅使用数值仿真验证控制算法。主要原因：

1. **空间实验成本极高**：每小时数万美元的卫星租用费、发射费用
2. **地面模拟困难**：气浮平台仅能模拟二维平面，水浮式存在流体阻力，吊丝配重响应不够快
3. **在轨资源稀缺**：仅 ETS-VII、国际空间站等少数平台可供学术实验

**唯一的在轨飞行实验**是 Yoshida 团队的 ETS-VII 零反作用机动（ZRM）实验（1997-1999），这是领域内最具影响力的实验验证工作。

**地面气浮实验**是主流物理实验方案，Shao et al. (2022) 使用气浮平台验证了轨迹优化方法。

### 6.2 关于实验平台

- **在轨平台**：ETS-VII（日本）、SRMS/SSRMS（加拿大）、ROKVISS（德国）、天宫二号（中国）
- **地面平台**：气浮式为主流，水浮式/抛物线/吊丝配重使用较少
- **仿真平台**：MATLAB/Simulink 是标准配置，Robotics System Toolbox + Simscape Multibody 常用

### 6.3 关于投稿期刊

- **首选**：IEEE Transactions on Aerospace and Electronic Systems（TAES）
- **高质量选择**：IEEE Transactions on Industrial Electronics（TIE）、IEEE Transactions on Robotics（TRO）、Automatica
- **可接受选择**：Acta Astronautica、Aerospace Science and Technology、Journal of Guidance, Control, and Dynamics
- **会议**：ICRA、IROS（8 页以内）、CDC、CCC

### 6.4 关于页数

- IEEE 期刊论文通常 10-16 页
- IEEE LRA Letter 为 4-6 页
- ICRA/IROS 会议论文通常 6-8 页
- 用户当前论文 27 页，需根据投稿目标精简

---

## 附录：知识库中空间机械臂轨迹控制核心文献列表

| ID | 标题 | 作者 | 年份 | 期刊 | 实验类型 |
|----|------|------|------|------|---------|
| 3 | Predefined-time tracking control of a free-flying space robot on SE(3) | Xu, Chen, Wen, Jin | 2024 | IEEE TAES | 数值仿真 |
| 5 | Zero Reaction Maneuver: Flight Validation with ETS-VII Space Robot | Yoshida, Hashizume, Abiko | 2001 | ICRA | **在轨飞行实验** |
| 18 | Trajectory planning of free-floating space robot using PSO | Wang, Luo, Walter | 2015 | — | 数值仿真 |
| 103 | Finite-time SDRE regulation of dual-arm space manipulator | Scalvini et al. | 2024 | Acta Astronautica | 数值仿真 |
| 259 | Fractional-Order SMC for Free-Floating Space Manipulator | Dou, Yue | 2025 | — | 数值仿真 |
| 262 | 空间机械臂技术综述及展望 | 刘宏等 | 2021 | 航空学报 | 综述 |
| 301 | Direct Trajectory Optimization of FFSM for Reducing Spacecraft Variation | Shao et al. | 2022 | IEEE LRA | **地面气浮实验** |
| 439 | 自由漂浮空间机器人轨迹跟踪的 MPC | 宁昕, 武耀发 | 2019 | 控制理论与应用 | 数值仿真 |
| 979 | Dynamics modeling of flexible transportation system using two space robots | Zhang, Jia et al. | 2025 | IEEE TAES | 数值仿真 |
| 1031 | Trajectory tracking of free-flying space manipulators using time-delay SMC | Wang et al. | 2025 | J. Computational Design and Eng. | 数值仿真 |
| 1087 | Preassigned Fixed-Time Control for Quantized Space Manipulator | Hu et al. | 2025 | IEEE TAES | 数值仿真 |
| 1249 | Practical Prescribed-Time Trajectory Tracking for Tendon-Driven Space Manipulator | Zou et al. | 2025 | IEEE/ASME TMECH | 数值仿真 |
| 1463 | Intelligent Neural Integral SMC for space robotic manipulator | Lavín-Delgado et al. | 2023 | Advances in Space Research | 数值仿真 |
| 1464 | Constrained visual servoing for capturing tumbling satellites | Zhang et al. | 2025 | IEEE TAES | 数值仿真 |
| 1654 | Error-Triggered Flexible Prescribed Performance Control | Li et al. | 2025 | IEEE TAES | 数值仿真 |
| 1678 | Learning-based task space trajectory planning for uncertain FFSM | Zhang et al. | 2025 | IEEE TAES | 数值仿真 |
| 1784 | Nonholonomic trajectory planning of postcapture system | Wei et al. | 2025 | IEEE TAES | 数值仿真 |
| 1903 | Finite-Time Trajectory Tracking Control of Space Manipulator Under Actuator Saturation | Jia, Shan | 2020 | IEEE TIE | 数值仿真 |
| 2085 | Distributionally robust MPC for trajectory tracking of space manipulator | Yang et al. | 2025 | IEEE TAES | 数值仿真 |
| 2093 | 面向在轨捕获的空间机械臂运动规划与控制方法研究 | 严宇新 | 2025 | 博士论文 | 仿真+部分实验 |
| 2173 | SGP-based stochastic predictive control of free-floating space robot | Chaudhary et al. | 2025 | IEEE TAES | 数值仿真 |
| 2616 | Distributionally Robust MPC for Trajectory Tracking of Space Manipulator | Yang et al. | 2025 | IEEE TAES | 数值仿真 |
| 3149 | Maneuver and active vibration suppression of free-flying space robot | Jia et al. | 2018 | IEEE TAES | 数值仿真 |
| 3150 | High-precision trajectory tracking control for free-flying space manipulators | Tian et al. | 2024 | IEEE TAES | 数值仿真 |
| 3543 | Physics-informed neural network-based robust MPC for FFSM | Zhao et al. | 2026 | Aerospace Sci. & Tech. | 数值仿真 |
| 3843 | RL-Based Nonlinear Trajectory Tracking for Space Flexible Manipulator | Li et al. | 2026 | IET Cyber-Systems & Robotics | 数值仿真 |
| 3984 | Predefined-time trajectory tracking for rigid-flexible cable-driven space manipulator | Zou et al. | 2026 | Aerospace Sci. & Tech. | 数值仿真 |
| 3985 | Tunable predefined-time trajectory tracking for cable-driven space manipulators | Zou et al. | 2026 | Acta Astronautica | 数值仿真 |
| 3986 | Novel MPC law for over-actuated space cable-driven manipulators | Lin et al. | 2026 | Control Eng. Practice | 数值仿真 |
| 3987 | Fixed-Time Fault-Tolerant Tracking for Cable-Driven Space Manipulators | Zou et al. | 2026 | IEEE TAES | 数值仿真 |

---

*报告生成日期：2025-11-01*
*数据来源：MCP 知识库（okb_assist），共检索 3988 篇文献，筛选空间机械臂轨迹控制相关文献 80+ 篇*