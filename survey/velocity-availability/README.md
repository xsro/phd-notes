我在 OKB（MCP 里的文献库，3114 篇已索引）里做了一轮全文正则检索，把「论文需要速度信息时，怎么说明它是可获取的 / 怎么获取」这两种写法整理如下。

## 一、论文里"申明速度可获取"的 4 种典型写法

**1) 直接假设可测（最省事，放在问题描述里）**
- `[636] 2007 Higher order sliding mode control based on integral sliding mode`："The velocity is supposed to be known with δw = 5% of error"（假设已知 + 给误差界，比裸假设更可信）
- `[1844] 2023 Safe Control of Euler-Lagrange Systems`：`ω` 精确值未知，记测得速度为 `ω̂`，并给出界 —— 退一步的说法。
- 中文对应：`[1480] 2021` 指出同类文献"大多假设目标速度与加速度可测"。

**2) 承认速度不可测/难测，再给工程理由（审稿人最吃这套）**
- `[1933] Li et al. 2023, IEEE TAC`（同 `[3082]` 博士论文版）："velocity sensors are expensive and may introduce additional noise"；控制律 (20) 依赖相对速度，所以另给无速度版本。
- `[151] Adaptive trajectory tracking of free-flying space manipulator`："the velocity … can not be measured as accurately as the position especially when the velocity sensors are with faults or great noises … even not installed with velocity sensors due to the technical limitations"。
- `[901] 2015 Flexible-joint free-floating space manipulators`："it may not be always possible to measure these velocities, or it may not even be desirable … signals delivered by tachometers might be contaminated with noise. Also, costs are increased when more sensors are used."
- `[75] 2024 Distributed weighted centroid formation tracking`："obtaining full states … particularly velocity measurements, can present substantial challenges due to constraints in sensor equipment, cost considerations, or environmental factors"。
- `[2049] 2022 Velocity-free trajectory tracking … flexible space manipulator`：姿态角速率传感器"poor quality / sensor anomaly / completely unavailable"。

**3) 承认可获取，但说明获取方式与代价（"两条路"写法，最严谨）**
- `[607] 2018 Affine formation maneuver control`："The absolute velocity ṗ_j may be transmitted from agent j to agent i via wireless communication **or** obtained by differentiating the position measurement p_j. Both … will result in measurement errors due to, e.g., communication delays."
- `[1080] 2018 Spacecraft formation flying`："Velocity can either be obtained from the Doppler shift … **or** by differentiating the position. The latter … will give more erroneous result if there is noise … A robust observer can be utilized for obtaining the velocity as well as disturbance estimates."

**4) 中文论文的措辞模板**
- `[1480] 2021`（事件触发 ESO 环绕控制）："量测信息不够丰富，如速度、加速度往往不可测或测不准"，构造 FTESO "在非周期采样条件下对于相对速度不可测和集总干扰未知的精准估计"。
- `[2250] 2025`（多智能体协同跟踪）："针对非均匀时滞网络下速度不可测二阶多智能体系统 … 引入基于能量耗散…"。

## 二、"如何获取"的 4 类手段（附可引原文）

| 手段 | 说明与代表文献 |
|---|---|
| **直接传感器** | 编码器+直流测速机：`[1927]` "500 pulses/turn incremental encoder and a DC tachometer making q₂ and q̇₂ available for feedback"；陀螺：`[772] 2010` "Angular velocities are obtained from three gyroscopes (Murata ENC-03R)"；多普勒/惯导：`[1080]` |
| **差分 + 滤波**（无测速机时最常见） | `[774] 2005`："Since no tachometers are present for velocity measurements, link velocity signals are calculated via a **filtered backward difference** algorithm"；`[1927]`：q̇₁ "obtained by differentiating the position signal (a filtering action…)"；`[495]`："velocity is obtained by numerically differentiating their position **followed by a first order low pass filter**"；`[166] 2012`：VICON 位姿 "numerically differentiated to compute the linear and angular velocities"；`[3808] 2018`："position signals are filtered and the velocity is estimated using a **discrete derivative algorithm**" |
| **微分器设计**（差分太噪，需要理论保证） | `[2307] 2022 A linear differentiator based on the extended dynamics approach`："owing to the sensitiveness to the high-frequency noise, it is almost not feasible to differentiate the measured signal directly … the differentiator is an artificially designed mechanism"；`[789] 2013 Robust exact uniformly convergent arbitrary order differentiator`（HOSM 微分器） |
| **观测器 / 状态估计** | `[151]`："The velocity observers are indispensable for the output feedback control. By using the velocity observers to recover the velocity information…"；`[901]` 基于观测器的自适应控制；`[1080]` HOSM 观测器同时估速度与扰动；`[1480]` FTESO；`[2250]/[2247]`：高增益观测器、二阶滑模观测器；`[75] 2024`：fixed-time 估计 |
| **通信/邻居获取** | `[546] 2016`：控制律要求邻居相对速度可得，否则控制律不成立；`[607]`：速度可经无线通信从邻居传过来 |

## 三、可直接套用的句子模板

- 假设式：`The relative/joint velocity is assumed to be measurable (available for feedback) via [encoder + tachometer / IMU], with bounded measurement error |δ| ≤ …`
- 差分式：`When velocity sensors are unavailable, the velocity is reconstructed by filtered backward differencing of the position measurement, i.e. v̂(k)= (q(k)−q(k−1))/T passed through a first-order low-pass filter.`
- 观测器式：`Since velocity sensors are expensive and prone to noise, we assume only position (and its derivative-free measurements) is available and design an extended-state / HOSM observer to recover the velocity in finite time.`
- 中文：`考虑到空间/水下环境中测速传感器成本高、噪声大甚至无法安装（或速度不可测、测不准），本文仅使用位置（位移/相对位置）量测，通过设计有限时间扩张状态观测器（FTESO）重构速度信息，并给出估计误差的有限时间收敛性证明。`

## 四、后续自查方式（OKB 工具）

`grep_search` 就够用，正则 + `doc_ids` 限范围很快：
- 全库正则（不加 doc_ids）：`velocit\w* (?:is|are) (?:obtained|estimated) (?:by|from|via)`、`velocity sensors? (?:are|is) (?:expensive|noisy)`、`without velocity measure?ments?`、`速度(?:信息)?(?:可测|不可测|可由|可通过)`
- 只查某篇上下文：`grep_search(doc_ids:"1933", query:"expensive", context:2)`
- 整节阅读：`read_markdown(id=151, section:"Problem formulation")`

需要我继续的话，可以：① 把这批引文按「无速度测量 / 差分 / 观测器」三类做成带年份的对比表；② 或针对你具体那篇（比如空间机械臂或编队控制）只筛 5–8 篇最贴合的可引文献。你倾向哪个？