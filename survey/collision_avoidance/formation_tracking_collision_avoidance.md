# 编队跟踪控制（Formation Tracking）中的避碰处理——论文精读

> 承接《formation_control_collision_avoidance_survey.md》。本篇**只聚焦 formation tracking / leader–follower
> tracking 这一类论文**，具体拆解它们各自"如何把避碰（agent 间 / 静态障碍 / 动态障碍）塞进跟踪控制器里"。
> 资料全部来自 `okb-assist`（MCP），括号内 `doc:` 为知识库文档编号，可 `read_markdown` 复读。

---

## 0. 这一类论文的共同结构

formation tracking 的典型设定是 **leader 知道参考轨迹 $y^d(t)$，followers 只知道与邻居/leader 的相对位移
$\delta_i$**，控制目标：

$$\varepsilon_i = x_i - x_1 - \delta_i \to 0 \quad(\text{follower}),\qquad \varepsilon_1 = x_1 - y^d \to 0 \quad(\text{leader})$$

**避碰是怎么"长"进这个框架的？** 观察 8 篇论文，共有 5 种"挂载点"：

| 挂载点 | 含义 | 代表论文 |
|---|---|---|
| ① 代价/成本项 | 在 MPC/优化目标里加避碰惩罚项 | `doc:1737` |
| ② 势场叠加 | 在控制律上叠加 APF 斥力项 | `doc:1547`, `doc:3207` |
| ③ 安全约束(QP) | 编队=CLF(可松弛)，避碰=CBF(硬约束)，合成 QP | `doc:3084`, `doc:1740` |
| ④ 性能/障碍函数变换 | 用 prescribed-performance / barrier 误差变换把"不许进入某距离"写成稳定条件 | `doc:2239`, `doc:1818` |
| ⑤ 奖励塑形 | DRL 奖励里加碰撞/障碍惩罚 | `doc:919`, `doc:1859` |

下面按论文逐个给出**具体控制器**，并标注它处理的是哪类碰撞。

---

## 1. LMPC 代价项法（同时处理 agent 间 + 障碍物）—— doc:1737

**Du 等 2024，《Model Predictive Formation Tracking-Containment Control for Multi-UAVs With Obstacle Avoidance》**
（IEEE TSMC），3-D 场景，leader–follower + tracking→containment 切换。

**核心做法**：不修改跟踪控制律本身，而是在 **Lyapunov-based MPC 的滚动时域代价**里加入两个避碰函数，
让优化器在"编队误差 / 输入 / 避障 / 互撞"之间自行权衡。

- **UAV–障碍物代价**（静态障碍为主，3-D 欧氏距离）：

$$H_{il}^{oa} = \max\!\Big\{-\mathcal M_{il}\,\ln\!\Big(\frac{\|D_{il}\|-\mathcal L_l}{\bar D_{il}-\mathcal L_l}\Big),\,0\Big\}$$

当 $\|D_{il}\|>\bar D_{il}$（超出探测范围）该项为 0，不触发避让；进入范围后势值快速上升。
$\mathcal L_l$ 最小安全距离，$\mathcal M_{il}$ 权重。

- **UAV–UAV 代价**（agent 间互碰）：

$$\mathfrak R_{ij}^{ca} = \sum_{j=1}^N a_{ij}\,\frac{\wp_{ij}}{1+\exp\big(\|D_{ij}\|-\bar{\mathcal D}_{ij}\big)}$$

$\bar{\mathcal D}_{ij}$ 为 agent 间最小安全距离，靠通信拓扑 $a_{ij}$ 取邻居。

- **整体代价 + 约束**（式 12–16）：

$$J_i=\min_{\hat\tau_i}\int_{t_k}^{t_k+T}\Big[\|\hat S_{i,1}\|_{Q_J}^2 + H_{il}^{oa}(\hat x_i) + \mathfrak R_{ij}^{ca}(\hat x_i,\hat x_j) + \|\hat\tau_i\|_{F_J}^2\Big]d\varrho$$

约束含动力学(13)、输入限(15)、以及**稳定性约束**(16)：要求 LMPC 得到的 Lyapunov 导数不大于
backstepping 辅助控制器 $\eta_i$ 的导数，从而**无需终端惩罚即可保证闭环稳定**。

**处理对象**：agent 间 ✅、障碍物 ✅（代价项对静态/动态障碍一视同仁，靠预测状态 $\hat x_i$ 体现动态性）。
**特点**：约束最丰富、可 3-D；缺点是随 UAV 数增大在线求解负担上升。

---

## 2. APF 势场叠加 + 角色切换 + 编队缩放 —— doc:1547

**Liu 等 2019，《Event-Triggered Coordination for Formation Tracking Control in Constrained Space》
（IEEE TCYB）**，受限空间、通信受限、事件触发。

- **跟踪/编队基础控制律**（leader 用(8)，follower 用(9)），纯基于相对位移，**本身不含避碰**。
- **空间避碰靠额外势场项** $U_i^c(x_i)$ 叠加（式12），把 forbidden space $\Pi$ 当作高势区产生斥力
  （即**静态障碍**为主；"spatial constraints"=约束空间/禁区）。
- **关键创新——编队缩放因子**：当区域原尺寸编队过不去时，引入 scaling factor 把编队整体放大/缩小，
  并用"coordinator"角色在检测到空间约束时触发，间接把环境信息广播给全网（`doc:1547` 摘要与 II–III 节）。
- **事件触发**：只在需要时通信，降低带宽，但避碰逻辑不变。

**处理对象（澄清）**：本文的"主动避碰"**只针对 spatial collisions（agent vs. 约束空间/障碍）**，
**并未把 agent 间互碰作为独立避碰问题处理**：
- 问题定义（式2）只要求 $x_i(t)\notin\Pi$，**没有** agent 间最小距离约束 $\|x_i-x_j\|>d_{\text{safe}}$；
- 控制律里**没有** agent 间排斥项，只有编队保持项 $\sum(x_i-x_j-\delta_{ij})$（让 agent 停到期望相对位置，
  属 formation keeping，不是安全避碰）；
- agent 间"安全"仅靠两道**被动/隐式**防线：① 预设编队间距 $\|\delta_{ij}\|$ 够大 → 正常不撞；
  ② 缩放下限 $0<\underline\lambda<1$（原文："To avoid **internal collisions among agents**, a lower limit
  $\underline\lambda$ is given"）——只是不让编队缩到自重叠，**是静态参数保护，非动态避碰**。
  严格按 Wang 2007 的"collision(agent间) vs obstacle(障碍)"划分，这是一篇 **obstacle / constrained-space
  avoidance 论文**，agent 间互碰被折叠进编队几何 + 缩放下限里。

**局限（关键批判）**：
1. **未处理 APF 局部极小**：第 3 页 Remark 1 只介绍 APF"简单有效"，**从未把局部极小列为要解决的问题**；
   甚至明确写"impulsive influence from spatial constraints … **not the focus of this paper**"。`grep` 全文
   `local minimum/minima/trapped/escape` 均为 0 命中。控制律 = 障碍排斥势 + 编队吸引梯度**直接相加**
   （如协调模式 follower 式15：$u_i=k_cF_i^c - k_n(\cdots)(x_i-x_1-\delta_i)-k_n\sum(\cdots)(x_i-x_j-\delta_{ij})-\cdots$），
   **正是产生伪平衡点的标准结构**；Lyapunov 稳定性只证编队误差收敛（局部），全局渐近稳定并不成立——
   **推导在原理上仍会遇到局部极小**，只是靠缩放/协调在仿真里"大概率不卡"，无形式化保证。
2. **agent 间互碰无显式保障**（见上"处理对象"）——遇障碍挤压等瞬态并不可靠。

**特点**：势场简单、可与事件触发/缩放/协调结合；但**既不根治局部极小，也不显式处理 agent 间避碰**，
是两处未闭环的漏洞。动态障碍 △（本文未强调，靠事件触发感知变化）。

---

## 3. 性能/障碍函数误差变换法（leader–follower，避碰=稳定条件）—— doc:2239

**Park & Yoo 2021，《Connectivity-maintaining and collision-avoiding performance function approach for
robust leader–follower formation control of multiple uncertain underactuated surface vessels》（Automatica）**。

**核心做法**：**完全不用势函数**，而是构造一个"连接保持 + 避碰"的 **performance/barrier 类误差变换**，
把"与邻居距离 > 某值""与障碍距离 > 某值""与 leader 保持连接"直接写进**新的形成误差** $e$，
再对 $e$ 做反步(backstepping)设计控制律（纵荡力 + 转艏力矩）。

- 同时在一个**单控制器**里保证四件事：① agent 间连接保持、② agent 间互碰避免、③ 障碍避让（且仍保持与
  leader 的连接）、④ 分布式编队跟踪。
- 与势场法对比的优势（文中 Remark）：势场法要分别为连接/互碰设计不同势函数再拼接，**导数相加易陷局部极小**；
  本文用统一误差变换，**无局部极小、无切换、无需自适应/函数逼近**（非线性完全未知也能用）。

**处理对象**：agent 间 ✅、障碍物 ✅（同时保连通）、动态障碍 △（USV 场景多静态/慢变）。
**特点**：低复杂度、预定性能边界可调、无局部极小——属于"把避碰变成稳定性证明的一部分"的路线。

---

## 4. 鲁棒 CBF + QP 法（leader–following，输入/速度饱和）—— doc:1740

**Fu 等 2024，《Robust collision-avoidance formation navigation of velocity and input-constrained
multirobot systems》（IEEE TCYB）**。

**核心做法**：两步走——

1. **名义编队跟踪控制器**：仅用相对位置 + 预定义时间观测器（predefined-time differentiator，式1–2）
   估计不可得的相对速度，得到不考虑避碰的跟踪律；
2. **鲁棒安全障碍条件**：推导线性约束作用于每个 follower 的控制输入；最终用**局部二次规划(QP)**求解
   安全编队导航控制器（摘要与 II–IV 节）。

- 障碍模型包含**静态与动态障碍**（"environment usually contains various static and dynamic obstacles"）；
- 在速度约束(VIC)下避碰更难（速度不能瞬时归零），鲁棒 CBF 在**有界控制输入**下仍能保证避碰；
- 扰动 $d_i$ 有界，用鲁棒 barrier 条件吸收。

**处理对象**：agent 间 ✅、静态 ✅、动态 ✅（障碍模型显式含 dynamic；鲁棒 CBF 把障碍位置喂入约束）。
**特点**：硬安全保证 + 输入/速度饱和兼容 + 分布式 QP 实时性好。

---

## 5. 分布式 CLF-CBF-QP 反应式编队 —— doc:3084

**Liu 等 2026，《Distributed Control Lyapunov and Control Barrier Certificates for Reactive Formation》
（IEEE TAC）**。leader–follower，一阶/二阶。

**核心做法**：把目标与安全的"SOB 要求"显式列出，再分别写成可分布式验证的证书：

- **编队/跟踪 = CLF（可松弛稳定约束）**：
  - 轨迹跟踪(TR)：$\bar A_1 u_1 \le -k_l V_1 + \bar A_1 v_l$，$V_1=\|p_1-p_l\|^2$
  - 编队形状(FS)：$A_i u_i \le -k_f W_i$，$W_i=\sum_{j\in\mathcal N_i}a_{ij}\|p_{ij}-d_{ij}\|^2$
- **安全 = CBF（硬约束）**，统称 SOB：
  - **SA（agent 间互碰）**：$\|p_{ij}\|>\underline\delta,\;\forall j\in\bar{\mathcal N}_i$
  - **OA（障碍物）**：圆形模型 $\|p_i-o_k\|\ge\delta_k$；SDF 模型 $s(p_i)>\delta_p$
  - **BS（输入有界）**：$-u_{\max}\le u_i\le u_{\max}$
- **反应式编队定义**：允许临时偏离 $d_{ij}$ 以避障，再恢复，即
  $\min_{u_i\in S_i}\Delta_l+\Delta_{ij}$，其中 $S_i$ 为满足 SOB 的控制器集合。

障碍用**圆形模型**或 **SDF（符号距离场，可在线由视觉生成）**；动态障碍通过把预测障碍位置喂入 CBF 约束体现。

**处理对象**：agent 间 ✅、静态 ✅、动态 ✅（SDF/圆形 + 预测）。
**特点**：可行性有理论保证、实时优于 MPC、允许"临时破编队保安全"。

---

## 6. 预定性能 + 编队重构法（输入饱和、无势场）—— doc:1818

**Liu 等 2022，《Adaptive distributed finite-time formation control for multi-UAVs under input
saturation without collisions》（Aerospace Science and Technology）**。

**核心做法**：针对 leader–follower 跟踪，用 **prescribed performance + 非线性映射**把 UAV 限制在指定区域以防互碰：
- 一部分是**编队重构算法**：按当前位置与期望位置合理匹配，给出各 UAV 的"避碰区域"；
- 另一部分是含 **prescribed-performance 项 + 位置误差项**的新型 Lyapunov 函数，把"不许进入某距离"转化为
  性能边界内的稳定问题（摘要与 Intro）。
- 区别于传统 APF（有局部极小），本文强调**不用势函数**即可实现全局互碰避免。
- 另用改进辅助动态系统（双曲函数）处理输入饱和。

**处理对象**：agent 间互碰 ✅（重点）、障碍 △（本文聚焦互碰，未展开动态障碍）。
**特点**：有限时间收敛、无局部极小、与输入饱和统一处理。

---

## 7. DRL 奖励塑形法（leader–follower，动态环境）—— doc:919 / doc:1859

**Zhou 等 2019（doc:919，《Adaptive leader-follower formation control and obstacle avoidance via DRL》**
IROS）与 **Sui 等 2021（doc:1859，IEEE TNNLS）**。

- **doc:919**：把定位(CNN)与控制(DRL)模块化，提出 Momentum Policy Gradient (MPG)。**碰撞/障碍避免通过
  reward shaping 直接加进奖励函数**（"features such as collision and obstacle avoidance can be easily
  integrated into a DRL controller"），从单 leader 跟踪自然扩展到编队+避碰。非完整轮式机器人。
- **doc:1859**：两阶段训练——先**模仿学习**用"一致性编队控制器 + ORCA"作专家示范，再 RL 阶段用
  **复合奖励**同时权衡编队保持与避碰；LSTM 感知不定数量障碍，处理**不确定动态环境**下的 leader–follower
  编队碰撞避免。

**处理对象**：agent 间 ✅、障碍物 ✅（含动态，靠奖励+感知）、静态 ✅。
**特点**：免复杂建模/控制律推导；安全性为经验性、依赖训练分布。

---

## 8. 横向对比：formation tracking 论文"怎么挂避碰"

| doc | 方法类别 | agent 间 | 静态障碍 | 动态障碍 | 跟踪框架特色 |
|---|:---:|:---:|:---:|:---:|:---|
| 1737 | MPC 代价项 | ✅ | ✅ | ✅(预测态) | LMPC + shifting function，tracking→containment |
| 1547 | APF 叠加 + 缩放 | ⚠️(仅隐式) | ✅(禁区/spatial) | △ | 事件触发 + coordinator + 编队缩放；**未处理局部极小、agent间无显式避碰** |
| 2239 | 性能/障碍函数变换 | ✅ | ✅(保连通) | △ | 无势函数、单控制器、反步 |
| 1740 | 鲁棒 CBF + QP | ✅ | ✅ | ✅ | 预定义时间观测器 + VIC 饱和 |
| 3084 | CLF-CBF-QP | ✅ | ✅ | ✅(SDF/预测) | 反应式编队（可临时破编队） |
| 1818 | 预定性能 + 重构 | ✅ | △ | △ | 有限时间、输入饱和、无势场 |
| 919 | DRL 奖励塑形 | ✅ | ✅ | ✅ | CNN 定位 + MPG，模块化 |
| 1859 | DRL + ORCA 示范 | ✅ | ✅ | ✅ | 模仿+RL 两阶段，LSTM 感知 |

---

## 9. 归纳：formation tracking 论文避碰的"套路"

1. **跟踪律与避碰解耦 vs 耦合**：
   - 解耦（最主流）：先设计纯跟踪/编队律，再把避碰作为**附加项/约束/奖励**挂上去
     （1737 代价项、1547 势场、919/1859 奖励、1740 的"名义律+QP"）。
   - 耦合：把避碰写进**误差/证书定义**，与稳定性证明一体（2239 性能函数、3084 CLF-CBF、1818 预定性能）。

2. **静态 vs 动态障碍在处理上的差异**：
   - 静态障碍：直接把障碍位置/禁区当作常量势源或 CBF 集合（1547、2239、1737 的 $H^{oa}$）。
   - 动态障碍：必须**预测**——MPC 用预测状态 $\hat x_i$（1737），CBF 用 SDF/预测障碍位置（3084、1740），
     DRL 用 LSTM 感知运动障碍（1859）。这是两者唯一的本质区别，挂载机制相同。

3. **agent 间互碰**几乎总是与编队项共享信息（邻居相对位移 $p_{ij}-d_{ij}$）：CBF 用 $\|p_{ij}\|>\underline\delta$、性能函数用连接/间距约束、MPC 用 $\mathfrak R^{ca}$ 代价。**例外是 `doc:1547`**——它只有编队保持项
   $\sum(x_i-x_j-\delta_{ij})$ 与缩放下限 $\underline\lambda$，**没有 agent 间排斥/最小距离硬约束**，
   agent 间安全被折叠进编队几何，属隐式兜底而非显式避碰。

4. **趋势**：从"APF 叠加"（易局部极小）走向"**硬安全约束 + 反应式**"（CBF/CLF-CBF/鲁棒 CBF），
   并允许**临时偏离编队**优先保安全后恢复（3084 反应式、1740 鲁棒 CBF）。

---

### 一句话结论
formation tracking 论文处理避碰的"标准动作"是：**以 leader–follower 相对位移跟踪律为底座，
再把避碰以"代价项 / 势场项 / CBF 硬约束 / 性能函数变换 / DRL 奖励"五种形式之一挂上去**；
静态障碍用常量势源或集合，动态障碍则必须在同一机制里额外代入**预测轨迹**。
