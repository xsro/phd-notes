# 集群控制（Swarm / Flocking / Consensus）中 Agent 间避碰方法调研

> 承接《formation_control_collision_avoidance_survey.md》与《formation_tracking_collision_avoidance.md》。
> 本篇把范围扩大到**一般集群控制**（swarm / flocking / consensus / rendezvous / aggregation / coverage 等，
> **不限于编队**），专看这类论文**如何实现 agent 之间的互碰避免（inter-agent collision avoidance）**。
> 资料来自 `okb-assist`（MCP），括号内 `doc:` 为知识库文档编号，可 `read_markdown` 复读。

---

## 0. 为什么"集群"里 agent 间避碰更突出

在 swarm 里，"障碍物"往往**就是其它 agent 本身**。Chung 等的 aerial-swarm 综述 (`doc:1742`) 明确指出：

> "the obstacles encountered by a robot include **other members of its swarm**, and collision avoidance has to
> factor in the need to maximize the performance of the swarm (e.g., avoid increasing the time to complete an assignment)."

即集群避碰的难点在于：**避碰目标与集群协作目标（聚合/一致/覆盖）相互冲突**，且规模大、去中心化。
Chmaj & Selvaraj 的 survey (`doc:2168`) 还专门把 *collision avoidance* 定义为
"avoiding the physical contact **between UAVs in one swarm**"，与 obstacle avoidance 区分开。

---

## 1. 方法一：Reynolds 三规则 + 局部人工势场（Flocking）

**代表**：`doc:198` Su 等《Flocking of Multi-Agents With a Virtual Leader》（Olfati-Saber 框架）、
`doc:2468` Do 2011《Flocking for multiple elliptical agents》、`doc:1742` 综述。

Reynolds 三启发式规则（1980s）：**Separation（与邻近个体保持距离）/ Alignment（速度一致）/ Cohesion（向群体靠拢）**。
控制论实现（`doc:198` Intro）是：

- **Separation** = 邻居间的**局部排斥人工势场**（local attractive/repulsive potential）；
- **Alignment** = 速度一致性（consensus）；
- **Cohesion** = 向群体质心/虚拟 leader 的吸引势。

Olfati-Saber (`doc:198` 引 [29]) 把三规则写成可分析的可扩展 flocking 算法：
总控制 = 速度一致项 + 编队保持（navigational feedback）+ **分离势场项**（距离过近时斥开）。
`doc:198` 进一步研究"只有少数 informed agents 知道虚拟 leader"时的 flocking，以及虚拟 leader 变速时的渐近跟踪。

**agent 间避碰 = Separation 规则**，本质仍是 APF 斥力。优点：生物启发、可扩展、分布式；
**缺点同前**：标准 APF 有局部极小风险，且分离项只是"经验规则"，缺乏硬性安全保证。

---

## 2. 方法二：聚合势场 + 滑模控制（Aggregation）

**代表**：`doc:1908` Gazi 等 2007《Aggregation in a swarm of non-holonomic agents using artificial
potentials and sliding mode control》、`doc:409` Han 等 2008《UAV swarm control using potential functions
and sliding mode control》。

- `doc:1908`：把 swarm **聚合（aggregation）**到质心 $p_c=\frac1N\sum p_i$ 的 $\epsilon$-邻域内（式2），
  用**生物启发的人工势函数**构造吸引+排斥力，再叠加滑模控制（SMC）以鲁棒抗扰。agent 间避碰由势函数中的
  **排斥分量**保证（太近 → 斥力 → 不重叠）。模型从点质量扩展到**非完整 unicycle 动力学**。
- `doc:409`：**行为式（behaviour-based）去中心化** UAV swarm，个体交互用 APF 建模；用"swarm geometry
  centre (SGC)"作为控制对象跟踪参考轨迹；滑模控制保证对模型不确定/环境变化的鲁棒性。
  特别提到"**modified collision avoidance method preventing local minima** without generating new path
  when UAVs meet pop-up threats"——即用变形障碍形状（修改势场）来规避 APF 局部极小，而非重规划路径。

**agent 间避碰 = 聚合势场的排斥项**。比纯 flocking 更强调"聚成一团但不撞"，适合聚集/搜索救援。
`doc:409` 还给出了一个**针对 APF 局部极小的具体补丁**（变形障碍势场），比 `doc:1547` 那种"不处理"更进一步。

---

## 3. 方法三：Lyapunov-like / Barrier Lyapunov 函数（把"不碰"写成稳定条件）

这是集群里**最严谨**的 agent 间避碰路线——不用 APF 的梯度势，而把"最小间距约束"写成
**障碍 Lyapunov 函数**，约束违反时函数→∞，从而在 Lyapunov 稳定性证明里**硬保证**不进入碰撞集。
核心工具是 **Nagumo 定理**（集合弱正不变 ⇔ 安全）。

**代表**：
- `doc:879` Panagou 等 2013（CDC）、`doc:2743` Panagou 等 2016（IEEE TAC）
  《Distributed coordination control for multi-robot networks using Lyapunov-like barrier functions》
- `doc:823` Singh & Jain 2024（Systems & Control Letters）《Collision avoidance and connectivity
  preservation using asymmetric barrier Lyapunov function with time-varying distance-constraints》
- `doc:1891` Ma & Chou 2024《Practical time-varying formation cooperative control … via safety constraints》

**具体做法（`doc:879` 最清楚）**：把每个 agent $i$ 视为"把邻居 $j$ 当物理障碍"，要求间距
$d_{ij}=\|r_i-r_j\|\ge d$（最小分离距离，$d\ge 2r_0$）。写成约束
$c_{ij}=(x_i-x_j)^2+(y_i-y_j)^2-d^2\ge 0$，再用**对数障碍函数**编码：

$$b_{ij}(\cdot) = -\ln\big(c_{ij}(\cdot)\big)$$

当 $c_{ij}\to 0$（即两 agent 逼近最小距离）时 $b_{ij}\to+\infty$，于是梯度型控制自然把 agent 推离。
多个目标（避碰 + 连通 + 收敛）用 **recentered barrier + max 近似**合成**单个 Lyapunov-like 函数**，
得到分布式梯度控制；并基于 Nagumo 定理给出冲突解决的充分必要条件（半协作/优先级化）。

`doc:823` 进一步用**非对称障碍 Lyapunov 函数（ABLF）**处理**时变距离约束**——碰撞避免（下界 $r_{C,\min}$）
与连通保持（上界 $r_{C,\max}$）的约束方向不同，故用非对称势；当间距低于触发阈值 $r_{C,\max}$ 时
碰撞避免 BLF 才激活，把 agent 推离危险边界 $r_{C,\min}$（`doc:1891` 引述并图示）。

**agent 间避碰 = 最小间距约束的障碍 Lyapunov 函数**。相比 APF：
- **无局部极小**（不是梯度势相加，而是约束违反度→∞的 barrier）；
- **有稳定性/安全的形式化保证**（forward invariance）；
- 可同时编码互碰 + 连通 + 收敛多目标。

---

## 4. 方法四：导航函数（Navigation Function）做无碰 rendezvous/聚集

**代表**：`doc:2945` Verginis & Dimarogonas 2019（CDC）《Adaptive leader-follower coordination of
lagrangian multi-agent systems under transient constraints》、Dimarogonas 系列
（"feedback stabilization and collision avoidance scheme for multiple non-point agents", Automatica 2006）、
De Gennaro & Jadbabaie 的 decentralized navigation function。

导航函数是把"目标点"设势阱、**障碍/邻居设势垒**的势函数，且经特殊构造保证**近全局无局部极小**
（与裸 APF 不同）。在 rendezvous / leader-follower 协调里，导航函数同时：① 把 agent 引向目标/会合点；
② 在 agent 彼此接近时抬升势垒 → **保证 rendezvous 过程中不碰**。
`doc:2945` 还把"碰撞规范（collision specifications）"与"连通规范"作为瞬态约束，做自适应协调。

**agent 间避碰 = 导航函数的互排斥势垒**（近全局收敛、无局部极小）。适合"先聚集/会合再任务"的集群场景。

---

## 5. 方法五：控制障碍函数 CBF / 嵌入式 DCBF（硬安全 + 反应式）

**代表**：`doc:3258` Liu 等 2026（IEEE/ASME TMECH）《Safety-Critical Fixed-Time Distributed Formation
Control for Miniature Robot Swarm Based on Embedded Dynamic Control Barrier Functions》、
`doc:3084`/`doc:1740`（前篇已述，CBF 把 SA 互碰写成硬约束）。

`doc:3258` 面向**微型机器人群**（受限感知/通信、易扰、未知动态模型），提出 **EMB-DCBF**
（embedded dynamic CBF）处理**时变安全约束 + 扰动 + 高相对阶**，再嵌进固定时间指令滤波反步框架：
- 用 **EMB-DCBF-QP** 做安全修正（分布式 QP），把互碰、动态障碍、输入饱和、有向通信连通**同时**
  作为安全约束（`doc:3258` 摘要与 Intro 明确列 "intervehicle collision avoidance"）；
- 配 **FxTESO**（固定时间扰动观测）+ **FxTTVE**（固定时间目标速度估计，首次把目标障碍速度估计并入 CBF-SCC）。
- 相比 HOCBF，EMB-DCBF 微分阶数更低、计算负担更小，适合微型群机载实时。

**agent 间避碰 = CBF 硬约束（SA: $\|p_{ij}\|>\underline\delta$）**。与前篇一致：硬安全保证 +
分布式 QP 实时 + 反应式（可临时偏离任务保安全）。这是目前微型/大规模 swarm 最主流的 safety-critical 路线。

---

## 6. 横向对比（集群场景的 agent 间避碰）

| 方法 | 代表 doc | agent 间避碰机制 | 硬安全保证 | 局部极小 | 多目标(连通/聚合) |
|---|---|---|:---:|:---:|:---:|
| Flocking 分离势 | 198, 2468, 1742 | Separation = 局部排斥 APF | ✗ | 有风险 | 与对齐/聚合耦合 |
| 聚合势 + SMC | 1908, 409 | 聚合势排斥项 | ✗ | 409 有变形补丁 | 聚集成团 |
| 障碍 Lyapunov (BLF) | 879, 2743, 823, 1891 | 最小间距约束 barrier→∞ | ✅ | **无** | 可同时编码互碰+连通+收敛 |
| 导航函数 | 2945, Dimarogonas | 互排斥势垒(近全局) | ✅(近全局) | **无** | 会合/聚集 |
| CBF / EMB-DCBF | 3258, 3084, 1740 | $\|p_{ij}\|>\underline\delta$ 硬约束 | ✅ | 无(QP 不可行≠极小) | 可同时多约束 |

---

## 7. 与"编队跟踪"论文的对照（关键区别）

1. **避碰对象不同**：编队跟踪论文里 agent 间避碰常"隐式"靠编队几何（如 `doc:1547` 只有编队保持项 +
   缩放下限，无显式互碰项）；而**集群论文把 agent 间互碰当作一等公民**——要么写进分离势/聚合势，
   要么写进 BLF/导航函数/CBF 的硬约束。理由：swarm 里"其它 agent 就是障碍"，不显式避就会真撞。
2. **目标冲突更尖锐**：集群要平衡"聚合/一致"与"分离"，避碰是与协作目标**直接冲突**的硬约束
   （`doc:1742`），因此更依赖能把多目标统一进一个 Lyapunov/QP 的 BLF、CBF 路线。
3. **去中心化/可扩展性要求更高**：swarm 通常拒绝中心式（`doc:409`："centralized … not suitable for
   swarm"），所以 flocking 势场、分布式 BLF、分布式 CBF-QP 比集中式 MPC 更受青睐（尽管 MPC 在编队跟踪里常见）。
4. **局部极小是分水岭**：裸 APF/flocking 势场（`198/1908`）有局部极小隐患；BLF（`879/823`）、
   导航函数（`2945`）、CBF（`3258`）从构造上规避。值得注意 `doc:409` 还专门给了"变形障碍势场"的
   抗局部极小补丁——这是比 `doc:1547`（完全不处理）更认真的态度。

---

### 一句话结论
集群控制的 agent 间避碰主要有五条路线：**① flocking 分离势（APF，简单但有局部极小）、
② 聚合势+SMC（聚集成团不撞）、③ 障碍 Lyapunov 函数（把最小间距写成→∞的 barrier，无局部极小、有形式化保证）、
④ 导航函数（近全局无碰会合）、⑤ CBF/EMB-DCBF（硬约束 + 分布式 QP，现代微型群主流）**。
总体趋势与编队跟踪一致——从经验势场走向**硬安全约束（BLF/CBF）+ 多目标统一**，且因 swarm 去中心化、
规模大、避碰即"防自撞"，对显式、可扩展、无局部极小的互碰机制需求更强。
