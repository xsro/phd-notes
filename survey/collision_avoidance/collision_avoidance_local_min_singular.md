# 避碰方法如何处理「局部极小值 / 特殊队形 / 奇异点」问题

> 承接三篇调研：
> 《formation_control_collision_avoidance_survey.md》（总览）、
> 《formation_tracking_collision_avoidance.md》（编队跟踪深读，含 `doc:1547` 批判）、
> 《swarm_control_agent_collision_avoidance.md》（集群 agent 间避碰）。
>
> 本篇把"避碰方法"按**三种失效模式**重新切片分析：
> ① **局部极小值（local minimum）**——梯度势相互抵消，agent 卡在伪平衡点；
> ② **特殊队形 / 奇异构型（special formation / singular configuration）**——刚体/距离编队里的共线、共面、退化平衡；
> ③ **奇异点 / QP 不可行（singular point / QP infeasibility）**——barrier→∞ 的 blow-up、重合 agent、约束冲突导致 QP 无解。
>
> 括号内 `doc:` 为 `okb-assist` 知识库编号，可 `read_markdown` 复读。

---

## 0. 三个概念先分清（它们不是一回事）

| 概念 | 出问题的机理 | 典型场景 | 后果 |
|---|---|---|---|
| 局部极小值 | APF 吸引+排斥梯度**抵消**，合力=0 但非目标点 | 障碍物夹在 agent 与目标之间（"U 形陷阱"） | agent 停滞、永远到不了目标 |
| 特殊队形 / 奇异构型 | 距离/刚体编队中构型**降维**（共线/共面），刚度矩阵 rank 丢失 | 2-D 三角形成"三点共线"、3-D 四面体"四点共面" | 梯度控制失去控制维度，卡死在退化平衡 |
| 奇异点 / QP 不可行 | ① barrier 趋近 0 时→∞ 造成控制 blow-up；② 重合 agent；③ 多约束在紧集上冲突 | CBF-QP 同时要求"不碰+到目标"但空间太挤 | 执行器饱和（BLF）或 QP 无可行解（CBF） |

> 关键区别：**局部极小**是"梯度势方法"的普遍病；**特殊队形/退化平衡**是"距离/刚体编队"的专属病；
> **奇异点/QP 不可行**是"硬约束方法（BLF/CBF）"在边界上的病。三者成因不同、解法也不同。

---

## 1. APF / Flocking / 聚合势 类 —— 受「局部极小 + 共线伪平衡」双重困扰

**代表**：`doc:1547`（事件触发编队跟踪，APF 仅引入未处理极小）、`doc:198`/`doc:2468`（flocking 分离势）、
`doc:1908`/`doc:409`（聚合势 + SMC）。

### 1.1 局部极小：标准 APF 的硬伤
APF 控制律 = 目标吸引梯度 − 障碍排斥梯度，直接相加。当多个障碍/邻居把 agent 夹住时，
吸引与排斥在某点精确抵消 → **合力为零的伪平衡点（spurious equilibrium）**，Lyapunov 只能证*局部*收敛。
`doc:1547` 即此结构（follower 模式 eq.15：`u_i = k_c F_i^c − k_n(⋯)(x_i−x_1−δ_i) − k_nΣ(⋯)(x_i−x_j−δ_ij) − ⋯`），
其 Remark 1 仅"引入"APF，正文与 "local minimum/minima/trapped/escape" **零命中**，并声明冲激问题
"not the focus of this paper"——**完全没处理局部极小**（详见《formation_tracking_collision_avoidance.md》§2 批判）。

### 1.2 有限补丁：变形障碍势场
`doc:409`（Han 2008）给了比 `doc:1547` 认真的处理——摘要与正文明确：
> "this paper suggests a **modified collision avoidance method preventing local minima** without generating new
> path when the UAVs meet the pop-up threats … the popup threat avoidance algorithm is also introduced by
> **modifying the shape of obstacle**."

即遇到突发威胁时**主动变形障碍形状**（而非重规划路径）打破梯度抵消，从而逃逸局部极小。这是 APF 路线里
"承认问题并打补丁"的代表。

### 1.3 特殊队形 / 共线伪平衡
flocking/聚合势本身不是"距离编队"，谈不上刚体 rank 丢失；但其分离项本质仍是局部排斥势，
在对称布局（如 agent 两两对称、合力抵消）下同样会出现**对称伪平衡 / 共线卡死**，且依旧缺硬安全保证。

### 1.4 奇异点
裸 APF 用**有限**排斥（高斯/指数势），agent 越近斥力有界 → 强吸引可"压过"斥力 → **真撞风险**（无 →∞ 屏障）。
这是它与 BLF/CBF 的根本区别。

**小结**：APF/flocking/聚合势对三类问题都**最脆弱**：局部极小靠补丁（409）、特殊队形靠经验、奇异点无屏障。

---

## 2. 距离 / 刚体编队控制 —— 「特殊队形 / 退化平衡」是它的主场问题

这一类（distance-based / rigid / infinitesimally rigid formation）的奇异构型是文献里被**最深入剖析**的对象。

### 2.1 退化平衡必然存在（Zhiyong Sun 博士论文 `doc:2786`）
`doc:2786` Lemma 3.1 给出核心结论：对可在 d 维实现的目标形状，总存在
**incorrect degenerate equilibria**——满足 `rank( Q̄(p) ) < d` 的平衡点，此时嵌入该构型的仿射空间维数 < d、
势函数非零（即目标形状**没达到**）。直接推论：
> "Consequently, there always exist **collinear equilibria for 2-D** formation systems and
> **collinear/coplanar equilibria for 3-D** formation systems."

即 2-D 三角形成"三点共线"、3-D 四面體成"四点共面"等降维构型，**一定是梯度控制律的平衡点**。

### 2.2 rank-preserving 性质 → 一旦共线就出不来
`doc:2786` 第 3.3 节 "Rank-Preserving Property for Formation Systems"：标准梯度控制律具有 rank 保持性质，
即构型矩阵的秩沿轨迹**不增不变**。含义很残酷：**一旦初始/中途落入共线（降秩）构型，控制律永远无法把它拉回满秩目标形状**——
共线集是**不变集（invariant set）**，这正是"特殊队形卡死"的严格表述。

### 2.3 距离误差 → 螺旋/匀速旋转运动
即便不严格共线，距离测量的小常数偏差也会引发特征运动：
- `doc:2786` §10.1.2 "Formation Control Systems: Some Practical Considerations"：
  > "generically a **helical motion** will be induced by small and constant mismatches between neighboring agents
  > in their distance measurements or perceived target distances."
- `doc:3107`（Van Vu 2021）：引言综述
  > "errors in distance measurements introduce uncertainties that eventually drive the formation to
  > **rotate at a constant angular speed**."
  `doc:3107` 本身正是为**同时**达成目标形状并**消除有界扰动影响**而设计控制律（基于非光滑 Lyapunov + Barbalat 引理）。

### 2.4 主流解法（突破奇异/退化）
`doc:2786` 给出的缓解路线（其总结）：
> "the standard gradient control law should be combined with additional mitigating approaches, e.g.
> **introduction of integral control** or **inclusion of an estimation loop**."

- **积分控制 / 估计回路**（`doc:2786`）：打破 rank-preserving 的"出不来"困境，使系统能逃离退化平衡。
- **滑模有限时间全局稳定**（`doc:786`，Lin 2022 IEEE TAC）：
  > "This article provides a general solution to this open problem based on the **sliding mode control** idea …
  > solve the open problem of **(almost) global and finite-time stabilization** of affine, rigid, and translational
  > formations in any dimensional space."
  控制律分两部分：主控制力在有限时间把全体 agent 拉到**仿射编队空间滑动面**并维持；附加控制力引导 leader 在面内收敛。
  滑模的"切换"特性天然避开共线退化平衡这个伪平衡。
- **改用 displacement-based**（`doc:665`，Rai & Mou）：
  > "Displacement-based formation control can generate **much richer formations** … can be changed with time to
  > perform any **rotation or scaling**."
  位移编队不依赖距离刚度矩阵，可由时变位移直接做旋转/缩放，对奇异构型更鲁棒（但需通信相对位置）。

**小结**：距离/刚体编队的"特殊队形/退化平衡"是理论硬伤，靠积分控制、估计回路、滑模全局稳定、或换位移编队破解；
与 APF 的局部极小是**不同机理**。

---

## 3. Barrier Lyapunov 函数（BLF / ABLF）—— barrier→∞ 消除局部极小，但奇异点有 blow-up

**代表**：`doc:879`/`doc:2743`（Panagou，Lyapunov-like barrier）、`doc:823`（Singh & Jain，非对称 BLF）、
`doc:1891`（Ma & Chou，时变编队 + 安全约束）。

### 3.1 为什么无局部极小
BLF 不把避碰写成"吸引−排斥势相加"，而是把最小间距约束写成
`b_ij = −ln(c_ij)`，`c_ij = (x_i−x_j)²+(y_i−y_j)²−d² ≥ 0`。
当 `c_ij → 0`（两 agent 逼近最小距离）时 `b_ij → +∞`，**约束违反度趋近 ∞ 而非梯度抵消为 0**——
所以不存在"合力为零的伪平衡点"。再基于 **Nagumo 定理**（集合弱正不变 ⇔ 安全）给出 forward invariance 的
**形式化不碰保证**（《swarm_control_agent_collision_avoidance.md》§3）。这是它相对 APF 的根本优势。

### 3.2 奇异点 / blow-up 风险
- **重合 agent（d_ij → 0）正是危险奇异点**：此时 `c_ij → −d²`、barrier 越界，控制增益→∞。
  实际执行器**饱和**会破坏 Lyapunov 证明的前提 → 失去硬保证。
- **非对称 BLF**（`doc:823`）用**时变距离约束**缓解：碰撞避免（下界 `r_C,min`）与连通保持（上界 `r_C,max`）约束方向不同，
  仅当间距低于触发阈值 `r_C,max` 时碰撞避免 BLF 才激活，把 agent 推离危险边界 `r_C,min`，避免常驻高增益。
- 它**不解决**距离编队的"共线 rank 丢失"问题（那是刚度矩阵问题，BLF 管的是 agent 间距屏障），
  但**能挡住 agent 重合这个奇异点**——前提是初始就在安全集内（forward invariance 只保"不越界"，不保"从界外救回"）。

**小结**：BLF 用 barrier→∞ 根除局部极小，代价是边界 blow-up + 执行器饱和；非对称/时变 BLF 缓解常驻高增益。

---

## 4. 导航函数（Navigation Function）—— 近全局无极小，设计规避奇异

**代表**：`doc:2945`（Verginis & Dimarogonas 2019）、Dimarogonas 系列（Automatica 2006 等）。

导航函数把目标设势阱、障碍/邻居设势垒，且经**特殊构造（Morse 函数性质）**保证**近全局无局部极小**——
与裸 APF 不同。在 rendezvous / leader-follower 协调里同时：① 引向目标/会合点；② agent 接近时抬升互排斥势垒
→ **保证会合过程不碰**，且近全局收敛（《swarm_control_agent_collision_avoidance.md》§4）。

- **局部极小**：构造上消除（近全局）。
- **特殊队形**：navigation function 关注的是"无碰会合/聚集"，而非距离编队形状，故无共线 rank 问题；
  但若目标点恰好与某障碍构型形成退化点，需靠 Morse 性质与势垒设计避开。
- **奇异点**：若两 agent 初始重合（d_ij=0），势垒→∞，与 BLF 类似需初始安全间距。

---

## 5. CBF / CLF-CBF-QP / EMB-DCBF —— 前向不变避极小，但紧约束 → QP 不可行

**代表**：`doc:3258`（Liu 2026，EMB-DCBF）、`doc:3084`/`doc:1740`（CBF 编队/互碰，前篇已述）。

### 5.1 为什么无局部极小
CBF 通过 **forward invariance**（`h(x) ≥ 0` 沿轨迹保持）保证安全，**不是梯度下降**，因此不存在 APF 那种
"梯度抵消卡死"。互碰写成硬约束 `‖p_ij‖ > δ̲`，QP 在每个采样时刻做最小修正以保持该集。

### 5.2 奇异点 / QP 不可行（本类的专属病）
- **约束冲突 → QP 无可行解**：当"不碰"与"到目标/保持编队"在**空间被压得很挤**时，
  满足所有 CBF 约束的 `u` 可能不存在 → QP infeasible。这不是"局部极小"，而是**实时可行域为空**。
- **解法**：
  - **CLF-CBF-QP 松弛**：把任务目标放宽为 CLF 软约束、安全放为 CBF 硬约束，冲突时优先安全（牺牲任务）。
  - **EMB-DCBF**（`doc:3258`）：embedded dynamic CBF 处理**时变安全约束 + 扰动 + 高相对阶**，微分阶数比 HOCBF 更低、
    计算负担更小，更适合微型群机载实时；配 FxTESO（固定时间扰动观测）+ FxTTVE（目标/障碍速度估计）提升紧约束下的可行性。
  - **初始安全集要求**：CBF 只保"不越界"，若初始已碰撞（h<0）则无法恢复——与 BLF 同病。

**小结**：CBF 用前向不变根除局部极小，但紧约束下 QP 可能无解，靠 CLF-CBF 松弛 / EMB-DCBF 低阶微分 / 初始安全集应对。

---

## 6. MPC / LMPC —— 全局滚动优化避极小，但计算重、且是非凸 NLP 局部最优

**代表**：`doc:966`/`doc:3075`（动态障碍 MPC）、`doc:1818`（Liu 2022，LMPC 编队）。

- **局部极小**：有限时域优化**显式求解**避障轨迹，靠预测窗口避开 U 形陷阱，优于反应式 APF；
  但底层是**非凸 NLP**，求解器返回的是**局部最优**——极端布局仍可能陷局部最优（非 APF 式梯度抵消，而是优化器收敛到次优解）。
- **特殊队形**：MPC 直接以状态约束编码编队，不受距离刚度 rank 限制；但共线/退化形状若作为约束边界需显式排除。
- **奇异点**：预测窗口末端或约束紧时优化可能 infeasible；常用**soft constraint / 松弛终端集**兜底。
- 代价：计算负担重，难满足微型群机载实时（故 swarm 更偏 CBF/BLF，见 `doc:409` "centralized not suitable for swarm"）。

---

## 7. DRL —— 无形式化保证，极小/奇异靠训练覆盖

**代表**：`doc:919`/`doc:1859`（深度强化学习避碰）。

- **局部极小 / 特殊队形 / 奇异点**均**无理论保证**：策略靠奖励函数（碰撞惩罚）在训练中"学会"绕开，
  对未见过的 U 形陷阱、共线退化、重合奇异可能失效；可解释性与安全认证弱。
- 适合高维感知输入、端到端避碰，但安全关键场景需与 CBF/BLF 等硬约束**混合**（CBF 做安全滤子）才稳妥。

---

## 8. 横向对比表（三类失效模式 × 方法）

| 方法 | 局部极小 | 特殊队形/退化平衡(共线/共面) | 奇异点/QP 不可行 | 代表 doc |
|---|:---:|:---:|:---:|---|
| APF / Flocking 分离势 | ❌ 易陷（409 变形补丁） | ❌ 对称伪平衡 | ❌ 有限斥力可真撞 | 1547, 198, 2468, 1908, 409 |
| 聚合势 + SMC | ⚠️ 409 补丁 | ⚠️ 经验 | ⚠️ 斥力有界 | 1908, 409 |
| 距离/刚体编队梯度 | ✅(无势极小) | ❌ **退化平衡必然存在**（`doc:2786` Lemma 3.1）+ rank-preserving 出不来 | ❌ 距离误差→螺旋/匀速转（2786/3107） | 2786, 3107, 665 |
| 距离/刚体 + 滑模全局 | ✅ | ✅ 滑模有限时间全局稳定（786） | ✅ 切换避退化 | 786 |
| 距离/刚体 + 积分/估计回路 | ✅ | ✅ 破 rank-preserving（2786） | ✅ | 2786 |
| Barrier Lyapunov (BLF/ABLF) | ✅ barrier→∞ | ➖ 不针对 rank 问题，但挡重合 | ⚠️ 重合→∞ blow-up + 饱和（823 时变缓解） | 879, 2743, 823, 1891 |
| 导航函数 | ✅ 近全局(Morse) | ➖ 无形状 rank 问题 | ⚠️ 初始重合→∞ | 2945, Dimarogonas |
| CBF / EMB-DCBF-QP | ✅ 前向不变 | ➖ 硬约束无极小 | ⚠️ 紧约束 QP 无解 → CLF-CBF 松弛 / EMB-DCBF | 3258, 3084, 1740 |
| MPC / LMPC | ✅ 预测避陷（非凸局部最优） | ✅ 状态约束编码 | ⚠️ infeasible → soft constraint | 966, 3075, 1818 |
| DRL | ❓ 训练覆盖无保证 | ❓ 同上 | ❓ 同上 | 919, 1859 |

图例：✅ 构造上规避/解决　⚠️ 有风险但可缓解　❌ 易失效　➖ 不属于该类问题域

---

## 9. 总结：三条线索是三种"分水岭"

1. **局部极小值**是**梯度势方法（APF / flocking / 聚合势）**的专属病；BLF（barrier→∞）、导航函数（Morse 近全局）、
   CBF（前向不变）、MPC（预测优化）从**构造原理**上就规避——二者是避碰方法演进的**第一道分水岭**
   （详见《swarm_control_agent_collision_avoidance.md》§7 点 4：`doc:1547` 完全不处理 vs `doc:409` 认真打补丁）。

2. **特殊队形 / 退化平衡（共线/共面）**是**距离/刚体编队**的专属病，与局部极小机理完全不同：
   `doc:2786` 证明退化平衡必然存在且具有 rank-preserving 不变性（一旦共线出不来），距离误差还会诱发螺旋/匀速旋转
   （`doc:2786` §10.1.2、`doc:3107`）。解法不走"势场改造"，而走**积分控制 / 估计回路 / 滑模有限时间全局稳定（786）/ 换位移编队（665）**。
   这是**第二道分水岭**——它只折磨"形状由距离刚度决定"的编队，不折磨 swarm 势场或 CBF/BLF。

3. **奇异点 / QP 不可行**是**硬约束方法（BLF / CBF）**在边界上的病：BLF 在 agent 重合时 barrier→∞ 致执行器饱和；
   CBF 在约束冲突紧集上 QP 无解。二者共享"forward invariance 只保不越界、不保从界外救回"的前提，
   解法分别是**非对称/时变 BLF（823）**与 **CLF-CBF 松弛 / EMB-DCBF 低阶微分（3258）**。这是**第三道分水岭**。

> 一句话：选避碰方法时先问"我会撞上哪类失效"——怕卡在陷阱选 BLF/NavFn/CBF/MPC；
> 做距离/刚体编队怕共线卡死选滑模全局（786）或加积分/估计回路（2786）或换位移编队（665）；
> 上微型实时群选 EMB-DCBF（3258）而非集中式 MPC；DRL 仅作高性能补充、需 CBF 滤子兜底。
