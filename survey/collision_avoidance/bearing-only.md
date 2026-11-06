# Bearing-Only 与 Obstacle / Collision Avoidance 结合：文献调研

> 调研目的：梳理"只用方位（bearing / line-of-sight angle）测量"的多智能体控制中，
> 如何同时实现**碰撞避免（agent ↔ agent）**与**避障（agent ↔ obstacle）**，
> 并回答一个关键问题：**这类方法到底需不需要距离信息来辅助？**
>
> 资料来源：控制理论文献知识库 `okb-assist`（MCP）。文中 `doc:NNNN` 为知识库文档编号，
> 可用 `read_markdown` / `get_toc` 复查原文。

---

## 0. 先读这里：bearing 如何与避碰结合——三条主思路 + 三条补充

核心矛盾一句话：**碰撞避免的安全集必须写成距离** `h = ‖p_i − p_j‖² − r²`，而 bearing 只是方向、**自身不含尺度**。
所以所有工作本质上都在回答同一个问题：**从哪里借一个"已知长度"（尺度）来把角度变成距离？**
已知长度放在哪里，就分出三条主思路：

| # | 思路 | 尺度（已知长度）在哪 | 典型测法 | 代表工作 | 还是严格 bearing-only？ |
|---|---|---|---|---|:---:|
| **①** | **自身双观测点 → 三角测距** | **在自己身上**：两个传感器间的**基线** `b` 已知 | 同时测邻机的**两条** bearing，交会得距离；`D ≈ b/Δθ`（远距） | 立体视觉/双目；多机或多时刻基线：`doc:328`（时空三角测量）、`doc:36`（双 UAV）、`doc:1107`（时间基线里程计） | 是（但需双目或已知基线） |
| **②** | **目标侧已知尺寸 → 张角测距** | **在目标身上**：邻机上的标定物**尺寸** `l_j / r_j` 已知 | 测**同一个物体上的两个点**（竖杆顶/底、球两侧切点）→ 张角 `θ` → 距离 | `doc:3948` §IV.A（elevation angle，2D 杆 / 3D 球）；变体 `doc:2061`（尺寸 ℓ **在线估**） | 是（加装已知尺寸物） |
| **③** | **不测距，改为结构/初值条件** | **哪也不借**：靠闭环轨迹本身不交叉 | 证明"初值误差足够小 + 构型间距足够大 ⇒ 永不相撞" | `doc:3103` Theorem 12（Zhao & Zelazo，**源头**）、`doc:2082`、`doc:2605`、`doc:2579` | 是（零额外信息，代价最保守） |
| ④ | 显式测距/相对位置（**放弃** bearing-only） | 直接测 | 障碍物位置、机间距离、短程红外 | `doc:3501`、`doc:237`、`doc:1055`（15 cm 内红外） | 否 |
| ⑤ | 角度扇区 / 碰撞锥（纯角度 + 最坏情况过估计） | 不借尺度，只保证"不重合" | 把视线两侧转 `±ψ` 围出禁区，把控制投影到区外 | `doc:3892`（机间，`n ≤ ⌈ψ_a/2ψ_b⌉`）、`doc:725`/`doc:701`/`doc:1655`（collision cone，经典式仍含 `r₀`） | 是（但只保点模型不撞） |
| ⑥ | 估计器路线（距离"算"出来） | 借可观性 | 机动增强 PE + 滤波器估距离；或 range-only 反推 bearing（对偶） | `doc:2785`（螺旋制导 + PLKF）、`doc:3171`（range-only + bearing 估计器） | 是（需额外机动/另一类传感器） |

**两条最有用的直觉**：

- **①与②在几何上等价**：都是给纯角度几何补一条**已知长度的基线**——区别只在基线画在"自己身上"（双目）
  还是"目标身上"（已知杆/球）。`doc:3948` 的 Lemma 4（`l_j = d_f`、`r_j = d_f/2` 时估计误差最小）
  本质上就是"基线长度取到与被测距离同量级时三角测距最准"。
- **③是 ①② 都不允许时的唯一出路**，但它给的不是"安全约束"而是"安全条件"：一旦初值不满足就无保证，
  且碰撞临界时 bearing 本身失效（`doc:3103` 原文："if two neighbors collide, the bearing between them will be
  mathematically invalid"）。

下文 §2~§5 按方法族展开（隐式条件 / CBF-QP / 角度反推距离 / 角度扇区），§6 把这些整理成"距离的 7 种来源"，
§8 附 Shiyu Zhao 相关工作盘点。

---

## 0.1 检索方式（可复现）

| 手段 | 查询 |
|---|---|
| `search_info`（元数据） | `bearing-only`、`bearing`、`collision avoidance`、`obstacle avoidance`、`bearing-based collision-free`、`safety-critical formation` |
| `grep_search`（全文正则） | `bearing[- ]?only[^.]{0,80}collision`、`bearing-only.{0,60}control barrier`、`collision[- free]{0,8}[^.]{0,80}bearing[- ]?only`、`auxiliary distance measurements`、`collision cone` |

筛出**同时涉及 bearing(-only) 与 safety** 的核心文献（含 1 篇综述）：

| doc | 年份/期刊 | 题目 | 安全对象 | 方法族 |
|---|---|---|---|---|
| `doc:3948` | 2026 TIE | Bearing-Only Collision-Free Formation Tracking: A Safety-Critical Control Approach | agent–agent | GP-ECBF + **方位反推距离** |
| `doc:3501` | 2019 IFAC WC | CBF-Based Bearing-Only Formation Tracking with Obstacle Avoidance | 静态凸障碍 | CBF-QP（闭式解） |
| `doc:237` | 2025 TASE | Bearing-based collision-free formation control for spacecrafts under dynamic ETM | agent / 静态+动态障碍 | Backstepping + CBF-QP |
| `doc:2082` | 2021 TCAS-II | Finite-time bearing-only formation tracking with collision avoidance | agent–agent | 梯度 + **初始条件充分条件** |
| `doc:3892` | 2024 JFI | Collision-free zero-communication finite-time localization & circumnavigation (bearing-only) | agent–agent | **角度扇区**投影，零通信 |
| `doc:1055` | 2015 Automatica | Enclosing a target by nonholonomic robots with bearing-only measurements | agent–agent（实验） | 行为式 + **短程测距** |
| `doc:2605` | 2021 Automatica | Safe convex-circumnavigation around multiple targets (bearing-only) | 目标凸包（keep-out） | 几何构型设计（隐式） |
| `doc:3949` | 预印本 | Bearing-Only Formation Tracking for EL MAS Without Communication | —— | **无**安全保证（对照组） |
| `doc:3171` | 2025 IoT-J | Range-only distributed safety-critical formation control（bearing 估计器 + CBF） | agent / 障碍 | **对偶问题**：range + bearing 估计 |
| `doc:3838` | 2026 ARC | Bearing-based multi-agent formation control: A survey and taxonomy | 综述（§9.3 安全性小节） | explicit vs implicit 分类 |

几何类旁证（角度几何避碰，非编队控制）：`doc:725`（collision cone 原始文献）、
`doc:701`/`doc:1655`（CC + CBF）、`doc:2061`（bearing-angle：用张角反推距离）。

---

## 1. 结论速览（先给答案）

1. **"纯 bearing + 严格安全性"在理论上是有结构性矛盾的**：碰撞避免的安全集必须用**距离**表达
   （`h(x) = ‖p_i − p_j‖² − r²`），而 bearing 只给出方向、**不含尺度**。
   `doc:3838` §9.3 明确写道：
   > "since many safety objectives (e.g., collision avoidance) are described by **distances** between
   > the neighboring agents, it is usually **hard** to achieve safety-critical formation control based on
   > bearing measurements."
   并把已有工作分为 **explicit（显式定义前向不变集，如 CBF）** 与 **implicit（用闭环轨迹/充分条件隐式刻画）** 两类。

2. 知识库中真正突破这一点的只有 `doc:3948`：**不测距**，而是用**已知物理尺度的俯仰角（elevation angle）**
   反算出机间距离，再送入 ECBF。也就是说，**"bearing-only" 并不是"零尺度信息"**，
   而是把尺度需求转移到了"已知物体尺寸 + 角度分辨率"上。

3. 因此回答"是否需要距离信息辅助"，应分成 4 个精度层级：

| 层级 | 需要的信息 | 代表工作 | 代价 |
|---|---|---|---|
| L0 纯 bearing + 假设 | 方位 + 初值/数量/角间距假设 | `doc:2082`、`doc:3892`、`doc:2605` | 保守、只对特定场景成立 |
| L1 bearing + **已知尺度** | 方位 + 邻机已知几何尺寸（杆长/球半径） | `doc:3948` | 需在机器人上加装已知尺寸的标定物，近距离时精度下降 |
| L2 bearing（编队）+ **局部测距** | 编队用方位，安全用短程测距/障碍物相对位置 | `doc:1055`（15 cm 红外）、`doc:237`、`doc:3501` | 少量距离传感器，但保证较弱（行为式）或需障碍物全局位置 |
| L3 range-only 对偶 | 测距 + 方位估计器 | `doc:3171` | 传感器与估计器设计复杂度互换 |

---

## 2. 技术路线一：隐式安全（用充分条件而非约束）

**核心想法**：不去约束 `‖p_i − p_j‖`，而是证明"在 bearing-only 控制器 + 某组初值/拓扑条件下，
轨迹本身不会相交"。CBF 完全不需要，也不需要任何距离测量。

### 2.1 有限时间梯度律 + 初值充分条件（`doc:2082`，TCAS-II 2021）

- 异构单积分机器人：`ṗ_i = S_i u_i`（`S_i = diag(s_i1,…,s_id) > 0`）。
- 控制器（有限时间尺度函数 `μ(t) = (T/(T−t))^h`）：
  `u_i = (a + b μ̇/μ) U_i Σ_{j∈N_i} (g_ij − g*_ij)`，`U_i = S_i^{-1}`。
- **碰撞避免 = 一条初值不等式**（Theorem 1, 式 (13)）：
  `‖δ(0)‖ ≤ (1/√n)( min_{i,j} ‖p*_i − p*_j‖ − γ )`，即"初始位置误差要远小于目标构型的最小间距"。
- 另有 (14) 式保证输入一致有界且 `C¹` 光滑：`b h λ_min(B_ff) > 2‖H̄‖(‖δ(0)‖ + ‖p̃*‖)`。
- 评价：**零额外传感、零通信**，但安全性是"初值依赖"的工程条件，不是任意初值下的硬保证。
- **源头**：这条充分条件最早由 Zhao & Zelazo（`doc:3103`, TAC 2016）在 §III.C 给出（Theorem 12、式 (17)），
  并明确承认 control law (9) 无法全局保证避碰（该文 Fig. 8 给了碰撞反例），建议与人工势场等机制配合使用。
  `doc:2082` 的式 (13) 与之同构。详见第 8 节。

### 2.2 几何构型设计实现 keep-out（`doc:2605`，Automatica 2021）

- 单智能体绕**多个静止目标凸包**的 "safe convex-circumnavigation"：目标位置用标量估计器在线估计，
  速度分解为"指向凸包"+"与其正交"两项。
- 安全性来自**构型设计**（沿凸包外扩的闭环轨迹天然不穿越凸包），而非约束求解。
- 需要：自身位置已知 + 目标的 bearing-only 观测（不需要到目标的距离）。

### 2.3 综述给出的定位（`doc:3838` §9.3）

- explicit（CBF/不变集）：保证硬，但"多数只能用于单积分模型"；
- implicit（靠闭环轨迹/恢复轨迹预测）：适应动态环境好，但依赖轨迹预测精度、对噪声敏感。
- 现状判断与本调研一致：**bearing-only 的 safety-critical 控制仍是开放问题**。

---

## 3. 技术路线二：CBF / ECBF-QP（显式安全集）

### 3.1 非完整机器人 + 手点（hand point）技巧（`doc:3501`，IFAC 2019）

- 系统：单轮机器人 `ṗ_i = u_i e_1i, θ̇_i = ω_i`（非完整，输入为 `(u_i, ω_i)`）。
- **关键技巧**：不用机器人中心 `p_i`，而用"手点" `q_i = p_i + l_i e_1i`（`l_i > 0` 为常数）。
  这样 `q̇_i = R_i(θ_i) B [u_i, ω_i]^T`，控制量同时出现在 `ḣ` 中，且
  `∇h = b_{o_k i} ≠ 0` 在边界上处处非零 ⇒ **相对度 1 的有效 CBF**（避免高阶 CBF）。
- CBF：`h(q_i) = ‖q_i − o_k‖ − (r_i + R_k)`（`o_k` 为障碍中心，`R_k` 为其半径，凸形障碍可用外接圆保守化）。
- QP（FC-CBF）：`min ½‖μ̄_i − μ_i‖²` s.t. `b_{io_k}^T μ̄_i − γ h(q_i) ≤ 0`，其中名义流
  `μ_i = k_p Σ_j (b_ij − b*_ij) + v_c`（`v_c` 为领航者共同的期望速度）。
- **闭式解**：`μ̄_i = (I₂ − b_{io_k}b_{io_k}^T) μ_i + γ h(q_i) b_{io_k}`
  = 名义速度在切向的投影 + 沿法向的减速项 ⇒ 只做最小修正，且能证明编队仍渐近收敛。
- 注意其**信息需求**：`h` 与 `b_{io_k}` 都用到障碍物位置 `o_k`，即**需要到障碍物的相对位置（距离+方位）**；
  编队部分才是纯 bearing。文中还标注两个工程缺陷：
  - Remark 7：`μ_i ∥ b_{io_k}` 退化为零控制，可能停在 `∂C` 上的坏平衡；
  - Remark 8：两个障碍贴得太近时 QP 可能不可行 ⇒ 用外包安全圆保守处理。

### 3.2 Backstepping + CBF-QP + 动态事件触发（`doc:237`，TASE 2025）

- 对象：多航天器（3D，双积分），编队目标用 bearing 约束 `‖g_k − g*_k‖ ≤ Θ₁` 表述。
- 明确假设："each spacecraft is equipped with onboard sensors that are able to obtain the
  **bearings and distances** among inter-neighbors and obstacles"。
- 安全约束直接写成距离不等式：`‖ρ_i − ρ_l‖ ≥ r + o_l`（障碍，`ρ_l(t)` 可随时间变，即**动障碍**）、
  `‖ρ_i − ρ_j‖ ≥ r`（机间）。
- CBF-QP 嵌在 backstepping 反步框架里（bearing-only 跟踪律作前馈，QP 作安全滤波），
  并用**输入式动态 ETM**（触发条件只用系统输入）调度控制器→执行器的传输，排除 Zeno 行为。
- 这是"**bearing 做编队 + 距离做安全**"最干净的样例，代价是传感器需求不再是 bearing-only。

### 3.3 GP-ECBF + 方位反推距离（`doc:3948`，TIE 2026）——最接近"纯 bearing"的一篇

结构分三层：

1. **名义 bearing-only 跟踪律**（式 (5)，无通信、时变速度目标）：
   `u_i = k_p Σ_j (g_ij − g*_ij) + k_v Σ_j ġ_ij + ξ Σ_j P_{g_ij} sign(ġ_ij)`，
   利用正交投影 `P_g = I − gg^T` 与 bearing Laplacian：速度误差 → 0，位置误差一致最终有界。
2. **方位反推距离**（§IV.A，下一篇单独讲）。
3. **安全层**：`h_i = ‖p_i − p_j‖² − r_ij²`（相对度 2）⇒ 需要 **ECBF**（指数 CBF），
   约束里出现未知项 `L_{d_i} L_{f_i} h_i = 2(p_i − p_j)^T δ_i(v_i, u_i)`
   （状态+输入相关不确定性）⇒ 用**GP + 注意力调制复合核（AMC kernel）**给出置信上界，
   转成 SOCP 求解，保证高概率安全。相对度 2 这一高相对度 + 状态/输入混合不确定性是其主要理论贡献
   （其 Table I 逐项对比了 ISSf-CBF / worst-case RCBF / adaptive RCBF / observer RCBF / GP-CBF / compound-kernel GP-CBF）。
4. 实验：4 台自研 mini-AUH（r=0.35 m）+ 6 相机 MoCap，验证 GP 估计与避碰（Fig. 5/6）。

**局限**：CBF 只处理机间碰撞（不含静态障碍）；需要 `r_ij` 对应的**距离估值**，其精度直接决定安全裕度。

---

## 4. 技术路线三：用 bearing 自己"造"距离（本篇最关键）

### 4.1 Elevation-angle 距离反演（`doc:3948` §IV.A）

前提：给每个邻机的机身上装一个**尺寸已知**的标定物，方位传感器能看到它的两个特征点。

- **2D**：邻机 `j` 竖直立一根长度 `l_j` 的杆，`i` 测得杆顶/杆底的**俯仰角** `θ_ij = arccos(g_ij^T g_ij')`：

  `‖p_i − p_j‖ = l_j g_ij^T g_ij' / √(1 − (g_ij^T g_ij')²)`  （式 (21)）

- **3D**：邻机 `j` 外套半径 `r_j` 的球，`i` 测得球两侧切点的张角 `θ_ij`：

  `‖p_i − p_j‖ = r_j √(2 / (1 − g_ij'^T g_ij''))`  （式 (22)）

- **精度设计（Lemma 4）**：在固定各向同性方位噪声 `Δg` 下，`|ΔD| = (l_j + D²/l_j)‖Δg‖`，
  在关键安全距离 `d_f` 处取极值 ⇒ 选 **`l_j = d_f`（2D）**、**`r_j = d_f/2`（3D）** 可使估计误差最小。
- 物理直觉：这就是**单目视觉的"张角测距"**——用已知尺寸的物体把角度信息换算成尺度；
  与 `doc:2061` 的 bearing-angle（张角）、`doc:3838` §7.1 的 elevation-angle rigidity 同源。
- 代价：① 需加装已知尺寸的结构；② 角度噪声被 `1/sin²θ` 放大，**近距/远距精度都差**，
  只在 `d_f` 附近最优；③ 依赖 `p_i` 已知（自身定位）。

### 4.2 角度扇区 + 最坏情况过估计（`doc:3892`，JFI 2024）

- 场景：`n ≥ 3` 智能体、**零通信**、有限时间目标定位 + 环绕（circumnavigation）+ 两重安全：
  AACA（机间不撞，`ρ_ij(t) > 0`）与 ATCA（不撞目标，`d_i(t) > 0`）。
- 只用：自身位置 `p_i`、到目标的 bearing `φ_iT`、到其他所有智能体的 bearing `φ_ij`（**无距离、无标签**）。
- 安全机制（纯角度几何）：
  1. **邻居策略**：用角度 `φ_ij` 定义"近邻 `N_i^c(t)` / 远邻 `N_i^d(t)`"（式 (14)(15)），
     目的是让安全修正**不破坏编队收敛**（Remark 3.2：这个区分是 Theorem 4.4 证明的关键）。
  2. **角间距估计器**：`β̂_{i,o_i} = 2φ_{i,o_i} − 3π` 或 `2φ_{i,o_i} + π`（式 (18)），
     只用本地角度即可得到与近邻的期望角间距误差，无需标签、无需通信。
  3. **禁区扇区**：`S_ij(t)`（式 (26)）用"把视线转 ±ψ_b"的两条射线围出的锥形区域，
     `S_in(t) = ∪_j S_ij(t)` 作为"可能导致碰撞区域的**过估计**"；`S_iT` 同理（要保证 PE 条件）。
     若名义控制 `u'_i ∈ S_i(t)`，则用投影替换：`u_i = g_is g_is^T u'_i`，其中 `g_is` 由在 `Q(u'_i) \ S_i(t)`
     上最大化内积的优化问题给出（式 (31)）。
  4. **可行性靠数量假设**：`3 ≤ n ≤ ⌈ψ_a/(2ψ_b)⌉`（Assumption 2.4，例 `ψ_a=85°、ψ_b=2°` ⇒ `n ≤ 22`），
     否则别的智能体可能把某个方向"全堵死"（Fig. 9 的不可行情形）。
- 评价：**知识库中最"纯 bearing"的机间避碰方案**——它不需要任何距离，但把代价换成
  "只保证不撞（`ρ_ij > 0`，点模型）"+"数量上限"+"角间距常数需谨慎选取"，
  且**没有给出最小安全间距 `r_min`**（点模型意义下的非碰撞，而非带半径的硬约束）。

### 4.3 bearing-only 的"绕飞半径"其实不需要测距（`doc:1055`，Automatica 2015）

- 单/多非完整机器人绕点目标、圆盘目标：控制器只用 bearing，但能收敛到**指定半径** `ρ*` 的圆上，
  "without distance measurement"——因为**期望半径是设计参数**，由控制器几何结构隐式实现，
  而非"测出来再比较"。这个区分很重要：**"不测距" ≠ "不用距离尺度"**。
- 多机协同用环形分布（角间距）机制避免聚集。
- 其**实验**部分非常坦诚（§4.1）：
  > "Designing a totally distributed and mathematically provable **bearing-only collision avoidance
  > algorithm remains an unsolved problem**… the current collision avoidance scheme uses **not only
  > bearing information**."
  实验里给机器人装了短程红外：在 `Q^i = {j | ‖x_j − x_i‖ ≤ d_s}`（`d_s = 15 cm`）内可同时测**方位与距离**，
  再用行为式速度调制 `ṽ_i = m_i v_i`（`Q_1^i` 正前方狭窄区 ⇒ 停；`Q_2^i/Q_3^i` ⇒ 加速/让行）。

---

## 5. 旁证：角度几何避碰（collision cone 家族）与"距离"的关系

- `doc:725`（Chakravarthy & Ghose, 1998）碰撞锥：判定条件本质上是
  "相对速度是否落在视线张成的锥内"，公式里显式含 `r₀`（距离）、`θ₀`（视线角）、`V_O, V_F`。
  即**经典 CC 是"角度几何 + 距离"**，不是纯 bearing。
- `doc:701`（2025）、`doc:1655`（ACC 2024）：把 CC 与 CBF 结合，得到**无需数值优化**的避碰律，
  但仍需相对位置/速度（UAV 视为移动障碍）。
- `doc:2061`（IJRR 2024）"bearing-angle"：用目标在图像中的**张角**（视觉尺寸）估计距离，
  是 §4.1 elevation-angle 的视觉版本——再次说明"纯角度 + 已知尺寸 = 尺度"。
- `doc:3044`（TASE 2021, barycentric-coordinate-based）也用 collision cone 做机间避碰，
  但其测量是**相对位置**（relative position），并非 bearing-only，引用时勿混用。

---

## 6. 是否需要距离信息辅助？——分层回答

### 6.1 为什么"纯 bearing"不够（三条硬理由）

1. **可观性**：bearing rigidity 只能把构型确定到"平移 + 尺度"（`doc:3838` §4.1），
   绝对尺度（机间距 `‖p_i − p_j‖`）本身不在 bearing 的信息里。
2. **约束形式**：CBF 需要 `h(x) ≥ 0` 且 `∇h` 在边界非零；只要能表达"安全"就必然出现距离
   （`doc:3501` 用 `‖q_i − o_k‖`，`doc:237` 用 `‖ρ_i − ρ_j‖`，`doc:3948` 用 `‖p_i − p_j‖²`）。
3. **工程失效模式**：`doc:3948` 指出近距离会**降低 bearing 测量质量**甚至导致系统失效，
   所以安全距离 `r_ij` 还承担"保证 bearing 有效"的第二重作用。

### 6.2 距离的 7 种"来源"（按侵入性排序）

| # | 距离从哪来 | 代表 | 是否需要额外硬件 | 保证强度 |
|---|---|---|---|---|
| 1 | **完全不需要**，改为初值/角间距/数量假设 | `doc:2082`、`doc:3892`、`doc:2605` | 否 | 弱~中（点模型、条件性） |
| 2 | 用**已知物理尺寸 + 俯仰角/张角**反算 | `doc:3948`、`doc:2061` | 加装已知尺寸杆/球 | 中~强（含置信界，误差可分析） |
| 3 | **短程测距**（近距才开） | `doc:1055` | 红外/超声/近距雷达 | 弱（行为式，实验验证） |
| 4 | 全程**相对位置/障碍物位置** | `doc:3501`、`doc:237` | 需障碍物地图或测距 | 强（CBF 前向不变） |
| 5 | **range-only** 反向补 bearing（对偶） | `doc:3171` | 测距 + 单路标 bearing 估计器 | 强（估计器收敛 + CBF 证明） |
| 6 | 靠**机动增强可观性** + 滤波器在线估距离 | `doc:2785`（螺旋制导 + PLKF 的 range control） | 否（但需额外机动，易与任务冲突） | 中（依赖 PE 条件） |
| 7 | **检测框张角 + 尺寸 ℓ 在线估计** | `doc:2061`（bearing-angle, IJRR 2024） | 否（用现成 bbox） | 中~强（ℓ 作状态估计，无需预知尺寸） |

`doc:3171` 特别值得并列阅读：它用 **range-only** 测量 + 一个**对单路标的 bearing 估计器**，
再用估计 bearing 构造 CBF 保证避障，并在四旋翼上做了杂乱实验室环境实验。
它和 `doc:3948` 构成一对镜像：**一篇把距离从 bearing 里推出来，一篇把 bearing 从距离里估出来**——
说明"到底缺 b 还是缺 d"取决于传感器配置，"混合 + 估计器"是被接受的工程路线。

### 6.3 实务建议（写给自己的课题）

- **若必须坚持 bearing-only（论文卖点）**：
  走 `doc:3948` 的路线 —— 名义律用投影/Lyapunov 的 bearing 控制（`doc:3948` 式 (5) 或 `doc:3949`），
  安全层用**已知尺寸的俯仰角反推距离** + ECBF；
  必须补做"距离估计误差 → 安全裕度"的敏感性分析（`doc:3948` Lemma 4 的 `|ΔD|` 公式即模板）。
- **若允许弱辅助（更容易发表/落地）**：
  分层设计——编队/环绕用 bearing（`doc:3501` 的 `μ_i`、`doc:3892` 的 `u'_i`），
  安全用**近距测距触发的 CBF**，并显式声明"安全层不是 bearing-only"（`doc:1055` 的坦诚写法反而更受认可）。
- **若想回避尺度问题**：把安全性降级为"隐式保证"（初值充分条件 `doc:2082`、构型 keep-out `doc:2605`，
  或角度扇区 `doc:3892`），并明确写出适用范围/数量上限/点模型假设。
- **需要小心的三个坑**：
  ① `doc:3501` Remark 7 的零控制死锁（`μ_i ∥ b`）与 Remark 8 的多障碍 QP 不可行；
  ② `doc:3892` 的数量上限 `n ≤ ⌈ψ_a/(2ψ_b)⌉`（规模一大就要重调 `ψ_a, ψ_b`）；
  ③ `doc:3948` 的距离估计精度只在 `d_f` 附近最优，且角度噪声被 `1/sin²θ` 放大。

---

## 7. 空白与可做的点

0. **思路①（自身双目基线）在避碰里是空的**：库内做三角剖分的（`doc:328`、`doc:36`、`doc:1107`）都是**定位/里程计**，
   没有一篇用"已知双目基线 + 交会距离"去构造避碰 CBF。理论上它比 `doc:3948` 的思路②更优雅
   （基线在自己身上，不用给每台邻机装标定物，且基线长度可做得比杆长更长 ⇒ 三角精度更高），
   难点是双目同步、小基线远距退化（`D ≈ b/Δθ`，基线小则角度噪声放大）。
1. **3D + 全向标定物**：`doc:3948` 的 3D 方案要球体（`r_j = d_f/2`），实际多机系统很难装；
   换成相机 + 已知标记板（视觉张角）是自然延伸。
2. **动态障碍**：知识库里 bearing-only 的工作几乎只处理机间碰撞（`doc:3948`）或静态凸障碍（`doc:3501`）；
   动态障碍只有 `doc:237`（且用了距离）。"bearing-only + 动障碍预测"是空位。
3. **相对度问题**：双积分/高阶系统的 `h` 相对度为 2 ⇒ 必须 ECBF（`doc:3948`）；
   非完整模型靠"手点"降阶（`doc:3501`）。把两者结合（非完整 + 高阶安全 + bearing-only）仍未见。
4. **鲁棒/随机保证**：只有 `doc:3948` 用 GP 给出高概率保证；
   bearing 噪声 + 距离反演误差的**联合**鲁棒性分析（ISSf / 自适应 CBF）还没人做。
5. **零通信 + 强保证的取舍**：`doc:3892` 零通信但只保证点模型不重合；`doc:3948` 零通信 + 硬安全但依赖尺度标定。
   二者的边界（"多少标定信息能换多少保证强度"）值得写成一个命题。

---

## 8. 附：Shiyu Zhao 的相关工作盘点（okb 检索结果）

检索口径：`search_info("Shiyu Zhao")` ⇒ 作者字段命中 **13 篇**（`Zhao Shiyu` = 0，库内统一英文名序）。
逐篇 `grep_search("collision|obstacle|avoid")` 判定相关性：

| doc | 年份/期刊 | 涉及安全? | 讨论方式 |
|---|---|:---:|---|
| `doc:3103` | 2016 TAC（Zhao & Zelazo） | ✅ **最相关** | 专设 §III.C *Collision Avoidance*：Theorem 12 给出充分条件 `‖δ(0)‖ < (1/√n)(min‖p*_i−p*_j‖ − γ)`；承认控制律本身不能全局保证；建议叠加人工势场 |
| `doc:2579` | 2018 TAC（Trinh, Zhao, …, Ahn） | ✅ 态度性 | "bearing vectors are undefined when the neighbor agents are collocated, the analysis is valid **when collision avoidance is guaranteed**"；并主张碰撞避免可**独立**由视觉技术处理（外挂模块） |
| `doc:2782` | 2019（Zhao & Zelazo，bearing rigidity 综述） | ✅ 思路性 | 用 bearing 独有的**尺度自由度（formation scale control）**应对环境，如穿过窄通道实现避障 |
| `doc:122` | 预印本（Li, Wang, Ding, Guo, Zhao） | ✅ 实现层 | bearing-only 协同追捕 + MARL：避碰写成奖励项（`r_5 = −10` if collision），观测还用到**通信共享的队友相对位置** ⇒ 软约束、无硬保证 |
| `doc:2785` | 2023 T-RO（Li, Ning, He, Lee, Zhao） | △ 距离可观性 | 3D bearing-only 跟随：**螺旋机动增强可观性** + 用**估计**距离做 range control（`ξ₁ = ‖Δp̂‖ − r_d`），不测距 |
| `doc:2061` | 2024 IJRR（Ning, Zhang, Li, Chen, Zhao） | △ 尺度问题 | bearing-angle：把**目标物理尺寸 ℓ 当作待估状态**（§4.2），用检测框张角提升可观性 ⇒ **不需预知尺寸**即可恢复尺度 |
| `doc:2781` | 2019 TAC | ✗ 仅动机 | 引言只提 "respond to dynamic environments, for example, avoid obstacles"，无避碰设计 |
| `doc:1435`/`doc:328`/`doc:2777`/`doc:1412`/`doc:1133` | — | ✗ | 无相关内容（`doc:3524` 仅作者简介提到研究兴趣） |

### 8.1 与本调研三条主线的对应

1. **§2.1 隐式安全谱系的源头就是 `doc:3103`**（早于 `doc:2082` 五年），且把"为什么需要这条假设"讲得最清楚：
   碰撞时 bearing 数学上无定义 ⇒ 稳定性结论只到碰撞前一刻。引用顺序应是 **3103 → 2082**。
2. **§4.1「bearing 反推距离」的"未知尺寸"版本 = `doc:2061`**。`doc:3948` 的 elevation-angle 需要**已知** `l_j / r_j`；
   `doc:2061` 把 `ℓ` 作为状态量一起估（球/圆柱直径、车辆高度、四旋翼轴距等"某方向不随视角变化"的尺寸），
   正好把 3948 的**主要弱点**变成可解问题——但它是**估计**问题、不是控制问题，没有避碰约束。
   两者拼起来才是完整链条：bearing + 张角 → 距离 → CBF。
3. **§6.2 距离来源表第 6/7 行 = `doc:2785` / `doc:2061`**：距离既可"由已知尺度换算"（3948），
   也可"由机动 + 滤波器估出"（2785），或"把未知尺寸在线估掉"（2061）。

### 8.2 两点注意事项

- **Zhao 学派整体把碰撞避免当"外挂模块"**（`doc:2579` 表述最直白）：其 bearing-only 主线中没有 CBF / 不变集 / 硬约束的工作。
  这与 `doc:3948`（TIE 2026）声称"首个从控制角度解决 bearing-only 避碰"并不矛盾——3948 的 related work 亦如此定位。
- **库内缺一篇最对口的文献**：Tron, Thomas, Loianno, Daniilidis, Kumar,
  *Bearing-only formation control with auxiliary distance measurements, leaders, and collision avoidance*（CDC 2016）
  —— 只在 `doc:2694` 参考文献 [108]、`doc:2782` 参考文献中出现，**正文未入库**。
  它才是"bearing 为主 + **辅助距离** + 避碰"最直接的先例，若要写"辅助距离"这一支建议补原文。

---

## 9. 参考文献（知识库 doc 索引）

- `doc:3103` Bearing Rigidity and Almost Global Bearing-Only Formation Stabilization, IEEE TAC, 2016（§III.C 避碰充分条件）
- `doc:2579` Bearing-based formation control of a group of agents with leader-first follower structure, IEEE TAC, 2018
- `doc:2782` Bearing rigidity theory and its applications: life beyond distance rigidity, 2019（尺度控制过窄通道）
- `doc:2781` Bearing-only formation tracking control of multiagent systems, IEEE TAC 64(11):4541–4554, 2019
- `doc:122` Cooperative Bearing-Only Target Pursuit via Multiagent Reinforcement Learning（避碰作奖励项）
- `doc:2785` Three-dimensional bearing-only target following via observability-enhanced helical guidance, IEEE T-RO, 2023
- `doc:2061` A bearing-angle approach for unknown target motion analysis based on visual measurements, IJRR, 2024
- `doc:3948` Bearing-Only Collision-Free Formation Tracking Control for MAS: A Safety-Critical Control Approach, IEEE TIE, 2026, doi:10.1109/tie.2026.3730129
- `doc:3501` Control Barrier Function-Based Bearing-Only Formation Tracking of Mobile Agents with Obstacle Avoidance, IFAC WC 2019（Tran, Lee, Yuan, Ahn）
- `doc:237` Bearing-Based Collision-Free Formation Control for Spacecrafts Under Dynamic Event-Triggered Input, IEEE TASE, 22:14855–14866, 2025
- `doc:2082` Finite-Time Bearing-Only Formation Tracking of Heterogeneous Mobile Robots With Collision Avoidance, IEEE TCAS-II, 2021, doi:10.1109/TCSII.2021.3066555
- `doc:3892` Collision-free zero-communication finite-time target localization and circumnavigation … bearing-only, J. Franklin Inst. 361:106972, 2024
- `doc:1055` Enclosing a target by nonholonomic mobile robots with bearing-only measurements, Automatica 53:400–407, 2015
- `doc:2605` Safe convex-circumnavigation of one agent around multiple targets using bearing-only measurements, Automatica, 2021
- `doc:3949` Bearing-Only Formation Tracking Control for Euler–Lagrange MAS Without Inter-Agent Communication（无安全层，作对照）
- `doc:3171` Range-only distributed safety-critical formation control based on contracting bearing estimators and CBFs, IEEE IoT-J 12(19):40968–40979, 2025
- `doc:3838` Bearing-based multi-agent formation control: A survey and taxonomy, Annual Reviews in Control, 2026（§7.1 elevation angle、§9.3 safety-critical）
- 旁证：`doc:725`（collision cone, 1998）、`doc:701`（CC+CBF, 2025）、`doc:1655`（CC+CBF, ACC 2024）、`doc:2061`（bearing-angle, IJRR 2024）、`doc:3044`（BCB + collision cone，用相对位置）
