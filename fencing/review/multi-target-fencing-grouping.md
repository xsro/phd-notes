---
title: "多目标 fencing：单/多凸包切换与分组机制"
params:
   math: true
---

调研对象（OKB 文档 ID）：

- **[A] Yu, Yang, Zhang (2026)**，*Adaptive cooperative fencing and dynamic reorganization control of UAV swarm for multiple maneuvering targets*，Complex & Intelligent Systems，OKB `id=2896`
- **[B] Hu, Yao, Wei, Lv (2024)**，*FSOC: Flexible subgrouping and ordering of multi-robot coordination for convoying multiple scattered targets*，IEEE TCNS，OKB `id=1954`

补充对照：

- **[C]** *Plug-and-play distributed fencing of multi-vehicle systems for multiple uncooperative targets with dynamic radius*，OKB `id=3116`
- **[D]** *Multi-robot systems: Coordinated fencing control and applications*（专著），OKB `id=1976`
- 本目录 `chen2019/`：单目标 fencing 的 singleton（共线退化）问题

---

## 0. 结论先行

多目标 fencing 的核心不是"把单目标控制器复制 M 份"，而是先回答两个问题：

1. **要几个凸包？**（一个包住全部目标 vs. 每个目标/每簇一个凸包）
2. **谁进哪个包？**（目标如何分簇、UAV 如何分配、分配是否可在线重构）

两篇文献正好代表两条技术路线：

| | [A] 2026 | [B] FSOC 2024 |
|---|---|---|
| 分组方式 | **显式两阶段**：目标 K-means 聚类 → 两轮拍卖分配 UAV | **隐式一体化**：把 subgrouping 变量 $\alpha_i$ 直接塞进 CBF 优化，与控制器同时求解 |
| 凸包数量 | 模式 S1（单包）/ S2（多包），阈值触发切换 | 恒为 M 个包（每个 scattered target 一个） |
| 分组触发 | 目标速度方向发散角 $\theta_{\max}$ 或目标间距 $\varepsilon_d$ 超阈值 | 无触发；每步由优化自然产生/改变分组 |
| 排序 | 势场平衡距离决定，无显式排序 | 显式证明"任意排序"均可（flexible ordering） |
| 失败重构 | 有完整规则（UAV 失效/目标失效/复合失效） | 靠重解优化自发重构（仿真验证，无显式规则） |
| 理论保证 | 分层：K-means 局部收敛、拍卖有限步终止、控制器收敛（主要证 S1） | 端到端：分组收敛 + 无无限切换 + 避碰 + 围合 + 速度一致 |

一句话：**[A] 是"先分组、再控制"的工程化框架；[B] 是"分组即控制"的优化式框架。**

---

## 1. [A] 2896：双模式 fencing + 目标聚类 + 拍卖分配

### 1.1 问题设定：两种多目标场景

论文把多目标 fencing 分成两类（Problem statement）：

- **Case 1（S1，单凸包）**：目标同速/运动状态相近（成簇）。一个凸包包住所有目标即可。
- **Case 2（S2，多凸包）**：目标速度/方向分散。必须把 UAV 分成子群，每个子群负责一个目标子集。

注意这里的"fencing"判据是**凸包包含**：目标到 UAV 位置凸包的距离

$$
D_s(p)=\min_{r\in \mathcal{C}(p)}\|p_t-r\|,
$$

当 $p_t\in\mathcal{C}(p)$ 时 $D_s=0$，即围合成功。与 Chen 2019 的 $P_{x_o}(x)\to 0$ 是同一判据。

### 1.2 控制律骨架（两种模式共用）

$$
u_i = u_i^p + u_i^c + u_i^{f_1} + u_i^{f_2}
$$

- **导航项** $u_i^p$：跟踪参考点（S1 用目标群几何中心，S2 用被分配目标的估计状态）
- **速度一致项** $u_i^c$：只与**同子群**邻居 $n_i\subseteq N_i$ 做速度对齐
- **机间避碰/成包项** $u_i^{f_1}$：势场，平衡距离 $R_{des}$
- **目标/障碍避碰项** $u_i^{f_2}$：把目标当成动态障碍，防止撞上目标

**关键差异在 $R_{des}$：**

$$
R_{mt}=\sup_{k}\|p_{t,k}-\bar p_{t}\|,\qquad
R_{des}=\kappa(R_{mt}+R_{red})
$$

- S1：$R_{mt}>0$，凸包要同时覆盖整个目标群的尺寸，$R_{des}$ 更大；
- S2：每个子群只包一个目标，$R_{mt}=0$，取
  $$
  2r_{in}<R_{des}=\kappa R_{red}<R_{com}.
  $$

势场 $\varphi_c$ 在 $(2r_{in},R_{des})$ 排斥、$R_{des}$ 处为零（全局极小）、$(R_{des},R_{com})$ 吸引——所以**包的大小由 $R_{des}$ 隐式决定，而不是预设编队**。这就是文中"self-organizing / 不依赖预设队形"的含义。

> 与 Chen 2019 的联系：$u_i^{f_2}$ 就是"目标斥力项"，正是 `chen2019/post.md` §4.1 讨论的扩展；但 [A] 没有 Chen 2019 的旋转项 $\beta R x_{ij}$，因此**没有显式排除共线 singleton**。

### 1.3 模式切换：什么时候从 1 个包变多个包

对第 $k$ 个目标定义时间窗 $[t_0,t]$ 内速度方向变化角

$$
\theta_k(t)=\cos^{-1}\!\left(
\frac{q_{t,k}(t)\cdot q_{t,k}(t_0)}
{\|q_{t,k}(t)\|\,\|q_{t,k}(t_0)\|}
\right),
\qquad
\theta_{\max}(t)=\max_{k\in V_T}\theta_k(t).
$$

再记目标间观测距离 $\varepsilon_d=\|\hat p_{i,k}-\hat p_{i,l}\|$，阈值 $\theta_{th}$、$\varepsilon_{\max}$。

- **S1 保持**：$\theta_{\max}<\theta_{th}$ 且 $\varepsilon_d<\varepsilon_{\max}$；
- **触发切换到 S2**：$\theta_{\max}\ge\theta_{th}$ **或** $\varepsilon_d\ge\varepsilon_{\max}$。

两个判据分别对应"**运动方向发散**"和"**空间上已经散开**"，比只看距离更稳健；且用的是**分布式 prescribed-time 观测器**给出的估计值，不要求全局可见。

> 实现上要注意：文中只给了进入 S2 的触发式，S1 条件作为其补集给出；没有显式 dwell-time / 迟滞，阈值附近可能抖动。这是可以改进的点。

### 1.4 分组流水线（Task scheduling，Fig. 3）

**Step 1 — 目标聚类（K-means）**

$$
\min_{\{C_l\}_{l=1}^{N_c}}
\sum_{l=1}^{N_c}\sum_{p_{t,i}\in C_l}
\|p_{t,i}-\mu_l\|^2
$$

- 按**目标位置**聚类，簇心 $\mu_l$ 为目标位置均值；单目标簇的簇心就是该目标位置。
- 目标函数只含几何距离，**不含速度**；速度相似性是通过 §1.3 的触发式间接体现的（先判定"发散了"，再按位置分簇）。
- 簇数 $N_c$ 在文中作为给定参数出现，**未给出自适应选取规则**（实现上需自己定，例如按 $\varepsilon_{\max}$ 做肘部法/DBSCAN）。

**Step 2 — 两轮拍卖分配 UAV（multi-round auction）**

记 $N_{\min}$ 为每簇最少 UAV 数（按任务可调；3D 中至少 4 架才能形成有内点的凸包）。

- **Round 1：最小保障分配**
  $$
  \mathcal{L}_l=\Big\{i_1^l,\dots,i_{N_{\min}}^l\Big\},
  \qquad
  i_k^l=\arg\min_{i\in V_{ava}}\|p_i-\mu_l\|
  $$
  逐簇取最近的 $N_{\min}$ 架，取完从 $V_{ava}$ 删除；距离相同按 **ID 小者优先** 保证唯一性。
  这一轮保证"每个簇都能成包"。
- **Round 2：冗余感知的均衡分配**（剩余 UAV $V_{rem}$）
  $$
  \text{Cost}^*=\arg\min_l
  \big(\omega_{1,l}|L_l|+\omega_{2,l}\|p_i-\mu_l\|\big),
  \quad \omega_{1,l}+\omega_{2,l}=1
  $$
  贪心：每架剩余 UAV 选 Cost 最小的簇，加入后从 $V_{rem}$ 删除，直到分配完。
  $|L_l|$ 是簇负载，$\omega_1$ 大→更均衡，$\omega_2$ 大→更近。

输出映射 $f:V\to\{1,\dots,N_c\}$。性质（Remark 3/5/6）：

- 有限步终止、确定性；
- 第一轮在"最小覆盖"约束下局部距离最优；第二轮在总距离与负载间折中；
- **不保证全局最优**（组合分配问题），但可复现、可实时。

### 1.5 失败/事件响应（这是 [A] 相对 [B] 的强项）

只在 S2 需要动态调度（S1 少一架不影响单包）。

**UAV 失效**：从 $L_l$ 删除该 UAV，然后

- 若 $|L_l|\ge N_{\min}$ 且 $C_l\neq\emptyset$：不处理；
- 若 $|L_l|<N_{\min}$ 且 $C_l\neq\emptyset$：从**冗余簇**（$|L_c|>N_{\min}$）调最近的一架
  $$
  j^*=\arg\min_{j\in L_c,\,c\neq l,\,|L_c|>N_{\min}}\|p_j-\mu_l\|
  $$
  然后 $L_l\gets L_l\cup\{j^*\}$，$L_c\gets L_c\setminus\{j^*\}$。

**目标失效**：从簇中删除；

- 簇内还有目标：继续任务（必要时按 Step 1 重分）；
- 簇空：释放该簇全部 UAV，各自按"就近非空簇"重新分配。

**复合失效**：UAV 与目标同时损失时，先处理 UAV 补位，再释放空簇 UAV 并再分配（Fig. 4 决策流程）。

**仿真中的量化结论**（task allocation comparison）：50 UAV / 10 目标，随机损失 10 UAV + 2 目标。对比方法 [20] 无重调度，失效目标的原 UAV 仍飞向"过期目标位置"；本文方法可跨簇重调度，在**分配时间、fencing 成功率（目标被 ≥4 架包围即算成功）、UAV 利用率**上更优，代价是单步计算时间略长。消融（去掉动态调度）显示 60 s/80 s 出现明显簇规模失衡，成功率与利用率下降。

### 1.6 稳定性证明思路（只证 S1，S2 称类似）

定义包围误差与速度误差（质心层面）

$$
\hat p=\frac1n\sum_i p_i-\bar p_t,\qquad
\hat q=\frac1n\sum_i q_i-\bar q_t,
$$

利用无向图对称性，速度一致项与势场项的求和相互抵消，得到解耦的二阶系统

$$
\ddot{\hat p}+c_2\dot{\hat p}+c_1\hat p=0
\;\Longrightarrow\;
\lim_{t\to\infty}\hat p=0,
$$

即**质心收敛到目标群中心**；而质心必在凸包内，故 $D_s\to 0$。再用 Lyapunov 函数

$$
V_c=\frac12\sum_i\Big(c_4\sum_{j\in N_i}a_{ij}\Psi_c(\|p_{ij}\|)
+c_1\tilde p_i^\top\tilde p_i+\tilde q_i^\top\tilde q_i\Big)
$$

证明 $\dot V_c\le 0$、速度共识，并用"碰撞则势能无穷大而 $V_c$ 有界"反证避碰。

> 注意：这是**质心意义**的围合证明——质心进入凸包（其实恒成立）并不自动等价于每个目标在凸包内；S1 靠 $R_{mt}$ 把包撑到覆盖目标群，属于设计保证而非定理直接结论。S2 的多包情形没有单独证明。

---

## 2. [B] FSOC 1954：把"分组"写进优化

### 2.1 与 [A] 的根本区别

[A] 是**两阶段**：先聚类目标、再拍卖分 UAV、最后各子群独立控制。
[B] 明确指出这种"先分配再控制"的两阶段设计像"孤岛"，计算量大且不灵活（Introduction 对 [18][19] 的批评），于是把**分组变量与控制器放进同一个 decentralized 约束优化**里同时求解。

### 2.2 问题设定与假设

- $N$ 机器人（单积分器 $\dot x_i=u_i$），$M$ 个 scattered target（$\dot r_k=v_k$，$v_k$ 连续可微）。
- **Assumption 2**：任意两目标距离 $\|r_k-r_l\|\gg R$（$R$ 为感知半径）——保证各子群独立、互不干扰。
- **Assumption 3**：$M(n+1)\le N$——$n$ 维空间中每个目标至少要 $n+1$ 个机器人才能围出**有内点**的凸包（2D 需 3 个，3D 需 4 个）。
- **Assumption 4**：初始无碰撞、机器人不与目标重合。
- **Assumption 5**：$b\gg c$（分组代价项主导）。

### 2.3 分组（flexible subgrouping）的定义

每个机器人有一个分组向量

$$
\alpha_i=[\alpha_{i,1},\dots,\alpha_{i,M}]^\top\in\{0,1\}^M,
\qquad
\alpha_{i,k}=1 \iff \text{机器人 } i \text{ 负责目标 } k,
$$

子群 $V_k=\{i:\alpha_{i,k}=1\}$，$\alpha=[\alpha_1,\dots,\alpha_N]^\top\in\{0,1\}^{N\times M}$。

**Definition 2（flexible subgrouping）** 只需两条：

1. 每个机器人只负责一个目标：$\alpha\mathbf{1}_M=\mathbf{1}_N$（即 $V_k$ 构成 $V$ 的一个划分）；
2. 每个目标至少 $n+1$ 个机器人：$\mathbf{1}_N^\top\alpha\ge(n+1)\mathbf{1}_M$。

**没有预设**子群数量、子群规模、机器人编号——这就是 "flexible" 的第一层含义。

**Definition 3（flexible-ordering convoying）** 对每个 $k$：

1. $\lim_{t\to\infty}\operatorname{dist}(r_k(t),\operatorname{co}(V_k(t)))=0$（目标进入子群凸包内部）；
2. $\lim_{t\to\infty}(u_i(t)-v_k(t))=0,\ i\in V_k$（与目标速度一致，形成刚性编队平移）；
3. $\|x_i-x_j\|>r,\ i\neq j\in V_k$（子群内避碰）。

**任意空间排序**都可：不预设机器人在凸包上的槽位/顺序，谁在哪个方位由约束自然决定（Fig. 1/Fig. 2 给出同一子群三种不同绕行顺序，形状相同）。这就是 "flexible ordering"。

### 2.4 一体化优化（Eq. 18）

每个机器人 $i$ 分布式求解（$\alpha_i,u_i,\delta^r,\delta^c$）：

$$
\min\;
\underbrace{b\big\|\mathbf{1}_N^\top\alpha-(n+1)\mathbf{1}_M\big\|^2}_{\text{(i) 分组/数量}}
+c\underbrace{\Big(\sum_{k}(\delta^r_{i,k})^2+\sum_{j\in N_i}(\delta^c_{i,j})^2\Big)}_{\text{(ii) 任务/避碰松弛}}
+\underbrace{\sum_{k}\alpha_{i,k}\|u_i-v_k\|^2}_{\text{(iii) 跟踪分配目标}}
$$

约束：

$$
\text{C1:}\quad
\frac{\partial\phi^r_{i,k}}{\partial x_i^\top}(u_i-v_k)
\le -\gamma(\phi^r_{i,k})+\delta^r_{i,k}+(1-\alpha_{i,k})\eta,
$$

$$
\text{C2:}\quad
\frac{\partial\phi^c_{i,j}}{\partial x_i^\top}(u_i-v_k)
\le -\gamma(\phi^c_{i,j})+\delta^c_{i,j},
\qquad j\in N_i,
$$

$$
\text{C3:}\quad
\alpha_i^\top\mathbf{1}=1,\qquad \alpha_{i,k}\in\{0,1\}.
$$

其中目标收敛 CBF 为

$$
\phi^r_{i,k}(x_i)=\|x_i-r_k\|,
$$

避碰 CBF 为

$$
\phi^c_{i,j}(x_i)=\frac{1}{\|x_i-x_j\|-r}-\frac{1}{\rho-r},
\qquad \rho\in(r,R].
$$

**几个关键设计点：**

1. **$\alpha$ 同时出现在代价和约束里**：当 $\alpha_{i,k}=1$ 时 C1 是正常约束，机器人被拉向目标 $k$；当 $\alpha_{i,k}=0$ 时 C1 右侧多出 $\eta\gg\delta^r$ 的惩罚松弛，等于"允许它远离目标 $k$"。所以**一个 $\alpha_i$ 就把"分到哪个组"和"往哪飞"统一了**，不需要外部分配器。
2. **$\|\mathbf{1}_N^\top\alpha-(n+1)\mathbf{1}_M\|^2$** 是软约束：既推动每簇达到 $n+1$，也在 $N>M(n+1)$ 时把多余机器人**均匀摊到各簇**（平方范数下最优是尽量均分）。
3. **C3 硬约束**保证"一个机器人只属于一个子群"，避免重复分配。
4. **CBF 松弛变量** $\delta^r,\delta^c$：为强对偶/KKT 解的存在性服务；代价是传统 forward-invariance 避碰证明失效，论文改用 Lyapunov 有界性反证（Lemma 1 / Appendix A）。
5. 求解是**每个机器人本地**的混合整数约束优化（$\alpha$ 二值），不需要中央分配；但 $M$ 增大时组合维度上升。

### 2.5 收敛性结论

- **Prop. 1**：在 Assumption 3 + 6 下，有限时间 $T$ 后每簇至少 $n+1$ 个机器人。反证：若某簇 $<n+1$，由 $N\ge M(n+1)$ 必存在另一簇 $>n+1$，挪一个机器人过去会严格降低分组代价，与最优性矛盾。
- **Prop. 2**：不会出现无限次子群切换（no infinite switching / chattering）。反证：若在 $T_1$ 从较近目标切到较远目标，$\delta^r$ 与 $\|u_i-v_k\|^2$ 都会变大，与最优性矛盾。**这相当于给"分组稳定性"提供了理论上的迟滞。**
- **Lemma 1–3**：子群内避碰、目标进入凸包、速度一致。
- **Theorem 1**：Assumptions 1–6 下 Definition 2 + Definition 3 全部成立。

### 2.6 仿真体现的"分组行为"

10 机器人、2 个 scattered targets：

- 同一初始构型可收敛到 **5+5** 或 **4+6** 等不同划分，且每个包内绕行顺序不同（Fig. 4(a)(b)）；
- 与固定分组 + 固定排序基准 [17] 相比，轨迹更短，收敛时间 **3.36 s vs 11.48 s**、**4.16 s vs 6.24 s**（Table I）；
- 机器人突然损坏（3 架或 4 架）：剩余机器人**自发**重组为四边形/三角形包，无需重新分配算法；
- 新目标突然出现：2 个包 → **自发出 3 个包**；
- 障碍环境：为省能量自发分成 3 个新子群绕障；
- 3D：5+4 与 4+5 等不同划分与顺序均可，验证可扩展性。

> 注意：[B] 的"重构"是**优化重解的自然结果**，没有 [A] 那样的显式失效规则，也没有"某簇只剩 2 个机器人时怎么办"的兜底逻辑；它靠 Assumption 3（总数够）和重优化保证，一旦某簇跌破 $n+1$，Prop. 1 的结论依赖 Assumption 6（分组代价有限时间后不再变），实际中可能来不及恢复。

---

## 3. 两篇的对照与互补

### 3.1 机制对照

| 维度 | [A] 2896 | [B] FSOC 1954 |
|---|---|---|
| 目标分组依据 | 目标**位置** K-means（触发后用速度方向/间距判定） | 无显式聚类；由 $\alpha$ 与代价隐式决定（本质是"就近 + 数量均衡"） |
| 分组变量 | 外部映射 $f:V\to\{1..N_c\}$ | 决策变量 $\alpha_i\in\{0,1\}^M$ |
| 机器人分配 | 两轮拍卖（最小保障 + 负载均衡贪心） | 与 $u_i$ 同时优化（MIQP 式） |
| 每簇最少数量 | $N_{\min}$（3D 取 4） | $n+1$（由 Assumption 3 保证总数） |
| 是否允许跨簇交互 | S2 只与同簇邻居作用 | C1 对非本簇目标加 $\eta$ 松弛；C2 仍与感知邻居作用 |
| 包的大小 | $R_{des}=\kappa(R_{mt}+R_{red})$，势场平衡 | 由 CBF 可行域 + 目标收敛/避碰拉锯自然形成 |
| 排序 | 无显式排序（势场平衡） | 显式证明任意排序可行 |
| 切换/重构 | 阈值触发 + 显式失效规则 | 每步重解 + Prop. 2 排除无限切换 |
| 计算 | 轻（K-means + 贪心，可实时） | 重（每机器人每步解含二值的优化） |
| 理论 | 分层收敛；控制器主要证 S1 | 端到端收敛 |
| 目标靠近 | 可靠 S1 合并为一个包 | Assumption 2 直接排除（$\gg R$） |

### 3.2 共同点（多目标 fencing 的通用配方）

1. **围合判据统一**：目标进入（子群）凸包内部，$\operatorname{dist}(r_k,\operatorname{co}(V_k))\to0$。
2. **每簇至少 $n+1$ 个**：$n$ 维空间形成有内点凸包的下界（2D=3，3D=4）。
3. **"一个机器人只属于一个子群"** 是硬约束（[A] 的划分、[B] 的 C3）。
4. **不预设队形/排序**：靠势场或 CBF 的"吸引（目标）+ 排斥（邻居）"平衡出包，降低对初始位置和规模的依赖。
5. **目标本身要当障碍排斥**：否则会撞上目标（对应 Chen 2019 §4.1 的目标斥力项），这也是 fencing 与纯包围（encirclement）的差别。

### 3.3 各自的薄弱点（可作研究切口）

**[A]**

- 只证 S1；S2 的"每簇收敛"没有独立定理。
- 无 singleton/共线排除机制。2D 下若某簇恰好 3 架且共线，凸包退化为线段，$D_s$ 判据失效；Chen 2019 的旋转项 $\beta(x_i,x_j,x_o)R x_{ij}/\|x_{ij}\|$ 可以直接补进来。
- 模式切换无迟滞/dwell-time，阈值附近可能反复切换。
- $N_c$ 未给出选取规则；K-means 对初始簇心敏感，且只按位置聚类，速度相近但位置远的目标会被拆开。
- 拍卖是贪心，不保证全局最优；跨簇重分配只看距离，不看剩余能量/通信。

**[B]**

- Assumption 2（目标间距 $\gg R$）很强；目标靠近时多个包会相互干扰甚至合并。
- 每步混合整数优化，$M$ 大时实时性存疑；$b\gg c$ 的取值敏感。
- 只有"至少 $n+1$"的下界，没有"凸包非退化"的保证；恰好 $n+1$ 时为单纯形，数值上容易退化。
- 失效/新目标依赖重优化，无显式兜底与事件日志；没有 [A] 那种"从冗余簇调人"的定向修复。
- 柔性排序虽好，但**没有碰撞以外的几何质量指标**（包是否均匀、覆盖角是否退化）。

---

## 4. 第三种范式：[C] 3116（作为对照）

[C] 不分组：把**一群不合作目标整体**（GUT）当成一个虚拟目标，分布式观测其"fencing center"（GUT 凸包内一点）和**动态 fencing radius**，再用分布式 balanced fencing 控制器让所有车辆收敛到该半径上并**等角分布**。

- 优点：无需目标聚类/UAV 分配；plug-and-play，车辆/目标可随时加入退出。
- 代价：只能形成**一个包**，适合目标相对聚集的场景；目标散开时包会过大、效率低。
- 与 [A] 的 S1 思路相近，但 [A] 用势场、[C] 用"中心 + 半径 + 等角"显式几何。

这提示一个自然的统一框架：**先用"目标群紧致度"在 [C]/S1（单包）与 [B]/S2（多包）之间选择，再用 [A] 的拍卖或 [B] 的 $\alpha$ 优化完成分配。**

---

## 5. 如果要在我们的 fencing 工作里落地

### 5.1 建议的分组决策流程

```
每个控制周期：
1. 估计各目标状态（位置/速度）        # 分布式观测器
2. 紧致度检验：
   θ_max < θ_th 且 ε_d < ε_max ?
   ├─ 是 → 单包模式：参考点 = 目标群几何中心，R_des = κ(R_mt + R_red)
   └─ 否 → 多包模式：
        a. 目标分簇（位置 K-means / 速度方向聚类 / DBSCAN）
        b. 每簇至少 n+1 个（3D: 4），留 1 个冗余更好
        c. 分配：最小保障（最近 N_min）+ 负载均衡（ω1|L| + ω2·dist）
        d. 每个子群独立跑 fencing 控制器（只与同簇邻居交互）
3. 事件触发重构（UAV 失效 / 目标丢失 / 新目标 / 簇合并分裂）
```

### 5.2 需要补的三件事

1. **凸包非退化**：在每簇 $n+1$ 的基础上加"非共线/非共面"检验，或直接引入 Chen 2019 的角度斥力项；否则 $D_s$ 判据在退化构型下没有意义。
2. **切换迟滞**：给 S1↔S2 加 dwell-time 或能量型滞回（$\theta_{th}^{\uparrow}>\theta_{th}^{\downarrow}$），避免阈值抖动；[B] 的 Prop. 2 是可借鉴的理论工具。
3. **簇间干涉**：当两簇距离小于 $2R_{des}$ 时，两个包会重叠。处理方式：
   - 合并为一个包（退回 S1）；
   - 或在分配代价里加簇间排斥项；
   - 或要求 $\|r_k-r_l\|>2R_{des}$（类似 [B] 的 Assumption 2，但用包尺寸而非感知半径表述更合理）。

### 5.3 与 `chen2019/` 笔记的衔接

- `chen2019/post.md` 的 (P1)(P2)(P3) 是**单目标**版本；多目标时 (P1) 要逐目标成立（$\forall k:\ \operatorname{dist}(r_k,\operatorname{co}(V_k))\to0$），(P3) 要逐子群成立。
- 单目标的"质心收敛 ⇒ 目标入包"论证在多包情形**不能直接相加**：每个子群的质心只收敛到自己的目标，跨子群没有抵消项。因此 [A] 只证 S1、[B] 用 CBF 逐子群证，是两种合理选择。
- singleton 问题在多目标下更严重：子群规模越小（尤其恰好 $n+1$），退化概率越高；建议把"最小簇规模"设为 $n+2$ 或加退化检测。

---

## 6. 关键公式速查

**[A] 2896**

- 凸包/距离：$\mathcal{C}(p)=\{\sum_i\Delta_i p_i:\Delta_i\ge0,\sum\Delta_i=1\}$，$D_s=\min_{r\in\mathcal{C}}\|p_t-r\|$
- S1 导航：$u_i^p=\bar a_{t,i}-c_1(p_i-\bar p_{t,i})-c_2(q_i-\bar q_{t,i})$
- S2 导航：$u_i^p=\hat a_{i,k}-c_1(p_i-\hat p_{i,k})-c_2(q_i-\hat q_{i,k})$
- 成包距离：$R_{des}=\kappa(R_{mt}+R_{red})$，S2 时 $R_{mt}=0$
- 切换：$\theta_{\max}\ge\theta_{th}$ 或 $\varepsilon_d\ge\varepsilon_{\max}$
- K-means：$\min\sum_l\sum_{p\in C_l}\|p-\mu_l\|^2$
- 拍卖 R1：$i_k^l=\arg\min_{i\in V_{ava}}\|p_i-\mu_l\|$（每簇 $N_{\min}$ 个）
- 拍卖 R2：$\arg\min_l(\omega_{1,l}|L_l|+\omega_{2,l}\|p_i-\mu_l\|)$
- 补位：$j^*=\arg\min_{j\in L_c,|L_c|>N_{\min}}\|p_j-\mu_l\|$

**[B] 1954**

- 分组：$\alpha_i\in\{0,1\}^M$，$\alpha\mathbf{1}_M=\mathbf{1}_N$，$\mathbf{1}_N^\top\alpha\ge(n+1)\mathbf{1}_M$
- 目标 CBF：$\phi^r_{i,k}=\|x_i-r_k\|$，约束含 $(1-\alpha_{i,k})\eta$
- 避碰 CBF：$\phi^c_{i,j}=\frac{1}{\|x_i-x_j\|-r}-\frac{1}{\rho-r}$
- 代价：$b\|\mathbf{1}_N^\top\alpha-(n+1)\mathbf{1}_M\|^2+c\sum(\delta^r)^2+c\sum(\delta^c)^2+\sum_k\alpha_{i,k}\|u_i-v_k\|^2$
- 保证：每簇 $\ge n+1$、无无限切换、目标入包、速度一致、子群内避碰
