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

> OKB 中其余可用的分组/分簇编队文献见 **§7**（按机制分了五类，并给出与 [A]/[B] 的关系）。

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

---

## 7. OKB 中其他"分组 / 分簇编队"文献（补充调研）

先给结论：**OKB 里没有第二篇与 [A]/[B] 完全同题的工作**，但有若干篇从不同侧面处理了"分组编队"：

- 最接近的是 **7299**（cluster formation containment：按目标位置 + 优先级把 leader 分成互不重叠的簇，follower 再填入各簇凸包）和 **663**（检测到目标分离后触发子群重组）；
- 其余可分成"分组编队控制"、"分组触发与任务分配"、"分组的图论/聚类根据"、"编队拆分/合并"、"多目标围合但不显式分组"五类。

### 7.1 总览表

| 类别 | ID | 年 / 期刊 | 分组机制（一句话） | 与 fencing 的关系 / 可借鉴点 |
|---|---|---|---|---|
| **分组编队** | 7299 | 2021, IEEE T-RO | leader 按**目标位置 + 优先级**用博弈规则分成互不重叠簇；各簇在对应目标周围成编队；follower 收敛到各簇 leader 张成的凸包 | 与 fencing 最接近：leader 围目标、follower 填凸包；分组同时用了几何与优先级 |
| 分组编队 | 3397 | 2026, Automatica | **双层**：簇内成指定形状，簇质心之间再成指定几何图案；多簇聚合博弈 + NE 寻求 | 可约束"多个包围圈之间"的相对布局，避免包与包重叠 |
| 分组编队 | 3078 | 2026, Automatica | 分层 cluster formation：估计层 + 局部最优控制层；预定义时间；混合零和博弈；含欺骗攻击 | 分组编队的抗攻击/预定义时间版本 |
| 分组编队 | 2835 | 2023, IEEE T-IV | 多组时变编队跟踪（MGFTC）+ 预定义时间 + 估计器分层（NMSV） | 多组 + 时变队形 + 预设收敛时间 |
| 分组编队 | 6228 | 2023, IEEE TCAS-II | 事件触发时变**分组编队**（TVGF），有向图、自适应、省通信 | 分组编队的通信效率版本 |
| 分组编队 | 2885 | 2025, IEEE TIE | 组间博弈（组领导者收敛到 NE）+ 组内 follower 相对组领导者做 TVFT；UAV+UGV 实验 | 组间竞争 + 组内合作，适合对抗/多任务 |
| 分组编队 | 1135 | 2022, IEEE TAES | clustered quadrotors 输出反馈编队跟踪（簇结构给定） | 多簇工程实现参考；不含分组算法 |
| **触发/重构** | 663 | 2026, Aerospace Sci. Tech. | 持续监测目标 cohesion；目标分离 → 资源重分配 + 目标-子群分配；时间戳更新 + 时延补偿预测 | 与 [A] 的 S1→S2 触发几乎同构，但跨域 UAV-USV、含通信约束 |
| 触发/重构 | 1015 | 2026, IEEE TASE | 按**通信连通性**把团队分成子群；连通子群做部分集中优化，失联机器人独立优化 | 与 [A] 的失效规则互补：按连通性而非任务分组 |
| 触发/重构 | 4266 | 2022, IEEE TAC | agent 动态合并成 coalition/cluster；簇内协调、簇外当不确定性；tube-based MPC | "分组本身是决策变量"的严格 MPC 版本 |
| 触发/重构 | 523 | 2012, IEEE TASE | 资源约束下的 coalition formation：在线 sequential + 离线 holistic | 分组 = 资源约束下的组队，可替换 [A] 的拍卖 |
| 触发/重构 | 1181 | 2019, Automatica | 竞争式动态任务分配，只有赢家被激活追目标；含目标更快时的围捕策略 | 与拍卖/竞争分配直接可比 |
| 触发/重构 | 7411 | 2025, Robotics and Autonomous Systems | 多目标 k-WTA 分配 + obstacle-aware particle model | 连续时间竞争分配，适合大规模 |
| **分组机理** | 5693 | 2022, Automatica | 簇数 = 通信图 **almost equitable partition** 的 cell 数；同 cell 受同一 mean-field 影响 | 分组由**通信拓扑**决定，可作为分组可辨识性依据 |
| 分组机理 | 5998 | 2022, IEEE TAC | 一般划分下有向图 cluster consensus 的充要条件（nonvicious circle condition） | 分组后组内一致性是否可达的判据 |
| 分组机理 | 6510 | 2017, IEEE TAC | 交互簇的 H∞ group consensus，给出簇间耦合结构/强度条件 | 组间干扰抑制 |
| 分组机理 | 4817 | 2011, IEEE TAC | agent 自行分成固定数量子群，并对每组的 "gain" 达成一致；量化感知/运动/通信受限对收敛速率的影响 | 自组织分组的收敛速率分析 |
| 分组机理 | 7278 | 2016, IEEE T-RO | 层次聚类识别不同分辨率下空间凝聚且分离的 robot groups | 用层次聚类动态定组，替代固定簇数的 K-means |
| **拆分/合并** | 513 | 2026, Automatica | 三角形**角刚性**编队的 merge/split：内角约束，两个子编队可合并/拆分以通过障碍 | 编队级"分组/合并"的几何操作，适合窄通道 |
| **单包基线** | 852 | 2025, Unmanned Systems | 多移动目标 constrained fencing：观测目标中心，equal fencing（等角分布）围绕多目标凸包；输出反馈 + 自适应神经动力学优化 | 单包 + 等角，和 [C] 3116 同类；可作 [A] S1 的对比 |
| 单包基线 | 3113 | 2026, IEEE RA-L | 与目标群凸包保持 social safety distance，同时保持平衡几何分布；unicycle 实物 | 单包 + 社会约束 |
| 单包基线 | 284 | 2025, 中国舰船研究 | USV 编队预定义时间多目标协同包围：位置层跟踪目标位置凸组合 + 速度层滑模；RBFNN 干扰估计 | 单包 + 编队 + 预定义时间（中文） |
| 不显式分组 | 2688 | 2020, ACC | ring-coupled unicycles：每辆车只与一个邻居交互，各自绕自己的目标指定半径，形成指定分布 | 隐式"一机一目标"，拓扑极轻（严格说不是单包） |
| 不显式分组 | 2318 | 2023, IEEE/CAA JAS | signed graph 结构平衡产生 **bipartite clustering**，agent 从两侧包围目标 | "聚类"来自符号图，不是任务分组 |
| 不显式分组 | 2605 | 2021, Automatica | 单 agent 绕多目标凸包的安全 convex-circumnavigation（bearing-only） | 单 agent 几何基线，凸包围合的几何基础 |
| 不显式分组 | 7294 | 2026, IEEE T-RO | 分布式 feedback optimization 做"监控 + 包围 + 巡逻"，触发通信，ROS 2 实物 | 包围与巡逻的多目标优化，工程化程度高 |
| 综述/专著 | 1976 | 2025, Springer | *Multi-Robot Systems: Coordinated Fencing Control and Applications* | fencing 背景与章节索引（含邻角排斥、ISS 等） |
| 中文/学位 | 1897 | 2022, 华中科技大学博士 | USV 集群协同合围：相角排斥、ISS、输出调节；多目标/方位约束 | 单包/多目标合围的系统性中文工作 |
| 中文/学位 | 626 | 2023, 江苏科技大学 | 多四旋翼分布式合围控制 | 中文硕士工作，可作背景 |
| 中文/学位 | 224 | 2026, 自动化学报 | 社会化协同无人机集群目标合围：层级交互、拓扑切换/丢包、一致性目标观测器 | 受限通信下的合围 |

### 7.2 逐类说明

#### (1) 分组 / 分簇编队控制

**[7299] 2021 T-RO — A Decentralized Cluster Formation Containment Framework for Multirobot Systems**

这是除 [A]/[B] 外最值得精读的一篇。它的两层结构与 fencing 的"围 + 填"天然对应：

1. **分组**：leader 机器人根据**目标的位置和优先级**，用博弈规则分配到一组**互不重叠**的簇（每个簇对应一个目标）；
2. **组内编队**：各簇 leader 在对应目标周围形成期望编队；
3. **组间 containment**：follower 收敛到各簇 leader 张成的凸包内。

可借鉴之处：

- 分组依据同时含**几何（目标位置）和优先级**，比 [A] 的纯 K-means 多了一个"目标价值/威胁度"维度；
- leader/follower 分层：leader 负责围合形状，follower 负责填充凸包（增加包围密度/冗余），这正好对应 fencing 中"包的大小"与"包内密度"两个可分开设计的量；
- 有实物实验（微型移动机器人），工程可信度较高。

局限：簇数/分组规则是博弈规则给定，未处理目标分裂/合并、机器人失效后的在线重构；也未讨论凸包退化。

**[3397] 2026 Automatica — Distributed optimal double-layer formation control for multi-cluster systems**

把"分组编队"写成**双层优化**：

- 内层：每个 cluster 内 agent 形成期望形状；
- 外层：各 cluster 的**质心**再按指定几何图案排列；
- 用多簇聚合博弈（multi-cluster aggregative game）+ 分布式 NE 寻求求解；
- 仿真示例就是**多目标包围**。

对多目标 fencing 的启发：多个包围圈不应各自为政，**圈与圈之间的相对位置**（质心图案）可以显式约束，从而避免两个包重叠或互相挤压。这正是 [A]/[B] 都没有处理的"簇间几何"。

**[3078] 2026 Automatica — Hierarchical distributed optimal cluster formation control**

- 估计层：每个 follower 预定义时间重构组 leader 的位置；
- 局部最优控制层：混合零和博弈（协作 + 欺骗攻击）。

价值：给出了 cluster formation 在**预定义时间 + 对抗/攻击**下的分层设计模板。如果我们的 fencing 需要考虑通信攻击或收敛时间保证，这是可直接套用的结构。

**[2835] 2023 T-IV — Prescribed-Time Time-Varying Multi-Group Formation Tracking (NMSV)**

- 多组（multi-group）+ 时变编队 + 预定义时间；
- 估计器层 + 局部控制层的分层结构；
- 组内按相对组 leader 的时变偏移定义队形。

价值："多组"的组内队形可以**随时间变化**，对应 fencing 中包围圈随目标尺寸/速度变化而伸缩的需求。

**[6228] 2023 TCAS-II — Distributed Adaptive Event-Triggered TVGF**

- 时变分组编队（time-varying grouping formation）；
- 事件触发 + 自适应，去掉对全局网络信息的依赖。

价值：如果分组/编队更新频率高，通信是瓶颈；这篇给出了"只在必要时通信"的分组编队版本。

**[2885] 2025 TIE — Adaptive Group Time-Varying Formation Tracking via Networked Game**

- 组间：group leader 收敛到 intergroup game 的 Nash 均衡；
- 组内：follower 相对 group leader 做 TVFT；
- UAV + UGV 实验验证。

价值：把"组间竞争"显式建模。若多目标 fencing 中存在目标优先级/威胁度差异，可以用组间博弈分配"谁来围哪个目标"，而不是只用距离/负载。

**[1135] 2022 TAES — Clustered Quadrotors Formation Tracking**

- 多簇四旋翼在无速度/角速度测量下跟踪参考轨迹并保持队形；
- 簇结构是**给定**的，不研究如何分组。

价值：作为"分组确定后，单组工程实现"的参考（输出反馈、饱和、级联控制）。

#### (2) 分组触发与动态重构（多目标任务）

**[663] 2026 Aerospace Science and Technology — Cross-domain UAV-USV multi-target tracking**

与 [A] 的相似度最高：

- 持续监测 target cohesion（目标聚集程度）；
- 目标**分离**时触发动态响应：**(1) 资源重分配 + (2) 目标-子群分配**；
- 每个子群跟踪一个特定目标；
- 时间戳更新协议 + 时延补偿预测，处理受限通信下的过期信息；
- 还包括 UAV 返航、USV 接力、待机 UAV 接管等全任务周期管理。

与 [A] 的差异：跨域异构（UAV 感知 + USV 续航）、强调通信时延与能量；分组触发用 cohesion 而不是 [A] 的"速度方向角 + 间距"双阈值。可以把它作为 [A] 的"通信受限版本"对照。

**[1015] 2026 TASE — Failure-Aware Multi-Robot Coordination**

- 把 failure 分成**临时**（感知/通信间歇丢失）与**永久**（不可逆失联）；
- 永久失联时按**通信连通性**把团队分成子群：连通子群做部分集中优化，失联机器人独立优化；
- 目标仍是 resilient target tracking。

价值：与 [A] 的"按任务/数量"分组互补——当通信被打断时，**分组的第一约束是连通性而不是目标数**。多目标 fencing 若在通信受限环境，需要同时考虑这两种分组依据。

**[4266] 2022 TAC — Coalitional Clustering MPC**

- agent 动态 merge 成 coalition/cluster；
- 簇内协调输入最大化集体性能，簇外耦合当不确定性；
- tube-based MPC 保证可行性/稳定性，即使 clustering 变化。

价值：这是"分组数量与成员**本身就是决策变量**"的严格版本，和 [B] 的 $\alpha$ 优化思想相近，但用 MPC/tube 而非 CBF；可作为 [B] 在"约束/鲁棒性"方向的对照。

**[523] 2012 TASE — Coalition-Based Task Allocation with Resource Constraints**

- 机器人因资源限制必须组队（coalition）；
- 在线：sequential coalition；离线：holistic 全局优化。

价值：如果 UAV 有能量/载荷约束，"每簇至少 $n+1$ 架"不够，还要满足资源可行性；coalition 形成是比拍卖更一般的分配框架。

**[1181] 2019 Automatica — Dynamic task allocation for moving target tracking**

- 竞争式分配：只有 competition 的 winner 被激活去追目标；
- 分布式协调模型 + 稳定性证明；
- 目标速度高于追踪者时的"besieging"策略。

价值：与 [A] 的拍卖、[B] 的 $\alpha$ 并列的第三种分配机制（winner-take-all 式），适合"目标数 > 可用机器人"的稀疏场景。

**[7411] 2025 RAS — k-WTA multi-target pursuit and encirclement**

- 多目标 k-winners-take-all（MK-WTA）做任务分配；
- obstacle-aware particle model 做避障；
- 仿真 + 差速轮实物。

价值：k-WTA 是连续时间、可并行实现的竞争分配，规模大时比拍卖更合适；但它是" pursuit/encirclement"而非严格的 convex hull fencing。

#### (3) 分组形成的图论 / 聚类根据

**[5693] 2022 Automatica — Cluster partitioning of heterogeneous multi-agent systems**

- 证明 agent 的动态会"聚类"成与通信图 **almost equitable partition** 的 cell 数相同的子群；
- 同一 cell 内的 agent 受同一个 mean-field 动态影响。

价值：给出了"分组不是任意选的"这一结构性结论。如果我们的分组与通信拓扑不一致，组内一致性可能无法实现；反之，可以**利用**拓扑的 equitable partition 来设计分组。

**[5998] 2022 TAC — Cluster Consensus under General Partition**

- 研究一般划分（含 cyclic partition）下 cluster consensus 的可实现性；
- 提出 **nonvicious circle condition**，并证明是充要条件。

价值：分组后组内共识的可达性判据；可用于检查"K-means/拍卖得到的分组"是否可被底层通信图支撑。

**[6510] 2017 TAC — Robust H∞ Group Consensus for Interacting Clusters**

- 单积分器 + 模型不确定性 + 外部干扰；
- 给出组间耦合结构与强度的充分条件，保证 H∞ group consensus。

价值：组间干扰抑制；多目标 fencing 中不同包围圈之间的相互影响可以建模为"interacting clusters"。

**[4817] 2011 TAC — Local Agent Requirements for Stable Emergent Group Distributions**

- agent 自行分成**固定数量**子群，并对每组的 "gain" 达成一致；
- 量化感知、运动、通信能力受限对分布收敛速率的影响；
- 应用于协同监视。

价值：自组织分组的收敛速率分析；如果分组是涌现的（而非集中分配的），这篇给了"局部能力 → 全局分布"的定量关系。

**[7278] 2016 T-RO — Coordinated Robot Navigation via Hierarchical Clustering**

- 用层次聚类在不同分辨率上识别"空间凝聚且分离"的 robot groups；
- 把连续构型空间映射到树，做反应式导航。

价值：**替代固定簇数的 K-means**。多目标 fencing 中目标数/簇数会变，层次聚类可以自动给出不同分辨率的组结构，并支持"组内/组间"两种导航模式。

#### (4) 编队拆分 / 合并

**[513] 2026 Automatica — Merging and splitting triangularly angle rigid formations**

- 用**内角约束**描述队形（比距离/方位刚性多自由度）；
- 研究两个角刚性子编队如何**合并**、一个编队如何**拆分**以通过复杂障碍；
- 给出分布式 vertex sequential reduction 算法与稳定性控制律。

价值：多目标 fencing 中"一个包分裂成两个"或"两个包合并"的几何操作，可以直接借用它的角刚性拆分/合并条件；比 [A] 的阈值切换更有几何保证。

#### (5) 多目标围合但不显式分组（单包 / 一机一目标 / 单 agent 几何基线）

这一类的共同点：**没有把机器人划分成若干子群**。多数把所有目标视为一个凸包/一个虚拟目标；个别是一机一目标或单 agent。

- **[852] 2025 Unmanned Systems**：多移动目标的 constrained fencing；分布式观测目标中心，**equal fencing**（等角分布）围绕多目标凸包；输出反馈 + 自适应神经动力学优化；ISS 稳定性。
- **[3113] 2026 RA-L**：与目标群凸包保持 **social safety distance**，同时保持平衡几何分布；unicycle 实物验证。
- **[284] 2025 中国舰船研究**：USV 编队预定义时间多目标协同包围；位置层跟踪目标位置的**凸组合**，速度层滑模；RBFNN 干扰估计。
- **[2688] 2020 ACC**：ring-coupled unicycles；每辆车只与一个邻居交互，各自绕自己的目标指定半径，形成指定分布。**隐式"一机一目标"**，但没有显式分组/凸包包含判据（放在本节是因为它同样无子群划分，并非单包）。
- **[2318] 2023 JAS**：signed graph 的结构平衡产生 **bipartite clustering**，agent 从两侧包围动态目标。"聚类"来自符号图，不是任务分配。
- **[2605] 2021 Automatica**：单 agent 绕多目标凸包的**安全 convex-circumnavigation**（bearing-only），不碰撞凸包。这是凸包围合最干净的几何基础。
- **[7294] 2026 T-RO**：分布式 feedback optimization 做"监控 + 包围 + 巡逻"，异步触发通信，ROS 2 实物。工程化程度最高。
- **[3116] 2026**（见 §0）：GUT 的 fencing center + 动态半径 + 等角分布，plug-and-play。

### 7.3 分组机制的四种类型学

把上面的文献按"分组是怎么来的"归类，便于写 related work 时定位：

| 类型 | 代表 | 分组依据 | 优点 | 缺点 |
|---|---|---|---|---|
| **任务驱动** | [A] 2896, [B] 1954, 7299, 663, 1181, 7411 | 目标位置/速度/优先级/竞争 | 直接服务任务，簇数可随目标变化 | 需要目标状态估计；分配可能抖动 |
| **图结构驱动** | 5693, 5998, 6510, 1015 | 通信图的 equitable partition / 连通性 | 组内一致性有理论保证 | 分组受拓扑限制，可能不符合任务需求 |
| **几何/聚类驱动** | 7278, 513, [A] 的 K-means | 空间凝聚性、角刚性、内角约束 | 几何解释清晰，支持拆分/合并 | 对目标运动方向不敏感 |
| **决策/博弈驱动** | 4266, 523, 3397, 3078, 2885 | coalition/博弈均衡 | 可显式建模资源、竞争、攻击 | 计算重，需要迭代求解 |
| **隐式/涌现** | 4817, 2318, 2688 | 局部规则/符号图/环耦合 | 分布式、轻量 | 无显式分组变量，难保证任务指标 |

> 注：[B] 的 $\alpha$ 优化同时属于"任务驱动"和"决策驱动"——它是把分组写成**决策变量**而非外部规则，这是它和 [A] 的本质区别。

### 7.4 对 related work 写作的建议

可以把"多目标 fencing"的文献综述拆成三条线，避免把分组编队和多目标围合混在一起：

1. **分组/分簇编队控制**：7299, 3397, 3078, 2835, 6228, 2885, 1135
   - 共同假设：簇结构/组数通常给定，或由通信图决定；
   - 少有针对"目标分裂/合并"的在线分组。
2. **多目标任务分配与重构**：663, 1181, 7411, 523, 4266, 1015
   - 共同目标：决定"谁去哪个目标"；
   - 与 fencing 的差别：多数只要求"到达/跟踪"，不要求形成**凸包包含**。
3. **多目标围合 / fencing**：852, 3113, 284, 2688, 2318, 2605, 3116, 7294
   - 多数把所有目标当成**一个凸包/一个虚拟目标**；
   - 只有 [A]/[B] 同时处理"目标分组 + 机器人分配 + 凸包包含"。

可以直接使用的 gap 表述：

> 现有分组编队工作 [7299, 3397, 3078] 多假定簇结构给定或由通信拓扑决定；多目标围合工作 [852, 3113, 284] 多将所有目标视为一个凸包；仅有 [A, B]（以及双层结构的 7299）同时处理目标分组与机器人分配。但 [A] 缺乏凸包退化与切换迟滞的理论保证，[B] 要求目标间距远大于感知半径且每步求解含二值变量的优化，7299 则未处理目标分裂/合并与机器人失效后的在线重构。

### 7.5 OKB 数据质量提示

- **3615**（ICRA 2022, Multi-Target Encirclement with Collision Avoidance via DRL）在 OKB 中的摘要/正文抽取明显错位（内容变成了分布式学习），引用前必须核对原文；本笔记未采用其内容。
- **6223**（Specified-Time Formation-Containment）的 TOC 与实际标题不一致（TOC 显示 affine formation maneuver），可能是元数据错配，暂不引用。
- **7299 / 6228 / 4817 / 5693 / 4266** 在 OKB 中有 Markdown 链接，但 `read_markdown` 返回空内容，目前只能使用摘要与元数据；引用具体公式/定理前需查原文。
- **1976** 是专著（Cognitive Intelligence and Robotics 系列），TOC 包含多章 fencing 内容，可作为背景引用而非单篇论文。
