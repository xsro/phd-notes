# 关键技术：Surplus（剩余变量）简介

> 数据来源（okb 服务器 `mcp__okb`）：
>
> - `6235`（[18] TAC 2025）、`6219`（[17] TAC 2025）—— 已读全文 markdown（Introduction / Algorithm Description / 参考文献表）；
> - `6180`（[12] TCSII 2022）—— markdown 尚未生成，仅取摘要；
> - `7269`（Cai & Ishii, Automatica 2012）—— 关键词与摘要核对（原文关键词含 *Surplus-based averaging*）；
> - `3795`（nonsmooth surplus, 2025）—— 仅取摘要。
>
> 相关文献的 okb 索引见文末第 8 节；调取方式：`mcp__okb__read_markdown(id=...)` / `mcp__okb__get_document_abstract(id=...)`。

## 1. 为什么要 surplus：非平衡有向图下的“平均漂移”

对无向图或权重平衡图，Laplacian 既有零行和又有零列和（$L\mathbf{1}=\mathbf{0}$ 且 $\mathbf{1}^{\top}L=\mathbf{0}^{\top}$），PI 型 DAC 算法里积分项满足

$$\sum_{i}y^{i}(t)\equiv\sum_{i}y^{i}(0)$$

因此能证明精确 DAC。

但现实中的单向量信息流（相机视野有限、发射功率不同、链路失效等）导致 **权重非平衡有向图**：此时 $\mathbf{1}^{\top}L=\mathbf{0}^{\top}$ 不再成立，$\sum_i y^i(t)$ 不再守恒，最终状态会收敛到

$$x^{i}(t)\to\frac{1}{\alpha N}\sum_{j}y^{j}(\infty)$$

而非期望的 $\dfrac{1}{\alpha N}\sum_{j}y^{j}(0)$，即出现 **average shift（平均漂移）**——所有一致性值整体偏离真实平均值 $\dfrac{1}{N}\sum_{j}u^{j}(t)$。

## 2. surplus 是什么：一个“本地记账变量”

**Surplus 就是每个智能体额外持有的一个一维辅助变量 $s^{i}(t)$，它在本地“记录”自己状态/积分变量的变化，使得某个总量守恒，从而把非平衡图造成的漂移抵消掉。** 直观上，它把非平衡图“隐含丢失的那部分质量”显式记在账上，而不是靠求左特征向量或反复平衡权重。

- 守恒性 $\sum_{i}\big(s^{i}+y^{i}\big)\equiv\text{const}$：核心是利用 $L'$ 的零列和，设计

$$\dot{s}^{i}=-\gamma\Big(\sum_{h\in\mathcal{N}_{i}^{+}}b_{ih}\,s^{i}-\sum_{j\in\mathcal{N}_{i}^{-}}b_{ji}\,s^{j}\Big)-\dot{y}^{i}$$

写紧凑形式为 $\dot{\boldsymbol{s}}=-\gamma L'\boldsymbol{s}-\dot{\boldsymbol{y}}$。

  左乘 $\mathbf{1}^{\top}$ 并用 $\mathbf{1}^{\top}L'=\mathbf{0}^{\top}$，得 $\sum_{i}\big(\dot{s}^{i}+\dot{y}^{i}\big)=0$，故总量守恒。

- 算法上：把 $\theta\,s^{i}(t)$（连续）/ $\nu\psi\,s^{i}(t)$（离散）作为扰动项注入积分环节 $\dot{y}^{i}$ / $z^{i}(t_{k+1})$，用 surplus 把漂移“还”回去；$s^{i}$ 与状态一起广播给邻居（通信量仅多 1 维）。

## 3. 来源与血缘：从 Cai & Ishii 到 push-sum

### 3.1 文献出处

**传统 surplus 的公认起点**（[18] 6235 参考文献 [16]、[17] 6219 参考文献 [20] 均指向同一篇；**该文现已入库，`okb id = 7269`**）：

- K. Cai, H. Ishii. **Average consensus on general strongly connected digraphs**[J]. Automatica, 2012, 48(11): 2750–2761. DOI: 10.1016/j.automatica.2012.08.003（`7269`）
- 时变拓扑推广：K. Cai, H. Ishii. Average consensus on arbitrary strongly connected digraphs with time-varying topologies[J]. IEEE TAC, 2014, 59(4): 1066–1071. DOI: 10.1109/TAC.2014.2305952（`7267`）

验证依据：`7269` 全文关键词即为 *Multi-agent consensus; **Surplus-based averaging**; Gossip algorithm; Directed graph; Eigenvalue perturbation* —— “surplus”这一术语与方法名正是由该文确立。

**术语定型（已被写进标题/方法名）**：

- G. Chen, Z. Li. Distributed optimal resource allocation over strongly connected digraphs: **a surplus-based approach**[J]. Automatica, 2021, 125: 109459.（`7268`）
- C. Shi, G. Yang. Distributed optimization under unbalanced digraphs with node errors: robustness of **surplus-based dual averaging** algorithm[J]. IEEE TCNS, 2021, 8(1): 331–341.（`7265`）
- D. Wang 等. **Surplus-based** accelerated algorithms for distributed optimization over directed networks[J]. Automatica, 2022, 146: 110569.（未入库）
- Y. Xu, T. Han, K. Cai 等. A distributed algorithm for resource allocation over dynamic digraphs[J]. IEEE TSP, 2017, 65(10): 2600–2612.（`7266`）

**词源**：`surplus` 原为会计/经济学术语“结余、盈余”（收入 − 支出之差）。它在这里记录的是“**实际状态（非平衡图）− 理想状态（平衡图）**”这个差额，本系列论文称之为 **average shift**。中文常译“剩余变量 / 盈余变量”。因此同属非平衡图四类方法的命名逻辑很整齐：左特征向量法（数学动机）、权重平衡法（图论动机）、push-sum（算法动机）、**surplus（记账隐喻）**。

### 3.2 surplus 与 push-sum 的关系（核心）

**一句话：两者是同一个“本地守恒量”思想的两种写法——push-sum 用「除法归一化」消掉不平衡带来的 *缩放*，surplus 用「减法记账」消掉不平衡带来的 *漂移*。**

**push-sum / ratio consensus 的写法**（离散）：每节点维护一对量 $\big(y^{i},w^{i}\big)$，二者用同一套权重传播

$$y^{i}(k+1)=\sum_{j\in\mathcal{N}_{i}^{-}}a_{ij}\,y^{j}(k),\qquad w^{i}(k+1)=\sum_{j\in\mathcal{N}_{i}^{-}}a_{ij}\,w^{j}(k)$$

节点对外输出（比值）为 $y^{i}(k)/w^{i}(k)$。

由于左乘 $\mathbf{1}^{\top}$ 得 $\sum_i y^i$ 与 $\sum_i w^i$ 各自守恒，而 $w^{i}(k)$ 收敛到左 Perron 向量的分量，相除后就把非平衡图造成的整体缩放抵消掉了。**关键机制：分子/分母成对传播 + 相除恢复比例**（权重常用 $a_{ij}=1/d_{j}^{\text{out}}$，故需出度信息；6235 把 Domínguez-García & Hadjicostis, IEEE TAC 2013 的“矩阵缩放”工作列为该路线代表）。

**surplus 的写法**：不加分母，而是给每节点一个“余额” $s^{i}$ 吸收积分项的增量

$$\dot{\boldsymbol{s}}=-\gamma L'\boldsymbol{s}-\dot{\boldsymbol{y}}\quad\Longrightarrow\quad \sum_{i}\big(s^{i}+y^{i}\big)\equiv\text{const}$$

只要保证“状态 + 余额”的**加性总和守恒**，漂移就被记在 $s^{i}$ 里、再由 PI 反馈还回去。**关键机制：局部记账 + 线性补偿**。

| 对比维度 | push-sum / ratio consensus | surplus |
| --- | --- | --- |
| 守恒量 | $\sum_i y^i$ 与 $\sum_i w^i$ 各自守恒 | $\sum_i(s^i+y^i)$ 守恒 |
| 消除不平衡的机制 | 相除 $y^{i}/w^{i}$，归一化掉缩放 | 相减记账，补偿掉漂移 |
| 额外状态维数 | 多 1 维分母 $w^{i}$ | 多 1 维 $s^{i}$ |
| 额外信息/初值要求 | 权重（如出度）；需成对维护两路量 | $s^{i}(0)=0$、$\sum_i y^i(0)=0$ |
| 与 PI / DAC 的兼容性 | 除法非线性，难以嵌入连续 PI 动力学 | 线性注入 $\theta s^{i}$，可直接嵌入 PI 与指定时间框架 |
| 典型结论 | 渐近/几何收敛到精确平均 | 有界可调稳态误差，特例下精确 |
| 代数本质 | 构造零列和/双随机矩阵的“比值版” | 构造零列和矩阵 $L'$（$\mathbf{1}^{\top}L'=\mathbf{0}^{\top}$）的“加性版” |

**为什么 DAC 中要升级成 integral surplus**：push-sum 的“分母”与静态 surplus 的“静态记账”都只能抵消 *不随时间变* 的偏移；而 PI 动力学 + 时变参考信号造成的 average shift 是随时间变化的（论文原话：*the traditional surplus is invalid for the DAC algorithm with PI dynamics*），故必须把记账对象改成积分变量的 **增量**（$\dot{s}^{i}=-\gamma(\cdot)-\dot{y}^{i}$ / 差分形式），守恒量变为 $\sum_i(s^i+y^i)$ 或 $\sum_i(s^i+z^i)$。

> 本节“出处/定型”部分可在 [18]/[17] 的参考文献表中核对；Cai & Ishii 2012 原文现已入库（`7269`，关键词即 *Surplus-based averaging*，见 8.2 节）。push-sum 一侧的机制细节为补充背景，未逐句核对。

## 4. 与其他四类非平衡图方法的对比

| 方法 | 思路 | 代价 |
| --- | --- | --- |
| 左特征向量法 | 用左特征向量构造零列和矩阵 | 需矩阵求逆/排序运算，需估计全局信息 |
| 权重平衡法 | 迭代生成双随机/平衡图 | 迭代协议开销 |
| push-sum | 显式使用出度调整不平衡 | 需出度信息、额外协议 |
| **surplus 法** | 辅助变量本地记账并补偿漂移 | 无需矩阵求逆/排序，无需迭代，轻量（仅 +1 维状态） |

这也是 [12] TCSII 2022 在 **有向网络 appointed-time 平均一致性** 中首次把 surplus 引入本系列工作的原因：用 $s^{i}$ 记录平均值漂移，再叠加指定时间控制框架（时间投影 + 运动规划 + 矩阵扰动分析）。

## 5. 从 surplus 到 **integral surplus（积分剩余）**

传统 surplus 只适用于 **静态** 目标（静态平均一致、分布式优化、资源分配）。到了 DAC，漂移由 **“时变参考信号 + PI 动力学” 共同引起**，是随时间变化的特殊漂移，静态记账无法记录，因此传统 surplus 失效。

**积分剩余**的改造要点：

1. 让 surplus 记录的是积分变量的 **变化量** 而非状态瞬间值：连续版 $\dot{s}^{i}=-\gamma(\cdot)-\dot{y}^{i}$（[18]）、离散版 $s^{i}(t_{k+1})=s^{i}(t_{k})-\nu\gamma(\cdot)-\big(z^{i}(t_{k+1})-z^{i}(t_{k})\big)$（[17]），守恒量变为 $\sum_i(s^i+y^i)$ / $\sum_i(s^i+z^i)$；
2. 加一个 **扰动参数** $\theta$（或 $\psi$）指定 surplus 对积分项的作用强度，其允许上界 $\bar{\theta}$ 由 $M(0)$ 的第三小特征值实部给出（[18] eq.(5)），必须 $\theta\in(0,\bar{\theta})$；
3. 分析难点变成 **“含扰动的耦合微分方程系统”**，需借助 **矩阵扰动理论** 给出稳态误差的精确表达式；
4. 结论形式：参考信号在一致性空间的正交投影本质有界 → 稳态误差有界且可由参数调节（[18]）；若存在 common function 等条件 → 精确 DAC；结合指定时间框架后，可在 **指定收敛时间** 内完成（[17]）。

## 6. 与后续工作的衔接

- [18] TAC 2025：integral surplus + PI 动力学，解决 **强连通但未必平衡** 有向图上的 DAC（有界可调稳态误差，特例下精确）。
- [17] TAC 2025：把 integral surplus 嵌入 **prescribed-time 框架**（时间段划分/时间投影 → 用 surplus 修正运动规划的 terminal condition → 最小能量控制器），并在其上叠加 **状态分解** 隐私保护，形成“指定时间 + 拓扑最一般 + 隐私”三者统一的方案。
- [3795] 2025（最新）：**nonsmooth surplus**——把 $s^{i}$ 嵌入 $\mathrm{sgn}\{\cdot\}$，与**非光滑** DAT 算法结合；用 1-范数 Lyapunov + 最小割分析，得到 **有限时间一致 + 渐近精确跟踪**，且初值任意。详见 [3795-nonsmooth-surplus.md](./3795-nonsmooth-surplus.md)。

## 7. 代价与未解问题

- 初始条件需满足 $\sum_i y^i(0)=0$、$s^{i}(0)=0$（论文 Remark 1 明确：方法容易实现，但如何进一步放宽仍是 open problem）；
- 一般是 **有界稳态误差**（可调）而非零误差，零误差需额外条件；
- surplus 需随状态一并广播，通信量增加一维。

## 8. 相关文献（okb 服务器索引）

> 下表条目均已在 okb 中检索核对；卷期页码以库内元数据为准，缺失者省略。排序：同一课题组谱系 → 方法源头与推广 → push-sum 血缘 → 背景阅读。

### 8.1 本系列的 surplus 谱系（同一课题组）

| 演进 | 文献 | okb id |
| --- | --- | --- |
| ① surplus 首用于有向图 appointed-time 平均一致 | ZHOU Y, LIU Y F, ZHAO Y, et al. Appointed-time average consensus over directed networks[J]. IEEE Transactions on Circuits and Systems II: Express Briefs, 2022. DOI: 10.1109/TCSII.2022.3152521 | `6180`（md 未生成） |
| ② **integral surplus**：DAC 首次打通非平衡有向图 | CAO R H, LIU Y F, ZHAO Y, et al. Dynamic average consensus over strongly connected digraphs based on integral surplus[J]. IEEE Transactions on Automatic Control, 2025. DOI: 10.1109/TAC.2025.3584699 | `6235` |
| ③ integral surplus + prescribed-time + 状态分解隐私 | CAO R H, ZHAO Y, LIU Y F, et al. Privacy-protected and prescribed-time dynamic average consensus over directed networks: an integral surplus-based approach[J]. IEEE Transactions on Automatic Control, 2025. DOI: 10.1109/TAC.2025.3578246 | `6219` |
| ④ **nonsmooth surplus**（最新，未进入 WoS 导出） | CAO R H, ZHAO Y. Distributed average tracking over directed communication networks: a nonsmooth surplus approach[J]. IEEE, 2025. DOI: 10.1109/TAC.2025.3629024 | `3795` |

第 ④ 篇要点（据全文）：常规 surplus 在存在**非光滑分量**（signum/投影类项）时会失效，故提出 *nonsmooth surplus*，把 surplus 与状态的交互**整体嵌入** $\mathrm{sgn}\{\cdot\}$，并在 $2n$ 维扩展矩阵 $\Upsilon(\zeta)$ 上重建零列和；分析改用 **Filippov 集值分析 + 1-范数 Lyapunov 函数 + 图论最小割**（矩阵扰动仅用于特征值条件），增益充分条件由最小割 $\mathrm{c}_{min}$ 给出，从而实现非平衡有向网络上的**精确** DAT。

**完整整理（问题 / 算法 / 创新点，带公式）→ [3795-nonsmooth-surplus.md](./3795-nonsmooth-surplus.md)**

> ⚠ 待核对：`3795` 库内 PDF 页眉写的是 *IEEE Transactions on Automatic Control*，而 DOI 前缀为 `TSMC`，正式出版信息需复核。

### 8.2 surplus 方法的源头与在优化/资源分配中的推广

| 文献 | 说明 | okb id |
| --- | --- | --- |
| CAI K, ISHII H. Average consensus on general strongly connected digraphs[J]. Automatica, 2012, 48(11): 2750-2761. DOI: 10.1016/j.automatica.2012.08.003 | **术语源头**：原文关键词即为 *Surplus-based averaging* | `7269` |
| CAI K, ISHII H. Average consensus on arbitrary strongly connected digraphs with time-varying topologies[J]. IEEE Transactions on Automatic Control, 2014. DOI: 10.1109/TAC.2014.2305952 | 推广到时变拓扑 | `7267` |
| CHEN G, LI Z. Distributed optimal resource allocation over strongly connected digraphs: a surplus-based approach[J]. Automatica, 2021. DOI: 10.1016/j.automatica.2020.109459 | surplus 进入资源分配，术语写进标题 | `7268` |
| SHI C X, YANG G H. Distributed optimization under unbalanced digraphs with node errors: robustness of surplus-based dual averaging algorithm[J]. IEEE Transactions on Control of Network Systems, 2021. DOI: 10.1109/TCNS.2020.3011837 | surplus 与 dual averaging 结合，并讨论节点误差鲁棒性 | `7265` |
| XU Y, HAN T R, CAI K, et al. A distributed algorithm for resource allocation over dynamic digraphs[J]. IEEE Transactions on Signal Processing, 2017. DOI: 10.1109/TSP.2017.2669896 | 动态有向图上的资源分配，被 6219 列为 surplus 路线的应用代表 | `7266` |

### 8.3 push-sum / ratio consensus 血缘（与 surplus 对照阅读）

| 文献 | 说明 | okb id |
| --- | --- | --- |
| NEDIC A, OLSHEVSKY A. Stochastic gradient-push for strongly convex functions on time-varying directed graphs[J]. IEEE Transactions on Automatic Control, 2016. DOI: 10.1109/TAC.2016.2529285 | 优化领域 push-sum（gradient-push）的代表工作 | `4400` |
| REZAEINIA P, GHARESIFARD B, LINDER T, et al. Push-sum on random graphs: almost sure convergence and convergence rate[J]. IEEE Transactions on Automatic Control, 2020. DOI: 10.1109/TAC.2019.2929206 | push-sum 在随机图上的几乎必然收敛与收敛速率 | `2863` |
| HADJICOSTIS C N, DOMINGUEZ-GARCIA A D. Privacy-preserving distributed averaging via homomorphically encrypted ratio consensus[J]. IEEE Transactions on Automatic Control, 2020. DOI: 10.1109/TAC.2020.2968876 | 非平衡图上 `y/w` **ratio consensus**（push-sum 的比值形式）代表，叠加同态加密保护隐私 | `5514` |

**对照要点**：push-sum 家族用“成对传播 + 相除”恢复比例；surplus 家族用“单变量记账 + 相加”补偿漂移。二者的守恒关系分别写在 $\sum_i y^i$ 与 $\sum_i w^i$、以及 $\sum_i(s^i+y^i)$ 上，这就是“同一思想的两种写法”的代数出处。

### 8.4 背景阅读

| 文献 | 说明 | okb id |
| --- | --- | --- |
| KIA S S, VAN SCOY B, CORTES J, et al. Tutorial on dynamic average consensus: the problem, its applications, and the algorithms[J]. IEEE Control Systems Magazine, 2019, 39(3): 40-72. DOI: 10.1109/MCS.2019.2900783 | DAC/DAT 领域权威综述（6235 的 [4]） | `3451` |
| SEN A, SAHOO S R, KOTHARI M. Distributed average tracking with incomplete measurement under a weight-unbalanced digraph[J]. IEEE Transactions on Automatic Control, 2022. DOI: 10.1109/TAC.2022.3179219 | 非平衡有向图上的 DAT，不依赖 surplus 的另一条路线，可作对照（6219 的 [16]） | `2302` |

> 延伸检索：okb 内 `1486`（Xian 等, IEEE TAC 2025）、`3067`（Fang 等, IEEE TCYB 2022）等文献正文亦出现 surplus，可作为二次检索入口；`6180` 的 markdown 待生成后可直接精读其 surplus 定义。
