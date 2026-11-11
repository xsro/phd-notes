# 3795｜非光滑剩余（Nonsmooth Surplus）DAT：问题 · 算法 · 创新点

| 项目 | 内容 |
| --- | --- |
| 题名 | Distributed Average Tracking over Directed Communication Networks: A Nonsmooth Surplus Approach |
| 作者 | Runhua Cao, Yu Zhao（西北工业大学自动化学院，通信作者 Yu Zhao） |
| 文献定位 | okb `3795`；DOI `10.1109/TAC.2025.3629024`（库内 PDF 页眉写作 *IEEE Transactions on Automatic Control*，正式出版信息待核对） |
| 基金 | NSFC 62422315、62573348；陕西省自然科学基础研究 2025JC-YBMS-667 等 |
| 数据来源 | okb `3795` 全文 markdown（§I 引言、§II 预备、§III 问题/算法/稳定性、§IV 应用、仿真） |

---

## 1. 问题：非平衡有向网络上的**精确** DAT

**记号**：$\mathcal{G}=(\mathcal{V},\mathcal{E})$ 强连通且**权重非平衡**有向图，$n$ 个智能体；$x^{i}(t)\in\mathbb{R}$ 为一致性状态，$r^{i}(t)\in\mathbb{R}$ 为**仅智能体 $i$ 可得**的时变参考信号；$a^{ij}>0\iff(j,i)\in\mathcal{E}$；$d_{in}^{i}=\sum_{j}a^{ij}$，$d_{out}^{i}=\sum_{l}a^{li}$；两套 Laplacian

$$L_{in}=D_{in}-A\ \ (\text{行和零：}L_{in}\mathbf{1}_{n}=\mathbf{0}_{n}),\qquad L_{out}=D_{out}-A\ \ (\text{列和零：}\mathbf{1}_{n}^{\top}L_{out}=\mathbf{0}_{n}^{\top})$$

**Problem 1**（非平衡有向网络上的 DAT）：

$$\lim_{t\to\infty}\left|x^{i}(t)-\frac{1}{n}\sum_{j=1}^{n}r^{j}(t)\right|=0 \tag{4}$$

**Assumption 1**：$\mathcal{G}$ 强连通且权重非平衡。
**Assumption 2**（只要求**相对差**有界，不要求信号本身有界）：

$$\left|r^{i}(t)-r^{j}(t)\right|\le \bar r_x,\qquad \left|\dot r^{i}(t)-\dot r^{j}(t)\right|\le \bar r_v,\qquad \forall i,j\in\mathcal{V}$$

**为什么难（Remark 2）**：标准一致性项 $-\sum_{j\in\mathcal{N}^{i}_{in}}a^{ij}(x^{i}-x^{j})$ 的效力依赖 Laplacian 的三条性质：

| 编号 | 性质 | 作用 | 非平衡图上 |
| --- | --- | --- | --- |
| a) | 单零特征值、其余实部为正 | 保证**一致** | 保持 |
| b) | 零空间由 $\mathbf{1}_{n}$ 张成 | 保证**一致** | 保持 |
| c) | 零列和（$\mathbf{1}_{n}^{\top}L=\mathbf{0}_{n}^{\top}$） | 保证**收敛到平均值** | **失效** |

c) 一坏，一致性值就整体偏离期望平均（average shift）。传统 surplus 正是用来"重建 c)"的，但**既有非光滑 DAT 算法（node-based / edge-based）把邻居交互嵌进了 $\mathrm{sgn}(\cdot)$ 等非光滑分量内部**，surplus 无法再以"光滑线性注入"的方式参与，于是出现两个同时待解的难点：

$$\underbrace{\dot s=-\gamma L's-\dot y}_{\text{integral surplus（6235）：线性，可做矩阵扰动分析}}\qquad\text{vs.}\qquad \underbrace{\dot s^{i}=-\tau s^{i}+\eta\,\mathrm{sgn}\{\cdots\}}_{\text{本文：非线性注入，常规工具全部失效}}$$

---

## 2. 算法：把 surplus 塞进非光滑函数

引入**非光滑剩余** $s^{i}(t)\in\mathbb{R}$（每智能体一维，可逐维推广；分量解耦，故一维分析即可推广到高维）：

$$\dot x^{i}(t)=-\tau\big(x^{i}(t)-r^{i}(t)\big)+\dot r^{i}(t)+u_{x}^{i}(t) \tag{5a}$$

$$\dot s^{i}(t)=-\tau s^{i}(t)+u_{s}^{i}(t) \tag{5b}$$

$$u_{x}^{i}(t)=\eta\,\mathrm{sgn}\Big\{-\sum_{j\in\mathcal{N}_{in}^{i}}a^{ij}\big(x^{i}(t)-x^{j}(t)\big)+\zeta s^{i}(t)\Big\} \tag{6a}$$

$$u_{s}^{i}(t)=\eta\,\mathrm{sgn}\Big\{\sum_{j\in\mathcal{N}_{in}^{i}}a^{ij}\big(x^{i}(t)-x^{j}(t)\big)-\zeta s^{i}(t)-d_{out}^{i}s^{i}(t)+\sum_{j\in\mathcal{N}_{in}^{i}}a^{ij}s^{j}(t)\Big\} \tag{6b}$$

参数含义：$\tau>0$ 跟踪/遗忘速率；$\eta>0$ 非光滑增益（取值范围见 §4）；$\zeta>0$ 扰动参数，刻画 $s^{i}$ 对 $x^{i}$ 的补偿强度。**初值 $x^{i}(0)$、$s^{i}(0)$ 可任意选取，无任何初始约束。**

**紧凑形式**（$\boldsymbol{x},\boldsymbol{s},\boldsymbol{r}\in\mathbb{R}^{n}$）：

$$\dot{\boldsymbol{x}}(t)=-\tau\big(\boldsymbol{x}(t)-\boldsymbol{r}(t)\big)+\dot{\boldsymbol{r}}(t)+\eta\,\mathrm{sgn}\big\{-L_{in}\boldsymbol{x}(t)+\zeta\boldsymbol{s}(t)\big\} \tag{7a}$$

$$\dot{\boldsymbol{s}}(t)=-\tau\boldsymbol{s}(t)+\eta\,\mathrm{sgn}\big\{L_{in}\boldsymbol{x}(t)-L_{out}\boldsymbol{s}(t)-\zeta\boldsymbol{s}(t)\big\} \tag{7b}$$

**2n 维扩展矩阵形式**：令 $P\triangleq\begin{bmatrix}0_{n}&-I_{n}\\0_{n}&I_{n}\end{bmatrix}$，

$$\begin{bmatrix}\boldsymbol{u}_x\\ \boldsymbol{u}_s\end{bmatrix}=-\eta\,\mathrm{sgn}\Big\{\Upsilon(\zeta)\begin{bmatrix}\boldsymbol{x}\\ \boldsymbol{s}\end{bmatrix}\Big\},\qquad \Upsilon(\zeta)=\begin{bmatrix}L_{in}&-\zeta I_{n}\\-L_{in}&L_{out}+\zeta I_{n}\end{bmatrix}=\Upsilon(0)+\zeta P \tag{8}$$

**设计的全部要害**在于让 $\Upsilon(\zeta)\in\mathbb{R}^{2n\times 2n}$ 重新满足 §1 那张表的三条：

| 要求 | 由谁保证 |
| --- | --- |
| a) 单零特征值、其余实部为正 | $\zeta$ 足够小 + **矩阵扰动理论**（Lemma 1、Lemma 2） |
| b) 零空间 = $\mathrm{col}(\mathbf{1}_{n},\mathbf{0}_{n})$ | 直接验证 $\Upsilon(\zeta)\,\mathrm{col}(\mathbf{1}_{n},\mathbf{0}_{n})=\mathbf{0}_{2n}$ |
| c) 零列和 | **必须用出度 Laplacian $L_{out}$**，因 $\mathbf{1}_{n}^{\top}L_{out}=\mathbf{0}_{n}^{\top}$ |

**Remark 4（一个易错点）**：若把 $L_{out}$ 换成 $L_{in}$（如 $\Upsilon=\begin{bmatrix}L_{in}&-\zeta I\\-L_{in}&L_{in}+\zeta I\end{bmatrix}$），特征值与零空间条件仍可成立，但**永远拿不到正确平均值**，表现退化为无向图算法——这说明"零列和必须在 $2n$ 维扩展系统里被真正重建"。

---

## 3. 两条关键引理

### 3.1 Lemma 2：扰动参数的上界 $\bar\zeta$

$$\bar\zeta\triangleq\frac{\big(\mathrm{Re}[\Lambda_{\Upsilon}^{3}(0)]\big)^{2n}}{\big(8[3d_{in}^{max}+d_{out}^{max}]+4\big)^{2n}} \tag{12}$$

当 $\zeta\in(0,\bar\zeta)$ 时，排序特征值满足

$$0=\Lambda_{\Upsilon}^{1}(\zeta)<\mathrm{Re}[\Lambda_{\Upsilon}^{2}(\zeta)]\le\cdots\le\mathrm{Re}[\Lambda_{\Upsilon}^{2n}(\zeta)] \tag{13}$$

推导三条支点：

1. $\zeta=0$ 时 $\Upsilon(0)=\mathrm{blkdiag}(L_{in},L_{out})$（下三角），零特征值**重数 2**（半单）；
2. 半单零特征值的一阶导数（Lemma 1）：$\dfrac{d\lambda_{\Upsilon}^{1}}{d\zeta}\Big|_{0}=0$，$\dfrac{d\lambda_{\Upsilon}^{2}}{d\zeta}\Big|_{0}=-\boldsymbol\varphi_{1}^{\top}\boldsymbol\varphi_{2}>0$（$\boldsymbol\varphi_{1}^{\top}$ 为 $L_{in}$ 的正左特征向量，$\boldsymbol\varphi_{2}$ 为 $L_{out}$ 的负右特征向量）；
3. 用**最优匹配距离**把"耦合造成的整体漂移"关进可控范围：

$$d_{\Upsilon}\le 4\big(6d_{in}^{max}+2d_{out}^{max}+\zeta\big)^{1-\frac{1}{2n}}\zeta^{\frac{1}{2n}}<\mathrm{Re}[\Lambda_{\Upsilon}^{3}(0)] \tag{18}$$

### 3.2 Lemma 3：1-范数的下界 = 图割

定义有向割权与最小割：

$$\mathrm{c}_{s}\triangleq\sum_{i\in s,\ j\in\bar s}a^{ij},\qquad \mathrm{c}_{min}\triangleq\min_{s\subsetneqq\mathcal{V},\,s\neq\emptyset}\{\mathrm{c}_{s}\}$$

对 $\boldsymbol{z}=\mathrm{col}(\boldsymbol{x},\boldsymbol{s})$ 与任意 $\boldsymbol\varepsilon\in\mathcal{F}[\mathrm{sgn}](\Upsilon(\zeta)\boldsymbol{z})$，存在与 $\boldsymbol\varepsilon$ 选取无关的下界

$$\boldsymbol\varepsilon^{\top}\Upsilon(\zeta)\boldsymbol{z}\ \ge\ \varpi\ \ge\ 2\min\{\mathrm{c}_{min},\zeta\}\qquad\big(\Upsilon(\zeta)\boldsymbol{z}\neq\mathbf{0}_{2n}\big) \tag{19}$$

**含义**：$\mathrm{sgn}(\cdot)$ 的"降速"下界完全由**图的割**决定——网络割越薄（$\mathrm{c}_{min}$ 越小），就需要更大的 $\zeta$ 兜底。这正是 §4 增益条件里出现 $\min\{\mathrm{c}_{min},2\zeta\}$ 的原因，也是"割分析"不可替代的地方。

---

## 4. 主定理与证明骨架

**Theorem 1**：在 Assumption 1–2 下，取 $\zeta\in(0,\bar\zeta)$、$\eta\in[\underline\eta,+\infty)$，其中

$$\underline{\eta}\triangleq\frac{2n(\tau\bar r_x+\bar r_v)\big(d_{in}^{max}+d_{out}^{max}\big)+\epsilon_0}{\min\{\mathrm{c}_{min},2\zeta\}} \tag{33}$$

（$\epsilon_0$ 为任意正常数），则算法 (5)(6) 解决 Problem 1。

令 $\boldsymbol\chi=\mathrm{col}(\boldsymbol{x},\boldsymbol{s})$，$\boldsymbol\chi_{r}=\mathrm{col}(\boldsymbol{r},\mathbf{0}_{n})$，误差系统为

$$\dot{\boldsymbol\chi}(t)=-\tau\big(\boldsymbol\chi(t)-\boldsymbol\chi_{r}(t)\big)+\dot{\boldsymbol\chi}_{r}(t)-\eta\,\mathrm{sgn}\big(\Upsilon(\zeta)\boldsymbol\chi(t)\big) \tag{34}$$

**阶段一：有限时间达成一致、surplus 归零。** 取**扰动型 1-范数 Lyapunov 函数**

$$V(\boldsymbol{e}_{c}(t))=\big\|\Upsilon(\zeta)\boldsymbol\chi(t)\big\|_{1},\qquad \boldsymbol{e}_{c}(t)\triangleq\Upsilon(\zeta)\boldsymbol\chi(t) \tag{35}$$

$V$ 局部 Lipschitz 但非光滑，需用 Clarke 广义梯度与 Filippov 集值 Lie 导数 $\dot{\tilde V}$；结合 Lemma 3 得

$$\tilde a\le-\eta\min\{\mathrm{c}_{min},2\zeta\}+2n(\tau\bar r_x+\bar r_v)\big(d_{in}^{max}+d_{out}^{max}\big)\le-\epsilon_0 \tag{39}$$

$$\Longrightarrow\ V(t)\le V(0)-\epsilon_0 t\ \Longrightarrow\ T=\frac{\|\Upsilon(\zeta)\boldsymbol\chi(0)\|_{1}}{\epsilon_0}$$

即 $V$ 在**有限时间 $T$** 内到零；由 (13) 知 $\Upsilon(\zeta)$ 只有单零特征值，故 $\boldsymbol x$ 一致、$\boldsymbol s\to\mathbf{0}$。

**阶段二：渐近实现精确 DAT。** 一致后 $u_{x}^{i}\equiv 0$，令 $e_{r}(t)\triangleq\sum_{i}(x^{i}-r^{i})$、$W=\tfrac12 e_{r}^{2}$，

$$\dot W=-\tau W+e_{r}\sum_{i}u_{x}^{i}\le-\frac{\tau}{2}W+\frac{1}{2\tau}\Big\|\sum_{i}u_{x}^{i}\Big\|_{2}^{2} \tag{43}\ \Longrightarrow\ W(e_r(t))\le e^{-\frac{\tau}{2}(t-T)}W(e_r(T))\to 0$$

于是 $\lim_{t\to\infty}x^{i}(t)=\frac1n\sum_{j}r^{j}(t)$，得到**精确** DAT（不是有界误差）。

> 结论形态很值得记：**"有限时间一致（靠非光滑 surplus）+ 渐近跟踪（靠 $-τ(x-r)+\dot r$）"的两阶段收敛**，而不是单一收敛速率。仿真取 $\eta=27$、$\tau=1.2$、$\zeta=0.2$（10 智能体）与 $\eta=80$（100 智能体）均验证。

---

## 5. 创新点（三条）

### i) 概念创新：nonsmooth surplus

把 surplus 与状态的交互**整体嵌入** $\mathrm{sgn}\{\cdot\}$：$\mathrm{sgn}$ 内部同时含一致性项 $\pm L_{in}\boldsymbol x$ 与补偿项 $\zeta\boldsymbol s$、以及 $L_{out}\boldsymbol s$。由此一次性拿到两件事：

$$u_{x}^{i}\ \text{中的}\ \zeta s^{i}\ \text{负责补偿 average shift};\qquad u_{s}^{i}\ \text{中的}\ L_{out}s\ \text{负责重建零列和}$$

- 对照 **integral surplus**（`6235`/`6219`）：那里是**线性注入** $\theta s^{i}$（加进 $\dot y^{i}$），所以可用矩阵扰动 + 二次型 Lyapunov；本文是**非线性注入**，那条技术路线整体失效。
- 收益：既能跟踪**动态**参考信号（传统 surplus 只处理过静态目标），又摆脱了无向/权重平衡要求。

### ii) 方法创新：最小割型稳定性分析

signum 使二次型 Lyapunov 失效，故构造 **扰动型 1-范数 Lyapunov 函数** $V=\|\Upsilon(\zeta)\boldsymbol\chi\|_{1}$，用 **Filippov 集值分析 + 图割视角**，把增益条件用**最小割 $\mathrm{c}_{min}$** 表出（式 (19)(33)）：

$$\eta\ \ge\ \underline\eta\sim\frac{\text{参考信号相对界}\times\text{度界}}{\min\{\mathrm{c}_{min},2\zeta\}}$$

这与传统 surplus 的"$\theta\in(0,\bar\theta)$（由 $\mathrm{Re}[\Lambda^{3}(0)]$ 给出）"形成互补：一个管**拓扑薄弱处**（割），一个管**特征值间距**。

### iii) 性质与可扩展性

1. **初值任意**：取消 $\sum_i y^{i}(0)=0$、$s^{i}(0)=0$ 一类初始约束 → 对初始化误差鲁棒，且支持"连入/断开"网络（仿真中拓扑在 $t=6\,\mathrm{s}$ 突变，全体智能体收敛到**新**平均值；$t\le 6$ 时两组各自收敛到本组动态平均）。
2. **计算量轻**：本文 $\mathcal{O}(2n)$；左特征向量法 $\mathcal{O}(n^{2}+n)$。100 智能体：$\mathcal{O}(200)$ vs $\mathcal{O}(100100)$。
3. **可扩展**：把 DAT 推广到**多动态目标编队包围 + 多样化探索网络**（§6）。

---

## 6. 应用：多动态目标分布式编队包围（§IV）

**探索矩阵** $Q=[q^{ik}]\in\mathbb{R}^{n\times n_{T}}$，$q^{ik}=1$ 表示智能体 $i$ 可获取目标 $k$ 的信息；informed 智能体：$\sum_{k}q^{ik}\ge1$，其数量记为 $n_{I}$。

**Problem 2**（非平衡有向网络 + 多样化探索网络上的编队包围）：

$$\lim_{t\to\infty}\left|x^{i}(t)-h^{i}(t)-\sum_{k=1}^{n_{T}}g^{k}p^{k}(t)\right|=0,\qquad \forall g^{k}\in\mathbb{R}^{+},\ \sum_{k=1}^{n_{T}}g^{k}=1 \tag{46}$$

（$h^{i}(t)$ 为时变队形，$p^{k}(t)$ 为目标位置。）**Assumption 3**：每个目标至少被一个智能体探索；**Assumption 4**：目标线性组合的"相对差"及其导数有界。

**算法 (47)** 在 (5) 上加入队形与目标项，关键是新设计的 **agent-target 映射结构**

$$\omega^{i}=\begin{cases}0, & \displaystyle\sum_{k=1}^{n_{T}}q^{ik}=0\\[2mm] \dfrac{n}{n_{I}\sum_{k=1}^{n_{T}}q^{ik}}, & \displaystyle\sum_{k=1}^{n_{T}}q^{ik}\ge1\end{cases} \tag{48}$$

$$\dot{\boldsymbol{x}}=-\tau(\boldsymbol{x}-\boldsymbol{h})+\dot{\boldsymbol{h}}+\Omega Q(\tau\boldsymbol{p}+\dot{\boldsymbol{p}}),\qquad \Omega=\mathrm{diag}\{\omega^{i}\}$$

有了 $\omega^{i}$ 做归一化，探索关系可从"严格一对一"放宽为 **一对多 / 多对一 / 多对无**，且所有智能体仍收敛到目标的**凸组合**上（Remark 9：若 $\sum_i h^{i}(t)\to0$，队形中心收敛到凸组合；取 $n_{T}=n$、$g^{k}=1/n$ 时 Problem 2 退化为 Problem 1，即 **DAT 是编队包围的特例**）。

---

## 7. 与 integral surplus 的对照（同组前后两篇）

| 维度 | integral surplus（`6235` / `6219`） | nonsmooth surplus（`3795`） |
| --- | --- | --- |
| surplus 注入方式 | 线性：$\dot s^{i}=-\gamma(\cdot)-\dot y^{i}$ | 非线性：$s^{i}$ 进入 $\mathrm{sgn}\{\cdot\}$ 内部 |
| 收敛结论 | 有界可调稳态误差（特例精确） | 有限时间一致 + **渐近精确** DAT |
| 分析工具 | 矩阵扰动 + 二次型 Lyapunov | Filippov 集值 + 1-范数 Lyapunov + **最小割** |
| 关键参数界 | $\theta\in(0,\bar\theta)$，$\bar\theta$ 由 $\mathrm{Re}[\Lambda^{3}_{M}(0)]$ 定 | $\eta\ge\underline\eta$，$\underline\eta$ 由 $\mathrm{c}_{min}$ 与 $\zeta$ 定 |
| 初值要求 | $\sum_i y^{i}(0)=0$、$s^{i}(0)=0$ | 任意 |
| 抖振 | 无（连续算法） | **有**（需边界层/高阶滑模） |
| 附加能力 | 指定时间 + 状态分解隐私 | 拓扑突变鲁棒 + 编队包围扩展 |

## 8. 局限与作者列出的后续方向

1. **抖振**：$\mathrm{sgn}$ 引起抖振。可用边界层 $h(z)=\dfrac{z}{\|z\|_{2}+\varepsilon}$，或 $\mathrm{sat}(z/\varepsilon)$、$\tanh(z/\varepsilon)$；取常值 $\varepsilon$ 得**有界**跟踪误差，取时变 $\varepsilon=\epsilon_{1}e^{-\epsilon t}$ 则兼顾消抖与零误差；作者计划后续结合**高阶滑模**（已在引言中作为对照文献）。
2. **参数依赖全局信息**：$\mathrm{Re}[\Lambda_{\Upsilon}^{3}(0)]$、$n$、$\mathrm{c}_{min}$ 都是全局量（Remark 8 自述这在非平衡图文献中很常见），可行方向是分布式参数估计 → **自适应 DAT**。
3. **出版信息待核对**：DOI 前缀为 `TSMC`，库内 PDF 页眉为 IEEE TAC。

> 关联笔记：[surplus.md](./surplus.md)（surplus 概念、与 push-sum 的关系、okb 文献索引）；[README.md](./README.md)（作者文献总表与国标格式）。
