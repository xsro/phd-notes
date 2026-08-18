# $\gamma_{i,s}$ 与 $\bar h$ 的选取分析（基于 Luo, Su, Zeng, IEEE TAC 2026）

本文针对论文 *Completely Distributed Joint State–Unknown Input Estimation via Interval Observer*（Luo, Su, Zeng, IEEE TAC, Vol. 71, No. 8, August 2026）中二阶滑模（Levant）微分器 (21) 的参数选取进行核查分析。所有引用均直接取自原文，未作任何臆造。

---

## 一、$\gamma_{i,s}$ 的选取规则（基于 Remark 6）

**Remark 6 原文（逐字引用）：**

> The parameters $\gamma_{i,s}^{(1)}$ and $\gamma_{i,s}^{(2)}$ are introduced to tune the dynamic gain of the sliding mode differentiator (21). The design idea draws inspiration from the "accuracy–convergence speed tradeoff" framework proposed in [38], which aims to balance estimation precision and convergence rapidity. To ensure that the differentiator can accurately estimate the derivative $\dot{\sigma}_{i,s}$ within finite time, we adopt the commonly used tuning rule
>
> $$ \gamma_{i,s}^{(1)} > \frac{1}{\bar{h}^2},\qquad \gamma_{i,s}^{(2)} > \bar{h} $$
>
> where $\bar{h} \ge |\ddot{\sigma}_{i,s}|$. These parameters can be selected solely based on locally available information at each agent, such as the known bounds of signal variation, without requiring any global network topology information.

**微分器 (21) 的引入句原文：**

> Furthermore, the estimation of $\dot{\sigma}_i$ will be carried out within a finite time using the differentiator described in [30]

**微分器 (21) 原文：**

$$
\begin{cases}
\dot{\alpha}_{i,s} = \beta_{i,s} = -\gamma_{i,s}^{(1)} |\alpha_{i,s} - \sigma_{i,s}|^{1/2} \mathrm{sign}(\alpha_{i,s} - \sigma_{i,s}) + \alpha_{i,s}^{(2)},\\[4pt]
\dot{\beta}_{i,s} = -\gamma_{i,s}^{(2)} \cdot \mathrm{sign}(\alpha_{i,s} - \beta_{i,s}),\qquad s = 1, 2, \dots, q_i
\end{cases} \tag{21}
$$

**为何这两个不等式保证 (21) 的有限时间收敛：**

式 (21) 即 Levant 二阶滑模微分器。其有限时间收敛性要求被微分信号 $\sigma_{i,s}(t)$ 的二阶导有界（即 $\bar h\ge|\ddot\sigma_{i,s}|$），并满足两条增益条件：

- **$\gamma_{i,s}^{(2)} > \bar h$**：提供足够大的“控制权威”（control authority），使得在滑模面附近 $\beta_{i,s}$（即 $\dot\alpha_{i,s}$ 对 $\dot\sigma_{i,s}$ 的估计）能够主导并抵消 $|\ddot\sigma_{i,s}|$ 的最大可能扰动，从而把跟踪误差压入二阶滑模层，实现有限时间内的精确微分。
- **$\gamma_{i,s}^{(1)} > 1/\bar h^2$**：设定终端吸引（terminal-attraction）收敛的标度。该条件来自 Levant 在文献 [38] 中的“精度–收敛速度折中”（accuracy–convergence speed tradeoff）框架——Remark 6 明确写道设计思想“draws inspiration from the 'accuracy–convergence speed tradeoff' framework proposed in [38]”。$\gamma^{(1)}$ 越大收敛越快，但过大会放大噪声/颤振；$\gamma^{(1)}>1/\bar h^2$ 是与 $\bar h$ 耦合的下界，保证在给定扰动幅值下仍能收敛。

**与自适应耦合增益 $\gamma_i,\rho_i$ 的区别（重要）：**

$\gamma_{i,s}^{(1)},\gamma_{i,s}^{(2)}$ 是**局部微分器 (21)** 的增益，用于估计 $\dot\sigma_i$；而式 (23)(24) 中的 $\rho_i,\gamma_i$ 是**分布式状态观测器 (22)** 的**自适应耦合增益**：

$$
\rho_i = \Bigl\| T_i^u \sum_{j=1}^{N} \ell_{ij} \hat{x}_j \Bigr\|^2 \tag{23}
$$
$$
\dot{\gamma}_i = \rho_i,\qquad \gamma_i(0) > 0. \tag{24}
$$

二者作用对象、更新机制、依赖信息完全不同，不可混淆。

**仅用局部信息：** Remark 6 末句明确指出——“These parameters can be selected solely based on locally available information at each agent, such as the known bounds of signal variation, **without requiring any global network topology information**.” 即 $\gamma_{i,s}^{(k)}$ 的选取不依赖全局拓扑。

---

## 二、$\bar h$ 如何计算 / 确定

**1. $\bar h$ 的定义——对 $\sigma_{i,s}$ 二阶导的已知上界，而非 $\vartheta$ 的 Lipschitz 常数**

Remark 6 中 $\bar h$ 被定义为

$$ \bar h \ge |\ddot\sigma_{i,s}(t)|, $$

即中间变量 $\sigma_{i,s}$ 的**二阶导**的已知上界。它**不是**未知输入 $\vartheta$ 自身的 Lipschitz 常数，也**不是**来自 Assumption 1（该假设仅关于通信拓扑 $G$ 的 iSCC 结构，见原文第 101 行）。$\sigma_{i,s}\in[0,1]$ 是由区间观测器上/下界定义的凸组合系数（式 (14)），其二阶导有界是微分器收敛的前提。

**2. $\bar h$ 为何存在且有限、并可局部获取——溯源链路**

- Eq. (3)：$\vartheta(t)$ 是 Lipschitz 连续函数，且 $\underline\vartheta(t)\le\vartheta(t)\le\bar\vartheta(t)$，即 $\vartheta$ 有界且 $\dot\vartheta$ 有界。
- Eq. (18)：$\dot y_i = C^{id}(A^{id}+K^{id}C^{id})\delta^{id} - C^{id}K^{id}y_i + C_i M\vartheta$，故 $\dot y_i$ 中包含 $C_i M\dot\vartheta$（有界）。
- Eq. (20) 与 Remark 4（第 498 行）：
  $$ \dot{\sigma}_i = \bigl[\mathrm{diag}(f_{i,1}(t)+\Phi_i)\bigr]^{-1} \dot{y}_i + \Psi_i(t). $$
  Remark 4 进一步说明：区间观测器内部状态 $\bar\xi^{id},\hat\xi^{id}$ 由 Hurwitz 稳定系统驱动，其导数连续有界，因此 $f_{i,1}(t)$ 与 $\Psi_i(t)$ “are smooth and bounded”。

综上，对 $\dot\sigma_i$ 再求导得 $\ddot\sigma_i$，其各项（Hurwitz 驱动的内部状态、有界的 $C_iM\dot\vartheta$、平滑有界的权重 $[\mathrm{diag}(f_{i,1}+\Phi_i)]^{-1}$）均有界，故 **$\bar h$ 存在且有限**。由于上述信号均来自**局部** $y_i$、已知边界 $\bar\vartheta,\underline\vartheta$ 及本地观测器状态，$\bar h$ 可由**局部信息**上界估计得到。

**3. 原文是否给出 $\bar h$ 的具体数值？（诚实核查 §IV）**

逐条核查 §IV “Numerical Examples”（Examples 1–3）：

- **Example 1**（第 908 行）：$\vartheta(t)=12\sin(2t)$，给出 $\bar\vartheta=12,\ \underline\vartheta=-12$，以及初始状态边界；但**未给出** $\bar h$ 或 $\gamma_{i,s}^{(1)},\gamma_{i,s}^{(2)}$ 的任何数值。
- **Example 2**（第 950、956 行）：$\bar\vartheta=[1/2;\,1/3]$（及 $\bar\vartheta=[\cos t/2;\sin t/3]$），$\underline\vartheta=-\bar\vartheta$；同样**未给出** $\bar h$ 或微分器增益数值。
- **Example 3**（第 965–966 行）：仅对比两类 $\bar\vartheta$（$[1/2;1/3]$ 与 $[2;3]$）对重构的影响；**未涉及** $\bar h$ 或 $\gamma_{i,s}^{(k)}$。

**结论（如实陈述）：** 论文**没有提供**计算 $\bar h$ 的显式公式，也未在仿真中给出 $\bar h$、$\gamma_{i,s}^{(1)}$、$\gamma_{i,s}^{(2)}$ 的具体取值；它把 $\bar h$ 当作一个“已知的局部上界”（known local bound），仿真取值（若存在）是 ad hoc 选定的。论文本身并未声称给出量化计算方法。

**4. 实现中如何选取 $\bar h$ 的实用建议（严格基于论文支持的范围）**

依据论文逻辑（$\vartheta$ 有界 Lipschitz、内部状态 Hurwitz 平滑、式 (18)(20)/Remark 4），实践中可按以下步骤本地上界化 $|\ddot\sigma_{i,s}|$：

1. 由已知边界 $\bar\vartheta,\underline\vartheta$ 及 $\vartheta$ 的 Lipschitz 常数估计 $\|\dot\vartheta\|_\infty$（如 Example 1 中 $\vartheta=12\sin2t\Rightarrow|\dot\vartheta|\le24$），并经式 (18) 的 $C_iM$ 通道、式 (20) 的平滑权重 $[\mathrm{diag}(f_{i,1}+\Phi_i)]^{-1}$ 传播，上界化 $\|\ddot y_i\|_\infty$。
2. 结合 Remark 4 中 $f_{i,1}(t),\Psi_i(t)$ 的光滑有界性，对 $\dot\sigma_i$ 再求导，得到 $|\ddot\sigma_{i,s}|$ 的保守上界，取为 $\bar h$（应留一定裕度）。
3. 再据 Remark 6 令 $\gamma_{i,s}^{(2)}>\bar h$、$\gamma_{i,s}^{(1)}>1/\bar h^2$ 选取微分器增益；在精度与抗噪间按 [38] 的折中调节。

注意：此"实用步骤"是对论文提供链条（式 (3)(18)(20)、Remark 4、Remark 6）的**工程化解释**，论文本身未给出封闭式计算公式，故应视为推导性指导而非原文定理。

---

## 三、Remark 6 中 "locally available information" 具体指哪些量

Remark 6 末尾声明：$\gamma_{i,s}^{(1)},\gamma_{i,s}^{(2)}$ 的选取"can be selected solely based on locally available information at each agent, such as the known bounds of signal variation, **without requiring any global network topology information**"。下面逐项列出 agent $i$ 在本地可以确知、并可用于确定 $\bar h$（进而选取 $\gamma_{i,s}^{(k)}$）的量。每一项均含：量 · 含义 · 论文引用 · 为何是本地。

1. **本地传感矩阵 $C_i$** — agent $i$ 自身的输出模型。Eq.(5) 明确写有 "$C_i \in \mathbb{R}^{q_i \times n}$ is a known matrix"（原文第 135 行），是其自身传感器配置，天然本地已知。

2. **本地测量 $y_i(t)$** — Eq.(5) 定义 $y_i(t)=C_i x(t)$；原文第 144 行强调 "Each UIO relies solely on its own local output $y_i(t)$"，即每个 agent 只用自己的输出，无需邻居的测量。

3. **全局对象矩阵 $A$** — 可检测性分解（第 169 行）与区间观测器/分布式观测器 (22) 都在 agent $i$ **本地**基于 $(A,C_i)$ 完成（"decompose it into its detectable and undetectable parts at agent $i$"）。$A$ 是被控对象的开环模型，属于 plant 知识而非网络拓扑信息，本地持有即可，不构成"全局拓扑"依赖。

4. **UI 分布矩阵 $M$** — 出现在本地方程 (22)、(11)、(9) 中（Eq.(22) 含 $M\hat\vartheta_i$；Eq.(11) 含 $T_i^{dT}M$；Eq.(9) 含 $T_i^{uT}M\vartheta$）。它描述未知输入如何作用于状态，是 plant 模型的一部分，本地可得。

5. **UI 上下界 $\bar\vartheta(t),\underline\vartheta(t)$** — Eq.(3) 给出 $\underline\vartheta(t)\le\vartheta(t)\le\bar\vartheta(t)$。原文第 950 行指出，边界信息"can usually be determined through physical laws, historical data, or engineering safety margins"（见 Example 2 / Remark 8 所述获取途径），来自物理规律或工程经验，**不来自邻居**，故每个 agent 本地可得。

6. **初值界 $\bar x(0),\underline x(0)$** — Eq.(4) 给出 $\underline x(0)\le x(0)\le\bar x(0)$；仅用于初始化本地区间观测器 (11) 的 $\bar\xi^{id}(0),\hat\xi^{id}(0)$（第 360–362 行），属本地设定，不涉及任何网络信息。

7. **区间观测器内部状态 $\bar\xi^{id},\hat\xi^{id}$（及输出界 $\bar y_i,\hat y_i$）** — 由 agent $i$ 自己的区间观测器 (11) 生成，$\bar y_i,\hat y_i$ 由 (13) 由其构造。Remark 4（第 501 行）明确：内部状态 $\bar\xi^{id},\hat\xi^{id}$ "are driven by a Hurwitz-stable system, their derivatives are continuous and bounded"。这些量是 agent $i$ 本地观测器的产物。

8. **可检测性分解量 $(T_i,T_i^d,T_i^u,A^{id},C^{id},K^{id},L^{id},\Omega^{id},\Xi^{id},S^{id})$** — 全部由本地 $(A,C_i)$ 算出：分解矩阵 $T_i,T_i^d,T_i^u$ 及 $A^{id},C^{id}$ 来自第 171–183 行的可检测性分解；$S^{id},\Omega^{id},\Xi^{id}$ 分别由第 233、319、284 行定义；$K^{id}$ 经极点配置使 $A^{id}+K^{id}C^{id}$ Hurwitz（第 183 行），$L^{id}$ 由本地 $(\aleph^{id},C^{id})$ 依 Algorithm 1（Remark 11）求解。均不依赖其他 agent。

9. **UI 代数函数辅助量 $f_{i,1},f_{i,2},f_{i,3},\Phi_i,\sigma_i$** — 由本地区间观测器输出、本地测量 $y_i$ 及本地分解矩阵构造：$f_{i,1}=\bar y_i-\hat y_i$（第 411 行），$f_{i,2},f_{i,3}$ 由第 432–442 行给出，$\Phi_i$ 与 $\sigma_i$ 由 (20)（第 469、472 行）定义，而 (14) 的 $\sigma_i$ 也只含本地 $\bar y_i,\hat y_i,y_i$。

10. **自适应耦合增益 $\gamma_i,\rho_i$ 与本地 Laplacian 行 $\ell_{ij}$** — Eqs.(23)(24)：$\rho_i=\bigl\|T_i^u\sum_{j=1}^N\ell_{ij}\hat x_j\bigr\|^2$（第 535 行），$\dot\gamma_i=\rho_i$（第 539 行）。求和只涉及 agent $i$ **自己那一行**的邻居权重 $\ell_{ij}$，即只需本地 Laplacian 行；增益随本地估计误差自适应更新，**不需要 Laplacian 的特征值/谱**或其他 agent 的增益。

### 明确不需要的全局量

论文所谓"无全局信息"（no global information）特指**网络拓扑**，并不包括 plant 模型 $(A,M)$。agent $i$ 不需要：

- 全局 Laplacian 的特征值或谱（因采用自适应增益 (23)(24) 取代需谱的静态耦合增益）；
- 网络直径、连通半径等图尺度量；
- 其他 agent 的 $C_j$ 或它们的可检测性分解；
- 聚合全局观测矩阵 $C=\mathrm{col}\{C_i\}$（仅 Eq.(6) 用于陈述，局部 agent 不必持有）；
- 任何集中式计算的耦合增益或协调器下发的参数。

它只需**自身 Laplacian 行 $\ell_{ij}$** 加本地 plant/UI/界数据即可完成全部设计，这正是"completely distributed"的含义。

### 回到 $\bar h$

综上所述，agent $i$ 可由本地已知信号变化界（$\bar\vartheta,\underline\vartheta$ 保证 $\vartheta$ Lipschitz 有界，结合自身区间观测器由 Hurwitz 动力学驱动、导数连续有界这一 Remark 4 性质）推出 $\ddot\sigma_{i,s}$ 的有限上界，从而确定 $\bar h\ge|\ddot\sigma_{i,s}|$。整条推导链（式 (3)(18)(20)、Remark 4、Remark 6）**只用本地量**：本地 $y_i$、$C_i$、本地 $(A,M)$、本地上下界、本地观测器状态与本地 Laplacian 行 $\ell_{ij}$。其中没有任何 Laplacian 特征值、网络谱或其他 agent 的数据进入 $\gamma_{i,s}^{(k)}$ 的选取，与 Remark 6 "without requiring any global network topology information" 完全一致。
