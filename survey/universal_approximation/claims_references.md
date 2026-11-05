# 结论 → 定理 → 出处：NN 拟合 `y = f(x) + d(t)`（|d| ≤ D）的误差保证

> 配套文档：`universal_approximation_in_okb.md`
> 说明：标注 **【库内】** 的文献可以在 okb-assist 里直接读到（给 doc id）；其余需外查。
> 标 **【自证】** 的命题在文献中通常不作为"定理"出现（过于初等），正确做法是写成 Lemma + 两行证明，引方法出处，而不是硬找一篇论文替你说话。

---

## 0. 一张表

| # | 结论 | 严格表述 | 权威出处 | 类型 |
|---|---|---|---|---|
| P1 | 连续 NN 的一致极限必连续 | 一致极限保连续 | Rudin, PMA 3rd ed., **Thm 7.12** 【库内 3812】 | 定理 |
| P2 | 不连续目标的 sup 误差地板 = 半振荡 | $\inf_{g\in C(K)}\|f-g\|_\infty=\tfrac12\max_x\mathrm{osc}(f,x)$ | 下界【自证】；上界用 Katětov–Tong 插入定理 | 定理（需拼装） |
| P3 | 有界噪声下 f 无法优于 $D/2$ 被辨识，且与 $n$ 无关 | minimax sup 误差 $=\Theta(D)$ | 两点法【自证】+ Tsybakov Ch.2；UBB 辨识 Milanese–Vicino / Norton / Walter–Piet-Lahanier | 自证 + 框架 |
| P4 | 随机零均值噪声下相合，速率 $n^{-2\beta/(2\beta+d)}$ | Stone 最优速率 | Stone (1982)；Ibragimov–Hasminskii (1981)；Györfi et al. (2002) | 定理 |
| P5 | NN 的逼近+估计分解，$\sigma^2$ 为预测地板 | $E[(y_{\rm new}-\hat f)^2]=E[(f-\hat f)^2]+\sigma^2$ | Barron (1994)；Schmidt-Hieber (2020)；ESL §7.3 | 定理 |
| P6 | 闭环/EIV 数据有偏，且不可由 $n\to\infty$ 消除 | 偏差由噪声–回归量相关性决定 | Forssell–Ljung (1999)；Söderström (2007)【库内二次引用 3227/3747】 | 定理 |
| P7 | NN 会吸收 $d$ 的低频谱（慢变成分） | 谱偏置 | Rahaman et al. (2019)；Tancik et al. (2020)【库内二次引用 3304】 | 经验+部分理论 |
| P8 | NN + 有界残差 ⇒ UUB，而非渐近 | - | Patre et al. (2008)【库内 2449】；Xian et al. (2004) | 定理 |

---

## 1. P1：为什么连续网络永远逼近不了不连续函数

**定理（Rudin, PMA, Thm 7.12）**【库内 3812，原文已核对】

> If $\{f_n\}$ is a sequence of continuous functions on $E$, and if $f_n\to f$ uniformly on $E$, then $f$ is continuous on $E$.

直接用：ReLU / tanh / sigmoid 网络都是连续函数 ⇒ 其一致极限属于 $C(K)$ ⇒ 若 $f\notin C(K)$，则无论网络多宽多深

$$\|\hat f-f\|_\infty\ \ge\ \mathrm{dist}_\infty(f,C(K))\ >\ 0 .$$

> 注：UAT 本身只对**连续**目标陈述（Hornik 1989 / Cybenko 1989；库内 3298 §2.2、3423 §3.2 均明确写了 continuity 前提）。所以"不连续情形"不是 UAT 的加强，而是它的边界。

---

## 2. P2：地板有多大——半振荡定理

**定义**：$\mathrm{osc}(f,x):=\inf_{\delta>0}\operatorname{diam} f(B(x,\delta))$，$\omega:=\sup_{x\in K}\mathrm{osc}(f,x)$。跳跃 $\Delta$ 就是 $\omega$ 的特例。

**命题**：$K$ 紧致、$f:K\to\mathbb R$ 有界，则
$$\mathrm{dist}_\infty\big(f,C(K)\big)=\frac{\omega}{2}.$$

**下界（3 行，可直接写进论文）**：设 $g\in C(K)$。对任意 $x$，取 $y_k,z_k\to x$ 使 $|f(y_k)-f(z_k)|\to \mathrm{osc}(f,x)$，则
$$|f(y_k)-f(z_k)|\le |f(y_k)-g(y_k)|+|g(y_k)-g(z_k)|+|g(z_k)-f(z_k)|\le 2\|f-g\|_\infty+|g(y_k)-g(z_k)|,$$
令 $k\to\infty$（$g$ 连续）得 $\mathrm{osc}(f,x)\le 2\|f-g\|_\infty$，取上确界即得 $\|f-g\|_\infty\ge\omega/2$。

**上界（存在性）**：记上/下包络 $\overline f(x)=\sup_{\delta}\sup_{B(x,\delta)}f$（上半连续）、$\underline f(x)=\inf_\delta\inf_{B(x,\delta)}f$（下半连续），则 $\underline f\le f\le\overline f$ 且 $\overline f-\underline f=\mathrm{osc}(f,\cdot)\le\omega$。取 $l_1=\overline f-\omega/2$（上半连续）、$u_1=\underline f+\omega/2$（下半连续），由 $\overline f-\underline f\le\omega$ 得 $l_1\le u_1$。**Katětov–Tong 插入定理**（紧度量空间正规）给出连续 $g$ 满足 $l_1\le g\le u_1$，于是 $|f-g|\le\omega/2$。

- Katětov, M. (1951), *On real-valued functions in topological spaces*, Fund. Math. 38:85–91.
- Tong, H. (1952), *Some characterizations of normal and perfectly normal spaces*, Duke Math. J. 19:417–428.

**工程化版本（更容易被接受）**：若 $f$ 分段连续、只有有限个跳跃点，最大跳跃 $\Delta_{\max}$，则可用磨光（mollifier）在宽度 $\eta$ 的过渡带内把跳跃抹平，得 $\|f-g_\eta\|_\infty\le \Delta_{\max}/2+\eta$。这就是"$\pm\Delta_{\max}/2$"的来源，不需要插入定理。

> ⚠️ 若你要的只是"$L^p$ 精度"：$C(K)$ 在 $L^p(K)$ 中稠密（$K$ 有界、$f\in L^p$），再叠加 UAT，就有 $\overline{\mathcal N}=L^p(K)$。出处是标准的稠密性（Rudin 或任意实分析教材；NN 侧见 Pinkus 1999 *Approximation Theory of the MLP Model in Neural Networks*, Acta Numerica，第 3 节讨论 $L^p$ 稠密性）。

---

## 3. P3：有界噪声下的不可辨识性（这是你最需要的"权威支撑"）

### 3.1 命题（可自证，建议写成 Lemma）

设 $y_i=f(x_i)+d_i$，$|d_i|\le D$，$\mathcal F$ 为任一含 $\{0,\,D\cdot\mathbf 1\}$ 的函数类（有界函数、Lipschitz 函数、连续函数都满足）。则任意估计器 $\hat f$：

$$\sup_{f\in\mathcal F}\ \sup_{|d|\le D}\ \big\|\hat f-f\big\|_\infty\ \ge\ \frac{D}{2},\qquad
\text{而插值估计 } \hat f(x_i)=y_i \text{ 达到} \ \le D .$$

**证明（两点法，3 行）**：取 $f_1\equiv 0,\ d_1\equiv 0$ 与 $f_2\equiv D,\ d_2\equiv -D$，二者产生**完全相同的数据** $y\equiv 0$，故 $\hat f$ 相同，于是
$$\max_{i=1,2}\|\hat f-f_i\|_\infty\ \ge\ \tfrac12\|f_1-f_2\|_\infty=\tfrac12 D . \qquad\blacksquare$$

**方法出处**（引用它来说明这是标准手法，不是临时发明）：
- A. B. Tsybakov, *Introduction to Nonparametric Estimation*, Springer 2009, **Ch. 2**（两点法/Le Cam 方法的统一框架；注意书中是随机噪声用 KL 散度，确定性有界噪声需在 $\|\cdot\|_\infty$ 上做，故需你自己写）。
- I. A. Ibragimov & R. Z. Has'minskii, *Statistical Estimation: Asymptotic Theory*, Springer 1981（minimax 下界的原始方法论）。

### 3.2 为什么"更多数据也没用"——集合隶属（UBB）辨识的语言

噪声只需"未知但有界"（unknown-but-bounded, UBB）时，数据给出的**全部信息**是可行集

$$\mathcal S=\Big\{g:\ |g(x_i)-y_i|\le D,\ \forall i\Big\},\qquad f\in\mathcal S .$$

$\mathcal S$ 在任何 $x$ 上的截面宽度恒为 $2D$，**与样本量无关**；它不会收敛到一个点。要让它收缩，必须加假设（噪声界"紧"、激励充分、模型类有限维且正确等）。

- M. Milanese & A. Vicino (1991), *Optimal estimation theory for dynamic systems with set membership uncertainty: an overview*, **Automatica** 27(6):997–1009.
- E. Walter & H. Piet-Lahanier (1990), *Estimation of parameter bounds from bounded-error data: a survey*, Math. Comput. Simulation 32:449–468.
- J. P. Norton (1987), *Identification and application of bounded-parameter models*, **Automatica** 23(4):497–507.

### 3.3 机器学习里的同名结论

- **Agnostic learning**：不对噪声做任何分布假设时，任何学习器的最优可达误差是"类内最优 + 估计误差"，不可能更好 —— M. J. Kearns, R. E. Schapire, L. M. Sellie (1994), *Toward efficient agnostic learning*, **Machine Learning** 17:115–141.
- **Malicious noise**：以比率 $\eta$ 的恶意噪声污染样本时，误差下界随 $\eta$ 线性上升，且一般不可学习 —— M. J. Kearns & M. Li (1993), *Learning in the presence of malicious errors*, **SIAM J. Comput.** 22(4):807–837.
- 教材：Shalev-Shwartz & Ben-David, *Understanding Machine Learning*, CUP 2014, Ch. 3（agnostic PAC，"no learner can beat the Bayes error"）。

> **给你的写法**：正文写 "By a two-point argument (cf. Tsybakov 2009, Ch. 2), any estimator suffers $\sup\|\hat f-f\|_\infty\ge D/2$; the plug-in interpolant achieves $D$, hence the minimax rate is $\Theta(D)$ and does not vanish as $n\to\infty$."

---

## 4. P4：随机零均值噪声——这时才有一致性

| 结论 | 定理级出处 |
|---|---|
| 最优全局速率 $n^{-\frac{2\beta}{2\beta+d}}$（$\beta$-Hölder，$d$ 维） | C. J. Stone (1982), *Optimal global rates of convergence for nonparametric regression*, **Ann. Statist.** 10(4):1040–1053 |
| 同速率的下界 | Ibragimov & Has'minskii (1981)；Stone (1982) 的下界部分 |
| 教材（含上/下界、分布无关） | L. Györfi, M. Kohler, A. Krzyżak, H. Walk (2002), *A Distribution-Free Theory of Nonparametric Regression*, Springer, **Ch. 3**【库内被 386、2157 引用】 |
| **专门对神经网络**的逼近+估计分解，风险 $O(1/\sqrt n)$ | A. R. Barron (1994), *Approximation and estimation bounds for artificial neural networks*, **Machine Learning** 14:115–133（其 1993 IT 版见【库内 1674/2105/2518/3215】） |
| ReLU 深度网络的 minimax 最优速率（composition 假设下） | J. Schmidt-Hieber (2020), *Nonparametric regression using deep neural networks with ReLU activation function*, **Ann. Statist.** 48(4):1875–1897 |
| 深度网络克服维数灾难（上界） | B. Bauer & M. Kohler (2019), *On deep learning as a remedy for the curse of dimensionality in nonparametric regression*, **Ann. Statist.** 47(4):2269–2300 |
| 预测误差的 $\sigma^2$ 地板 | Hastie, Tibshirani, Friedman, *ESL* 2nd ed., **§7.3**（$E[(y-\hat f)^2]={\rm Var}(\varepsilon)+E[(\hat f-E\hat y)^2]$） |

**关键区分（写论文时务必分开报）**：
- **估计误差** $\|\hat f-f\|$：随机噪声下 → 0（相合）。
- **预测误差** $\|y_{\rm new}-\hat f(x)\|$：始终 $\ge\sigma$，这是噪声本身的方差，不是估计失败。

---

## 5. P6：闭环/误差变量（EIV）偏差

**机理**：若 $x$ 由被控对象产生、而 $d$ 进入对象状态，则 $\mathbb E[d\mid x]\neq0$，回归目标变成 $\mathbb E[y\mid x]=f(x)+\mathbb E[d\mid x]\neq f(x)$，偏差不可由 $n\to\infty$ 消除（是**渐近偏差**，不是方差）。

| 出处 | 内容 |
|---|---|
| U. Forssell & L. Ljung (1999), *Closed-loop identification revisited*, **Automatica** 35(7):1215–1241 | 闭环数据偏差的经典分析；**【库内 3227/3747 的词条直接引用了它】** |
| T. Söderström (2007), *Errors-in-variables methods in system identification*, **Automatica** 43(6):939–958 | EIV 综述（回归量带噪 ⇒ 衰减偏置） |
| L. Ljung, *System Identification: Theory for the User*, 2nd ed., Prentice Hall 1999 | "bias distribution"：估计收敛到使**滤波后噪声谱**最小化的参数（章节号按你手上版本核对） |
| J. Schoukens & L. Ljung (2019), *Nonlinear system identification: a user-oriented road map*, **IEEE Control Systems Magazine** 39(4):24–44 | 非线性（含 NN）辨识中的噪声/闭环偏差与实验设计 |
| 【库内 3227 / 3747】*Encyclopedia of Systems and Control*, "System Identification" 词条 | 明文写："the noisy output measurements are fed back to the input of the system which creates **a dependency between the input and output disturbance**"（可直接引用为二次文献，但正式引用应落到 Forssell–Ljung） |

**补救**（同样有文献支撑）：开环/激励设计（Schoukens–Ljung 2019）、工具变量（Söderström 2007）、多实验独立噪声拼接（Forssell–Ljung 1999 的 dual-Youla / 两阶段法）。

---

## 6. P7：为什么"慢变 $d$"会被吸进 $\hat f$（谱偏置）

- N. Rahaman et al. (2019), *On the spectral bias of neural networks*, **ICML**（PMLR 97:5301–5310）：网络优先拟合低频成分。
- Z.-Q. J. Xu, Y. Zhang, Y. Xiao (2019), *Frequency principle: Fourier analysis sheds light on deep neural networks*, arXiv:1901.06523（后发表于 Comm. Comput. Phys.）——"F-Principle"：训练先拟合低频。
- M. Tancik et al. (2020), *Fourier features let networks learn high frequency functions in low dimensional domains*, **NeurIPS**——反证：不加 Fourier 特征则高频学不动。**【库内 3304 引用了它，并明确用了 "spectral bias" 一词】**
- 与系统辨识一致：Ljung (1999) 的 bias distribution（估计吸收与回归量相关的那部分扰动）。

> 严谨性提示：这是"倾向性/经验 + 部分理论"，写作时用 "spectral bias suggests…"，不要写成定理。

---

## 7. P8：控制里的直接后果（NN + 有界残差 ⇒ UUB）

- P. M. Patre, W. MacKunis, K. Kaiser, W. E. Dixon (2008), *Asymptotic tracking for uncertain dynamic systems via a multilayer neural network feedforward and RISE feedback control structure*, **IEEE TAC** 53(5):1275–1281 **【库内 2449】**。原文："NN-based controllers yield global uniformly ultimately bounded (UUB) stability results … due to **residual functional reconstruction inaccuracies**"。同文档 **Remark 1** 给出 $\|\varepsilon\|<\varepsilon_N$ 的来源说明（引 Stone–Weierstrass）。
- B. Xian, D. M. Dawson, M. S. de Queiroz, J. Chen (2004), *A continuous asymptotic tracking control strategy for uncertain nonlinear systems*, **IEEE TAC** 49(7):1206–1211（RISE：用积分项吃掉有界残差）。
- 同族陈述在【库内 983】【库内 3111】重复出现（"NNs are universal approximators … UUB due to residual … inaccuracies"）。
- 反例路线（明确提出不依赖万能逼近器）：【库内 478】【库内 3499】。

---

## 8. 可直接放进论文的 LaTeX 片段

```latex
\begin{lemma}[Bounded-noise identifiability floor]
Let $y_i=f(x_i)+d_i$ with $|d_i|\le D$, and let $\mathcal F$ contain $0$ and $D\mathbf 1$.
Then for any estimator $\hat f$,
\[
  \sup_{f\in\mathcal F}\sup_{|d|\le D}\|\hat f-f\|_\infty \ge D/2 ,
\]
while the interpolating estimator $\hat f(x_i)=y_i$ achieves $\|\hat f-f\|_\infty\le D$
on the sample points. Consequently the minimax error is $\Theta(D)$ and does \emph{not}
vanish as the sample size grows.
\end{lemma}
\begin{proof}
The pairs $(f,d)=(0,0)$ and $(D,-D)$ generate identical data, hence identical estimates;
the triangle inequality yields $\max_{i\in\{1,2\}}\|\hat f-f_i\|_\infty\ge\|f_1-f_2\|_\infty/2=D/2$.
The upper bound follows from $|\hat f(x_i)-f(x_i)|=|d_i|\le D$.
\end{proof}
```

```latex
\begin{lemma}[Uniform approximation floor for discontinuous targets]
Let $K$ be compact, $f:K\to\mathbb R$ bounded, and
$\mathrm{osc}(f,x)=\inf_{\delta>0}\mathrm{diam}\,f(B(x,\delta))$. Then
\[
  \inf_{g\in C(K)}\|f-g\|_\infty=\tfrac12\sup_{x\in K}\mathrm{osc}(f,x).
\]
In particular, if $f$ has a jump of size $\Delta$ at some $x_0$, every continuous
approximant --- hence every feedforward network with continuous activation ---
satisfies $\|\hat f-f\|_\infty\ge \Delta/2$.
\end{lemma}
```

---

## 9. 缺口提示：okb-assist 里**没有**这些书

检索确认库内缺失（`search_info` 与 `grep_search` 均无命中，或仅被他文引用）：

| 缺失 | 为什么需要 | 建议补入 |
|---|---|---|
| Tsybakov, *Introduction to Nonparametric Estimation* (2009) | 两点法/minimax 下界的标准框架 | 高 |
| Györfi et al., *A Distribution-Free Theory of Nonparametric Regression* (2002) | 非参回归速率、分布无关界 | 高 |
| Ljung, *System Identification* 2nd ed. (1999) | 偏差分布、闭环偏差 | 高 |
| Anthony & Bartlett, *Neural Network Learning: Theoretical Foundations* (1999) | NN 的估计误差/VC 界 | 中 |
| Stone (1982) Ann. Statist. | 最优速率原典 | 中 |
| Kearns–Li (1993) / Kearns–Schapire–Sellie (1994) | 恶意噪声/agnostic 的误差地板 | 中 |
| Schmidt-Hieber (2020) Ann. Statist. | ReLU 网络 minimax 速率 | 中 |

> 建议：把上表前 4 本 PDF 补进 okb-assist，之后同类问题就能在库内闭环检索（现在只能靠"他文引用"间接碰到 Györfi、Forssell–Ljung）。

---

## 10. 一句话总结（怎么引用最稳）

> "有界扰动下 $f$ 只能被辨识到 $\pm D$（两点法，cf. Tsybakov 2009 Ch. 2；UBB 辨识：Milanese–Vicino 1991），误差阶 $\Theta(D)$ 与样本量无关；随机零均值噪声下才相合，速率 $n^{-2\beta/(2\beta+d)}$（Stone 1982）；连续网络对不连续目标的 sup 误差地板为半振荡（Rudin Thm 7.12 + Katětov–Tong）；闭环数据另有不可消除的偏差（Forssell–Ljung 1999）。"
