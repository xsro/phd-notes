# 为什么 $\mu_1 > \gamma / \alpha_H$ 让交叉项 $\tilde{S}_d(1_N \otimes v)$ 指数衰减，从而满足 Lemma 1 的 $F_2(t) \to 0$

> 论文：He Cai, Frank L. Lewis, Guoqiang Hu, Jie Huang,
> *"The adaptive distributed observer approach to the cooperative output regulation of linear multi-agent systems"*, Automatica, 2016.
> 对应本地文件 `3343.md`。
>
> 本文只解释**一个**问题：条件 $\mu_1 > \gamma / \alpha_H$ 如何使交叉项 $\tilde{S}_d(1_N \otimes v)$ 指数衰减到 0，从而正好满足 Lemma 1 收敛证明中对 $F_2(t) \to 0$（指数）的要求。关于 $\mu_2 > \gamma / \alpha_H$ 如何让齐次矩阵 Hurwitz 的讨论，见文末附记，不构成本文主线。

---

## 1. 问题：我们要解释什么

在 Lemma 2(ii) 的收敛证明中（`3343.md:168–174`），$\tilde{\eta} = col(\tilde{\eta}_1,\dots,\tilde{\eta}_N)$ 满足

$$
\dot{\tilde{\eta}} = [(I_N \otimes S) - \mu_2(H \otimes I_q)] \tilde{\eta} + \tilde{S}_d \tilde{\eta} + \tilde{S}_d (1_N \otimes v). \tag{11}
$$

论文把 (11) 套用 Lemma 1（`3343.md:124–130`，即系统 $\dot{x} = \epsilon F x + F_1(t)x + F_2(t)$）。要由 Lemma 1 推出 $\tilde{\eta}(t) \to 0$ **指数地**，必须保证两个扰动项都**指数衰减到 0**：

- $F_1(t) = \tilde{S}_d(t)$ $\to$ 由 Lemma 2(i) 直接得到（$\tilde{S}_i \to 0$ 指数），**不需要** $\mu_1 > \gamma/\alpha_H$；
- **$F_2(t) = \tilde{S}_d(t)(1_N \otimes v(t))$ $\to$ 这正是要解释的交叉项**，要用统一的速率估计保证它指数衰减，需取 $\mu_1 > \gamma/\alpha_H$。

本文要证明的，就是下面这句话：

> 若 $\mu_1 \alpha_H - \gamma > 0$，即 $\mu_1 > \gamma / \alpha_H$，则交叉项 $\tilde{S}_d(t)(1_N \otimes v(t))$ 指数衰减到 0，从而满足 Lemma 1 的 $F_2(t)$ 条件。

这里的“不等式等价”只指 $\mu_1 \alpha_H - \gamma > 0 \Longleftrightarrow \mu_1 > \gamma/\alpha_H$。作为对所有初值的统一充分条件，它正好解释原文为什么这样选 $\mu_1$；若初值有特殊消零结构（例如 $v(0)=0$ 或某些模态未被激发），该条件未必是某个具体轨迹的必要条件。

注意 $\mu_1$ **不出现在**齐次矩阵 $(I_N \otimes S) - \mu_2(H \otimes I_q)$ 中；它只通过 $\tilde{S}_d$ 的衰减速率影响交叉项。所以 $\mu_1 > \gamma/\alpha_H$ 与 $\mu_2 > \gamma/\alpha_H$ 职责不同（见 §5 与附记）。

---

## 2. 相关定义与已知估计

先把后面要用到的符号和**已证**估计列清楚（均来自 `3343.md`，此处只引用结论）。

### 2.1 Leader / exosystem 与 $\gamma$

Leader 信号（`3343.md:36`，式 (1)）：

$$
\dot{v} = S v, \qquad v(0) \text{ 给定}.
$$

定义 leader 矩阵 $S$ 的**最大实部增长率**（Lemma 2(ii)，`3343.md:174`）：

$$
\gamma = \max\bigl(\Re(\sigma(S))\bigr).
$$

由常系数线性系统的基本性质，$v(t) = e^{St}v(0)$ 的范数满足

$$
\|v(t)\| \le c \cdot e^{\gamma t} \cdot (1+t^{\,m-1}),
$$

其中 $m$ 可取为相关 Jordan 块的最大尺寸；多项式因子来自非半单（non-semi-simple）Jordan 块。若 $S$ 半单（例如所有 Jordan 块都是 $1\times 1$），则简化为 $\|v(t)\| \le c \cdot e^{\gamma t}$。本文去掉了 Cai & Huang (2016) 的"半单且零实部"假设，因此要保留这个多项式因子。

于是沿全体 follower 堆叠的信号满足

$$
\|1_N \otimes v(t)\| \le c' \cdot e^{\gamma t} \cdot (1+t^{\,m-1})
\quad \bigl(\text{半单时即 } c' \cdot e^{\gamma t}\bigr). \tag{v-bound}
$$

即 $1_N \otimes v$ 的指数增长率至多为 $\gamma$，并可能带一个多项式因子。$\gamma > 0$ 时存在正指数增长模态；$\gamma = 0$ 且 $S$ 非半单时可能出现 ramp 等多项式增长；$\gamma < 0$ 时则指数衰减。

### 2.2 分布式矩阵估计误差 $\tilde{S}_i$ 与 Lemma 2(i)

定义每个 follower 对 $S$ 的估计误差（`3343.md:158`）：

$$
\tilde{S}_i = S_i - S.
$$

由 (5a)（`3343.md:113`）$\dot{S}_i = \mu_1 \sum_{j=0}^N a_{ij}(S_j - S_i)$，把 $\tilde{S} = col(\tilde{S}_1,\dots,\tilde{S}_N)$ 堆叠，可得紧致形式（`3343.md:160`）：

$$
\dot{\tilde{S}} = -\mu_1 (H \otimes I_q) \tilde{S},
\qquad
H = L + \operatorname{diag}(a_{10},\dots,a_{N0}).
$$

其中 $L$ 是 follower 子图 Laplacian，$H$ 因 Assumption 4（含 leader 为根的有向生成树）而**所有特征值实部为正**（`3343.md:160` 引 Hu & Hong 2007 Lemma 4）。定义图连通性 / 谱隙

$$
\alpha_H = \min\bigl(\Re(\sigma(H))\bigr) > 0. \tag{alphaH}
$$

Lemma 2(i)（`3343.md:160`）给出逐块估计：对某个 $\beta_H > 0$，

$$
\|\tilde{S}_i(t)\| \le \beta_H \|\tilde{S}_i(0)\| \, e^{-\mu_1 \alpha_H t}, \qquad i=1,\dots,N. \tag{S-tilde-decay}
$$

即**矩阵估计误差以速率 $\mu_1 \alpha_H$ 指数衰减**——这是关键：$\mu_1$ 越大、图越连通（$\alpha_H$ 越大），衰减越快。

记 $\tilde{S}_d = blockdiag\{\tilde{S}_1,\dots,\tilde{S}_N\}$，由 (S-tilde-decay) 还有整体界

$$
\|\tilde{S}_d(t)\| \le C \, e^{-\mu_1 \alpha_H t}. \tag{S-d-decay}
$$

（常数 $C$ 吸收 $\beta_H$ 与最大初值。）

技术上，如果不直接采用原文 Lemma 2(i) 给出的速率估计，而是从一般非对称矩阵 $H$ 的 Jordan 形式出发，$\tilde{S}_d(t)$ 的上界也可能多出一个多项式因子，或写成任意略慢的速率 $e^{-\mu_1(\alpha_H-\varepsilon)t}$。由于后面使用的是严格不等式 $\mu_1\alpha_H>\gamma$，这些更保守的写法仍能被指数裕度吸收，不改变本文解释的下界形式。

---

## 3. 交叉项的界：把两个速率相乘

现在对交叉项 $\tilde{S}_d(t)(1_N \otimes v(t))$ 取范数，并用次乘性 $\|AB\| \le \|A\|\cdot\|B\|$：

$$
\begin{aligned}
\|\tilde{S}_d(t)(1_N \otimes v(t))\|
&\le \|\tilde{S}_d(t)\| \cdot \|1_N \otimes v(t)\| \\[2mm]
&\le \bigl(C \, e^{-\mu_1 \alpha_H t}\bigr)
   \cdot \bigl(c' \, e^{\gamma t} \, (1+t^{\,m-1})\bigr)
   && \text{代入 (S-d-decay) 与 (v-bound)} \\[2mm]
&= C c' \cdot (1+t^{\,m-1}) \cdot e^{-(\mu_1 \alpha_H - \gamma)\, t}. \tag{cross-bound}
\end{aligned}
$$

这正是要的核心界：**交叉项是"一个指数衰减项"乘以"一个多项式项"**——

- 指数部分的净指数为 $-(\mu_1 \alpha_H - \gamma) t$；
- 多项式部分 $1+t^{m-1}$ 来自 $S$ 的非半单纯性，且**完全不依赖** $\mu_1, \alpha_H, \gamma$。

---

## 4. 何时指数衰减：$\mu_1 \alpha_H - \gamma > 0 \Longleftrightarrow \mu_1 > \gamma / \alpha_H$

判断 (cross-bound) 这个统一上界是否指数趋于 0，只看指数项的净指数符号：

$$
\mu_1 \alpha_H - \gamma \;\begin{cases}
> 0 & \Rightarrow e^{-(\mu_1\alpha_H-\gamma)t} \text{ 指数衰减，且压过多项式因子}; \\[1mm]
= 0 & \Rightarrow \text{统一上界只剩多项式量，一般不能保证衰减}; \\[1mm]
< 0 & \Rightarrow \text{统一上界含正指数增长，一般不能保证衰减}.
\end{cases}
$$

由于 $\alpha_H > 0$（式 (alphaH)），不等式方向在除以 $\alpha_H$ 时不变，故

$$
\boxed{\;\mu_1 \alpha_H - \gamma > 0 \quad \Longleftrightarrow \quad \mu_1 > \frac{\gamma}{\alpha_H}\;}. \tag{core}
$$

这就是本文要说明的**核心不等式** $\mu_1 \alpha_H - \gamma > 0$。它正是 $\mu_1 > \gamma / \alpha_H$。

### 4.1 多项式因子被指数项压制

需确认 (core) 成立时，(cross-bound) 确实指数衰减，即便有 $1+t^{m-1}$。对任意指数衰减率 $\lambda \triangleq \mu_1 \alpha_H - \gamma > 0$，以及任意多项式次数 $m-1$，都有熟知的极限

$$
\lim_{t\to\infty} (1+t^{\,m-1}) e^{-\lambda t} = 0,
$$

且收敛是指数阶的：存在 $C'', \lambda' \in (0, \lambda)$ 使 $(1+t^{m-1}) e^{-\lambda t} \leq C'' e^{-\lambda' t}$。因此 (cross-bound) ⇒

$$
\|\tilde{S}_d(t)(1_N \otimes v(t))\|
\le C''' e^{-\lambda' t} \xrightarrow[t\to\infty]{} 0
\quad\text{指数地}.
$$

换言之，**$S$ 是否半单只改变常数项与多项式尾巴，不改变"净指数为正即可保证交叉项指数衰减"这一结论**。

### 4.2 两种情形

- **$\gamma \leq 0$（S 无正实部特征值）**：此时对任意 $\mu_1 > 0$，
  $\mu_1 \alpha_H - \gamma \geq \mu_1 \alpha_H > 0$，(core) **自动成立**。即便 $\gamma=0$ 时 $v(t)$ 可能有 ramp 这类多项式增长，$e^{-\mu_1\alpha_H t}$ 仍会压过该多项式因子。不需要任何显式下界——这正是 `3343.md:174` 第一句中 "for any positive $\mu$" 的由来。
- **$\gamma > 0$（S 含正指数增长模态）**：必须**显式选取** $\mu_1 > \gamma / \alpha_H$。否则净指数 $\mu_1 \alpha_H - \gamma \leq 0$，上述统一估计无法保证交叉项衰减，$\tilde{\eta} \to 0$ 的证明链条在此处断裂。

---

## 5. 对接 Lemma 1：$F_2(t)$ 就是该交叉项

现在把 (11) 与 Lemma 1 的 $\dot{x} = \epsilon F x + F_1(t)x + F_2(t)$ 逐一对齐（`3343.md:127`，式 (6)）：

| Lemma 1 中的量 | 在 (11) 中的对应 | 说明 |
|---|---|---|
| $x$ | $\tilde{\eta}$ | 待证趋于 0 的误差 |
| $\epsilon F$ | $(I_N \otimes S) - \mu_2(H \otimes I_q)$ | 齐次部分，需 Hurwitz（由 $\mu_2 > \gamma/\alpha_H$ 保证，见附记） |
| $F_1(t)$ | $\tilde{S}_d(t)$ | 需 $\to 0$ 指数 |
| **$F_2(t)$** | **$\tilde{S}_d(t)(1_N \otimes v(t))$** | 需 $\to 0$ 指数 |

- **$F_1(t) = \tilde{S}_d \to 0$ 指数**：直接来自 Lemma 2(i) 的 (S-tilde-decay)，**与 $\mu_1 > \gamma/\alpha_H$ 无关**（对任意 $\mu_1 > 0$ 都成立，见 `3343.md:158`）。所以 $F_1$ 这一项不带来 $\mu_1$ 的下界。
- **$F_2(t) = \tilde{S}_d(1_N \otimes v) \to 0$ 指数**：正是 §3–§4 证明的结论；要用该统一速率估计保证它指数衰减，只需 $\mu_1 \alpha_H - \gamma > 0$，即 $\mu_1 > \gamma / \alpha_H$。

Lemma 1(i) 的结论是：若 $F$ Hurwitz 且 $F_1(t), F_2(t) \to 0$（指数），则对任意初值 $x(t) \to 0$ 指数地。于是只要

1. $\mu_2 > \gamma/\alpha_H$（使 $\epsilon F$ Hurwitz），且
2. $\mu_1 > \gamma/\alpha_H$（使 $F_2(t)$ 指数衰减）；

两项都满足，Lemma 1 即推出 $\tilde{\eta}(t) \to 0$ 指数地，`3343.md:174` 的证明收口。

> 关键区分：$\mu_2$ 决定齐次矩阵是否 Hurwitz；$\mu_1$ 决定交叉项 $F_2$ 是否指数衰减。二者下界形式相同（$> \gamma/\alpha_H$），但职责不同。本文主线只解释 $\mu_1$ 那一侧。

---

## 6. 直觉总结

可以把交叉项理解为**两个相反趋势的乘积**：

- leader 信号 $1_N \otimes v$ 的指数增长率由 $\gamma$ 控制，并可能带多项式因子（ramp 对应 $\gamma=0$ 但有多项式增长）；
- 矩阵估计误差 $\tilde{S}_d$ 按速率 $\mu_1 \alpha_H$ **衰减**，其中 $\mu_1$ 是耦合增益、$\alpha_H$ 是图连通性（谱隙）。

要用统一估计保证乘积 $\tilde{S}_d(1_N \otimes v)$ 归零，必须让**指数衰减快过指数增长**：

$$
\underbrace{\mu_1 \alpha_H}_{\text{衰减速率}} \;>\; \underbrace{\gamma}_{\text{增长速率}}
\quad\Longleftrightarrow\quad
\mu_1 > \frac{\gamma}{\alpha_H}.
$$

- 图越连通（$\alpha_H$ 越大）$\to$ 衰减越快，所需 $\mu_1$ 越小；
- leader 信号增长越快（$\gamma$ 越大）$\to$ 所需 $\mu_1$ 越大；
- 若 leader 无正指数增长（$\gamma \leq 0$），则任意正 $\mu_1$ 都够，因为衰减侧恒为正，并会压过可能出现的多项式增长。

一句话：**$\mu_1 > \gamma/\alpha_H$ 的物理解读是——分布式观测器估计 $S$ 的收敛速度，必须胜过 leader 信号 $v$ 的增长速度，这样残差 $\tilde{S}_d$ 与 $v$ 的乘积才会指数消失，Lemma 1 的 $F_2(t) \to 0$ 条件才满足。**

---

## 附记：关于 $\mu_2 > \gamma / \alpha_H$（对照，非本文主线）

$\mu_2$ 的下界来自齐次矩阵

$$
M = (I_N \otimes S) - \mu_2 (H \otimes I_q)
$$

需为 Hurwitz。由于 $(I_N \otimes S)$ 与 $(H \otimes I_q)$ 作用于不同张量腿而可交换，可同时三角化，$M$ 的特征值为

$$
\sigma(M) = \{\, s_k - \mu_2 h_j : s_k \in \sigma(S),\ h_j \in \sigma(H) \,\},
$$

其实部最大者为 $\gamma - \mu_2 \alpha_H$。$M$ Hurwitz ⇔ $\gamma - \mu_2 \alpha_H < 0$ ⇔ $\mu_2 > \gamma/\alpha_H$（由 $\alpha_H > 0$）。这与 $\mu_1$ 的下界**形式相同但职责不同**：$\mu_2$ 管齐次部分稳定，$\mu_1$ 管交叉项 $F_2$ 衰减。

Remark 4（`3343.md:344`）还给出对应的 $\mu_3$ 下界：

$$
\mu_3 \ge \frac{\mu_1 \alpha_H}{\min\sigma(Q_i^\top Q_i)}
> \frac{\gamma}{\min\sigma(Q_i^\top Q_i)},
$$

其中 $Q_i$ 为 $\eta_i$ 动力学 (3) 的可观测性矩阵（Assumption 3 满秩）。

---

## 半单是什么意思

这里的“半单”（semi-simple）说的是矩阵特征值的代数重数和几何重数相同，也就是每个特征值对应的 Jordan 块都只有 $1\times 1$，没有长度大于 1 的 Jordan 链。直观上讲，矩阵虽然可能有重复特征值，但不会带来“特征向量不够用”的退化。

它之所以重要，是因为线性系统

$$
\dot{x} = Sx
$$

的解是 $e^{St}x(0)$。如果 $S$ 半单，那么 $e^{St}$ 的增长/衰减基本就是纯指数型，由特征值实部决定；如果 $S$ 非半单，Jordan 块会额外带出 $t,t^2,\dots$ 这样的多项式因子。所以本文前面才要把 $v(t)$ 写成“指数项乘一个多项式项”的统一上界。

更具体地看，一个大小为 $r$ 的 Jordan 块可以写成

$$
J = \lambda I + N,
$$

其中 $\lambda$ 是特征值，$N$ 是只有上超对角线为 1 的幂零矩阵，例如

$$
N =
\begin{bmatrix}
0 & 1 & 0 & \cdots & 0 \\
0 & 0 & 1 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & 1 \\
0 & 0 & 0 & \cdots & 0
\end{bmatrix},
\qquad N^r = 0.
$$

因为 $\lambda I$ 和 $N$ 可交换，所以

$$
e^{Jt}
= e^{(\lambda I+N)t}
= e^{\lambda t} e^{Nt}.
$$

而 $N^r=0$，指数级数在第 $r-1$ 阶后就截断：

$$
e^{Nt}
= I + tN + \frac{t^2}{2!}N^2 + \cdots + \frac{t^{r-1}}{(r-1)!}N^{r-1}.
$$

于是

$$
e^{Jt}
= e^{\lambda t}
\left(
I + tN + \frac{t^2}{2!}N^2 + \cdots + \frac{t^{r-1}}{(r-1)!}N^{r-1}
\right).
$$

这就是多项式因子的来源：长度为 $r$ 的 Jordan 链最多会带出 $t^{r-1}$。例如

$$
J =
\begin{bmatrix}
\lambda & 1 \\
0 & \lambda
\end{bmatrix}
\quad\Rightarrow\quad
e^{Jt}
= e^{\lambda t}
\begin{bmatrix}
1 & t \\
0 & 1
\end{bmatrix},
$$

所以会出现 $t e^{\lambda t}$；若 Jordan 块大小为 3，则会出现 $\frac{t^2}{2}e^{\lambda t}$。这也是为什么 $\gamma=0$ 时，非半单的零特征值会产生 ramp、多项式参考信号：指数部分 $e^{0t}=1$ 不增长，但 Jordan 块留下了 $t,t^2,\dots$。

在这篇笔记里，你可以把“$S$ 半单”简单理解成：

- 没有额外的 Jordan 多项式因子；
- 信号行为更干净，只有指数型主导；
- 一旦放松这个假设，就要把多项式尾巴一起算进去。
