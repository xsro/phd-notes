# Cai et al. (2017) —— Lemma 3 中 `x̄` 有界性与收敛性的推导

> 来源：He Cai, Frank L. Lewis, Guoqiang Hu, Jie Huang,
> *The adaptive distributed observer approach to the cooperative output regulation of linear multi-agent systems*,
> Automatica 75 (2017), 299–305。
>
> 本笔记对应原文 Lemma 3 的证明，重点解释方程 (16)–(19)，以及原文中
> “Then, `x̄` is bounded by Remark 2” 这一步为什么需要补充说明。

## 0. 结论先行

1. `PᵀAᵀAP` 一般**不是 Hurwitz**。由 SVD 分解，
   \[
   P^T A^T A P
   =
   \begin{bmatrix}
   \bar A^T\bar A & 0\\
   0 & 0_{n-k}
   \end{bmatrix},
   \]
   其中 `\bar Aᵀ\bar A > 0`，但后 `n-k` 个特征值为零。
2. 因此，不能直接对整个 `x̄` 使用论文 Lemma 1 或 Remark 2；它们要求对应的标称矩阵 Hurwitz。
3. `x̄` 的有界性应通过值域与零空间的分块分析得到：
   - 值域分量受到 `-\varepsilon\bar A^T\bar A` 的指数稳定作用；
   - 零空间分量的极限系统是积分器，但其驱动项是指数衰减的，因此收敛到某个常数。
4. 只有在先证明 `x̄` 有界之后，才能由
   \[
   d(t)=\varepsilon P^T(A^TA-\mathcal A^T\mathcal A)P\bar x
   +\varepsilon P^T\tilde A^Tb
   \]
   推出 `d(t)` 指数趋于零。若反过来用 `d(t)→0` 证明 `x̄` 有界，就会形成循环论证。

## 1. Lemma 3 的设定

设
\[
A\in\mathbb R^{m\times n},\qquad b\in\mathbb R^m,
\]
并满足
\[
\operatorname{rank}(A)=\operatorname{rank}(A,b)=k.
\]
因此线性方程 `Ax=b` 一致可解，但当 `k<n` 时，解一般不唯一。

令 `\mathcal A(t)` 为有界、分段连续的时变矩阵，并定义
\[
\tilde A(t)\triangleq \mathcal A(t)-A.
\]
假设 `\tilde A(t)` 以速率 `\alpha>0` 指数趋于零，即存在常数 `c_A>0` 使
\[
\|\tilde A(t)\|\leq c_Ae^{-\alpha(t-t_0)},\qquad t\geq t_0.
\]

考虑系统
\[
\dot x=-\varepsilon\mathcal A(t)^T\bigl(\mathcal A(t)x-b\bigr),
\qquad \varepsilon>0. \tag{12}
\]

目标是证明：对任意初值，解在 `[t_0,\infty)` 上有界，并且存在某个 `x^*` 满足
\[
Ax^*=b,\qquad x(t)-x^*\to0
\]
指数收敛。这里的 `x^*` 可以依赖于初值；当 `A` 不满列秩时，不能预先指定唯一的解。

## 2. SVD 分解与 `PᵀAᵀAP` 的结构

取正交矩阵 `P=[P_1\;P_0]\in\mathbb R^{n\times n}`，使得
\[
AP=
\begin{bmatrix}
\bar A & 0_{m\times(n-k)}
\end{bmatrix},
\qquad
\bar A\in\mathbb R^{m\times k}.
\]

由于 `\operatorname{rank}(A)=k`，矩阵 `\bar A` 列满秩，因此
\[
G\triangleq\bar A^T\bar A>0.
\]
于是
\[
P^TA^TAP=
\begin{bmatrix}
G&0\\
0&0_{n-k}
\end{bmatrix}. \tag{13}
\]

同时，由 `AP_0=0` 可得
\[
P^TA^Tb=
\begin{bmatrix}
\bar A^Tb\\
0
\end{bmatrix}. \tag{14}
\]

所以，`PᵀAᵀAP` 只是半正定矩阵，而不是 Hurwitz 矩阵。只有在 `k=n` 时，它才没有零特征值。

由一致性条件，`b\in\operatorname{Range}(A)=\operatorname{Range}(\bar A)`。因此存在唯一的
\[
\bar x_1^*=G^{-1}\bar A^Tb
\]
满足
\[
\bar A\bar x_1^*=b.
\]
令任意 `\bar x_2^*\in\mathbb R^{n-k}`，并定义
\[
x^*=P
\begin{bmatrix}
\bar x_1^*\\
\bar x_2^*
\end{bmatrix}.
\]
则 `Ax^*=b`。

## 3. 方程 (16) 的推导

令
\[
\bar x=P^Tx.
\]
将系统 (12) 左乘 `Pᵀ`，并在右端加一项、减一项 `\varepsilon P^TA^TAx`，得到
\[
\begin{aligned}
\dot{\bar x}
&=-\varepsilon P^T\mathcal A^T\mathcal A x
  +\varepsilon P^T\mathcal A^Tb\\
&=-\varepsilon P^TA^TAP\bar x
  +\varepsilon P^TA^Tb+d(t), \tag{16}
\end{aligned}
\]
其中
\[
d(t)
=\varepsilon P^T(A^TA-\mathcal A^T\mathcal A)P\bar x
 +\varepsilon P^T\tilde A^Tb. \tag{D}
\]

注意
\[
\begin{aligned}
A^TA-\mathcal A^T\mathcal A
&=A^TA-(A+\tilde A)^T(A+\tilde A)\\
&=-A^T\tilde A-\tilde A^TA-\tilde A^T\tilde A.
\end{aligned}
\]
因此，`AᵀA-\mathcal Aᵀ\mathcal A` 以速率 `\alpha` 指数趋于零。

但是，式 (D) 中还含有 `\bar x(t)`。所以目前只能说：

- 若已知 `\bar x` 有界，则 `d(t)` 有界并以速率 `\alpha` 指数趋于零；
- 不能先假定 `d(t)→0`，再用它证明 `\bar x` 有界。

## 4. 为什么不能直接套用 Remark 2

论文 Remark 2 的结论适用于形如
\[
\dot z=\varepsilon Fz+F_2(t)
\]
且 `F` Hurwitz 的系统。

在这里，标称矩阵为
\[
-P^TA^TAP
=-
\begin{bmatrix}
G&0\\
0&0
\end{bmatrix},
\]
它含有零特征值。因此不能把整个 `\bar x` 当作 Remark 2 中的 `z` 直接处理。

论文中的

> Then, `\bar x` is bounded by Remark 2

省略了零空间分量的有界性证明。严格的证明应先对值域和零空间进行分块，或者等价地对误差 `x-x^*` 进行投影分解。

## 5. 非循环的有界性证明

### 5.1 以一个精确解为中心化变量

取上面构造的任意解 `x^*`，满足 `Ax^*=b`，并令
\[
z=P^T(x-x^*)
=\operatorname{col}(z_1,z_2).
\]
由于
\[
\mathcal A(t)x^*-b
=(A+\tilde A(t))x^*-b
=\tilde A(t)x^*,
\]
可得
\[
\dot z
=-\varepsilon C(t)z+r(t), \tag{C}
\]
其中
\[
C(t)\triangleq P^T\mathcal A(t)^T\mathcal A(t)P,
\qquad
r(t)\triangleq-\varepsilon P^T\mathcal A(t)^T\tilde A(t)x^*.
\]

因为 `\mathcal A(t)→A` 指数快速，且 `\mathcal A(t)` 有界，
\[
C(t)\to
C_\infty\triangleq
\begin{bmatrix}
G&0\\
0&0
\end{bmatrix},
\qquad
\|C(t)-C_\infty\|+\|r(t)\|
=O(e^{-\alpha t}). \tag{C1}
\]

将 `C(t)` 按照 `z=\operatorname{col}(z_1,z_2)` 分块：
\[
\begin{aligned}
\dot z_1&=-\varepsilon C_{11}(t)z_1
             -\varepsilon C_{12}(t)z_2+r_1(t),\\
\dot z_2&=-\varepsilon C_{21}(t)z_1
             -\varepsilon C_{22}(t)z_2+r_2(t). \tag{C2}
\end{aligned}
\]

### 5.2 先证明 `z` 有界

取足够大的 `T`，使得对所有 `t\geq T`，
\[
C_{11}(t)\geq \frac{1}{2}G>0.
\]
记
\[
a\triangleq\frac{\varepsilon}{2}\lambda_{\min}(G)>0.
\]
由变分常数公式和 (C1)，存在常数 `c>0` 使得
\[
\begin{aligned}
\|z_1(t)\|
&\leq ce^{-a(t-T)}\|z_1(T)\|\\
&\quad+c\int_T^t e^{-a(t-s)}e^{-\alpha(s-T)}
\bigl(\|z_2(s)\|+1\bigr)\,ds,\\
\|z_2(t)\|
&\leq \|z_2(T)\|
+c\int_T^t e^{-\alpha(s-T)}
\bigl(\|z_1(s)\|+\|z_2(s)\|+1\bigr)\,ds.
\end{aligned} \tag{C3}
\]

由于 `e^{-\alpha(t-T)}` 可积，(C3) 配合 Gronwall 不等式给出
\[
\sup_{t\geq T}\bigl(\|z_1(t)\|+\|z_2(t)\|\bigr)<\infty.
\]
在有限区间 `[t_0,T]` 上，这是一个系数分段连续的线性系统，不会发生有限时间逃逸，因此 `z(t)` 也有界。于是
\[
z(t)\ \text{以及}\ x(t)=x^*+Pz(t)\ \text{均有界}. \tag{B}
\]

这一步补上了论文原证明中被 Remark 2 一句话略去的关键环节。

### 5.3 再证明收敛速度

由 (B)，式 (D) 中的 `\bar x` 有界，因此
\[
\|d(t)\|\leq c_de^{-\alpha(t-t_0)}.
\]
于是 (16) 的分块形式为
\[
\dot{\bar x}_1
=-\varepsilon G\bar x_1+\varepsilon\bar A^Tb+d_1(t), \tag{17a}
\]
\[
\dot{\bar x}_2=d_2(t). \tag{17b}
\]

对第一式令
\[
\tilde x_1=\bar x_1-\bar x_1^*.
\]
因为 `-G` Hurwitz，
\[
\dot{\tilde x}_1=-\varepsilon G\tilde x_1+d_1(t),
\]
从而
\[
\tilde x_1(t)\to0
\]
指数收敛。对任意 `\varepsilon>0`，其收敛速率为某个正数；若
\[
\varepsilon>
\frac{\alpha}{\lambda_{\min}(G)}, \tag{R}
\]
则可以使值域分量至少以速率 `\alpha` 收敛。

再看零空间分量。由 `d_2(t)=O(e^{-\alpha t})`，
\[
\bar x_2(t)
=\bar x_2(t_0)+\int_{t_0}^t d_2(\tau)\,d\tau
\]
收敛到某个有限常数 `\bar x_2^*`，且
\[
\bar x_2(t)-\bar x_2^*=O(e^{-\alpha t}).
\]

因此取
\[
x^*=P
\begin{bmatrix}
\bar x_1^*\\
\bar x_2^*
\end{bmatrix},
\]
就有
\[
Ax^*=b,\qquad
x(t)-x^*
=P
\begin{bmatrix}
\bar x_1(t)-\bar x_1^*\\
\bar x_2(t)-\bar x_2^*
\end{bmatrix}
\to0
\]
指数收敛。这就是原文式 (18)–(19) 的结论。

## 6. 两种分块视角的关系

直接对 `\bar x=P^Tx` 分块，可以得到
\[
\dot{\bar x}_1
=-\varepsilon G\bar x_1+\varepsilon\bar A^Tb+d_1(t),
\qquad
\dot{\bar x}_2=d_2(t).
\]
这有助于说明：

- `\bar x_1` 是由正定矩阵 `G` 稳定的值域分量；
- `\bar x_2` 属于 `\ker A`，标称系统中没有恢复力；
- `\bar x_2` 是否有界，取决于 `d_2` 是否可积。

但由于 `d_2` 的定义中含有 `\bar x`，这组方程单独使用时会暴露出循环。以 `x-x^*` 为中心化变量后，先用 (C2)–(C3) 得到有界性，再推出 `d(t)` 指数衰减，逻辑更加完整。

## 7. 与 Lemma 4 的衔接

Lemma 4 中令
\[
Q_i(t)
=S_i(t)^T\otimes
\begin{bmatrix}
I&0\\
0&0
\end{bmatrix}
-I_q\otimes
\begin{bmatrix}
A_i&B_i\\
C_i&D_i
\end{bmatrix},
\]
以及
\[
b_i=\operatorname{vec}
\begin{bmatrix}
E_i\\
F_i
\end{bmatrix}.
\]

由 Lemma 2，`\tilde S_i(t)=S_i(t)-S` 指数趋于零，因此
\[
Q_i(t)\to Q_i
\]
也指数趋于零。Assumption 3 保证极限方程
\[
Q_i\chi_i^*=b_i
\]
可解。于是可以将 Lemma 3 应用于
\[
\mathcal A(t)=Q_i(t),\qquad A=Q_i,\qquad b=b_i.
\]
得到在线求解器 (22) 的解 `\zeta_i(t)` 有界，并收敛到某个满足
`Q_i\chi_i^*=b_i` 的解。若增益 `\mu_3` 足够大，则收敛速率至少可以达到 `\tilde S_i(t)` 的指数衰减速率。

## 8. 关键提醒

- `PᵀAᵀAP` 一般不是 Hurwitz，而是 `\operatorname{diag}(\bar Aᵀ\bar A,0)`。
- 不能直接对整个 `\bar x` 使用 Lemma 1 或 Remark 2。
- `d(t)→0` 需要以 `\bar x` 有界为前提，不能用它反过来证明 `\bar x` 有界。
- 零空间分量不是由 Hurwitz 稳定性控制，而是因为其驱动项可积，最终收敛到常数。
- 当 `A` 不满列秩时，极限解 `x^*` 通常不唯一，并且会依赖于初始条件。
- 若只要求指数收敛，任意 `\varepsilon>0` 足够；若要求收敛速率至少为 `\alpha`，需要满足类似 (R) 的条件。
