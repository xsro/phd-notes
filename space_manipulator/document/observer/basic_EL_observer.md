# 定理 1：(25 in okb-883) 观测 (24 in okb-883)

本文件只做一件事：证明文献 `okb-883`(Remark 7) 的扰动观测器 (25) 能够观测系统 (24)，并把证明所需的假设、增益条件、误差方程和常值扰动情形下的收敛结论写完整。

## 1. 原文陈述与记号

考虑 (24 in okb-883)
$$
M _ { i } ( q _ { i } ) \ddot { q } _ { i } + C ( q _ { i } , \dot { q } _ { i } ) \dot { q } _ { i } + g ( q _ { i } ) = u _ { i } + d _ { i }
\tag{24 in okb-883}
$$
及其观测器 (25 in okb-883)
$$
\begin{array} { r } { \left\{ \begin{array} { l } { \dot { z } _ { i } = - L _ { \mathrm { d } } ( q _ { i } , \dot { q } _ { i } ) z _ { i } + L _ { d } ( q _ { i } , \dot { q } _ { i } ) \big [ C _ { i } ( q _ { i } , \dot { q } _ { i } ) \dot { q } _ { i } + g ( q _ { i } ) } \\ { \qquad - \tau _ { i } - p _ { i } ( q _ { i } , \dot { q } _ { i } ) \big ] } \\ { \qquad \dot { d } _ { i } = z _ { i } + p _ { i } ( q _ { i } , \dot { q } _ { i } ) } \end{array} \right. } \end{array}
\tag{25 in okb-883}
$$
其中 $z_i$ 是内部辅助状态，观测器增益 $L_i(q_i,\dot q_i)$ 与辅助函数 $p_i(q_i,\dot q_i)$ 需满足
\[
L_i(q_i,\dot q_i)M(q_i)\ddot q_i
=\left[\frac{\partial p_i}{\partial q_i}\ \ \frac{\partial p_i}{\partial\dot q_i}\right]
\begin{bmatrix}\dot q_i\\ \ddot q_i\end{bmatrix}
\tag{25a}
\]
（原文该式 OCR 有损，其可实现的实质内容是 $\partial p_i/\partial\dot q_i=L_iM$，见 §4）。原文称：由 Chen 等与 Mohammadi 等的工作可知，在适当增益选取下扰动估计误差指数收敛到零。原文没有给出增益选取方式与误差方程。

- `okb-883` Flexible distributed fencing control for Euler–Lagrange multi-agent systems with moving target
- `okb-2985` W. H. Chen, D. J. Ballance, P. J. Gawthrop, and J. O'Reilly, A nonlinear disturbance observer for robotic manipulators, IEEE Trans. Ind. Electron. 47 (2000), no. 4, 932–938.
- `okb-2985` A. Mohammadi, H. J. Marquez, and M. Tavakoli, Nonlinear disturbance observers: Design and applications to Euler–Lagrange systems, IEEE Control Syst. 37 (2017), no. 4, 50–72.

## 2. 假设与动量通道

对 (24) 作如下标准假设（全部为证明所必需，缺一不可）：

- **(A1)** $M\in C^1$，$C,g$ 已知且足够光滑；沿所讨论轨迹可计算 $M(q)$、$C(q,\dot q)\dot q$、$g(q)$ 及 $\dot M(q,\dot q)=\sum_j(\partial M/\partial q_j)\dot q_j$。
- **(A2)** $q,\dot q$ 与**实际施加**的输入 $u$ 可测（因而 $\tau=u-g(q)$ 可算）；不测 $\ddot q$；等式针对精确模型与理想连续时间信号。
- **(A3)** 扰动为常值：$d\in\mathbb R^n$，$\dot d=0$。
- **(A4)** 增益对称正定。

记 $q=q_m$。为简化记号，令 $\tau:=u-g(q)$（即把 (24) 中的重力项并入输入；此处 $u$ 是 (24) 的输入），于是 (24) 等价于
\[
M(q)\ddot q+C(q,\dot q)\dot q=\tau+d ,
\tag{1}
\]
其中已用 (A3) 把 $d(t)$ 写成常数 $d$。定义可测的广义动量与已知输入
\[
y:=M(q)\dot q,\qquad
u_p:=\tau-C(q,\dot q)\dot q+\dot M(q,\dot q)\dot q
=u-g(q)-C\dot q+\dot M\dot q .
\tag{2}
\]
由乘积法则与 (1)，对几乎所有 $t$
\[
\dot y=\dot M\dot q+M\ddot q=u_p+d .
\tag{3}
\]
**这就是 (25) 成立的全部机制**：被估计的量直接是原扰动力矩 $d$，而不是 $M^{-1}d$；因此 $d$ 为常值时并不要求 $M$ 为常数。以下运算逐分量进行，也可把增益写成 $l_iI_n$ 的向量形式。

## 3. 定理 1 与证明

**定理 1（(25) 观测 (24)）。** 设 (A1)–(A4) 成立。取常数增益矩阵
\[
L=\operatorname{diag}(l_1,\dots,l_n),\qquad l_1,\dots,l_n>0,\qquad \underline\lambda:=\min_i l_i ,
\]
并按
\[
\boxed{
\begin{aligned}
 p&=L\,y=L\,M(q)\dot q,\\
 \hat d&=z+p=z+L\,M(q)\dot q,\\
 \dot z&=-Lz-L\big(p+u_p\big)
\end{aligned}}
\tag{8}
\]
构造观测器，$z(0)$ 任意有限。则估计误差 $e:=\hat d-d$ 满足
\[
\dot e=-Le,\qquad e(t)=e^{-Lt}e(0),\qquad
\|\hat d(t)-d\|\le e^{-\underline\lambda t}\|\hat d(0)-d\|,
\qquad
|\hat d_i(t)-d_i|\le e^{-l_it}|\hat d_i(0)-d_i| .
\tag{9}
\]
即 (25 in okb-883) 指数地观测 (24 in okb-883)：误差以任意快的指数速率 $\underline\lambda$ 收敛到零，界与初值有关而与轨迹无关；全程只用到 $q,\dot q,\tau$，不需要 $\ddot q$，也不需要 $M$ 为常数或 $M^{-1}$ 可解析求逆。

**证明。** 先说明可实现性：(8) 中 $p=L y$ 与 $u_p$ 均由可测的 $q,\dot q,\tau$ 和已知模型算出，右端不出现 $\ddot q$，故 (8) 是合法的因果观测器。再算误差动态。由 $\hat d=z+p$、(2)、(3)，
\[
\dot{\hat d}=\dot z+L\dot y=-Lz-L(p+u_p)+L(u_p+d)=-L(z+p)+Ld=-L(\hat d-d).
\]
由 (A3) $\dot d=0$，故
\[
\dot e=\dot{\hat d}-\dot d=-Le .
\]
线性定常系统 $\dot e=-Le$ 的解为 $e(t)=e^{-Lt}e(0)$。因 $L$ 对称正定对角，其特征值为 $l_i$，故 $\|e^{-Lt}\|\le e^{-\underline\lambda t}$，且逐分量 $|e_i(t)|=e^{-l_it}|e_i(0)|$，即 (9)。右端对任意有限 $e(0)$ 有限，并在 $t\to\infty$ 时趋零。证毕。

**注。** 证明的关键是 (8) 中 $\dot z$ 的 $-Lu_p$ 与 $L\dot y$ 的 $+Lu_p$ 精确抵消，因而全部模型项（含 $\dot M\dot q$ 与 $\ddot q$）都不进入误差方程，只剩 $-L(\hat d-d)$。这正对应 (25a) 增益条件要达到的效果；若按 (25) 的字面形式实现，该项不会自动消失，见 §4。

## 4. 用原文变量写出的等价形式：(25) 中缺一项

取 (25) 的变量与条件 $\partial p/\partial\dot q=LM$，$p=p(q,\dot q)$；注意 (25) 中的 $\tau$ 就是 (24) 中的 $u$。由 (24) 解出 $M\ddot q=u+d-C\dot q-g$，得
\[
y=M\dot q,\qquad \dot y=\big[u-C\dot q-g\big]+d+\dot M\dot q ,
\]
且 $\big[C\dot q+g-u\big]=-\big[u-C\dot q-g\big]$ 正是 (25) 方括号内去掉 $p$ 的部分。由于本文的 $\tau=u-g$，故 $u_p$ 与它相差 $\dot M\dot q$：
\[
-L\big(p+u_p\big)=L\big[C\dot q+g-u\big]-L\,p-L\dot M(q,\dot q)\dot q .
\]
因此 (8) 等价于
\[
\dot z=-Lz+L\Big[C(q,\dot q)\dot q+g(q)-u\Big]-L\,p-L\dot M(q,\dot q)\dot q,
\qquad \hat d=z+p ,
\tag{8$'$}
\]
它与原文 (25) 的差别只有最后一项 $-L\dot M\dot q$。

**该项不能省略。** 照抄 (25)（即丢掉 $-L\dot M\dot q$）并用真实的 $\dot p=\frac{\partial p}{\partial q}\dot q+LM\ddot q$ 计算，得
\[
\dot{\hat d}=\dot z+\dot p=-L(\hat d-d)+\frac{\partial p}{\partial q}\dot q
\ \Longrightarrow\
\dot e=-Le+\frac{\partial p}{\partial q}\dot q .
\]
取 $p=LM\dot q$ 时 $\frac{\partial p}{\partial q}\dot q=L\dot M(q,\dot q)\dot q$，即上一段的缺项。于是除非常值扰动下 $\dot M\dot q\equiv0$（例如 $M$ 为常数），误差方程中始终含有非零的模型项，只能得到最终界 $\sup_t\|\dot M\dot q\|/\underline\lambda$，无法断言 $\hat d\to d$。

**缺项的来源。** 原文的推导实际使用了 $\dot p=LM\ddot q$，即把 $p$ 当成不显含 $q$；这要求 $\partial p/\partial q=0$，而条件 (25a) 又要求 $\partial p/\partial\dot q=LM$，二者仅在 $M$ 不依赖 $q$（例如 $M$ 常数）且 $p$ 可取为仅含 $\dot q$ 的函数时同时成立。一般情形下这样的 $p$ 不存在，$\dot p$ 必含 $\frac{\partial p}{\partial q}\dot q$，它在 $\dot z$ 中被保留，构成上述缺项。补上该项（等价地，改用动量坐标把 $\dot M\dot q$ 吸收进 $u_p$）即得无遗漏的 (8)。**在这一意义上，定理 1 的 (8) 是把 (25) 补充完整后的可证明版本，(9) 是其误差界。**

## 5. 与本定理有关的边界说明

1. **增益选取。** (A4) 只需 $L$ 对称正定；取对角阵时收敛速率即 $\min_i l_i$，可任意加大但代价是对测量噪声与 $\dot M$ 的计算误差更敏感。若取 $L=L(q,\dot q)$ 为对称正定矩阵值函数（沿轨迹可微、且作为 $t$ 的函数已知），则需在 (8) 中补一项可计算项
\[
\dot z=-Lz-L(p+u_p)-\dot L(q,\dot q)\,y,\qquad p=L(q,\dot q)y,
\]
此时仍有 $\dot e=-Le$，(9) 中的 $\underline\lambda$ 换成 $\inf_t\lambda_{\min}L(t)$（并要求该下界为正）。
2. **扰动不是严格常值时。** 这一情形不属于定理 1 的结论，但可直接读出余项：若 $\|\dot d(t)\|\le L_d$，则 $\dot e=-Le-\dot d$ 给出
\[
\|e(t)\|\le e^{-\underline\lambda t}\|e(0)\|+\frac{L_d}{\underline\lambda}\big(1-e^{-\underline\lambda t}\big),
\]
即只有最终界 $L_d/\underline\lambda$，不存在零误差结论；指数收敛仅在 (A3) 下成立。
3. **原文记号。** (25) 标注 $z_i\in\mathbb R^n$，但其右端显然是逐关节方程，应为每个关节一个辅助状态；本文按 $z\in\mathbb R^n$ 逐分量处理。
4. **(25) 的实际适用范围。** 由 §4，(25) 只在 $\dot M\dot q\equiv0$（典型情形为 $M$ 常数）或已补上缺项时才成立；原文用 $M\ddot q$ 直接代入而未处理 $\dot p$ 中 $q$ 依赖项，是其证明缺失的根源。定理 1 的 (8) 无此限制。
