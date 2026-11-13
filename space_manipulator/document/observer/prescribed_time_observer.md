# 仅用 $q_m,\dot q_m,\tau$ 的预设时刻扰动估计：两个可证明定理

## 1. 模型与动量通道

考虑关节空间动力学
\[
M(q)\ddot q+C(q,\dot q)\dot q=\tau+d(t),\qquad q=q_m,
\tag{1}
\]
其中 $M=M_e$、$C=C_e$。假设 $M\in C^1$，沿所讨论轨迹可计算 $M(q)$、$C(q,\dot q)\dot q$、$\dot M(q,\dot q)=\sum_j(\partial M/\partial q_j)\dot q_j$；$q,\dot q$ 和**实际施加**的 $\tau$ 可测。所有等式针对精确模型和理想连续时间信号。无需测量 $\ddot q$。

定义可测的广义动量与已知输入
\[
y:=M(q)\dot q,\qquad
u_p:=\tau-C(q,\dot q)\dot q+\dot M(q,\dot q)\dot q.
\tag{2}
\]
由乘积法则与 (1)，几乎处处有
\[
\dot y=\dot M\dot q+M\ddot q=\nu_p+d(t).
\tag{3}
\]
**因此被估计的量直接是原扰动力矩 $d(t)$，而不是 $M^{-1}(q)d(t)$。** 特别地，$d$ 为常值时不要求 $M$ 为常数。以下所有观测器运算逐分量进行；也可将增益乘以 $I_n$ 写为向量式。

约定：这里“在预设时刻 $T_o$ 收敛”指对任何有限的观测器初值，$\lim_{t\uparrow T_o}\hat d(t)=d(T_o)$，且收敛时刻上界与初值无关。除非另行证明，不把“$T_o$ 时刻准确”理解为“以后对任意时变扰动永久准确”。

## 2. 观测器

给定任意 $T_o>0$，在 $0\le t<T_o$ 记 $s=T_o-t$。取 $r_1=3,r_2=4$，即 $a_1=r_1+r_2-1=6$、$a_2=r_1r_2-a_1=6$，定义
\[
\boxed{\begin{aligned}
\dot z_1&=\nu_p+z_2+\frac{6}{s}(y-z_1),\\
\dot z_2&=\frac{6}{s^2}(y-z_1),\\
\hat d&=z_2.
\end{aligned}}
\tag{4}
\]
所有右端只用到 $q,\dot q,\tau,t,z_1,z_2$。该设计是**奇异增益的预设时刻观测器**，不是非奇异固定时间超扭转观测器。

令
\[
e_1=y-z_1,\qquad e_2=d-z_2,\qquad x=\begin{bmatrix}e_1\\s e_2\end{bmatrix},\qquad
\rho=\log\frac{T_o}{s}.
\tag{5}
\]
利用 $d\rho/dt=1/s$，$ds/dt=-1$，由 (3)--(4) 得
\[
\frac{dx}{d\rho}=Ax+\begin{bmatrix}0\\s^2\dot d\end{bmatrix},
\qquad
A=\begin{bmatrix}-6I_n&I_n\\-6I_n&-I_n\end{bmatrix}.
\tag{6}
\]
矩阵 $A$ 的每个标量块的特征值为 $-3,-4$，因此存在只由 $A$ 决定的有限常数 $K\ge1$，使对全部 $\rho\ge0$ 有
\[
\|e^{A\rho}\|\le K e^{-3\rho}.
\tag{7}
\]

## 3. 定理一：原扰动 $d$ 为常值

**定理 1（常值扰动力矩的预设时刻精确估计及保持）。** 假设 (1)--(3) 成立，且 $d(t)\equiv d_0\in\mathbb R^n$（未知常向量）。观测器 (4) 对任意有限初值在每个紧区间 $[0,T_o-\epsilon]$ 上有唯一经典解，且
\[
\|y(t)-z_1(t)\|\le K\|x(0)\|\left(\frac{s}{T_o}\right)^3,
\qquad
\|d_0-z_2(t)\|\le\frac{K\|x(0)\|}{T_o}
\left(\frac{s}{T_o}\right)^2.
\tag{8}
\]
因此 $\lim_{t\uparrow T_o}z_1(t)=y(T_o)$、$\lim_{t\uparrow T_o}z_2(t)=d_0$。若在 $T_o$ 按极限初始化，并于 $t\ge T_o$ 采用
\[
\dot z_1=\nu_p+z_2,\qquad \dot z_2=0,
\tag{9}
\]
则 $z_1(t)=y(t)$、$z_2(t)=d_0$ 对一切 $t\ge T_o$ 成立。

**证明。** $\dot d=0$，故 (6) 退化为 $dx/d\rho=Ax$；由 (7) 有 $\|x(\rho)\|\le Ke^{-3\rho}\|x(0)\|$。因为 $e^{-\rho}=s/T_o$、$e_1=x_1$、$e_2=x_2/s$，即得 (8)。两误差趋零，故 (4) 在 $T_o$ 的左极限为正确的 $y,d_0$。对延拓律 (9)，结合 $\dot y=\nu_p+d_0$ 可得 $\dot e_1=e_2$、$\dot e_2=0$；以零误差为初值，唯一解恒为零。证毕。

> **原扰动而非等效扰动常值。** 本定理并不需要 $M^{-1}(q)d_0$ 为常值，这正是使用动量通道 (3) 的理由。

## 4. 定理二：确定性时变扰动 $d(t)$

**定理 2（有界变化率时变扰动的预设时刻精确估计）。** 假设 (1)--(3) 成立，$d:[0,T_o]\to\mathbb R^n$ 是**仅关于时间的确定性函数**，绝对连续，且存在有限常数 $L$ 使
\[
\|\dot d(t)\|\le L\quad\text{对几乎所有 }t\in[0,T_o].
\tag{10}
\]
不要求在线设计知道 $L$ 的数值。对观测器 (4)，任意有限初始误差满足
\[
\begin{aligned}
\|y(t)-z_1(t)\|
&\le K\|x(0)\|\left(\frac{s}{T_o}\right)^3
+KLT_o^2\left[\left(\frac{s}{T_o}\right)^2-
\left(\frac{s}{T_o}\right)^3\right],\\
\|d(t)-z_2(t)\|
&\le\frac{K\|x(0)\|}{T_o}\left(\frac{s}{T_o}\right)^2
+KL\left[s-\frac{s^2}{T_o}\right].
\end{aligned}
\tag{11}
\]
因此 $\lim_{t\uparrow T_o}z_1(t)=y(T_o)$，$\lim_{t\uparrow T_o}z_2(t)=d(T_o)$。这是对每个满足 (10) 的函数成立的**确定性预设时刻**结论，与初始误差及 $L$ 的大小无关（只要求二者有限）。

**证明。** 在每个 $t<T_o$ 上，(6) 为具有局部可积强迫项的线性系统，其变常数公式为
\[
x(\rho)=e^{A\rho}x(0)
+\int_0^\rho e^{A(\rho-u)}
\begin{bmatrix}0\\T_o^2e^{-2u}\dot d(t(u))\end{bmatrix}du,
\qquad t(u)=T_o(1-e^{-u}).
\tag{12}
\]
由 (7)、(10) 和 $s=T_oe^{-\rho}$ 得
\[
\begin{aligned}
\|x(\rho)\|
&\le Ke^{-3\rho}\|x(0)\|
+KLT_o^2\int_0^\rho e^{-3(\rho-u)}e^{-2u}du\\
&=Ke^{-3\rho}\|x(0)\|
+KLT_o^2(e^{-2\rho}-e^{-3\rho}).
\end{aligned}
\tag{13}
\]
再用 $e_1=x_1$、$e_2=x_2/s$，得到 (11)。其右端均随 $s\downarrow0$ 趋零。因为绝对连续的 $d$ 及假设中的 $y$ 在 $T_o$ 有连续值，所述极限成立。证毕。

**到达时刻与持续保持的区别。** 定理 2 只证明到 $T_o$ 的精确极限，没有证明对任意时变 $d(t)$ 都有 $z_2(t)=d(t)$ 于所有 $t\ge T_o$。特别是若在 $T_o$ 后直接用定理 1 的 (9)，则 $z_2$ 不变而 $d(t)$ 可变，通常立即产生误差。若后续控制定理需要 $\hat d(t)\equiv d(t)$ 于 $t\ge T_o$，必须另外给出并证明一个**持续精确跟踪**的后续观测器（例如在明确增益条件下的滑模微分器），或将该要求替换为残差界。本文件不把尚未证明的后续跟踪当作结论。

## 5. 使用范围及实现边界

1. **与论文控制器对接。** 可取 $\hat d=z_2$，并令 $\tau=M_e(q)(\ddot q_d+\nu)+C_e(q,\dot q)\dot q-\hat d$。观测器 (2)、(4) 所用的 $\tau$ 必须是实际施加值；不需要测量 $\ddot q$。当 $M_e$ 的表达还依赖基座位姿时，必须取得完整模型计算 $y,\nu_p$。
2. **奇异增益与噪声。** 随 $s\downarrow0$，$6/s$ 和 $6/s^2$ 无界。严格结论以精确测量、连续时间和无饱和为前提；若以 $\max\{s,\epsilon\}$ 截断、离散采样或加入噪声，上述零误差结论不再直接成立，需要单独推导实际残差界。
3. **扰动条件。** 定理 2 要求 $d$ 绝对连续且变化率有限，不要求随机模型、概率假设或导数上界的已知数值；跳变扰动与理想白噪声不在证明范围内。
4. **有关“预设时间”的精确用语。** 两定理都保证任意有限初值在指定 $T_o$ 时的误差极限为零，且时间上界与初值无关；定理 1 另外证明了 $T_o$ 后的持续精确估计，定理 2 没有。若定义要求观测器在 $T_o$ **之前某个严格更早的时刻**到达零点，连续线性观测器 (4) 一般不满足该更强要求。
