下面只给出：**问题、算法、证明**。对象是前面 MATLAB 代码对应的最简单一阶系统。

---

# 1. 问题

考虑一阶不确定系统

\[
\dot x(t)=b(t)u(t)+d(t),
\]

其中：

- \(x(t)\in\mathbb R\) 可测；
- \(u(t)\in\mathbb R\) 为控制输入；
- \(b(t)\) 为控制方向系数，其符号未知且允许切换；
- \(d(t)\) 为有界扰动。

假设存在已知常数 \(\underline b>0\)、\(\bar d>0\)，使得

\[
|b(t)|\geq \underline b>0,
\]

\[
|d(t)|\leq \bar d.
\]

控制目标是在不知道 \(b(t)\) 符号的情况下，设计 \(u(t)\)，使得状态 \(x(t)\) 收敛到零，或者在控制方向发生切换后重新进入收敛过程。

需要说明：若 \(b(t)\) 无限频繁切换且无驻留时间条件，则一般不能保证全局渐近收敛。下面证明针对每一个控制方向保持不变的时间区间成立；若 \(b(t)\) 只发生有限次切换，则最终有

\[
\lim_{t\to\infty}x(t)=0.
\]

---

# 2. 算法

定义积分状态

\[
q(t)=\int_0^t x(\tau)\,d\tau,
\]

即

\[
\dot q(t)=x(t).
\]

定义滑模变量

\[
\sigma(t)=x(t)+c q(t),
\]

其中 \(c>0\)。

设计周期滑模控制律

\[
u(t)=R(x(t))
\operatorname{sgn}
\left[
\sin\left(\frac{\pi}{\varepsilon}\sigma(t)\right)
\right],
\]

其中 \(\varepsilon>0\)，且状态相关增益取为

\[
R(x)=\frac{c|x|+\bar d+\lambda}{\underline b},
\]

其中 \(\lambda>0\) 为鲁棒裕度。

因此完整算法为

\[
\boxed{
\begin{aligned}
\dot q &=x,\\[2mm]
\sigma&=x+cq,\\[2mm]
R(x)&=\frac{c|x|+\bar d+\lambda}{\underline b},\\[2mm]
u&=R(x)\operatorname{sgn}
\left[
\sin\left(\frac{\pi}{\varepsilon}\sigma\right)
\right].
\end{aligned}}
\]

该控制律不使用 \(b(t)\) 的符号。

---

# 3. 证明

由

\[
\sigma=x+cq
\]

可得

\[
\dot\sigma=\dot x+c\dot q.
\]

由于

\[
\dot x=b(t)u+d(t),\qquad \dot q=x,
\]

因此

\[
\dot\sigma=b(t)u+d(t)+cx.
\]

记

\[
N(t)=d(t)+cx(t).
\]

则

\[
\dot\sigma=b(t)u+N(t).
\]

由于

\[
|d(t)|\leq \bar d,
\]

有

\[
|N(t)|=|d(t)+cx(t)|
\leq \bar d+c|x(t)|.
\]

由增益设计

\[
R(x)=\frac{c|x|+\bar d+\lambda}{\underline b},
\]

并利用

\[
|b(t)|\geq \underline b,
\]

得到

\[
|b(t)|R(x)
\geq
\underline b R(x)
=
c|x|+\bar d+\lambda.
\]

因此

\[
|b(t)|R(x)-|N(t)|
\geq
\lambda>0.
\]

即

\[
\boxed{
|b(t)|R(x)>|N(t)|.
}
\]

这是保证周期滑模到达性的关键条件。

---

## 3.1 当 \(b(t)>0\) 时

考虑奇数倍周期滑模面

\[
\sigma=k\varepsilon,\qquad k=2m+1,\quad m\in\mathbb Z.
\]

令

\[
\tilde\sigma=\sigma-k\varepsilon.
\]

在 \(\sigma=k\varepsilon\) 附近，有

\[
\sin\left(\frac{\pi}{\varepsilon}\sigma\right)
=
\sin\left(k\pi+\frac{\pi}{\varepsilon}\tilde\sigma\right).
\]

因为 \(k\) 为奇数，

\[
\sin(k\pi+\theta)=-\sin\theta.
\]

因此局部有

\[
\operatorname{sgn}
\left[
\sin\left(\frac{\pi}{\varepsilon}\sigma\right)
\right]
=
-\operatorname{sgn}(\tilde\sigma).
\]

闭环滑模变量动力学为

\[
\dot{\tilde\sigma}
=
\dot\sigma
=
-b(t)R(x)\operatorname{sgn}(\tilde\sigma)+N(t).
\]

取 Lyapunov 函数

\[
V=\frac12\tilde\sigma^2.
\]

则

\[
\dot V
=
\tilde\sigma\dot{\tilde\sigma}.
\]

代入得

\[
\dot V
=
-b(t)R(x)|\tilde\sigma|+\tilde\sigma N(t).
\]

由于

\[
\tilde\sigma N(t)\leq |\tilde\sigma||N(t)|,
\]

且 \(b(t)>0\)，所以

\[
\dot V
\leq
-\left(b(t)R(x)-|N(t)|\right)|\tilde\sigma|.
\]

因为 \(b(t)=|b(t)|\)，并且

\[
|b(t)|R(x)-|N(t)|\geq \lambda,
\]

所以

\[
\dot V
\leq
-\lambda|\tilde\sigma|.
\]

又因为

\[
|\tilde\sigma|=\sqrt{2V},
\]

故

\[
\dot V\leq -\lambda\sqrt{2V}.
\]

因此奇数倍滑模面

\[
\sigma=(2m+1)\varepsilon
\]

在 \(b(t)>0\) 时是有限时间吸引的。

---

## 3.2 当 \(b(t)<0\) 时

考虑偶数倍周期滑模面

\[
\sigma=k\varepsilon,\qquad k=2m,\quad m\in\mathbb Z.
\]

令

\[
\tilde\sigma=\sigma-k\varepsilon.
\]

在该滑模面附近，

\[
\sin\left(\frac{\pi}{\varepsilon}\sigma\right)
=
\sin\left(k\pi+\frac{\pi}{\varepsilon}\tilde\sigma\right).
\]

因为 \(k\) 为偶数，

\[
\sin(k\pi+\theta)=\sin\theta.
\]

因此

\[
\operatorname{sgn}
\left[
\sin\left(\frac{\pi}{\varepsilon}\sigma\right)
\right]
=
\operatorname{sgn}(\tilde\sigma).
\]

闭环动力学为

\[
\dot{\tilde\sigma}
=
b(t)R(x)\operatorname{sgn}(\tilde\sigma)+N(t).
\]

由于 \(b(t)<0\)，有

\[
b(t)=-|b(t)|.
\]

因此

\[
\dot{\tilde\sigma}
=
-|b(t)|R(x)\operatorname{sgn}(\tilde\sigma)+N(t).
\]

仍取

\[
V=\frac12\tilde\sigma^2.
\]

则

\[
\dot V
=
-|b(t)|R(x)|\tilde\sigma|+\tilde\sigma N(t).
\]

于是

\[
\dot V
\leq
-\left(|b(t)|R(x)-|N(t)|\right)|\tilde\sigma|.
\]

由

\[
|b(t)|R(x)-|N(t)|\geq \lambda
\]

得到

\[
\dot V\leq -\lambda|\tilde\sigma|
=-\lambda\sqrt{2V}.
\]

因此偶数倍滑模面

\[
\sigma=2m\varepsilon
\]

在 \(b(t)<0\) 时是有限时间吸引的。

---

## 3.3 有限时间到达性

由

\[
\dot V\leq -\lambda\sqrt{2V}
\]

可得，当 \(V>0\) 时，

\[
\frac{d}{dt}\sqrt{V}
=
\frac{\dot V}{2\sqrt V}
\leq
-\frac{\lambda}{\sqrt2}.
\]

积分得

\[
\sqrt{V(t)}
\leq
\sqrt{V(t_0)}
-
\frac{\lambda}{\sqrt2}(t-t_0).
\]

因此在有限时间

\[
T_r\leq
\frac{\sqrt{2V(t_0)}}{\lambda}
=
\frac{|\tilde\sigma(t_0)|}{\lambda}
\]

内，有

\[
V(T_r)=0.
\]

即

\[
\sigma(T_r)=k\varepsilon.
\]

所以，在控制方向固定的区间内，\(\sigma(t)\) 可以有限时间到达相应的吸引周期滑模面。

---

## 3.4 滑模面上的状态收敛性

在理想滑模运动中，

\[
\sigma(t)=k\varepsilon,
\]

因此

\[
\dot\sigma(t)=0.
\]

又由于

\[
\sigma=x+cq,
\]

得到

\[
\dot\sigma=\dot x+cx.
\]

因此滑模面上满足

\[
\dot x+cx=0.
\]

其解为

\[
x(t)=x(T_r)e^{-c(t-T_r)},\qquad t\geq T_r.
\]

因为 \(c>0\)，所以

\[
\lim_{t\to\infty}x(t)=0.
\]

---

# 4. 结论

对于系统

\[
\dot x=b(t)u+d(t),
\]

在假设

\[
|b(t)|\geq \underline b>0,\qquad |d(t)|\leq \bar d
\]

成立的条件下，控制律

\[
u=
\frac{c|x|+\bar d+\lambda}{\underline b}
\operatorname{sgn}
\left[
\sin\left(\frac{\pi}{\varepsilon}(x+cq)\right)
\right],
\qquad \dot q=x,
\]

具有如下性质：

1. 不需要知道 \(b(t)\) 的符号；
2. 当 \(b(t)>0\) 时，奇数倍滑模面 \(\sigma=(2m+1)\varepsilon\) 有限时间吸引；
3. 当 \(b(t)<0\) 时，偶数倍滑模面 \(\sigma=2m\varepsilon\) 有限时间吸引；
4. 在理想滑模面上有

\[
\dot x+cx=0,
\]

因此

\[
x(t)\to 0.
\]

若 \(b(t)\) 只发生有限次符号切换，则最后一次切换后系统重新到达对应周期滑模面，并最终满足

\[
\lim_{t\to\infty}x(t)=0.
\]