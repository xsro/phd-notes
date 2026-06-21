下面给出当前算法的完整叙述，分为三部分：

1. **问题 formulation**  
2. **控制算法 design**  
3. **稳定性证明：有限时间进入周期滑模并满足 \(\dot s_{i0}=0\)**  

这里证明只到

\[
\dot s_{i0}=0
\]

为止。由

\[
\dot s_{i0}=0
\]

进一步推出合围、避碰、速度一致的过程，可引用已有 fencing formation 文献中的名义系统分析结果。

---

# 1. 问题描述

考虑 \(N\) 个卫星 agent 和一个目标 target。设 agent \(i\) 的位置、速度分别为

\[
p_i(t)\in\mathbb R^m,\qquad v_i(t)\in\mathbb R^m,
\]

目标的位置、速度分别为

\[
p_0(t)\in\mathbb R^m,\qquad v_0(t)\in\mathbb R^m.
\]

## 1.1 Agent 动力学：控制方向未知

考虑 agent \(i\) 的二阶动力学为

\[
\dot p_i=v_i,
\]

\[
\dot v_i=B_i(t)u_i+d_i,
\]

其中

\[
u_i\in\mathbb R^m
\]

是控制输入，

\[
d_i\in\mathbb R^m
\]

是有界扰动，

\[
B_i(t)\in\mathbb R^{m\times m}
\]

是未知输入增益矩阵。

为了使用逐分量周期滑模控制，假设 \(B_i(t)\) 具有对角结构：

\[
B_i(t)
=
\operatorname{diag}\{b_{i1}(t),\dots,b_{im}(t)\}.
\]

其中每个 \(b_{i\ell}(t)\) 的符号未知，并允许在正负之间切换，但不允许穿越零。具体假设为：存在已知常数

\[
\underline b_{i\ell}>0
\]

使得

\[
0<\underline b_{i\ell}\le |b_{i\ell}(t)|,
\qquad \ell=1,\dots,m.
\]

扰动满足

\[
\|d_i(t)\|_\infty\le \bar d_i.
\]

## 1.2 Target 动力学

目标系统为

\[
\dot p_0=v_0,
\]

\[
\dot v_0=u_0,
\]

其中目标加速度有界：

\[
\|u_0(t)\|_\infty\le \bar u_0.
\]

## 1.3 相对变量

定义 agent \(i\) 关于目标的相对位置和相对速度为

\[
p_{i0}=p_i-p_0,
\]

\[
v_{i0}=v_i-v_0.
\]

于是

\[
\dot p_{i0}=v_{i0},
\]

\[
\dot v_{i0}=B_i(t)u_i+d_i-u_0.
\]

定义

\[
w_i=d_i-u_0.
\]

由扰动和目标加速度有界性，有

\[
\|w_i(t)\|_\infty
\le
\bar d_i+\bar u_0.
\]

记

\[
\bar w_i=\bar d_i+\bar u_0.
\]

因此对每个分量有

\[
|w_{i\ell}(t)|\le \bar w_i.
\]

---

# 2. 控制算法

## 2.1 积分滑模变量

定义辅助变量

\[
\eta_i\in\mathbb R^m
\]

满足

\[
\dot\eta_i=k_1p_{i0}-\varphi_i,
\]

其中 \(k_1>0\)，\(\varphi_i\) 是合围与避碰设计中的排斥项。

定义积分滑模变量

\[
s_{i0}
=
\eta_i+k_2p_{i0}+v_{i0},
\]

其中 \(k_2>0\)。

排斥项可取为

\[
\varphi_i
=
\sum_{j\in\mathcal N_i}
\rho(\|p_{ij}\|)
\frac{p_{ij}}{\|p_{ij}\|},
\]

其中

\[
p_{ij}=p_i-p_j,
\]

并且

\[
\rho(s)
=
\max
\left\{
\frac{1}{s-d}
-
\frac{1}{\mu-d},
0
\right\},
\qquad s>d.
\]

这里 \(d>0\) 是安全距离，\(\mu>d\) 是作用距离。

---

## 2.2 滑模变量导数

由

\[
s_{i0}
=
\eta_i+k_2p_{i0}+v_{i0}
\]

可得

\[
\dot s_{i0}
=
\dot\eta_i+k_2\dot p_{i0}+\dot v_{i0}.
\]

代入

\[
\dot\eta_i=k_1p_{i0}-\varphi_i,
\]

\[
\dot p_{i0}=v_{i0},
\]

\[
\dot v_{i0}=B_i(t)u_i+w_i,
\]

得到

\[
\dot s_{i0}
=
k_1p_{i0}-\varphi_i+k_2v_{i0}
+
B_i(t)u_i+w_i.
\]

定义

\[
\Delta_i
=
k_1p_{i0}-\varphi_i+k_2v_{i0}.
\]

则

\[
\dot s_{i0}
=
B_i(t)u_i+\Delta_i+w_i.
\]

逐分量写为

\[
\dot s_{i0,\ell}
=
b_{i\ell}(t)u_{i\ell}
+
\Delta_{i\ell}
+
w_{i\ell}.
\]

令

\[
N_{i\ell}
=
\Delta_{i\ell}+w_{i\ell}.
\]

则

\[
\dot s_{i0,\ell}
=
b_{i\ell}(t)u_{i\ell}
+
N_{i\ell}.
\]

---

## 2.3 周期滑模控制律

对每个 agent \(i\) 和每个分量 \(\ell=1,\dots,m\)，设计控制律

\[
u_{i\ell}
=
R_{i\ell}(t)
\operatorname{sgn}
\left[
\sin
\left(
\frac{\pi}{\varepsilon_{i\ell}}s_{i0,\ell}
\right)
\right],
\]

其中

\[
\varepsilon_{i\ell}>0
\]

是周期滑模参数。

控制增益设计为

\[
R_{i\ell}(t)
=
\frac{
|\Delta_{i\ell}(t)|+\bar w_i+\lambda_{i\ell}
}{
\underline b_{i\ell}
},
\]

其中

\[
\lambda_{i\ell}>0
\]

是鲁棒裕度。

由于

\[
\bar w_i=\bar d_i+\bar u_0,
\]

也可写成

\[
R_{i\ell}(t)
=
\frac{
|\Delta_{i\ell}(t)|+\bar d_i+\bar u_0+\lambda_{i\ell}
}{
\underline b_{i\ell}
}.
\]

向量形式为

\[
u_i
=
R_i(t)
\operatorname{sgn}
\left[
\sin(\Pi_i s_{i0})
\right],
\]

其中

\[
R_i(t)
=
\operatorname{diag}
\{R_{i1}(t),\dots,R_{im}(t)\},
\]

\[
\Pi_i
=
\operatorname{diag}
\left\{
\frac{\pi}{\varepsilon_{i1}},
\dots,
\frac{\pi}{\varepsilon_{im}}
\right\}.
\]

这里 \(\sin(\cdot)\) 和 \(\operatorname{sgn}(\cdot)\) 均按分量作用。

因此算法总结为

\[
\boxed{
\begin{aligned}
\dot\eta_i
&=
k_1p_{i0}-\varphi_i,
\\[1mm]
s_{i0}
&=
\eta_i+k_2p_{i0}+v_{i0},
\\[1mm]
\Delta_i
&=
k_1p_{i0}-\varphi_i+k_2v_{i0},
\\[1mm]
R_{i\ell}(t)
&=
\frac{
|\Delta_{i\ell}(t)|+\bar d_i+\bar u_0+\lambda_{i\ell}
}{
\underline b_{i\ell}
},
\\[1mm]
u_{i\ell}
&=
R_{i\ell}(t)
\operatorname{sgn}
\left[
\sin
\left(
\frac{\pi}{\varepsilon_{i\ell}}s_{i0,\ell}
\right)
\right].
\end{aligned}
}
\]

该控制律不使用 \(b_{i\ell}(t)\) 的符号信息。

---

# 3. 稳定性证明：有限时间到达并满足 \(\dot s_{i0}=0\)

下面证明：在控制方向符号保持不变的时间区间内，每个分量 \(s_{i0,\ell}\) 有限时间到达某个周期滑模面，并在理想滑模意义下满足

\[
\dot s_{i0,\ell}=0.
\]

若 \(b_{i\ell}(t)\) 发生有限次符号切换，则每次切换后系统会重新进入到达阶段；最后一次切换后，系统最终进入周期滑模运动并满足

\[
\dot s_{i0}=0.
\]

若允许无限频繁切换，则需要额外的驻留时间假设，以保证每次切换后有足够时间重新到达相应滑模面。

---

## 3.1 逐分量闭环系统

固定 agent \(i\) 和分量 \(\ell\)。为简化记号，令

\[
s=s_{i0,\ell},
\]

\[
b=b_{i\ell}(t),
\]

\[
u=u_{i\ell},
\]

\[
\Delta=\Delta_{i\ell},
\]

\[
w=w_{i\ell},
\]

\[
N=\Delta+w,
\]

\[
\varepsilon=\varepsilon_{i\ell},
\]

\[
R=R_{i\ell}(t),
\]

\[
\lambda=\lambda_{i\ell},
\]

\[
\underline b=\underline b_{i\ell}.
\]

则闭环系统为

\[
\dot s
=
bR
\operatorname{sgn}
\left[
\sin
\left(
\frac{\pi}{\varepsilon}s
\right)
\right]
+
N.
\]

由增益设计

\[
R=
\frac{|\Delta|+\bar w_i+\lambda}{\underline b}.
\]

又由假设

\[
|b|\ge \underline b,
\]

因此

\[
|b|R
\ge
\underline b R
=
|\Delta|+\bar w_i+\lambda.
\]

另一方面，

\[
N=\Delta+w,
\]

且

\[
|w|\le \bar w_i.
\]

所以

\[
|N|
\le
|\Delta|+|w|
\le
|\Delta|+\bar w_i.
\]

因此

\[
|b|R-|N|
\ge
\lambda.
\]

即

\[
\boxed{
|b|R>|N|.
}
\]

这是周期滑模可达性的关键不等式。

---

## 3.2 当 \(b>0\) 时的到达性

假设在当前时间区间内

\[
b(t)>0.
\]

考虑奇数倍周期滑模面

\[
s=k\varepsilon,
\qquad
k=2r+1,\quad r\in\mathbb Z.
\]

令

\[
\tilde s=s-k\varepsilon.
\]

在该滑模面的吸引区间内，即

\[
|\tilde s|<\varepsilon,
\]

有

\[
s=k\varepsilon+\tilde s.
\]

于是

\[
\sin
\left(
\frac{\pi}{\varepsilon}s
\right)
=
\sin
\left(
k\pi+\frac{\pi}{\varepsilon}\tilde s
\right).
\]

由于 \(k\) 为奇数，

\[
\sin(k\pi+\theta)=-\sin\theta.
\]

因此在该区间内，

\[
\operatorname{sgn}
\left[
\sin
\left(
\frac{\pi}{\varepsilon}s
\right)
\right]
=
-\operatorname{sgn}(\tilde s).
\]

于是

\[
\dot{\tilde s}
=
\dot s
=
-bR\operatorname{sgn}(\tilde s)+N.
\]

取 Lyapunov 函数

\[
V=\frac12\tilde s^2.
\]

则

\[
\dot V
=
\tilde s\dot{\tilde s}.
\]

代入上式，得

\[
\dot V
=
-bR|\tilde s|+\tilde sN.
\]

利用

\[
\tilde sN\le |\tilde s||N|,
\]

有

\[
\dot V
\le
-\left(bR-|N|\right)|\tilde s|.
\]

由于 \(b>0\)，有 \(b=|b|\)，因此

\[
\dot V
\le
-\left(|b|R-|N|\right)|\tilde s|.
\]

由

\[
|b|R-|N|\ge \lambda
\]

得到

\[
\dot V
\le
-\lambda|\tilde s|.
\]

又因为

\[
|\tilde s|=\sqrt{2V},
\]

所以

\[
\dot V
\le
-\lambda\sqrt{2V}.
\]

因此 \(V\) 在有限时间内收敛到零。

也就是说，当 \(b>0\) 时，奇数倍周期滑模面

\[
s=(2r+1)\varepsilon
\]

是有限时间吸引的。

---

## 3.3 当 \(b<0\) 时的到达性

假设在当前时间区间内

\[
b(t)<0.
\]

考虑偶数倍周期滑模面

\[
s=k\varepsilon,
\qquad
k=2r,\quad r\in\mathbb Z.
\]

令

\[
\tilde s=s-k\varepsilon.
\]

在该滑模面的吸引区间内，

\[
|\tilde s|<\varepsilon.
\]

有

\[
\sin
\left(
\frac{\pi}{\varepsilon}s
\right)
=
\sin
\left(
k\pi+\frac{\pi}{\varepsilon}\tilde s
\right).
\]

由于 \(k\) 为偶数，

\[
\sin(k\pi+\theta)=\sin\theta.
\]

因此

\[
\operatorname{sgn}
\left[
\sin
\left(
\frac{\pi}{\varepsilon}s
\right)
\right]
=
\operatorname{sgn}(\tilde s).
\]

闭环动力学变为

\[
\dot{\tilde s}
=
bR\operatorname{sgn}(\tilde s)+N.
\]

由于

\[
b=-|b|,
\]

所以

\[
\dot{\tilde s}
=
-|b|R\operatorname{sgn}(\tilde s)+N.
\]

仍取 Lyapunov 函数

\[
V=\frac12\tilde s^2.
\]

则

\[
\dot V
=
\tilde s\dot{\tilde s}
=
-|b|R|\tilde s|+\tilde sN.
\]

由

\[
\tilde sN\le |\tilde s||N|
\]

可得

\[
\dot V
\le
-\left(|b|R-|N|\right)|\tilde s|.
\]

再由

\[
|b|R-|N|\ge \lambda
\]

得到

\[
\dot V
\le
-\lambda|\tilde s|
=
-\lambda\sqrt{2V}.
\]

因此 \(V\) 在有限时间内收敛到零。

也就是说，当 \(b<0\) 时，偶数倍周期滑模面

\[
s=2r\varepsilon
\]

是有限时间吸引的。

---

## 3.4 有限时间到达估计

由

\[
\dot V
\le
-\lambda\sqrt{2V}
\]

可得，当 \(V>0\) 时，

\[
\frac{d}{dt}\sqrt V
=
\frac{\dot V}{2\sqrt V}
\le
-\frac{\lambda}{\sqrt 2}.
\]

积分可得

\[
\sqrt{V(t)}
\le
\sqrt{V(t_0)}
-
\frac{\lambda}{\sqrt 2}(t-t_0).
\]

因此，到达时间满足

\[
T_r
\le
\frac{\sqrt{2V(t_0)}}{\lambda}
=
\frac{|\tilde s(t_0)|}{\lambda}.
\]

所以每个分量 \(s_{i0,\ell}\) 都能在有限时间内到达相应的周期滑模面。

---

## 3.5 滑模面上的等效运动与 \(\dot s=0\)

一旦 \(s\) 到达对应的吸引周期滑模面

\[
s=k\varepsilon,
\]

在理想滑模意义下，系统保持在该面上，因此

\[
\dot s=0.
\]

从 Filippov 滑模角度看，在吸引滑模面处，控制项在两侧取值分别对应 \(+R\) 和 \(-R\)。由于已经证明

\[
|N|<|b|R,
\]

所以存在等效控制

\[
u_{\mathrm{eq}}
=
-\frac{N}{b}
\]

满足

\[
|u_{\mathrm{eq}}|<R.
\]

因此该等效控制属于不连续控制的凸包范围内，从而可以维持

\[
\dot s=bu_{\mathrm{eq}}+N=0.
\]

于是，对于每个分量均有

\[
\dot s_{i0,\ell}=0.
\]

因此向量形式为

\[
\boxed{
\dot s_{i0}=0.
}
\]

---

# 4. 降阶名义合围动力学

由

\[
s_{i0}
=
\eta_i+k_2p_{i0}+v_{i0}
\]

可得

\[
\dot s_{i0}
=
\dot\eta_i+k_2\dot p_{i0}+\dot v_{i0}.
\]

又因为

\[
\dot\eta_i=k_1p_{i0}-\varphi_i,
\]

\[
\dot p_{i0}=v_{i0},
\]

所以

\[
\dot s_{i0}
=
k_1p_{i0}-\varphi_i+k_2v_{i0}+\dot v_{i0}.
\]

在周期滑模运动中，

\[
\dot s_{i0}=0.
\]

因此

\[
\dot v_{i0}+k_2v_{i0}+k_1p_{i0}-\varphi_i=0.
\]

又因为

\[
v_{i0}=\dot p_{i0},
\]

于是得到

\[
\boxed{
\ddot p_{i0}
+
k_2\dot p_{i0}
+
k_1p_{i0}
-
\varphi_i
=
0.
}
\]

等价地，

\[
\boxed{
\ddot p_{i0}
+
k_2\dot p_{i0}
+
k_1p_{i0}
=
\varphi_i.
}
\]

这正是已有 fencing formation 文献中常见的名义合围动力学形式，即：

- \(k_1p_{i0}\) 提供目标吸引；
- \(k_2\dot p_{i0}\) 提供阻尼；
- \(\varphi_i\) 提供卫星间排斥与避碰。

因此，本控制器的作用是：在控制方向未知、扰动存在、目标加速度有界的情况下，使实际系统有限时间进入滑模运动，并在滑模运动中恢复已有文献分析的名义合围系统。

---

# 5. 定理表述

可以将上述结果概括为以下定理。

---

## Theorem

Consider the multi-agent system

\[
\dot p_i=v_i,
\]

\[
\dot v_i=B_i(t)u_i+d_i,
\]

and the target dynamics

\[
\dot p_0=v_0,
\]

\[
\dot v_0=u_0.
\]

Suppose that

\[
B_i(t)=\operatorname{diag}\{b_{i1}(t),\dots,b_{im}(t)\},
\]

\[
0<\underline b_{i\ell}\le |b_{i\ell}(t)|,
\qquad \ell=1,\dots,m,
\]

\[
\|d_i(t)\|_\infty\le \bar d_i,
\]

\[
\|u_0(t)\|_\infty\le \bar u_0.
\]

Define

\[
s_{i0}
=
\eta_i+k_2p_{i0}+v_{i0},
\]

\[
\dot\eta_i=k_1p_{i0}-\varphi_i,
\]

and

\[
\Delta_i
=
k_1p_{i0}-\varphi_i+k_2v_{i0}.
\]

For each component \(\ell\), apply

\[
u_{i\ell}
=
R_{i\ell}(t)
\operatorname{sgn}
\left[
\sin
\left(
\frac{\pi}{\varepsilon_{i\ell}}s_{i0,\ell}
\right)
\right],
\]

where

\[
R_{i\ell}(t)
=
\frac{
|\Delta_{i\ell}(t)|+\bar d_i+\bar u_0+\lambda_{i\ell}
}{
\underline b_{i\ell}
},
\qquad
\lambda_{i\ell}>0.
\]

Then, on any time interval over which the sign of \(b_{i\ell}(t)\) is constant, \(s_{i0,\ell}\) reaches a corresponding periodic sliding surface in finite time. In the ideal sliding mode sense,

\[
\dot s_{i0,\ell}=0.
\]

Consequently,

\[
\dot s_{i0}=0.
\]

On the sliding manifold, the closed-loop relative dynamics reduce to

\[
\ddot p_{i0}
+
k_2\dot p_{i0}
+
k_1p_{i0}
-
\varphi_i
=
0.
\]

If the above nominal relative dynamics satisfy the assumptions of existing fencing formation results, then the desired convex-hull fencing, collision avoidance, and velocity consensus properties follow from those results.

---

# 6. 必须注明的技术条件

为了使上述表述在论文中严谨，建议明确写出以下条件：

1. **输入增益必须是对角结构**  
   即

   \[
   B_i(t)=\operatorname{diag}\{b_{i1}(t),\dots,b_{im}(t)\}.
   \]

   如果 \(B_i(t)\) 是一般未知矩阵，上述逐分量周期滑模控制不能直接成立。

2. **控制方向不能穿越零**

   \[
   |b_{i\ell}(t)|\ge \underline b_{i\ell}>0.
   \]

   符号可以切换，但不能连续经过零点。

3. **切换不能无限快**  
   若控制方向只发生有限次切换，则最后一次切换后可保证进入滑模。若发生无限次切换，需要引入驻留时间条件。

4. **滑模意义为理想滑模或 Filippov 意义**  
   因为控制律含有不连续项

   \[
   \operatorname{sgn}
   \left[
   \sin
   \left(
   \frac{\pi}{\varepsilon}s
   \right)
   \right].
   \]

5. **合围证明引用名义系统文献**  
   本证明只保证实际系统在有限时间后满足

   \[
   \dot s_{i0}=0,
   \]

   从而降阶为

   \[
   \ddot p_{i0}
   +
   k_2\dot p_{i0}
   +
   k_1p_{i0}
   -
   \varphi_i
   =
   0.
   \]

   后续 convex-hull fencing、collision avoidance 和 velocity consensus 应引用已有名义系统结论。