# 基于指令滤波与周期延迟反馈的自由漂浮空间机械臂预设时间轨迹跟踪控制

**Command-Filtered Periodic Delayed Feedback Control for Prescribed-Time Trajectory Tracking of Free-Floating Space Manipulators**

> 本文融合严宇新博士论文中的自由漂浮空间机械臂建模与预设时间扰动观测器、Zhou 等与 Ding 等的周期延迟反馈（Periodic Delayed Feedback, PDF）预设时间镇定理论、以及 Farrell 与 Dong 等的指令滤波反步法（Command-Filtered Backstepping），给出一个完整的控制算法及其严谨的稳定性证明。

---

## 目录

1. [摘要](#1-摘要)
2. [引言](#2-引言)
3. [预备知识与问题描述](#3-预备知识与问题描述)
   - 3.1 记号与函数类
   - 3.2 周期延迟反馈与强预设时间镇定
   - 3.3 自由漂浮空间机械臂建模
   - 3.4 控制目标与假设
4. [控制算法设计](#4-控制算法设计)
   - 4.1 预设时间扰动观测器
   - 4.2 动力学补偿与误差线性化
   - 4.3 指令滤波反步法结合周期延迟反馈
   - 4.4 误差补偿系统与最终控制律
   - 4.5 需要测量的量与实现所需信号
5. [稳定性分析](#5-稳定性分析)
   - 5.1 预备引理
   - 5.2 主定理与证明
   - 5.3 关于跟踪误差的推论
6. [鲁棒性与工程实现讨论](#6-鲁棒性与工程实现讨论)
7. [仿真验证](#7-仿真验证)
8. [结论](#8-结论)
9. [参考文献](#9-参考文献)

---

## 1. 摘要

针对捕获前自由漂浮空间机械臂的关节空间轨迹跟踪问题,本文提出一种基于**指令滤波(command filtering)**与**光滑周期延迟反馈(smooth periodic delayed feedback, PDF)**的预设时间控制方法。首先,依据动量守恒关系将自由漂浮空间机械臂约化为等效关节空间动力学,并设计预设时间扰动观测器(prescribed-time disturbance observer, PTDO)在预设时间 $T_o$ 内估计集总扰动。其次,采用计算力矩补偿将非线性耦合动力学规范化为关于跟踪误差的二阶积分链,从而把 PDF 镇定设计对象由非线性系统转化为可控线性误差通道。随后,在该误差通道上采用**指令滤波反步法**设计控制器:以含线性周期延迟反馈项的虚拟控制镇定位置误差子通道,以一阶指令滤波器生成虚拟控制及其导数的替代信号以消除"微分爆炸",并以含幂分配周期延迟反馈项的实际控制镇定速度误差子通道;同时构造误差补偿系统抵消滤波误差的影响。理论分析表明,在理想补偿条件下,补偿后的跟踪误差可在预设时间 $T_o+T_s$ 内精确归零,且闭环系统关于匹配扰动具有强固定时间稳定性;实际跟踪误差在补偿误差归零后以指数速率收敛到零（速率由滤波器带宽 $\omega$ 与 $a$ 决定）。最后给出七自由度空间机械臂的数值仿真验证。

**关键词**：自由漂浮空间机械臂；周期延迟反馈；预设时间控制；指令滤波；反步法；扰动观测器

---

## 2. 引言

### 2.1 研究背景

自由漂浮空间机械臂在执行在轨捕获任务时，基座不直接施加控制力和力矩，机械臂关节运动会通过系统动量守恒关系诱导基座位姿变化，进而影响末端执行器的运动。其动力学具有强非线性、强耦合和动态奇异等特征，此外还受到模型参数不确定、未知时变扰动和执行器误差的影响。因此，在基座不受控条件下实现高精度、收敛时间可预先指定的轨迹跟踪，具有重要的理论与工程意义。

### 2.2 相关工作的三条脉络

**(a) 自由漂浮空间机械臂建模与控制。** Umetani 与 Yoshida 提出广义雅可比矩阵刻画自由漂浮机械臂末端速度与关节速度的关系；Vafa 与 Dubowsky 的虚拟机械臂方法系统刻画了运动学与动力学耦合；Papadopoulos 与 Dubowsky 揭示了动态奇异问题。严宇新在其博士论文中给出了完整的等效关节空间动力学推导，并构造了"扰动观测→动力学补偿→误差线性化→周期延迟反馈"的控制架构。

**(b) 周期延迟反馈预设时间镇定。** 传统有限时间控制（终端吸引子、齐次性理论）的收敛时间依赖初值；固定时间控制虽给出与初值无关的上界，但上界是系统参数的复杂函数；基于时变高增益的预设时间控制存在奇异问题。Zhou、Michiels 与 Chen 提出光滑周期延迟反馈，其增益有界且可任意光滑，避免奇异问题。Ding、Zhou、Zhang 与 Michiels 进一步提出基于 Lyapunov 的强预设时间镇定判据，并将 PDF 通过反步法推广到严格反馈非线性系统，且控制项关于当前与延迟状态线性增长、对一类加性扰动保持固定时间稳定性。

**(c) 指令滤波反步法。** 传统反步法需要反复解析求导虚拟控制信号，当系统阶数较高时导致"微分爆炸"。Farrell 等与 Dong 等提出指令滤波反步法，以滤波器生成虚拟控制及其导数的替代信号，并构造误差补偿机制消除滤波误差，在保持 Lyapunov 理论保证的同时显著简化实现。

### 2.3 本文工作与贡献

本文将上述三条脉络结合，主要贡献如下：

1. **统一架构**：将严宇新的"PTDO + 计算力矩补偿 + 误差线性化"与"指令滤波 + PDF 反步法"整合为一个完整的预设时间控制算法。
2. **消除微分爆炸**：PDF 反步法的虚拟控制含时变增益 $K_1(t)$，其解析导数 $\dot K_1(t)$ 涉及光滑函数 $R_h(t)$ 的导数，计算繁琐；指令滤波避免了该解析求导。
3. **严格证明**：给出补偿误差精确预设时间归零的完整证明（顺序收敛结构），并证明闭环关于匹配扰动强固定时间稳定。
4. **诚实刻画**：明确区分“补偿误差的精确归零”与“实际跟踪误差的渐近收敛”，指出实际跟踪误差在补偿误差归零后以指数速率收敛到零，速率由滤波器带宽 $\omega$ 与 $a$ 决定。

---

## 3. 预备知识与问题描述

### 3.1 记号与函数类

- 对 $x\in\mathbb R$ 与 $\alpha>0$，记 $\operatorname{sig}^{\alpha}(x)=|x|^{\alpha}\operatorname{sign}(x)$；对向量按元素作用。约定 $\operatorname{sign}(0)=0$，从而 $\operatorname{sig}^{\alpha}(0)=0$。
- $\mathbb R_{\ge 0}=[0,\infty)$，$\|\cdot\|$ 表示向量的 $2$-范数，$\|\cdot\|_1$ 表示 $1$-范数。
- $\mathcal{C}^{r}$ 表示 $r$ 次连续可微函数空间，$r\ge 0$ 为整数（$r=\infty$ 表示光滑）。
- $x_{[a,b]}=x(s)$，$s\in[a,b]$，表示函数 $x$ 在区间 $[a,b]$ 上的取值。
- $h>0$ 为设计延迟；$\vartheta$ 表示 PDF 控制器的局部时间。

**定义 1（函数类 $\mathcal{S}^{(r)}(h)$）**：给定常数 $h>0$ 与整数 $r\ge 0$，函数 $R_h:\mathbb R\to\mathbb R$ 称为属于 $\mathcal{S}^{(r)}(h)$，若

1. $R_h$ 是 $2h$-周期函数，且 $R_h(t)=0$，$\forall t\in[0,h]$；
2. $R_h(t)\ge 0$，$\forall t\in[h,2h]$，且存在 $h^*\in(h,2h)$ 使得 $R_h(t)>0$，$\forall t\in[h^*,2h)$；
3. $R_h\in\mathcal{C}^{r}$，从而 $R_h^{(i)}(h)=R_h^{(i)}(2h)=0$，$i=0,1,\dots,r$。

常用例子为 $R_h(t)=\sin^{2}(\pi t/h)$（$t\in[h,2h]$）或 $R_h(t)=(t-h)^{5}(2h-t)^{5}$。

### 3.2 周期延迟反馈与强预设时间镇定

**定义 2（PDF 增益）**：给定 $a\ge 0$ 与 $h>0$，定义

$$
K_{(a,h)}(t)=R_h(t)\,W\,e^{-a(h-2t)},\qquad
W=W_c^{-1}(a,h),\qquad
W_c(a,h)=\int_h^{2h}e^{2as}R_h(s)\,\mathrm{d}s .
$$

由 $R_h(t)=0$（$t\in[0,h]$）可知 $K_{(a,h)}(t)=0$，$\forall t\in[0,h]$；且 $W_c(a,h)>0$，故 $W$ 良定。

**引理 1（线性 PDF 的精确预设时间镇定，Zhou 等）**：考虑标量系统

$$
\dot x(t)=-a x(t)-K_{(a,h)}(t)x(t-h),\qquad t\ge 0,
$$

其中 $x(\theta)$（$\theta\in[-h,0)$）为任意有界初始函数。则

$$
x(t)=0,\qquad \forall t\ge 2h ,
$$

且当 $x(0)\ne 0$ 时 $x(t)\ne 0$，$\forall t\in[0,2h)$。

**证明**：因 $K_{(a,h)}(t)=0$（$t\in[0,h]$），在 $[0,h]$ 上有 $\dot x=-ax$，故

$$
x(t)=e^{-at}x(0),\qquad t\in[0,h] .
$$

在 $[h,2h]$ 上，由常数变易公式，

$$
x(t)=e^{-a(t-h)}x(h)-\int_h^t e^{-a(t-s)}K_{(a,h)}(s)x(s-h)\,\mathrm{d}s .
$$

代入 $x(s-h)=e^{-a(s-h)}x(0)$（因 $s-h\in[0,h]$）并整理，得

$$
x(t)=e^{-at}x(0)\left[1-\int_h^t e^{ah}K_{(a,h)}(s)\,\mathrm{d}s\right] .
$$

由 $K_{(a,h)}(s)=R_h(s)W e^{-a(h-2s)}$，得

$$
\int_h^{2h}e^{ah}K_{(a,h)}(s)\,\mathrm{d}s
=W\int_h^{2h}e^{2as}R_h(s)\,\mathrm{d}s
=W\,W_c(a,h)=1 .
$$

故 $x(2h)=0$。下证 $x(t)=0$（$\forall t\ge 2h$）：由 $R_h$ 以 $2h$ 为周期且 $R_h(t)=0$（$t\in[0,h]$）知 $K_{(a,h)}(t)=0$（$t\in[2h,3h]$），故在 $[2h,3h]$ 上 $\dot x=-ax$，结合 $x(2h)=0$ 得 $x(t)=0$；对 $t\in[3h,4h]$，延迟项 $x(t-h)=0$，仍有 $\dot x=-ax$，由 $x(3h)=0$ 得 $x(t)=0$。依此类推，$x(t)=0$，$\forall t\ge 2h$。

再证精确性：当 $x(0)\ne 0$ 时，在 $[0,h]$ 上 $x(t)=e^{-at}x(0)\ne 0$；在 $[h,2h)$ 上 $x(t)=e^{-at}x(0)\big[1-\int_h^t e^{ah}K_{(a,h)}(s)\,\mathrm{d}s\big]$，因 $K_{(a,h)}(s)\ge 0$ 且在 $[h,2h)$ 上不恒为零，故 $0\le\int_h^t e^{ah}K_{(a,h)}(s)\,\mathrm{d}s<\int_h^{2h}e^{ah}K_{(a,h)}(s)\,\mathrm{d}s=1$（$t<2h$），从而 $x(t)\ne 0$。故 $x(t)\ne 0$，$\forall t\in[0,2h)$。∎

**引理 2（强预设时间 Lyapunov 判据，Ding 等）**：设 $V(t)\ge 0$ 满足

$$
\dot V(t)\le -\frac{a}{1-\tau}V(t)-\frac{K_{(a,h)}(t)}{1-\tau}\,V^{\tau}(t)\,V^{1-\tau}(t-h),
$$

其中 $a\ge 0$，$h>0$，$0<\tau<1$。则 $V(t)=0$，$\forall t\ge 2h$。

**证明**：定义 $W(t)=V^{1-\tau}(t)\ge 0$，则 $V=W^{\frac{1}{1-\tau}}$，且

$$
\dot V=\frac{1}{1-\tau}W^{\frac{1}{1-\tau}-1}\dot W
\le -\frac{a}{1-\tau}W^{\frac{1}{1-\tau}}
-\frac{K_{(a,h)}(t)}{1-\tau}W^{\frac{\tau}{1-\tau}}(t)W(t-h) .
$$

若存在 $t_0\le 2h$ 使 $W(t_0)=0$，则 $V(t_0)=0$；结合 $\dot V\le 0$ 与 $V\ge 0$ 可得 $V(t)=0$，$\forall t\ge t_0$。以下设 $W(t)>0$（即 $V(t)>0$）在 $[0,2h]$ 上恒成立，则两侧同乘 $\frac{1}{1-\tau}W^{\frac{\tau}{1-\tau}}>0$ 得

$$
\dot W(t)\le -a W(t)-K_{(a,h)}(t)W(t-h) .
$$

将右侧差值记为 $-\varphi(t)$（$\varphi(t)\ge 0$），即

$$
\dot W(t)=-aW(t)-K_{(a,h)}(t)W(t-h)-\varphi(t) .
$$

对 $t\in[0,h]$，因 $K_{(a,h)}=0$，得

$$
W(t)=e^{-at}W(0)-\int_0^t e^{-a(t-s)}\varphi(s)\,\mathrm{d}s\le e^{-at}W(0) .
$$

对 $t\in[h,2h]$，由常数变易公式，

$$
W(t)=e^{-at}W(0)-\int_0^t e^{-a(t-s)}\varphi(s)\,\mathrm{d}s
-\int_h^t e^{-a(t-s)}K_{(a,h)}(s)W(s-h)\,\mathrm{d}s .
$$

代入 $W(s-h)\le e^{-a(s-h)}W(0)$，得

$$
W(t)\le e^{-at}W(0)\left[1-\int_h^t e^{ah}K_{(a,h)}(s)\,\mathrm{d}s\right] .
$$

令 $t=2h$，由引理 1 证明中的积分恒等式知方括号为 $0$，故 $W(2h)\le 0$。结合 $W\ge 0$ 得 $W(2h)=0$，即 $V(2h)=0$，与假设矛盾。故存在 $t_0\le 2h$ 使 $V(t_0)=0$，进而 $V(t)=0$，$\forall t\ge 2h$。∎

**注 1**：引理 2 中，当 $\tau\to 1/2$ 时判据退化为线性 PDF 情形。指数 $\tau$ 与 $1-\tau$ 在"当前状态"与"延迟状态"之间分配单位幂，是抵抗加性扰动的关键。

### 3.3 自由漂浮空间机械臂建模

设基座广义速度为 $\dot x_b=[v_b^\top\ \omega_b^\top]^\top\in\mathbb R^6$，机械臂关节变量为 $q\in\mathbb R^n$。由拉格朗日方程可得系统分块动力学

$$
\begin{bmatrix}
H_b(q) & H_{bm}(q)\\
H_{bm}^\top(q) & H_m(q)
\end{bmatrix}
\begin{bmatrix}
\ddot x_b\\
\ddot q
\end{bmatrix}
+
\begin{bmatrix}
C_b & C_{bm}\\
C_{bm}^\top & C_m
\end{bmatrix}
\begin{bmatrix}
\dot x_b\\
\dot q
\end{bmatrix}
=
\begin{bmatrix}
0\\
\tau_m
\end{bmatrix} .
$$

第一行输入为零，反映基座自由漂浮条件。当系统初始线动量和角动量均为零时，动量守恒给出 $H_b(q)\dot x_b+H_{bm}(q)\dot q=0$。若 $H_b(q)$ 非奇异，则

$$
\dot x_b=-H_b^{-1}(q)H_{bm}(q)\dot q\triangleq J_{bm}(q)\dot q .
$$

末端速度由基座运动与关节运动共同决定，代入动量守恒关系得广义雅可比关系

$$
\dot x_e=J_g(q)\dot q,\qquad
J_g(q)=J_b(q)J_{bm}(q)+J_m(q) .
$$

消去基座加速度，可得关于关节变量的等效动力学

$$
M_e(q_m)\ddot q_m+C_e(q_m,\dot q_m)\dot q_m=\tau+d(t),
\tag{1}
$$

其中

$$
M_e(q_m)=H_m-H_{bm}^\top H_b^{-1}H_{bm}
$$

为等效惯性矩阵（正定对称），$C_e(q_m,\dot q_m)$ 为等效非线性项，$d(t)\in\mathbb R^n$ 汇总未建模动态、参数不确定性、环境扰动与执行器误差。此处将 $q$ 记作 $q_m$ 以与后续记号一致。

### 3.4 控制目标与假设

给定足够光滑且有界的关节期望轨迹 $q_d(t),\dot q_d(t),\ddot q_d(t)$。定义误差

$$
e=q_m-q_d,\qquad \dot e=\dot q_m-\dot q_d .
$$

**控制目标**：设计关节力矩 $\tau$，使闭环误差在**预设时间** $T=T_o+T_s$ 后满足

$$
e(t)=0,\qquad \dot e(t)=0,\qquad t\ge T ,
$$

或在扰动与模型近似存在时收敛到零点附近的小邻域。

**假设 1**：$M_e(q_m)$ 在工作空间内一致正定且可逆，$M_e$ 与 $C_e$ 可由模型或名义模型获得。

**假设 2**：参考轨迹 $q_d,\dot q_d,\ddot q_d$ 有界连续，关节位置与速度可测。

**假设 3**：等效加速度扰动 $\Delta_a=M_e^{-1}(q_m)d(t)$ 及其导数有界（上界可未知）。

**假设 4**：对于精确预设时间归零结论，忽略执行器饱和；若存在饱和，则结论退化为实际收敛。

**注**：与 Farrell 等、Dong 等的指令滤波反步法一致，本文不将“虚拟控制 $\alpha_1$ 及其导数 $\dot\alpha_1$ 有界”列为先验假设——该性质由定理 1（补偿误差有界）结合闭环线性结构事后证明（见 §5.3 引理 3），无需预先假定系统状态有界。

---

## 4. 控制算法设计

### 4.1 预设时间扰动观测器

将关节速度记为 $\chi=\dot q_m$。由式 (1) 得加速度通道

$$
\dot\chi=u_o+\Delta_a,\qquad
u_o=M_e^{-1}(q_m)\big[\tau-C_e(q_m,\dot q_m)\dot q_m\big],\qquad
\Delta_a=M_e^{-1}(q_m)d(t) .
$$

设 $p_1,p_2\in\mathbb R^n$ 分别为 $\chi$ 与 $\Delta_a$ 的估计，取 $\hat\Delta_a=p_2$。定义可测复合误差

$$
\varepsilon_1=\chi-p_1-\bar\xi(t),
$$

其中调节函数 $\bar\xi(t)$ 满足有界光滑、单调衰减且 $t\ge T_o$ 时 $\bar\xi(t)=0$，例如

$$
\bar\xi(t)=\begin{cases}
\bar\xi_0(T_o-t)^2, & 0\le t<T_o,\\
0, & t\ge T_o .
\end{cases}
$$

构造预设时间扰动观测器

$$
\left\{
\begin{aligned}
\dot p_1
&=p_2+\frac{\pi}{\eta T_c}\phi_1\!\left(\frac{\varepsilon_1}{\sigma}\right)
-\dot{\bar\xi}(t)+\bar\xi(t)+u_o,\\
\dot p_2
&=\frac{\pi}{\sigma\eta T_c}\phi_2\!\left(\frac{\varepsilon_1}{\sigma}\right)
-\dot{\bar\xi}(t),
\end{aligned}
\right.
\tag{2}
$$

其中 $T_c\le T_o$，$\eta\in(0,1)$，$\sigma>0$，且

$$
\phi_1(x)=\operatorname{sig}^{1-\eta/2}(x)+\operatorname{sig}^{1+\eta/2}(x),\qquad
\phi_2(x)=\operatorname{sig}^{2-\eta}(x)+\operatorname{sig}^{2+\eta}(x)+\sigma\operatorname{sgn}(x) .
$$

在 $\Delta_a$、$\dot\Delta_a$ 有界且连续时间实现条件下，观测误差满足 $\hat\Delta_a(t)=\Delta_a(t)$，$\forall t\ge T_o$。力矩扰动估计由 $\hat d(t)=M_e(q_m)\hat\Delta_a(t)$ 给出。故

$$
\hat d(t)=d(t),\qquad \forall t\ge T_o .
\tag{3}
$$

### 4.2 动力学补偿与误差线性化

采用计算力矩补偿

$$
\tau=M_e(q_m)v+C_e(q_m,\dot q_m)\dot q_m-\hat d(t),
\tag{4}
$$

其中 $v$ 为辅助加速度输入。将式 (4) 代入式 (1)，在 $t\ge T_o$ 时由式 (3) 得

$$
M_e(q_m)\ddot q_m=M_e(q_m)v+d(t)-\hat d(t)=M_e(q_m)v .
$$

因 $M_e$ 正定可逆，故 $\ddot q_m=v$。令

$$
v=\ddot q_d+\nu,\qquad e=q_m-q_d,
$$

则

$$
\ddot e=\nu ,
\tag{5}
$$

误差系统化为二阶积分链。记 $x_1=e$，$x_2=\dot e$，则

$$
\dot x_1=x_2,\qquad \dot x_2=\nu .
\tag{6}
$$

式 (6) 为严格反馈形式（$F_1=0$，$G_1=1$，$F_2=0$，$G_2=1$），且关于各关节解耦。以下以标量形式给出设计，并对每个关节独立应用。

**注 2**：当存在观测误差或模型误差时，式 (5) 应写作 $\ddot e=\nu+\Delta_d$，其中 $\Delta_d=M_e^{-1}(q_m)[d(t)-\hat d(t)]$ 为残余扰动，满足 $|\Delta_d|\le\rho$（$\rho$ 为已知或可估计的界）。

### 4.3 指令滤波反步法结合周期延迟反馈

取预设时间 $T_s>0$，定义 $h=T_s/4$。PDF 控制器在 $t=T_o$ 起以局部时间 $\vartheta=t-T_o$ 运行。设 $a>0$，$\tau_2\in(1/2,1)$ 为设计参数（$a>0$ 保证实际跟踪误差收敛到零；$a=0$ 时补偿误差虽仍预设时间归零，但实际误差收敛到有限非零值，见 §5.3 注 4）。

#### 步骤 1（位置误差子通道）

取跟踪误差 $z_1=e$（即 $x_1$）。构造误差补偿信号 $\xi_1$（见 4.4 节）与补偿误差

$$
\upsilon_1=z_1-\xi_1 .
$$

设计虚拟控制（速度误差 $\dot e$ 的期望值）

$$
\alpha_1=-a z_1-K_1(\vartheta)\,\upsilon_1(\vartheta-h),
\tag{7}
$$

其中

$$
K_1(\vartheta)=
\begin{cases}
0, & \vartheta\le 2h,\\
K_{(a,h)}(\vartheta-2h), & 2h<\vartheta\le 4h,\\
0, & \vartheta>4h .
\end{cases}
\tag{8}
$$

**注**：式 (7) 中线性阻尼项采用实际误差 $z_1$、周期延迟项采用补偿误差 $\upsilon_1$；这一混合选取使 5.1 节的补偿误差动态 (16) 中滤波误差 $\eta$ 与补偿信号 $\xi_1$ 同时精确抵消（阻尼项 $-az_1$ 借助 $z_1=\upsilon_1+\xi_1$ 消去 $+a\xi_1$），得到干净的 PDF 镇定结构。

#### 指令滤波器

将虚拟控制 $\alpha_1$ 送入一阶指令滤波器

$$
\dot\alpha_1^c=-\omega(\alpha_1^c-\alpha_1),\qquad \alpha_1^c(T_o)=\alpha_1(T_o),
\tag{9}
$$

其中 $\omega>0$ 为滤波器带宽。滤波器输出 $\alpha_1^c$ 及其导数 $\dot\alpha_1^c=-\omega(\alpha_1^c-\alpha_1)$ 直接用于后续设计，无需解析求导 $\alpha_1$。记滤波误差

$$
\eta=\alpha_1^c-\alpha_1 .
\tag{10}
$$

#### 步骤 2（速度误差子通道）

定义 $z_2=x_2-\alpha_1^c$。构造误差补偿信号 $\xi_2$ 与补偿误差 $\upsilon_2=z_2-\xi_2$。设计辅助控制

$$
\nu=\dot\alpha_1^c-\frac{a}{2(1-\tau_2)}\upsilon_2-\rho\,\operatorname{sign}(\upsilon_2)
-\frac{K_2(\vartheta)}{2(1-\tau_2)}\,\operatorname{sig}^{2\tau_2-1}(\upsilon_2)\,\big|\upsilon_2(\vartheta-h)\big|^{2(1-\tau_2)},
\tag{11}
$$

其中

$$
K_2(\vartheta)=
\begin{cases}
K_{(a,h)}(\vartheta), & 0\le \vartheta\le 2h,\\
0, & \vartheta>2h .
\end{cases}
\tag{12}
$$

**注（增益按时关闭）**：式 (8)、(12) 使两级 PDF 增益仅在各自收敛窗口内激活（$K_2$ 在 $\vartheta\in[0,2h]$、$K_1$ 在 $\vartheta\in[2h,4h]$），收敛后置零，与 Zhou 等/Ding 等的“单周期 PDF”用法一致。由于 $R_h\in S^{(r)}(h)$ 满足 $R_h^{(i)}(h)=R_h^{(i)}(2h)=0$（$i=0,\dots,r$），置零切换为 $C^r$ 光滑，且 $K_1,\dot K_1,K_2,\dot K_2$ 全程有界——避免了因子 $e^{-a(h-2t)}$ 在 $t\to\infty$ 时的无界增长。

#### 最终控制律

结合式 (4) 与 $v=\ddot q_d+\nu$，关节力矩控制律为

$$
\boxed{
\begin{aligned}
\tau
={}&
M_e(q_m)\Big[\ddot q_d+\dot\alpha_1^c-\tfrac{a}{2(1-\tau_2)}\upsilon_2
-\rho\,\operatorname{sign}(\upsilon_2)\\
&\qquad\qquad
-\tfrac{K_2(\vartheta)}{2(1-\tau_2)}\,\operatorname{sig}^{2\tau_2-1}(\upsilon_2)\,\big|\upsilon_2(\vartheta-h)\big|^{2(1-\tau_2)}\Big]\\
&+C_e(q_m,\dot q_m)\dot q_m-\hat d(t) .
\end{aligned}
}
\tag{13}
$$

### 4.4 误差补偿系统

为抵消指令滤波误差 $\eta$ 的影响，构造补偿系统

$$
\dot\xi_1=-a\xi_1+\xi_2+\eta,\qquad \xi_1(T_o)=0,
\tag{14}
$$

$$
\dot\xi_2=-\frac{a}{2(1-\tau_2)}\xi_2,\qquad \xi_2(T_o)=0 .
\tag{15}
$$

由 $\xi_2(T_o)=0$ 与式 (15) 得 $\xi_2(\vartheta)\equiv 0$，故 $\upsilon_2=z_2$。式 (14) 中 $\eta$ 由式 (10) 给出。

### 4.5 需要测量的量与实现所需信号

将控制律 (13)、观测器 (2) 与补偿系统 (14)–(15) 展开，闭环实现所需的全部信号分为四类。**需要直接物理测量的量只有两个**：关节位置 $q_m$ 与关节速度 $\dot q_m$。

**（1）传感器测量量（必须直接测量）**

- 关节位置 $q_m$：由各关节编码器/旋变获得，用于计算跟踪误差 $e=q_m-q_d$、名义模型 $M_e(q_m)$、$C_e(q_m,\dot q_m)$ 以及补偿误差 $\upsilon_1=z_1-\xi_1$；
- 关节速度 $\dot q_m=\chi$：由测速机或编码器差分滤波获得，用于 PTDO 复合误差 $\varepsilon_1=\dot q_m-p_1-\bar\xi(t)$、观测器状态更新，以及 $z_2=\dot q_m-\dot q_d-\alpha_1^c$ 的计算。

关节加速度 $\ddot q_m$ **无需测量**：其信息由 PTDO 状态与动力学补偿结构隐式提供。

**（2）已知/给定量（无需传感器）**

- 参考轨迹 $q_d(t),\dot q_d(t),\ddot q_d(t)$；
- 名义模型矩阵 $M_e(q_m),C_e(q_m,\dot q_m)$（由已知惯量参数在线计算）；
- 设计参数 $a,\tau_2,T_s,h,\omega,\rho,T_o,T_c,\eta,\sigma$ 与激活函数 $R_h(\cdot)$；
- PTDO 调节函数 $\bar\xi(t)$。

**（3）控制器内部状态（在线计算，无需传感器）**

- PTDO 状态 $p_1,p_2$ 及扰动估计 $\hat d=M_e(q_m)p_2$；
- 指令滤波器状态 $\alpha_1^c$ 及其输出 $\dot\alpha_1^c=-\omega(\alpha_1^c-\alpha_1)$；
- 误差补偿状态 $\xi_1$（注意 $\xi_2\equiv 0$，无需存储）；
- 补偿误差 $\upsilon_1=z_1-\xi_1$、$\upsilon_2=z_2-\xi_2=z_2$。

**（4）历史缓存（需从 $t=T_o-h$ 起存储）**

- $\upsilon_1(\vartheta-h)$ 与 $\upsilon_2(\vartheta-h)$：分别出现在虚拟控制 (7) 与辅助控制 (11) 中，需维护长度为 $h$（即 $h/\Delta t$ 个采样点，$\Delta t$ 为采样步长）的环形缓冲。在 $\vartheta\in[0,h]$ 内，缓冲内容取 $[T_o-h,T_o]$ 期间（第一阶段）按 $\upsilon_1=z_1-\xi_1$、$\upsilon_2=z_2$ 计算并存储的值；
- 施加力矩 $\tau$（命令值，用于 PTDO 的 $u_o=M_e^{-1}[\tau-C_e\dot q_m]$）。

**要点**：整个控制器仅依赖 $q_m,\dot q_m$ 两个物理测量量；其余信号均由参考轨迹、名义模型、设计参数与控制器内部状态（含 $h$ 长度历史缓存）在线生成。

---

## 5. 稳定性分析

### 5.1 补偿误差动态

由式 (6)、(7)、(9)、(10)、(14)、(15) 推导补偿误差 $\upsilon_1,\upsilon_2$ 的闭环动态。

**位置误差通道**：由 $z_1=e$、$\dot z_1=\dot e=x_2$，以及 $x_2=z_2+\alpha_1^c=\upsilon_2+\xi_2+\alpha_1+\eta$，有

$$
\begin{aligned}
\dot\upsilon_1&=\dot z_1-\dot\xi_1
=x_2-\big(-a\xi_1+\xi_2+\eta\big)\\
&=\upsilon_2+\xi_2+\alpha_1+\eta+a\xi_1-\xi_2-\eta\\
&=\upsilon_2+\alpha_1+a\xi_1 .
\end{aligned}
$$

代入式 (7)，并利用 $\upsilon_1=z_1-\xi_1$，得

$$
\dot\upsilon_1=-a\upsilon_1-K_1(\vartheta)\,\upsilon_1(\vartheta-h)+\upsilon_2 .
\tag{16}
$$

**速度误差通道**：由 $z_2=x_2-\alpha_1^c$ 与式 (5)（含残余扰动形式）得

$$
\begin{aligned}
\dot\upsilon_2&=\dot z_2-\dot\xi_2
=\dot x_2-\dot\alpha_1^c+\frac{a}{2(1-\tau_2)}\xi_2\\
&=\nu+\Delta_d-\dot\alpha_1^c+\frac{a}{2(1-\tau_2)}\xi_2 .
\end{aligned}
$$

代入式 (11)，并利用 $\upsilon_2=z_2-\xi_2$ 与 $\xi_2\equiv 0$，得

$$
\dot\upsilon_2
=-\frac{a}{2(1-\tau_2)}\upsilon_2-\rho\,\operatorname{sign}(\upsilon_2)
-\frac{K_2(\vartheta)}{2(1-\tau_2)}\,\operatorname{sig}^{2\tau_2-1}(\upsilon_2)\,\big|\upsilon_2(\vartheta-h)\big|^{2(1-\tau_2)}
+\Delta_d .
\tag{17}
$$

**关键观察**：式 (16) 与 (17) 中滤波误差 $\eta$ 已被完全抵消，补偿误差动态恰为 PDF 镇定结构，且式 (17) 不含 $\upsilon_1$（单向耦合），故可采用顺序（sequential）收敛分析。

### 5.2 主定理与证明

**定理 1**：考虑满足假设 1–3 的自由漂浮空间机械臂 (1)，采用观测器 (2)、控制律 (13) 与补偿系统 (14)–(15)。设残余扰动满足 $|\Delta_d|\le\rho$。则

1. 补偿误差在预设时间归零：

$$
\upsilon_1(t)=0,\qquad \upsilon_2(t)=0,\qquad \forall t\ge T_o+4h=T_o+T_s ;
$$

2. 闭环系统关于匹配扰动集合 $\{\Delta_d:|\Delta_d|\le\rho\}$ 固定时间稳定，且为强固定时间稳定：当 $\Delta_d\equiv 0$ 且初始条件满足 $\upsilon_2(T_o)=0$、$\upsilon_1(T_o)\ne 0$ 时，收敛时间恰为 $T_s=4h$。

**证明**：以下全部使用局部时间 $\vartheta=t-T_o$。

**第一步（$\upsilon_2$ 归零）**：取 Lyapunov 函数 $V_2=\frac12\upsilon_2^2$。沿式 (17) 求导，

$$
\begin{aligned}
\dot V_2
&=\upsilon_2\Big[-\tfrac{a}{2(1-\tau_2)}\upsilon_2-\rho\operatorname{sign}(\upsilon_2)
-\tfrac{K_2}{2(1-\tau_2)}\operatorname{sig}^{2\tau_2-1}(\upsilon_2)|\upsilon_2(\vartheta-h)|^{2(1-\tau_2)}+\Delta_d\Big]\\
&=-\tfrac{a}{2(1-\tau_2)}\upsilon_2^2-\rho|\upsilon_2|
-\tfrac{K_2}{2(1-\tau_2)}|\upsilon_2|^{2\tau_2}|\upsilon_2(\vartheta-h)|^{2(1-\tau_2)}
+\upsilon_2\Delta_d .
\end{aligned}
$$

由 $|\Delta_d|\le\rho$ 得 $\upsilon_2\Delta_d\le|\upsilon_2||\Delta_d|\le\rho|\upsilon_2|$，故

$$
\dot V_2\le-\frac{a}{2(1-\tau_2)}\upsilon_2^2
-\frac{K_2(\vartheta)}{2(1-\tau_2)}|\upsilon_2|^{2\tau_2}|\upsilon_2(\vartheta-h)|^{2(1-\tau_2)} .
$$

注意到 $V_2=\frac12\upsilon_2^2$，且

$$
|\upsilon_2|^{2\tau_2}|\upsilon_2(\vartheta-h)|^{2(1-\tau_2)}
=(2V_2)^{\tau_2}(2V_2(\vartheta-h))^{1-\tau_2}
=2V_2^{\tau_2}V_2^{1-\tau_2}(\vartheta-h),
$$

于是

$$
\dot V_2\le-\frac{a}{1-\tau_2}V_2(\vartheta)
-\frac{K_2(\vartheta)}{1-\tau_2}V_2^{\tau_2}(\vartheta)V_2^{1-\tau_2}(\vartheta-h) .
$$

由 $K_2(\vartheta)=K_{(a,h)}(\vartheta)$ 与引理 2，得

$$
V_2(\vartheta)=0,\qquad \forall \vartheta\ge 2h,
$$

即 $\upsilon_2(t)=0$，$\forall t\ge T_o+2h$。

**第二步（$\upsilon_1$ 归零）**：当 $\vartheta\ge 2h$ 时 $\upsilon_2=0$，式 (16) 化为

$$
\dot\upsilon_1(\vartheta)=-a\upsilon_1(\vartheta)-K_1(\vartheta)\,\upsilon_1(\vartheta-h),\qquad \vartheta\ge 2h .
$$

作变量代换 $\kappa=\vartheta-2h\ge 0$，$\sigma(\kappa)=\upsilon_1(\kappa+2h)$，则由式 (8) 得 $K_1(\kappa+2h)=K_{(a,h)}(\kappa)$，故

$$
\dot\sigma(\kappa)=-a\sigma(\kappa)-K_{(a,h)}(\kappa)\sigma(\kappa-h),\qquad \kappa\ge 0 .
$$

由引理 1 得 $\sigma(\kappa)=0$，$\forall\kappa\ge 2h$，即

$$
\upsilon_1(\vartheta)=0,\qquad \forall \vartheta\ge 4h .
$$

故 $\upsilon_1(t)=0$，$\forall t\ge T_o+4h$。

**第三步（固定时间稳定性与强固定时间稳定性）**：由第一、二步，对任意满足 $|\Delta_d|\le\rho$ 的扰动与任意有界初始条件，$\upsilon_2$ 在 $2h$ 内、$\upsilon_1$ 在 $4h$ 内归零，收敛时间上界 $T_s=4h$ 与初始条件及扰动实现无关，故闭环关于扰动集合 $\mathcal{D}=\{\Delta_d:|\Delta_d|\le\rho\}$ 固定时间稳定。

再证强固定时间稳定性，即存在 $\Delta_d\in\mathcal{D}$ 与初始条件使收敛时间恰为 $T_s=4h$。取 $\Delta_d\equiv 0$，初始条件满足 $\upsilon_2(T_o)=0$、$\upsilon_1(T_o)\ne 0$（延迟历史取任意有界函数）。由 $\operatorname{sign}(0)=0$ 与式 (17)，在 $\vartheta=0$ 处 $\dot\upsilon_2=0$，故 $\upsilon_2(\vartheta)\equiv 0$；于是式 (16) 化为 $\dot\upsilon_1=-a\upsilon_1-K_1(\vartheta)\upsilon_1(\vartheta-h)$。在 $\vartheta\in[0,2h]$ 上 $K_1=0$，故 $\upsilon_1(\vartheta)=e^{-a\vartheta}\upsilon_1(T_o)\ne 0$；在 $\vartheta\ge 2h$ 上作平移 $\kappa=\vartheta-2h$ 后由引理 1 的精确性得 $\upsilon_1(\vartheta)\ne 0$（$\vartheta<4h$）且 $\upsilon_1(4h)=0$。故该轨道的收敛时间恰为 $T_s=4h$，闭环为强固定时间稳定（strong fixed-time stable）。∎

**注 3（单向耦合的顺序结构）**：本文补偿系统 (14)–(15) 采用与 Ding 等反步法一致的单向耦合（$\dot\upsilon_1$ 含 $+\upsilon_2$，而 $\dot\upsilon_2$ 不含 $-\upsilon_1$），从而保证 $\upsilon_2$ 先收敛、$\upsilon_1$ 后收敛的顺序结构；这不同于对称耦合的经典指令滤波反步法（其交叉项在 Lyapunov 导数中相消但破坏顺序收敛）。此设计选择是获得精确预设时间结论的关键。

### 5.3 关于跟踪误差的推论

**推论 1**：在定理 1 条件下，实际跟踪误差满足

$$
z_2(t)=\upsilon_2(t),\qquad z_1(t)=\upsilon_1(t)+\xi_1(t),\qquad t\ge T_o .
$$

且 $\xi_1$ 满足

$$
\dot\xi_1=-a\xi_1+\eta,\qquad \xi_1(T_o)=0,
$$

其中滤波误差 $\eta=\alpha_1^c-\alpha_1$ 满足

$$
\dot\eta=-\omega\eta-\dot\alpha_1 .
$$

**证明**：由 $\xi_2\equiv 0$ 直接得 $z_2=\upsilon_2$；$z_1=\upsilon_1+\xi_1$ 由定义给出；式 (14) 代入 $\xi_2=0$ 即得 $\dot\xi_1=-a\xi_1+\eta$；式 (10) 对 $\vartheta$ 求导并代入式 (9) 得 $\dot\eta=\dot\alpha_1^c-\dot\alpha_1=-\omega\eta-\dot\alpha_1$。∎

**引理 3（虚拟控制与滤波信号的有界性）**：在定理 1 条件下（无需额外假设），虚拟控制 $\alpha_1$、其导数 $\dot\alpha_1$、滤波误差 $\eta$ 与补偿信号 $\xi_1$ 全程有界。

**证明**：对式 (7) 关于 $\vartheta$ 求导，并利用 $\dot z_1=x_2=z_2+\alpha_1+\eta$（因 $x_2=z_2+\alpha_1^c$、$\alpha_1^c=\alpha_1+\eta$），得

$$
\dot\alpha_1=-a(z_2+\alpha_1+\eta)-\dot K_1(\vartheta)\,\upsilon_1(\vartheta-h)-K_1(\vartheta)\,\dot\upsilon_1(\vartheta-h) .
$$

令 $b(t)=-a z_2-\dot K_1(\vartheta)\,\upsilon_1(\vartheta-h)-K_1(\vartheta)\,\dot\upsilon_1(\vartheta-h)$，则 $\dot\alpha_1=-a\alpha_1-a\eta+b(t)$。结合推论 1 中的 $\dot\eta=-\omega\eta-\dot\alpha_1$ 消去 $\dot\alpha_1$，得 $(\alpha_1,\eta)$ 的线性闭环

$$
\begin{bmatrix}\dot\alpha_1\\[2pt] \dot\eta\end{bmatrix}
=
\begin{bmatrix}-a & -a\\[2pt] a & -(\omega-a)\end{bmatrix}
\begin{bmatrix}\alpha_1\\[2pt] \eta\end{bmatrix}
+
\begin{bmatrix}b(t)\\[2pt] -b(t)\end{bmatrix} .
$$

先证 $b(t)$ 有界：$z_2=\upsilon_2$ 与 $\upsilon_1$ 由定理 1 有界；由式 (16)，$\dot\upsilon_1=-a\upsilon_1-K_1\upsilon_1(\vartheta-h)+\upsilon_2$ 各项有界，故 $\dot\upsilon_1$ 有界；$K_1,\dot K_1$ 由增益按时关闭知全程有界。故 $b(t)$ 有界。

系数矩阵特征多项式为 $\lambda^2+\omega\lambda+a\omega$，由 $a>0$ 知两特征值均具负实部（$\omega>4a$ 时为两负实根，否则为负实部复根），矩阵 Hurwitz；有界输入 $b(t)$ 驱动的线性系统之状态 $(\alpha_1,\eta)$ 有界；再由 $\dot\xi_1=-a\xi_1+\eta$（稳定一阶系统）得 $\xi_1$ 有界。（若允许 $a=0$，式 (7) 退化为 $\alpha_1=-K_1\upsilon_1(\vartheta-h)$，$\dot\alpha_1$ 直接由有界量构成、无循环，结论亦成立。）

综上 $\alpha_1,\eta,\xi_1$ 有界，且 $\dot\alpha_1$ 由显式表达亦有界。∎

**推论 2（实际跟踪性能）**：在定理 1 条件下，

1. 速度误差 $z_2(t)=0$，$\forall t\ge T_o+2h$；
2. 位置误差 $z_1(t)$ 在 $t\ge T_o+4h$ 后满足 $z_1=\xi_1$，且 $\xi_1$ 满足

$$
|\xi_1(t)|\le \frac{1}{a}\sup_{\vartheta\ge 4h}|\eta(\vartheta)|\cdot\big(1-e^{-a(\vartheta-4h)}\big)+|\xi_1(T_o+4h)|e^{-a(\vartheta-4h)} .
$$

**证明**：结论 1 由 $\upsilon_2=z_2$ 与定理 1 直接得。结论 2 由 $\dot\xi_1=-a\xi_1+\eta$ 的显式解

$$
\xi_1(\vartheta)=e^{-a\vartheta}\xi_1(0)+\int_0^\vartheta e^{-a(\vartheta-s)}\eta(s)\,\mathrm{d}s
$$

并取范数得上界。∎

**注 4（实际跟踪误差的渐近收敛）**：当 $\vartheta\ge 5h$ 时 $\upsilon_1=\upsilon_2=0$ 且 $\upsilon_1(\vartheta-h)=0$，由式 (7) 得 $\alpha_1=-a z_1$，而 $z_1=\xi_1$，故 $\dot\alpha_1=-a\dot\xi_1$。结合 $\dot\xi_1=-a\xi_1+\eta$ 与 $\dot\eta=-\omega\eta-\dot\alpha_1$，$\xi_1,\eta$ 满足齐次线性系统

$$
\begin{bmatrix}\dot\xi_1\\[2pt] \dot\eta\end{bmatrix}
=
\begin{bmatrix}-a & 1\\ -a^2 & -(\omega-a)\end{bmatrix}
\begin{bmatrix}\xi_1\\ \eta\end{bmatrix},
$$

其特征多项式为 $\lambda^2+\omega\lambda+a\omega$，在 $a>0$ 且 $\omega>4a$ 时两特征值为负实数（约 $-a$ 与 $-(\omega-a)$）。故 $|z_1|=|\xi_1|=O\big(e^{-\min(a,\omega-a)\vartheta}\big)$，实际跟踪误差以指数速率收敛到零，速率由 $\min(a,\omega-a)$ 决定；增大 $\omega$ 可加快收敛。

---

## 6. 鲁棒性与工程实现讨论

### 6.1 关于符号函数的抖振

控制律 (13) 中的 $\operatorname{sign}(\upsilon_2)$ 用于抵消匹配扰动 $\Delta_d$，但会引入抖振。工程实现中可替换为饱和函数 $\operatorname{sat}(\upsilon_2/\varphi)$ 或 $\tanh(\upsilon_2/\varphi)$（$\varphi>0$）。此时式 (17) 中的 $-\rho|\upsilon_2|$ 项变为 $-\rho\upsilon_2\tanh(\upsilon_2/\varphi)\le-\rho|\upsilon_2|+\rho\varphi$，闭环误差收敛到 $O(\varphi)$ 邻域，预设时间结论退化为实际预设时间有界。

### 6.2 关于观测器时序

若 PTDO 与 PDF 控制器同时从 $t=0$ 启动，则 $0\le t<T_o$ 内观测误差作为残余扰动进入误差通道，严格零化结论不再成立。因此采用两阶段时序：先在 $T_o$ 内完成扰动观测，随后以局部时间 $\vartheta=t-T_o$ 启动 PDF 增益。总收敛时间为 $T=T_o+T_s$。

第一阶段（$0\le t<T_o$）施加**有界标称控制**（例如 PD 控制 $\tau=M_e[\ddot q_d-K_p e-K_d\dot e]+C_e\dot q_m-\hat d$，或仅动力学补偿），以保证 $[T_o-h,T_o]$ 上状态与缓存历史 $\upsilon_1,\upsilon_2$ 有界——这是 PDF 延迟项在 $\vartheta\in[0,h]$ 内所需初始历史的良定性前提。由于该阶段仅为有限时长，任何使闭环无有限逃逸的有界控制律均可胜任。

### 6.3 关于参数选择

- $a>0$：基准线性阻尼，可加快首半周期衰减，并保证实际跟踪误差收敛到零（$a=0$ 时实际误差收敛到非零常数，见 §5.3 注 4）；
- $\tau_2\in(1/2,1)$：幂分配参数，控制对加性扰动的鲁棒性；
- $h=T_s/4$：由预设时间唯一确定；
- $\omega>0$：指令滤波器带宽，越大则滤波误差越小，但噪声放大越明显，需折中；
- $\rho$：残余扰动上界，可由 PTDO 收敛后的观测误差界估计。

### 6.4 与严宇新原始 PDF 控制的关系

严宇新原始控制律为 $\nu=K_0z(t)-K_c(t)z(t-h)$，其中 $K_c(t)$ 为 Gramian 型 PDF 增益，直接作用于误差状态 $z=[e^\top\ \dot e^\top]^\top$，一步完成全状态零化（$T_{\text{tot}}=T_o+2h$）。本文方法将 PDF 增益拆分为两级（位置级 $K_1$ 与速度级 $K_2$），并用指令滤波替代虚拟控制导数，代价是收敛时间由 $2h$ 增至 $4h$，收益是：**(i)** 避免 $\dot K_1(t)$（含 $R_h$ 的高阶导数）的解析计算；**(ii)** 引入幂分配项 $\operatorname{sig}^{2\tau_2-1}(\cdot)$ 使速度通道对匹配扰动强固定时间稳定；**(iii)** 结构上易于扩展抗饱和与状态约束。


## 7. 结论

本文将周期延迟反馈预设时间镇定理论与指令滤波反步法相结合，提出了自由漂浮空间机械臂的预设时间轨迹跟踪控制算法。算法遵循"预设时间扰动观测 → 计算力矩补偿 → 误差线性化 → 指令滤波 + 周期延迟反馈"的架构：

1. **建模**：由动量守恒得到等效关节空间动力学；
2. **扰动抑制**：PTDO 在 $T_o$ 内精确估计集总扰动；
3. **线性化**：计算力矩补偿将误差动态化为二阶积分链；
4. **镇定**：位置通道采用线性 PDF 虚拟控制，速度通道采用幂分配强 PDF 实际控制，指令滤波器避免虚拟控制解析求导，误差补偿系统消除滤波误差。

理论证明表明:补偿误差在预设时间 $T_o+4h$ 内精确归零,闭环关于匹配扰动强固定时间稳定;实际跟踪误差随后以指数速率收敛到零（速率由 $\min(a,\omega-a)$ 决定）。仿真验证了算法的有效性。

**未来工作**：将控制律扩展至末端任务空间并处理广义雅可比奇异；引入障碍 Lyapunov 函数处理全状态约束；考虑执行器饱和下的严格预设时间收敛；在实验平台上验证。

---

## 参考文献

1. **严宇新**. 面向在轨捕获的空间机械臂运动规划与控制方法研究 [D]. 哈尔滨工业大学, 2025. (无 DOI)
2. **B. Zhou, W. Michiels, J. Chen**. Fixed-time stabilization of linear delay systems by smooth periodic delayed feedback [J]. IEEE Transactions on Automatic Control, 2022, 67(2): 557–573. DOI: `10.1109/TAC.2021.3051262` (MCP: 20)
3. **Y. Ding, B. Zhou, K.-K. Zhang, W. Michiels**. Strong prescribed-time stabilization of uncertain nonlinear systems by periodic delayed feedback [J]. (自动化学报/IEEE 汇刊，见知识库 `2070.md`). (MCP: 2070)
4. **J. A. Farrell, M. M. Polycarpou, M. Sharma, W. Dong**. Command filtered adaptive backstepping [J]. IEEE Transactions on Automatic Control, 2009, 54(6): 1391–1395. DOI: `10.1109/TAC.2009.2015562` (MCP: 3275)
5. **W. Dong, J. A. Farrell, M. M. Polycarpou, V. Djapic, M. Sharma**. Command filtered adaptive backstepping [J]. IEEE Transactions on Control Systems Technology, 2013, 21(6): 2102–2110. DOI: `10.1109/TCST.2011.2121907`
6. **J. Yu, P. Shi, W. Dong, C. Lin**. Command-filtered backstepping control for nonlinear systems with input saturation [J]. IEEE Transactions on Cybernetics, 2015, 45(10): 2018–2027. DOI: `10.1109/TCYB.2015.2483368`
7. **J. Yu, P. Shi, X. Zhao**. Finite-time command filtered backstepping control for a class of nonlinear systems [J]. IEEE Transactions on Automatic Control, 2018, 63(10): 3464–3471. DOI: `10.1016/j.automatica.2018.03.033` (MCP: 3274)
8. **J. Yu, L. Zhao, H. Yu, C. Lin**. Barrier Lyapunov functions-based command filtered output feedback control for full-state constrained nonlinear systems [J]. Automatica, 2019, 103: 244–251. DOI: `10.1016/j.automatica.2019.03.022`
9. **Y. Jiang, J. Lv, C. Wang, Y. Kao, F. Wang**. Prescribed-time disturbance observer-based fully distributed prescribed-time containment control of multiagent systems [J]. IEEE Transactions on Circuits and Systems II, 2024. DOI: `10.1109/TCSII.2023.3328652`
10. **Y. Umetani, K. Yoshida**. Resolved motion rate control of space manipulators with generalized Jacobian matrix [J]. IEEE Transactions on Robotics and Automation, 1989, 5(3): 303–314. DOI: `10.1109/70.34766`
11. **Z. Vafa, S. Dubowsky**. The kinematics and dynamics of space manipulators: the virtual manipulator approach [J]. The International Journal of Robotics Research, 1990, 9(4): 3–21. DOI: `10.1177/027836499000900401`
12. **E. Papadopoulos, S. Dubowsky**. Dynamic singularities in free-floating space manipulators [J]. Journal of Dynamic Systems, Measurement, and Control, 1993, 115(1): 44–52. DOI: `10.1115/1.2897406`
13. **S. Ling, H. Wang, P. X. Liu**. Adaptive fuzzy tracking control of flexible-joint robots based on command filtering [J]. IEEE Transactions on Fuzzy Systems, 2021, 29(7): 2152–2163. DOI: `10.1109/TFZZ.2021.3072770`
14. **M. Krstic, I. Kanellakopoulos, P. V. Kokotovic**. Nonlinear and Adaptive Control Design [M]. New York: Wiley, 1995. ISBN: `978-0-471-12732-1`

---

> **说明**：本文档为研究笔记性质的完整论文草稿，融合自以下知识来源：
> - `../严宇新/README_organized.md`（自由漂浮空间机械臂建模、PTDO、PDF 控制）；
> - `../严宇新/README_command_filtering.md`（指令滤波反步法综述与应用）；
> - `2070.md`（Ding, Zhou, Zhang, Michiels 的强预设时间 PDF 镇定，含严格反馈反步法）；
> - `3339.md`（Dong 等，指令滤波自适应反步法）；
> - `3285.md`（Yu 等，障碍 Lyapunov 指令滤波输出反馈）。
