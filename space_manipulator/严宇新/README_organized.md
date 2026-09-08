# 基于光滑周期延迟反馈的自由漂浮空间机械臂预设时间轨迹跟踪控制

> 本文整理自严宇新论文及相关研究工作，涵盖自由漂浮空间机械臂建模、控制律设计、理论分析与仿真验证。

---

## 目录

1. [引言](#1-引言)
2. [自由漂浮空间机械臂建模](#2-自由漂浮空间机械臂建模)
3. [问题描述](#3-问题描述)
4. [控制算法设计](#4-控制算法设计)
   - [4.1 预设时间扰动观测器](#41-预设时间扰动观测器)
   - [4.2 动力学补偿与误差通道规范化](#42-动力学补偿与误差通道规范化)
   - [4.3 PDF 线性误差系统设计](#43-pdf-线性误差系统设计)
   - [4.4 关节力矩控制律](#44-关节力矩控制律)
5. [稳定性与性能分析](#5-稳定性与性能分析)
6. [仿真验证](#6-仿真验证)
7. [结论](#7-结论)
8. [参考文献](#8-参考文献)
9. [仿真代码概览](#9-仿真代码概览)

---

## 1. 引言

随着在轨服务、空间碎片清理和失效航天器维护需求的增长，空间机械臂已成为执行非合作目标接近、捕获与操作任务的重要装备。与地面固定基座机械臂不同，捕获前空间机械臂通常处于自由漂浮状态，基座不直接受控，机械臂关节运动会通过系统动量守恒关系诱导基座线速度和角速度变化，进而改变末端执行器的实际运动。

严宇新针对在轨捕获任务指出，空间机械臂动力学模型具有高度非线性和强耦合特性，并且轨迹跟踪过程中还会受到模型参数不确定、未知时变扰动和执行器故障等因素影响。因此，如何在基座不直接施加控制力矩的条件下实现高精度、可预测收敛时间的轨迹跟踪，是自由漂浮空间机械臂控制中的关键问题。

### 1.1 自由漂浮空间机械臂的控制难点

- **非完整约束**：在线动量和角动量初值为零的自由漂浮模式下，基座速度可由机械臂关节速度唯一诱导，末端速度不再由固定基座雅可比矩阵决定，而应由包含基座反作用运动的广义雅可比矩阵描述。
- **动力学耦合**：基座与机械臂之间存在强烈的动力学耦合，导致系统表现出动态奇异特性。
- **不确定性**：模型参数不确定、未知时变扰动和执行器误差等。

### 1.2 相关研究

- Umetani 和 Yoshida 提出广义雅可比矩阵，为自由漂浮空间机械臂的速度级建模奠定基础。
- Vafa 和 Dubowsky 的虚拟机械臂方法从动量守恒角度系统刻画了空间机械臂运动学与动力学耦合。
- Papadopoulos 和 Dubowsky 揭示了自由漂浮空间机械臂中的动态奇异问题。
- 近年来，固定时间、预定义时间和指定时间控制被用于自由漂浮空间机械臂轨迹跟踪。

### 1.3 本文方法

本文提出一种基于**光滑周期延迟反馈（smooth periodic delayed feedback, PDF）**的关节空间轨迹跟踪控制方法。核心思路是"扰动观测 → 动力学补偿 → 误差线性化 → 周期延迟反馈"：

1. 依据动量守恒关系将捕获前自由漂浮空间机械臂约化为等效关节空间模型；
2. 通过预设时间扰动观测器得到集总扰动估计；
3. 利用计算力矩补偿得到从辅助加速度到跟踪误差的线性可控通道；
4. 在该误差通道中嵌入光滑周期延迟反馈。

### 1.4 本文主要工作

- 给出从基座反作用运动到广义雅可比矩阵、等效关节空间动力学的推导链条；
- 引入预设时间扰动观测补偿机制；
- 构造面向 PDF 设计的误差通道规范化方法；
- 结合 PDF 理论中的 \(R_h(t)\)、可控 Gramian 和单周期状态转移矩阵零化条件，给出关节力矩控制律；
- 基于七自由度空间机械臂参数搭建 MATLAB 仿真验证。

---

## 2. 自由漂浮空间机械臂建模

### 2.1 系统动力学

设基座广义速度为 \(\dot x_b=[v_b^\top\ \omega_b^\top]^\top\in\mathbb R^6\)，机械臂关节变量为 \(q\in\mathbb R^n\)，系统总动能可写为：

\[
T=\frac12
\begin{bmatrix}
\dot x_b^\top & \dot q^\top
\end{bmatrix}
\begin{bmatrix}
H_b(q) & H_{bm}(q)\\
H_{bm}^\top(q) & H_m(q)
\end{bmatrix}
\begin{bmatrix}
\dot x_b\\
\dot q
\end{bmatrix},
\]

其中 \(H_b\) 为基座惯性张量，\(H_m\) 为固定基座机械臂惯性张量，\(H_{bm}\) 为基座与机械臂之间的耦合惯性张量。

由拉格朗日方程可得系统分块动力学：

\[
\begin{bmatrix}
H_b & H_{bm}\\
H_{bm}^\top & H_m
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
\end{bmatrix},
\]

其中 \(\tau_m\in\mathbb R^n\) 为关节驱动力矩。第一行输入为零，反映了捕获前基座不施加外力和外力矩的自由漂浮条件。

### 2.2 动量守恒约束

当系统初始线动量和角动量均为零时，动量守恒方程可写为：

\[
H_b(q)\dot x_b+H_{bm}(q)\dot q=0.
\]

若 \(H_b(q)\) 非奇异，则有：

\[
\dot x_b=-H_b^{-1}(q)H_{bm}(q)\dot q
\triangleq J_{bm}(q)\dot q.
\]

该式说明基座速度不是独立可控变量，而是由关节速度通过基座--机械臂雅可比矩阵 \(J_{bm}\) 诱导产生。

### 2.3 广义雅可比矩阵

末端速度由基座运动和机械臂关节运动共同决定：

\[
\dot x_e=J_b(q)\dot x_b+J_m(q)\dot q.
\]

将动量守恒关系代入，得到自由漂浮空间机械臂的广义雅可比关系：

\[
\dot x_e=\left[J_b(q)J_{bm}(q)+J_m(q)\right]\dot q
\triangleq J_g(q)\dot q.
\]

\(J_g(q)\) 不仅依赖机械臂构型，还与基座姿态、连杆质量和惯性参数相关；当 \(J_g\) 接近奇异时，末端轨迹跟踪会受到关节速度放大和基座反作用耦合的共同影响。

### 2.4 等效关节空间动力学

由动量守恒方程对时间求导并代入系统动力学，可消去基座加速度，得到关于关节变量的等效动力学：

\[
H_g(q)\ddot q+C_g(q,\dot q)\dot q=\tau_m+d(t),
\]

其中

\[
H_g(q)=H_m(q)-H_{bm}^\top(q)H_b^{-1}(q)H_{bm}(q),
\]

\(C_g\) 为等效非线性项，\(d(t)\) 汇总未建模动态、参数不确定性、环境扰动和执行器误差。

为与后续控制设计记号一致，将 \(q\) 记作 \(q_m\)，将 \(H_g,C_g\) 分别记作 \(M_e,C_e\)：

\[
M_e(q_m)\ddot q_m+C_e(q_m,\dot q_m)\dot q_m=\tau+d(t).
\]

该模型保留了自由漂浮基座对关节动力学的等效影响，同时为计算力矩补偿和误差线性化提供了接口。

---

## 3. 问题描述

给定足够光滑且有界的关节期望轨迹 \(q_d(t),\dot q_d(t),\ddot q_d(t)\)。定义关节位置误差和速度误差为：

\[
e=q_m-q_d,\qquad \dot e=\dot q_m-\dot q_d,
\]

并令误差状态：

\[
z=\begin{bmatrix}e^\top & \dot e^\top\end{bmatrix}^\top\in\mathbb R^{2n}.
\]

**控制目标**：在捕获前自由漂浮条件下设计关节力矩 \(\tau\)，使闭环误差在预设时间 \(T\) 后满足：

\[
e(t)=0,\qquad \dot e(t)=0,\qquad t\ge T,
\]

或在存在扰动和模型近似时收敛到零点附近的小邻域。

### 假设条件

- **假设1**：\(M_e(q_m)\) 在工作空间内一致正定且可逆，\(M_e(q_m)\) 与 \(C_e(q_m,\dot q_m)\) 可由模型获得或可由名义模型近似。
- **假设2**：参考轨迹 \(q_d,\dot q_d,\ddot q_d\) 有界且连续，关节位置和速度可测。
- **假设3**：人工延迟项 \(z(t-h)\) 可由历史缓存获得，其中 \(h>0\) 为设计延迟。
- **假设4**：若采用预设时间扰动观测器，则等效加速度扰动 \(\Delta_a=M_e^{-1}(q_m)d(t)\) 及其导数有界，但上界可未知。
- **假设5**：对于严格固定时间精确归零结论，暂不考虑执行器饱和和模型误差，且扰动能够被精确补偿；若存在观测误差、模型近似或持续未补偿扰动，则讨论实际固定时间收敛性能。

---

## 4. 控制算法设计

### 4.1 预设时间扰动观测器

对等效关节空间模型，先设计扰动观测器来生成后续动力学补偿所需的 \(\hat d(t)\)。将关节速度记为 \(\chi=\dot q_m\)，由等效模型可得：

\[
\dot\chi=u_o+\Delta_a,
\qquad
u_o=M_e^{-1}(q_m)\bigl[\tau-C_e(q_m,\dot q_m)\dot q_m\bigr],
\qquad
\Delta_a=M_e^{-1}(q_m)d(t).
\]

其中 \(\Delta_a\) 为匹配到加速度通道的集总扰动。若能够在预设时间 \(T_o\) 内得到 \(\hat\Delta_a\)，则力矩扰动估计可由 \(\hat d(t)=M_e(q_m)\hat\Delta_a(t)\) 给出。

设 \(z_1,z_2\in\mathbb R^n\) 分别为 \(\chi\) 和 \(\Delta_a\) 的估计值，取 \(\hat\Delta_a=z_2\)。定义可测复合误差：

\[
\varepsilon_1=\chi-z_1-\bar\xi(t),
\]

其中 \(\bar\xi(t)\in\mathbb R^n\) 为调节函数，满足有界光滑、单调衰减且 \(t\ge T_o\) 时 \(\bar\xi(t)=0\)。例如可取：

\[
\bar\xi(t)=
\begin{cases}
\bar\xi_0(T_o-t)^2, & 0\le t<T_o,\\
0, & t\ge T_o.
\end{cases}
\]

记 \(\operatorname{sig}^{\alpha}(x)=\operatorname{sgn}(x)|x|^\alpha\)，并按元素作用于向量。构造如下预设时间扰动观测器：

\[
\left\{
\begin{aligned}
\dot z_1
&=z_2+\frac{\pi}{\eta T_c}
\phi_1\left(\frac{\varepsilon_1}{\sigma}\right)
-\dot{\bar\xi}(t)+\bar\xi(t)+u_o,\\
\dot z_2
&=\frac{\pi}{\sigma\eta T_c}
\phi_2\left(\frac{\varepsilon_1}{\sigma}\right)
-\dot{\bar\xi}(t),
\end{aligned}
\right.
\]

其中 \(T_c\le T_o\)、\(\eta\in(0,1)\)、\(\sigma>0\)，且：

\[
\begin{aligned}
\phi_1(x)&=\operatorname{sig}^{1-\eta/2}(x)+\operatorname{sig}^{1+\eta/2}(x),\\
\phi_2(x)&=\operatorname{sig}^{2-\eta}(x)+\operatorname{sig}^{2+\eta}(x)+\sigma\operatorname{sgn}(x).
\end{aligned}
\]

在 \(\Delta_a\) 与 \(\dot\Delta_a\) 有界且连续时间实现的条件下，观测误差可在 \(t=T_o\) 收敛到零。

### 4.2 动力学补偿与误差通道规范化

在获得 \(\hat d(t)\) 后，采用如下动力学补偿结构：

\[
\tau=M_e(q_m)v+C_e(q_m,\dot q_m)\dot q_m-\hat d(t),
\]

其中 \(v\) 为辅助加速度输入。当观测器已收敛并满足 \(\hat d(t)=d(t)\) 时，且 \(M_e(q_m)\) 正定可逆，因此 \(\ddot q_m=v\)。

令 \(v=\ddot q_d+\nu\)，并定义 \(e=q_m-q_d\)，\(z=[e, \dot e]^T\)。则 \(\ddot e=\ddot q_m-\ddot q_d=\nu\)，误差系统可写成二阶积分链形式：

\[
\dot z=Az+B\nu,
\]

其中

\[
A=\begin{bmatrix}0&I_n\\0&0\end{bmatrix},\qquad
B=\begin{bmatrix}0\\I_n\end{bmatrix}.
\]

矩阵对 \((A,B)\) 可控，因此 PDF 增益的设计对象由原非线性动力学转移为线性可控误差通道。这个步骤是本文方法成立的关键：PDF 理论只需作用在线性误差系统上，而不直接处理自由漂浮机械臂的非线性耦合项。

### 4.3 PDF 线性误差系统设计

文献中的基本对象是可控线性时滞系统，其固定时间镇定依赖周期延迟结构：

\[
\dot x(t)=F(t)x(t)+G(t)x(t-h),
\]

其中 \(F(t),G(t)\) 为 \(2h\) 周期矩阵，且 \(G(t)=0\) 在每个周期的前半段 \([0,h]\) 成立。由分步法可知，系统在一个周期后的状态满足 \(x(2(k+1)h)=\Delta(h)x(2kh)\)，其中单周期状态转移矩阵（单值矩阵）为：

\[
\Delta(h)=\Phi(2h,0)+\int_h^{2h}\Phi(2h,s)G_0(s)\Phi(s-h,0)\,ds.
\]

若 \(\Delta(h)\) 为幂零矩阵，即存在最小整数 \(\nu_0\ge1\) 使 \(\Delta^{\nu_0}(h)=0\)，则 \(x(t)=0,\ t\ge 2\nu_0h\)。特别地，若通过增益设计使 \(\Delta(h)=0\)，则系统可在一个完整周期后归零。

为将该思想用于误差系统，先取常值反馈 \(K_0\) 使 \(A_c=A+BK_0\) 成为期望的基准闭环矩阵，再引入周期延迟反馈：

\[
\nu(t)=K_0z(t)-K_c(t)z(t-h).
\]

此时误差闭环为：

\[
\dot z(t)=A_cz(t)-BK_c(t)z(t-h).
\]

令 \(R_h(t)\) 为 \(2h\) 周期光滑函数，满足：

\[
R_h(t)=0,\quad t\in[0,h],\qquad
R_h(t)\ge0,\quad t\in[h,2h],
\]

且在 \([h,2h]\) 的某个子区间上正定。根据文献构造，可定义：

\[
W_c(A_c,h)=\int_h^{2h}e^{-A_cs}BR_h(s)B^\top e^{-A_c^\top s}\,ds,
\]

并取：

\[
K_c(t)=R_h(t)B^\top e^{-A_c^\top t}W_c^{-1}(A_c,h)e^{A_c(h-t)}.
\]

由于 \((A_c,B)\) 与 \((A,B)\) 同可控，且 \(R_h(t)\) 在后半周期内正定，\(W_c(A_c,h)\) 非奇异。将上述增益代入单周期状态转移矩阵可使 \(\Delta(h)=0\)，从而得到固定时间零化性质。

### 4.4 关节力矩控制律

结合动力学补偿和 PDF 误差输入，PDF 关节力矩控制律可写为：

\[
\boxed{
\tau
=
M_e(q_m)
\left[
\ddot q_d
+
K_0z(t)
-
K_c(t)z(t-h)
\right]
+
C_e(q_m,\dot q_m)\dot q_m
-
\hat d(t).
}
\]

若观测器与 PDF 控制器同时从 \(t=0\) 启动，则 \(0\le t<T_o\) 内的观测误差会作为残余扰动进入误差系统，严格 \(2h\) 零化结论不再直接成立。为保持理论闭环清晰，可采用两阶段时序：先在 \(T_o\) 内完成扰动观测，随后以局部时间 \(\vartheta=t-T_o\) 启动 PDF 增益 \(K_c(\vartheta)\)。此时理想条件下总预设收敛时间为：

\[
T_{\mathrm{tot}}=T_o+2h .
\]

---

## 5. 稳定性与性能分析

### 命题1（扰动观测器收敛性）

若采用预设时间扰动观测器，且 \(\Delta_a\) 与 \(\dot\Delta_a\) 有界，选择 \(T_c\le T_o\)，则在观测器收敛后有 \(\hat d(t)=d(t),\ \forall t\ge T_o\)。

### 命题2（理想固定时间跟踪）

在命题1成立后，若 \(K_0\) 使 \(A_c=A+BK_0\)，且周期延迟增益 \(K_c(t)\) 按 Gramian 型 PDF 设计，并从 \(t=T_o\) 起以局部时间 \(\vartheta=t-T_o\) 启动 PDF 增益，则理想模型条件下关节跟踪误差满足：

\[
e(t)=0,\qquad \dot e(t)=0,\qquad \forall t\ge T_o+2h.
\]

**证明**：由命题1可知，当 \(t\ge T_o\) 时有 \(\hat d(t)=d(t)\)。动力学补偿给出 \(\ddot q_m=v\)。代入 \(v=\ddot q_d+\nu\) 后有 \(\ddot e=\nu\)，误差系统可精确写为 \(\dot z=Az+B\nu\)。再代入 PDF 误差输入，得到闭环误差系统 \(\dot z(t)=A_cz(t)-BK_c(t)z(t-h)\)。

对闭环系统，有 \(F(t)=A_c\)、\(G(t)=-BK_c(t)\)。由于 \(R_h(t)=0,\ t\in[0,h]\)，所以 \(K_c(t)=0,\ t\in[0,h]\)，闭环系统属于周期延迟系统形式。其单周期状态转移矩阵为：

\[
\Delta_K(h)=e^{2A_ch}
-\int_h^{2h}e^{A_c(2h-s)}BR_h(s)B^\top e^{-A_c^\top s}W_c^{-1}(A_c,h)e^{A_c(h-s)}e^{A_c(s-h)}\,ds.
\]

由 \(e^{A_c(h-s)}e^{A_c(s-h)}=I\) 和 \(W_c(A_c,h)\) 的定义可得 \(\Delta_K(h)=e^{2A_ch}-e^{2A_ch}W_c(A_c,h)W_c^{-1}(A_c,h)=0\)。

因此，按单周期状态转移矩阵零化条件，局部时间 \(\vartheta\ge2h\) 后有 \(z(t)=0\)。由于 \(\vartheta=t-T_o\)，可得 \(z(t)=0,\ \forall t\ge T_o+2h\)。

### 命题3（幂零情形）

若采用一般周期延迟增益，且相应单周期状态转移矩阵 \(\Delta(h)\) 为幂零矩阵，即 \(\Delta^{\nu_0}(h)=0\)，则理想补偿条件下有：

\[
e(t)=0,\qquad \dot e(t)=0,\qquad \forall t\ge T_o+2\nu_0h.
\]

### 实际系统考虑

当模型存在近似误差或扰动补偿不完全时，设残余扰动为 \(\Delta_d(t)=M_e^{-1}(q_m)[d(t)-\hat d(t)+\Delta_m(t)]\)，其中 \(\Delta_m(t)\) 表示名义模型与真实模型之间的等效误差，则误差系统变为：

\[
\dot z(t)=A_cz(t)-BK_c(t)z(t-h)+B\Delta_d(t).
\]

若 \(\Delta_d(t)\) 有界，则误差动态可视为固定时间稳定线性延迟系统受到有界输入扰动。此时严格归零一般不再成立，但闭环轨迹可在预设时间附近进入与扰动上界、模型误差和反馈增益有关的小邻域。

---

## 6. 仿真验证

### 6.1 仿真设置

- **机械臂自由度**：\(n=7\)，惯性参数依据严宇新论文表2-1中的空间机械臂参数整理。
- **仿真步长**：\(T_s=10^{-3}\,\mathrm{s}\)，总时长为 \(10\,\mathrm{s}\)。
- **参考轨迹**：初始点 \(q_r(0)=[0,\pi/3,0,\pi/4,\pi/4,0,\pi/6]^\top\)，终端点 \(q_f=[\pi/18,\pi/6,\pi/5,0,-\pi/4,-\pi/3,\pi/12]^\top\)。期望轨迹由五次多项式在 \(8\,\mathrm{s}\) 内生成。
- **初始误差**：\(q(0)=q_r(0)+\frac{\pi}{180}[4,-3,3,-4,3,-2,2]^\top\)，\(\dot q(0)=0\)。
- **PDF 人工延迟**：\(h=1\,\mathrm{s}\)。
- **激活函数**：
  \[
  R_h(t)=
  \begin{cases}
  0, & \theta\in[0,h),\\
  \sin^4\left(\pi(\theta-h)/h\right), & \theta\in[h,2h),
  \end{cases}
  \quad \theta=t\bmod 2h .
  \]
- **反馈基准增益**：
  \[
  \begin{aligned}
  K_p&=\operatorname{diag}(25,25,25,22,22,20,20),\\
  K_d&=\operatorname{diag}(10,10,10,9,9,8,8),
  \end{aligned}
  \]
  并令 \(K_0=[-K_p\ -K_d]\)。
- **扰动**：\(d_i(t)=0.15\sin(0.7t+0.3i)+0.05\cos(1.3t+0.2i),\quad i=1,\ldots,7\)。
- **预设时间扰动观测器参数**：\(T_o=T_c=1\,\mathrm{s}\)、\(\eta=0.3\)、\(\sigma=0.4\)，采用观测--PDF 两阶段时序。

### 6.2 仿真指标

| 控制器与工况 | \(\|e(10)\|\) (rad) | \(\|\dot e(10)\|\) (rad/s) | \(\mathrm{RMS}(e)\) (rad) | \(\max|\tau_i|\) (N·m) |
|:---|:---|:---|:---|:---|
| PD，无扰动 | \(4.028\times10^{-9}\) | \(1.802\times10^{-8}\) | \(8.622\times10^{-3}\) | 9.129 |
| PDF，无扰动 | \(3.290\times10^{-10}\) | \(1.145\times10^{-9}\) | \(8.622\times10^{-3}\) | 9.129 |
| PD，有扰动 | \(1.221\times10^{-2}\) | \(1.197\times10^{-2}\) | \(9.455\times10^{-3}\) | 9.129 |
| PDF，有扰动 | \(1.235\times10^{-2}\) | \(1.177\times10^{-2}\) | \(9.456\times10^{-3}\) | 9.129 |
| PDF+PTDO，有扰动 | \(3.139\times10^{-5}\) | \(4.226\times10^{-4}\) | \(8.623\times10^{-3}\) | 9.129 |

### 6.3 结果分析

- 无扰动时，PD 与 PDF 均可实现高精度跟踪，PDF 在终端误差上略低于 PD。
- 在有界扰动下，未启用扰动观测器的 PD 和 PDF 控制均只能实现小邻域跟踪。
- 加入 PTDO 后，PDF+PTDO 算例的终端位置误差降至 \(3.139\times10^{-5}\,\mathrm{rad}\)，终端速度误差降至 \(4.226\times10^{-4}\,\mathrm{rad/s}\)，且最大控制力矩未出现明显增加。
- 观测器终端观测误差范数为 \(8.489\times10^{-2}\)，均方根观测误差为 \(3.325\times10^{-2}\)。

仿真结果验证了所提控制结构的可运行性：在名义模型下，基于 Gramian 构造的光滑周期延迟反馈能够实现高精度跟踪；在有界扰动条件下，预设时间扰动观测补偿能够显著降低终端跟踪误差。

---

## 7. 结论

本文针对捕获前自由漂浮空间机械臂轨迹跟踪问题，提出了基于光滑周期延迟反馈的关节空间控制方法。该方法先由动量守恒关系得到等效关节空间动力学，并针对有界集总扰动设计预设时间扰动观测器来生成 \(\hat d(t)\)；在获得扰动估计后，再构造面向误差状态的动力学补偿接口，将 PDF 反馈设计嵌入可控线性误差通道，最后利用当前误差和延迟误差构造周期时变反馈律。

理论分析表明，在理想建模、精确补偿和连续时间实现条件下，闭环误差动态可继承线性 PDF 控制的固定时间镇定性质；在存在观测误差、模型近似和数值离散时，应将结论理解为实际收敛或小邻域跟踪。

后续工作：
1. 将名义关节空间模型替换为完整的自由漂浮空间机械臂递推动力学和广义雅可比模型；
2. 在末端任务空间轨迹、关节限位和广义雅可比奇异规避约束下验证控制器性能；
3. 进一步考虑执行器饱和、测量噪声、模型不确定性和持续扰动，建立更严格的鲁棒固定时间或实际固定时间稳定性证明。

---

## 8. 参考文献

1. 严宇新. 面向在轨捕获的空间机械臂运动规划与控制方法研究 [D]. 哈尔滨工业大学, 2025.
2. Umetani Y, Yoshida K. Resolved Motion Rate Control of Space Manipulators with Generalized Jacobian Matrix [J]. IEEE Transactions on Robotics and Automation, 1989, 5(3): 303--314.
3. Vafa Z, Dubowsky S. The Kinematics and Dynamics of Space Manipulators: The Virtual Manipulator Approach [J]. The International Journal of Robotics Research, 1990, 9(4): 3--21.
4. Papadopoulos E, Dubowsky S. Dynamic Singularities in Free-Floating Space Manipulators [J]. Journal of Dynamic Systems, Measurement, and Control, 1993, 115(1): 44--52.
5. Wang M, Luo J, Walter U. Trajectory Planning of Free-Floating Space Robot Using Particle Swarm Optimization (PSO) [J]. Acta Astronautica, 2015.
6. 宁昕, 武耀发. 自由漂浮空间机器人轨迹跟踪的模型预测控制 [J]. 控制理论与应用, 2019, 36(5): 687--696.
7. Yu X, Guo J, Zhang J. Time Delay Estimation-Based Reactionless Augmented Adaptive Sliding Mode Control of a Space Manipulator's Pregrasping a Target [J]. Robotica, 2022.
8. Li J, Cao X, Liu M. Fixed-Time Fault-Tolerant Control for Free-Floating Space Manipulator with Prescribed Performance Constraints and Actuator Faults [J]. SCIENTIA SINICA Technologica, 2025.
9. Jiang Y, Lv J, Wang C, Kao Y, Wang F. Prescribed-Time Disturbance Observer-Based Fully Distributed Prescribed-Time Containment Control of Multiagent Systems [J]. IEEE Transactions on Circuits and Systems II: Express Briefs, 2024.
10. Zhou Z, Michiels R, Chen Q. Fixed-time stabilization of linear delay systems by smooth periodic delayed feedback [J]. (待引用).

---

## 9. 仿真代码概览

### 9.1 主仿真脚本 (`simulation/main_ffsm_pdf_tracking_yan_params.m`)

- 基于严宇新论文表2-1中的物理参数构建正定名义关节空间模型。
- 实现完整的 PDF 控制律：动力学补偿 + 周期延迟反馈 + 预设时间扰动观测器。
- 支持多种算例对比：PD、PDF、PDF+PTDO，有/无扰动。
- 输出仿真结果到 `latest_results.mat`，供绘图脚本使用。

### 9.2 绘图脚本 (`simulation/plot_ffsm_pdf_tracking_results.m`)

- 加载 `latest_results.mat` 中的仿真结果。
- 生成论文用图：基座平动、末端轨迹、关节姿态、跟踪误差、控制力矩、扰动观测对比等。

### 9.3 辅助脚本 (`simulation/main.m`)

- 独立的辅助分析脚本，用于计算特定参数下的性能指标（如 \(\Theta\) 函数、最大比值场等）。

---

## 附录：文件结构

```
严宇新/
├── main.md                          # 本文档（核心内容整理）
├── paper.tex                        # LaTeX 源文件
├── README.md                        # 原始记录标记
├── .gitignore                       # Git 忽略规则
├── refs/
│   ├── local.bib                    # 参考文献 BibTeX 条目
│   ├── 10.1109_TAC.2021.3051262.md # 占位文件（非真实内容）
│   └── 面向在轨捕获的空间机械臂运动规划与控制方法研究_严宇新.md  # 占位文件（非真实内容）
├── sections/
│   ├── 01-introduction.tex          # 引言
│   ├── 02-modeling.tex              # 建模
│   ├── 03-problem.tex               # 问题描述
│   ├── 04-controller.tex            # 控制算法
│   ├── 05-stability.tex             # 稳定性分析
│   ├── 06-simulation.tex            # 仿真（主）
│   ├── 06-simulation2.tex           # 仿真（补充表格）
│   └── 07-conclusion.tex            # 结论
└── simulation/
    ├── main.m                       # 辅助分析脚本
    ├── main_ffsm_pdf_tracking_yan_params.m  # 主仿真脚本
    └── plot_ffsm_pdf_tracking_results.m     # 绘图脚本
```

---

> **整理说明**：本文档基于严宇新论文及相关研究资料的完整内容整理而成，保留了原始 LaTeX 公式和学术表述，可作为论文写作、学术交流或进一步研究的参考文档。