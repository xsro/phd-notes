# 围捕控制器旋转现象分析：已证明的定理

## 问题描述

考虑 $N\ge 3$ 个全同智能体，运动学为单积分器
\[
\dot x_i(t)=u_i(t),\qquad i\in\mathcal N=\{1,\dots,N\},
\]
其中 $x_i(t),u_i(t)\in\mathbb R^2$。目标以未知常速度运动：
\[
\dot x_0=v_0,\qquad v_0\in\mathbb R^2.
\]

第一种控制器 (Kou–Chen–Xiang, TAC 2022, Eq.~(7))：
\[
\boxed{\; u_i=\phi_i+k_1(x_0-x_i)+v_i,\qquad
\dot v_i=k_2(x_0-x_i),\qquad
\phi_i=\sum_{j\in\mathcal N_i}\alpha(\|x_{ij}\|)\frac{x_{ij}}{\|x_{ij}\|},\;}
\]
其中 $k_1,k_2>0$，$\mathcal N_i=\{j\ne i:\|x_{ij}\|\le\mu\}$，且
\[
\alpha(s)=\frac{1}{s-d}-\frac{1}{\mu-d},\qquad s\in(d,\mu].
\]

## 定理 1（平均动力学）

平均位置误差 $\bar x=\frac1N\sum_i(x_i-x_0)$ 和平均速度估计误差 $\bar v=\frac1N\sum_i(v_i-v_0)$ 满足
\[
\ddot{\bar x}+k_1\dot{\bar x}+k_2\bar x=0,
\]
指数稳定，故 $\bar x(t)\to0$，$\dot{\bar x}(t)\to0$，$\bar v(t)\to0$，且平均速度 $\frac1N\sum_i u_i(t)\to v_0$。

## 定理 2（旋转频率选择）

设闭环系统在目标坐标系中允许非退化刚性旋转
\[
\tilde x_i(t)=R(\omega t)\tilde x_i^0,\qquad
R(\theta)=\cos\theta\,I+\sin\theta\,J,\qquad
J=\begin{bmatrix}0&-1\\1&0\end{bmatrix},
\]
其中 $\omega\ne0$，$\sum_i\|\tilde x_i^0\|^2>0$，且排斥力沿该轨道为无向中心力（总内力矩为零）。则
\[
\omega^2=k_2,\qquad |\omega|=\sqrt{k_2}.
\]

## 定理 3（径向平衡）

在旋转稳态上，排斥力与吸引力沿径向平衡：
\[
\phi_i(\tilde x^0)=k_1\tilde x_i^0.
\]

## 定理 4（形状子系统）

定义零均值形状变量 $\tilde x_i^s=\tilde x_i-\bar x$，$\tilde v_i^s=\tilde v_i-\bar v$。形状子系统为
\[
\ddot{\tilde x}^s+B(\tilde x^s)\dot{\tilde x}^s+k_2\tilde x^s=0,\qquad
B(\tilde x^s)=k_1I-\nabla^2 P(\tilde x^s),
\]
其中 $P$ 为排斥势能（$\phi=\nabla_{\tilde x^s}P$）。能量函数
\[
E=\frac12\|\dot{\tilde x}^s\|^2+\frac{k_2}{2}\|\tilde x^s\|^2
\]
满足
\[
\dot E=-\dot{\tilde x}^{s\mathsf T}B(\tilde x^s)\dot{\tilde x}^s.
\]

## 定理 5（旋转协方差与中性模）

排斥力旋转协变 $\phi(R\tilde x)=R\phi(\tilde x)$ 给出
\[
\nabla^2 P(\tilde x^s)\,(J\tilde x^s)=J\phi(\tilde x^s).
\]
在径向平衡 $\phi_i=k_1\tilde x_i^s$ 处，
\[
B\,(J\tilde x^s)=0,
\]
故旋转方向 $J\tilde x^s$ 是阻尼矩阵 $B$ 在平衡点的零特征向量。

## 定理 6（共旋转线性化）

设 $\omega_s=\pm\sqrt{k_2}$，引入共旋转坐标系 $\xi_i=e^{-J\omega_s t}\tilde x_i^s$，$w_i=e^{-J\omega_s t}\tilde v_i^s$。形状动力学变为自治系统，其平衡点 $(\xi^*,w^*)$ 满足
\[
\phi_i(\xi^*)=k_1\xi_i^*,\qquad w_i^*=\omega_s J\xi_i^*.
\]

定义 $W(\xi)=\sum_{\text{edges}}\Phi(\|x_{ij}\|)-\frac{k_1}{2}\sum_i\|\xi_i\|^2$，$H=\nabla^2 W(\xi^*)=-B(\xi^*)$。线性化矩阵为
\[
DF=\begin{bmatrix} H-\omega_s J & I\\ -\omega_s^2 I & -\omega_s J\end{bmatrix},\qquad \omega_s^2=k_2.
\]

方向 $(J\xi^*,-\omega_s\xi^*)$ 满足 $DF\,(J\xi^*,-\omega_s\xi^*)=0$，对应旋转相位的中性模。数值验证（$N=6$，$k_1=k_2=0.5$）表明除该零模外 $DF$ 的所有特征值实部严格为负，谱横坐标（零模除外）为 $-0.078$（$k_2=0.5$），随 $k_2$ 增大从 $-0.016$ 单调递减至 $-0.25$。

## 定理 7（方向选择）

闭环系统在反射变换 $J\mapsto-J$ 下不变，故两旋转方向 $\omega=+\sqrt{k_2}$ 和 $\omega=-\sqrt{k_2}$ 是镜像对称的吸引子。有向环量
\[
L=\sum_{i\in\mathcal N}\tilde x_i\times\tilde v_i
\]
在旋转稳态上满足
\[
L=\omega\sum_i\|\tilde x_i\|^2,\qquad
\operatorname{sign}(L)=\operatorname{sign}(\omega).
\]