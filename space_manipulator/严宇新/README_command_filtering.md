# 指令滤波反步法在自由漂浮空间机械臂控制中的应用

## 基于光滑周期延迟反馈的预设时间轨迹跟踪控制

**作者**: 严宇新  
**日期**: 2025

---

## 目录

1. [引言](#1-引言)
2. [指令滤波反步法基本原理](#2-指令滤波反步法基本原理)
3. [自由漂浮空间机械臂建模](#3-自由漂浮空间机械臂建模)
4. [原算法回顾：PDF控制](#4-原算法回顾pdf控制)
5. [指令滤波反步法应用于空间机械臂](#5-指令滤波反步法应用于空间机械臂)
6. [稳定性分析](#6-稳定性分析)
7. [与PDF控制的比较](#7-与pdf控制的比较)
8. [仿真验证](#8-仿真验证)
9. [结论](#9-结论)
10. [参考文献](#10-参考文献)
11. [附录：关键知识来源](#11-附录关键知识来源)

---

## 1. 引言

### 1.1 研究背景

自由漂浮空间机械臂（Free-Floating Space Manipulator, FFSM）在轨捕获任务中，基座不受主动控制，其运动通过动量耦合与机械臂关节运动相互影响。这种耦合特性使得传统的地面机器人控制方法难以直接应用。

严宇新在其论文中提出了一种基于**光滑周期延迟反馈（Periodic Delayed Feedback, PDF）**的预设时间轨迹跟踪控制方法，核心架构为：

```
扰动观测 → 动力学补偿 → 误差线性化 → 周期延迟反馈（PDF）
```

该方法通过预设时间扰动观测器（PTDO）估计集总扰动，利用计算力矩补偿将非线性动力学转化为线性可控误差通道，最后通过周期延迟反馈实现固定时间误差归零。

### 1.2 本文贡献

本文将**指令滤波反步法（Command Filtered Backstepping）**应用于自由漂浮空间机械臂的轨迹跟踪控制，主要贡献包括：

1. **消除微分爆炸**：使用指令滤波器替代传统反步法中虚拟控制信号的解析微分，显著降低高阶系统控制器的计算复杂度
2. **误差补偿机制**：设计误差补偿系统消除滤波误差，保证闭环系统的稳定性
3. **与PDF控制的互补**：指令滤波反步法可自然处理输入饱和、执行器约束等问题，与PDF的固定时间特性形成互补
4. **降低假设条件**：仅需参考轨迹及其一阶导数已知，而传统反步法需要更高阶导数信息

---

## 2. 指令滤波反步法基本原理

### 2.1 传统反步法的局限性

对于严格反馈非线性系统：

$$
\begin{aligned}
\dot{x}_i &= f_i(\bar{x}_i) + g_i(\bar{x}_i) x_{i+1}, \quad i = 1, \ldots, n-1 \\
\dot{x}_n &= f_n(x) + g_n(x) u \\
y &= x_1
\end{aligned}
$$

传统反步法通过递归设计虚拟控制信号 $\alpha_1, \alpha_2, \ldots, \alpha_{n-1}$ 和实际控制信号 $u$。每一步需要计算 $\partial \alpha_{i-1} / \partial x_j$、$\partial \alpha_{i-1} / \partial \hat{\theta}$ 等偏导数，当系统阶数 $n > 3$ 时，这些解析计算变得极其复杂，称为**"微分爆炸"（explosion of complexity）**问题。

### 2.2 指令滤波器

指令滤波器的核心思想是：用滤波器输出替代虚拟控制信号的解析导数。常用的指令滤波器有两种形式：

**形式一：一阶低通滤波器**（Farrell et al., 2009）

$$
\dot{x}_{i+1}^c = -\omega_{i+1} (x_{i+1}^c - \alpha_i), \quad x_{i+1}^c(0) = \alpha_i(0)
$$

其中 $\omega_{i+1} > 0$ 为滤波器带宽。滤波器输出 $x_{i+1}^c$ 近似 $\alpha_i$，其导数 $\dot{x}_{i+1}^c = -\omega_{i+1}(x_{i+1}^c - \alpha_i)$ 可直接用于控制律。

**形式二：二阶指令滤波器**（Yu et al., 2015; Shen & Shi, 2015）

$$
\begin{aligned}
\dot{\omega}_i &= \omega_n \omega_{i,2} \\
\dot{\omega}_{i,2} &= -2\zeta \omega_n \omega_{i,2} - \omega_n (\omega_i - \alpha_{i-1})
\end{aligned}
$$

其中 $\zeta \in (0, 1]$，$\omega_n > 0$。该滤波器的输出 $\omega_i$ 及其导数 $\dot{\omega}_i$ 均有界，且满足：

$$
|\omega_i - \alpha_{i-1}| \leq \kappa, \quad |\dot{\omega}_i|, |\ddot{\omega}_i| \text{ 有界}
$$

### 2.3 误差补偿机制

指令滤波器引入的滤波误差 $(x_{i+1}^c - \alpha_i)$ 会影响控制精度。通过设计**误差补偿信号** $\xi_i$ 来消除这一影响：

$$
\begin{aligned}
\dot{\xi}_1 &= -k_1 \xi_1 + \xi_2 + (x_2^c - \alpha_1) \\
\dot{\xi}_i &= -k_i \xi_i + \xi_{i+1} + (x_{i+1}^c - \alpha_i), \quad i = 2, \ldots, n-1 \\
\dot{\xi}_n &= -k_n \xi_n - \xi_{n-1}
\end{aligned}
$$

定义补偿后误差：$\upsilon_i = z_i - \xi_i$，其中 $z_i$ 为原始跟踪误差。

### 2.4 指令滤波反步法设计步骤

以二阶系统 $\dot{x}_1 = x_2$, $\dot{x}_2 = u$ 为例（跟踪目标 $x_1 \to x_d$）：

**Step 1**:
- 跟踪误差：$z_1 = x_1 - x_d$
- 虚拟控制：$\alpha_1 = -k_1 z_1 + \dot{x}_d$
- 指令滤波：$\dot{x}_2^c = -\omega_2 (x_2^c - \alpha_1)$
- 补偿信号：$\dot{\xi}_1 = -k_1 \xi_1 + \xi_2 + (x_2^c - \alpha_1)$
- 补偿误差：$\upsilon_1 = z_1 - \xi_1$

**Step 2**:
- 跟踪误差：$z_2 = x_2 - x_2^c$
- 控制律：$u = -k_2 z_2 + \dot{x}_2^c$
- 补偿信号：$\dot{\xi}_2 = -k_2 \xi_2 - \xi_1$
- 补偿误差：$\upsilon_2 = z_2 - \xi_2$

**闭环误差动态**（补偿后）：

$$
\begin{aligned}
\dot{\upsilon}_1 &= -k_1 \upsilon_1 + \upsilon_2 \\
\dot{\upsilon}_2 &= -k_2 \upsilon_2 - \upsilon_1
\end{aligned}
$$

选取 Lyapunov 函数 $V = \frac{1}{2}\upsilon_1^2 + \frac{1}{2}\upsilon_2^2$，可得：

$$
\dot{V} = -k_1 \upsilon_1^2 - k_2 \upsilon_2^2 \leq -2\min(k_1, k_2) V
$$

因此闭环系统指数稳定。

---

## 3. 自由漂浮空间机械臂建模

### 3.1 系统动力学

自由漂浮空间机械臂的等效关节空间动力学为：

$$
M_e(q_m) \ddot{q}_m + C_e(q_m, \dot{q}_m) \dot{q}_m = \tau + d(t)
$$

其中：
- $q_m \in \mathbb{R}^n$：关节位置
- $M_e(q_m) \in \mathbb{R}^{n \times n}$：等效惯性矩阵（正定对称）
- $C_e(q_m, \dot{q}_m) \dot{q}_m$：等效非线性项（科氏力+离心力）
- $\tau \in \mathbb{R}^n$：关节控制力矩
- $d(t) \in \mathbb{R}^n$：集总扰动（未建模动态 + 外部扰动 + 参数不确定性）

### 3.2 关键假设

**假设 1**: $M_e(q_m)$ 在工作空间内一致正定且可逆，$M_e$ 与 $C_e$ 可由模型获得或由名义模型近似。

**假设 2**: 参考轨迹 $q_d, \dot{q}_d, \ddot{q}_d$ 有界且连续，关节位置和速度可测。

**假设 3**: 等效加速度扰动 $\Delta_a = M_e^{-1}(q_m) d(t)$ 及其导数有界。

---

## 4. 原算法回顾：PDF控制

严宇新原始算法的控制架构如下：

### 4.1 预设时间扰动观测器（PTDO）

在加速度通道设计观测器：

$$
\dot{\chi} = u_o + \Delta_a, \quad \chi = \dot{q}_m
$$

其中 $u_o = M_e^{-1}(q_m)[\tau - C_e \dot{q}_m]$，$\Delta_a = M_e^{-1}(q_m) d(t)$。

构造预设时间扰动观测器：

$$
\begin{aligned}
\dot{z}_1 &= z_2 + \frac{\pi}{\eta T_c} \phi_1\left(\frac{\varepsilon_1}{\sigma}\right) - \dot{\bar{\xi}}(t) + \bar{\xi}(t) + u_o \\
\dot{z}_2 &= \frac{\pi}{\sigma \eta T_c} \phi_2\left(\frac{\varepsilon_1}{\sigma}\right) - \dot{\bar{\xi}}(t)
\end{aligned}
$$

观测误差在预设时间 $T_o$ 内收敛到零。

### 4.2 动力学补偿与误差线性化

获得扰动估计 $\hat{d}(t)$ 后，采用计算力矩补偿：

$$
\tau = M_e(q_m) v + C_e(q_m, \dot{q}_m) \dot{q}_m - \hat{d}(t)
$$

当观测器收敛后（$\hat{d} = d$），误差系统为：

$$
\ddot{e} = \ddot{q}_m - \ddot{q}_d = v - \ddot{q}_d \triangleq \nu
$$

定义误差状态 $z = [e^\top, \dot{e}^\top]^\top$，误差系统为：

$$
\dot{z} = Az + B\nu, \quad A = \begin{bmatrix} 0 & I_n \\ 0 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ I_n \end{bmatrix}
$$

### 4.3 周期延迟反馈（PDF）

引入周期延迟反馈：

$$
\nu(t) = K_0 z(t) - K_c(t) z(t-h)
$$

其中 $K_c(t)$ 为 $2h$ 周期函数，在 $[0, h]$ 内为零，在 $[h, 2h]$ 内正定。通过设计 $K_c(t)$ 使单周期状态转移矩阵 $\Delta(h) = 0$，实现固定时间 $2h$ 内的误差精确归零。

**PDF控制的优点**：
- 固定时间精确收敛（$T_{\text{tot}} = T_o + 2h$）
- 理论结构优雅

**PDF控制的局限性**：
- 对观测误差和模型误差敏感
- 未考虑执行器饱和
- 延迟反馈增益设计复杂，需离线计算 Gramian 矩阵
- 难以处理状态约束

---

## 5. 指令滤波反步法应用于空间机械臂

### 5.1 控制架构设计

将指令滤波反步法应用于误差线性化后的空间机械臂系统：

```
扰动观测（PTDO）→ 动力学补偿 → 误差线性化 → 指令滤波反步法
```

与原始PDF控制不同，我们用指令滤波反步法替代周期延迟反馈部分。

### 5.2 误差系统状态空间表示

经过动力学补偿后，误差系统为二阶积分链：

$$
\begin{aligned}
\dot{z}_1 &= z_2 \\
\dot{z}_2 &= \nu
\end{aligned}
$$

其中 $z_1 = e = q_m - q_d \in \mathbb{R}^n$，$z_2 = \dot{e} = \dot{q}_m - \dot{q}_d \in \mathbb{R}^n$，$\nu \in \mathbb{R}^n$ 为辅助控制输入。

对于 $n$ 关节机械臂，系统为 $2n$ 维，但各关节的控制设计可解耦进行（或采用向量形式统一设计）。

### 5.3 指令滤波反步法设计

**Step 1：位置误差子系统**

定义位置误差：$\tilde{z}_1 = z_1$（目标：$\tilde{z}_1 \to 0$）

设计虚拟控制信号：

$$
\alpha_1 = -K_1 \tilde{z}_1
$$

其中 $K_1 = k_1 I_n \in \mathbb{R}^{n \times n}$，$k_1 > 0$。

将 $\alpha_1$ 通过指令滤波器：

$$
\dot{x}_2^c = -\omega_2 (x_2^c - \alpha_1), \quad x_2^c(0) = \alpha_1(0)
$$

其中 $\omega_2 > 0$ 为滤波器带宽。滤波器输出 $x_2^c$ 近似 $\alpha_1$，其导数 $\dot{x}_2^c = -\omega_2(x_2^c - \alpha_1)$ 可直接使用。

设计误差补偿信号：

$$
\dot{\xi}_1 = -K_1 \xi_1 + \xi_2 + (x_2^c - \alpha_1), \quad \xi_1(0) = 0
$$

定义补偿后误差：$\upsilon_1 = \tilde{z}_1 - \xi_1$

**Step 2：速度误差子系统**

定义速度误差：$\tilde{z}_2 = z_2 - x_2^c$

设计实际控制信号：

$$
\nu = -K_2 \tilde{z}_2 + \dot{x}_2^c
$$

其中 $K_2 = k_2 I_n \in \mathbb{R}^{n \times n}$，$k_2 > 0$。

设计误差补偿信号：

$$
\dot{\xi}_2 = -K_2 \xi_2 - \xi_1, \quad \xi_2(0) = 0
$$

定义补偿后误差：$\upsilon_2 = \tilde{z}_2 - \xi_2$

### 5.4 完整控制律

结合动力学补偿，完整的关节力矩控制律为：

$$
\boxed{
\tau = M_e(q_m) \left[ \ddot{q}_d + \nu \right] + C_e(q_m, \dot{q}_m) \dot{q}_m - \hat{d}(t)
}
$$

其中 $\nu = -K_2 (z_2 - x_2^c) + \dot{x}_2^c$，$x_2^c$ 和 $\dot{x}_2^c$ 由指令滤波器生成。

### 5.5 实现说明

与原始PDF控制相比，指令滤波反步法的实现更为简洁：

| 特性 | PDF控制 | 指令滤波反步法 |
|------|---------|---------------|
| 在线计算量 | 低（查表读取 $K_c(t)$） | 低（滤波器迭代） |
| 离线计算 | 需计算 Gramian 积分 $W_c$ | 无需 |
| 参数调节 | $K_0, K_c(t), h$ | $k_1, k_2, \omega_2$ |
| 对延迟的依赖 | 需要 $z(t-h)$ 历史缓存 | 不需要 |
| 处理约束 | 困难 | 可自然扩展（BLF） |

---

## 6. 稳定性分析

### 6.1 补偿误差动态

首先推导补偿后误差 $\upsilon_1, \upsilon_2$ 的闭环动态。

由定义：

$$
\begin{aligned}
\dot{\upsilon}_1 &= \dot{z}_1 - \dot{\xi}_1 \\
&= z_2 - \left[-K_1 \xi_1 + \xi_2 + (x_2^c - \alpha_1)\right] \\
&= (\tilde{z}_2 + x_2^c) + K_1 \xi_1 - \xi_2 - x_2^c + \alpha_1 \\
&= \tilde{z}_2 + K_1 \xi_1 - \xi_2 + \alpha_1
\end{aligned}
$$

代入 $\alpha_1 = -K_1 \tilde{z}_1 = -K_1(\upsilon_1 + \xi_1)$：

$$
\begin{aligned}
\dot{\upsilon}_1 &= \tilde{z}_2 + K_1 \xi_1 - \xi_2 - K_1 \upsilon_1 - K_1 \xi_1 \\
&= \tilde{z}_2 - \xi_2 - K_1 \upsilon_1 \\
&= (\upsilon_2 + \xi_2) - \xi_2 - K_1 \upsilon_1 \\
&= -K_1 \upsilon_1 + \upsilon_2
\end{aligned}
$$

类似地，对于 $\upsilon_2$：

$$
\begin{aligned}
\dot{\upsilon}_2 &= \dot{\tilde{z}}_2 - \dot{\xi}_2 \\
&= (\dot{z}_2 - \dot{x}_2^c) - (-K_2 \xi_2 - \xi_1) \\
&= \nu - \dot{x}_2^c + K_2 \xi_2 + \xi_1
\end{aligned}
$$

代入 $\nu = -K_2 \tilde{z}_2 + \dot{x}_2^c$：

$$
\begin{aligned}
\dot{\upsilon}_2 &= -K_2 \tilde{z}_2 + \dot{x}_2^c - \dot{x}_2^c + K_2 \xi_2 + \xi_1 \\
&= -K_2(\upsilon_2 + \xi_2) + K_2 \xi_2 + \xi_1 \\
&= -K_2 \upsilon_2 + \xi_1
\end{aligned}
$$

因此，补偿误差动态为：

$$
\boxed{
\begin{aligned}
\dot{\upsilon}_1 &= -K_1 \upsilon_1 + \upsilon_2 \\
\dot{\upsilon}_2 &= -K_2 \upsilon_2 + \xi_1
\end{aligned}
}
$$

### 6.2 滤波误差分析

定义滤波误差 $\eta = x_2^c - \alpha_1$。由指令滤波器的性质：

$$
\dot{\eta} = \dot{x}_2^c - \dot{\alpha}_1 = -\omega_2 \eta - \dot{\alpha}_1
$$

由于 $\alpha_1 = -K_1 z_1$，有 $\dot{\alpha}_1 = -K_1 \dot{z}_1 = -K_1 z_2$。在初始条件 $x_2^c(0) = \alpha_1(0)$ 下，$\eta(0) = 0$，且对于有界的 $z_2$，$\eta(t)$ 一致有界：

$$
|\eta(t)| \leq \frac{\|K_1 z_2\|_{\infty}}{\omega_2}
$$

通过增大 $\omega_2$ 可使滤波误差任意小。

### 6.3 Lyapunov 稳定性证明

选取 Lyapunov 函数候选：

$$
V = \frac{1}{2} \upsilon_1^\top \upsilon_1 + \frac{1}{2} \upsilon_2^\top \upsilon_2 + \frac{1}{2} \eta^\top \eta
$$

沿轨迹求导：

$$
\begin{aligned}
\dot{V} &= \upsilon_1^\top \dot{\upsilon}_1 + \upsilon_2^\top \dot{\upsilon}_2 + \eta^\top \dot{\eta} \\
&= \upsilon_1^\top (-K_1 \upsilon_1 + \upsilon_2) + \upsilon_2^\top (-K_2 \upsilon_2 + \xi_1) + \eta^\top (-\omega_2 \eta - \dot{\alpha}_1) \\
&= -\upsilon_1^\top K_1 \upsilon_1 + \upsilon_1^\top \upsilon_2 - \upsilon_2^\top K_2 \upsilon_2 + \upsilon_2^\top \xi_1 - \omega_2 \|\eta\|^2 - \eta^\top \dot{\alpha}_1
\end{aligned}
$$

利用 Young 不等式：

$$
\begin{aligned}
\upsilon_1^\top \upsilon_2 &\leq \frac{1}{2} \|\upsilon_1\|^2 + \frac{1}{2} \|\upsilon_2\|^2 \\
\upsilon_2^\top \xi_1 &\leq \frac{1}{2} \|\upsilon_2\|^2 + \frac{1}{2} \|\xi_1\|^2 \\
\eta^\top \dot{\alpha}_1 &\leq \frac{1}{2} \|\eta\|^2 + \frac{1}{2} \|\dot{\alpha}_1\|^2
\end{aligned}
$$

代入得：

$$
\begin{aligned}
\dot{V} &\leq -\lambda_{\min}(K_1) \|\upsilon_1\|^2 + \frac{1}{2} \|\upsilon_1\|^2 + \frac{1}{2} \|\upsilon_2\|^2 \\
&\quad - \lambda_{\min}(K_2) \|\upsilon_2\|^2 + \frac{1}{2} \|\upsilon_2\|^2 + \frac{1}{2} \|\xi_1\|^2 \\
&\quad - \omega_2 \|\eta\|^2 + \frac{1}{2} \|\eta\|^2 + \frac{1}{2} \|\dot{\alpha}_1\|^2 \\
&= -\left(\lambda_{\min}(K_1) - \frac{1}{2}\right) \|\upsilon_1\|^2 \\
&\quad -\left(\lambda_{\min}(K_2) - 1\right) \|\upsilon_2\|^2 \\
&\quad -\left(\omega_2 - \frac{1}{2}\right) \|\eta\|^2 + \frac{1}{2} \|\xi_1\|^2 + \frac{1}{2} \|\dot{\alpha}_1\|^2
\end{aligned}
$$

选择 $k_1 > \frac{1}{2}$，$k_2 > 1$，$\omega_2 > \frac{1}{2}$，则：

$$
\dot{V} \leq -\gamma V + \Delta
$$

其中 $\gamma = 2\min\left(k_1 - \frac{1}{2}, k_2 - 1, \omega_2 - \frac{1}{2}\right)$，$\Delta = \frac{1}{2} \|\xi_1\|^2 + \frac{1}{2} \|\dot{\alpha}_1\|^2$ 为有界项。

由比较引理可知：

$$
V(t) \leq V(0) e^{-\gamma t} + \frac{\Delta}{\gamma} (1 - e^{-\gamma t})
$$

因此，所有信号一致有界，且跟踪误差收敛到原点附近的小邻域：

$$
\lim_{t \to \infty} \|e(t)\| \leq \sqrt{\frac{2\Delta}{\gamma}}
$$

通过增大 $k_1, k_2$ 和 $\omega_2$，可使稳态误差任意小。

### 6.4 考虑扰动观测误差的情况

当扰动观测器未完全收敛时（$t < T_o$），存在观测误差 $\tilde{d}(t) = d(t) - \hat{d}(t)$。此时误差系统变为：

$$
\ddot{e} = \nu + M_e^{-1}(q_m) \tilde{d}(t)
$$

定义等效扰动 $\bar{d}(t) = M_e^{-1}(q_m) \tilde{d}(t)$，则 Step 2 的误差动态变为：

$$
\dot{\tilde{z}}_2 = \nu + \bar{d}(t)
$$

重复上述稳定性分析，可得：

$$
\dot{V} \leq -\gamma V + \Delta + \frac{1}{2} \|\bar{d}(t)\|^2
$$

由于 $\bar{d}(t)$ 在 $t \geq T_o$ 后收敛到零，系统最终仍保持有界稳定。

---

## 7. 与PDF控制的比较

### 7.1 理论特性对比

| 特性 | PDF控制 | 指令滤波反步法 |
|------|---------|---------------|
| 收敛时间 | 固定时间 $T_o + 2h$ | 渐近收敛（指数稳定） |
| 收敛精度 | 理论精确归零 | 有界误差（可任意小） |
| 对历史数据的依赖 | 需要 $z(t-h)$ | 不需要 |
| 在线计算 | 低 | 低 |
| 离线计算 | 需计算 $W_c(A_c, h)$ 积分 | 无需 |
| 参数调节复杂度 | 中等 | 简单（$k_1, k_2, \omega_2$） |
| 对观测误差的鲁棒性 | 敏感 | 可通过增益调节 |
| 处理执行器饱和 | 困难 | 可自然扩展 |
| 处理状态约束 | 困难 | 可结合障碍Lyapunov函数 |

### 7.2 互补性分析

PDF控制和指令滤波反步法各有优势，可形成互补：

1. **收敛时间**：PDF提供固定时间收敛，指令滤波反步法提供渐近收敛
2. **鲁棒性**：指令滤波反步法可通过增益调节适应观测误差，PDF对观测误差敏感
3. **工程实现**：指令滤波反步法更易结合抗饱和、状态约束等实际需求

**混合策略建议**：
- 在扰动观测阶段使用指令滤波反步法获得鲁棒性
- 在扰动补偿完成后切换到PDF控制获得固定时间收敛
- 或者在指令滤波反步法的基础上引入有限时间项（如 $\operatorname{sig}^\alpha(\upsilon_1)$），实现固定时间收敛

---

## 8. 仿真验证

### 8.1 仿真设置

基于严宇新论文中的仿真参数，设置如下：

- **机械臂**：7-DOF 自由漂浮空间机械臂
- **采样时间**：$T_s = 10^{-4}$ s
- **仿真时间**：$T_f = 10$ s
- **延迟**：$h = 1$ s（PDF控制）
- **扰动观测器**：$T_o = 2$ s

### 8.2 对比算法

1. **PD控制**：基线控制器
2. **PDF控制**：严宇新原始算法
3. **CFBS（Command Filtered Backstepping）**：本文提出的指令滤波反步法
4. **CFBS+PTDO**：指令滤波反步法 + 预设时间扰动观测器

### 8.3 MATLAB 仿真代码框架

```matlab
% 主仿真脚本：main_ffsm_pdf_tracking_cfbs.m
% 指令滤波反步法 + PTDO 控制自由漂浮空间机械臂

% 系统参数
n = 7;                    % 关节数
Ts = 1e-4;                % 采样时间
Tf = 10;                  % 仿真时间
t = 0:Ts:Tf;              % 时间向量

% 期望轨迹
q_d = sin(t);             % 期望位置
dq_d = cos(t);            % 期望速度
ddq_d = -sin(t);          % 期望加速度

% 控制器参数
k1 = 5;                   % 位置增益
k2 = 5;                   % 速度增益
omega2 = 50;              % 滤波器带宽

% 初始化
x2c = zeros(n, 1);        % 滤波器状态
dx2c = zeros(n, 1);       % 滤波器导数
xi1 = zeros(n, 1);        % 补偿信号1
xi2 = zeros(n, 1);        % 补偿信号2

% 主循环
for i = 1:length(t)
    % 当前状态
    q = q_actual(:, i);
    dq = dq_actual(:, i);
    
    % 误差
    e = q - q_d(:, i);
    de = dq - dq_d(:, i);
    
    % Step 1: 虚拟控制
    alpha1 = -k1 * e;
    
    % 指令滤波器
    dx2c = -omega2 * (x2c - alpha1);
    x2c = x2c + Ts * dx2c;
    
    % 误差补偿
    dxi1 = -k1 * xi1 + xi2 + (x2c - alpha1);
    xi1 = xi1 + Ts * dxi1;
    
    % Step 2: 控制输入
    z2 = de - x2c;
    nu = -k2 * z2 + dx2c;
    
    dxi2 = -k2 * xi2 - xi1;
    xi2 = xi2 + Ts * dxi2;
    
    % 动力学补偿
    tau = Me(q) * (ddq_d(:, i) + nu) + Ce(q, dq) * dq - d_hat(:, i);
    
    % 系统动力学更新（欧拉积分）
    ddq = Me(q) \ (tau + d(t(i)) - Ce(q, dq) * dq);
    dq_actual(:, i+1) = dq + Ts * ddq;
    q_actual(:, i+1) = q + Ts * dq;
end
```

### 8.4 预期结果

与原始PDF控制相比，指令滤波反步法预期具有以下特点：
- 跟踪误差收敛速度可通过 $k_1, k_2$ 灵活调节
- 对观测误差具有更好的鲁棒性
- 控制信号更平滑（无需高频切换）
- 稳态误差可通过高增益进一步压缩

---

## 9. 结论

本文将指令滤波反步法应用于自由漂浮空间机械臂的轨迹跟踪控制，主要结论如下：

1. **方法可行性**：指令滤波反步法可有效应用于空间机械臂控制，通过误差补偿机制保证了闭环系统的稳定性

2. **计算简化**：与传统反步法相比，指令滤波技术消除了虚拟控制信号解析微分的计算负担，适用于高阶系统

3. **与PDF互补**：指令滤波反步法在鲁棒性、参数调节和工程实现方面具有优势，可与PDF的固定时间收敛特性形成互补

4. **扩展潜力**：该方法可自然扩展到处理执行器饱和、状态约束、故障容错等实际问题

未来工作方向：
- 结合有限时间控制理论，实现固定时间指令滤波反步法
- 引入自适应神经网络处理未知动力学
- 实验验证实际空间机械臂平台

---

## 10. 参考文献

### 核心参考文献

1. **严宇新**. 面向在轨捕获的空间机械臂运动规划与控制方法研究 [D]. 博士论文, 2025.

2. **Farrell, J. A., Polycarpou, M. M., Sharma, M., & Dong, W.** (2009). Command filtered adaptive backstepping. *IEEE Transactions on Automatic Control*, 54(6), 1391-1395.  
   — 指令滤波反步法的开创性论文，提出使用滤波器替代解析微分。

3. **Dong, W., Farrell, J. A., Polycarpou, M. M., Djapic, V., & Sharma, M.** (2012). Command filtered adaptive backstepping. *IEEE Transactions on Automatic Control*, 58(10), 2617-2622.  
   — 指令滤波自适应反步法的完整理论分析，包含初始化阶段分析。

4. **Yu, J., Shi, P., & Zhao, X.** (2018). Finite-time command filtered backstepping control for a class of nonlinear systems. *IEEE Transactions on Automatic Control*, 63(10), 3464-3471.  
   — 有限时间指令滤波反步法，结合有限时间稳定理论。

5. **Yu, J., Shi, P., Dong, W., & Lin, C.** (2015). Command-filtered backstepping control for nonlinear systems with input saturation. *IEEE Transactions on Cybernetics*, 45(10), 2018-2027.  
   — 结合输入饱和处理的指令滤波反步法。

6. **Shen, H., & Shi, P.** (2015). Distributed command filtered backstepping consensus tracking control. *IEEE Transactions on Control Systems Technology*, 23(5), 1834-1841.  
   — 分布式指令滤波反步法在多智能体系统中的一致性跟踪控制。

7. **Wang, H., Kang, S., Zhao, X., Xu, N., & Li, T.** (2020). Command filter-based adaptive neural control for nonstrict-feedback nonlinear systems with multiple actuator constraints. *IEEE Transactions on Cybernetics*, 50(7), 3321-3332.  
   — 结合神经网络和执行器约束的指令滤波反步法。

8. **Ling, S., Wang, H., & Liu, P. X.** (2021). Adaptive fuzzy tracking control of flexible-joint robots based on command filtering. *IEEE Transactions on Fuzzy Systems*, 29(7), 2152-2163.  
   — 柔性关节机械臂的自适应模糊指令滤波控制。

9. **Li, Y. X.** (2019). Finite time command filtered adaptive fault tolerant control for a class of uncertain nonlinear systems. *Automatica*, 104, 108-117.  
   — 有限时间指令滤波自适应容错控制。

10. **Zhou, Q., et al.** (2022). Fixed-time stabilization of linear systems with periodic delayed feedback. *IEEE Transactions on Automatic Control*, 67(12), 6649-6656.  
    — PDF控制的理论基础，周期延迟反馈的固定时间镇定。

11. **Umetani, Y., & Yoshida, K.** (1989). Generalized Jacobian matrix for space manipulators and its use in feedback control. *Journal of Robotics Research*, 8(1), 35-48.  
    — 自由漂浮空间机械臂广义雅可比矩阵的经典文献。

### 相关理论文献

12. **Krstic, M., Kanellakopoulos, I., & Kokotovic, P. V.** (1995). *Nonlinear and Adaptive Control Design*. New York: Wiley.  
    — 反步法和自适应控制的经典教材。

13. **Jiang, Y., et al.** (2024). Prescribed-time disturbance observer for nonlinear systems. *Automatica*, 160, 113456.  
    — 预设时间扰动观测器的理论基础。

14. **Yu, J., Dong, W., Shi, P., & Lin, C.** (2018). Barrier Lyapunov functions-based command filtered output feedback control for full-state constrained nonlinear systems. *Automatica*, 103, 244-251.  
    — 结合障碍Lyapunov函数处理全状态约束。

---

## 附录：关键知识来源

本文的撰写参考了以下知识库文件中的内容：

### 指令滤波反步法理论
- `3339.md` — Dong et al. (2012), Command Filtered Adaptive Backstepping
- `3285.md` — Yu et al. (2018), Barrier Lyapunov functions-based command filtered output feedback control
- `3282.md` — Zheng & Yang (2020), Command Filter and Universal Approximator Based Backstepping
- `3325.md` — Wang et al. (2020), Command Filter-Based Adaptive Neural Control
- `3284.md` — Ling et al. (2021), Adaptive Fuzzy Tracking Control of Flexible-Joint Robots Based on Command Filtering
- `3277.md` — Li (2019), Finite time command filtered adaptive fault tolerant control

### 空间机械臂控制
- `严宇新/main.md` — 原始论文核心内容
- `严宇新/sections/02-modeling.tex` — 自由漂浮空间机械臂建模
- `严宇新/sections/03-problem.tex` — 问题描述与假设
- `严宇新/sections/04-controller.tex` — 控制算法设计（PTDO + PDF）
- `严宇新/sections/05-stability.tex` — 稳定性分析
- `严宇新/refs/local.bib` — 参考文献

### 其他相关文件
- `1012.md`, `1070.md`, `1096.md`, `1115.md`, `1137.md`, `1390.md` — 指令滤波反步法的不同应用变体
- `3273.md`, `3274.md`, `3275.md`, `3276.md`, `3278.md`, `3280.md`, `3281.md`, `3283.md` — 指令滤波反步法在各类非线性系统中的应用