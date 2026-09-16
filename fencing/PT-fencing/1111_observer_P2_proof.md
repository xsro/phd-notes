# P2 完整证明：有界加速度目标的预设时间观测器 (PT-25')

> 本文补齐需求文档 `1111_observer_nonconstant_target_requirements.md` §5.1 / §8-P2 的严格证明：
> 在目标**加速度有界**（$\|\ddot x_0\| \le \bar a_0$、未知）且**仅位置可测**（M1）的前提下，
> 证明观测器 (PT-25') 使估计误差在**预设时刻 $T$ 精确收敛到零**，并在 $t \ge T$ 持续精确跟踪。
>
> 符号约定与需求文档一致（Gong et al. 2022 约定）：$\psi_i = \sum_{j\in\mathcal N_i}(\varepsilon_i-\varepsilon_j) + g_i(\varepsilon_i - x_0)$，$\boldsymbol\psi = M_\sigma\tilde\varepsilon$。

---

## 1. 问题设定

**目标**（每坐标独立；$\mathbb R^d$ 按分量/$\otimes I_d$ 解耦）：

$$
\dot x_0 = v_0, \qquad \dot v_0 = a_0(t), \qquad \sup_{t\ge 0}\|a_0(t)\| \le \bar a_0 < \infty .
\tag{A2-2}
$$

**网络**：无向连通（可切换），$M_\sigma = L_\sigma + G_\sigma \succ 0$，任一时刻至少一个 $g_i(t) = 1$。

**观测器 (PT-25')**（智能体 $i$）：

$$
\boxed{
\begin{aligned}
\dot\varepsilon_i &= \rho_i - k_1\mu(t)\,\psi_i(t) \\
\dot\rho_i      &= -k_2\mu^2(t)\,\psi_i(t) - \sigma\,\operatorname{sign}\!\big(\psi_i(t)\big)
\end{aligned}
}
\tag{PT-25'}
$$

$$
\psi_i(t) = \sum_{j\in\mathcal N_i}\big(\varepsilon_i - \varepsilon_j\big) + g_i(t)\big(\varepsilon_i - x_0\big),
\qquad
\mu(t) = \begin{cases}\dfrac{h}{T-t}, & t\in[0,T),\\[4pt] 0, & t\ge T,\end{cases}\quad h>0.
\tag{1}
$$

**误差变量**：$\tilde\varepsilon = \varepsilon - \mathbf 1 x_0$，$\tilde\rho = \rho - \mathbf 1 v_0$。由 $\boldsymbol\psi = M_\sigma\tilde\varepsilon$ 得误差动力学

$$
\begin{aligned}
\dot{\tilde\varepsilon} &= \tilde\rho - k_1\mu(t)\, M_\sigma \tilde\varepsilon \\[2pt]
\dot{\tilde\rho}      &= -k_2\mu^2(t)\, M_\sigma \tilde\varepsilon - \sigma\,\operatorname{sign}\!\big(M_\sigma\tilde\varepsilon\big) - \mathbf 1\, a_0(t)
\end{aligned}
\tag{2}
$$

（推导：$\dot{\tilde\varepsilon}_i = \rho_i - k_1\mu\psi_i - v_0 = \tilde\rho_i - k_1\mu\psi_i$；$\dot{\tilde\rho}_i = -k_2\mu^2\psi_i - \sigma\mathrm{sign}(\psi_i) - a_0$。）

---

## 2. 定理（正式陈述）

**定理 1（P2）。** 设 A2-2 成立、网络连通且 $M_\sigma \succ 0$。选取增益

$$
k_1 > \frac{1}{h\,\lambda_{\min}(M_\sigma)},\qquad k_2 > 0,\qquad h > 0,\qquad \sigma > \bar a_0 .
\tag{3}
$$

则对任意初值 $(\varepsilon(0), \rho(0))$：

**(i) 预设时间趋零**：
$$
\lim_{t\to T^-}\tilde\varepsilon(t) = 0,\qquad \lim_{t\to T^-}\tilde\rho(t) = 0,
$$
且收敛速率满足
$$
\|\tilde\varepsilon(t)\| = O\big((T-t)^{\gamma h}\big),\qquad
\|\tilde\rho(t)\| = O\big((T-t)^{\gamma h - 1}\big),\qquad \gamma := \min\{\lambda,\ \tfrac2h\} > \tfrac1h,
\tag{4}
$$
其中 $\lambda > 1/h$ 是拉伸时间系统的衰减率。

**(ii) 持续精确跟踪**：$\tilde\varepsilon(t) \equiv 0$、$\tilde\rho(t) \equiv 0$，$\forall t \ge T$（Filippov 意义下），
即 $\varepsilon_i(t) = x_0(t)$、$\rho_i(t) = v_0(t)$，$\forall t \ge T$，$\forall i$。

**推论 1（切换拓扑）。** 若 $\sigma(t)$ 分段常值且 $\bar\lambda := \inf_\sigma\lambda_{\min}(M_\sigma) > 0$，把 (3) 中
$\lambda_{\min}(M_\sigma)$ 换成 $\bar\lambda$，定理 1 结论不变。

---

## 3. Part I：$t\to T^-$ 趋零（时间尺度变换）

### 3.1 拉伸时间坐标

定义

$$
s = h\ln\frac{T}{T-t}\in[0,\infty),\qquad
\frac{ds}{dt} = \frac{h}{T-t} = \mu(t),\qquad
\dot\mu = \frac{h}{(T-t)^2} = \frac{\mu^2}{h},
\tag{5}
$$

及尺度化状态

$$
\xi = \tilde\varepsilon, \qquad \eta = \frac{\tilde\rho}{\mu}.
\tag{6}
$$

由 $\frac{d}{dt} = \mu\frac{d}{ds}$，对 (2) 计算：

$$
\frac{d\xi}{ds} = \frac{1}{\mu}\dot{\tilde\varepsilon}
= \frac{1}{\mu}\big(\tilde\rho - k_1\mu M_\sigma\xi\big)
= \eta - k_1 M_\sigma\xi .
\tag{7}
$$

对 $\eta = \tilde\rho/\mu$，先算 $\frac{d\eta}{dt} = \frac{\dot{\tilde\rho}}{\mu} - \frac{\tilde\rho\,\dot\mu}{\mu^2} = \frac{\dot{\tilde\rho}}{\mu} - \frac{\tilde\rho}{h}$（用到 $\dot\mu/\mu^2 = 1/h$），再

$$
\frac{d\eta}{ds} = \frac{1}{\mu}\frac{d\eta}{dt}
= \frac{\dot{\tilde\rho}}{\mu^2} - \frac{\tilde\rho}{\mu h}
= -k_2 M_\sigma\xi - \frac{\sigma}{\mu^2}\operatorname{sign}\!\big(M_\sigma\xi\big) - \frac{\mathbf 1 a_0}{\mu^2} - \frac{\eta}{h}.
\tag{8}
$$

（其中 $\frac{\dot{\tilde\rho}}{\mu^2} = -k_2M_\sigma\xi - \frac{\sigma}{\mu^2}\mathrm{sign}(M_\sigma\xi) - \frac{\mathbf1 a_0}{\mu^2}$。）

综合 (7)(8)，令 $z = [\xi^T,\eta^T]^T$：

$$
\boxed{
\frac{dz}{ds} = \underbrace{\begin{pmatrix} -k_1 M_\sigma & I_n \\ -k_2 M_\sigma & -\frac1h I_n \end{pmatrix}}_{\bar A_\sigma} z
+ \underbrace{\begin{pmatrix} 0 \\[2pt] -\dfrac{1}{\mu^2}\big(\sigma\,\operatorname{sign}(M_\sigma\xi) + \mathbf 1 a_0\big) \end{pmatrix}}_{\bar B_\sigma(s)}
}
\tag{9}
$$

**关键一步**：时变系统 (2) 化为 $s$ 上的 LTI 系统 $\bar A_\sigma$ + 扰动 $\bar B_\sigma(s)$。由 $\mu = \frac{h}{T}e^{s/h}$，

$$
\frac{1}{\mu^2} = \frac{T^2}{h^2}\,e^{-2s/h},
\qquad
\|\bar B_\sigma(s)\| \le \frac{T^2}{h^2}\,(\sigma + \bar a_0)\sqrt n\, e^{-2s/h} =: b_0\, e^{-2s/h}.
\tag{10}
$$

即**扰动（含滑模项与未知加速度）在 $s$ 时间上指数衰减**（速率 $2/h$）。

### 3.2 $\bar A_\sigma$ 的 Hurwitz 性与衰减率

$M_\sigma \succ 0$ 对称，对角化 $M_\sigma = U\Lambda U^T$，$\Lambda = \operatorname{diag}\{\lambda_j\}$，$\lambda_j \ge \lambda_{\min}(M_\sigma) > 0$。
系统按 $\lambda_j$ 解耦，每个块

$$
\bar A_j = \begin{pmatrix} -k_1\lambda_j & 1 \\ -k_2\lambda_j & -\frac1h \end{pmatrix},
\qquad
\det(sI - \bar A_j) = s^2 + \big(k_1\lambda_j + \tfrac1h\big)s + \big(\tfrac{k_1}{h} + k_2\big)\lambda_j .
$$

$k_1,k_2,h>0$ 时两根实部均为负，故 $\bar A_\sigma$ Hurwitz。进一步要求衰减率 **$> 1/h$**：作平移 $w = s + \tfrac1h$ 得

$$
w^2 + \big(k_1\lambda_j - \tfrac1h\big)w + k_2\lambda_j = 0 .
$$

两根 $w$ 实部为负 $\iff k_1\lambda_j > \tfrac1h$ 且 $k_2\lambda_j > 0$，即条件 (3)。于是存在 $\lambda > 1/h$、$c_1>0$ 使
$\|e^{\bar A_\sigma s}\| \le c_1 e^{-\lambda s}$（对所有 $\sigma$，只要 $\lambda_{\min}(M_\sigma)\ge\bar\lambda$）。

### 3.3 受迫解的收敛

$$
z(s) = e^{\bar A_\sigma s}z(0) + \int_0^s e^{\bar A_\sigma(s-\tau)}\bar B_\sigma(\tau)\,d\tau ,
$$

$$
\|z(s)\| \le c_1 e^{-\lambda s}\|z(0)\| + c_1 b_0\!\int_0^s e^{-\lambda(s-\tau)}e^{-2\tau/h}d\tau
= O\!\Big(e^{-\min\{\lambda,\,2/h\}s}\Big).
$$

（若 $\lambda = 2/h$，积分为 $s e^{-2s/h}$，仍是 $O(e^{-\gamma s})$，$\forall\gamma<2/h$。）

故 $\|\xi(s)\|,\ \|\eta(s)\| = O(e^{-\gamma s})$，$\gamma = \min\{\lambda, 2/h\}$。回代 $e^{-\gamma s} = \big(\tfrac{T-t}{T}\big)^{\gamma h}$：

$$
\|\tilde\varepsilon(t)\| = \|\xi\| = O\big((T-t)^{\gamma h}\big),
\qquad
\|\tilde\rho(t)\| = \mu\|\eta\| = \frac{h}{T}e^{s/h}\,O\big(e^{-\gamma s}\big) = O\big((T-t)^{\gamma h - 1}\big).
$$

由条件 (3)：$\lambda > 1/h \Rightarrow \gamma h = \min\{\lambda h, 2\} > 1$，故 $\gamma h > 1$ 且 $\gamma h - 1 > 0$。
于是 $\lim_{t\to T^-}\tilde\varepsilon(t) = 0$、$\lim_{t\to T^-}\tilde\rho(t) = 0$。**Part I 证毕。** $\blacksquare$

> **注 1**：Part I 全程只用 $\|a_0\|\le\bar a_0$ 有界，**甚至不需要滑模项**（取 $\sigma=0$，扰动界 (10) 仍成立）。
> 滑模项的唯一作用是 $t\ge T$ 的持续跟踪（Part II）。
>
> **注 2（速率上界与增益建议）**：收敛指数受**扰动衰减速率** $2/h$ 限制，即 $\gamma = \min\{\lambda, 2/h\}$。
> 最优速率为 $\tilde\varepsilon = O((T-t)^2)$、$\tilde\rho = O((T-t))$，在 $\lambda \ge 2/h$ 时达到。
> 由 §3.2（复根情形）$\lambda = \tfrac12(k_1\lambda_{\min}(M) + 1/h)$，故建议取
> $k_1 \ge \tfrac{3}{h\lambda_{\min}(M)}$（即 $\lambda \ge 2/h$）以获得最快的 $\tilde\rho$ 收敛；
> 定理本身只需 $k_1 > \tfrac{1}{h\lambda_{\min}(M)}$（保证 $\tilde\rho$ 趋零，但指数 $\gamma h - 1$ 可能很小）。

---

## 4. Part II：$t \ge T$ 持续精确跟踪（滑模等价控制）

对 $t\ge T$，$\mu\equiv 0$，观测器退化为

$$
\dot\varepsilon_i = \rho_i, \qquad \dot\rho_i = -\sigma\,\operatorname{sign}(\psi_i),\qquad \psi_i = \sum_{j\in\mathcal N_i}(\varepsilon_i-\varepsilon_j) + g_i(\varepsilon_i - x_0),
$$

误差动力学

$$
\dot{\tilde\varepsilon} = \tilde\rho, \qquad
\dot{\tilde\rho} = -\sigma\,\operatorname{sign}\!\big(M_\sigma\tilde\varepsilon\big) - \mathbf 1\, a_0(t).
\tag{11}
$$

由 Part I 及解的连续性，$\tilde\varepsilon(T) = 0$、$\tilde\rho(T) = 0$。

**命题。** $(\tilde\varepsilon, \tilde\rho)\equiv(0,0)$ 是 (11) 在 $[T,\infty)$ 上的 Filippov 解。

**证明。** 在 $\tilde\varepsilon \equiv 0$ 上：$\dot{\tilde\varepsilon} = \tilde\rho = 0$ 自动成立（取 $\tilde\rho\equiv 0$）。对 $\dot{\tilde\rho}$，
$\operatorname{sign}(0) \in [-1,1]^n$（Filippov），故

$$
\dot{\tilde\rho} \in -\sigma[-1,1]^n - \mathbf 1 a_0 = \big\{-\sigma v - \mathbf 1 a_0 : v\in[-1,1]^n\big\}.
$$

取**等价控制**

$$
v^* = -\frac{a_0(t)}{\sigma}\,\mathbf 1 .
\tag{12}
$$

由 $\|a_0\| \le \bar a_0 < \sigma$，有 $\|v^*\|_\infty = |a_0|/\sigma < 1$，故 $v^* \in (-1,1)^n \subset [-1,1]^n$ 合法，且

$$
\dot{\tilde\rho} = -\sigma v^* - \mathbf 1 a_0 = \mathbf 1 a_0 - \mathbf 1 a_0 = 0 .
$$

故 $\tilde\rho \equiv 0$ 保持，滑模解 $(\tilde\varepsilon,\tilde\rho)\equiv(0,0)$ 成立。**Part II 证毕。** $\blacksquare$

**等价控制的物理解释**：滑模项 $-\sigma\,\mathrm{sign}(\psi_i)$ 在滑模面上提供平均控制
$-\sigma v^* = \mathbf 1 a_0(t)$，**逐点精确抵消**未知加速度 $a_0(t)$，从而 $\rho_i(t) = v_0(t)$ 始终精确跟踪，
$\varepsilon_i(t) = x_0(t)$ 由 $\dot\varepsilon_i = \rho_i$ 精确保持。注意 $v^*$ 与网络无关——这是
$M_\sigma\mathbf 1 = \mathbf g_\sigma$（$M_\sigma^{-1}\mathbf g_\sigma = \mathbf 1$）的直接推论，故对任意拓扑/可见性分布都成立。

---

## 5. 合起来：预设时间精确收敛

- $t \in [0,T)$：Part I → 误差按 (4) 幂次趋零，$t=T$ 时（由连续性）**精确为零**；
- $t \ge T$：Part II → 零误差是 Filippov 不变解，滑模等价控制持续抵消 $a_0$，误差恒为零。

定理 1 证毕。$\blacksquare$

---

## 6. 切换拓扑的推广（推论 1）

- **Part I**：$M_{\sigma(s)}$ 分段常值且 $\lambda_{\min}(M_\sigma)\ge\bar\lambda>0$。由 §3.2，所有 $\bar A_\sigma$ 具有
**一致的**衰减率 $\lambda>1/h$（衰减率只依赖 $k_1\bar\lambda$），故 $\|e^{\bar A_\sigma s}\|\le c_1 e^{-\lambda s}$ 对 $\sigma$ 一致，
切换系统的状态转移矩阵满足同样的指数界（逐段乘积），受迫解分析与 §3.3 相同。扰动界 $b_0$（式 (10)）与拓扑无关。
- **Part II**：等价控制 $v^* = -\tfrac{a_0}{\sigma}\mathbf 1$ 对任意 $M_\sigma$ 均合法（$M_\sigma^{-1}\mathbf g_\sigma = \mathbf 1$ 恒成立）。

故推论 1 成立。$\blacksquare$

---

## 7. 关于滑模吸引性的注记（鲁棒性）

定理 1 的 Part II 只证明了**零的不变性**（在 $t=T$ 精确到达零后保持为零）。若在 $t=T$ 附近存在微小扰动/数值误差，
需要滑模的**有限时间吸引性**把轨迹拉回零。这是标准的二阶滑模结果：

- **单智能体**（$n=1$）：(11) 即 $\ddot{\tilde\varepsilon} = -\sigma\mathrm{sign}(\tilde\varepsilon) - a_0$，是"库仑摩擦振子"，
当 $\sigma > \bar a_0$ 时原点全局有限时间稳定（能量 $\tfrac12\dot{\tilde\varepsilon}^2 + (\sigma-\bar a_0)|\tilde\varepsilon|$ 每个周期严格下降）。
- **多智能体**：$\dot{\tilde\varepsilon} = \tilde\rho$，$\dot{\tilde\rho} = -\sigma\mathrm{sign}(M_\sigma\tilde\varepsilon) - \mathbf1 a_0$ 的有限时间吸引性，
可用 Gong et al. (2022) 的 Lyapunov 技巧（$\dot V^b\le 0$ 的逐项分析，见其 Th. 1 式 (5)–(8)）证明。

因此在实际实现中，即使 $t=T$ 时有微小误差，滑模项会在有限时间内将其消除，PT-25' 具有对量测噪声/数值误差的鲁棒性
（代价是抖振，可用饱和函数 $\operatorname{sat}(\cdot)$ 或超螺旋算法连续化，见需求文档 §8）。

---

## 8. 结论与洞察

1. **时变增益 $\mu,\mu^2$ 负责"预设时间收敛"**：时间尺度变换把发散增益"压缩"成拉伸时间上的 Hurwitz 系统，
   使误差在 $t=T$ 精确趋零，速率与初值、网络无关。
2. **滑模项 $\sigma\,\mathrm{sign}(\psi_i)$ 负责"持续跟踪"**：在滑模面上以等价控制 $\sigma v^* = \mathbf 1 a_0$ 精确抵消
   未知有界加速度，保证 $t\ge T$ 恒零误差。**它不影响 Part I 的收敛（Part I 对 $\sigma=0$ 也成立）。**
3. **增益条件** $k_1 > \tfrac{1}{h\lambda_{\min}(M_\sigma)}$、$k_2>0$、$\sigma > \bar a_0$ 充分且简洁，
   滑模增益只需略大于加速度上界，**无需额外的图依赖裕度**。
4. 本证明同时确认了需求文档 §5.1 的"预期收敛定理"，并把它升级为严格定理（定理 1 + 推论 1）。
