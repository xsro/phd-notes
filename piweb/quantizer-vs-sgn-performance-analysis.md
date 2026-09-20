# 均匀量化器 Q_Δ(s) 与符号函数 sgn(s) 的性能对比分析

## 1. 问题设定

在滑模控制律

$$
u = \frac{1}{b}\bigl[-f(x) + \ddot{x}_d - \lambda\dot{e} - ks - \eta \,\psi(s)\bigr]
$$

中，切换项 $\psi(s)$ 取三种形式：

| 制度 | 切换函数 | 死区 |
|---|---|---|
| **理想符号函数** | $\operatorname{sgn}(s) = \begin{cases} 1, & s>0 \\ 0, & s=0 \\ -1, & s<0 \end{cases}$ | 仅 $s=0$ |
| **有死区均匀量化器** | $Q_\Delta(s) = \Delta\cdot\operatorname{round}(s/\Delta)$ | $\|s\|\le\Delta/2$ |
| **无死区均匀量化器** | $Q_\Delta^{\mathrm{nd}}(s) = \begin{cases} \frac{\Delta}{2}+\Delta\lfloor\frac{s}{\Delta}\rfloor, & s>0 \\ 0, & s=0 \\ -\frac{\Delta}{2}-\Delta\lfloor-\frac{s}{\Delta}\rfloor, & s<0 \end{cases}$ | 无（仅 $s=0$） |

三种制度的闭环滑模动态分别为

$$
\begin{aligned}
\text{sgn:}&\qquad \dot{s} = -ks - \eta\operatorname{sgn}(s) + d(t),\\[4pt]
\text{Q}_\Delta\text{:}&\qquad \dot{s} = -ks - \eta Q_\Delta(s) + d(t),\\[4pt]
\text{Q}_\Delta^{\mathrm{nd}}\text{:}&\qquad \dot{s} = -ks - \eta Q_\Delta^{\mathrm{nd}}(s) + d(t).
\end{aligned}
$$

以下从**收敛速度**、**抗扰能力**、**稳态误差**、**抖振特性**和**可实现性**五个维度进行系统对比。分析中均假设 $\lambda>0$、$k>0$、$\eta>0$、$\Delta>0$，扰动满足 $|d(t)|\le D$。

### 函数波形对比

下图给出了 $\Delta=1.0$ 时三种切换函数的波形对比。注意有死区量化器在 $|s|\le\Delta/2$ 区间内输出为零（绿色阴影），而无死区量化器在任意小的 $|s|>0$ 处均有至少 $\Delta/2$ 的输出幅值。

![sgn vs quantizer 3-way](sgn_vs_quantizer_3way.png)

### 无死区量化器的关键性质

$Q_\Delta^{\mathrm{nd}}(s)$ 满足：

1. **严格同号**：$s\,Q_\Delta^{\mathrm{nd}}(s) > 0$ 对所有 $s\neq 0$ 成立；
2. **量化误差有界**：$|Q_\Delta^{\mathrm{nd}}(s) - s| \le \Delta/2$；
3. **无死区**：$Q_\Delta^{\mathrm{nd}}(s) = 0$ 仅当 $s=0$；
4. **最小切换幅值**：$|Q_\Delta^{\mathrm{nd}}(s)| \ge \Delta/2$ 对所有 $s\neq 0$ 成立；
5. **死区外下界**：当 $|s| > \Delta/2$ 时，$s\,Q_\Delta^{\mathrm{nd}}(s) \ge s^2 - \frac{\Delta}{2}|s|$（与有死区量化器相同）。

---

## 2. 无扰动情况（$d(t)\equiv 0$）

### 2.1 收敛速度

选取 $V_s = \frac{1}{2}s^2$。

**sgn(s) 制度：**

$$
\begin{aligned}
\dot{V}_s &= s(-ks - \eta\operatorname{sgn}(s)) \\
&= -ks^2 - \eta|s|.
\end{aligned}
$$

由于 $\eta|s|>0$（$s\neq 0$），收敛由两项驱动：

- 二次项 $-ks^2$：主导大 $|s|$ 时的收敛；
- 线性项 $-\eta|s|$：在小 $|s|$ 时仍提供恒定驱动力。

由 $\dot{V}_s \le -\eta|s|$ 可得有限时间到达：

$$
|s(t)| \le \max\!\Bigl\{|s(0)| - \eta t,\;0\Bigr\},
$$

即滑模变量可在**有限时间**内精确到达 $s=0$。到达后的理想滑模运动为 $\dot{s}=0$（等效控制），系统沿滑模面滑动。

**Q_Δ(s) 制度：**

$$
\begin{aligned}
\dot{V}_s &= s(-ks - \eta Q_\Delta(s)) \\
&= -ks^2 - \eta sQ_\Delta(s) \\
&\le -ks^2 \qquad (\text{因 } sQ_\Delta(s)\ge 0).
\end{aligned}
$$

- **死区外**（$|s|>\Delta/2$）：利用下界 $sQ_\Delta(s) \ge s^2 - \frac{\Delta}{2}|s|$，
  $$
  \dot{V}_s \le -(k+\eta)s^2 + \frac{\eta\Delta}{2}|s|.
  $$
  当 $|s|$ 较大时，$-(k+\eta)s^2$ 主导，收敛速度甚至**快于** sgn 制度（等效切换增益为 $k+\eta$ 而非 $k$）；当 $|s|$ 接近 $\Delta/2$ 时，$\frac{\eta\Delta}{2}|s|$ 项削弱驱动力。

- **死区内**（$|s|\le\Delta/2$）：$Q_\Delta(s)=0$，
  $$
  \dot{V}_s = -ks^2,
  $$
  仅由连续反馈 $-ks$ 驱动。此时**没有**有限时间到达特性，$s$ 以指数速率 $\dot{s}=-ks$ 渐近收敛到零。

**Q_Δ^{nd}(s) 制度（无死区）：**

$$
\begin{aligned}
\dot{V}_s &= s(-ks - \eta Q_\Delta^{\mathrm{nd}}(s)) \\
&= -ks^2 - \eta sQ_\Delta^{\mathrm{nd}}(s).
\end{aligned}
$$

利用性质 $sQ_\Delta^{\mathrm{nd}}(s) > 0$（$s\neq 0$）和最小幅值性质 $|Q_\Delta^{\mathrm{nd}}(s)| \ge \Delta/2$：

- **小 $|s|$ 阶段**（$0<|s|\le\Delta/2$）：$|Q_\Delta^{\mathrm{nd}}(s)| = \Delta/2$，
  $$
  \dot{V}_s = -ks^2 - \eta\cdot\frac{\Delta}{2}|s|.
  $$
  此时存在线性项 $-\eta\frac{\Delta}{2}|s|$，提供**恒定的最小切换驱动力**。由 $\dot{V}_s \le -\eta\frac{\Delta}{2}|s|$ 可得有限时间到达：
  $$
  |s(t)| \le \max\!\Bigl\{|s(0)| - \eta\frac{\Delta}{2}\,t,\;0\Bigr\}.
  $$
  即使 $|s|$ 在 $[0,\Delta/2]$ 范围内，也能在有限时间内收敛到零。

- **大 $|s|$ 阶段**（$|s|>\Delta/2$）：与有死区量化器相同，
  $$
  \dot{V}_s \le -(k+\eta)s^2 + \frac{\eta\Delta}{2}|s|.
  $$
  等效增益为 $k+\eta$，收敛速度快于 sgn(s)。

**对比小结：**

| 指标 | sgn(s) | Q_Δ(s)（有死区） | Q_Δ^{nd}(s)（无死区） |
|---|---|---|---|
| 大 abs(s) 阶段 | 指数收敛（速率 $\approx k$） | 指数收敛（速率 $\approx k+\eta$，**更快**） | 指数收敛（速率 $\approx k+\eta$，**更快**） |
| 小 abs(s) 阶段 | **有限时间到达** $s=0$ | 指数收敛（速率 $k$），无有限时间特性 | **有限时间到达** $s=0$（速率 $\eta\Delta/2$） |
| 最终状态 | 精确收敛到 $s=0$ | 精确收敛到 $s=0$（死区内 $-ks$ 持续驱动） | 精确收敛到 $s=0$ |

**关键差异**：sgn(s) 在滑模面附近凭借恒定切换幅值 $\eta$ 实现有限时间到达；Q_Δ(s) 在死区内关闭切换项，仅靠连续反馈 $-ks$ 实现渐近收敛；Q_Δ^{nd}(s) 在任意小的 $|s|$ 处仍有 $\Delta/2$ 的最小切换幅值，兼具有限时间到达能力和无死区特性。

---

### 2.2 跟踪误差收敛

两种制度下 $s(t)$ 均收敛到零，误差系统 $\dot{e} = -\lambda e + s$ 为指数稳定线性系统，输入 $s(t)\to 0$，故

$$
\lim_{t\to\infty} e(t) = 0, \qquad \lim_{t\to\infty} \dot{e}(t) = 0.
$$

**收敛速度差异**：sgn(s) 制度下 $s$ 有限时间归零，误差系统在 $s$ 归零后以 $e^{-\lambda t}$ 自由衰减；Q_Δ(s) 制度下 $s$ 在死区内指数衰减，相当于叠加了一个指数衰减的输入，最终误差收敛速率由 $\lambda$ 和 $k$ 共同决定。数值上 sgn(s) 的整体收敛通常更快。

---

## 3. 有界扰动情况（$|d(t)|\le D$）

### 3.1 抗扰能力与稳态误差

**sgn(s) 制度：**

$$
\begin{aligned}
\dot{V}_s &= -ks^2 - \eta|s| + sd(t) \\
&\le -ks^2 - (\eta - D)|s|.
\end{aligned}
$$

若 $\eta > D$，则 $\dot{V}_s < 0$ 对所有 $s\neq 0$ 成立。这意味着：

- 理想情况下，滑模变量**仍然收敛到 $s=0$**；
- 匹配扰动被完全抑制——这是滑模控制的核心优势；
- 扰动仅影响到达阶段的速率，不影响最终精度。

**Q_Δ(s) 制度：**

- **死区外**（$|s|>\Delta/2$）：
  $$
  \dot{V}_s \le -(k+\eta)s^2 + \Bigl(\frac{\eta\Delta}{2} + D\Bigr)|s|.
  $$
  当 $|s| > \dfrac{\eta\Delta/2 + D}{k+\eta}$ 时 $\dot{V}_s<0$。

- **死区内**（$|s|\le\Delta/2$）：
  $$
  \dot{V}_s \le -ks^2 + D|s| = -|s|(k|s| - D).
  $$
  当 $|s| > D/k$ 时 $\dot{V}_s<0$。

最终有界性：

$$
\boxed{\limsup_{t\to\infty}|s(t)| \le \max\!\Bigl(\frac{\Delta}{2},\;\frac{D}{k}\Bigr)}.
$$

**Q_Δ^{nd}(s) 制度（无死区）：**

由于无死区量化器对任意 $s\neq 0$ 均有 $|Q_\Delta^{\mathrm{nd}}(s)| \ge \Delta/2$，可取统一的下界估计。利用 $sQ_\Delta^{\mathrm{nd}}(s) \ge \frac{\Delta}{2}|s|$（当 $0<|s|\le\Delta/2$）和 $sQ_\Delta^{\mathrm{nd}}(s) \ge s^2-\frac{\Delta}{2}|s|$（当 $|s|>\Delta/2$），分两种情况讨论：

- **小 $|s|$**（$0<|s|\le\Delta/2$）：$Q_\Delta^{\mathrm{nd}}(s) = \pm\Delta/2$，
  $$
  \begin{aligned}
  \dot{V}_s &= -ks^2 - \eta\cdot\frac{\Delta}{2}|s| + sd(t) \\
  &\le -ks^2 - \Bigl(\eta\frac{\Delta}{2} - D\Bigr)|s|.
  \end{aligned}
  $$
  若 $\eta\Delta/2 > D$，则 $\dot{V}_s < 0$ 对所有 $0<|s|\le\Delta/2$ 成立，滑模变量将继续向 $s=0$ 收敛。

- **大 $|s|$**（$|s|>\Delta/2$）：与有死区量化器相同，
  $$
  \dot{V}_s \le -(k+\eta)s^2 + \Bigl(\frac{\eta\Delta}{2} + D\Bigr)|s|.
  $$
  当 $|s| > \dfrac{\eta\Delta/2 + D}{k+\eta}$ 时 $\dot{V}_s<0$。

**最终有界性：**

若 $\eta\Delta/2 > D$，则 $\dot{V}_s<0$ 对所有 $s\neq 0$ 成立，滑模变量**指数收敛到零**，稳态误差为 $0$。

若 $\eta\Delta/2 \le D$，则存在稳态误差。由于 $r_1 = \frac{\eta\Delta/2 + D}{k+\eta} \le \max(\Delta/2, D/k)$，且小 $|s|$ 区域由 $-\eta\Delta/2|s| + D|s|$ 主导，可得：

$$
\boxed{\limsup_{t\to\infty}|s(t)| \le \frac{D}{\eta\Delta/2} \cdot \frac{\Delta}{2} = \frac{D}{k+\eta}\cdot\frac{\eta\Delta/2}{\eta\Delta/2} \text{（精确形式略复杂）}}
$$

更精确地，令 $\mu = \eta\Delta/2 - D$，当 $\mu > 0$ 时系统收敛到零；当 $\mu \le 0$ 时，小 $|s|$ 区域内 $\dot{V}_s$ 的符号由 $-ks^2 + (\eta\Delta/2 - D)|s|$ 决定，最终界为：

$$
\boxed{\limsup_{t\to\infty}|s(t)| \le \frac{D - \eta\Delta/2}{k} \quad (\text{若 } D > \eta\Delta/2)}.
$$

**对比小结：**

| 指标 | sgn(s) | Q_Δ(s)（有死区） | Q_Δ^{nd}(s)（无死区） |
|---|---|---|---|
| 扰动下最终界 | $0$（$\eta>D$ 时） | $\max(\Delta/2,\;D/k)$ | $0$（$\eta\Delta/2>D$ 时）；否则 $\frac{D-\eta\Delta/2}{k}$ |
| 扰动抑制能力 | **完全抑制**（$\eta>D$） | 有界抑制，稳态误差 $\ge\Delta/2$ | **部分抑制**（条件更宽松：$\eta\Delta/2>D$） |
| 死区影响 | 无 | 有（$\Delta/2$） | 无 |

**核心差异**：sgn(s) 依靠恒定切换幅值 $\eta>D$ 完全抵消扰动；Q_Δ(s) 在死区内切换关闭，必然存在 $\ge\Delta/2$ 的稳态误差；Q_Δ^{nd}(s) 在死区外有与 Q_Δ(s) 相同的收敛特性，在死区内仍有 $\Delta/2$ 的最小切换幅值，当 $\eta\Delta/2>D$ 时可实现零稳态误差——条件比 sgn(s) 的 $\eta>D$ 更宽松（因 $\Delta/2<1$ 时），但比 Q_Δ(s) 的抗扰能力更强。

---

### 3.2 扰动下的到达时间

**sgn(s)：**

由 $\dfrac{d}{dt}|s| \le -k|s| - (\eta - D) \le -(\eta-D)$，到达时间上界为

$$
T_{\mathrm{reach}}^{\operatorname{sgn}} \le \frac{|s(0)|}{\eta - D}.
$$

**Q_Δ(s)：**

死区外 $\dfrac{d}{dt}|s| \le -(k+\eta)|s| + \dfrac{\eta\Delta}{2} + D$，仅能保证进入 $|s|\le r_1$ 的时间上界，且最终只能进入 $\max(\Delta/2, D/k)$ 的邻域，不能保证到达原点。

---

## 4. 抖振特性

这是两种制度**最本质的区别**。

### 4.1 sgn(s) 的抖振

理想 sgn(s) 在 $s=0$ 处发生幅值为 $2\eta$ 的不连续跳变。在实际系统中：

- 执行器具有有限带宽和响应时间，无法实现无限频率切换；
- 传感器存在测量噪声，使 $s$ 在零附近随机波动；
- 未建模动态会激发高频切换。

结果：控制信号 $u$ 呈现**高频、等幅的锯齿形抖振**，频率由系统惯性和采样周期决定，幅值约为 $\eta$。抖振会：

- 加剧执行器磨损；
- 激励未建模高频动态；
- 降低控制精度。

### 4.2 Q_Δ(s) 的抖振

均匀量化器将连续的 $s$ 映射到离散电平 $\{k\Delta\}$：

- 当 $s$ 在 $[-\Delta/2,\;\Delta/2]$ 内时，$Q_\Delta(s)=0$，**切换项完全关闭**；
- 当 $s$ 跨越量化阈值时，输出发生幅值为 $\eta\Delta$ 的跳变。

与 sgn(s) 相比：

| 特性 | sgn(s) | Q_Δ(s) |
|---|---|---|
| 零点附近切换幅值 | $\eta$（恒定） | $0$（死区内关闭） |
| 切换频率 | 理论上无限高 | 受限于 $s$ 跨越阈值的频率 |
| 抖振幅值 | $\approx\eta$ | $\approx\eta\Delta$（死区外）或 $0$（死区内） |
| 控制信号平滑度 | 极差 | 较好（死区内为连续信号 $-ks$） |

**关键优势**：Q_Δ(s) 在滑模面附近（$|s|\le\Delta/2$）**完全消除了切换动作**，控制信号退化为连续反馈 $-ks$，从根本上抑制了抖振。代价是引入了大小为 $\Delta/2$ 的死区，导致稳态误差。

### 4.3 Q_Δ^{nd}(s) 的抖振

无死区量化器将连续的 $s$ 映射到离散电平 $\{\pm\Delta/2,\;\pm3\Delta/2,\;\pm5\Delta/2,\dots\}$：

- **无死区**：任意小的 $|s|>0$ 均有切换输出，最小幅值为 $\eta\Delta/2$；
- 当 $s$ 跨越量化阈值（$0,\;\pm\Delta,\;\pm2\Delta,\dots$）时，输出发生幅值为 $\eta\Delta$ 的跳变。

与 Q_Δ(s) 和 sgn(s) 的对比：

| 特性 | sgn(s) | Q_Δ(s)（有死区） | Q_Δ^{nd}(s)（无死区） |
|---|---|---|---|
| 零点附近切换幅值 | $\eta$（恒定） | $0$（死区内关闭） | $\eta\Delta/2$（恒定最小值） |
| 切换频率 | 理论上无限高 | 受限于 $s$ 跨越阈值的频率 | 受限于 $s$ 跨越阈值的频率 |
| 抖振幅值 | $\approx\eta$ | $0$（死区内）或 $\approx\eta\Delta$（死区外） | $\approx\eta\Delta/2$（全范围） |
| 控制信号平滑度 | 极差 | 较好（死区内连续） | 中等（无死区，但阶梯高度 $\eta\Delta$） |

**折中分析**：Q_Δ^{nd}(s) 的抖振幅值约为 $\eta\Delta/2$，介于 sgn(s) 的 $\eta$ 和 Q_Δ(s) 的 $0$（死区内）之间。它在零点附近无法像 Q_Δ(s) 那样完全消除切换，但相比 sgn(s) 仍大幅降低了切换幅值（减小因子 $\Delta/2$）。同时，由于不存在死区，它避免了稳态误差问题。

---

## 5. 可实现性

| 方面 | sgn(s) | Q_Δ(s)（有死区） | Q_Δ^{nd}(s)（无死区） |
|---|---|---|---|
| 数字实现 | 需极高采样频率才能逼近 | 可直接实现，$\operatorname{round}(\cdot)$ 为标准函数 | 可直接实现，$\lfloor\cdot\rfloor$ 为标准函数 |
| 对噪声的敏感性 | 极高（噪声直接触发切换） | 中等（噪声需克服 $\Delta/2$ 死区） | 较低（噪声需克服 $\Delta/2$ 才改变输出） |
| 执行器要求 | 需高频响应 | 可接受较低带宽 | 可接受较低带宽 |
| 参数整定 | 仅 $\eta$ | $\eta$ 和 $\Delta$ | $\eta$ 和 $\Delta$ |

---

## 6. 综合对比表

| 维度 | sgn(s) | Q_Δ(s)（有死区） | Q_Δ^{nd}(s)（无死区） | 胜出 |
|---|---|---|---|---|
| **无扰动收敛速度** | 大误差阶段较慢，小误差阶段有限时间到达 | 大误差阶段更快（$k+\eta$），小误差阶段指数收敛 | 大误差阶段更快（$k+\eta$），小误差阶段有限时间到达 | **Q_Δ^{nd}(s)** |
| **无扰动最终精度** | 精确到 $s=0$ | 精确到 $s=0$（$-ks$ 驱动） | 精确到 $s=0$ | 平手 |
| **抗扰能力** | $\eta>D$ 时完全抑制，稳态误差 $0$ | 稳态误差 $\max(\Delta/2, D/k)$ | $\eta\Delta/2>D$ 时零稳态误差；否则有界 | **sgn(s)**（条件最简单） |
| **抖振抑制** | 严重抖振 | 死区内无切换，抖振显著减轻 | 全范围切换幅值 $\eta\Delta/2$，抖振中等 | **Q_Δ(s)**（死区内最优） |
| **可实现性** | 理想化，实际需近似 | 可直接数字实现 | 可直接数字实现 | **Q_Δ(s) / Q_Δ^{nd}(s)** |
| **参数灵活性** | 仅 $\eta$ | $\eta$ 和 $\Delta$ | $\eta$ 和 $\Delta$ | **Q_Δ(s) / Q_Δ^{nd}(s)** |
| **对测量噪声** | 极敏感 | 有死区滤波效果 | 有死区外滤波效果 | **Q_Δ(s)** |
| **稳态误差（有扰动）** | $0$（理想） | $\ge\Delta/2$ | $0$（条件满足时） | **sgn(s) / Q_Δ^{nd}(s)** |

---

## 7. 设计启示

### 7.1 sgn(s) 的适用场景

- 仿真研究、理论分析中作为理想基准；
- 对收敛速度要求极高、且可容忍剧烈抖振的场合（如某些开关电源）；
- 执行器带宽足够高、噪声极低的实验平台。

### 7.2 Q_Δ(s) 的适用场景

- 实际数字控制系统（DSP/FPGA 实现）；
- 执行器带宽有限的机电系统；
- 对抖振敏感的场合（如机器人关节控制、飞行器舵面控制）；
- 需要同时兼顾收敛速度和稳态精度的场景。

### 7.3 Q_Δ^{nd}(s) 的适用场景

- 需要零稳态误差但又无法承受 sgn(s) 级别抖振的场合；
- 扰动较小（$D < \eta\Delta/2$）且对收敛速度要求较高的系统；
- 作为 Q_Δ(s) 和 sgn(s) 之间的折中方案——比 sng(s) 抖振小，比 Q_Δ(s) 无死区。

### 7.4 参数折中原则

- **$\Delta$ 的选择**：$\Delta$ 越小，死区（对 Q_Δ(s)）越小、稳态误差越小，但切换越频繁、抖振越严重；$\Delta$ 越大，死区越大、稳态误差越大，但切换越少、控制越平滑。应根据传感器噪声水平和执行器分辨率确定。
- **$\eta$ 的选择**：$\eta$ 增大可提高抗扰能力和收敛速度，但增大了切换幅值，需在抗扰性和抖振之间折中。
- **$k$ 的选择**：$k$ 增大可减小扰动下的稳态误差 $D/k$，同时加快死区内的收敛，但可能放大噪声。
- **无死区量化器的特殊折中**：$\eta\Delta/2 > D$ 是零稳态误差的条件。增大 $\eta$ 或减小 $\Delta$ 均可使条件更容易满足，但 $\eta$ 增大会增加抖振，$\Delta$ 减小也会增加切换频率。

---

## 8. 结论

将滑模控制中的 $\operatorname{sgn}(s)$ 替换为量化器，本质上是在**理想滑模的强鲁棒性**与**实际可实现性**之间进行折中。本文对比了三种切换函数：

1. **$\operatorname{sgn}(s)$**：理想滑模基准，收敛最快、抗扰最强，但抖振严重，实际不可实现；
2. **$Q_\Delta(s)$（有死区均匀量化器）**：死区内切换完全关闭，抖振抑制最优，代价是引入 $\Delta/2$ 死区导致稳态误差；
3. **$Q_\Delta^{\mathrm{nd}}(s)$（无死区均匀量化器）**：兼具有限时间到达能力和无死区特性，抖振介于两者之间，当 $\eta\Delta/2>D$ 时可实现零稳态误差。

**无扰动时**：三种制度均能使系统全局渐近稳定。$\operatorname{sgn}(s)$ 和 $Q_\Delta^{\mathrm{nd}}(s)$ 凭借恒定切换幅值实现有限时间到达，收敛快于 $Q_\Delta(s)$；$Q_\Delta^{\mathrm{nd}}(s)$ 在大误差时等效增益更大（$k+\eta$），追赶速度最快。

**有扰动时**：$\operatorname{sgn}(s)$ 在 $\eta>D$ 时理论上完全抑制匹配扰动，但伴随剧烈抖振；$Q_\Delta(s)$ 引入 $\max(\Delta/2, D/k)$ 的稳态误差，但死区内控制信号平滑；$Q_\Delta^{\mathrm{nd}}(s)$ 在 $\eta\Delta/2>D$ 时可实现零稳态误差，条件比 $\operatorname{sgn}(s)$ 的 $\eta>D$ 更宽松，同时抖振幅值仅为 $\eta\Delta/2$。

**工程选择**：
- 对抖振极度敏感且可容忍稳态误差 → 选 $Q_\Delta(s)$；
- 需要零稳态误差且扰动较小 → 选 $Q_\Delta^{\mathrm{nd}}(s)$；
- 仿真研究、理论极限 → 选 $\operatorname{sgn}(s)$。

实际数字控制系统中，$Q_\Delta(s)$ 和 $Q_\Delta^{\mathrm{nd}}(s)$ 均可直接实现。通过合理选择 $\Delta$ 和 $\eta$，可以在收敛速度、抗扰能力、稳态精度和抖振抑制之间取得满意的平衡。若需进一步消除稳态误差，可结合积分项或自适应增益设计。

---

## 参考来源

- 本文基于《均匀量化滑模控制及 Lyapunov 稳定性证明》中的系统模型与分析框架编写。
- 蔡中泽：《含有不匹配干扰的二阶非线性系统滑模控制研究》，哈尔滨工业大学，2024。