# 双积分器滑模控制：量化器作用于滑模面 vs 作用于控制信号

比较两种量化接入方式，三种量化器。

---

## 1. 系统模型

$$
\ddot{x} = u + d(t),\quad
\begin{cases}
\dot{x}_1 = x_2,\\
\dot{x}_2 = u + d(t),
\end{cases}
\quad d(t) = A\sin(\omega t),\quad
s = c x_1 + x_2.
\tag{1}
$$

---

## 2. 三种量化器

- **对数量化器** (Fu & Xie 2005)

  $$f_{\log}(v) = \rho^{\,i} u_0,\quad \frac{1+\rho}{2}\rho^{\,i} u_0 \le |v| < \frac{1+\rho}{2\rho}\rho^{\,i} u_0,\; i \in \mathbb{Z}$$
  $$\delta = \frac{1-\rho}{1+\rho},\qquad |f_{\log}(v) - v| \le \delta|v|$$
  - 无穷电平，等比缩放
  - $v \to 0$ 时输出 $\to 0$（无死区）

- **Ceil 量化器**

  $$f_{\text{ceil}}(v) = \operatorname{sgn}(v) \cdot \lceil |v| \rceil$$
  - 等距阶梯，远离零取整（向上取整）
  - $|v| < 1$ 时输出 $\pm 1$（最小幅值恒定）

- **Floor 量化器**

  $$f_{\text{floor}}(v) = \operatorname{sgn}(v) \cdot \lfloor |v| \rfloor$$
  - 等距阶梯，**向零取整**（向下取整）
  - $|v| < 1$ 时输出 **0**（存在死区）

<p align="center">
  <img src="figures/quantizers_comparison.png" width="650" alt="三种量化器对比">
  <br>
  <em>图 1：三种量化器输入-输出特性对比（$\rho=0.5$）</em>
</p>

---

## 3. 方式 A：量化器作用于滑模面 $s$

量化器替代切换函数 $\tanh(\mu s)$：

$$
u = -c x_2 - k \cdot f(s).
\tag{2}
$$

### 3.1 算法

- **A1 — 理想（tanh）**
  $$u = -c x_2 - k\tanh(\mu s),\qquad \dot{s} = -k\tanh(\mu s) + d$$

- **A2 — 对数量化器**
  $$u = -c x_2 - k f_{\log}(s),\qquad \dot{s} = -k(1+\Delta)s + d,\; 1+\Delta \ge 1-\delta > 0$$

- **A3 — Ceil 量化器**
  $$u = -c x_2 - k \cdot \operatorname{sgn}(s)\lceil |s|\rceil,\qquad \dot{s} = -k\operatorname{sgn}(s)\lceil |s|\rceil + d$$

- **A4 — Floor 量化器**
  $$u = -c x_2 - k \cdot \operatorname{sgn}(s)\lfloor |s|\rfloor,\qquad \dot{s} = -k\operatorname{sgn}(s)\lfloor |s|\rfloor + d$$

A4 的关键缺陷：当 $|s| < 1$ 时 $\lfloor |s| \rfloor = 0$，切换项消失，系统退化为开环 $u = -c x_2$，完全无法抵抗扰动。

### 3.2 仿真结果

<p align="center">
  <img src="figures/appA_weak_k3.0.png" width="1000" alt="Approach A 弱扰动">
  <br>
  <em>图 2：量化器作用于 $s$ — 弱扰动 $d(t)=0.3\sin(2t)$，$k=3.0$</em>
</p>

<p align="center">
  <img src="figures/appA_strong_k3.0.png" width="1000" alt="Approach A 强扰动">
  <br>
  <em>图 3：量化器作用于 $s$ — 强扰动 $d(t)=0.8\sin(3t)$，$k=3.0$</em>
</p>

### 3.3 稳态精度

| 扰动 | $k$ | A1 理想 | A2 对数 | A3 Ceil | A4 Floor |
|---|---|---|---|---|---|
| 无 | 1.5 | 0 | $3\times 10^{-6}$ | $2\times 10^{-5}$ | **1.0** |
| $0.3\sin(2t)$ | 1.5 | 0.0025 | 0.073 | **$2\times 10^{-5}$** | **0.85** |
| $0.3\sin(2t)$ | 3.0 | 0.0013 | 0.058 | **$3\times 10^{-5}$** | **0.85** |
| $0.8\sin(3t)$ | 1.5 | 0.0071 | 0.151 | **$2\times 10^{-5}$** | **0.79** |
| $0.8\sin(3t)$ | 3.0 | 0.0034 | 0.125 | **$3\times 10^{-5}$** | **0.79** |

**结论**：
- Ceil 型（A3）扰动抑制最佳，$\|s\|_{\text{ss}} \sim 10^{-5}$，因为最小输出 $\pm 1$ 提供恒定切换力
- 对数型（A2）在 $s\approx 0$ 输出 $\to 0$，扰动抑制有限
- **Floor 型（A4）完全失败**——死区内切换力为零，$s$ 漂移至 $\sim 0.8$ 才被下一级电平捕获

---

## 4. 方式 B：量化器作用于控制信号 $u$

量化器缩放完整的控制信号：

$$
u = f(-c x_2 - k\tanh(\mu s)).
\tag{3}
$$

### 4.1 算法

- **B1 — 理想（tanh）**
  $$u = -c x_2 - k\tanh(\mu s),\qquad \dot{s} = -k\tanh(\mu s) + d$$

- **B2 — 对数量化器**
  $$u = f_{\log}(-c x_2 - k\tanh(\mu s)),\qquad \dot{s} = -\Delta\cdot c x_2 - (1+\Delta)k\tanh(\mu s) + d$$

- **B3 — Floor 量化器**
  $$u = \operatorname{sgn}(u_r)\lfloor|u_r|\rfloor,\; u_r = -c x_2 - k\tanh(\mu s)$$
  滑模动力学与 B2 形式相同，但 $f(\cdot) = \operatorname{sgn}(\cdot)\lfloor|\cdot|\rfloor$

- **B4 — Ceil 量化器**
  $$u = \operatorname{sgn}(u_r)\lceil|u_r|\rceil,\; u_r = -c x_2 - k\tanh(\mu s)$$
  滑模动力学与 B2 形式相同，但 $f(\cdot) = \operatorname{sgn}(\cdot)\lceil|\cdot|\rceil$

### 4.2 仿真结果

<p align="center">
  <img src="figures/appB_weak_k3.0.png" width="850" alt="Approach B 弱扰动">
  <br>
  <em>图 4：量化器作用于 $u$ — 弱扰动 $d(t)=0.3\sin(2t)$，$k=3.0$</em>
</p>

<p align="center">
  <img src="figures/appB_strong_k3.0.png" width="850" alt="Approach B 强扰动">
  <br>
  <em>图 5：量化器作用于 $u$ — 强扰动 $d(t)=0.8\sin(3t)$，$k=3.0$</em>
</p>

### 4.3 稳态精度

| 扰动 | $k$ | B1 理想 | B2 对数 | B3 Floor | **B4 Ceil** |
|---|---|---|---|---|---|
| 无 | 1.5 | 0 | $1\times 10^{-6}$ | 0.016 | **$1\times 10^{-5}$** |
| $0.3\sin(2t)$ | 1.5 | 0.0025 | 0.0029 | 0.015 | **$1\times 10^{-5}$** |
| $0.3\sin(2t)$ | 3.0 | 0.0013 | 0.0014 | 0.007 | **$1\times 10^{-5}$** |
| $0.8\sin(3t)$ | 1.5 | 0.0071 | 0.0075 | 0.015 | **$1\times 10^{-5}$** |
| $0.8\sin(3t)$ | 3.0 | 0.0034 | 0.0036 | 0.007 | **$1\times 10^{-5}$** |

**结论**：
- 对数型（B2）扰动抑制接近理想，因为 $\tanh$ 的高频切换被保留
- Floor 型（B3）仍能工作，$\|s\|_{\text{ss}} \sim 0.007$，但信号截断降低了精度
- **Ceil 型（B4）最优**，$\|s\|_{\text{ss}} \sim 10^{-5}$，因为 Ceil 量化器远离零取整，保证了 $u$ 的最小幅值

---

## 5. 综合对比

| 量化器 | 作用在 $s$ 上 | 作用在 $u$ 上 |
|---|---|---|
| **对数型** | 扰动抑制差（$0.06$–$0.15$），趋近段光滑 | 接近理想（$0.001$–$0.008$），滑模条件需验证 |
| **Ceil 型** | **最优（$2\times 10^{-5}$）**，恒定最小切换力 | **最优（$1\times 10^{-5}$）**，$	anh$ 切换保持 |
| **Floor 型** | **无法工作**（$0.8$–$1.0$），死区消除切换 | 可用（$0.007$–$0.015$），精度损失约 2 倍 |

### 5.1 原因分析

Floor 量化器的死区（$|v|<1$ 输出 0）是问题的根源：

- **作用在 $s$ 上（A4）**：当 $|s|<1$ 时 $f_{\text{floor}}(s)=0$，等效控制 $u=-c x_2$ 无切换项。若系统受扰动，$s$ 漂移至 $|s|\ge 1$ 时切换恢复，但 $s$ 在 0 附近无法稳定——形成大幅极限环（$|s| \approx 0.8$）。
- **作用在 $u$ 上（B3）**：即使 $f_{\text{floor}}(u_r)=0$，$u_r = -c x_2 - k\tanh(\mu s)$ 本身在滑模段仍持续切换，$\tanh(\mu s)$ 的符号变化保证 $u_r$ 跨越 $\pm 1$ 边界，Floor 量化器的死区不会完全消除切换力。

### 5.2 选择建议

- **扰动抑制优先** → **Ceil 量化器**（作用在 $u$ 或 $s$ 均可，$\|s\|_{\text{ss}} \sim 10^{-5}$）
- **趋近段行为优先，可容忍一定稳态误差** → 方式 B + **对数**
- **避免使用** → 方式 A + **Floor**（死区致命）；方式 B + **Floor**（精度差）

---

## 参考文献

1. M. Fu and L. Xie, "The sector bound approach to quantized feedback control," *IEEE Trans. Autom. Control*, vol. 50, no. 11, pp. 1698–1711, 2005.
2. V. I. Utkin, *Sliding Modes in Control and Optimization*. Springer, 1992.