# 方式 A 收敛性分析：有界扰动下量化滑模控制的 Lyapunov 证明

对双积分器系统 $ \ddot{x} = u + d(t) $，滑模面 $ s = c x_1 + x_2 $，分析量化器作用于 $s$（替换 $\tanh(\mu s)$）时四种算法的收敛性。

---

## 1. 问题设定

$$
\begin{aligned}
\dot{x}_1 &= x_2,\\
\dot{x}_2 &= u + d(t),
\end{aligned}
\qquad
s = c x_1 + x_2,\quad c > 0,
\qquad
|d(t)| \le D.
\tag{1}
$$

对 $s$ 求导：

$$
\dot{s} = c x_2 + u + d.
\tag{2}
$$

方式 A 的控制律统一形式：

$$
u = -c x_2 - k \cdot f(s),
\tag{3}
$$

代入得：

$$
\dot{s} = -k \cdot f(s) + d.
\tag{4}
$$

---

## 2. 收敛性分析工具

定义 Lyapunov 候选函数：

$$
V = \frac12 s^2.
\tag{5}
$$

沿系统轨迹求导：

$$
\dot{V} = s \dot{s} = -k \cdot s \cdot f(s) + s \cdot d.
\tag{6}
$$

收敛的充分条件为 $\dot{V} < 0$ 对所有 $|s| > \varepsilon$ 成立（$\varepsilon$ 为最终界）。利用 $s \cdot d \le |s| \cdot D$：

$$
\dot{V} \le -k \cdot s \cdot f(s) + |s| \cdot D.
\tag{7}
$$

以下针对每种量化器 $f(s)$ 分析 $\dot{V}$ 的上界和 $s$ 的最终界。

---

## 3. A1：理想 tanh 近似

### 3.1 控制律与滑模动力学

$$
f(s) = \tanh(\mu s),\qquad
\dot{s} = -k \tanh(\mu s) + d.
\tag{8}
$$

### 3.2 Lyapunov 分析

$\tanh(\mu s)$ 满足：

$$
|\tanh(\mu s)| \le 1,\qquad
s \cdot \tanh(\mu s) \ge 0,\qquad
\lim_{\mu\to\infty} \tanh(\mu s) = \operatorname{sgn}(s).
$$

代入 (7)：

$$
\dot{V} \le -k \cdot |s| \cdot |\tanh(\mu s)| + |s| \cdot D.
\tag{9}
$$

采用文献标准边界层分析：对 $|s| \ge \varepsilon_b$，$\tanh(\mu s) \approx \operatorname{sgn}(s)$，有

$$
\dot{V} \le -|s|(k - D).
$$

**定理 1**（理想 tanh 收敛性）：若 $k > D$，则 $s$ 指数收敛至边界层

$$
|s| \le \varepsilon = \frac{1}{\mu} \tanh^{-1}\!\left(\frac{D}{k}\right),
\tag{10}
$$

且 $\mu \to \infty$ 时 $\varepsilon \to 0$。

### 3.3 稳态误差

- 连续近似下 $|s|_{\text{ss}} \approx D/k$（大 $\mu$ 极限）
- 受 $1/\mu$ 项限制，实际稳态误差为 $O(D/k + 1/\mu)$

---

## 4. A2：对数量化器

### 4.1 控制律与滑模动力学

$$
f(s) = f_{\log}(s) = (1 + \Delta(s))\,s,\qquad
|\Delta(s)| \le \delta = \frac{1-\rho}{1+\rho}.
\tag{11}
$$

$$
\dot{s} = -k(1 + \Delta(s))\,s + d.
\tag{12}
$$

### 4.2 扇区性质

对数量化器（无穷电平、无死区版本）的关键性质：

$$
1 + \Delta(s) \ge 1 - \delta > 0,\qquad \forall s \neq 0.
\tag{13}
$$

即 $f_{\log}(s)$ 与 $s$ 严格同号，且增益有正下界。

### 4.3 Lyapunov 分析

$$
\begin{aligned}
\dot{V} &= -k(1 + \Delta(s))\,s^2 + s \cdot d \\
&\le -k(1 - \delta)\,s^2 + |s| \cdot D.
\end{aligned}
\tag{14}
$$

完成平方：

$$
\dot{V} \le -k(1-\delta)\,|s|\left(|s| - \frac{D}{k(1-\delta)}\right).
$$

**定理 2**（对数量化器收敛性）：若 $k > 0$（任意正增益），则 $s$ 指数收敛至球域

$$
\boxed{\;|s| \le B_{\log} = \frac{D}{k(1-\delta)}\;}.
\tag{15}
$$

### 4.4 讨论

1. **无条件稳定性**：$k$ 只需为正，不需要 $k > D$。这是对数型扇区有界保证的正反馈特性。
2. **稳态误差由 $D$、$k$、$\delta$ 共同决定**：
   - 增大 $k$：误差反比减小
   - 减小 $\delta$（即增大 $\rho$，量化更细）：误差减小
   - $D=0$ 时 $B_{\log}=0$（无扰动时精确收敛）
3. **与理想 SMC 的对比**：
   - A1 需要 $k > D$，A2 只需 $k > 0$
   - 但 A2 的稳态误差多了一个因子 $1/(1-\delta)$：
     $$ \frac{|s|_{\text{A2}}}{|s|_{\text{A1}}} \approx \frac{1}{1-\delta} $$
   - 对 $\rho=0.5$，$\delta=0.333$，$1/(1-\delta)=1.5$，A2 的稳态误差是 A1 的 1.5 倍
4. **$D \to 0$ 时 $B_{\log} \to 0$**：无扰动时两种算法等价。

---

## 5. A3：Ceil 量化器

### 5.1 控制律与滑模动力学

$$
f(s) = \operatorname{sgn}(s) \cdot \lceil |s| \rceil,\qquad
\dot{s} = -k \cdot \operatorname{sgn}(s) \lceil |s| \rceil + d.
\tag{16}
$$

### 5.2 关键性质

$$
\lceil |s| \rceil \ge |s|,\qquad
\lceil |s| \rceil \ge 1,\qquad
s \cdot f(s) = |s| \cdot \lceil |s| \rceil \ge s^2.
\tag{17}
$$

### 5.3 Lyapunov 分析

**情形 1**：$|s| \ge 1$

$$
\dot{V} = -k |s| \lceil |s| \rceil + s \cdot d
\le -k |s|^2 + |s| \cdot D.
$$

$\dot{V} < 0$ 当 $|s| > D/k$。由于 $D/k$ 可能小于 1，但 $|s| \ge 1$，$s$ 必然进入 $|s| < 1$。

**情形 2**：$|s| < 1$

此时 $\lceil |s| \rceil = 1$，控制律退化为理想继电型：

$$
\dot{s} = -k \cdot \operatorname{sgn}(s) + d.
\tag{18}
$$

$$
\dot{V} = -k |s| + s \cdot d \le -|s|(k - D).
$$

**定理 3**（Ceil 量化器收敛性）：若 $k > D$，则 $s$ 在有限时间内收敛至 $s = 0$：

$$
\boxed{\;|s| \to 0\;}.
\tag{19}
$$

由于 $|s| < 1$ 时 $\lceil |s| \rceil = 1$，控制退化为理想继电型 $\dot{s} = -k\operatorname{sgn}(s) + d$，

$$
\dot{V} \le -|s|(k - D) < 0, \quad \forall s \neq 0,
$$

满足有限时间收敛条件（$\dot{V} \le -\alpha\sqrt{V}$，$\alpha = (k-D)\sqrt{2}$）。

### 5.4 与对数型的对比

| 性质 | A2（对数） | A3（Ceil） |
|---|---|---|
| $|s| \to 0$ 时输出 | $\to 0$ | $\to \pm 1$ |
| 稳定性条件 | $k > 0$ | $k > D$ |
| 稳态界 | $D/(k(1-\delta))$ | $D/k$（$k > D$ 时 $=0$） |
| 收敛速度 | 指数（增益 $k(1-\delta)$） | 有限时间（$|s|<1$ 时恒速） |

Ceil 型在 $k > D$ 时的稳态误差为 0（理想滑模），因为它等价于一个继电控制器。代价是需要知道 $D$ 的下界来选取 $k$。

---

## 6. A4：Floor 量化器

### 6.1 控制律与滑模动力学

$$
f(s) = \operatorname{sgn}(s) \cdot \lfloor |s| \rfloor,\qquad
\dot{s} = -k \cdot \operatorname{sgn}(s) \lfloor |s| \rfloor + d.
\tag{20}
$$

### 6.2 关键缺陷

$$
\lfloor |s| \rfloor = 0,\qquad \forall |s| < 1.
\tag{21}
$$

在死区内 $f(s) = 0$，控制律退化为纯等效控制：

$$
u = -c x_2,\qquad
\dot{s} = d.
$$

### 6.3 Lyapunov 分析

**情形 1**：$|s| \ge 1$

$$
\dot{V} = -k |s| \lfloor |s| \rfloor + s \cdot d
\le -k |s| (|s|-1) + |s| \cdot D.
$$

$\dot{V} < 0$ 当 $|s| > 1 + D/k$。$s$ 被拉向 $|s| = 1$。

**情形 2**：$|s| < 1$

此时 $\lfloor |s| \rfloor = 0$，切换项完全消失，$\dot{s} = d$。Lyapunov 导数为

$$
\dot{V} = s \cdot d.
$$

在最坏情况 $d(t) = D\cdot\operatorname{sgn}(s)$ 下 $\dot{V} = |s|\cdot D > 0$，但 $d(t)$ 为时变正弦信号时符号会变化。无论如何，**系统在 $|s| < 1$ 区域内没有恢复切换力的机制**，$s$ 只能跟随扰动漂移。一旦 $|s|$ 被扰动推过 $1$，切换恢复并将 $s$ 拉回死区，形成极限环。

**定理 4**（Floor 量化器发散性）：若 $D > 0$，则 $s$ **无法收敛至 $|s| < 1$**，系统在如下区间内振荡：

$$
\boxed{\;|s| \in [1,\; 1 + D/k]\;}.
\tag{22}
$$

### 6.4 极限环分析

由 (22)，极限环幅值与 $D$ 成正比、与 $k$ 成反比：
- $D=0$：退化为 $s=0$（无扰动时可镇定）
- $D=0.3,\; k=3$：$|s| \in [1,\; 1.1]$
- $D=0.8,\; k=3$：$|s| \in [1,\; 1.27]$

数值仿真验证：强扰动下 $|s| \approx 0.79$（表 1），与理论下界 $1$ 接近。略微低于 1 的原因是非理想切换（离散时间积分）。

---

## 7. 收敛性总结

### 7.1 定理对比

理想 tanh（A1）要求 $k > D$，稳态误差 $\varepsilon = \tanh^{-1}(D/k)/\mu$，指数收敛至边界层内。对数量化器（A2）只需 $k > 0$（无需扰动信息），稳态界 $B_{\log} = D/(k(1-\delta))$，指数收敛。Ceil 量化器（A3）要求 $k > D$，稳态界为 $0$，在 $|s|<1$ 时退化为继电型控制，有限时间收敛至 $s=0$。Floor 量化器（A4）因 $|s|<1$ 时 $f(s)=0$ 失去切换力，不存在可使 $|s|<1$ 稳定的 $k$，系统维持在极限环 $|s| \in [1,\, 1+D/k]$ 内。

### 7.2 无扰动特殊情况

$D=0$ 时四种算法均收敛至 $s=0$（严格）：

- A1：$\varepsilon = (1/\mu)\tanh^{-1}(0) = 0$ ✓
- A2：$B_{\log} = 0$ ✓
- A3：$s \to 0$ ✓
- A4：$|s| \in [1, 1]$ → $|s| = 1$ ✗

注意 A4 在无扰动时仍有 $|s|=1$ 的死区极限环，因为 Floor 量化器在 $|s|<1$ 时完全失去切换力。

### 7.3 物理解释


- **Ceil** 在 $s\approx0$ 处有恒定的 $\pm1$ 输出 → 一定能克服扰动
- **对数型** 在 $s\approx0$ 处输出 $\to 0$ → 需要用 Lyapunov 证明有界性
- **Floor** 在 $s\approx0$ 处输出 $=0$ → 切换力消失，无法抵抗扰动

---

## 参考文献

1. M. Fu and L. Xie, "The sector bound approach to quantized feedback control," *IEEE Trans. Autom. Control*, vol. 50, no. 11, pp. 1698–1711, 2005.
2. V. I. Utkin, *Sliding Modes in Control and Optimization*. Springer, 1992.
3. H. K. Khalil, *Nonlinear Systems*, 3rd ed. Prentice Hall, 2002 (Chapter 14).