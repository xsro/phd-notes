# 预设性能控制（Prescribed Performance Control, PPC）

## 1. 什么是预设性能控制

预设性能控制（Bechlioulis & Rovithakis, 2008）的核心思想是：**在控制器设计之前，就由用户显式地规定跟踪误差的动态性能边界**，并保证误差在整个时间轴上都落在该边界之内。具体地，对于误差信号 $e(t)$，要求对一切 $t \ge 0$ 满足

$$
|e(t)| \le \rho(t)
$$

其中 $\rho(t) > 0$ 是一个**性能函数（performance function）**，由设计者指定，具备以下特性：

1. $\rho(t)$ 单调递减、光滑、严格正；
2. $\rho(0)$ 大于误差初值的绝对值，即 $\rho(0) > |e(0)|$；
3. $\displaystyle \lim_{t\to\infty} \rho(t) = \rho_\infty > 0$（稳态误差上界）；
4. 收敛速率由 $\rho(t)$ 的衰减率决定。

因此，预设性能同时约束了三件事：

- **超调量**：$|e(t)| < \rho(0)$，即瞬态不越过初始边界；
- **收敛速度**：误差包络以 $\rho(t)$ 的速率衰减；
- **稳态精度**：$|e(t)| < \rho_\infty$（若要渐近收敛到 0，可取 $\rho_\infty = 0$）。

## 2. 性能函数

常用指数型性能函数：

$$
\rho(t) = (\rho_0 - \rho_\infty)\,e^{-\ell t} + \rho_\infty
$$

参数含义：

| 参数 | 含义 |
|------|------|
| $\rho_0$ | 初始边界，须满足 $\rho_0 > |e(0)|$ |
| $\rho_\infty$ | 稳态误差上界（$\rho_\infty = 0$ 时保证渐近收敛） |
| $\ell > 0$ | 衰减速率，越大收敛越快 |

## 3. 误差变换：把“有约束”变为“无约束”

直接让 $|e| \le \rho(t)$ 是一个时变约束，难以直接用于 Lyapunov 设计。PPC 的关键技巧是引入一个光滑、严格单调的变换函数 $T(\cdot)$：

$$
T(\xi) = \tanh \xi \in (-1, 1), \qquad T(0)=0
$$

令归一化误差 $z(t) = e(t)/\rho(t)$，并定义

$$
z(t) = T\big(\xi(t)\big) \quad \Longleftrightarrow \quad \xi(t) = T^{-1}\big(z(t)\big) = \operatorname{artanh}\!\left(\frac{e(t)}{\rho(t)}\right) = \frac{1}{2}\ln\frac{1 + e/\rho}{1 - e/\rho}
$$

**重要性质**：只要 $\xi(t)$ 始终保持有限，就有 $|z(t)| = |T(\xi)| < 1$，从而 $|e(t)| < \rho(t)$ 自动成立。于是问题转化为：**设计一个控制器，使 $\xi(t)$ 不发散**（通常令 $\xi \to 0$）。

对 $\xi$ 求导（记 $\zeta = d\xi/dz = 1/(1-z^2) > 0$）：

$$
\dot{\xi} = \zeta\,\dot{z} = \frac{1}{1-z^2}\cdot\frac{1}{\rho}\big(\dot{e} - z\,\dot{\rho}\big)
$$

## 4. 示例：双积分系统镇定

考虑双积分器

$$
\dot{x}_1 = x_2, \qquad \dot{x}_2 = u
$$

目标：将状态 $(x_1, x_2)$ 镇定到原点，并对位置误差 $e = x_1$ 施加预设性能约束。

### 4.1 设定性能与变换

1. 选取性能函数 $\rho(t) = (\rho_0 - \rho_\infty)e^{-\ell t} + \rho_\infty$，满足 $\rho_0 > |x_1(0)|$。
2. 定义 $z = x_1/\rho$，$\xi = \operatorname{artanh}(z)$，$\zeta = 1/(1-z^2)$。
3. 求 $\xi$ 的演化：

$$
\dot{\xi} = \frac{1}{\rho(1-z^2)}\big(x_2 - z\,\dot{\rho}\big)
$$

### 4.2 反步（backstepping）设计

**第一步（虚拟控制）：** 希望 $\dot{\xi} = -c_1\xi$。令 $x_2$ 跟踪虚拟控制量

$$
\alpha = \rho(1-z^2)(-c_1\xi) + z\,\dot{\rho}, \qquad c_1 > 0
$$

当 $x_2 = \alpha$ 时，恰好有 $\dot{\xi} = -c_1\xi$。

**第二步（实际控制）：** 定义虚拟控制误差

$$
s = x_2 - \alpha
$$

则 $x_2 = \alpha + s$，代入 $\dot{\xi}$ 得

$$
\dot{\xi} = -c_1\xi + \frac{\zeta}{\rho}\,s
$$

取 Lyapunov 函数 $V = \tfrac12 \xi^2 + \tfrac12 s^2$，其导数为

$$
\dot{V} = \xi\dot{\xi} + s\dot{s}
      = -c_1\xi^2 + \frac{\xi\zeta}{\rho}s + s\big(u - \dot{\alpha}\big)
$$

为消去交叉项并使 $\dot{V} \le 0$，选取实际控制律

$$
\boxed{\,u = \dot{\alpha} - c_2 s - \frac{\xi\,\zeta}{\rho}, \qquad c_2 > 0\,}
$$

其中 $\zeta = 1/(1-z^2)$，$\dot{\alpha}$ 为 $\alpha$ 对时间的全导数（因 $\rho,\dot{\rho},\ddot{\rho}$ 均为已知设计函数，$\alpha$ 仅依赖于 $x_1$ 与这些已知函数，故 $\dot{\alpha}$ 可显式计算）。

### 4.3 稳定性

代入控制律后

$$
\dot{V} = -c_1\xi^2 - c_2 s^2 \;\le\; 0
$$

故 $V(t)$ 有界，$\xi, s \in \mathcal{L}_\infty$，进而 $x_1, x_2$ 有界。由Barbălat 引理及 $\dot{V}$ 一致连续，可得 $\xi \to 0,\; s \to 0$，于是 $x_2 \to \alpha \to 0$ 且 $z = \tanh\xi \to 0$，即

$$
\lim_{t\to\infty} x_1(t) = 0, \qquad |x_1(t)| < \rho(t)\ \forall t\ge 0
$$

**结论**：系统在保证位置误差始终落在预设包络 $\rho(t)$ 内的同时，实现了原点镇定。

## 5. 设计要点与参数选择

- **$\rho_0$ 必须大于 $|x_1(0)|$**，否则初始时刻 $\xi(0)$ 无定义（分母 $1-z^2=0$）。这是 PPC 唯一的“初值可解性”约束。
- **$\rho_\infty = 0$** 可保证 $x_1$ 渐近收敛到原点；若取 $\rho_\infty > 0$，则稳态有界 $|x_1| \le \rho_\infty$。
- **$\ell$ 越大**，包络衰减越快，但控制量幅值通常越大；$\ell, c_1, c_2$ 需结合执行器饱和折中。
- 反步法中的 $\dot{\alpha}$ 会引入 $\rho$ 的二阶导数 $\ddot{\rho}$，故性能函数需足够光滑（指数型满足）。

## 6. 拓展：从“预设性能”到“预设时间” PPC

前面的设计保证 $|e(t)|<\rho(t)$ 并**渐近**收敛（$t\to\infty$）。若要求误差在**用户指定的有限时刻 $T$** 之前就进入并保持在性能包络内（甚至精确归零），即“预设时间（prescribed-time）性能控制”，需要沿两条主线改造：

1. **性能函数改为有限时间到达型**：使 $\rho(t)$ 在 $t=T$ 时恰好达到目标值（如 $\bar\rho$ 或 0）；
2. **控制器具备预设时间收敛能力**：使变换误差 $\xi(t)$ 在 $t=T$ 时精确归零。

由于 $e(t)=\rho(t)\tanh\xi(t)$，只要 $\xi(T)=0$，就有 $e(T)=0$ 而与 $\rho(T)$ 无关——这是把 PPC 推向预设时间的关键观察。

### 6.1 有限时间性能函数

$$
\rho(t)=
\begin{cases}
\bar\rho + (\rho_0-\bar\rho)\left(1-\dfrac{t}{T}\right)^r, & 0\le t < T\\[6pt]
\bar\rho, & t \ge T
\end{cases}
\qquad (r\ge 2,\ \bar\rho \ge 0)
$$

- $r\ge 2$ 保证 $\rho(t)$ 在 $t=T$ 处至少 $C^1$，使 $\dot\rho,\ddot\rho$ 在 $T$ 附近有界；
- $\bar\rho>0$ 为小的稳态残差（取 $\bar\rho=0$ 可获精确归零，但会在 $t\to T^-$ 时引入 $1/\rho$ 项，见 6.4）；
- 仍须满足 $\rho_0>|e(0)|$。

### 6.2 预设时间虚拟控制

标量系统

$$
\dot\xi = -a(t)\,\xi, \qquad a(t)=\frac{\pi}{2T}\sec^2\!\left(\frac{\pi t}{2T}\right)
$$

的解为

$$
\xi(t)=\xi(0)\cos\!\left(\frac{\pi t}{2T}\right)
$$

显然 $\xi(T)=0$、$\xi(t)\equiv 0\ (t\ge T)$，即 $\xi$ 在**精确时刻 $T$** 收敛。令虚拟控制 $\alpha$ 使 $\dot\xi$ 跟踪该目标：

$$
\alpha = \rho(1-z^2)\big(-a(t)\,\xi\big) + z\,\dot\rho, \qquad a(t)=\frac{\pi}{2T}\sec^2\!\left(\frac{\pi t}{2T}\right)
$$

当 $x_2=\alpha$ 时即得 $\dot\xi=-a(t)\xi$。

### 6.3 反步闭环（与 4.2 同构）

仍定义 $s=x_2-\alpha$，则

$$
\dot\xi = -a(t)\,\xi + \frac{\zeta}{\rho}\,s, \qquad \zeta=\frac{1}{1-z^2}
$$

取 Lyapunov 函数 $V=\tfrac12\xi^2+\tfrac12 s^2$，沿用第 4 节的控制律形式

$$
u = \dot\alpha - c_2 s - \frac{\xi\,\zeta}{\rho}, \qquad c_2>0
$$

可得

$$
\dot V = -a(t)\,\xi^2 - c_2 s^2 \le 0
$$

且 $-a(t)\xi^2 = -\dfrac{\pi}{2T}\xi_0^2$ 在 $t\to T^-$ 时保持有界。于是 $\xi\to 0,\ s\to 0$，从而在 $t=T$ 时

$$
e(T)=\rho(T)\tanh\xi(T)=0
$$

并对 $t\ge T$ 维持 $e(t)\equiv 0$。**即系统在用户指定的时刻 $T$ 精确镇定，且全程满足 $|e(t)|<\rho(t)$。**

### 6.4 设计要点与代价

- **增益发散是本质代价**：为在有限时间强制收敛，虚拟增益 $a(t)\sim\sec^2(\pi t/2T)$ 与 $\dot\alpha$ 中的 $a(t)\xi$ 项在 $t\to T^-$ 时趋于无穷。这正是预设时间控制的标志，要求执行器不饱和；若饱和，需配合抗饱和或放宽 $T$。
- **$\bar\rho=0$ 的额外奇异性**：性能函数若在 $T$ 处归零，则控制律中的 $\xi\zeta/\rho=\xi/(\rho(1-z^2))$ 出现发散。实践中取 $\bar\rho>0$ 极小即可兼顾“近似精确归零”与有界控制，且稳态残差被 $\bar\rho$ 限定。
- **与有限时间/固定时间 PPC 的辨析**：若改用 $\dot\xi=-k_1\xi-k_2\operatorname{sgn}(\xi)|\xi|^\alpha$ 等有限时间/固定时间律，收敛时间依赖初值（或仅已知上界），不再是“用户精确指定 $T$”。预设时间控制用显式时变增益换取收敛时刻的精确可设。

### 6.5 另一条路线：时间重标定（Time-Base Generator, TBG）

令时间映射 $\tau = t/(T-t)$ 把 $[0,T)$ 映到 $[0,\infty)$。在重标定时间 $\tau$ 下，把第 4 节的**标准（渐近）PPC 控制器**中的 $t$ 替换为 $\tau$，使系统相对 $\tau$ 渐近收敛；由于 $\tau\to\infty\Leftrightarrow t\to T^-$，原时间下即实现预设时间收敛。该路线控制增益含 $(T-t)^{-2}$ 因子，同样在 $t\to T$ 时发散。相比 6.2 的显式余弦构造，TBG 更易与现成 PPC 控制器对接，但增益增长更快。

## 7. 与“Funnel / Fencing”思路的关联

PPC 用一条随时间收缩的性能包络 $\rho(t)$ 来约束误差；许多“fencing / funnel”类方法（如 funnel control）思路相近，只是用漏斗边界（funnel boundary）来界定允许的跟踪误差范围。两者都把“瞬态 + 稳态性能”显式前置到设计阶段，区别主要在约束形式与处理手段（变换函数 vs. 障碍/漏斗李雅普诺夫函数）。本目录后续可在此基础上扩展至存在匹配/非匹配不确定性的鲁棒与自适应预设性能控制。

## 参考资料

- Bechlioulis, C. P., & Rovithakis, G. A. (2008). *Robust adaptive control of feedback linearizable MIMO nonlinear systems with prescribed performance*. IEEE Transactions on Automatic Control, 53(9), 2090–2099.
- Bechlioulis, C. P., & Rovithakis, G. A. (2014). *A low-complexity global approximation-free control scheme with prescribed performance for unknown pure feedback systems*. Automatica, 50(4), 1217–1226.
