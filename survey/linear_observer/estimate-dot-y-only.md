# 只估计 $\dot y$，不需要估计 $x$：问题、条件与方法

## 问题设定

自治（无输入）线性系统：

$$
\begin{cases}
\dot x = A x \\
y = C x
\end{cases}
$$

- $x \in \mathbb{R}^n$：未知
- $A \in \mathbb{R}^{n \times n}$：未知
- $C \in \mathbb{R}^{p \times n}$：未知
- **仅已知**：输出 $y(t) \in \mathbb{R}^p$
- **目标**：估计 $\dot y(t)$，**不关心 $x$ 是什么**

---

## 一、根本结论：$\dot y$ 由 $y$ 唯一确定，不依赖 $A, C$ 的知识

这是整个问题最核心的洞察，而且几乎是一个"定义层面"的事实：

$$
\dot y(t) = \frac{d}{dt} y(t)
$$

**导数由函数本身定义**。因此，只要掌握了输出信号 $y(t)$（作为时间的连续函数），$\dot y(t)$ 在数学上就是唯一确定的——完全不需要 $A$、$C$、$x(0)$ 的任何知识。

这里没有"歧义"的原因在于：$y(t) = C e^{At}x(0)$ 是解析函数（指数函数的有限线性组合），而解析函数由其自身唯一确定，包括它的各阶导数。换言之，如果有两组不同的参数 $(A_1,C_1,x_1(0))$ 和 $(A_2,C_2,x_2(0))$ 产生了**完全相同**的输出 $y(t)$，那么它们的导数也必然相同——因为它们本质上是同一个函数。

> **与估计 $x$ 的本质区别**：从 $y$ 反推 $x$ 时，相似等价类导致 $x$ 不唯一（$x$ 不可辨识）；但从 $y$ 求 $\dot y$ 时，**不存在这个歧义**——$\dot y$ 就是 $y$ 的导数，只要 $y$ 已知，$\dot y$ 就确定了。

---

## 二、$\dot y$ 的表达式与物理意义

$$
\dot y = C \dot x = C A x
$$

- $\dot y$ 是状态 $x$ 的线性组合，组合系数为 $CA$
- $\dot y \in \mathbb{R}^p$，与 $y$ 同维
- 物理上，$\dot y$ 是输出信号的变化率（变化趋势）

从信号处理角度看：

$$
y(t) = \sum_{i=1}^{n} c_i e^{\lambda_i t} \quad\Longrightarrow\quad \dot y(t) = \sum_{i=1}^{n} \lambda_i c_i e^{\lambda_i t}
$$

其中 $\lambda_i$ 是 $A$ 的特征值，$c_i$ 是与 $x(0)$ 和 $C$ 相关的系数。**$\dot y$ 就是 $y$ 的导数**——这本质上是一个经典信号处理问题，而不是系统辨识问题。

---

## 三、需要什么条件？

### 3.1 理论层面（完美、无噪声情形）

| 条件 | 说明 | 是否必需 |
|---|---|---|
| $y(t)$ 可测 | 显然必需 | ✅ |
| $y(t)$ 可导 | 自治线性系统的输出天然解析，自动满足 | ✅ 自动成立 |
| $y(t)$ 不恒为零 | 若 $y(t) \equiv 0$，则 $\dot y \equiv 0$，无需估计 | ⚠️ 平凡情形 |

> **理论结论**：在无噪声情况下，**不需要任何关于 $A, C$ 的先验知识**，也不需要可观测性、持续激励等条件。只要 $y(t)$ 已知，$\dot y(t)$ 就是确定的。

### 3.2 工程层面（实际有噪声情形）

实际中 $y(t)$ 含测量噪声 $v(t)$：

$$
\tilde y(t) = y(t) + v(t)
$$

此时估计 $\dot y$ 的难点在于：**微分操作放大高频噪声**。

$$
\mathcal{F}\{\dot y\}(s) = s \cdot \mathcal{F}\{y\}(s)
$$

幅频特性 $|s| = \omega$，高频段噪声被线性放大。

**工程上需要的条件**：

| 条件 | 说明 |
|---|---|
| **足够的信噪比** | 噪声功率谱在信号频带内应远小于信号功率谱 |
| **信号频带已知或可估计** | 微分器需要截止频率，需要知道信号的最高有效频率 |
| **足够的采样频率** | 采样率至少为信号最高频率的 2 倍以上（Nyquist），实际建议 5–10 倍 |
| **$y(t)$ 非零且非平稳** | 若 $y(t) \equiv \text{const}$，则 $\dot y = 0$，平凡；若 $y(t)$ 趋于常数（稳定系统），后期 $\dot y \to 0$，估计变难 |
| **信号充分激发** | $y(t)$ 的频谱不能太窄，否则微分器难以区分信号变化和噪声 |

---

## 四、实用估计方法

所有方法的共同点：**都不需要 $A, C$ 的知识**，本质上都是信号处理手段。

### 方法 1：线性跟踪微分器（Linear Tracking Differentiator, TD）

由韩京清提出，经典形式：

$$
\begin{cases}
\dot v_1 = v_2 \\
\dot v_2 = -R \cdot \mathrm{sign}\!\left(v_1 - y + \frac{v_2|v_2|}{2R}\right)
\end{cases}
$$

- $v_1$ 跟踪 $y(t)$，$v_2$ 估计 $\dot y(t)$
- 参数 $R$ 控制跟踪速度：$R$ 越大，跟踪越快但噪声放大越严重
- **优点**：不需要 $A, C$ 的任何知识，纯信号处理
- **缺点**：需要调节 $R$

### 方法 2：高增益观测器（High-Gain Observer）

将 $\dot y$ 视为新状态，构造：

$$
\begin{cases}
\dot{\hat y} = \hat z - \beta_1 (\hat y - y) \\
\dot{\hat z} = -\beta_2 (\hat y - y)
\end{cases}
$$

或更一般地，对 $y$ 建立 $n_y$ 阶积分器模型，用高增益线性观测器：

$$
\begin{aligned}
\dot{\hat y} &= \hat y_1 + \beta_1 (y - \hat y) \\
\dot{\hat y}_1 &= \hat y_2 + \beta_2 (y - \hat y) \\
&\ \vdots \\
\dot{\hat y}_{n_y-1} &= \hat y_{n_y} + \beta_{n_y} (y - \hat y) \\
\dot{\hat y}_{n_y} &= \beta_{n_y+1} (y - \hat y)
\end{aligned}
$$

其中 $\hat y$ 是 $y$ 的估计，$\hat y_1$ 是 $\dot y$ 的估计，依此类推。

- **优点**：结构简单，理论成熟（只要增益足够大，估计误差指数收敛）
- **缺点**：高增益放大噪声（Peaking 现象）

### 方法 3：滑模微分器（Sliding Mode Differentiator）

由 Levant 提出，二阶滑模（super-twisting）形式：

$$
\begin{cases}
\dot v_1 = v_2 - k_1 |v_1 - y|^{1/2} \mathrm{sign}(v_1 - y) \\
\dot v_2 = -k_2 \mathrm{sign}(v_1 - y)
\end{cases}
$$

- $v_1$ 跟踪 $y$，$v_2$ 估计 $\dot y$
- **优点**：有限时间收敛，对噪声有一定鲁棒性
- **缺点**：存在抖振（chattering），需用边界层技术平滑

### 方法 4：卡尔曼滤波微分（Kalman Filter Differentiator）

将 $y$ 建模为随机过程（如 Brownian motion 驱动的积分器链），用 KF 同时估计 $y$ 和 $\dot y$：

状态向量 $\hat x_{\mathrm{diff}} = [y, \dot y]^\mathrm{T}$（对每个输出分量），过程模型：

$$
\frac{d}{dt}\begin{bmatrix} y \\ \dot y \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \begin{bmatrix} y \\ \dot y \end{bmatrix} + \begin{bmatrix} 0 \\ 1 \end{bmatrix} w(t)
$$

测量：$z = y + v(t)$

- **优点**：最优估计框架，噪声处理系统化
- **缺点**：需要噪声协方差先验，模型阶数选择影响性能

### 方法 5：频域方法（低通滤波 + 微分）

$$
\hat{\dot y}(t) = \mathcal{F}^{-1}\left\{ s \cdot H_{\mathrm{LP}}(s) \cdot \mathcal{F}\{\tilde y(t)\} \right\}
$$

其中 $H_{\mathrm{LP}}(s)$ 是低通滤波器，截止频率根据信号频带和噪声水平选择。

常用实现：Butterworth 滤波器 + 数值微分，或直接使用一阶惯性环节近似微分：

$$
\hat{\dot y}(s) = \frac{s}{\tau s + 1} \tilde y(s)
$$

- $\tau$ 越小，微分越精确但噪声放大越严重
- $\tau$ 越大，噪声抑制越好但相位滞后越大

---

## 五、与估计 $x$ 的对比

| 维度 | 估计 $x$ | 估计 $\dot y$ |
|---|---|---|
| 需要 $A, C$ 先验？ | ✅ 需要（或在线辨识） | ❌ 完全不需要 |
| 可辨识性问题 | 相似等价类，$x$ 不唯一 | 无歧义，$\dot y$ 由 $y$ 唯一确定 |
| 可观测性要求 | 必需 | 不需要（$\dot y$ 是 $y$ 的导数） |
| 持续激励要求 | 在线方法必需 | 不需要（但信号不能为零） |
| 噪声敏感度 | 中等 | **高**（微分放大噪声） |
| 方法复杂度 | 高（观测器/滤波器设计） | **低**（经典信号处理） |
| 适用场景 | 需要状态反馈控制、状态可视化 | 需要变化率信号用于监控、预警、前馈 |

---

## 六、关键澄清

### 6.1 $\dot y$ 的估计不需要 $A, C$，但这不意味着 $A, C$ 对问题没有影响

$A$ 的特征值决定 $y(t)$ 的衰减/振荡速度，间接影响 $\dot y$ 的估计难度：

- 若 $A$ 稳定，$y(t) \to 0$，$\dot y(t) \to 0$，后期信噪比恶化
- 若 $A$ 有纯虚特征值，$y(t)$ 持续振荡，$\dot y$ 估计相对容易
- 若 $A$ 不稳定，$y(t)$ 发散，实际中不可行

### 6.2 估计 $\dot y$ 不等于绕过可辨识性问题

如果最终目标是**基于 $\dot y$ 的值来做决策或控制**，那么 $\dot y$ 的估计质量仍然依赖于信号质量。而信号质量本身受 $A, C$ 和初始条件影响。

### 6.3 如果 $y$ 是离散采样的

离散采样下，$\dot y$ 的估计变为差分：

$$
\dot y_k \approx \frac{y_{k+1} - y_k}{\Delta t}
$$

但前向差分噪声放大因子为 $1/\Delta t$。更稳健的做法：

- 中心差分：$\dot y_k \approx \frac{y_{k+1} - y_{k-1}}{2\Delta t}$（噪声抑制略好）
- 五点差分：更高精度
- **先滤波再差分：推荐做法**

---

## 七、推荐方案

### 首选：跟踪微分器（TD）或滑模微分器

- 无需 $A, C$ 先验知识
- 无需调节复杂的自适应律
- 仅需调节 1–2 个增益参数
- 适合在线实时应用

### 次选：卡尔曼滤波微分器

- 需要噪声统计先验，但可以在线估计
- 在低信噪比下性能优于简单微分器

### 不推荐：直接数值差分

除非采样率极高且噪声极低。

---

## 八、参数选取：控制理论视角

前述方法都需要调节参数。下面从控制理论的角度，给出各方法的参数物理意义、选取原则与具体配方。

### 8.1 线性跟踪微分器（TD）

**参数**：速度因子 $R$（连续形式）；若采用韩京清改进的 fal 型 TD，还包括非线性指数 $\alpha$ 与线性段宽度 $\delta$。

**物理意义**：TD 的连续形式本质上是双积分器在时间最优控制下的闭环系统，其切换曲线为 $v_2^2 = 2R|v_1 - y|$。$R$ 是允许的最大控制加速度。

**选取规则**：
1. 设信号 $y(t)$ 的最大变化率估计为 $c = \max|\dot y|$（可由幅值 $A$ 与带宽 $\omega_s$ 粗估为 $c \approx A\omega_s$）。
2. TD 对斜坡输入 $y = ct$ 存在稳态跟踪滞后：
   $$
   v_1 - y = -\frac{c|c|}{2R}, \qquad |e_{ss}| = \frac{c^2}{2R}
   $$
   若要求滞后不超过 $\varepsilon_{\mathrm{lag}}$，则需
   $$
   R \ge \frac{c^2}{2\varepsilon_{\mathrm{lag}}}
   $$
3. 在线调试：先按上式取 $R$，再根据噪声水平下调（$R$ 越小，噪声放大越轻，但跟踪越慢）。

**fal 型 TD 的补充**（实际离散实现常用）：
$$
\dot v_1 = v_2,\qquad \dot v_2 = -R\,\mathrm{fal}(v_1 - y,\ \alpha,\ \delta)
$$
其中
$$
\mathrm{fal}(e,\alpha,\delta) =
\begin{cases}
|e|^{\alpha} \mathrm{sign}(e), & |e| > \delta \\[2pt]
e / \delta^{1-\alpha}, & |e| \le \delta
\end{cases}
$$
- $\alpha \in (0,1)$，通常取 $0.5$；越小非线性越强，但配合 $\delta$ 后近零增益由线性段承接。
- $\delta$ 为线性段宽度，通常取 $\delta \approx$ 采样步长 $h$ 或略大。$\delta$ 小则非线性强但易抖振，$\delta$ 大则平滑但响应慢。

### 8.2 高增益观测器

**参数**：增益 $\beta_1,\dots,\beta_{n_y+1}$，或等价地，观测器带宽 $\omega_o$。

**物理意义**：观测器误差动态的特征多项式为
$$
s^{n_y+1} + \beta_1 s^{n_y} + \cdots + \beta_{n_y+1}
$$
通过配置极点来设计 $\beta_i$。标准做法是将所有极点配置在 $-\omega_o$：
$$
(s+\omega_o)^{n_y+1} = s^{n_y+1} + \binom{n_y+1}{1}\omega_o s^{n_y} + \cdots + \omega_o^{\,n_y+1}
$$
从而
$$
\beta_i = \binom{n_y+1}{i}\,\omega_o^{\,i},\quad i=1,\dots,n_y+1
$$
等价地，用小参数 $\varepsilon = 1/\omega_o$ 表示为 $\beta_i = \alpha_i/\varepsilon^i$，其中 $\alpha_i = \binom{n_y+1}{i}$。

**噪声放大**：以 $n_y=1$（估计 $\dot y$）为例，测量噪声 $v$ 到导数估计 $\hat y_1$ 的传递函数为
$$
\frac{\hat y_1(s)}{v(s)} = \frac{\omega_o^2}{s^2 + 2\omega_o s + \omega_o^2}
$$
噪声功率与 $\omega_o$ 成正比（全频积分意义下），且瞬态响应存在 **peaking**：对测量阶跃，$\hat y_1$ 的峰值约为 $\omega_o \cdot |\text{阶跃幅度}|$。$\omega_o$ 越大，peaking 越严重。

**选取规则**：
1. 估计信号带宽 $\omega_s$（由上升时间 $t_r \approx 0.35/\omega_s$，或对 $y$ 做 FFT 得主频）。
2. 取 $\omega_o \approx 2\omega_s \sim 10\omega_s$：
   - 信噪比高 → 取大值（接近 $10\omega_s$），估计快；
   - 信噪比低 → 取小值（接近 $2\omega_s$），抑制噪声。
3. 按极点配置公式算出 $\beta_i$。
4. 若估计的是高阶导数（$n_y \ge 2$），噪声放大随阶数急剧上升，需相应压低 $\omega_o$。

### 8.3 滑模微分器（Super-Twisting）

**参数**：$k_1, k_2$。

**前提假设**：$\dot y$ 满足 Lipschitz 条件，即 $|\ddot y| \le L$（$L$ 为已知或可估计的上界）。

**物理意义**：super-twisting 算法是二阶滑模控制，误差动态在有限时间内收敛到滑模面 $v_1 = y$，随后 $v_2$ 有限时间内收敛到 $\dot y$。

**收敛的充分条件**（Levant）：在 $|\ddot y| \le L$ 下，取
$$
k_2 > L,\qquad k_1 > 0 \text{ 且足够大}
$$
工程上常用的经验充分条件为
$$
k_2 > L,\qquad k_1 \ge 2\sqrt{k_2}\quad (\text{或 } k_1 \ge \sqrt{2(k_2+L)})
$$

**选取规则**：
1. 估计 $L = \max|\ddot y|$：对正弦类信号 $y \approx A\sin(\omega t)$，有 $L \approx A\omega^2$；或由干净数据的二阶差分估计。
2. 取 $k_2 = (1.1 \sim 2)\,L$。
3. 取 $k_1 = 2\sqrt{k_2}$（默认），信噪比低时可适当增大。
4. 抖振抑制：将 $\mathrm{sign}(v_1-y)$ 替换为饱和函数
   $$
   \mathrm{sat}_{\delta}(v_1-y) = \begin{cases}
   \mathrm{sign}(v_1-y), & |v_1-y| > \delta \\
   (v_1-y)/\delta, & |v_1-y| \le \delta
   \end{cases}
   $$
   或直接用 $|v_1-y|^{1/2}\mathrm{sign}(v_1-y)$ 的平滑变体。$\delta$ 取小值（如采样步长量级）。

### 8.4 卡尔曼滤波微分器

**参数**：过程噪声强度 $q$（$w \sim$ 白噪声，$\mathbb{E}[w^2]=q$）与测量噪声方差 $r$。

**物理意义**：KF 在"模型信任度"与"测量信任度"之间做最优折衷。比值 $q/r$ 决定滤波器带宽。

模型：
$$
\frac{d}{dt}\begin{bmatrix} y \\ \dot y \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}\begin{bmatrix} y \\ \dot y \end{bmatrix} + \begin{bmatrix} 0 \\ 1 \end{bmatrix} w,\quad z = y + v
$$
对于该双积分器模型，稳态 Kalman 增益具有如下量级关系：
$$
K_y \approx 2\omega_f,\qquad K_{\dot y} \approx \omega_f^2
$$
其中 $\omega_f$ 为滤波器等效带宽，满足
$$
\omega_f \sim \left(\frac{q}{r}\right)^{1/4}
$$

**选取规则**：
1. 离线估计测量噪声方差 $r$（可在静止/已知恒定输出时采集一段数据计算样本方差）。
2. 根据期望带宽 $\omega_f$ 反推 $q = r\,\omega_f^4$。
   - $\omega_f$ 的选取同高增益观测器：$\omega_f \approx 2\omega_s \sim 10\omega_s$，信噪比高取大、低取小。
3. 在线自适应：若 $r$ 未知，可用 Innovation 自适应估计（如 Sage-Husa 自适应滤波或基于残差的 $r$ 在线估计），$q$ 则通过调节 $\omega_f$ 的 pilot 模型来整定。
4. 离散实现时，用离散化后的模型跑标准 KF；稳态下可预计算增益以节省算力。

### 8.5 频域方法（低通滤波 + 微分）

**参数**：低通滤波器截止频率 $\omega_c = 1/\tau$（一阶惯性环节 $\frac{1}{\tau s+1}$），或 Butterworth 滤波器的阶数 $n$ 与截止频率 $\omega_c$。

**物理意义**：微分器 $H_d(s) = \frac{s}{\tau s+1}$ 是一个高通滤波器，幅频 $|H_d(j\omega)| = \frac{\omega}{\sqrt{1+(\omega\tau)^2}}$，相频 $\angle H_d = 90^\circ - \arctan(\omega\tau)$。
- 低频段（$\omega \ll 1/\tau$）：近似理想微分，幅值 $\approx \omega$，相位 $\approx 90^\circ$。
- 高频段（$\omega \gg 1/\tau$）：幅值饱和于 $1/\tau$，噪声被限幅——这正是噪声抑制的来源。

**选取规则**：
1. 设信号有效带宽为 $\omega_s$，噪声显著起作用的频率下限为 $\omega_n$（通常 $\omega_n > \omega_s$）。
2. 理想情况：$\omega_s < \omega_c < \omega_n$。
   - $\omega_c$ 太低（$\tau$ 大）：信号畸变——真 $\dot y$ 的幅值被衰减、相位滞后增大。
   - $\omega_c$ 太高（$\tau$ 小）：噪声几乎无衰减地通过。
3. 经验取法：$\omega_c \approx 3\omega_s \sim 5\omega_s$（在白噪声假设下），再根据实际噪声水平微调。
4. 多阶 Butterworth 低通级联微分（$H_d(s) = s\cdot H_{\mathrm{LP}}(s)$）：
   - 阶数 $n$ 越高，高频噪声抑制越强（衰减斜率 $-(2n-1)$ dB/dec），但过渡带变宽、相位滞后增加。
   - 推荐 $n=2$ 或 $4$，截止频率 $\omega_c$ 同上。

### 8.6 参数选取总览

| 方法 | 核心参数 | 控制理论含义 | 选取主线 |
|---|---|---|---|
| 线性 TD | $R$ | 最大控制加速度 | $R \ge c^2/(2\varepsilon_{\mathrm{lag}})$，$c=\max|\dot y|$ |
| 高增益观测器 | $\omega_o$ | 观测器带宽 | $\omega_o \approx 2\omega_s \sim 10\omega_s$ |
| 滑模微分器 | $k_1,k_2$ | 滑模增益（与 $L=\max|\ddot y|$ 相关） | $k_2 > L$，$k_1 \approx 2\sqrt{k_2}$ |
| 卡尔曼滤波 | $q/r$ | 滤波器带宽 $\omega_f \sim (q/r)^{1/4}$ | $\omega_f \approx 2\omega_s \sim 10\omega_s$ |
| 频域微分 | $\omega_c=1/\tau$ | 微分器截止频率 | $\omega_s < \omega_c < \omega_n$ |

**通用原则**：
- 所有方法的参数本质上都在"**响应速度**"与"**噪声抑制**"之间权衡。
- 信号带宽 $\omega_s$ 是共同的参考基准——它是选取所有参数的起点。
- 信噪比决定参数可取的"激进程度"：高 SNR 可激进（大带宽/大增益），低 SNR 需保守。
- 实际调试时，先用干净信号（或离线估计的 $\omega_s, L$）确定参数初值，再在真实噪声数据上微调。

---

## 九、一句话总结

> **估计 $\dot y$ 比估计 $x$ 简单得多——它只是信号的导数，不需要 $A, C$ 的任何知识，也不受相似等价类歧义的影响。唯一的工程难点是噪声放大，用一个合适的微分器（TD / 滑模 / KF）即可解决。条件只有一个：$y(t)$ 不能是常数。**