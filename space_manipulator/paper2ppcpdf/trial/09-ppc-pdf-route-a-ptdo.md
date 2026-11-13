# 路线 A（二）：PPC + PDF + 预设时间扰动观测器（PTDO）

> 本文件在 `08-ppc-pdf-route-a.md` 的基础上，用**预设时间扰动观测器（PTDO）**取代"\(d(t)\) 已知"这一强假设，
> 使算法只依赖两个可测量：关节角 \(q_m\) 与关节角速度 \(\dot q_m\)。
>
> 三段时序：
>
> | 时段 | 机制 | 结论 |
> |---|---|---|
> | \(0\le t<T_o\) | PTDO 收敛 + 漏斗型 PPC | \(\lvert e_i(t)\rvert<\rho_i(t)\)（**对观测峰值结构免疫**） |
> | \(T_o\le t<T_o+2h\) | PDF（此时 \(\hat d\equiv d\)，通道无扰动） | 单周期零化 |
> | \(t\ge T_o+2h\) | 稳态 | \(e\equiv\dot e\equiv0\)（连续时间理想条件） |
>
> 总预设时间 \(T_{\mathrm{pre}}=T_o+2h\)，与 `sections/05-stability.tex` 的推论 `cor:prescribed_time` 完全一致。
>
> **与文件 08 的关系**：文件 08 是"已知扰动前馈版"，本文件是"扰动在线估计版"。本文件**严格更强**（假设更弱），且**唯一代价**是引入一个必须满足 \(T_c\le T_o\) 的观测器整定约束（见 §7）。

---

## 1. 为什么要加观测器：文件 08 的两处硬伤

| 硬伤 | 文件 08 的表现 | 本文件如何消除 |
|---|---|---|
| ① \(d(t)\) 必须解析已知 | 阶段 II 只能得 \(\Gamma_h\bar\Delta_a\)，且实测 \(\Gamma_h\bar\Delta_a>\vartheta\varepsilon\) 会**放大**误差（文件 08 §8.1） | \(t\ge T_o\) 后 \(\hat d\equiv d\) ⟹ 通道严格无扰动 ⟹ 可解释"归零" |
| ② \(\bar w\) 未知时 \(\vartheta_i\) 不可量化 | \(\vartheta_i\to1\)，无法校核 \(M_h\|z(T_1)\|\le\min\varepsilon_i\) | \(\bar w=\sup_{[0,T_o]}\|\varepsilon_2\|\) 有**显式可算估计**（引理 2），且**只在有限窗口 \([0,T_o]\) 上需要** |

关键结构性事实：**观测器的精确性只在 \(t\ge T_o\) 被需要，而 PPC 的包络保证对任意有限残差都成立**。两者因此可以共存而不互相污染。

---

## 2. 信号清单：到底省掉了什么

| 信号 / 先验 | 文件 08（无观测器） | 本文件 |
|---|---|---|
| 关节角 \(q_m\) | 需要 | 需要（同一） |
| 关节角速度 \(\dot q_m\) | 需要 | 需要（同一） |
| 关节角加速度 \(\ddot q_m\)（加速度计 / 数值微分） | 不需要 | **不需要**（PTDO 只用 \(u_o=M_e^{-1}(\tau-C_e\dot q_m)\)） |
| 集总扰动 \(d(t)\) 的**数值** | **需要（强假设）** | **不需要** |
| \(d(t)\) 的**上界** \(\bar d\) | 不需要（定理层面） | **不需要** |
| \(\Delta_a\) 及其导数上界 | 不需要（定理层面） | 不需要（定理层面） |
| 模型 \(M_e,C_e\) | 需要 | 需要（同一） |
| 力矩 \(\tau\) | 自己算出 | 自己算出（PTDO 用作已知输入） |
| 内部缓存 | 阶段 II 的 \(h\) 秒 \(z\) 历史 | 同（**且不需要 \(T_o\) 之前的历史**） |

**结论**：可测量从"\(\{q,\dot q,d\}\)"降为"\(\{q,\dot q\}\)"；先验从"\(d\) 的解析式"降为"\(d\) 与 \(\dot d\) 有界（数值未知）"，即论文假设 `as:disturbance` 原样成立即可。这就是"减少需要观测的信号"的确切含义。

---

## 3. 系统与假设

沿用文件 08 的模型。本文件用以下假设替换 A3：

**A3′（扰动有界，数值未知）** \(\Delta_a(t)=M_e^{-1}(q_m)d(t)\) 与 \(\dot\Delta_a(t)\) 一致有界，但**上界未知**（= 论文假设 `as:disturbance`）。

**A6（观测器设计条件）** 存在设计参数 \(T_c\le T_o\)、\(\eta\in(0,1)\)、\(\sigma>0\) 使 PTDO 在连续时间下于 \(T_c\) 内精确收敛（这是论文定理 `thm:ptdo` 的结论；本文件按需引用）；并要求 **\(T_c\le T_o\)**。

A1（模型精确）、A2（\(q,\dot q\) 可测）、A4（初始在漏斗内）、A5（理想实现）不变。

---

## 4. 控制结构

### 4.1 力矩律（三段统一的唯一公式）

\[
\boxed{\ \tau(t)=M_e(q_m)\bigl[\ddot q_d(t)+\nu(t)\bigr]+C_e(q_m,\dot q_m)\dot q_m-\hat d(t)\ }
\tag{4.1}
\]

其中

\[
\hat d(t)=M_e(q_m)\,\hat\Delta_a(t)=M_e(q_m)\,z_2(t)
\tag{4.2}
\]

由 PTDO 的内部状态 \(z_2\) 给出，**不需要 \(d\)**。三个时段只有 \(\nu\) 不同：

$$
\nu(t)=
\begin{cases}
\nu^{\mathrm{PPC}}(t), & 0\le t<T_o \quad\text{（文件 08 式 (4.4)，漏斗型 PPC 律，}T_1\to T_o\text{）}\\[2pt]
K_0z(t)-K_c(t-T_o)\,z(t-h), & t\ge T_o\quad\text{（式 6.2，PDF）}
\end{cases}
$$

### 4.2 误差通道

代入动力学：

\[
\ddot q_m=\ddot q_d+\nu+M_e^{-1}(d-\hat d)
\ \Longrightarrow\
\boxed{\ \ddot e_i(t)=\nu_i(t)+w_i(t),\qquad w(t):=M_e^{-1}(q_m)\bigl[d(t)-\hat d(t)\bigr]=\varepsilon_2(t)\ }
\tag{4.3}
\]

由 (4.2) 得 \(M_e^{-1}(d-\hat d)=\Delta_a-z_2=\varepsilon_2\)，即**残差就是 PTDO 的第二误差分量**。这条恒等是整个方案的枢纽：观测器定理直接给出 \(w(t)\equiv0\ (t\ge T_o)\)。

---

## 5. PTDO 与阶段 I

### 5.1 观测器（论文定理 `thm:ptdo` 原样引用）

\[
\begin{cases}
\dot z_1=z_2+\dfrac{\pi}{\eta T_c}\phi_1\!\Bigl(\dfrac{\varepsilon_1}{\sigma}\Bigr)-\dot{\bar\xi}(t)+u_o,\\[6pt]
\dot z_2=\dfrac{\pi}{\sigma\eta T_c}\phi_2\!\Bigl(\dfrac{\varepsilon_1}{\sigma}\Bigr),
\end{cases}
\qquad
\begin{cases}
\phi_1(x)=\mathrm{sig}^{1-\eta/2}(x)+\mathrm{sig}^{1+\eta/2}(x),\\
\phi_2(x)=\mathrm{sig}^{2-\eta}(x)+\mathrm{sig}^{2+\eta}(x)+\sigma\,\mathrm{sgn}(x),
\end{cases}
\tag{5.1}
\]

\(\varepsilon_1=\dot q_m-z_1-\bar\xi(t)\)，\(u_o=M_e^{-1}(\tau-C_e\dot q_m)\)，\(\bar\xi(t)=\bar\xi_0(T_o-t)^2\,\mathbb 1_{t<T_o}\)。

**定理 PTDO（引用，论文定理 `thm:ptdo`）** 在 A3′ 与连续时间下，若 \(T_c\le T_o\)，则
\(\varepsilon_1(t)=0,\ \varepsilon_2(t)=0,\ \forall t\ge T_o\)，从而 \(w(t)\equiv0,\ t\ge T_o\)。

### 5.2 引理 2（阶段 I 的残差有界，显式）

> **引理 2** 设 \(\sup_{[0,T_o]}|\varepsilon_1(t)|\le\sigma\bar x\)（\(\bar x<\infty\)，即不出现无界峰值）。则
> \[
> \bar w:=M_2:=\sup_{0\le t\le T_o}\|\varepsilon_2(t)\|_\infty
> \le 2\|\Delta_a\|_\infty+\|\varepsilon_2(0)\|_\infty
> +\frac{\pi}{\sigma\eta T_c}\Bigl[(1+\bar x)^{2+\eta}+(1+\bar x)^{2-\eta}+\sigma\Bigr]T_o .
> \]

**证明.** 由 (5.1) 第二式积分得 \(z_2(t)=z_2(0)+\dfrac{\pi}{\sigma\eta T_c}\displaystyle\int_0^t\phi_2\!\Bigl(\dfrac{\varepsilon_1(s)}{\sigma}\Bigr)\mathrm ds\)，
代入 \(\varepsilon_2=\Delta_a-z_2\)：

\[
\varepsilon_2(t)=\varepsilon_2(0)+\Delta_a(t)-\Delta_a(0)-\int_0^t\frac{\pi}{\sigma\eta T_c}\phi_2\!\Bigl(\frac{\varepsilon_1(s)}{\sigma}\Bigr)\mathrm ds .
\]

取 \(\ell_\infty\) 范数；因 \(|\varepsilon_1(\cdot)|\le\sigma\bar x\)，对 \(|x|\le\bar x\) 有
\(|\mathrm{sig}^{a}(x)|=|x|^{a}\le(1+\bar x)^{a}\)（\(a>0\)），故
\(|\phi_2(x)|\le(1+\bar x)^{2-\eta}+(1+\bar x)^{2+\eta}+\sigma\)。合并即得。\(\blacksquare\)

> **注 1（\(\bar w\) 只需要在有限窗口上、且只需要一个保守估计）** 引理 2 的估计是保守的（含 \(T_o\) 线性因子与峰值 \(\bar x\)）。但由定理 1（见 §5.3），**包络不破不依赖 \(\bar w\)**；\(\bar w\) 只用于：(i) 计算进带深度 \(\vartheta_i\)；(ii) 校核命题 1 的 \(M_h\|z(T_o)\|\le\min\varepsilon_i\)。故 \(\bar w\) 的粗糙性只影响**验收余量**，不影响正确性。

### 5.3 定理 4（阶段 I：包络保持，对观测峰值结构免疫）

本定理即文件 08 的定理 1，只是把 \(w\) 换成 \(\varepsilon_2\)，作用区间换成 \([0,T_o]\)（控制律即文件 08 式 (4.4)，性能函数即文件 08 式 (4.1) 并把 \(T_1\to T_o\)）。

> **定理 4** 在 A1、A2、A4 与 A3′ 下，取性能函数 (4.1)（把 \(T_1\) 换为 \(T_o\)）与控制律 (5.4)。
> 记 \(c_i=\dfrac{\sigma_i^2}{2k_{2i}}\)，\(\sigma_i=\dfrac{\bar w}{\varepsilon_i}\)，\(\bar w\) 由引理 2 给出。则闭环在 \([0,T_o]\) 上整体存在，且
> \[
> \boxed{\ |e_i(t)|\le\vartheta_i\rho_i(t)<\rho_i(t),\quad
> \vartheta_i=\sqrt{1-e^{-2\max\{V_i(0),\bar V_i\}}}<1,\quad
> \bar V_i=\tfrac12\ln\frac{c_i+k_{1i}}{k_{1i}}+\frac{c_i}{k_{2i}},\quad \forall t\in[0,T_o].\ }
> \]
> 特别地 \(|e_i(T_o)|\le\vartheta_i\varepsilon_i<\varepsilon_i\)。

**证明.** 完全同文件 08 定理 1：\(\theta_i=e_i/\rho_i\) 的动力学为 (4.5)，\(s_i=\dot\theta_i+k_{1i}\theta_i\) 的动力学为 (4.6)（\(w_i\to\varepsilon_{2,i}\)），\(\dot V_i\le-k_{1i}\theta_i^2/(1-\theta_i^2)-\tfrac12k_{2i}s_i^2+c_i\)，无逃逸 + 比较论证即得。唯一补充：定理 4 不要求 \(\varepsilon_2\) 由任何特定机制产生，只要求有限 ⟹ 与定理 PTDO 的收敛性无循环依赖（观测器的 \(u_o\) 只依赖已施加的 \(\tau\)，不依赖 \(\nu\) 的形式）。\(\blacksquare\)

> **推论 4（PPC 的价值所在：与纯 PD 对照）**
> 同样的观测峰值，两种 Phase I 的后果完全不同：
> \[
> \text{纯 PD：}\ \|z(t)\|\le\kappa e^{-\lambda t}\|z(0)\|+\frac{\kappa}{\lambda}\sup_{[0,T_o]}\|\varepsilon_2\|\quad(\text{随峰值线性恶化});\qquad
> \text{PPC：}\ |e_i(t)|<\rho_i(t)\quad(\text{与峰值幅度无关}).
> \]
> 也就是说：**PPC 把"观测器峰值"从"破坏精度的风险"降级为"浪费进带余量的风险"**。这是文件 08 与 `06-ppc-pdf-hybrid.md` 中"两机制正交"论断的定量证据，也是本方案相对论文现有 `prop:phaseI_ISS` 的实质提升。

---

## 6. 阶段 II：精确补偿下的单周期零化

### 6.1 交接状态

由定理 4 与文件 08 的引理 1（把 \(T_1\) 换成 \(T_o\)）：

\[
\|z(T_o)\|\le\Bigl(\sum_i\vartheta_i^2\varepsilon_i^2+\sum_i\varepsilon_i^2\bigl(\sqrt{2V_i^{\max}}+k_{1i}\bigr)^2\Bigr)^{1/2}=:B_o .
\tag{6.1}
\]

### 6.2 PDF 段

以 \(\vartheta=t-T_o\) 为局部时间。由 (4.3) 与定理 PTDO（\(w\equiv0\)）：

\[
\frac{dz}{d\vartheta}=A_cz(\vartheta)-BK_c(\vartheta)z(\vartheta-h),
\qquad
\nu=K_0z-K_c(\vartheta)z(\vartheta-h),\quad K_c\equiv0\ \text{于}\ [0,h).
\tag{6.2}
\]

由文件 08 定理 2（即论文定理 `thm:pdf_gain`）：\(\Delta_K(h)=0\) ⟹ \(z(\vartheta)=0,\ \forall\vartheta\ge2h\)。

### 6.3 定理 5（主定理：双重预设，无需 \(d\) 先验）

> **定理 5** 设 A1、A2、A3′、A4、A5、A6 成立。取控制律 (4.1)(4.2)、观测器 (5.1)、\(\nu\) 按 §4.1 三段切换、\(\rho\) 按 (4.1)（\(T_1=T_o\)）。则
> \[
> \boxed{
> \begin{aligned}
> &\text{(i)}\quad |e_i(t)|<\rho_i(t),\quad \forall t\in[0,T_o],\ i=1,\dots,n;\\
> &\text{(ii)}\quad |e_i(T_o)|\le\vartheta_i\varepsilon_i<\varepsilon_i
> \quad\text{（预设时间 }T_o\text{ 进入预设误差带）};\\
> &\text{(iii)}\quad e_i(t)=\dot e_i(t)=0,\quad \forall t\ge T_o+2h
> \quad\text{（再经预设时间 }2h\text{ 精确归零）}.
> \end{aligned}}
> \]
> 全程只依赖测量 \(\{q_m,\dot q_m\}\) 与模型 \(\{M_e,C_e\}\)；**不需要** \(d(t)\) 的数值、也不需要其任何上界。

**证明.** (i) 定理 4。(ii) 推论（文件 08 推论 1，\(T_1=T_o\)）+ \(\rho_i(T_o)=\varepsilon_i\)。(iii) 交接：\(z(T_o)\) 有界（6.1）；\(t\ge T_o\) 时 \(\hat d\equiv d\)（定理 PTDO）⟹ 误差通道 (4.3) 中 \(w\equiv0\)；PDF 增益满足 \(\Delta_K(h)=0\)（文件 08 定理 2）对任意有界初值给出 \(z(\vartheta)=0,\ \vartheta\ge2h\)。回代时间即得。\(\blacksquare\)

### 6.4 命题 3（阶段 II 的包络校验，与文件 08 命题 1 同）

沿用文件 08 命题 1 的 \(M_h\)：若 \(M_hB_o\le\min_i\varepsilon_i\)，则 \(|e_i(t)|\le\varepsilon_i,\ \forall t\ge0\)（全程包络）；否则
\(|e_i(t)|\le M_hB_o\)（\(t\in[T_o,T_o+2h)\)），之后严格为 \(0\)。

> **注 2（阶段 II 的"包络下凹"是 PDF 的固有代价）** PDF 在单周期内先做"注入—抵消"，中间过程允许 \(|z|\) 暂时增大（这正是 \(M_h\) 的物理含义）。要把 \(M_h\) 压到 1 附近，只能减小 \(h\) 或减小 \(\|K_c\|\)，二者分别受 Gramian 条件数与 \(\Delta_K(h)=0\) 精度的约束。这是"预设包络"与"预设时间"之间唯一真正的牺牲点，必须显式说明，不能声称"全程包络无条件成立"。

---

## 7. 严格性边界与不可行性记录

### 7.1 \(T_c\le T_o\) 与抖振的定量冲突（**最需要处理的一条**）

论文定理 PTDO 要求 \(T_c\le T_o\)。但论文 §6 的实测整定是

\[
T_o=1\,\mathrm s,\qquad T_c=5\,\mathrm s\qquad(\text{即 }T_c>T_o),
\]

并明确记录："\(T_c=1\,\mathrm s\) 时增益过大，在 \(2\,\mathrm{ms}\) 步长下力矩抖动达 \(10\,\mathrm{N\,m}\) 量级、饱和占空比 \(17\%\)、调节时间延长到 \(8.0\,\mathrm s\)、终端误差 \(6.0\times10^{-5}\,\mathrm{rad}\)；取 \(T_c=5\,\mathrm s\) 后饱和占空比 \(7.2\%\)、\(t_s=3.5\,\mathrm s\)、终端误差 \(4.0\times10^{-6}\,\mathrm{rad}\)。"

**因此本文件的定理 5 在论文当前的仿真参数下并不成立**（\(w(t)\not\equiv0\) 于 \(t=T_o\)）。三条出路，必须在论文中明示其中一条：

| 出路 | 代价 |
|---|---|
| (a) 令 \(T_o=T_c=5\,\mathrm s\)，则 \(T_{\mathrm{pre}}=5+2h\)，定理成立 | 总预设时间变长；PPC 需在 5 s 内维持包络（可行，\(\rho\) 收缩更慢反而更安全） |
| (b) 保持 \(T_o=1\,\mathrm s\)、\(T_c=1\,\mathrm s\)，改用**滤波型/变幂次**的 \(\phi_2\)（去掉 \(\mathrm{sgn}\) 或用饱和函数替换）以抑制抖振 | 偏离论文定理 1 的原始形式，需重证或引证变体 |
| (c) 承认"实际有界"：\(t\ge T_o+2h\) 后 \(\|z\|\le\Gamma_h\bar w_{\mathrm{disc}}\)，\(\bar w_{\mathrm{disc}}\) 为采样残差 | 结论从"归零"降级为"\(\Gamma_h\times\)残差带" |

> **建议**：(a) 与 (c) 同时写 —— (a) 保证连续时间定理成立，(c) 给出离散实现的定量降级。这样"预设时间 + 预设包络"的论断在任何时候都不会被参数自相矛盾打破。

### 7.2 观测峰值 vs \(\rho\) 的收缩速度

峰值 \(\bar x\) 增大 ⟹ \(\bar w\) 增大 ⟹ \(\vartheta_i\to1\) ⟹ 进带深度变差。虽然包络不破（定理 4），但若 \(\vartheta_i\varepsilon_i\) 已逼近任务指标，则"进带"就成了字面意义上的"贴到带边"。约束：

\[
\vartheta_i=\sqrt{1-e^{-2\max\{V_i(0),\bar V_i\}}},\qquad
\bar V_i\approx\tfrac12\ln\Bigl(1+\frac{\bar w^2\varepsilon_i^{-2}}{2k_{2i}k_{1i}}\Bigr)+\frac{\bar w^2\varepsilon_i^{-2}}{2k_{2i}^2}
\]

要求 \(\vartheta_i\varepsilon_i\le \epsilon_i^{\mathrm{task}}\) 就给出了 \(k_{2i}\)、\(\varepsilon_i\)、\(T_o\) 的**下界**。注意 \(\bar w\propto T_o\)（引理 2），而 \(\rho\) 的收缩速度 \(\propto1/T_o\)：**\(T_o\) 存在一个最优值**，过小则峰值主导，过大则包络收缩太慢。

### 7.3 离散采样 ⟹ 严格"归零"不可能

三处不连续来源：

1. PTDO 的 \(\mathrm{sgn}(\cdot)\) 在 \(T_s=2\,\mathrm{ms}\) 下的切换（论文实测观测残差 RMS \(4.8\times10^{-3}\)）；
2. \(\nu\) 在 \(t=T_o\) 的切换跳变（文件 08 注 4）；
3. \(\Delta_K(h)\neq0\)（\(\operatorname{cond}(W_c)=4.91\times10^{11}\)，论文实测 PDF 终端误差与 PD 同为 \(10^{-9}\ \mathrm{rad}\)）。

三者叠加的结果是命题 `prop:approx_null` 形式的上界 \(\Gamma_h\bar\Delta_r/(1-\varepsilon_\Delta)\)。**定理 5 的"归零"只能作为连续时间命题陈述**，工程表述必须带上这三项。

### 7.4 阶段 II 不需要 \(T_o\) 之前的历史（实现简化）

\(K_c(\vartheta)\equiv0\) 于 \(\vartheta\in[0,h)\)，故 \(z(\vartheta-h)\) 只在 \(\vartheta\in[h,2h)\) 被用到，对应 \(t\in[T_o,T_o+h)\) 的 \(z\)。**缓存从 \(t=T_o\) 开始即可**，无需在 PPC 阶段存历史。这一点在文件 08/本文件都成立，是两段式相对"全程叠加"的实现优势。

### 7.5 不能声称的东西（红线）

- ❌ "PPC 加速了 PDF 的收敛"——PDF 的 \(2h\) 与 PPC 无关；
- ❌ "加 PPC 后可以放宽 \(\Delta_K(h)=0\)"——不能，\(\Delta_K(h)\) 是纯代数条件；
- ❌ "全程包络无条件成立"——阶段 II 需 \(M_hB_o\le\min_i\varepsilon_i\)；
- ❌ "观测器上界未知即可"——**定理**可以，**定量验收**不行（引理 2 需要一个 \(\bar x\) 假设）。

---

## 8. 完整算法

### 8.1 离线

| 步 | 内容 |
|---|---|
| 1 | 选 \(T_o\)（同时是 PTDO 的预设收敛时间上限与 PPC 的进带时刻）、\(T_c\le T_o\)、\(\eta,\sigma,\bar\xi_0\) |
| 2 | 选 \(\rho_{0i}>\varepsilon_i>0\)、\(m\ge5\)，得 \(\rho_i\) 解析式 |
| 3 | 选 \(k_{1i},k_{2i}\)；用 \(\vartheta_i\varepsilon_i\le\epsilon^{\mathrm{task}}_i\) 反解下界（§7.2） |
| 4 | 选 \(K_0,h,R_h\)；数值积分 \(W_c\)，Tikhonov 正则化求逆 |
| 5 | **校验** \(\|\Delta_K(h)\|\)（数值）与 \(\operatorname{cond}(W_c)\)；不达标则回 4 |
| 6 | 算 \(M_h\)（命题 1/3）与 \(B_o\)（式 6.1），校核 \(M_hB_o\le\min_i\varepsilon_i\) |
| 7 | 生成 \(K_c\) 查找表 |

### 8.2 在线

```
读 q, q_dot, qd, qd_dot, qd_ddot, t
e = q - qd ; e_dot = q_dot - qd_dot
nu_o = Me(q)\(tau_prev - Ce(q,q_dot)*q_dot)     % 上一拍力矩
[z1, z2] = PTDO_step(q_dot, nu_o, z1, z2, t, To, Tc, eta, sigma)   % 式(5.1)
d_hat = Me(q)*z2
if t < To:
    rho, rho_dot, rho_ddot = perf(t, To)        % 式(4.1)
    theta = e./rho
    theta_dot = (e_dot - theta.*rho_dot)./rho
    s = theta_dot + k1.*theta
    nu = rho.*(-k2.*s - theta./(1-theta.^2) - k1.*theta_dot) + rho_ddot.*theta + 2*rho_dot.*theta_dot
else:
    z = [e; e_dot]; push z to ring buffer (from t = To)
    Kc = lookup(theta_local = mod(t-To,2h))
    nu = K0*z - Kc*buffer(t-h)
end
tau_prev = sat(Me(q)*(qd_ddot+nu) + Ce(q,q_dot)*q_dot - d_hat, tau_max)
输出 tau_prev
```

**在线计算量**：PTDO 为 \(O(n)\) 标量运算（无矩阵求逆，\(u_o\) 复用 `Me` 的已有分解）；PPC 为 \(O(n)\)；PDF 为两次 \(2n\times n\) 矩阵乘；总体与"PD + 一阶观测器"同阶。

---

## 9. 与文件 08、与论文主文的对照

| 项目 | 文件 08（无观测器） | 本文件 | 论文现状 |
|---|---|---|---|
| \(d\) 已知 | **必须** | 不需要 | 有 PTDO，但 \(T_c=5>T_o=1\) |
| 可测量 | \(q,\dot q,d\) | \(q,\dot q\) | \(q,\dot q\) |
| Phase I 保证 | 包络（定理 1） | 包络（定理 4，对峰值免疫） | ISS 界（`prop:phaseI_ISS`） |
| 进带时刻 | \(T_1\) 预设 | \(T_o\) 预设 | 无数值保证（实测 3.5 s） |
| 归零时刻 | \(T_1+2h\) | \(T_o+2h\) | \(T_o+2h\)（`cor:prescribed_time`） |
| 阶段 II 包络 | 命题 1 | 命题 3 | 无（`cor:arbitrary_time` 仅给 \(\Gamma_h^*\) 界） |
| 未解决 | 无观测器 ⟹ 不归零 | \(T_c\le T_o\) 与抖振冲突 | Phase I 无预设含义 |
| 新定理数 | 3（定理 1–3）+ 1 命题 | +2（定理 4、5）+ 1 引理 + 1 命题 | — |

**注 3（本文件相对论文的净增量）** 论文已有 PTDO（定理 `thm:ptdo`）与 PDF（定理 `thm:pdf_gain`），两者的组合也已给出总时间 \(T_o+2h\)。本文件的增量只有一处、但很关键：**把 \(0\le t<T_o\) 的"有无穷时间常数的 ISS 界"替换为"有预设包络与预设进带时刻"**。因此论文改动的最小闭环是：

1. 新增 §"性能函数与漏斗变换"（定义 + 式 (4.1) + 引理 1）；
2. 把命题 `prop:phaseI_ISS` **替换**为定理 4（ISS 界保留为注/对照）；
3. 定理 `thm:main_ideal` 的结论增补一条 (i)：\(|e_i(t)|<\rho_i(t)\)；
4. 新增命题 3 与 §7 的 \(\Delta_K(h)\) 验收环节。

---

## 10. 数值验证清单

1. **包络与峰值同图**：\(|e_i(t)|\)、\(\rho_i(t)\)、\(\varepsilon_i\)、\(\|\varepsilon_2(t)\|\)（四曲线同轴）→ 直观证明"峰值不破包络"。
2. **PD vs PPC 的 Phase I 对照**：同一观测器、同一峰值，PD 的 \(\max|e|\) 应随 \(\bar x\) 线性恶化，PPC 应恒在包络内。
3. **进带时刻**：实测 \(t_{\mathrm{band}}^i=\inf\{t:|e_i(t)|\le\varepsilon_i\}\) 与 \(T_o\) 对比（应 \(\le T_o\)）。
4. **归零时刻**：实测 \(\inf\{t:\|z\|\le10^{-9}\}\) 与 \(T_o+2h\) 对比。
5. **\(T_c\) 扫描**：\(T_c\in\{0.3,0.6,1,5\}\times T_o\)，画力矩峰值 / 饱和占空比 / 终端误差 → 定量暴露 §7.1 的冲突。
6. **\(\varepsilon_\Delta\) 敏感性**：人为扰动 \(\Delta_K(h)\)，画出稳态误差与 \(1/(1-\varepsilon_\Delta)\) 的一致性。
7. **阶段 II 越界比**：\(\max_{t\in[T_o,T_o+2h]}|e_i|/\varepsilon_i\) vs \(M_hB_o/\varepsilon_i\)。
