# 路线 A（一）：PPC + PDF 两段式预设时间控制（不含扰动观测器）

> 定位：本文件给出"漏斗型 PPC（阶段 I）→ 周期延迟反馈（阶段 II）"的完整算法、定理与证明，
> 目标是**双重预设**：
>
> 1. 预设时间 \(T_1\)：关节跟踪误差进入**预设误差带** \(\{|e_i|\le\varepsilon_i\}\)；
> 2. 再经预设时间 \(2h\)（\(h\) 为人工延迟），误差**精确归零**。
>
> 总预设时间 \(T_{\mathrm{pre}}=T_1+2h\)，与初始误差无关。
>
> 本文档对应论文 `sections/04-controller.tex`、`sections/05-stability.tex` 的可选扩展章节。
> 不含扰动观测器：**要求 \(d(t)\) 已知**（见 §8 不可行性记录，这是本版本最关键的适用边界）。

---

## 1. 结论摘要（先给能做与不能做）

| 结论 | 是否成立 | 条件 |
|---|---|---|
| \(|e_i(t)|<\rho_i(t),\ \forall t\in[0,T_1]\)（包络不破，**不依赖扰动上界已知**） | ✅ 定理 1 | \(d\) 已知（或 \(w\equiv0\)），\(\lvert e_i(0)\rvert<\rho_{0i}\) |
| \(|e_i(T_1)|\le\vartheta_i\varepsilon_i<\varepsilon_i\)（**预设时间进入误差带**） | ✅ 推论 1 | 同上 |
| \(e(t)=0,\ \forall t\ge T_1+2h\)（**预设时间精确归零**） | ✅ 定理 3 | \(d\) 已知 + 精确模型 + 连续时间 + 无饱和 |
| 阶段 II 误差仍待在包络内 | ⚠️ 有条件（命题 1） | \(M_h\|z(T_1)\|\le\min_i\varepsilon_i\) |
| \(d\) 未知时仍能"归零" | ❌ **不成立** | 见 §8.1，仅得 \(\Gamma_h\bar w\) |
| \(d\) 未知时仍能给出 \(\vartheta_i\) 的数值 | ❌ 不成立 | 见 §8.2，只能给"\(<1\)" |

---

## 2. 系统、误差通道与假设

### 2.1 等效关节空间模型（沿用论文式 `joint_model_relabelled`）

\[
M_e(q_m)\ddot q_m+C_e(q_m,\dot q_m)\dot q_m=\tau+d(t),\qquad q_m\in\mathbb R^n .
\]

定义跟踪误差与状态

\[
e=q_m-q_d,\qquad \dot e=\dot q_m-\dot q_d,\qquad z=\begin{bmatrix}e\\ \dot e\end{bmatrix}\in\mathbb R^{2n}.
\]

### 2.2 假设

**A1（等效惯性）** \(M_e(q_m)\) 对称、连续、一致正定且**精确已知**；\(C_e(q_m,\dot q_m)\dot q_m\) 精确已知。

**A2（可测性）** \(q_m(t)\)、\(\dot q_m(t)\) 可实时测量；\(q_d,\dot q_d,\ddot q_d\) 连续有界。

**A3（扰动已知——本版本的强假设）** 集总扰动 \(d(t)\) 在 \(t\ge0\) 上**已知**（解析式或在线可算），且
\(\Delta_a(t)=M_e^{-1}(q_m)d(t)\) 有界。若 \(d\) 仅"有界但未知"，则本文件的定理 1/2 仍成立（见注 3），但定理 3 的"归零"退化为 §8.1 的有界结论。

**A4（初始条件在漏斗内）** \(|e_i(0)|<\rho_{0i},\ i=1,\dots,n\)。

**A5（理想实现条件）** 连续时间实现；无执行器饱和；无测量噪声；无模型误差。

> **A1+A3+A5 的合取**即论文假设 `as:ideal` 的等价形式。混合控制器在阶段 I 额外需要：
> (i) \(q_d\) 的二阶可微性（因为 (2.1) 要用 \(\ddot q_d\)，A2 已含）；
> (ii) 性能函数 \(\rho_i\) 的解析二阶导数（因 (4.4) 要用 \(\rho^{(2)}\)，由式 (4.1) 解析给出，不需求解任何微分方程）。
> 注意 \(\rho_i\) **不依赖 \(q_d(0)\) 或 \(e_i(0)\)**，故不存在“初始条件反推包络”的循环依赖。

### 2.3 补偿后的误差通道（关键引理）

取"计算力矩 + 已知扰动前馈补偿"

\[
\boxed{\ \tau=M_e(q_m)\bigl(\ddot q_d+\nu\bigr)+C_e(q_m,\dot q_m)\dot q_m-d(t)\ }
\tag{2.1}
\]

代入动力学并左乘 \(M_e^{-1}\)：

\[
\ddot q_m=\ddot q_d+\nu\quad\Longrightarrow\quad \boxed{\ \ddot e(t)=\nu(t)\ }
\tag{2.2}
\]

即误差通道是**严格解耦的 \(n\) 个二阶积分器**。注意：本文件不使用观测器，全靠 A3 把 \(d\) 消掉。

---

## 3. 控制结构总览

```
t ∈ [0, T1)                     t ∈ [T1, T1+2h)                t ≥ T1+2h
─────────────────────────────   ────────────────────────────   ────────────
阶段 I：漏斗型 PPC              阶段 II：PDF（单周期零化）       稳态
|e_i(t)| < ρ_i(t)  (包络)       Δ_K(h)=0 ⇒ z(ϑ)=0  @ ϑ=2h      e ≡ 0, ė ≡ 0
ρ_i(T1) = ε_i      (进带)       ν = K0 z − Kc(ϑ)z(ϑ−h)          τ = M_e q̈_d + C_e q̇ − d
ν = ν^PPC(e,ė,ρ,ρ̇,ρ̈)          Kc(ϑ)=0, ϑ∈[0,h)                （纯逆动力学）
─────────────────────────────   ────────────────────────────   ────────────
     ↑ 预设时间 T1                     ↑ 预设时间 2h（h 任选）
```

统一写法：\(\nu(t)=\nu^{\mathrm{PPC}}(t)\cdot\mathbb 1_{t<T_1}+\bigl[K_0z(t)-K_c(t-T_1)z(t-h)\bigr]\cdot\mathbb 1_{t\ge T_1}\)。

**缓存需求**：阶段 II 只在 \(\vartheta\in[h,2h)\) 用到 \(z(\vartheta-h)\)，即 \(t\in[T_1,T_1+h)\) 的 \(z\)。因 \(K_c(\vartheta)\equiv0,\ \vartheta\in[0,h)\)，**不需要 \(T_1\) 之前的历史数据**，只需缓存 \(h\) 秒。

---

## 4. 阶段 I：漏斗型 PPC

### 4.1 性能函数（指定时间可达型）

对每个关节 \(i\)，取

\[
\boxed{\
\rho_i(t)=
\begin{cases}
\varepsilon_i+(\rho_{0i}-\varepsilon_i)\left(1-\dfrac{t}{T_1}\right)^{m}, & 0\le t<T_1,\\[8pt]
\varepsilon_i, & t\ge T_1,
\end{cases}
\qquad m=2k+1\ge 5 }
\tag{4.1}
\]

其中 \(\rho_{0i}>\varepsilon_i>0\) 为预设常数，\(T_1>0\) 为预设时间。取 \(m\ge5\) 的理由：
\(\rho_i\in C^{m-1}\)，从而 \(\rho_i,\dot\rho_i,\ddot\rho_i\) 在 \(t=T_1\) 连续，
且在 \(T_1\) 处 \(\rho_i^{(j)}(T_1)=0,\ j=1,\dots,m-1\)。

性质（可直接验证，对应文献 `[2093]` 定义 5.3、(5-15) 的 ATPF 形式，取 \(k_t=(m-1)/2\)）：

1. \(\rho_i>0\)，\(\dot\rho_i(t)=-m(\rho_{0i}-\varepsilon_i)(1-t/T_1)^{m-1}/T_1\le0\)：单调递减；
2. \(\rho_i(T_1)=\varepsilon_i\)，且 \(\rho_i\equiv\varepsilon_i,\ t\ge T_1\)：**在预设时间精确到达误差带**；
3. \(\rho_i(0)=\rho_{0i}\)：初始包络由设计者给出（非由 \(e_i(0)\) 反推，避免初值依赖）。

> **注 1（与传统 PPF 的区别）** 指数型 \(\rho=(\rho_0-\rho_\infty)e^{-lt}+\rho_\infty\) 只在 \(t\to\infty\) 渐近取到 \(\rho_\infty\)，"进入误差带的时间"不可预设；式 (4.1) 把"进带时刻"变成一个显式参数 \(T_1\)，这正是要求 1 的实现基础。

> **注 2（初值依赖）** 传统 PPC 需 \(|e_i(0)|<\rho_{0i}\) 才能启动。若初始误差范围不可预知，可改用 `[2093]` 的 MPPF（式 5-12），其 \(\rho(0)\to\infty\)：即取
> \(\rho_i(t)=\csc\!\bigl(\tfrac{\pi(t+l_i)}{2t_i}\bigr)\bigl[\varepsilon_i+(\rho_{0i}-\varepsilon_i)(1-t/T_1)^{m}\bigr]\)，\(t\in[0,t_i-l_i)\)。
> 代价是 \(\rho_i(0)\) 极大 ⟹ 阶段 I 前段的实际包络几乎无约束，且 \(\rho_0\) 的含义变为"过渡结束时刻的包络值"。本文件保留式 (4.1) 并显式声明 A4。

### 4.2 归一化坐标、误差变换与辅助状态

归一化误差与变换误差：

\[
\theta_i(t)=\frac{e_i(t)}{\rho_i(t)}\in(-1,1),\qquad
\varepsilon_i^{\mathrm{tr}}=\operatorname{arctanh}\theta_i=\frac12\ln\frac{1+\theta_i}{1-\theta_i}
\tag{4.2}
\]

这即误差变换函数 \(S_{\mathrm{tran}}(\varepsilon^{\mathrm{tr}})=\tanh(\varepsilon^{\mathrm{tr}})\) 的反函数（对称情形；非对称 \(\delta_{a},\delta_{b}\) 见注 4）。
按 `[2093]` 引理 5.8 的思路：**\(\varepsilon^{\mathrm{tr}}_i\) 有界 \(\iff|e_i|<\rho_i\) 严格成立**。因此"保包络"等价于"\(\varepsilon^{\mathrm{tr}}\) 有界"。

辅助（反步）状态：

\[
\boxed{\ s_i(t)=\dot\theta_i(t)+k_{1i}\theta_i(t),\qquad
\dot\theta_i=\frac{\dot e_i-\theta_i\dot\rho_i}{\rho_i}\ }
\tag{4.3}
\]

其中 \(k_{1i}>0\)。式 (4.3) 只用可测的 \(e_i,\dot e_i\) 与已知的 \(\rho_i,\dot\rho_i\)，**不需要加速度**。

### 4.3 阶段 I 控制律

\[
\boxed{\
\nu_i^{\mathrm{PPC}}(t)=
\rho_i\Bigl[-k_{2i}s_i-\frac{\theta_i}{1-\theta_i^{2}}-k_{1i}\dot\theta_i\Bigr]
+\ddot\rho_i\,\theta_i+2\dot\rho_i\,\dot\theta_i
\qquad (i=1,\dots,n)
}\tag{4.4}
\]

其结构为三项之和：**漏斗反步项**（方括号内，\(-\theta/(1-\theta^2)\) 是漏斗/障碍项）＋**性能函数二阶前馈**（\(\ddot\rho\,\theta\)）＋**归一化交叉项**（\(2\dot\rho\dot\theta\)）。后两项来自坐标变换引起的"虚拟强迫"，用来把 \(\theta\) 动力学变成时不变形式。

### 4.4 定理 1（阶段 I：包络保持 + 进带）

> **定理 1** 考虑误差通道 (2.2)，控制律取 (4.4)，性能函数取 (4.1)。若 A1、A2、A4 成立且 \(w:=M_e^{-1}(d-\hat d)\) 满足 \(\|w(t)\|_\infty\le\bar w<\infty\)（本文件取 \(d\) 已知 ⟹ \(\hat d=d\) ⟹ \(w\equiv0\)），则闭环在 \([0,T_1]\) 上无逃逸（整体存在，见证明第 (5) 步），且
> \[
> |\theta_i(t)|\le\vartheta_i:=\sqrt{1-e^{-2\max\{V_i(0),\bar V_i\}}}<1,
> \qquad \forall t\in[0,T_1],
> \]
> 从而
> \[
> \boxed{\ |e_i(t)|\le\vartheta_i\,\rho_i(t)<\rho_i(t),\qquad \forall t\in[0,T_1].\ }
> \]
> 其中
> \[
> V_i=\tfrac12\ln\frac{1}{1-\theta_i^2}+\tfrac12 s_i^2,\qquad
> \bar V_i=\tfrac12\ln\frac{c_i+k_{1i}}{k_{1i}}+\frac{c_i}{k_{2i}},\qquad
> c_i=\frac{\sigma_i^2}{2k_{2i}},\qquad
> \sigma_i=\frac{\bar w}{\varepsilon_i}.
> \]
> **\(\vartheta_i<1\) 对任意有限 \(\bar w\) 成立**，与 \(\bar w\) 的具体数值无关。

**证明.**

**（1）降阶与 \(\theta\) 的动力学。** 由 \(\theta_i=e_i/\rho_i\) 得

\[
\dot e_i=\dot\rho_i\theta_i+\rho_i\dot\theta_i,\qquad
\ddot e_i=\ddot\rho_i\theta_i+2\dot\rho_i\dot\theta_i+\rho_i\ddot\theta_i .
\]

代入 (2.2) \(\ddot e_i=\nu_i+w_i\) 并整理：

\[
\boxed{\ \ddot\theta_i=\frac{\nu_i+w_i-\ddot\rho_i\theta_i-2\dot\rho_i\dot\theta_i}{\rho_i}\ }
\tag{4.5}
\]

**（2）辅助状态的动力学。** 对 (4.3) 求导并代入 (4.5)：

\[
\dot s_i=\ddot\theta_i+k_{1i}\dot\theta_i
=\frac{\nu_i+w_i-\ddot\rho_i\theta_i-2\dot\rho_i\dot\theta_i}{\rho_i}+k_{1i}\dot\theta_i .
\]

将 (4.4) 的 \(\nu_i\) 代入，右端前馈项精确相消：

\[
\frac{\nu_i-\ddot\rho_i\theta_i-2\dot\rho_i\dot\theta_i}{\rho_i}
=-k_{2i}s_i-\frac{\theta_i}{1-\theta_i^2}-k_{1i}\dot\theta_i,
\]

故

\[
\boxed{\ \dot s_i=-k_{2i}s_i-\frac{\theta_i}{1-\theta_i^{2}}+\frac{w_i}{\rho_i},\qquad
\dot\theta_i=s_i-k_{1i}\theta_i\ }
\tag{4.6}
\]

即闭环被化为 \((\theta_i,s_i)\) 平面上的**无扰动标称系统 + 有界匹配项 \(w_i/\rho_i\)**。

**（3）Lyapunov 分析。** 沿 (4.6) 求导：

\[
\dot V_i=\frac{\theta_i\dot\theta_i}{1-\theta_i^{2}}+s_i\dot s_i
=\frac{\theta_i(s_i-k_{1i}\theta_i)}{1-\theta_i^{2}}
+s_i\Bigl(-k_{2i}s_i-\frac{\theta_i}{1-\theta_i^{2}}+\frac{w_i}{\rho_i}\Bigr).
\]

\(\theta_i s_i/(1-\theta_i^2)\) 项**精确相消**（这正是 (4.4) 中漏斗项系数取 \(1\) 的原因），得

\[
\dot V_i=-\frac{k_{1i}\theta_i^{2}}{1-\theta_i^{2}}-k_{2i}s_i^{2}+\frac{s_iw_i}{\rho_i}.
\tag{4.7}
\]

由 \(\rho_i(t)\ge\varepsilon_i\) 与 \(\|w\|_\infty\le\bar w\) 有 \(|w_i|/\rho_i\le\sigma_i\)。用 Young 不等式
\(|s_i|\sigma_i\le\tfrac12k_{2i}s_i^2+\dfrac{\sigma_i^2}{2k_{2i}}\)：

\[
\boxed{\ \dot V_i\le-\frac{k_{1i}\theta_i^{2}}{1-\theta_i^{2}}-\frac{k_{2i}}{2}s_i^{2}+c_i\ }
\tag{4.8}
\]

**（4）无逃逸与一致界。** 记 \(\Omega_i=\{(\theta_i,s_i):\dot V_i\ge0\}\)。由 (4.8)，在 \(\Omega_i\) 上必有

\[
\frac{k_{1i}\theta_i^{2}}{1-\theta_i^{2}}\le c_i,\qquad
\frac{k_{2i}}{2}s_i^{2}\le c_i ,
\]

即 \(|\theta_i|\le\bar\theta_i\)，\(\bar\theta_i^2=\dfrac{c_i}{c_i+k_{1i}}<1\)（对任意有限 \(c_i\)；若 \(c_i=0\) 则 \(\bar\theta_i=0\)），且 \(s_i^2\le2c_i/k_{2i}\)。于是

\[
V_i\le\tfrac12\ln\frac{1}{1-\bar\theta_i^2}+\tfrac12\cdot\frac{2c_i}{k_{2i}}
=\tfrac12\ln\frac{c_i+k_{1i}}{k_{1i}}+\frac{c_i}{k_{2i}}=\bar V_i
\qquad\text{在 }\Omega_i\text{ 上}.
\]

因此在 \(\Omega_i\) 上有 \(\dot V_i\le0\) 与 \(V_i\le\bar V_i\) 同时成立（否则 \(\dot V_i<0\)）。标准比较论证给出

\[
V_i(t)\le\max\{V_i(0),\bar V_i\}=:V_i^{\max},\qquad \forall t\in[0,T_1]\ \text{（只要解存在）}.
\tag{4.9}
\]

**（5）解的整体存在与严格内点性。** (4.4) 的右端在开集 \(\{|\theta_i|<1\}\times\mathbb R\) 上局部 Lipschitz（\(\rho_i,\dot\rho_i,\ddot\rho_i\) 在 \([0,T_1]\) 上连续、\(|\theta_i|<1\) 时 \(1-\theta_i^2>0\)），故局部解存在唯一。设 \(T^\ast\) 为首次满足 \(|\theta_i(T^\ast)|=1\) 的时刻。在 \([0,T^\ast)\) 上 (4.9) 成立，于是

\[
\tfrac12\ln\frac{1}{1-\theta_i^2(t)}\le V_i(t)\le V_i^{\max}
\ \Longrightarrow\
\theta_i^2(t)\le1-e^{-2V_i^{\max}}<1 ,
\]

与 \(|\theta_i(T^\ast)|=1\) 矛盾。故 \(T^\ast=+\infty\)，且

\[
|\theta_i(t)|\le\vartheta_i=\sqrt{1-e^{-2V_i^{\max}}},\quad \forall t\in[0,T_1].
\]

由 \(|\theta_i|=|e_i|/\rho_i\) 即得 \(|e_i(t)|\le\vartheta_i\rho_i(t)<\rho_i(t)\)。\(\blacksquare\)

**注 3（不需要扰动上界即可保包络）** 定理 1 的 \(\vartheta_i<1\) 只要求 \(\bar w\) **有限**。也就是说：即便完全不知道 \(\bar w\)（对应论文假设 `as:disturbance` 中"上界未必已知"），包络也**不会破**；未知的 \(\bar w\) 只影响**实际贴合程度** \(\vartheta_i\)（\(\bar w\to\infty\) 时 \(\vartheta_i\to1\)）。这是本方案相对"把 PPC 直接叠在 PDF 上"的结构性优势，同时也说明 §8.2 的问题：\(\vartheta_i\) 的**数值**确实依赖 \(\bar w\)。

### 4.5 推论 1（预设时间进入误差带）

> **推论 1** 在定理 1 的条件下，因 \(\rho_i(T_1)=\varepsilon_i\)，
> \[
> \boxed{\ |e_i(T_1)|\le\vartheta_i\varepsilon_i<\varepsilon_i,\qquad i=1,\dots,n,\ }
> \]
> 且 \(\vartheta_i\) 与初始误差大小无关（只依赖 \(k_{1i},k_{2i},\bar w,\varepsilon_i\)）。

这就是"**在预设时间 \(T_1\) 内进入预设误差带**"的严格形式。

### 4.6 推论 2（\(d\) 已知时的定量收敛率）

若 \(d\) 已知（\(w\equiv0\)），则 \(c_i=0\)、\(\bar V_i=0\)，且

\[
\dot V_i\le-\frac{k_{1i}\theta_i^2}{1-\theta_i^2}-k_{2i}s_i^2\le-2\underline k_i V_i,
\qquad \underline k_i=\min\{k_{1i},k_{2i}\},
\]

其中用到 \(\ln\frac{1}{1-\theta^2}\le\frac{\theta^2}{1-\theta^2}\)。故

\[
\boxed{\ V_i(t)\le V_i(0)e^{-2\underline k_it},\qquad
|\theta_i(t)|\le\sqrt{1-e^{-2V_i(0)e^{-2\underline k_it}}}\ }
\tag{4.10}
\]

即误差**比包络衰减得更快**（以指数速率进入 \(0\) 相对包络）。特别地

\[
|e_i(T_1)|\le\varepsilon_i\sqrt{1-e^{-2V_i(0)e^{-2\underline k_iT_1}}} ,
\]

当 \(2\underline k_iT_1\gg\ln V_i(0)\) 时右端远小于 \(\varepsilon_i\)。这给了一个**整定准则**：\(2\underline kT_1\gtrsim\ln(1/|e(0)|/\varepsilon)\)，例如 \(|e(0)|=10^{-2},\varepsilon=10^{-3}\) 只需 \(2\underline kT_1\gtrsim2.3\)。

### 4.7 阶段 II 的初始条件

> **引理 1（交接状态有界）** 在定理 1 条件下，
> \[
> \|z(T_1)\|\le\Bigl(\sum_i\vartheta_i^2\varepsilon_i^2+\sum_i M_{1i}^2\Bigr)^{1/2},\qquad
> M_{1i}=\varepsilon_i\bigl(\sqrt{2V_i^{\max}}+k_{1i}\bigr)
> \]
> （因 \(\dot\rho_i(T_1)=0\)，故 \(|\dot e_i(T_1)|=\varepsilon_i|\dot\theta_i(T_1)|\le\varepsilon_i(|s_i|+k_{1i}|\theta_i|)\)）。

---

## 5. 阶段 II：PDF 单周期零化

### 5.1 局部化与闭环

以局部时间 \(\vartheta=t-T_1\) 重新计时。由 (2.2)（\(w=0\)，即 \(d\) 已被 (2.1) 精确抵消）：

\[
\frac{dz}{d\vartheta}=Az+B\nu,\qquad
A=\begin{bmatrix}0&I_n\\0&0\end{bmatrix},\quad B=\begin{bmatrix}0\\I_n\end{bmatrix}.
\tag{5.1}
\]

取

\[
\boxed{\ \nu(\vartheta)=K_0z(\vartheta)-K_c(\vartheta)z(\vartheta-h),\qquad K_c(\vartheta)=0,\ \vartheta\in[0,h)\ }
\tag{5.2}
\]

其中 \(K_0\) 使 \(A_c=A+BK_0\) Hurwitz（\((A,B)\) 可控，极点配置保证存在），\(K_c\) 按 Gramian 型设计。闭环：

\[
\frac{dz}{d\vartheta}=A_cz(\vartheta)-BK_c(\vartheta)z(\vartheta-h).
\tag{5.3}
\]

### 5.2 定理 2（单周期零化，自洽重述）

> **定理 2** 取 \(R_h:\mathbb R\to[0,\infty)\) 为 \(2h\) 周期、\(\ C^1\) 函数，满足 \(R_h\equiv0\) 于 \([0,h]\)、在每个 \([h,2h]\) 上的某正测度子集上严格正。定义
> \[
> W_c=\int_h^{2h}e^{-A_cs}BR_h(s)B^{\mathsf T}e^{-A_c^{\mathsf T}s}\,\mathrm ds\succ0,\qquad
> K_c(\vartheta)=\begin{cases}0,&\vartheta\in[0,h),\\ R_h(\vartheta)B^{\mathsf T}e^{-A_c^{\mathsf T}\vartheta}W_c^{-1}e^{A_c(h-\vartheta)},&\vartheta\in[h,2h),\end{cases}
> \]
> 并以 \(2h\) 为周期延拓。则 DDE (5.3) 的单周期状态转移矩阵满足 \(\Delta_K(h)=0\)，从而
> \[
> z(\vartheta)=0,\qquad \forall\vartheta\ge2h .
> \]

**证明（分步法）。**

*前段* \(\vartheta\in[0,h]\)：\(K_c=0\)，\(z(\vartheta)=e^{A_c\vartheta}z(0)\)，特别 \(z(h)=e^{A_ch}z(0)\)，且对 \(\vartheta\in[h,2h]\) 有 \(z(\vartheta-h)=e^{A_c(\vartheta-h)}z(0)\)。

*后段* \(\vartheta\in[h,2h]\)：由变分公式

\[
z(\vartheta)=e^{A_c(\vartheta-h)}z(h)-\int_h^\vartheta e^{A_c(\vartheta-s)}BK_c(s)\,e^{A_c(s-h)}z(0)\,\mathrm ds .
\]

在 \(\vartheta=2h\)：

\[
z(2h)=\Bigl[e^{A_c2h}-\int_h^{2h}e^{A_c(2h-s)}BK_c(s)e^{A_c(s-h)}\mathrm ds\Bigr]z(0).
\tag{5.4}
\]

代入 \(K_c(s)\)：先注意 \(e^{A_c(h-s)}e^{A_c(s-h)}=I_{2n}\)（同一矩阵的指数互为逆），再用
\(e^{A_c(2h-s)}=e^{A_c2h}e^{-A_cs}\)（同一矩阵 \(A_c\) 的指数可合并），得

\[
\int_h^{2h}e^{A_c(2h-s)}BK_c(s)e^{A_c(s-h)}\mathrm ds
=e^{A_c2h}\underbrace{\Bigl[\int_h^{2h}e^{-A_cs}BR_h(s)B^{\mathsf T}e^{-A_c^{\mathsf T}s}\mathrm ds\Bigr]}_{=W_c}W_c^{-1}
=e^{A_c2h}.
\]

（一致性检验：\(K_c\) 中含 \(e^{A_c(h-s)}\)，恰好抵消延迟状态带来的 \(e^{A_c(s-h)}\)；\(W_c\succ0\) 保证 \(W_c^{-1}\) 存在，这一点由 \((A_c,B)\) 可控 + \(R_h\) 在正测度子集上严格正保证。）

与 (5.4) 中第一项相消，得 \(z(2h)=0\)，即 \(\Delta_K(h)=0\)。

**永久零化**：\(\vartheta\in[2h,3h)\) 时 \(K_c(\vartheta)=0\)（每周期前半段），故 \(\dot z=A_cz\)，由 \(z(2h)=0\) 得 \(z\equiv0\)；
\(\vartheta\in[3h,4h)\) 时 \(K_c\ne0\)，但延迟状态 \(z(\vartheta-h)=0\)，故仍为 \(\dot z=A_cz\) 且 \(z(3h)=0\)，得 \(z\equiv0\)。
逐步归纳即得 \(z(\vartheta)=0,\ \forall\vartheta\ge2h\)。\(\blacksquare\)

### 5.3 定理 3（总体：双重预设）

> **定理 3（PPC + PDF 双重预设）** 设 A1–A5 成立（其中 A3：\(d\) 已知），控制律取 (2.1) 与
> \(\nu=\nu^{\mathrm{PPC}}(t)\)（\(t<T_1\)，式 (4.4)）、\(\nu=K_0z-K_c(t-T_1)z(t-h)\)（\(t\ge T_1\)，式 (5.2)），
> 性能函数取 (4.1)。则闭环满足
> \[
> \boxed{\ |e_i(t)|<\rho_i(t)\ (t\in[0,T_1]);\qquad |e_i(T_1)|\le\vartheta_i\varepsilon_i\ (i=1,\dots,n);\qquad e(t)\equiv0,\ t\ge T_1+2h.\ }
> \]
> 总预设时间 \(T_{\mathrm{pre}}=T_1+2h\)，与 \(z(0)\) 无关。

**证明.** 阶段 I 由定理 1 与推论 1；交接由引理 1 保证 \(z(T_1)\) 有界；阶段 II 由定理 2，对**任意**有界初值 \(z(T_1)\) 给出 \(z(\vartheta)=0,\ \vartheta\ge2h\)，回代 \(t=T_1+\vartheta\) 即得。切换处 \(\nu\) 的取值可以跳变（定理 2 只把 \(z(T_1)\) 当作初值，不要求 \(\nu\) 连续），唯一要求是 A5 的无饱和。\(\blacksquare\)

**注 4（切换处连续性）** 取 \(m\ge5\) 时 \(\dot\rho(T_1)=\ddot\rho(T_1)=0\)，(4.4) 中 \(\ddot\rho\theta+2\dot\rho\dot\theta\to0\)，但漏斗项 \(-\theta/(1-\theta^2)\) 与 \(K_0z\) 一般不相等，故 \(\nu\) 仍有一次跳变，幅度
\[
\|\Delta\nu(T_1)\|\le\|\nu^{\mathrm{PPC}}(T_1^-)-K_0z(T_1)\| ,
\]
对应力矩跳变 \(\Delta\tau=M_e\Delta\nu\)。若不可接受，可在 \([T_1-\delta,T_1]\) 内用 \(C^2\) 混合 \(\nu=(1-\beta)\nu^{\mathrm{PPC}}+\beta K_0z\)，\(\beta\in[0,1]\) 光滑；但该窗口内的包络保持需按定理 1 的 \(w\)-处理重新计数（混合项可视为附加有界匹配扰动），结论退化为定理 1 形式。

### 5.4 命题 1（阶段 II 是否仍在包络内）

阶段 II 是线性系统，其误差可能短暂超出 \(\varepsilon_i\)。给出可离线计算的判据。

> **命题 1** 记 \(\mathcal E_h:=\max_{\vartheta\in[0,h]}\|e^{A_c\vartheta}\|\)。则在阶段 II 的 \(z(\vartheta)\)（初值 \(z_0=z(T_1)\)）满足
> \[
> \|z(\vartheta)\|\le M_h\|z_0\|,\qquad
> M_h:=\mathcal E_h^{2}\Bigl(1+\|B\|\int_h^{2h}\|K_c(s)\|\,\mathrm ds\Bigr).
> \]
> 因此若
> \[
> \boxed{\ M_h\|z(T_1)\|\le\min_{i}\varepsilon_i\ }
> \]
> 则全程包络 \(|e_i(t)|\le\varepsilon_i\ (\forall t\ge0)\) 成立；若该式不成立，则只能给出
> \(|e_i(t)|\le M_h\|z(T_1)\|,\ t\in[T_1,T_1+2h)\) 与 \(|e_i(t)|=0,\ t\ge T_1+2h\)。

**证明.** \(\vartheta\in[0,h]\)：\(z=e^{A_c\vartheta}z_0\)。\(\vartheta\in[h,2h]\)：用变分公式
\(z(\vartheta)=e^{A_c(\vartheta-h)}z(h)-\int_h^\vartheta e^{A_c(\vartheta-s)}BK_c(s)z(s-h)\mathrm ds\)，取范数，注意 \(\|z(s-h)\|\le\mathcal E_h\|z_0\|\)（前段结论）并用 Grönwall 型估计得所述界。\(\blacksquare\)

**推论 3（设计条件）** 命题 1 的判据把 \(h\)、\(K_0\)、\(K_c\)、\(\varepsilon_i\) 联系起来：\(h\) 越大，\(\mathcal E_h=\|e^{A_ch}\|\) 越大（指数增长），故 \(h\) 应取在 PDF 理论可接受的小量级（\(h\sim1/\|A_c\|\)），这与论文 Remark（Gramian 条件数 vs \(h\)）结论一致。

---

## 6. 完整算法（可直接编码）

### 6.1 离线设计（一次性）

| 步 | 内容 | 输出 |
|---|---|---|
| 1 | 选 \(T_1\)（进带时刻）、\(\rho_{0i}>\varepsilon_i>0\)、\(m\ge5\) | \(\rho_i\) 的表（可解析计算，无需存表） |
| 2 | 选 \(k_{1i},k_{2i}\)；用推论 2 校验 \(2\underline k_iT_1\gtrsim\ln(\rho_{0i}/\varepsilon_i)\) | PPC 增益 |
| 3 | 选 \(K_0\)（\(A_c\) Hurwitz）、\(h\)、\(R_h\)（如 \(\sin^4\)） | \(A_c,h,R_h\) |
| 4 | 数值积分 \(W_c\)（梯形/Gauss），求 \(W_c^{-1}\)（建议 Tikhonov 或截断 SVD） | \(W_c^{-1}\) |
| 5 | **校验 \(\|\Delta_K(h)\|\)**（数值代入式 (5.4)）与 \(\operatorname{cond}(W_c)\) | 验收指标 |
| 6 | 计算 \(M_h\)（命题 1），校验 \(M_h\|z(T_1)\|_\text{估}\le\min_i\varepsilon_i\) | 是否保全程包络 |
| 7 | 计算 \(K_c\) 查找表（\(\vartheta\in[0,2h)\) 网格） | 表 |

### 6.2 在线（每采样步，\(T_s\)）

```
读 q, qd, qd_dot, qd_ddot, t
e = q - qd ;  e_dot = q_dot - qd_dot
if t < T1:
    rho, rho_dot, rho_ddot  ← 式(4.1)解析式
    theta = e ./ rho
    theta_dot = (e_dot - theta.*rho_dot)./rho
    s = theta_dot + k1.*theta
    nu = rho.*(-k2.*s - theta./(1-theta.^2) - k1.*theta_dot) + rho_ddot.*theta + 2*rho_dot.*theta_dot
    d_hat = d(t)                       % A3：已知扰动
else:
    z  = [e; e_dot];  push z into ring buffer
    theta_local = mod(t - T1, 2h)
    Kc  = lookup(Kc_table, theta_local)
    z_d = buffer(t - h)                   % 只需 T1 之后的 h 秒历史
    nu  = K0*z - Kc*z_d
    d_hat = d(t)
end
tau = Me(q)*(qd_ddot + nu) + Ce(q,q_dot)*q_dot - d_hat
tau = sat(tau, tau_max)
```

计算量：阶段 I 为 \(O(n)\) 标量运算；阶段 II 为一次 \(2n\times n\) 与 \(n\times2n\) 乘法，与 PD 同阶（与论文 Remark 一致）。

### 6.3 参数整定的耦合约束（务必成对检查）

1. \(k_{1i},k_{2i}\uparrow\) ⟹ \(\vartheta_i\) 更小、包络贴得更紧，但力矩峰值 \(\uparrow\)（与 \(\tau_{\max}\) 冲突）。
2. \(\varepsilon_i\downarrow\) ⟹ 进带更紧，但 \(\sigma_i=\bar w/\varepsilon_i\uparrow\) ⟹ \(\vartheta_i\to1\)，边际收益递减；**\(\vartheta_i\to1\) 就是"贴着包络飞"的数学表现**。
3. \(T_1\downarrow\) ⟹ \(\dot\rho\propto1/T_1\) 增大 ⟹ \(\nu\) 中 \(2\dot\rho\dot\theta\) 项增大 ⟹ 力矩峰值 \(\uparrow\)。给出保守估计
   \(\|\nu\|_\infty\lesssim\frac{m(\rho_0-\varepsilon)}{T_1}\cdot 2\sqrt{2V^{\max}}+\ldots\)，实测标定更可靠。
4. \(h\downarrow\) ⟹ PDF 收敛更快，但 \(\operatorname{cond}(W_c)\uparrow\)（论文实测 \(n=7\) 时 \(4.91\times10^{11}\)）。

---

## 7. 与论文主文的映射

| 本文件 | 论文对应 | 关系 |
|---|---|---|
| 式 (2.1) 中的 \(-d(t)\) | 式 `computed_torque` 中的 \(-\hat d(t)\) | 本文件用"已知 \(d\)"替代观测器 |
| 定理 1、推论 1、2 | 命题 `prop:phaseI_ISS`（ISS 界） | **替换**：把 ISS 界升级为包络界 |
| 引理 1 | — | 新增（交接初值的有界性） |
| 定理 2 | 定理 `thm:pdf_gain` | 完全沿用（补自洽证明） |
| 定理 3 | 定理 `thm:main_ideal` | 把 \(T_o\) 换成 \(T_1\)，并增加进带结论 |
| 命题 1、推论 3 | 推论 `cor:arbitrary_time` | 新增（阶段 II 的包络校验） |
| §8 | 假设 `as:disturbance` | 明确冲突点 |

---

## 8. 不可行性记录（严格性边界）

### 8.1 无观测器时"精确归零"不成立（**关键**）

若 A3 不成立（\(d\) 未知），阶段 II 的通道变为

\[
\frac{dz}{d\vartheta}=A_cz-BK_c(\vartheta)z(\vartheta-h)+B\Delta_a(\vartheta),
\qquad \Delta_a=M_e^{-1}d ,
\]

由周期变分公式与 \(\Delta_K(h)=0\)：

\[
z(2h)=\int_0^{2h}\mathcal S_h(2h,s)B\Delta_a(s)\,\mathrm ds
\ \Longrightarrow\
\|z(\vartheta)\|\le\Gamma_h\bar\Delta_a,\qquad \Gamma_h:=\max_{\theta\in[0,2h]}\int_0^\theta\|\mathcal S_h(\theta,s)B\|\mathrm ds,
\]

即**只能得到 \(\Gamma_h\bar\Delta_a\) 的残余带，不可能为零**（除非 \(\Delta_K\) 之外另有机制抵消 \(\Delta_a\)）。

**定量失配**：本文件的阶段 I 已把 \(e\) 压到 \(\vartheta_i\varepsilon_i\)。若

\[
\Gamma_h\bar\Delta_a>\vartheta_i\varepsilon_i ,
\]

则阶段 II 不仅不能"归零"，反而**把误差放大**。用论文既有仿真的量级可以说明这不是理论上的空谈：

- 论文 §6 实测（有扰动、无观测器补偿）：\(\|e(T)\|\approx5.4\times10^{-2}\,\mathrm{rad}\)；
- 可设 \(\varepsilon_i=10^{-3}\,\mathrm{rad}\)；
- 故 \(5.4\times10^{-2}\gg10^{-3}\)，**阶段 II 会破坏阶段 I 的精度**。

**结论**：本文件的"两段式预设时间"只有在 (a) \(d(t)\) 解析已知（可直接前馈抵消），或 (b) 附加鲁棒机制把 \(\bar\Delta_a\) 压到 \(\Gamma_h\bar\Delta_a\ll\vartheta\varepsilon\) 时才成立。在自由漂浮空间机械臂的捕获前场景中，(a) 通常不成立 —— **这正是必须引入观测器的原因**，见 `09-ppc-pdf-route-a-ptdo.md`。

> **注 5（可用的退路）** 若坚持不用观测器，可把要求从"归零"降为"进入 \(\Gamma_h\bar\Delta_a\) 带"，并令**目标带由最外层决定**：
> \(\varepsilon_i^{\mathrm{final}}=\max\{\vartheta_i\varepsilon_i,\ \Gamma_h\bar\Delta_a\}\)。此时"预设时间归零"应改写为"预设时间进入 \(\varepsilon^{\mathrm{final}}\) 带"，**不再是零**。这在理论上诚实，但会削弱论文的核心主张（预设时间精确跟踪），因此不建议作为主线。

### 8.2 \(\bar w\) 未知 ⟹ \(\vartheta_i\) 的数值不可知

定理 1 保证 \(\vartheta_i<1\)，但

\[
\vartheta_i\to1\quad(c_i\to\infty),\qquad c_i=\frac{\bar w^2}{2k_{2i}\varepsilon_i^2}.
\]

若 \(\bar w\) 未知，只能说"进带"，**不能量化"进到带内多深"**，也无法定量校核命题 1 的条件 \(M_h\|z(T_1)\|\le\min_i\varepsilon_i\)。
→ 工程上仍需一个**保守的 \(\bar w\) 估计**（可用于整定与验收，但不进入稳定性证明），这与论文假设 `as:disturbance`（"上界未必已知"）在逻辑上是相容的：**定理不依赖 \(\bar w\)，验收依赖**。

### 8.3 饱和下严格包络不可保

(4.4) 的漏斗项 \(-\theta/(1-\theta^2)\to-\infty\) 当 \(|\theta|\to1\)。若 \(\tau_{\max}\) 有限且初始点靠近包络（或 \(\rho_0\) 取太小），实际施加力矩被截断，(4.6) 中的"精确相消"失效，定理 1 的证明断裂。严格处理需要把饱和写成附加匹配扰动 \(\Delta_\tau\) 并假定 \(\|\Delta_\tau\|\) 有界，此时定理 1 的结论形式不变（\(w\to w+M_e^{-1}\Delta_\tau\)），但**要求 \(\bar w\) 包含饱和裕度**，并显式检查
\(\bar w_\text{total}\) 是否使 \(\vartheta_i\) 仍 \(<1\)（恒成立）且使 \(\vartheta_i\varepsilon_i\) 满足任务指标（不恒成立）。→ 建议在论文中以"实际包络"表述：\(|e_i|\le\vartheta_i\rho_i+\eta\)，\(\eta\) 由饱和裕度给出。

### 8.4 Gramian 病态 ⟹ 定理 2 的实际 \(\Delta_K(h)\)

论文实测 \(n=7\) 时 \(\operatorname{cond}(W_c)=4.91\times10^{11}\)，且 PDF 与 PD 的终端误差一致（\(10^{-9}\ \mathrm{rad}\) 量级）。这说明**有限精度下 \(\Delta_K(h)\neq0\)**，若只有 \(\|\Delta_K(h)\|\le\varepsilon_\Delta\)，则

\[
\|z_{k+1}\|\le\varepsilon_\Delta\|z_k\|+\Gamma_h\bar\Delta_a
\ \Longrightarrow\
\limsup\|z_k\|\le\frac{\Gamma_h\bar\Delta_a}{1-\varepsilon_\Delta},
\]

且若 \(\varepsilon_\Delta\ge1\)，定理 2/3 的"归零"**完全失效**（连有界性都无法从该递推得到）。→ 因此离线第 5 步"校验 \(\|\Delta_K(h)\|\)"是**算法可靠性的必要环节**，而不是可选的数值检查。

### 8.5 结论汇总

| 要求 | 无观测器（本文件） | 备注 |
|---|---|---|
| 预设时间 \(T_1\) 进带 | ✅ 严格（定理 1 + 推论 1） | 不需要 \(\bar w\) 已知 |
| 再经 \(2h\) 归零 | ⚠️ 仅当 \(d\) 已知 | §8.1 |
| 全程包络 | ⚠️ 需 \(\varepsilon_\Delta\approx0\)、\(M_h\|z(T_1)\|\le\min\varepsilon_i\)、无饱和 | 命题 1 |
| 工程可用性 | 需 \(d\) 的解析模型 | 实际场景通常没有 |

---

## 9. 数值验证清单（最小集）

1. **包络图**：七关节 \(e_i(t)\) 与 \(\pm\rho_i(t)\)、\(\pm\varepsilon_i\) 同轴；标注 \(T_1\)、\(T_1+2h\)；统计 violation time \(=\int\mathbb 1\{|e_i|>\rho_i\}\mathrm dt\)（应为 0）。
2. **进带验证**：实测 \(|e_i(T_1)|/\varepsilon_i\) vs 理论 \(\vartheta_i\)（用假定的 \(\bar w\) 计算）：理论应为保守上界。
3. **归零验证**：理想算例（\(d\) 已知）下 \(|z(T_1+2h)|\) 应为机器精度；若非，检查 \(\|\Delta_K(h)\|\)。
4. **阶段 II 包络校验**：实测 \(\max_{t\in[T_1,T_1+2h]}|e_i|/\varepsilon_i\) vs 命题 1 的 \(M_h\|z(T_1)\|/\varepsilon_i\)。
5. **\(\bar w\) 扫描**：\(\bar w\in\{0,10^{-4},10^{-3},10^{-2}\}\)，画 \(\vartheta_i\) 与实测进带深度，验证"\(\bar w\uparrow\Rightarrow\vartheta\uparrow\to1\)"。
6. **退化检查**：若把定理 1 的 PPC 换成纯 PD（\(K_0\)），同一算例的 \(|e(T_1)|\) 应显著更大，且无包络保证。
