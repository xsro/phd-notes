# 方向六：预设性能控制（PPC）与 PDF 的结合

## 1. 目标

当前论文的误差刻画有两条结论：

- **定理 3（理想）**：\(e(t)=0,\ t\ge T_o+2h\)，但与"到达前的路径"无关；
- **定理 4 / 命题 1（非理想）**：\(t\ge T_o+2h\) 后有 \(\|z\|\le\Gamma_h\bar\Delta_r/(1-\varepsilon_\Delta)\)。

也就是说，**\(t<T_o+2h\) 的瞬态只有 Phase I 的 ISS 界**（式 `phaseI_ISS`），没有任何"预设"含义：

\[
\|z(t)\|\le\kappa e^{-\lambda t}\|z(0)\|+\frac{\kappa}{\lambda}\sup_{0\le s\le t}\|\varepsilon_2(s)\|,
\]

这个界随 \(\|z(0)\|\)、随观测峰值恶化，且不是设计者事先指定的。

**PPC 恰好补这一块**。本方向的目标是给出"预设包络 + 预设时间"的双重预设结论：

\[
|e_i(t)|<\rho_i(t),\ \forall t\ge 0
\qquad\text{且}\qquad
e(t)=0,\ \forall t\ge T_o+2h.
\]

## 2. 两种机制为什么正交，以及冲突在哪

| | 约束"何时归零" | 约束"路径/超调" | 依赖 |
|---|---|---|---|
| PDF | ✅ 硬上界 \(T_o+2h\)，与 \(z(0)\) 无关 | ❌ 只有区间界 | 线性周期时滞结构 |
| PPC | ❌ 指数型只能渐近（除非用有限时刻可达性能函数） | ✅ 显式包络 \(\rho(t)\) | 非线性时变误差变换 |

二者在**语义上正交**，因此可以组合；冲突只出现在**结构层面**：

1. 定理 2 的核心是 \(\Delta_K(h)=0\)，这依赖闭环为 \(\dot z=A_cz-BK_c(t)z(t-h)\) 的**线性、周期、时滞**结构；
2. PPC 的 \(e=\rho(t)S(\varepsilon)\) 是**非线性且时变**的微分同胚，代入后系统变成非线性泛函微分方程，\(\Delta_K(h)\) 失去定义，定理 2/3/命题 1 的链条全部作废；
3. 若对延迟状态也用同一 \(\rho\)，会额外引入 \(\rho(t)\) 与 \(\rho(t-h)\) 不一致产生的 \(\dot\rho\,z(t-h)/\rho\) 型强迫项，直接破坏 Gramian 反解的前提。

**但两者有一个天然的"时间窗口错位"**，这是所有可行方案的共同基础：

- 观测器在 \(T_c\le T_o\) 内收敛，故 \(\varepsilon_2(t)\) 在 \([T_c,T_o]\) 已趋零；
- PDF 在 \(t=T_o\) 才激活（两阶段时序见 `04-controller.tex` §4.4）。

因此**PPC 只需在 \([0,T_o)\) 工作，PDF 只在 \([T_o,\infty)\) 工作**，两机制的有效区间几乎不重叠。这使分段方案的风险远低于"全程叠加"。

## 3. 路线 A（推荐）：两段式 PPC → PDF

### 3.1 有限时刻可达性能函数（与观测器调节函数同构）

取与式 `eq:observer_error` 中 \(\bar\xi(t)=\bar\xi_0(T_o-t)^2\) **同一函数族**：

\[
\rho_i(t)=
\begin{cases}
(\rho_{0i}-\rho_{\infty i})\left(1-\dfrac{t}{T_o}\right)^2+\rho_{\infty i}, & 0\le t<T_o,\\[6pt]
\rho_{\infty i}, & t\ge T_o,
\end{cases}
\qquad i=1,\dots,n .
\]

性质：

- \(\rho_i(0)=\rho_{0i}\)，\(\rho_i(T_o)=\rho_{\infty i}\) —— **在观测时间处精确到达**，与 PDF 的激活时刻对齐；
- \(\lim_{t\to T_o^-}\dot\rho_i=0\)，故 \(\rho_i\in C^1\)；
- \(\lim_{t\to T_o^-}\ddot\rho_i=2(\rho_{0i}-\rho_{\infty i})/T_o^2\neq 0\)，故 \(\rho_i\notin C^2\) —— 这一条会在 §3.4 处理；
- 避免了指数型性能函数"永远到不了 \(\rho_\infty\)"与 PDF 精确零化的语义冲突。

### 3.2 Phase I 的 PPC 律（在已线性化的二阶通道上）

由引理 1，Phase I 的通道为精确双积分器 + 观测残差：

\[
\ddot e_i=\nu_i-\Delta_{a,i}+\hat\Delta_{a,i}=\nu_i+\varepsilon_{2,i},
\]

是**逐关节解耦**的标量二阶系统。取变换

\[
e_i(t)=\rho_i(t)S(\varepsilon_i),\qquad
S(\varepsilon)=\frac{2}{\pi}\arctan\varepsilon\ \ \bigl(S'>0,\ S'(\varepsilon)=\tfrac{2}{\pi(1+\varepsilon^2)}\bigr),
\]

反解 \(\varepsilon_i=\tan\!\bigl(\tfrac{\pi}{2}\xi_i\bigr),\ \xi_i=e_i/\rho_i\)。由

\[
\ddot e_i=\ddot\rho_iS_i+2\dot\rho_iS_i'\dot\varepsilon_i+\rho_iS_i''\dot\varepsilon_i^2+\rho_iS_i'\ddot\varepsilon_i=\nu_i+\varepsilon_{2,i}
\]

令变换后系统为 \(\ddot\varepsilon_i=-k_{1i}\varepsilon_i-k_{2i}\dot\varepsilon_i\)，得显式控制输入

\[
\boxed{\;
\nu_i=\rho_iS_i'\bigl(-k_{1i}\varepsilon_i-k_{2i}\dot\varepsilon_i\bigr)
+\ddot\rho_iS_i+2\dot\rho_iS_i'\dot\varepsilon_i+\rho_iS_i''\dot\varepsilon_i^2
\;}
\]

其中 \(\dot\varepsilon_i=(\dot e_i-\xi_i\dot\rho_i)/(\rho_iS_i')\) 只用可测的 \(e_i,\dot e_i\) 与已知的 \(\rho_i,\dot\rho_i\)，**不需要加速度、不需要模型、不需要扰动上界**。这正是 PPC 相对 PDF 的独立性优势。

PPC 不需要反步、不需要神经网络、不需要 RBF —— 因为引理 1 已经把通道压成了严格反馈的自由二阶线性形。**这是本方向相比一般空间机械臂 PPC 文献的最大实施优势**（文献综述里 Yao 2021、Zhou 等都要处理非线性耦合）。

### 3.3 包络保持命题（候选，需新证明）

记 \(\varepsilon_i\) 状态的能量 \(V_i=\tfrac12(\varepsilon_i^2+\dot\varepsilon_i^2)\)。由上式，

\[
\dot V_i=-k_{1i}\varepsilon_i^2-k_{2i}\dot\varepsilon_i^2
+\frac{\dot\varepsilon_i\,\varepsilon_{2,i}}{\rho_iS_i'}
=-k_{1i}\varepsilon_i^2-k_{2i}\dot\varepsilon_i^2
+\frac{\pi(1+\varepsilon_i^2)}{2\rho_i}\,\dot\varepsilon_i\,\varepsilon_{2,i}.
\]

障碍项随 \(|\varepsilon_i|\) 二次增长（\(\propto(1+\varepsilon_i^2)/\rho_i\)），而阻尼项为二次 —— 因此**只要 \(k_{1i},k_{2i}\) 与 \(\rho_{\infty i}\) 满足相容性不等式，\(V_i\) 就有一致上界**，从而 \(\varepsilon_i\) 有界，于是

\[
|\xi_i|=|S(\varepsilon_i)|<1
\ \Longrightarrow\
|e_i(t)|<\rho_i(t),\qquad \forall t\in[0,T_o).
\]

**候选命题（包络保持）**：若

\[
\text{(i)}\ |e_i(0)|<\rho_{0i};\qquad
\text{(ii)}\ \rho_{\infty i}\ \ge\ \frac{\pi}{2}\,\frac{|\varepsilon_{2,i}|_\infty}{k_{2i}}\,\Phi(k_{1i},k_{2i}),
\]

其中 \(\Phi(\cdot)\) 为某个由 \(k_1,k_2\) 决定的常数，则 \(|e_i(t)|<\rho_i(t)\) 对一切 \(t\ge 0\) 成立。

**注意**：条件 (ii) 是本方向的**核心新条件，也是最大风险**。它把 PPC 的"严格保包络"和论文假设 4 的"扰动上界未必已知"直接对立起来（见 §8 风险 1）。可行的弱化表述见 §6。

### 3.4 Phase II：无缝移交 PDF

\(t=T_o\) 时：

1. 由 §3.3，\(|e_i(T_o)|\le\rho_{\infty i}\)，即 \(z(T_o)\) 有**设计者指定的**界（而不是 ISS 界）；
2. 定理 2 对**任意有界初值**成立，不要求 \(z(T_o)\) 的形式，故直接适用；
3. 以局部时间 \(\vartheta=t-T_o\) 启动 \(K_c(\vartheta)\)，单周期零化给出 \(z=0,\ \vartheta\ge2h\)，回到绝对时间即 \(t\ge T_o+2h\)。

需要补一个**切换相容性引理**：\(\nu\) 由 \(\nu^{\mathrm{PPC}}\) 跳到 \(K_0z-K_c(0)z(t-h)=K_0z\)（因为 \(K_c(\vartheta)=0,\ \vartheta\in[0,h)\)），跳变量

\[
\|\Delta\nu(T_o)\|\le\|\nu^{\mathrm{PPC}}(T_o^-)-K_0z(T_o)\|,
\]

由于 \(\dot\rho_i(T_o^-)=0\) 且 \(\rho_i\) 在 \(T_o\) 处 C¹，该跳变主要来自 \(\ddot\rho_iS_i\) 项，量级 \(\sim2(\rho_{0i}-\rho_{\infty i})/T_o^2\)。对应力矩跳变 \(\Delta\tau=M_e\Delta\nu\)，必须与 \(20\,\mathrm{N\cdot m}\) 限幅比较。若不可接受，两种修补：

- 用 \(C^2\) 性能函数（如 \(\rho\) 用 \((1-t/T_o)^4\) 或 smooth bump）；
- 或在 \(t=T_o\) 前 \(2h\) 内对 \(\nu\) 做线性淡出（\(\nu=(1-\beta(t))\nu^{\mathrm{PPC}}+\beta(t)K_0z\)），但此时保包络结论需要重新证明。

## 4. 路线 B（最低风险）：PPC 只作用于外层参考

不动引理 1、定理 2–4，把 PPC 放在任务空间参考生成器上（式 `eq:task_space_velocity`）：

\[
\dot q_d=J_g^\#\bigl[\dot x_e^d-K_x\Psi(\varepsilon_x)\bigr],
\qquad
x_e-x_e^d=\rho_x S(\varepsilon_x).
\]

- 优点：主论文定理体系零改动，只新增"参考塑形"一节；
- 缺点：包络只对**指令**成立，实际 \(e\) 还叠加 PDF/PD 跟随误差和 Gramian 数值残差，故 \(\rho\) 必须额外留出 \(\Gamma_h\bar\Delta_r\) 的裕度，此时包络失去"紧"的意义，接近方向 1 的界；
- 定位：适合做"任务空间集成"（结论 §5 已列为未来工作）的配套小改动，不宜作为主创新。

## 5. 路线 D（新意最强、代价最高）：时变基线 + \(\Phi\)-加权 PDF

把 \(K_0\) 换成由 PPC 设计出的**时变**基线增益 \(K_0(t)\)（等价于 \(A_c(t)=A+BK_0(t)\) 时变），使前半周期就带包络性质，则定理 2 可整体推广：

令 \(\Phi(t,s)\) 为 \(\dot z=A_c(t)z\) 的状态转移矩阵，定义

\[
W_c=\int_h^{2h}\Phi(2h,s)BR_h(s)B^\top\Phi^\top(2h,s)\,\mathrm ds,
\]

\[
K_c(s)=R_h(s)B^\top\Phi^\top(2h,s)\,W_c^{-1}\,\Phi(2h,0)\,\Phi^{-1}(s-h,0),
\qquad s\in[h,2h].
\]

代入变分公式即得

\[
\int_h^{2h}\Phi(2h,s)BK_c(s)\Phi(s-h,0)\,\mathrm ds=\Phi(2h,0)
\ \Longrightarrow\
\Delta_K(h)=0,
\]

即**单周期零化在时变基线下仍然成立**，从而全程包络与预设时间可同时获得（无需分段）。

代价：

1. \(\Phi\) 无解析式，需在线/离线数值积分（每周期一次即可，\(A_c(t)\) 是 \(T_o+2h\) 周期的）；
2. \(W_c\) 的条件数会比现有 Remark 中 \(10^5\!-\!10^6\) 的量级**更差**（\(\Phi\) 的指数加权换成一般转移矩阵），而现有仿真已表明 \(n=7\) 时 PDF 退化为 PD；
3. 需要 \(\Phi^{-1}(s-h,0)\) 的存储与求逆，内存与计算量上升；
4. \(A_c(t)\) 的一致稳定性（而非逐点 Hurwitz）需要额外论证，通常要借助一致完全可控性与时变 Lyapunov 不等式。

**建议**：仅作为理论扩展写进"未来工作"，或用 \(n\le3\) 的数值例子单独说明可行性，不进入主线。

## 6. 参数映射（对齐现有仿真条件）

| 量 | 取值来源 | 建议值 |
|---|---|---|
| \(T_o\) | 观测器预设时间 | \(1\ \mathrm s\)（现有） |
| \(T_c\) | \(\le T_o\) | \(0.6\ \mathrm s\) |
| \(h\) | Gramian 条件数折衷 | 现有值 |
| \(T_{\mathrm{pre}}\) | \(T_o+2h\) | 现有 |
| \(\rho_{0i}\) | 初值 \(0.5^\circ\approx8.7\times10^{-3}\ \mathrm{rad}\) | \(1.2\times10^{-2}\ \mathrm{rad}\)（留 40% 裕度） |
| \(\rho_{\infty i}\) | \(\ge\Gamma_h\bar\Delta_r\)，与仿真 \(10^{-3}\ \mathrm{rad}\) 对齐 | \(1\times10^{-3}\ \mathrm{rad}\) |
| \(k_{1i},k_{2i}\) | 与 \(K_0\) 的极点对应 | 取 \(K_0\) 的两重极点配置 |
| 饱和 | \(20\ \mathrm{N\cdot m}\) | 约束 \(\rho_{\infty}\) 不能过小 |

关键：**\(\rho_{\infty i}\) 不能小于定理 4 的 \(\Gamma_h\bar\Delta_r\)**，否则 Phase II 一旦进入，误差立刻顶到包络边界 —— 这就是 §3.3 相容性条件的物理含义，也是"两个预设不能独立选取"的唯一硬约束。

## 7. 全程误差刻画（可与方向 1 合并成一张表）

| 时段 | 方向 1（仅有 PDF） | 方向 6（PPC + PDF） |
|---|---|---|
| \(t<T_o\) | ISS 界，依赖 \(\|z(0)\|\)、\(\sup\|\varepsilon_2\|\) | **\(\|e_i(t)\|<\rho_i(t)\)**（预设） |
| \(T_o\le t<T_o+2h\) | \(\|z\|\le\Gamma_h^*\bar\Delta_r\) | 定理 2 路径，无扰动时 \(z\to0\) |
| \(t\ge T_o+2h\) | \(\Gamma_h\bar\Delta_r/(1-\varepsilon_\Delta)\) | 同左（不可改进，受模型/饱和限制） |

结论：方向 6 只改第一行，但第一行正是现论文唯一的空白。

## 8. 最小仿真验证清单（暂不改主脚本）

1. **基线算例**：\([0,T_o)\) 用 §3.2 的 \(\nu^{\mathrm{PPC}}\)，\([T_o,\infty)\) 用原 PDF；画七关节的 \(e_i(t)\) 与 \(\pm\rho_i(t)\) 同轴曲线，统计 **violation time**。
2. **破包络临界扫描**：固定 \(\rho_0\)，逐步减小 \(\rho_\infty\)，找出开始破包络/开始饱和的临界值，与 §3.3 相容性不等式对比。
3. **\(\varepsilon_2\) 灵敏度**：人为放大观测器残差（\(T_c\) 取大/小），验证 PPC 对扰动上界的依赖。
4. **切换冲击**：测 \(t=T_o\) 处 \(\|\Delta\tau\|\)，与 \(20\ \mathrm{N\cdot m}\) 比较，验证 §3.4 的跳变估计。
5. **是否值得 PDF**：\(n=7\) 时 \(\mathrm{cond}(W_c)\) 与 \(\|\Delta_K(h)\|\) 实测值，配合包络画出"PDF 段无增益"的证据。
6. **对比指标**：除终端误差、RMS、\(t_s\)、\(\max\|\tau\|\)、饱和率外，增加"包络违反时长"和"最大越界比 \(\max|e_i|/\rho_i\)"，用于与 FONTSM/FOSM 对比。

## 9. 风险

1. **\(\rho_\infty\) 下界条件与假设 4 冲突**（最主要）。假设 4 明确"上界未必已知"，而严格保包络需要 \(\bar\Delta_r\) 的先验信息。两条出路：
   - 改成**实际保包络**：证明 \(|e_i|\le\rho_i+\eta\)，\(\eta\) 由 \(\bar\Delta_r\) 决定，把严格性换成显式裕度（与方向 1 风格一致，推荐）；
   - 或用自适应/估计式 \(\hat\rho_\infty\)（引入额外动态，理论成本上升）。
2. **\(\rho\notin C^2\)**：本构造在 \(T_o\) 处 \(\ddot\rho\) 跳变，控制律含 \(\ddot\rho\) 项，必然产生力矩跳变。要么换 \(C^2\) 性能函数，要么接受一次跳变并给出界。
3. **变换增益发散与饱和矛盾**：\(|\xi|\to1\) 时 \(\nu\) 幅值发散，实际必然饱和；饱和后又破包络。必须显式讨论"饱和—包络—\(\rho_\infty\)"三角约束（可与方向 4 合并）。
4. **逐关节漏斗数量**：\(2n=14\) 维状态若全做包络，需 7 个位置漏斗 + 7 个速度漏斗，设计参数成倍增加；建议**只对位置误差做包络**，速度由 PPC 的 \(\dot\varepsilon\) 隐式约束，避免漏斗互相打架。
5. **审稿风险（最需注意）**：PDF 已给出"有限时间精确归零"，逻辑上强于 PPC 的"渐近包络"，容易被质疑"堆砌"。必须在文中把 PPC 的定位收窄为两件事：**(a) 补 \(t<T_o+2h\) 的瞬态空白；(b) 提供与执行器饱和/力矩峰值直接挂钩的设计旋钮**。绝不能说成"PDF 收敛慢，用 PPC 加速"。

## 10. 可进论文的定位表述

> 针对光滑周期延迟反馈在预设时间到达之前缺少瞬态约束的问题，本文利用计算力矩补偿后误差通道的精确解耦二阶线性结构，在观测阶段嵌入预设性能误差变换，构造与预设观测时间同构的有限时刻可达性能函数，使关节位置误差在整个暂态过程内严格位于可设计的性能包络之中；在 \(t=T_o\) 时刻包络精确到达稳态半径并无缝移交周期延迟反馈，由单周期零化在 \(T_o+2h\) 实现精确归零，从而给出"预设包络 + 预设时间"的双重预设结果。

## 11. 与其它方向的关系

- **与方向 1 互补且必须成对出现**：方向 1 给 PDF 段（\(t\ge T_o+2h\)）的界，方向 6 给 PPC 段（\(t<T_o\)）的界，合起来才是完整的"全程误差刻画"。建议一起推进。
- **与方向 4 强耦合**：\(\rho_\infty\) 的下界就是 \(\Gamma_h\bar\Delta_r\)，而后者由力矩峰值与 Gramian 条件数决定；方向 4 的"增益/饱和协同设计"在方向 6 下获得了直接的设计意义（用 \(\rho\) 反解允许的增益）。
- **与方向 5 冲突点**：方向 5 要缩短 \(h\)、\(T_o\) 以提高响应，但 \(\rho\) 的收缩速度被 \(T_o\) 限制（\((1-t/T_o)^2\) 的斜率 \(\propto1/T_o\)），\(T_o\) 过小会导致 \(\ddot\rho\) 跳变项爆炸。二者需要一个联合约束 \(2(\rho_0-\rho_\infty)/T_o^2\lesssim\Delta\tau_{\max}/\bar m\)。
- **与方向 2、3 正交**，可叠加。
