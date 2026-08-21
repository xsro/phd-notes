# Comment on Cai et al. (2016) — Cooperative Output Regulation via Adaptive Distributed Observer

**Paper:** "The adaptive distributed observer approach to the cooperative output regulation of linear multi-agent systems"  
**Authors:** He Cai, Frank L. Lewis, Guoqiang Hu, Jie Huang  
**Venue:** Automatica / preprint (2016)

---

## 一、符号与排版问题

1. **LaTeX 格式混乱**（已修复，但原文本中大量存在）：
   - `${ \dot { x } } _ { i }$`、`$\bar { \bar { \mathcal { V } } }$`、`$\bar { \epsilon } \bar { P } F + \epsilon F ^ { T } \bar { \bar { P } } = - I _ { n }$` 等写法严重影响可读性。
   - `\cal` 与 `\mathcal` 混用，`\Re` 未定义为 `\operatorname{Re}`。
2. **符号不一致**：
   - 同一矩阵在不同位置使用不同记号（如 $\bar{Q}_1$ 与 $\hat{Q}_1$、$\bar{q}_1$ 与 $\hat{q}_1$）。
   - 下标混乱：$q_1, q_3, q_2$ 同时出现，缺乏明确对应关系。
3. **维度标注错误**：
   - Lemma 1 中写 $\bar{P} \in \mathcal{R}^{\bar{n} \times n}$，但 $\bar{n}$ 从未定义，应为 $n$。

---

## 二、Lemma 1 的证明存在严重逻辑漏洞

Lemma 1 讨论系统 $\dot{x} = \epsilon F x + F_1(t)x + F_2(t)$ 在 $F_1, F_2$ 指数衰减时 $x \to 0$ 的性质。

- **循环论证**：为证明 (7) 全局指数稳定，引用了 Khalil 的 Lemma 9.5 和 Corollary 9.1，但这些引理本身依赖 Lyapunov 函数的构造。论文虽构造了 $V_P = x^T \bar{P} x$，却未完成对 $\dot{V}_P$ 的完整估计，直接跳至更复杂的结论。
- **ISS 推理不严谨**：先声称系统关于 $F_2$ 是 ISS 的，再称"由于 $\dot{x} = \epsilon F x$ 指数稳定，系统关于 $F_4(t) = F_1(t)x + F_2(t)$ 也是 ISS 的"。但 $F_4$ 依赖于 $x$，不能直接套用标准 ISS 结论，需要小增益定理或 Gronwall 不等式等额外论证。
- **定理引用堆砌**：连续引用 Khalil (2002) Theorem 4.12 和 Chen & Huang (2015) Theorem 2.7，未说明这些定理如何具体应用于当前系统。

**更简洁的证法**：直接用 $V = x^T P x$（$P$ 满足 $P(\epsilon F) + (\epsilon F)^T P = -I$），估计
$$\dot{V} \le -\|x\|^2 + 2\|P\|\|F_1\|\|x\|^2 + 2\|P\|\|x\|\|F_2\|$$
利用 $F_1, F_2$ 的指数衰减性完成证明，无需引入时变 $\bar{Q}_1(t)$ 等复杂构造。

---

## 三、Lemma 2 第 (ii) 部分：leader 状态增长时的处理不充分

- 推导 $\tilde{\eta}$ 的动态方程 (11) 时出现项 $\tilde{S}_d (1_N \otimes v)$。论文声称即使 $S$ 有正实部特征值（导致 $v(t)$ 指数增长），只要 $\mu_1 \alpha_H > \gamma$（$\gamma = \max \operatorname{Re}(\sigma(S))$），该项仍指数衰减。
- **问题**：该结论仅在 $\|\tilde{S}_i(t)\| \le \beta e^{-\mu_1 \alpha_H t}$ 且 $\|v(t)\| \le C e^{\gamma t}$ 时成立。但论文只给出了 $\tilde{S}_i$ 的**渐近**衰减率，未给出一致的指数界（即未证明存在与初始条件无关的 $\beta$）。对于时变系统，仅凭渐近衰减率不足以保证乘积衰减——需要一致指数稳定性。

---

## 四、Lemma 3：证明过程存在循环依赖和维度混淆

- **循环论证**：证明 $\bar{x}$ 有界时，论文称"$\bar{x}$ 有界 by Remark 2"，但 Remark 2 本身依赖于 Lemma 1 的结论，而 Lemma 1 正是待证命题依赖的基础。
- **分解不严谨**：对 $A$ 进行正交分解 $AP = [\bar{A} \;\; 0]$ 是可能的（类似秩分解），但论文未说明如何构造这样的 $P$，也未验证 $\bar{A}$ 的列满秩性质。
- **$d(t)$ 的表达式不完整**：$d(t)$ 中包含 $(A^T A - \mathcal{A}(t)^T \mathcal{A}(t)) P \bar{x}$，但 $\mathcal{A}(t)^T \mathcal{A}(t)$ 展开后有 $A^T \tilde{A} + \tilde{A}^T A + \tilde{A}^T \tilde{A}$ 等交叉项，论文未明确说明这些项如何归入 $d(t)$。

---

## 五、Lemma 4：向量化公式**符号歧义**（非错误）

经过重新推导，之前指出的"维度不匹配"问题**并不成立**——公式本身在数学上是正确的。问题出在**符号歧义**：

- 论文将调节器方程 (3) 写为：
  $$\begin{bmatrix} I_{n_i} & 0 \\ 0 & 0 \end{bmatrix} \begin{bmatrix} X_i \\ U_i \end{bmatrix} S - \begin{bmatrix} A_i & B_i \\ C_i & D_i \end{bmatrix} \begin{bmatrix} X_i \\ U_i \end{bmatrix} = \begin{bmatrix} E_i \\ F_i \end{bmatrix}$$
  其中块矩阵 $\begin{bmatrix} I_{n_i} & 0 \\ 0 & 0 \end{bmatrix}$ 的维度**未明确说明**。

- **正确解释**：该块矩阵应为 $(n_i+p_i) \times (n_i+m_i)$，即：
  $$\begin{bmatrix} I_{n_i} & 0_{n_i \times m_i} \\ 0_{p_i \times n_i} & 0_{p_i \times m_i} \end{bmatrix}$$
  这样第一项 $S^T \otimes M$ 和第二项 $I_q \otimes N$ 的维度均为 $q(n_i+p_i) \times q(n_i+m_i)$，**可以相减**。

- **验证**：展开后得到 $X_i S - A_i X_i - B_i U_i = E_i$ 和 $-C_i X_i - D_i U_i = F_i$，与原始调节器方程一致。

**实际问题**：论文未标注零块的具体维度，读者容易误认为 $M$ 是 $(n_i+m_i) \times (n_i+m_i)$ 方阵，从而得出"维度不匹配"的错误结论。建议明确写为：
$$M_i = \begin{bmatrix} I_{n_i} & 0_{n_i \times m_i} \\ 0_{p_i \times n_i} & 0_{p_i \times m_i} \end{bmatrix}$$

**结论**：向量化公式本身正确，但符号不规范，需要修正表述而非修改数学。

---

## 六、Theorem 1/2 证明中的小问题

- **Theorem 1**：$\tilde{K}_{\eta i}(t) v$ 的衰减性依赖于 $v(t)$ 的增长与 $\tilde{K}_{\eta i}(t)$ 衰减的比较。论文只说明了 $\tilde{K}_{\eta i}(t) \to 0$ 指数衰减，但未明确说明 $v(t)$ 的增长界（多项式 vs 指数）。对于 ramp 信号这没问题，但论文声称适用于一般信号（包括 $S$ 有正实部的情况），则需要更仔细的论证。
- **Remark 4**：$\mu_3$ 的条件 $\mu_3 \ge \frac{\mu_1 \alpha_H}{\min(\sigma(Q_i^T Q_i))}$ 缺乏推导，且与 Lemma 3 中条件 $\varepsilon > \frac{\alpha}{\min(\sigma(\bar{A}^T \bar{A}))}$ 的对应关系不清晰。

---

## 七、其他观察

1. **Example 中的 $S$ 矩阵**：$\begin{bmatrix} 0 & \theta \\ 0 & 0 \end{bmatrix}$ 有特征值 0 且亏损（defective），这正好验证了论文"不要求 semi-simple"的主张。但 Lemma 2 的分析中并未明确涵盖亏损特征值的情形（只讨论了实部符号），这是一个隐含的缺口。
2. **数值例子缺乏细节**：Figures 2-3 只给出了误差曲线，没有展示 $S_i(t)$ 的估计过程、$\zeta_i(t)$ 的收敛过程等关键瞬态信息。
3. **引用规范**：文中 "Su and Huang (2012a,b)" 的缩写形式在正式出版物中建议展开。

---

## 总结

| 严重程度 | 问题 | 位置 |
|---------|------|------|
| 🟠 较重 | Lemma 1 证明循环论证、ISS 推理不严谨 | Lemma 1 |
| 🟠 较重 | Lemma 3 证明依赖未验证的 Remark 2，循环依赖 | Lemma 3 |
| 🟡 中等 | Lemma 2 未建立一致指数界，乘积衰减论证不充分 | Lemma 2(ii) |
| 🟡 中等 | Lemma 4 块矩阵维度标注歧义（非错误，但易误导） | Lemma 4 |
| 🟡 中等 | 大量 LaTeX 格式和符号不一致 | 全文 |
| 🟢 轻微 | Theorem 1/2 证明细节缺失、Example 数据不完整 | Theorems & Sec 5 |

**核心结论**：论文的**思路有价值**（自适应分布式观测器 + 在线调节器方程求解），Lemma 4 的向量化公式在正确解释下是成立的，但符号标注不规范。更实质性的问题集中在 Lemma 1 的证明循环和 Lemma 3 的循环依赖上，建议作者优先简化并重构这两个证明。