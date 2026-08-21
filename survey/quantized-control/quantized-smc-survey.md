# 量化控制与滑模控制结合的文献调研

> A Literature Survey on the Integration of Quantized Control and Sliding Mode Control
>
> 调研基于控制理论文献知识库（okb-assist / MCP 工具），参考文献见文末。
> 配套总报告见同目录 `report.md`。

---

## 摘要

滑模控制（Sliding Mode Control, SMC）因对匹配不确定性/扰动的强鲁棒性而被广泛应用；但其实现依赖对状态（或输出）的精确、连续获取。在网络化控制场景下，测量/控制量须经有限字长量化并通过受限带宽信道传输，由此引入**量化误差**与**量化器饱和**，会破坏理想滑模运动的产生条件，并使"到达"阶段出现"错层（mismatched）"问题。本报告聚焦"量化控制 + 滑模控制"的交叉方向，梳理其数学建模、核心困难与代表性方法：从基础的量化测量下事件触发 SMC（Bandyopadhyay & Behera）、量化反馈 SMC 的到达律方法（Zheng, Yu & Xue），到面向切换系统/网络攻击的动态量化与 T–S 模糊 SMC（Lian & Li；Ye et al.），并给出方法对比与趋势分析。

---

## 1. 为什么把量化与滑模结合

理想 SMC 由两段构成：
1. **到达阶段**：不连续控制律驱动状态在有限时间到达滑模面 $s(x)=0$；
2. **滑模运动**：状态被约束在滑模面邻域，对匹配扰动完全不敏。

但实际部署面临两类离散化：
- **事件触发（event-triggered）**：控制量仅在触发时刻更新，轨迹只能在滑模面*附近*有界（**实际滑模 / practical sliding mode**）；
- **量化（quantization）**：控制器/滑模面基于量化后的测量 $\hat x = q(x)$ 计算，引入 $s$ 的计算误差。

当二者叠加且信道带宽受限时，还需考虑**量化器饱和**（状态幅值超出量化范围导致信息丢失）。这些正是该交叉方向要解决的核心问题。

---

## 2. 数学建模

### 2.1 滑模基础

对 LTI 系统 $\dot x = Ax+Bu$（或离散 $\dot x$ 对应 $x_{k+1}=Ax_k+Bu_k$），取滑模面
$$
s(x) = Cx = 0 \quad (\text{或 } s = Gx \text{ 降阶形式}),
\tag{1}
$$
不连续控制律取
$$
u = -\rho\,\mathrm{sgn}(s) - K\,\mathrm{sat}(s/\phi),
\tag{2}
$$
其中 $\rho>0$ 为切换增益。Bandyopadhyay & Behera [B18] 给出滑模可存在的增益条件：
$$
\rho > |c^{\!\top} B|\,d_0,
\tag{3}
$$
其中 $d_0$ 为测量误差（含量化误差）的上界。该式表明：**量化误差上限直接抬升了保证滑模存在所需的最小切换增益**——这是量化与 SMC 结合的第一个定量后果。

离散时间下常用**到达律（reaching law）**描述滑模面的演化，例如 Gao 型：
$$
s(k+1) = (1-qT)\,s(k) - \eta T\,\mathrm{sgn}(s(k)), \qquad q,\eta>0,
\tag{4}
$$
保证 $s(k)\to 0$ 且穿越时符号翻转，产生实际滑模。

### 2.2 量化测量下的滑模面计算

控制器实际可用的滑模变量为
$$
\hat s = C\,\hat x = C\,q(x) = s(x) + C\bigl(q(x)-x\bigr).
\tag{5}
$$
量化误差 $e_q = q(x)-x$ 使 $\hat s$ 偏离真实 $s$，进而使触发/切换决策产生偏差，滑模运动被限制在边界层
$$
|s(x)| \le \Delta, \qquad \Delta = \mathcal{O}\!\bigl(\|C\|\cdot\|e_q\|_{\max} + \text{触发阈值}\bigr).
\tag{6}
$$

### 2.3 事件触发条件

Bandyopadhyay & Behera [B18] 将事件生成器置于传感器端（可直接获得真实测量用于触发判断），触发条件为
$$
\|e(t)\| \le \sigma\,\alpha,\qquad \sigma\in(0,1),
\tag{7}
$$
其中 $e(t)$ 为测量与上次传输值之差，$\alpha$ 为待设计参数；在该框架下可给出实际滑模存在的**充分条件**。

---

## 3. 核心困难

1. **量化误差抬升切换增益**（式 (3)）：误差越大，所需 $\rho$ 越大，易加剧抖振（chattering）。
2. **量化器饱和**：固定范围静态量化器在大幅度值下饱和，丢失符号信息，使 SMC 律"失明"。
3. **错层/不匹配（mismatched）控制**：事件触发采样与系统切换（切换系统）叠加，使控制作用与当前模态不匹配。
4. **率—鲁棒性折衷**：有限编码长度下，量化分辨率与可达鲁棒边界层相互制约。

---

## 4. 代表性文献与方法

### 4.1 量化测量下的事件触发 SMC（Bandyopadhyay & Behera, 2018）

专著 *Event-triggered Sliding Mode Control* 第 6 章专论"**量化状态测量下的事件触发 SMC**"[B18]：

- 假设量化器为**静态、固定饱和电平**；事件生成器位于传感器端，真实测量可用于触发判断；
- 先给出 LTI 系统的量化 SMC，详细分析滑模运动的稳定性与滑模阶段系统行为；
- 事件触发框架下给出控制律（式 (6.3)）的实际滑模存在**充分条件**；
- 数值显示触发式量化 SMC 信号（Fig. 6.3）。

**要点**：首次系统将"事件触发 + 量化测量"统一进 SMC 框架，指出稳态轨迹被约束在滑模流形邻域（实际滑模），且界仅取决于设计参数。

### 4.2 量化反馈 SMC 的到达律方法（Zheng, Yu & Xue, 2018）

Zheng, Yu & Xue [Z18]（*Automatica*, 91:126–135，被多篇文献引用）提出"**Quantized feedback sliding-mode control: An event-triggered approach**"：以**到达律**为核心，将量化反馈与事件触发协同设计，处理有限编码长度下的滑模到达问题。该文是该方向的奠基性工作之一（其结论被广泛引用，如 [L21]、[C24] 的参考文献均列之）。

### 4.3 事件触发 + 动态量化：切换系统（Lian & Li, 2021）

Lian & Li [L21]（*IEEE Trans. Automatic Control*）研究**含外部扰动与不确定非线性的切换线性系统**的 SMC，状态经事件触发采样并经有限带宽数字信道传输。核心贡献：

- 提出**事件触发机制（ETM）+ 有限信息 SMC 律**；
- 提出**离散时间水平集方法（discrete-time level-sets method）**设计**动态量化策略（DQP）**：当扰动上界已知时，DQP 可在**每个触发时刻避免量化器饱和**，无需在线检测；
- 给出**切换律与量化参数条件**保证状态轨迹收敛；并推广到扰动上界未知情形；
- 以单连杆机械臂为应用示例。

**要点**：用"动态量化 + 水平集"替代静态量化，从根本上化解饱和问题，是量化 SMC 从 LTI 走向切换/非线性系统的关键一步。

### 4.4 无人机/网络攻击下的量化 SMC（Ye et al., 2022）

Ye, Zhang, Cheng & Wu [Y22]（*IEEE Trans. Vehicular Technology*）研究**无人艇（UMV）在 DoS 攻击下的安全控制**：

- 海洋环境通信资源受限，在**传感器→控制器**与**控制器→执行器**双侧均采用**动态事件触发机制**；
- 触发输出数据经**对数量化器**量化后送远端控制站；
- 基于 **T–S 模糊**理论将非线性 UMV 建模，并将 **DoS 攻击 + 量化效应**统一为**混合切换 T–S 模糊系统**；
- 提出**基于观测器的 SMC 方案**，观测器增益与控制器增益通过求解一组**矩阵不等式（LMIs）**获得；
- 基准 UMV 仿真验证有效性。

同作者 Ye et al. [Y23] 进一步给出 UMV 在 DoS 下的**有限时间弹性 SMC**，揭示有限时间稳定（FTS）与攻击参数间的显式关系。

### 4.5 受限编码长度 + 自适应量化器（Li, Niu & Ho, 2022）

Li, Niu & Ho [LN22]（*IEEE Trans. Automatic Control*, 67(9):4738–4745，见于 [L24] 引用）提出"**Limited coding-length-based sliding mode control with adaptive quantizer's parameter**"：在**编码长度受限**下，令**量化器参数自适应调整**以匹配可用比特预算，在鲁棒性与通信代价间取得平衡。该方法将"编码长度"作为显式约束纳入 SMC 设计，是数据率约束视角下的代表性工作。

---

## 5. 方法对比

| 文献 | 系统类 | 量化方式 | SMC 设计 | 核心手段 | 主要结果 |
|---|---|---|---|---|---|
| Bandyopadhyay & Behera [B18] | LTI | 静态、固定饱和 | 事件触发 SMC | 事件条件 (7) + 充分条件 | 实际滑模存在 |
| Zheng, Yu & Xue [Z18] | 一般 | 量化反馈 + 事件触发 | 到达律方法 | 量化—触发协同 | 有限编码下到达 |
| Lian & Li [L21] | 不确定切换 | 动态量化 (DQP) | 事件触发 SMC | 水平集法避饱和 | 无饱和、收敛 |
| Ye et al. [Y22] | 非线性 UMV | 对数 + 双侧事件触发 | 观测器 SMC | T–S + 切换 + LMI | DoS 下稳定 |
| Li, Niu & Ho [LN22] | 一般 | 自适应量化器 | 编码长度受限 SMC | 量化参数自适应 | 率—鲁棒折衷 |

---

## 6. 研究趋势与开放问题

1. **从静态到动态/自适应量化**：静态量化易饱和（[B18]），动态量化（[L21]）与自适应量化参数（[LN22]）成为主流，以在有限比特下兼顾大幅值与高精度。
2. **与事件触发深度协同**：量化与事件触发共同决定实际滑模边界层（式 (6)），二者联合调度是降低通信负载的关键。
3. **面向安全/网络化场景**：DoS 攻击、T–S 模糊、切换系统下的量化 SMC（[Y22]、[Y23]）是应用热点。
4. **有限时间/固定时间性能**：在量化约束下实现有限时间滑模到达仍是难点（齐次性、齐次量化工具可借鉴总报告 §5.3）。
5. **开放问题**：给定编码长度下量化 SMC 的**最优率—鲁棒边界**刻画、多回路/大规模网络下的分布式量化 SMC、以及量化误差与抖振抑制的联合优化，仍待深入。

---

## 7. 结论

量化与滑模控制的结合，本质是在"鲁棒性（SMC 强项）"与"信息率（量化硬约束）"之间取得平衡。研究脉络清晰地从"静态量化 + 事件触发下的实际滑模分析"（Bandyopadhyay & Behera）与"到达律协同设计"（Zheng et al.），演进到"动态量化避免饱和"（Lian & Li）、"网络攻击/模糊切换下的观测器 SMC"（Ye et al.）以及"编码长度受限的自适应量化 SMC"（Li, Niu & Ho）。未来方向将更聚焦于率—鲁棒最优性、有限时间性能与大规模分布式场景。

---

## 参考文献

[B18] B. Bandyopadhyay and A. K. Behera, *Event-triggered Sliding Mode Control*. Springer, 2018.（第 6 章：Event-Triggered Sliding Mode Control with Quantized State Measurements）

[Z18] B.-C. Zheng, X. Yu, and Y. Xue, "Quantized feedback sliding-mode control: An event-triggered approach," *Automatica*, vol. 91, pp. 126–135, 2018.

[L21] J. Lian and C. Li, "Event-triggered sliding mode control of uncertain switched systems via hybrid quantized feedback," *IEEE Trans. Automatic Control*, 2021.

[Y22] Z. Ye, D. Zhang, J. Cheng, and Z.-G. Wu, "Event-triggering and quantized sliding mode control of UMV systems under DoS attack," *IEEE Trans. Vehicular Technology*, 2022. (DOI: 10.1109/TVT.2022.3175726)

[Y23] Z. Ye, D. Zhang, C. Deng, H. Yan, and G. Feng, "Finite-time resilient sliding mode control of nonlinear UMV systems subject to DoS attacks," 2023.

[LN22] J. Li, Y. Niu, and D. W. C. Ho, "Limited coding-length-based sliding mode control with adaptive quantizer's parameter," *IEEE Trans. Automatic Control*, vol. 67, no. 9, pp. 4738–4745, 2022.

[L24] J. Li, X.-G. Yan, and Y. Niu, "Finite-time boundedness of interconnected system using decentralized output-feedback sliding mode control," *IEEE Trans. Automatic Control*, 2024.（引用 [LN22]）

[C24] J. Cheng, J. Xu, J. H. Park, H. Yan, and D. Zhang, "Protocol-based SMC for singularly perturbed switching systems with sojourn probabilities," *Automatica*, 2024.（引用 [Z18]）

---

*本报告由控制理论文献知识库（okb-assist）检索生成，覆盖 2018–2024 年量化控制与滑模控制结合的代表性文献。标注 [Z18]、[LN22] 的文献为知识库内其他文献所引用（本身未直接入库），引用时建议核对原文。*
