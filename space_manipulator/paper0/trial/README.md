# 论文创新方向试验记录

本目录用于记录当前论文可能采用的创新方向。暂不修改主论文和仿真代码，先分别保存理论构思、关键公式、证明路线、实现要求和风险。

## 文件索引

1. [`01-robust-prescribed-time-bound.md`](01-robust-prescribed-time-bound.md)：残余扰动下的预设时间实际有界性理论，当前最推荐。
2. [`02-uncertain-model-robust-compensation.md`](02-uncertain-model-robust-compensation.md)：名义模型下的鲁棒补偿（v2）。核心结论：惯性误差是辅助输入上的**静态乘性增益**而非加性扰动，可在 \(\nu\) 上左乘 \(\hat B_o^{-1}\) 精确消除，不需鲁棒项；并给出零化残差与延迟的标度 \(e^{-\|\lambda_\ast\|h}\)。
3. [`03-coupling-aware-pdf.md`](03-coupling-aware-pdf.md)：考虑自由漂浮耦合和广义雅可比退化的构型相关 PDF。
4. [`04-torque-constrained-pdf-design.md`](04-torque-constrained-pdf-design.md)：面向力矩峰值和 Gramian 条件数的 PDF 增益设计。
5. [`05-time-delay-co-design.md`](05-time-delay-co-design.md)：预设时间、延迟和控制峰值的协同设计。
6. [`06-ppc-pdf-hybrid.md`](06-ppc-pdf-hybrid.md)：预设性能控制（PPC）与 PDF 的分段结合，补足预设时间到达前的瞬态包络。与方向 1 互补，建议成对推进。
7. [`07-time-varying-baseline-pdf.md`](07-time-varying-baseline-pdf.md)：路线 D，时变基线 \(A_c(t)=A+BK_0(t)\) 下的 \(\Phi\)-加权零化 PDF。可作方向 3 的数学载体，并顺带给出两条可直接采纳的主论文改进。

## 推荐推进顺序

优先推进方向 1，再结合方向 4 或方向 5。这样能够在不改变现有控制结构的前提下，形成：

> 理想单周期零化 + 非理想条件下实际误差界 + 面向数值条件和执行器约束的参数设计。

方向 2 可作为鲁棒性扩展，方向 3 理论价值较高但需要更多完整自由漂浮动力学和时变系统分析。

方向 6 与方向 1 直接互补：方向 1 刻画 \(t\ge T_o+2h\) 的稳态界，方向 6 刻画 \(t<T_o\) 的瞬态包络，合并后形成完整的“全程误差刻画”，且与方向 4 的饱和/增益协同设计天然耦合。注意 PPC 的误差变换**不能**直接替换全时域的 PDF 辅助输入（会破坏单周期零化的线性周期时滞结构），只能按“观测段 PPC + PDF 段零化”分段使用。

方向 7（路线 D）的直接结论：**精确零化不要求 \(A_c(t)\) 以 \(2h\) 为周期，只有残余扰动下的 \(\Gamma_h\) 递推界才要求周期性**；且若把权重函数 \(R_h\) 强化为 \([h,2h]\) 上的 bump 函数（端点各阶导数为零），可在**不修改任何定理结论**的前提下消除延迟项激活时的力矩跳变。这两条建议直接回写主论文 §4.3.2。