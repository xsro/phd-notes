# 论文创新方向试验记录

本目录用于记录当前论文可能采用的创新方向。暂不修改主论文和仿真代码，先分别保存理论构思、关键公式、证明路线、实现要求和风险。

## 文件索引

1. [`01-robust-prescribed-time-bound.md`](01-robust-prescribed-time-bound.md)：残余扰动下的预设时间实际有界性理论，当前最推荐。
2. [`02-uncertain-model-robust-compensation.md`](02-uncertain-model-robust-compensation.md)：名义模型下的鲁棒动力学补偿。
3. [`03-coupling-aware-pdf.md`](03-coupling-aware-pdf.md)：考虑自由漂浮耦合和广义雅可比退化的构型相关 PDF。
4. [`04-torque-constrained-pdf-design.md`](04-torque-constrained-pdf-design.md)：面向力矩峰值和 Gramian 条件数的 PDF 增益设计。
5. [`05-time-delay-co-design.md`](05-time-delay-co-design.md)：预设时间、延迟和控制峰值的协同设计。

## 推荐推进顺序

优先推进方向 1，再结合方向 4 或方向 5。这样能够在不改变现有控制结构的前提下，形成：

> 理想单周期零化 + 非理想条件下实际误差界 + 面向数值条件和执行器约束的参数设计。

方向 2 可作为鲁棒性扩展，方向 3 理论价值较高但需要更多完整自由漂浮动力学和时变系统分析。