# IEEE TAES — Correspondence 版本

## 文件

| 文件 | 说明 |
|---|---|
| `../paper_taes.tex` | TAES Correspondence 主文件（`\documentclass[journal]{IEEEtran}`） |
| `sec1_intro.tex` | 引言：相关工作三条线 + 延迟反馈动机 + 4 条贡献 |
| `sec2_model.tex` | 降阶动力学：动量约化、Schur 补、两条实现 Remark |
| `sec3_controller.tex` | 观测器（含预设定时变增益）、计算力矩、Gramian/PDF 增益 |
| `sec4_stability.tex` | 单周期零化定理、精确跟踪推论、实用界定理 |
| `sec5_simulation.tex` | 参数表、算例设置、结果表、3 组图、6 段讨论 |
| `sec6_conclusion.tex` | 结论与三条拓展 |
| `appendix.tex` | Appendix A：Gramian 正定、观测器误差系统、残余加速度的显式估计 |
| `bios.tex` | 作者简介模板（终稿时取消 `paper_taes.tex` 里的注释） |

## 与原完整版（`paper.tex`，11 页）的差别

**保留**：降阶模型的精确性、观测器接口、单周期零化定理、实用界、全部数值结果。

**压缩**：
- 引言 2 页 → 1 页（相关工作从 6 小节并为三条线，正文引用 50 → 24 篇）
- 第 II 节去掉动能推导、模型性质与若干中间方程，只留 (1)–(6) 与两条 Remark
- 第 III 节合并小节标题，观测器引理与 Gramian 引理移入 Appendix A
- 第 IV 节把“精确跟踪”降为推论，删去重复定义的 $\Gamma_h^\star$ 与工程化 $\varepsilon_\Delta$ 段落（后者并入 (23)）
- 第 V 节：参数表保留 8 行；结果表由 7 列压到 6 列；图由 3 组 12 面板压到 3 组 7 面板
- 结论 1 页 → 0.3 页

**结果**：6 页（含 24 条参考文献与附录 A），无 overfull box，`latexmk` 单次通过。

## 编译

```bash
cd space_manipulator/paper0en
latexmk -pdf paper_taes.tex
```

## 投稿前 checklist（TAES）

- [ ] 题名/摘要在 250 词内（当前 ~190 词），Index Terms 按字母序 ✓
- [ ] 首页脚注补全：投稿日期、基金号、通讯作者、单位与邮箱
- [ ] 若期刊要求匿名，删除 `\thanks` 中的单位/邮箱，改用 "Author details omitted for review"
- [ ] 图：录用后提交矢量 PDF（`figures/*.pdf` 已是矢量）或 300 dpi PNG
- [ ] 作者简介：取消 `paper_taes.tex` 末尾 `\input{taes/bios}` 的注释并填写
- [ ] Supplementary：如需压到 6 页以内，把 `appendix.tex`（0.6 页）移入补充材料；TAES 允许补充材料不计入页数
- [ ] Cover letter 里明确两点：
  1. 与 Zhou, Michiels & Chen (TAC 2022) 的区别 —— 从线性时滞系统推广到受约束的 Lagrangian 机械系统（精确降阶 + 观测器接口 + 实用界 + 条件数设计准则）；
  2. 无实验的正面表述 —— 目标是给出可复现的数值基准（参数、$\pm20$ N·m 限幅、调节时间、峰值力矩、饱和占空比、基座漂移全部公开），实验验证列为后续工作。
