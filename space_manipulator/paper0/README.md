# 基于漏斗预设性能与光滑周期延迟反馈的自由漂浮空间机械臂预设时间轨迹跟踪控制

## 项目概述

自由漂浮空间机械臂的关节空间轨迹跟踪控制。**当前主线（v2，2026-05）**为
"漏斗预设性能（PPC）—预设时间扰动观测（PTDO）—动力学补偿—周期延迟零化（PDF）"
两段式结构：观测阶段用漏斗型 PPC 保证预设包络并在预设时间 \(T_o\) 精确进带，
随后由 PDF 单周期零化在 \(T_o+2h\) 精确归零。论文已重写并编译为 **18 页** PDF
（`paper.pdf`，24 页），理论与数值验证均已完成（含 \(T_c\) 扫描）。

> 旧版（v1，28 页）为"全程 PDF + PTDO"结构，无预设性能阶段、且仿真整定
> \(T_c=5\,\mathrm{s}>T_o=1\,\mathrm{s}\) 违反理论前提。旧稿完整保留在
> commit `93c88b1`（`git show 93c88b1:sections/04-controller.tex` 等）。

---

## 目录结构

```
paper0/
├── paper.tex                  # 主文档（ctexart, amsthm 定理环境）
├── paper.pdf                  # 当前编译版本（24 页）
├── sections/                  # 论文章节（v2 重写）
│   ├── 01-introduction.tex    # 两条技术路线的互补与结构性障碍
│   ├── 02-modeling.tex        # 等效关节模型、误差通道、性能函数与误差变换、假设
│   ├── 03-controller.tex      # 三段时序、PTDO、漏斗型 PPC、Gramian PDF
│   ├── 04-analysis.tex        # 主定理、阶段 II 包络判据、非理想界、不可行边界
│   ├── 05-simulation.tex      # 数值验证：M1–M7 指标、4 张图、\(T_c\) 扫描与不可行性分析
│   └── 06-conclusion.tex
├── trial/                     # 技术路线试验记录（06/08/09 为 PPC+PDF 主线文档）
├── refs/                      # 参考文献
│   ├── local.bib
│   └── my_library.bib         # Zotero 引用
├── figures/                   # 旧版仿真图（v2 暂未引用，待重跑后替换）
└── simulation/                # MATLAB 仿真代码（文献 259 参数）
```

---

## 已完成内容

### 📄 论文 v2（当前，18 页）

| 章节 | 内容 |
|------|------|
| §1 引言 | 建模/PPC/预设时间/PDF 四条线；指出 PPC 与 PDF 语义正交但结构不可叠加；4 条编号贡献对应定理 |
| §2 建模与问题 | 完整→Schur 补→等效关节模型；引理 2.1（通道解耦，含 Kalman 秩证明）；定义 2.1/2.2（性能函数与误差变换）；引理 2.2（性能函数 C^{m-1} 光滑性）；问题 2.1；假设 2.1–2.6 |
| §3 控制器 | 唯一力矩律 + 三段时序；定理 3.1（PTDO，引用证明）；引理 3.1（阶段 I 残差显式界）；定理 3.2（包络保持 + 进带，4 步完整证明）；定理 3.3（单周期零化，自洽证明）；参数耦合约束 |
| §4 分析 | 引理 4.1（交接状态界）；**定理 4.1（双重预设主定理）**；命题 4.1（阶段 II 包络判据 \(M_hB_o\le\min\rho_{i\infty}\)，含 3 条结构耦合）；推论 4.1/4.2（指数贴合率、与纯 PD 的定量对照）；命题 4.2（非理想显式界）；§4.6 不可行边界 (a)(b)(c)；表 1 理论总结 |
| §5 数值验证 | 对象与参数；表 2 控制器参数；§5.3 离线判据（\(\operatorname{cond}(W_c)=4.9\times10^{11}\)、\(\lVert\Delta_K(h)\rVert=3.0\times10^{-5}\)、\(\Gamma_h=0.397\)、\(M_h^{\mathrm{ap}}=7.73\)）；表 3 M1–M7 四算例；图 1–3；§5.5 不可行性与工程边界 |
| §6 结论 | 5 条结论 + 5 条后续工作 |

### 🧪 技术路线文档（`trial/`）

| 文件 | 内容 |
|------|------|
| `06-ppc-pdf-hybrid.md` | 方案总览与三条路线比较 |
| `08-ppc-pdf-route-a.md` | 路线 A 严谨完整版（无观测器，要求 \(d\) 已知）：定理 1–3、不可行性记录 |
| `09-ppc-pdf-route-a-ptdo.md` | 加 PTDO 版：只测 \(\{q,\dot q\}\)，总预设时间 \(T_o+2h\)，\(T_c\le T_o\) 冲突分析 |
| `07-time-varying-baseline-pdf.md` | 路线 D：时变基线的 \(\Phi\)-加权零化（未来工作） |

### 🖥️ 仿真代码（全部基于文献 259 参数）

| 文件 | 方法 | 状态 |
|------|------|------|
| `ffsm259_model.m` | **DH 正解 + 质心雅可比求和惯性**，Schur 补得 `M_θm/M_r`，中心差分 Christoffel 得 `C_θm`，广义雅可比 `J` | ✅ 与 Robotics System Toolbox 交叉校核：正运动学 **2.3e−16 m**、固定基座质量矩阵 **8.5e−16**、性质 2（`Ṁ−2C` 反对称）**1.4e−16**、动量守恒 5e−14 |
| `main_dou_yue_259_compare.m` | **10 算例 × 2 种增益标定** — PD / PDF / PDF+FTDOB vs FONTSM / FOSM 的三种补偿器 | ✅ 运行成功（约 9–10 min） |
| `fonsm259_step.m` + `ftdob_step.m` | Eq (15)(35)(36)(37)(44)(50)(57) 逐条实现 | ✅ 与原文 Case (1)(2)(3) 对应 |
| `validate_ffsm259_model.m` | 模型正定性 / 工具箱对比 / 能量与动量守恒 | ✅ 全部通过 |
| `perf_rho.m` | 五次 smootherstep 性能函数 \(\rho,\dot\rho,\ddot\rho\)（论文式 11） | ✅ 新增 |
| `ppc_step.m` | 漏斗型预设性能律（论文式 21–22），含 \(\lvert\xi\rvert\to1\) 数值保护 | ✅ 新增 |
| `ppc_settings.m` / `ppc_theory.m` | 包络与增益构造；定理 3.2 的 \(\vartheta_i\)、\(B_o\) 计算 | ✅ 新增 |
| `pdf_periodian.m` | 单周期映射 \(\Delta_K(h)\)、\(\Gamma_h\)、\(M_h^{\mathrm{ap}}\) 与精确算子范数 \(M_{h,i}\) | ✅ 新增 |
| `main_ppc_pdf_verify.m` | **v2 主验证脚本**：4 算例 × 三段时序，输出 M1–M7 与 3 张图（约 25 s） | ✅ 新增 |
| `main_ppc_tc_sweep.m` | **\(T_c\) 扫描**：10 个 \(T_c/T_o\) × 2 种阶段 I 控制器，量化式 (42) 的三角约束（约 2 min） | ✅ 新增 |
| `sim_ppc_case.m` / `sim_metrics.m` | 单算例三段时序仿真 / 验收指标（主验证与扫描脚本共用） | ✅ 新增 |

### 📈 v2 数值验证结论（`main_ppc_pdf_verify.m`，约 25 s）

对象：文献 259 七自由度浮基机械臂，\(\tau_{\max}=20\,\mathrm{N\cdot m}\)，\(T_o=5\,\mathrm s\)，\(h=1\,\mathrm s\)（总预设时间 7 s），\(T_c=3\,\mathrm s\)。

| 指标 | PD+PTDO+PDF（消融 A） | **PPC+PTDO+PDF（本文）** | PPC 宽包络 | PPC 无扰动 |
|---|---|---|---|---|
| M1a 阶段 I 包络违反 [s·joint] | 6.24 | **0.00** | 0.002 | 0.00 |
| M1b 阶段 II 越界时长 | 0.00 | 0.32 | 0.00 | 0.00 |
| M2 进带深度（/ρ∞） | 0.078 | **0.059** | 0.201 | 0.0065 |
| M4 PDF 段越界比 | 0.60 | 1.24 | 0.69 | 0.013 |
| M5 稳态 ‖z‖∞ | 9.5e-4 | 2.0e-3 | 2.2e-3 | 5.8e-5 |
| M7 饱和占空比 | 8.6% | **0.93%** | 3.6% | 0.36% |

要点：阶段 I 的包络违反被完全消除（PD 在同一条包络下违反 6.24 s·joint、归一化比峰值 10.6）；PDF 段存在 1.24 倍的短暂越界，加宽误差带即消失（对应命题 4.1）；无扰动时终端误差 1.2e-8 rad。

### ⏱️ \(T_c/T_o\) 扫描（`main_ppc_tc_sweep.m`，约 2 min）

包络取与观测瞬态相容的 \(\rho_0=6\times10^{-2}\)、\(\rho_\infty=5\times10^{-3}\) rad，`env='feasible'`：

| \(T_c/T_o\) | 峰值 ‖ε₂‖ | 阶段 I 饱和 | 稳态 ‖z‖∞ | 终端 |e| |
|---|---|---|---|---|
| 0.2 | 1.09 | 73.4% | 5.9 | 2.3 |
| 0.3 | 0.295 | 25.0% | 8.2e-3 | 3.5e-5 |
| 0.6 | 0.295 | 13.9% | 1.2e-3 | 9.3e-6 |
| 1.0 | 0.295 | 6.0% | 9.5e-5 | 4.0e-6 |
| 2.0 | 0.295 | 0.0% | 7.7e-5 | 7.1e-7 |
| 5.0 | 0.295 | 0.0% | 9.2e-5 | 9.8e-7 |

结论：\(T_c/T_o\in[0.8,1]\) 为最优区间（既满足 \(T_c\le T_o\)，又使抖振可忽略）；\(T_c/T_o\le0.3\) 时抖振使残差恶化 3 个数量级，\(T_c/T_o=0.2\) 处 PPC 因障碍项在饱和下失效而失稳（劣于纯 PD）。另有一套 `env='tight'`（\(\rho_0=2|e_0|\)、\(\rho_\infty=10^{-3}\)）的扫描结果，用于对比"紧包络"下包络-瞬态相容性的边界。

## 环境要求

- **MATLAB R2025b** 或更高版本
- **Toolbox**: Control System Toolbox（`expm`）；Robotics System Toolbox 仅用于模型校核的可选对比（缺失时自动跳过）
- **LaTeX**: XeLaTeX + ctex + biber + biblatex（gb7714-2015 样式）
- **字体**: Songti SC, PingFang SC（macOS 预装）

## 仿真运行方法

### 主对比（文献 259 参数，约 9–10 分钟）

```bash
cd simulation
matlab -batch "main_dou_yue_259_compare"
```

输出：
- 终端：两张对比表（`kappa` 标定 / `kappa = 1` 原文增益）
- 图片：`figures/dou_yue_259_compare.png`
- 数据：`simulation/results_dou_yue_259.mat`

### 模型校核与参数核查（各约 1 分钟）

```bash
matlab -batch "validate_ffsm259_model"         # 对称性/正定性/工具箱对比/性质 2
matlab -batch "dou_yue_259_check_consistency"  # Table 1 一致性核查
matlab -batch "calib_kappa"                    # 增益标定扫描（约 2 分钟）
matlab -batch "tune_ptdo"                      # PTDO 参数扫描（约 3 分钟）
```

### 生成论文图片（约 1 分钟）

```bash
matlab -batch "make_paper_figures"
```

输出 `figures/` 下的三张论文图（按 6.3 in 版心宽、300 dpi 导出）：
`end_effector_joint_position_trajectory.png`（图 1）、
`joint_tracking_error_torque_summary.png`（图 2）、
`disturbance_observer.png`（图 3），并打印表 2 所需的全部数值。

### 单次求值

```matlab
addpath('simulation');
P = dou_yue_259_parameters();
S = ffsm259_model(P, P.q0, zeros(P.n,1));
S.Me        % 等效关节空间惯性 (Schur 补)
S.Cmat      % 等效科里奥利矩阵
S.J         % 广义雅可比 (3 x n)
S.pe        % 末端位置
```

### 论文编译

```bash
xelatex paper.tex
biber paper
xelatex paper.tex
xelatex paper.tex
```

---

## 已知问题

> 下列 1、2、4 条涉及已删除的旧脚本，保留结论供论文 §6 引用；
> 3 条（Gramian 病态）在文献 259 参数下同样成立（`cond(W_c) = 4.9e11`）。

### 1. ~~CRBA vs Jacobian 法 H 矩阵分歧~~ ✅ 已解决（旧模型）

**状态**: 已定位并验证；两套旧实现（`compute_floating_base_inertia.m` 的
Jacobian 求和法与 `spatial_crba.m` 的 CRBA）已**删除**，现统一由
`ffsm259_model.m` 实现，并经 Robotics System Toolbox 三方校核通过
（正运动学 2.3e−16 m，固定基座质量矩阵 8.5e−16）。

**当时的根因**: Jacobian 构造有两个 bug：基座速度未做 `adj(inv(T))` 变换、
`Ad_base_to_body` 方向取反。

### 2. PTDO 过补偿 🔴

**症状**: 启用 PTDO 后终端误差从 1.37e-04 rad 增至 3.33e-04 rad，力矩峰值从 259 Nm 增至 1921 Nm。

**原因**: 观测器增益 `π/(η·T_c)` = π/(0.3·1) ≈ 10.5 对 7-DOF 系统过大，导致 `delta_a_hat` 振荡发散。

**修复方向**: 降低 η → 0.1，增大 T_c → 2.0，引入饱和限幅 `|delta_a_hat| ≤ 0.5 rad/s²`。

### 3. PDF = PD（Gramian 数值退化）🟡

**症状**: PDF 与 PD 终端误差完全一致（均为 1.33e-09 rad）。

**原因**: MIMO 系统 Gramian 条件数 ~5×10⁵（14 维状态空间），正则化后 PDF 增益退化为等效比例反馈。

**论文处理**: 已在 §5 Gramian 病态性 Remark 中诚实讨论，指出 PDF 在 n ≤ 3 时效果最佳，高维 MIMO 需改用结构化延迟反馈参数化。

### 4. CRBA C(q,0) “不为零” 🟢（旧模型）

**实际**: 当时的测试代码只接收了 1 个输出参数（返回 H 而非 C），所以
`norm(C0)` 实际是 `norm(H)`。C(q,0) = 0 的性质已由数值 Christoffel 方法
的内部逻辑保证；现实现中 `ffsm259_model(P,q,0).Ce_qd` 可直接验证为零。

---

## 下一步建议

### ~~P0 — 实现 PPC 阶段并重跑仿真~~ ✅ 已完成（见 `main_ppc_pdf_verify.m`）

原计划内容（保留备查）：

论文 v2 的定理 3.2 与定理 4.1 需要按新三段时序重新验证，现有 `results_dou_yue_259.mat`
对应旧时序、不可用。最小工作量：

1. 新增 `perf_rho.m`：按式 (11) 计算 \(\rho_i,\dot\rho_i,\ddot\rho_i\)（\(m=5\)，\(T_o\) 可配）；
2. 新增 `ppc_step.m`：按式 (21)(22) 由 \((e,\dot e,\rho^{(\cdot)})\) 算 \(\nu^{\mathrm{PPC}}\)；
3. 在 `main_dou_yue_259_compare.m` 中把 \(t<T_o\) 的分支由 `K0*z` 换成 `ppc_step`，
   并把 \(T_o\) 提到 5 s（与 \(T_c\) 对齐，满足 \(T_c\le T_o\)）；
4. 按论文表 2 输出 M1–M7 七项指标（尤其包络违反时长与阶段 II 越界比 \(M_hB_o/\rho_{i\infty}\)）。

### P0' — 任务空间扩展
1. 把 `fonsm259_step.m` 的滑模面从关节误差换成末端误差
   `x_e^d - x_e` 与 `J_E⁺(ẍ^r - J̇θ̇)`（原文 Eq 48–51）
2. 叠加基座姿态稳定的二级任务（Eq 52–56，需 `H̄_w/H̄_m/C̄_b1/C̄_m1` 的 v/w 分块）
3. 与现有关节空间结果对比，验证两种任务表述的一致性

### ~~P1 — PTDO 参数整定~~ ✅ 已完成（`main_ppc_tc_sweep.m`）

原计划内容（保留备查）：
1. 降低观测器增益：η = 0.1, T_c = 2.0
2. 增加饱和保护：`delta_a_hat = max(min(z2, 0.5), -0.5)`
3. 增大观测阶段时序 `T_o` 至 2-3 秒

### P2 — PDF Gramian 条件数优化（1 周）
1. 实验不同 `h` 值（0.5/1.0/2.0）对条件数的影响
2. 改用截断 SVD 替代 Tikhonov 正则化
3. 尝试结构化延迟反馈参数化（`-K_c(t) = ρ(t)·K_h`）

### P3 — 强饱和工况（3 天）
1. 降低 `τmax` 或增大初始误差，使 259 的三种补偿器差异显现
   （现工况饱和占空比仅 ~4.6%）
2. 在 `main_dou_yue_259_compare.m` 中新增一个 `cfg` 分支并单独成表

### P4 — 蒙特卡洛统计分析（3 天）
1. 对随机初始误差运行 30 次 MC 仿真
2. 计算各方法的统计误差分布

### 投稿路线
- **当前状态**: 适合 arXiv 预印本或内部技术报告
- **P0+P1+P2 完成后**: ICRA / IROS 等 8 页顶会
- **全部完成后**: IEEE Trans. Robotics / Automatica / Acta Astronautica

---

## 物理参数

七自由度空间机械臂参数来自严宇新论文表 2-1：

| 体 | 质量 [kg] | 惯性 Ixx/ Iyy/ Izz [kg·m²] | a_i [m] | b_i [m] |
|---|---|---|---|---|
| 基座 | 1000 | 72 / 72 / 72 | — | (0.6, 0, 0) |
| 连杆 1 | 4.25 | 0.05 / 1.28 / 1.28 | (0.6, 0, 0) | (0.6, 0, 0) |
| 连杆 2 | 7 | 0.09 / 1.46 / 1.46 | (1.5, 0, 0.6) | (1.5, 0, 0.6) |
| 连杆 3 | 7 | 0.09 / 1.46 / 1.46 | (1.5, 0, 0.6) | (1.5, 0, 0.6) |
| 连杆 4 | 4.25 | 0.05 / 0.89 / 0.89 | (0, -0.5, 0) | (0, -0.5, 0) |
| 连杆 5 | 4.25 | 0.05 / 0.89 / 0.89 | (0, 0, 0.5) | (0, 0, 0.5) |
| 连杆 6 | 4.25 | 0.05 / 0.89 / 0.89 | (0, 0, 0.5) | (0, 0, 0.5) |
| 连杆 7 | 4.25 | 0.021 / 0.53 / 0.53 | (0.3, 0, 0) | (0.3, 0, 0) |