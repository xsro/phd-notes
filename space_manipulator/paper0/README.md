# 基于光滑周期延迟反馈的自由漂浮空间机械臂预设时间轨迹跟踪控制

## 项目概述

本仓库实现自由漂浮空间机械臂的关节空间轨迹跟踪控制，控制方法为"扰动观测（PTDO）—动力学补偿—误差线性化—光滑周期延迟反馈（PDF）"四阶段结构。论文已编译为 27 页 PDF（`paper.pdf`），包含理论推导、定理-证明体系、扰动有界性分析和仿真验证。

---

## 目录结构

```
paper0/
├── paper.tex                  # 主文档（ctexart, amsthm 定理环境）
├── paper.pdf                  # 当前编译版本（27 页）
├── sections/                  # 论文章节
│   ├── 01-introduction.tex
│   ├── 02-modeling.tex
│   ├── 03-problem.tex
│   ├── 04-controller.tex
│   ├── 05-stability.tex
│   ├── 06-simulation.tex
│   └── 07-conclusion.tex
├── refs/                      # 参考文献
│   ├── local.bib
│   └── my_library.bib         # Zotero 引用
├── figures/                   # 仿真输出图片
└── simulation/                # MATLAB 仿真代码（文献 259 参数）
    ├── README.md                       # 仿真说明（含文献 259 模型与参数）
    ├── main_dou_yue_259_compare.m      # 主脚本：参数取 259，10 算例对比
    ├── dou_yue_259_parameters.m        # Table 1/2 参数（OKB doc 259）
    ├── dou_yue_259_control_params.m    # Table 3 控制参数
    ├── dou_yue_259_disturbance.m       # Eq (74) 扰动力矩
    ├── ffsm259_model.m                 # DH + Schur 补完整动力学 ✅
    ├── fonsm259_init.m / fonsm259_step.m   # FONTSM / FOSM 控制器
    ├── ftdob_step.m                    # 有限时间扰动观测器
    ├── build_pdf_gain.m / pdf_gain_at.m    # PDF Gramian 增益
    ├── validate_ffsm259_model.m        # 模型校核（Robotics Toolbox 对比）
    ├── dou_yue_259_check_consistency.m # Table 1 一致性核查
    └── calib_kappa.m                   # 增益标定扫描
```

> 原基于严宇新表 2-1 的仿真脚本已删除（见 §已归档），可用
> `git log -- simulation/` 找回旧版本。

---

## 已完成内容

### 📄 论文（7 个文件全量重写）

| 部分 | 内容 |
|------|------|
| paper.tex | 添加 `amsthm` 宏包（theorem/lemma/proof/definition/assumption/remark），27 页无编译错误 |
| §1 引言 | 结构化 "相关工作" + "本文贡献"，贡献项编号化，对应后文定理 |
| §2 建模 | 浮基 → Schur 补推导，空间 CRBA 数值实现说明，3 条形式化假设 |
| §3 问题定义 | 问题 1（预设时间跟踪）、定义 1/2（单周期/幂零零化）、5 条假设环境 |
| §4 控制器 | 定理 1（PTDO 收敛）、引理 1（误差线性化+Kalman 秩证明）、定义 3（可控 Gramian）、定理 2（PDF 单周期零化+完整变分公式证明） |
| §5 稳定性 | 定理 3（精确跟踪，5 条理想条件）、定理 4（周期边界有界跟踪，完整证明）、命题 1（近似零化稳态上界）、Gramian 病态性 Remark、推论 1-3、理论结果总结表 |
| §6 仿真 | Jacobian 法完整数据、诚实讨论 PDF/PTDO 数值局限、模型定位 Remark |
| §7 结论 | 5 点未来工作（CRBA 验证、Gramian 优化、任务空间集成、离散时间理论、实验验证） |

### 🖥️ 仿真代码（全部基于文献 259 参数）

| 文件 | 方法 | 状态 |
|------|------|------|
| `ffsm259_model.m` | **DH 正解 + 质心雅可比求和惯性**，Schur 补得 `M_θm/M_r`，中心差分 Christoffel 得 `C_θm`，广义雅可比 `J` | ✅ 与 Robotics System Toolbox 交叉校核：正运动学 **2.3e−16 m**、固定基座质量矩阵 **8.5e−16**、性质 2（`Ṁ−2C` 反对称）**1.4e−16**、动量守恒 5e−14 |
| `main_dou_yue_259_compare.m` | **10 算例 × 2 种增益标定** — PD / PDF / PDF+FTDOB vs FONTSM / FOSM 的三种补偿器 | ✅ 运行成功（约 9–10 min） |
| `fonsm259_step.m` + `ftdob_step.m` | Eq (15)(35)(36)(37)(44)(50)(57) 逐条实现 | ✅ 与原文 Case (1)(2)(3) 对应 |
| `validate_ffsm259_model.m` | 模型正定性 / 工具箱对比 / 能量与动量守恒 | ✅ 全部通过 |

### 📊 文献 259（Dou & Yue 2025, IJRNC）参数复现与算法对比

被控对象、初始状态、扰动与饱和限幅全部取自 OKB doc 259 的 Table 1/2 与 Eq (74)，
初始误差 0.5°，κ = 0.01，Ts = 2 ms，Tf = 20 s：

| 控制器 | \|e(T)\| [rad] | RMS_ss | t_s [s] | max\|τ\| | sat% |
|---|---|---|---|---|
| PD（无扰动） | 2.50e-14 | 1.44e-06 | 3.18 | 20.0 | 5.3 |
| PDF（无扰动） | 2.58e-14 | 1.47e-06 | 3.15 | 20.0 | 5.3 |
| PD（有扰动） | 5.43e-02 | 6.60e-02 | — | 20.0 | 5.7 |
| PDF（有扰动） | 6.02e-02 | 6.91e-02 | — | 20.0 | 5.7 |
| **PDF + FTDOB[259]** | **1.24e-04** | **6.29e-05** | 3.45 | 20.0 | 5.6 |
| FONTSM+FTDOB（无补偿） | 1.99e-03 | 1.20e-02 | — | 20.0 | 4.6 |
| FONTSM+FTDOB+FTAC[29] | 1.41e-03 | 1.16e-02 | — | 20.0 | 4.6 |
| FONTSM+FTDOB+FTASC | 1.97e-03 | 1.20e-02 | — | 20.0 | 4.6 |
| FONTSM+FTDOB+FTASC+自适应 | 1.30e-03 | 1.14e-02 | — | 20.0 | 4.7 |
| FOSM+FTDOB+FTASC | 1.29e-02 | 3.39e-02 | — | 14.7 | 0.0 |

要点：（a）`Table 3` 的增益需按 `M_θm` 量级重新标定（κ，稳定边界 ≈ 0.015），
否则全程饱和并发散；（b）加上 259 的 FTDOB 后，本工程的 PDF 稳态精度
（6.3e-05）明显优于 FONTSM 族（1.1e-02 ~ 1.2e-02）；（c）同等条件下
FONTSM 比 FOSM 好约 2.8 倍，与原文结论方向一致。详见 `simulation/README.md`。

### 📊 已归档：旧参数（严宇新表 2-1）的历史结果

> 对应脚本（`run_final_paper_sim.m`、`main_ffsm_pdf_tracking_yan_params.m`、
> `compute_floating_base_*.m`、`spatial_crba.m`）已删除，数据保留供论文 §6 引用。
> 基座 1000 kg / I=72，连杆 4.25–7 kg；`τmax` 无限制（峰值 259 Nm）。

| Controller | \|e(T)\| [rad] | RMS(e) [rad] | max\|τ\| [Nm] |
|---|---|---|---|
| PD(no dist) | **1.33e-09** | 5.04e-10 | 258.9 |
| PDF(no dist) | **1.33e-09** | 5.04e-10 | 258.9 |
| PD(dist) | 1.37e-04 | 5.19e-05 | 258.9 |
| PDF(dist) | 1.37e-04 | 5.19e-05 | 258.9 |
| PDF+PTDO(dist) | 3.33e-04 | 1.26e-04 | 1920.6 |

---

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
```

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

### P0 — 任务空间扩展（建议优先）
1. 把 `fonsm259_step.m` 的滑模面从关节误差换成末端误差
   `x_e^d - x_e` 与 `J_E⁺(ẍ^r - J̇θ̇)`（原文 Eq 48–51）
2. 叠加基座姿态稳定的二级任务（Eq 52–56，需 `H̄_w/H̄_m/C̄_b1/C̄_m1` 的 v/w 分块）
3. 与现有关节空间结果对比，验证两种任务表述的一致性

### P1 — PTDO 参数整定（1 周）
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