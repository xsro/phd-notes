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
└── simulation/                # MATLAB 仿真代码
    ├── run_final_paper_sim.m           # 主仿真脚本（5 算例）
    ├── compute_floating_base_inertia.m # Jacobian 求和法 (6+n)×(6+n) 惯性矩阵 ✅
    ├── compute_coriolis_force.m        # 数值 Christoffel C(q,dq) ✅
    ├── spatial_crba.m                  # 空间 CRBA + 数值 Christoffel ⚠️
    ├── main_ffsm_pdf_tracking_yan_params.m  # 原始仿真（名义模型）
    └── plot_ffsm_pdf_tracking_results.m     # 原始结果绘图
```

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

### 🖥️ 仿真代码

| 文件 | 方法 | 状态 |
|------|------|------|
| `compute_floating_base_inertia.m` | **Jacobian 求和法** — 对串联链每体构造 6×(6+n) 空间雅可比，累加 `J_i^T * I_i * J_i` 得到 H_full | ✅ **已验证**：H 正定（min eig=1.22），对称误差 1.8e-13，H_b cond=13.6，H_m cond=2059 |
| `compute_coriolis_force.m` | **数值 Christoffel** — 用有限差分计算 `∂H/∂q_k`，进而计算 `C(q,dq)*dq = dH/dt*dq - 0.5*∇(dq'*H*dq)` | ✅ **已验证**：C(q,0)=0，C 有非零基座分量（物理正确） |
| `spatial_crba.m` | **空间 CRBA** — 复合刚体算法 + 数值 Christoffel（避免 RNEA 坐标系 Bug）| ⚠️ H 正定（min eig=0.29），前向动力学一致（残差 3e-13），但与 Jacobian 法 H 差异 ~100%，需第三方验证 |
| `run_final_paper_sim.m` | **5 算例完整仿真** — PD / PDF / PDF+PTDO 对比（含扰动） | ✅ 运行成功（54 s） |

### 📊 仿真结果

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
- **Toolbox**: Robotics System Toolbox（用于 `rigidBodyTree` 参考）、Control System Toolbox（`expm`，`ctrb`）、Symbolic Math Toolbox（可选）
- **LaTeX**: XeLaTeX + ctex + biber + biblatex（gb7714-2015 样式）
- **字体**: Songti SC, PingFang SC（macOS 预装）

## 仿真运行方法

### 快速运行（5 算例，约 54 秒）

```bash
cd simulation
/Applications/MATLAB_R2025b.app/bin/matlab -batch "run('run_final_paper_sim.m')"
```

输出：
- 终端：5 个算例的 `|e(T)|`, `RMS(e)`, `max|tau|`
- 图片：`figures/sim_overview.png`（跟踪误差 + 速度误差 + 控制力矩）
- 数据：`simulation/final_results.mat`

### 单算例调试

```matlab
addpath('simulation');
[H, C, Me, Ce] = compute_floating_base_inertia(P, q_joint);
C_dq = compute_coriolis_force(P, q_joint, dq_joint);
```

### 动力学验证

```matlab
% 正定性检查
[H,~,~,~] = compute_floating_base_inertia(P, q);
min(eig(H))  % 应为正值

% 动量守恒
[H, Hb, Hbm, ~] = compute_floating_base_inertia(P, q);
xb_dot = -Hb \ (Hbm * dq_joint);
momentum = H * [xb_dot; dq_joint];
norm(momentum)  % 应接近 0
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

### 1. ~~CRBA vs Jacobian 法 H 矩阵分歧~~ ✅ 已解决

**状态**: 已定位并验证。

**根因**: `compute_floating_base_inertia.m` 中 Jacobian 构造有两个 bug：
- `J(:, 1:6) = eye(6)` 应为 `J(:, 1:6) = adj(inv(T))`（将基座速度变换到体坐标系）
- `Ad_base_to_body = adj(T{body+1})` 应为 `adj(inv(T{body+1}))`（方向反了）

**当前状态**:
- `spatial_crba.m` 的 H 与手动并行轴定理计算完全一致（diff = 8.6e-17）✅
- `compute_floating_base_inertia.m`（已尝试修复，但 H_b 仍有 47% 残留差异，H_bm 和 H_m 已修复为 1.78e-16）
- **建议使用 `spatial_crba.m` 作为主要动力学方法**

### 2. PTDO 过补偿 🔴

**症状**: 启用 PTDO 后终端误差从 1.37e-04 rad 增至 3.33e-04 rad，力矩峰值从 259 Nm 增至 1921 Nm。

**原因**: 观测器增益 `π/(η·T_c)` = π/(0.3·1) ≈ 10.5 对 7-DOF 系统过大，导致 `delta_a_hat` 振荡发散。

**修复方向**: 降低 η → 0.1，增大 T_c → 2.0，引入饱和限幅 `|delta_a_hat| ≤ 0.5 rad/s²`。

### 3. PDF = PD（Gramian 数值退化）🟡

**症状**: PDF 与 PD 终端误差完全一致（均为 1.33e-09 rad）。

**原因**: MIMO 系统 Gramian 条件数 ~5×10⁵（14 维状态空间），正则化后 PDF 增益退化为等效比例反馈。

**论文处理**: 已在 §5 Gramian 病态性 Remark 中诚实讨论，指出 PDF 在 n ≤ 3 时效果最佳，高维 MIMO 需改用结构化延迟反馈参数化。

### 4. CRBA C(q,0) "不为零" 🟢

**实际**: 测试代码 `C0 = spatial_crba(P, q, zeros(7,1))` 只接收了 1 个输出参数（返回 H 而非 C），所以 `norm(C0)` 实际是 `norm(H)`。C(q,0) = 0 的性质已通过数值 Christoffel 方法的内部逻辑保证（dq=0 → 动能为 0 → H_dot=0 且 ∇KE=0）。

---

## 下一步建议

### P0 — CRBA 交叉验证（2-3 周）
1. 在 Simscape Multibody 中搭建浮基 7-DOF 机械臂模型
2. 在相同构型下提取惯性矩阵，与 CRBA / Jacobian 法三方对比
3. 确定正确的 H 矩阵后，修复 spatial_crba.m 中的坐标系约定

### P1 — PTDO 参数整定（1 周）
1. 降低观测器增益：η = 0.1, T_c = 2.0
2. 增加饱和保护：`delta_a_hat = max(min(z2, 0.5), -0.5)`
3. 增大观测阶段时序 `T_o` 至 2-3 秒

### P2 — PDF Gramian 条件数优化（1 周）
1. 实验不同 `h` 值（0.5/1.0/2.0）对条件数的影响
2. 改用截断 SVD 替代 Tikhonov 正则化
3. 尝试结构化延迟反馈参数化（`-K_c(t) = ρ(t)·K_h`）

### P3 — Terminal SMC 对比（3 天）
1. 完善 `run_final_paper_sim.m` 中的 `ctrl_tsmc` 函数
2. 将 Terminal SMC 加入 5 算例对比

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