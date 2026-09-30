# 仿真验证：指令滤波 + PDF + PTDO 预设时间轨迹跟踪

本目录包含论文《基于指令滤波与周期延迟反馈的自由漂浮空间机械臂预设时间轨迹跟踪控制》的 MATLAB 仿真代码。

## 文件说明

| 文件 | 用途 |
|------|------|
| `run_cfpdf_sim.m` | **仿真主程序**：运行完整 7-DOF 自由漂浮空间机械臂轨迹跟踪仿真，输出数据到 `out/` |
| `plot_cfpdf_sim.m` | **画图程序**：读取仿真数据，生成论文中所有结果图片到 `out/` |
| `main_cfpdf_tracking.m` | **精简版仿真**：含基本定量分析与简单绘图（不依赖独立画图脚本） |

## 环境要求

- MATLAB R2025b（或兼容版本）
- 无需额外工具箱

## 快速运行

### 1. 运行仿真

```bash
cd /path/to/paper1/simulation
matlab -batch "run('run_cfpdf_sim.m');"
```

或直接在 MATLAB 桌面中打开并运行 `run_cfpdf_sim.m`。

仿真输出摘要：
```
======== CF-PDF+PTDO 仿真 ========
Ts = 1.0e-04, Tf = 10.0 s, N = 100001
n = 7 DOF, To = 1.0 s, Tsp = 4.0 s, h = 1.000 s
...
结果已保存到 .../simulation/out/sim_data.mat
```

### 2. 生成图片

```bash
matlab -batch "run('plot_cfpdf_sim.m');"
```

生成 7 组图片（PNG + PDF 双格式）：

| 图片 | 说明 | 对应论文图号 |
|------|------|------------|
| `joint_tracking_error_torque_summary` | 7×3 拼接：位置跟踪、速度跟踪、误差、力矩 | 图 2 (summary) |
| `error_norm` | 跟踪误差 2-范数 | — |
| `error_norm_log` | 对数坐标 + $T_o$ / $T_o+T_s$ 标记 | 图 3 (norms) |
| `control_torque` | 控制力矩时间历程 | 图 4 (torque) |
| `arm_configuration` | 基座平移、末端轨迹、3D 构型 | 图 1 (arm) |
| `joint_position_tracking` | 各关节位置跟踪（分图） | — |
| `joint_velocity_tracking` | 各关节速度跟踪（分图） | — |

### 3. 一步运行（仿真 + 画图）

```bash
matlab -batch "run('run_cfpdf_sim.m'); run('plot_cfpdf_sim.m');"
```

## 输出目录

所有输出文件保存到 `simulation/out/`：

```
simulation/out/
├── sim_data.mat                      # 仿真数据（约 25 MB）
├── arm_configuration.png/.pdf        # 机械臂构型图
├── control_torque.png/.pdf           # 控制力矩图
├── error_norm.png/.pdf               # 误差范数图
├── error_norm_log.png/.pdf           # 误差范数（对数坐标）
├── joint_position_tracking.png/.pdf  # 关节位置跟踪
├── joint_velocity_tracking.png/.pdf  # 关节速度跟踪
└── joint_tracking_error_torque_summary.png/.pdf  # 综合响应图
```

## 论文编译（含仿真图片）

在 `paper1/` 根目录下四次编译即可得到包含仿真图片的完整 PDF：

```bash
cd ..   # 回到 paper1/
xelatex paper_command_filtered_pdf.tex
bibtex  paper_command_filtered_pdf
xelatex paper_command_filtered_pdf.tex
xelatex paper_command_filtered_pdf.tex
```

## 参数说明

主要设计参数在 `run_cfpdf_sim.m` 头部集中定义：

| 参数 | 值 | 含义 |
|------|-----|------|
| `n` | 7 | 关节数 |
| `Ts` | 1e-4 s | 采样步长 |
| `Tf` | 10 s | 总仿真时间 |
| `a` | 1.0 | 线性阻尼系数 |
| `tau2` | 5/7 | 幂分配参数（速度通道） |
| `Tsp` | 4 s | 预设收敛时间（$T_s$） |
| `h` | 1 s | PDF 半周期（$h=T_s/4$） |
| `om` | 50 | 指令滤波器带宽 |
| `rho` | 0.5 | 残余扰动上界 |
| `To` | 1 s | PTDO 预设时间（$T_o$） |

## 控制架构

```
PTDO 扰动估计 ──→ 计算力矩补偿 ──→ 误差线性化 ──→ 指令滤波 PDF 反步法
   t∈[0,T_o]                            t∈[T_o, T_o+T_s]
```

## 注意事项

- **PTDO 收敛**：预设时间扰动观测器在 $t=T_o$ 时精确收敛。在此之前，观测误差作为残余扰动进入通道
- **PDF 时序**：PDF 控制器在 $t=To$ 以局部时间 $\vth=t-T_o$ 启动
- **两阶段策略**：$t<T_o$ 阶段施加有界标称控制（PD 控制），$t\ge T_o$ 后启动 PDF
- **符号函数光滑化**：$\sign(\cdot)$ 以 $\tanh(100\cdot)$ 替代以避免抖振
- **精度**：$t=T_o+T_s=5\text{s}$ 处 $\|e\|_2=4.71\times10^{-4}$ rad，$t=10\text{s}$ 终端 $\|e\|_2=9.14\times10^{-4}$ rad

## 结果曲线解读

- **图 1 (arm_configuration)**：自由漂浮基座在动量守恒下反作用平动，末端轨迹受基座漂移与关节运动共同影响
- **图 2 (summary)**：7 个关节均能在五次多项式轨迹下平滑跟踪，误差在预设时刻接近归零
- **图 3 (error_norm_log)**：对数坐标清晰显示 $T_o=1\text{s}$ 和 $T_o+T_s=5\text{s}$ 处的误差骤降
- **图 4 (control_torque)**：力矩光滑无抖振，最大 $44.03$ N·m，RMS $0.967$ N·m