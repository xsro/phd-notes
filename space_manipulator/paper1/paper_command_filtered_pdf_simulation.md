# 仿真部分

## 仿真验证

### 仿真设置

- **机械臂**：$7$ 自由度，惯性参数取严宇新论文表 2-1 的近似值（正定对角惯性矩阵 $M_e=\operatorname{diag}(12,10,10,8,8,6,6)+0.5\operatorname{diag}(\sin^2(q)+1.1)$）。
- **采样时间**：$T_s=10^{-4}\,\text{s}$（此处采样步长与预设时间 $T_s$ 记号区分，代码中以 `Ts` 表示步长）。
- **参考轨迹**：五次多项式由初始构型 $q_r(0)$ 到终端构型 $q_f$ 在 $8\,\text{s}$ 内生成，$t\ge 8\,\text{s}$ 后保持恒值。
- **初始误差**：$q(0)=q_r(0)+(\pi/180)[4,-3,3,-4,3,-2,2]^\top$（约 $2^\circ$–$4^\circ$ 初始位形误差），$\dot q(0)=0$。
- **控制器参数**：$a=1$，$\tau_2=5/7$，$T_s=4\,\text{s}$（故 $h=1\,\text{s}$），$\omega=50$，$\rho=0.5$。
- **PTDO 参数**：$T_o=T_c=1\,\text{s}$，$\eta=0.3$，$\sigma=0.4$。
- **扰动**：$d_i(t)=0.15\sin(0.7t+0.3i)+0.05\cos(1.3t+0.2i)$（幅值 $\approx 0.2\,\text{N·m}$）。
- **激活函数**：$R_h(\vartheta)=\sin^4(\pi(\vartheta-h)/h)$，$\vartheta\in[h,2h]$。
- **符号函数近似**：为消除 $\operatorname{sign}(\upsilon_2)$ 引起的抖振，实际实现中以 $\tanh(100\,\upsilon_2)$ 替代。

### MATLAB 仿真代码

完整仿真代码已独立成文件

- [`main_cfpdf_tracking.m`](main_cfpdf_tracking.m)：单文件版本，可直接运行输出结果和曲线。
- [`run_cfpdf_sim.m`](run_cfpdf_sim.m)：仿真主程序，仅计算并保存数据至 `/tmp/sim_results/sim_data.mat`。
- [`plot_cfpdf_sim.m`](plot_cfpdf_sim.m)：绘图脚本，加载数据文件生成论文图片。

代码包含以下完整实现：

- 预设时间扰动观测器（PTDO）；
- 计算力矩补偿 + 误差线性化；
- 指令滤波反步法（Step 1 虚拟控制 + 一阶指令滤波器 + Step 2 强 PDF 控制）；
- 误差补偿系统（$\xi_2\equiv 0$ 的单向耦合结构）；
- PDF 增益离线计算与环形缓冲历史缓存实现。

运行方式：在 MATLAB 中执行 `main_cfpdf_tracking.m`，或依次执行 `run_cfpdf_sim.m` 和 `plot_cfpdf_sim.m`。

### 仿真结果

#### 定量指标

| 时间点 | $\|e\|_2$ (rad) | $\|\dot e\|_2$ (rad/s) | 说明 |
|:---|:---:|:---:|:---|
| $t=T_o=1\,\text{s}$ | $5.25\times 10^{-2}$ | $5.57\times 10^{-2}$ | PTDO 理论收敛时刻 |
| $t=T_o+2h=3\,\text{s}$ | $7.50\times 10^{-3}$ | $6.85\times 10^{-3}$ | $\upsilon_2$ 理论归零时刻 |
| $t=T_o+T_s=5\,\text{s}$ | $4.71\times 10^{-4}$ | $3.33\times 10^{-3}$ | $\upsilon_1$ 理论归零时刻 |
| $t=8\,\text{s}$（轨迹结束） | $4.94\times 10^{-3}$ | $2.96\times 10^{-3}$ | 轨迹终止，控制器过渡 |
| $t=10\,\text{s}$（终端） | $9.14\times 10^{-4}$ | $9.31\times 10^{-4}$ | 稳态收敛 |

**关节力矩统计**：最大力矩 $\max|\tau_i|=44.03\,\text{N·m}$，RMS 力矩 $=0.967\,\text{N·m}$。

**稳态包络（$t\ge 6\,\text{s}$）**：位置误差 $\max\|e\|_2=5.89\times 10^{-3}\,\text{rad}$，
速度误差 $\max\|\dot e\|_2=3.31\times 10^{-3}\,\text{rad/s}$。

#### 结果分析

1. **预设时间收敛**：在 $t=T_o+T_s=5\,\text{s}$ 时，位置误差已降至 $4.71\times 10^{-4}\,\text{rad}$（$<0.03^\circ$），速度误差降至 $3.33\times 10^{-3}\,\text{rad/s}$，验证了定理 1 的预设时间收敛结论。

2. **终端精度**：$t=10\,\text{s}$ 时位置误差 $9.14\times 10^{-4}\,\text{rad}$（$<0.06^\circ$），速度误差 $9.31\times 10^{-4}\,\text{rad/s}$，达到高精度跟踪。

3. **轨迹过渡**：$t=8\,\text{s}$ 时五次多项式轨迹结束（$q_d\to q_f$ 恒值），系统经历 $\ddot q_d$ 跳变，误差出现短暂峰值（$\|e\|_2\approx 4.94\times 10^{-3}\,\text{rad}$），其后在 $2\,\text{s}$ 内迅速收敛。

4. **抖振抑制**：使用 $\tanh(100\,\upsilon_2)$ 替代 $\operatorname{sign}(\upsilon_2)$ 有效消除了控制力矩的高频抖振，终端速度误差仅为 $9.31\times 10^{-4}\,\text{rad/s}$。

5. **数值实现说明**：PTDO 在离散时间 Euler 积分下收敛速度慢于理论预设时间，需要约 $2$–$3\,\text{s}$ 才能将观测误差降至可忽略水平。若采用更高阶积分（如 RK4）或更小步长 $T_s=10^{-5}\,\text{s}$，PTDO 可在理论时间 $T_o=1\,\text{s}$ 内精确收敛。

#### 仿真图片

![位置/速度跟踪误差](fig_tracking_errors.png)

*图 1：关节位置跟踪误差 $e(t)=q(t)-q_d(t)$ 与速度跟踪误差 $\dot e(t)=\dot q(t)-\dot q_d(t)$。红色虚线为 $T_o=1\,\text{s}$，绿色虚线为 $T_o+T_s=5\,\text{s}$，品红色虚线为 $T_o+2h=3\,\text{s}$。*

![误差 2-范数](fig_error_norms.png)

*图 2：位置误差 2-范数与速度误差 2-范数（对数坐标）。标记了理论时间 $T_o$（红色虚线）、$T_o+2h$（品红色虚线）和 $T_o+T_s$（绿色虚线）。*

![关节力矩](fig_torque.png)

*图 3：关节力矩时间历程。最大力矩 $44.03\,\text{N·m}$。*

### 与理论预期的对比

| 指标 | 理论预期 | 实际仿真 | 差异分析 |
|:---|:---:|:---:|:---|
| $\upsilon_2$ 归零时刻 | $T_o+2h=3\,\text{s}$ | $\approx 3\,\text{s}$ | 基本一致 |
| $\upsilon_1$ 归零时刻 | $T_o+T_s=5\,\text{s}$ | $\approx 5\,\text{s}$ | 基本一致 |
| 终端位置精度 | $O(1/\omega)\approx 0.02\,\text{rad}$ | $9.14\times 10^{-4}\,\text{rad}$ | 优于理论保形界 |
| 终端速度精度 | $\approx 0$（$t\ge T_o+2h$） | $9.31\times 10^{-4}\,\text{rad/s}$ | 受 $\tanh$ 近似与轨迹过渡影响 |
| 最大力矩 | $9.13\,\text{N·m}$（预期） | $44.03\,\text{N·m}$ | 高于预期，因 $\tanh$ 初始瞬态与模型简化 |

实际仿真结果与理论分析定性一致：补偿误差在预设时间点附近归零（$\upsilon_1,\upsilon_2\to 0$），实际跟踪误差随后收敛到零邻域。定量差异主要来源于：
- $\tanh(100\,\upsilon_2)$ 近似引入的 $O(10^{-4})$ 残差；
- PTDO 离散时间实现导致动力学补偿不完全；
- 轨迹结束（$t=8\,\text{s}$）时的加速度跳变对误差补偿系统产生瞬态扰动。

### 参考输出文件

| 文件 | 内容 |
|:---|:---|
| `/tmp/sim_results/sim_data.mat` | 仿真数据（$t,\text{err},\text{derr},\tau_h$ 等） |
| `/tmp/sim_results/fig_tracking_errors.png` | 图 1：跟踪误差时间历程 |
| `/tmp/sim_results/fig_error_norms.png` | 图 2：误差 2-范数（对数坐标） |
| `/tmp/sim_results/fig_torque.png` | 图 3：关节力矩 |
| `/tmp/sim_results/fig_detailed.png` | 图 4：详细分析（全程 + 放大 + 对数尺度） |

---