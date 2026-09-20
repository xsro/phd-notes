# 独轮车模型上微分平坦技术应用报告

> **撰写日期**：2025年  
> **资料来源**：okb-assist 知识库（MCP）及课题组既有笔记  
> **核心参考文献**：`doc:2091` Pliego-Jiménez et al., *Automatica*, 2021

---

## 1. 引言

独轮车（Unicycle）模型是非完整移动机器人（Wheeled Mobile Robot, WMR）中最经典的一类动力学模型，广泛应用于无人车、无人艇、移动机器人编队等领域。其状态包括笛卡尔位置 $(x, y)$ 和航向角 $\theta$，控制输入为前向速度 $v$ 和角速度 $\omega$。由于非完整约束的存在，该系统不能通过光滑静态反馈实现任意点的镇定，这使得轨迹跟踪与镇定控制的设计长期依赖于时变反馈、反步法、滑模控制等技术。

**微分平坦（Differential Flatness）** 是一类特殊的非线性系统结构特性。具有该特性的系统，其所有状态和控制输入均可由一组特定输出（平坦输出，flat output）及其有限阶导数参数化表示。这意味着轨迹规划问题可以被简化为平坦输出空间中的几何路径设计问题，从而将复杂的非线性控制问题转化为相对直观的几何规划问题。

本报告基于 `doc:2091`（Pliego-Jiménez et al., *Automatica*, 2021）等核心文献，系统阐述微分平坦技术在独轮车模型上的应用，涵盖模型特性、平坦性证明、动态反馈线性化、输出反馈控制-观测器设计、稳定性分析及实验验证。

---

## 2. 独轮车运动学模型

考虑独轮车机器人的广义坐标为笛卡尔位置 $\boldsymbol{x} = \mathrm{col}(x, y)$ 和航向角 $\theta$。其运动学模型为：

$$
\dot{\boldsymbol{x}} = v \Theta, \quad \dot{\theta} = \omega \tag{1}
$$

其中 $\Theta = \mathrm{col}(\cos\theta, \sin\theta) \in \mathbb{R}^2$，输入 $v \in \mathbb{R}$ 和 $\omega \in \mathbb{R}$ 分别为前向速度和转向速度。系统满足非完整约束：

$$
\Theta^T S \dot{\boldsymbol{x}} = 0, \quad S = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix} \tag{2}
$$

$S$ 为反对称矩阵。该约束表明机器人的速度方向始终沿其航向方向。

---

## 3. 微分平坦基本概念

### 3.1 定义

一个非线性系统被称为**微分平坦（differentially flat）**的，如果存在一组输出 $\boldsymbol{\xi} = h(\boldsymbol{x}, \boldsymbol{u}, \dot{\boldsymbol{u}}, \dots, \boldsymbol{u}^{(q)})$（称为平坦输出），使得系统的所有状态 $\boldsymbol{x}$ 和控制输入 $\boldsymbol{u}$ 均可表示为 $\boldsymbol{\xi}$ 及其有限阶导数的函数：

$$
\boldsymbol{x} = \Phi(\boldsymbol{\xi}, \dot{\boldsymbol{\xi}}, \dots, \boldsymbol{\xi}^{(r)}), \quad
\boldsymbol{u} = \Psi(\boldsymbol{\xi}, \dot{\boldsymbol{\xi}}, \dots, \boldsymbol{\xi}^{(r+1)})
$$

平坦输出的维数等于控制输入的维数。

### 3.2 微分平坦的意义

微分平坦系统的核心优势在于：

1. **轨迹规划简化**：只需在平坦输出空间中设计几何路径 $\boldsymbol{\xi}_d(t)$，即可通过上述参数化关系自动生成参考状态和控制输入。
2. **反馈线性化**：平坦系统通常可经动态反馈实现输入-输出线性化，从而将非线性控制问题转化为线性系统的控制问题。
3. **观测器设计**：平坦输出的可测量性使得基于平坦性的状态估计和输出反馈控制成为可能。

---

## 4. 独轮车模型的微分平坦特性

### 4.1 平坦输出的确定

对于独轮车运动学模型 (1)，**笛卡尔位置 $\boldsymbol{\xi} = \boldsymbol{x} = \mathrm{col}(x, y)$ 即为平坦输出**。这一结论由 Oriolo, de Luca & Vendittelli (2002) 给出并被后续研究广泛引用（`doc:2091` 引用该工作）。

### 4.2 参数化关系推导

由模型方程 $\dot{\boldsymbol{x}} = v \Theta$ 可知：

- 前向速度：$v = \|\dot{\boldsymbol{x}}\|$
- 航向角：$\Theta = \frac{\dot{\boldsymbol{x}}}{\|\dot{\boldsymbol{x}}\|}$，即 $\theta = \mathrm{atan2}(\dot{y}, \dot{x})$
- 角速度：$\omega = \frac{d}{dt}\mathrm{atan2}(\dot{y}, \dot{x})$

进一步求导可得：

$$
\ddot{\boldsymbol{x}} = \dot{v} \Theta + v \omega S \Theta \tag{3}
$$

这表明，给定平坦输出 $\boldsymbol{\xi}(t)$ 及其导数 $\dot{\boldsymbol{\xi}}(t)$、$\ddot{\boldsymbol{\xi}}(t)$，即可唯一确定系统的所有状态 $(\boldsymbol{x}, \theta)$ 和控制输入 $(v, \omega)$。这验证了独轮车模型的微分平坦特性。

---

## 5. 基于微分平坦的动态反馈线性化

### 5.1 输入-输出模型

平坦输出 $\boldsymbol{\xi}$ 的一阶导数不包含角速度 $\omega$：

$$
\dot{\boldsymbol{\xi}} = v \Theta \tag{4}
$$

为使 $\omega$ 显式出现，对平坦输出求二阶导数：

$$
\ddot{\boldsymbol{\xi}} = \dot{v} \Theta + v \omega S \Theta \tag{5}
$$

引入附加积分器 $\dot{v} = a$（$a$ 为新的控制输入），可将上式写为：

$$
\ddot{\boldsymbol{\xi}} = \Phi^{-1}(\theta, v) \boldsymbol{u} \tag{6}
$$

其中 $\boldsymbol{u} = \mathrm{col}(a, \omega)$，且

$$
\Phi^{-1}(\theta, v) = \begin{bmatrix} \Theta & v S \Theta \end{bmatrix}, \quad
\Phi(\theta, v) = \begin{bmatrix} \Theta & v^{-1} S \Theta \end{bmatrix}^T \tag{7}
$$

### 5.2 矩阵性质

矩阵 $\Phi(\theta, v)$ 在 $v \neq 0$ 时非奇异，且满足以下重要性质：

$$
\Phi(\theta, v) \boldsymbol{a} = \Phi(\boldsymbol{a}, -v) \Theta \tag{8a}
$$
$$
\Phi(\theta_1 + \theta_2, v) = \Phi(\theta_1, v) + \Phi(\theta_2, v) \tag{8b}
$$

性质 (8a) 在后续稳定性分析中起关键作用，使得交叉项可以被有效界定。

### 5.3 控制律设计

设 $\boldsymbol{\xi}_d(t) \in \mathbb{R}^2$ 为期望轨迹（具有连续有界的导数），定义跟踪误差 $\Delta\boldsymbol{\xi} = \boldsymbol{\xi} - \boldsymbol{\xi}_d(t)$。基于反馈线性化思想，设计控制律：

$$
\boldsymbol{u} = \Phi(\hat{\Theta}, v) \boldsymbol{\tau}, \quad
\boldsymbol{\tau} = -K_c(\dot{\boldsymbol{\xi}}_0 - \dot{\boldsymbol{\xi}}_r) + \ddot{\boldsymbol{\xi}}_r \tag{9}
$$

其中 $\hat{\Theta}$ 为航向角估计值，$\dot{\boldsymbol{\xi}}_r$ 和 $\ddot{\boldsymbol{\xi}}_r$ 分别为参考速度和参考加速度：

$$
\dot{\boldsymbol{\xi}}_r = \dot{\boldsymbol{\xi}}_d(t) - A_c(\hat{\boldsymbol{\xi}} - \boldsymbol{\xi}_d(t)), \quad
\dot{\boldsymbol{\xi}}_0 = \dot{\hat{\boldsymbol{\xi}}} - A_o \tilde{\boldsymbol{\xi}} \tag{10}
$$

$A_c, A_o, K_c \in \mathbb{R}^{2 \times 2}$ 为对称正定增益矩阵。

将控制律 (9) 代入输入-输出模型 (6)，闭环动态为：

$$
\ddot{\boldsymbol{\xi}} = -K_c(\dot{\boldsymbol{\xi}}_0 - \dot{\boldsymbol{\xi}}_r) + \ddot{\boldsymbol{\xi}}_r - \Phi^{-1}(\theta, v) \Phi(\tilde{\Theta}, v) \boldsymbol{\tau} \tag{11}
$$

其中 $\tilde{\Theta} = \Theta - \hat{\Theta}$ 为航向角估计误差。

---

## 6. 输出反馈控制-观测器方案

实际应用中，往往只能获取笛卡尔位置测量（如通过视觉系统），而航向角 $\theta$ 和笛卡尔速度 $\dot{\boldsymbol{\xi}}$ 不可测。为此，设计非线性观测器实现输出反馈控制。

### 6.1 航向角观测器

航向角 $\theta$ 不显式出现在运动学方程 (1) 中，因此转而估计 $\Theta$。$\Theta$ 的时间导数为：

$$
\dot{\Theta} = \omega(t) S \Theta \tag{12}
$$

提出的观测器结构为：

$$
\hat{\Theta} = \boldsymbol{\rho} + v(t) T \boldsymbol{\xi} \tag{13}
$$

其中 $T \in \mathbb{R}^{2 \times 2}$ 为对称正定矩阵，辅助状态 $\boldsymbol{\rho}$ 的动态为：

$$
\dot{\boldsymbol{\rho}} = -(v^2(t) T - \omega(t) S)[\boldsymbol{\rho} + v(t) T \boldsymbol{\xi}] - \dot{v}(t) T \boldsymbol{\xi} \tag{14}
$$

该观测器本质上是一个时变系统，以 $\boldsymbol{\xi}$ 为输入、$\hat{\Theta}$ 为输出。初始条件可选为 $\boldsymbol{\rho}(0) = \hat{\Theta}(0) - v(t_0) T \boldsymbol{\xi}(0)$。

### 6.2 速度观测器

为克服笛卡尔速度不可测的问题，设计高增益观测器：

$$
\dot{\hat{\boldsymbol{\xi}}} = (A_0 + K_d)\tilde{\boldsymbol{\xi}} + \boldsymbol{\eta} \tag{15a}
$$
$$
\dot{\boldsymbol{\eta}} = \dddot{\boldsymbol{\xi}}_d(t) - A_c(\dot{\hat{\boldsymbol{\xi}}} - \dot{\boldsymbol{\xi}}_d(t)) + K_d A_0 \tilde{\boldsymbol{\xi}} \tag{15b}
$$

其中 $\tilde{\boldsymbol{\xi}} = \boldsymbol{\xi} - \hat{\boldsymbol{\xi}}$，$K_d = K_c + K_o$，$A_0, A_c, K_c, K_o \in \mathbb{R}^{2 \times 2}$ 均为对称正定矩阵。

### 6.3 闭环系统动态

引入辅助变量：

$$
\boldsymbol{r} = \dot{\boldsymbol{\xi}} - \dot{\boldsymbol{\xi}}_0 = \dot{\boldsymbol{\xi}} + A_0 \tilde{\boldsymbol{\xi}} \tag{16}
$$
$$
\boldsymbol{s} = \dot{\boldsymbol{\xi}} - \dot{\boldsymbol{\xi}}_r = \Delta\dot{\boldsymbol{\xi}} + A_c \Delta\boldsymbol{\xi} - A_c \tilde{\boldsymbol{\xi}} \tag{17}
$$

结合观测器动态 (15) 和控制律 (9)，可得 $\boldsymbol{s}$ 和 $\boldsymbol{r}$ 的动态：

$$
\dot{\boldsymbol{s}} = -K_c \boldsymbol{s} + K_c \boldsymbol{r} - \Phi^{-1}(\Theta, v) \Phi(\boldsymbol{\tau}, -v) \tilde{\Theta} \tag{18}
$$
$$
\dot{\boldsymbol{r}} = -K_o \boldsymbol{r} - K_c \boldsymbol{s} - \Phi^{-1}(\theta, v) \Phi(\boldsymbol{\tau}, -v) \tilde{\Theta} \tag{19}
$$

最终，闭环系统可写为级联形式：

$$
\dot{\boldsymbol{z}} = A \boldsymbol{z} + \Omega(t, \boldsymbol{z}) \tag{20a}
$$
$$
\dot{\tilde{\Theta}} = -v^2(t) \Gamma \tilde{\Theta} + \omega(t) S \tilde{\Theta} \tag{20b}
$$

其中 $\boldsymbol{z} = \mathrm{col}(\Delta\boldsymbol{\xi}, \tilde{\boldsymbol{\xi}}, \boldsymbol{s}, \boldsymbol{r}) \in \mathbb{R}^8$，矩阵 $A$ 可通过适当选取增益配置为 Hurwitz 矩阵，$\Omega(t, \boldsymbol{z})$ 为非线性互连项。

---

## 7. 稳定性分析

### 7.1 主要结论

**命题 1**：若前向速度满足 $0 < v_{\min} \leq |v(t)| \leq v_{\max}$，且期望轨迹的导数一致有界，则级联系统 (20) 的平衡点 $(\boldsymbol{z}, \tilde{\Theta}) = (\boldsymbol{0}, \boldsymbol{0})$ 是全局一致渐近稳定（UGAS）的。

### 7.2 证明思路

证明基于级联系统的 Lyapunov 理论（Loria, 2008）：

1. **子系统 $\dot{\boldsymbol{z}} = A\boldsymbol{z}$ 的稳定性**：由于 $A$ 是 Hurwitz 矩阵，存在正定矩阵 $P$ 使得 $A^T P + PA = -Q$（$Q > 0$）。取 Lyapunov 函数 $V_1(\boldsymbol{z}) = \boldsymbol{z}^T P \boldsymbol{z}$，其沿轨迹的导数满足 $\dot{V}_1 \leq -\lambda_{\min}(Q) \|\boldsymbol{z}\|^2$。

2. **航向角误差子系统的稳定性**：取 $V_{\Theta} = \|\tilde{\Theta}\|^2 / 2$，由 (20b) 可得：
   $$
   \dot{V}_{\Theta} \leq -\gamma v^2(t) \|\tilde{\Theta}\|^2 \leq -2 v_{\min}^2 \gamma V_{\Theta}
   $$
   其中 $\gamma = \lambda_{\min}(T)$。因此 $\tilde{\Theta} = \boldsymbol{0}$ 是全局指数稳定的。

3. **互连项的界定**：利用性质 (8a) 和矩阵不等式，可证互连项满足：
   $$
   \|\Omega(t, \boldsymbol{z}) \tilde{\Theta}\| \leq \delta_1 \|\tilde{\Theta}\| \|\boldsymbol{z}\| + \delta_2 \|\tilde{\Theta}\|
   $$
   其中 $\delta_1, \delta_2 > 0$ 为有界常数。

4. **级联稳定性定理的应用**：验证 Loria (2008) 级联稳定性定理的所有条件，得出全局一致渐近稳定性结论。

---

## 8. 实验验证

### 8.1 实验平台与设置

实验在 Khepera III 移动机器人上完成，使用由 8 台 Optitrack 相机组成的视觉系统测量笛卡尔位置。期望轨迹为双纽线（lemniscate）：

$$
\xi_{1d}(t) = 0.5 \sin\left(\frac{2\pi t}{15}\right), \quad
\xi_{2d}(t) = 0.5 \sin\left(\frac{2\pi t}{30}\right)
$$

### 8.2 控制参数

控制与观测器增益设置为：
- $A_c = 3.15 I$, $A_0 = 4.5 I$
- $K_c = 0.31 I$, $K_o = 1.5 I$
- $T = 1380 I$
- 初始条件：$v(0) = 0.09$, $\hat{\Theta}(0) = \mathrm{col}(0, 1)$

### 8.3 实验结果

1. **轨迹跟踪**（Fig. 1）：实际轨迹与期望双纽线轨迹高度吻合。
2. **航向角观测**（Fig. 2）：尽管初始估计误差较大，暂态过程后观测器性能良好，$\hat{\Theta}_1$ 和 $\hat{\Theta}_2$ 均收敛至真实值。
3. **速度估计**（Fig. 3）：估计的笛卡尔速度范数在暂态后收敛至期望值，验证了高增益观测器的有效性。

实验结果表明，尽管仅使用笛卡尔位置测量，所提出的控制-观测器方案在存在未建模动力学和测量噪声的情况下仍能实现良好的渐近轨迹跟踪。

---

## 9. 方法优势与技术要点总结

| 维度 | 说明 |
|------|------|
| **平坦输出** | 笛卡尔位置 $\boldsymbol{\xi} = (x, y)$，可直接测量 |
| **控制框架** | 动态反馈线性化（引入积分器 $\dot{v} = a$） |
| **观测器设计** | 航向角观测器（基于 $\Theta$ 参数化）+ 高增益速度观测器 |
| **稳定性工具** | 级联系统 Lyapunov 理论（Loria, 2008） |
| **测量需求** | 仅需笛卡尔位置（视觉/GPS 即可） |
| **核心条件** | $v(t)$ 一致有界且远离零（$|v(t)| \geq v_{\min} > 0$） |
| **验证方式** | Khepera III 实验平台 + Optitrack 视觉系统 |

---

## 10. 应用前景与展望

微分平坦技术在独轮车模型上的成功应用，为以下研究方向提供了坚实基础：

1. **多智能体协同**：利用平坦输出的几何特性，可在平坦输出空间中直接规划多智能体的协同轨迹，简化碰撞避免和队形保持（参见 `doc:3090` UAV-USV 协同控制框架）。
2. **输出反馈与鲁棒控制**：将平坦性与 Immersion and Invariance（I&I）观测器、滑模控制等技术结合，增强对模型不确定性和外部扰动的鲁棒性。
3. **实时轨迹重规划**：平坦输出空间中的几何路径规划比状态空间中的微分包含约束处理更为直观，适合动态环境下的实时避障。
4. **扩展至高阶系统**：将独轮车的微分平坦结论推广至自行车模型（Bicycle Model）、Euler-Lagrange 系统等更复杂的非完整系统。

---

## 参考文献（核心）

1. **Pliego-Jiménez, J., Martínez-Clark, R., Cruz-Hernández, C., & Arellano-Delgado, A.** (2021). Trajectory tracking of wheeled mobile robots using only cartesian position measurements. *Automatica*, 133, 109756. (`doc:2091`)
2. **Oriolo, G., de Luca, A., & Vendittelli, M.** (2002). WMR control via dynamic feedback linearization: Design, implementation and experiments. *IEEE Transactions on Control Systems Technology*, 10(6), 835–852.
3. **Zhang, Z., Liu, S., Wang, H., & Wu, H.** (2025). A cooperative control framework for path following of UAV-USV cooperative system in maritime scenarios. *Ocean Engineering*. (`doc:3090`)
4. **Charlet, B., Levine, J., & Marino, R.** (1989). On dynamic feedback linearization. *Systems & Control Letters*, 13(2), 143–153.
5. **Loria, A.** (2008). From feedback to cascade-interconnected systems: Breaking the loop. *47th IEEE Conference on Decision and Control*, pp. 4109–4114.

---

> **备注**：本报告基于 okb-assist 知识库中 `doc:2091`（Pliego-Jiménez et al., 2021, *Automatica*）为核心素材撰写，该文献明确给出了独轮车运动学模型以笛卡尔位置为平坦输出的结论，并基于此设计了动态反馈线性化控制器与非线性观测器，最终通过实验验证了方案的有效性。报告中的数学推导和控制框架均忠实于该文献的理论体系。