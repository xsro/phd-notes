根据知识库中的文献资料，我为您整理了free-flying/free-floating space robot相关文献中考虑的系统模型，按模型类型分类整理如下：

---

## 一、Lagrange方程（拉格朗日动力学模型）

### 模型出处
- **Parlaktuna & Ozkan (2004)**: "Adaptive control of free-floating space manipulators using dynamically equivalent manipulator model", *Robotics and Autonomous Systems*
- **Papadopoulos & Dubowsky (1991)**: "On the nature of control algorithms for free-floating space manipulators", *IEEE Trans. Robot. Autom.*
- **曾岑 (2013)**: "在轨服务空间机械臂运动及任务规划方法研究", 大连理工大学博士论文

### 数学模型

自由漂浮空间机械臂的Lagrange动力学方程：

$$\bar{M}(\bar{q})\ddot{\bar{q}} + \bar{C}(\bar{q}, \dot{\bar{q}})\dot{\bar{q}} = \bar{\tau}$$

其中：
- $\bar{q} = [\phi, \theta, \psi, \theta_2, \cdots, \theta_{n+1}]^T \in \mathbb{R}^{n+3}$：广义坐标（基座3个姿态角 + n个关节角）
- $\bar{M}(\bar{q}) \in \mathbb{R}^{(n+3) \times (n+3)}$：惯性矩阵
- $\bar{C}(\bar{q}, \dot{\bar{q}})$：科里奥利力和离心力向量
- $\bar{\tau} = [0, 0, 0, \tau_2, \cdots, \tau_{n+1}]^T$：关节力矩向量

系统总动能：
$$T = \sum_{i=1}^{n+1} \left[ \frac{1}{2}m_i(\dot{\bar{\rho}}_i)^T \dot{\bar{\rho}}_i + \frac{1}{2}\bar{\omega}_i^T \bar{R}_i^0 \bar{I}_i (\bar{R}_i^0)^T \bar{\omega}_i \right]$$

### 简述
该模型基于拉格朗日力学，将自由漂浮空间机械臂视为多体系统。由于系统不受外力作用，势能为零，总能量等于动能。该模型**具有非线性参数化特性**，无法直接使用基于线性参数化模型的自适应控制方法。

---

## 二、Newton-Euler方程（牛顿-欧拉动力学方法）

### 模型出处
- **Yoshida & Umetani (1993)**: 引自多篇空间机器人综述文献
- **丛佩超 (2009)**: "空间机械臂抓取目标的碰撞前构型规划与控制问题研究", 哈尔滨工业大学博士论文

### 数学模型

多体系统动力学方程（含外部作用力）：

$$\begin{bmatrix} \boldsymbol{H}_b & \boldsymbol{H}_{bm} \\ \boldsymbol{H}_{bm}^T & \boldsymbol{H}_m \end{bmatrix} \begin{bmatrix} \ddot{\boldsymbol{x}}_b \\ \ddot{\phi} \end{bmatrix} + \begin{bmatrix} \boldsymbol{c}_b \\ \boldsymbol{c}_m \end{bmatrix} = \begin{bmatrix} \mathcal{F}_b \\ \boldsymbol{\tau} \end{bmatrix} + \begin{bmatrix} \boldsymbol{J}_b^T \\ \boldsymbol{J}_m^T \end{bmatrix} \mathcal{F}_h$$

其中：
- $\dot{\boldsymbol{x}}_b = (\boldsymbol{v}_b^T, \omega_b^T)^T$：基座线速度和角速度
- $\boldsymbol{H}_b \in \mathbb{R}^{6 \times 6}$：基座惯性矩阵
- $\boldsymbol{H}_m \in \mathbb{R}^{n \times n}$：机械臂惯性矩阵
- $\boldsymbol{H}_{bm} \in \mathbb{R}^{6 \times n}$：基座-机械臂耦合惯性矩阵
- $\mathcal{F}_b$：作用在基座质心的力和力矩
- $\mathcal{F}_h$：作用在机械臂末端的力和力矩
- $\boldsymbol{\tau}$：关节力矩

### 简述
Newton-Euler方法从单个刚体出发，通过递推方式建立系统动力学方程。该方法**计算效率高（复杂度为$O(n)$）**，适合实时控制。相比Lagrange方法，Newton-Euler方法可以更直观地处理碰撞力和约束力，特别适合空间机器人在轨捕获过程的碰撞动力学分析。

---

## 三、广义雅可比矩阵（Generalized Jacobian Matrix, GJM）

### 模型出处
- **Umetani & Yoshida (1989)**: "Resolved motion rate control of space manipulators using generalized Jacobian matrix", *IEEE Trans. Robot. Autom.*, 5(3):303-314
- **曾岑 (2013)**: "在轨服务空间机械臂运动及任务规划方法研究"
- **丛佩超 (2009)**: "空间机械臂抓取目标的碰撞前构型规划与控制问题研究"

### 数学模型

末端执行器速度与关节速度的关系：

$$\begin{bmatrix} \mathbf{v}_e \\ \boldsymbol{\omega}_e \end{bmatrix} = J^*(\Psi_b, \Theta, m_i, I_i) \dot{\Theta}$$

广义雅可比矩阵的构成：

$$J^* = J_m - J_b I_b^{-1} I_M$$

其中各分量：
- **机械臂雅可比矩阵** $J_m$：由矢量积法得出
- **基座运动雅可比矩阵** $J_b$：包含基座平移和旋转
- **系统惯性矩阵** $I_b$：包含系统总质量和质心位置
- **惯性耦合矩阵** $I_M$：包含各连杆质量、惯量与几何参数

动量守恒约束：

$$M_s \dot{r}_g = 0 \quad \text{(线动量守恒)}$$
$$H_s \omega_b + H_M \dot{\Theta} = 0 \quad \text{(角动量守恒)}$$

基座速度与关节速度的耦合关系：

$$\begin{bmatrix} v_b \\ \omega_b \end{bmatrix} = -I_b^{-1} I_M \dot{\Theta}$$

### 简述
广义雅可比矩阵将线动量和角动量守恒方程与系统运动学方程结合，建立了**末端执行器速度与关节速度之间的映射关系**。与传统雅可比矩阵不同，$J^*$不仅与几何参数有关，还与惯性参数（质量、转动惯量）有关。该模型的奇异性是**路径相关**的（动力学奇异），而非纯运动学奇异。计算复杂度为$O(n^2)$。

---

## 四、动态等效机械臂模型（Dynamically Equivalent Manipulator, DEM）

### 模型出处
- **Parlaktuna & Ozkan (2004)**: "Adaptive control of free-floating space manipulators using dynamically equivalent manipulator model", *Robotics and Autonomous Systems*, 48(2-3):111-126
- **Liang, Xu & Bergerman (1998)**: "Mapping a space manipulator to a dynamically equivalent manipulator", *J. Dyn. Syst. Meas. Control*

### 数学模型

DEM的动力学方程（与固定基座机械臂形式相同）：

$$\bar{M}'(\bar{q}')\ddot{\bar{q}}' + \bar{C}'(\bar{q}', \dot{\bar{q}}')\dot{\bar{q}}' = \bar{\tau}'$$

**参数映射关系**：

质量映射：
$$m_i' = m_i \frac{(\sum_{k=1}^{n+1} m_k)^2}{\sum_{k=1}^{i-1} m_k \sum_{k=1}^{i} m_k}, \quad i = 2,\ldots,n+1$$

惯性张量映射：
$$\bar{I}_i' = \bar{I}_i, \quad i = 1,\ldots,n+1$$

几何参数映射：
$$\bar{W}_1 = \frac{\bar{R}_1 m_1}{\sum_{k=1}^{n+1} m_k}$$
$$\bar{l}_{ci} = \bar{L}_i \left(\frac{\sum_{k=1}^{i-1} m_k}{\sum_{k=1}^{n+1} m_k}\right), \quad i = 2,\ldots,n+1$$

DEM动力学的重要性质：
1. $\bar{M}'(\bar{q}')$对称正定
2. 矩阵$(\dot{\bar{M}}' - 2\bar{C}')$是斜对称的
3. **可以线性参数化**：$\bar{M}'\ddot{\bar{q}}' + \bar{C}'\dot{\bar{q}}' = \bar{Y}(\bar{q}', \dot{\bar{q}}', \ddot{\bar{q}}')\bar{\varphi}$

### 简述
DEM将自由漂浮空间机械臂映射为一个固定基座机械臂（第一个关节为被动球关节），保持运动学和动力学等效性。**核心优势**是其动力学方程可以线性参数化，使得自适应控制方法得以实施，而无需使用反作用轮或推进器控制基座。

---

## 五、虚拟机械臂模型（Virtual Manipulator, VM）

### 模型出处
- **Vafa & Dubowsky (1987, 1990)**: 引自多篇文献，原始提出于IEEE International Conference on Robotics and Automation
- **曾岑 (2013)**: "在轨服务空间机械臂运动及任务规划方法研究"

### 数学模型

虚拟机械臂的基本定义：
- 将实际空间机械臂和基座的**共同质心**作为虚拟基座（Virtual Ground, VG）
- 虚拟机械臂是一个**无质量的运动链**
- 第一个关节是**被动球关节**（3个转动自由度）
- 虚拟基座在不受外力时**位置保持不变**

虚拟机械臂末端执行器位姿与实际空间机械臂位姿**一致**。

### 简述
VM方法基于线动量和角动量守恒定律，通过质心分析将自由漂浮系统转化为一个可以使用地面机器人建模方法的理想机械臂。**优点**是可以将成熟的地面机器人控制理论直接应用于空间机器人；**缺点**是改变了系统结构，模型不直观，需要大量前期处理工作。

---

## 六、Kane方程（Kane's Method）

### 模型出处
- **Jia & Shan (2019)**: "Finite-Time Trajectory Tracking Control of Space Manipulator Under Actuator Saturation", *IEEE Trans. Ind. Electron.*, 66(12):9456-9466

### 数学模型

广义惯性力和广义主力方程：

$$\boldsymbol{F}_{Ii} + \boldsymbol{F}_{Ai} = \mathbf{0}, \quad i = 1,\ldots,n$$

广义惯性力：
$$\boldsymbol{F}_{Ii} = \sum_{j=1}^{N} \int_{j}^{p} \boldsymbol{V}_{mj}^{i} \cdot \boldsymbol{a}_{mj} \, dm^{j}$$

惯性速度和角速度：
$$\boldsymbol{v}_{mj} = \sum_{i=1}^{n} {}^{p}\boldsymbol{V}_{mj}^{i} u_{i} + \boldsymbol{v}_{mjt}$$
$$\boldsymbol{\omega}_{mj} = \sum_{i=1}^{n} {}^{p}\boldsymbol{\Omega}_{mj}^{i} u_{i} + \boldsymbol{\omega}_{mjt}$$

其中：
- $u_i$：广义速度（generalized speed）
- ${}^{p}\boldsymbol{V}_{mj}^{i}$：偏速度（partial velocity）
- ${}^{p}\boldsymbol{\Omega}_{mj}^{i}$：偏角速度（partial angular velocity）
- $\boldsymbol{a}_{mj}$：加速度

### 简述
Kane方法是一种面向计算机实现的多体系统建模方法，**比Lagrange方法和Newton-Euler方法更高效**（计算复杂度为$O(n)$），特别适合多关节机械臂的动力学计算。该方法通过偏速度和偏角速度的概念，避免了约束力的显式求解。

## 七、SE(3)上的几何动力学模型（基于Lie群方法）

### 模型出处
- **Xu, Chen, Wen & Jin (2024)**: "Predefined-Time Tracking Control of a Free-Flying Space Robot on SE(3)", *IEEE Trans. Aerospace and Electronic Systems*, vol. 60, no. 5, pp. 5906–5919 [1]

### 数学模型

**1. 运动学模型（微分运动学）**

基座航天器：
$$\dot{\boldsymbol{g}}_s = \boldsymbol{g}_s \boldsymbol{V}_s^{\wedge} \tag{16a}$$

末端执行器：
$$\dot{\boldsymbol{g}}_e = \boldsymbol{g}_e \boldsymbol{V}_e^{\wedge} \tag{16b}$$

其中 $\boldsymbol{g}_s, \boldsymbol{g}_e \in SE(3)$ 为位姿矩阵，$\boldsymbol{V}_s, \boldsymbol{V}_e \in \mathbb{R}^6$ 为体坐标系下的速度旋量（twist）。

末端执行器速度旋量与基座和关节速度的关系：
$$\boldsymbol{V}_e = \mathrm{Ad}_{g_{se}}^{-1} \boldsymbol{V}_s + \boldsymbol{J}_e(\boldsymbol{\phi})\dot{\boldsymbol{\phi}} \tag{14}$$

其中 $\boldsymbol{J}_e(\boldsymbol{\phi}) \in \mathbb{R}^{6 \times 6}$ 为末端执行器的几何雅可比矩阵（geometric Jacobian）：
$$\boldsymbol{J}_e(\boldsymbol{\phi}) = \left[ \left(\boldsymbol{g}_{se}^{-1}\frac{\partial \boldsymbol{g}_{se}}{\partial \phi_1}\right)^{\vee} \quad \left(\boldsymbol{g}_{se}^{-1}\frac{\partial \boldsymbol{g}_{se}}{\partial \phi_2}\right)^{\vee} \cdots \left(\boldsymbol{g}_{se}^{-1}\frac{\partial \boldsymbol{g}_{se}}{\partial \phi_6}\right)^{\vee} \right] \tag{15}$$

**2. 关节空间动力学方程**

$$\boldsymbol{M}(\boldsymbol{\phi})\dot{\boldsymbol{\psi}} + \boldsymbol{C}(\boldsymbol{\phi}, \boldsymbol{\psi})\boldsymbol{\psi} = \boldsymbol{F} \tag{27}$$

其中广义速度向量：
$$\boldsymbol{\psi} = \begin{bmatrix} \boldsymbol{V}_s \\ \dot{\boldsymbol{\phi}} \end{bmatrix} \in \mathbb{R}^{12} \tag{22}$$

惯性矩阵：
$$\boldsymbol{M}(\boldsymbol{\phi}) = \boldsymbol{M}_s + \sum_{i=1}^{6} \boldsymbol{M}_i(\boldsymbol{\phi}) \in \mathbb{R}^{12 \times 12} \tag{26}$$

**3. 工作空间动力学方程**

将动力学从关节空间转换到工作空间，定义工作空间速度：
$$\boldsymbol{\vartheta} = \begin{bmatrix} \boldsymbol{V}_s \\ \boldsymbol{V}_e \end{bmatrix} \in \mathbb{R}^{12} \tag{29}$$

工作空间动力学方程：
$$\widetilde{\boldsymbol{M}}(\boldsymbol{\phi})\dot{\boldsymbol{\vartheta}} + \widetilde{\boldsymbol{C}}(\boldsymbol{\phi}, \boldsymbol{\psi})\boldsymbol{\vartheta} = \widetilde{\boldsymbol{F}} \tag{38}$$

其中：
$$\widetilde{\boldsymbol{M}}(\boldsymbol{\phi}) = \boldsymbol{J}_p^{-T}\boldsymbol{M}(\boldsymbol{\phi})\boldsymbol{J}_p^{-1}$$
$$\widetilde{\boldsymbol{C}}(\boldsymbol{\phi}, \boldsymbol{\psi}) = -\boldsymbol{J}_p^{-T}\boldsymbol{M}(\boldsymbol{\phi})\boldsymbol{J}_p^{-1}\dot{\boldsymbol{J}}_p\boldsymbol{J}_p^{-1} + \boldsymbol{J}_p^{-T}\boldsymbol{C}(\boldsymbol{\phi}, \boldsymbol{\psi})\boldsymbol{J}_p^{-1}$$

工作空间雅可比矩阵：
$$\boldsymbol{J}_p = \begin{bmatrix} \boldsymbol{I} & \boldsymbol{0} \\ \mathrm{Ad}_{g_{se}}^{-1} & \boldsymbol{J}_e(\boldsymbol{\phi}) \end{bmatrix} \tag{31}$$

**4. 跟踪误差运动学**

定义位姿误差函数（configuration error function）：
$$U_s = k_s \Gamma(\widetilde{\boldsymbol{R}}_s) + \frac{h_s}{2}\widetilde{\boldsymbol{p}}_s^T\widetilde{\boldsymbol{p}}_s \tag{47}$$

其中 $\Gamma(\widetilde{\boldsymbol{R}}_s) = 2 - \sqrt{\mathrm{tr}(\widetilde{\boldsymbol{R}}_s) + 1}$ 为旋转误差函数。

跟踪误差运动学：
$$\dot{\boldsymbol{\delta}} = \boldsymbol{\Xi}\widetilde{\boldsymbol{\vartheta}} \tag{60}$$

其中 $\boldsymbol{\delta} = [\boldsymbol{\delta}_s^T \quad \boldsymbol{\delta}_e^T]^T \in \mathbb{R}^{12}$ 为位姿跟踪误差。

**5. 预定义时间控制律**

$$\widetilde{\boldsymbol{F}} = \widetilde{\boldsymbol{C}}(\boldsymbol{\phi}, \boldsymbol{\psi})\boldsymbol{\vartheta} - \widetilde{\boldsymbol{M}}(\boldsymbol{\phi})\left(\boldsymbol{\lambda} - \dot{\boldsymbol{\varpi}} + \frac{1}{\epsilon_2}\boldsymbol{\mu}^T + \frac{\pi}{\epsilon_2 \alpha_3 T_p}\left(\left(\frac{\epsilon_2}{2}\right)^{\frac{2-\alpha_3}{2}}\boldsymbol{\rho}_2\|\boldsymbol{\rho}_2\|^{-\alpha_3} + \left(\frac{\epsilon_2}{2}\right)^{\frac{2+\alpha_3}{2}}\boldsymbol{\rho}_2\|\boldsymbol{\rho}_2\|^{\alpha_3}\right)\right) \tag{74}$$

预定义收敛时间：
$$T_m = \frac{3^{\underline{\alpha}/2}\overline{\alpha}}{\underline{\alpha}}T_p \tag{77}$$

### 模型特点与创新点

1. **SE(3)几何建模**：基于特殊欧几里得群SE(3)和Lie代数se(3)建立运动学和动力学模型，避免了传统欧拉角和四元数表示中的奇异性（singularity）和非唯一性（nonuniqueness）问题。

2. **全局唯一表示**：SE(3)能够全局且唯一地描述刚体的耦合旋转和平移运动，解决了传统姿态表示方法的局限性。

3. **非线性减少**：由于速度和变换都在体坐标系中表达，推导出的动力学方程的非线性显著降低。

4. **预定义时间稳定性**：收敛时间的上界$T_m$可以直接作为控制参数预设，与初始状态无关。

5. **几何动态面控制**：结合几何描述和动态面控制技术，设计了新的控制器结构。

### 应用场景

该模型适用于自由飞行空间机器人的**位姿跟踪控制**问题，特别是需要在预定义时间内完成基座航天器和末端执行器同时跟踪期望轨迹的任务，如在轨服务（on-orbit servicing）、空间碎片清除（space debris removal）等。

---

## 模型对比总结（更新版）

| 序号 | 模型类型 | 主要特点 | 适用场景 | 关键文献 |
|:---|:---|:---|:---|:---|
| 1 | **Lagrange方程** | 基于能量，非线性参数化 | 理论分析、仿真 | Papadopoulos & Dubowsky (1991) |
| 2 | **Newton-Euler方程** | 递推形式，可处理约束力 | 实时控制、碰撞分析 | Yoshida (1993) |
| 3 | **广义雅可比矩阵** | 结合动量守恒，路径相关奇异 | 速度级运动学控制 | Umetani & Yoshida (1989) |
| 4 | **动态等效机械臂** | 可线性参数化，映射为固定基座模型 | 自适应控制设计 | Liang, Xu & Bergerman (1998) |
| 5 | **虚拟机械臂** | 质心基座，运动学等效 | 控制理论移植 | Vafa & Dubowsky (1987) |
| 6 | **Kane方程** | 高效计算机实现 | 多体系统实时仿真 | Jia & Shan (2019) |
| 7 | **SE(3)几何模型** | Lie群全局表示，无奇异性，预定义时间控制 | 位姿跟踪、几何控制 | **Xu et al. (2024) [1]** |

---

**参考文献：**

[1] S. Xu, T. Chen, H. Wen, and D. Jin, "Predefined-Time Tracking Control of a Free-Flying Space Robot on SE(3)," *IEEE Trans. Aerosp. Electron. Syst.*, vol. 60, no. 5, pp. 5906–5919, Oct. 2024.


**主要参考文献**：
1. Umetani Y, Yoshida K. Resolved motion rate control of space manipulators using generalized Jacobian matrix. IEEE Trans. Robot. Autom., 1989, 5(3):303-314.
2. Papadopoulos E, Dubowsky S. On the nature of control algorithms for free-floating space manipulators. IEEE Trans. Robot. Autom., 1991, 7(6):750-758.
3. Parlaktuna O, Ozkan M. Adaptive control of free-floating space manipulators using dynamically equivalent manipulator model. Robotics and Autonomous Systems, 2004, 48(2-3):111-126.
4. Jia S, Shan J. Finite-Time Trajectory Tracking Control of Space Manipulator Under Actuator Saturation. IEEE Trans. Ind. Electron., 2019, 66(12):9456-9466.
5. 曾岑. 在轨服务空间机械臂运动及任务规划方法研究[D]. 大连理工大学, 2013.
6. 丛佩超. 空间机械臂抓取目标的碰撞前构型规划与控制问题研究[D]. 哈尔滨工业大学, 2009.