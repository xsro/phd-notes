# 同时估计 $\dot{x}=Ax,\ y=Cx$ 中状态 $x$、矩阵 $A$、输出矩阵 $C$

## 问题背景

系统：
$$
\begin{cases}
\dot x = A x \\
y = C x
\end{cases}
$$

其中 $x\in\mathbb R^{n},\ A\in\mathbb R^{n\times n},\ C\in\mathbb R^{p\times n}$；无输入 $u$，为自治线性系统。

---

## 核心结论

1. 标准 Luenberger 观测器：**必须已知 $A, C$，仅估计状态 $x$**，不能辨识 $A, C$。
2. 同时估计 $x, A, C$ 不再是纯线性观测问题，属于**状态-参数联合估计**，主流两条路线：**自适应观测器**、**增广状态滤波 (EKF)**。
3. 自治系统（$u=0$）存在可辨识性难点：必须系统充分激励，模态充分激发，否则 $A, C$ 无法唯一收敛到真值。

---

## 一、已知 $A, C$，只观测状态 $x$（Luenberger 观测器）

观测器：
$$
\dot{\hat x}=A\hat x + L(y-C\hat x)
$$

观测误差 $e=x-\hat x$：
$$
\dot e = (A-LC)e
$$

设计 $L$ 使 $A-LC$ 赫尔维茨，则 $e\to0$。

> **局限**：$A, C$ 全部作为先验模型，完全未知时此结构失效。

---

## 二、自适应观测器框架：同时估计 $\hat x, \hat A, \hat C$

### 参数回归形式

改写系统，把参数写成参数回归形式：
$$
\begin{aligned}
\dot x &= A x = \big(x^\mathrm T \otimes I_n\big)\cdot \mathrm{vec}(A) \\
y &= C x = \big(x^\mathrm T \otimes I_p\big)\cdot \mathrm{vec}(C)
\end{aligned}
$$

记：
- $\theta_1=\mathrm{vec}(A)\in\mathbb R^{n^2}$：$A$ 向量化参数
- $\theta_2=\mathrm{vec}(C)\in\mathbb R^{pn}$：$C$ 向量化参数

则：
$$
\dot x=\Phi_1(x)\theta_1,\quad y=\Phi_2(x)\theta_2
$$

### 自适应观测器结构

$$
\begin{cases}
\dot{\hat x}= \hat A \hat x + L\big(y-\hat C \hat x\big) \\
\dot{\hat \theta}_1 = \Gamma_1 \Phi_1(\hat x)^\mathrm T P e_y \\
\dot{\hat \theta}_2 = \Gamma_2 \Phi_2(\hat x)^\mathrm T P e_y
\end{cases}
$$

其中：
- $e_y = y-\hat C\hat x$：输出误差；
- $\Gamma_1>0,\ \Gamma_2>0$：自适应学习增益矩阵；
- $P>0$：Lyapunov 方程 $(A-LC)^\mathrm T P+P(A-LC)=-Q$ 的正定解；
- $\hat A = \mathrm{vec}^{-1}(\hat\theta_1),\ \hat C=\mathrm{vec}^{-1}(\hat\theta_2)$，由向量还原矩阵。

### 误差动力学

状态误差 $e_x=x-\hat x$：
$$
\dot e_x = A x -\big(\hat A\hat x+L(y-\hat C\hat x)\big)
$$

代入 $y=Cx$：
$$
\dot e_x = (A-LC)e_x + \big(A-\hat A\big)\hat x + L\big(C-\hat C\big)\hat x
$$

右侧多出两项参数误差耦合项，**不再是线性时不变误差系统**，整体是**非线性自适应系统**。

### 收敛条件（自治系统极易不满足）

1. **持续激励 (PE) 条件**：对于自治系统 $\dot x=Ax$，轨迹由 $A$ 的模态决定。只有当系统轨迹 $x(t)$ 充分丰富，$\Phi_1,\Phi_2$ 满足持续激励：
   $$
   \exists \alpha>0,\ T>0,\quad \int_{t}^{t+T}\Phi(\tau)^\mathrm T\Phi(\tau)d\tau \succeq \alpha I
   $$
   才能保证 $\hat A\to A,\ \hat C\to C$。
   
   > ⚠️ 如果系统渐近趋于零（$A$ 稳定），$x(t)\to0$，回归矩阵 $\Phi\to0$，**PE 失效，参数 $A, C$ 无法收敛**。自治稳定系统是自适应辨识的典型坑。

2. **可观测条件**：$(A, C)$ 可观测，保证状态可恢复。
3. 不能全局保证收敛，一般只能证明有界性；参数收敛需要 PE。

---

## 三、增广状态 EKF：把 $A, C$ 元素扩进状态向量

### 增广系统构造

把参数当作常数状态，构造增广系统。增广状态：
$$
x_\mathrm{aug}=
\begin{bmatrix}
x \\
\mathrm{vec}(A) \\
\mathrm{vec}(C)
\end{bmatrix}
\in \mathbb R^{n+n^2+pn}
$$

原系统：
$$
\begin{cases}
\dot x = A x \\
\mathrm{vec}(\dot A)=0 \\
\mathrm{vec}(\dot C)=0 \\
y = C x
\end{cases}
$$

增广系统状态方程是非线性的：$\dot x_\mathrm{aug}=f(x_\mathrm{aug}),\ y=h(x_\mathrm{aug})$。

套用 EKF：每一时刻对 $f, h$ 做雅可比线性化，做预测-更新循环，同时输出：
- $\hat x$：状态估计
- $\hat A=\mathrm{vec}^{-1}(\widehat{\mathrm{vec}(A)})$
- $\hat C=\mathrm{vec}^{-1}(\widehat{\mathrm{vec}(C)})$

### EKF 优缺点

✅ 实现直观，不需要手动推导自适应律；

❌
1. **维度爆炸**：$n=4$ 时增广维度 $4+16+4p$，计算量大；
2. **雅可比截断误差**，容易发散；
3. 同样强依赖持续激励；自治稳定系统 $x\to0$ 时参数估计漂移。

> EKF 是工程常用，但理论上不保证全局收敛。

---

## 四、离线方案：子空间辨识（批量，非观测器）

如果可以采集一段时间序列 $\{y(t)\}$，自治系统子空间辨识可以批量算出 $A, C$，之后再用 Luenberger 算 $x$。

$$
Y = \mathcal O_i X
$$

$\mathcal O_i$ 是可观矩阵。

- **优点**：批量数据，不需要实时在线；
- **缺点**：不是实时观测器，不能边运行边更新 $A, C$。

---

## 五、可辨识性本质问题：为什么不能随便把 $A, C$ 都估出来

对于 $\dot x=Ax,\ y=Cx$，存在等价变换：令 $\bar x=T x$，$T$ 可逆。
$$
\begin{cases}
\dot{\bar x}= T A T^{-1}\bar x = \bar A \bar x \\
y = C T^{-1}\bar x=\bar C \bar x
\end{cases}
$$

输入输出 $y(t)$ 完全一样，但是 $A\to TAT^{-1},\ C\to CT^{-1}$。

> **重要结论**：仅靠输出 $y(t)$，只能辨识系统的等价相似类，不能唯一确定 $A, C$，除非施加额外约束（例如固定坐标系、固定某些矩阵元素）。

这是自治系统状态-参数联合估计最容易忽略的理论点。即使 PE 满足，不加约束，得到的 $(\hat A,\hat C)$ 只是和真实系统相似等价，不一定等于原矩阵。

---

## 六、方案对比汇总

| 方法 | 估计 $x$ | 估计 $A$ | 估计 $C$ | 纯线性 | 在线实时 | 关键约束 |
|---|---|---|---|---|---|---|
| Luenberger 观测器 | ✅ | ❌ | ❌ | ✅ | ✅ | $A, C$ 已知 |
| 自适应观测器 | ✅ | ✅ | ✅ | ❌ | ✅ | PE 条件；相似等价；自治稳定易失效 |
| EKF 增广滤波 | ✅ | ✅ | ✅ | ❌ | ✅ | PE；维数爆炸；易漂移 |
| 子空间辨识 | ✅（后处理） | ✅ | ✅ | – | ❌ | 批量时序数据 |

### 工程实践建议

1. 若系统是自治且稳定，$x(t)$ 衰减到 0：**不建议在线同时估计 $A, C$**，激励不足参数会飘；优先离线辨识得 $A, C$，再用观测器看状态。
2. 若必须在线：尽量保证系统有丰富动态；给 $A, C$ 施加结构约束（如已知部分元素）消除相似变换歧义。
3. 自适应观测器理论性强；EKF 更适合快速原型仿真。

---

> 如需进一步推导自适应观测器的完整 Lyapunov 稳定性证明，或需要仿真代码示例，请告知。