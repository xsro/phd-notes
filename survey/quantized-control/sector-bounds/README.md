# 量化器扇区有界条件（Sector-Bounded Conditions）可视化

基于 M. Fu 和 L. Xie, *"The sector bound approach to quantized feedback control,"* IEEE Trans. Autom. Control, vol. 50, no. 11, pp. 1698–1711, 2005 的核心理论，绘制对数量化器与一般（均匀）量化器的扇区有界条件。

---

## 0. 快速开始

```bash
cd sector-bounds
uv run --with matplotlib python quantizer_sector_bounds.py
```

依赖（`uv` 自动管理）：
- `matplotlib`
- `numpy`

---

## 1. 量化器的乘性误差模型

设 $v \in \mathbb{R}$ 为量化器输入，$f(v)$ 为量化输出。量化误差可以统一建模为**乘性不确定性**：

$$
f(v) = \bigl(1 + \Delta(v)\bigr)\, v,
\qquad
\Delta(v) \triangleq \frac{f(v) - v}{v}.
\tag{1}
$$

扇区有界（sector-bounded）条件指的是：$\Delta(v)$ 被约束在一个固定的闭区间内，即

$$
\Delta(v) \in [\delta^{-},\,\delta^{+}], \qquad \forall v \neq 0.
\tag{2}
$$

这一模型的关键是：**量化误差不再被孤立地视为加性扰动，而是以相对误差（乘性）的形式统一嵌入到控制律中**。Fu & Xie 证明了：对于静态对数量化器，该乘性模型是非保守的（non-conservative）——量化反馈镇定问题与对应的扇区不确定鲁棒镇定问题完全等价。

---

## 2. 对数量化器

### 2.1 定义

对数量化器（logarithmic quantizer）的量化电平族为

$$
\mathcal{U} = \bigl\{\pm u^{(i)} : u^{(i)} = \rho^{i} u^{(0)},\; i = 0, \pm 1, \pm 2, \dots\bigr\} \cup \{0\},
\tag{3}
$$

其中 $\rho \in (0,1)$ 为**量化密度**（quantization density），$u^{(0)} > 0$ 为基准电平。量化密度 $\rho$ 越小，相邻电平间距越大，量化越粗。

### 2.2 分段定义与判决边界

对数量化器 $f(v)$ 的完整分段表达式（以 $\mathbb{R}_{\ge 0}$ 为例）为

$$
f(v)=
\begin{cases}
\rho^{i} u^{(0)}, &
\displaystyle \frac{1+\rho}{2}\cdot\rho^{i} u^{(0)} \le |v| <
\frac{1+\rho}{2\rho}\cdot\rho^{i} u^{(0)},\quad i \in \mathbb{Z},\\[2.5ex]
0, & v = 0.
\end{cases}
\tag{4}
$$

即对于 $v \neq 0$，量化电平为 $u_i = \rho^{\,i} u^{(0)}$，**判决边界由扇区参数 $\delta$ 决定**：

$$
\underbrace{u_i \cdot \frac{1+\rho}{2}}_{\text{下边界}}
\;\le\; |v| \;<
\underbrace{u_i \cdot \frac{1+\rho}{2\rho}}_{\text{上边界}},
\qquad
\delta = \frac{1-\rho}{1+\rho}.
\tag{5}
$$

由于 $\frac{1+\rho}{2\rho} = \frac{1+\rho}{2}\cdot\frac{1}{\rho}$，上级上边界与下級下边界自然衔接：

$$
u_i \cdot \frac{1+\rho}{2\rho} = u_{i-1} \cdot \frac{1+\rho}{2}.
\tag{6}
$$

因此区间在 $\mathbb{R}_{>0}$ 上无缝覆盖：$i \to +\infty$ 时区间收缩至 0，$i \to -\infty$ 时区间趋向 $+\infty$。

**注**：与有限量化器不同，这里的 $i$ 取遍所有整数，$v \neq 0$ 的每一点都落在唯一的一个非零电平区间内，**没有死区**。理论分析中不需要特殊的默认输出值。Fu & Xie 原文第 II 节使用这一无穷电平定义，保证了扇区有界条件 $|\Delta(v)| \le \delta$ 对 **所有** $v \neq 0$ 成立。

### 2.3 扇区界

定义

$$
\delta \triangleq \frac{1 - \rho}{1 + \rho},
\tag{7}
$$

则对数量化器满足**对称扇区有界**条件（对所有 $v \neq 0$）：

$$
\Delta(v) \in [-\delta,\;\delta], \qquad \forall v \neq 0.
\tag{8}
$$

即

$$
-\delta\,|v| \;\le\; f(v) - v \;\le\; \delta\,|v|, \qquad \forall v \neq 0.
\tag{9}
$$

量化器输出始终位于两条射线 $f = (1+\delta)v$ 与 $f = (1-\delta)v$ 所夹的扇区内。

### 2.4 输入-输出特性与扇区参数

下图左侧展示了无穷级对数量化器的阶梯状输入-输出特性（$i \in \mathbb{Z}$，无死区）。右侧展示了对应的扇区参数 $\Delta(v) = (f(v)-v)/v$：在所有 $v \neq 0$ 处严格约束在 $[-\delta, \delta]$ 内。

<p align="center">
  <img src="figures/log_quantizer_characteristic.png" width="700" alt="对数量化器 I/O 特性及扇区参数">
  <br>
  <em>图 1：对数量化器 ($\rho=0.5$) 的输入-输出特性（左）及扇区参数 $\Delta(v)$（右）</em>
</p>

### 2.5 关于死区的说明

有限量化器和均匀量化器在小信号区间需要设置默认输出值 $u_0$（Fu & Xie 原文第 II 节）：若 $u_0 = 0$ 则 $\delta^- = -1$；若 $u_0 \neq 0$ 则 $\delta^+ = \infty$。

但对数量化器不同。由于 $i$ 取遍所有整数，电平数**可数无穷**且向零任意逼近，**不需要死区**。扇区 $[-\delta, \delta]$ 对 $\forall v \neq 0$ 全局成立。

实际硬件中只能实现有限电平，此时死区不可避免——低于最小电平的输入被映射为 0，$\Delta(v) \equiv -1$ 超出名义扇区。这种情况下的处理方法见有限字母表量化器（finite-alphabet quantizer）相关文献。

### 2.6 关键性质

1. **相对误差有界**：$|f(v) - v| \le \delta\,|v|$ 对 $\forall v \neq 0$ 成立，误差与信号幅值成比例。
2. **保符号性**：$f(v)\,v \ge 0$，等号当且仅当 $v = 0$。这是滑模控制中滑模面计算不受符号反转影响的关键保证。
3. **极限情况**：
   - $\rho \to 1$（量化极细）：$\delta \to 0$，退化至理想连续反馈。
   - $\rho \to 0$（量化极粗）：$\delta \to 1$，扇区扩展至整个第一/三象限。

### 2.7 量化密度与扇区界的对应

$$
\begin{array}{c|ccccc}
\rho & 0.1 & 0.3 & 0.5 & 0.7 & 0.9 \\ \hline
\delta & 0.818 & 0.538 & 0.333 & 0.176 & 0.053
\end{array}
$$

可见即使 $\rho = 0.5$，相对误差界已达 $33.3\%$。量化反馈的鲁棒性设计本质是在 $\rho$（通信效率）与 $\delta$（鲁棒裕度）之间权衡。

---

## 3. 一般（均匀）量化器

### 3.1 定义

均匀量化器以固定步长 $\Delta_u > 0$ 等距划分实轴：

$$
q_u(v) = \Delta_u \left\lfloor \frac{v}{\Delta_u} \right\rceil,
\qquad |q_u(v) - v| \le \frac{\Delta_u}{2}.
\tag{10}
$$

### 3.2 扇区界的不对称性

<p align="center">
  <img src="figures/general_quantizer_sector.png" width="700" alt="均匀量化器非对称扇区">
  <br>
  <em>图 2：一般（均匀）量化器的输入-输出特性（左）与非对称扇区参数 $\Delta(v)$（右），扇区 $[\delta^{-},\delta^{+}]$ 不对称且 $v \to 0$ 时无界</em>
</p>

均匀量化器的绝对误差虽有界 $\frac{\Delta_u}{2}$，但**相对误差**为

$$
\Delta(v) = \frac{q_u(v) - v}{v}.
\tag{11}
$$

当 $v \to 0$ 时，$|\Delta(v)| \to \infty$，因此均匀量化器无法用单一的有限区间 $[-\delta, \delta]$ 描述。实际中可取两个不对称参数：

$$
\Delta(v) \in [\delta^{-},\; \delta^{+}],
\qquad
\delta^{-} = \min_{v \neq 0} \Delta(v),\;
\delta^{+} = \max_{v \neq 0} \Delta(v).
\tag{12}
$$

对于有限饱和电平的情况，在大幅值处还会出现截断效应（saturation）：$\delta^{+} > 0$ 进一步扩大。

### 3.3 与对数量化器的对比

| 特性 | 对数量化器 | 均匀量化器 |
|---|---|---|
| 扇区形状 | 对称 $[-\delta,\delta]$ | 非对称 $[\delta^{-},\delta^{+}]$ |
| 相对误差界 | $\delta \in (0,1)$ | 在 $v \to 0$ 时无界 |
| 误差结构 | 乘性（相对误差比例缩放） | 加性（绝对误差恒定） |
| 电平等距 | 否（对数间距） | 是（均匀间距） |
| 全局镇定 | 无限电平可实现 | 有限电平时只能实际镇定 |

---

## 4. 扇区有界条件的控制理论含义

### 4.1 量化反馈镇定等价于鲁棒镇定

考虑离散 LTI 系统 $x_{k+1} = A x_k + B u_k$，量化状态反馈 $u_k = f(K x_k)$。将 (1) 代入得

$$
x_{k+1} = A x_k + B(1 + \Delta(K x_k)) K x_k.
\tag{13}
$$

Fu & Xie 的核心引理（Lemma II.1）表明：二次镇定条件

$$
\nabla V(x) = V\bigl((A + B(1+\Delta(Kx))K)x\bigr) - V(x) < 0, \quad \forall x \neq 0,
$$

等价于如下不确定系统的二次镇定问题：

$$
x_{k+1} = A x_k + B(1 + \Delta) K x_k, \qquad \Delta \in [-\delta, \delta],
\tag{14}
$$

其中 $\Delta$ 与状态无关。**这一等价性是无保守的**（non-conservative），因为对数量化器中 $\Delta(v)$ 可以取到扇区内的任意值。

### 4.2 最大允许扇区界

对于 SISO 系统，使 (14) 可二次镇定的最大 $\delta$ 为

$$
\delta_{\sup} = \frac{1}{\displaystyle\inf_{K} \bigl\| K(zI - A - BK)^{-1} B \bigr\|_{\infty}} = \frac{1}{\prod_i |\lambda_i^{u}|},
\tag{15}
$$

其中 $\lambda_i^{u}$ 为 $A$ 的不稳定特征值。对应的最粗量化密度为

$$
\rho_{\inf} = \frac{1 - \delta_{\sup}}{1 + \delta_{\sup}} = \frac{\prod_i |\lambda_i^{u}| - 1}{\prod_i |\lambda_i^{u}| + 1}.
\tag{16}
$$

**物理解释**：系统不稳定程度越高（$\prod |\lambda_i^{u}|$ 越大），容许的扇区界 $\delta_{\sup}$ 越小，所需量化密度 $\rho_{\inf}$ 越大（量化越细）。这是数据率定理（Data Rate Theorem）在扇区界语言下的等价表述。

### 4.3 MIMO 推广

对于 MIMO 系统，每个输入通道有独立的量化密度 $\rho_j$ 和扇区界 $\delta_j$。镇定条件转化为 $H_{\infty}$ 范数条件：存在对角缩放矩阵 $\Gamma > 0$ 使得

$$
\bigl\| \Lambda\,\Gamma\,K\,(zI - A - BK)^{-1}B\,\Gamma^{-1} \bigr\|_{\infty} < 1,
\qquad \Lambda = \operatorname{diag}\{\delta_1, \dots, \delta_m\}.
\tag{17}
$$

### 4.4 性能扩展

同样的扇区方法可直接扩展到 **LQR 性能控制**（quadratic cost）和 **$H_{\infty}$ 性能控制**，只需将二次型或 $H_{\infty}$ 范数约束与扇区不确定性联合处理，最终转化为 LMI 求解（见 Fu & Xie 原文第 V–VI 节）。

---

## 5. 在滑模控制中的意义

在量化滑模控制中，扇区有界条件直接联系到滑模的存在条件。

考虑滑模面 $s(x) = C x$，基于量化测量 $\hat{x} = q(x)$ 计算 $\hat{s} = C \hat{x}$。由 (1) 得

$$
\hat{s} = C\bigl(1 + \Delta(x)\bigr)x = s(x) + C\,\Delta(x)\,x.
\tag{18}
$$

滑模存在的切换增益条件（Bandyopadhyay & Behera 2018）为

$$
\rho > |c^{\!\top} B|\,d_0,
$$

其中 $d_0$ 为测量误差上界。利用扇区参数 $\delta$，可直接量化 $d_0$：

$$
d_0 = \max \|C\Delta(x)x\| \le \delta\,\|C\|\,\|x\|.
\tag{19}
$$

因此 $\delta$（即量化密度 $\rho$）直接决定了保证滑模存在所需的最小切换增益——**量化越粗，抖振越严重**。

---

## 7. 参考文献

1. M. Fu and L. Xie, "The sector bound approach to quantized feedback control," *IEEE Trans. Autom. Control*, vol. 50, no. 11, pp. 1698–1711, 2005.
2. N. Elia and S. K. Mitter, "Stabilization of linear systems with limited information," *IEEE Trans. Autom. Control*, vol. 46, no. 9, pp. 1384–1400, 2001.
3. B. Bandyopadhyay and A. K. Behera, *Event-triggered Sliding Mode Control*. Springer, 2018 (Chapter 6).
4. J. Lian and C. Li, "Event-triggered sliding mode control of uncertain switched systems via hybrid quantized feedback," *IEEE Trans. Autom. Control*, 2021.