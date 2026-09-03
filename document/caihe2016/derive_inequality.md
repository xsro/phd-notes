# 从变分公式到 Gronwall 不等式：详细推导

> 本文逐行解释 `cai2017_lemma3_xbar_bounded.tex` 中从 (C2) 的变分公式出发，结合 (C3)–(C4) 得到积分不等式，再用 Gronwall 不等式证明有界性的完整过程。

---

## 0. 符号与前提回顾

### 系统方程 (C2)

\[
\dot{z} = A_0 z + \Delta(t) z + r(t),
\]

其中：

- \(z(t) \in \mathbb{R}^n\) 是误差状态；
- \(A_0 = -\varepsilon C_\infty = -\varepsilon \begin{bmatrix} G & 0 \\ 0 & 0 \end{bmatrix}\) 是常矩阵；
- \(\Delta(t)\) 和 \(r(t)\) 是时变扰动。

### 扰动衰减估计 (C3)

\[
\|\Delta(t)\| + \|r(t)\| \leq c_0 e^{-\alpha(t-t_0)}, \qquad t \geq t_0.
\]

由此可推出分项估计：

\[
\|\Delta(t)\| \leq c_0 e^{-\alpha(t-t_0)}, \qquad
\|r(t)\| \leq c_0 e^{-\alpha(t-t_0)}.
\]

### 矩阵指数有界性 (C4)

\[
e^{A_0 t} = \begin{bmatrix} e^{-\varepsilon G t} & 0 \\ 0 & I_{n-k} \end{bmatrix},
\qquad
\|e^{A_0 t}\| \leq 1, \quad t \geq 0.
\]

这里使用的是谱范数（算子 2-范数）。由于 \(A_0\) 对称，谱范数等于谱半径，而 \(e^{A_0 t}\) 的特征值为 \(e^{-\varepsilon \lambda_i(G) t} \in (0,1]\) 和 \(1\)，故谱半径为 1。

---

## 1. 变分公式（常数变易法）

对于非齐次线性系统

\[
\dot{z} = A_0 z + f(t), \qquad z(t_0) = z_0,
\]

其中 \(f(t) = \Delta(t)z(t) + r(t)\)，其解可表示为：

\[
\boxed{
z(t) = e^{A_0(t-t_0)} z(t_0) + \int_{t_0}^{t} e^{A_0(t-s)} f(s) \, ds
}.
\tag{1}
\]

这就是变分公式（variation of constants / Duhamel 原理）。推导思路如下：

令 \(z(t) = e^{A_0(t-t_0)} y(t)\)，则 \(y(t_0) = z(t_0)\)。求导：

\[
\dot{z} = A_0 e^{A_0(t-t_0)} y + e^{A_0(t-t_0)} \dot{y}
= A_0 z + e^{A_0(t-t_0)} \dot{y}.
\]

与 \(\dot{z} = A_0 z + f(t)\) 对比得：

\[
e^{A_0(t-t_0)} \dot{y} = f(t)
\implies \dot{y} = e^{-A_0(t-t_0)} f(t).
\]

积分：

\[
y(t) = z(t_0) + \int_{t_0}^{t} e^{-A_0(s-t_0)} f(s) \, ds.
\]

代回 \(z(t) = e^{A_0(t-t_0)} y(t)\) 即得 (1)。

---

## 2. 第一步：对变分公式取范数

对 (1) 两边取范数，用三角不等式：

\[
\|z(t)\|
\leq \big\| e^{A_0(t-t_0)} z(t_0) \big\|
+ \left\| \int_{t_0}^{t} e^{A_0(t-s)} f(s) \, ds \right\|.
\tag{2}
\]

积分范数满足：

\[
\left\| \int_{t_0}^{t} g(s) \, ds \right\|
\leq \int_{t_0}^{t} \|g(s)\| \, ds,
\]

因此：

\[
\|z(t)\|
\leq \big\| e^{A_0(t-t_0)} z(t_0) \big\|
+ \int_{t_0}^{t} \big\| e^{A_0(t-s)} f(s) \big\| \, ds.
\tag{3}
\]

---

## 3. 第二步：处理矩阵指数项

### 3.1 第一项

利用算子范数的次乘性 \(\|XY\| \leq \|X\| \cdot \|Y\|\)：

\[
\big\| e^{A_0(t-t_0)} z(t_0) \big\|
\leq \|e^{A_0(t-t_0)}\| \cdot \|z(t_0)\|.
\]

由 (C4)，当 \(t \geq t_0\) 时 \(\|e^{A_0(t-t_0)}\| \leq 1\)，故：

\[
\big\| e^{A_0(t-t_0)} z(t_0) \big\| \leq \|z(t_0)\|.
\tag{4}
\]

### 3.2 第二项中的矩阵指数

同理，对被积函数中的矩阵指数：

\[
\big\| e^{A_0(t-s)} f(s) \big\|
\leq \|e^{A_0(t-s)}\| \cdot \|f(s)\|.
\]

因为 \(t \geq s \geq t_0\)，故 \(t-s \geq 0\)，由 (C4) 得 \(\|e^{A_0(t-s)}\| \leq 1\)，所以：

\[
\big\| e^{A_0(t-s)} f(s) \big\|
\leq \|f(s)\|.
\tag{5}
\]

---

## 4. 第三步：展开 \(f(s)\) 并放缩

回顾 \(f(s) = \Delta(s) z(s) + r(s)\)，代入 (5)：

\[
\big\| e^{A_0(t-s)} f(s) \big\|
\leq \|\Delta(s) z(s) + r(s)\|.
\]

用三角不等式拆开：

\[
\|\Delta(s) z(s) + r(s)\|
\leq \|\Delta(s) z(s)\| + \|r(s)\|.
\]

再用次乘性：

\[
\|\Delta(s) z(s)\| \leq \|\Delta(s)\| \cdot \|z(s)\|.
\]

因此：

\[
\big\| e^{A_0(t-s)} f(s) \big\|
\leq \|\Delta(s)\| \cdot \|z(s)\| + \|r(s)\|.
\tag{6}
\]

---

## 5. 第四步：代入扰动衰减估计 (C3)

由 (C3) 的分项估计：

\[
\|\Delta(s)\| \leq c_0 e^{-\alpha(s-t_0)}, \qquad
\|r(s)\| \leq c_0 e^{-\alpha(s-t_0)}.
\]

代入 (6) 得：

\[
\big\| e^{A_0(t-s)} f(s) \big\|
\leq c_0 e^{-\alpha(s-t_0)} \|z(s)\| + c_0 e^{-\alpha(s-t_0)}.
\tag{7}
\]

---

## 6. 第五步：组装成最终不等式

将 (4) 和 (7) 代入 (3)：

\[
\begin{aligned}
\|z(t)\|
&\leq \|z(t_0)\|
+ \int_{t_0}^{t} \Big(
c_0 e^{-\alpha(s-t_0)} \|z(s)\|
+ c_0 e^{-\alpha(s-t_0)}
\Big) \, ds \\[6pt]
&= \|z(t_0)\|
+ c_0 \int_{t_0}^{t} e^{-\alpha(s-t_0)} \|z(s)\| \, ds
+ c_0 \int_{t_0}^{t} e^{-\alpha(s-t_0)} \, ds.
\end{aligned}
\tag{8}
\]

这就是目标不等式：

\[
\boxed{
\|z(t)\|
\leq \|z(t_0)\|
+ c_0 \int_{t_0}^{t} e^{-\alpha(s-t_0)} \|z(s)\| \, ds
+ c_0 \int_{t_0}^{t} e^{-\alpha(s-t_0)} \, ds
}.
\]

---

## 7. 第六步：应用 Gronwall 不等式

不等式 (8) 的右端仍含有未知的 \(\|z(s)\|\) 在积分内，无法直接得出有界性。Gronwall 不等式可以处理这种"自指"型积分不等式。

### 7.1 整理成标准形式

先估计最后一项积分：

\[
c_0 \int_{t_0}^{t} e^{-\alpha(s-t_0)} \, ds
= c_0 \left[ -\frac{1}{\alpha} e^{-\alpha(s-t_0)} \right]_{t_0}^{t}
= \frac{c_0}{\alpha} \left(1 - e^{-\alpha(t-t_0)}\right)
\leq \frac{c_0}{\alpha}.
\]

将此项并入常数项，令：

\[
y(t) = \|z(t)\|, \qquad
a = \|z(t_0)\| + \frac{c_0}{\alpha}, \qquad
b(s) = c_0 e^{-\alpha(s-t_0)},
\]

则 (8) 可写为：

\[
y(t) \leq a + \int_{t_0}^{t} b(s) y(s) \, ds.
\tag{9}
\]

### 7.2 Gronwall 不等式（积分形式）

> **定理（Gronwall）**：若非负函数 \(y(t)\) 满足
> \[
> y(t) \leq a + \int_{t_0}^{t} b(s) y(s) \, ds,
> \qquad a \geq 0, \; b(s) \geq 0,
> \]
> 则
> \[
> y(t) \leq a \exp\!\left( \int_{t_0}^{t} b(s) \, ds \right).
> \]

### 7.3 代入计算

计算 \(b(s)\) 的积分：

\[
\int_{t_0}^{t} b(s) \, ds
= c_0 \int_{t_0}^{t} e^{-\alpha(s-t_0)} \, ds
\leq \frac{c_0}{\alpha}.
\]

应用 Gronwall 不等式：

\[
\begin{aligned}
y(t)
&\leq a \exp\!\left( \int_{t_0}^{t} b(s) \, ds \right) \\[4pt]
&\leq \left( \|z(t_0)\| + \frac{c_0}{\alpha} \right)
\exp\!\left( \frac{c_0}{\alpha} \right).
\end{aligned}
\tag{10}
\]

即：

\[
\boxed{
\|z(t)\|
\leq \left( \|z(t_0)\| + \frac{c_0}{\alpha} \right) e^{c_0/\alpha},
\qquad t \geq t_0.
}
\tag{C5}
\]

---

## 8. 结论

不等式 (C5) 的右端与 \(t\) 无关，因此：

\[
\sup_{t \geq t_0} \|z(t)\| < \infty,
\]

即 \(z(t)\) 在 \([t_0, \infty)\) 上一致有界。进而 \(\bar{x}(t) = z(t) + P^T x^*\) 也有界。

---

## 9. 推导链路总览

```
(C2)  动力学方程
  │
  ▼
变分公式 (1)     解析表达式
  │
  ▼
取范数 + 三角不等式 (2)–(3)
  │
  ▼
(C4)  ‖e^{A₀t}‖ ≤ 1     去掉矩阵指数 (4)–(5)
  │
  ▼
展开 f(s) = Δz + r      拆开被积函数 (6)
  │
  ▼
(C3)  扰动指数衰减       代入衰减率 (7)
  │
  ▼
积分线性拆分             得到 (8)
  │
  ▼
Gronwall 不等式          吸收积分 (9)–(10)
  │
  ▼
(C5)  一致有界估计
```

每一步的放缩依据都在前文标注了对应的条件编号，逻辑链条完整且无循环。