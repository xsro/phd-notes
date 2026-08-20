# Cai et al. (2017) — Lemma 3 中 `x̄` 有界性推导笔记

> 来源：He Cai, Frank L. Lewis, Guoqiang Hu, Jie Huang,
> *"The adaptive distributed observer approach to the cooperative output regulation of linear multi-agent systems"*, Automatica 75 (2017) 299–305.
> 对应原文 Lemma 3 的证明，特别是方程 (16) → (17) → (19)。
>
> 核心问题：`PᵀAᵀAP` 是不是 Hurwitz？如何得到 `x̄` 有界、`d(t)` 有界？

---

## 0. 结论速览

1. **`PᵀAᵀAP` 不是 Hurwitz**，它是 `diag(ĀᵀĀ, 0_{n−k})`，含 `n−k` 个零特征值（半负定、奇异）。
   ⇒ 论文中 "Then, `x̄` is bounded by Remark 2" 这句是**写法上的偷步**，不能直接把 Lemma 1（要求 `F` Hurwitz）套到整个 `x̄` 上。
2. 真正的 `x̄` 有界性来自**分块分解**：`x̄₁` 维 Hurwitz 稳定；`x̄₂` 维是纯积分器，靠 `d₂(t)→0` 收敛到常数。
3. `d(t)` 有界且 `d(t)→0`（指数速率 α）的依据是 `𝒜(t)→A` 指数快，使 `AᵀA−𝒜ᵀ𝒜→0`。
4. 论文把"先 `x̄` 有界 ⇒ `d→0`"与"用 `d→0` ⇒ 推 `x̄₂` 有界"揉成一句，逻辑略循环；严谨做法需 **bootstrap 论证**或 **`x̃=x−x*` 投影分解**。

---

## 1. 背景：Lemma 3 的设定

已知 `A ∈ R^{m×n}`, `b ∈ R^{m}`, `rank(A)=rank(A,b)=k`。
`𝒜(t) ∈ R^{m×n}` 有界、分段连续，且 `Ã(t) ≜ 𝒜(t)−A → 0` 指数快（速率 α）。

考虑系统

```math
\dot{x} = -\varepsilon 𝒜(t)^T (𝒜(t)x - b) \tag{12}
```

目标：证 (12) 有唯一有界解，且存在 `x*` 满足 `Ax*=b` 使 `x(t)→x*` 指数快。

---

## 2. SVD 分解与 `PᵀAᵀAP` 的结构

由奇异值分解，存在正交阵 `P ∈ R^{n×n}` 使

```math
AP = [\bar{A} \quad 0_{m\times(n-k)}],\qquad \bar{A}\in R^{m\times k}
```

因 `rank(A)=k` 且前 `k` 列捕获满秩，故 `Ā` 列满秩 ⇒ `ĀᵀĀ` 正定（`k×k`）。

由此

```math
A = [\bar{A}\quad 0]\,P^T
```

```math
A^TA = P \begin{bmatrix} \bar{A}^T\bar{A} & 0 \\ 0 & 0 \end{bmatrix} P^T
\quad\Longrightarrow\quad
P^TA^TAP = \begin{bmatrix} \bar{A}^T\bar{A} & 0 \\ 0 & 0_{n-k} \end{bmatrix} \tag{13}
```

**结论**：`PᵀAᵀAP` 仅半负定，后 `n−k` 个特征值为 0。**不是 Hurwitz。**

> 注：`PᵀAᵀb = [Āᵀb; 0]`，第二块为 0（因 `A P₂ = 0`，`P₂` 为 `P` 的后 `n−k` 列）。

---

## 3. 方程 (16) 的来历

令 `x̄ = Pᵀx`。对 (12) 左乘 `Pᵀ` 并"加一项减一项" `AᵀA`：

```math
\begin{aligned}
\dot{\bar{x}}
&= P^T\bigl(-\varepsilon 𝒜^T𝒜 x + \varepsilon 𝒜^T b\bigr) \\
&= -\varepsilon P^TA^TAP\,\bar{x}
   + \varepsilon P^TA^Tb
   + \underbrace{\varepsilon P^T(A^TA-𝒜^T𝒜)P\,\bar{x}
                + \varepsilon P^T\tilde{A}^T b}_{d(t)} \tag{16}
\end{aligned}
```

即

```math
\dot{\bar{x}} = -\varepsilon P^TA^TAP\,\bar{x} + \varepsilon P^TA^Tb + d(t)
```

---

## 4. `d(t)` 为什么有界、且 `d(t)→0`

```math
d(t) = \varepsilon P^T(A^TA-𝒜^T𝒜)P\,\bar{x} + \varepsilon P^T\tilde{A}^T b
```

展开矩阵差：

```math
A^TA - 𝒜^T𝒜 = A^TA - (A+\tilde{A})^T(A+\tilde{A})
            = -A^T\tilde{A} - \tilde{A}^TA - \tilde{A}^T\tilde{A}
```

- `Ã(t)→0` 指数快（速率 α）且有界；`𝒜, A` 有界 ⇒ `AᵀA−𝒜ᵀ𝒜` **有界且指数衰减到 0**。
- `Ãᵀb → 0` 指数快。

于是：

- 一旦 `x̄` 有界，第一项 = （有界量）×（指数衰减矩阵）⇒ 有界且 `→0`（速率 α）；
- 第二项 `εPᵀÃᵀb` ⇒ 指数衰减到 0。

**结论**：`d(t)` 有界，且 `d(t)→0` 指数快（速率 α）——**前提是 `x̄` 有界**。
（也正是论文 (16) 下方的 "Then `x̄` is bounded ... and thus `lim d(t)=0` exponentially"。）

---

## 5. `x̄` 有界的严格推导（分块）

将 `x̄ = col(x̄₁, x̄₂)`、`d = col(d₁, d₂)`，其中 `x̄₁, d₁ ∈ R^k`，`x̄₂, d₂ ∈ R^{n−k}`。
代入 (16) 并用 (13) 与 `PᵀAᵀb=[Āᵀb;0]`：

```math
\dot{\bar{x}}_1 = -\varepsilon \bar{A}^T\bar{A}\,\bar{x}_1 + \varepsilon \bar{A}^T b + d_1(t) \tag{17a}
```

```math
\dot{\bar{x}}_2 = d_2(t) \tag{17b}
```

- **`x̄₁` 维**：`−εĀᵀĀ` 是 Hurwitz（因 `ĀᵀĀ` 正定）。只要 `d₁` 有界，该维指数稳定 ⇒ `x̄₁` 有界（且收敛到 `x̄₁*=(ĀᵀĀ)⁻¹Āᵀb`）。
- **`x̄₂` 维**：`x̄₂(t)=x̄₂(0)+∫₀ᵗ d₂(τ)dτ`。由第 4 节，`d₂(t)→0` 指数快 ⇒ 积分收敛到有限常数 ⇒ **`x̄₂` 有界**。

两维合起来 ⇒ **`x̄` 有界**。

> 论文此处把"先 `x̄` 有界 ⇒ `d→0`"与"用 `d→0` ⇒ 推 `x̄₂` 有界"合并成一句，存在轻微自指。
> 严格化做法见第 6 节（bootstrap 或 `x̃=x−x*` 投影）。

---

## 6. 严谨、非循环的替代证明（推荐理解方式）

取特解 `x* = A⁺b`（最小范数解，满足 `Ax*=b`，对应 `x̄*=[(ĀᵀĀ)⁻¹Āᵀb; 0]`），令 `x̃ = x − x*`。
由 (12) 且 `𝒜x* = Ax* + Ãx* = b + Ãx*`：

```math
\begin{aligned}
\dot{\tilde{x}}
&= -\varepsilon 𝒜^T𝒜(x^*+\tilde{x}) + \varepsilon 𝒜^T b \\
&= -\varepsilon 𝒜^T𝒜\,\tilde{x} - \varepsilon 𝒜^T(𝒜x^*-b) \\
&= -\varepsilon 𝒜^T𝒜\,\tilde{x} - \varepsilon 𝒜^T\tilde{A}x^* \tag{i}
\end{aligned}
```

令 `P₀` 为 `P` 的后 `n−k` 列（`A` 的零空间基），`ξ = P₀^T x̃`（零空间分量），`x̃₁` 为值域分量。

- **值域分量**：`x̃₁̇ = −ε(ĀᵀĀ + 小量)x̃₁ + 衰减项`。因 `ĀᵀĀ` 正定，该维指数稳定 ⇒ `x̃₁` 有界且 `→0`。
- **零空间分量**：`ξ̇ = −ε Γ(t) ξ + h(t)`，其中
  `Γ(t) = P₀^T Ã^T 𝒜 P₀ → 0` 指数快，`h(t) = −ε P₀^T Ã^T(𝒜 x̃₁ + Ã x*) → 0` 指数快。
  长时间后自项 `Γ(t)ξ` 可忽略，`ξ̇ ≈ h(t)→0` 且可积 ⇒ `ξ` 收敛到常数 ⇒ 有界。

⇒ `x̃ = x − x*` 有界且收敛 ⇒ `x` 有界，且 `x(t)→x*` 指数快（速率至少 α）。
此即论文 (18)→(19) 要证的结论，且无需经过 `d(t)` 的自指。

---

## 7. 与后续 Lemma 4 的衔接

Lemma 4 把上述结果用于调节器方程 (3)。令

```math
Q_i(t) = S_i(t)^T\otimes\begin{bmatrix}I&0\\0&0\end{bmatrix}
        - I_q\otimes\begin{bmatrix}A_i&B_i\\C_i&D_i\end{bmatrix},\qquad
b_i = \mathrm{vec}\!\begin{bmatrix}E_i\\F_i\end{bmatrix}
```

因 Lemma 2 给出 `S̃_i→0` 指数快，故 `Q_i(t)→Q_i` 指数快（对应 Lemma 3 中 `𝒜(t)→A`）。
直接套用 Lemma 3（取 `𝒜(t)=Q_i(t)`, `A=Q_i`）即得 (22) 的解 `ζ_i(t)` 有界且
`ζ_i(t)→χ_i*`（满足 `Q_iχ_i*=b_i`）指数快，即 (23)。

---

## 8. 关键提醒清单

- [ ] `PᵀAᵀAP` **不是** Hurwitz，是 `diag(ĀᵀĀ, 0)`。
- [ ] 不能直接用 Lemma 1 / Remark 2 套整个 `x̄`（那要求 `F` Hurwitz）。
- [ ] `x̄` 有界 = `x̄₁`（Hurwitz 维）+ `x̄₂`（积分器，`d₂→0` 保证不漂移）。
- [ ] `d(t)→0` 的根源：`𝒜(t)→A` 指数快 ⇒ `AᵀA−𝒜ᵀ𝒜→0`。
- [ ] 严谨证明用 `x̃=x−x*` 投影分解，避免论文中的轻微自指。
