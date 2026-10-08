好的，我已经从原文中找到了关键段落。让我总结每种文献：

---

## 各参考文献中关于 Forward Completeness 的原文及如何引用

### 1. Angeli & Sontag (1999) — 光滑系统的奠基论文

**原文定义**（SURVEY.md 已有完整引用，这里补充原文表述）：

> *"A finite-dimensional continuous-time system is forward complete if solutions exist globally, for positive time."*
>
> 假设 $f$ 和 $h$ 为 **locally Lipschitz continuous**。

**引用方式**：
```
@article{angeli1999forward,
  title={Forward completeness, unboundedness observability, 
         and their Lyapunov characterizations},
  author={Angeli, David and Sontag, Eduardo D.},
  journal={Systems \& Control Letters},
  volume={38},
  number={4},
  pages={209--217},
  year={1999},
  publisher={Elsevier}
}
```

**在非光滑 FC 介绍中引用**：作为背景引用，指出该文的 Lipschitz 假设限制了其适用范围，然后引出非光滑情形的推广。

---

### 2. Marley, Skjetne, Teel (2023) — 微分包含的 Forward Completeness ⭐ **最直接相关**

**原文定义**（Section II.C: Constrained Differential Inclusions）：

> *"A solution $x$ to (6) [the differential inclusion] is defined on a time domain denoted $\operatorname{dom} x$. Solutions that cannot be extended are said to be maximal while solutions that exist on an unbounded time domain, $\operatorname{dom} x = [0, \infty)$ are said to be **complete**."*

关于解的三种可能：
> *"By [29, Th. 6.30], a system that satisfies Assumption 1 is well posed ... Additionally, maximal solutions are either **complete**, **escape to infinity in finite time**, or **exist on a compact time domain** $\operatorname{dom} \boldsymbol{x} = [t_0, t_1]$, with $t_0 \leq t_1$ and $x(t_1) \in \partial X$."*

该文的 Assumption 1（微分包含的标准假设）：
> *"$X$ is a closed set. $F : \mathbb{R}^n \rightrightarrows \mathbb{R}^n$ is **outer semicontinuous** and **locally bounded** on $X$. For each $x \in X$, $F(x)$ is a **nonempty convex set**."*

Remark 3 明确区分了光滑和非光滑 FC 的区别：
> *"Theorem 1 differs from the results in [9] in two main aspects: 1) Theorem 1 states UGpAS while [9, Proposition 4] states asymptotic stability under an implicit assumption of forward completeness; 2) Theorem 1 is **stated for differential inclusions** and does **not include a Lipschitz assumption** on the control input."*

**引用方式**：
```bibtex
@article{marley2023sufficient,
  title={Sufficient conditions for uniform asymptotic stability and 
         input-to-state stability using high-order control barrier functions},
  author={Marley, Mathias and Skjetne, Roger and Teel, Andrew R.},
  journal={IEEE Transactions on Automatic Control},
  volume={69},
  number={5},
  pages={2803--2818},
  year={2023},
  publisher={IEEE}
}
```

**在非光滑 FC 介绍中引用**：作为 "微分包含框架下 forward completeness 的定义" 的直接参考文献。该文直接引用 Goebel-Sanfelice-Teel 的混杂系统理论，且 Assumption 1 给出了微分包含解的良定性条件。

---

### 3. Shevitz & Paden (1994) — 非光滑 Lyapunov 稳定性理论的基础

**原文不直接定义 FC**，而是建立 Filippov 解 + Clarke 梯度的非光滑 Lyapunov 稳定性框架：

> *"This paper develops nonsmooth Lyapunov stability theory and LaSalle's invariance principle for a class of nonsmooth Lipschitz continuous Lyapunov functions and absolutely continuous state trajectories. Computable tests based on **Filippov's differential inclusion** and **Clarke's generalized gradient** are derived."*

其链式法则提供了非光滑 Lyapunov 函数沿 Filippov 解的导数计算工具：

> *"Using this framework, theorems were proved for existence, uniqueness, and continuous dependence on initial conditions."*

**引用方式**：
```bibtex
@article{shevitz1994lyapunov,
  title={Lyapunov stability theory of nonsmooth systems},
  author={Shevitz, Daniel and Paden, Brad},
  journal={IEEE Transactions on Automatic Control},
  volume={39},
  number={9},
  pages={1910--1914},
  year={1994},
  publisher={IEEE}
}
```

**在非光滑 FC 介绍中引用**：作为非光滑 Lyapunov 函数的理论基础，说明如何用 Clarke 梯度替代标准梯度来构造 FC 的充分条件。

---

### 4. Bacciotti & Ceragioli (1999) — 不连续系统 + 微分包含的稳定性定理

**原文**：

> *"We study stability and stabilizability properties of systems with discontinuous right-hand side (with solutions intended in Filippov's sense) by means of **locally Lipschitz continuous and regular Lyapunov functions**. The stability result is obtained in the more general context of **differential inclusions**."*

该文给出了在不连续向量场下，使用非光滑 Lyapunov 函数证明稳定性的充分条件，为 FC 的 Lyapunov 刻画提供了工具。

**引用方式**：
```bibtex
@article{bacciotti1999stability,
  title={Stability and stabilization of discontinuous systems and 
         nonsmooth Lyapunov functions},
  author={Bacciotti, Andrea and Ceragioli, Francesca},
  journal={ESAIM: Control, Optimisation and Calculus of Variations},
  volume={4},
  pages={361--376},
  year={1999}
}
```

**在非光滑 FC 介绍中引用**：作为微分包含稳定性充分性条件的参考文献。

---

### 5. Cortés (2008) — 非光滑系统综合教程

**原文**（tutorial 风格，覆盖 Filippov 解的存在性等）：

> *"Discontinuous dynamical systems arise in a large number of applications... What is needed is a set of tools which allow the analysis of differential equations with discontinuous right-hand sides. The seminal contribution in this area was made by **Filippov** who developed a solution concept for differential equations whose right-hand sides were only required to be Lebesgue measurable."*

该文详细讨论了 Carathéodory 解、Filippov 解、解的存在性和唯一性条件，以及非光滑 Lyapunov 稳定性理论。

**引用方式**：
```bibtex
@article{cortes2008discontinuous,
  title={Discontinuous dynamical systems},
  author={Cort{\'e}s, Jorge},
  journal={IEEE Control Systems Magazine},
  volume={28},
  number={3},
  pages={36--73},
  year={2008},
  publisher={IEEE}
}
```

**在非光滑 FC 介绍中引用**：作为非光滑系统基础概念的综合性参考文献，涵盖解的存在性、唯一性、Filippov 框架等。

---

### 6. Haddad (2014) — 不连续右端函数的非光滑稳定性理论

**原文**：

> *"We consider dynamical systems with Lebesgue measurable and locally essentially bounded vector fields characterized by differential inclusions involving **Filippov set-valued maps** specifying a set of directions for the system velocity and admitting **Filippov solutions** with absolutely continuous curves."*

建立了不连续系统的耗散性、稳定性、最优控制理论，推广了小增益定理和正性定理到 Filippov 系统。

**引用方式**：
```bibtex
@article{haddad2014nonlinear,
  title={Nonlinear differential equations with discontinuous right-hand sides: 
         Filippov solutions, nonsmooth stability and dissipativity theory, 
         and optimal discontinuous feedback control},
  author={Haddad, Wassim M.},
  journal={Communications in Applied Analysis},
  volume={18},
  pages={243--282},
  year={2014}
}
```

---

### 7. Goebel, Sanfelice, Teel (2012) — 混杂系统（被 Marley 等引用的 [29]）

**虽不在 OKB-Assist 知识库中，但极其重要**。Marley 等人反复引用该书（编号 [29]），包括解的定义、良定性、well-posedness 等。混杂/微分包含的 forward completeness 定义源于此。

```bibtex
@book{goebel2012hybrid,
  title={Hybrid Dynamical Systems: Modeling, Stability, and Robustness},
  author={Goebel, Rafal and Sanfelice, Ricardo G. and Teel, Andrew R.},
  year={2012},
  publisher={Princeton University Press}
}
```

---

## 在论文中如何组织引用（建议）

在介绍非光滑系统的 forward completeness 时，建议按以下递进结构引用：

> **(1) 定义转型** — 非光滑系统的 FC 定义从经典 ODE 框架迁移到 **Filippov 微分包含**框架。解用绝对连续的 Filippov 解代替经典 ODE 解（Shevitz & Paden, 1994; Cortés, 2008），forward completeness 等价于每个最大解的时间域 $\operatorname{dom} x = [0, \infty)$（Goebel, Sanfelice & Teel, 2012; Marley, Skjetne & Teel, 2023）。
>
> **(2) 存在性条件** — 在 $F$ 上半连续、非空紧凸值、局部有界的标准假设下，Filippov 解局部存在（Cortés, 2008; Haddad, 2014），从而可以在微分包含框架内讨论 FC。
>
> **(3) Lyapunov 刻画（充分性）** — 若存在 proper、局部 Lipschitz、正则的函数 $V$ 使得沿 Clarke 梯度方向 $\max_{\xi\in\partial V(x)}\max_{v\in F(x)} \xi^\top v \leq V(x)$，则微分包含是 forward complete 的（Bacciotti & Ceragioli, 1999; Shevitz & Paden, 1994）。
>
> **(4) Lyapunov 刻画（必要性）** — 与光滑系统（Angeli & Sontag, 1999）的完整 converse Lyapunov 定理不同，非光滑系统 FC 的必要性方向**尚未完全建立**，是一个开放研究方向。