# 未知控制方向多智能体合围


## 1. 问题与记号

$N$ 个智能体 + 1 个目标，$p_i,v_i,p_0,v_0\in\mathbb R^m$。智能体 $i$ 的第 $\ell$ 个分量的**相对动力学**为
$$
\dot p_{i0,\ell}=v_{i0,\ell},\qquad
\dot v_{i0,\ell}=b_{i\ell}(t)\,u_{i\ell}+w_{i\ell}(t),
\tag{1.1}
$$
其中 $p_{i0}=p_i-p_0,\ v_{i0}=v_i-v_0$，$w_{i\ell}=d_{i\ell}-u_{0\ell}$ 且 $|w_{i\ell}|\le\bar w_{i\ell}$。
$B_i(t)=\operatorname{diag}\{b_{i1},\dots,b_{im}\}$ 对角、符号未知、可切换但不穿零。

**可测/本地可算量**：$p_{i0}$（$b_i=1$ 的智能体直接测；其余由静态分布式线性方程精确恢复，见 §2.3）、$p_{ij}=p_i-p_j$、以及由此算出的排斥/吸引项 $\varphi_i$。
**不可测/不可算量**：$v_{i0}$、$\dot\varphi_i$、$b_{i\ell}$（含符号与幅值）、$w_{i\ell}$。

**目标（弱化版）**

| 编号 | 现文（result1/2/3） | 本文（result 4，可接受） |
|---|---|---|
| P1 | $\lim_t\operatorname{dist}(p_0,\operatorname{co}(p))=0$ | 收敛到调参可以使得任意小的范围 |
| P2 | $\|p_{ij}(t)\|>d,\ \forall t$ | 同（不变，不变式） |
| P3 | $\|v_i-v_0\|\to0$ | $\|v_i-v_0\|$ 有界 |

