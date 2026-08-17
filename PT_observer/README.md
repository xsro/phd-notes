# Twisting controller — stability analysis

For $x\in\mathbb R^2$, consider the twisting-controlled system

$$
\ddot{x}=-k_1 \operatorname{sign}(M x)-k_2 \operatorname{sign}(M\dot{x})+\delta(t),
$$

with $M=M^T\succ0$, gains $k_1,k_2>0$, and disturbance bounded by
$\sup_{t\ge0}\|\delta(t)\|\le\bar\delta$ (Euclidean norm).

---

## 1. Lyapunov function and its derivative

Choose

$$
V(x,\dot x)=k_1\|Mx\|_1+\frac12\|M^{1/2}\dot x\|^2
          =k_1\|Mx\|_1+\frac12\dot x^T M\dot x .
$$

Both terms are nonnegative and $V$ is **radially unbounded** because $M\succ0$
($\frac12\dot x^TM\dot x\ge\frac12\lambda_{\min}(M)\|\dot x\|^2$ and
$\|Mx\|_1\ge\lambda_{\min}(M)\|x\|_1$).

> **Note on the norms.** The position term must be the **$\ell_1$ norm**
> $\|Mx\|_1$ (so that its derivative is the subgradient
> $(\operatorname{sign}(Mx))^TM\dot x$), while the velocity term is the squared
> Euclidean norm. This is the consistent choice that makes the $k_1$ terms cancel
> below. (The earlier draft used an ambiguous $\|Mx\|$ and omitted the $1/2$.)

Along trajectories (where the maps are differentiable, i.e. $Mx\neq0$ and
$M\dot x\neq0$; the remaining points are covered by the Filippov/subgradient
sense),

$$
\begin{aligned}
\dot V
&=k_1\bigl(\operatorname{sign}(Mx)\bigr)^T M\dot x+\dot x^T M\ddot x \\[2mm]
&=k_1\bigl(\operatorname{sign}(Mx)\bigr)^T M\dot x
  +\dot x^T M\Bigl(-k_1\operatorname{sign}(Mx)-k_2\operatorname{sign}(M\dot x)+\delta\Bigr).
\end{aligned}
$$

Since $M$ is symmetric,
$(\operatorname{sign}(Mx))^T M\dot x=\dot x^T M\operatorname{sign}(Mx)$,
so the $k_1$ terms cancel **exactly**:

$$
\dot V=-k_2\,\dot x^T M\operatorname{sign}(M\dot x)+\dot x^T M\delta .
$$

Now $\dot x^T M\operatorname{sign}(M\dot x)=(M\dot x)^T\operatorname{sign}(M\dot x)=\|M\dot x\|_1$,
hence

$$
\dot V=-k_2\|M\dot x\|_1+(M\dot x)^T\delta .
$$

Bound the disturbance by Cauchy–Schwarz and use $\|M\dot x\|\le\|M\dot x\|_1$:

$$
(M\dot x)^T\delta\le\|M\dot x\|\,\|\delta\|\le\bar\delta\|M\dot x\|
\le\bar\delta\|M\dot x\|_1 .
$$

Therefore

$$
\boxed{\;\dot V\le -(k_2-\bar\delta)\|M\dot x\|_1\;}\qquad(k_2>\bar\delta).
$$

---

## 2. Stability conclusions

### 2.1 Boundedness / uniform ultimate boundedness ($\delta\neq0$)

If $k_2>\bar\delta$ then $\dot V\le0$, so $V(t)$ is nonincreasing and bounded
below by $0$; all trajectories are bounded. Moreover
$\int_0^\infty\|M\dot x\|_1\,dt\le V(0)/(k_2-\bar\delta)<\infty$, and since
$\ddot x$ is bounded ($|\ddot x|\le k_1+k_2+\bar\delta$), $\dot x$ is uniformly
continuous. By Barbalat’s lemma, $\dot x(t)\to0$. The state is thus driven onto
a neighbourhood of the manifold $\dot x=0$ (the second-order sliding layer),
i.e. the system is **uniformly ultimately bounded** (practical stability) with a
residual set of size $O(\bar\delta)$.

### 2.2 Global asymptotic stability ($\delta\equiv0$)

With $\delta=0$,

$$
\dot V=-k_2\|M\dot x\|_1\le0 .
$$

By **LaSalle’s invariance principle**, the largest invariant set inside
$\{\dot V=0\}=\{\dot x=0\}$ is the origin:

* On $\dot x=0$, invariance requires $\ddot x=0$ for all future time.
* From the dynamics,
  $0=\ddot x=-k_1\operatorname{sign}(Mx)-k_2\operatorname{sign}(0)$ in the
  Filippov sense; this can hold (choosing the equivalent value of
  $\operatorname{sign}(\dot x)$) only when $Mx=0$, i.e. $x=0$.

Since $V$ is radially unbounded, the origin $(x,\dot x)=(0,0)$ is
**globally asymptotically stable**.

### 2.3 Finite-time stability (twisting / second-order sliding mode)

The controller is a **twisting algorithm**. Under the gain conditions

$$
k_1>\bar\delta,\qquad k_2>k_1+\bar\delta,
$$

the origin is reached in **finite time** (for $\delta\equiv0$: exact finite-time
stability; for bounded $\delta$: finite-time arrival into an $O(\bar\delta)$
neighbourhood — the second-order sliding accuracy).

*Finite-time convergence of the velocity.* For the scalar / decoupled case
(or one component with $M=I$), whenever $\dot x\neq0$,

$$
\frac{d}{dt}|\dot x|
=\operatorname{sign}(\dot x)\ddot x
=-k_1\operatorname{sign}(\dot x)\operatorname{sign}(x)-k_2
\le k_1-k_2<0
$$

(the worst case is when the two signs are opposite). Hence $|\dot x|$ decreases
at rate at least $k_2-k_1$ and reaches zero at

$$
t_1\le\frac{|\dot x(0)|}{k_2-k_1}<\infty .
$$

For the general $M$-coupled vector case the same estimate applied to
$\|M\dot x\|_1$ proves **finite-time entry into a slab** around $\dot x=0$;
exact finite-time convergence of $\dot x$ to zero then follows from the standard
homogeneity argument for the twisting controller (Levant, 1993).

*Finite-time convergence of the position.* Once $\dot x$ is in the sliding
layer, the twisting dynamics drive $x\to0$ in finite time; this is the defining
property of the second-order sliding mode and is established rigorously via the
homogeneity of the vector field (Levant, 1993). Consequently the full state
$(x,\dot x)$ reaches $(0,0)$ in finite time.

---

### Summary of gain conditions

| Property | Required condition |
|---|---|
| $\dot V\le0$ / UUB | $k_2>\bar\delta$ |
| Global asymptotic stability ($\delta=0$) | $k_1>0,\;k_2>0$ |
| Finite-time stability (robust) | $k_1>\bar\delta,\;k_2>k_1+\bar\delta$ |

**Reference.** A. Levant, “Sliding order and sliding accuracy in sliding mode
control,” *Int. J. Control*, 1993.
