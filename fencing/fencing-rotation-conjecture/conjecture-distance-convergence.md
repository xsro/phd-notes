# Plausible Long-Time Alternatives in the Fencing Closed Loop

This note states the currently most defensible conjectural picture for the fencing closed loop. The
system has a sign-changing damping mechanism in the shape variables, and the simulations in this
repository include a persistent breathing orbit where distances remain bounded away from collision
but continue to oscillate. The natural global statement is therefore a classification of the possible
long-time alternatives.

1. several structural facts are rigorous;
2. a three-way long-time picture is plausible;
3. the full classification is still a conjecture in the high-dimensional
   system.

---

## 1. Closed-loop system

There are \(N\) vehicles in the plane,

\[
\dot x_i = u_i,\qquad x_i(t)\in \mathbb R^2,\qquad i=1,\dots,N,
\]

and a target with unknown constant velocity,

\[
\dot x_0=v_0,\qquad v_0\in\mathbb R^2.
\]

The controller is

\[
u_i=\phi_i+k_1(x_0-x_i)+v_i,\qquad
\dot v_i=k_2(x_0-x_i),\qquad k_1,k_2>0,
\]

with pairwise central repulsion

\[
\phi_i=\sum_{j\in\mathcal N_i}
\alpha(\|x_{ij}\|)\frac{x_{ij}}{\|x_{ij}\|},\qquad
x_{ij}=x_i-x_j,\qquad
\mathcal N_i=\{j\ne i:\|x_{ij}\|\le \mu\},
\]

where

\[
\alpha(s)=\frac{1}{s-d}-\frac{1}{\mu-d},\qquad s\in(d,\mu],
\]

and \(\alpha(s)=0\) for \(s>\mu\). Here \(d>0\) is the collision distance and \(\mu>d\) is the sensing
radius. The force is continuous at \(s=\mu\), but its derivative is not continuous there.

Use target-relative variables

\[
\tilde x_i=x_i-x_0,\qquad \tilde v_i=v_i-v_0,
\]

and zero-mean shape variables

\[
\widehat x=\frac1N\sum_i\tilde x_i,\qquad
\widehat v=\frac1N\sum_i\tilde v_i,\qquad
\tilde x_i^s=\tilde x_i-\widehat x,\qquad
\tilde v_i^s=\tilde v_i-\widehat v .
\]

Let \(J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}\) and
\(R(\theta)=\cos\theta\,I+\sin\theta\,J\).

---

## 2. What is rigorous

### 2.1 The centroid dynamics are exponentially stable

The pairwise repulsion is central, so \(\sum_i\phi_i=0\). Averaging the target-relative equations gives

\[
\dot{\widehat x}=-k_1\widehat x+\widehat v,\qquad
\dot{\widehat v}=-k_2\widehat x,
\]

or

\[
\ddot{\widehat x}+k_1\dot{\widehat x}+k_2\widehat x=0.
\]

For every \(k_1,k_2>0\), \(\widehat x(t)\to0\), \(\dot{\widehat x}(t)\to0\), and
\(\widehat v(t)\to0\) exponentially. The remaining difficulty is entirely in the
\(2(N-1)\)-dimensional shape subsystem.

### 2.2 Finite-time collision is prevented

If all initial gaps satisfy \(\|x_i(0)-x_j(0)\|>d\), then no pair reaches \(d\) in finite time.
Indeed, when a gap \(s=\|x_i-x_j\|\) approaches \(d\), the mutual term in the radial gap equation has
leading contribution \(2\alpha(s)\sim2/(s-d)\), directed toward increasing \(s\). All other state
components are finite on a finite time interval, so they cannot overcome this singular outward term
at the boundary.

This proves forward invariance of the open collision-free domain. It does **not** prove that the
distances converge, nor does it rule out \(s(t)\to d^+\) as \(t\to\infty\).

### 2.3 The shape subsystem has a sign-indefinite damping matrix

Let \(P(\tilde x^s)=\sum_{\{i,j\}}\Phi(\|x_i-x_j\|)\), with \(\Phi'(s)=\alpha(s)\). Since
\(\phi=\nabla_{\tilde x^s}P\), differentiating the shape equation and eliminating the observer gives

\[
\boxed{\;
\ddot{\tilde x}^s+B(\tilde x^s)\dot{\tilde x}^s+k_2\tilde x^s=0,\qquad
B=k_1I-\nabla^2P(\tilde x^s).
\;}
\]

The natural shape energy

\[
E=\frac12\|\dot{\tilde x}^s\|^2+\frac{k_2}{2}\|\tilde x^s\|^2
\]

satisfies

\[
\dot E=-\dot{\tilde x}^{sT}B(\tilde x^s)\dot{\tilde x}^s .
\]

For a single interacting edge of length \(s\), the Hessian of \(\Phi(s)\) has eigenvalues

\[
\alpha'(s)=-\frac{1}{(s-d)^2}
\quad\text{in the radial direction,}
\]

and

\[
\frac{\alpha(s)}s
=\frac1s\left(\frac{1}{s-d}-\frac{1}{\mu-d}\right)>0
\quad\text{in the tangential direction.}
\]

Thus the radial part contributes positive damping to \(B\), while the tangential part contributes
negative damping. Moreover

\[
\frac{\alpha(s)}s\to+\infty\qquad(s\to d^+),
\]

so there is no finite gain \(k_1\) that makes \(B\) uniformly positive definite on the full
collision-free domain.

On a compact set separated from the boundary, one can bound the tangential anti-damping by a finite
constant depending on the separation margin, the graph, and \(N\). That is only a local or conditional
estimate, not a global convergence theorem.

### 2.4 Rigid rotations have the selected angular speed

Suppose a nondegenerate rigid rotation exists on a time interval:

\[
\tilde x_i(t)=R(\omega t)\tilde x_i^0,\qquad
\sum_i\|\tilde x_i^0\|^2>0.
\]

Then all pairwise distances are constant, and the central-force identity gives zero total internal
torque:

\[
\sum_i \tilde x_i\times\phi_i=0.
\]

Using \(\dot{\tilde x}_i=\omega J\tilde x_i\) and
\(\ddot{\tilde x}_i=-\omega^2\tilde x_i\), differentiating the first-order closed loop and dotting
with \(\tilde x_i\), then summing over \(i\), yields

\[
-k_2\sum_i\|\tilde x_i\|^2
=-\omega^2\sum_i\|\tilde x_i\|^2 .
\]

Nondegeneracy therefore forces

\[
\omega^2=k_2,\qquad |\omega|=\sqrt{k_2}.
\]

The radial balance accompanying such a rotation is

\[
\phi_i=k_1\tilde x_i.
\]

This selects the angular speed, but it does not uniquely select the rotating shape.

### 2.5 Collinear symmetry-restricted cases approach jamming

Certain degenerate subspaces are invariant. For example, if all agents, the target, and the initial
observer errors are collinear, then every force remains collinear and no rotation can be generated.

In the collinear subspace the pairwise Hessian has only radial contributions, hence
\(\nabla^2P\preceq0\) and

\[
B=k_1I-\nabla^2P\succeq k_1I.
\]

Consequently

\[
\dot E\le -k_1\|\dot{\tilde x}^s\|^2 .
\]

This gives boundedness of \(\tilde x^s,\dot{\tilde x}^s\) and
\(\dot{\tilde x}^s\in L^2(0,\infty)\). If the collinear trajectory were eventually separated from
the collision boundary, then \(B\) would also be bounded, Barbalat's lemma would give
\(\dot{\tilde x}^s\to0\), and the equation would force every interior omega-limit point to be
\(\tilde x^s=0\), contradicting the positive collision distance.

Therefore a collinear trajectory cannot remain eventually in a compact subset of the collision-free
domain. Equivalently,

\[
\liminf_{t\to\infty}\min_{i\ne j}\|x_i(t)-x_j(t)\|=d .
\]

The stronger statements often observed numerically,

\[
\tilde x^s(t)\to\xi^*\ne0,\qquad
\tilde v^s(t)=-k_2\xi^*t+o(t),\qquad
s(t)-d=O(1/t),
\]

are consistent with the asymptotic balance
\(\phi-k_1\tilde x^s+\tilde v^s\approx0\), but require an additional boundary convergence argument.
They should be stated as a proved theorem only after that boundary analysis is supplied.

---

## 3. The sign-changing damping mechanism

The main mechanism shaping the long-time dynamics is built into the energy identity:

\[
\dot E=-\dot{\tilde x}^{sT}B\dot{\tilde x}^s
\]

and \(B\) has tangential anti-damping whenever \(\alpha(s)/s>k_1\). Near the collision boundary this
anti-damping is arbitrarily strong:

\[
\frac{\alpha(s)}s\to+\infty\qquad(s\to d^+).
\]

A mechanism of this form naturally supports self-sustained oscillation: energy is injected in one
part of shape space and removed in another.

The saved Case C data in this repository is consistent with this interpretation. For
\(N=5\), \(d=5\), \(\mu=9\), \(k_1=k_2=0.5\), the late-time pairwise distances remain bounded away
from \(d\) but continue to oscillate with nontrivial amplitude. In the stored run
\(t\in[0.5,3000]\), the last 30 percent of samples have minimum distance about \(5.245\), while the
per-pair distance oscillation amplitudes range from about \(0.211\) to \(5.494\). This does not by
itself prove a periodic orbit, but it is direct numerical evidence for the breathing-recurrence
alternative.

---

## 4. Main conjecture

The following is the most plausible global statement suggested by the analysis and simulations.
It should be treated as a conjecture, not as a theorem.

> **Conjecture (bounded long-time alternatives).**
> For the closed loop above, every solution starting in the collision-free domain exists for all
> \(t\ge0\), remains bounded in the shape variables, and has one of the following long-time behaviors:
>
> 1. **Rigid rotation.** The shape converges modulo rotation to a nondegenerate relative equilibrium,
>    with angular speed \(|\omega|=\sqrt{k_2}\), and all pairwise distances converge to constants
>    greater than \(d\).
> 2. **Jamming.** At least one gap tends to \(d\) as \(t\to\infty\). In the typical jammed regime,
>    positions converge, observer errors diverge linearly, and the active gap closes like \(1/t\).
> 3. **Breathing recurrence.** The shape remains in a compact subset of the collision-free domain but
>    does not converge to a relative equilibrium. The observed case is a breathing limit cycle:
>    distances are bounded and oscillatory, while the formation does not settle to a fixed shape.

The term "breathing recurrence" is intentionally broader than "limit cycle". In the full
\(2(N-1)\)-dimensional shape system, a proof that every nonconvergent compact omega-limit set is a
single periodic orbit is currently missing. Calling the global result a strict trichotomy into
"rotation, jamming, or limit cycle" would therefore overstate what is known.

In low-dimensional invariant reductions, a genuine trichotomy may be provable after deriving the
exact reduced equations and establishing a Poincare-Bendixson-type mechanism or a monotone return
map. Without that extra argument, the high-dimensional system could in principle support more
complicated compact recurrent dynamics.

---

## 5. Current status

### Established facts

- The centroid locks onto the target exponentially.
- Finite-time collision is prevented by the singular repulsive term.
- The shape energy is not a global Lyapunov function in two dimensions.
- No finite \(k_1\) makes the damping matrix \(B\) uniformly positive definite on the whole
  collision-free domain.
- Any nondegenerate rigid rotation must have angular speed \(\sqrt{k_2}\).
- Collinear degenerate initial conditions cannot generate a nonzero rotation and are driven toward
  the collision boundary in the sense
  \(\liminf d_{\min}(t)=d\).
- Numerical evidence supports a third long-time behavior: bounded breathing oscillation.

### Open parts

- All bounded recurrent interior dynamics are periodic orbits.
- The Case C numerical orbit is an exact attracting limit cycle rather than a long transient or a
  more complicated recurrent set.
- Every jammed trajectory has a unique shape limit.
- Every jammed active gap has the precise asymptotic \(s(t)-d\sim C/t\).
- A global three-way classification holds for all \(N\), all parameters, and all collision-free
  initial conditions.

---

## 6. Numerical caveat

The barrier makes the ODE stiff. As \(s\to d^+\),

\[
\alpha(s)\sim\frac1{s-d},\qquad
\alpha'(s)=-\frac1{(s-d)^2}.
\]

Fixed-step explicit methods can create misleading blow-up or spurious oscillation near the boundary.
The simulations in this repository should therefore be run with a stiff solver such as BDF and tight
tolerances, as the current scripts do.

When measuring rotation, the per-agent angular velocity

\[
\omega_i=\frac{\tilde x_{i,1}\dot{\tilde x}_{i,2}
-\tilde x_{i,2}\dot{\tilde x}_{i,1}}{\|\tilde x_i\|^2}
\]

can become large simply because an agent is close to the target. A meaningful rigid-rotation
diagnostic should report both the mean angular velocity and the spread
\(\max_i\omega_i-\min_i\omega_i\), together with the pairwise distances.

---

## 7. Summary table

| Claim | Status |
| --- | --- |
| \(\widehat x\to0\), \(\widehat v\to0\) exponentially | Proved |
| No finite-time collision from collision-free initial data | Proved |
| Rigid rotation implies \(|\omega|=\sqrt{k_2}\) | Proved |
| Collinear trajectories approach the collision boundary in liminf | Proved by energy/Barbalat away from boundary |
| Collinear trajectories always have a unique jammed limit and \(1/t\) gap law | Plausible, needs boundary proof |
| Global rotating/jammed/breathing classification | Plausible conjecture, open |
