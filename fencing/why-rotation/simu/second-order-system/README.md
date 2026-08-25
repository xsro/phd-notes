# Second-Order System Simulation: \(\ddot{x} + k_1\dot{x} + k_2 x = 0\), \(x \in \mathbb{R}^2\)

## Overview

Simulates the 2D second-order linear system

\[
\ddot{x} + k_1 \dot{x} + k_2 x = 0, \qquad x = \begin{pmatrix} x_1 \\ x_2 \end{pmatrix} \in \mathbb{R}^2
\]

with parameters \(k_1 = k_2 = 1\). The system decouples into two independent scalar damped harmonic oscillators.

## Files

- `simulate.m` — MATLAB script using `ode45` to solve the ODE and plot:
  1. \(x_1(t)\) and \(x_2(t)\)
  2. \(\theta(t) = \operatorname{atan2}(x_2(t), x_1(t))\)

## Analytical Derivation of \(\theta(t)\)

### 1. General solution of each component

The characteristic equation for each scalar component is

\[
s^2 + k_1 s + k_2 = 0,
\]

with roots

\[
s = -\frac{k_1}{2} \pm i\,\omega, \qquad \omega = \frac{\sqrt{4k_2 - k_1^2}}{2}.
\]

(We assume the underdamped case \(k_1^2 < 4k_2\) so that \(\omega > 0\).)

Each component has the damped oscillatory solution

\[
x_i(t) = e^{-k_1 t/2}\bigl(A_i \cos(\omega t) + B_i \sin(\omega t)\bigr).
\]

In amplitude–phase form:

\[
x_1(t) = R_1 e^{-k_1 t/2} \cos(\omega t - \phi_1), \qquad
x_2(t) = R_2 e^{-k_1 t/2} \cos(\omega t - \phi_2),
\]

where

\[
R_i = \sqrt{A_i^2 + B_i^2}, \qquad \phi_i = \operatorname{atan2}(B_i, A_i).
\]

### 2. \(A_i, B_i\) from initial conditions

At \(t = 0\):

\[
x_i(0) = A_i.
\]

The derivative is

\[
\dot{x}_i(t) = e^{-k_1 t/2}\Bigl(
  -\tfrac{k_1}{2}\bigl(A_i \cos(\omega t) + B_i \sin(\omega t)\bigr)
  - A_i\,\omega \sin(\omega t)
  + B_i\,\omega \cos(\omega t)
\Bigr).
\]

At \(t = 0\):

\[
\dot{x}_i(0) = -\tfrac{k_1}{2}A_i + B_i\,\omega.
\]

Solving for \(B_i\):

\[
\boxed{A_i = x_i(0)}, \qquad
\boxed{B_i = \frac{\dot{x}_i(0) + \frac{k_1}{2}\,x_i(0)}{\omega}}.
\]

### 3. Derivative of the angle \(\theta = \operatorname{atan2}(x_2, x_1)\)

Using

\[
\dot{\theta} = \frac{x_1 \dot{x}_2 - x_2 \dot{x}_1}{x_1^2 + x_2^2},
\]

substitute the amplitude–phase solutions.  Write

\[
\begin{aligned}
x_i(t)   &= R_i e^{-k_1 t/2} \cos(\omega t - \phi_i), \\
\dot{x}_i(t) &= e^{-k_1 t/2}\Bigl(
  -\tfrac{k_1}{2} R_i \cos(\omega t - \phi_i)
  - R_i\,\omega \sin(\omega t - \phi_i)
\Bigr).
\end{aligned}
\]

The numerator:

\[
\begin{aligned}
x_1 \dot{x}_2 - x_2 \dot{x}_1
&= e^{-k_1 t} R_1 R_2 \Bigl[
  \cos(\omega t - \phi_1)\bigl(-\tfrac{k_1}{2}\cos(\omega t - \phi_2) - \omega\sin(\omega t - \phi_2)\bigr) \\
&\qquad\qquad\; -
  \cos(\omega t - \phi_2)\bigl(-\tfrac{k_1}{2}\cos(\omega t - \phi_1) - \omega\sin(\omega t - \phi_1)\bigr)
\Bigr] \\[4pt]
&= e^{-k_1 t} R_1 R_2 \,\omega \Bigl[
  \cos(\omega t - \phi_2)\sin(\omega t - \phi_1)
  - \cos(\omega t - \phi_1)\sin(\omega t - \phi_2)
\Bigr] \\[4pt]
&= e^{-k_1 t} R_1 R_2 \,\omega \sin\bigl((\omega t - \phi_1) - (\omega t - \phi_2)\bigr) \\[4pt]
&= e^{-k_1 t} R_1 R_2 \,\omega \sin(\phi_2 - \phi_1).
\end{aligned}
\]

The denominator:

\[
x_1^2 + x_2^2 = e^{-k_1 t}\Bigl(
  R_1^2 \cos^2(\omega t - \phi_1) + R_2^2 \cos^2(\omega t - \phi_2)
\Bigr).
\]

The exponential factor \(e^{-k_1 t}\) cancels completely, giving

\[
\boxed{\dot{\theta}(t) = \frac{R_1 R_2 \,\omega \sin(\phi_2 - \phi_1)}{R_1^2 \cos^2(\omega t - \phi_1) + R_2^2 \cos^2(\omega t - \phi_2)}}.
\]

### 4. Key observations

- **Numerator is constant.** Therefore \(\dot{\theta}\) never changes sign (unless zero). The angle \(\theta(t)\) is **strictly monotonic** — it either always increases or always decreases.
- **In-phase / anti-phase case:** If \(\phi_1 = \phi_2\) or \(\phi_1 = \phi_2 + \pi\), then \(\sin(\phi_2 - \phi_1) = 0\) and \(\dot{\theta} \equiv 0\). The angle is constant.
- **General integral form:** Using \(\cos^2\alpha = \frac{1+\cos 2\alpha}{2}\), the denominator becomes \(A + C\cos(2\omega t - \Phi)\), and

\[
\theta(t) = \theta(0) + \int_0^t \frac{R_1 R_2 \,\omega \sin(\phi_2 - \phi_1)}{A + C\cos(2\omega \tau - \Phi)}\,d\tau,
\]

which evaluates to an arctangent expression (elementary but cumbersome).

### 5. Example: \(k_1 = k_2 = 1\), with initial conditions from `simulate.m`

For \(k_1 = k_2 = 1\):

\[
\omega = \frac{\sqrt{4\cdot 1 - 1^2}}{2} = \frac{\sqrt{3}}{2}.
\]

The initial conditions used in `simulate.m` are

\[
x(0) = \begin{pmatrix} 1 \\ 0 \end{pmatrix}, \qquad
\dot{x}(0) = \begin{pmatrix} 0 \\ 1 \end{pmatrix}.
\]

Using the formulas from §2:

\[
\begin{aligned}
A_1 &= x_1(0) = 1, \quad
B_1 = \frac{\dot{x}_1(0) + \frac{1}{2}x_1(0)}{\omega}
     = \frac{0 + \frac{1}{2}}{\sqrt{3}/2} = \frac{1}{\sqrt{3}}, \\[4pt]
A_2 &= x_2(0) = 0, \quad
B_2 = \frac{\dot{x}_2(0) + \frac{1}{2}x_2(0)}{\omega}
     = \frac{1 + 0}{\sqrt{3}/2} = \frac{2}{\sqrt{3}}.
\end{aligned}
\]

Hence

\[
\begin{aligned}
R_1 &= \sqrt{1^2 + \left(\tfrac{1}{\sqrt{3}}\right)^2} = \tfrac{2}{\sqrt{3}}, \quad
\phi_1 = \operatorname{atan2}\!\bigl(\tfrac{1}{\sqrt{3}},\,1\bigr) = \tfrac{\pi}{6}, \\[4pt]
R_2 &= \sqrt{0^2 + \left(\tfrac{2}{\sqrt{3}}\right)^2} = \tfrac{2}{\sqrt{3}}, \quad
\phi_2 = \operatorname{atan2}\!\bigl(\tfrac{2}{\sqrt{3}},\,0\bigr) = \tfrac{\pi}{2}.
\end{aligned}
\]

Since \(\sin(\phi_2 - \phi_1) = \sin(\tfrac{\pi}{3}) \neq 0\), \(\theta(t)\) evolves monotonically, as confirmed by the numerical plot.

---

## Why \(\theta(t)\) Does Not Converge — And Why That's Not a Contradiction

A natural question: since \(x_1(t), x_2(t) \to 0\), the system is clearly convergent. Yet \(\theta(t)\) does not converge. **Does this mean coordinate transformation can change convergence?**

### No — the system is convergent in the original coordinates

Each component is a damped harmonic oscillator:

\[
x_i(t) = e^{-k_1 t/2}\bigl(A_i \cos(\omega t) + B_i \sin(\omega t)\bigr) \to 0 \quad (t \to \infty).
\]

So \((x_1, x_2) \to (0, 0)\) — the origin is globally asymptotically stable. **The convergence is real and unchanged.**

### The polar coordinate transformation is singular at the origin

The mapping

\[
(x_1, x_2) \mapsto (r, \theta) = \bigl(\sqrt{x_1^2 + x_2^2},\; \operatorname{atan2}(x_2, x_1)\bigr)
\]

is smooth for \(r > 0\), but at \(r = 0\):

- \(\theta\) is **undefined** — the angle at the origin is arbitrary
- The Jacobian determinant vanishes (the transformation is not invertible there)

Convergence is preserved under **homeomorphisms** (continuous bijections with continuous inverses). The polar transformation is **not** a homeomorphism at the origin — it has a coordinate singularity there. Therefore, convergence in Cartesian coordinates does **not** imply convergence of \(\theta\).

### Geometric picture: a spiral

The phase portrait is a spiral sink. Trajectories wind inward toward the origin infinitely many times:

- **Radial direction:** \(r \sim e^{-k_1 t/2} \to 0\) — converges
- **Angular direction:** \(\theta\) keeps rotating — does **not** converge

This is perfectly consistent. A spiral can approach the origin while its angle sweeps around forever. The angle is simply not a well-defined continuous function at the limit point.

### Broader context

This phenomenon is generic in nonlinear dynamics:

| Phenomenon | Radius behavior | Angle behavior |
|------------|----------------|----------------|
| Stable focus | \(r \to 0\) | \(\theta\) rotates forever |
| Center | \(r\) constant | \(\theta\) periodic |
| Limit cycle | \(r \to \text{const}\) | \(\theta\) periodic |

In all cases, **non-convergence of the angle does not indicate instability** — it reflects that the angular coordinate is singular at the origin (or undefined on the limit set).

### Bottom line

> Coordinate transformation cannot change the true convergence of the system. The system does converge to the origin. \(\theta(t)\) does not converge because \(\theta\) is **not a well-defined coordinate at \(r = 0\)** — it is a coordinate singularity, not a dynamical property. For stability analysis one should use Cartesian coordinates or Lyapunov functions, not the angular variable.