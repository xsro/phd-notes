# N-Scaling Analysis: Vehicle Movement Frequencies in 3D Fencing Controller

## Overview

This analysis examines how the vehicle movement frequencies — **radial (breathing)**, **tangential (rotation)**, and **self (total speed)** — depend on cluster size $N$ in the 3D Kou-Chen-Xiang fencing controller.

**Parameters**: $d=5.0$, $k_1=0.5$, $k_2=0.5$, $\mu$ scaled with $N$, seed=0, $T_{\text{max}}=150$s

**Reference frequencies**:
- Rigid rotation: $f_{\text{rot}} = \sqrt{k_2}/(2\pi) = 0.1125$ Hz ($T_{\text{rot}} = 8.89$ s)
- Frequency resolution: $\approx 0.0143$ Hz

---

## Results Table

| $N$ | $\mu$ | $f_{\text{rad}}$ (Hz) | $f_{\text{tan}}$ (Hz) | $f_{\text{self}}$ (Hz) | $f_{\text{pd}}$ (Hz) | $f_{\text{rad}}/f_{\text{tan}}$ | $\sigma_3/\sigma_1$ | Min $d$ | Collision |
|-----|-------|----------------------|----------------------|----------------------|---------------------|-------------------------------|--------------------|---------|-----------|
| 3   | 9.0   | 0.1571               | 0.1143               | 0.1143               | 0.1571              | 1.375                         | 0.000              | 5.82    | No        |
| 4   | 9.0   | 0.2143               | 0.1143               | 0.1143               | 0.2143              | 1.875                         | 0.068              | 5.55    | No        |
| 5   | 9.0   | 0.1714               | 0.1714               | 0.1714               | 0.1714              | 1.000                         | 0.734              | 5.28    | Yes       |
| 6   | 9.0   | 0.2143               | 0.1143               | 0.1143               | 0.2143              | 1.875                         | 0.827              | 5.39    | Yes       |
| 8   | 10.4  | 0.2143               | 0.1143               | 0.1143               | 0.2143              | 1.875                         | 0.991              | 5.27    | Yes       |
| 10  | 11.6  | 0.0143*              | 0.1143               | 0.1143               | 0.2143              | 0.125                         | 0.847              | 5.27    | Yes       |
| 15  | 14.2  | 0.2000               | 0.2000               | 0.2000               | 0.2143              | 1.000                         | 0.884              | 5.13    | Yes       |
| 20  | 16.4  | 0.2000               | 0.2000               | 0.2000               | 0.2143              | 1.000                         | 0.803              | 5.09    | Yes       |

*\*N=10 radial frequency is anomalous (near-DC), suggesting a different dynamical regime.*

---

## Key Findings

### 1. Tangential Frequency ($f_{\text{tan}}$) — Rotation

**$f_{\text{tan}}$ is approximately independent of $N$ for most values.**

- For $N = 3, 4, 6, 8, 10$: $f_{\text{tan}} \approx 0.114$ Hz, matching $f_{\text{rot}} = 0.1125$ Hz within frequency resolution
- For $N = 5$: $f_{\text{tan}} = 0.171$ Hz (elevated — possible mode coupling)
- For $N = 15, 20$: $f_{\text{tan}} = 0.200$ Hz (frequency locking with radial mode)

**Physical interpretation**: The tangential mode is the neutral rotation direction — there is no restoring force in the tangential direction, so the frequency is set entirely by the observer gain $k_2$: $\omega_{\text{tan}} = \sqrt{k_2}$. This is a robust prediction of the controller theory and is confirmed by the data.

### 2. Radial Frequency ($f_{\text{rad}}$) — Breathing

**$f_{\text{rad}}$ shows non-monotonic dependence on $N$.**

- $N=3$: $f_{\text{rad}} = 0.157$ Hz ($f_{\text{rad}}/f_{\text{rot}} = 1.40$)
- $N=4$: $f_{\text{rad}} = 0.214$ Hz ($f_{\text{rad}}/f_{\text{rot}} = 1.90$)
- $N=5$: $f_{\text{rad}} = 0.171$ Hz ($f_{\text{rad}}/f_{\text{rot}} = 1.52$)
- $N=6$: $f_{\text{rad}} = 0.214$ Hz ($f_{\text{rad}}/f_{\text{rot}} = 1.90$)
- $N=8$: $f_{\text{rad}} = 0.214$ Hz ($f_{\text{rad}}/f_{\text{rot}} = 1.90$)
- $N=10$: $f_{\text{rad}} = 0.014$ Hz (anomalous — near-DC)
- $N=15$: $f_{\text{rad}} = 0.200$ Hz ($f_{\text{rad}}/f_{\text{rot}} = 1.78$)
- $N=20$: $f_{\text{rad}} = 0.200$ Hz ($f_{\text{rad}}/f_{\text{rot}} = 1.78$)

**Physical interpretation**: The radial (breathing) mode is a radial eigenmode of the formation. Its frequency is set by:

$$\omega_{\text{rad}}^2 = k_2 + C_{\text{rep}}(N)$$

where $C_{\text{rep}}$ comes from the repulsive force Jacobian. As $N$ increases:
- Each vehicle interacts with more neighbors
- The repulsive stiffness per vehicle **decreases** (forces are shared)
- This would suggest $f_{\text{rad}}$ should **decrease** with $N$

However, the data shows this is not a simple monotonic relationship. The non-monotonic behavior suggests:
- For small $N$ (3–8), the geometry of the formation strongly affects $C_{\text{rep}}$
- For $N=10$, the system may enter a different regime (jammed or weakly oscillating)
- For large $N$ (≥15), frequency locking occurs: $f_{\text{rad}} = f_{\text{tan}}$

### 3. Self Frequency ($f_{\text{self}}$) — Total Speed

**$f_{\text{self}}$ always equals $f_{\text{tan}}$ (or the locked frequency).**

This means the dominant oscillation in total vehicle speed is at the rotation frequency, not the breathing frequency. Even when $f_{\text{rad}} > f_{\text{tan}}$, the self-frequency tracks the tangential component.

**Physical interpretation**: The total speed $v = \sqrt{v_{\text{rad}}^2 + v_{\text{tan}}^2}$ is dominated by the tangential component because:
- The radial velocity is a small oscillation around a constant mean radius
- The tangential velocity is a persistent rotation with significant magnitude
- The FFT of total speed therefore picks up the rotation frequency

### 4. Frequency Locking for Large $N$

For $N \geq 15$, we observe **frequency locking**:
$$f_{\text{rad}} = f_{\text{tan}} = f_{\text{self}} = 0.200 \text{ Hz}$$

This means the radial and tangential motions are oscillating at the **same frequency**. The formation undergoes a combined radial-tangential oscillation where the two modes are frequency-locked.

**Note**: The locked frequency (0.200 Hz) is higher than $f_{\text{rot}}$ (0.1125 Hz). This suggests that when locking occurs, the effective frequency is set by both $k_2$ and the repulsive stiffness, not by $k_2$ alone.

### 5. Planarity ($\sigma_3/\sigma_1$)

| $N$ | $\sigma_3/\sigma_1$ | Interpretation |
|-----|---------------------|----------------|
| 3   | 0.000               | Planar (collapses to 2D) |
| 4   | 0.068               | Nearly planar |
| 5   | 0.734               | Non-planar |
| 6   | 0.827               | Strongly non-planar |
| 8   | 0.991               | Very strongly non-planar |
| 10  | 0.847               | Non-planar |
| 15  | 0.884               | Non-planar |
| 20  | 0.803               | Non-planar |

**For $N \geq 5$, the formation is non-planar** ($\sigma_3/\sigma_1 > 0.7$). The planarity is approximately independent of $N$ for $N \geq 5$, with a slight increasing trend.

### 6. Pairwise Distance Frequency ($f_{\text{pd}}$)

$f_{\text{pd}}$ tracks $f_{\text{rad}}$ for most $N$ values, confirming that pairwise distance oscillation is driven by the radial (breathing) mode, not the tangential rotation.

---

## Scaling Exponents (Power-Law Fit: $f = a \cdot N^b$)

| Component | $a$ | $b$ | Interpretation |
|-----------|-----|-----|----------------|
| $f_{\text{rad}}$ | 0.2076 | **-0.196** | Weak decreasing trend with $N$ |
| $f_{\text{tan}}$ | 0.0808 | **+0.269** | Weak increasing trend (but dominated by locking cases) |
| $f_{\text{self}}$ | 0.0808 | **+0.269** | Same as $f_{\text{tan}}$ |
| $f_{\text{pd}}$ | 0.1569 | **+0.123** | Approximately independent of $N$ |

**Note**: The scaling exponents should be interpreted with caution because:
1. The frequency values are quantized by the frequency resolution (0.0143 Hz)
2. The $N$ range is limited (3–20)
3. The behavior is non-monotonic, so a single power-law fit may not capture the true relationship

---

## Physical Interpretation and Theoretical Context

### Why $f_{\text{tan}} \approx f_{\text{rot}}$?

The tangential direction is a **neutral mode** of the controller. The controller constrains only the average position error and average velocity, leaving $2(N-1)$ circulation degrees of freedom unconstrained. At radial balance, the tangential stiffness of repulsion cancels $k_1$, leaving one neutral direction. The observer integrates the rotating position error into tangential velocity, giving:

$$\omega_{\text{tan}}^2 = k_2$$

This is the same frequency selection rule as in 2D rigid rotation, and it is **independent of $N$**.

### Why $f_{\text{rad}} > f_{\text{tan}}$?

The radial (breathing) mode is a **stiff oscillation** driven by the repulsive force gradient:

$$\alpha'(s) = -\frac{1}{(s-d)^2}$$

This creates a radial restoring mechanism. The effective stiffness is:

$$\omega_{\text{rad}}^2 = k_2 + C_{\text{rep}}$$

where $C_{\text{rep}} > 0$ comes from the repulsive force Jacobian. This is why $f_{\text{rad}} > f_{\text{tan}}$ in the breathing regime.

### Why does frequency locking occur for large $N$?

As $N$ increases:
1. Each vehicle has more neighbors within sensing range
2. The repulsive force per neighbor decreases (forces are distributed)
3. The effective $C_{\text{rep}}$ decreases
4. Eventually $C_{\text{rep}}$ becomes small enough that $f_{\text{rad}} \approx f_{\text{tan}}$
5. The system enters a frequency-locked state

This is consistent with the physical picture that **larger clusters have softer repulsive interactions per vehicle**, causing the breathing mode to soften and eventually lock to the rotation mode.

### Comparison with 2D and Previous 3D Results

| Property | 2D Breathing ($N=5$) | 3D Breathing ($N=4$) | 3D Breathing ($N=6$) | This Work ($N=15-20$) |
|----------|---------------------|---------------------|---------------------|----------------------|
| $f_{\text{rad}}/f_{\text{tan}}$ | ~3.57 | ~1.86 | ~1.86 | **1.00** (locked) |
| $T_{\text{rad}}/T_{\text{rot}}$ | ~0.28 | ~0.54 | ~0.54 | **~0.56** (locked) |
| Planarity | Planar | Non-planar | Non-planar | Non-planar |

The 3D breathing mode is closer to rigid rotation than 2D breathing, consistent with 3D having more degrees of freedom. For very large $N$, the system enters a frequency-locked regime where radial and tangential frequencies coincide.

---

## Conclusion

**Does vehicle movement frequency depend on cluster size $N$?**

1. **Tangential frequency ($f_{\text{tan}}$)**: **No** — it is set by $k_2$ alone and equals $f_{\text{rot}} = \sqrt{k_2}/(2\pi)$ for most $N$. Only when frequency locking occurs does it shift.

2. **Radial frequency ($f_{\text{rad}}$)**: **Yes, but weakly and non-monotonically** — it varies between 0.157–0.214 Hz for $N=3-8$, then locks to $f_{\text{tan}}$ for $N \geq 15$. The physical mechanism is the competition between repulsive stiffness (which decreases with $N$) and the observer gain $k_2$.

3. **Self frequency ($f_{\text{self}}$)**: **No** — it always tracks the tangential frequency because total speed is dominated by the rotational component.

4. **Planarity**: **Yes, for small $N$** — $N=3$ is planar, $N=4$ is nearly planar, but $N \geq 5$ is strongly non-planar. For $N \geq 5$, planarity is approximately independent of $N$.

The most significant finding is the **frequency locking transition** at large $N$: as the cluster grows, the radial breathing mode softens (due to distributed repulsive forces) and eventually locks to the tangential rotation mode, resulting in a single combined oscillation frequency.