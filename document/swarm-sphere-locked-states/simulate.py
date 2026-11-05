"""Locked states of the heterogeneous swarm-sphere model -- numerical example.

Reference (OKB doc id 4292):
    D. Kim, "On the Convergence Properties of a Heterogeneous Multiagent System
    on the Unit Sphere", IEEE TAC, 2023, doi:10.1109/TAC.2023.3297510.

System (1) of the paper, with x_i(t) in R^m:

    xdot_i = Omega_i x_i + (kappa/N) * sum_k ( x_k - <x_i, x_k> x_i ),
    x_i(0) in S^{m-1},

where Omega_i is skew-symmetric (natural frequency), kappa > 0 the coupling.

Theorem 1.1 regime used here: m = 2d with d = 1, i.e. m = 2,

    Omega_i = [[0, a_i], [-a_i, 0]]   (a_i real  ->  the "D_i" of the paper),
    kappa > 8 (d+1)^2 / d^2 * D(Omega) = 32 * D(Omega),
    D(X^0)^2 < ( kappa + sqrt(kappa^2 - 2 kappa D(Omega)) ) / kappa.

The theorem only asserts that the limits d_ij = lim_t ||x_i - x_j|| exist
(the paper's Remark 3.3 states the structure of the limit is unknown), so the
values below are measured numerically.  Two independent initial conditions are
integrated: in the strong-coupling regime they must give the same limits.

Run (no project, ephemeral env):
    uv run --no-project --with numpy --with matplotlib python simulate.py
"""

import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless: write PNG instead of opening a window
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------- parameters
N, m = 5, 2                       # m = 2d with d = 1
rng = np.random.default_rng(0)

a = np.array([0.0, 0.4, -0.3, 0.7, -0.6])          # natural frequencies of the paper
Omega = np.zeros((N, m, m))                        # Omega_i = [[0, a_i], [-a_i, 0]]
for i in range(N):
    Omega[i] = np.array([[0.0, a[i]], [-a[i], 0.0]])


def diameter(Omega):
    """D(Omega) = max_{i,j} || Omega_i - Omega_j ||_F  (eq. (4) of the paper)."""
    return max(np.linalg.norm(Omega[i] - Omega[j]) for i in range(N) for j in range(N))


D_OMEGA = diameter(Omega)
kappa = 40.0 * D_OMEGA                             # satisfies kappa > 32 * D(Omega)
T, dt = 40.0, 0.01


# ----------------------------------------------------------------------------- simulator
def rhs(X, Omega, kappa):
    """Vector field of (1); rows of X are the agent positions."""
    S = X.sum(axis=0)                              # sum_k x_k
    G = X @ X.T                                    # <x_i, x_k>
    drive = (S[None, :] - G @ X) / N               # sum_k (x_k - <x_i,x_k> x_i)
    return np.einsum("nij,nj->ni", Omega, X) + kappa * drive


def simulate(X0, Omega=Omega, kappa=kappa, T=T, dt=dt):
    """Classical RK4; the unit sphere is kept invariant by renormalising rows."""
    X = X0 / np.linalg.norm(X0, axis=1, keepdims=True)
    steps = int(T / dt)
    hist = np.empty((steps + 1, X.shape[0], X.shape[1]))
    hist[0] = X
    for n in range(1, steps + 1):
        k1 = rhs(X, Omega, kappa)
        k2 = rhs(X + 0.5 * dt * k1, Omega, kappa)
        k3 = rhs(X + 0.5 * dt * k2, Omega, kappa)
        k4 = rhs(X + dt * k3, Omega, kappa)
        X = X + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        X /= np.linalg.norm(X, axis=1, keepdims=True)
        hist[n] = X
    return hist


def pairwise_distances(hist):
    """||x_i - x_j|| for every (i,j) over time, shape (steps+1, N, N)."""
    return np.linalg.norm(hist[:, :, None, :] - hist[:, None, :, :], axis=-1)


def random_initial(N, m, seed):
    r = np.random.default_rng(seed)
    return r.normal(size=(N, m))


# ----------------------------------------------------------------------------- two runs
X0_a = random_initial(N, m, seed=1)
X0_b = random_initial(N, m, seed=2)

hist_a, hist_b = simulate(X0_a), simulate(X0_b)
dist_a, dist_b = pairwise_distances(hist_a), pairwise_distances(hist_b)
t = np.arange(dist_a.shape[0]) * dt

D0_a = np.linalg.norm(X0_a[:, None, :] - X0_a[None, :, :], axis=-1).max()
D0_a /= np.linalg.norm(X0_a, axis=1).max()  # diameter of the initial data on the sphere
bound = (kappa + np.sqrt(kappa**2 - 2 * kappa * D_OMEGA)) / kappa

print(f"N = {N}, m = {m} (d = 1), a = {a}")
print(f"D(Omega) = {D_OMEGA:.4f},  kappa = {kappa:.2f}  (need kappa > 32 D(Omega) = {32*D_OMEGA:.2f})")
print(f"initial-diameter bound (3)2: D(X0)^2 < {bound:.4f}   [run A: D(X0) = {D0_a:.4f}]")
print()
print("measured limits  d_ij = lim_t ||x_i(t) - x_j(t)||")
print("        run A      run B     |difference|")
d_inf_a = dist_a[-1]
d_inf_b = dist_b[-1]
for i in range(N):
    for j in range(i + 1, N):
        print(f"  ({i},{j})  {d_inf_a[i, j]:8.5f}   {d_inf_b[i, j]:8.5f}   {abs(d_inf_a[i,j]-d_inf_b[i,j]):.2e}")
print()
print(f"max |d_ij(A) - d_ij(B)| = {np.abs(d_inf_a - d_inf_b).max():.2e}   "
      "(limit is independent of the initial data)")


# ----------------------------------------------------------------------------- figure
fig = plt.figure(figsize=(12.0, 5.4), dpi=150)
gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.35], wspace=0.28)

# --- left: final (locked) configuration on S^1 -------------------------------
ax = fig.add_subplot(gs[0, 0])
th = np.linspace(0, 2 * np.pi, 400)
ax.plot(np.cos(th), np.sin(th), color="0.75", lw=1.0)

Xf = hist_a[-1]
Xi = hist_a[0]
ax.scatter(Xi[:, 0], Xi[:, 1], s=45, facecolors="none", edgecolors="0.65",
           linewidths=1.0, zorder=2, label="initial")
ax.scatter(Xf[:, 0], Xf[:, 1], s=90, c=np.arange(N), cmap="viridis",
           edgecolors="k", linewidths=0.6, zorder=3, label="final ($t=40$)")

for i in range(N):
    for j in range(i + 1, N):
        ax.plot([Xf[i, 0], Xf[j, 0]], [Xf[i, 1], Xf[j, 1]],
                color="0.55", lw=1.0, ls="--", zorder=1, alpha=0.8)

centroid = Xf.mean(axis=0)
for i in range(N):
    ax.annotate(f"$x_{{{i+1}}}$", Xf[i] * 1.14, ha="center", va="center", fontsize=10)
    ax.arrow(centroid[0], centroid[1], 0.92 * (Xf[i, 0] - centroid[0]),
             0.92 * (Xf[i, 1] - centroid[1]), head_width=0.02, head_length=0.03,
             color=plt.cm.viridis(i / max(N - 1, 1)), lw=1.2, alpha=0.9, zorder=2,
             length_includes_head=True)

ax.set_aspect("equal")
ax.set_xlim(-1.3, 1.3)
ax.set_ylim(-1.3, 1.3)
ax.set_xlabel("$x^{(1)}$")
ax.set_ylabel("$x^{(2)}$")
ax.set_title("locked configuration on $\\mathbb{S}^{1}$ (final positions)")
ax.legend(loc="lower left", fontsize=8, frameon=True)

# --- right: pairwise distances converging to definite values -----------------
ax = fig.add_subplot(gs[0, 1])
cmap = plt.cm.plasma
k = 0
for i in range(N):
    for j in range(i + 1, N):
        ax.plot(t, dist_a[:, i, j], color=cmap(k / (N * (N - 1) / 2 - 1)), lw=1.4,
                label=f"$d_{{{i+1}{j+1}}}$")
        ax.plot(t, dist_b[:, i, j], color=cmap(k / (N * (N - 1) / 2 - 1)), lw=1.0,
                ls=":", alpha=0.9)
        ax.axhline(d_inf_a[i, j], color=cmap(k / (N * (N - 1) / 2 - 1)),
                   lw=0.7, ls="--", alpha=0.55)
        k += 1

ax.set_xlabel("t")
ax.set_ylabel("$\\|x_i(t) - x_j(t)\\|$")
ax.set_title("relative distances $\\to$ definite values (no consensus)")
ax.set_xlim(0, T)
ax.set_ylim(0, 2.05)
ax.grid(alpha=0.3)
ax.legend(ncol=2, fontsize=8, title="solid: run A, dotted: run B", title_fontsize=7)

fig.suptitle("Heterogeneous swarm-sphere model (Kim 2023, Thm 1.1): "
             f"$m={m}$, $N={N}$, $\\kappa={kappa:.0f}$", fontsize=11)
fig.tight_layout(rect=(0, 0, 1, 0.95))
fig.savefig("locked_states.png", bbox_inches="tight")
print("\nfigure written to locked_states.png")
