"""Shared machinery for the fencing-controller simulations.

The closed loop is integrated in the *target frame*, so the target sits at the
origin for all times:

    xi_i = x_i - x_0,   vi_i = v_i - v_0

    d xi_i/dt = phi_i - k1 xi_i + vi_i
    d vi_i/dt = -k2 xi_i

with the pairwise central repulsion

    phi_i = sum_{j in N_i} alpha(|x_ij|) x_ij/|x_ij|,
    alpha(s) = 1/(s-d) - 1/(mu-d)  on (d, mu], 0 for s > mu.
"""

import numpy as np
from scipy.integrate import solve_ivp


# --------------------------------------------------------------------------- #
# repulsion
# --------------------------------------------------------------------------- #
def alpha_scalar(s, d, mu):
    if s > mu:
        return 0.0
    if s <= d:
        return np.inf
    return 1.0 / (s - d) - 1.0 / (mu - d)


def repulsion(xi, d, mu):
    """phi_i for all i.  xi: (N, dim) -> (N, dim)."""
    n = len(xi)
    diff = xi[:, None, :] - xi[None, :, :]              # (n, n, dim)
    dist = np.linalg.norm(diff, axis=2)                # (n, n)
    np.fill_diagonal(dist, np.inf)
    act = (dist > d) & (dist <= mu)
    with np.errstate(divide="ignore", invalid="ignore"):
        a = np.where(act, 1.0 / (dist - d) - 1.0 / (mu - d), 0.0)
        coef = np.where(act, a / np.where(dist > 0.0, dist, 1.0), 0.0)
    return np.einsum("ij,ijk->ik", coef, diff)


def distances(xi):
    """Upper-triangular pairwise distances -> (n*(n-1)/2,)."""
    n = len(xi)
    return np.array([np.linalg.norm(xi[i] - xi[j])
                     for i in range(n) for j in range(i + 1, n)])


# --------------------------------------------------------------------------- #
# single integrator (first controller)
# --------------------------------------------------------------------------- #
def rhs_single(N, dim, d, mu, k1, k2):
    """Dimension-agnostic right-hand side (dim = 2 or 3)."""
    m = N * dim

    def rhs(t, z):
        xi = z[:m].reshape(N, dim)
        vt = z[m:].reshape(N, dim)
        phi = repulsion(xi, d, mu)
        return np.concatenate([(phi - k1 * xi + vt).ravel(),
                               (-k2 * xi).ravel()])
    return rhs


def integrate_single(xi0, vt0, d, mu, k1, k2, tf, n_eval=400,
                     method="BDF", rtol=1e-9, atol=1e-11):
    """Integrate the closed loop; return dict with sampled trajectories."""
    xi0 = np.asarray(xi0, float)
    vt0 = np.asarray(vt0, float)
    N, dim = xi0.shape
    m = N * dim
    z0 = np.concatenate([xi0.ravel(), vt0.ravel()])
    t_eval = np.linspace(0.0, tf, n_eval)
    sol = solve_ivp(rhs_single(N, dim, d, mu, k1, k2), (0.0, tf), z0,
                    method=method, t_eval=t_eval, rtol=rtol, atol=atol)
    if not sol.success:
        raise RuntimeError(sol.message)
    z = sol.y.T
    return {"t": sol.t,
            "xi": z[:, :m].reshape(-1, N, dim),
            "vt": z[:, m:].reshape(-1, N, dim),
            "dim": dim}


# --------------------------------------------------------------------------- #
# diagnostics
# --------------------------------------------------------------------------- #
def velocity(xi, vt, k1, d, mu):
    """u_i = d xi_i/dt = phi_i - k1 xi_i + vi_i (target frame)."""
    out = np.empty_like(xi)
    for k in range(len(xi)):
        out[k] = repulsion(xi[k], d, mu) - k1 * xi[k] + vt[k]
    return out


def omega_per_agent(xi, vxi):
    """omega_i = (xi x vxi)/|xi|^2, per agent per time.  (T, N) plane only."""
    assert xi.shape[-1] == 2
    cross = xi[..., 0] * vxi[..., 1] - xi[..., 1] * vxi[..., 0]
    r2 = np.sum(xi ** 2, axis=-1)
    with np.errstate(divide="ignore", invalid="ignore"):
        return cross / r2


def steady_omega(res, k1, d, mu, tail=0.2):
    """Mean and spread of the per-agent angular velocity on the last `tail`."""
    T = len(res["t"])
    sl = slice(int((1.0 - tail) * T), T)
    vxi = velocity(res["xi"][sl], res["vt"][sl], k1, d, mu)
    w = omega_per_agent(res["xi"][sl], vxi)
    return float(np.nanmean(w)), float(np.nanmax(w) - np.nanmin(w))


def target_frame_snapshot(res, idx):
    return res["xi"][idx]


def rotation_to_comoving(xi, omega, dt):
    """Rotate a single snapshot into the frame co-rotating at `omega`."""
    th = -omega * dt
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return xi @ R.T


def repulsion_hessian(xi, d, mu):
    """Hessian of the repulsive potential P = sum_{i<j} Phi(|x_i-x_j|).

    Phi'(s) = alpha(s).  Returned as a (N*dim, N*dim) matrix acting on
    the flattened shape vector.
    """
    n, dim = xi.shape
    H = np.zeros((n * dim, n * dim))
    for i in range(n):
        for j in range(i + 1, n):
            e = xi[i] - xi[j]
            s = np.linalg.norm(e)
            if s > mu or s <= d:
                continue
            a = 1.0 / (s - d) - 1.0 / (mu - d)
            ap = -1.0 / (s - d) ** 2
            u = e / s
            # Hessian of Phi(|e|) w.r.t. e
            Hij = ap * np.outer(u, u) + (a / s) * (np.eye(dim) - np.outer(u, u))
            H[i * dim:(i + 1) * dim, j * dim:(j + 1) * dim] = -Hij
            H[j * dim:(j + 1) * dim, i * dim:(i + 1) * dim] = -Hij
            H[i * dim:(i + 1) * dim, i * dim:(i + 1) * dim] += Hij
            H[j * dim:(j + 1) * dim, j * dim:(j + 1) * dim] += Hij
    return H


def project_shape(M, n, dim=2):
    """Restrict an (n*dim) x (n*dim) operator to the zero-mean subspace."""
    Q = np.kron(np.eye(n) - np.ones((n, n)) / n, np.eye(dim))
    return Q.T @ M @ Q


def double_integrator_rhs(N, d, mu, k1, k2, k3):
    def rhs(t, z):
        e = z[: 2 * N].reshape(N, 2)
        v = z[2 * N: 4 * N].reshape(N, 2)
        w = z[4 * N:].reshape(N, 2)
        phi = repulsion(e, d, mu)
        return np.concatenate([v.ravel(),
                               (-k1 * e - k2 * v + phi + w).ravel(),
                               (-k3 * e).ravel()])
    return rhs


def integrate_double(e0, v0, w0, d, mu, k1, k2, k3, tf, n_eval=400,
                     method="BDF", rtol=1e-9, atol=1e-11):
    e0, v0, w0 = np.asarray(e0, float), np.asarray(v0, float), np.asarray(w0, float)
    N = len(e0)
    z0 = np.concatenate([e0.ravel(), v0.ravel(), w0.ravel()])
    t_eval = np.linspace(0.0, tf, n_eval)
    sol = solve_ivp(double_integrator_rhs(N, d, mu, k1, k2, k3),
                    (0.0, tf), z0, method=method, t_eval=t_eval,
                    rtol=rtol, atol=atol)
    if not sol.success:
        raise RuntimeError(sol.message)
    z = sol.y.T
    return {"t": sol.t,
            "e": z[:, : 2 * N].reshape(-1, N, 2),
            "v": z[:, 2 * N: 4 * N].reshape(-1, N, 2),
            "w": z[:, 4 * N:].reshape(-1, N, 2)}


# --------------------------------------------------------------------------- #
# figure defaults
# --------------------------------------------------------------------------- #
def style():
    import matplotlib as mpl
    mpl.rcParams.update({
        "figure.dpi": 150,
        "savefig.dpi": 300,
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Nimbus Roman", "Times",
                       "DejaVu Serif"],
        "font.size": 9,
        "axes.labelsize": 9,
        "axes.titlesize": 9.5,
        "legend.fontsize": 8,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "grid.linewidth": 0.5,
        "axes.axisbelow": True,
        "lines.linewidth": 1.3,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
        "pdf.fonttype": 42,
        "mathtext.fontset": "stix",   # Times-like math to match the text font
    })
