"""
Shared ODE machinery for the fencing rotation conjecture.

Provides the interaction kernel α(s), the RHS builder, the integrator
wrapper, and a dense-output evaluation helper so that simulation scripts
can focus on initial conditions while sharing the same numerical backend.
"""

import numpy as np
import warnings
from scipy.integrate import solve_ivp


# ── Interaction kernel ───────────────────────────────────────────────────────
def alpha(s: float, d: float, mu: float) -> float:
    """Repulsive interaction kernel (document Eq.~1)."""
    if s > mu:
        return 0.0
    if s <= d:
        warnings.warn(f"Collision risk: s={s:.4f} <= d={d}")
        return 1e6
    return 1.0 / (s - d) - 1.0 / (mu - d)


# ── ODE right-hand side ──────────────────────────────────────────────────────
def make_rhs(d: float, mu: float, k1: float, k2: float):
    """Return a function rhs(t, z) for the fencing ODE system."""
    def rhs(t, z):
        n = len(z) // 4
        xi = z[:2 * n].reshape(n, 2)
        vt = z[2 * n:].reshape(n, 2)
        phi = np.zeros_like(xi)
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                e = xi[i] - xi[j]
                s = np.linalg.norm(e)
                phi[i] += alpha(s, d, mu) * e / s
        #  d(xi)/dt = phi  -  k1·xi  +  vt
        #  d(vt)/dt = -k2·xi
        return np.concatenate([(phi - k1 * xi + vt).ravel(),
                               (-k2 * xi).ravel()])
    return rhs


# ── Integrator ───────────────────────────────────────────────────────────────
def integrate(xi0, vt0, d, mu, k1, k2, tf,
              method="BDF", rtol=1e-10, atol=1e-12):
    """Integrate the fencing ODE and return (rhs, OdeSolution)."""
    n = len(xi0)
    rhs = make_rhs(d, mu, k1, k2)
    z0 = np.concatenate([np.asarray(xi0).ravel(), np.asarray(vt0).ravel()])
    sol = solve_ivp(rhs, [0, tf], z0, method=method,
                    rtol=rtol, atol=atol, dense_output=True)
    assert sol.success, f"Integration failed: {sol.message}"
    return rhs, sol


def evaluate(sol, n, t_eval):
    """Evaluate dense solution at t_eval, return (pos_all, vel_all).

    pos_all : ndarray, shape (len(t_eval), n, 2)
    vel_all : ndarray, shape (len(t_eval), n, 2)
    """
    z_all = sol.sol(t_eval)                     # (4n, n_frames)
    pos_all = z_all[:2 * n].T.reshape(-1, n, 2)
    vel_all = z_all[2 * n:].T.reshape(-1, n, 2)
    return pos_all, vel_all