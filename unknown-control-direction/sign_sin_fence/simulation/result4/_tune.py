# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy"]
# ///
"""Parameter tuning helper (temporary)."""
import numpy as np

N, m = 4, 2
d_safe, mu, l0, c_a, c_b = 0.45, 1.20, 0.75, 0.30, 0.02
b_switch = np.array([25.0, 45.0])
b_min = 0.80 * np.ones((N, m))


def bmat(t, bmax=2.0):
    B = np.zeros((N, m))
    for i in range(N):
        for l in range(m):
            mag = min(b_min[i, l] + 0.3 * ((2 * i + 3 * l) % 3), bmax)
            B[i, l] = (-1) ** int(np.sum(t >= b_switch + 3 * i + 2 * l)) * mag
    return B


def coeff(s):
    if s < 1e-9:
        return 0.0
    rho = max(1.0 / (s - d_safe) - 1.0 / (mu - d_safe), 0.0) if s < mu else 0.0
    return rho - c_a * max(s - l0, 0.0) - (c_b * max(1.0 / (mu - s) - 1.0 / (mu - l0), 0.0) if s < mu else 0.0)


def field(p):
    phi = np.zeros((N, m))
    for i in range(N):
        for j in range(i + 1, N):
            pij, s = p[i] - p[j], np.linalg.norm(p[i] - p[j])
            if s < 1e-9:
                continue
            c = coeff(s)
            phi[i] += c * pij / s
            phi[j] -= c * pij / s
    return phi


def run(k1=1.0, k2=1.0, kap1=1.0, kap2=1.0, c1=3.0, c2=8.0, a1=4.0, a2=4.0,
        tau=8.0, T=40.0, dt=2e-3, clip=0.9999, umax=500.0, verbose=False):
    p = np.array([[2.00, 0.40], [0.40, 1.80], [-1.80, -0.30], [-0.20, -1.90]])
    v = np.array([[0.00, 0.10], [-0.10, 0.00], [0.00, -0.10], [0.10, 0.00]])
    eta = np.zeros((N, m))
    xi1 = np.zeros((N, m))
    xi2 = np.zeros((N, m))
    p0 = np.zeros(2)
    v0 = np.array([0.15, 0.0])
    Nt = int(T / dt) + 1
    r1m = r2m = um = 0.0
    ncl = np.zeros(3)
    dist = []
    sep = []
    vrel = []
    for k in range(Nt):
        t = k * dt
        phit = np.sin(np.pi * t / (2 * tau)) if t < tau else 1.0
        ph = field(p)
        y = eta + k2 * (p - p0)
        e1 = phit * y
        r1 = np.clip(e1 / kap1, -clip, clip)
        ncl[0] += np.sum(np.abs(e1 / kap1) > clip)
        e1v = np.tan(0.5 * np.pi * r1)
        alpha1 = c1 * e1v * np.cos(e1v)
        e2 = xi2 - alpha1
        r2 = np.clip(e2 / kap2, -clip, clip)
        ncl[1] += np.sum(np.abs(e2 / kap2) > clip)
        u = -c2 * np.tan(0.5 * np.pi * r2)
        ncl[2] += np.sum(np.abs(u) > umax)
        u = np.clip(u, -umax, umax)
        r1m = max(r1m, np.max(np.abs(r1)))
        r2m = max(r2m, np.max(np.abs(r2)))
        um = max(um, np.max(np.abs(u)))
        dist.append(np.linalg.norm(p.mean(0) - p0))
        sep.append(min(np.linalg.norm(p[i] - p[j]) for i in range(N) for j in range(i + 1, N)))
        vrel.append(np.max(np.linalg.norm(v - v0, axis=1)))
        if k == Nt - 1:
            break
        eps = 0.2 * np.sin(1.7 * t + 0.4 * np.arange(N))[:, None] * np.array([0.8, 0.7])
        u0 = 0.15 * np.array([0.8 * np.sin(0.6 * t), 0.8 * np.cos(0.5 * t)])
        p = p + dt * v
        v = v + dt * (bmat(t) * u + eps)
        eta = eta + dt * (k1 * (p - p0) - ph)
        xi1, xi2 = xi1 + dt * (-a1 * xi1 + xi2), xi2 + dt * (-a2 * xi1 + u)
        p0 = p0 + dt * v0
        v0 = v0 + dt * u0
    dist = np.array(dist)
    sep = np.array(sep)
    return dict(r1=r1m, r2=r2m, umax=um, dmax=dist.max(), dtail=dist[-int(0.25 * Nt):].max(),
                sepmin=sep.min(), vrel=max(vrel), clips=ncl)


cases = [
    dict(kap1=1, kap2=2, c1=20, c2=20, a1=1, a2=1, tau=8, dt=1e-4, T=20, umax=1e4),
    dict(kap1=1, kap2=2, c1=20, c2=20, a1=1, a2=1, tau=8, dt=2e-4, T=20, umax=1e4),
    dict(kap1=1, kap2=2, c1=20, c2=20, a1=1, a2=1, tau=8, dt=5e-4, T=20, umax=1e4),
]
for i, kw in enumerate(cases):
    r = run(**kw)
    print(f"case {i}: {kw}")
    print("   ", {kk: (round(float(vv), 4) if not hasattr(vv, '__len__') else vv) for kk, vv in r.items()})
