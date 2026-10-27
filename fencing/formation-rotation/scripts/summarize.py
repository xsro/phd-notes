"""Print every number quoted in the report's simulation section."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fencing_lib import distances
import make_figures as M

D, MU, K1, K2 = M.D, M.MU, M.K1, M.K2

def cache(name):
    z = np.load(os.path.join(M.CACHE, name + ".npz"))
    return {k: z[k] for k in z.files}

print("== 2D rotation (k2=0.5) ==")
r = cache("rot_k2-0.5")
w = M.omega_trace(r["xi"], r["vt"], K1)
sl = M.tail_slice(r["t"], 0.25)
print(f"  omega = {np.nanmean(w[sl]):.6f} (sqrt(k2)={np.sqrt(K2):.6f}), spread = {np.nanmax(w[sl])-np.nanmin(w[sl]):.2e}")
print(f"  radii = {np.round(np.sort(np.linalg.norm(r['xi'][-1],axis=1)),3)}")
print(f"  hexagon branch r* = {M.circle_root(K1,K2):.4f}")
g = M.pairdist_trace(r)
print(f"  gaps (end) = {np.round(np.sort(g[-1]),3)}")
print(f"  mean|vt| end = {np.linalg.norm(r['vt'][-1],axis=1).mean():.4f}")

print("== k2 sweep ==")
res = np.load(os.path.join(M.CACHE, "k2_sweep.npy"))
for k2, om, sp in res:
    print(f"  k2={k2:5.2f}: |omega|={abs(om):.6f} rel.err={abs(abs(om)-np.sqrt(k2))/np.sqrt(k2):.2e} spread={sp:.1e}")

print("== breathing (Case C, k1=k2=0.5) ==")
b = cache("breath_cc_k1-0.5_k2-0.5")
gb = M.pairdist_trace(b)
f, _, _, _ = M.dominant_freq(b["t"][M.tail_slice(b["t"],0.5)], gb[M.tail_slice(b["t"],0.5),0])
print(f"  f={f:.4f} Hz T={1/f:.4f}s  omega_eff/sqrt(k2)={2*np.pi*f/np.sqrt(K2):.3f}")
last = gb[M.tail_slice(b["t"],0.5)]
print(f"  min gap = {last.min():.3f}  pair amplitudes = {np.round(np.sort(last.max(0)-last.min(0))[::-1][:5],2)}")
wb = M.omega_trace(b["xi"][M.tail_slice(b['t'],0.5)], b["vt"][M.tail_slice(b['t'],0.5)], K1)
print(f"  mean omega = {np.nanmean(wb):.3f}, spread = {np.nanmax(wb)-np.nanmin(wb):.3f}")

print("== bistability / k1 ==")
for tag, k1 in [("cc_lo", 0.3), ("cc", 0.5), ("cc_hi", 0.7)]:
    rr = cache(f"breath_{tag}_k1-{k1:g}_k2-0.5")
    gg = M.pairdist_trace(rr)
    t2 = rr["t"] > 0.5 * rr["t"][-1]
    print(f"  k1={k1}: min gap last half = {gg[t2].min():.3f}, amp = {gg[t2].max(0).max()-gg[t2].min(0).min():.3f}")

print("== 3D ==")
r3 = cache("3d_random")
g3 = M.pairdist_trace(r3)
f3, _, _, _ = M.dominant_freq(r3["t"][M.tail_slice(r3["t"],0.5)], g3[M.tail_slice(r3["t"],0.5),0])
X = r3["xi"][M.tail_slice(r3["t"],0.5)]
X = X - X.mean(1, keepdims=True)
sv = np.array([np.linalg.svd(x, compute_uv=False) for x in X]).mean(0)
print(f"  3D breathing f={f3:.4f} Hz T={1/f3:.3f}s sigma3/sigma1={sv[2]/sv[0]:.3f}")
p3 = cache("3d_planar")
wp = M.omega_trace(p3["xi"][..., :2], p3["vt"][..., :2], K1)
slp = M.tail_slice(p3["t"], 0.25)
print(f"  planar omega={np.nanmean(wp[slp]):.6f} spread={np.nanmax(wp[slp])-np.nanmin(wp[slp]):.1e} max|z|={np.abs(p3['xi'][...,2]).max():.1e}")

print("== jam ==")
j = cache("jam_collinear")
gj = M.pairdist_trace(j).min(axis=1)
for tt in [50, 100, 200, 400]:
    k = np.searchsorted(j["t"], tt)
    print(f"  t={tt}: min gap-d={gj[k]-D:.3e}, (gap-d)t={((gj[k]-D)*j['t'][k]):.4f}, |v|={np.linalg.norm(j['vt'][k],axis=(0,1)):.1f}")

print("== double ==")
for k2 in M.K2_DBL:
    dd = cache(f"double3_k2-{k2:g}")
    e, v = dd["e"], dd["v"]
    om = (e[...,0]*v[...,1]-e[...,1]*v[...,0])/np.sum(e**2,-1)
    sl2 = M.tail_slice(dd["t"],0.5)
    print(f"  k2={k2}: |omega|={abs(np.nanmean(om[sl2])):.6f} pred={np.sqrt(M.K3_DBL/k2):.6f} radii_end={np.round(np.sort(np.linalg.norm(e[-1],axis=1)),2)}")

print("== direction ==")
for eps in [-1e-2,-1e-6,-1e-12,1e-12,1e-6,1e-2]:
    dr = cache(f"dir_eps-{eps:g}")
    wd = M.omega_trace(dr["xi"][M.tail_slice(dr['t'],0.25)], dr["vt"][M.tail_slice(dr['t'],0.25)], K1)
    tr = M.omega_trace(dr["xi"], dr["vt"], K1).mean(1)
    tgt = np.nanmean(wd)
    bad = np.where(np.abs(tr - tgt) > 1e-3)[0]
    tl = dr["t"][bad[-1]+1] if len(bad) and bad[-1]+1 < len(tr) else 0.0
    print(f"  eps={eps:8.0e}: omega={tgt:+.6f} lock t<={tl:.0f}s")
sy = cache("dir_symmetric")
L = np.sum(sy["xi"][...,0]*sy["vt"][...,1]-sy["xi"][...,1]*sy["vt"][...,0], axis=1)
ws = M.omega_trace(sy["xi"], sy["vt"], K1)
print(f"  symmetric: |L| start={abs(L[0]):.2e} end={abs(L[-1]):.2f}, omega_end={ws[-1].mean():.6f}, spread_end={ws[-1].max()-ws[-1].min():.1e}")
