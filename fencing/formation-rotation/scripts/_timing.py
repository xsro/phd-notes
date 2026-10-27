import time, numpy as np, sys
sys.path.insert(0,'.')
from fencing_lib import *
d,mu,k1,k2 = 5.0,9.0,0.5,0.5
th=np.linspace(0,2*np.pi,6,endpoint=False)
xi0=np.column_stack([8*np.cos(th),8*np.sin(th)]); vt0=np.zeros((6,2))
for tf,ne in [(80,400)]:
    t0=time.time(); r=integrate_single(xi0,vt0,d,mu,k1,k2,tf,ne); dt=time.time()-t0
    w,sp=steady_omega(r,k1,d,mu)
    print(f"2D rot tf={tf}: {dt:.1f}s omega={w:.5f} spread={sp:.2e} nsteps_ok")
XI0=np.array([[-10.584563,-12.532987],[-5.382648,5.904905],[-0.177347,9.175920],[-7.352506,-4.805249],[-6.288338,12.435830]])
VT0=np.array([[3.528048,-1.274511],[-0.511988,-1.485443],[1.972071,-3.679895],[-3.460489,-0.767699],[-2.039248,2.761800]])
for tf in [600,1200]:
    t0=time.time(); r=integrate_single(XI0,VT0,d,mu,k1,k2,tf,600); dt=time.time()-t0
    print(f"2D breathing tf={tf}: {dt:.1f}s")
# 3D
rng=np.random.default_rng(0)
def rand3d(n,dcol):
    x=[]
    while len(x)<n:
        p=rng.uniform(-15,18,3)
        if all(np.linalg.norm(p-q)>dcol*1.6 for q in x): x.append(p)
    return np.array(x)
xi3=rand3d(6,5.0)
t0=time.time(); r=integrate_single(xi3,np.zeros((6,3)),d,mu,k1,k2,300,300); print(f"3D breathing T=300: {time.time()-t0:.1f}s")
