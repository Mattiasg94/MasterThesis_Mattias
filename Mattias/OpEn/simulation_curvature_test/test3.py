from setup import *
import numpy as np



def get_omega_ego(v,x,y,th):
    vx= v*np.cos(th)
    vy= v*np.sin(th)
    v = [-vx,-vy]
    p= np.array([x-center[0],y-center[1]])
    b_hat=p/np.linalg.norm(p)
    v_b_unit=np.dot(v,b_hat)
    print(v_b_unit)
    w=np.linalg.norm(v)-v_b_unit
    return w,np.linalg.norm(v)


v=0.5
x=10
y=5
th=0
w,v=get_omega_ego(v,x,y,th)
print(w,v)