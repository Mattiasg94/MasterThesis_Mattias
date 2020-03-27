import time
import casadi.casadi as cs
#-------- Both
ts = 0.4
(nu, nx, nref, nu_init,N) = (2, 3, 3,2, 40)
nObs = (6)
(Qx, Qy, Qtheta) = (10, 10, 0)
(Rv, Rw) = (0, 0)
(vmin, vmax) = (0, 0.52)
(wmin, wmax) = (-3, 3)
(dv,dw)=(0.1,0.05)

def model_dd(x, y, theta, v, w):
    x += ts*cs.cos(theta)*v
    y += ts*cs.sin(theta)*v
    theta += ts*w
    return x, y, theta
#------- Point
(Qtx, Qty, Qttheta) = (10, 10, 10)

#-------- Traj
len_traj=N
NperTraj=N/len_traj
xtraj=[]
ytraj=[]
thetatraj=[]

#------- run_opt stuff
plot_x = []
plot_y = []
plot_theta = []
total_sec = 0
just_changed_ref = 0
justChanged = False
close_to_target= False
last_time=time.time()
# ------- Camera setup
first_cam_measurment=True