import time
import casadi.casadi as cs
#-------- Both
ts = 2
(nu, nx, nref, nu_init,n_actv_lane,N) = (2, 3, 3,2,1, 12)
nObs = (6*2)
(Qx, Qy, Qtheta) = (10, 10, 0)
(Rv, Rw) = (1, 1)
(Rvj, Rwj) = (5, 5)
(vmin, vmax) = (0, 0.5)
(wmin, wmax) = (-3, 3)
(dv,dw)=(0.05,0.05)

def model_dd(x, y, theta, v, w):
    x += ts*cs.cos(theta)*v
    y += ts*cs.sin(theta)*v
    theta += ts*w
    return x, y, theta
#------- Point
(Qtx, Qty, Qttheta) = (100, 100, 100)
ymin=3.5
lane1=4
lower_road_line=4.5
lane2=5
upper_road_line=5.5
lane3=6
ymax=6.5
three_lanes=True
#-------- Traj
len_traj=N
NperTraj=N/len_traj
xtraj=[]
ytraj=[]
thetatraj=[]

#------- run_opt stuff
plot_history=[[],[]]
plot_x = []
plot_y = []
plt_x=[-1,41]
plt_y=[0,10]
plot_theta = []
total_sec = 0
just_changed_ref = 0
justChanged = False
close_to_target= False
last_time=time.time()
warm_start=False