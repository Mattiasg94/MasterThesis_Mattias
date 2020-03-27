from setup import *
from build_opt_traj import len_traj
from build_opt_traj_tilde import len_traj, model_dd_tilde,v_ref
import opengen as og
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
import math
import time

# -------Init ego
(x_init, y_init, theta_init,r) = (0, 6, 0,0.26)
(xref, yref, thetaref) = (40, 6, 0)
(v_init, w_init) = (0, 0)
Y_REF = [11, 10.99, 10.96, 10.91, 10.84, 10.76, 10.65, 10.53, 10.39, 10.24, 10.08, 9.91, 9.72, 9.53, 9.34, 9.14, 8.94, 8.74, 8.55, 8.35, 8.17, 7.99, 7.82, 7.67, 7.53, 7.4, 7.29, 7.19, 7.12, 7.06, 7.02, 7.0, 7.0, 7.03, 7.07, 7.13, 7.21, 7.3, 7.42, 7.55, 7.69, 7.85, 8.02, 8.2, 8.39, 8.58, 8.78, 8.98, 9.17, 9.37, 9.57, 9.76, 9.94, 10.11, 10.27, 10.42, 10.55, 10.67, 10.77, 10.85, 10.92, 10.97, 10.99]
X_REF = [0.0, 0.16, 0.32, 0.48, 0.65, 0.81, 0.97, 1.13, 1.29, 1.45, 1.61, 1.77, 1.94, 2.1, 2.26, 2.42, 2.58, 2.74, 2.9, 3.06, 3.23, 3.39, 3.55, 3.71, 3.87, 4.03, 4.19, 4.35, 4.52, 4.68, 4.84, 5.0, 5.16, 5.32, 5.48, 5.65, 5.81, 5.97, 6.13, 6.29, 6.45, 6.61, 6.77, 6.94, 7.1, 7.26, 7.42, 7.58, 7.74, 7.9, 8.06, 8.23, 8.39, 8.55, 8.71, 8.87, 9.03, 9.19, 9.35, 9.52, 9.68, 9.84, 10.0]
THETA_REF=[0.0, -0.07841, -0.156035, -0.2321, -0.3058475, -0.37654, -0.44347, -0.5059675, -0.56341, -0.6152225, -0.66089, -0.6999525, -0.7320225, -0.7567775, -0.77397, -0.78343, -0.7850625, -0.7788525, -0.7648575, -0.7432225, -0.71416, -0.6779625, -0.6349925, -0.585675, -0.5305075, -0.47004, -0.404875, -0.3356625, -0.2631, -0.187905, -0.110835, -0.0326575, 0.0458475, 0.1238925, 0.2007025, 0.275505, 0.347555, 0.4161325, 0.4805525, 0.54017, 0.5943925, 0.6426725, 0.684535, 0.719555, 0.7473875, 0.76775, 0.7804425, 0.7853375, 0.782385, 0.7716175, 0.7531375, 0.7271325, 0.6938625, 0.6536625, 0.6069275, 0.55413, 0.495795, 0.4325075, 0.3648975, 0.2936425, 0.2194525, 0.14307, 0.0652575]

Y_REF.extend([Y_REF[-1]]*len_traj*2)
X_REF.extend([X_REF[-1]]*len_traj*2)
# THETA_REF.insert(0,0)
THETA_REF.extend([THETA_REF[-1]]*len_traj*2)
modes=['point','traj','point','point']
mode=modes[0]
run_with_tilde=False
# -------Init Obstacles
penalty_margin=0.3
(X_OBS, Y_OBS, THETA_OBS, R_OBS) = ([15,20], [5,6], [math.pi,math.pi], [0.26,0.26])
(V_OBS, W_OBS) = ([0.15,0.05], [0,0])

# -------Init static stuff

R_CONE=[j+r+penalty_margin for j in R_OBS]
NOW_POS_OBS=[X_OBS,Y_OBS]
LAST_POS_OBS=NOW_POS_OBS.copy()
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
u_star = [0.0] * (nu*N)
if mode=='point':
    mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
    mng.start()
else:
    if run_with_tilde:
        mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj_tilde')
    else:
        mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj')
    mng2.start()

def get_cone_const(x,y,theta,x_obs,y_obs,uk):
    rterm = (x-x_obs)**2+(y-y_obs)**2
    uterm = ((cs.cos(theta)*uk-v_obs)*(x-x_obs) +  (cs.sin(theta)*uk-v_obs)*(y-y_obs))**2
    lterm = (cs.cos(theta)*uk-v_obs)**2+(cs.sin(theta)*uk-v_obs)**2
    # d=rterm-(uterm/lterm)
    # r=np.sqrt((x-x_obs)**2+(y-y_obs)**2)
    vab=np.sqrt(uterm/lterm)
    x_c=np.cos(theta)*vab
    y_c=np.sin(theta)*vab
    # print('d:',np.sqrt(d),'Const:',np.sqrt(d)>=r_obs+r)
    X_C.append(x_c+x)
    Y_C.append(y_c+y)


def calc_obs_v_est(LAST_POS_OBS,NOW_POS_OBS):
    global last_time
    V_OBS_EST=[]
    for i in range(2):
        dist=np.sqrt(np.power(LAST_POS_OBS[0][i]-NOW_POS_OBS[0][i],2)+np.power(LAST_POS_OBS[1][i]-NOW_POS_OBS[1][i],2))
        dt=time.time()-last_time
        last_time=time.time()
        V_OBS_EST.append(dist/ts) # byt ut mot dt senare #TODO
    X_OBS_tlf=[i for i in X_OBS]
    Y_OBS_tlf=[i for i in Y_OBS]
    LAST_POS_OBS=[X_OBS_tlf,Y_OBS_tlf]
    return V_OBS_EST,LAST_POS_OBS

def dist_to_ref(x_init, y_init):
    ref_abs_lst = []
    for i in range(len(X_REF)):
        ref_abs_lst.append(
            math.sqrt((X_REF[i]-x_init)**2+(Y_REF[i]-y_init)**2))
    min_val = min(ref_abs_lst)
    idx = ref_abs_lst.index(min_val)
    return idx+1


def calculate_turn_dir(angle_ego):
    angle_ego = np.degrees(angle_ego)
    angle_ref = math.degrees(math.atan2(
        yref-y_init, xref-x_init))  # check this one!
    clws_angle_ref = angle_ref if angle_ref >= 0 else 180+abs(180+angle_ref)
    if abs(clws_angle_ref-angle_ego) >= 180:
        return -ts*dw
    else:
        return ts*dw


def animate(i):
    ax.cla()
    ax.set_xlim((plt_x[0], plt_x[1]))
    ax.set_ylim((plt_y[0], plt_y[1]))
    xs = [X[0], X[0] + np.cos(theta_init)*0.3]
    ys = [Y[0], Y[0] + np.sin(theta_init)*0.3]
    ax.plot(plot_history[0],plot_history[1],'--', color='g', markersize=0.1)
    ax.plot(X, Y, 'o', color='g', markersize=3)
    ax.plot(plot_x[-1], plot_y[-1], '-o', color='b', markersize=2)
    if mode=='point':
        ax.add_patch(plt.Circle((end_xref, end_yref), 0.3, color='y'))
    else:
        #ax.plot(xtraj, ytraj, '--', color='y',)
        ax.plot(X_REF, Y_REF, '--', color = 'y',markersize=1)
    for i in range(len(X_OBS)):
        ax.add_patch(plt.Circle((X_OBS[i], Y_OBS[i]), R_OBS[i], color='r'))
        ax.add_patch(plt.Circle((X_OBS[i], Y_OBS[i]), R_OBS[i]+r,edgecolor='r',fill=None,linestyle='dashed'))
        ax.add_patch(plt.Circle((X_OBS[i], Y_OBS[i]), R_OBS[i]+R_CONE[i], edgecolor='r',fill=None,linestyle='dotted'))
    # for i in range(len(X_C)):
    #     ax.add_patch(plt.Circle((X_C[i], Y_C[i]), 0.1, color='b'))
    ax.plot(xs, ys, '-', color='r')
    ax.plot(plt_x, [ymin]*2, '-', linewidth=0.5,color='b')
    if three_lanes:
        ax.plot(plt_x, [lower_road_line]*2, '--', linewidth=0.2,color='b')
        ax.plot(plt_x, [upper_road_line]*2, '--', linewidth=0.2,color='b')
    else:
        ax.plot(plt_x, [(ymax-ymin)//2+ymin]*2, '--', linewidth=0.2,color='b')
    ax.plot(plt_x, [ymax]*2, '-', linewidth=0.5,color='b')
    

i=-1
idx_mode=0
while i<200:
    i+=1
    if close_to_target:
        break
        last_mode=mode
        plot_x=[plot_x[-1]]
        plot_y=[plot_y[-1]] # curr pos
        idx_mode=idx_mode+1 if not idx_mode==len(modes) else 0
        mode=modes[idx_mode]
        just_changed_ref = 5
        justChanged = True  
        if idx_mode==0:
            (x_obs, y_obs, theta_obs, r_obs) = (-10, -10, 0, 0.5) 
            (v_obs, w_obs) = (0.0, 0.00001)   
        elif idx_mode==1:
            (xref, yref, thetaref) = (9, 1, 0)
            (x_obs, y_obs, theta_obs, r_obs) = (9, 5, 0, 0.5)
            (v_obs, w_obs) = (-0.1, 0.00001)
        else:
            (xref, yref, thetaref) = (0, 9.5, 0)
            (x_obs, y_obs, theta_obs, r_obs) = (6, 6, 5*math.pi/4, 0.5)
            (v_obs, w_obs) = (0.1, 0.00001)
        now_obs_pos=(x_obs,y_obs)
        last_obs_pos=(x_obs,y_obs)
    
    V_OBS_EST,LAST_POS_OBS=calc_obs_v_est(LAST_POS_OBS,NOW_POS_OBS)
    plot_x.append(x_init)
    plot_y.append(y_init)
    plot_theta.append(theta_init)
    plot_history[0].append(x_init)
    plot_history[1].append(y_init)
    p_lst = [x_init, y_init, theta_init, v_init, w_init, xref, yref, thetaref]
    for lst in (X_OBS, Y_OBS, THETA_OBS, V_OBS_EST, W_OBS, R_CONE):
        p_lst.extend(lst)
    if ymin<=y_init<lower_road_line:
        actv_lane=lane1
    elif lower_road_line<=y_init<upper_road_line:
        actv_lane=lane2
    else:
        actv_lane=lane3
    p_lst.append(actv_lane)
    u_star=[0.0] * (nu*N) if not warm_start else u_star
    if mode=='point':
        if justChanged and last_mode!=mode:
            mng2.kill()
            mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
            mng.start()
        end_xref = xref
        end_yref = yref
        solution = mng.call(p_lst, initial_guess=u_star)
    else:
        if justChanged and last_mode!=mode:
            mng.kill()
            if run_with_tilde:
                mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj_tilde')            
            else:
                mng2 = og.tcp.OptimizerTcpManager('optimizers/ref_traj')
            mng2.start()
        idx = dist_to_ref(x_init, y_init)
        xtraj = X_REF[idx:idx+len_traj]
        ytraj = Y_REF[idx:idx+len_traj]
        thetatraj = THETA_REF[idx:idx+len_traj]
        thetatraj = THETA_REF[i:i+len_traj]
        end_xref = xtraj[-1]
        end_yref = ytraj[-1]
        p_lst.extend(xtraj)
        p_lst.extend(ytraj)
        p_lst.extend(thetatraj)
        solution = mng2.call(p_lst, initial_guess=u_star)
    if just_changed_ref > 0:
        just_changed_ref -= 0.5
    justChanged = False
    for j in range(len(X_OBS)):
        (X_OBS[j], Y_OBS[j], THETA_OBS[j]) = model_dd(X_OBS[j], Y_OBS[j], THETA_OBS[j], V_OBS[j], W_OBS[j])
    NOW_POS_OBS=(X_OBS,Y_OBS)
    try:
        u_star = solution['solution']
        # print(solution['penalty'])
    except:
        print('No Solution')
        continue
    total_sec += solution['solve_time_ms']/1000
    uv = u_star[0:nu*N:2]
    uw = u_star[1:nu*N:2]
    X = [0.0]*(N+1)
    X[0] = x_init
    Y = [0.0]*(N+1)
    Y[0] = y_init
    THETA = [0.0]*(N+1)
    THETA[0] = theta_init
    collision = False
    X_OBS_FUTURE=X_OBS.copy()
    Y_OBS_FUTURE=Y_OBS.copy()
    (X_C,Y_C)=([],[])
    for t in range(0, N):
        u_t = u_star[t*nu:(t+1)*nu]
        x = X[t]
        y = Y[t]
        theta = THETA[t]
        X[t+1] = x + ts*np.cos(theta)*u_t[0]#+np.random.randn(1)[0]/100
        Y[t+1] = y + ts*np.sin(theta)*u_t[0]#+np.random.randn(1)[0]/100 
        THETA[t+1] = theta + ts*u_t[1]
        for j in range(len(X_OBS)):
            X_OBS_FUTURE[j]=X_OBS_FUTURE[j] + ts*np.cos(THETA_OBS[j])*V_OBS_EST[j]
            Y_OBS_FUTURE[j]=Y_OBS_FUTURE[j] + ts*np.sin(THETA_OBS[j])*V_OBS_EST[j]
            #get_cone_const(X[t+1],Y[t+1],THETA[t+1],x_obs_future,y_obs_future,u_t[0])
            if np.sqrt(np.power(X[t+1]-X_OBS_FUTURE[j],2)+np.power(Y[t+1]-Y_OBS_FUTURE[j],2)) <= (R_OBS[j]+r):
                if u_t[0]>=V_OBS_EST[j] or True:
                    collision = True
        if run_with_tilde:
            x, y, theta,x_tild, y_tild, theta_tild = model_dd_tilde(X[t+1], Y[t+1], THETA[t+1], u_t[0]-v_ref, u_t[1],xtraj[t],ytraj[t],thetatraj[t],u_t[0])
    # test
    # (x,xkm1,x_obs,y,ykm1,y_obs)=(X[1],X[0],X_OBS[0],Y[1],Y[0],Y_OBS[0])
    # s=np.linalg.norm(theta_init-THETA_OBS[0])
    # print(s)
    # print('--------')
    curr_dist2ref = math.sqrt((X[0]-end_xref)**2+(Y[0]-end_yref)**2)
    avg_progress = math.sqrt((sum(X[1:])/len(X[1:])-end_xref)**2+(sum(Y[1:])/len(X[1:])-end_yref)**2) < (curr_dist2ref+just_changed_ref)
    progress = math.sqrt((X[-1]-end_xref)**2+ (Y[-1]-end_yref)**2) < (curr_dist2ref+just_changed_ref)
    close_to_target = math.sqrt((X[0]-end_xref)**2+(Y[0]-end_yref)**2) < 0.2
    if not collision and progress and avg_progress and not close_to_target:
        (x_init, y_init, theta_init) = (X[1], Y[1], THETA[1])
        (v_init, w_init) = (uv[0], uw[0])
    if not close_to_target and not progress and not collision:
        theta_init += calculate_turn_dir(theta_init) 
    ani = animation.FuncAnimation(fig, animate, interval=100000)
    plt.pause(0.0005)
print('total_sec', total_sec)
print('avrage time_ms',total_sec*1000/i)
plt.show()
try:
    mng2.kill()
except:
    mng.kill()
