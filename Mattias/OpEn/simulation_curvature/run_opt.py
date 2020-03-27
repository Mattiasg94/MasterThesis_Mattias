from setup import *
import opengen as og
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
import math
import time
# -------Init road
plot_x_curv,plot_y_curv=get_curvature_plots()
# -------Init ego
(x_init,init_lane, theta_init,r) = (0,1, 0,0.26)
(xref, yref_num_line, thetaref) = (lenght, 1, 0)
yref=get_y_from_lane(yref_num_line,xref)
(v_init, w_init) = (0, 0)
# -------Init Obstacles
penalty_margin=0.3
(X_OBS,Y_LANE_OBS, THETA_OBS, R_OBS) = ([15,25],[1,2] ,[math.pi,math.pi], [0.26,0.26])
Y_OBS=[get_y_from_lane(Y_LANE_OBS[0],X_OBS[0]),get_y_from_lane(Y_LANE_OBS[1],X_OBS[1])]
(V_OBS, W_OBS) = ([0.1,-0.1], [0,0])
# -------Init static stuff
R_CONE=[j+r+penalty_margin for j in R_OBS]
NOW_POS_OBS=[X_OBS,Y_OBS]
LAST_POS_OBS=NOW_POS_OBS.copy()
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
u_star = [0.0] * (nu*N)
yref_radius=road_radius_frm_lane(yref_num_line)
mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
mng.start()
y_init=get_y_from_lane(init_lane,x_init) #plot_y_curv[init_lane-1][0]+linewidth/2

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
    xs = [X[0], X[0] + np.cos(theta_init)*0.3]
    ys = [Y[0], Y[0] + np.sin(theta_init)*0.3]
    ax.plot(plot_history[0],plot_history[1],'--', color='g', markersize=0.1)
    ax.plot(X, Y, 'o', color='g', markersize=3)
    ax.plot(x_init, y_init, '-o', color='b', markersize=2)
    for i in range(len(X_OBS)):
        ax.add_patch(plt.Circle((X_OBS[i], Y_OBS[i]), R_OBS[i], color='r'))
        ax.add_patch(plt.Circle((X_OBS[i], Y_OBS[i]), R_OBS[i]+r,edgecolor='r',fill=None,linestyle='dashed'))
        ax.add_patch(plt.Circle((X_OBS[i], Y_OBS[i]), R_OBS[i]+R_CONE[i], edgecolor='r',fill=None,linestyle='dotted'))
    # for i in range(len(X_C)):
    #     ax.add_patch(plt.Circle((X_C[i], Y_C[i]), 0.1, color='b'))
    ax.plot(xs, ys, '-', color='r')
    for lane in range(lines):
        if lane==0 or lane==(lines-1):
            ax.plot(plot_x_curv[lane],plot_y_curv[lane], '-', color='k', linewidth=0.5)
        else:
            ax.plot(plot_x_curv[lane],plot_y_curv[lane], '--', color='b', linewidth=1)
    ax.set_xlim(-1,max(plot_x_curv[lines-1])+1)
    ax.set_ylim(0,max(plot_y_curv[lines-1])+2)
    ax.add_patch(plt.Circle((xref, yref), 0.3, color='y'))
   

i=-1
while i<200:
    i+=1
    if close_to_target:
        break
    V_OBS_EST,LAST_POS_OBS=calc_obs_v_est(LAST_POS_OBS,NOW_POS_OBS)
    plot_history[0].append(x_init)
    plot_history[1].append(y_init)
    p_lst = [x_init, y_init, theta_init, v_init, w_init, xref, yref, thetaref]
    for lst in (X_OBS, Y_OBS, THETA_OBS, V_OBS_EST, Y_LANE_OBS, R_CONE):
        p_lst.extend(lst)
    radius_ego=np.sqrt((x_init-lane_offset_x)**2+(y_init+(lane_border_min-lane_offset_y))**2)
    # if radius_ego<(lane_border_min+linewidth):
    #      actv_lane=lane1
    # elif (lane_border_min+linewidth)<=radius_ego:
    #     actv_lane=lane2
    actv_lane=yref_radius
    p_lst.append(actv_lane)
    u_star=[0.0] * (nu*N) if not warm_start else u_star
    solution = mng.call(p_lst, initial_guess=u_star)
    if just_changed_ref > 0:
        just_changed_ref -= 0.5
    justChanged = False
    for j in range(len(X_OBS)):
        (X_OBS[j], Y_OBS[j])=obs_move_line(Y_LANE_OBS[j],V_OBS[j],X_OBS[j], Y_OBS[j])
        # (X_OBS[j], Y_OBS[j], THETA_OBS[j]) = model_dd(X_OBS[j], Y_OBS[j], THETA_OBS[j], V_OBS[j], W_OBS[j])
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
    # test
    # (x,xkm1,x_obs,y,ykm1,y_obs)=(X[1],X[0],X_OBS[0],Y[1],Y[0],Y_OBS[0])
    # s=np.linalg.norm(theta_init-THETA_OBS[0])
    # print(s)
    # print('--------')
    curr_dist2ref = math.sqrt((X[0]-xref)**2+(Y[0]-yref)**2)
    avg_progress = math.sqrt((sum(X[1:])/len(X[1:])-xref)**2+(sum(Y[1:])/len(X[1:])-yref)**2) < (curr_dist2ref+just_changed_ref)
    progress = math.sqrt((X[-1]-xref)**2+ (Y[-1]-yref)**2) < (curr_dist2ref+just_changed_ref)
    close_to_target = math.sqrt((X[0]-xref)**2+(Y[0]-yref)**2) < 0.2
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
