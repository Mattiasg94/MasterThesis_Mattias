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
(x_init,init_lane, theta_init,r_ego) = (0,1, 0,0.26)
(xref, yref_num_line, thetaref) = (lenght, 1, 0)
yref=get_y_from_lane(yref_num_line,xref)
(v_init, w_init) = (vmax, 0)
# -------Init Obstacles
penalty_margin=0.3
(X_OBS,Y_LANE_OBS, THETA_OBS, R_OBS) = ([30,15],[2,1] ,[math.pi,math.pi], [0.26,0.26])
Y_OBS=[get_y_from_lane(Y_LANE_OBS[0],X_OBS[0]),get_y_from_lane(Y_LANE_OBS[1],X_OBS[1])]
V_OBS=[-0.6,0.2]
for j in range(2):
    _,_,THETA_OBS[j],W_OBS[j]=obs_move_line(Y_LANE_OBS[j],V_OBS[j],X_OBS[j], Y_OBS[j],THETA_OBS[j],ts)
# -------Init static stuff
R_CONE=[j+r_ego+penalty_margin for j in R_OBS]
NOW_POS_OBS=[X_OBS,Y_OBS]
LAST_POS_OBS=NOW_POS_OBS.copy()
fig = plt.figure(figsize=(10,5))
ax = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)
yref_radius=road_radius_frm_lane(yref_num_line)
mng = og.tcp.OptimizerTcpManager('optimizers/ref_point')
mng.start()
y_init=get_y_from_lane(init_lane,x_init) #plot_y_curv[init_lane-1][0]+linewidth/2
print('[Collision inactivated]')

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

def calculate_position_frm_u_opt(u_opt,n,Ts):
    uv = u_opt[0:nu*n:2]
    uw = u_opt[1:nu*n:2]
    X = [0.0]*(n+1)
    X[0] = x_init
    Y = [0.0]*(n+1)
    Y[0] = y_init
    THETA = [0.0]*(n+1)
    THETA[0] = theta_init
    X_OBS_FUTURE=X_OBS.copy()
    Y_OBS_FUTURE=Y_OBS.copy()
    collision=False
    for t in range(0, n):
        u_t = u_opt[t*nu:(t+1)*nu]
        x = X[t]
        y = Y[t]
        theta = THETA[t]
        X[t+1] = x + Ts[t]*np.cos(theta)*u_t[0]#+np.random.randn(1)[0]/100
        Y[t+1] = y + Ts[t]*np.sin(theta)*u_t[0]#+np.random.randn(1)[0]/100 
        THETA[t+1] = theta + Ts[t]*u_t[1]
        for j in range(len(X_OBS)):
            X_OBS_FUTURE[j]=X_OBS_FUTURE[j] + Ts[t]*np.cos(THETA_OBS[j])*V_OBS_EST[j]
            Y_OBS_FUTURE[j]=Y_OBS_FUTURE[j] + Ts[t]*np.sin(THETA_OBS[j])*V_OBS_EST[j]
            # get_cone_const(THETA_OBS[j],X[t+1],Y[t+1],THETA[t+1],X_OBS_FUTURE[j],X_OBS_FUTURE[j],u_t[0],V_OBS_EST[j])
            if np.sqrt(np.power(X[t+1]-X_OBS_FUTURE[j],2)+np.power(Y[t+1]-Y_OBS_FUTURE[j],2)) <= (R_OBS[j]+r_ego):
                if u_t[0]>=V_OBS_EST[j]:
                    pass
                    print('collision')
                    #collision = True
    return X,Y,THETA,collision,uv,uw

def animate(i):
    ax.cla()
    ax2.cla()
    xs = [X[0], X[0] + np.cos(theta_init)*0.3]
    ys = [Y[0], Y[0] + np.sin(theta_init)*0.3]
    ax.plot(plot_history[0],plot_history[1],'--', color='g', markersize=0.1)
    ax.plot(X, Y, 'o', color='g', markersize=3)
    ax.plot(x_init, y_init, '-o', color='b', markersize=2)
    for i in range(len(X_OBS)):
        ax.add_patch(plt.Circle((X_OBS[i], Y_OBS[i]), R_OBS[i], color='r'))
        ax.add_patch(plt.Circle((X_OBS[i], Y_OBS[i]), R_OBS[i]+r_ego,edgecolor='r',fill=None,linestyle='dashed'))
        ax.add_patch(plt.Circle((X_OBS[i], Y_OBS[i]), R_CONE[i], edgecolor='r',fill=None,linestyle='dotted'))
    ax.plot(xs, ys, '-', color='r')
    for lane in range(lines):
        if lane==0 or lane==(lines-1):
            ax.plot(plot_x_curv[lane],plot_y_curv[lane], '-', color='k', linewidth=0.5)
        else:
            ax.plot(plot_x_curv[lane],plot_y_curv[lane], '--', color='b', linewidth=1)
    ax.set_xlim(-1,max(plot_x_curv[lines-1])+1)
    ax.set_ylim(0,max(plot_y_curv[lines-1])+2)
    ax.add_patch(plt.Circle((xref, yref), 0.3, color='y'))
    ax2.plot(np.array(X), np.array(Y), 'b-', label='data')
    ax2.plot(np.array(X), curve_fit_func(np.array(X), *popt), 'r-',
            label='fit: a=%5.3f, b=%5.3f, c=%5.3f,d=%5.3f,e=%5.3f' % tuple(popt))
    ax2.text(0.95, 0.01, round(curve_fit_error,1), verticalalignment='bottom', horizontalalignment='right', transform=ax2.transAxes, color='black', fontsize=15)
    for j in range(2):
        ax.add_patch(plt.Circle((np.array(x_impact[j]),np.array(y_impact[j])), 0.2, color='y'))
    
i=-1
avrage_curve_fit_error=[]
print_cone=True
def print_cone_const(x_obs, y_obs,theta_obs, v_obs, r_obs, v, x, y, theta):
    v_obs_x=cs.cos(theta_obs)*v_obs
    v_obs_y=cs.sin(theta_obs)*v_obs
    rterm = (x-x_obs)**2+(y-y_obs)**2
    uterm = ((cs.cos(theta)*v-v_obs_x)*(x-x_obs) +
             (cs.sin(theta)*v-v_obs_y)*(y-y_obs))
    lterm = (cs.cos(theta)*v-v_obs_x)**2+(cs.sin(theta)*v-v_obs_y)**2
    cone = r_obs**2*lterm-(rterm*lterm-uterm**2)
    if cone>0:
        print('Cone above 0',round(cone,5))

while i<200:
    i+=1
    if close_to_target:
        break
    V_OBS_EST,LAST_POS_OBS=calc_obs_v_est(LAST_POS_OBS,NOW_POS_OBS)
    plot_history[0].append(x_init)
    plot_history[1].append(y_init)
    #------ Regular optimizer -------#
    if not run_vel_obs:
        if change_optimizer:
            mng,change_optimizer=switch_optimizer(mng,'point_ref')
        p_lst = [x_init, y_init, theta_init, v_init, w_init, xref, yref, thetaref]
        for lst in (X_OBS, Y_OBS, THETA_OBS, V_OBS_EST, Y_LANE_OBS, R_CONE):
            p_lst.extend(lst)
        actv_lane=yref_radius
        p_lst.append(actv_lane)
        u_opt=[0.1] * (nu*N) if not warm_start else u_opt
        solution = mng.call(p_lst, initial_guess=u_opt)
        try:
            u_opt = solution['solution']
            # print(solution['penalty'])
        except:
            print('[No Solution]')
            continue
        X,Y,THETA,collision,uv,uw=calculate_position_frm_u_opt(u_opt,N,ts_lst)
    #------ End -------#
    #------ VO optimizer -------#
    elif change_optimizer:
        if change_optimizer:
            mng,change_optimizer=switch_optimizer(mng,'vel_obs')
            num=0
        p_lst=[]
        for lst in (X_OBS, Y_OBS, THETA_OBS, V_OBS_EST,W_OBS, Y_LANE_OBS, R_CONE,[xref, yref, thetaref]):
            p_lst.extend(lst)
        # just because we do not produce new green dots only move forward i need to shorten X,Y
        for h in range(num):
            X.append(X[-1])
            Y.append(Y[-1])
            THETA.append(THETA[-1])
        p_lst.extend(X[num+1:])
        p_lst.extend(Y[num+1:])
        p_lst.extend(THETA[num+1:])
        num+=1
        v_opt=[0.5] * (N) #if not warm_start else t_opt
        solution = mng.call(p_lst, initial_guess=v_opt)
        try:
            uv = solution['solution']
        except:
            print('[No Solution]')
            continue
        u_opt, dt_lst=([],[])
        for f,v in enumerate(uv):
            s=np.sqrt((X[f]-X[f+1])**2+(Y[f]-Y[f+1])**2)
            dt=s/v
            dt_lst.append(dt)       
            w=(THETA[f+1]-THETA[f])/dt
            u_opt.extend([v,w])
        u_opt_test=u_opt.copy()
        print('[uv]',uv)
        print('[dt_lst]',dt_lst)
    #------ End -------#

    dt = ts if not run_vel_obs else dt_lst[backup_idx-1] # since VO changes dt we must do the same for obs
    for j in range(len(X_OBS)):
        (X_OBS[j], Y_OBS[j],THETA_OBS[j],W_OBS[j])=obs_move_line(Y_LANE_OBS[j],V_OBS[j],X_OBS[j], Y_OBS[j],THETA_OBS[j],dt)
    NOW_POS_OBS=(X_OBS,Y_OBS)
    # check convergense
    converged,avrage_curve_fit_error,curve_fit_error,popt=check_converged(X,Y,avrage_curve_fit_error,x_init,y_init,xref,yref)
    
    if not run_vel_obs:
        close_to_target = math.sqrt((X[0]-xref)**2+(Y[0]-yref)**2) < 0.2
        if not collision and converged and not close_to_target:
            (u_opt_km1,backup_idx)=(u_opt.copy(),1)
            (x_init, y_init, theta_init) = (X[1], Y[1], THETA[1])
            (v_init, w_init) = (uv[0], uw[0])
        elif not converged and i!=0:
            X,Y,THETA,collision,uv,uw=calculate_position_frm_u_opt(u_opt_km1[backup_idx*2:],N-backup_idx,ts_lst)
            (x_init, y_init, theta_init) = (X[1], Y[1], THETA[1])
            (v_init, w_init) = (uv[0], uw[0])
            backup_idx+=1
        if np.sqrt((X_OBS[0]-x_init)**2 - (Y_OBS[0]-y_init)**2)<overtaking_dist and not run_vel_obs_deactivated:
            run_vel_obs=True
            change_optimizer=True
    else:
        X,Y,THETA,collision,uv,uw=calculate_position_frm_u_opt(u_opt_test[backup_idx*2:],N-backup_idx,dt_lst[backup_idx:]) #TODO
        (x_init, y_init, theta_init) = (X[1], Y[1], THETA[1])
        (v_init, w_init) = (uv[0], uw[0])
        time.sleep(1)#3-dt_lst[backup_idx-1])
        backup_idx+=1
        if print_cone:
            print_cone=False
            for i in range(len(uv)):
                for j in range(len(X_OBS)):
                    print_cone_const(X_OBS[j], Y_OBS[j],THETA_OBS[j], V_OBS[j], R_OBS[j], uv[backup_idx-2], X[i], Y[i], THETA[i])
            # break
    (x_impact,y_impact)=([],[])
    for j in range(2):
        v_tan=get_tang_v_ego(v_init,x_init,y_init,theta_init)
        t_impact,arc=get_intersection_time(x_init,y_init,v_tan,X_OBS[j],Y_OBS[j],W_OBS[j])
        print(arc)
        x_imp,y_imp,_,_=obs_move_line(Y_LANE_OBS[j],V_OBS[j],X_OBS[j], Y_OBS[j],THETA_OBS[j],t_impact)
        x_impact.append(x_imp)
        y_impact.append(y_imp)
    
    # if not close_to_target and not converged and not collision:
    #     theta_init += calculate_turn_dir(theta_init)
    #     print('TURNING')
    total_sec += solution['solve_time_ms']/1000
    ani = animation.FuncAnimation(fig, animate, interval=100000)
    plt.pause(0.0001)
print('total_sec', total_sec)
print('avrage_curve_fit_error',round(sum(avrage_curve_fit_error)/len(avrage_curve_fit_error),1),round(min(avrage_curve_fit_error),1),round(max(avrage_curve_fit_error),1))
print('avrage time_ms',total_sec*1000/i)
plt.show()
mng.kill()
