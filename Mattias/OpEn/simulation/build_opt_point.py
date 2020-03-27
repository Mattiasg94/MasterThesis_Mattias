from setup import *
import opengen as og
import casadi.casadi as cs
import numpy as np


def lane_keeping(lane_cost, ymin, ymax, y,actv_lane,yref,ACTIVATE_CONE):
    passed_ymin=cs.if_else(y<ymin,1,-1)
    passed_ymax=cs.if_else(y>ymax,1,-1)
    lane_cost += cs.fmax(0.0, passed_ymin*10000000)
    lane_cost += cs.fmax(0.0, passed_ymax*10000000)
    lane_cost+=cs.fmax(0.0,-1000*cs.sqrt((ymin-y)**2)+(1000/2))
    activate_lane_keep=cs.logic_and(ACTIVATE_CONE[0],ACTIVATE_CONE[1])
    # lane_cost += cs.if_else(activate_lane_keep,(cs.sqrt((actv_lane-y)**2)**2)*Qy*0.5,0)
    lane_cost += cs.if_else(activate_lane_keep,(cs.sqrt((yref-y)**2)**2)*Qy*4,0)
    return lane_cost


def cone_constraint(c, x_obs, y_obs,theta_obs, v_obs, r_obs, uk, x, y, theta, obs_dist,ego_dist,xkm1,ykm1,xref,yref):
    v_obs_x=cs.cos(theta_obs)*v_obs
    v_obs_y=cs.sin(theta_obs)*v_obs
    rterm = (x-x_obs)**2+(y-y_obs)**2
    uterm = ((cs.cos(theta)*uk[0]-v_obs_x)*(x-x_obs) +
             (cs.sin(theta)*uk[0]-v_obs_y)*(y-y_obs))
    lterm = (cs.cos(theta)*uk[0]-v_obs_x)**2+(cs.sin(theta)*uk[0]-v_obs_y)**2
    # -------distance const
    # c += cs.fmax(0.0, r_obs - (x_obs-x)**2 - (y_obs-y)**2)
    # -------regular cone
    #c += cs.fmax(0.0, r_obs**2*lterm-(rterm*lterm-uterm**2))
    # -------cone only when dist ego > dist obs
    cone = r_obs**2*lterm-(rterm*lterm-uterm**2)
    passed_obs=cs.if_else((obs_dist-ego_dist)>0 ,True,False)
    obs_faster_than_ego=cs.if_else(uk[0]<v_obs ,True,False) 
    obs_driving_towards=cs.if_else(cs.norm_2(theta-theta_obs)>=(cs.pi/2),True,False)
    skip_due_todir_and_vel=cs.if_else(obs_driving_towards,False,cs.if_else(obs_faster_than_ego,True,False))
    deactivate_activate_cone=cs.if_else(passed_obs,True,cs.if_else(skip_due_todir_and_vel,True,False))
    c += cs.fmax(0.0, cs.if_else(deactivate_activate_cone,0,cs.fmax(0.0, cone)))
    # decide what side to drive
    # side_obs =cs.sign((x_obs-xkm1)*(yref-ykm1)-(y_obs-ykm1)*(xref-xkm1)) # neg if on left
    # s=cs.sign((x-xkm1)*(y_obs-ykm1)-(y-ykm1)*(x_obs-xkm1)) # neg if on left
    # #c +=cs.fmax(0.0,s*5)
    return c,deactivate_activate_cone


def build_opt():
    u = cs.SX.sym('u', nu*N)
    p = cs.SX.sym('p', nx+nref+nObs+nu_init+n_actv_lane)
    (x, y, theta) = (p[0], p[1], p[2])
    ukm1 = [p[3], p[4]]
    (xref, yref, thetaref) = (p[5], p[6], p[7])
    (X_OBS, Y_OBS, THETA_OBS, V_OBS, W_OBS, R_OBS) = ([p[8], p[9]], [p[10], p[11]], [p[12], p[13]],[p[14], p[15]],[p[16], p[17]],[p[18], p[19]])
    actv_lane = p[20]
    stage_cost = lane_cost = jerk_cost = 0
    c = 0
    ACTIVATE_CONE=[]
    for t in range(0, nu*N, nu):
        uk = u[t:t+2]
        stage_cost += Qx*(x-xref)**2 + Qy*(y-yref)**2 + \
            Qtheta*(theta-thetaref)**2
        stage_cost += Rv*uk[0]**2+Rw*uk[1]**2
        (xkm1,ykm1)=(x,y)
        (x, y, theta) = model_dd(x, y, theta, uk[0], uk[1])
        
        for j in range(len(X_OBS)):
            # obstacles
            (X_OBS[j], Y_OBS[j], THETA_OBS[j]) = model_dd(X_OBS[j], Y_OBS[j], THETA_OBS[j], V_OBS[j], W_OBS[j])
            ego_dist = cs.sqrt((x-xref)**2+(y-yref)**2)
            obs_dist = cs.sqrt((X_OBS[j]-xref)**2+(Y_OBS[j]-yref)**2)
            c ,activate_cone= cone_constraint(c, X_OBS[j], Y_OBS[j],THETA_OBS[j], V_OBS[j], R_OBS[j], uk, x, y, theta, obs_dist,ego_dist,xkm1,ykm1,xref,yref)
            ACTIVATE_CONE.append(activate_cone)
        lane_cost = lane_keeping(lane_cost, ymin, ymax, y,actv_lane,yref,ACTIVATE_CONE)
        # passed_obs_by3M = cs.if_else(obs_dist-(ego_dist+3)>0,1,-1)
        # dist_actv_lane = cs.sqrt((actv_lane-y)**2)
        # lane_cost += 10*cs.fmax(0.0, passed_obs_by3M*(dist_actv_lane**2)*Qy*4)
    terminal_cost = Qtx*(x-xref)**2 + Qty*(y-yref)**2 + Qttheta*(theta-thetaref)**2
    
    # -------Acceleration constraint
    (dv_c, dw_c) = (0, 0)
    for t in range(0, nu*N, nu):
        uk = u[t:t+2]
        dv_c += cs.fmax(0.0, 100*(uk[0]-ukm1[0]-dv))
        dw_c += cs.vertcat(cs.fmax(0.0, 100*(uk[1]-ukm1[1]-dw)),
                           cs.fmax(0.0, 100*(ukm1[1]-uk[1]-dw)))
        jerk_cost+= Rvj*(uk[0]-ukm1[0])**2+Rwj*(uk[1]-ukm1[1])**2
        ukm1 = uk
    total_cost = terminal_cost+stage_cost+lane_cost+jerk_cost
    # -------Boundery constrint
    umin = []
    umax = []
    for i in range(N):
        umin.extend([vmin, wmin])
        umax.extend([vmax, wmax])

    set_c = og.constraints.Zero()
    set_y = og.constraints.BallInf(None, 1e12)
    bounds = og.constraints.Rectangle(umin, umax)
    C = cs.vertcat(dv_c, dw_c)
    problem = og.builder.Problem(u, p, total_cost).with_penalty_constraints(
        C).with_constraints(bounds).with_aug_lagrangian_constraints(c, set_c, set_y)

    build_config = og.config.BuildConfiguration()\
        .with_build_directory("optimizers")\
        .with_build_mode("debug")\
        .with_rebuild(False)\
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("ref_point")
    update_pen = 2
    init_pen = 100
    num_out_pen = 4

    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-9)\
        .with_initial_tolerance(1e-5)\
        .with_max_outer_iterations(num_out_pen)\
        .with_delta_tolerance(1e-2)\
        .with_penalty_weight_update_factor(update_pen)\
        .with_initial_penalty(init_pen).with_sufficient_decrease_coefficient(0.7)\
        # .with_max_duration_micros(200*1000) 

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                              meta,
                                              build_config,
                                              solver_config) \
        .with_verbosity_level(1)

    builder.build()
    return update_pen, init_pen, num_out_pen


if __name__ == '__main__':
    update_pen, init_pen, num_out_pen = build_opt()
    h = init_pen
    for i in range(num_out_pen-1):
        h = h*update_pen
    print('[MAXIMUM PENALTY]', h)
    import run_opt
