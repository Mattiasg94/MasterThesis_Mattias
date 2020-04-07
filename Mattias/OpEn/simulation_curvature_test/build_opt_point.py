from setup import *
import opengen as og
import casadi.casadi as cs
import numpy as np


def lane_keeping(lane_cost, y,x,actv_lane,yref,ACTIVATE_CONE):
    global lane_border_min,lane_border_max,lane_offset_y,lane_offset_x,center_of_road,lane1,lane2
    # border constraint
    radius_ego=cs.sqrt((x-lane_offset_x)**2+(y+(lane_border_min-lane_offset_y))**2)
    # Googla x^3 så ser du att den börjar öka vid ca 5-10.
    # multiplicera dist_ymin med 10 så börjar den vid 0.5-1 ist
    dist_ymin=(lane1-radius_ego)*12 
    dist_ymax=(radius_ego-lane2)*12  
    lane_cost+=cs.fmax(0.1,dist_ymin**3+10*dist_ymin)
    lane_cost+=cs.fmax(0.1,dist_ymax**3+10*dist_ymax)
    # active lane keeping constraint
    activate_lane_keep=cs.logic_and(ACTIVATE_CONE[0],ACTIVATE_CONE[1])
    lane_cost += cs.if_else(activate_lane_keep,500*(radius_ego-actv_lane)**2,0*(radius_ego-center_of_road)**2) #TODO
    return lane_cost


def cone_constraint(c, x_obs, y_obs, r_cone, uk, x, y, theta, passed_obs):
    v_obs_x=0
    v_obs_y=0
    r_vec=cs.vertcat(x-x_obs,y-y_obs)
    vab=cs.vertcat(cs.cos(theta)*uk[0]-v_obs_x,cs.sin(theta)*uk[0]-v_obs_y)
    rterm = (cs.norm_2(r_vec))**2
    lterm = (cs.norm_2(vab))**2
    uterm = (cs.dot(r_vec,vab))**2
    cone = (r_cone**2)*lterm-(rterm*lterm-uterm)
    skip_due_far_away=cs.if_else(cs.sqrt((x_obs-x)**2 - (y_obs-y)**2)<10,False,True)
    deactivate_cone=cs.if_else(passed_obs,True,skip_due_far_away)
    c += cs.if_else(deactivate_cone,0,cs.fmax(0.0, cone))

    return c,False


def build_opt():
    u = cs.SX.sym('u', nu*N)
    p = cs.SX.sym('p', nx+nref+nObs+nu_init+n_actv_lane)
    (x, y, theta) = (p[0], p[1], p[2])
    ukm1 = [p[3], p[4]]
    (xref, yref, thetaref) = (p[5], p[6], p[7])
    (X_OBS, Y_OBS, THETA_OBS, V_OBS, Y_LANE_OBS, R_CONE) = ([p[8], p[9]], [p[10], p[11]], [p[12], p[13]],[p[14], p[15]],[p[16], p[17]],[p[18], p[19]])
    actv_lane = p[20]
    c = stage_cost = lane_cost = jerk_cost = 0
    # close_to_obs=cs.if_else(cs.sqrt((X_OBS[0]-x)**2 - (Y_OBS[0]-y)**2)<10,True,False)
    # close_obs_cost += cs.if_else(close_to_obs,cs.fmax(0.0, 1000*(R_OBS[0] - cs.sqrt((X_OBS[0]-x_t)**2 + (Y_OBS[0]-y_t)**2))**2),0)
    # radius_ego=cs.sqrt((x_t-lane_offset_x)**2+(y_t+(lane_border_min-lane_offset_y))**2)
    # close_obs_cost += cs.if_else(close_to_obs,1000*(radius_ego-actv_lane)**2,0)

    for t in range(0, nu*N, nu):
        uk = u[t:t+2]
        stage_cost += Qx*(x-xref)**2 + Qy*(y-yref)**2 +  Qtheta*(theta-thetaref)**2
        stage_cost += Rv*uk[0]**2+Rw*uk[1]**2
        (xkm1,ykm1)=(x,y)
        (x, y, theta) = model_dd(x, y, theta, uk[0], uk[1])
        CONE_DEACTIVATED=[]
        W_OBS=[0,0] # TODO
        for j in range(len(X_OBS)):
            # obstacles handeling
            ego_dist = cs.sqrt((x-xref)**2+(y-yref)**2)
            obs_dist = cs.sqrt((X_OBS[j]-xref)**2+(Y_OBS[j]-yref)**2)
            passed_obs=cs.if_else(obs_dist>ego_dist ,True,False)
            v_tan=get_tang_v_ego(uk[0],x,y,theta)
            t_impact,arc=get_intersection_time(x,x,v_tan,X_OBS[j],Y_OBS[j],V_OBS[j])
            x_impact,y_impact,THETA_OBS[j],W_OBS[j]=obs_move_line(Y_LANE_OBS[j],V_OBS[j],X_OBS[j], Y_OBS[j],THETA_OBS[j],t_impact)


            c ,cone_deactivated= cone_constraint(c, x_impact, y_impact, R_CONE[j], uk, x, y, theta, passed_obs)
            CONE_DEACTIVATED.append(cone_deactivated)
        lane_cost = lane_keeping(lane_cost, y,x,actv_lane,yref,CONE_DEACTIVATED)
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
    lag_const=cs.vertcat(dv_c, dw_c,c)
    problem = og.builder.Problem(u, p, total_cost).with_constraints(bounds).with_aug_lagrangian_constraints(lag_const, set_c, set_y)
#release
    build_config = og.config.BuildConfiguration()\
        .with_build_directory("optimizers")\
        .with_build_mode("debug")\
        .with_rebuild(False)\
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("ref_point")
    update_pen = 2
    init_pen = 100
    num_out_pen = 6

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
#obs_driving_towards=cs.if_else(cs.norm_2(theta-theta_obs)>=(cs.pi/2),True,False)