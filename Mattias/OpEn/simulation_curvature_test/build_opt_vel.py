from setup import *
import opengen as og
import casadi.casadi as cs
import numpy as np

def cone_constraint(c,x_obs, y_obs,theta_obs, v_obs, r_cone, v, x, y, theta,passed_obs):
    v_obs_x=cs.cos(theta_obs)*v_obs
    v_obs_y=cs.sin(theta_obs)*v_obs
    r_vec=cs.vertcat(x-x_obs,y-y_obs)
    vab=cs.vertcat(cs.cos(theta)*v-v_obs_x,cs.sin(theta)*v-v_obs_y)
    rterm = (cs.norm_2(r_vec))**2
    lterm = (cs.norm_2(vab))**2
    uterm = (cs.dot(r_vec,vab))**2
    cone = (r_cone**2)*lterm-(rterm*lterm-uterm)
    # c += cs.if_else(passed_obs,0,cs.fmax(0.0, cone))
    c+=cs.fmax(0.0, cone)
    return c
    
(x_traj,y_traj,theta_traj,T)=([],[],[],[])
def build_opt():
    v = cs.SX.sym('v', N)
    p = cs.SX.sym('p',nObs+2+ntraj+3) #TODO 
    
    (X_OBS, Y_OBS, THETA_OBS, V_OBS,W_OBS, Y_LANE_OBS, R_CONE) = ([p[1], p[2]], [p[3], p[4]], [p[5], p[6]],[p[7], p[8]],[p[9], p[10]],[p[11], p[12]],[p[13], p[14]])
    (xref, yref, thetaref) = (p[15], p[16], p[17])
    c = stage_cost  = 0
    v_cost=0
    for i in range(N):
        x_traj.append(p[17+i])
        y_traj.append(p[17+N+i])
        theta_traj.append(p[17+2*N+i])
    
    for i in range(0,N):
        vk = v[i]
        stage_cost += 1e4*((vmax-vk)*5)**2
        for j in range(2):
            ego_dist = cs.sqrt((x_traj[i]-xref)**2+(y_traj[i]-yref)**2)
            obs_dist = cs.sqrt((X_OBS[j]-xref)**2+(Y_OBS[j]-yref)**2)
            passed_obs=cs.if_else(obs_dist>ego_dist ,True,False)
            (X_OBS[j], Y_OBS[j],THETA_OBS[j],W_OBS[j])=obs_move_line(Y_LANE_OBS[j],V_OBS[j],X_OBS[j], Y_OBS[j],THETA_OBS[j],ts) #TOD does cone know about w_obs?
            c = cone_constraint(c,X_OBS[j], Y_OBS[j],THETA_OBS[j], V_OBS[j], R_CONE[j], vk, x_traj[i], y_traj[i], theta_traj[i],passed_obs)

    total_cost = stage_cost
    vmin_lst = []
    vmax_lst = []
    for i in range(N):
        vmin_lst.append(vmin)
        vmax_lst.append(vmax)
    lag_const=cs.vertcat(c)
    set_c = og.constraints.Zero()
    set_y = og.constraints.BallInf(None, 1e12)
    bounds = og.constraints.Rectangle(vmin_lst,vmax_lst)
    problem = og.builder.Problem(v, p, total_cost).with_constraints(bounds).with_aug_lagrangian_constraints(lag_const, set_c, set_y)

    build_config = og.config.BuildConfiguration()\
        .with_build_directory("optimizers")\
        .with_build_mode("debug")\
        .with_rebuild(False)\
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("vel_obs")
    update_pen = 2
    init_pen = 100
    num_out_pen = 6

    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-3)\
        .with_initial_tolerance(1e-3)\
        .with_max_outer_iterations(num_out_pen)\
        .with_delta_tolerance(1e-2)\
        .with_penalty_weight_update_factor(update_pen)\
        .with_initial_penalty(init_pen)#.with_sufficient_decrease_coefficient(0.7)\
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