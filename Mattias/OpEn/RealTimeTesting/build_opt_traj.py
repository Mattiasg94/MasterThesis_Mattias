from setup import *
import opengen as og
import casadi.casadi as cs
import numpy as np


ntraj=3*len_traj

def build_opt():
    u = cs.SX.sym('u', nu*N)
    p = cs.SX.sym('p', nx+nref+nObs+nu_init+ntraj)
    (x, y, theta) = (p[0], p[1], p[2])
    ukm1=[p[3],p[4]]
    (xref, yref, thetaref) = (p[5], p[6], p[7])
    (x_obs, y_obs, theta_obs, v_obs, w_obs, r_obs) = (
        p[8], p[9], p[10], p[11], p[12], p[13])
    
    for i in range(14,14+len_traj):
        xtraj.append(p[i])
        ytraj.append(p[i+len_traj])
        thetatraj.append(p[i+2*len_traj])
    cost = 0
    c = 0
    # -------Collission constrint
    k=0
    for j,t in enumerate(range(0, nu*N, nu)):
        uk = u[t:t+2]
        # -------Reference trajectory
        cost += Qx*(x-xtraj[j])**2 + Qy*(y-ytraj[j])**2 + Qtheta*(theta-thetatraj[j])**2
        # if j%NperTraj==0 and not t==0:
        #     k+=1
        # -------Reference Point
        # cost += Qx*(x-xref)**2 + Qy*(y-yref)**2 + Qtheta*(theta-thetaref)**2
        # -------
        cost += Rv*uk[0]**2+Rw*uk[1]**2
        (x, y, theta) = model_dd(x, y, theta, uk[0], uk[1])
        (x_obs, y_obs, theta_obs) = model_dd(x_obs, y_obs, theta_obs, v_obs, w_obs)
        rterm = (x-x_obs)**2+(y-y_obs)**2
        uterm = ((cs.cos(theta)*uk[0]-v_obs)*(x-x_obs) +  (cs.sin(theta)*uk[0]-v_obs)*(y-y_obs))**2
        lterm = (cs.cos(theta)*uk[0]-v_obs)**2+(cs.sin(theta)*uk[0]-v_obs)**2
        # -------distance const
        # c += cs.fmax(0.0, r_obs - (x_obs-x)**2 - (y_obs-y)**2)
        # -------regular cone
        #c += cs.fmax(0.0, r_obs**2*lterm-(rterm*lterm-uterm**2))
        # -------cone only when dist ego > dist obs
        cone = r_obs**2*lterm-(rterm*lterm-uterm)
        ego_dist = cs.sqrt((x-xref)**2+(y-yref)**2)
        obs_dist = cs.sqrt((x_obs-xref)**2+(y_obs-yref)**2)
        c += cs.fmax(0.0, -(obs_dist-ego_dist)) * (cs.fmax(0.0, cone))
    #cost += Qtx*(x-xtraj[-1])**2 + Qty*(y-ytraj[-1])**2 + Qttheta*(theta-thetaref)**2
    #cost += Qtx*(x-xref)**2 + Qty*(y-yref)**2 + Qttheta*(theta-thetaref)**2
    # -------Acceleration constraint
    (dv_c, dw_c) = (0, 0)
    for t in range(0, nu*N, nu):
        uk = u[t:t+2]
        dv_c += cs.fmax(0.0, uk[0]-ukm1[0]-dv)
        dw_c += cs.vertcat(cs.fmax(0.0, uk[1]-ukm1[1]-dw),
                           cs.fmax(0.0, ukm1[1]-uk[1]-dw))
        ukm1 = uk
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
    problem = og.builder.Problem(u, p, cost).with_penalty_constraints(
        C).with_constraints(bounds).with_aug_lagrangian_constraints(c,set_c,set_y)

    build_config = og.config.BuildConfiguration()\
        .with_build_directory("optimizers")\
        .with_build_mode("debug")\
            .with_rebuild(False)\
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta()\
        .with_optimizer_name("ref_traj")
    
    solver_config = og.config.SolverConfiguration()\
        .with_tolerance(1e-5)\
        .with_initial_tolerance(1e-5)\
        .with_max_outer_iterations(10)\
        .with_delta_tolerance(1e-2)\
        .with_penalty_weight_update_factor(5).with_initial_penalty(100).with_sufficient_decrease_coefficient(0.7)

    builder = og.builder.OpEnOptimizerBuilder(problem,
                                              meta,
                                              build_config,
                                              solver_config) \
        .with_verbosity_level(1)

    builder.build()


if __name__ == '__main__':
    build_opt()
    import run_opt
