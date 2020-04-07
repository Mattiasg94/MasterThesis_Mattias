from setup import *
import sys
static_3=False
overtaking_right=True
infeasible_case=False
sudden_obs=False


if sum([static_3,overtaking_right,infeasible_case,sudden_obs])>1:
    print('[More than 1 scenario active]')
    sys.exit()

# -------Init ego
(x_init,init_lane, theta_init,r_ego) = (0,1, 0,0.26)
(xref_final,ref_init_lane, thetaref) = (lenght, 1, 0)
yref_final=get_y_from_lane(ref_init_lane,xref_final)
(v_init, w_init) = (vmax, 0)

if static_3:
    penalty_margin=0.3
    (X_OBS,Y_LANE_OBS, THETA_OBS, R_OBS) = ([13,20,27],[1,2,1] ,[0,0,0], [0.26,0.26,0.26])
    Y_OBS=[get_y_from_lane(Y_LANE_OBS[0],X_OBS[0]),get_y_from_lane(Y_LANE_OBS[1],X_OBS[1]),get_y_from_lane(Y_LANE_OBS[2],X_OBS[2])]
    V_OBS=[0,0,0]
elif overtaking_right:
    penalty_margin=0.3
    (X_OBS,Y_LANE_OBS, THETA_OBS, R_OBS) = ([13,30],[1,2] ,[0,0], [0.26,0.26])
    Y_OBS=[get_y_from_lane(Y_LANE_OBS[0],X_OBS[0]),get_y_from_lane(Y_LANE_OBS[1],X_OBS[1])]
    V_OBS=[0.1,-0.2]
elif infeasible_case:
    penalty_margin=0.3
    (X_OBS,Y_LANE_OBS, THETA_OBS, R_OBS) = ([20,20],[1,2] ,[0,0], [0.26,0.26])
    Y_OBS=[get_y_from_lane(Y_LANE_OBS[0],X_OBS[0]),get_y_from_lane(Y_LANE_OBS[1],X_OBS[1])]
    V_OBS=[0,0]
elif sudden_obs:
    penalty_margin=0.3
    (X_OBS,Y_LANE_OBS, THETA_OBS, R_OBS) = ([-16,-100],[1,2] ,[0,0], [0.26,0.26])
    Y_OBS=[get_y_from_lane(Y_LANE_OBS[0],X_OBS[0]),get_y_from_lane(Y_LANE_OBS[1],X_OBS[1])]
    V_OBS=[0,0]

def get_angle_from_lane(lane,x,y):
    radius = road_radius_frm_lane(lane)
    x_displaced = x-center[0]
    y_displaced = y-center[1]
    curr_th = np.arccos(x_displaced/(np.linalg.norm([x_displaced, y_displaced])))
    theta=curr_th-np.pi/2
    return theta