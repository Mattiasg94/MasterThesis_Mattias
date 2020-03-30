import time
import casadi.casadi as cs
import numpy as np
# -------- Ego car init
ts = 2
(nu, nx, nref, nu_init, n_actv_lane, N) = (2, 3, 3, 2, 1, 12)
nObs = (6*2)
(Qx, Qy, Qtheta) = (10, 10, 0)
(Rv, Rw) = (1, 1)
(Rvj, Rwj) = (5, 5)
(vmin, vmax) = (0, 0.5)
(wmin, wmax) = (-3, 3)
(dv, dw) = (0.05, 0.05)
(Qtx, Qty, Qttheta) = (100, 100, 100)
# ------- run_opt stuff
plot_history = [[], []]
plt_x = [-1, 41]
plt_y = [0, 10]
total_sec = 0
just_changed_ref = 0
justChanged = False
close_to_target = False
last_time = time.time()
warm_start = False
#---- plot
linewidth = 1
lines = 3  # number of lines drawn
road_radius = 1000
lane_offset_y = 2
lenght = 50
lane_offset_x = lenght/2
lane_border_min = road_radius
lane_border_max = road_radius+linewidth*lines
center_of_road=lane_border_min+linewidth # TODO what if 4 lines?
lane1 = lane_border_min+(linewidth/2)
lane2 = lane1+linewidth
# ----- Methods


def model_dd(x, y, theta, v, w):
    x += ts*cs.cos(theta)*v
    y += ts*cs.sin(theta)*v
    theta += ts*w
    return x, y, theta


def obs_move_line(lane, v, x, y):
    global ts
    v=-v
    radius=road_radius_frm_lane(lane)
    x_displaced = x-lane_offset_x
    y_displaced = y+(lane_border_min-lane_offset_y)
    curr_th = np.arccos(
        x_displaced/(np.linalg.norm([x_displaced, y_displaced])))
    d = ts*v
    th_new = curr_th+d/radius
    x_displaced = radius*np.cos(th_new)
    y_displaced = radius*np.sin(th_new)
    x = x_displaced+lane_offset_x
    y = y_displaced-(lane_border_min-lane_offset_y)
    return x, y


def get_curvature_plots():
    global linewidth, lines, lane_offset_y, lenght, theta_span
    plot_x_curv = {}
    plot_y_curv = {}
    th_lst = []
    r = road_radius
    l = 1
    th_max = np.pi*3/4
    th_min = np.pi/4
    for lane in range(lines):
        X = []
        Y = []
        d_th = np.arcsin(l/(2*r))
        th = th_max
        while th_min <= th <= th_max:
            x = r*np.cos(th)+lenght/2
            y = r*np.sin(th)-(road_radius-lane_offset_y)
            if 0 <= x <= lenght:
                th_lst.append(th)
                X.append(x)
                Y.append(y)
            th -= d_th
        plot_x_curv[lane] = X
        plot_y_curv[lane] = Y
        r += linewidth
    theta_span = abs(th_lst[0]-th_lst[-1])
    return plot_x_curv, plot_y_curv


plot_x_curv, plot_y_curv = get_curvature_plots()


def road_radius_frm_lane(lane):
    global road_radius,linewidth
    radius=(road_radius-linewidth/2)+lane*linewidth
    return radius


def get_y_from_lane(lane, x):
    global plot_x_curv, plot_y_curv
    x_val_in_list = min(plot_x_curv[lane-1], key=lambda i: abs(i-x))
    idx = plot_x_curv[lane-1].index(x_val_in_list)
    y = plot_y_curv[lane-1][idx]+linewidth/2
    return y
