import time
import casadi.casadi as cs
import numpy as np
import opengen as og
from scipy.optimize import curve_fit
# -------- Ego car init
ts = 2
(nu, nx, nref, nu_init, n_actv_lane, N) = (2, 3, 3, 2, 1, 15)
ts_lst=[ts]*N
ntraj = 3*N
nObs = (6*2)
(Qx, Qy, Qtheta) = (10, 10, 0)
(Rv, Rw) = (1, 1)
(Rvj, Rwj) = (5, 5)
(vmin, vmax) = (0.01, 0.5)
(wmin, wmax) = (-3, 3)
(dv, dw) = (0.05, 0.05)
(Qtx, Qty, Qttheta) = (100, 100, 100)
u_opt = [0.0] * (nu*N)
(V_OBS,W_OBS) = ([0,0],[0,0])
# ------- run_opt stuff
plot_history = [[], []]
plt_x = [-1, 41]
plt_y = [0, 10]
total_sec = 0
curve_fit_error_max = 40
overtaking_dist=10
last_time = time.time()
close_to_target = False
warm_start = False
converged = True
run_vel_obs = False
change_optimizer = False
#---- plot
linewidth = 1
lines = 3  # number of lines drawn
road_radius = 900
lane_offset_y = 2
lenght = 40
lane_offset_x = lenght/2
lane_border_min = road_radius
lane_border_max = road_radius+linewidth*lines
center_of_road = lane_border_min+linewidth  # TODO what if 4 lines?
lane1 = lane_border_min+(linewidth/2)
lane2 = lane1+linewidth
# ----- Methods

def check_converged(X,Y,avrage_curve_fit_error,x_init,y_init,xref,yref):
    xdata=np.array(X)
    ydata=np.array(Y)
    popt, pcov = curve_fit(curve_fit_func, xdata, ydata)
    curve_fit_error=sum(np.sqrt(np.diag(pcov)))
    if curve_fit_error>curve_fit_error_max:
        converged=False
        if curve_fit_error!=float("inf"):
            print('Not Converged',int(curve_fit_error))
    else:
        if np.sqrt(np.power(x_init-xref,2)+np.power(y_init-yref,2))>10:
            avrage_curve_fit_error.append(int(curve_fit_error))
        converged=True
    return converged,avrage_curve_fit_error,curve_fit_error,popt

def switch_optimizer(mng, optimizer):
    print('[Switch optimizer to vel]')
    mng.kill()
    mng = og.tcp.OptimizerTcpManager('optimizers/'+optimizer)
    mng.start()
    print('[start','optimizers/'+optimizer+']')
    return mng, False


def curve_fit_func(x, a, b, c, d, e):
    return a*x**4+b*x**3+c*x**2+d*x+e
# def curve_fit_func(x, a, b, c,d):
#     return a*x**3+b*x**2+c*x+d


def model_dd(x, y, theta, v, w):
    x += ts*cs.cos(theta)*v
    y += ts*cs.sin(theta)*v
    theta += ts*w
    return x, y, theta

def obs_move_line(lane, v, x, y,theta,dt): #TODO update theta at init
    v = -v
    radius = road_radius_frm_lane(lane)
    x_displaced = x-lane_offset_x
    y_displaced = y+(lane_border_min-lane_offset_y)
    curr_th = np.arccos(
        x_displaced/(np.linalg.norm([x_displaced, y_displaced])))
    d = dt*v
    th_new = curr_th+d/radius
    x_displaced = radius*np.cos(th_new)
    y_displaced = radius*np.sin(th_new)
    x = x_displaced+lane_offset_x
    y = y_displaced-(lane_border_min-lane_offset_y)
    theta=th_new
    w=(th_new-curr_th)/dt #TODO which is the right way?
    return x, y,theta,w


def get_curvature_plots():
    global linewidth, lines, lane_offset_y, lenght
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
    return plot_x_curv, plot_y_curv


plot_x_curv, plot_y_curv = get_curvature_plots()


def road_radius_frm_lane(lane):
    global road_radius, linewidth
    radius = (road_radius-linewidth/2)+lane*linewidth
    return radius


def get_y_from_lane(lane, x):
    global plot_x_curv, plot_y_curv
    x_val_in_list = min(plot_x_curv[lane-1], key=lambda i: abs(i-x))
    idx = plot_x_curv[lane-1].index(x_val_in_list)
    y = plot_y_curv[lane-1][idx]+linewidth/2
    return y
