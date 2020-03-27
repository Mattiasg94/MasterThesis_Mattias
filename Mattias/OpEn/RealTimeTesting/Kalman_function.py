import numpy as np
np.set_printoptions(threshold=3)
np.set_printoptions(suppress=True)
import random as rnd
from numpy import genfromtxt
import time 
import matplotlib.pyplot as plt
from scipy.stats import norm
 

def JacobianA(x,u,dt):
    a1 = -u[0]*np.sin(x[2])*dt
    a2 = u[0]*np.cos(x[2])*dt

    JA = np.matrix([[ 1, 0, a1],
                    [ 0, 1,  a2],
                    [ 0, 0,    1]], dtype='float')
    return JA

def JacobianH(x,dt):

    JH = np.diag([1.0, 1.0, 1.0])

    return JH

def prediction(X_hat_t_1,u,P_t_1,Q_t,dt):
    X_hat_t = np.matrix([[0,0,0]],dtype='float').T 
    X_hat_t[0] = X_hat_t_1[0] #+ u[0]*np.cos(X_hat_t_1[2])*dt
    X_hat_t[1] = X_hat_t_1[1] # + u[0]*np.sin(X_hat_t_1[2])*dt
    X_hat_t[2] = X_hat_t_1[2] #+ u[1]*dt  
    
    # Project the error covariance ahead
    P_t_1 = JacobianA(X_hat_t_1,u,dt)*P_t_1*JacobianA(X_hat_t_1,u,dt).T + Q_t
    # print(P_t_1)
    return X_hat_t, P_t_1
 

def update(X_hat_t,u,P_t,Z_t,R_t,dt):
    hx = np.array([[float(X_hat_t[0])],
                    [float(X_hat_t[1])],
                    [float(X_hat_t[2])]])   

    S = np.matrix(JacobianH(X_hat_t,dt)*P_t*JacobianH(X_hat_t,dt).T + R_t, dtype='float')

    K = (P_t*JacobianH(X_hat_t,dt)) * np.linalg.inv(S)

    Z = Z_t.reshape(JacobianH(X_hat_t,dt).shape[0],1)
    # print("Z:\n",Z)
    y = Z - (hx)                         # Innovation or Residual
    X_t = X_hat_t + (K*y)
 
    # Update the error covariance
    I = np.eye(X_hat_t.shape[0])
    P_t = (I - (K*JacobianH(X_hat_t,dt)))*P_t

    return X_t,P_t
m = []
def KalmanFilter(P_t,R_t,Q_t,X_hat_t,uv,uw,newMeasurement,dt):  
    X_hat_t,P_hat_t = prediction(X_hat_t,np.vstack((uv,uw)),P_t,Q_t,dt)
    # Z_t = np.vstack((newMeasurement[0],newMeasurement[1],newMeasurement[2]))
    Z_t=newMeasurement
    X_t,P_t = update(X_hat_t,np.vstack((uv,uw)),P_t,Z_t,R_t,dt)
        
    x0 = (float(X_t[0]))
    x1 = (float(X_t[1]))
    x2 = (float(X_t[2]))
    # print(P_t)
    return x0, x1, x2, P_t 
