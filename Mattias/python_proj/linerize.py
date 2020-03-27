import numpy as np
import math

def lin_model(x,u,dt):
    # A=np.array([[1,0,-dt*u[0]*math.sin(x[2])]
    #           ,[0,1,dt*u[0]*math.cos(x[2])]
    #           ,[0,0,1]])
    A=np.array([[1,0,0]
            ,[0,1,0]
            ,[0,0,1]])
    B=np.array([[dt*math.cos(x[2]),0]
            ,[dt*math.sin(x[2]),0],
                [0,dt]])
    return A,B