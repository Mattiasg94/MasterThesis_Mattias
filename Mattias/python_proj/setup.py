# function [u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta]=setup(x0,u0,xr,ur,ub,lb,Nsim,N)
import numpy as np


def getInitVals(x0,u0,xr,ur,Nsim,N):
    x=np.array(x0)
    u=np.array(u0)
    Q=np.array([[40,0,0],[0,40,0],[0,0,40]])
    R=np.array([[1,0],[0,1]])
    return x,u,Q,R
# x0=[1,2,0]
# u0=[2,3]
# xr=[10,5,0]
# ur=[0,0]
# Nsim=2
# N=2
# getInitVals(x0,u0,xr,ur,Nsim,N)


# R=[1 0
#     0 1];
# MQ=[];
# MR=[];
# for i=1:N
#    MQ=blkdiag(MQ,Q); 
#    MR=blkdiag(MR,R);    
# end
# Mxr=[];
# Mur=[];
# for i=1:N
#     Mxr=[Mxr;xr];
#     Mur=[Mur;ur];
# end
# u1_delta=[1,0,-1,0];
# u2_delta=[0,1,0,-1];
# Mu1_delta=zeros(N-1,2*N);
# Mu2_delta=zeros(N-1,2*N);
# for i=1:N-1
#    Mu1_delta(i,2*(i-1)+1:2*(i-1)+4)=u1_delta;
#    Mu2_delta(i,2*(i-1)+1:2*(i-1)+4)=u2_delta;
# end