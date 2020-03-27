import matplotlib.pyplot as plt
from linerize import lin_model
from setup import getInitVals
N=10
Nsim=20
dt=0.5
ur=[0,0] 
xr=[10,5,0]
x0=[0,5,0]
u0=[0,0]
dv=0.5
dw=10

obstacles=[[4,5,0]] 
obstacles_u=[[0.1,0]]
r_obs=0.5


close_obstacles=obstacles
close_obstacles_u=obstacles_u
feasible=True
x,u,Q,R=getInitVals(x0,u0,xr,ur,Nsim,N)


for k in range(Nsim+1):
    if feasible:
        A,B =lin_model(x[k],u[k],dt)

        x[k+1] =A*x[k] + B*u[k]
        print(x)
    # else

    #    textbox=annotation('textbox', [0.7, 0.1, 0.1, 0.1], 'String', "Wait");
    #     u(k,:)=u(k-1,:)/2;
    #     x(k+1,:) =A*x(k,:)' + B*u(k,:)';
    

# circle1 = plt.Circle((10, 5), 0.4, color='r')


# fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
# # (or if you have an existing figure)
# # fig = plt.gcf()
# # ax = fig.gca()

# ax.set_xlim((0, 10))
# ax.set_ylim((0, 10))

# ax.add_artist(circle1)
# plt.show()