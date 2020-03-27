import matplotlib.pyplot as plt
from gurobi_opt import optimizer
import time
import numpy as np
from linerize import lin_model
def plot():
    circle1 = plt.Circle((10, 5), 0.4, color='g')
    circle2 = plt.Circle((obs[0],obs[1]), r, color='r')
    fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
    # (or if you have an existing figure)
    # fig = plt.gcf()

    # ax = fig.gca()
    ax.set_xlim((0, 10))
    ax.set_ylim((0, 10))
    ax.add_artist(circle1)
    ax.add_artist(circle2)
    plt.plot(X,Y,'bo')
    plt.show()
X=[]
Y=[]
# init

qk=[0,5,0.2]
N=7
time_limit=20
du=[0.5,0.1]
obs=[5,5]
r=0.5
Method=-1
k=0
use_first_input=True
start=time.time()
while qk[0]-9<0 and qk[1]-9<0:
    x,y,th,v,w=optimizer(qk[0],qk[1],qk[2],obs,r,N,du,time_limit,Method)
    if x==11:
        break
    if use_first_input:
        A,B=lin_model(qk,[v,w],0.5)
        qk1=np.dot(A,qk)+np.dot(B,[v,w])
        qk=[qk1.item(0),qk1.item(1),qk1.item(2)]
        X.append(qk[0])
        Y.append(qk[1])
    else:
        X.extend(x)
        Y.extend(y)
        qk=[x[-1],y[-1],th[-1]]
    k+=1
    

print('Number of iterations=',k)
print('Total time in sek',time.time()-start)
plot()

