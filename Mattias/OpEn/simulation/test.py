import matplotlib.pyplot as plt
from math import pi,cos,sin,asin

lanes=3
x_plot=[]
y_plot=[]
plot_x={}
plot_y={}

r=1
l=1
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
th_max=pi
th_min=0
for lane in range(lanes):
    x=[]
    y=[]
    ax.cla()
    th=pi
    r+=1
    d_th=asin(l/(2*r))
    while th_min<=th<=th_max:
        x.append(r*cos(th))
        y.append(r*sin(th))
        th-=d_th
    plot_x[lane]=x
    plot_y[lane]=y
# if lane==0 or lane==(lanes-1):
#     print(lane)
#     ax.plot(x,y , '--', linewidth=0.2,color='b')
# else:

ax.plot(plot_x[0],plot_y[0], '-', color='k', linewidth=0.5)
ax.plot(plot_x[1],plot_y[1], '--', color='b', linewidth=1)
ax.plot(plot_x[2],plot_y[2], '-', color='k', linewidth=0.5)
axis=5
plt.xlim(-axis,axis)
plt.ylim(-axis,axis)
plt.show()