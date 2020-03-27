clc
clear all
close all

A=[1 0
   0 1];
B=[1 0
   0 1];
C=[1 0
   0 1];
sys=ss(A,B,C,0);
Ts=1;
sysd=c2d(sys,Ts);


N=5;
Nsim=20;
num_states=4;
u = zeros(Nsim , 2);
x = zeros(Nsim + 1, 2);
x0=[5;0];
x(1,:) = x0;
lb=[3.5 0 -0.5 -0.5 ];
ub=[5.5 10 0.5 0.5];
xf=[5;10];

lb1=[];lb2=[];
ub1=[];ub2=[];
for i=1:N
    lb1=[lb1,lb(1:2)];
    lb2=[lb2,lb(3:4)];
    ub1=[ub1,ub(1:2)];
    ub2=[ub2,ub(3:4)];
end
lb=[lb1,lb2];
ub=[ub1,ub2];

Z0=0;

%% plot
obstacles=[[5,4],[4,6]];

xcont = linspace(x0(1)-5,xf(1)+5);
ycont = linspace(x0(2),xf(2)+5);
[X,Y] = meshgrid(xcont,ycont);
fun = (X-xf(1)).^2 + (Y-xf(2)).^2;
contour(X,Y,fun,100)
hold on 
for i=1:length(obstacles)/2
    scatter(obstacles(2*i-1),obstacles(2*i),'k','LineWidth',1.5)
end
grid on,set(gca,'ytick',min(xcont):max(xcont)),set(gca,'xtick',min(ycont):max(ycont))
xline(3.5,'--r');
xline(4.5,'--r');
xline(5.5,'--r');

i=1;
inc_slope=0.1;
for k = 2:Nsim+1
    
    [Z,exitflag] = optimizer_fmincon(Z0,A, B, N, xf, x(k-1,:)',lb,ub,obstacles);
    obstacles(2)=obstacles(2)+0.05;
    obstacles(4)=obstacles(4)-0.05;
    for i=1:length(obstacles)/2
    scatter(obstacles(2*i-1),obstacles(2*i),'k','LineWidth',1.5)
    end
    Z0=Z;
    u(k-1,:)=Z(N*2+1:N*2+2);
    x(k,:) = A*x(k-1,:)' + B*u(k-1,:)';
    i=i+1;
    if exitflag==-2
        disp("----unfeasible!----")
       break
    end
    plot(x(:,1),x(:,2),'ok')
    pause(0.1)
end
disp("done")


    
