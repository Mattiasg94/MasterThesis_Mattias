clc
clear all, close all

N=1;
Nsim=20;
dt=0.5;
ur=[0;0]; 
xr=[10;5;0];
x0=[0;4;0];
u0=[0;0];
lb_x=[0 0 -2*pi];
ub_x=[10 10 2*pi];
lb_u=[0 -1];
ub_u=[1 1];
lb=[-inf -inf -inf lb_x -inf -inf lb_u];
ub=[ inf  inf  inf ub_x  inf  inf ub_u];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu_delta]=setup(x0,xr,ur,ub,lb,Nsim,N);
%% plot
obstacles=0;%[[5,4],[4,6]];
xcont = linspace(0,xr(1)+5);
ycont = linspace(0-5,xr(2)+5);
[X,Y] = meshgrid(xcont,ycont);
fun = (X-xr(1)).^2 + (Y-xr(2)).^2;
contour(X,Y,fun,100)
hold on 
for i=1:length(obstacles)/2
    scatter(obstacles(2*i-1),obstacles(2*i),'k','LineWidth',1.5)
end
grid on,set(gca,'ytick',min(xcont):max(xcont)),set(gca,'xtick',min(ycont):max(ycont))
% xline(3.5,'--r');
% xline(4.5,'--r');
% xline(5.5,'--r');

uk=u0;
for k = 2:Nsim+1 
    u_tilde(k-1,:)=(uk-ur);
    x_tilde(k-1,:)=(x(k-1,:)'-xr);
    [A,B] = Linearized_discrete_DD_model(xr,ur,dt); 
    [Z,fval,exitflag] = optimizer_fmincon(Z0,A,B,MQ,MR,Mxr,Mur,Mu_delta,N,x_tilde(k-1,:)',u_tilde(k-1,:)',x(k-1,:)',lb,ub,obstacles);
%     obstacles(2)=obstacles(2)+0.05;
%     obstacles(4)=obstacles(4)-0.05;
    for i=1:length(obstacles)/2
    scatter(obstacles(2*i-1),obstacles(2*i),'k','LineWidth',1.5)
    end
    Z0=Z;
    u(k-1,:)=Z(N*3*2+1:N*3*2+2)'+ur;
    uk=u(k-1,:)';
    x(k,:) =A*x(k-1,:)' + B*u(k-1,:)';
    if exitflag==-2
       disp("----Unfeasible!----")
       break
    end
%     for i=1:N-1
%         test(k,i)=abs(Z(10*i)-Z(10*(i+1)));
%     end
    plot(x(:,1),x(:,2),'ok')
    pause(0.01)
end
% figure(2)
% plot(test)

subplot(2,1,1)
plot(x_tilde)
legend('x','y','theta')
title("x_bar")
subplot(2,1,2)
plot(u_tilde)
legend('u1','u2')
title("u_bar")
disp("-----done-----")


    
