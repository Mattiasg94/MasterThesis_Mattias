clc
clear all, close all

N=10;
Nsim=30;
dt=0.5;
ur=[0.1;0]; 
xr=[10;5;0];
x0=[0;4;0];
u0=[0;1];
lb=[-10  -10 -2*pi -10 -10 -2*pi -1 -1];
ub=[10 10 2*pi 10 10 2*pi 1 1];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR]=setup(x0,ub,lb,Nsim,N);
%% plot
obstacles=0;%[[5,4],[4,6]];
xcont = linspace(x0(1),xr(1)+5);
ycont = linspace(x0(2)-5,xr(2)+5);
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
    [Z,fval,exitflag] = optimizer_fmincon(xr,Z0,A, B,MQ,MR, N,x_tilde(k-1,:)',u_tilde(k-1,:)',lb,ub,obstacles);
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
    elseif exitflag==0
      disp("----Converged!----")
      break
    end
    plot(x(:,1),x(:,2),'ok')
    pause(0.01)
end
figure(2)
subplot(2,1,1)
plot(x_tilde)
legend('x','y','theta')
title("x_bar")
subplot(2,1,2)
plot(u_tilde)
legend('u1','u2')
title("u_bar")



    
