clc
clear all, close all

N=10;
Nsim=20;
dt=0.5;
ur=[0.1;0]; 
xr=[10;5;0];
x0=[0;4;0];
u0=[0;0];
lb_x=[-1 -1 -2*pi];
ub_x=[10 10 2*pi];
lb_u=[-1 -0.5];
ub_u=[1 0.5];
lb=[-inf -inf -inf lb_x -inf -inf lb_u];
ub=[ inf  inf  inf ub_x  inf  inf ub_u];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu_delta]=setup(x0,xr,ur,ub,lb,Nsim,N);
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
for k = 1:Nsim+1 
    u_tilde(k,:)=(uk-ur);
    x_tilde(k,:)=(x(k,:)'-xr);
    [A,B] = Linearized_discrete_DD_model(x(k,:)',uk,dt);
    [Z,fval,exitflag] = optimizer_fmincon(xr,Z0,A,B,MQ,MR,Mxr,Mur...
        ,N,x_tilde(k,:)',u_tilde(k,:)',lb,ub,obstacles);
%     obstacles(2)=obstacles(2)+0.05;
%     obstacles(4)=obstacles(4)-0.05;
    for i=1:length(obstacles)/2
    scatter(obstacles(2*i-1),obstacles(2*i),'k','LineWidth',1.5)
    end
    Z0=Z;
% This plots the optimizers x and y over the whole horizon. It cannot turn!
%     X_TILDE=Z(1:3*N);
%     X=Z(3*N+1:6*N);
%     for j=1:N
%         X_plot(j,:)=X(3*j-2:3*j);
%         X_TILDE_plot(j,:)=X_TILDE(3*j-2:3*j);
%         plot(X_plot(:,1),X_plot(:,2),'ok')
%         pause(0.1)
%     end
%    figure(2)
%    plot(X_TILDE_plot(:,2))
    u(k,:)=Z(N*3*2+1:N*3*2+2)'+ur;
    uk=u(k,:)';    
    x(k+1,:) =A*x(k,:)' + B*u(k,:)';
     
    if exitflag==-2
       disp("----Unfeasible!----")
       break
    end    
    plot(x(:,1),x(:,2),'ok')
    pause(0.01)
    for i=1:N-1
    test(k)=fval;
    end
end

figure(2)
plot(test)
% subplot(2,1,1)
% plot(x_tilde)
% legend('x','y','theta')
% title("x_bar")
% subplot(2,1,2)
% plot(u_tilde)
% legend('u1','u2')
% title("u_bar")



    
