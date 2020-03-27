clc
clear all, close all

N=10;
Nsim=20;
dt=0.5;
ur=[0;0]; 
xr=[10;5;0];
x0=[0;2;-pi/2];
u0=[0;0];
lb_x=[0 0 -2*pi];
ub_x=[10 10 2*pi];
lb_u=[0 -0.5];
ub_u=[1 0.5];
lb=[-inf -inf -inf lb_x -inf -inf lb_u];
ub=[ inf  inf  inf ub_x  inf  inf ub_u];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu_delta]=setup(x0,xr,ur,ub,lb,Nsim,N);
%% plot
obstacles=0;%[[5,4],[4,6]];
xcont = linspace(x0(1),xr(1)+5);
ycont = linspace(x0(2)-5,xr(2)+5);
[X,Y] = meshgrid(xcont,ycont);
fun = (X-xr(1)).^2 +(Y-xr(2)).^2;
contour(X,Y,fun,100)
hold on 
%grid on,set(gca,'ytick',min(xcont):max(xcont)),set(gca,'xtick',min(ycont):max(ycont))



u(1,:)=u0;
for k = 1:Nsim+1 
    [A,B] = Linearized_discrete_DD_model(x(k,:)',u(k,:)',dt);    
    x(k+1,:) =A*x(k,:)' + B*u(k,:)';
    u_tilde(k,:)=(u(k,:)'-ur);
    x_tilde(k,:)=(x(k,:)'-xr);  
    [Z,fval,exitflag] = optimizer_fmincon(x(k,:)',u(k,:)',dt,xr,Z0,A,B,MQ,MR,Mxr,Mur...
        ,N,x_tilde(k,:)',u_tilde(k,:)',lb,ub,obstacles);
    %Xk=Z(N*3+1:N*3+3)';
    u(k+1,:)=Z(N*8+1:N*8+2)';
    Z0=Z;
% This plots the optimizers x and y over the whole horizon. It cannot turn!
    Zx_tilde=Z(1:3*N);
    Zx=Z(3*N+1:6*N);
    for j=1:N
        Zx_plot(j,:)=Zx(3*j-2:3*j);
        Zx_tilde_plot(j,:)=Zx_tilde(3*j-2:3*j);        
    end    

    if exitflag==-2
       disp("----Unfeasible!----")
       break
    end    
    plot(x(:,1),x(:,2),'ok')
    track=plot(Zx_plot(:,1),Zx_plot(:,2),'*r')
    pause(1)
    delete(track)
    
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



    
