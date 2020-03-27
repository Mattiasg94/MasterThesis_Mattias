clc
clear all, close all
tic
N=10;
Nsim=20;
dt=0.5;
ur=[0;0];
xr=[10;10;0];
x0=[0;0;pi/4];   %%% X0
u0=[0;0];
lb_x=[0 0 -2*pi];
ub_x=[inf inf 2*pi];
lb_u=[0 -0.5];
ub_u=[1 0.5];
dv=5;
dw=1;
lb=[-inf -inf -inf lb_x -inf -inf lb_u];
ub=[ inf  inf  inf ub_x  inf  inf ub_u];
[u,x,u_tilde,x_tilde,lb,ub,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta]=setup(x0,u0,xr,ur,ub,lb,Nsim,N);
%% plot
obstacles={[4;4.5;0]};%,[5;5;pi/2]}; %,[5;4.5;0]};
obstacles_u={[0;0]};%,[.1;0.3]}; %,[0.25;0]};
% [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{1},obstacles_u{1},dt)
plot_obstacles = plot(1); textbox=plot(1); plot_obstacles_radius=plot(1);
r_obs=1;
% xcont = linspace(x0(1),xr(1)+5);
% ycont = linspace(x0(2)-5,xr(2)+5);
% [X,Y] = meshgrid(xcont,ycont);
% fun = (X-xr(1)).^2 +(Y-xr(2)).^2;
% contour(X,Y,fun,100)
% scatter(xr(1),xr(2),50,'y','LineWidth',5);
viscircles([xr(1),xr(2)],0.1,'Color','y','Linewidth',5);
xlim([0,11])
ylim([0,11])
hold on 

Vfval=zeros(Nsim,1);
Vfval(1)=inf;
feasible=1;
close_obstacles=obstacles;
close_obstacles_u=obstacles_u;
for k = 1:Nsim+1 
    if feasible
        [A,B] = Linearized_discrete_DD_model(x(k,:)',u(k,:)',dt);    
        x(k+1,:) =A*x(k,:)' + B*u(k,:)';
        u_tilde(k,:)=(u(k,:)'-ur);
        x_tilde(k,:)=(x(k,:)'-xr);
        delete(textbox)
    else
       delete(textbox)
       textbox=annotation('textbox', [0.7, 0.1, 0.1, 0.1], 'String', "Wait");
        u(k,:)=u(k-1,:)/2;
        x(k+1,:) =A*x(k,:)' + B*u(k,:)';
    end
    [Z,fval,exitflag] = optimizer_fmincon(x(k,:)',u(k,:)',dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta...
        ,N,lb,ub,close_obstacles,close_obstacles_u,r_obs);
    x_iter = 3*N+1;
    y_iter = 3*N+2;
    th_iter = 3*N+3;
    for i=1:N
%     r = [Z(x_iter);Z(y_iter)]-obstacles{1}(1:2);
%     vab = Z(8*N+1+(2*(i-1)))*[cos(Z(th_iter)); sin(Z(th_iter))] - [obstacles_u{1}(1);obstacles_u{1}(2)];
%         if dot(r,vab)>0
%             disp("dot(r,vab)= "+dot(r,vab)+" iter= "+ i)
%         end
        x_iter = x_iter + 3;
        y_iter = y_iter + 3;
        th_iter = th_iter + 3;
    end
    u(k+1,:)=Z(N*8+1:N*8+2)';
    Z0=Z;
    Zx_tilde=Z(1:3*N);
    Zx=Z(3*N+1:6*N);
    for j=1:N
        Zx_plot(j,:)=Zx(3*j-2:3*j);
        Zx_tilde_plot(j,:)=Zx_tilde(3*j-2:3*j);        
    end    
    xs = [x(k,1) x(k,1) + cos(x(k,3))*0.3];
    ys = [x(k,2) x(k,2) + sin(x(k,3))*0.3];
    plot(xs,ys)
    plot(x(:,1),x(:,2),'ok')
    xlim([0,11])
    ylim([0,11])
    track=plot(Zx_plot(:,1),Zx_plot(:,2),'*g');
    delete(plot_obstacles) 
    delete(plot_obstacles_radius)
    j=1;
    close_obstacles={};
    close_obstacles_u={};
    for i=1:length(obstacles)
        %plot_obstacles(i) = scatter(obstacles{i}(1),obstacles{i}(2),'g','LineWidth',1);
        plot_obstacles(i)=viscircles([obstacles{i}(1),obstacles{i}(2)],0.1,'Color','r','Linewidth',2);
        plot_obstacles_radius(i)=viscircles([obstacles{i}(1),obstacles{i}(2)],r_obs,'LineStyle','--','Color','r','Linewidth',0.8);
        [A_obstacles,B_obstacles] = Linearized_discrete_DD_model(obstacles{i},obstacles_u{i},dt);
        obstacles{i}=A_obstacles*obstacles{i}+B_obstacles*obstacles_u{i};
        if abs(x(k+1,1)-obstacles{i}(1))+abs(x(k+1,2)-obstacles{i}(2))<20
           close_obstacles{j}=obstacles{i};
           close_obstacles_u{j}=obstacles_u{i};
            j=j+1;
        end
    end
    pause(1)    
    delete(track)
    
    
    Vfval(k+1)=fval;
    last_dist_xr=abs(Z(6*N-2)-xr(1));
    last_dist_yr=abs(Z(6*N-1)-xr(2));

    if abs(x(k+1,1)-xr(1))<0.2 && abs(x(k+1,2)-xr(2))<0.2
        break
    end
    if exitflag==-2
       if  Vfval(k)-fval<=225
           disp("----converge slowly----")
       end
       if last_dist_xr>0.1 && last_dist_yr>0.1
           disp("----Unfeasible!----")
           feasible=0;
       end
       continue
    else
        feasible=1;
    end
    
end
toc
% figure(2)
% plot(Vfval)
% figure(3)
% subplot(2,1,1)
% plot(x_tilde)
% legend('x','y','theta')
% title("x_bar")
% subplot(2,1,2)
% plot(u_tilde)
% legend('u1','u2')
% title("u_bar")



    
