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

N=1;
Nsim=10;
u = zeros(Nsim , 2);
x = zeros(Nsim + 1, 2);
x0=[0;0];
x(1,:) = x0;
lb=[0 0 0.1 0.1 zeros(1,8*N)];
ub=[10 10 1 1,ones(1,8*N)*100000];
xf=[ub(1);ub(2)];
lb=[ones(1,N)*lb(1),ones(1,N)*lb(2),ones(1,N)*lb(3),ones(1,N)*lb(4),zeros(1,8*N)];
ub=[ones(1,N)*ub(1),ones(1,N)*ub(2),ones(1,N)*ub(3),ones(1,N)*ub(4)];
Z0=0;
%% plot
obstacle=[3.3,3.3];
xcont = linspace(x0(1),xf(1)+5);
ycont = linspace(x0(2),xf(2)+5);
[X,Y] = meshgrid(xcont,ycont);
fun = (X-xf(1)).^2 + (Y-xf(2)).^2;
contour(X,Y,fun,100)
hold on 
rectangle('Position',[obstacle(1)-0.5 obstacle(2)-0.5 1 1])
grid on,set(gca,'ytick',x0(1):xf(1)+5),set(gca,'xtick',x0(2):xf(1)+5)

A_bar=[0    -1
     0     1
    -1     0
     1     0
     0    -1
     0     1
    -1     0
     1     0
    ];

b_bar=[[-0.5; 0.5; -0.5; 0.5];plus([-0.5; 0.5; -0.5; 0.5],obstacle(1))];
unfeasible=[];
i=1;
for k = 2:Nsim+1
    [Z,exitflag,MA_bar,Mb_bar] = optimizer_fmincon(Z0,A, B, N, xf, x(k-1,:)',lb,ub,A_bar,b_bar,[2.8,2.8]);
  
    Z0=Z;
    u(k-1,:)=Z(N*2+1:N*2+2);
    x(k,:) = A*x(k-1,:)' + B*u(k-1,:)';
    if exitflag==-2
        disp("----unfeasible!----")
        rectangle('Position',[x(k,1)-0.5 x(k,2)-0.5 1 1],'EdgeColor','r')
        unfeasible(i)=k;
        i=i+1;
       %break
    else
        rectangle('Position',[x(k,1)-0.5 x(k,2)-0.5 1 1],'EdgeColor','b')
    end
    plot(x(:,1),x(:,2),'*r')
    b_bar=[plus([-0.5; 0.5; -0.5; 0.5],x(k,1));plus([-0.5; 0.5; -0.5; 0.5],obstacle(1))];
    lt0(:,k)=Mb_bar'*Z(N*4+1:end)';
    beta(:,k)=Z(N*4+1:end)';
    eq(:,k)=MA_bar'*Z(N*4+1:end)';
    pause(0.1)
end
figure(2)
subplot(3,1,1);
plot(lt0')
for i=1:length(unfeasible)
xline(unfeasible(i));
end
title("b_bar'*beta<0")
subplot(3,1,2);
plot(beta')
for i=1:length(unfeasible)
xline(unfeasible(i));
end
title("beta>=0")
subplot(3,1,3);
plot(eq')
for i=1:length(unfeasible)
xline(unfeasible(i));
end
title("MA_bar'*beta=0")
disp("done")


    
