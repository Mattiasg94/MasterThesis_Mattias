function [Z,fval,exitflag] = optimizer_fmincon(xk,uk,dt,xr,Z0,A,B,MQ,MR,Mxr,Mur,N,x_tilde,u_tilde,lb,ub,obstacles)
%     Omega = A;
%     for k = 2:N
%         Omega = [Omega;A^k];
%     end
% 
%     Gamma = [];
%     for i = 1:N
%         row = [];
%         for j = 1:N
%             row = [row, A^abs(i-j)*B*(j<=i)];
%         end
%         Gamma = [Gamma; row];
%     end
    if Z0==0
       %Z0=[Omega*x_tilde;zeros(N*3,1);zeros(N*2,1);zeros(N*2,1)]'; 
       Z0=[zeros(N*3,1);zeros(N*3,1);zeros(N*2,1);zeros(N*2,1)]'; 
    end
    % Z = [x_tilde,x;u_tilde,u]
    Mxr=[];
    for i=1:N
        Mxr=[Mxr;xr];
    end
%     Vxr=[];
%     for i=1:N
%         Vxr=[Vxr;xr];
%     end
%     Vx0=[x0;zeros(3*(N-1),1)];
%     Vu0=[u0;zeros(2*(N-1),1)];
%     Mx0=zeros(3*N);
%     Mx0(1,1)=1;Mx0(2,2)=1;Mx0(3,3)=1;
%     Mu0=zeros(2*N);
%     Mu0(1,1)=1;Mu0(2,2)=1;
    Ain=[
    ];
    bin=[
        ];
    Aeq=[
         %eye(3*N),zeros(3*N),-Gamma ,zeros(N*3,2*N)
         -eye(3*N),eye(3*N),zeros(N*3,2*N),zeros(N*3,2*N)
         zeros(2*N,3*N),zeros(2*N,3*N),-eye(2*N),eye(2*N)
         %zeros(3*N),eye(3*N),zeros(N*3,2*N),-Gamma
         %zeros(3*N),Mx0,zeros(3*N,2*N),zeros(3*N,2*N)
         %zeros(2*N,3*N),zeros(2*N,3*N),zeros(2*N),Mu0
        ];
    beq=[
         %Omega*x_tilde
         Mxr
         Mur 
         %Omega*xk
         %Vx0
         %Vu0
        ];
    
    % f=V*Z
    obj_fun = @(Z) objective_func(Z,MQ,MR,N,obstacles);
    nonl_con = @(Z) nonlcon(Z,N,xk,uk,dt);
    options = optimoptions('fmincon','Display','off','Algorithm','SQP'); %,'TolCon',1e-6
    [Z,fval,exitflag] = fmincon(obj_fun,Z0,Ain,bin,Aeq,beq,lb,ub,nonl_con,options);
    exitflag
end







