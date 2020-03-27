function [Z,fval,exitflag] = optimizer_fmincon(xk,uk,dt,dv,dw,Z0,MQ,MR,Mxr,Mur,Mu1_delta,Mu2_delta,N,lb,ub,obstacles,obstacles_u,r_obs)

    if Z0==0
       Z0=[zeros(N*3,1);zeros(N*3,1);zeros(N*2,1);zeros(N*2,1)]';
    end
    Z0==0;
    % Z = [x_tilde,x;u_tilde,u]

    Ain=[
        %Delta v
        zeros(1,N*3),zeros(1,N*3),zeros(1,N*2),[1,zeros(1,N*2-1)]
        zeros(N-1,N*3),zeros(N-1,N*3),zeros(N-1,N*2),Mu1_delta
        zeros(N-1,N*3),zeros(N-1,N*3),zeros(N-1,N*2),-Mu1_delta    
        %Delta
        zeros(1,N*3),zeros(1,N*3),zeros(1,N*2),[0,1,zeros(1,N*2-2)]
        zeros(N-1,N*3),zeros(N-1,N*3),zeros(N-1,N*2),Mu2_delta
        zeros(N-1,N*3),zeros(N-1,N*3),zeros(N-1,N*2),-Mu2_delta
    ];
    bin=[
        %Delta v
        uk(1)+dv
        ones(2*(N-1),1)*dv
        %Delta w
        uk(2)+dw
        ones(2*(N-1),1)*dw
        ];
    Aeq=[
         -eye(3*N),eye(3*N),zeros(N*3,2*N),zeros(N*3,2*N)
         zeros(2*N,3*N),zeros(2*N,3*N),-eye(2*N),eye(2*N)
        ];
    beq=[
         Mxr
         Mur 
        ];
    
    % f=V*Z
    obj_fun = @(Z) objective_func(Z,MQ,MR,N,obstacles);
    nonl_con = @(Z) nonlcon(Z,N,xk,uk,dt,obstacles,obstacles_u,r_obs);
    options = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunEvals',10000); %,'TolCon',1e-6
    [Z,fval,exitflag] = fmincon(obj_fun,Z0,Ain,bin,Aeq,beq,lb,ub,nonl_con,options);
end








