function [Z,fval,exitflag] = optimizer_fmincon(Z0,A,B,MQ,MR,Mxr,Mur,Mu_delta,N ,x_tilde,u_tilde,xk,lb,ub,obstacles)

Omega = A;
    for k = 2:N
        Omega = [Omega;A^k];
    end

    Gamma = [];
    for i = 1:N
        row = [];
        for j = 1:N
            row = [row, A^abs(i-j)*B*(j<=i)];
        end
        Gamma = [Gamma; row];
    end
    if Z0==0
       Z0=[Omega*x_tilde;zeros(N*3,1);zeros(N*2,1);zeros(N*2,1)]'; 
    end
    % Z = [x_tilde,x;u_tilde,u]
    Ain=[
%         zeros(N-1,3*N),zeros(N-1,3*N),zeros(N-1,2*N),Mu_delta
%         zeros(N-1,3*N),zeros(N-1,3*N),zeros(N-1,2*N),-Mu_delta
    ];
    bin=[
%         ones(N*2-2,1)*10^-1
        ];
    Aeq=[
         eye(3*N),zeros(3*N),-Gamma ,zeros(N*3,2*N)
         %zeros(3*N),eye(3*N),zeros(N*3,2*N),-Gamma
         -eye(3*N),eye(3*N),zeros(N*3,2*N),zeros(N*3,2*N)
         zeros(2*N,3*N),zeros(2*N,3*N),-eye(2*N),eye(2*N)
        ];
    beq=[
         Omega*x_tilde
         %Omega*xk
         Mxr
         Mur         
        ];
    
    % f=V*Z
    obj_fun = @(Z) objective_func(Z,MQ,MR,N,obstacles);
    nonl_con = @(Z) nonlcon(Z,N);
    options = optimoptions('fmincon','Display','off'); %,'TolCon',1e-6
    [Z,fval,exitflag] = fmincon(obj_fun,Z0,Ain,bin,Aeq,beq,lb,ub,[],options);
%     Z(17)-Z(19)
    %assert(abs(Z(17)-Z(19))<=10^-5)
end






