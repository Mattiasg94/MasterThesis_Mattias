function [Z,exitflag] = optimizer_fmincon(Z0,A, B, N ,xf,xk,lb,ub,obstacles)
    
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
       Z0=[Omega*xk;zeros(N*2,1)]'; 
    end
    % Z = [x;u]
%     Mbelt=zeros(N,N*2);
%     for i=1:N
%         Mbelt(i,2*i-1:2*i)=ones(1,2);
%     end    
%     if  true %1<=xk(2) && xk(2)<=6 
%        activate=1;
%     else
%         activate=0;
%     end
%     Mx=[];
%     mx=[activate,0];
%     for i=1:N
%         Mx=blkdiag(Mx,mx);
%     end  

    Ain=[   
    ];
    bin=[
        ];

    Aeq=[
        eye(2*N),-Gamma
        ];
    beq=[
        Omega*xk
        ];
    
    % f=V*Z
    fun = @(Z) objective_func(Z,N,xf,obstacles);
    %nonl_con = @(Z) nonlcon(Z,N,obstacles);
    options = optimoptions('fmincon','Display','off'); %,'TolCon',1e-6
    [Z,fval,exitflag] = fmincon(fun,Z0,Ain,bin,Aeq,beq,lb,ub,[],options);
end







