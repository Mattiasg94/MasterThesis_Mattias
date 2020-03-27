function [Z,exitflag,MA_bar,Mb_bar] = optimizer_fmincon(Z0,A, B, N ,xf,xk,lb,ub,A_bar,b_bar,obstacle)
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
       Z0=[Omega*xk;zeros(N*2,1);zeros(N*8,1)]'; 
    else
        Z0(N*4+1:end)=zeros(N*8,1)'; 
    end
    Z0=[Omega*xk;zeros(N*2,1);zeros(N*8,1)]'; 
    % Z = [x;u,beta]
    Mbelt=zeros(N,N*2);
    for i=1:N
        Mbelt(i,2*i-1:2*i)=ones(1,2);
    end
    Mb_bar=[];
    MA_bar=[];
    for i=1:N
       MA_bar = blkdiag(MA_bar,A_bar);
       Mb_bar = blkdiag(Mb_bar,b_bar);     
    end
    
    Ain=[   
        zeros(N,N*2),zeros(N,N*2),Mb_bar'
    ];

    bin=[
        -ones(N,1)*(10^-8)
        ];

    Aeq=[
        zeros(2*N),zeros(2*N),MA_bar'
        eye(2*N),-Gamma, zeros(N*2,8*N)
        ];
    beq=[
        zeros(N*2,1)
        Omega*xk
        ];
    
    % f=V*Z
    
    fun = @(Z) AnonymousFunc(Z,N,xf,obstacle);
    %nonl_con = @(Z) nonlcon(Z,N,obstacle);
%     Z0=[Omega*xk;zeros(N*2,1);zeros(N*8,1)]';
    options = optimoptions('fmincon','Display','off'); %,'TolCon',1e-6
    [Z,fval,exitflag] = fmincon(fun,Z0,Ain,bin,Aeq,beq,lb,ub,[],options);
%     f=[-2*ones(N,2*N),ones(N,2*N),zeros(N,8*N)]';
%     [Z,fval,exitflag] = linprog(f,Ain,bin,Aeq,beq,lb,ub);
    
end







