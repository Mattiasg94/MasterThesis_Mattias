function obj_fun = objective_func(Z,MQ,MR,N,obstacles)
obj_fun=Z(1:N*3)*MQ*Z(1:N*3)'+Z(N*6+1:N*8)*MR*Z(N*6+1:N*8)';
end

