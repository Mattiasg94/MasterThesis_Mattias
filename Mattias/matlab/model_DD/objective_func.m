function obj_fun = objective_func(Z,MQ,MR,N,obstacles)

obj_fun=Z(1:N*3)*MQ*Z(1:N*3)'+Z(N*6+1:end)*MR*Z(N*6+1:end)';
% for i = 1:N
%     fi=N*3+1+(i-1)*3;
%     se=N*3+2+(i-1)*3;
%     th=N*3+3+(i-1)*3;
%     f = f + abs(Z(fi)-10)+ abs(Z(se)-5);%+ abs(Z(th)-0);
%     %f = f + Z(i*8-7:i*8-5)*Q*Z(i*8-7:i*8-5)'+Z(i*8-1:i*8)*R*Z(i*8-1:i*8)';
% end
end

