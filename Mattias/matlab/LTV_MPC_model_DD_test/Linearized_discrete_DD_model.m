function [A,B] = Linearized_discrete_DD_model(x,u,dt)
% A = [ 1, 0, -dt*u(1)*sin(x(3));
%       0, 1,  dt*u(1)*cos(x(3));
%       0, 0,    1];
A=eye(3);
B = [ dt*cos(x(3)) 0;
      dt*sin(x(3)) 0;
      0     dt];
  
  
%   A = [ 1, dt*u(2), 0;
%       -dt*u(2), 1,  dt*u(1);
%       0, 0,    1];
% 
% B = [ dt 0;
%       0 0;
%       0     dt];
end


