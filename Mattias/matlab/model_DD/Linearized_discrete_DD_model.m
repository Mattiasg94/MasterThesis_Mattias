function [A,B] = Linearized_discrete_DD_model(xr,ur,dt)
% "Discretize Linearize"
A = [ 1, 0, -dt*ur(1)*sin(xr(3));
      0, 1,  dt*ur(1)*cos(xr(3));
      0, 0,    1];

B = [ dt*cos(xr(3)) 0;
      dt*sin(xr(3)) 0;
      0     dt];
end


