function [cin,ceq] = nonlcon(Z,N,obstacle)
r=1;
cin=0;
ceq=[];
for i = 1:2:2*N 
    cin = cin+ abs(Z(i))+2
end
% cin = (Z(1)-obstacle(1))^2 + (Z(2)-obstacle(2))^2-r^2