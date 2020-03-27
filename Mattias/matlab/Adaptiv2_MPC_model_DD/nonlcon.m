function [cin,ceq] = nonlcon(Z,N)
ceq=[];
for i=1:N-1
   cin(i,1)=abs(Z(i*10)-Z((i+1)*10))-1;
end
