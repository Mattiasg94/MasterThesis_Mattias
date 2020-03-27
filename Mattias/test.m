clc
clear all
Z_length=12;
N=2;
syms v1 v2 w1 w2
% u=[v1,w1,v2,w2,v3,w3,v4,w4]';

% A=[    
%     1,0,-1,0 % 2
%     0,0, 1,0,-1,0 %3
%     0,0, 0,0, 1,0,-1,0 %4
%     ]

u_delta=[1,0,-1,0];
Mu_delta=zeros(N-1,2*N)
for i=1:N-1
   Mu_delta(i,2*(i-1)+1:2*(i-1)+4)=u_delta;   
end
Mu_delta