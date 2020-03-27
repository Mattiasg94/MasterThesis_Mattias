function [cin,ceq] = nonlcon(Z,N,obstacles)
% Mx=[];
% mx=[activate,0];
% for i=1:N
%     Mx=blkdiag(Mx,mx);
% end  

cin=0;
ceq=[];
min_dist=0.2; % low=close to
inc_slope=0.1;
if obstacles~=0
for i = 1:N    
    for j=1:length(obstacles)/2
        if j==1
            gt_distance=obstacles(j*2-1)-min_dist;
            obs_const(j)=Z(i*2-1)-gt_distance-inc_slope*(Z(i*2)-obstacles(j*2))^2;
        else
            obs_const(j)=-Z(i*2-1)+gt_distance-inc_slope*(Z(i*2)-obstacles(j*2))^2;
        end
    %obs_const(j)=Z(2*j-1)-gt_distance-inc_slope*(Z(2*j)-obstacles(j*2))^2;
    end    
    cin(2*i-1:2*i,1) = obs_const;
end
cin=cin';
end
