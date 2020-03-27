clear all
clc

pb=[1;1];
pa=[2;2];
vb=[0.01;0];
va=[1;1];

r=[pa(1)-pb(1);pa(2)-pb(2)];
vab=[va(1)-vb(1);va(2)-vb(2)];
% a=(pa(1)-pb(1))/norm(pa(1)-pb(1));
% b=(pa(1)-pb(2))/norm(pa(1)-pb(2));
% r=[a;b];
% a2=(va(1)-vb(1))/norm(va(1)-vb(1));
% b2=(va(2)-vb(2))/norm(va(2)-vb(2));
% vab=[a2;b2];
norm(r)^2
(dot(r,vab))/norm(vab)
(dot(r,vab))^2/norm(vab)^2
d=sqrt(norm(r)^2-(dot(r,vab))^2/norm(vab)^2)

rterm=(pa(1)-pb(1))^2+(pa(2)-pb(2))^2;
uterm=((va(1)-vb(1))*(pa(1)-pb(1))+(va(2)-vb(2))*(pa(2)-pb(2)))^2;
lterm=(va(1)-vb(1))^2+(va(2)-vb(2))^2;
d=sqrt(rterm-uterm/lterm);

% plot([0,va(1)],[0,va(2)]
% hold on
% plot([0,pb(1)],[0,pb(2)])
% 
% % xlim([0,1.5])
% % ylim([0,1.5])
% 
% circle(pb(1),pb(2),d);
% 
% function h = circle(x,y,r)
% hold on
% th = 0:pi/50:2*pi;
% xunit = r * cos(th) + x;
% yunit = r * sin(th) + y;
% h = plot(xunit, yunit);
% hold off
% end