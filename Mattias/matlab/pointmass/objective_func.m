function f = objective_func(Z,N,xf,obstacles)
f = 0;
obst=0;
dist_to_obst_strenght=0.4;
for i = 1:N
    goal=(Z(2*i-1)-xf(1))^2 + (Z(2*i)-xf(2))^2;
    for j=1:length(obstacles)/2
    obst=obst+1/((Z(2*i-1)-obstacles(2*j-1))^2 + (Z(2*i)-obstacles(2*j))^2);
    end
    f = f + goal+dist_to_obst_strenght*obst;
end
end
