function f = AnonymousFunc(Z,N,xf,obstacle)
f = 0;

for i = 1:2:2*N 
    f = f + (Z(i)-xf(1))^2 + (Z(i+1)-xf(2))^2;%+ 1/(abs((Z(i+1)-obstacle(1)))^2 + (Z(2)-obstacle(2))^2);
end

end