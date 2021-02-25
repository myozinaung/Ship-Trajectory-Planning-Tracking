function cst = cstTriangle(x,y,a,b,c)
% This is not correct

cst = zeros(1,length(x));
for i = 1:length(x)
X = [x(i), y(i)];
cst(i) = -((abs(X-a)+abs(X-b)-abs(a-b))...
         .*(abs(X-b)+abs(X-c)-abs(b-c))...   
         .*(abs(X-c)+abs(X-a)-abs(c-a)));
end

end