function cst = cstSoftRect(x,y,sRect,t)

x0  = sRect(1);
y0  = sRect(2);
a   = sRect(3);
b   = sRect(4);
psi = sRect(5);
vel = sRect(6);
x_exponent = sRect(7); % The exponent in the equation ( "2" for ellipse)
y_exponent = sRect(8);

vx = vel * cos(psi);
vy = vel * sin(psi);

x0 = x0 + vx.*t;
y0 = y0 + vy.*t;

   
cst = -((((x-x0).*cos(psi)+(y-y0).*sin(psi))./a).^x_exponent ...
       +(((x-x0).*sin(psi)-(y-y0).*cos(psi))./b).^y_exponent - 1); 

end