function cst = cstEllipse(x,y,elli,t)

x0  = elli(1);
y0  = elli(2);
a   = elli(3); % Radius in x-axis
b   = elli(4); % Radius in y-axis
psi = elli(5);
vel = elli(6);

vx = vel * cos(psi);
vy = vel * sin(psi);

x0 = x0 + vx.*t;
y0 = y0 + vy.*t;

cst = -((((x-x0).*cos(psi)+(y-y0).*sin(psi))./a).^2 ...
       +(((x-x0).*sin(psi)-(y-y0).*cos(psi))./b).^2 - 1);

end