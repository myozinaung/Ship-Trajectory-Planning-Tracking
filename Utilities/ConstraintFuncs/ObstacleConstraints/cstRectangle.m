function cst = cstRectangle(x,y,rect)

x0  = rect(1);
y0  = rect(2);
a   = rect(3);
b   = rect(4);
psi = rect(5);

cst = -(abs(((x-x0).*cos(psi)+(y-y0).*sin(psi))./a ...
          + ((x-x0).*sin(psi)-(y-y0).*cos(psi))./b)...
      + abs(((x-x0).*cos(psi)+(y-y0).*sin(psi))./a ...
          - ((x-x0).*sin(psi)-(y-y0).*cos(psi))./b)-1);

end