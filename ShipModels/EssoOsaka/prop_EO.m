function [XP, YP, NP] = prop_EO(x,u)

%% Obtain x, u and y
u_vel   = x(4);
v_vel   = x(5);
r       = x(6);

n       = u(2);

if abs(u_vel) < 1e-8 
    u_vel = 1e-8;
end
%%   Principal Particulars
L   = 3.0;
d   = 0.20114;
U   = sqrt(u_vel^2+v_vel^2);
rho = 1000/9.8;

if (abs(U) < 1.0d-8)
    v_nd = 1.0d-8;
    r_nd = 1.0d-8;
else
    v_nd = v_vel/U;
    r_nd = r*L/U;
end

%   Coefficients relating propeller
Dp    =  0.084;
t     =  0.22;
wP0   =  0.614;%0.386;
tau   =  0.871;
CP_nd = -0.359;
xP_nd = -0.517;%2018-1-31 change wP0 &xP_nd (0.386 to 0.614,0.517 to -0.517)
a1    =  0.3278;
a2    = -0.3223;
a3    = -0.156;

A1    = -7.9e-5;
A2    =  7.99e-3;
A3    = -4.93e-3;
A4    = -5.87e-3;
A5    = -5.58e-4;

B1    = 3.5e-5;
B2    = -3.17e-3;
B3    = 1.96e-3;
B4    = 2.33e-3;%12/27/2017 
B5    = 2.25e-4;
%   Hachii master paper data
C3    = -0.251;
C6    = -0.175;
C7    =  0.33;
C10   = -0.233;

Jsyn  = -0.35;
Jsyn0 = -0.06;

wP    = wP0 - tau*abs(v_nd+xP_nd*r_nd) - CP_nd*(v_nd+xP_nd*r_nd)^2;
if(wP > 1)
    wP = 1;
elseif wP < 0
    wP = 0;
end

Js = u_vel/(Dp*n);
if abs(n) < 1e-3
    XP = 0;
    YP = 0;
    NP = 0;  
    
elseif n > 0 % (1st and 4th Quadrants)
    J  = Js*(1-wP);
    KT = a1+a2*J+a3*J^2;

    XP = rho*Dp^4*n^2*(1-t)*KT;
    YP = 0;
    NP = 0;
 
else % (n < 0, reverse gear, 2nd and 3rd Quadrants)
    if (Js>=C10)
        XP = rho*Dp^4*n^2*(C6+C7*Js);
    else
        XP = rho*Dp^4*n^2*C3;
    end

    if Jsyn <= Js && Js <= Jsyn0
        YP = 0.5*rho*L  *d*(n*Dp)^2*(A1+A2*Js);
        NP = 0.5*rho*L^2*d*(n*Dp)^2*(B1+B2*Js);
    elseif Js < Jsyn
        YP = 0.5*rho*L  *d*(n*Dp)^2*(A3+A4*Js);
        NP = 0.5*rho*L^2*d*(n*Dp)^2*(B3+B4*Js);
    else
        YP = 0.5*rho*L  *d*(n*Dp)^2*A5;
        NP = 0.5*rho*L^2*d*(n*Dp)^2*B5;
    end
end