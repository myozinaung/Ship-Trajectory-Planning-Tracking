function [XR, YR, NR] = rudd_EO(x,u)

%% Obtain x, u and y
u_vel = x(4);
v_vel = x(5);
r     = x(6);

delta = u(1);
n     = u(2);

if abs(u_vel) < 1e-8 
    u_vel = 1e-8;
end
%%   Principal Particulars
L   = 3.0;
rho = 1000/9.8;

%   Coefficients relating propeller
DP    = 0.084;
wP0   = 0.614;%0.386;
tau   = 0.871;
CP_nd = -0.359;
xP_nd = -0.517;%2018-1-31 change wP0 &xP_nd (0.386 to 0.614,0.517 to -0.517)
a1    = 0.3278;
a2    = -0.3223;
a3    = -0.156;

%   Definition of coefficients
AR      =  0.01063;
lambda  =  1.539;
tR      =  0.19;
aH      =  0.393;
xR      = -1.5;
xH_nd   = -0.45;
k       =  0.288;
epsilon =  1.42;
lR_nd   = -1.08;
gammaN  =  0.4406;
gammaP  =  0.3506;

xH = xH_nd*L;
lR = lR_nd*L;
fa = 6.13*lambda/(2.25+lambda);
%%
U   = sqrt(u_vel^2+v_vel^2);
if (abs(U) < 1.0e-8)
    v_nd = 1.0e-8;
    r_nd = 1.0e-8;
else
    v_nd = v_vel/U;
    r_nd = r*L/U;
end 

wP     = wP0 - tau*abs(v_nd+xP_nd*r_nd) - CP_nd*(v_nd+xP_nd*r_nd)^2;

if(wP > 1)
    wP = 1;
elseif wP < 0
    wP = 0;
end
uP = u_vel*(1-wP);

if (n == 0)
    Js = 1.0e10;
else
    Js = u_vel/(DP*n);
end

if n >= 0
    J = Js*(1-wP);
    KT = a1 + a2*J + a3*J^2;
    Ep = epsilon + k*(sqrt(1+8*KT/(pi()*J^2))-1);

    uR = uP*Ep;
    if v_vel+xR*r >= 0
        vR = gammaP*(v_vel+lR*r);
    else
        vR = gammaN*(v_vel+lR*r);
    end
    UR = sqrt(uR^2+vR^2);
    aR = delta + atan(vR/uR);
    
    if u_vel >= 0                   % (u > 0 & n > 0 ==> 1st Quadrant)
        FN =  0.5*rho*AR*fa*UR^2*sin(aR); 
    else                            % (u < 0 & n > 0 ==> 4th Quadrant)
        FN = -0.5*rho*AR*fa*UR^2*sin(aR); 
    end
    
else % (n < 0)
    UR = sqrt(u_vel^2+(-v_vel+xR*r)^2);
    aR = delta + atan((-v_vel+xR*r)/abs(u_vel));
%     if UR >= 0
    if u_vel >= 0                   % (u > 0 & n < 0 ==> 2nd Quadrant)
        FN =  0.5*rho*AR*fa*UR^2*sin(aR);
    else                            % (u < 0 & n < 0 ==> 3rd Quadrant)
        FN = -0.5*rho*AR*fa*UR^2*sin(aR);
    end
end

XR = -(1-tR)*    FN*sin(delta);
YR = -(1+aH)*    FN*cos(delta);
NR = -(xR+aH*xH)*FN*cos(delta);

if (abs(n) < 1e-3 && abs(U) < 1e-3)
    XR = 0;
    YR = 0;
    NR = 0;
end