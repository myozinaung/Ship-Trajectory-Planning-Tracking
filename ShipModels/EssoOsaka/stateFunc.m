function dxdt = stateFunc(t,x,u,p)
wind_para = p(1:2);

%% Obtain variables
psi   = x(3);
u_vel = x(4);
v_vel = x(5);
r     = x(6);

%%   Parameters
L       = 3.0;
d       = 0.20114;

m_nd    = 0.2453;
mx_nd   = 0.0467;
my_nd   = 0.2579;
IJzz_nd = 0.0286;
xG_nd   = 0.03119;
rho     = 1000/9.8;

m       = m_nd *  (0.5*rho*L^2*d);
mx      = mx_nd*  (0.5*rho*L^2*d);
my      = my_nd*  (0.5*rho*L^2*d);
IJzz    = IJzz_nd*(0.5*rho*L^4*d);
xG      = xG_nd*L;

%%
[XH, YH, NH]  = hull_EO(x);
[XP, YP, NP]  = prop_EO(x,u);
[XR, YR, NR]  = rudd_EO(x,u);
[XA, YA, NA]  = wind_EO(x,wind_para);
% XA = 0; YA = 0; NA = 0;

X = XH + XP + XR + XA;
Y = YH + YP + YR + YA;
N = NH + NP + NR + NA;

%% Compute dxdt
dxdt = x;
dxdt(1) = u_vel*cos(psi) - v_vel*sin(psi);
dxdt(2) = u_vel*sin(psi) + v_vel*cos(psi);
dxdt(3) = r;
dxdt(4) = (X + m*v_vel*r)/(m+mx);
dxdt(5) = (Y - m*u_vel*r)/(m+my);
dxdt(6) = (N - xG*Y)/IJzz;

end