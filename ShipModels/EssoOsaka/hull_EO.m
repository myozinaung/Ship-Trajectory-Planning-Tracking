function [XH, YH, NH]  = hull_EO(x)
i_max = 100; %   The number of numerical integration in low speed model

%% Obtain x, u and y
u_vel     = x(4);
v_vel     = x(5);
r         = x(6);

if abs(u_vel) < 1e-8 
    u_vel = 1e-8;
end
%%   Principal Particulars
L       = 3.0;
B       = 0.48925;
d       = 0.20114;

mx_nd   = 0.0467;
my_nd   = 0.2579;
rho     = 1000/9.8;

Xuu_nd  = -0.02139;
Xvr_nd  =  0.4585;
Yv_nd   = -0.37283;
Yr_nd   =  0.116;
Nv_nd   = -0.14575;
Nr_nd   = -0.04849;

CD      = -0.0591*L/d + 1.848;
C_rY    =  0.52  *L/B - 1.062;
C_rN    =  0.0742*L/d - 0.297;

X0F_nd  =  Xuu_nd;
X0A_nd  = -0.03189;

%%
U       = sqrt(u_vel^2+v_vel^2);
beta    = -atan2(v_vel,u_vel);

Y_ad = 0.0;
N_ad = 0.0;
for i = 1:i_max
    x0 = -0.5+(i-1)/i_max;
    x1 = -0.5+(i)  /i_max;

    comp0 = v_vel + C_rY*r*L*x0;
    comp1 = v_vel + C_rY*r*L*x1;
    comp2 = v_vel + C_rN*r*L*x0;
    comp3 = v_vel + C_rN*r*L*x1;

    Y_ad = Y_ad + 0.5*(abs(comp0)*comp0    + abs(comp1)*comp1)   /i_max;
    N_ad = N_ad + 0.5*(abs(comp2)*comp2*x0 + abs(comp3)*comp3*x1)/i_max;
end

YHN_nd = -CD*Y_ad;
NHN_nd = -CD*N_ad;

XH = 0.5*rho*L*d*   ((X0F_nd + (X0A_nd-X0F_nd)*(abs(beta)/pi))*u_vel*U + (Xvr_nd+my_nd)*v_vel*r*L);
YH = 0.5*rho*L*d*   (Yv_nd*v_vel*abs(u_vel) + (Yr_nd-mx_nd)*r*L*u_vel + YHN_nd);
NH = 0.5*rho*L^2*d* (Nv_nd*v_vel*u_vel + Nr_nd*r*L*abs(u_vel) + NHN_nd);