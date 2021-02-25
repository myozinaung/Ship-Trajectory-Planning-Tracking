function [X_A, Y_A, N_A] = wind_EO(x,wind_para)
%% Get the required parameters %%
% Get the ship parameters
Loa   = 3.0;
B     = 0.5;
rho_a = 1.205;

% Get the wind related ship parameters (values by MYO)
A_T   = 0.1524;     % Transverse projected area "m^2"
A_L   = 0.2917;     % Lateral projected area "m^2"
A_OD  = 0.1531;     % Area Over the Deck
                          % Lateral projected area of Superstructure (A_SS) and LNG tanks, Containers, etc. on the deck "m^2"
x_AL  = 0.03339;     % Distance from midhsip to center of A_L    "m"
z_AL  = 0.04936;     % Distance from Waterline to center of A_L  "m"

x_AOD = -1.23339;     % Distance from midship to center of A_OD   "m"
H_BR  = 0.33210;     % Distance from Waterline to top of Superstructure (Bridge) "m"

% Values by Miyakuchi and Dimas
% A_T   = 0.13544;     % Transverse projected area "m^2"
% A_L   = 0.52038;     % Lateral projected area "m^2"
% A_OD  = 0.14230;     % Area Over the Deck
%                      % Lateral projected area of Superstructure (A_SS) and LNG tanks, Containers, etc. on the deck "m^2"
% x_AL  = -0.01299;     % Distance from midhsip to center of A_L    "m"
% z_AL  = 0.10359;     % Distance from Waterline to center of A_L  "m"
% 
% x_AOD = -1.23339;     % Distance from midship to center of A_OD   "m"
% H_BR  = 0.2940;     % Distance from Waterline to top of Superstructure (Bridge) "m"

%% Get the variables
psi = x(3);psi = rem(psi+sign(psi)*pi,2*pi)- sign(psi)*pi;
u   = x(4);
v   = x(5);

% Get the Wind Speed & Direction
U_wind = wind_para(1);     % Relative to Earth-fixed (NED) coordinate
Dir_wind = wind_para(2);  % Wind is blowing from Dir_wind

psiR = psi - Dir_wind; psiR = rem(psiR+sign(psiR)*pi,2*pi)- sign(psiR)*pi;
%% Coefficients for wind forces and moments calculation %%
% from Prof. T. Fujiwara Paper

% "CX" for Surge force 
x00 = -0.3300;
x01 =  0.2930;
x02 =  0.0193;
x03 =  0.6820;

x10 = -1.3530;
x11 =  1.7000;
x12 =  2.8700;
x13 = -0.4630;
x14 = -0.5700;
x15 = -6.6400;
x16 = -0.0123;
x17 =  0.0202;

x30 =  0.8300;
x31 = -0.4130;
x32 = -0.0827;
x33 = -0.5630;
x34 =  0.8040;
x35 = -5.6700;
x36 =  0.0401;
x37 = -0.1320;

x50 =  0.0372;
x51 = -0.0075;
x52 = -0.1030;
x53 =  0.0921;

% "CY" for Sway force
y10 =  0.6840;
y11 =  0.7170;
y12 = -3.2200;
y13 =  0.0281;
y14 =  0.0661;
y15 =  0.2980;

y30 = -0.4000;
y31 =  0.2820;
y32 =  0.3070;
y33 =  0.0519;
y34 =  0.0526;
y35 = -0.0814;
y36 =  0.0582;

y50 =  0.1220;
y51 = -0.1660;
y52 = -0.0054;
y53 = -0.0481;
y54 = -0.0136;
y55 =  0.0864;
y56 = -0.0297;

% "CN" for Yaw Moment
n10 =  0.2990;
n11 =  1.7100;
n12 =  0.1830;
n13 = -1.0900;
n14 = -0.0442;
n15 = -0.2890;
n16 =  4.2400;
n17 = -0.0646;
n18 =  0.0306;

n20 =  0.1170;
n21 =  0.1230;
n22 = -0.3230;
n23 =  0.0041;
n24 = -0.1660;
n25 = -0.0109;
n26 =  0.1740;
n27 =  0.2140;
n28 = -1.0600;

n30 =  0.0230;
n31 =  0.0385;
n32 = -0.0339;
n33 =  0.0023;

%% Calculation of of each components %%

% No. of Superstructure
ss = 1.0;

% CX
X0 = x00 + ss*x01*(B*H_BR/A_T) + ss*x02*(x_AL/z_AL) + x03*(A_OD/(Loa^2));
X1 = x10+x11*A_L/(Loa*B)+x12*Loa*z_AL/A_L+x13*Loa*H_BR/A_L+ss*x14*A_OD/A_L+x15*A_T/(Loa*B)+x16*((Loa^2)/A_T)+x17*(Loa/z_AL);
X3 = x30+ss*x31*A_L/(Loa*H_BR)+x32*A_L/A_T+x33*Loa*z_AL/A_L+ss*x34*A_OD/A_L+ss*x35*A_OD/(Loa^2)+x36*x_AL/z_AL+ss*x37*x_AOD/Loa;
X5 = x50+ss*x51*A_L/A_OD+ss*x52*x_AOD/Loa+x53*A_L/Loa/B;
	
% CY
Y1 = y10+ss*y11*x_AOD/Loa+y12*x_AL/Loa+ss*y13*A_L/A_OD+y14*x_AL/z_AL+ss*y15*A_T/B/H_BR;
Y3 = y30+y31*A_L/Loa/B+y32*Loa*z_AL/A_L+ss*y33*x_AOD/Loa+ss*y34*B/H_BR+ss*y35*A_OD/A_L+ss*y36*A_T/B/H_BR;
Y5 = y50+y51*A_L/Loa/B+ss*y52*Loa/H_BR+ss*y53*x_AOD/Loa+y54*(B^2)/A_T+y55*x_AL/Loa+y56*Loa*z_AL/A_L;

% CN
N1 = n10+n11*x_AL/Loa+n12*Loa*z_AL/A_L+n13*A_T/A_L+n14*x_AL/z_AL+n15*A_L/Loa/B+n16*A_T/Loa^2+n17*(B^2)/A_T+ss*n18*x_AOD/Loa;
N2 = n20+ss*n21*x_AOD/Loa+n22*x_AL/Loa+ss*n23*A_L/A_OD+n24*A_T/(B^2)+ss*n25*Loa/H_BR+ss*n26*A_T/B/H_BR+n27*A_L/Loa/B+n28*A_L/(Loa^2);
N3 = n30+ss*n31*x_AOD/Loa+ss*n32*A_T/B/H_BR+n33*A_L/A_T;

%% Relative Wind Speed calculation
uxr = U_wind * cos(psiR) + u;       % x-component of wind speed
uyr = U_wind * sin(psiR) - v;       % y-component of wind speed
alpha = atan2(uyr,uxr);           % Wind angle of attack (Wind Direction)
alpha = rem(alpha+sign(alpha)*pi,2*pi)- sign(alpha)*pi;
uxr = abs(uxr);
uyr = abs(uyr);

U_windr = sqrt(uxr^2+uyr^2);       % Resultant relative wind speed ralative to ship

% In case of "Control" Wind speed & Direction obtained from anemometer are relative to "Ship Speed" - Not need to calculate Relative Wind speed & Direction 

%% Coefficients for each Wind forces & Moments
CX = X0 + X1*cos(alpha) + X3*cos(3*alpha) + X5*cos(5*alpha);
CY = Y1*sin(alpha) + Y3*sin(3*alpha) + Y5*sin(5*alpha);
CN = N1*sin(alpha) + N2*sin(2*alpha) + N3*sin(3*alpha);

% Calculation of Wind Forces & Moments
X_A = 0.5*rho_a*(U_windr^2) * A_T * CX;        % Wind-induced surge resistnace
Y_A = 0.5*rho_a*(U_windr^2) * A_L * CY;        % Wind-induced sway resistance
N_A = 0.5*rho_a*(U_windr^2) * A_L * Loa * CN;  % Wind-induced yaw moment

