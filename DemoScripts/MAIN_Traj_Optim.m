%% ~~~~~~~~~~~~~~~~~~~~~~ OCP Formulation ~~~~~~~~~~~~~~~~~~~~~~ %%

clc; 
clear;
%% Actuator Limits
% Control Limts [Min, Max]
deltaBnd =  deg2rad([-35, 35]);    % [rad]
rpsBnd   = [-10, 10];              % [rps]

% Control Rate Limits [Min, Max]
deltaRateBnd = deg2rad([-30, 30]);    % [rad/s]
rpsRateBnd   = [-10, 10];             % [rps/s]

%% Boundary Conditions (Initial and Final Conditions)
%% Initial Conditions (Upper and Lower are assumed same)
t0    = 0;

xpos0 = 0;
ypos0 = 0;
psi0  = 0;
uvel0 = 0.35;
vvel0 = 0;
r0    = 0;

delta0 = 0;
rps0   = 0;

deltaRate0 = 0;
rpsRate0   = 0;

%% Final Conditions [Upper, Lower]
% if Upper = Lower >> Fixed
% if [-inf, inf]   >> Free
tF    = [0, inf];

xposF = [100, 100];
yposF = [0, 0];
psiF  = [0, 0];
uvelF = [-inf, inf];
vvelF = [-inf, inf];
rF    = [-inf, inf];

deltaF = deltaBnd;
rpsF   = rpsBnd;

deltaRateF = deltaRateBnd;
rpsRateF   = rpsRateBnd;

%% Spatial Constraints
%% X-Y Bounds
xposBnd = [-inf, inf];
yposBnd = [-inf, inf];
p.bndXY  = [xposBnd; yposBnd];
    
%% Obstacles (Circle, Ellipse, Rectangle, etc.)
obs_ori = obstacles_COLREG; % Location and Size of obstacles are defined in this function
Breadth = 0.5;
radi_inflate = Breadth/sqrt(2); % inflation radius
obs     = inflateObs(obs_ori, radi_inflate);
p.obs   = obs;

%% Other States Path Bounds
psiBnd  = [-inf, inf];
uvelBnd = [-inf, inf];
vvelBnd = [-inf, inf];
rBnd    = [-inf, inf];

%% Guesses
tF_guess = 100;
% State and Control guesses are linear interpolation between final and initial conditions by default at grid points
% Better strategy can be applied if avaialble
%% Objective Function
bndObj  = @(t0,x0,tF,xF)( ((tF - t0)/tF_guess) ); % Boundary Cost
pathObj = [];   % Path Cost

%% Ship Dynamics
para = [0, 0]; % [U_wind, Dir_wind]
%%% "4" Mano Modes %%%
% 'x6u1' = 6 States(3DOF),           1 Control(delta),     const RPS
% 'x7u1' = 7 States(3DOF+delta),     1 Control(deltaRate), const RPS
% 'x6u2' = 6 States(3DOF),           2 Control(delta,rps)
% 'x8u2' = 8 States(3DOF+delta,rps), 2 Control(deltaRate, rpsRate)
% constant RPS is defined in the State Matrix Function
ManoMode = 'x8u2'; % possible: 'x6u1' 'x7u1' 'x6u2' 'x8u2'
switch ManoMode % State Matrix Function is selected according to "ManoMode"
    case 'x6u1'
        dynamics = @(t,x,u)( stateMatrix_EO_x6u1(t,x,u,para) );
    case 'x7u1'
        dynamics = @(t,x,u)( stateMatrix_EO_x7u1(t,x,u,para) );        
    case 'x6u2'
        dynamics = @(t,x,u)( stateMatrix_EO_x6u2(t,x,u,para) );
    case 'x8u2'
        dynamics = @(t,x,u)( stateMatrix_EO_x8u2(t,x,u,para) );
end
%% Constraint Function
bndCst = [];
pathCst = @(t,x,u)( cstShipBerth(t,x,u,p) );

%% Transcription Method and Grid Size (Mesh Refinement)
% Methods
% 1 = Trapezoidal
% 2 = HermiteSimpson
% 3 = Chebyshev
% 4 = Euler (no. of subStep = 5)
% 5 = rungeKutta 4th order (no. of subStep = 5)
methods = [1, 1, 2]; % no. of method = no. of mesh refinement steps
grid_no = [20, 40, 40]; % corresponding grid size for each method

%% NLP Solver Settings (refer to MATLAB "optimset")
Algorithm    = 'sqp'; % possible: 'sqp' 'interior-point'
Display_iter = 'iter';% possible: 'iter' 'none' etc.
MaxIter      = 1000;  % Maximum no. of iteration
MaxFunEvals  = 1e6;   % Maximum no. of Objective function evaluation
MaxTime      = 600;   % [sec] Time limit for NLP solver
TolFun       = 1e-5;  % Optimality Tolerence
TolX         = 1e-5;  % Step Tolerence
TolCon       = 1e-5;  % Constraint Tolerence
ScaleProblem = 'none';% possible: 'none' 'obj-and-constr' 'jacobian'

%% Plot obstacles with initial and final pose
previewScene; 

%% ~~~~~~~~~~~~~~~~ Generate the OCP problem struct ~~~~~~~~~~~~~~~~ %%
% User may not need to change from this point forward
%% Functions (Objective, Dynamics, Constraint)
problem.func.bndObj   = bndObj;
problem.func.pathObj  = pathObj;

problem.func.dynamics = dynamics;

problem.func.bndCst   = bndCst;
problem.func.pathCst  = pathCst;

%% Bounds
% Time Bounds
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low   = tF(1);
problem.bounds.finalTime.upp   = tF(2);

switch ManoMode
    case 'x6u1'
        x0 = [xpos0; ypos0; psi0; uvel0; vvel0; r0];
        xF = [xposF; yposF; psiF; uvelF; vvelF; rF];
        xBnd = [xposBnd; yposBnd; psiBnd; uvelBnd; vvelBnd; rBnd];

        u0 = delta0;
        uF = deltaF';
        uBnd = deltaBnd';
        
    case 'x7u1'
        x0 = [xpos0; ypos0; psi0; uvel0; vvel0; r0; delta0];
        xF = [xposF; yposF; psiF; uvelF; vvelF; rF; deltaF];
        xBnd = [xposBnd; yposBnd; psiBnd; uvelBnd; vvelBnd; rBnd; deltaBnd];
        
        u0 = deltaRate0;
        uF = deltaRateF';
        uBnd = deltaRateBnd;
                
    case 'x6u2'
        x0 = [xpos0; ypos0; psi0; uvel0; vvel0; r0];
        xF = [xposF; yposF; psiF; uvelF; vvelF; rF];
        xBnd = [xposBnd; yposBnd; psiBnd; uvelBnd; vvelBnd; rBnd];

        u0 = [delta0; rps0];
        uF = [deltaF; rpsF];
        uBnd = [deltaBnd; rpsBnd];
        
    case 'x8u2'
        x0 = [xpos0; ypos0; psi0; uvel0; vvel0; r0; delta0; rps0];
        xF = [xposF; yposF; psiF; uvelF; vvelF; rF; deltaF; rpsF];
        xBnd = [xposBnd; yposBnd; psiBnd; uvelBnd; vvelBnd; rBnd; deltaBnd; rpsBnd];
        
        u0 = [deltaRate0; rpsRate0];
        uF = [deltaRateF; rpsRateF]; 
        uBnd = [deltaRateBnd; rpsRateBnd];
end
xF_low = xF(:,1);
xF_upp = xF(:,2);
x_low  = xBnd(:,1);
x_upp  = xBnd(:,2);
uF_low = uF(:,1);
uF_upp = uF(:,2);
u_low  = uBnd(:,1);
u_upp  = uBnd(:,2);

%%
% Initial State
problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;

% Final State
problem.bounds.finalState.low   = xF_low;
problem.bounds.finalState.upp   = xF_upp;

% Path State Bounds
problem.bounds.state.low = x_low;
problem.bounds.state.upp = x_upp;

% Path Control Bounds
problem.bounds.control.low = u_low;
problem.bounds.control.upp = u_upp;

%% Guesses
problem.guess.time    = [t0, tF_guess];
problem.guess.state   = [x0,(xF_low+xF_upp)/2];
problem.guess.control = [u0,(uF_low+uF_upp)/2];

%% Transcription Method and NLP Solver Options
for i = 1:length(methods)
    % Choose Transcription Methods
    switch methods(i)
        case 1
        problem.options(i).method = 'trapezoid';
        problem.options(i).trapezoid.nGrid = grid_no(i);

        case 2    
        problem.options(i).method = 'hermiteSimpson';
        problem.options(i).hermiteSimpson.nSegment = grid_no(i);

        case 3
        problem.options(i).method = 'chebyshev';
        problem.options(i).chebyshev.nColPts = grid_no(i);

        case 4
        problem.options(i).method = 'euler';
        problem.options(i).euler.nSegment = grid_no(i);
        problem.options(i).euler.nSubStep = 5;

        case 5
        problem.options(i).method = 'rungeKutta';
        problem.options(i).rungeKutta.nSegment = grid_no(i);
        problem.options(i).rungeKutta.nSubStep = 5;
    end
    
    % NLP solver (fmincon) Options
    problem.options(i).nlpOpt = optimset(...
    'Display', Display_iter,...
    'MaxIter', MaxIter,...
    'TolFun', TolFun,... % Optimality Tol
    'TolX', TolX,...   % Step Tol
    'TolCon', TolCon,...
    'MaxFunEvals', MaxFunEvals,...
    'Algorithm', Algorithm,... % sqp interior-point % sqp >> sometime bad results, but less iteration and faster
    'MaxTime', MaxTime, ...
    'ScaleProblem', ScaleProblem); %  none obj-and-constr jacobian
end
%% Save this Script and associated functions in a Zip
