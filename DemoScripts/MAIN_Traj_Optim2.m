%% ~~~~~~~~~~~~~~~~~~~~~~ OCP Formulation ~~~~~~~~~~~~~~~~~~~~~~ %%
clc; 
clear;

%% "4" MANOEUVRING MODES %%
% 1. 'x6u1' = 6 States(3DOF),           1 Control(delta),     const RPS
% 2. 'x7u1' = 7 States(3DOF+delta),     1 Control(deltaRate), const RPS
% 3. 'x6u2' = 6 States(3DOF),           2 Control(delta,rps)
% 4. 'x8u2' = 8 States(3DOF+delta,rps), 2 Control(deltaRate, rpsRate)
% constant RPS is defined in the State Matrix Function

ManoModes(1) = "x8u2"; % Path 1 (WP1 to WP2)
ManoModes(2) = "x8u2"; % Path 2 (WP2 to WP3)

constRPS = 10; % constatnt is RPS is defined in stateMatrix function, This is just for multi-phase transition

%% Waypoints Time Bounds [t(sec)]
tWP_low(1) = 0;
tWP_upp(1) = 0;

tWP_low(2) = 0.01;
tWP_upp(2) = inf;

tWP_low(3) = 0.01;
tWP_upp(3) = inf;

%% Waypoints and Paths States Bounds [xpos(m), ypos(m), psi(rad), uvel(m/s), vvel(m/s), r(rad/s)]
% Waypoint 1
xWP_low(:,1) = [-50;5;deg2rad(-45);0.35;0;0];
xWP_upp(:,1) = xWP_low(:,1);

% from WP1 to WP2
xPath_low(:,1) = [-Inf;-Inf;-Inf;-Inf;-Inf;-Inf];
xPath_upp(:,1) = [Inf;Inf;Inf;Inf;Inf;Inf];

% Waypoint 2
xWP_low(:,2) = [-15;-2.5;deg2rad(45);0.01;-0.5;-0.5];
xWP_upp(:,2) = [-15;-2.5;deg2rad(45);1;0.5;0.5];

% from WP2 to WP3
xPath_low(:,2) = [-Inf;-Inf;-Inf;-Inf;-Inf;-Inf];
xPath_upp(:,2) = [Inf;Inf;Inf;Inf;Inf;Inf];

% Waypoint 3
xWP_low(:,3) = [0-0.01;0-0.01;deg2rad(0-1);0-0.001;-0.005; -0.005];
xWP_upp(:,3) = [0+0.01;0+0.01;deg2rad(0+1);0+0.001;0.005;  0.005];

%% Paths Controls Bounds [delta(rad), rps]
uPath_low(:,1) = [deg2rad(-35);10];
uPath_upp(:,1) = [deg2rad(35);10];

uPath_low(:,2) = [deg2rad(-35);-10];
uPath_upp(:,2) = [deg2rad(35);10];

%% Paths Control Rates Bounds [deltaRate(rad/s), rpsRate(rps/s)]
uRatePath_low(:,1) = [deg2rad(-30);0];
uRatePath_upp(:,1) = [deg2rad(30);0];

uRatePath_low(:,2) = [deg2rad(-30);-10];
uRatePath_upp(:,2) = [deg2rad(30);10];

%% FUNCTIONS
%% Guesses
t_guess(1,:) = [0, 100];
t_guess(2,:) = [100, 200];
% State and Control guesses are linear interpolation between final and initial conditions by default at grid points
% Better strategy can be applied if avaialble

%% Objective Function
bndObj{1}  = @(t0,x0,tF,xF)( (tF - t0)/(t_guess(1,2)-t_guess(1,1)) ); % Boundary Cost
pathObj{1} = [];   % Path Cost

bndObj{2}  = @(t0,x0,tF,xF)( (tF - t0)/(t_guess(2,2)-t_guess(2,1)) );
pathObj{2} = [];

%% Ship Dynamics
para = [0, 0]; % [U_wind, Dir_wind], para can't be struct if stateFunc is turned into MEX file
for i = 1:length(ManoModes)
    switch ManoModes(i) % State Matrix Function is selected according to "ManoMode"
        case "x6u1"
            dynamics{i} = @(t,x,u)( stateMatrix_EO_x6u1(t,x,u,para) );
        case "x7u1"
            dynamics{i} = @(t,x,u)( stateMatrix_EO_x7u1(t,x,u,para) );        
        case "x6u2"
            dynamics{i} = @(t,x,u)( stateMatrix_EO_x6u2(t,x,u,para) );
        case "x8u2"
            dynamics{i} = @(t,x,u)( stateMatrix_EO_x8u2(t,x,u,para) );
    end
end

%% Spatial Constraints   
%% Obstacles (Circle, Ellipse, Rectangle, etc.)
% Location and Size of obstacles are defined in obstacles function
Breadth = 0.5; 
% radi_inflate = Breadth/sqrt(2); % inflation radius
radi_inflate = Breadth/2*1; % inflation radius

obs_ori{1} = obstacles_P5A;
p{1}.obs   = inflateObs(obs_ori{1}, radi_inflate);
p{1}.bndXY = [xPath_low(1,1), xPath_upp(1,1), xPath_low(2,1), xPath_upp(2,1)];

obs_ori{2} = obstacles_P5B;
p{2}.obs   = inflateObs(obs_ori{2}, radi_inflate);
p{2}.bndXY = [xPath_low(1,2), xPath_upp(1,2), xPath_low(2,2), xPath_upp(2,2)];

%% Constraint Function
bndCst{1} = [];
pathCst{1} = @(t,x,u)( cstShipBerth(t,x,u,p{1}) );
% pathCst{1} = [];

bndCst{2} = [];
% pathCst{2} = @(t,x,u)( cstShipBerth(t,x,u,p{2}) );
pathCst{2} = [];

%% Plot obstacles with initial and final pose
% previewScene; 

%% Transcription Method and Grid Size (Mesh Refinement)
% Methods
% 1 = Trapezoidal
% 2 = HermiteSimpson
% 3 = Chebyshev
% 4 = Euler (no. of subStep = 5)
% 5 = rungeKutta 4th order (no. of subStep = 5)
methods{1} = [1, 1, 1, 1, 2]; % no. of method = no. of mesh refinement steps
grid_no{1} = [5, 5, 10, 20, 20, 40]; % corresponding grid size for each method

methods{2} = [1, 1, 2, 2]; % no. of method = no. of mesh refinement steps
% grid_no{2} = [5, 5, 10, 20, 20, 40]; % corresponding grid size for each method
grid_no{2} = [20, 20, 20, 40];

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

%% ~~~~~~~~~~~~~~~~ Generate the OCP problem struct ~~~~~~~~~~~~~~~~ %%
% User may not need to change from this point forward
for j = 1:size(xPath_low,2)
    %% FUNCTIONS (Objective, Dynamics, Constraint)
    ocp{j}.func.bndObj   = bndObj{j};
    ocp{j}.func.pathObj  = pathObj{j};

    ocp{j}.func.dynamics = dynamics{j};

    ocp{j}.func.bndCst   = bndCst{j};
    ocp{j}.func.pathCst  = pathCst{j};

    %% BOUNDS
    %% Time Bounds
    ocp{j}.bounds.initialTime.low = tWP_low(j);
    ocp{j}.bounds.initialTime.upp = tWP_upp(j);
    ocp{j}.bounds.finalTime.low   = tWP_low(j+1);
    ocp{j}.bounds.finalTime.upp   = tWP_upp(j+1);

    %% State Bounds
% [x0_low, x0_upp, xF_low, xF_upp, x_low, x_upp, u_low, u_upp] = assignXUBounds(ManoModes(j), ...
% xWP_low(:,j), xWP_upp(:,j), xWP_low(:,j+1), xWP_upp(:,j+1), ...
% xPath_low(:,j), xPath_upp(:,j), uPath_low(:,j), uPath_upp(:,j), uRatePath_low(:,j), uRatePath_upp(:,j));
    switch ManoModes(j)
        case 'x6u1'
            x0_low = xWP_low(:,j);
            x0_upp = xWP_upp(:,j);
            xF_low = xWP_low(:,j+1);
            xF_upp = xWP_upp(:,j+1);
            x_low  = xPath_low(:,j);
            x_upp  = xPath_upp(:,j);

            u_low = uPath_low(1,j);
            u_upp = uPath_upp(1,j);

        case 'x7u1'
            x0_low = [xWP_low(:,j); 0];
            x0_upp = [xWP_upp(:,j); 0];
            xF_low = [xWP_low(:,j+1); uPath_low(1,j)];
            xF_upp = [xWP_upp(:,j+1); uPath_upp(1,j)];
            x_low  = [xPath_low(:,j); uPath_low(1,j)];
            x_upp  = [xPath_upp(:,j); uPath_upp(1,j)];

            u_low = uRatePath_low(1,j);
            u_upp = uRatePath_upp(1,j);

        case 'x6u2'
            x0_low = xWP_low(:,j);
            x0_upp = xWP_upp(:,j);
            xF_low = xWP_low(:,j+1);
            xF_upp = xWP_upp(:,j+1);
            x_low  = xPath_low(:,j);
            x_upp  = xPath_upp(:,j);

            u_low = uPath_low(:,j);
            u_upp = uPath_upp(:,j);

        case 'x8u2'
            x0_low = [xWP_low(:,j); uPath_low(:,j)];
            x0_upp = [xWP_upp(:,j); uPath_upp(:,j)];
            xF_low = [xWP_low(:,j+1); uPath_low(:,j)];
            xF_upp = [xWP_upp(:,j+1); uPath_upp(:,j)];
            x_low  = [xPath_low(:,j); uPath_low(:,j)];
            x_upp  = [xPath_upp(:,j); uPath_upp(:,j)];

            u_low = uRatePath_low(:,j);
            u_upp = uRatePath_upp(:,j);
    end

    % Initial State
    ocp{j}.bounds.initialState.low = x0_low;
    ocp{j}.bounds.initialState.upp = x0_upp;

    % Final State
    ocp{j}.bounds.finalState.low   = xF_low;
    ocp{j}.bounds.finalState.upp   = xF_upp;

    % Path State Bounds
    ocp{j}.bounds.state.low = x_low;
    ocp{j}.bounds.state.upp = x_upp;

    %% Control Bounds (Path)
    ocp{j}.bounds.control.low = u_low;
    ocp{j}.bounds.control.upp = u_upp;

    %% GUESSES
    ocp{j}.guess.time    = t_guess(j,:);
    ocp{j}.guess.state   = [x0_low,(xF_low+xF_upp)/2];
    ocp{j}.guess.control = [(u_low+u_upp)/2,(u_low+u_upp)/2];
    % Replace NaN with 0 in case operation on inf results NaN
    ocp{j}.guess.state(isnan(ocp{j}.guess.state)) = 0; 
    ocp{j}.guess.control(isnan(ocp{j}.guess.control)) = 0;

    %% Transcription Method and NLP Solver Options
    for i = 1:length(methods{j})
        % Choose Transcription Methods
        switch methods{j}(i)
            case 1
            ocp{j}.options(i).method = 'trapezoid';
            ocp{j}.options(i).trapezoid.nGrid = grid_no{j}(i);

            case 2    
            ocp{j}.options(i).method = 'hermiteSimpson';
            ocp{j}.options(i).hermiteSimpson.nSegment = grid_no{j}(i);

            case 3
            ocp{j}.options(i).method = 'chebyshev';
            ocp{j}.options(i).chebyshev.nColPts = grid_no{j}(i);

            case 4
            ocp{j}.options(i).method = 'euler';
            ocp{j}.options(i).euler.nSegment = grid_no{j}(i);
            ocp{j}.options(i).euler.nSubStep = 5;

            case 5
            ocp{j}.options(i).method = 'rungeKutta';
            ocp{j}.options(i).rungeKutta.nSegment = grid_no{j}(i);
            ocp{j}.options(i).rungeKutta.nSubStep = 5;
        end

        % NLP solver (fmincon) Options
        ocp{j}.options(i).nlpOpt = optimset(...
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

end % for end

%% SOLVE Trajectory Planning
soln{1} = optimTraj(ocp{1});
%%
for j = 2:length(ocp)
    ocp{j} = modifyXU0(ManoModes(j-1:j), constRPS, soln{j-1}, ocp{j});
    soln{j} = optimTraj(ocp{j});
end

%% PLOTs
t_gridAll = [];
xpos_gridAll = [];
ypos_gridAll = [];
psi_gridAll = [];
uvel_gridAll = [];
vvel_gridAll = [];
r_gridAll = [];

delta_gridAll = [];
rps_gridAll = [];

deltaRate_gridAll = [];
rpsRate_gridAll = [];

for i = 1:length(soln)
    t_grid = soln{i}(end).grid.time;
    z_grid = soln{i}(end).grid.state;
    u_grid = soln{i}(end).grid.control;
    
    if i > 1
        idx0 = 2;
    else
        idx0 = 1;
    end
    t_gridAll = [t_gridAll, t_grid(idx0:end)];
    xpos_gridAll = [xpos_gridAll, z_grid(1,idx0:end)];
    ypos_gridAll = [ypos_gridAll, z_grid(2,idx0:end)];
    psi_gridAll = [psi_gridAll, z_grid(3,idx0:end)];
    uvel_gridAll = [uvel_gridAll, z_grid(4,idx0:end)];
    vvel_gridAll = [vvel_gridAll, z_grid(5,idx0:end)];
    r_gridAll = [r_gridAll, z_grid(6,idx0:end)];
    
    switch ManoModes(i)
        case "x6u1"
            delta_gridAll = [delta_gridAll, u_grid(1,idx0:end)];
            rps_gridAll = constRPS*ones(1,length(delta_gridAll));
            deltaRate_gridAll = [];
            rpsRate_gridAll = zeros(1,length(delta_gridAll));
        case "x7u1"
            delta_gridAll = [delta_gridAll, z_grid(7,idx0:end)];
            rps_gridAll = constRPS*ones(1,length(delta_gridAll));
            deltaRate_gridAll = [deltaRate_gridAll, u_grid(1,idx0:end)];
            rpsRate_gridAll = zeros(1,length(delta_gridAll));
        case "x6u2"
            delta_gridAll = [delta_gridAll, u_grid(1,idx0:end)];
            rps_gridAll = [rps_gridAll, u_grid(2,idx0:end)]; 
            deltaRate_gridAll = [];
            rpsRate_gridAll = [];
        case "x8u2"
            delta_gridAll = [delta_gridAll, z_grid(7,idx0:end)];
            rps_gridAll = [rps_gridAll, z_grid(8,idx0:end)];
            deltaRate_gridAll = [deltaRate_gridAll, u_grid(1,idx0:end)];
            rpsRate_gridAll = [rpsRate_gridAll, u_grid(2,idx0:end)];
    end
end

%%
% t_grid = soln{1}(end).grid.time;
% z_grid = soln{1}(end).grid.state;
% u_grid = soln{1}(end).grid.control;
% plotShip(t_grid,z_grid(1,:),z_grid(2,:),z_grid(3,:)*180/pi,[],3,50,'red',0,'red',0.5); hold on;

%% Trajectory Plots
previewScene;
plotShip(t_gridAll,xpos_gridAll,ypos_gridAll,psi_gridAll*180/pi,[],3,50,'red',0,'red',0.5); hold on;
hold off;

%% States Plots
figure;
subplot(3,2,1)
plot(t_gridAll, xpos_gridAll); grid on;
subplot(3,2,3)
plot(t_gridAll, ypos_gridAll); grid on;
subplot(3,2,5)
plot(t_gridAll, rad2deg(psi_gridAll)); grid on;
subplot(3,2,2)
plot(t_gridAll, uvel_gridAll); grid on;
subplot(3,2,4)
plot(t_gridAll, vvel_gridAll); grid on;
subplot(3,2,6)
plot(t_gridAll, rad2deg(r_gridAll)); grid on;

%% Controls Plots
figure;
subplot(2,1,1)
plot(t_gridAll, rad2deg(delta_gridAll)); grid on;

subplot(2,1,2)
plot(t_gridAll, rps_gridAll); grid on;

%% Save this Script and associated functions in a Zip
