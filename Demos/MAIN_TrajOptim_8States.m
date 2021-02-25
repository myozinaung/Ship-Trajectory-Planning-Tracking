%% First Plan the path with Mesh Refinement, Second Simulate and recalcualte path, apply first control only, Squeezing Horizon MPC
% x_vector = [x_pos, y_pos, psi, u_vel, v_vel, r, delta, rps];
% u_vector = [delta_rate, rps_rate] % Actual Contorl rates as Control Vector, so that Constraints can be imposed on them
%%
clc; 
clear;
addpath ../../
%%
Breadth = 0.5;
radInfl = Breadth/sqrt(2); % inflation radius
obs_ori = obstacles_COLREG;
p.obs   = inflateObs(obs_ori,radInfl);
p.bndXY = [-inf, inf, -inf, inf]; % [xmin,xmax, ymin, ymax];
para = [0, 0]; % Wind speed (NED) and direction
tF_guess = 300;

%%  STATE, COST(path and bnd), CONSTRAINTS(path and bnd) Functions
problem.func.dynamics = @(t,x,u)( stateMatrix_EO(t,x,u,para) );

% problem.func.pathObj  = @(t,x,u)( costShipBerth(t,x,u) ); 
problem.func.bndObj   = @(t0,x0,tF,xF)( ((tF - t0)/tF_guess) ); % minimum time % Primary (may apply Weight if use together with other costs)
% When divided by t_F, objFunc becomes constant "1", only constraints are considered, faster NLP (especially in SQP) and less optimal
% Not good result, especially when rps is not fixed, rps control speed --> time
% Even if the optimality is not satisfactory, it could be used as Iinitil Guess
% Minimum time Objective function should be divided by "t_guess" to make Objective function just below "1"

problem.func.pathCst  = @(t,x,u)( cstShipBerth(t,x,u,p) ); % same for both Ctrl rate (for now)
% problem.func.bndCst   = @(t0,x0,tF,xF,u)( bndcstShipBerth(t0,x0,tF,xF,u) ); % may be for stopping propeller/engine

%% BOUNDS
deltaMax  = deg2rad(35);
rpsMin    = 10; % equal or lower than 1.1 >> a little problem
rpsMax    = 10;
deltaRate = deg2rad(30); % = p.uRate
rpsRate   = 10;
p.uRate   = [deltaRate; rpsRate]; 
p.uLast   = [0; 0]; % value not important, reassign in the sim

% Time Bounds
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low   = 0.01; %0.01
problem.bounds.finalTime.upp   = inf;

% Initial State
problem.bounds.initialState.low = [0; 0; 0; 0.35; 0; 0; -deltaMax*0; rpsMax];
problem.bounds.initialState.upp = [0; 0; 0; 0.35; 0; 0; deltaMax*0; rpsMax];

% Final State
problem.bounds.finalState.low   = [100-0.01; 0-0.01; 0-pi/360; 0-0.01*1; -0.5; -0.5; -deltaMax*0; rpsMin];
problem.bounds.finalState.upp   = [100+0.01; 0+0.01; 0+pi/360; 1+0.01*1; 0.5; 0.5; deltaMax*0; rpsMax];

% Path State Bounds
problem.bounds.state.low = [p.bndXY(1);p.bndXY(3);-inf;-inf;-inf;-inf;-deltaMax;rpsMin];
problem.bounds.state.upp = [p.bndXY(2);p.bndXY(4); inf; inf; inf; inf; deltaMax;rpsMax];

% Path Control Bounds
problem.bounds.control.low = [-deltaRate; -rpsRate];
problem.bounds.control.upp = [deltaRate; rpsRate];

%% Guesses
problem.guess.time    = [0, tF_guess];
problem.guess.state   = [(problem.bounds.initialState.low+problem.bounds.initialState.upp)/2,...
    (problem.bounds.finalState.low+problem.bounds.finalState.upp)/2];
problem.guess.control = [0,0; 0,0];

%% Solver Options
%%% Remember 1st Calculation may not use bndCst and pathCst (check "optimTraj")
% 1 = Trapezoidal, 2 = HermiteSimpson, 3 = Chebyshev, 4 = Euler, 5 = rungeKutta

grid_no = [20, 40, 40]; % Berth, try only with Trapzoidal to avoid oscillation
method  = [1, 1, 2, 1, 2, 2, 2]; % Berth 
% RK method ==> Not good for refinement, Infeasible solution at small grid_no
% Chebyshev ==> Global Collocation ==> fit to a higher order polynomial ==>
% doesn't need larger grid point (poly order) ==> could skip the obstacles

for i = 1:length(grid_no)
    problem.options(i).nlpOpt = optimset(...
    'Display','iter',...
    'MaxIter',1000,...
    'TolFun',1e-3,... % Optimality Tol
    'TolX',1e-5,...   % Step Tol
    'TolCon',1e-5,...
    'MaxFunEvals',2e6,...
    'Algorithm','sqp',... % sqp interior-point % sqp >> sometime bad results, but less iteration and faster
    'MaxTime',600, ...
    'ScaleProblem', 'none'); %  none obj-and-constr jacobian

%     problem.options(i).nlpOpt = knitro_options(...
%     'algorithm', 4, ...
%     'outlev', 4 ,...
%     'maxit', 1000, ...
%     'xtol', 1e-5, ...
%     'feastol', 1e-5, ...
%     'opttol', 1e-5);

    switch method(i)
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
end

%% SOLVE
Init_CPUTimeStart = tic;
soln = optimTraj(problem); % Initial Solution with Mesh Refinement
Init_CPUTime = toc(Init_CPUTimeStart);
disp(['Initial Solution CPU Time: ',num2str(Init_CPUTime),' sec']);


%% SIMULATION %% Initial Solution should be very good (smooth, feasiable, globally optimal)
%%% Tips to speed up OCP/NLP %%%
% - Large t_update
% - Small Mesh/Grid Size
% - Relaxed NLP Tolerences
% - Relaxed Bounds/Constraints
% - Good NLP guesses
% - Scaling (Non dimensionalize)
% - Use sigle control var(delta) when rps is fixed
% - Use 6 State version (but oscillation may occur)
% - Use Code Gen (mex files)
% - Use Analytical functions

t_RefTraj = linspace(soln(end).grid.time(1), soln(end).grid.time(end), 1000);
z_RefTraj = soln(end).interp.state(t_RefTraj);

t_start  = 0;
t_end    = floor(soln(end).grid.time(end));
t_SimEnd    = 170;
t_step   = 0.01; % for Ship Simulation model
t_update = 0.1; % Control Update time (sec) for OCP
grid_size_min = 5; % when coarse mesh is used, found the segments in the solution at the collocation point in early solution
grid_size0 = 20; % = grid_no(end); % initially same as Initial Solution
rad_F = 1; % radius of accaptance for final point
p.dt = t_update;

para = [0.5, deg2rad(45)]*1; % Wind speed (Space-fixed) and direction

% Reference Tracking Weights (Weights should consider the unit of the variables, also balance with t_F cost(min time))
% Q_weights = diag([1, 1, 1, 0, 0, 0, 1, 0]); % [x, y, psi, u, v, r, delta, rps], for CA
Q_weights = diag([2, 2, 2, 1, 1, 1, 0, 0]); % [x, y, psi, u, v, r, delta, rps], for Berth
% Q_weights = diag([1, 1, 1, 1, 0.5, 0.5, 0.2, 0.2]); % [x, y, psi, u, v, r, delta, rps], for Berth
R_weights = diag([0, 0]); % [deltaRate, rpsRate]

data_len   = (t_SimEnd - t_start)/t_step + 1;
no_state   = length(problem.bounds.initialState.low);
no_control = length(problem.bounds.control.low);

% for saving data
state_data   = zeros(data_len,no_state);
control_data = zeros(data_len,no_control);
NLP_status = zeros(data_len,1);

% Initialize State and Control vector
state_vec   = [problem.bounds.initialState.low(1:6); soln(end).grid.state(7:8,1)];
control_vec = soln(end).grid.control(1:no_control,1); % Only first solution
problem_now = problem;
soln_now    = soln(end);

% Use relaxed setting to speed up in Real-time
problem_now.options = problem.options(end); % Mesh size should be low enough to realize realtime, limit iteration no.
problem_now.options.nlpOpt = optimset(...
    'Display','final',...
    'MaxIter',200,...
    'TolFun',1e-3,... % Optimality Tol
    'TolX',1e-3,...   % Step Tol
    'TolCon',1e-3,...
    'MaxFunEvals',5e4,...
    'Algorithm','sqp',... % sqp interior-point % sqp >> sometime bad results, but less iteration and faster
    'MaxTime',t_update+60);
problem_now.options.method = 'trapezoid';
problem_now.options.trapezoid.nGrid = grid_size0; % Coarse grid --> problem with small obstacles

% problem_now.options.method = 'hermiteSimpson';
% problem_now.options.hermiteSimpson.nSegment = grid_size0;

% problem_now.options.method = 'rungeKutta'; % NLP problem still exists.
% problem_now.options.rungeKutta.nSegment = 2;
% problem_now.options.rungeKutta.nSubStep = 5;
%% Realtime Sim Starts
sim_CPUTimeStart = tic;
disp('Start Simulation');
i = 1; % index for data saving
t_temp   = 0;
for t_now = t_start:t_step:t_SimEnd 
    
    % for saving State and Control Data
    state_data(i,:)   = state_vec;
    control_data(i,:) = control_vec;
    NLP_status(i) = soln_now(end).info.exitFlag;
    i = i + 1;
    % Compute the next state vector using the 4th Runge-Kutta-Gill method.
    next_state_vec = rungeKuttaGill(t_now, state_vec(1:6), state_vec(7:8), t_step, para); % use the previous control sequence with time interpolation when control is not updating
    % Only 1st u is applied --> each time step, the solution are kind of independent --> not smooth throughout the simulation
    
        
    % Run OCP for next time step without mesh refinement, guess = previous solution sequences
    t_temp = t_temp + t_step;
    if abs(t_temp - t_update) < 1e-5
%     if rem(t_now,t_update) == 0
        disp(['Simulation Time: ',num2str(t_now),' sec']);
        disp(['State Vector: ',num2str(next_state_vec',4)]);
        disp(['Control Vector: ',num2str(state_vec(7:8)',4)]);
        
        % For variable mesh size (if grid size changes, initial guess needs to use interpolation, if grid size change the guess might not be good enough --> slower
%         grid_size = % change grid size 
%         % problem_now.options.trapezoid.nGrid = grid_size;
%         problem_now.options.hermiteSimpson.nSegment = grid_size;
        
        % Constraint on Actual Control Changes (Rate)
        p.uLast = state_vec(7:8);
%         problem_now.func.pathCst  = @(t,x,u)( cstShipBerthSim(t,x,u,p) );
        problem_now.func.pathCst  = []; % TEST: remove constraints
        problem_now.func.bndCst  = []; % TEST: remove constraints
        
%         problem_now.func.bndObj = []; % remove min t_F
        
       [x_ref, u_ref, t_ref] = refGenerator(t_now, t_end, grid_size0, soln, next_state_vec, t_RefTraj, z_RefTraj, 'position');
        
        % if arrived inside the circle at the final point, use xF as xRef,
        % remove pathObj
        xF = (problem.bounds.finalState.low + problem.bounds.finalState.upp)/2; 
        xposF = xF(1);
        yposF = xF(2);
        xpos = next_state_vec(1);
        ypos = next_state_vec(2);
        if ((xpos-xposF)^2+(ypos-yposF)^2 < rad_F^2)
            disp('Arrived within the Circle.');
%             x_ref = repmat(xF,1,grid_size0);
            problem_now.func.pathObj  = []; % only min tF
        end
        
        problem_now.func.pathObj  = @(t,x,u)( costShipBerthSim(t,x,u,x_ref,u_ref, Q_weights, R_weights) ); 
        
        problem_now.bounds.initialState.low = [next_state_vec; -deltaMax;rpsMin];
        problem_now.bounds.initialState.upp = [next_state_vec; deltaMax; rpsMax];

        [t_guess, z_guess, u_guess] = guessGenerator(t_ref, t_end, soln, soln_now, 'auto');
        problem_now.guess.time    = t_guess;
        problem_now.guess.state   = z_guess;
        problem_now.guess.control = u_guess;

        soln_now = optimTraj(problem_now); % if the intermediate solution is not good, it could effect the next solution
          % WARNING: should commented out the 1st refinement without constraints the "optimTraj"
        t_temp = 0;
    end
    control_vec = soln_now(end).grid.control(1:no_control,1); % Only first control
    state_vec   = [next_state_vec; soln_now(end).grid.state(7:8,1)]; % 6 States from ODE solver + 2 States (controls) from OCP solution
    
end
sim_CPUTime = toc(sim_CPUTimeStart);
disp(['Total Simulation CPU Time: ',num2str(sim_CPUTime),' sec']);

%% Get Solution for PLOTTING
t = linspace(soln(end).grid.time(1), soln(end).grid.time(end), data_len);
t_sim = linspace(t_start, t_SimEnd, data_len);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);

t_grid = soln(end).grid.time;
z_grid = soln(end).grid.state;
u_grid = soln(end).grid.control;

cpuTime = zeros(length(problem.options),1);
for itr = 1:length(problem.options)
    cpuTime(itr) = soln(itr).info.nlpTime;
end
cpuTime;
cpuTimeAll = sum(cpuTime);
% 
%% Plot Comparison between Offline Path and Realtime Sim Results
% Compare State Timeseries
figure
subplot(3,2,1)
plot(t,z(1,:),t_sim,state_data(:,1)); grid on;
subplot(3,2,3)
plot(t,z(2,:),t_sim,state_data(:,2)); grid on;
subplot(3,2,5)
plot(t,z(3,:),t_sim,state_data(:,3)); grid on;
subplot(3,2,2)
plot(t,z(4,:),t_sim,state_data(:,4)); grid on;
subplot(3,2,4)
plot(t,z(5,:),t_sim,state_data(:,5)); grid on;
subplot(3,2,6)
plot(t,z(6,:),t_sim,state_data(:,6)); grid on;

% Compare Real Control Timeseries
figure
subplot(2,1,1)
plot(t_grid,z_grid(7,:)*180/pi,'o'); grid on; hold on
plot(t_sim,state_data(:,7)*180/pi,'-o','MarkerSize',3); hold off;
subplot(2,1,2)
plot(t_grid,z_grid(8,:),'o'); grid on; hold on
plot(t_sim,state_data(:,8),'-o','MarkerSize',3); hold off;

% Compare Control (Rate) Timeseries
figure
subplot(2,1,1)
plot(t_grid,u_grid(1,:),'-o'); grid on; hold on
plot(t_sim,control_data(:,1),'.'); hold off;
subplot(2,1,2)
plot(t_grid,u_grid(2,:),'-o'); grid on; hold on
plot(t_sim,control_data(:,2),'.'); hold off;

% Compare Trajectory
figure
plot(z_grid(1,:),z_grid(2,:),'o'); grid on; hold on
plot(state_data(:,1),state_data(:,2),'LineWidth', 2); axis equal; hold off;

%% Plot Only Offline Path
figure
plotShip(t_grid,z_grid(1,:),z_grid(2,:),z_grid(3,:)*180/pi,[],3,50,'red',0,'red',0.5); hold on;
% plotShip(t_sim,state_data(:,1),state_data(:,2),state_data(:,3)*180/pi,[],3,20,'blue',0,'blue',0.5); hold on;
% plot(z_grid(1,:), z_grid(2,:), 'o'); 
% plot([p.bndXY(1), p.bndXY(1)],[0, 20]);
if isfield(obs_ori,'circ')
    drawCircle(obs_ori.circ); hold on;
end
% if isfield(obs_ori,'rect')
%     drawRect(obs_ori.rect); hold on;
% end
if isfield(obs_ori,'elli')
    drawEllipse(obs_ori.elli); hold on;
end
if isfield(obs_ori,'sRect')
    drawSoftRect(obs_ori.sRect); hold on; 
end

grid on; axis equal; hold off;
% xlim([-53, 10]); ylim([-20, 13]);

figure
plot(t_grid, z_grid(4,:), '.'); grid on; hold on;
plot(t, z(4,:)); hold off;

figure
subplot(2,1,1)
plot(t_grid, z_grid(7,:), 'o'); grid on; hold on;
plot(t, z(7,:),'LineWidth',2); hold off;
subplot(2,1,2)
plot(t_grid, z_grid(8,:), 'o'); grid on; hold on;
plot(t, z(8,:)); hold off;
%% Save workspace
fileName=['CA_',datestr(now, 'yyyy-mm-dd_HH-MM-SS'),'.mat'];
save(fileName);