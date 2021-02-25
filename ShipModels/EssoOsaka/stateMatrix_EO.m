%% State Matrix Function for 3DOF Esso Osaka(EO) MMG Model %%
% This function generates a state matrix of the size = (no_state, no_grid)
% ODE State Vector ==> ODE State Matrix (State Vector at every grid point)

%% "4" Options %%
% 1. (6x,1u) Constant RPS >> 6 States and 1 Control
% 2. (7x,1u) Constant RPS and delta as State and delta_dot as Control >> 6+1 States and 1 Control
% 3. (6x,2u) Normal >> 6 States[x_pos, y_pos, psi, u_vel, v_vel, r] and 2 Controls[delta, n]
% 4. (8x,2u) Real Controls as States(x) and Control Rates as Controls(u) >> 6+2 States[x_pos, y_pos, psi, u_vel, v_vel, r, delta, n] and 2 Controls[delta_dot, n_dot]

%% 
function dxdt_mat = stateMatrix_EO(t,x,u,p)
dxdt_mat = x; % Initialize

% n_const = 10; % Constant RPS (Uncomment only when using Option 1 or 2)

for i = 1:length(t)
    %%% Uncomment only one Option
%     dxdt_mat(1:6,i) = stateFunc_EO(t,x(1:6,i),[u(1,i), n_const],p); % Option 1 only
%     dxdt_mat(1:6,i) = stateFunc_EO(t,x(1:6,i),[x(7,i), n_const],p); % Option 2 only
%     dxdt_mat(1:6,i) = stateFunc_EO(t,x(1:6,i),u(1:2,i),p); % Option 3 only
    dxdt_mat(1:6,i) = stateFunc(t,x(1:6,i),x(7:8,i),p); % Option 4 only
%     

end

% For Option 4 only
dxdt_mat(7,:) = u(1,:); % Uncomment when using Option 2 or 4
dxdt_mat(8,:) = u(2,:); % Uncomment when using Option 4

end