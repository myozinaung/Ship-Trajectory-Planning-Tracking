%% State Matrix Function for 3DOF Esso Osaka(EO) MMG Model %%
% This function generates a state matrix of the size = (no_state, no_grid)
% ODE State Vector ==> ODE State Matrix (State Vector at every grid point)

%% (7x,1u) Constant RPS and delta as State and delta_dot as Control >> 6+1 States and 1 Control
% State Vector:     x = [x_pos, y_pos, psi, u_vel, v_vel, r, delta]
% Control Vector:   u = [delta_dot]

%% 
function dxdt_mat = stateMatrix_EO_x7u1(t,x,u,p)

dxdt_mat = x; % Initialize
n_const = 10; % Constant RPS
for i = 1:length(t)
    dxdt_mat(1:6,i) = stateFunc_EO(t,x(1:6,i),[x(7,i), n_const],p);
end
dxdt_mat(7,:) = u(1,:);

end