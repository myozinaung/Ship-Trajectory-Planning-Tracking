%% State Matrix Function for 3DOF Esso Osaka(EO) MMG Model %%
% This function generates a state matrix of the size = (no_state, no_grid)
% ODE State Vector ==> ODE State Matrix (State Vector at every grid point)

%% (6x,1u) Constant RPS >> 6 States and 1 Control
% State Vector:     x = [x_pos, y_pos, psi, u_vel, v_vel, r]
% Control Vector:   u = [delta]

%% 
function dxdt_mat = stateMatrix_EO_6x1u(t,x,u,p)

dxdt_mat = x; % Initialize
n_const = 10; % Constant RPS
for i = 1:length(t)
    dxdt_mat(1:6,i) = stateFunc(t,x(1:6,i),[u(1,i), n_const],p);
end

end