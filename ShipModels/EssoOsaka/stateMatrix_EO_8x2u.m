%% State Matrix Function for 3DOF Esso Osaka(EO) MMG Model %%
% This function generates a state matrix of the size = (no_state, no_grid)
% ODE State Vector ==> ODE State Matrix (State Vector at every grid point)

%% (8x,2u) Real Controls as States(x) and Control Rates as Controls(u)
% State Vector:     x = [x_pos, y_pos, psi, u_vel, v_vel, r, delta, n]
% Control Vector:   u = [delta_dot, n_dot]

%% 
function dxdt_mat = stateMatrix_EO_8x2u(t,x,u,p)

dxdt_mat = x; % Initialize
for i = 1:length(t)
    dxdt_mat(1:6,i) = stateFunc(t,x(1:6,i),x(7:8,i),p);
end
dxdt_mat(7,:) = u(1,:);
dxdt_mat(8,:) = u(2,:);

end