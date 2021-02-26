%% State Matrix Function for 3DOF Esso Osaka(EO) MMG Model %%
% This function generates a state matrix of the size = (no_state, no_grid)
% ODE State Vector ==> ODE State Matrix (State Vector at every grid point)

%% (6x,2u) 6 States and 2 Controls
% State Vector:     x = [x_pos, y_pos, psi, u_vel, v_vel, r]
% Control Vector:   u = [delta, n]

%% 
function dxdt_mat = stateMatrix_EO_x6u2(t,x,u,p)

dxdt_mat = x; % Initialize
for i = 1:length(t)
    dxdt_mat(1:6,i) = stateFunc_EO(t,x(1:6,i),u(1:2,i),p);
end

end