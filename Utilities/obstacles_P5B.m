function obs = obstacles_P5B

%% %%%%%%% Obstacles for Collision Avoidance %%%%%%%%%%%%%%%%%%
obs = [];
%% Rectangles [x0, y0, a, b, psi]
rect(1,:) = [0, -10.35, 15, 20, 0]; % Lower Vertical
% rect(2,:) = [-7.5, -16.35, 30, 8, 0]; % Lower Horizontal
% rect(3,:) = [-9.5, -10.35, 16.9706, 11.314, deg2rad(45)]; % Lower Inclined
% 
% rect(4,:) = [-7.5, 9.8, 30, 8, 0]; % Upper Horizontal
% rect(5,:) = [-21, 5.3, 3, 17, 0]; % Upper Vertical
% rect(6,:) = [-14, 5.3, 19.7990, 4.2430, deg2rad(45)]; % Upper Inclined

obs.rect  = rect;

%% Circles [x0, y0, rad]
% circ(1,:) = [-45, -5, 5];
% obs.circ  = circ;

%% Ellipses [x0, y0, a, b, psi, vel]
% elli(1,:) = [-30, 0, 10, 4, pi/2, 0];
% obs.elli  = elli;

%% Soft Rectangles [x0, y0, a, b, psi, vel, x_exponent, y_exponent]
% sRect(1,:) =  [5, 0.15, 2.5, 0.5, 0, 0, 4, 2];
% sRect(2,:) = [-5, 0.15, 2.5, 0.5, 0, 0, 4, 2];
% 
% obs.sRect  = sRect;



end
