%% Obstacles for Collision Avoidance %%
function obs = obstacles_COLREGs
obs = [];

%% Ellipses [x0, y0, a, b, psi, vel]
L    = 3.0; % Target Ship(TS) length

%  [x0, y0, psi, vel]
TS = [100, 0, pi, 0.3]; % for Head-on
% TS = [50, -50, deg2rad(90), 0.3]; % for Crossing

%% Coldwell's Domain for Head-on and Crossing
dist = sqrt((1.1*L)^2 + (0.75*L)^2);
phi  = atan(0.75/1.1);
elli(1,:) = [TS(1)+dist*cos(TS(3)-phi), TS(2)+dist*sin(TS(3)-phi), 5*L, 2.5*L, TS(3), TS(4)]; 

%% Coldwell's Domain for Overtaking
% elli(1,:) = [TS(1), TS(2), 6*L, 1.75*L, TS(3), TS(4)]; 

%%
obs.elli  = elli;

end