function ocp_new = modifyXU0(ManoModes, constRPS, soln_last, ocp_now)

%% First, extract [xF, uF, uRateF] from previous solution
t = soln_last(end).grid.time;
z = soln_last(end).grid.state;
u = soln_last(end).grid.control;

switch ManoModes(1)
    case 'x6u1'
        xF = z(1:6,end);
        uF = [u(1,end); constRPS];
        deltaRateF = (u(1,end)-u(1,end-1))/(t(end)-t(end-1));
        uRateF = [deltaRateF; 0];

    case 'x7u1'
        xF = z(1:6,end);
        uF = [z(7,end); constRPS];
        uRateF = [u(1,end); 0];

    case 'x6u2'
        xF = z(1:6,end);
        uF = u(1:2,end);
        deltaRateF = (u(1,end)-u(1,end-1))/(t(end)-t(end-1));
        rpsRateF   = (u(2,end)-u(2,end-1))/(t(end)-t(end-1));
        uRateF = [deltaRateF; rpsRateF];        

    case 'x8u2'
        xF = z(1:6,end);
        uF = z(7:8,end);
        uRateF = u(1:2,end); 

end

%% Next, assign the current OCP with new initial conditions
ocp_new = ocp_now;

ocp_new.bounds.initialTime.low = t(end);
ocp_new.bounds.initialTime.upp = t(end);
ocp_new.bounds.finalTime.low   = t(end);
ocp_new.guess.time(1) = t(end);
switch ManoModes(2)
    case 'x6u1'
        ocp_new.bounds.initialState.low = xF;
        ocp_new.bounds.initialState.upp = ocp_new.bounds.initialState.low;
        
    case 'x7u1'
        ocp_new.bounds.initialState.low = [xF; uF(1)];
        ocp_new.bounds.initialState.upp = ocp_new.bounds.initialState.low;

    case 'x6u2'
        ocp_new.bounds.initialState.low = xF;
        ocp_new.bounds.initialState.upp = ocp_new.bounds.initialState.low;

    case 'x8u2'
        ocp_new.bounds.initialState.low = [xF; uF];
        ocp_new.bounds.initialState.upp = ocp_new.bounds.initialState.low;

end

    ocp_new.guess.state(:,1)   = ocp_new.bounds.initialState.low;
    % Replace NaN with 0 in case operation on inf results NaN
    ocp_new.guess.state(isnan(ocp_new.guess.state)) = 0; 

end
