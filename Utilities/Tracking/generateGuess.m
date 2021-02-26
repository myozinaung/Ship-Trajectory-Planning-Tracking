function [t_guess, z_guess, u_guess] = generateGuess(t_ref, t_end, soln, soln_now, Type)

switch Type
    case 'previous'
        % Previous Solution as Guess (this is fastest, but if last solution is infeasible(NLP) that guess may not be good)
        t_guess = soln_now.grid.time;
        z_guess = soln_now.grid.state; % move one step forward --> slower
        u_guess = soln_now.grid.control; % 
        
    case 'offline'        
%         % Trimming and Interpolation of Initial Soultion(Fine Mesh) as Guess (Slower)
        t_interp = linspace(t_ref(1), t_end, length(soln_now.grid.time));
        t_guess  = soln_now.grid.time; % alternative = (t_guess - t_now);
        z_guess  = soln(end).interp.state(t_interp); % need to move forward 
        u_guess  = soln(end).interp.control(t_interp);
        
    case 'auto'
        t_guess = soln_now.grid.time;
        if soln_now.info.exitFlag < 0
            t_interp = linspace(t_ref(1), t_end, length(soln_now.grid.time));
            z_guess  = soln(end).interp.state(t_interp); % need to move forward 
            u_guess  = soln(end).interp.control(t_interp);
        else            
            z_guess = soln_now.grid.state; % move one step forward --> slower
            u_guess = soln_now.grid.control; %  
        end
            
    otherwise
        t_guess = soln_now.grid.time;
        z_guess = soln_now.grid.state; % move one step forward --> slower
        u_guess = soln_now.grid.control;     
end