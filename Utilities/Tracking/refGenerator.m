function [x_ref, u_ref, t_ref] = refGenerator(t_now, t_end, grid_size0, soln, x, t_RefTraj, z_RefTraj, Type)

switch Type 
    case 'position' % POSITION-based Reference Generation %%
        inc = 1; % Need tuning for optimal tracking
        xpos = x(1);
        ypos = x(2);

        xpos_RefTraj = z_RefTraj(1,:);
        ypos_RefTraj = z_RefTraj(2,:);

        distAll = sqrt((xpos_RefTraj-xpos).^2 + (ypos_RefTraj-ypos).^2);
        [~, idxMin] = min(distAll);
        idxMin = idxMin + inc; % increase some point ahead
        if idxMin >= length(xpos_RefTraj) % for final WP
            idxMin = length(xpos_RefTraj) - 1;
        end
        t_ref0 = t_RefTraj(idxMin);
        if t_ref0 >= t_end
            t_ref0 = t_end - 0.1;
        end
    
    case 'time' % TIME-based Reference Generation %%
        t_ref0 = t_now;
    otherwise
        t_ref0 = t_now;
end


t_ref = linspace(t_ref0, t_end, grid_size0);
x_ref = soln(end).interp.state(t_ref);
u_ref = soln(end).interp.control(t_ref);

end