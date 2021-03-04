function [x0_low, x0_upp, xF_low, xF_upp, x_low, x_upp, u_low, u_upp] = assignXUBounds(ManoMode, xWP0_low, xWP0_upp, xWPF_low, xWPF_upp, xPath_low, xPath_upp,  uPath_low, uPath_upp, uRatePath_low, uRatePath_upp)  

%% State Bounds
    switch ManoMode
        case 'x6u1'
            x0_low = xWP0_low;
            x0_upp = xWP0_upp;
            xF_low = xWPF_low;
            xF_upp = xWPF_upp;
            x_low  = xPath_low;
            x_upp  = xPath_upp;

            u_low = uPath_low(1);
            u_upp = uPath_upp(1);

        case 'x7u1'
            x0_low = [xWP0_low; 0];
            x0_upp = [xWP0_upp; 0];
            xF_low = [xWPF_low; uPath_low(1)];
            xF_upp = [xWPF_upp; uPath_upp(1)];
            x_low  = [xPath_low; uPath_low(1)];
            x_upp  = [xPath_upp; uPath_upp(1)];

            u_low = uRatePath_low(1);
            u_upp = uRatePath_upp(1);

        case 'x6u2'
            x0_low = xWP0_low;
            x0_upp = xWP0_upp;
            xF_low = xWPF_low;
            xF_upp = xWPF_upp;
            x_low  = xPath_low;
            x_upp  = xPath_upp;

            u_low = uPath_low;
            u_upp = uPath_upp;

        case 'x8u2'
            x0_low = [xWP0_low; uPath_low];
            x0_upp = [xWP0_upp; uPath_upp];
            xF_low = [xWPF_low; uPath_low];
            xF_upp = [xWPF_upp; uPath_upp];
            x_low  = [xPath_low; uPath_low];
            x_upp  = [xPath_upp; uPath_upp];

            u_low = uRatePath_low;
            u_upp = uRatePath_upp;
    end
