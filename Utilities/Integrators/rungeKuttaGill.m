function x_next_vec = rungeKuttaGill(t, x, u, p, dt, f)

    k1_vec  = f(t, x, u, p);
    tmp_vec = x + 0.5*dt.*k1_vec;

    k2_vec  = f(t+0.5*dt, tmp_vec, u, p);
    tmp_vec = x + dt.*0.5.*(sqrt(2)-1).*k1_vec + dt.*(1-(1/sqrt(2)))*k2_vec;

    k3_vec  = f(t+0.5*dt, tmp_vec, u, p);
    tmp_vec = x - dt.*0.5.*sqrt(2).*k2_vec + dt.*(1+(1/sqrt(2))).*k3_vec;

    k4_vec  = f(t+dt, tmp_vec, u, p);
    x_next_vec = x + dt.*(k1_vec+(2-sqrt(2)).*k2_vec+(2+sqrt(2)).*k3_vec+k4_vec)./6;

end