function x_next = rk4(t, x, u, h, t_opt)
    if (nargin < 5)
        t_opt = 1;
    end
    
    k1 = ROBOT_ode(x, u, t, t_opt);
    k2 = ROBOT_ode(x + k1 * h / 2, u, t + h / 2, t_opt);
    k3 = ROBOT_ode(x + k2 * h / 2, u, t + h / 2, t_opt);
    k4 = ROBOT_ode( x + k3 * h, u, t + h, t_opt);
    x_next = x + (k1 + 2 * k2 + 2 * k3 + k4) * h / 6;
end