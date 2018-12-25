function x_next_int = rk4(x_int, u_int, t_int, h, t_time_optimal)
    if (nargin < 5)
        t_time_optimal = 1;
    end

    k1 = robot_ode(t_int, x_int, u_int, t_time_optimal);
    k2 = robot_ode(t_int + h / 2, x_int + k1 * h / 2, u_int, t_time_optimal);
    k3 = robot_ode(t_int + h / 2, x_int + k2 * h / 2, u_int, t_time_optimal);
    k4 = robot_ode(t_int + h, x_int + k3 * h, u_int, t_time_optimal);
    
    x_next_int = x_int + (k1 + 2 * k2 + 2 * k3 + k4) * h / 6;
end