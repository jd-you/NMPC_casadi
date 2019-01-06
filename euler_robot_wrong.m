function x_next = euler(t, x, u, h, t_opt)
    if (nargin < 5)
        t_opt = 1;
    end
    
    x_next = x + h .* ROBOT_ode_wrong(x, u, t, t_opt);
end