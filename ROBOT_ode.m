function dx = ROBOT_ode(x, u, t, t_opt)
    p = params;
    
    unscaled_x = x ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
    unscaled_u = u./ [p.scal_u1; p.scal_u2];
    
    B = [p.b1 + p.b2 * cos(unscaled_x(2)), p.b3 + p.b4 * cos(unscaled_x(2)); ...
         p.b3 + p.b4 * cos(unscaled_x(2)), p.b5];

    %C = -p.c1 * sin(unscaled_x(2)) .* [unscaled_x(3), unscaled_x(3) + unscaled_x(4); ...
     %                         -unscaled_x(3), 0];
    C = -p.c1 * sin(unscaled_x(2)) .* [unscaled_x(4), unscaled_x(3) + unscaled_x(4); ...
                              -unscaled_x(3), 0];
    
    g = [p.g1 * cos(unscaled_x(1)) + p.g2 * cos(unscaled_x(1) + unscaled_x(2)); ...
        p.g2 * cos(unscaled_x(1) + unscaled_x(2))];
    
    unscaled_dx = [unscaled_x(3); unscaled_x(4); B ^ (-1) * (unscaled_u - g - C * unscaled_x(3:4))];
    
    dx = unscaled_dx .* [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2] * t_opt;
end