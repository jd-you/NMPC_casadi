function dx = robot_ode(t, x, u, t_time_optimal)
    [PAR, ~, SCu] = parameters;
    b1 = PAR.b1;
    b2 = PAR.b2;
    b3 = PAR.b3;
    b4 = PAR.b4;
    b5 = PAR.b5;
    c1 = PAR.c1;
    g1 = PAR.g1;
    g2 = PAR.g2;
    l1 = PAR.l1;
    l2 = PAR.l2;
    
    q1 = x(1,1);
    w1 = x(2,1);
    q2 = x(3,1);
    w2 = x(4,1);
    
    u1 = u(1);
    u2 = u(2);
    
    B = [b1 + b2 * cos(q2), b3 + b4 * cos(q2); b3 + b4 * cos(q2), b5];
    C = -c1 * sin(q2) .* [w1, w1 + w2; -w1, 0]; % -w2
    g = [g1 * cos(q1) + g2 * cos(q1 + q2); g2 * cos(q1 + q2)];
    
    dq1 = w1;
    dq2 = w2;
    dw = B^-1 * (u ./ SCu - g - C * [w1; w2]);
    dw1 = dw(1);
    dw2 = dw(2);
    
    dx = t_time_optimal * [dq1; dw1; dq2; dw2];
end