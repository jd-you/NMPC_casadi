function p = params()
    p = struct;
    
    %% definition of system paramters
    p.b1 = 200.0;
    p.b2 = 50.0;
    p.b3 = 23.5;
    p.b4 = 25.0;
    %p.b5 = 122.5;
    p.b5 = p.b3;
    p.c1 = -25.0;
    p.g1 = 784.8;
    p.g2 = 245.3;
    p.l1 = 0.5;
    p.l2 = 0.5;
    
    %% scaling factors
    p.scal_q1 = 1;
    p.scal_q2 = 1;
    p.scal_w1 = 1;
    p.scal_w2 = 1;
    p.scal_u1 = 1e-2;
    p.scal_u2 = 1e-2;
    
    %% bound
    p.u_lb = -1000.0;
    p.u_ub = 1000.0;
    p.w_lb = -3 * pi / 2;
    p.w_ub = 3 * pi / 2;
    
    %% initial state
    p.x_0 = [-5; -4; 0; 0];
    p.x_end = [pi / 2; 0; 0; 0];
    
end