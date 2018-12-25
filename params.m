function p = params()
    p = struct;
    
    %% definition of system paramters
    p.b1 = 200.0;
    p.b2 = 50.0;
    p.b3 = 23.5;
    p.b4 = 25.0;
    p.b5 = 122.5;
    % p.b5 = p.b3;
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
    p.scal_u1 = 1;
    p.scal_u2 = 1;
    
    %% 

end