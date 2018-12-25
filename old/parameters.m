function [PAR, CON, SCu] = parameters
    %% define system constants
    PAR.b1 = 200;
    PAR.b2 = 50;
    PAR.b3 = 23.5;
    PAR.b4 = 25;
    PAR.b5 = 122.5;
    PAR.c1 = -25;
    PAR.g1 = 784.8;
    PAR.g2 = 245.3;
    PAR.l1 = 0.5;
    PAR.l2 = 0.5;
    
    %% define Constraints
    CON.u = [-1000, 1000];
    CON.w = [-3 * pi / 2, 3 * pi / 2];
    
    %% define scaling factors
    SCu = 1e-2;
    
end