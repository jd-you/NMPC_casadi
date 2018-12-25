clear;
clc;


    %% get the parameters of the modell
    [~, CON, SCu] = parameters;
    umin = SCu .* [CON.u(1); CON.u(1)];
    umax = SCu .* [CON.u(2); CON.u(2)];
    wmin = [CON.w(1); CON.w(1)];
    wmax = [CON.w(2); CON.w(2)];
    
    nx = 4;
    nu = 2;
    
    %% build ocp (one iteration of NMPC) horizon, delta, J, beta
    import casadi.*
    ocp = casadi.Opti();

    T_horizon = 3;
    delta_t = 0.1;
    T_end = 3;
    X_0 = wrapTo2Pi([-5; 0; -4; 0]);
    X_end = [pi/2; 0; 0; 0];
    U_end = [0; 0];
    
    N_ocp = T_horizon / delta_t;
    
    X_ocp = ocp.variable(nx, N_ocp + 1);
    U_ocp = ocp.variable(nu, N_ocp);
    X_ocp_0 = ocp.parameter(nx, 1);
    
    J = 0;
    Q = 1e-2*diag([1,1,1,1]);
    R = 1e2*diag([1,1]);
    beta = 8e6;
    
    Q_2 = 1e-2*diag([1,1,1,1]);
    R_2 = 1e-2*diag([1,1]);
    beta_2 = 8e4;
    
    for i = 1:N_ocp
        X_next = rk4(X_ocp(:, i), U_ocp(:, i), delta_t * i, delta_t);
        ocp.subject_to(X_next == X_ocp(:, i + 1));
        
        dx = X_ocp(:, i + 1) - X_end;
            ocp.set_value(X_ocp_0, X_nmpc(:,i))
        du = U_ocp(:, i) - U_end;
        
        J = J + 0.5 * dx' * Q * dx + 0.5 * du' * R * du;
    end
    J = J + beta * (X_ocp(:, end) - X_end)' * (X_ocp(:, end) - X_end) + beta * du' * du;
    
    ocp.subject_to(repmat(umin, 1, N_ocp) <= U_ocp <= repmat(umax, 1, N_ocp));
    ocp.subject_to(repmat(wmin, 1 ,N_ocp + 1) <= [X_ocp(2, :); X_ocp(4, :)] <= repmat(wmax, 1, N_ocp + 1));
    ocp.subject_to(X_ocp(:, 1) == X_ocp_0);
    %ocp.subject_to(X_ocp(:, end) == X_end);
    % ocp.subject_to(U_ocp(:, end) == 0);
    ocp.minimize(J);
    ocp.solver('ipopt');
%     [ocp_1, X_ocp_0_1, U_ocp_1, X_ocp_1] = construct_ocp(T_horizon, delta_t, Q_1, R_1, beta_1, nx, nu, U_end, X_end, umin, umax, wmin, wmax);
%     [ocp_2, X_ocp_0_2, U_ocp_2, X_ocp_2] = construct_ocp(T_horizon, delta_t, Q_2, R_2, beta_2, nx, nu, U_end, X_end, umin, umax, wmin, wmax);
    ocp.set_value(X_ocp_0, X_0);
    sol = ocp.solve();
    x = full(sol.value(X_ocp))
    u = full(sol.value(U_ocp))
    
    
    %% NMPC
    N_nmpc = T_end / delta_t;
    
    X_nmpc = [X_0];
    U_nmpc  = [];
    for i = 1:N_nmpc
            if i ~= 1
                ocp.set_initial(X_ocp, full(sol.value(X_ocp)));
                ocp.set_initial(U_ocp, full(sol.value(U_ocp)));
            end
            ocp.set_value(X_ocp_0, X_nmpc(:,i));
            sol = ocp.solve();
            U = full(sol.value(U_ocp(:, 1)));
            X = rk4(X_nmpc(:,i), U, delta_t * i, delta_t);

        
        U_nmpc = [U_nmpc, U]
        X_nmpc = [X_nmpc, X]
    end
    