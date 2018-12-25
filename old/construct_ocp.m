function [ocp, X_ocp_0, U_ocp, X_ocp] = construct_ocp(T_horizon, delta_T, Q, R, beta, nx, nu, Uss, Xss, umin, umax, wmin, wmax)
    import casadi.*;
    ocp = casadi.Opti();
    
    N_ocp = T_horizon / delta_T;
    X_ocp = ocp.variable(nx, N_ocp + 1);
    U_ocp = ocp.variable(nu, N_ocp);
    X_ocp_0 = ocp.parameter(nx, 1);
    
    J = 0;
    for i = 1:N_ocp
        X_next = rk4(X_ocp(:, i), U_ocp(:, i), delta_T * i, delta_T);
        ocp.subject_to(X_next == X_ocp(:, i + 1));
        
        dx = X_ocp(:, i + 1) - Xss;
        du = U_ocp(:, i) - Uss;
        
        J = J + delta_T * (0.5 * dx' * Q * dx + 0.5 * du' * R * du);
    end
    %J = J + beta * (X_ocp(:, end) - Xss)' * (X_ocp(:, end) - Xss);
    
    ocp.subject_to(repmat(umin, 1, N_ocp) <= U_ocp <= repmat(umax, 1, N_ocp));
    ocp.subject_to(repmat(wmin, 1 ,N_ocp + 1) <= [X_ocp(2, :); X_ocp(4, :)] <= repmat(wmax, 1, N_ocp + 1));
    ocp.subject_to(X_ocp(:, 1) == X_ocp_0);
    ocp.subject_to(X_ocp(:, end) == Xss);
    % ocp.subject_to(U_ocp(:, end) == 0);
    ocp.minimize(J);
    ocp.solver('ipopt');