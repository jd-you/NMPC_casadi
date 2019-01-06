function [x_opt, u_opt, t] = ocp_b(sampling_point, integrator_type)

import casadi.*;

opti = casadi.Opti();
p = params();

%% define start/end time and sampling rate
t_f = 3;
t_0 = 0;
N = sampling_point;
t_s = (t_f - t_0) / N;

%% define variables
x = opti.variable(4, N + 1);
u = opti.variable(2, N);

%% objective function
J = 0;
t = t_0;

Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

R = [1 0;
    0 1];

x_0 = p.x_0;
x_end = p.x_end;

for i = 1:N
    % continuity constraint
    if strcmp(integrator_type, 'rk4')
        x_next = rk4_robot(t(end), x(:, i), u(:, i), t_s);
    elseif strcmp(integrator_type, 'heun')
        x_next = heun_robot(t(end), x(:, i), u(:, i), t_s);
    elseif strcmp(integrator_type, 'euler')
        x_next = euler_robot(t(end), x(:, i), u(:, i), t_s);
    elseif strcmp(integrator_type, 'euler_wrong')
        x_next = euler_robot_wrong(t(end), x(:, i), u(:, i), t_s);
    end
    
    opti.subject_to(x(:,i+1) == x_next);
    t = [t, t(end) + t_s];
    
    % objective function
    J = J + u(:, i)' * R * u(:, i) + (x(:, i + 1) - x_end)' * Q * (x(:, i + 1) - x_end);
end

opti.minimize(J);

%% constraints
opti.subject_to(x(:, 1) == x_0);
opti.subject_to(x(:, end) == x_end);
opti.subject_to(p.w_lb <= x(3, :) <= p.w_ub);
opti.subject_to(p.w_lb <= x(4, :) <= p.w_ub);
opti.subject_to(p.scal_u1 * p.u_lb <= u(1, :) <= p.scal_u1 * p.u_ub);
opti.subject_to(p.scal_u2 * p.u_lb <= u(2, :) <= p.scal_u2 * p.u_ub);

%% initial guess
opti.set_initial(x, repmat([0;0;0;0], [1, N + 1]));
opti.set_initial(u, repmat([0;0],[1, N]));

%% solve
opti.solver('ipopt');
sol = opti.solve();

%% plot result
x_opt = full(sol.value(x));
u_opt = full(sol.value(u));
%plot_result(x_opt, u_opt, t);
end