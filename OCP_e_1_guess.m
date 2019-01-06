function [x_solution, u_solution, t_opt_solution] = OCP_e_1_guess()
opti = casadi.Opti();
p = params();

%% define start/end time and sampling rate
t_f = 1;
t_0 = 0;
N = 50;
t_s = (t_f - t_0) / N;

%% define decision variables
x = opti.variable(4, N + 1);
u = opti.variable(2, N);
t_opt = opti.variable();

%% objective function 
J = t_opt;
t = t_0;

x_0 = p.x_0;
x_end = p.x_end;

for i = 1:N
    % continuity constraint
    x_next = euler_robot(t(end), x(:, i), u(:, i), t_s, t_opt);
    opti.subject_to(x(:,i+1) == x_next);
    t = [t, t(end) + t_s];
end

opti.minimize(J);

%% constraints
opti.subject_to(x(:, 1) == x_0);
opti.subject_to(x(:, end) == x_end);
opti.subject_to(p.w_lb <= x(3, :) <= p.w_ub);
opti.subject_to(p.w_lb <= x(4, :) <= p.w_ub);
opti.subject_to(p.scal_u1 * p.u_lb <= u(1, :) <= p.scal_u1 * p.u_ub);
opti.subject_to(p.scal_u2 * p.u_lb <= u(2, :) <= p.scal_u2 * p.u_ub);

opti.subject_to(t_opt > 0);

%% solve
opti.solver('ipopt');
sol = opti.solve();

%% plot result
x_solution = full(sol.value(x)) ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution = full(sol.value(u)) ./ [p.scal_u1; p.scal_u2];
t_opt_solution = full(sol.value(t_opt));


end