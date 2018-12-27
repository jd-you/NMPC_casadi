clear;
clc;

import casadi.*;

opti = casadi.Opti();
p = params();

%% define start/end time and sampling rate
t_f = 3;
t_0 = 0;
N = 50;
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
    x_next = rk4(t(end), x(:, i), u(:, i), t_s);
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

%% solve
opti.solver('ipopt');
sol = opti.solve();

%% plot result
x_solution = full(sol.value(x)) ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution = full(sol.value(u)) ./ [p.scal_u1; p.scal_u2];

% plot in q1-q2-plane


hold on;
grid on;

subplot(4, 1, 1);
plot(t, x_solution(1, :));
xlabel('t/s');
ylabel('q1/rad');

subplot(4, 1, 2);
plot(t, x_solution(2, :));
xlabel('t/s');
ylabel('q2/rad');

subplot(4, 1, 3);
plot(t, x_solution(3, :));
xlabel('t/s');
ylabel('w1/rad/s');

subplot(4, 1, 4);
plot(t, x_solution(4, :));
xlabel('t/s');
ylabel('w2/rad/s');
suptitle('q1-q2-plane');

% plot in x-y-plane
figure;
hold on;
grid on;
x_ = p.l1 .* cos(x_solution(1, :)) + p.l2 .* cos(x_solution(1, :) + x_solution(2, :));
y_ = p.l1 .* sin(x_solution(1, :)) + p.l2 .* sin(x_solution(1, :) + x_solution(2, :));
subplot(2, 1, 1);
plot(t, x_);
xlabel('t/s');
ylabel('x/m');

subplot(2, 1, 2);
plot(t, y_);
xlabel('t/s');
ylabel('x/m');
suptitle('x-y-plane');

% plot inputs
figure;
hold on;
grid on;

subplot(2, 1, 1);
stairs(t(2:length(t)), u_solution(1, :));
xlabel('t/s');
ylabel('u1/Nm');
subplot(2, 1, 2);

stairs(t(2:length(t)), u_solution(2, :));
xlabel('t/s');
ylabel('u2/Nm');
suptitle('inputs');
