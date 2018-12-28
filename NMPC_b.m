clear;
clc;

import casadi.*;
p = params();

%% define OCP (horizon, J, sampling rate)
opti = casadi.Opti();

t_horizon = 1;
t_0 = 0;
N = 100;
t_s = (t_horizon - t_0) / N;
t_opt = 3;

x = opti.variable(4, N + 1);
u = opti.variable(2, N);

J = 0;
Q = 1*eye(4);
R = 0*eye(2);

x_0 = opti.parameter(4, 1);
x_end = p.x_end;

for i = 1:N
    x_next = euler_robot((i - 1) * t_s, x(:, i), u(:, i), t_s * t_opt);
    opti.subject_to(x(:, i + 1) == x_next);
    
    J = J + u(:, i)' * R * u(:, i) + (x(:, i + 1) - x_end)' * Q * (x(:, i + 1) - x_end);
end

opti.subject_to(x(:, 1) == x_0);
opti.subject_to(x(:, end) == x_end);
opti.subject_to(p.w_lb <= x(3, :) <= p.w_ub);
opti.subject_to(p.w_lb <= x(4, :) <= p.w_ub);
opti.subject_to(p.scal_u1 * p.u_lb <= u(1, :) <= p.scal_u1 * p.u_ub);
opti.subject_to(p.scal_u2 * p.u_lb <= u(2, :) <= p.scal_u2 * p.u_ub);

opti.subject_to(u(:, end) == 0);


opti.minimize(J);
opti.solver('ipopt');

%% NMPC
t_end = 5;
N_nmpc = t_end / t_s;

x_guess = [];
u_guess = [];
opti.set_value(x_0, p.x_0);

x_ = p.x_0;

x_result = [x_];
u_result = [];
t = [0];

for i = 1:N_nmpc
    if i == 1
        opti.set_initial(x, repmat([0; 0; 0; 0], 1, N + 1));
        opti.set_initial(u, repmat([0; 0], 1, N));
    else
        opti.set_initial(x, x_guess);
        opti.set_initial(u, u_guess);
    end
    
    sol = opti.solve();
    
    x_guess = [full(sol.value(x(:, 2:end))), [0; 0; 0; 0]];
    u_guess = [full(sol.value(u(:, 2:end))), [0; 0]];
    
    
    u_ = full(sol.value(u(:, 1)));
    x_ = euler_robot((i - 1) * t_s, x_, u_, t_s * t_opt);
    
    opti.set_value(x_0, x_);
    
    x_result = [x_result, x_];
    u_result = [u_result, u_];
    t = [t, i * t_s];
end

%% plot
x_solution = x_result ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution = u_result ./ [p.scal_u1; p.scal_u2];

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