import casadi.*;

clear;
clc;

robot_opt = casadi.Opti();
%% define starting/end time and sample points
t_opt = robot_opt.variable(1,1);
tf = 1;
t0 = 0;
N = 100;
ts = (tf - t0) / N;

%% define variables
x = robot_opt.variable(4, N + 1);
u = robot_opt.variable(2, N);
%% construct the objective function
J = 0;
t = t0;

Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

R = [1 0;
    0 1];

x0 = [-5 + 2 * pi; 0; -4 + 2 * pi; 0];
xend = [pi / 2; 0; 0; 0];
[~,CON,~] = parameters;
for i = 1:N
    x_next = rk4(x(:,i), u(:,i), t(end), ts, t_opt);
    
    t = [t, t(end) + ts];

    robot_opt.subject_to(x(:,i+1) == x_next);
end

J = t_opt;

robot_opt.minimize(J);
robot_opt.subject_to(x(:,1) == x0);
robot_opt.subject_to(x(:,end) == xend);
robot_opt.subject_to(CON.x(1) <= x(2,:) <= CON.x(2));
robot_opt.subject_to(CON.x(1) <= x(4,:) <= CON.x(2));
robot_opt.subject_to(CON.u(1) <= u(1,:) <= CON.u(2));
robot_opt.subject_to(CON.u(1) <= u(2,:) <= CON.u(2));

robot_opt.subject_to(0 <= t_opt <= 3);
robot_opt.set_initial(t_opt, 2);

robot_opt.set_initial(x,repmat(x0,1,N+1));
robot_opt.set_initial(u,repmat([1000;1000],1,N));
robot_opt.set_initial(t_opt,2);

robot_opt.solver('ipopt');
sol = robot_opt.solve();

sol_x = full(sol.value(x));
sol_u = full(sol.value(u));
sol_t = full(sol.value(t_opt));



options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
x_ode45_tol = [];
t_ode45_tol = [];
x0_ode45 = x0;

for i = 1:N
    tspan = [t(i), t(i+1)] .* sol_t;
    f = @(t_ode, x_ode)robot_ode(t_ode, x_ode, sol_u(:,i), 1);
    [t_ode45, x_ode45] = ode45( f, tspan, x0_ode45, options);
    x_ode45_tol = [x_ode45_tol, x_ode45'];
    t_ode45_tol = [t_ode45_tol, t_ode45'];
    x0_ode45 = x_ode45_tol(:,end);
end

for i = 1:4
    subplot(2,2,i);
    hold on;
    plot(sol_t * t, sol_x(i,:));
    plot(t_ode45_tol, x_ode45_tol(i,:));
end

figure;

for i = 1:2
    subplot(2,1,i);
    hold on;
    stairs(sol_t * t(1:(length(t) - 1)), sol_u(i,:));
    axis([0 3 -1500 1500]);
end
figure;
plot_gif(sol_x, 0.03);





