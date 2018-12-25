import casadi.*;

clear;
clc;

robot_opt = casadi.Opti();
%% define starting/end time and sample points
tf = 10;
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

R = [0.1 0;
    0 0.1];

x0 = [-5; 0; -4; 0];
xend = [pi / 2; 0; 0; 0];

for i = 1:N
    x_next = rk4(x(:,i), u(:,i), t, ts);
    
    t = [t, t(end) + ts];
    
    robot_opt.subject_to(x(:,i+1) == x_next);
    
    J = J + u(:,i)' * R * u(:,i) + (x(:,i + 1) - xend)' * Q * (x(:,i + 1) - xend) ;
end

[~,CON,~] = parameters;

robot_opt.minimize(J);
robot_opt.subject_to(x(:,1) == x0);
robot_opt.subject_to(x(:,end) == xend);
robot_opt.subject_to(CON.w(1) <= x(2,:) <= CON.w(2));
robot_opt.subject_to(CON.w(1) <= x(4,:) <= CON.w(2));
robot_opt.subject_to(CON.u(1) <= u <= CON.u(2));

robot_opt.solver('ipopt');
sol = robot_opt.solve();

sol_x = full(sol.value(x));
sol_u = full(sol.value(u));

for i = 1:4
    subplot(4,1,i);
    hold on;
    plot(t, sol_x(i,:));
end

figure;

for i = 1:2
    subplot(2,1,i);
    hold on;
    plot(t(1:(length(t) - 1)), sol_u(i,:));
end
figure;
%plot_gif(sol_x, 0.03);



