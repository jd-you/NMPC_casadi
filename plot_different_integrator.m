clear;
clc;

[x_opt_100_rk4, u_opt_100_rk4, t] = ocp_b(100, 'rk4');
[x_opt_100_heun, u_opt_100_heun, t] = ocp_b(100, 'heun');
[x_opt_100_euler, u_opt_100_euler, t] = ocp_b(100, 'euler');

p = params();
x_solution_rk4 = x_opt_100_rk4 ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_rk4 = u_opt_100_rk4 ./ [p.scal_u1; p.scal_u2];

x_solution_heun = x_opt_100_heun ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_heun = u_opt_100_heun ./ [p.scal_u1; p.scal_u2];

x_solution_euler = x_opt_100_euler ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_euler = u_opt_100_euler ./ [p.scal_u1; p.scal_u2];

%% plot q1-q2-plane
figure;
fig1 = gcf;
fig1.PaperUnits = 'inches';
fig1.PaperPosition = [0, 0, 8, 8]

subplot(2, 2, 1);
axis([0, 3, -5.5, 2]);
hold on;
grid on;
plot(t, x_solution_rk4(1, :), 'LineWidth', 2);
plot(t, x_solution_heun(1, :), 'LineWidth', 2);
plot(t, x_solution_euler(1, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('q1/rad');
legend('RK4', 'Heun', 'Euler','Location','southeast');

subplot(2, 2, 2);
axis([0, 3, -4.5, 1]);
hold on;
grid on;
plot(t, x_solution_rk4(2, :), 'LineWidth', 2);
plot(t, x_solution_heun(2, :), 'LineWidth', 2);
plot(t, x_solution_euler(2, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('q2/rad');
legend('RK4', 'Heun', 'Euler','Location','southeast');

subplot(2, 2, 3);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t, x_solution_rk4(3, :), 'LineWidth', 2);
plot(t, x_solution_heun(3, :), 'LineWidth', 2);
plot(t, x_solution_euler(3, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('w1/rad/s');
legend('RK4', 'Heun', 'Euler','Location','southeast');

subplot(2, 2, 4);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t, x_solution_rk4(4, :), 'LineWidth', 2);
plot(t, x_solution_heun(4, :), 'LineWidth', 2);
plot(t, x_solution_euler(4, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('w2/rad/s');
suptitle('q1-q2-plane');
legend('RK4', 'Heun', 'Euler','Location','southeast');
%print('different_integrator_q1q2','-dpng','-r0'); 

%% plot x-y-plane
x_rk4 = p.l1 .* cos(x_solution_rk4(1, :)) + p.l2 .* cos(x_solution_rk4(1, :) + x_solution_rk4(2, :));
y_rk4 = p.l1 .* sin(x_solution_rk4(1, :)) + p.l2 .* sin(x_solution_rk4(1, :) + x_solution_rk4(2, :));

x_heun = p.l1 .* cos(x_solution_heun(1, :)) + p.l2 .* cos(x_solution_heun(1, :) + x_solution_heun(2, :));
y_heun = p.l1 .* sin(x_solution_heun(1, :)) + p.l2 .* sin(x_solution_heun(1, :) + x_solution_heun(2, :));

x_euler = p.l1 .* cos(x_solution_euler(1, :)) + p.l2 .* cos(x_solution_euler(1, :) + x_solution_euler(2, :));
y_euler = p.l1 .* sin(x_solution_euler(1, :)) + p.l2 .* sin(x_solution_euler(1, :) + x_solution_euler(2, :));
figure;
fig2 = gcf;
fig2.PaperUnits = 'inches';
fig2.PaperPosition = [0, 0, 8, 8];

subplot(2, 1, 1);
axis([0, 3, -1, 1]);
hold on;
grid on;
plot(t, x_rk4, 'LineWidth', 2);
plot(t, x_heun, 'LineWidth', 2);
plot(t, x_euler, 'LineWidth', 2);
xlabel('t/s');
ylabel('x/m');
legend('RK4', 'Heun', 'Euler','Location','southeast');

subplot(2, 1, 2);
axis([0, 3, -0.5, 1]);
hold on;
grid on;
plot(t, y_rk4, 'LineWidth', 2);
plot(t, y_heun, 'LineWidth', 2);
plot(t, y_euler, 'LineWidth', 2);
xlabel('t/s');
ylabel('y/m');
suptitle('x-y-plane');
legend('RK4', 'Heun', 'Euler','Location','southeast');
%print('different_integrator_xy','-dpng','-r0');

%% plot inputs
figure;
fig3 = gcf;
fig3.PaperUnits='inches';
fig3.PaperPosition = [0 0 8 8];

subplot(2, 1, 1);
axis([0, 3, -1000, 1000]);
hold on;
grid on;
stairs(t(2:length(t)), u_solution_rk4(1, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_heun(1, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_euler(1, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('u1/Nm');
legend('RK4', 'Heun', 'Euler');

subplot(2, 1, 2);
axis([0, 3, -1000, 1000]);
hold on;
grid on;
stairs(t(2:length(t)), u_solution_rk4(2, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_heun(2, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_euler(2, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('u2/Nm');
suptitle('inputs');
legend('RK4', 'Heun', 'Euler');
% print('different_integrator_input','-dpng','-r0');