clear;
clc;

[x_opt_ocp, u_opt_ocp, t_ocp] = ocp_b(50, 'euler');
[x_nmpc, u_nmpc, t_nmpc] = nmpc(0, 0, 0); % nmpc without model error, noise.
[x_nmpc_with_error, u_nmpc_with_error, t_nmpc_with_error] = nmpc(1, 0, 0); % nmpc with model error, but without noise

%% plot q1-q2-plane
p = params();
x_solution_ocp = x_opt_ocp ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_ocp = u_opt_ocp ./ [p.scal_u1; p.scal_u2];

x_solution_nmpc = x_nmpc ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_nmpc = u_nmpc ./ [p.scal_u1; p.scal_u2];

x_solution_nmpc_with_error = x_nmpc_with_error ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_nmpc_with_error = u_nmpc_with_error ./ [p.scal_u1; p.scal_u2];


figure;
fig1 = gcf;
fig1.PaperUnits = 'inches';
fig1.PaperPosition = [0, 0, 8, 8];

subplot(2, 2, 1);
axis([0, 3, -5.5, 2]);
hold on;
grid on;
plot(t_ocp, x_solution_ocp(1, :), 'LineWidth', 2);
plot(t_nmpc, x_solution_nmpc(1, :), 'LineWidth', 2);
plot(t_nmpc_with_error, x_solution_nmpc_with_error(1, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('q1/rad');
legend('OCP', 'NMPC', 'NMPC with error','Location','southeast');

subplot(2, 2, 2);
axis([0, 3, -4.5, 1]);
hold on;
grid on;
plot(t_ocp, x_solution_ocp(2, :), 'LineWidth', 2);
plot(t_nmpc, x_solution_nmpc(2, :), 'LineWidth', 2);
plot(t_nmpc_with_error, x_solution_nmpc_with_error(2, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('q2/rad');
legend('OCP', 'NMPC', 'NMPC with error','Location','southeast');

subplot(2, 2, 3);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_ocp, x_solution_ocp(3, :), 'LineWidth', 2);
plot(t_nmpc, x_solution_nmpc(3, :), 'LineWidth', 2);
plot(t_nmpc_with_error, x_solution_nmpc_with_error(3, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('w1/rad/s');
legend('OCP', 'NMPC', 'NMPC with error','Location','southeast');

subplot(2, 2, 4);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_ocp, x_solution_ocp(4, :), 'LineWidth', 2);
plot(t_nmpc, x_solution_nmpc(4, :), 'LineWidth', 2);
plot(t_nmpc_with_error, x_solution_nmpc_with_error(4, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('w2/rad/s');
suptitle('q1-q2-plane');
legend('OCP', 'NMPC', 'NMPC with error','Location','southeast');

%print('model_q1q2','-dpng','-r0');
%% plot x-y-plane

x_rk4 = p.l1 .* cos(x_solution_ocp(1, :)) + p.l2 .* cos(x_solution_ocp(1, :) + x_solution_ocp(2, :));
y_rk4 = p.l1 .* sin(x_solution_ocp(1, :)) + p.l2 .* sin(x_solution_ocp(1, :) + x_solution_ocp(2, :));

x_heun = p.l1 .* cos(x_solution_nmpc(1, :)) + p.l2 .* cos(x_solution_nmpc(1, :) + x_solution_nmpc(2, :));
y_heun = p.l1 .* sin(x_solution_nmpc(1, :)) + p.l2 .* sin(x_solution_nmpc(1, :) + x_solution_nmpc(2, :));

x_euler = p.l1 .* cos(x_solution_nmpc_with_error(1, :)) + p.l2 .* cos(x_solution_nmpc_with_error(1, :) + x_solution_nmpc_with_error(2, :));
y_euler = p.l1 .* sin(x_solution_nmpc_with_error(1, :)) + p.l2 .* sin(x_solution_nmpc_with_error(1, :) + x_solution_nmpc_with_error(2, :));

figure;
fig2 = gcf;
fig2.PaperUnits = 'inches';
fig2.PaperPosition = [0, 0, 8, 8];

subplot(2, 1, 1);
axis([0, 3, -1, 1]);
hold on;
grid on;
plot(t_ocp, x_rk4, 'LineWidth', 2);
plot(t_nmpc, x_heun, 'LineWidth', 2);
plot(t_nmpc_with_error, x_euler, 'LineWidth', 2);
xlabel('t/s');
ylabel('x/m');
legend('OCP', 'NMPC', 'NMPC with error','Location','southeast');

subplot(2, 1, 2);
axis([0, 3, -0.5, 1]);
hold on;
grid on;
plot(t_ocp, y_rk4, 'LineWidth', 2);
plot(t_nmpc, y_heun, 'LineWidth', 2);
plot(t_nmpc_with_error, y_euler, 'LineWidth', 2);
xlabel('t/s');
ylabel('y/m');
legend('OCP', 'NMPC', 'NMPC with error','Location','southeast');
suptitle('x-y-plane');

% print('model_xy','-dpng','-r0');
%% plot inputs
figure;
fig3 = gcf;
fig3.PaperUnits='inches';
fig3.PaperPosition = [0 0 8 8];
subplot(2, 1, 1);

axis([0, 3, -1000, 1200]);
hold on;
grid on;
stairs(t_ocp(2:length(t_ocp)), u_solution_ocp(1, :), 'LineWidth', 2);
stairs(t_nmpc(2:length(t_nmpc)), u_solution_nmpc(1, :), 'LineWidth', 2);
stairs(t_nmpc_with_error(2:length(t_nmpc_with_error)), u_solution_nmpc_with_error(1, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('u1/Nm');
legend('OCP', 'NMPC', 'NMPC with error');

subplot(2, 1, 2);
axis([0, 3, -1000, 500]);
hold on;
grid on;
stairs(t_ocp(2:length(t_ocp)), u_solution_ocp(2, :), 'LineWidth', 2);
stairs(t_nmpc(2:length(t_nmpc)), u_solution_nmpc(2, :), 'LineWidth', 2);
stairs(t_nmpc_with_error(2:length(t_nmpc_with_error)), u_solution_nmpc_with_error(2, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('u2/Nm');
legend('OCP', 'NMPC', 'NMPC with error');
suptitle('inputs');
% print('model_input','-dpng','-r0');