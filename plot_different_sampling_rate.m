clear;
clc;

[x_opt_10, u_opt_10, t_10] = ocp_b(10, 'euler');
[x_opt_50, u_opt_50, t_50] = ocp_b(50, 'euler');
[x_opt_100, u_opt_100, t_100] = ocp_b(100, 'euler');

p = params();
x_solution_10 = x_opt_10 ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_10 = u_opt_10 ./ [p.scal_u1; p.scal_u2];

x_solution_50 = x_opt_50 ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_50 = u_opt_50 ./ [p.scal_u1; p.scal_u2];

x_solution_100 = x_opt_100 ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_100 = u_opt_100 ./ [p.scal_u1; p.scal_u2];

%% plot q1-q2-plane
figure;
fig1 = gcf;
fig1.PaperUnits = 'inches';
fig1.PaperPosition = [0, 0, 8, 8];

subplot(2, 2, 1);
axis([0, 3, -5.5, 2]);
hold on;
grid on;
plot(t_10, x_solution_10(1, :), 'LineWidth', 2);
plot(t_50, x_solution_50(1, :), 'LineWidth', 2);
plot(t_100, x_solution_100(1, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('q1/rad');
legend('sampling: 10', 'sampling: 50', 'sampling: 100','Location','southeast');

subplot(2, 2, 2);
axis([0, 3, -4.5, 1]);
hold on;
grid on;
plot(t_10, x_solution_10(2, :), 'LineWidth', 2);
plot(t_50, x_solution_50(2, :), 'LineWidth', 2);
plot(t_100, x_solution_100(2, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('q2/rad');
legend('sampling: 10', 'sampling: 50', 'sampling: 100','Location','southeast');

subplot(2, 2, 3);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_10, x_solution_10(3, :), 'LineWidth', 2);
plot(t_50, x_solution_50(3, :), 'LineWidth', 2);
plot(t_100, x_solution_100(3, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('w1/rad/s');
legend('sampling: 10', 'sampling: 50', 'sampling: 100','Location','southeast');

subplot(2, 2, 4);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_10, x_solution_10(4, :), 'LineWidth', 2);
plot(t_50, x_solution_50(4, :), 'LineWidth', 2);
plot(t_100, x_solution_100(4, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('w2/rad/s');
suptitle('q1-q2-plane');
legend('sampling: 10', 'sampling: 50', 'sampling: 100','Location','southeast');

%print('different_samplingrate_q1q2','-dpng','-r0');
%% plot x-y-plane

x_rk4 = p.l1 .* cos(x_solution_10(1, :)) + p.l2 .* cos(x_solution_10(1, :) + x_solution_10(2, :));
y_rk4 = p.l1 .* sin(x_solution_10(1, :)) + p.l2 .* sin(x_solution_10(1, :) + x_solution_10(2, :));

x_heun = p.l1 .* cos(x_solution_50(1, :)) + p.l2 .* cos(x_solution_50(1, :) + x_solution_50(2, :));
y_heun = p.l1 .* sin(x_solution_50(1, :)) + p.l2 .* sin(x_solution_50(1, :) + x_solution_50(2, :));

x_euler = p.l1 .* cos(x_solution_100(1, :)) + p.l2 .* cos(x_solution_100(1, :) + x_solution_100(2, :));
y_euler = p.l1 .* sin(x_solution_100(1, :)) + p.l2 .* sin(x_solution_100(1, :) + x_solution_100(2, :));

figure;
fig2 = gcf;
fig2.PaperUnits = 'inches';
fig2.PaperPosition = [0, 0, 8, 8];

subplot(2, 1, 1);
axis([0, 3, -1, 1]);
hold on;
grid on;
plot(t_10, x_rk4, 'LineWidth', 2);
plot(t_50, x_heun, 'LineWidth', 2);
plot(t_100, x_euler, 'LineWidth', 2);
xlabel('t/s');
ylabel('x/m');
legend('sampling: 10', 'sampling: 50', 'sampling: 100','Location','southeast');

subplot(2, 1, 2);
axis([0, 3, -0.5, 1]);
hold on;
grid on;
plot(t_10, y_rk4, 'LineWidth', 2);
plot(t_50, y_heun, 'LineWidth', 2);
plot(t_100, y_euler, 'LineWidth', 2);
xlabel('t/s');
ylabel('y/m');
legend('sampling: 10', 'sampling: 50', 'sampling: 100','Location','southeast');
suptitle('x-y-plane');

%print('different_samplingrate_xy','-dpng','-r0');
%% plot inputs
figure;
fig3 = gcf;
fig3.PaperUnits='inches';
fig3.PaperPosition = [0 0 8 8];
subplot(2, 1, 1);

axis([0, 3, -1000, 1000]);
hold on;
grid on;
stairs(t_10(2:length(t_10)), u_solution_10(1, :), 'LineWidth', 2);
stairs(t_50(2:length(t_50)), u_solution_50(1, :), 'LineWidth', 2);
stairs(t_100(2:length(t_100)), u_solution_100(1, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('u1/Nm');
legend('sampling: 10', 'sampling: 50', 'sampling: 100');

subplot(2, 1, 2);
axis([0, 3, -1000, 1000]);
hold on;
grid on;
stairs(t_10(2:length(t_10)), u_solution_10(2, :), 'LineWidth', 2);
stairs(t_50(2:length(t_50)), u_solution_50(2, :), 'LineWidth', 2);
stairs(t_100(2:length(t_100)), u_solution_100(2, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('u2/Nm');
legend('sampling: 10', 'sampling: 50', 'sampling: 100');
suptitle('inputs');
%print('different_samplingrate_input','-dpng','-r0');