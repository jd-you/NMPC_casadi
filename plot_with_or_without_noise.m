clear;
clc;

[x_nmpc, u_nmpc, t_nmpc] = nmpc(0, 0, 0); % nmpc without model error and noise
[x_nmpc_with_noise, u_nmpc_with_noise, t_nmpc_with_noise] = nmpc(0, 1, 0.1); % nmpc without model error, but with noise, its standard deviation is 0.2

%% plot q1-q2-plane
p = params();

x_solution_nmpc = x_nmpc ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_nmpc = u_nmpc ./ [p.scal_u1; p.scal_u2];

x_solution_nmpc_with_noise = x_nmpc_with_noise ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
u_solution_nmpc_with_noise = u_nmpc_with_noise ./ [p.scal_u1; p.scal_u2];


figure;
fig1 = gcf;
fig1.PaperUnits = 'inches';
fig1.PaperPosition = [0, 0, 8, 8];

subplot(2, 2, 1);
axis([0, 3, -5.5, 2]);
hold on;
grid on;
plot(t_nmpc, x_solution_nmpc(1, :), 'LineWidth', 2);
plot(t_nmpc_with_noise, x_solution_nmpc_with_noise(1, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('q1/rad');
legend('NMPC', 'NMPC with noise','Location','southeast');

subplot(2, 2, 2);
axis([0, 3, -4.5, 1]);
hold on;
grid on;
plot(t_nmpc, x_solution_nmpc(2, :), 'LineWidth', 2);
plot(t_nmpc_with_noise, x_solution_nmpc_with_noise(2, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('q2/rad');
legend('NMPC', 'NMPC with noise','Location','southeast');

subplot(2, 2, 3);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_nmpc, x_solution_nmpc(3, :), 'LineWidth', 2);
plot(t_nmpc_with_noise, x_solution_nmpc_with_noise(3, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('w1/rad/s');
legend('NMPC', 'NMPC with noise','Location','southeast');

subplot(2, 2, 4);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_nmpc, x_solution_nmpc(4, :), 'LineWidth', 2);
plot(t_nmpc_with_noise, x_solution_nmpc_with_noise(4, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('w2/rad/s');
suptitle('q1-q2-plane');
legend('NMPC', 'NMPC with noise','Location','southeast');

%print('noise_q1q2','-dpng','-r0');
%% plot x-y-plane

x_heun = p.l1 .* cos(x_solution_nmpc(1, :)) + p.l2 .* cos(x_solution_nmpc(1, :) + x_solution_nmpc(2, :));
y_heun = p.l1 .* sin(x_solution_nmpc(1, :)) + p.l2 .* sin(x_solution_nmpc(1, :) + x_solution_nmpc(2, :));

x_euler = p.l1 .* cos(x_solution_nmpc_with_noise(1, :)) + p.l2 .* cos(x_solution_nmpc_with_noise(1, :) + x_solution_nmpc_with_noise(2, :));
y_euler = p.l1 .* sin(x_solution_nmpc_with_noise(1, :)) + p.l2 .* sin(x_solution_nmpc_with_noise(1, :) + x_solution_nmpc_with_noise(2, :));

figure;
fig2 = gcf;
fig2.PaperUnits = 'inches';
fig2.PaperPosition = [0, 0, 8, 8];

subplot(2, 1, 1);
axis([0, 3, -1, 1]);
hold on;
grid on;
plot(t_nmpc, x_heun, 'LineWidth', 2);
plot(t_nmpc_with_noise, x_euler, 'LineWidth', 2);
xlabel('t/s');
ylabel('x/m');
legend('NMPC', 'NMPC with noise','Location','southeast');

subplot(2, 1, 2);
axis([0, 3, -0.5, 1]);
hold on;
grid on;
plot(t_nmpc, y_heun, 'LineWidth', 2);
plot(t_nmpc_with_noise, y_euler, 'LineWidth', 2);
xlabel('t/s');
ylabel('y/m');
legend('NMPC', 'NMPC with noise','Location','southeast');
suptitle('x-y-plane');

%print('noise_xy','-dpng','-r0');
%% plot inputs
figure;
fig3 = gcf;
fig3.PaperUnits='inches';
fig3.PaperPosition = [0 0 8 8];
subplot(2, 1, 1);

axis([0, 3, -1000, 1200]);
hold on;
grid on;
stairs(t_nmpc(2:length(t_nmpc)), u_solution_nmpc(1, :), 'LineWidth', 2);
stairs(t_nmpc_with_noise(2:length(t_nmpc_with_noise)), u_solution_nmpc_with_noise(1, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('u1/Nm');
legend('NMPC', 'NMPC with noise');

subplot(2, 1, 2);
axis([0, 3, -1000, 500]);
hold on;
grid on;
stairs(t_nmpc(2:length(t_nmpc)), u_solution_nmpc(2, :), 'LineWidth', 2);
stairs(t_nmpc_with_noise(2:length(t_nmpc_with_noise)), u_solution_nmpc_with_noise(2, :), 'LineWidth', 2);
xlabel('t/s');
ylabel('u2/Nm');
legend('NMPC', 'NMPC with noise');
suptitle('inputs');
%print('noise_input','-dpng','-r0');