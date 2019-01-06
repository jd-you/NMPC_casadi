function [x_nmpc, u_nmpc, t_nmpc] = nmpc(model_error, noise_error, standard_deviation)
import casadi.*;
p = params();

%% define OCP (horizon, J, sampling rate)
opti = casadi.Opti();

t_horizon = 1;
t_0 = 0;
N = 50;
t_s = (t_horizon - t_0) / N;
t_opt = 3;

x = opti.variable(4, N + 1);
u = opti.variable(2, N);

J = 0;
Q = 1*eye(4);
R = 1*eye(2);

x_0 = opti.parameter(4, 1);
x_end = p.x_end;

for i = 1:N
    if(model_error == 1)
        x_next = euler_robot_wrong((i - 1) * t_s, x(:, i), u(:, i), t_s * t_opt);
    else
        x_next = euler_robot((i - 1) * t_s, x(:, i), u(:, i), t_s * t_opt);
    end
    
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
t_end = 3;
N_nmpc = t_end / t_s;

x_guess = [];
u_guess = [];
opti.set_value(x_0, p.x_0);

x_ = p.x_0;
x_real = p.x_0;

x_result = [x_];
u_result = [];
t_nmpc = [0];

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
    if(noise_error == 1)
        noise = normrnd(0, standard_deviation, 4, 1);
        x_real = heun_robot((i - 1) * t_s, x_real, u_, t_s * t_opt);
        x_ = x_real +noise;

    else
        x_real = heun_robot((i - 1) * t_s, x_real, u_, t_s * t_opt);
        x_ = x_real;
    end
    
    opti.set_value(x_0, x_);
    
    x_result = [x_result, x_real];
    u_result = [u_result, u_];
    t_nmpc = [t_nmpc, i * t_s];
end

x_nmpc = x_result;
u_nmpc = u_result;

end