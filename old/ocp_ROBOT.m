% function ocp
clear;
clc;

import casadi.*;
ocp = casadi.Opti();

%% get the parameters
[~, CON, SCu] = parameters;
umin = SCu .* [CON.u(1); CON.u(1)];
umax = SCu .* [CON.u(2); CON.u(2)];
wmin = [CON.w(1); CON.w(1)];
wmax = [CON.w(2); CON.w(2)];

%% build ocp
T_end = 3;
delta_t = 0.01;
% delta_t = 0.1;
n_ocp = T_end / delta_t;
nu = 2;
nx = 4;

X = ocp.variable(nx, n_ocp + 1);
U = ocp.variable(nu, n_ocp);
X0 = ocp.parameter(nx, 1);
J = 0;

% modify the inital state and terminal state;
X_0 = [-5; 0; -4; 0];
%X_0 = wrapTo2Pi([-5; 0; -4; 0]);
ocp.set_value(X0, X_0);

X_end = [0; 0; 0; 0];

if (X_0(1) <= pi * 3/2 && X_0(1) >= 0)
    X_end(1) = pi / 2;
else
    X_end(1) = pi * 5/2;
end

if (X_0(3) <= pi)
    X_end(3) = 0;
else
    X_end(3) = 2 * pi;
end

U_end = 0;

% define the weighting in the objective function
Q = 1e3 * eye(4);
R = 1e3 * eye(2);


for i = 1:n_ocp
    X_next = rk4(X(:, i), U(:, i), delta_t * i, delta_t);
    ocp.subject_to(X_next == X(:, i + 1));
    ocp.subject_to(wmin <= [X(2, i + 1); X(4, i + 1)] <=  wmax);
    ocp.subject_to(umin <= U(:,i) <= umax);
    
    J = J + 0.5 * (X(:, i + 1) - X_end)' * Q *(X(:, i + 1) - X_end) + 0.5 * (U(:, i) - U_end)'  * R * (U(:, i) - U_end);
end

ocp.subject_to(X(:, 1) == X0);
ocp.subject_to(X(:, end) == X_end);
ocp.subject_to(U(:, end) == U_end);

ocp.minimize(J);
ocp.solver('ipopt');

sol = ocp.solve();

X_opt = full(sol.value(X));
U_opt = full(sol.value(U));

t = 0:delta_t:T_end;
plot(t,X_opt);

