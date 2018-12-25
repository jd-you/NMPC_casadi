function x_next = rk4(ode, x0, t0, h, u)
    k1 = h * ode(t0, x0, u);
    k2 = h * ode(t0 + h / 2, x0 + k1 / 2, u);
    k3 = h * ode(t0 + h / 2, x0 + k2 / 2, u);
    k4 = h * ode(t0 + h, x0 + k3, u);
    
    x_next = x0 + 1 / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
end