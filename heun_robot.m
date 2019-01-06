function xf = heun_robot(t, x, u, h, t_opt)
    if (nargin < 5)
        t_opt = 1;
    end
 xt = x + h*ROBOT_ode(x,u,t,t_opt);
 xf = x + h/2*(ROBOT_ode(x,u,t,t_opt)+ROBOT_ode(xt,u,t+h,t_opt));
end