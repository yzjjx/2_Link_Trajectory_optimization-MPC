function [g,geq] = nonlcon(x)
    % g is inequality <= 0
    % geq is equality = 0
    F_vec = x(1:end-1);
    g = -F_vec;

    T = x(end);
    % 等式约束
    tspan = [0 T];
    z0 = [0;1;0;1];

    dynamics = @(t,z) jetpackControlDynamics(t,z,x);
    [tout,zout] = ode45(dynamics,tspan,z0);

    xfinal = L_1*cos(zout(end,1))+L_2*cos(zout(end,2));
    yfinal = L_1*sin(zout(end,1))+L_2*sin(zout(end,2));
    dyfinal = zout(end,4);

    xtarget = 20;
    ytarget = 0;
    geq = [xfinal-xtarget;yfinal-ytarget;dyfinal];
end