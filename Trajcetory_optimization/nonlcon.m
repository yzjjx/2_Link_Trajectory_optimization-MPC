function [g,geq] = nonlcon(u)
    % g is inequality <= 0
    % geq is equality = 0
    F_vec = u(1:end-1);
    g = -F_vec;
    % g = [];

    T = u(end);
    % 等式约束
    tspan = [0 T];
    x0 = [0;1;0;1];

    dynamics = @(t,x) ControlDynamics(t,x,u);
    [tout,xout] = ode45(dynamics,tspan,x0);

    q1_final = xout(end,1);
    q2_final = xout(end,2);
    dq1_final = xout(end,3);
    dq2_final = xout(end,4);

    q1_target = deg2rad(20);
    q2_target = deg2rad(30);
    geq = [q1_final-q1_target;q2_final-q2_target;dq1_final;dq2_final];
end