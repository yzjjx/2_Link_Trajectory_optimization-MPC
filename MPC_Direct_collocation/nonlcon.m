function [g,geq] = nonlcon(w,x_current,dt)
    % g is inequality <= 0
    % geq is equality = 0
    % 不等式约束
    N = numel(w)/6;
    tau_max = 100*ones(N,1);
    tau_min = -100*ones(N,1);

    tau1 = w(1:N);
    tau2 = w(N+1:2*N);
    q1 = w(2*N+1:3*N);
    q2 = w(3*N+1:4*N);
    dq1 = w(4*N+1:5*N);
    dq2 = w(5*N+1:6*N);
    g = [-tau1+tau_min;tau1-tau_max;-tau2+tau_min;tau2-tau_max];

    % 等式约束
    geq_ic = [q1(1);dq1(1);q2(1);dq2(1)]-x_current;
    % geq_fc = [q1(N)-q1_target;q2(N)-q2_target;dq1(N);dq2(N)];

    % 前向欧拉：用区间起点的状态和力矩计算角加速度
    ddq1 = zeros(N-1, 1);
    ddq2 = zeros(N-1, 1);
    for k = 1:N-1
        u_k = [tau1(k); tau2(k)];
        x_k = [q1(k); dq1(k); q2(k); dq2(k)];
        dx_k = ControlDynamics((k-1)*dt, x_k, u_k);
        ddq1(k) = dx_k(2);
        ddq2(k) = dx_k(4);
    end

    left = 1:N-1;
    right = 2:N;

    % 位置缺陷
    geq_q1_defect = (q1(right) - q1(left))/dt - dq1(left);
    geq_q2_defect = (q2(right) - q2(left))/dt - dq2(left);

    % 速度缺陷
    geq_dq1_defect = (dq1(right) - dq1(left))/dt - ddq1;
    geq_dq2_defect = (dq2(right) - dq2(left))/dt - ddq2;

    geq = [geq_ic;geq_q1_defect;geq_q2_defect;geq_dq1_defect;geq_dq2_defect];
end
