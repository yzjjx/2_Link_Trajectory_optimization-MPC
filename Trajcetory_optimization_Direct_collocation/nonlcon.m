function [g,geq] = nonlcon(w)
    q1_target = deg2rad(20);
    q2_target = deg2rad(30);
    % g is inequality <= 0
    % geq is equality = 0
    % 不等式约束
    T = w(end);
    N = (numel(w)-1)/6;
    tau_max = 100*ones(N,1);
    tau_min = -100*ones(N,1);

    tau1 = w(1:N);
    tau2 = w(N+1:2*N);
    q1 = w(2*N+1:3*N);
    q2 = w(3*N+1:4*N);
    dq1 = w(4*N+1:5*N);
    dq2 = w(5*N+1:6*N);
    g = [-T;-tau1+tau_min;tau1-tau_max;-tau2+tau_min;tau2-tau_max];

    % 等式约束
    geq_ic = [q1(1);q2(1);dq1(1);dq2(1)];
    geq_fc = [q1(N)-q1_target;q2(N)-q2_target;dq1(N);dq2(N)];
    % 缺陷约束
    dt = T/(N-1);

    % ddq获取，有N个节点，但是只有N-1个相邻区间
    t_points = linspace(0, T, N);
    ddq1 = zeros(N-1, 1);
    ddq2 = zeros(N-1, 1);

    for k = 1:N
        x_k = [q1(k); dq1(k); q2(k); dq2(k)];
        dx_k = ControlDynamics(t_points(k), x_k, w);
        ddq1(k) = dx_k(2);
        ddq2(k) = dx_k(4);
    end

    %% 显式欧拉
    % 位置等式约束
    % geq_q1_defect = (q1(2:N)-q1(1:N-1))/dt-dq1(1:N-1);
    % geq_q2_defect = (q2(2:N)-q2(1:N-1))/dt-dq2(1:N-1);

    % % 速度等式约束
    % geq_dq1_defect = (dq1(2:N)-dq1(1:N-1))/dt-ddq1(1:N-1);
    % geq_dq2_defect = (dq2(2:N)-dq2(1:N-1))/dt-ddq2(1:N-1);

    % %% 隐式欧拉
    % % % 位置等式约束
    % geq_q1_defect  = (q1(2:N)  - q1(1:N-1))  / dt - dq1(2:N);
    % geq_q2_defect  = (q2(2:N)  - q2(1:N-1))  / dt - dq2(2:N);
    % 
    % % % 速度等式约束
    % geq_dq1_defect = (dq1(2:N) - dq1(1:N-1)) / dt - ddq1(2:N);
    % geq_dq2_defect = (dq2(2:N) - dq2(1:N-1)) / dt - ddq2(2:N);

    % 梯形约束

    left = 1:N-1;
    right = 2:N;

    % 位置缺陷：角度差分减去两端角速度的平均值
    % geq_q1_defect = (q1(left)-q1(right))/dt-dq1();
    geq_q1_defect = diff(q1)/dt - (dq1(left) + dq1(right))/2;
    geq_q2_defect = diff(q2)/dt - (dq2(left) + dq2(right))/2;

    % 速度缺陷：角速度差分减去两端角加速度的平均值
    geq_dq1_defect = diff(dq1)/dt - (ddq1(left) + ddq1(right))/2;
    geq_dq2_defect = diff(dq2)/dt - (ddq2(left) + ddq2(right))/2;


    geq = [geq_ic;geq_fc;geq_q1_defect;geq_q2_defect;geq_dq1_defect;geq_dq2_defect];
end