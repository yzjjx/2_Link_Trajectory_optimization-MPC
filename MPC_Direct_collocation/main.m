clc;clear;close all;

N = 20;
tau1_init = 40*ones(N,1);
tau2_init = 2*ones(N,1);
q1_init = ones(N,1);
dq1_init = ones(N,1);
q2_init = ones(N,1);
dq2_init = ones(N,1);
T_init = 3;

% Time Horizon
T = 2;
dT = T/(N-1);
q_ref = deg2rad([30, 30]);
w0 = [tau1_init;tau2_init;q1_init;q2_init;dq1_init;dq2_init];
x_ref = [deg2rad(20); 0; deg2rad(30); 0];
% 仿真参数
tspan = [0 10];
x0 = [0;0;0;0];

t_sim = [];
x_sim = [];
num_steps = numel(tspan(1):dT:tspan(2));
t_ref = zeros(num_steps,1);
q_ref_history = zeros(num_steps,2);

% 放在 for 循环前
options = optimoptions('fmincon', ...
    'OptimalityTolerance', 1e-3, ...
    'MaxFunctionEvaluations', 1e5);

%% 创建MPC循环
% 每次采样时间dT，获取当前状态z0，进行轨迹优化
t_current = tspan(1);

for iter = 1:num_steps
    q_ref = q_ref+deg2rad(5)*randn(1,2);
    t_ref(iter) = t_current;
    q_ref_history(iter,:) = q_ref;
    objective = @(w) cost_fun(w,q_ref);

    % 线性约束没有
    A = [];b = [];Aeq=[];beq = [];lb = [];ub = [];

    constraint = @(w) nonlcon(w, x0, dT);
    [w_star,fvel,exitflag] = fmincon(objective,w0,A,b,Aeq,beq,lb,ub,constraint,options);

    tau1 = w_star(1:N);
    tau2 = w_star(N+1:2*N);
    % 将第一个力矩输入给动力学
    u = [tau1(1);tau2(1)];
    dynamics_call = @(t,x) ControlDynamics(t, x, u);

    tspan_iter = [t_current t_current+dT-1e-6];
    [tout,zout] = ode45(dynamics_call,tspan_iter,x0);
    t_current = t_current + dT;

    % 更新当前状态
    x0 = zout(end,:).';

    t_sim = [t_sim;tout];
    x_sim = [x_sim;zout];

end

%% 1. 绘制最后一轮 MPC 预测的力矩
% tau1、tau2 保存的是最后一轮预测结果，横轴用预测时间
t_pred = (0:N-1)' * dT;

figure('Name', '最后一轮 MPC 预测力矩');
subplot(2,1,1)
plot(t_pred, tau1, 'k-o');
ylabel('\tau_1 (N·m)');
grid on

subplot(2,1,2)
plot(t_pred, tau2, 'k-o');
ylabel('\tau_2 (N·m)');
xlabel('预测时间 (s)');
grid on

%% 2. 绘制实际闭环状态
% 当前 for 循环会多执行一个采样区间，绘图只取指定的仿真时间
keep = t_sim <= tspan(2);
t = t_sim(keep);
x = x_sim(keep,:);       % 列顺序：[q1, dq1, q2, dq2]

labels = {'q_1 (rad)', 'dq_1 (rad/s)', 'q_2 (rad)', 'dq_2 (rad/s)'};

figure('Name', 'MPC 闭环状态');
for j = 1:4
    subplot(4,1,j)
    plot(t, x(:,j), 'k-', 'LineWidth', 1.5);
    ylabel(labels{j});
    grid on
end
xlabel('时间 (s)');


%% 3. 用实际闭环关节角制作动画
L1 = 0.45;
L2 = 0.45;
FPS = 20;
speed = 1;               % 1 为正常速度，0.25 为四分之一速

plot_mpc_animation(t, x, t_ref, q_ref_history, L1, L2, FPS, speed);
