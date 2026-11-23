clc;clear;close all;

N = 20;
tau1_init = 40*ones(N,1);
tau2_init = 2*ones(N,1);
q1_init = ones(N,1);
dq1_init = ones(N,1);
q2_init = ones(N,1);
dq2_init = ones(N,1);
T_init = 3;
w0 = [tau1_init;tau2_init;q1_init;q2_init;dq1_init;dq2_init;T_init];

% 线性约束没有
A = [];
b = [];
Aeq=[];
beq = [];
lb = []; 
ub = [];

% Sets the options of the optimization
% -- we want to display the progress of the optimization
% over each iterations (Display = 'iter')
options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',10e6);

% Run our nonlinear programming (NLP) solver (fmincon)
[w_star,fvel,exitflag] = fmincon(@cost_fun,w0,A,b,Aeq,beq,lb,ub,@nonlcon,options);

T = w_star(end);
t_vec = linspace(0,T,N);
tau1 = w_star(1:N);
tau2 = w_star(N+1:2*N);

% 计算最优力矩轨迹
figure
subplot(2,1,1)
plot(t_vec(1:end), tau1,'k-x','LineWidth',2)
title('Optimal force trajectory')
xlabel('time(s)');
ylabel('Tau1(N)')

subplot(2,1,2)
plot(t_vec(1:end),tau2,'k-x','LineWidth',2)
title('Optimal force trajectory')
xlabel('time(s)');
ylabel('Tau2(N)')

dynamics = @(t,x) ControlDynamics(t,x,w_star);
tspan = [0 T];
x0 = [0;0;0;0];

[t_check,x_check] = ode45(dynamics,tspan,x0);
q1_sim = x_check(:,1);
dq1_sim = x_check(:,2);
q2_sim = x_check(:,3);
dq2_sim = x_check(:,4);

% 绘图
figure
subplot(4,1,1)
plot(t_check,q1_sim,'k-','LineWidth',2)
xlabel('time(s)')
ylabel('q1 pos(rad)')

subplot(4,1,2)
plot(t_check,dq1_sim,'k-','LineWidth',2)
xlabel('time(s)')
ylabel('q1 vel(rad/s)')

subplot(4,1,3)
plot(t_check,q2_sim,'k-','LineWidth',2)
xlabel('time(s)')
ylabel('q2 pos(rad)')

subplot(4,1,4)
plot(t_check,dq2_sim,'k-','LineWidth',2)
xlabel('time(s)')
ylabel('q2 vel(rad/s)')

%% 制作两连杆动画
L1 = 0.45; % 第一连杆长度 (m)
L2 = 0.45; % 第二连杆长度 (m)
FPS = 20;
SLOMO = 1/4; % 以四分之一速度播放

% 将 ode45 的结果插值到动画时间点，并保留最后一帧
T_anim = t_check(end);
t_anim = 0:SLOMO/FPS:T_anim;
if t_anim(end) < T_anim
    t_anim(end+1) = T_anim;
end

q1_anim = interp1(t_check,q1_sim,t_anim);
q2_anim = interp1(t_check,q2_sim,t_anim);

% 第一连杆末端与机械臂末端的坐标
x_joint = L1*cos(q1_anim);
y_joint = L1*sin(q1_anim);
x_tip = x_joint + L2*cos(q1_anim + q2_anim);
y_tip = y_joint + L2*sin(q1_anim + q2_anim);

% 动画循环
fig_anim = figure('Color', 'w');
ax_anim = axes('Parent', fig_anim);
view_limit = 1.1 * (L1 + L2);

for iter = 1:numel(t_anim)
    cla(ax_anim);
    hold(ax_anim, 'on');

    plot(ax_anim, x_tip(1:iter), y_tip(1:iter), 'b-', 'LineWidth', 1.2);
    plot(ax_anim, [0, x_joint(iter), x_tip(iter)], ...
                  [0, y_joint(iter), y_tip(iter)], ...
                  'ko-', 'LineWidth', 2, 'MarkerFaceColor', 'w');
    plot(ax_anim, x_tip(iter), y_tip(iter), 'ro', ...
                  'MarkerSize', 8, 'MarkerFaceColor', 'r');

    axis(ax_anim, 'equal');
    axis(ax_anim, [-view_limit, view_limit, -view_limit, view_limit]);
    grid(ax_anim, 'on');
    xlabel(ax_anim, 'x (m)');
    ylabel(ax_anim, 'y (m)');
    title(ax_anim, sprintf('Two-link motion, t = %.2f s', t_anim(iter)));

    drawnow;
    pause(1/FPS);
end
