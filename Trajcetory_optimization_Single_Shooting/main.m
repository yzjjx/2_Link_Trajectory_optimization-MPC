clc;clear;close all;

T0 = 1;% 终止时间猜测
tau0 = [
    43.495935   0.587717
    42.199269   0.506572
    40.869976   0.570731
    39.414871   0.716834
    37.783157   0.895560
    36.032843   1.069802
    34.332614   1.214756
    32.926474   1.307898
];

N = size(tau0, 1); 

u0 = [reshape(tau0.', [], 1); T0];

% N = 7;
% u0 = 3*ones(N,1);
% u0(end) = 3;

% 线性约束没有
A = [];
b = [];
Aeq=[];
beq = [];
lb = [-inf(numel(u0) - 1, 1); 1e-3]; 
ub = [];

% Sets the options of the optimization
% -- we want to display the progress of the optimization
% over each iterations (Display = 'iter')
options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',10e6);

% Run our nonlinear programming (NLP) solver (fmincon)
[u_star,fvel,exitflag] = fmincon(@cost_fun,u0,A,b,Aeq,beq,lb,ub,@nonlcon,options);

T = u_star(end);
t_vec = linspace(0,T,N);
tau = reshape(u_star(1:end-1),2,[])';
tau1_vec = tau(:,1);

% 计算最优力矩轨迹
figure
subplot(2,1,1)
plot(t_vec(1:end), tau(:,1),'k-x','LineWidth',2)
title('Optimal force trajectory')
xlabel('time(s)');
ylabel('Tau1(N)')

subplot(2,1,2)
plot(t_vec(1:end), tau(:,2),'k-x','LineWidth',2)
title('Optimal force trajectory')
xlabel('time(s)');
ylabel('Tau2(N)')

dynamics = @(t,x) ControlDynamics(t,x,u_star);
tspan = [0 T];
x0 = [0;1;0;1];

[t_check,x_check] = ode45(dynamics,tspan,x0);
q1_sim = x_check(:,1);
dq1_sim = x_check(:,3);
q2_sim = x_check(:,2);
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
figure
set(gcf,'Color',[1,1,1]);
view_limit = 1.1*(L1 + L2);
for iter = 1:numel(t_anim)
    cla
    hold on
    plot(x_tip(1:iter),y_tip(1:iter),'b-','LineWidth',1.2)
    plot([0,x_joint(iter),x_tip(iter)],[0,y_joint(iter),y_tip(iter)], ...
         'ko-','LineWidth',2,'MarkerFaceColor','w')
    plot(x_tip(iter),y_tip(iter),'ro','MarkerSize',8,'MarkerFaceColor','r')
    axis equal
    axis([-view_limit,view_limit,-view_limit,view_limit])
    grid on
    xlabel('x (m)')
    ylabel('y (m)')
    title(sprintf('Two-link motion, t = %.2f s',t_anim(iter)))
    drawnow
    pause(1/FPS)
end
