clc;clear;close all;

tspan = [0 10];
x0 = [1; 0; 1; 0];
u0 = [0; 0; 0; 0; tspan(end)];

% ODE45仿真
[tout, xout] = ode45(@(t,x) ControlDynamics(t,x,u0), tspan, x0);

% 绘制关节角
figure;
subplot(2, 1, 1);
plot(tout, xout(:, 1:2), 'LineWidth', 1.5);
ylabel('关节角 (rad)');
legend('q_1', 'q_2');
grid on;

% 绘制关节角速度
subplot(2, 1, 2);
plot(tout, xout(:, 3:4), 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('关节角速度 (rad/s)');
legend('dq_1', 'dq_2');
grid on;