clc;clear;close all;

tspan = [0 10];
x0 = [1; 0; 1; 0];

% ODE45仿真
[tout, xout] = ode45(@ControlDynamics, tspan, x0);

% 绘图
xout = tout;
