# 二连杆机器人MPC系列仿真
## 简介
目标为：让二连杆机器人末端到指定的位置

使用机器人模型为：

首先建立状态空间模型：

两个关节力矩输入：
$$
u(t) = [\tau_1;\tau_2]
$$

已知的状态就是关节一和关节二的位置和速度，末端的位置和速度就是等式约束（到达指定位置）

$$
x = \begin{bmatrix}q_1\\
q_2 \\
\dot{q}_1 \\
\dot{q}_2 \\
\end{bmatrix}
$$

可以得到：

$$
\dot{x} = \begin{bmatrix}\dot{q}_1 \\
\dot{q}_2 \\
\ddot{q}_1 \\
\ddot{q}_2 \\
\end{bmatrix}
$$

## 仿真模型建立
在之前的代码中，可以得到模型：
```
采样次数：28；参数单项式：9；独立参数组合：5。
固定参数向量 pi_param =
m_2*L_1^2 + m_1*lc_1^2 + I_zz1
                  L_1*lc_2*m_2
            L_1*m_2 + lc_1*m_1
            m_2*lc_2^2 + I_zz2
                      lc_2*m_2
 
反推得到的符号回归矩阵 Y =
[ddq_1, - sin(q_2)*dq_2^2 - 2*dq_1*sin(q_2)*dq_2 + 2*ddq_1*cos(q_2) + ddq_2*cos(q_2), g*cos(q_1), ddq_1 + ddq_2, g*cos(q_1 + q_2)]
[    0,                                             sin(q_2)*dq_1^2 + ddq_1*cos(q_2),          0, ddq_1 + ddq_2, g*cos(q_1 + q_2)]
 
前三次随机实验的符号力矩（每列为一次实验）=
[- 3*m_2*L_1^2 + (12943*m_2*L_1*lc_2)/3562 - (3569*g*m_2*L_1)/4369 - 3*m_1*lc_1^2 - (3569*g*m_1*lc_1)/4369 + (3*m_2*lc_2^2)/2 + (3576189*g*m_2*lc_2)/7781189 - 3*I_zz1 + (3*I_zz2)/2, (32143*L_1*lc_2*m_2)/14578 - (7*L_1^2*m_2)/2 - (7*lc_1^2*m_1)/2 - (209*L_1*g*m_2)/241 - (7*I_zz1)/2 - (209*g*lc_1*m_1)/241 + (1754601*g*lc_2*m_2)/1756649, (m_2*L_1^2)/2 - (44256*m_2*L_1*lc_2)/6961 - (329*g*m_2*L_1)/1129 + (m_1*lc_1^2)/2 - (329*g*m_1*lc_1)/1129 + (11*m_2*lc_2^2)/2 + (5526169*g*m_2*lc_2)/7858969 + I_zz1/2 + (11*I_zz2)/2]
[                                                                                          (3*I_zz2)/2 + (3*lc_2^2*m_2)/2 + (12123*L_1*lc_2*m_2)/1781 + (3576189*g*lc_2*m_2)/7781189,                                                                                                             (lc_2*m_2*(11347003*L_1 + 3509202*g))/3513298,                                                                                          (11*I_zz2)/2 + (11*lc_2^2*m_2)/2 - (20741*L_1*lc_2*m_2)/13922 + (5526169*g*lc_2*m_2)/7858969]
 
验证通过：tau - Y*pi_param = 0。
```

### ddq的求法
因为现在是使用回归方式进行建模，回归式为：

$$
\tau = \beta(q, \dot{q}, \ddot{q}) \pi = M(q, \pi) \ddot{q} + h(q, \dot{q}, \pi)
$$

那么只要得到质量矩阵 $M$ 和偏置力矩 $h$ ，就可以矩阵计算得到 $\ddot{q}$

h的求法可以参考代码：`Trajcetory_optimization\cal_h.m`

## 轨迹优化：单重打靶
打靶法与配点法最大的区别就是，**打靶法的决策向量只有控制量，没有状态量**

整个代码文件为：`Trajcetory_optimization`

构建的整个控制输入为：

$$
u=\begin{bmatrix}\tau_{11}\\
\tau_{21} \\
 \tau_{12}\\
\tau_{22} \\
\tau_{13} \\
\tau_{23}\\
...\\
\tau_{1N} \\
\tau_{2N}\\
T
\end{bmatrix}
$$

### 仿真模型建立
这个函数的意思就是在每个t时间点持续输入u，然后用动力学方程计算出来x的值

主要代码：

```matlab
T = u(end);
N = (numel(u) - 1)/2;
t_points = linspace(0, T, N);
tau_points = reshape(u(1:end-1), 2, [])';
u = interp1(t_points, tau_points, t)';
```

在上面代码里面，第一行是提取控制时间，第二行是计算多少步的控制输入，第三行是线性插值，第四行是对力矩的转换，这里将控制输入转换为：

$$
tau \_ points = \begin{bmatrix}
\tau_{11}  & \tau_{21}\\
\tau_{12}  & \tau_{22} \\
\tau_{13}  & \tau_{23} \\
... & ...\\
\tau_{1N}  & \tau_{2N}
\end{bmatrix}
$$

然后最后一行是得到一个一行两列的点，在转置回来

建立仿真模型的代码参考：`Trajcetory_optimization\ControlDynamics.m`

可以查看初始值为: $x_0 = [1;0;1;0]$ 的效果，即初始q1位置为1，q1速度为1

<div align="center">
    <img src="Trajcetory_optimization_Single_Shooting\fig\初始x1010.png">
    <br>
</div>

这个给定输入的仿真使用了ode45，主要代码为：

```matlab
tspan = [0 10];
x0 = [1; 0; 1; 0];
u0 = [0; 0; 0; 0; tspan(end)];

% ODE45仿真
[tout, xout] = ode45(@(t,x) ControlDynamics(t,x,u0), tspan, x0);
```

ODE45是数值积分方法，ODE算出来的就是数值计算法的 $\dot{x}$ ，也就是ODE455负责将加速度积分为速度，将速度积分为角度

**也就是说动力学得到了当前时刻的dx和ddx，然后积分方法得到下一步的x和dx**

该代码可以查看：`Trajcetory_optimization\sim_2link.m`

### 代价函数建立
代价函数建立的代码参见`Trajcetory_optimization\cost_fun.m`

主要公式为：

$$
f(u) = (\sum_{i=1}^{N}F_i^2 )\Delta t
$$

为了方便计算平方，这里面同样使用了矩阵的转换reshape函数

主要代码为：

```matlab
tau_nodes = reshape(F_vec, 2, [])';
% cost = sum(F_vec(1:end-1).^2)*dt; 
torque_sq = tau_nodes(:,1).^2 + tau_nodes(:,2).^2;

cost = dt * (0.5*torque_sq(1) ...
            + sum(torque_sq(2:end-1)) ...
            + 0.5*torque_sq(end));
```

### 非线性约束建立
非线性约束代码参见：`Trajcetory_optimization\nonlcon.m`

首先建立不等式约束，可以规定所有力为正数，有：

```matlab
F_vec = u(1:end-1);
g = -F_vec;
```

然后建立等式约束，即每个关节角移动到指定位置  
在建立等式约束的时候，因为需要知道最终运动时间的关节角，所以要进行ODE仿真

```matlab
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
```
因此，单重打靶主要是进行完整的积分，然后关注最后一个点是否达到了指定的状态
### 主函数：fmincon

因为系统不稳定容易发散，建议直接给差不多的初始猜测，这样才能运行完整，其中制作连杆运动动画为AI生成，其余均为自己编写，最终效果如下图所示：

输入力矩为：

<div align="center">
    <img src="Trajcetory_optimization_Single_Shooting\fig\输入力矩.png">
    <br>
</div>

关节角与关节速度为：

<div align="center">
    <img src="Trajcetory_optimization_Single_Shooting\fig\轨迹优化结果.png">
    <br>
</div>

轨迹运动动画为：

<div align="center">
    <img src="Trajcetory_optimization_Single_Shooting\fig\轨迹运动.gif">
    <br>
</div>

注意：**轨迹优化依赖于较好的初始猜测，才能最终收敛到指定的大小，如果都给1，那么基本上无法收敛**

在main和nonlcon存在注释的不添加力矩约束和给定固定初始值的代码，可以尝试取消注释查看

## 轨迹规划：直接配点法
所有代码可以查看文件：`Trajcetory_optimization_Direct_collocation`
对于不稳定系统或者欠驱动系统，直接配点法比单重打靶更加有效

直接配点法与单重打靶的区别之一是决策变量加入了状态参数，使用前向欧拉作为约束

### 非线性约束建立
代码参考：`MPC_Direct_collocation\nonlcon.m`

相比于单重打靶，这里的非线性约束就不再使用ODE45进行完全仿真，而是使用前向欧拉

这里的不等式约束加入了力矩的最大值与最小值作为约束，代码如下：

```matlab
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
```

### 配点约束对比
#### 显式欧拉法
显式欧拉N=20，下面为结果：

<div align="center">
    <img src="Trajcetory_optimization_Direct_collocation\fig\配点法：显式欧拉：位置.png">
    <br>
</div>

<div align="center">
    <img src="Trajcetory_optimization_Direct_collocation\fig\配点法：显式欧拉.gif">
    <br>
</div>

#### 隐式欧拉法
显式欧拉N=40，下面为结果：

<div align="center">
    <img src="Trajcetory_optimization_Direct_collocation\fig\配点法：隐式欧拉：加多N点.png">
    <br>
</div>

<div align="center">
    <img src="Trajcetory_optimization_Direct_collocation\fig\配点法：隐式欧拉：配点N增多.gif">
    <br>
</div>

#### 梯形法
N=20，下面为结果

<div align="center">
    <img src="Trajcetory_optimization_Direct_collocation\fig\梯形法：N=20.png">
    <br>
</div>

<div align="center">
    <img src="Trajcetory_optimization_Direct_collocation\fig\配点法：梯形法：N=20.gif">
    <br>
</div>

#### 结论
隐式欧拉精度大于显式欧拉，二者都需要增多决策向量进行精度的提升

梯形法精度最高，不需要额外增多决策变量

## 直接配点法：MPC

实现基于直接配点法的MPC控制，代码文件参考：`MPC_Direct_collocation`

这里非线性不等式约束的代码不变

代价函数也非常好理解

$$
\begin{aligned}
J(w)
&= \underbrace{
\sum_{k=1}^{N}
\left[
(q_1^{\mathrm{ref}}-q_{1,k})^2
+(q_2^{\mathrm{ref}}-q_{2,k})^2
\right]
}_{\text{角度跟踪误差}}
\\
&\quad + \underbrace{
10^{-4}\sum_{k=1}^{N-1}
\left[
(\tau_{1,k+1}-\tau_{1,k})^2
+(\tau_{2,k+1}-\tau_{2,k})^2
\right]
}_{\text{力矩平滑项}}.
\end{aligned}
$$

### 主循环

主循环需要定义预测步长，也就是预测时间T，MPC最主要的就是将最优化部分放到循环里面求解，下面给出MPC的主要流程：

MPC（模型预测控制）
1. 测量状态：获取系统当前的状态。
2. 执行轨迹优化：在固定的预测时间范围 $T$ 内进行轨迹优化。
3. 执行最优控制输入：将优化得到的控制输入 $u(t)$ 施加到系统上，持续 $T_s$ 秒。
4. 返回第一步：重新测量状态，重复上述过程。

根据这个流程，看代码：`MPC_Direct_collocation\main.m`

主要是在循环里面持续计算非线性约束和fmincon最优化

```matlab
constraint = @(w) nonlcon(w, x0, dT);
[w_star,fvel,exitflag] = fmincon(objective,w0,A,b,Aeq,beq,lb,ub,constraint,options);
```

然后将计算的第一组力矩输入给动力学，然后用ode45得到机械臂施加力矩后走到了什么新的状态，作为下一组优化的起点，在真实系统中，可以由传感器获取新状态

```matlab
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
```

## 控制频率计算
预测时间为2s，预测节点为20

对于控制频率的计算就是两次给tau之间的时间，也就是循环执行一次的时间

### 运行结果
不加扰动跟踪效果：

<div align="center">
    <img src="MPC_Direct_collocation\fig\MPC跟踪.gif">
    <br>
</div>

末端添加扰动：
<div align="center">
    <img src="MPC_Direct_collocation\fig\MPC跟踪_加扰动.gif">
    <br>
</div>