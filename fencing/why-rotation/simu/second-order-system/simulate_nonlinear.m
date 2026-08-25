% simulate_nonlinear.m
% 仿真系统: \ddot{x} + k_1 \dot{x} + k_2 x - 1/x = 0, 其中 x \in \mathbb{R}^2
% 即每个分量独立满足: \ddot{x}_i + k_1 \dot{x}_i + k_2 x_i - 1/x_i = 0
% 绘制 k_1 = k_2 = 1 时 x 随时间的变化曲线, 以及 atan2(x(2), x(1)) 的变化曲线

clear; clc; close all;

% 参数
k1 = 1;
k2 = 1;

% 初始条件: [x1, x2, v1, v2]
% 注意: 1/x 在 x=0 处奇异, 所以初始条件应使 x1, x2 始终同号且不穿过零
x0 = [2; 3; 0; 0.5];  % 可根据需要修改

% 时间范围
tspan = [0, 10];

% 定义 ODE 函数
% 状态向量 y = [x1, x2, v1, v2]
% dy/dt = [v1; v2; -k1*v1 - k2*x1 + 1/x1; -k1*v2 - k2*x2 + 1/x2]
odefun = @(t, y) [y(3); y(4); ...
                   -k1*y(3) - k2*y(1) + 1/y(1); ...
                   -k1*y(4) - k2*y(2) + 1/y(2)];

% 求解 ODE
[t, y] = ode45(odefun, tspan, x0);

% 提取状态
x1 = y(:, 1);
x2 = y(:, 2);
v1 = y(:, 3);
v2 = y(:, 4);

% 计算 atan2(x(2), x(1))
theta = atan2(x2, x1);

% 图1: x(1) 和 x(2) 随时间的变化
figure(1);
plot(t, x1, 'b-', 'LineWidth', 1.5); hold on;
plot(t, x2, 'r-', 'LineWidth', 1.5);
xlabel('时间 t');
ylabel('x');
title('x 随时间的变化 (k_1 = k_2 = 1, 非线性项 -1/x)');
legend('x_1(t)', 'x_2(t)');
grid on;
hold off;

% 图2: atan2(x(2), x(1)) 随时间的变化
figure(2);
plot(t, theta, 'g-', 'LineWidth', 1.5);
xlabel('时间 t');
ylabel('atan2(x_2, x_1)');
title('atan2(x(2), x(1)) 随时间的变化 (k_1 = k_2 = 1, 非线性项 -1/x)');
grid on;