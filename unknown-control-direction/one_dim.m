% 积分微分方程仿真：dx = -x - y -1 , dy = x - 1/x , y=∫₀ᵗ(x-1/x)dτ
clear; clc; close all;

%% 1. 仿真配置
tspan = [0, 15];
x0    = 2;        % 初值 x(0)>0，可改为0.5, 1.5, 3等测试
y0    = 0;
z0    = [x0; y0];
opts  = odeset('RelTol',1e-7,'AbsTol',1e-10);

%% 2. 求解ODE
[t, Z] = ode45(@ode_fun, tspan, z0, opts);
x = Z(:,1);
y = Z(:,2);

%% 3. 绘图：x(t)、积分项y(t)
figure('Color','w');
subplot(2,1,1);
plot(t, x, 'b-','LineWidth',1.5); grid on;
yline(0,'r--','x=0边界');
xlabel('t'); ylabel('x(t)');
title('状态 x(t) 时域曲线');

subplot(2,1,2);
plot(t, y, 'r-','LineWidth',1.5); grid on;
xlabel('t'); ylabel('y(t)=\int_0^t (x-1/x)d\tau');
title('积分状态 y(t)');

%% 4. 残差校验方程正确性
dx_num = gradient(x, t);
dx_theo = -x - y - 1;
res = abs(dx_num - dx_theo);

figure('Color','w');
semilogy(t, res, 'k'); grid on;
xlabel('t'); ylabel('残差 |\dot{x}_{数值}-\dot{x}_{理论}|');
title('ODE数值求解残差');

%% ODE右端函数
function dz = ode_fun(t, z)
    x = z(1);
    y = z(2);
    % 数值兜底防除零
    if x < 1e-8
        error('x趋近于0，奇异，仿真终止');
    end
    dx = -x - y - 1;
    dy = x - 1/x;
    dz = [dx; dy];
end