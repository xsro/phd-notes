% 多维积分微分系统仿真 n=2
% dx = -x - y - ones(n,1)
% dy = x - x/norm(x)^2
clear; clc; close all;

%% 1. 设置维度与初值
n = 2;
tspan = [0, 20];
x0 = [0.8; 0.6];   % ||x0||=1 >0，可换小初值如[0.1;0.05]
y0 = zeros(n,1);
z0 = [x0; y0];     % 总状态 [x; y]，长度2n

opts = odeset('RelTol',1e-7,'AbsTol',1e-10);
[t, Z] = ode45(@ode_fun, tspan, z0, opts);

Nt = length(t);
x_all = Z(:,1:n);
y_all = Z(:,n+1:2*n);
r = vecnorm(x_all,2,2); % 每一时刻||x(t)||

%% 绘图1：各分量 x1,x2 与时变模长r(t)
figure('Color','w');
subplot(2,1,1);
plot(t, x_all(:,1), 'b-', t, x_all(:,2), 'r-.','LineWidth',1.3);
grid on; yline(0,'k--');
xlabel('t'); ylabel('x_1(t), x_2(t)');
legend('x_1','x_2'); title('x各分量时域曲线');

subplot(2,1,2);
plot(t, r, 'm-','LineWidth',1.5); grid on;
yline(0,'r--','原点壁垒 r=0');
xlabel('t'); ylabel('r(t)=\|x(t)\|');
title('状态模长 \|x(t)\|');

%% 绘图2：相平面轨迹 (x1,x2)
figure('Color','w');
plot(x_all(:,1), x_all(:,2), 'g-','LineWidth',1.2); hold on;
plot(0,0,'ro','MarkerSize',10,'DisplayName','原点(奇点)');
grid on; axis equal;
xlabel('x_1'); ylabel('x_2'); title('二维相平面轨线');
legend;

%% 残差校验（验证ODE转化正确）
dx_num = zeros(size(x_all));
for i=1:n
    dx_num(:,i) = gradient(x_all(:,i), t);
end
dx_theo = -x_all - y_all - ones(size(x_all));
res = vecnorm(dx_num - dx_theo,2,2);

figure('Color','w');
semilogy(t, res, 'k'); grid on;
xlabel('t'); ylabel('||\dot{x}_{num}-\dot{x}_{theo}||');
title('ODE求解残差范数');

%% ODE右端函数，支持任意n维
function dz = ode_fun(t, z)
    n = length(z)/2;
    x = z(1:n);
    y = z(n+1:end);
    
    r2 = x'*x;
    r = sqrt(r2);
    % 数值兜底：理论不会触发，仅防止奇异报错
    if r < 1e-8
        error('||x||趋近0，奇异终止仿真');
    end
    
    dx = -x - y - ones(n,1);
    dy = x - x / r2;
    dz = [dx; dy];
end