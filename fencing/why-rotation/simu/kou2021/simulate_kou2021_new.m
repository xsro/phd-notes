% simulate_kou2021_new.m
% 仿真 Kou et al. (2022) "Cooperative Fencing Control of Multiple Vehicles
% for a Moving Target with an Unknown Velocity"
% 控制器 (a): MTF With Average Velocity Tracking
%
% 绘制:
%   1. 每个 vehicle 的轨迹图 (含目标轨迹)
%   2. 每个 vehicle 的速度大小变化曲线
%   3. 每个 vehicle 速度在目标位移切向的分量变化曲线

clear; clc; close all;

%% 参数 (论文 Section IV)
N = 6;          % 车辆数
mu = 9;         % 邻居半径
d = 5;          % 碰撞阈值
k1 = 0.5;       % 吸引力增益
k2 = 0.5;       % 速度估计增益
v0 = [3; 1];    % 目标真实速度 (对车辆未知)
x0_0 = [0; 20]; % 目标初始位置

% 车辆初始位置: 半径为10的圆上均匀分布
x_init = zeros(2, N);
for i = 1:N
    a = (i-1)*pi/3;
    x_init(:, i) = 10 * [cos(a); sin(a)];
end

% 速度估计初始值
v_est_init = zeros(2, N);

% 拼接状态向量: [x_i; v_est_i]
y0 = [reshape(x_init, [], 1); reshape(v_est_init, [], 1)];

% 仿真时间
tspan = [0, 60];

%% ODE 求解
fprintf('正在求解 ODE ...\n');
[t, y] = ode45(@(t,y) dyn(t, y, N, k1, k2, mu, d, v0, x0_0), tspan, y0);
fprintf('求解完成, 共 %d 个时间步.\n', length(t));

%% 提取状态
x_all = y(:, 1:2*N);               % (Nt x 2N) 车辆位置
v_est_all = y(:, 2*N+1:4*N);      % (Nt x 2N) 速度估计
x_target = x0_0' + v0' .* t;      % (Nt x 2) 目标位置

%% 计算每个 vehicle 的控制输入 u_i 和速度
% 同时计算速度分量
u_all = zeros(length(t), 2*N);     % 控制输入 = 实际速度
v_parallel = zeros(length(t), N);  % 平行于 v0 的分量速度大小
v_tangential = zeros(length(t), N); % 垂直于 v0 (切向) 的分量速度大小
v_tang_target = zeros(length(t), N); % 围绕目标运动的切向速度大小 (相对速度)
omega = zeros(length(t), N);       % 相对目标的角速度

v0_hat = v0 / norm(v0);            % 目标位移方向单位向量
v0_perp = [-v0_hat(2); v0_hat(1)]; % 垂直于目标位移方向

for k = 1:length(t)
    xk = reshape(x_all(k, :), 2, N);       % 2 x N
    vk = reshape(v_est_all(k, :), 2, N);   % 2 x N
    xt = x_target(k, :)';                  % 2 x 1
    
    for i = 1:N
        % 计算排斥力 phi_i
        phi = [0; 0];
        for j = 1:N
            if j == i, continue; end
            xij = xk(:,i) - xk(:,j);
            r = norm(xij);
            alpha = collision_avoidance(r, 100, d, mu);
            if alpha > 0
                phi = phi + alpha * xij / r;
            end
        end
        
        % 控制输入 (即车辆实际速度)
        ui = phi + k1*(xt - xk(:,i)) + vk(:,i);
        u_all(k, 2*i-1:2*i) = ui';
        
        % 速度在目标位移方向 (v0) 上的分解
        % 平行分量 (沿 v0 方向)
        v_par = dot(ui, v0_hat);
        v_parallel(k, i) = v_par;
        
        % 切向分量 (垂直于 v0 方向)
        v_tan = dot(ui, v0_perp);
        v_tangential(k, i) = v_tan;
        
        % 围绕目标运动的切向速度 (相对速度的切向分量)
        % 相对速度 u_i - v_0 在垂直于 (x_i - x_0) 方向的分量
        rel_pos = xk(:,i) - xt;
        rel_vel = ui - v0;
        dist2 = norm(rel_pos)^2;
        % 切向速度大小 = ||rel_pos × rel_vel|| / ||rel_pos||
        cross_val = abs(rel_pos(1)*rel_vel(2) - rel_pos(2)*rel_vel(1));
        if dist2 > 1e-10
            v_tang_target(k, i) = cross_val / sqrt(dist2);
            omega(k, i) = cross_val / dist2;
        else
            v_tang_target(k, i) = 0;
            omega(k, i) = 0;
        end
    end
end

% 每个 vehicle 的速度大小
speed = zeros(length(t), N);
for i = 1:N
    speed(:, i) = sqrt(u_all(:, 2*i-1).^2 + u_all(:, 2*i).^2);
end

%% ====================== 绘图 ======================

% 颜色
colors = lines(N);

%% 图1: 每个 vehicle 的轨迹图 (含目标轨迹)
figure('Position', [100, 100, 800, 700]);
hold on; grid on; box on;
axis equal;

% 绘制目标轨迹
h=cell(N+1);
h{N+1}=plot(x_target(:,1), x_target(:,2), 'k-', 'LineWidth', 2.5, 'DisplayName', '目标轨迹');

% 绘制每个 vehicle 的轨迹
for i = 1:N
    xi = x_all(:, 2*i-1);
    yi = x_all(:, 2*i);
    h{i}=plot(xi, yi, '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Vehicle %d', i));
    % 标记起点
    plot(xi(1), yi(1), 'o', 'Color', colors(i,:), 'MarkerSize', 8, ...
         'MarkerFaceColor', colors(i,:));
    % 标记终点
    plot(xi(end), yi(end), 's', 'Color', colors(i,:), 'MarkerSize', 8, ...
         'MarkerFaceColor', colors(i,:));
end

% 标记目标起点和终点
plot(x_target(1,1), x_target(1,2), 'ko', 'MarkerSize', 10, ...
     'MarkerFaceColor', 'k', 'DisplayName', '目标起点');
plot(x_target(end,1), x_target(end,2), 'ks', 'MarkerSize', 10, ...
     'MarkerFaceColor', 'k', 'DisplayName', '目标终点');

% 每隔一定时间绘制目标位置点
step = max(1, floor(length(t)/20));
for k = 1:step:length(t)
    plot(x_target(k,1), x_target(k,2), 'k.', 'MarkerSize', 6);
end

xlabel('x'); ylabel('y');
title('Vehicle 与目标轨迹图');
legend(horzcat(h{:}),'Location', 'best');
set(gca, 'FontSize', 12);
hold off;

%% 图2: 每个 vehicle 的速度大小变化曲线
figure('Position', [100, 100, 800, 500]);
hold on; grid on; box on;
for i = 1:N
    plot(t, speed(:, i), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Vehicle %d', i));
end
xlabel('时间 t (s)');
ylabel('速度大小 ||u_i||');
title('每个 Vehicle 的速度大小变化曲线');
legend('Location', 'best');
set(gca, 'FontSize', 12);
hold off;

%% 图3: 速度在目标位移切向的分量变化曲线
% 目标位移方向为 v0 = [3,1]^T, 切向即垂直于 v0 的方向
figure('Position', [100, 100, 800, 500]);
hold on; grid on; box on;
for i = 1:N
    plot(t, v_tangential(:, i), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Vehicle %d', i));
end
yline(0, 'k--', 'LineWidth', 0.5);
xlabel('时间 t (s)');
ylabel('切向速度分量 (垂直于 v_0)');
title('Vehicle 速度在目标位移切向的分量变化曲线');
legend('Location', 'best');
set(gca, 'FontSize', 12);
hold off;

%% 图4: 围绕目标运动的切向速度 (相对速度的切向分量)
figure('Position', [100, 100, 800, 500]);
hold on; grid on; box on;
for i = 1:N
    plot(t, v_tang_target(:, i), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Vehicle %d', i));
end
xlabel('时间 t (s)');
ylabel('围绕目标的切向速度 (相对)');
title('Vehicle 相对目标的切向速度变化曲线');
legend('Location', 'best');
set(gca, 'FontSize', 12);
hold off;

%% 图5: 角速度
figure('Position', [100, 100, 800, 500]);
hold on; grid on; box on;
for i = 1:N
    plot(t, omega(:, i), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Vehicle %d', i));
end
xlabel('时间 t (s)');
ylabel('\omega_i (rad/s)');
title('每个 Vehicle 相对目标的角速度');
legend('Location', 'best');
set(gca, 'FontSize', 12);
hold off;

%% 图6: 综合图 - 速度的平行分量和切向分量
figure('Position', [100, 100, 1000, 600]);
subplot(2,1,1);
hold on; grid on; box on;
for i = 1:N
    plot(t, v_parallel(:, i), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Vehicle %d', i));
end
yline(norm(v0), 'k--', 'LineWidth', 1, 'DisplayName', sprintf('||v_0||=%.1f', norm(v0)));
xlabel('时间 t (s)');
ylabel('平行分量 (沿 v_0)');
title('Vehicle 速度沿目标位移方向的分量');
legend('Location', 'best');
set(gca, 'FontSize', 11);
hold off;

subplot(2,1,2);
hold on; grid on; box on;
for i = 1:N
    plot(t, v_tangential(:, i), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Vehicle %d', i));
end
yline(0, 'k--', 'LineWidth', 0.5);
xlabel('时间 t (s)');
ylabel('切向分量 (⊥ v_0)');
title('Vehicle 速度垂直于目标位移方向的分量');
legend('Location', 'best');
set(gca, 'FontSize', 11);
hold off;

%% 保存图片
fprintf('正在保存图片 ...\n');

% 图1: 轨迹图
figure(1);
saveas(gcf, 'kou_trajectories.png');
fprintf('  -> kou_trajectories.png 已保存\n');

% 图2: 速度大小
figure(2);
saveas(gcf, 'kou_speed.png');
fprintf('  -> kou_speed.png 已保存\n');

% 图3: 切向速度分量 (垂直于v0)
figure(3);
saveas(gcf, 'kou_tangential_to_v0.png');
fprintf('  -> kou_tangential_to_v0.png 已保存\n');

% 图4: 围绕目标切向速度
figure(4);
saveas(gcf, 'kou_tangential_around_target.png');
fprintf('  -> kou_tangential_around_target.png 已保存\n');

% 图5: 角速度
figure(5);
saveas(gcf, 'kou_omega.png');
fprintf('  -> kou_omega.png 已保存\n');

% 图6: 综合图
figure(6);
saveas(gcf, 'kou_velocity_components.png');
fprintf('  -> kou_velocity_components.png 已保存\n');

fprintf('仿真结束.\n');

%% ====================== ODE 动力学 ======================
function dydt = dyn(t, y, N, k1, k2, mu, d, v0, x0_0)
    % 解析状态
    x = reshape(y(1:2*N), 2, N);          % 位置 (2 x N)
    v_est = reshape(y(2*N+1:4*N), 2, N);  % 速度估计 (2 x N)
    
    % 目标当前位置
    x_target = x0_0 + v0 * t;  % 2 x 1
    
    dx = zeros(2, N);
    dv = zeros(2, N);
    
    for i = 1:N
        % 排斥力 phi_i
        phi = [0; 0];
        for j = 1:N
            if j == i, continue; end
            xij = x(:,i) - x(:,j);
            r = norm(xij);
            alpha = collision_avoidance(r, 100, d, mu);
            if alpha > 0
                phi = phi + alpha * xij / r;
            end
        end
        
        % 控制器 (a): u_i = phi_i + k1*(x0 - x_i) + v_i
        dx(:,i) = phi + k1*(x_target - x(:,i)) + v_est(:,i);
        
        % 速度估计更新: \dot{v}_i = k2*(x0 - x_i)
        dv(:,i) = k2*(x_target - x(:,i));
    end
    
    dydt = [reshape(dx, [], 1); reshape(dv, [], 1)];
end