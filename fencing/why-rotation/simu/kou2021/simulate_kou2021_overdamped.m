% simulate_kou2021_overdamped.m
% 过阻尼情形 (k1=3, k2=0.5, k1^2=9 > 4k2=2)
% 用于对比验证: 过阻尼时队形不旋转

clear; clc; close all;

%% 参数
N = 6; mu = 9; d = 5; k1 = 3; k2 = 0.5;
v0 = [3; 1]; x0_0 = [0; 20];
x_init = zeros(2, N);
for i = 1:N
    a = (i-1)*pi/3;
    x_init(:, i) = 10 * [cos(a); sin(a)];
end
v_est_init = zeros(2, N);
y0 = [reshape(x_init, [], 1); reshape(v_est_init, [], 1)];
tspan = [0, 60];

%% ODE 求解
fprintf('正在求解 ODE (过阻尼, k1=3, k2=0.5) ...\n');
[t, y] = ode45(@(t,y) dyn(t, y, N, k1, k2, mu, d, v0, x0_0), tspan, y0);
fprintf('求解完成, 共 %d 个时间步.\n', length(t));

%% 提取状态
x_all = y(:, 1:2*N);
x_target = x0_0' + v0' .* t;

%% 计算角速度
omega = zeros(length(t), N);
for k = 1:length(t)
    xk = reshape(x_all(k, :), 2, N);
    vk = reshape(y(k, 2*N+1:4*N), 2, N);
    xt = x_target(k, :)';
    for i = 1:N
        phi = [0; 0];
        for j = 1:N
            if j == i, continue; end
            xij = xk(:,i) - xk(:,j); r = norm(xij);
            alpha = collision_avoidance(r, 100, d, mu);
            if alpha > 0
                phi = phi + alpha * xij / r;
            end
        end
        ui = phi + k1*(xt - xk(:,i)) + vk(:,i);
        rel_pos = xk(:,i) - xt;
        rel_vel = ui - v0;
        cross_val = rel_pos(1)*rel_vel(2) - rel_pos(2)*rel_vel(1);
        dist2 = norm(rel_pos)^2;
        omega(k, i) = cross_val / dist2;
    end
end

%% 绘图: 轨迹图
colors = lines(N);
figure('Position', [100, 100, 800, 700]);
hold on; grid on; box on; axis equal;
h=cell(N+1);
h{N+1}=plot(x_target(:,1), x_target(:,2), 'k-', 'LineWidth', 2.5, 'DisplayName', '目标轨迹');

for i = 1:N
    xi = x_all(:, 2*i-1); yi = x_all(:, 2*i);
    h{i}=plot(xi, yi, '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Vehicle %d', i));
    plot(xi(1), yi(1), 'o', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
    plot(xi(end), yi(end), 's', 'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:));
end
plot(x_target(1,1), x_target(1,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(x_target(end,1), x_target(end,2), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
xlabel('x'); ylabel('y');
title('过阻尼情形 (k_1=3, k_2=0.5): Vehicle 与目标轨迹图');
legend(horzcat(h{:}),'Location', 'best');
set(gca, 'FontSize', 12);
hold off;
saveas(gcf, 'kou_trajectories_overdamped.png');
fprintf('  -> kou_trajectories_overdamped.png 已保存\n');

%% 绘图: 角速度
figure('Position', [100, 100, 800, 500]);
hold on; grid on; box on;
for i = 1:N
    plot(t, omega(:, i), '-', 'Color', colors(i,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Vehicle %d', i));
end
xlabel('时间 t (s)'); ylabel('\omega_i (rad/s)');
title('过阻尼情形 (k_1=3, k_2=0.5): 角速度');
legend('Location', 'best');
set(gca, 'FontSize', 12);
hold off;
saveas(gcf, 'kou_omega_overdamped.png');
fprintf('  -> kou_omega_overdamped.png 已保存\n');

fprintf('仿真结束.\n');

%% ODE 动力学
function dydt = dyn(t, y, N, k1, k2, mu, d, v0, x0_0)
    x = reshape(y(1:2*N), 2, N);
    v_est = reshape(y(2*N+1:4*N), 2, N);
    x_target = x0_0 + v0 * t;
    dx = zeros(2, N); dv = zeros(2, N);
    for i = 1:N
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
        dx(:,i) = phi + k1*(x_target - x(:,i)) + v_est(:,i);
        dv(:,i) = k2*(x_target - x(:,i));
    end
    dydt = [reshape(dx, [], 1); reshape(dv, [], 1)];
end