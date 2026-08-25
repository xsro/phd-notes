% plot_avg_error.m
% 绘制平均位置误差 hat{x}(t) 的时域演化
% 验证欠阻尼条件下的振荡与过阻尼条件下的单调收敛

clear; clc; close all;

%% 参数
N = 6; mu = 9; d = 5;
v0 = [3; 1]; x0_0 = [0; 20];
x_init = zeros(2, N);
for i = 1:N
    a = (i-1)*pi/3;
    x_init(:, i) = 10 * [cos(a); sin(a)];
end
v_est_init = zeros(2, N);
y0 = [reshape(x_init, [], 1); reshape(v_est_init, [], 1)];
tspan = [0, 60];

%% 欠阻尼情形 (k1=0.5, k2=0.5)
k1 = 0.5; k2 = 0.5;
fprintf('求解欠阻尼情形 (k1=%.1f, k2=%.1f) ...\n', k1, k2);
[t1, y1] = ode45(@(t,y) dyn(t,y,N,k1,k2,mu,d,v0,x0_0), tspan, y0);
x1 = y1(:, 1:2*N);
x_target1 = x0_0' + v0' .* t1;
% 平均位置误差
x_avg1 = zeros(length(t1), 2);
for k = 1:length(t1)
    xk = reshape(x1(k,:), 2, N);
    x_avg1(k,:) = mean(xk, 2)';
end
x_hat1 = x_avg1 - x_target1;
err1 = vecnorm(x_hat1, 2, 2);

%% 过阻尼情形 (k1=3, k2=0.5)
k1 = 3; k2 = 0.5;
fprintf('求解过阻尼情形 (k1=%.1f, k2=%.1f) ...\n', k1, k2);
[t2, y2] = ode45(@(t,y) dyn(t,y,N,k1,k2,mu,d,v0,x0_0), tspan, y0);
x2 = y2(:, 1:2*N);
x_target2 = x0_0' + v0' .* t2;
x_avg2 = zeros(length(t2), 2);
for k = 1:length(t2)
    xk = reshape(x2(k,:), 2, N);
    x_avg2(k,:) = mean(xk, 2)';
end
x_hat2 = x_avg2 - x_target2;
err2 = vecnorm(x_hat2, 2, 2);

%% 绘图: 平均位置误差 ||hat{x}||
figure('Position', [100, 100, 800, 500]);
hold on; grid on; box on;
plot(t1, err1, 'b-', 'LineWidth', 2, 'DisplayName', sprintf('欠阻尼 (k_1=%.1f, k_2=%.1f)', 0.5, 0.5));
plot(t2, err2, 'r-', 'LineWidth', 2, 'DisplayName', sprintf('过阻尼 (k_1=%.1f, k_2=%.1f)', 3, 0.5));
xlabel('时间 t (s)');
ylabel('||\hat{x}(t)||');
title('平均位置误差 ||\hat{x}(t)|| 对比');
legend('Location', 'best');
set(gca, 'FontSize', 12);
hold off;
saveas(gcf, 'kou_avg_error.png');
fprintf('  -> kou_avg_error.png 已保存\n');

%% 绘图: hat{x} 的分量 (欠阻尼)
figure('Position', [100, 100, 800, 500]);
hold on; grid on; box on;
plot(t1, x_hat1(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', '\hat{x}_1');
plot(t1, x_hat1(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', '\hat{x}_2');
% 理论预测包络
alpha = 0.5/2;
omega = sqrt(4*0.5 - 0.5^2)/2;
env = vecnorm(x_hat1(1,:)) * exp(-alpha * t1);
plot(t1, env, 'k--', 'LineWidth', 1, 'DisplayName', sprintf('包络 e^{-%.2ft}', alpha));
plot(t1, -env, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
xlabel('时间 t (s)');
ylabel('\hat{x}(t) 分量');
title('欠阻尼情形: 平均位置误差分量 (含理论包络)');
legend('Location', 'best');
set(gca, 'FontSize', 12);
hold off;
saveas(gcf, 'kou_avg_error_components.png');
fprintf('  -> kou_avg_error_components.png 已保存\n');

%% 绘图: 相位图 (欠阻尼) - hat{x} 在平面上的轨迹
figure('Position', [100, 100, 800, 700]);
hold on; grid on; box on; axis equal;
plot(x_hat1(:,1), x_hat1(:,2), 'b-', 'LineWidth', 1.5);
plot(x_hat1(1,1), x_hat1(1,2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', '起点');
plot(x_hat1(end,1), x_hat1(end,2), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', '终点');
plot(0, 0, 'k+', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', '原点');
xlabel('\hat{x}_1'); ylabel('\hat{x}_2');
title('欠阻尼情形: \hat{x}(t) 平面轨迹 (螺旋收敛)');
legend('Location', 'best');
set(gca, 'FontSize', 12);
hold off;
saveas(gcf, 'kou_avg_error_phase.png');
fprintf('  -> kou_avg_error_phase.png 已保存\n');

fprintf('全部完成.\n');

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
            xij = x(:,i) - x(:,j); r = norm(xij);
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