% plot_cfpdf_sim.m
% 绘图脚本：加载仿真数据并生成论文图片
% 用法：先运行 run_cfpdf_sim.m，再运行本脚本

clear; clc; close all;

%% 加载数据
datadir = '/tmp/sim_results';
load(fullfile(datadir, 'sim_data.mat'));

%% 图片 1: 关节位置/速度跟踪误差
figure('Position', [50, 50, 1000, 600]);
subplot(2,1,1);
plot(t, err', 'LineWidth', 0.8);
ylabel('e (rad)');
grid on; title('关节位置跟踪误差 e(t)=q(t)-q_d(t)');
xlim([0 Tf]);
subplot(2,1,2);
plot(t, derr', 'LineWidth', 0.8);
ylabel('de (rad/s)');
xlabel('t (s)');
grid on; title('关节速度跟踪误差 de(t)=dq(t)-dq_d(t)');
xlim([0 Tf]);
saveas(gcf, fullfile(datadir, 'fig_tracking_errors.png'));
fprintf('已保存: fig_tracking_errors.png\n');

%% 图片 2: 误差 2-范数
figure('Position', [50, 50, 1000, 600]);
subplot(2,1,1);
plot(t, err_norm, 'b-', 'LineWidth', 1.5);
ylabel('||e||_2 (rad)');
grid on; title('位置误差 2-范数');
xline(To, 'r--', 'T_o');
xline(To+Tsp, 'g--', 'T_o+T_s');
xlim([0 Tf]); set(gca, 'YScale', 'log');
subplot(2,1,2);
plot(t, derr_norm, 'r-', 'LineWidth', 1.5);
ylabel('||de||_2 (rad/s)');
xlabel('t (s)');
grid on; title('速度误差 2-范数');
xline(To, 'r--', 'T_o');
xline(To+2*h, 'm--', 'T_o+2h');
xline(To+Tsp, 'g--', 'T_o+T_s');
xlim([0 Tf]); set(gca, 'YScale', 'log');
saveas(gcf, fullfile(datadir, 'fig_error_norms.png'));
fprintf('已保存: fig_error_norms.png\n');

%% 图片 3: 关节力矩
figure('Position', [50, 50, 1000, 400]);
plot(t, tau_h', 'LineWidth', 0.6);
ylabel('tau_i (N-m)');
xlabel('t (s)');
grid on; title('关节力矩');
xlim([0 Tf]);
saveas(gcf, fullfile(datadir, 'fig_torque.png'));
fprintf('已保存: fig_torque.png\n');

%% 图片 4: 全部关节误差（放大初始段和终端段）
figure('Position', [50, 50, 1200, 800]);
subplot(2,2,1);
plot(t, err', 'LineWidth', 0.8);
ylabel('e (rad)'); grid on;
title('位置误差（全程）'); xlim([0 Tf]);
xline(To, 'r--', 'T_o'); xline(To+Tsp, 'g--', 'T_o+T_s');

subplot(2,2,2);
plot(t, derr', 'LineWidth', 0.8);
ylabel('de (rad/s)'); grid on;
title('速度误差（全程）'); xlim([0 Tf]);
xline(To, 'r--', 'T_o'); xline(To+2*h, 'm--', 'T_o+2h');
xline(To+Tsp, 'g--', 'T_o+T_s');

subplot(2,2,3);
idx_zoom = find(t >= To & t <= To+Tsp+1);
plot(t(idx_zoom), err(:,idx_zoom)', 'LineWidth', 1.2);
ylabel('e (rad)'); grid on;
title('位置误差（t in [T_o, T_o+T_s+1]）');
xline(To+Tsp, 'g--', 'T_o+T_s');

subplot(2,2,4);
idx_zoom2 = find(t >= To+Tsp-1 & t <= Tf);
semilogy(t(idx_zoom2), abs(err(:,idx_zoom2))', 'LineWidth', 1.0);
ylabel('|e| (rad)'); grid on;
title('位置误差绝对值（t in [T_o+T_s-1, T_f]，对数坐标）');
xlabel('t (s)');
saveas(gcf, fullfile(datadir, 'fig_detailed.png'));
fprintf('已保存: fig_detailed.png\n');

%% 输出说明
fprintf('\n=== 图片列表 ===\n');
fprintf('1. fig_tracking_errors.png    — 位置/速度跟踪误差时间历程\n');
fprintf('2. fig_error_norms.png         — 误差 2-范数（对数坐标，含 T_o, T_s 标记）\n');
fprintf('3. fig_torque.png              — 关节力矩\n');
fprintf('4. fig_detailed.png            — 详细分析（全程 + 放大 + 对数尺度）\n');