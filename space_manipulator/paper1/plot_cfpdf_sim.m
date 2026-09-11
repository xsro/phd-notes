%% plot_cfpdf_sim.m
% Plot simulation results in the style of 严宇新 paper (7x3 tiled summary)
% Usage: run run_cfpdf_sim.m first, then this script.

clear; clc; close all;

%% LaTeX interpreter (matching 严宇新 style)
set(groot, 'DefaultTextInterpreter', 'latex');
set(groot, 'DefaultAxesTickLabelInterpreter', 'latex');
set(groot, 'DefaultLegendInterpreter', 'latex');

%% Load data
datadir = '/tmp/sim_results';
S = load(fullfile(datadir, 'sim_data.mat'));
t = S.t; err = S.err; derr = S.derr; tau_h = S.tau_h;
qd = S.qd; dqd = S.dqd; ddqd = S.ddqd;
err_norm = S.err_norm; derr_norm = S.derr_norm; tau_max = S.tau_max;
a = S.a; tau2 = S.tau2; Tsp = S.Tsp; h = S.h;
om = S.om; rho = S.rho; To = S.To; Ts = S.Ts; Tf = S.Tf; n = S.n;

% Reconstruct joint position and velocity
q = qd + err;
dq = dqd + derr;

%% ---- Figure 1: Joint tracking, errors and torques (7x3 tiled) ----
% Matching the style of 严宇新 paper's fig:joint_summary
joint_colors = lines(n);

figure('Name', 'Joint tracking, errors and torques', 'Color', 'w', ...
       'Position', [80, 80, 1700, 1050]);
tiledlayout(7, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

for i = 1:n
    % Column 1: Joint position tracking
    nexttile((i-1)*3 + 1);
    plot(t, q(i,:), 'Color', joint_colors(i,:), 'LineWidth', 1.0);
    hold on;
    plot(t, qd(i,:), '--', 'Color', joint_colors(i,:), 'LineWidth', 0.9);
    grid on;
    ylabel(sprintf('$q_%d$', i));
    if i == 1
        title('Joint position tracking');
        legend({'Actual', 'Desired'}, 'Location', 'best', 'FontSize', 7);
    end
    if i == n
        xlabel('$t$ [s]');
    else
        set(gca, 'XTickLabel', []);
    end
    xlim([0 Tf]);

    % Column 2: Joint velocity tracking
    nexttile((i-1)*3 + 2);
    plot(t, dq(i,:), 'Color', joint_colors(i,:), 'LineWidth', 1.0);
    hold on;
    plot(t, dqd(i,:), '--', 'Color', joint_colors(i,:), 'LineWidth', 0.9);
    grid on;
    ylabel(sprintf('$\\dot q_%d$', i));
    if i == 1
        title('Joint velocity tracking');
        legend({'Actual', 'Desired'}, 'Location', 'best', 'FontSize', 7);
    end
    if i == n
        xlabel('$t$ [s]');
    else
        set(gca, 'XTickLabel', []);
    end
    xlim([0 Tf]);
end

% Column 3, rows 1-2: Position tracking errors
nexttile(3, [2 1]);
plot(t, err, 'LineWidth', 0.95);
grid on;
xlabel('$t$ [s]');
ylabel('$e_i$ [rad]');
title('Position tracking error');
legend(arrayfun(@(i)sprintf('$e_%d$', i), 1:n, 'UniformOutput', false), ...
       'Location', 'eastoutside', 'FontSize', 7);
xlim([0 Tf]);

% Column 3, rows 3-4: Velocity tracking errors
nexttile(9, [2 1]);
plot(t, derr, 'LineWidth', 0.95);
grid on;
xlabel('$t$ [s]');
ylabel('$\\dot e_i$ [rad/s]');
title('Velocity tracking error');
legend(arrayfun(@(i)sprintf('$\\dot e_%d$', i), 1:n, 'UniformOutput', false), ...
       'Location', 'eastoutside', 'FontSize', 7);
xlim([0 Tf]);

% Column 3, rows 5-7: Control torque
nexttile(15, [3 1]);
plot(t, tau_h, 'LineWidth', 0.95);
grid on;
xlabel('$t$ [s]');
ylabel('$\\tau_i$ [N m]');
title('Control torque');
legend(arrayfun(@(i)sprintf('$\\tau_%d$', i), 1:n, 'UniformOutput', false), ...
       'Location', 'eastoutside', 'FontSize', 7);
xlim([0 Tf]);

% Save
print(gcf, fullfile(datadir, 'joint_tracking_error_torque_summary'), '-dpng', '-r200');
print(gcf, fullfile(datadir, 'joint_tracking_error_torque_summary'), '-dpdf');
fprintf('Saved: joint_tracking_error_torque_summary.{png,pdf}\n');

%% ---- Figure 2: Error norms ----
figure('Name', 'Error norm', 'Color', 'w', 'Position', [100, 100, 600, 400]);
plot(t, err_norm, 'b', 'LineWidth', 1.5);
hold on;
plot(t, derr_norm, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('$t$ [s]');
ylabel('Norm');
legend({'$\\|e\\|$', '$\\|\\dot e\\|$'}, 'Location', 'best');
title('Tracking error norms');
xlim([0 Tf]);
print(gcf, fullfile(datadir, 'error_norm'), '-dpng', '-r200');
print(gcf, fullfile(datadir, 'error_norm'), '-dpdf');
fprintf('Saved: error_norm.{png,pdf}\n');

%% ---- Figure 3: Error norms (log scale, with To and To+Ts markers) ----
figure('Name', 'Error norm (log)', 'Color', 'w', 'Position', [100, 100, 700, 500]);
semilogy(t, err_norm + 1e-10, 'b', 'LineWidth', 1.5);
hold on;
semilogy(t, derr_norm + 1e-10, 'r--', 'LineWidth', 1.5);
xline(To, 'k:', '$T_o$', 'LabelOrientation', 'horizontal', 'FontSize', 10);
xline(To + Tsp, 'k--', '$T_o+T_s$', 'LabelOrientation', 'horizontal', 'FontSize', 10);
grid on;
xlabel('$t$ [s]');
ylabel('Norm');
legend({'$\\|e\\|$', '$\\|\\dot e\\|$'}, 'Location', 'northeast');
title('Tracking error norms (log scale)');
xlim([0 Tf]);
print(gcf, fullfile(datadir, 'error_norm_log'), '-dpng', '-r200');
print(gcf, fullfile(datadir, 'error_norm_log'), '-dpdf');
fprintf('Saved: error_norm_log.{png,pdf}\n');

%% ---- Figure 4: Control torque (single figure, matching 严宇新 style) ----
figure('Name', 'Control torque', 'Color', 'w', 'Position', [100, 100, 700, 400]);
plot(t, tau_h, 'LineWidth', 1.0);
grid on;
xlabel('$t$ [s]');
ylabel('$\\tau_i$ [N m]');
title('Control torque');
legend(arrayfun(@(i)sprintf('$\\tau_%d$', i), 1:n, 'UniformOutput', false), ...
       'Location', 'eastoutside', 'FontSize', 8);
xlim([0 Tf]);
print(gcf, fullfile(datadir, 'control_torque'), '-dpng', '-r200');
print(gcf, fullfile(datadir, 'control_torque'), '-dpdf');
fprintf('Saved: control_torque.{png,pdf}\n');

%% ---- Figure 5: Individual joint tracking (two columns, 严宇新 style) ----
figure('Name', 'Joint position tracking', 'Color', 'w', ...
       'Position', [100, 100, 1000, 1000]);
tiledlayout(4, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:n
    nexttile;
    plot(t, q(i,:), 'Color', joint_colors(i,:), 'LineWidth', 1.0);
    hold on;
    plot(t, qd(i,:), '--', 'Color', joint_colors(i,:), 'LineWidth', 0.9);
    grid on;
    xlabel('$t$ [s]');
    ylabel(sprintf('$q_%d$', i));
    if i == 1
        legend({'Actual', 'Desired'}, 'Location', 'best', 'FontSize', 7);
    end
    xlim([0 Tf]);
end
print(gcf, fullfile(datadir, 'joint_position_tracking'), '-dpng', '-r200');
print(gcf, fullfile(datadir, 'joint_position_tracking'), '-dpdf');
fprintf('Saved: joint_position_tracking.{png,pdf}\n');

%% ---- Figure 6: Joint velocity tracking ----
figure('Name', 'Joint velocity tracking', 'Color', 'w', ...
       'Position', [100, 100, 1000, 1000]);
tiledlayout(4, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for i = 1:n
    nexttile;
    plot(t, dq(i,:), 'Color', joint_colors(i,:), 'LineWidth', 1.0);
    hold on;
    plot(t, dqd(i,:), '--', 'Color', joint_colors(i,:), 'LineWidth', 0.9);
    grid on;
    xlabel('$t$ [s]');
    ylabel(sprintf('$\\dot q_%d$', i));
    if i == 1
        legend({'Actual', 'Desired'}, 'Location', 'best', 'FontSize', 7);
    end
    xlim([0 Tf]);
end
print(gcf, fullfile(datadir, 'joint_velocity_tracking'), '-dpng', '-r200');
print(gcf, fullfile(datadir, 'joint_velocity_tracking'), '-dpdf');
fprintf('Saved: joint_velocity_tracking.{png,pdf}\n');

%% Done
fprintf('\n=== All figures saved to %s ===\n', datadir);
fprintf('Generated %d figures in 严宇新 style.\n', 6);