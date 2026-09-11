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

%% ---- Figure 5: Arm configuration (base + arm at initial and final times) ----
% Reconstruct joint history
q_all = qd + err;

% DH parameters (from 严宇新 paper, same as P structure)
% Link vectors a_i and b_i [m], columns: B0(base)..B7(last link)
avec = [0,   0.6, 1.5, 1.5, 0,    0,   0,   0.3;
        0,   0,   0,   0,  -0.5, 0,   0,   0;
        0,   0,   0.6, 0.6, 0,    0.5, 0.5, 0 ];
bvec = [0.6, 0.6, 1.5, 1.5, 0,    0,   0,   0.3;
        0,   0,   0,   0,  -0.5, 0,   0,   0;
        0,   0,   0.6, 0.6, 0,    0.5, 0.5, 0 ];
mass_all = [1000, 4.25, 7, 7, 4.25, 4.25, 4.25, 4.25];
mass_links = mass_all(2:end);
total_mass = sum(mass_all);

% Joint axes (3x7, columns = axes for joints 1..7)
axes_local = [0 0 1; 0 1 0; 0 1 0; 1 0 0; 0 1 0; 1 0 0; 0 0 1].';

% Compute arm configuration at each time step
N_full = size(t, 2);
base_pos = zeros(3, N_full);
ee_pos = zeros(3, N_full);

% Use first time step as reference for momentum conservation
[nodes0, com0] = arm_nodes_and_com(q_all(:,1), avec, bvec, axes_local, mass_links);
weighted_com0 = com0 * mass_links(:);

for k = 1:N_full
    [nodes_rel, com_rel] = arm_nodes_and_com(q_all(:,k), avec, bvec, axes_local, mass_links);
    weighted_com = com_rel * mass_links(:);
    base_pos(:,k) = (weighted_com0 - weighted_com) / total_mass;
    ee_pos(:,k) = nodes_rel(:,end) + base_pos(:,k);
end

% Indices for initial and final
idx0 = 1;
node_frames = round(linspace(1, N_full, 6));

% Compute base-relative nodes for initial and final
[nodes_initial, ~] = arm_nodes_and_com(q_all(:,1), avec, bvec, axes_local, mass_links);
[nodes_final_rel, ~] = arm_nodes_and_com(q_all(:,end), avec, bvec, axes_local, mass_links);
nodes_final = nodes_final_rel + base_pos(:,end);
nodes_initial = nodes_initial + base_pos(:,1);

base_drift = norm(base_pos(:,end) - base_pos(:,1));

% XY planar projection of base trajectory
figure('Name', 'Arm configuration', 'Color', 'w', ...
       'Position', [100, 100, 1350, 700]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

% Left: 3D view
nexttile;
plot3(ee_pos(1,:), ee_pos(2,:), ee_pos(3,:), 'b', 'LineWidth', 1.5);
hold on;
plot3(base_pos(1,:), base_pos(2,:), base_pos(3,:), 'k:', 'LineWidth', 1.2);
plot3(nodes_initial(1,:), nodes_initial(2,:), nodes_initial(3,:), ...
      '-o', 'Color', [0.10 0.60 0.10], 'LineWidth', 1.8, ...
      'MarkerSize', 5, 'MarkerFaceColor', [0.10 0.60 0.10]);
plot3(nodes_final(1,:), nodes_final(2,:), nodes_final(3,:), ...
      '-s', 'Color', [0.60 0.15 0.50], 'LineWidth', 1.8, ...
      'MarkerSize', 5, 'MarkerFaceColor', [0.60 0.15 0.50]);
plot3(nodes_initial(1,1), nodes_initial(2,1), nodes_initial(3,1), ...
      'kp', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
plot3(nodes_final(1,1), nodes_final(2,1), nodes_final(3,1), ...
      'kh', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
plot3(nodes_final(1,end), nodes_final(2,end), nodes_final(3,end), ...
      'kd', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
for i = 2:size(nodes_initial,2)-1
    text(nodes_initial(1,i), nodes_initial(2,i), nodes_initial(3,i), ...
         sprintf('  J%d', i-1), 'FontSize', 8, 'Color', [0.10 0.60 0.10]);
end
text(nodes_initial(1,1), nodes_initial(2,1), nodes_initial(3,1), ...
     '  Base(0)', 'FontSize', 9, 'Color', 'k', 'FontWeight', 'bold');
text(nodes_final(1,1), nodes_final(2,1), nodes_final(3,1), ...
     '  Base(T)', 'FontSize', 9, 'Color', 'k', 'FontWeight', 'bold');
text(nodes_final(1,end), nodes_final(2,end), nodes_final(3,end), ...
     '  EE', 'FontSize', 9, 'Color', 'k', 'FontWeight', 'bold');
grid on;
axis equal;
view(42, 24);
xlabel('$x$ [m]'); ylabel('$y$ [m]'); zlabel('$z$ [m]');
title(sprintf('Free-floating arm (base drift %.3f m)', base_drift));
legend({'EE trajectory', 'Base trajectory', 'Initial arm', 'Final arm', ...
        'Initial base', 'Final base', 'Final EE'}, ...
        'Location', 'southoutside', 'NumColumns', 2, 'FontSize', 8);

% Right: joint angle comparison (initial vs final)
nexttile;
joint_labels = arrayfun(@(i)sprintf('$q_%d$', i), 1:n, 'UniformOutput', false);
bar(1:n, [q_all(:,1), q_all(:,end)], 'grouped');
set(gca, 'XTickLabel', joint_labels);
xlabel('Joint'); ylabel('$q_i$ [rad]');
grid on; title('Joint angles: initial vs steady state');
legend({'Initial ($t=0$)', 'Steady ($t=10$s)'}, 'Location', 'best');

print(gcf, fullfile(datadir, 'arm_configuration'), '-dpng', '-r200');
print(gcf, fullfile(datadir, 'arm_configuration'), '-dpdf');
fprintf('Saved: arm_configuration.{png,pdf}\n');

%% ---- Figure 6: Individual joint tracking ----
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

%% ---- Figure 7: Joint velocity tracking ----
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
fprintf('Generated %d figures in 严宇新 style.\n', 7);

%% ----------------------------------------------------------------
%% Helper functions
%% ----------------------------------------------------------------

function [nodes, link_com] = arm_nodes_and_com(q, avec, bvec, axes_local, mass_links)
% Compute base-relative joint nodes and link COM positions.
%   q: 7x1 joint angles
%   avec: 3x8 link a-vectors (column 1 = base, columns 2..8 = links)
%   axes_local: 3x7 joint axes
%   Returns:
%   nodes: 3x8 node positions (node 1 = base origin, node 8 = end-effector)
%   link_com: 3x7 link COM positions in base-relative frame

    n = size(q, 1);
    R = eye(3);
    p = zeros(3, 1);
    nodes = zeros(3, n+1);
    link_com = zeros(3, n);
    nodes(:,1) = p;
    for i = 1:n
        R = R * axis_angle_rotation(axes_local(:, i), q(i));
        link_vec = avec(:, i+1);
        if norm(link_vec) < 1e-9
            link_vec = bvec(:, i+1);
        end
        link_com(:, i) = p + 0.5 * R * link_vec;
        p = p + R * link_vec;
        nodes(:, i+1) = p;
    end
end

function R = axis_angle_rotation(axis, theta)
    axis = axis / norm(axis);
    K = [0, -axis(3), axis(2);
         axis(3), 0, -axis(1);
         -axis(2), axis(1), 0];
    R = eye(3) + sin(theta)*K + (1 - cos(theta))*(K*K);
end