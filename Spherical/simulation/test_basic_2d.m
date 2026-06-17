%% test_basic_2d.m  —  Basic 2D verification of spherical fencing
%  5 agents, stationary target, fully connected graph.

clear; clc; close all;

%% Parameters
dim = 2;
n   = 5;
r   = 3;

% Gain condition: kr/ka >= 2*n*r^2/(n-1) = 2*5*9/4 = 22.5
ka = 1;
kr = 50;   % kr/ka = 50 > 22.5  ✓
ko = 10;

T  = 50;
dt = 0.005;

%% Initial conditions
% Target at origin
P0_init = [0, 0];

% Agents placed at distance 6 from target, separated by 72°
angles = (0:n-1)' * (2*pi/n) + pi/10;  % slight offset to avoid symmetry
P_init = 6 * [cos(angles), sin(angles)];

% Verify spanning condition: rank should be 2
assert(rank(P_init - P0_init) == dim, 'Initial positions do not span R^2!');

%% Fully connected graph
adj = ones(n) - eye(n);
w   = ones(n);

%% Run simulation
fprintf('Running 2D simulation: %d agents, r=%.1f, T=%.0f ...\n', n, r, T);
[t, P_hist, P0_hist, dist_hist, centroid_err] = ...
    spherical_fencing_sim(dim, n, r, ka, kr, ko, T, dt, ...
                          P_init, P0_init, [], adj, w);
fprintf('Done. Final centroid error: %.4f\n', centroid_err(end));

%% --- Figure 1: Trajectory Plot ---
figure('Name', '2D Trajectories', 'Position', [100, 100, 700, 600]);
hold on; axis equal; grid on;

% Target sphere
theta = linspace(0, 2*pi, 200);
plot(r*cos(theta), r*sin(theta), 'k--', 'LineWidth', 1.2);

% Target position
plot(P0_init(1), P0_init(2), 'kp', 'MarkerSize', 14, ...
     'MarkerFaceColor', 'y', 'DisplayName', 'Target');

% Agent trajectories and final positions
colors = lines(n);
for i = 1:n
    traj = squeeze(P_hist(:, i, :));
    plot(traj(:,1), traj(:,2), '-', 'Color', colors(i,:), ...
         'LineWidth', 0.8, 'HandleVisibility', 'off');
    plot(P_init(i,1), P_init(i,2), 'o', 'Color', colors(i,:), ...
         'MarkerSize', 8, 'MarkerFaceColor', colors(i,:), ...
         'DisplayName', sprintf('Agent %d (init)', i));
    plot(traj(end,1), traj(end,2), 's', 'Color', colors(i,:), ...
         'MarkerSize', 10, 'MarkerFaceColor', colors(i,:), ...
         'DisplayName', sprintf('Agent %d (final)', i));
end

xlabel('x'); ylabel('y');
title(sprintf('2D Spherical Fencing (n=%d, r=%.1f, k_r/k_a=%.0f)', n, r, kr/ka));
legend('Location', 'bestoutside');
hold off;
saveas(gcf, 'fig_basic_2d_trajectory.png');

%% --- Figure 2: Convergence Plots ---
figure('Name', '2D Convergence', 'Position', [150, 150, 900, 400]);

% Subplot 1: distance to target
subplot(1,2,1);
hold on; grid on;
for i = 1:n
    plot(t, dist_hist(:,i), '-', 'Color', colors(i,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Agent %d', i));
end
yline(r, 'k--', 'LineWidth', 1.5, 'DisplayName', sprintf('r = %.1f', r));
xlabel('Time'); ylabel('||p_i - p_0||');
title('Distance to Target');
legend('Location', 'best');
hold off;

% Subplot 2: centroid error
subplot(1,2,2);
hold on; grid on;
plot(t, centroid_err, 'b-', 'LineWidth', 1.5);
yline(0.01, 'r--', 'LineWidth', 1, 'DisplayName', '0.01 threshold');
xlabel('Time'); ylabel('||\bar{p} - p_0||');
title('Centroid Error');
legend('Location', 'best');
hold off;

saveas(gcf, 'fig_basic_2d_convergence.png');

%% --- Print Results ---
fprintf('\n=== Final State ===\n');
for i = 1:n
    fprintf('Agent %d: dist = %.4f (error = %.4f)\n', ...
            i, dist_hist(end,i), abs(dist_hist(end,i) - r));
end
fprintf('Centroid error: %.6f\n', centroid_err(end));

if all(abs(dist_hist(end,:) - r) < 0.05) && centroid_err(end) < 0.05
    fprintf('\n✓ CONVERGENCE VERIFIED\n');
else
    fprintf('\n✗ CONVERGENCE NOT REACHED — check parameters\n');
end
