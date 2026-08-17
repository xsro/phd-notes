%% test_moving_target.m  —  Verify tracking of a moving target
%  3D, 8 agents, target moving at constant velocity.

clear; clc; close all;

%% Parameters
dim = 3;
n   = 6;
r   = 3;

ka = 1;
kr = 60;   % threshold = 2*8*9/7 ≈ 20.6, so 60 > 20.6  ✓
ko = 10;

T  = 60;
dt = 0.005;

%% Initial conditions
P0_init = [0, 0, 0];
P0_vel  = [0.3, 0.2, 0.1];  % constant target velocity

% 8 agents on two rings (z=+2 and z=-2) to span R^3
angles = (0:2)' * (2*pi/4);
P_ring1 = 5 * [cos(angles), sin(angles),  2*ones(3,1)];
P_ring2 = 5 * [cos(angles+pi/4), sin(angles+pi/4), -2*ones(3,1)];
P_init  = [P_ring1; P_ring2];

assert(rank(P_init - P0_init) == dim, 'Initial positions do not span R^3!');

%% Fully connected graph
adj = ones(n) - eye(n);
w   = ones(n);

%% Run simulation
fprintf('Running 3D moving-target simulation: %d agents, T=%.0f ...\n', n, T);
[t, P_hist, P0_hist, dist_hist, centroid_err] = ...
    spherical_fencing_sim(dim, n, r, ka, kr, ko, T, dt, ...
                          P_init, P0_init, P0_vel, adj, w);
fprintf('Done. Final centroid error: %.4f\n', centroid_err(end));

%% --- Figure 1: 3D Trajectory ---
figure('Name', '3D Moving Target', 'Position', [100, 100, 800, 700]);
hold on; grid on;

% Target trajectory
plot3(P0_hist(:,1), P0_hist(:,2), P0_hist(:,3), ...
      'k-', 'LineWidth', 2, 'DisplayName', 'Target path');
plot3(P0_init(1), P0_init(2), P0_init(3), 'kp', 'MarkerSize', 14, ...
      'MarkerFaceColor', 'y', 'DisplayName', 'Target (init)');

% Agent trajectories
colors = lines(n);
for i = 1:n
    traj = squeeze(P_hist(:, i, :));
    plot3(traj(:,1), traj(:,2), traj(:,3), '-', ...
          'Color', colors(i,:), 'LineWidth', 0.6, ...
          'HandleVisibility', 'off');
    plot3(traj(1,1), traj(1,2), traj(1,3), 'o', 'Color', colors(i,:), ...
          'MarkerSize', 7, 'MarkerFaceColor', colors(i,:), ...
          'DisplayName', sprintf('Agent %d', i));
    plot3(traj(end,1), traj(end,2), traj(end,3), 's', 'Color', colors(i,:), ...
          'MarkerSize', 9, 'MarkerFaceColor', colors(i,:), ...
          'HandleVisibility', 'off');
end

% Draw a sphere at the final target position
[P0s_x, P0s_y, P0s_z] = sphere(30);
P0_final = P0_hist(end,:);
surf(P0_final(1)+r*P0s_x, P0_final(2)+r*P0s_y, P0_final(3)+r*P0s_z, ...
     'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', [0.5 0.5 1], ...
     'DisplayName', 'Target sphere (final)');

xlabel('x'); ylabel('y'); zlabel('z');
title(sprintf('3D Moving Target (v = [%.1f, %.1f, %.1f])', P0_vel));
legend('Location', 'bestoutside');
view(30, 25);
axis equal;
hold off;
saveas(gcf, 'fig_moving_target_3d.png');

%% --- Figure 2: Convergence ---
figure('Name', 'Moving Target Convergence', 'Position', [150, 150, 900, 400]);

subplot(1,2,1);
hold on; grid on;
for i = 1:n
    plot(t, dist_hist(:,i), '-', 'Color', colors(i,:), 'LineWidth', 1, ...
         'DisplayName', sprintf('Agent %d', i));
end
yline(r, 'k--', 'LineWidth', 1.5, 'DisplayName', sprintf('r = %.1f', r));
xlabel('Time'); ylabel('||p_i - p_0||');
title('Distance to Target');
legend('Location', 'best');
hold off;

subplot(1,2,2);
hold on; grid on;
plot(t, centroid_err, 'b-', 'LineWidth', 1.5);
yline(0.01, 'r--', 'LineWidth', 1, 'DisplayName', '0.01 threshold');
xlabel('Time'); ylabel('||\bar{p} - p_0||');
title('Centroid Error');
legend('Location', 'best');
hold off;

saveas(gcf, 'fig_moving_target_convergence.png');

%% --- Print Results ---
fprintf('\n=== Final State ===\n');
for i = 1:n
    fprintf('Agent %d: dist = %.4f (error = %.4f)\n', ...
            i, dist_hist(end,i), abs(dist_hist(end,i) - r));
end
fprintf('Centroid error: %.6f\n', centroid_err(end));

if all(abs(dist_hist(end,:) - r) < 0.1) && centroid_err(end) < 0.1
    fprintf('\n✓ MOVING TARGET TRACKING VERIFIED\n');
else
    fprintf('\n✗ TRACKING NOT VERIFIED — check parameters\n');
end
