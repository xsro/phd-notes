%% test_gain_condition.m  —  Sweep kr to verify the gain condition
%  Threshold: kr/ka >= 2*n*r^2/(n-1).
%  We fix ka=1, r=3, n=5, sweep kr below/at/above threshold.

clear; clc; close all;

%% Fixed parameters
dim = 2;
n   = 5;
r   = 3;
ka  = 1;
ko  = 10;
T   = 50;
dt  = 0.005;

% Threshold: 2*n*r^2/(n-1) = 2*5*9/4 = 22.5
threshold = 2*n*r^2/(n-1);
fprintf('Gain condition threshold: kr/ka >= %.1f\n', threshold);

% Test values: below, at, and above threshold
kr_values = [5, 15, 22.5, 50];
n_cases   = length(kr_values);

%% Initial conditions (shared)
P0_init = [0, 0];
angles = (0:n-1)' * (2*pi/n) + pi/10;
P_init = 6 * [cos(angles), sin(angles)];

adj = ones(n) - eye(n);
w   = ones(n);

%% Run all cases
results = struct();
for c = 1:n_cases
    kr = kr_values(c);
    fprintf('\n--- Case %d: kr = %.1f (kr/ka = %.1f %s %.1f) ---\n', ...
            c, kr, kr, char(8805*(kr>=threshold) + 8804*(kr<threshold)), threshold);

    [t, P_hist, ~, dist_hist, centroid_err] = ...
        spherical_fencing_sim(dim, n, r, ka, kr, ko, T, dt, ...
                              P_init, P0_init, [], adj, w);

    results(c).kr = kr;
    results(c).P_hist = P_hist;
    results(c).dist_hist = dist_hist;
    results(c).centroid_err = centroid_err;
    results(c).P_final = squeeze(P_hist(end,:,:));

    fprintf('  Final dist errors: ');
    fprintf('%.3f ', abs(dist_hist(end,:) - r));
    fprintf('\n  Centroid error: %.4f\n', centroid_err(end));
end

%% --- Figure 1: Final Configurations (side by side) ---
figure('Name', 'Gain Condition: Final Configurations', ...
       'Position', [50, 100, 1200, 350]);

theta_circle = linspace(0, 2*pi, 200);
for c = 1:n_cases
    subplot(1, n_cases, c);
    hold on; axis equal; grid on;

    % Target sphere
    plot(r*cos(theta_circle), r*sin(theta_circle), 'k--', 'LineWidth', 1.2);
    plot(0, 0, 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y');

    % Final agent positions
    P_final = results(c).P_final;
    colors = lines(n);
    for i = 1:n
        plot(P_final(i,1), P_final(i,2), 'o', 'Color', colors(i,:), ...
             'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
    end

    kr = results(c).kr;
    ratio_str = sprintf('k_r/k_a = %.1f', kr);
    if kr >= threshold
        title({ratio_str, '(≥ threshold)', sprintf('centroid err: %.3f', ...
               results(c).centroid_err(end))}, 'Color', [0 0.5 0]);
    else
        title({ratio_str, '(< threshold!)', sprintf('centroid err: %.3f', ...
               results(c).centroid_err(end))}, 'Color', [0.8 0 0]);
    end
    xlabel('x'); ylabel('y');
    hold off;
end
saveas(gcf, 'fig_gain_condition_final.png');

%% --- Figure 2: Distance Curves Comparison ---
figure('Name', 'Gain Condition: Distance Comparison', ...
       'Position', [100, 150, 900, 500]);
hold on; grid on;

line_styles = {'-', '--', '-.', ':'};
for c = 1:n_cases
    kr = results(c).kr;
    dist_mean = mean(results(c).dist_hist, 2);  % average over agents
    plot(t, dist_mean, line_styles{c}, 'LineWidth', 1.8, ...
         'DisplayName', sprintf('k_r = %.1f (ratio = %.1f)', kr, kr/ka));
end
yline(r, 'k:', 'LineWidth', 2, 'DisplayName', sprintf('r = %.1f', r));
xline(0, 'k--', 'HandleVisibility', 'off');

xlabel('Time'); ylabel('Mean ||p_i - p_0||');
title(sprintf('Effect of k_r on Convergence (k_a = %.1f, threshold = %.1f)', ...
      ka, threshold));
legend('Location', 'best');
hold off;
saveas(gcf, 'fig_gain_condition_distance.png');

%% --- Summary Table ---
fprintf('\n========================================\n');
fprintf('  Gain Condition Verification Summary\n');
fprintf('========================================\n');
fprintf('  kr/ka   | Threshold | Status\n');
fprintf('---------|-----------|--------\n');
for c = 1:n_cases
    kr = results(c).kr;
    ratio = kr / ka;
    final_err = mean(abs(results(c).dist_hist(end,:) - r));
    if ratio >= threshold && final_err < 0.05
        status = '✓ Converged';
    elseif ratio < threshold && final_err > 0.1
        status = '✗ Poor convergence (expected)';
    else
        status = '~ Marginal';
    end
    fprintf('  %5.1f   |  %5.1f    | %s\n', ratio, threshold, status);
end
fprintf('========================================\n');
