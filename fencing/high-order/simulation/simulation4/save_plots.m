%SAVE_PLOTS  Load saved simulation data and export all figures.
%   Run simulate.m first if the result.mat files do not exist.

clc;
close all;
set(0, 'DefaultFigureVisible', 'off');

base_dir = fileparts(mfilename('fullpath'));
out_root = fullfile(base_dir, 'out');
plot_case(fullfile(out_root, 'safe_case'));
plot_case(fullfile(out_root, 'stress_case'));

% Regenerate the legacy root figures when out/result.mat is present.
legacy_file = fullfile(out_root, 'result.mat');
if exist(legacy_file, 'file')
    plot_result_file(legacy_file, out_root);
end

%--------------------------------------------------------------------------
function plot_case(outdir)
plot_result_file(fullfile(outdir, 'result.mat'), outdir);
end

%--------------------------------------------------------------------------
function plot_result_file(result_file, outdir)
if ~exist(result_file, 'file')
    error('Missing result file: %s. Run simulate.m first.', result_file);
end
data = load(result_file, 'results', 'params');
results = data.results;
params = data.params;
plot_trajectories(results, params, outdir);
plot_weighted_center(results, outdir);
plot_pairwise(results, params, outdir);
plot_residuals(results, params, outdir);
plot_energy(results, outdir);
plot_hull(results, outdir);

[min_dist, idx] = min(results.pairwise_distances, [], 'all', 'linear');
[row, col] = ind2sub(size(results.pairwise_distances), idx);
fprintf('\nSimulation 4 %s plots complete.\n', params.case_name);
fprintf('Minimum pairwise distance: %.6f at %s, t = %.3f\n', ...
    min_dist, results.pairwise_labels{row}, results.t(col));
end

%--------------------------------------------------------------------------
function plot_trajectories(results, params, outdir)
colors = lines(params.N);
fig = figure('Color', 'w', 'Position', [100, 100, 900, 660]);
hold on;
traj_handles = gobjects(params.N, 1);
for i = 1:params.N
    P = squeeze(results.positions(:, i, :));
    traj_handles(i) = plot(P(1, :), P(2, :), 'LineWidth', 1.8, 'Color', colors(i, :));
    plot(P(1, 1), P(2, 1), 'o', 'Color', colors(i, :), ...
        'MarkerFaceColor', colors(i, :), 'MarkerSize', 5, 'HandleVisibility', 'off');
    plot(P(1, end), P(2, end), 's', 'Color', colors(i, :), ...
        'MarkerFaceColor', colors(i, :), 'MarkerSize', 5, 'HandleVisibility', 'off');
end
target_handle = plot(results.target_pos(1, :), results.target_pos(2, :), ...
    'k-', 'LineWidth', 2.1);
plot(results.target_pos(1, 1), results.target_pos(2, 1), 'ks', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 6, 'HandleVisibility', 'off');
plot(results.target_pos(1, end), results.target_pos(2, end), 'kd', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 6, 'HandleVisibility', 'off');
axis equal;
grid on;
xlabel('x');
ylabel('y');
title('Result 4 command-filtered backstepping trajectories');
if isfield(params, 'case_name')
    title(sprintf('Result 4 command-filtered backstepping trajectories (%s)', ...
        strrep(params.case_name, '_', '\_')));
end
legend([traj_handles; target_handle], ...
    [arrayfun(@(i) sprintf('agent %d, m=%d', i, params.orders(i)), ...
    1:params.N, 'UniformOutput', false), {'target'}], 'Location', 'bestoutside');
exportgraphics(fig, fullfile(outdir, 'trajectories.pdf'), 'ContentType', 'vector');
end

%--------------------------------------------------------------------------
function plot_weighted_center(results, outdir)
fig = figure('Color', 'w', 'Position', [120, 120, 880, 500]);
plot(results.t, results.weighted_distance, 'k', 'LineWidth', 2.0);
grid on;
xlabel('time (s)');
ylabel('||kappa-weighted center - target||');
title('Weighted-center tracking error');
if isfield(results, 'case_name')
    title(sprintf('Weighted-center tracking error (%s)', strrep(results.case_name, '_', '\_')));
end
exportgraphics(fig, fullfile(outdir, 'weighted_center_distance.pdf'), 'ContentType', 'vector');
end

%--------------------------------------------------------------------------
function plot_pairwise(results, params, outdir)
fig = figure('Color', 'w', 'Position', [120, 120, 920, 540]);
hold on;
hPair = gobjects(size(results.pairwise_distances, 1), 1);
for p = 1:size(results.pairwise_distances, 1)
    hPair(p) = plot(results.t, results.pairwise_distances(p, :), 'LineWidth', 1.4);
end
hSafety = yline(params.safe_distance, 'k--', 'LineWidth', 1.4);
grid on;
xlabel('time (s)');
ylabel('distance');
title('Pairwise inter-agent distances');
if isfield(params, 'case_name')
    title(sprintf('Pairwise inter-agent distances (%s)', strrep(params.case_name, '_', '\_')));
    if strcmp(params.case_name, 'stress_case')
        ylim([0, 10]);
    end
end
legend([hPair; hSafety], [results.pairwise_labels, {'safety distance'}], ...
    'Location', 'bestoutside');
exportgraphics(fig, fullfile(outdir, 'pairwise_distances.pdf'), 'ContentType', 'vector');
end

%--------------------------------------------------------------------------
function plot_residuals(results, params, outdir)
colors = lines(params.N);
fig = figure('Color', 'w', 'Position', [120, 120, 900, 540]);
hold on;
hResidual = gobjects(params.N, 1);
for i = 1:params.N
    hResidual(i) = plot(results.t, results.residual_norms(i, :), ...
        'LineWidth', 1.6, 'Color', colors(i, :));
end
grid on;
xlabel('time (s)');
ylabel('||r_i||');
title('Perturbation in the realized first-order APF motion');
if isfield(params, 'case_name')
    title(sprintf('Realized APF perturbation (%s)', strrep(params.case_name, '_', '\_')));
end
legend(hResidual, arrayfun(@(i) sprintf('agent %d', i), ...
    1:params.N, 'UniformOutput', false), 'Location', 'bestoutside');
exportgraphics(fig, fullfile(outdir, 'residual_norms.pdf'), 'ContentType', 'vector');
end

%--------------------------------------------------------------------------
function plot_energy(results, outdir)
fig = figure('Color', 'w', 'Position', [120, 120, 880, 500]);
plot(results.t, results.ideal_energy, 'LineWidth', 2.0);
grid on;
xlabel('time (s)');
ylabel('W');
title('Ideal APF energy along the filtered-backstepping closed loop');
if isfield(results, 'case_name')
    title(sprintf('Ideal APF energy (%s)', strrep(results.case_name, '_', '\_')));
end
exportgraphics(fig, fullfile(outdir, 'ideal_energy.pdf'), 'ContentType', 'vector');
end

%--------------------------------------------------------------------------
function plot_hull(results, outdir)
fig = figure('Color', 'w', 'Position', [120, 120, 880, 500]);
yyaxis left;
plot(results.t, results.hull_area, 'LineWidth', 1.8);
ylabel('convex-hull area');
yyaxis right;
stairs(results.t, double(results.inside_hull), 'k', 'LineWidth', 1.4);
ylim([-0.05, 1.05]);
ylabel('target inside hull');
grid on;
xlabel('time (s)');
title('Fencing and hull size');
if isfield(results, 'case_name')
    title(sprintf('Fencing and hull size (%s)', strrep(results.case_name, '_', '\_')));
end
exportgraphics(fig, fullfile(outdir, 'hull_status.pdf'), 'ContentType', 'vector');
end
