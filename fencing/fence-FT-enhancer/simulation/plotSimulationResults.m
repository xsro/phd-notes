function plotSimulationResults(tBase, base, tEnh, enh, params, metrics, figDir)
setPlotDefaults();

plotCentroidNorm(tBase, base, tEnh, enh, params, metrics, figDir);
plotCentroidPhase(base, enh, figDir);
plotEstimatorDisagreement(tEnh, enh, params, figDir);
plotAgentTargetTrajectories(base, params, figDir, 'agent_target_trajectories_original');
plotAgentTargetTrajectories(enh, params, figDir, 'agent_target_trajectories_enhanced');
plotPairDistanceHistory(tBase, base, tEnh, enh, params, figDir);
end

function plotCentroidNorm(tBase, base, tEnh, enh, params, metrics, figDir)
fig = figure('Color', 'w', 'Position', [100, 100, 940, 420]);
ax = axes('Parent', fig);
prepareAxes(ax);
plot(ax, tBase, base.centroidNorm, '-', 'Color', [0.16, 0.36, 0.62], ...
    'LineWidth', 2.2, 'DisplayName', 'Original SMC-iAPF');
plot(ax, tEnh, enh.centroidNorm, '-', 'Color', [0.78, 0.24, 0.18], ...
    'LineWidth', 2.2, 'DisplayName', 'With centroid enhancer');
yline(ax, params.threshold, 'k--', 'LineWidth', 1.4, ...
    'DisplayName', 'Threshold');
drawSwitchingTimes(ax, params);
if ~isnan(metrics.enhSettle)
    xline(ax, metrics.enhSettle, '--', 'Color', [0.78, 0.24, 0.18], ...
        'LineWidth', 1.2, 'HandleVisibility', 'off');
end
xlabel(ax, 'Time (s)');
ylabel(ax, '$\|c(t)\|$', 'Interpreter', 'latex');
ylim(ax, [0, 1.05 * max([base.centroidNorm; enh.centroidNorm])]);
legend(ax, 'Location', 'northeast', 'Interpreter', 'latex');
exportFigure(fig, figDir, 'centroid_norm_comparison');
end

function plotCentroidPhase(base, enh, figDir)
fig = figure('Color', 'w', 'Position', [120, 120, 760, 620]);
ax = axes('Parent', fig);
prepareAxes(ax);
plot(ax, base.centroid(:, 1), base.centroid(:, 2), '-', ...
    'Color', [0.16, 0.36, 0.62], 'LineWidth', 2.0, 'DisplayName', 'Original');
plot(ax, enh.centroid(:, 1), enh.centroid(:, 2), '-', ...
    'Color', [0.78, 0.24, 0.18], 'LineWidth', 2.0, 'DisplayName', 'Enhanced');
scatter(ax, base.centroid(1, 1), base.centroid(1, 2), 65, 'k', 'filled', ...
    'DisplayName', 'Initial centroid');
scatter(ax, 0, 0, 90, 'p', 'MarkerFaceColor', [0.10, 0.55, 0.22], ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Target');
xlabel(ax, '$c_x$', 'Interpreter', 'latex');
ylabel(ax, '$c_y$', 'Interpreter', 'latex');
axis(ax, 'equal');
legend(ax, 'Location', 'best', 'Interpreter', 'latex');
exportFigure(fig, figDir, 'centroid_phase_comparison');
end

function plotEstimatorDisagreement(tEnh, enh, params, figDir)
fig = figure('Color', 'w', 'Position', [140, 140, 940, 420]);
ax = axes('Parent', fig);
prepareAxes(ax);
plot(ax, tEnh, enh.centroidEstimateSpread, '-', 'Color', [0.40, 0.32, 0.58], ...
    'LineWidth', 2.0, 'DisplayName', '$\max_{i,j}\|z_i^p-z_j^p\|$');
plot(ax, tEnh, enh.velocityEstimateSpread, '-', 'Color', [0.18, 0.55, 0.42], ...
    'LineWidth', 2.0, 'DisplayName', '$\max_{i,j}\|z_i^v-z_j^v\|$');
drawSwitchingTimes(ax, params);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Estimator disagreement');
legend(ax, 'Location', 'northeast', 'Interpreter', 'latex');
exportFigure(fig, figDir, 'estimator_disagreement');
end

function plotAgentTargetTrajectories(series, params, figDir, fileName)
fig = figure('Color', 'w', 'Position', [160, 160, 780, 660]);
ax = axes('Parent', fig);
prepareAxes(ax);
colors = lines(params.N);

for i = 1:params.N
    pi = squeeze(series.positions(:, :, i));
    plot(ax, pi(:, 1), pi(:, 2), '-', 'Color', colors(i, :), ...
        'LineWidth', 1.8, 'DisplayName', sprintf('Agent %d', i));
    scatter(ax, pi(1, 1), pi(1, 2), 42, colors(i, :), 'o', ...
        'filled', 'HandleVisibility', 'off');
    scatter(ax, pi(end, 1), pi(end, 2), 55, colors(i, :), '^', ...
        'filled', 'HandleVisibility', 'off');
end

plot(ax, series.centroid(:, 1), series.centroid(:, 2), 'k-', ...
    'LineWidth', 2.4, 'DisplayName', 'Centroid');
plot(ax, series.target(:, 1), series.target(:, 2), 'k--', ...
    'LineWidth', 1.5, 'DisplayName', 'Target trajectory');
scatter(ax, series.target(1, 1), series.target(1, 2), 110, 'p', ...
    'MarkerFaceColor', [0.10, 0.55, 0.22], 'MarkerEdgeColor', 'k', ...
    'DisplayName', 'Target');

finalP = squeeze(series.positions(end, :, :));
if params.N >= 3
    hullIdx = convhull(finalP(1, :), finalP(2, :));
    plot(ax, finalP(1, hullIdx), finalP(2, hullIdx), ':', ...
        'Color', [0.15, 0.15, 0.15], 'LineWidth', 1.4, ...
        'DisplayName', 'Final convex hull');
end

xlabel(ax, '$p_x$', 'Interpreter', 'latex');
ylabel(ax, '$p_y$', 'Interpreter', 'latex');
axis(ax, 'equal');
legend(ax, 'Location', 'bestoutside', 'Interpreter', 'latex');
exportFigure(fig, figDir, fileName);
end

function plotPairDistanceHistory(tBase, base, tEnh, enh, params, figDir)
fig = figure('Color', 'w', 'Position', [180, 180, 980, 650]);
layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile(layout, 1);
prepareAxes(ax);
plot(ax, tBase, base.minPairDistance, '-', 'Color', [0.16, 0.36, 0.62], ...
    'LineWidth', 2.0, 'DisplayName', 'Original minimum');
plot(ax, tEnh, enh.minPairDistance, '-', 'Color', [0.78, 0.24, 0.18], ...
    'LineWidth', 2.0, 'DisplayName', 'Enhanced minimum');
yline(ax, params.dSafe, 'k--', 'LineWidth', 1.3, 'DisplayName', '$d$');
drawSwitchingTimes(ax, params);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Minimum distance');
legend(ax, 'Location', 'best', 'Interpreter', 'latex');

ax = nexttile(layout, 2);
prepareAxes(ax);
pairColors = lines(size(enh.pairDistances, 2));
for ell = 1:size(enh.pairDistances, 2)
    label = sprintf('%d-%d', enh.pairI(ell), enh.pairJ(ell));
    plot(ax, tEnh, enh.pairDistances(:, ell), '-', ...
        'Color', pairColors(ell, :), 'LineWidth', 1.2, ...
        'DisplayName', label);
end
yline(ax, params.dSafe, 'k--', 'LineWidth', 1.3, 'DisplayName', '$d$');
yline(ax, params.mu, 'k:', 'LineWidth', 1.3, 'DisplayName', '$\mu$');
drawSwitchingTimes(ax, params);
xlabel(ax, 'Time (s)');
ylabel(ax, 'Pairwise distance');
legend(ax, 'Location', 'eastoutside', 'Interpreter', 'latex');

exportFigure(fig, figDir, 'pair_distance_history');
end

function drawSwitchingTimes(ax, params)
xline(ax, params.Ts, ':', 'Color', [0.35, 0.35, 0.35], ...
    'LineWidth', 1.2, 'DisplayName', '$T_s$');
xline(ax, params.Te, ':', 'Color', [0.1, 0.1, 0.1], ...
    'LineWidth', 1.5, 'DisplayName', '$T_e$');
end

function prepareAxes(ax)
ax.Toolbar.Visible = 'off';
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
set(ax, 'FontSize', 14, 'LineWidth', 1.0, 'GridAlpha', 0.18);
end

function setPlotDefaults()
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultLegendFontName', 'Times New Roman');
end

function exportFigure(fig, figDir, name)
exportgraphics(fig, fullfile(figDir, [name, '.pdf']), 'ContentType', 'vector');
exportgraphics(fig, fullfile(figDir, [name, '.png']), 'Resolution', 200);
end
