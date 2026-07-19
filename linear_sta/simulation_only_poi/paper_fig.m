close all;
warning('off', 'MATLAB:print:ContentTypeImageSuggested');

set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultLegendFontName', 'Times New Roman');

baseDir = fileparts(mfilename('fullpath'));
dataFile = fullfile(baseDir, 'data', 'cw_only_poi_data.mat');
metricsFile = fullfile(baseDir, 'data', 'cw_only_poi_metrics.mat');
figDir = fullfile(baseDir, 'figures');
if ~exist(figDir, 'dir'); mkdir(figDir); end

simData = load(dataFile, 't', 'params');
metricData = load(metricsFile, 'series');
t = simData.t(:);
params = simData.params;
series = metricData.series;
N = params.N;

agentColors = [0.20, 0.48, 0.72;
               0.86, 0.55, 0.22;
               0.27, 0.67, 0.48;
               0.66, 0.47, 0.79;
               0.52, 0.58, 0.62];
pairColors = [0.85, 0.33, 0.10;
              0.93, 0.69, 0.13;
              0.49, 0.18, 0.56;
              0.47, 0.67, 0.19;
              0.30, 0.75, 0.93;
              0.64, 0.08, 0.18;
              0.00, 0.45, 0.74;
              0.80, 0.22, 0.00;
              0.95, 0.62, 0.05;
              0.50, 0.20, 0.60];
outputFiles = strings(7, 1);

%% Figure 1: trajectories and convex hull snapshots
trajView = [-68.4724, 16.0744];
trajAspect = [1, 1, 4];
trajSnapTimes = [3, 10, 20];
trajPointTimes = [0, 3, 10, 20];

fig = figure('Color', 'w', 'Position', [100, 50, 900, 900]);
ax = axes('Parent', fig, 'Position', [0.10, 0.08, 0.70, 0.86]);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot3(ax, series.p0(1, :), series.p0(2, :), series.p0(3, :), ...
    '-', 'Color', [0.15, 0.15, 0.15], 'LineWidth', 2.2, ...
    'DisplayName', 'Target trajectory');
for i = 1:N
    plot3(ax, squeeze(series.p(1, i, :)), squeeze(series.p(2, i, :)), ...
        squeeze(series.p(3, i, :)), 'Color', agentColors(i, :), ...
        'LineWidth', 2.0, 'DisplayName', sprintf('Satellite %d', i));
end

hullColors = [0.88, 0.95, 1.00;
              0.62, 0.82, 0.95;
              0.34, 0.65, 0.88];
for q = 1:numel(trajSnapTimes)
    [~, idx] = min(abs(t - trajSnapTimes(q)));
    drawHull(ax, series.p(:, :, idx).', hullColors(q, :), ...
        0.20 + 0.04 * q, 0.8 + 0.3 * q);
    scatter3(ax, series.p0(1, idx), series.p0(2, idx), series.p0(3, idx), ...
        75, [0.95, 0.08, 0.08], '*', 'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
end

for q = 1:numel(trajPointTimes)
    [~, idx] = min(abs(t - trajPointTimes(q)));
    points = series.p(:, :, idx).';
    for i = 1:N
        scatter3(ax, points(i, 1), points(i, 2), points(i, 3), ...
            72, agentColors(i, :), 'filled', ...
            'MarkerEdgeColor', [0.95, 0.95, 0.95], 'LineWidth', 0.6, ...
            'HandleVisibility', 'off');
    end
end

allPoints = series.p0.';
for i = 1:N
    allPoints = [allPoints; squeeze(series.p(:, i, :)).']; %#ok<AGROW>
end
xyzMin = min(allPoints, [], 1);
xyzMax = max(allPoints, [], 1);
xyzPad = 0.06 * max(xyzMax - xyzMin, [1, 1, 1]);
xlim(ax, [xyzMin(1) - xyzPad(1), xyzMax(1) + xyzPad(1)]);
ylim(ax, [xyzMin(2) - xyzPad(2), xyzMax(2) + xyzPad(2)]);
zlim(ax, [xyzMin(3) - xyzPad(3), xyzMax(3) + xyzPad(3)]);
xlabel(ax, 'X(km)'); ylabel(ax, 'Y(km)'); zlabel(ax, 'Z(km)');
view(ax, trajView); daspect(ax, trajAspect); axis(ax, 'vis3d');
lgd = legend(ax, 'Location', 'northeastoutside', 'FontSize', 11, 'Box', 'on');
lgd.ItemTokenSize = [14, 8];
set(ax, 'FontSize', 12, 'LineWidth', 1.0, 'GridAlpha', 0.18);
outputFiles(1) = fullfile(figDir, 'cw_only_poi_tra.pdf');
exportgraphics(fig, outputFiles(1), 'ContentType', 'vector');

%% Figure 2: position estimation errors
outputFiles(2) = plotAgentSignals(t, series.estErrP, agentColors, ...
    '$\|\hat p_{i0}-p_{i0}\|$', fullfile(figDir, 'cw_only_poi_est_p_error.pdf'));

%% Figure 3: velocity estimation errors
outputFiles(3) = plotAgentSignals(t, series.estErrV, agentColors, ...
    '$\|\hat v_{i0}-v_{i0}\|$', fullfile(figDir, 'cw_only_poi_est_v_error.pdf'));

%% Figure 4: centroid and all pair distances
distXLim = [0, 20];
distYLim = [0.0, 6.5];
fig = figure('Color', 'w', 'Position', [100, 100, 1000, 420]);
ax = axes('Parent', fig);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot(ax, t, series.centroidError, '-', 'Color', [0.0, 0.0, 1.0], ...
    'LineWidth', 2.5, 'DisplayName', '$\Vert \bar{p}-p_0 \Vert$');
pairIndex = 1;
for i = 1:N-1
    for j = i+1:N
        plot(ax, t, series.pairDistance(:, pairIndex), ...
            'Color', pairColors(pairIndex, :), 'LineWidth', 1.9, ...
            'DisplayName', sprintf('$\\Vert p_{%d%d} \\Vert$', i, j));
        pairIndex = pairIndex + 1;
    end
end
yline(ax, params.dSafe, 'r--', 'LineWidth', 2.5, 'DisplayName', '$d$');
yline(ax, params.muRange, 'k--', 'LineWidth', 2.5, 'DisplayName', '$\mu$');
yline(ax, params.ld, ':', 'LineWidth', 1.8, 'DisplayName', '$l_d$');
yline(ax, params.lmu, ':', 'LineWidth', 1.8, 'DisplayName', '$l_\mu$');
xlabel(ax, 'Time (s)'); ylabel(ax, '');
xlim(ax, distXLim); ylim(ax, distYLim);
lgd = legend(ax, 'Location', 'northeast', 'NumColumns', 4, ...
    'Interpreter', 'latex', 'FontSize', 12, 'Box', 'on');
lgd.ItemTokenSize = [14, 8];
set(ax, 'FontSize', 16, 'LineWidth', 1.0, 'GridAlpha', 0.20);
outputFiles(4) = fullfile(figDir, 'cw_only_poi_dist_agents.pdf');
exportgraphics(fig, outputFiles(4), 'ContentType', 'vector');

%% Figure 5: estimated sliding variables
outputFiles(5) = plotAgentSignals(t, series.sHat, agentColors, ...
    '$\|\hat s_{i0}\|$', fullfile(figDir, 'cw_only_poi_hat_si0.pdf'));

%% Figure 6: control input norms
outputFiles(6) = plotAgentSignals(t, series.uNorm, agentColors, ...
    '$\|u_i\|$', fullfile(figDir, 'cw_only_poi_control.pdf'));

%% Figure 7: observer injections and compensation
fig = figure('Color', 'w', 'Position', [100, 80, 950, 720]);
layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact');
signalSet = {series.nuPNorm, series.nuVNorm, series.gammaNorm};
yLabels = {'$\|\nu_{p,i}\|$', '$\|\nu_{v,i}\|$', '$\|\Gamma_i\|$'};
for q = 1:3
    ax = nexttile(layout);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    for i = 1:N
        plot(ax, t, signalSet{q}(:, i), 'Color', agentColors(i, :), ...
            'LineWidth', 1.6, 'DisplayName', sprintf('Satellite %d', i));
    end
    ylabel(ax, yLabels{q}, 'Interpreter', 'latex');
    xlim(ax, [t(1), t(end)]);
    set(ax, 'FontSize', 13, 'LineWidth', 1.0, 'GridAlpha', 0.20);
end
xlabel(nexttile(layout, 3), 'Time (s)');
legend(nexttile(layout, 1), 'Location', 'eastoutside');
outputFiles(7) = fullfile(figDir, 'cw_only_poi_injection.pdf');
exportgraphics(fig, outputFiles(7), 'ContentType', 'vector');

for i = 1:numel(outputFiles)
    info = dir(outputFiles(i));
    assert(~isempty(info) && info.bytes > 0, 'Figure export failed: %s', outputFiles(i));
end
disp(['Saved MATLAB figures to: ', figDir]);

function outputFile = plotAgentSignals(t, values, colors, yText, outputFile)
fig = figure('Color', 'w', 'Position', [100, 100, 900, 360]);
ax = axes('Parent', fig);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
for i = 1:size(values, 2)
    plot(ax, t, values(:, i), 'Color', colors(i, :), 'LineWidth', 2.0, ...
        'DisplayName', sprintf('Satellite %d', i));
end
xlabel(ax, 'Time (s)');
ylabel(ax, yText, 'Interpreter', 'latex');
xlim(ax, [t(1), t(end)]);
legend(ax, 'Location', 'northeast');
set(ax, 'FontSize', 16, 'LineWidth', 1.0, 'GridAlpha', 0.20);
exportgraphics(fig, outputFile, 'ContentType', 'vector');
end

function drawHull(ax, points, faceColor, faceAlpha, edgeLineWidth)
if size(unique(points, 'rows'), 1) < 4
    return;
end
try
    faces = convhulln(points);
    patch(ax, 'Vertices', points, 'Faces', faces, ...
        'FaceColor', faceColor, 'FaceAlpha', faceAlpha, ...
        'EdgeColor', 0.75 * faceColor, 'LineWidth', edgeLineWidth, ...
        'HandleVisibility', 'off');
catch
end
end
