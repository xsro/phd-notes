clear; clc; close all;

baseDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(baseDir, 'data');
figDir = fullfile(baseDir, 'figures');
if ~exist(dataDir, 'dir'); mkdir(dataDir); end
if ~exist(figDir, 'dir'); mkdir(figDir); end

params.N = 5;
params.dim = 2;
params.tSpan = [0, 20];
params.tEval = linspace(params.tSpan(1), params.tSpan(2), 2001);
params.tEnhance = 2.0;       % Known upper bound after observer/average consensus.
params.threshold = 1e-2;

% Nominal centroid dynamics induced by the original SMC-iAPF layer. This is
% the reduced centroid channel used only for comparing convergence rates.
params.kpNom = 0.45;
params.kvNom = 1.15;

% Fixed-time center enhancer parameters.
params.lambda1 = 1.30;
params.lambda2 = 0.38;
params.gamma = 0.62;
params.delta = 1.45;
params.ell1 = 2.25;
params.ell2 = 0.85;
params.rho = 0.55;
params.eta = 1.35;
params.kw = 2.50;
params.derivativeFloor = 1e-5;
params.signBoundary = 2e-3;

% A small pentagonal formation around a centroid offset from the target.
formation = [ 1.10,  0.35, -0.80, -1.05,  0.40;
              0.10,  0.95,  0.70, -0.55, -1.20];
c0 = [3.2; -2.4];
nu0 = [-0.35; 0.20];
p0 = formation + c0;
v0 = repmat(nu0, 1, params.N);

centroid0 = mean(p0, 2);
centroidVelocity0 = mean(v0, 2);
y0 = [centroid0; centroidVelocity0];

odeOpt = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.02);

fprintf('Running centroid comparison simulation...\n');
[tBase, yBase] = ode45(@(t, y) centroidDynamics(t, y, params, false), ...
    params.tEval, y0, odeOpt);
[tEnh, yEnh] = ode45(@(t, y) centroidDynamics(t, y, params, true), ...
    params.tEval, y0, odeOpt);

series.base.c = yBase(:, 1:2);
series.base.nu = yBase(:, 3:4);
series.base.err = vecnorm(series.base.c, 2, 2);
series.base.vel = vecnorm(series.base.nu, 2, 2);
series.enh.c = yEnh(:, 1:2);
series.enh.nu = yEnh(:, 3:4);
series.enh.err = vecnorm(series.enh.c, 2, 2);
series.enh.vel = vecnorm(series.enh.nu, 2, 2);

metrics.baseSettle = settlingTime(tBase, series.base.err, params.threshold);
metrics.enhSettle = settlingTime(tEnh, series.enh.err, params.threshold);
metrics.baseFinal = series.base.err(end);
metrics.enhFinal = series.enh.err(end);
metrics.threshold = params.threshold;

fprintf('Centroid threshold: %.1e\n', params.threshold);
fprintf('Baseline settling time: %s s\n', formatTime(metrics.baseSettle));
fprintf('Enhanced settling time: %s s\n', formatTime(metrics.enhSettle));
fprintf('Final centroid norm, baseline: %.4e\n', metrics.baseFinal);
fprintf('Final centroid norm, enhanced: %.4e\n', metrics.enhFinal);

save(fullfile(dataDir, 'centroid_comparison.mat'), ...
    'params', 'tBase', 'yBase', 'tEnh', 'yEnh', 'series', 'metrics');

plotComparison(tBase, series.base, tEnh, series.enh, params, metrics, figDir);
fprintf('Saved data to %s\n', dataDir);
fprintf('Saved figures to %s\n', figDir);

function dy = centroidDynamics(t, y, params, useEnhancer)
c = y(1:2);
nu = y(3:4);
nominalAcc = -params.kpNom * c - params.kvNom * nu;

enhancerAcc = zeros(2, 1);
if useEnhancer && t >= params.tEnhance
    enhancerAcc = fixedTimeEnhancer(c, nu, nominalAcc, params);
end

dy = [nu; nominalAcc + enhancerAcc];
end

function a = fixedTimeEnhancer(c, nu, nominalAcc, params)
phiGamma = signedPower(c, params.gamma);
phiDelta = signedPower(c, params.delta);
sigma = nu + params.lambda1 * phiGamma + params.lambda2 * phiDelta;

phiGammaDot = signedPowerDerivative(c, nu, params.gamma, params.derivativeFloor);
phiDeltaDot = signedPowerDerivative(c, nu, params.delta, params.derivativeFloor);

% The first term cancels the known nominal centroid dynamics in this reduced
% simulation. In the document proof, the same term can be viewed as part of
% the bounded equivalent disturbance handled by the robust sign component.
a = -nominalAcc ...
    - params.lambda1 * phiGammaDot ...
    - params.lambda2 * phiDeltaDot ...
    - params.ell1 * signedPower(sigma, params.rho) ...
    - params.ell2 * signedPower(sigma, params.eta) ...
    - params.kw * smoothSign(sigma, params.signBoundary);
end

function y = signedPower(x, q)
y = abs(x).^q .* signWithZero(x);
end

function y = signedPowerDerivative(x, xdot, q, floorValue)
scale = max(abs(x), floorValue).^(q - 1);
y = q * scale .* xdot;
end

function y = signWithZero(x)
y = sign(x);
y(abs(x) < eps) = 0;
end

function y = smoothSign(x, boundary)
y = x ./ (abs(x) + boundary);
end

function ts = settlingTime(t, err, threshold)
idx = find(err <= threshold, 1, 'first');
ts = NaN;
if isempty(idx)
    return;
end
for k = idx:numel(err)
    if all(err(k:end) <= threshold)
        ts = t(k);
        return;
    end
end
end

function text = formatTime(value)
if isnan(value)
    text = 'not reached';
else
    text = sprintf('%.3f', value);
end
end

function plotComparison(tBase, base, tEnh, enh, params, metrics, figDir)
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultLegendFontName', 'Times New Roman');

fig = figure('Color', 'w', 'Position', [100, 100, 940, 420]);
ax = axes('Parent', fig);
ax.Toolbar.Visible = 'off';
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot(ax, tBase, base.err, '-', 'Color', [0.16, 0.36, 0.62], ...
    'LineWidth', 2.2, 'DisplayName', 'Original centroid channel');
plot(ax, tEnh, enh.err, '-', 'Color', [0.78, 0.24, 0.18], ...
    'LineWidth', 2.2, 'DisplayName', 'With fixed-time enhancer');
yline(ax, params.threshold, 'k--', 'LineWidth', 1.4, ...
    'DisplayName', 'Threshold');
xline(ax, params.tEnhance, ':', 'Color', [0.2, 0.2, 0.2], ...
    'LineWidth', 1.4, 'DisplayName', 'Enhancer on');
if ~isnan(metrics.enhSettle)
    xline(ax, metrics.enhSettle, '--', 'Color', [0.78, 0.24, 0.18], ...
        'LineWidth', 1.2, 'HandleVisibility', 'off');
end
xlabel(ax, 'Time (s)');
ylabel(ax, '$\|c(t)\|$', 'Interpreter', 'latex');
set(ax, 'FontSize', 14, 'LineWidth', 1.0, 'GridAlpha', 0.18);
ylim(ax, [0, 1.05 * max([base.err; enh.err])]);
legend(ax, 'Location', 'northeast', 'Interpreter', 'latex');
exportgraphics(fig, fullfile(figDir, 'centroid_norm_comparison.pdf'), ...
    'ContentType', 'vector');
exportgraphics(fig, fullfile(figDir, 'centroid_norm_comparison.png'), ...
    'Resolution', 200);

fig = figure('Color', 'w', 'Position', [120, 120, 760, 620]);
ax = axes('Parent', fig);
ax.Toolbar.Visible = 'off';
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot(ax, base.c(:, 1), base.c(:, 2), '-', 'Color', [0.16, 0.36, 0.62], ...
    'LineWidth', 2.0, 'DisplayName', 'Original');
plot(ax, enh.c(:, 1), enh.c(:, 2), '-', 'Color', [0.78, 0.24, 0.18], ...
    'LineWidth', 2.0, 'DisplayName', 'Enhanced');
scatter(ax, base.c(1, 1), base.c(1, 2), 65, 'k', 'filled', ...
    'DisplayName', 'Initial centroid');
scatter(ax, 0, 0, 90, 'p', 'MarkerFaceColor', [0.10, 0.55, 0.22], ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Target');
xlabel(ax, '$c_x$', 'Interpreter', 'latex');
ylabel(ax, '$c_y$', 'Interpreter', 'latex');
axis(ax, 'equal');
set(ax, 'FontSize', 14, 'LineWidth', 1.0, 'GridAlpha', 0.18);
legend(ax, 'Location', 'best', 'Interpreter', 'latex');
exportgraphics(fig, fullfile(figDir, 'centroid_phase_comparison.pdf'), ...
    'ContentType', 'vector');
exportgraphics(fig, fullfile(figDir, 'centroid_phase_comparison.png'), ...
    'Resolution', 200);
end
