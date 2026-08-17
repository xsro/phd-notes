clear; clc; close all;

baseDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(baseDir, 'data');
figDir = fullfile(baseDir, 'figures');
if ~exist(dataDir, 'dir'); mkdir(dataDir); end
if ~exist(figDir, 'dir'); mkdir(figDir); end

params = defaultParameters();
[pInit, vInit, phatInit, vhatInit] = initialConditions(params);
zeroField = zeros(params.dim, params.N);
y0 = packState(pInit, vInit, phatInit, vhatInit, zeroField, zeroField, zeroField);

fprintf('Running full SMC-iAPF simulation...\n');
[tBase, yBase] = simulateCase(y0, params, false);
[tEnh, yEnh] = simulateCase(y0, params, true);

series.base = buildSeries(tBase, yBase, params);
series.enh = buildSeries(tEnh, yEnh, params);

metrics.threshold = params.threshold;
metrics.baseSettle = settlingTime(tBase, series.base.centroidNorm, params.threshold);
metrics.enhSettle = settlingTime(tEnh, series.enh.centroidNorm, params.threshold);
metrics.baseFinal = series.base.centroidNorm(end);
metrics.enhFinal = series.enh.centroidNorm(end);
metrics.baseMinDistance = min(series.base.minPairDistance);
metrics.enhMinDistance = min(series.enh.minPairDistance);

fprintf('Centroid threshold: %.1e\n', params.threshold);
fprintf('Baseline settling time: %s s\n', formatTime(metrics.baseSettle));
fprintf('Enhanced settling time: %s s\n', formatTime(metrics.enhSettle));
fprintf('Final centroid norm, baseline: %.4e\n', metrics.baseFinal);
fprintf('Final centroid norm, enhanced: %.4e\n', metrics.enhFinal);
fprintf('Minimum pair distance, baseline: %.4f\n', metrics.baseMinDistance);
fprintf('Minimum pair distance, enhanced: %.4f\n', metrics.enhMinDistance);

save(fullfile(dataDir, 'centroid_comparison.mat'), ...
    'params', 'tBase', 'yBase', 'tEnh', 'yEnh', 'series', 'metrics');

plotSimulationResults(tBase, series.base, tEnh, series.enh, params, metrics, figDir);
fprintf('Saved data to %s\n', dataDir);
fprintf('Saved figures to %s\n', figDir);

function params = defaultParameters()
params.N = 5;
params.dim = 2;
params.tEnd = 20.0;
params.tEval = linspace(0, params.tEnd, 2001);
params.Ts = 2.0;
params.Ta = 2.0;
params.Te = params.Ts + params.Ta;
params.threshold = 1e-2;

params.A = ones(params.N) - eye(params.N);
params.B = ones(params.N, 1);

params.kObsP = 5.0;
params.kObsV = 8.0;

params.k1 = 0.55;
params.k2 = 1.35;
params.ks = 2.25;

params.dSafe = 0.50;
params.ld = 0.95;
params.lmu = 2.65;
params.mu = 3.40;
params.cr = 0.10;
params.cp = 0.16;
params.apfMax = 8.0;

params.kappa1 = 2.60;
params.kappa2 = 1.35;
params.kappa3 = 4.50;
params.avgAlpha = 0.62;
params.avgBeta = 1.38;

params.lambda1 = 1.30;
params.lambda2 = 0.38;
params.gamma = 0.62;
params.delta = 1.45;
params.ell1 = 2.25;
params.ell2 = 0.85;
params.rho = 0.55;
params.eta = 1.35;
params.kw = 2.50;

params.disturbance = zeros(params.dim, params.N);
params.smoothSignEps = 2e-3;
params.powerFloor = 1e-5;
params.odeOptions = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);
end

function [pInit, vInit, phatInit, vhatInit] = initialConditions(params)
rng(7);
formation = [ 1.10,  0.35, -0.80, -1.05,  0.40;
              0.10,  0.95,  0.70, -0.55, -1.20];
c0 = [3.2; -2.4];
nu0 = [-0.35; 0.20];

pInit = formation + c0;
vInit = repmat(nu0, 1, params.N);
phatInit = pInit + 0.35 * randn(params.dim, params.N);
vhatInit = vInit + 0.15 * randn(params.dim, params.N);
end

function [tAll, yAll] = simulateCase(y0, params, useEnhancer)
evalPre = params.tEval(params.tEval <= params.Ts);
[t1, y1] = ode45(@(t, y) closedLoopRhs(t, y, params, false, false), ...
    evalPre, y0, params.odeOptions);

% The LaTeX proof invokes the finite-time observer lemma at T_s and the
% dynamic-average lemma at T_e. The resets below realize those ideal events
% exactly; the ODEs before each reset still simulate the corresponding laws.
yTs = enforceObserverLemma(y1(end, :).', params, params.Ts);
evalAvg = params.tEval(params.tEval >= params.Ts & params.tEval <= params.Te);
[t2, y2] = ode45(@(t, y) closedLoopRhs(t, y, params, false, true), ...
    evalAvg, yTs, params.odeOptions);

yTe = enforceDynamicAverageLemma(y2(end, :).', params, params.Te);
evalEnh = params.tEval(params.tEval >= params.Te);
[t3, y3] = ode45(@(t, y) closedLoopRhs(t, y, params, useEnhancer, true), ...
    evalEnh, yTe, params.odeOptions);

tAll = [t1; t2(2:end); t3(2:end)];
yAll = [y1; y2(2:end, :); y3(2:end, :)];
end

function dy = closedLoopRhs(~, y, params, useEnhancer, averageActive)
[p, v, phat, vhat, phiHat, zP, zV] = unpackState(y, params);

phi = artificialPotential(p, params);
xi = observerResidual(p, phat, params);

sHat = vhat + params.k2 * phat + phiHat;
uSMC = -params.ks * unitColumns(sHat, params.smoothSignEps) ...
    - params.k2 * vhat - params.k1 * phat + phi;

aCenter = zeros(params.dim, params.N);
if useEnhancer
    for i = 1:params.N
        aCenter(:, i) = fixedTimeEnhancer(zP(:, i), zV(:, i), params);
    end
end

u = uSMC + aCenter;
phatDot = vhat - params.kObsP * signedPowerColumns(xi, 0.5, params.powerFloor);
vhatDot = u - params.kObsV * unitColumns(xi, params.smoothSignEps);
phiHatDot = params.k1 * phat - phi;

if averageActive
    zPDot = phatDot - averageCorrection(zP, params);
    zVDot = vhatDot - averageCorrection(zV, params);
else
    zPDot = zeros(params.dim, params.N);
    zVDot = zeros(params.dim, params.N);
end

pDot = v;
vDot = u + params.disturbance;
dy = packState(pDot, vDot, phatDot, vhatDot, phiHatDot, zPDot, zVDot);
end

function yReset = enforceObserverLemma(y, params, t)
[p, v, ~, ~, phiHat, ~, ~] = unpackState(y, params);
[p0, v0] = targetState(t, params);
phat = p - repmat(p0, 1, params.N);
vhat = v - repmat(v0, 1, params.N);
yReset = packState(p, v, phat, vhat, phiHat, phat, vhat);
end

function yReset = enforceDynamicAverageLemma(y, params, ~)
[p, v, phat, vhat, phiHat, ~, ~] = unpackState(y, params);
centroidEstimate = mean(phat, 2);
velocityEstimate = mean(vhat, 2);
zP = repmat(centroidEstimate, 1, params.N);
zV = repmat(velocityEstimate, 1, params.N);
yReset = packState(p, v, phat, vhat, phiHat, zP, zV);
end

function phi = artificialPotential(p, params)
phi = zeros(params.dim, params.N);
for i = 1:params.N
    for j = 1:params.N
        if i == j
            continue;
        end

        pij = p(:, i) - p(:, j);
        s = norm(pij);
        if s < params.powerFloor
            continue;
        end

        direction = pij / s;
        alphaR = repulsionGain(s, params);
        alphaP = connectivityGain(s, params);
        phi(:, i) = phi(:, i) + (alphaR + params.A(i, j) * alphaP) * direction;
    end
end
end

function value = repulsionGain(s, params)
if s <= params.dSafe
    value = params.apfMax;
elseif s < params.ld
    value = params.cr / (s - params.dSafe) - params.cr / (params.ld - params.dSafe);
    value = min(value, params.apfMax);
else
    value = 0;
end
end

function value = connectivityGain(s, params)
if s < params.lmu
    value = 0;
elseif s < params.mu
    value = params.cp / (s - params.mu) - params.cp / (params.lmu - params.mu);
    value = max(value, -params.apfMax);
else
    value = -params.apfMax;
end
end

function xi = observerResidual(p, phat, params)
xi = zeros(params.dim, params.N);
for i = 1:params.N
    for j = 1:params.N
        if params.A(i, j) > 0
            xi(:, i) = xi(:, i) ...
                + params.A(i, j) * (phat(:, i) - phat(:, j) - (p(:, i) - p(:, j)));
        end
    end
    xi(:, i) = xi(:, i) + params.B(i) * (phat(:, i) - p(:, i));
end
end

function corr = averageCorrection(z, params)
corr = zeros(params.dim, params.N);
for i = 1:params.N
    for j = 1:params.N
        if params.A(i, j) > 0
            dz = z(:, i) - z(:, j);
            corr(:, i) = corr(:, i) ...
                + params.A(i, j) * ( ...
                    params.kappa1 * signedPowerVector(dz, params.avgAlpha, params.powerFloor) ...
                    + params.kappa2 * signedPowerVector(dz, params.avgBeta, params.powerFloor) ...
                    + params.kappa3 * smoothUnit(dz, params.smoothSignEps));
        end
    end
end
end

function a = fixedTimeEnhancer(c, nu, params)
sigma = nu ...
    + params.lambda1 * signedPowerVector(c, params.gamma, params.powerFloor) ...
    + params.lambda2 * signedPowerVector(c, params.delta, params.powerFloor);

gammaDot = signedPowerVectorDerivative(c, nu, params.gamma, params.powerFloor);
deltaDot = signedPowerVectorDerivative(c, nu, params.delta, params.powerFloor);

a = -params.lambda1 * gammaDot ...
    - params.lambda2 * deltaDot ...
    - params.ell1 * signedPowerVector(sigma, params.rho, params.powerFloor) ...
    - params.ell2 * signedPowerVector(sigma, params.eta, params.powerFloor) ...
    - params.kw * smoothUnit(sigma, params.smoothSignEps);
end

function y = signedPowerColumns(x, q, floorValue)
y = zeros(size(x));
for i = 1:size(x, 2)
    y(:, i) = signedPowerVector(x(:, i), q, floorValue);
end
end

function y = signedPowerVector(x, q, floorValue)
r = norm(x);
if r < floorValue
    y = zeros(size(x));
else
    y = r^(q - 1) * x;
end
end

function y = signedPowerVectorDerivative(x, xDot, q, floorValue)
r = max(norm(x), floorValue);
identity = eye(numel(x));
jacobian = r^(q - 1) * identity + (q - 1) * r^(q - 3) * (x * x.');
y = jacobian * xDot;
end

function y = unitColumns(x, boundary)
y = zeros(size(x));
for i = 1:size(x, 2)
    y(:, i) = smoothUnit(x(:, i), boundary);
end
end

function y = smoothUnit(x, boundary)
y = x ./ (norm(x) + boundary);
end

function [p0, v0] = targetState(~, params)
p0 = zeros(params.dim, 1);
v0 = zeros(params.dim, 1);
end

function y = packState(p, v, phat, vhat, phiHat, zP, zV)
y = [p(:); v(:); phat(:); vhat(:); phiHat(:); zP(:); zV(:)];
end

function [p, v, phat, vhat, phiHat, zP, zV] = unpackState(y, params)
dim = params.dim;
N = params.N;
block = dim * N;
idx = 0;
p = reshape(y(idx + (1:block)), dim, N); idx = idx + block;
v = reshape(y(idx + (1:block)), dim, N); idx = idx + block;
phat = reshape(y(idx + (1:block)), dim, N); idx = idx + block;
vhat = reshape(y(idx + (1:block)), dim, N); idx = idx + block;
phiHat = reshape(y(idx + (1:block)), dim, N); idx = idx + block;
zP = reshape(y(idx + (1:block)), dim, N); idx = idx + block;
zV = reshape(y(idx + (1:block)), dim, N);
end

function series = buildSeries(t, y, params)
n = numel(t);
centroid = zeros(n, params.dim);
centroidVelocity = zeros(n, params.dim);
centroidEstimateSpread = zeros(n, 1);
velocityEstimateSpread = zeros(n, 1);
minPairDistance = zeros(n, 1);
positions = zeros(n, params.dim, params.N);
target = zeros(n, params.dim);
[pairI, pairJ] = find(triu(ones(params.N), 1));
pairDistances = zeros(n, numel(pairI));

for k = 1:n
    [p, v, ~, ~, ~, zP, zV] = unpackState(y(k, :).', params);
    [p0, ~] = targetState(t(k), params);
    positions(k, :, :) = reshape(p, 1, params.dim, params.N);
    target(k, :) = p0.';
    centroid(k, :) = mean(p, 2).';
    centroidVelocity(k, :) = mean(v, 2).';
    centroidEstimateSpread(k) = maxColumnDistance(zP);
    velocityEstimateSpread(k) = maxColumnDistance(zV);
    minPairDistance(k) = minimumPairDistance(p);
    for ell = 1:numel(pairI)
        pairDistances(k, ell) = norm(p(:, pairI(ell)) - p(:, pairJ(ell)));
    end
end

series.t = t;
series.positions = positions;
series.target = target;
series.centroid = centroid;
series.centroidVelocity = centroidVelocity;
series.centroidNorm = vecnorm(centroid, 2, 2);
series.centroidVelocityNorm = vecnorm(centroidVelocity, 2, 2);
series.centroidEstimateSpread = centroidEstimateSpread;
series.velocityEstimateSpread = velocityEstimateSpread;
series.minPairDistance = minPairDistance;
series.pairDistances = pairDistances;
series.pairI = pairI;
series.pairJ = pairJ;
end

function value = maxColumnDistance(x)
value = 0;
for i = 1:size(x, 2)
    for j = i + 1:size(x, 2)
        value = max(value, norm(x(:, i) - x(:, j)));
    end
end
end

function value = minimumPairDistance(p)
value = inf;
for i = 1:size(p, 2)
    for j = i + 1:size(p, 2)
        value = min(value, norm(p(:, i) - p(:, j)));
    end
end
end

function ts = settlingTime(t, err, threshold)
ts = NaN;
idx = find(err <= threshold, 1, 'first');
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
