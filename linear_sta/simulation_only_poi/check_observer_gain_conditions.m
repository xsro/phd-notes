clear; clc;

baseDir = fileparts(mfilename('fullpath'));
addpath(baseDir);

paramsSim = loadSimulationParams(baseDir);

% Proof-only parameters. These are not simulation gains in main.m.
proof.gamma = 1.0;
proof.etaP = 1.0;
proof.etaV = 1.0;
proof.c0 = 0.005;              % [] means choose 80% of the admissible upper bound.
proof.deltaSamples = 20001;     % Finite-horizon sampled bound on delta_i.
proof.C0GridSize = 181;
proof.C0LogMin = -20;
proof.C0LogMax = 8;
proof.C0RefineCount = 12;
proof.C0SafetyFactor = 1.05;    % Numerical padding, not an interval proof.

pairs(1).label = 'current simulation gains';
pairs(1).obsK1 = paramsSim.obsK1;
pairs(1).obsK2 = paramsSim.obsK2;
pairs(2).label = 'sufficient-condition witness';
pairs(2).obsK1 = 3.0;
pairs(2).obsK2 = 1.2;

reports = repmat(struct(), 1, numel(pairs));
for pairIndex = 1:numel(pairs)
    params = paramsSim;
    params.obsK1 = pairs(pairIndex).obsK1;
    params.obsK2 = pairs(pairIndex).obsK2;

    reports(pairIndex).label = pairs(pairIndex).label;
    reports(pairIndex).report = computeGainConditionReport(params, proof);

    fprintf('\n=== %s ===\n', pairs(pairIndex).label);
    printGainConditionReport(reports(pairIndex).report);
end

outFile = fullfile(baseDir, 'data', 'gain_condition_report.mat');
if ~exist(fileparts(outFile), 'dir')
    mkdir(fileparts(outFile));
end
save(outFile, 'reports', 'pairs', 'paramsSim', 'proof');
fprintf('\nSaved report to %s\n', outFile);

function report = computeGainConditionReport(params, proof)
mu1 = params.mu1;
mu2 = params.mu2;
k1 = params.obsK1;
k2 = params.obsK2;
kappa = k2 / k1;
gamma = proof.gamma;

lambdaH = min(eig((params.H + params.H') / 2));
barDelta = estimateDeltaBound(params, proof.deltaSamples);
barD = barDelta / k1;

c0Upper = kappa * gamma - 2 * gamma * barD / (mu1^2);
if isempty(proof.c0)
    c0 = 0.8 * c0Upper;
else
    c0 = proof.c0;
end

rhoP = norm(params.Ap, 2) / (k1 * lambdaH);
rhoV = norm(params.Av, 2);
etaP = proof.etaP;
etaV = proof.etaV;

Cq = rhoP / (mu2^2) * (1 + (1 + gamma) * etaP / 2) ...
    + sqrt(2) * rhoV / mu2 * etaV / 2;
CW = rhoP / (mu2^2) * (gamma + (1 + gamma) / (2 * etaP)) ...
    + sqrt(2) * rhoV / mu2 * (gamma + 1 / (2 * etaV));

C0Info = estimateC0Numerically(mu1, mu2, kappa, gamma, c0, barD, proof);
C0 = proof.C0SafetyFactor * C0Info.value;

report.params.N = params.N;
report.params.mu1 = mu1;
report.params.mu2 = mu2;
report.params.obsK1 = k1;
report.params.obsK2 = k2;
report.params.kappa = kappa;
report.params.lambdaH = lambdaH;
report.params.ApNorm = norm(params.Ap, 2);
report.params.AvNorm = norm(params.Av, 2);
report.params.tSpan = params.tSpan;
report.params.distAmp = params.distAmp;

report.proof.gamma = gamma;
report.proof.etaP = etaP;
report.proof.etaV = etaV;
report.proof.c0 = c0;
report.proof.c0Upper = c0Upper;
report.proof.barDelta = barDelta;
report.proof.barD = barD;
report.proof.rhoP = rhoP;
report.proof.rhoV = rhoV;

report.constants.C0Numeric = C0Info.value;
report.constants.C0Used = C0;
report.constants.C0SafetyFactor = proof.C0SafetyFactor;
report.constants.C0Arg = C0Info.arg;
report.constants.Cq = Cq;
report.constants.CW = CW;

report.conditions.gain1.margin = c0Upper - c0;
report.conditions.gain1.pass = c0 > 0 && report.conditions.gain1.margin > 0;
report.conditions.gain2.lhs = k1 * lambdaH;
report.conditions.gain2.rhs = C0 + Cq;
report.conditions.gain2.margin = report.conditions.gain2.lhs - report.conditions.gain2.rhs;
report.conditions.gain2.pass = report.conditions.gain2.margin > 0;
report.conditions.gain2.requiredObsK1AtComputedConstants = report.conditions.gain2.rhs / lambdaH;
report.conditions.gain3.margin = c0 - CW;
report.conditions.gain3.pass = report.conditions.gain3.margin > 0;
report.conditions.allPass = report.conditions.gain1.pass ...
    && report.conditions.gain2.pass && report.conditions.gain3.pass;
end

function C0Info = estimateC0Numerically(mu1, mu2, kappa, gamma, c0, barD, proof)
gridN = proof.C0GridSize;
logGrid = linspace(proof.C0LogMin, proof.C0LogMax, gridN);
vals = exp(logGrid);

best.value = 0;
best.arg = struct('sigma', 1, 't', 0, 'u', 0, 'q', 0, 'N', 0, ...
    'method', 'grid');
starts = zeros(0, 3);

for sigma = [-1, 1]
    for i = 1:gridN
        t = vals(i);
        for j = 1:gridN
            u = vals(j);
            [ratio, qVal, nVal] = C0Ratio(t, u, sigma, mu1, mu2, ...
                kappa, gamma, c0, barD);
            if ratio > best.value
                best.value = ratio;
                best.arg = struct('sigma', sigma, 't', t, 'u', u, ...
                    'q', qVal, 'N', nVal, 'method', 'grid');
            end
            starts = keepBestStarts(starts, ratio, log(t), log(u), sigma, ...
                proof.C0RefineCount);
        end
    end
end

opts = optimset('Display', 'off', 'TolX', 1e-11, 'TolFun', 1e-11, ...
    'MaxIter', 2000, 'MaxFunEvals', 5000);
for row = 1:size(starts, 1)
    sigma = starts(row, 3);
    z0 = starts(row, 1:2).';
    obj = @(z) boundedNegativeRatio(z, sigma, mu1, mu2, kappa, ...
        gamma, c0, barD, proof.C0LogMin, proof.C0LogMax);
    zOpt = fminsearch(obj, z0, opts);
    zOpt = min(max(zOpt, proof.C0LogMin), proof.C0LogMax);
    t = exp(zOpt(1));
    u = exp(zOpt(2));
    [ratio, qVal, nVal] = C0Ratio(t, u, sigma, mu1, mu2, ...
        kappa, gamma, c0, barD);
    if ratio > best.value
        best.value = ratio;
        best.arg = struct('sigma', sigma, 't', t, 'u', u, ...
            'q', qVal, 'N', nVal, 'method', 'fminsearch');
    end
end

C0Info = best;
end

function value = boundedNegativeRatio(z, sigma, mu1, mu2, kappa, gamma, ...
    c0, barD, logMin, logMax)
penalty = 0;
if any(z < logMin) || any(z > logMax)
    dz = max(logMin - z, 0) + max(z - logMax, 0);
    penalty = 1e6 * sum(dz.^2);
end
z = min(max(z, logMin), logMax);
t = exp(z(1));
u = exp(z(2));
ratio = C0Ratio(t, u, sigma, mu1, mu2, kappa, gamma, c0, barD);
value = -ratio + penalty;
end

function starts = keepBestStarts(starts, ratio, logT, logU, sigma, maxCount)
candidate = [logT, logU, sigma, ratio];
starts = [starts; candidate]; %#ok<AGROW>
[~, order] = sort(starts(:, 4), 'descend');
starts = starts(order(1:min(maxCount, numel(order))), :);
end

function [ratio, qVal, nVal] = C0Ratio(t, u, sigma, mu1, mu2, ...
    kappa, gamma, c0, barD)
f = @(x) mu1 * x + mu2 * x.^2;
g = @(x) 0.5 * mu1^2 + 1.5 * mu1 * mu2 * x + mu2^2 * x.^2;

qVal = sigma * f(t) - f(u);
hVal = -sigma * t^2 + (1 + gamma) * u^2;
nVal = -kappa * hVal * sigma * g(t) + barD * abs(hVal) + c0 * u^2 * g(u);

den = qVal^2;
if den <= 1e-24
    if nVal > 0
        ratio = inf;
    else
        ratio = 0;
    end
else
    ratio = max(nVal, 0) / den;
end
end

function barDelta = estimateDeltaBound(params, sampleCount)
tGrid = linspace(params.tSpan(1), params.tSpan(2), sampleCount);
barDelta = 0;
for k = 1:numel(tGrid)
    t = tGrid(k);
    [p0, v0, a0] = targetReferenceLocal(t);
    u0 = a0 - params.Ap * p0 - params.Av * v0;
    for i = 1:params.N
        deltaI = u0 - satelliteDisturbanceLocal(t, i, params);
        barDelta = max(barDelta, max(abs(deltaI)));
    end
end
end

function params = loadSimulationParams(baseDir)
dataFile = fullfile(baseDir, 'data', 'cw_only_poi_data.mat');
if exist(dataFile, 'file')
    loaded = load(dataFile, 'params');
    params = loaded.params;
    return;
end

params.N = 5;
params.dim = 3;
params.mu_e = 3.986e5;
params.r0 = 7000;
params.n = sqrt(params.mu_e / params.r0^3);
params.Ap = [3 * params.n^2, 0, 0;
             0, 0, 0;
             0, 0, -params.n^2];
params.Av = [0, 2 * params.n, 0;
             -2 * params.n, 0, 0;
             0, 0, 0];

params.mu1 = 1.2;
params.mu2 = 1.0;
params.obsK1 = 3.0;
params.obsK2 = 5.0;
params.epsObs = 1e-3;

params.ctrlK1 = 1.0;
params.ctrlK2 = 1.0;
params.stK1 = 2.0;
params.stK2 = 2.0;
params.epsCtrl = 1e-3;

params.dSafe = 1;
params.ld = 2.8;
params.lmu = 3;
params.muRange = 4;
params.cr = 5.0;
params.cp = 5.0;
params.alphaMax = 1000;
params.apfGapFloor = 1e-4;
params.distAmp = 0.5;
params.tSpan = [0, 20];

params.informed = [1, 3];
params.beta = zeros(params.N, 1);
params.beta(params.informed) = 1;

pInit = [-3.2, -0.8, -0.8, -3.2, -2.0;
         -1.0, -0.8, -3.2, -3.2, -2.0;
         -3.2, -3.2, -3.2, -3.2, -4.0];
params.Aest = zeros(params.N);
for i = 1:params.N
    for j = i+1:params.N
        if norm(pInit(:, i) - pInit(:, j)) < params.muRange
            params.Aest(i, j) = 1;
            params.Aest(j, i) = 1;
        end
    end
end
params.Lest = diag(sum(params.Aest, 2)) - params.Aest;
params.H = params.Lest + diag(params.beta);
end

function d = satelliteDisturbanceLocal(t, i, params)
phase = 0.3 * (i - 1);
d = params.distAmp * [sin(0.5 * t + phase);
                      cos(0.6 * t + phase);
                      0.8 * sin(0.4 * t + 0.15 * (i - 1))];
end

function [p0, v0, a0] = targetReferenceLocal(t)
alpha = pi / 6;
p0 = [-5 + 0.6 * cos(alpha * t);
      1 + 0.4 * sin(alpha * t);
      t + 0.3 * (1 - cos(alpha * t))];
v0 = [-0.6 * alpha * sin(alpha * t);
      0.4 * alpha * cos(alpha * t);
      1 + 0.3 * alpha * sin(alpha * t)];
a0 = [-0.6 * alpha^2 * cos(alpha * t);
      -0.4 * alpha^2 * sin(alpha * t);
      0.3 * alpha^2 * cos(alpha * t)];
end

function printGainConditionReport(report)
fprintf('\nPosition-only observer gain-condition check\n');
fprintf('------------------------------------------------------------\n');
fprintf('Simulation parameters:\n');
fprintf('  N                         = %d\n', report.params.N);
fprintf('  mu1, mu2                  = %.12g, %.12g\n', ...
    report.params.mu1, report.params.mu2);
fprintf('  obsK1, obsK2, kappa        = %.12g, %.12g, %.12g\n', ...
    report.params.obsK1, report.params.obsK2, report.params.kappa);
fprintf('  lambda_H                  = %.12e\n', report.params.lambdaH);
fprintf('  ||A_p||_2, ||A_v||_2       = %.12e, %.12e\n', ...
    report.params.ApNorm, report.params.AvNorm);
fprintf('  sampled bar_delta          = %.12e\n', report.proof.barDelta);
fprintf('  bar_d = bar_delta / obsK1  = %.12e\n', report.proof.barD);

fprintf('\nProof parameters used by this script:\n');
fprintf('  gamma, c0                 = %.12g, %.12g\n', ...
    report.proof.gamma, report.proof.c0);
fprintf('  eta_p, eta_v              = %.12g, %.12g\n', ...
    report.proof.etaP, report.proof.etaV);
fprintf('  c0 upper bound             = %.12e\n', report.proof.c0Upper);

fprintf('\nComputed constants:\n');
fprintf('  C0 numeric candidate       = %.12e\n', report.constants.C0Numeric);
fprintf('  C0 used with safety factor = %.12e\n', report.constants.C0Used);
fprintf('  Cq                         = %.12e\n', report.constants.Cq);
fprintf('  CW                         = %.12e\n', report.constants.CW);
fprintf('  C0 maximizer candidate     = sigma=%+d, t=%.12e, u=%.12e, method=%s\n', ...
    report.constants.C0Arg.sigma, report.constants.C0Arg.t, ...
    report.constants.C0Arg.u, report.constants.C0Arg.method);

fprintf('\nInequality checks:\n');
printCondition('gain 1: 0 < c0 < kappa*gamma - 2*gamma*bar_d/mu1^2', ...
    report.conditions.gain1.pass, report.conditions.gain1.margin);
fprintf('  gain 2 lhs k1*lambda_H     = %.12e\n', report.conditions.gain2.lhs);
fprintf('  gain 2 rhs C0 + Cq         = %.12e\n', report.conditions.gain2.rhs);
fprintf('  approx. obsK1 required     = %.12e\n', ...
    report.conditions.gain2.requiredObsK1AtComputedConstants);
printCondition('gain 2: k1*lambda_H > C0 + Cq', ...
    report.conditions.gain2.pass, report.conditions.gain2.margin);
printCondition('gain 3: c0 > CW', ...
    report.conditions.gain3.pass, report.conditions.gain3.margin);
fprintf('  all numeric checks pass    = %d\n', report.conditions.allPass);

fprintf('\nNote: C0 is a floating-point global-search candidate with padding, not a\n');
fprintf('certified interval upper bound. Use interval arithmetic/SOS if the paper\n');
fprintf('needs a rigorous numerical certificate.\n');
end

function printCondition(name, pass, margin)
if pass
    status = 'PASS';
else
    status = 'FAIL';
end
fprintf('  %-62s %s, margin = %.12e\n', name, status, margin);
end
