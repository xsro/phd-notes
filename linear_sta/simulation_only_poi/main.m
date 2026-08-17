clear; clc; close all;

baseDir = fileparts(mfilename('fullpath'));
addpath(baseDir);
dataDir = fullfile(baseDir, 'data');
figDir = fullfile(baseDir, 'figures');
if ~exist(dataDir, 'dir'); mkdir(dataDir); end
if ~exist(figDir, 'dir'); mkdir(figDir); end

params.N = 5;
params.dim = 3;
params.mu_e = 3.986e5;          % km^3/s^2
params.r0 = 7000;               % km
params.n = sqrt(params.mu_e / params.r0^3);
params.Ap = [3*params.n^2, 0, 0;
             0, 0, 0;
             0, 0, -params.n^2];
params.Av = [0, 2*params.n, 0;
             -2*params.n, 0, 0;
             0, 0, 0];

% Position-only GSTA observer.
params.mu1 = 1.2;
params.mu2 = 1.0;
params.obsK1 = 3.0;
params.obsK2 = 5.0;
params.epsObs = 1e-3;

% Fencing controller with the double-channel observer compensation.
params.ctrlK1 = 1.0;
params.ctrlK2 = 1.0;
params.stK1 = 2.0;
params.stK2 = 2.0;
params.epsCtrl = 1e-3;

params.dSafe = 1;               % km
params.ld = 2.8;                % km
params.lmu = 3;                 % km
params.muRange = 4;             % km
params.cr = 5.0;
params.cp = 5.0;
params.alphaMax = 1000;
params.apfGapFloor = 1e-4;
params.distAmp = 0.5;           % km/s^2
params.tSpan = [0, 20];         % s

if ~(params.dSafe < params.ld && params.ld < params.lmu ...
        && params.lmu < params.muRange)
    error('APF parameters must satisfy dSafe < ld < lmu < muRange.');
end

params.informed = [1, 3];
params.beta = zeros(params.N, 1);
params.beta(params.informed) = 1;

pInit = [-3.2, -0.8, -0.8, -3.2, -2.0;
         -1.0, -0.8, -3.2, -3.2, -2.0;
         -3.2, -3.2, -3.2, -3.2, -4.0];
vInit = zeros(3, params.N);
etaInit = zeros(3, params.N);
psiInit = zeros(3, params.N);
phatInit = ones(3, params.N);
vhatInit = ones(3, params.N);

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

minDist0 = inf;
for i = 1:params.N-1
    for j = i+1:params.N
        minDist0 = min(minDist0, norm(pInit(:, i) - pInit(:, j)));
    end
end
if minDist0 <= params.dSafe
    error('Initial inter-satellite distance violates dSafe.');
end
if numel(unique(conncomp(graph(params.Aest)))) > 1
    error('The initial estimation graph Aest is not connected.');
end
if min(eig((params.H + params.H') / 2)) <= 0
    error('The anchored matrix H must be positive definite.');
end

p0Init = [-4.4; 1; 0];
v0Init = [0; 0.4 * pi / 6; 1];
y0 = [pInit(:); vInit(:); etaInit(:); psiInit(:); ...
      p0Init; v0Init; phatInit(:); vhatInit(:)];

odeOpt = odeset('RelTol', 1e-5, 'AbsTol', 1e-7, 'MaxStep', 0.02);
fprintf('[%s] Starting position-only observer simulation...\n', ...
    char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
simTimer = tic;
[t, y] = ode45(@(tt, yy) dya(tt, yy, params), params.tSpan, y0, odeOpt);
simElapsed = toc(simTimer);
fprintf('[%s] ode45 finished. Elapsed time: %.2f s\n', ...
    char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), simElapsed);

dataFile = fullfile(dataDir, 'cw_only_poi_data.mat');
save(dataFile, 't', 'y', 'params');

metricsFile = fullfile(dataDir, 'cw_only_poi_metrics.mat');
validate_results(t, y, params, metricsFile, true);

run(fullfile(baseDir, 'paper_fig.m'));
fprintf('[%s] Validation and figure export finished.\n', ...
    char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
