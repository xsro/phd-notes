clear; close all; clc;

% Result 3 simulation:
% APF-filtered individual PPC with unknown target total acceleration and
% unknown agent disturbances.  The controller does not use u0+d0 or d_i.

rootDir = fileparts(fileparts(mfilename('fullpath')));
outDir = fullfile(rootDir, 'simulation', 'out');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

%% Parameters
N = 4;
n = 2;
T = 25;
dt = 0.005;
t = 0:dt:T;
K = numel(t);

safeDist = 0.65;
mu = 2.60;
kRep = 0.85;

gamma1 = 2.0;
gamma2 = 2.4;

rho0 = 4.20;
rhoInf = 0.55;
ell = 0.22;

c1 = 1.8;
c2 = 5.0;
adaptGamma = 1.8;
kappa0 = 0.25;
leak = 0.04;
satEps = 0.06;

%% Initial conditions
p0 = [0.0; 0.0];
v0 = [0.22; -0.04];

P = [ 2.25,  1.10, -1.95, -1.05;
      0.85,  2.20, -1.25, -2.05];
V = repmat(v0, 1, N) + [ 0.05, -0.04,  0.03, -0.03;
                        -0.03,  0.04, -0.04,  0.02];

Phi1 = zeros(n, N);
Phi2 = zeros(n, N);
kappaHat = kappa0 * ones(1, N);
alphaPrev = zeros(n, N);

pHist = zeros(n, N, K);
p0Hist = zeros(n, K);
centerErr = zeros(1, K);
rhoHist = zeros(1, K);
pairDist = zeros(nchoosek(N, 2), K);
etaMax = zeros(1, K);
etaNormHist = zeros(N, K);
kappaHist = zeros(N, K);

pairs = nchoosek(1:N, 2);

%% Simulation loop
for k = 1:K
    tk = t(k);

    rho = (rho0 - rhoInf) * exp(-ell * tk) + rhoInf;
    dotRho = -ell * (rho0 - rhoInf) * exp(-ell * tk);
    sigma = 0.5 * rho^2;
    dotSigma = rho * dotRho;

    pHist(:, :, k) = P;
    p0Hist(:, k) = p0;
    centerErr(k) = norm(mean(P, 2) - p0);
    rhoHist(k) = rho;
    kappaHist(:, k) = kappaHat(:);

    for q = 1:size(pairs, 1)
        pairDist(q, k) = norm(P(:, pairs(q, 1)) - P(:, pairs(q, 2)));
    end

    phi = compute_apf(P, safeDist, mu, kRep);
    aPhi = gamma1^2 * Phi1 - (gamma1 + gamma2) * Phi2 + phi;

    eta = P - p0 - Phi1;
    chi = V - v0 + gamma1 * Phi1 - Phi2;
    etaNormHist(:, k) = vecnorm(eta).';
    etaMax(k) = max(etaNormHist(:, k));

    U = zeros(n, N);
    alphaNow = zeros(n, N);

    for i = 1:N
        eta_i = eta(:, i);
        chi_i = chi(:, i);
        etaNormSq = eta_i' * eta_i;

        e_i = 0.5 * etaNormSq;
        z_i = min(e_i / sigma, 0.995);
        theta_i = atanh(z_i);
        zeta_i = 1.0 / (1.0 - z_i^2);

        dotEStar = sigma * (1.0 - z_i^2) * (-c1 * theta_i) + z_i * dotSigma;
        if etaNormSq > 1e-10
            alpha_i = (dotEStar / etaNormSq) * eta_i;
        else
            alpha_i = zeros(n, 1);
        end
        alphaNow(:, i) = alpha_i;

        if k == 1
            dotAlpha_i = zeros(n, 1);
        else
            dotAlpha_i = (alpha_i - alphaPrev(:, i)) / dt;
        end

        s_i = chi_i - alpha_i;
        sat_i = max(-1, min(1, s_i / satEps));

        U(:, i) = aPhi(:, i) + dotAlpha_i ...
            - c2 * s_i ...
            - (zeta_i / sigma) * eta_i * theta_i ...
            - kappaHat(i) * sat_i;

        kappaDot = adaptGamma * norm(s_i, 1) - leak * (kappaHat(i) - kappa0);
        kappaHat(i) = max(kappa0, kappaHat(i) + dt * kappaDot);
    end

    alphaPrev = alphaNow;

    if k == K
        break;
    end

    a0 = target_accel(tk);
    D = agent_disturbance(tk, N);

    % Plant and filter integration.
    P = P + dt * V;
    V = V + dt * (U + D);
    p0 = p0 + dt * v0;
    v0 = v0 + dt * a0;

    Phi1 = Phi1 + dt * (-gamma1 * Phi1 + Phi2);
    Phi2 = Phi2 + dt * (-gamma2 * Phi2 + phi);
end

%% Figures
fig1 = figure('Color', 'w', 'Position', [100, 100, 760, 430]);
plot(t, centerErr, 'LineWidth', 2.2); hold on;
plot(t, rhoHist, '--', 'LineWidth', 2.2);
etaColors = lines(N);
for i = 1:N
    plot(t, etaNormHist(i, :), ':', 'LineWidth', 1.7, ...
        'Color', etaColors(i, :));
end
grid on; box on;
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Distance', 'Interpreter', 'latex');
etaLabels = arrayfun(@(i) sprintf('$\\|\\eta_%d(t)\\|$', i), ...
    1:N, 'UniformOutput', false);
legend([{'$\|\bar p-p_0\|$', '$\rho(t)$'}, etaLabels], ...
    'Interpreter', 'latex', 'Location', 'northeast');
title('Centroid and filtered-error distances under Result 3', ...
    'Interpreter', 'latex');
exportgraphics(fig1, fullfile(outDir, 'result3_centroid_ppc.pdf'), ...
    'ContentType', 'vector');
exportgraphics(fig1, fullfile(outDir, 'result3_centroid_ppc.png'), ...
    'Resolution', 300);

fig2 = figure('Color', 'w', 'Position', [120, 120, 760, 430]);
hold on;
for q = 1:size(pairDist, 1)
    plot(t, pairDist(q, :), 'LineWidth', 1.6);
end
yline(safeDist, 'k--', 'LineWidth', 2.0);
grid on; box on;
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Pairwise distance', 'Interpreter', 'latex');
pairLabels = arrayfun(@(q) sprintf('$d_{%d%d}$', pairs(q,1), pairs(q,2)), ...
    1:size(pairs, 1), 'UniformOutput', false);
legend([pairLabels, {'$d$'}], 'Interpreter', 'latex', ...
    'Location', 'eastoutside');
title('Inter-agent distances under Result 3', 'Interpreter', 'latex');
exportgraphics(fig2, fullfile(outDir, 'result3_pairwise_distances.pdf'), ...
    'ContentType', 'vector');
exportgraphics(fig2, fullfile(outDir, 'result3_pairwise_distances.png'), ...
    'Resolution', 300);

save(fullfile(outDir, 'result3_simulation.mat'), ...
    't', 'centerErr', 'rhoHist', 'pairDist', 'safeDist', ...
    'etaNormHist', 'etaMax', 'kappaHist', 'pHist', 'p0Hist', 'pairs');

fprintf('Result 3 simulation finished.\n');
fprintf('Final centroid error: %.4f\n', centerErr(end));
fprintf('Final PPC envelope: %.4f\n', rhoHist(end));
fprintf('Minimum pairwise distance: %.4f\n', min(pairDist, [], 'all'));
fprintf('Figures saved in: %s\n', outDir);

%% Local functions
function phi = compute_apf(P, safeDist, mu, kRep)
    [n, N] = size(P);
    phi = zeros(n, N);
    for i = 1:N
        for j = i+1:N
            pij = P(:, i) - P(:, j);
            rij = norm(pij);
            if rij < mu
                dir = pij / max(rij, 1e-9);
                margin = max(rij - safeDist, 1e-4);
                active = 1.0 / margin - 1.0 / (mu - safeDist);
                mag = kRep * active^2;
                fij = mag * dir;
                phi(:, i) = phi(:, i) + fij;
                phi(:, j) = phi(:, j) - fij;
            end
        end
    end
end

function a0 = target_accel(t)
    a0 = [0.10 * sin(0.55 * t) + 0.06 * cos(1.15 * t);
          0.08 * cos(0.45 * t) - 0.04 * sin(0.90 * t)];
end

function D = agent_disturbance(t, N)
    D = zeros(2, N);
    for i = 1:N
        D(:, i) = [0.045 * sin(0.85 * t + 0.7 * i);
                   0.040 * cos(0.65 * t + 0.4 * i)];
    end
end
