function [metrics, series] = validate_results(t, y, params, metricsFile, enforce)
%VALIDATE_RESULTS Compute and optionally enforce the simulation criteria.

if nargin < 5
    enforce = true;
end

t = t(:);
N = params.N;
dim = params.dim;
block = dim * N;
numT = numel(t);
expectedDim = 6 * block + 2 * dim;
if size(y, 1) ~= numT || size(y, 2) ~= expectedDim
    error('validate_results:StateSizeMismatch', ...
        'Expected y to have size %d-by-%d.', numT, expectedDim);
end

series.p = zeros(dim, N, numT);
series.v = zeros(dim, N, numT);
series.p0 = zeros(dim, numT);
series.v0 = zeros(dim, numT);
series.phat = zeros(dim, N, numT);
series.vhat = zeros(dim, N, numT);
series.estErrP = zeros(numT, N);
series.estErrV = zeros(numT, N);
series.centroidError = zeros(numT, 1);
series.sHat = zeros(numT, N);
series.uNorm = zeros(numT, N);
series.nuPNorm = zeros(numT, N);
series.nuVNorm = zeros(numT, N);
series.gammaNorm = zeros(numT, N);
series.pairDistance = zeros(numT, N * (N - 1) / 2);

initialEdges = find(triu(params.Aest, 1) > 0);
[edgeI, edgeJ] = ind2sub([N, N], initialEdges);
series.initialEdgeDistance = zeros(numT, numel(initialEdges));

for k = 1:numT
    yk = y(k, :).';
    series.p(:, :, k) = reshape(yk(1:block), dim, N);
    series.v(:, :, k) = reshape(yk(block + (1:block)), dim, N);
    series.p0(:, k) = yk(4 * block + (1:dim));
    series.v0(:, k) = yk(4 * block + dim + (1:dim));
    series.phat(:, :, k) = reshape(...
        yk(4 * block + 2 * dim + (1:block)), dim, N);
    series.vhat(:, :, k) = reshape(...
        yk(5 * block + 2 * dim + (1:block)), dim, N);

    [~, aux] = dya(t(k), yk, params);
    series.estErrP(k, :) = aux.estErrP.';
    series.estErrV(k, :) = aux.estErrV.';
    series.centroidError(k) = norm(mean(series.p(:, :, k), 2) - series.p0(:, k));
    series.sHat(k, :) = vecnorm(aux.sHat, 2, 1);
    series.uNorm(k, :) = vecnorm(aux.u, 2, 1);
    series.nuPNorm(k, :) = vecnorm(aux.nuP, 2, 1);
    series.nuVNorm(k, :) = vecnorm(aux.nuV, 2, 1);
    series.gammaNorm(k, :) = vecnorm(aux.Gamma, 2, 1);

    pairIndex = 1;
    for i = 1:N-1
        for j = i+1:N
            series.pairDistance(k, pairIndex) = norm(...
                series.p(:, i, k) - series.p(:, j, k));
            pairIndex = pairIndex + 1;
        end
    end
    for e = 1:numel(initialEdges)
        series.initialEdgeDistance(k, e) = norm(...
            series.p(:, edgeI(e), k) - series.p(:, edgeJ(e), k));
    end
end

windowLength = max(2, ceil(0.1 * numT));
initialWindow = 1:windowLength;
finalWindow = (numT - windowLength + 1):numT;

metrics.maxFinalPositionError = max(series.estErrP(finalWindow, :), [], 'all');
metrics.maxFinalVelocityError = max(series.estErrV(finalWindow, :), [], 'all');
metrics.maxFinalCentroidError = max(series.centroidError(finalWindow));
metrics.minPairDistance = min(series.pairDistance, [], 'all');
metrics.maxInitialEdgeDistance = max(series.initialEdgeDistance, [], 'all');
metrics.maxControlNorm = max(series.uNorm, [], 'all');
metrics.allFinite = all(isfinite(y), 'all') ...
    && all(isfinite(series.estErrP), 'all') ...
    && all(isfinite(series.estErrV), 'all') ...
    && all(isfinite(series.uNorm), 'all');

metrics.initialPositionRms = sqrt(mean(series.estErrP(initialWindow, :).^2, 'all'));
metrics.finalPositionRms = sqrt(mean(series.estErrP(finalWindow, :).^2, 'all'));
metrics.initialVelocityRms = sqrt(mean(series.estErrV(initialWindow, :).^2, 'all'));
metrics.finalVelocityRms = sqrt(mean(series.estErrV(finalWindow, :).^2, 'all'));
metrics.initialCentroidRms = sqrt(mean(series.centroidError(initialWindow).^2));
metrics.finalCentroidRms = sqrt(mean(series.centroidError(finalWindow).^2));
metrics.finalToInitialPositionRms = metrics.finalPositionRms ...
    / max(metrics.initialPositionRms, eps);
metrics.finalToInitialVelocityRms = metrics.finalVelocityRms ...
    / max(metrics.initialVelocityRms, eps);
metrics.finalToInitialCentroidRms = metrics.finalCentroidRms ...
    / max(metrics.initialCentroidRms, eps);
metrics.safetyMargin = metrics.minPairDistance - params.dSafe;
metrics.connectivityMargin = params.muRange - metrics.maxInitialEdgeDistance;

save(metricsFile, 'metrics', 'series');
printMetrics(metrics);

if enforce
    assert(metrics.allFinite, 'Non-finite state or derived signal detected.');
    assert(metrics.minPairDistance > params.dSafe, 'Safety distance violated.');
    assert(metrics.maxInitialEdgeDistance < params.muRange, 'Initial edge lost.');
    assert(metrics.finalToInitialPositionRms < 0.05, ...
        'Position estimate did not converge enough.');
    assert(metrics.finalToInitialVelocityRms < 0.05, ...
        'Velocity estimate did not converge enough.');
    assert(metrics.finalToInitialCentroidRms < 0.05, ...
        'Centroid error did not converge enough.');
end
end

function printMetrics(metrics)
fprintf('\nPosition-only observer validation metrics:\n');
fprintf('  max final position error   = %.6e km\n', metrics.maxFinalPositionError);
fprintf('  max final velocity error   = %.6e km/s\n', metrics.maxFinalVelocityError);
fprintf('  max final centroid error   = %.6e km\n', metrics.maxFinalCentroidError);
fprintf('  position RMS ratio         = %.6e\n', metrics.finalToInitialPositionRms);
fprintf('  velocity RMS ratio         = %.6e\n', metrics.finalToInitialVelocityRms);
fprintf('  centroid RMS ratio         = %.6e\n', metrics.finalToInitialCentroidRms);
fprintf('  minimum pair distance      = %.6e km\n', metrics.minPairDistance);
fprintf('  maximum initial-edge dist. = %.6e km\n', metrics.maxInitialEdgeDistance);
fprintf('  maximum control norm       = %.6e km/s^2\n', metrics.maxControlNorm);
fprintf('  safety margin              = %.6e km\n', metrics.safetyMargin);
fprintf('  connectivity margin        = %.6e km\n\n', metrics.connectivityMargin);
end

