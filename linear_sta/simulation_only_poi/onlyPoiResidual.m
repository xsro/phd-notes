function zeta = onlyPoiResidual(p, p0, phat, Aest, beta)
%ONLYPOIRESIDUAL Build the distributed residual from relative positions.

[dim, N] = size(p);
zeta = zeros(dim, N);

for i = 1:N
    for j = 1:N
        if Aest(i, j) ~= 0
            zeta(:, i) = zeta(:, i) + Aest(i, j) * (...
                (phat(:, i) - phat(:, j)) - (p(:, i) - p(:, j)));
        end
    end

    if beta(i) ~= 0
        zeta(:, i) = zeta(:, i) + beta(i) * (...
            phat(:, i) - (p(:, i) - p0));
    end
end
end

