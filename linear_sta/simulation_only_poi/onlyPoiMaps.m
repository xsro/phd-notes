function [phi1, phi2] = onlyPoiMaps(z, mu1, mu2, epsObs)
%ONLYPOIMAPS Evaluate the boundary-layer GSTA correction maps.

s = z ./ max(abs(z), epsObs);
rootTerm = sqrt(abs(z)) .* s;

phi1 = mu1 * rootTerm + mu2 * z;
phi2 = 0.5 * mu1^2 * s ...
    + 1.5 * mu1 * mu2 * rootTerm ...
    + mu2^2 * z;
end

