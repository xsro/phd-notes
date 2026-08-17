function [dy, aux] = dya(t, y, params)
%DYA Closed-loop CW dynamics with the position-only distributed observer.

N = params.N;
dim = params.dim;
block = dim * N;

idxP = 1:block;
idxV = block + (1:block);
idxEta = 2 * block + (1:block);
idxPsi = 3 * block + (1:block);
idxP0 = 4 * block + (1:dim);
idxV0 = 4 * block + dim + (1:dim);
idxPhat = 4 * block + 2 * dim + (1:block);
idxVhat = 5 * block + 2 * dim + (1:block);

p = reshape(y(idxP), dim, N);
v = reshape(y(idxV), dim, N);
eta = reshape(y(idxEta), dim, N);
psi = reshape(y(idxPsi), dim, N);
p0 = y(idxP0);
v0 = y(idxV0);
phat = reshape(y(idxPhat), dim, N);
vhat = reshape(y(idxVhat), dim, N);

[~, ~, a0Ref] = targetReference(t);
u0 = a0Ref - params.Ap * p0 - params.Av * v0;

xi0TrueP = p - p0;
xi0TrueV = v - v0;
zeta = onlyPoiResidual(p, p0, phat, params.Aest, params.beta);
[phi1, phi2] = onlyPoiMaps(zeta, params.mu1, params.mu2, params.epsObs);
nuP = params.obsK1 * phi1;
nuV = params.obsK2 * phi2;

phi = zeros(dim, N);
for i = 1:N
    phi(:, i) = compositeAPFTerm(i, p, params);
end

[u, sHat, Gamma, stTerm] = onlyPoiControllerTerms(...
    phat, vhat, eta, psi, phi, nuP, nuV, params);

pDot = v;
vDot = zeros(dim, N);
for i = 1:N
    dSat = satelliteDisturbance(t, i, params);
    vDot(:, i) = params.Ap * p(:, i) + params.Av * v(:, i) ...
        + u(:, i) + dSat;
end

etaDot = params.ctrlK1 * phat - phi;
psiDot = -0.5 * params.stK2 * smoothSign(sHat, params.epsCtrl);
p0Dot = v0;
v0Dot = params.Ap * p0 + params.Av * v0 + u0;
phatDot = vhat - nuP;
vhatDot = params.Ap * phat + params.Av * vhat + u - nuV;

dy = [pDot(:); vDot(:); etaDot(:); psiDot(:); ...
      p0Dot; v0Dot; phatDot(:); vhatDot(:)];

if nargout > 1
    aux.sHat = sHat;
    aux.u = u;
    aux.phi = phi;
    aux.zeta = zeta;
    aux.phi1 = phi1;
    aux.phi2 = phi2;
    aux.nuP = nuP;
    aux.nuV = nuV;
    aux.Gamma = Gamma;
    aux.stTerm = stTerm;
    aux.phat = phat;
    aux.vhat = vhat;
    aux.xi0TrueP = xi0TrueP;
    aux.xi0TrueV = xi0TrueV;
    aux.estErrP = vecnorm(phat - xi0TrueP, 2, 1).';
    aux.estErrV = vecnorm(vhat - xi0TrueV, 2, 1).';
    aux.u0 = u0;
end
end

function phi = compositeAPFTerm(i, p, params)
phi = zeros(3, 1);

for j = 1:params.N
    if j == i
        continue;
    end

    pij = p(:, i) - p(:, j);
    distance = norm(pij);
    direction = pij / max(distance, 1e-9);

    alphaR = 0;
    if distance < params.ld
        safeGap = max(distance - params.dSafe, params.apfGapFloor);
        arDistance = params.cr / safeGap;
        arLd = params.cr / (params.ld - params.dSafe);
        alphaR = arDistance - arLd;
    end

    alphaP = 0;
    if params.Aest(i, j) > 0 && distance >= params.lmu
        safeGap = max(params.muRange - distance, params.apfGapFloor);
        alphaP = -params.cp / safeGap ...
            + params.cp / (params.muRange - params.lmu);
    end

    alphaR = max(min(alphaR, params.alphaMax), -params.alphaMax);
    alphaP = max(min(alphaP, params.alphaMax), -params.alphaMax);
    phi = phi + (alphaR + alphaP) * direction;
end
end

function value = smoothSign(x, epsVal)
value = x ./ max(abs(x), epsVal);
end

function d = satelliteDisturbance(t, i, params)
phase = 0.3 * (i - 1);
d = params.distAmp * [sin(0.5 * t + phase);
                      cos(0.6 * t + phase);
                      0.8 * sin(0.4 * t + 0.15 * (i - 1))];
end

function [p0, v0, a0] = targetReference(t)
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
