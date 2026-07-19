function [u, sHat, Gamma, stTerm] = onlyPoiControllerTerms(...
    phat, vhat, eta, psi, phi, nuP, nuV, params)
%ONLYPOICONTROLLERTERMS Evaluate the compensated fencing controller.

sHat = eta + params.ctrlK2 * phat + vhat;
sSmooth = sHat ./ max(abs(sHat), params.epsCtrl);
signedSqrt = sqrt(abs(sHat)) .* sSmooth;

Gamma = params.ctrlK2 * nuP + nuV;
stTerm = -params.stK1 * signedSqrt + psi;

u = -params.Ap * phat - params.Av * vhat ...
    - params.ctrlK1 * phat - params.ctrlK2 * vhat ...
    + phi + Gamma + stTerm;
end

