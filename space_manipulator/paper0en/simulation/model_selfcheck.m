function m = model_selfcheck(P)
%MODEL_SELFCHECK  Consistency checks of the reduced free-floating model.
%
%   m = MODEL_SELFCHECK(P) verifies, for a fixed test configuration, that
%   (i)   the full inertia matrix is symmetric and positive definite,
%   (ii)  the zero-momentum condition H_b vb + H_bm dq = 0 is reproduced,
%   (iii) the reduced model satisfies the skew-symmetry identity
%         dq' (Mdot_e dq - 2 C_e dq) = 0,
%   (iv)  the kinetic energy of the reduced model is conserved for tau = 0.
%
%   The values printed by the main script are the ones referenced in the
%   implementation remark of the paper.

n = P.n;
q  = P.q0 + deg2rad([5; -4; 3; -2; 4; -3; 2]);
dq = 0.1*[1; -1; 0.6; 0.4; -0.5; 0.3; -0.2];

[H, ~] = ffsm_dou_crba(P, q);
m.min_eig_full = min(eig(H));
m.sym_err = norm(H - H.');

Hbb = H(1:6,1:6); Hbm = H(1:6,7:end); Hmm = H(7:end,7:end);
vb = -Hbb \ (Hbm*dq);
m.momentum_residual = norm(Hbb*vb + Hbm*dq);

[Me, Cvec] = ffsm_dou_reduced(P, q, dq);
m.min_eig_Me = min(eig(Me));
eps = 1e-6;
Mp = ffsm_dou_reduced(P, q + eps*dq, dq);
Mm = ffsm_dou_reduced(P, q - eps*dq, dq);
Mdot = (Mp - Mm)/(2*eps);
m.skew_residual = abs(dq.'*(Mdot*dq - 2*Cvec));
m.skew_scale = norm(dq)*norm(Mdot*dq);

% energy conservation of the reduced model under tau = 0
Ts = 1e-4; N = round(1/Ts);
qk = q; dqk = dq;
E0 = 0.5*dqk.'*Me*dqk;
for k = 1:N
    [Mk, Ck] = ffsm_dou_reduced(P, qk, dqk);
    dqk = dqk + Ts*(Mk \ (-Ck));
    qk  = qk  + Ts*dqk;
end
[Mk, ~] = ffsm_dou_reduced(P, qk, dqk);
m.energy_drift = abs(0.5*dqk.'*Mk*dqk - E0)/E0;

end
