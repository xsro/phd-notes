function [Me, Cvec, Hbb, Hbm, Hmm, kin] = ffsm_dou_reduced(P, q, dq)
%FFSM_DOU_REDUCED  Equivalent joint-space model of the free-floating chain.
%
%   [Me, Cvec] = FFSM_DOU_REDUCED(P, q, dq) returns the reduced dynamics
%
%       Me(q) qdd + Cvec(q, dq) = tau,       Cvec = Ce(q, dq) dq,
%
%   obtained by eliminating the base acceleration with the zero-momentum
%   condition (Assumption 1 of the paper):
%
%       Me   = H_m  - H_bm' H_b^{-1} H_bm          (Schur complement)
%       Cvec = C_m  - H_bm' H_b^{-1} C_b,
%
%   where the bias forces C_b, C_m are evaluated at the actual base
%   velocity vb = -H_b^{-1} H_bm dq. Base reaction motion is therefore
%   retained exactly in the equivalent model.

[H_full, kin] = ffsm_dou_crba(P, q);

Hbb = H_full(1:6, 1:6);
Hbm = H_full(1:6, 7:end);
Hmm = H_full(7:end, 7:end);

Hbb_inv_Hbm = Hbb \ Hbm;
Me = Hmm - Hbm.' * Hbb_inv_Hbm;
Me = 0.5 * (Me + Me.');

vb = -Hbb_inv_Hbm * dq;

[Cb, Cm] = ffsm_dou_bias(P, q, dq, vb, kin);
Cvec = Cm - Hbm.' * (Hbb \ Cb);

end
