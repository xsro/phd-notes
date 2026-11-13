function [dob, tau_hat] = ftdob_step(dob, M, Cmat, dq, tau_act, Ts)
% FTDOB_STEP  有限时间扰动观测器一步更新 (Dou & Yue 2025, Eq (35)(36))
%
%   p      = H_theta_m * dq
%   p_hat' = k1*phi1(p - p_hat) + C_theta_m' * dq + tau_mc + Delta_tau + tau_hat
%   tau_hat' = k2*phi2(p - p_hat)
% with
%   phi1 = mu1 sig(e)^alpha + mu2 sig(e)^beta
%   phi2 = mu1^2 alpha sig(e)^{2alpha-1}
%        + mu1 mu2 (alpha+beta) sig(e)^{alpha+beta-1}
%        + mu2^2 beta sig(e)^{2beta-1},      e = p - p_hat
%
% 说明: 式 (34) 中 p 的导数含 tau_mc + Delta_tau, 而 Delta_tau 是饱和偏差,
% 因此这里直接用"实际施加的力矩" tau_act = tau_mc + Delta_tau。
%
% 输入/输出 dob 为观测器结构体 (含 phat, tauhat 与增益), tau_hat 为估计值。

    if isempty(dob) || ~isfield(dob, 'enabled') || ~dob.enabled
        tau_hat = zeros(size(dq));
        return;
    end

    ep = M*dq - dob.phat;
    phi1 = dob.mu1*sig_pow(ep, dob.alpha) + dob.mu2*sig_pow(ep, dob.beta);
    phi2 = dob.mu1^2*dob.alpha*sig_pow(ep, 2*dob.alpha - 1) ...
         + dob.mu1*dob.mu2*(dob.alpha + dob.beta)*sig_pow(ep, dob.alpha + dob.beta - 1) ...
         + dob.mu2^2*dob.beta*sig_pow(ep, 2*dob.beta - 1);

    dob.phat   = dob.phat   + Ts*( dob.k1*phi1 + Cmat.'*dq + tau_act + dob.tauhat );
    dob.tauhat = dob.tauhat + Ts*( dob.k2*phi2 );
    tau_hat = dob.tauhat;
end

% ------------------------------------------------------------------------
function y = sig_pow(x, r)
    y = abs(x).^r .* sign(x);
end
