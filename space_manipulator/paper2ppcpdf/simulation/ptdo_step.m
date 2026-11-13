function [st, da_hat] = ptdo_step(st, chi, u_o, t, Ts, obs)
% PTDO_STEP  预设时间扰动观测器 (Prescribed-Time Disturbance Observer)
%
% 论文 Eq (19)-(22)（观测通道 \\dot{chi} = u_o + Delta_a, 见 Eq (17)）:
%
%   eps1 = chi - z1 - xi_bar(t)                                    (19)
%   phi1 = sig(eps1/sigma)^{1-eta/2} + sig(eps1/sigma)^{1+eta/2}
%   phi2 = sig(eps1/sigma)^{2-eta} + sig(eps1/sigma)^{2+eta}
%          + sigma*sign(eps1/sigma)                                (21)(22)
%
%   \\dot z1 = z2 + pi/(eta*T_c) * phi1 - \\dot xi + xi + u_o          (20)
%   \\dot z2 = pi/(sigma*eta*T_c) * phi2 - \\dot xi
%
%   Delta_a_hat = z2
%
% 调节函数 xi_bar(t): t < T_o 时 xi = xi0*(T_o-t)^2, \\dot xi = -2*xi0*(T_o-t);
% t >= T_o 时为 0（预设观测时间 T_o）。
%
% 输入
%   st  观测器状态 (z1, z2)
%   chi 观测通道量测, 取关节角速度 \\dot q
%   u_o 观测通道输入, u_o = M^{-1}(tau_act - C*\\dot q)
%   t   当前时间 [s]
%   Ts  步长 [s]
%   obs 结构体: T_o, T_c, eta, sigma, xi0
%
% 输出
%   st      更新后的状态
%   da_hat  等效加速度扰动估计 [n x 1]

    [xi, xi_dot] = regulation(t, obs);

    eps1 = chi - st.z1 - xi;
    se   = eps1 ./ obs.sigma;

    phi1 = sigp(se, 1 - obs.eta/2) + sigp(se, 1 + obs.eta/2);
    phi2 = sigp(se, 2 - obs.eta) + sigp(se, 2 + obs.eta) + obs.sigma*sign(se);

    z1_dot = st.z2 + (pi/(obs.eta*obs.T_c))*phi1 - xi_dot + xi + u_o;
    z2_dot = (pi/(obs.sigma*obs.eta*obs.T_c))*phi2 - xi_dot;

    st.z1 = st.z1 + Ts*z1_dot;
    st.z2 = st.z2 + Ts*z2_dot;

    da_hat = st.z2;
end

% ------------------------------------------------------------------------
function [xi, xi_dot] = regulation(t, obs)
    if t < obs.T_o
        r = obs.T_o - t;
        xi     = obs.xi0 .* (r^2);
        xi_dot = -2 * obs.xi0 .* r;
    else
        xi     = zeros(size(obs.xi0));
        xi_dot = zeros(size(obs.xi0));
    end
end

function y = sigp(x, p)
    y = sign(x) .* (abs(x).^p);
end
