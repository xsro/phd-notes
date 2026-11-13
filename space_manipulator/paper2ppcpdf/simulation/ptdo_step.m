function [st, da_hat] = ptdo_step(st, chi, u_o, t, Ts, obs)
% PTDO_STEP  预设时间扰动观测器 (Prescribed-Time Disturbance Observer)
%
% 论文式 (14)-(16)（观测通道 \\dot{chi} = u_o + Delta_a, 见式 (13)）:
%
%   eta1 = chi - z1 - xi_bar(t)                                   (14)
%   phi1 = sig(eta1/sigma)^{1-eta/2} + sig(eta1/sigma)^{1+eta/2}   (16)
%   phi2 = sig(eta1/sigma)^{2-eta} + sig(eta1/sigma)^{2+eta}
%          + sigma*sgn(eta1/sigma)                                 (16)
%
%   \\dot z1 = z2 + pi/(eta*T_c) * phi1(eta1/sigma) - \\dot xi + u_o    (15)
%   \\dot z2 = pi/(sigma*eta*T_c) * phi2(eta1/sigma)                 (15)
%
%   Delta_a_hat = z2
%
% 与式 (15)-(16) 逐项一致：\\dot z1 中不含 +xi_bar，\\dot z2 中不含 -\\dot xi_bar。
% 唯一的实现选择是时间积分（显式 Euler，步长 Ts）与 sgn 在 0 处取 0
% （属 Filippov 集 [-1,1] 内的单值选取）。
%
% 调节函数 xi_bar(t)=xi0*(T_o-t)^2*1_{t<T_o}: t < T_o 时 xi = xi0*(T_o-t)^2,
% \\dot xi = -2*xi0*(T_o-t)；t >= T_o 时为 0（预设观测时间 T_o，且 xi_bar 在
% T_o 处 C^1）。按式 (14) 之后的说明取 xi0 = (chi(0)-z1(0))/T_o^2 可使
% eta1(0)=0；取 xi0 = 0 则退化为不施加调节函数。
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

    eta1 = chi - st.z1 - xi;              % 式 (14): eta1 = chi - z1 - xi_bar
    se   = eta1 ./ obs.sigma;             % 式 (15)-(16) 的自变量 eta1/sigma

    phi1 = sigp(se, 1 - obs.eta/2) + sigp(se, 1 + obs.eta/2);                 % 式 (16)
    phi2 = sigp(se, 2 - obs.eta) + sigp(se, 2 + obs.eta) + obs.sigma*sign(se);  % 式 (16)

    z1_dot = st.z2 + (pi/(obs.eta*obs.T_c))*phi1 - xi_dot + u_o;   % 式 (15)
    z2_dot = (pi/(obs.sigma*obs.eta*obs.T_c))*phi2;                 % 式 (15)

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
