function th = ppc_theory(pp, e0, de0, w_bar)
% PPC_THEORY  由实测残差上界 w_bar 计算定理 3.2 的理论进带深度
%
% 定理 3.2:  |e_i(t)| <= vartheta_i * rho_i(t),
%   V_i    = 1/2*log(1/(1-xi_i^2)) + 1/2*s_i^2
%   c_i    = sigma_i^2/(2*k2_i),  sigma_i = w_bar/rho_inf_i
%   Vbar_i = 1/2*log((c_i+k1_i)/k1_i) + c_i/k2_i
%   vartheta_i = sqrt(1 - exp(-2*max(V_i(0), Vbar_i)))
%
% 输入
%   pp     ppc_settings 结构体
%   e0,de0 初始误差 (n x 1)
%   w_bar  阶段 I 残差上界 sup ||w||_inf (本函数用于"oracle 校验": 取同一次
%          仿真实测的 sup ||eps_2||, 即最小的合法 w_bar)
% 输出
%   th.vartheta  [n x 1] 理论贴合系数
%   th.V0, th.Vbar, th.c, th.B_o, th.xi0, th.s0

    n = numel(e0);

    % ---- 初始归一化状态 ----
    xi0 = e0 ./ pp.rho0;
    % 五次 smootherstep 包络: rho_dot(0) = 0
    rd0 = zeros(n, 1);
    xid0 = (de0 - xi0.*rd0) ./ pp.rho0;
    s0   = xid0 + pp.k1.*xi0;

    V0 = 0.5*log(1./(1 - xi0.^2)) + 0.5*s0.^2;

    % ---- 残差驱动项 ----
    c    = (w_bar./pp.rho_inf).^2 ./ (2*pp.k2);
    Vbar = 0.5*log((c + pp.k1)./pp.k1) + c./pp.k2;

    Vmax = max(V0, Vbar);
    vartheta = sqrt(1 - exp(-2*Vmax));

    % ---- 交接状态界 B_o (引理 4.1) ----
    B_o = sqrt( sum(vartheta.^2 .* pp.rho_inf.^2) + ...
                sum(pp.rho_inf.^2 .* (sqrt(2*Vmax) + pp.k1).^2) );

    th = struct('vartheta', vartheta, 'V0', V0, 'Vbar', Vbar, 'c', c, ...
                'Vmax', Vmax, 'B_o', B_o, 'xi0', xi0, 's0', s0, ...
                'w_bar', w_bar, 'n', n);
end
