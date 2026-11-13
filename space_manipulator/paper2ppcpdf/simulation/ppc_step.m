function [nu, dg] = ppc_step(e, de, rho, rho_dot, rho_ddot, k1, k2)
% PPC_STEP  漏斗型预设性能律 (论文式 (21)-(22))
%
%   归一化误差  xi_i      = e_i / rho_i                    (论文式 (21))
%   辅助状态    s_i       = xi_dot_i + k1_i*xi_i
%   控制律      nu_i      = rho_i*[-k2_i*s_i - xi_i/(1-xi_i^2) - k1_i*xi_dot_i]
%                           + rho_ddot_i*xi_i + 2*rho_dot_i*xi_dot_i        (22)
%
% 其中 -xi/(1-xi^2) 为障碍项, 系数必须取 1 才能与 xi 动力学中的交叉项
% 精确相消 (定理 3.2 证明第 (2) 步)。
%
% 输入
%   e, de              关节位置/速度误差 [n x 1]
%   rho, rho_dot, rho_ddot   性能函数及其导数 (perf_rho 给出) [n x 1]
%   k1, k2             漏斗增益 [n x 1] 或标量
% 输出
%   nu   辅助加速度输入 [n x 1]
%   dg   诊断结构体: xi, xi_dot, s, barrier_inf, xi_violation

    n = numel(e);
    k1 = expand(k1, n);   k2 = expand(k2, n);

    xi = e ./ rho;

    % ---- 数值保护: 理论上 |xi| < 1 (定理 3.2) ----
    xi_abs = abs(xi);
    viol = max(xi_abs) >= 1;
    if viol
        s_ = sign(xi);  s_(s_ == 0) = 1;
        xi = min(xi_abs, 1 - 1e-9) .* s_;
    end

    xi_dot = (de - xi .* rho_dot) ./ rho;
    s      = xi_dot + k1 .* xi;

    barrier = xi ./ (1 - xi.^2);
    % ---- 数值限幅: 仅当包络已被破坏 (例如力矩饱和所致) 时生效, 避免 nu 爆炸 ----
    % 仅作数值保护 (防止 xi=1 处的浮点除法溢出)。若把限幅取小 (例如
    % |xi|=0.95 处的 9.7), 包络被力矩饱和破坏后律不再提供足够的恢复力矩,
    % 数值实验表明会从"贴住包络"退化为数十弧度的漂移。
    bmax = 1e4;
    barrier = min(max(barrier, -bmax), bmax);

    nu = rho .* (-k2.*s - barrier - k1.*xi_dot) ...
       + rho_ddot .* xi + 2*rho_dot .* xi_dot;

    dg = struct('xi', xi, 'xi_dot', xi_dot, 's', s, ...
                'barrier_inf', norm(barrier, inf), ...
                'xi_violation', viol, 'xi_abs_max', max(xi_abs));
end

% ------------------------------------------------------------------------
function v = expand(v, n)
    if isscalar(v), v = v*ones(n, 1); end
    v = v(:);
end
