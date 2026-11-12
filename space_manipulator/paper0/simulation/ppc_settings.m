function pp = ppc_settings(To, e0, opt)
% PPC_SETTINGS  构造性能函数参数与漏斗增益 (论文式 (11)、(22))
%
% 输入
%   To   预设观测/进带时间 [s]
%   e0   初始关节位置误差 [n x 1] (用于取 rho0, 见假设 2.4)
%   opt  可选: .rho_inf (标量或 [n x 1], 默认 1e-3)
%              .margin  (rho0 = margin*|e0|, 默认 2.0)
%              .rho0_min(初始包络下限, 默认 8e-3 rad)
%              .m       (性能函数指数, 默认 5)
%              .k1,.k2  (漏斗增益, 默认 0.5, 1 -> omega=0.71, zeta=0.71)
% 输出
%   pp   结构体: To, m, rho0, rho_inf, k1, k2, kmin, guide
%
% 设计约束 (推论 4.1): 2*min(k1,k2)*To >= ln(max(rho0/rho_inf)) 才能保证
% 误差在 T_o 时充分贴近 rho_inf; 本函数给出校验结果 pp.guide。

    n = numel(e0);
    if nargin < 3, opt = struct(); end
    opt = def(opt, 'rho_inf', 1e-3);
    opt = def(opt, 'margin', 2.0);
    opt = def(opt, 'rho0_min', 0);
    opt = def(opt, 'm', 5);
    opt = def(opt, 'k1', 0.5);
    opt = def(opt, 'k2', 1.0);

    pp.To      = To;
    pp.m       = opt.m;
    pp.rho_inf = opt.rho_inf*ones(n, 1);
    % 初始包络: 取初始误差的 margin 倍 (可选硬下限 rho0_min)
    % 注意: 设定硬下限会使个别轻质关节 (第 7 关节, Me_77 ~ 0.28) 的初始归一化
    % 误差变小, 从而削弱其障碍项权柄, 反而在 t ~ 2 s 处"贴住"包络并因饱和失稳
    % (见论文第 5 节的不可行性说明), 故默认关闭。
    pp.rho0    = max(opt.margin*abs(e0), opt.margin*pp.rho_inf);
    if opt.rho0_min > 0
        pp.rho0 = max(pp.rho0, opt.rho0_min);
    end
    pp.k1      = opt.k1*ones(n, 1);
    pp.k2      = opt.k2*ones(n, 1);
    pp.kmin    = min(pp.k1(1), pp.k2(1));

    % 推论 4.1 的整定校验
    need  = max(log(pp.rho0./pp.rho_inf));
    have  = 2*pp.kmin*To;
    pp.guide = struct('need', need, 'have', have, 'ok', have >= need);
end

% ------------------------------------------------------------------------
function s = def(s, f, v)
    if ~isfield(s, f), s.(f) = v; end
end
