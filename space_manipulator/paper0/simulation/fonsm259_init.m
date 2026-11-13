function c = fonsm259_init(method, gains, dob, asc, n, Ts, mem_len, kappa)
% FONSM259_INIT  初始化 Dou & Yue (2025) 分数阶滑模控制器状态。
%
% method  : 'fonsm' (Eq 15 滑模面) | 'fosm' (Eq 77, Shao 等人)
% gains   : dou_yue_259_control_params() 的 .fonsm 或 .fosm 字段
% dob     : .ftdob 字段, 结构体; enabled 由 dob_enable 控制
% asc     : .ftasc / .ftac 字段, 配合 asc.mode = 'ftasc'|'ftac'|'none'
% n, Ts   : 维数与步长
% mem_len : 分数阶算子的短记忆长度 (采样点数), 默认 750
%
% 说明: 分数阶算子采用 Grünwald-Letnikov 短记忆离散,
%       D^{xi-1}[.] 与 D^{xi}[.] 都由同一个积分核实现 (Caputo 定义)。

    if nargin < 7 || isempty(mem_len)
        mem_len = 750;
    end
    if nargin < 8 || isempty(kappa)
        kappa = 1;
    end

    c.method = method;
    c.kappa  = kappa;   % 修正通道增益标定系数 (见主脚本说明)
    c.n = n;
    c.Ts = Ts;

    % ---- 滑模面增益 ----
    c.C1 = gains.C1;
    c.C2 = gains.C2;
    c.xi = gains.xi;
    c.b  = gains.b;
    if strcmp(method, 'fonsm')
        c.r1 = gains.r1;
        c.r2 = gains.r2;
        c.delta = gains.delta;
        % Eq (15) 的边界层系数, 由滑模面及其导数在 |e| = delta 处的连续性确定
        c.kchi1 = (c.r1 - c.r2)/(1 - c.r2) * c.delta^(c.r1 - 1);
        c.kchi2 = (1 - c.r1)/(1 - c.r2) * c.delta^(c.r1 - c.r2);
        c.lambda1 = gains.lambda1;
        c.lambda2 = gains.lambda2;
        c.rho3 = gains.rho3;
    else
        c.lambda1 = gains.C1;   % FOSM: 结构同 Eq (77), 用同一组趋近增益
        c.lambda2 = 0*gains.C1;
        c.rho3 = 0.5;
    end

    % ---- 分数阶记忆 ----
    c.nu = c.xi - 1;                       % ∈ (-1,0)
    c.mem_len = mem_len;
    c.memA = zeros(mem_len, n);
    c.memB = zeros(mem_len, n);
    c.cnt  = 0;
    % GL 权重 w_0 = 1, w_k = (1 - (nu+1)/k) w_{k-1}
    w = ones(mem_len, 1);
    for k = 2:mem_len
        w(k) = w(k-1) * (1 - (c.nu + 1)/(k-1));
    end
    c.w = w;

    % ---- 有限时间扰动观测器 (Eq 35/36) ----
    c.dob = dob;
    c.dob.enabled = false;                 % 默认关闭, 由 fonsm259_enable_dob 打开
    c.dob.phat = dob.phat0;
    c.dob.tauhat = zeros(n, 1);

    % ---- 抗饱和补偿器 ----
    c.asc = asc;
    if ~isfield(c.asc, 'mode')
        c.asc.mode = 'none';
    end
    c.asc.eta = zeros(n, 1);
    if strcmp(c.asc.mode, 'ftasc')
        c.asc.Leta = c.asc.L0;
    else
        c.asc.Leta = 1;
    end
end
