% SIM_METRICS  验收指标 M1..M7 (论文第 5 节表 3)
%
%   M1a/M1b : 观测阶段 / PDF 阶段的包络违反时长 [s*joint]
%   M2      : T_o 时刻的进带深度 max_i |e_i(T_o)|/rho_inf
%   M3      : 首次达到 ||z||_inf <= 1e-5 的时刻; M4: PDF 阶段的越界比
%   M5      : t >= To+2h 后的 sup||z||_inf 与命题 4.2 的上界
%   M6      : ||Delta_K(h)|| 与 cond(W_c)
%
% 输入 r (sim_ppc_case 输出), th (ppc_theory 输出), pp, an (pdf_periodian 输出), cfg

function mt = sim_metrics(r, th, pp, an, cfg)
    t = cfg.t;  To = cfg.To;  h = cfg.h;  n = cfg.n;
    E = r.raw.E;  RHO = r.raw.RHO;  ZN = r.raw.ZN;

    % ---- M1: 包络违反时长 (分观测阶段 / PDF 阶段) ----
    viol  = abs(E) > RHO;
    msk_a = t < To;
    mt.viol_time    = sum(trapz(t, double(viol), 2));
    mt.viol_time_p1 = sum(trapz(t(msk_a), double(viol(:, msk_a)), 2));
    mt.viol_time_p2 = sum(trapz(t(~msk_a), double(viol(:, ~msk_a)), 2));
    mt.viol_joints  = find(any(viol, 2)).';

    % ---- M2: 进带深度 ----
    [~, kTo] = min(abs(t - To));
    mt.e_at_To    = abs(E(:, kTo));
    mt.band_ratio = max(mt.e_at_To ./ pp.rho_inf);

    % ---- M3: 归零时刻 (阈值 1e-9 只可能在理想算例中达到) ----
    idx = find(t >= To & ZN <= 1e-9, 1, 'first');
    if isempty(idx), mt.zero_time = NaN; else, mt.zero_time = t(idx); end
    msk_after = t >= To + 2*h;
    mt.ss_sup  = max(ZN(msk_after));
    mt.ss_min  = min(ZN(msk_after));
    % 达到 1e-5 (工程精度) 的时刻, 供非理想算例参考
    idx2 = find(t >= To & ZN <= 1e-5, 1, 'first');
    if isempty(idx2), mt.t_1e5 = NaN; else, mt.t_1e5 = t(idx2); end

    % ---- M4: 阶段 II 越界比 ----
    msk2 = t >= To & t < To + 2*h;
    mt.stage2_ratio  = max(max(abs(E(:, msk2)), [], 2) ./ pp.rho_inf);
    % 精确判据: |e_i| <= Mpos(i)*B_o  (命题 4.1, 行块形式)
    mt.stage2_theory = max(an.Mpos .* th.B_o ./ pp.rho_inf);
    % 先验闭式上界 (保守)
    mt.stage2_theory_gr = an.M_h * th.B_o / min(pp.rho_inf);

    % ---- M5: 稳态残差 vs 显式上界 ----
    Dr = max(max(abs(r.raw.DRI(:, msk_after))));
    epsD = min(an.deltaK, 1 - 1e-12);
    mt.ss_bound = an.Gamma * Dr / (1 - epsD);
    mt.Delta_r_bar = Dr;
    mt.eps_Delta = epsD;

    % ---- M6 ----
    mt.deltaK = an.deltaK;
    mt.cond   = an.cond;
end

% ------------------------------------------------------------------------
