% SIM_PPC_CASE  单个算例的三段时序仿真 (PPC -> PTDO -> PDF)
%
% 由 main_ppc_pdf_verify.m 的局部函数抽出, 供主验证脚本与 T_c 扫描脚本共用。
%   [0, To)     : PTDO 收敛 + 漏斗型 PPC (cs.kind='ppc') 或纯基准反馈 ('pd')
%   [To, To+2h) : PDF 单周期零化 (局部时间 vartheta = t - To)
%
% 输入
%   cfg, P, ctrl, ref   仿真设置 / 对象 / 基准增益+PDF 增益 / 参考轨迹
%   cs                  算例 (name, kind, dist, rho_inf)
%   pp                  ppc_settings 结构体
%   oprm                ptdo_step 参数 (T_o, T_c, eta, sigma, xi0)
% 输出
%   r   name, kind, m(统计), raw(时间序列), pp

function r = sim_ppc_case(cfg, P, ctrl, ref, cs, pp, oprm)
    n = cfg.n;  N = cfg.N;  Ts = cfg.Ts;
    To = cfg.To;  h = ctrl.h;

    q  = cfg.q_init;
    dq = cfg.dq_init;

    Mdel = round(h/Ts);
    z_hist = zeros(2*n, Mdel + 1);
    tau_cmd_prev = zeros(n, 1);
    tau_act_prev = zeros(n, 1);
    ost = struct('z1', zeros(n,1), 'z2', zeros(n,1));

    E   = zeros(n, N);  Ed  = zeros(n, N);  TAU = zeros(n, N);
    RHO = zeros(n, N);  RD  = zeros(n, N);  RDD = zeros(n, N);
    EPS2 = zeros(n, N); DAH = zeros(n, N);  DDA = zeros(n, N);
    NU  = zeros(n, N);  DRI = zeros(n, N);  ZN  = zeros(1, N);
    PE  = zeros(1, N);  SAT = false(1, N);
    XI  = zeros(n, N);
    nu_max = 0;  xiviol = false;

    for k = 1:N-1
        tk = cfg.t(k);

        % ---------- 扰动 (Eq 74) 与被控对象 ----------
        if cs.dist, d = dou_yue_259_disturbance(tk, n); else, d = zeros(n, 1); end
        Sa = ffsm259_model(P, q, dq);
        Ma = Sa.Me;  Caqd = Sa.Ce_qd;  Mn = Ma;  Cn = Sa.Cmat;
        pe = Sa.pe;
        DDA(:,k) = Ma \ d;

        e  = q - ref.qd(:,k);
        de = dq - ref.dqd(:,k);
        z  = [e; de];

        % ---------- 性能函数 ----------
        [rho, rd, rdd] = perf_rho(tk, pp);
        RHO(:,k) = rho;  RD(:,k) = rd;  RDD(:,k) = rdd;

        % ---------- PTDO ----------
        u_o = Mn \ (tau_act_prev - Cn*dq);
        [ost, da_hat] = ptdo_step(ost, dq, u_o, tk, Ts, oprm);
        DAH(:,k) = da_hat;
        EPS2(:,k) = DDA(:,k) - da_hat;      % w = Delta_a - z2 (供诊断)

        % ---------- 辅助输入 nu: 三段时序 ----------
        if tk < To
            switch cs.kind
                case 'ppc'
                    [nu, dg] = ppc_step(e, de, rho, rd, rdd, pp.k1, pp.k2);
                    XI(:,k) = dg.xi;
                    nu_max = max(nu_max, dg.barrier_inf);
                    xiviol = xiviol || dg.xi_violation;
                otherwise
                    nu = ctrl.K0*z;
                    XI(:,k) = e ./ rho;
            end
        else
            if tk - Ts < To                       % 交接拍: 清空历史, 只保留当前
                z_hist = zeros(2*n, Mdel + 1);
            end
            z_hist = [z, z_hist(:, 1:end-1)];
            Kc = pdf_gain_at(ctrl.pdf, tk - To);
            nu = ctrl.K0*z - Kc*z_hist(:, end);
        end
        NU(:,k) = nu;

        tau_cmd = Cn*dq + Mn*(ref.ddqd(:,k) + nu - da_hat);

        % ---------- 饱和 ----------
        tau_act = min(max(tau_cmd, P.tau_min), P.tau_max);
        SAT(k)  = any(abs(tau_cmd) > P.tau_max);

        % ---------- 等效残余扰动 (命题 4.2 的 Delta_r) ----------
        DRI(:,k) = Ma \ (d - Mn*da_hat) + Ma \ (tau_act - tau_cmd);

        % ---------- 记录 + 半隐式积分 ----------
        TAU(:,k) = tau_act;
        E(:,k)   = e;  Ed(:,k) = de;
        ZN(k)    = norm(z, inf);
        PE(k)    = norm(pe - ref.pe(:,k));

        ddq = Ma \ (tau_act + d - Caqd);
        dq  = dq + Ts*ddq;
        q   = q  + Ts*dq;

        tau_cmd_prev = tau_cmd;
        tau_act_prev = tau_act;
    end

    % 末拍
    eN = q - ref.qd(:,end);  deN = dq - ref.dqd(:,end);
    E(:,end) = eN;  Ed(:,end) = deN;  ZN(end) = norm([eN; deN], inf);
    TAU(:,end) = TAU(:,max(end-1,1));
    RHO(:,end) = RHO(:,max(end-1,1));

    % ---- 基础统计 ----
    m.e_end     = norm(E(:,end));
    m.de_end    = norm(Ed(:,end));
    m.max_tau   = max(abs(TAU), [], 'all');
    m.sat_ratio = mean(SAT);
    msk_a = cfg.t < To;
    m.sat_ratio_p1 = mean(SAT(msk_a));
    m.sat_ratio_p2 = mean(SAT(~msk_a));
    [~, k_tau]     = max(max(abs(TAU), [], 1));
    m.t_max_tau    = cfg.t(k_tau);
    m.i_max_tau    = find(max(abs(TAU), [], 1) == max(max(abs(TAU), [], 1)), 1);
    m.ee_end    = PE(end);
    m.xi_barrier_max = nu_max;
    m.xi_violation   = xiviol;
    % 阶段 I 残差上界 (oracle): 只在 [0, To) 上取
    msk1   = cfg.t < To;
    m.w_bar = max(max(abs(EPS2(:, msk1))));
    m.maxe_phase1 = max(max(abs(E(:, msk1))));
    m.pe_phase1   = max(PE(msk1));

    raw = struct('E', E, 'Ed', Ed, 'TAU', TAU, 'RHO', RHO, 'RD', RD, ...
        'RDD', RDD, 'EPS2', EPS2, 'DAH', DAH, 'DDA', DDA, 'NU', NU, ...
        'DRI', DRI, 'ZN', ZN, 'PE', PE, 'SAT', SAT, 'XI', XI, ...
        'e_end', m.e_end);
    r = struct('name', cs.name, 'kind', cs.kind, 'm', m, 'raw', raw, 'pp', pp);
end

% ------------------------------------------------------------------------
