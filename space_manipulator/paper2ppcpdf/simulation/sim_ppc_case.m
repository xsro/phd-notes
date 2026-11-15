% SIM_PPC_CASE  单个算例的三段时序仿真 (PPC -> 分段观测器 -> PDF)
%
% 由 main_ppc_pdf_verify.m 的局部函数抽出, 供主验证脚本与 T_o 扫描脚本共用。
%   [0, To)     : 分段观测器阶段 I (奇异增益) + 漏斗型 PPC (cs.kind='ppc')
%                 或纯基准反馈 ('pd')
%   [To, To+2h) : 分段观测器阶段 II (超螺旋) + PDF 单周期零化
%
% 观测器与力矩律严格按论文式 (eq:ptdo)、(eq:hatd)、(eq:computed_torque):
%   y = M_e*dq,  u_p = tau_act - C_e*dq + Mdot_e*dq,
%   tau_cmd = C_e*dq + M_e*(ddq_d + nu) - d_hat.
%
% 采样实现的因果次序 (每拍 k):
%   (1) 取 d_hat(t_k) = z2 (上一拍已推进到 t_k);
%   (2) 由 d_hat(t_k) 与 nu(t_k) 算 tau_cmd、限幅得 tau_act;
%   (3) 用 y(t_k) 与 tau_act 把观测器从 t_k 推进到 t_k+Ts。
% 这样观测器使用的零阶保持输入正是区间 [t_k,t_k+Ts] 内实际施加的力矩。
%
% 输入
%   cfg, P, ctrl, ref   仿真设置 / 对象 / 基准增益+PDF 增益 / 参考轨迹
%   cs                  算例 (name, kind, dist, rho_inf)
%   pp                  ppc_settings 结构体
%   oprm                ptdo_step 参数 (To, kappa1, kappa2)
% 输出
%   r   name, kind, m(统计), raw(时间序列), pp

function r = sim_ppc_case(cfg, P, ctrl, ref, cs, pp, oprm)
    n = cfg.n;  N = cfg.N;  Ts = cfg.Ts;
    To = cfg.To;  h = ctrl.h;

    q  = cfg.q_init;
    dq = cfg.dq_init;

    Mdel = round(h/Ts);
    z_hist = zeros(2*n, Mdel + 1);
    ost = struct('z1', zeros(n,1), 'z2', zeros(n,1));

    E   = zeros(n, N);  Ed  = zeros(n, N);  TAU = zeros(n, N);
    RHO = zeros(n, N);  RD  = zeros(n, N);  RDD = zeros(n, N);
    EPS2 = zeros(n, N); DAH = zeros(n, N);  DDA = zeros(n, N);
    NU  = zeros(n, N);  DRI = zeros(n, N);  ZN  = zeros(1, N);
    PE  = zeros(1, N);  SAT = false(1, N);
    XI  = zeros(n, N);  E1  = zeros(n, N);
    Z1  = zeros(n, N);  Z2N = zeros(n, N);  YM = zeros(n, N);
    nu_max = 0;  xiviol = false;

    for k = 1:N-1
        tk = cfg.t(k);

        % ---------- 扰动 (Eq 74) 与被控对象 ----------
        if cs.dist, d = dou_yue_259_disturbance(tk, n); else, d = zeros(n, 1); end
        Sa = ffsm259_model(P, q, dq);
        Ma = Sa.Me;  Caqd = Sa.Ce_qd;  Cn = Sa.Cmat;  Mn = Ma;
        pe = Sa.pe;

        e  = q - ref.qd(:,k);
        de = dq - ref.dqd(:,k);
        z  = [e; de];

        % ---------- 性能函数 ----------
        [rho, rd, rdd] = perf_rho(tk, pp);
        RHO(:,k) = rho;  RD(:,k) = rd;  RDD(:,k) = rdd;

        % ---------- 观测器输出 (已推进到 t_k) ----------
        d_hat = ost.z2;
        DAH(:,k) = d_hat;
        DDA(:,k) = d;
        EPS2(:,k) = Ma \ (d - d_hat);             % w (加速度通道残差)

        % ---------- 辅助输入 nu: 三段时序 ----------
        if tk < To
            switch cs.kind
                case 'ppc'
                    [nu, dg] = ppc_step(e, de, rho, rd, rdd, pp.k1, pp.k2);
                    XI(:,k) = dg.xi;
                    nu_max = max(nu_max, dg.barrier_inf);
                    xiviol = xiviol || dg.xi_violation;
                    % 可选的交接整形: 末段速度阻尼 (t in [To-delta, To))
                    if isfield(oprm, 'damp') && ~isempty(oprm.damp) && ...
                            tk >= To - oprm.damp(2)
                        nu = nu - oprm.damp(1)*de;
                    end
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

        % ---------- 计算力矩 + 饱和 ----------
        tau_cmd = Cn*dq + Mn*(ref.ddqd(:,k) + nu) - d_hat;
        tau_act = min(max(tau_cmd, P.tau_min), P.tau_max);
        SAT(k)  = any(abs(tau_cmd) > P.tau_max);

        % ---------- 观测器: 由 t_k 推进到 t_k+Ts (式 (eq:ptdo)) ----------
        y   = Mn*dq;                              % 动量
        u_s = -Caqd + Sa.Mdot_qd;                 % 平滑输入 (u_p - tau)
        [ost, ~, odg] = ptdo_step(ost, y, tau_act, u_s, tk, Ts, oprm);
        E1(:,k) = odg.e1;
        Z1(:,k) = ost.z1;  Z2N(:,k) = ost.z2;  YM(:,k) = y;

        % ---------- 等效残余扰动 (命题 prop:approx 的 Delta_r) ----------
        DRI(:,k) = Ma \ (d - d_hat) + Ma \ (tau_act - tau_cmd);

        % ---------- 记录 + 半隐式积分 ----------
        TAU(:,k) = tau_act;
        E(:,k)   = e;  Ed(:,k) = de;
        ZN(k)    = norm(z, inf);
        PE(k)    = norm(pe - ref.pe(:,k));

        ddq = Ma \ (tau_act + d - Caqd);
        dq  = dq + Ts*ddq;
        q   = q  + Ts*dq;
    end

    % 末拍
    eN = q - ref.qd(:,end);  deN = dq - ref.dqd(:,end);
    E(:,end) = eN;  Ed(:,end) = deN;  ZN(end) = norm([eN; deN], inf);
    TAU(:,end) = TAU(:,max(end-1,1));
    RHO(:,end) = RHO(:,max(end-1,1));
    DAH(:,end) = DAH(:,max(end-1,1));
    DDA(:,end) = DDA(:,max(end-1,1));
    EPS2(:,end) = EPS2(:,max(end-1,1));

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
    % 观测器诊断: 阶段 I / II 的扰动估计误差 (同一时刻)
    msk2 = cfg.t >= To;
    m.d_hat_err_max = max(max(abs(DDA(:, msk2) - DAH(:, msk2))));
    msk_ss = cfg.t >= To + 2*h;
    m.d_hat_err_ss  = max(max(abs(DDA(:, msk_ss) - DAH(:, msk_ss))));

    raw = struct('E', E, 'Ed', Ed, 'TAU', TAU, 'RHO', RHO, 'RD', RD, ...
        'RDD', RDD, 'EPS2', EPS2, 'DAH', DAH, 'DDA', DDA, 'NU', NU, ...
        'DRI', DRI, 'ZN', ZN, 'PE', PE, 'SAT', SAT, 'XI', XI, ...
        'E1', E1, 'Z1', Z1, 'Z2N', Z2N, 'YM', YM, 'e_end', m.e_end);
    r = struct('name', cs.name, 'kind', cs.kind, 'm', m, 'raw', raw, 'pp', pp);
end
