%% main_dou_yue_259_compare.m
% =========================================================================
% 自由漂浮空间机械臂: 参数取文献 [259]，并与其控制器对比
%
% 参数来源 (OKB 文献库 document id 259):
%   Bo Dou, Xiaokui Yue, "Fractional-Order Sliding Mode Control for
%   Free-Floating Space Manipulator With Disturbance and Input Saturation",
%   Int. J. Robust Nonlinear Control, 2025, 35(8): 3094-3115.
%   DOI: 10.1002/rnc.7826
%   -> Table 1 物理参数, Table 2 初始状态, Table 3 控制参数,
%      Eq (74) 扰动力矩, tau_max = 20 Nm
%
% 对比对象
%   (A) 本工程的 PDF 控制器 (Gramian 型光滑周期延迟反馈), PD, PDF+FTDOB
%   (B) 文献 [259] 的 FONTSM + FTDOB + 固定时间抗饱和补偿器 (提出方法)
%       FONTSM + FTDOB + 有限时间补偿器 [29]      (原文 Case 2)
%       FONTSM + FTDOB 无补偿                     (原文 Case 3)
%       FOSM (Shao 等) + FTDOB + FTASC            (原文 Part II 对比对象)
%       FONTSM + FTDOB + FTASC + 自适应增益 L_eta (原文 Eq 44)
%
% 任务说明
%   原文一级任务是末端笛卡尔轨迹跟踪, 二级任务是基座姿态稳定 (Eq 48-56,
%   需 H_w/H_m/C_b1/C_m1 的 v/w 分块与零空间投影)。本工程的对比统一在
%   **关节空间**进行: 同样的被控对象、参考轨迹、扰动与饱和限幅, 只替换控制律,
%   因此 (A)(B) 面对的误差通道一致, 比较是公平的; 末端误差另由正运动学评估。
%
% 关于增益标定系数 kappa
%   Table 3 的增益是按原文的笛卡尔误差量级 (mm) 标定的, 直接用于本对象
%   的关节空间 (M_theta_m 对角元 0.28~96 kg m^2) 时, 初始指令力矩会达到
%   ~450 Nm, 即 tau_max = 20 Nm 的 20 倍以上, 全程饱和, 各补偿器方案
%   无法区分。为此对 259 控制器的"修正通道"引入单一标定系数 kappa:
%       ddq_r = ddq_d + kappa*( ... )                     (Eq 50 的修正项)
%   kappa 不改变控制律结构, 只把闭环修正增益标定到与被控对象匹配的量级。
%   本脚本同时输出 kappa 标定后与 kappa = 1 (原文增益直用) 两张表。
%
% 运行:  matlab -batch "main_dou_yue_259_compare"
% =========================================================================
clear; clc; close all;

%% ---------------- 1. 仿真设置 ----------------
cfg.Ts       = 2e-3;                 % 积分步长 [s]
cfg.Tf       = 20.0;                 % 仿真时长 [s]
cfg.Tref     = 8.0;                  % 参考轨迹过渡时间 [s]
cfg.n        = 7;
cfg.mem_len  = 750;                  % 分数阶短记忆长度 [采样点] = 1.5 s
cfg.uncertainty = false;             % true: 启用 Part II 的 m7/I7 摄动
cfg.kappa    = 0.01;                 % 259 控制器的增益标定系数 (见文件头)
cfg.save_figures = true;
cfg.show_figures = false;

cfg.t = 0:cfg.Ts:cfg.Tf;
cfg.N = numel(cfg.t);

script_dir = fileparts(mfilename('fullpath'));
cfg.figure_dir  = fullfile(script_dir, '..', 'figures');
cfg.result_file = fullfile(script_dir, 'results_dou_yue_259.mat');
if cfg.save_figures && ~exist(cfg.figure_dir, 'dir')
    mkdir(cfg.figure_dir);
end
if ~cfg.show_figures
    set(0, 'DefaultFigureVisible', 'off');
end

%% ---------------- 2. 参数 (文献 259 Table 1/2) ----------------
P  = dou_yue_259_parameters();
CP = dou_yue_259_control_params();
n  = P.n;

fprintf('参数来源: OKB doc %d -- %s\n', P.ref.id, P.ref.venue);
fprintf('  基座 m=%.0f kg, I=%.0f kg m^2; 连杆质量 %s kg\n', ...
    P.mass(1), P.I{1}(1,1), mat2str(P.mass(2:end)));
fprintf('  tau_max = %g Nm; q0 = %s deg\n', P.tau_max(1), mat2str(rad2deg(P.q0).', 4));
fprintf('  Me 对角量级: %s kg m^2\n', mat2str(diag(ffsm259_model(P, P.q0).Me).', 4));

%% ---------------- 3. 参考轨迹 (关节空间五次多项式) ----------------
q_ref0  = P.q0;
q_ref_f = P.q0 + deg2rad([20; -15; 15; -20; 15; -15; 10]);
[ref.qd, ref.dqd, ref.ddqd] = quintic_joint_trajectory(q_ref0, q_ref_f, cfg.Tref, cfg.t);

ref.pe = zeros(3, cfg.N);
for k = 1:cfg.N
    ref.pe(:,k) = ffsm259_model(P, ref.qd(:,k)).pe;
end

cfg.q_init  = P.q0 + deg2rad([0.5; -0.4; 0.4; -0.5; 0.4; -0.3; 0.2]);
cfg.dq_init = P.dq0;

%% ---------------- 4. PDF 控制器参数 (本工程方法) ----------------
Kp = diag([30, 30, 30, 25, 25, 20, 8]);
Kd = diag([12, 12, 12, 10, 10,  9, 4]);
ctrl.K0  = [-Kp, -Kd];
ctrl.h   = 1.0;
ctrl.pdf = build_pdf_gain(ctrl.K0, n, ctrl.h);
fprintf('\nPDF Gramian: cond(W_c) = %.3e, h = %.1f s\n', ctrl.pdf.condition_number, ctrl.h);

%% ---------------- 5. 算例定义 ----------------
%  kind: 'pd' | 'pdf' | 'pdfdob' | 'fonsm' | 'fosm'
%  asc : 'none' | 'ftac' | 'ftasc'
cases = struct('name', {}, 'kind', {}, 'dob', {}, 'asc', {}, 'dist', {}, 'adp', {});

cases(end+1) = mk("PD, 无扰动",                'pd',     false, 'none',  false, false);
cases(end+1) = mk("PDF, 无扰动",               'pdf',    false, 'none',  false, false);
cases(end+1) = mk("PD, 有扰动",                'pd',     false, 'none',  true,  false);
cases(end+1) = mk("PDF, 有扰动",               'pdf',    false, 'none',  true,  false);
cases(end+1) = mk("PDF+FTDOB[259], 有扰动",    'pdfdob', true,  'none',  true,  false);
cases(end+1) = mk("FONTSM+FTDOB, 无补偿",      'fonsm',  true,  'none',  true,  false);
cases(end+1) = mk("FONTSM+FTDOB+FTAC[29]",     'fonsm',  true,  'ftac',  true,  false);
cases(end+1) = mk("FONTSM+FTDOB+FTASC",        'fonsm',  true,  'ftasc', true,  false);
cases(end+1) = mk("FONTSM+FTDOB+FTASC+自适应", 'fonsm',  true,  'ftasc', true,  true);
cases(end+1) = mk("FOSM+FTDOB+FTASC",          'fosm',   true,  'ftasc', true,  false);
nc = numel(cases);

%% ---------------- 6. 主循环 (两种增益标定) ----------------
regimes = [cfg.kappa, 1.0];
regime_name = {sprintf('kappa = %g (按本对象标定)', cfg.kappa), 'kappa = 1 (Table 3 增益直用)'};
all_results = cell(numel(regimes), nc);

for gi = 1:numel(regimes)
    kappa = regimes(gi);
    fprintf('\n==== 仿真 [%s]: %d 个算例, %.0f s @ Ts = %.0e ====\n', ...
        regime_name{gi}, nc, cfg.Tf, cfg.Ts);
    tic;
    for ci = 1:nc
        cs = cases(ci);
        fprintf('  [%2d/%2d] %-28s ', ci, nc, cs.name);
        r = run_one(cfg, P, CP, ctrl, ref, cs, kappa);
        all_results{gi, ci} = r;
        m = r.m;
        fprintf('|e(T)|=%.2e RMS=%.2e max|tau|=%5.1f sat=%5.1f%% t_s=%s\n', ...
            m.final_e, m.rms_e, m.max_tau, 100*m.sat_ratio, num2str(m.settle, '%.2f'));
    end
    fprintf('  耗时 %.1f s\n', toc);
end

%% ---------------- 7. 对比汇总 ----------------
for gi = 1:numel(regimes)
    fprintf('\n========== 对比汇总 [%s] ==========\n', regime_name{gi});
    fprintf('tau_max = %g Nm, Ts = %.0e s, 扰动 Eq(74), 初始误差 %s deg\n', ...
        P.tau_max(1), cfg.Ts, mat2str(rad2deg(cfg.q_init - P.q0).', 3));
    fprintf('%-30s %10s %10s %10s %8s %7s %8s %7s %10s %9s\n', ...
        '控制器', '|e(T)|', 'RMS(e)', 'RMS_ss', 'IAE', 't_s', ...
        'max|tau|', 'sat%', 'EE-RMS', 'DOB-err');
    fprintf('%s\n', repmat('-', 1, 120));
    B = all_results(gi, :);
    for ci = 1:nc
        m = B{ci}.m;
        fprintf('%-30s %10.3e %10.3e %10.3e %8.3e %7s %8.1f %7.1f %10.3e %9.2e\n', ...
            B{ci}.name, m.final_e, m.rms_e, m.rms_e_ss, m.iae, ...
            num2str(m.settle, '%.2f'), m.max_tau, 100*m.sat_ratio, m.ee_rms, m.dob_err);
    end
end

if cfg.save_figures
    plot_compare(cfg, all_results(1, :), fullfile(cfg.figure_dir, 'dou_yue_259_compare.png'));
end
save(cfg.result_file, 'cfg', 'P', 'CP', 'ctrl', 'cases', 'all_results', 'ref', 'regime_name');
fprintf('\n结果已保存: %s\n', cfg.result_file);

%% ========================================================================
%  单算例仿真
% ========================================================================
function r = run_one(cfg, P, CP, ctrl, ref, cs, kappa)
    n = cfg.n;  N = cfg.N;  Ts = cfg.Ts;

    q  = cfg.q_init;
    dq = cfg.dq_init;
    Nz = round(ctrl.h/Ts) + 1;
    z_hist = zeros(2*n, Nz);
    tau_cmd_prev = zeros(n, 1);
    tau_act_prev = zeros(n, 1);

    % ---- 控制器状态 ----
    Ctl = [];
    if any(strcmp(cs.kind, {'fonsm', 'fosm'}))
        if strcmp(cs.kind, 'fonsm')
            Ctl = fonsm259_init('fonsm', CP.fonsm, CP.ftdob, CP.ftasc, n, Ts, cfg.mem_len, kappa);
        else
            Ctl = fonsm259_init('fosm', CP.fosm, CP.ftdob, CP.ftasc, n, Ts, cfg.mem_len, kappa);
        end
        Ctl.dob.enabled = cs.dob;
        switch cs.asc
            case 'ftac'
                Ctl.asc = CP.ftac;  Ctl.asc.mode = 'ftac';
                Ctl.asc.eta = zeros(n,1);  Ctl.asc.Leta = 1;
            case 'ftasc'
                Ctl.asc = CP.ftasc; Ctl.asc.mode = 'ftasc';
                Ctl.asc.eta = zeros(n,1); Ctl.asc.Leta = CP.ftasc.L0;
            otherwise
                Ctl.asc = struct('mode', 'none', 'eta', zeros(n,1), 'Leta', 1);
        end
        Ctl.asc.adaptive = cs.adp;
    end
    if strcmp(cs.kind, 'pdfdob')
        dob = CP.ftdob;
        dob.enabled = cs.dob;
        dob.phat = dob.phat0;
        dob.tauhat = zeros(n, 1);
    end

    E = zeros(n, N);  Ed = zeros(n, N);  TAU = zeros(n, N);
    ETA = zeros(n, N);  TH = zeros(n, N);  DST = zeros(n, N);
    SAT = false(1, N);  PE = zeros(1, N);

    for k = 1:N-1
        tk = cfg.t(k);

        % ---------- 被控对象 ----------
        if cfg.uncertainty
            Pa = P;  Pa.mass = P.unc.mass;  Pa.I = P.unc.I;
            Sa = ffsm259_model(Pa, q, dq);
            Sn = ffsm259_model(P,  q, dq);
            Ma = Sa.Me;  Caqd = Sa.Ce_qd;   Mn = Sn.Me;  Cn = Sn.Cmat;
            pe = Sn.pe;
        else
            Sa = ffsm259_model(P, q, dq);
            Ma = Sa.Me;  Caqd = Sa.Ce_qd;   Mn = Ma;  Cn = Sa.Cmat;
            pe = Sa.pe;
        end

        % ---------- 控制器 ----------
        switch cs.kind
            case 'pd'
                e  = q - ref.qd(:,k);
                de = dq - ref.dqd(:,k);
                nu = ctrl.K0*[e; de];        % = -Kp e - Kd de  =>  ë = -Kp e - Kd ė
                tau_cmd = Cn*dq + Mn*(ref.ddqd(:,k) + nu);

            case 'pdf'
                z = [q - ref.qd(:,k); dq - ref.dqd(:,k)];
                z_hist = [z, z_hist(:,1:end-1)];
                nu = ctrl.K0*z - pdf_gain_at(ctrl.pdf, tk)*z_hist(:,end);
                tau_cmd = Cn*dq + Mn*(ref.ddqd(:,k) + nu);

            case 'pdfdob'
                z = [q - ref.qd(:,k); dq - ref.dqd(:,k)];
                z_hist = [z, z_hist(:,1:end-1)];
                [dob, tau_hat] = ftdob_step(dob, Mn, Cn, dq, tau_act_prev, Ts);
                nu = ctrl.K0*z - pdf_gain_at(ctrl.pdf, tk)*z_hist(:,end);
                tau_cmd = Cn*dq + Mn*(ref.ddqd(:,k) + nu) - tau_hat;

            otherwise
                [tau_cmd, Ctl, lg] = fonsm259_step(Ctl, q, dq, ...
                    ref.qd(:,k), ref.dqd(:,k), ref.ddqd(:,k), ...
                    Mn, Cn, tau_act_prev, tau_cmd_prev);
                ETA(:,k) = lg.eta;  TH(:,k) = lg.tau_hat;
        end

        % ---------- 饱和 ----------
        tau_act = min(max(tau_cmd, P.tau_min), P.tau_max);
        SAT(k)  = any(abs(tau_cmd) > P.tau_max);

        % ---------- 扰动 (Eq 74) ----------
        if cs.dist
            d = dou_yue_259_disturbance(tk, n);
        else
            d = zeros(n, 1);
        end
        DST(:,k) = d;

        % ---------- 记录 + 半隐式积分 ----------
        TAU(:,k) = tau_act;
        E(:,k)   = ref.qd(:,k)  - q;
        Ed(:,k)  = ref.dqd(:,k) - dq;
        PE(k)    = norm(pe - ref.pe(:,k));

        ddq = Ma \ (tau_act + d - Caqd);
        dq  = dq + Ts*ddq;
        q   = q  + Ts*dq;

        tau_cmd_prev = tau_cmd;
        tau_act_prev = tau_act;
    end

    % 末点
    S = ffsm259_model(P, q);
    E(:,end)  = ref.qd(:,end)  - q;
    Ed(:,end) = ref.dqd(:,end) - dq;
    PE(end)   = norm(S.pe - ref.pe(:,end));
    TAU(:,end) = TAU(:,max(end-1,1));

    en = vecnorm(E, 2, 1);
    m.final_e   = norm(E(:,end));
    m.final_de  = norm(Ed(:,end));
    m.rms_e     = sqrt(mean(en.^2));
    m.iae       = trapz(cfg.t, en);
    m.max_tau   = max(abs(TAU), [], 'all');
    m.sat_ratio = mean(SAT);
    m.settle    = settling_time(cfg.t, en, 1e-3);
    m.ee_rms    = sqrt(mean(PE.^2));
    m.ee_final  = PE(end);
    m.rms_eta   = sqrt(mean(sum(ETA.^2, 1)));
    ss = cfg.t >= cfg.Tref;                       % 稳态窗口 (轨迹结束之后)
    m.rms_e_ss  = sqrt(mean(en(ss).^2));
    m.e_max_ss  = max(en(ss));
    m.dob_err   = max(abs(TH(:,ss) - DST(:,ss)), [], 'all');   % 观测器估计误差 (稳态窗口)

    r = struct('name', cs.name, 'kind', cs.kind, 'asc', cs.asc, 'dob', cs.dob, ...
        'dist', cs.dist, 'kappa', kappa, 'm', m, 'E', E, 'Ed', Ed, ...
        'TAU', TAU, 'PE', PE, 'ETA', ETA, 'TH', TH, 'DST', DST, 'e_norm', en);
end

% ------------------------------------------------------------------------
function c = mk(name, kind, dob, asc, dist, adp)
    c.name = string(name);  c.kind = kind;  c.dob = dob;
    c.asc = asc;  c.dist = dist;  c.adp = adp;
end

function [qd, dqd, ddqd] = quintic_joint_trajectory(q0, qf, Tref, t)
    s    = min(t./Tref, 1);
    phi  = 10*s.^3 - 15*s.^4 + 6*s.^5;
    dphi = (30*s.^2 - 60*s.^3 + 30*s.^4)./Tref;
    ddphi= (60*s - 180*s.^2 + 120*s.^3)./Tref^2;
    dphi(t > Tref)  = 0;
    ddphi(t > Tref) = 0;
    delta = qf - q0;
    qd    = q0 + delta*phi;
    dqd   = delta*dphi;
    ddqd  = delta*ddphi;
end

function ts = settling_time(t, sig, thr)
    idx = find(sig > thr, 1, 'last');
    if isempty(idx)
        ts = 0;
    elseif idx >= numel(t)
        ts = NaN;
    else
        ts = t(idx+1);
    end
end

function plot_compare(cfg, R, fname)
    nc = numel(R);
    cols = lines(nc);
    figure('Position', [50 50 1300 760]);

    subplot(2,2,1); hold on; grid on;
    for i = 1:nc
        plot(cfg.t, max(R{i}.e_norm, 1e-14), 'Color', cols(i,:), 'LineWidth', 1.1);
    end
    set(gca, 'YScale', 'log');
    xlabel('t [s]'); ylabel('||e|| [rad]'); title('关节跟踪误差');
    legend(cellfun(@(r) char(r.name), R, 'UniformOutput', false), ...
        'FontSize', 6, 'Location', 'east');

    subplot(2,2,2); hold on; grid on;
    for i = 1:nc
        plot(cfg.t, vecnorm(R{i}.Ed, 2, 1), 'Color', cols(i,:), 'LineWidth', 1.1);
    end
    set(gca, 'YScale', 'log');
    xlabel('t [s]'); ylabel('||de|| [rad/s]'); title('速度误差');

    subplot(2,2,3); hold on; grid on;
    for i = 1:nc
        plot(cfg.t, max(abs(R{i}.TAU), [], 1), 'Color', cols(i,:), 'LineWidth', 1.1);
    end
    yline(20, 'k--');
    xlabel('t [s]'); ylabel('max|\tau_i| [Nm]'); title('控制力矩 (限幅 20 Nm)');

    subplot(2,2,4); hold on; grid on;
    for i = 1:nc
        plot(cfg.t, vecnorm(R{i}.ETA, 2, 1), 'Color', cols(i,:), 'LineWidth', 1.1);
    end
    xlabel('t [s]'); ylabel('||\eta||'); title('抗饱和辅助变量');

    saveas(gcf, fname);
    fprintf('图已保存: %s\n', fname);
end
