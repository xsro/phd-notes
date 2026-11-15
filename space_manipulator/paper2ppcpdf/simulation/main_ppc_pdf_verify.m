%% main_ppc_pdf_verify.m
% =========================================================================
% 论文三段时序的数值验证:  漏斗 PPC -> 分段观测器 -> PDF 单周期零化
%
%   [0, To)        分段观测器阶段 I (奇异增益, 定理 thm:obs1) + 漏斗 PPC
%   [To, To+2h)    分段观测器阶段 II (超螺旋, 定理 thm:sta) + PDF 零化
%   [To+2h, Tf]    稳态 (命题 prop:approx)
%
% 观测器与力矩律严格按论文式 (eq:ptdo)、(eq:hatd)、(eq:computed_torque):
%
%   y     = M_e(q)*dq,        y_dot = u_p + d
%   u_p   = tau_act - C_e*dq + Mdot_e*dq
%   tau   = C_e*dq + M_e*(ddq_d + nu) - d_hat,      d_hat = z2
%
% 验收指标 M1..M7 与论文第 5 节表 tab:metrics 一一对应:
%   M1 包络违反时长       定理 thm:ppc
%   M2 进带深度 vs vartheta  定理 thm:ppc
%   (M3 归零时刻: 采样实现的残差地板为 O(Ts), 达不到 1e-9, 不列表)
%   M4 阶段 II 越界比 vs M_h*B_o/rho_inf  命题 prop:stage2
%   M5 稳态残差 vs Gamma_h*Delta_r/(1-eps_Delta)  命题 prop:approx
%   M6 ||Delta_K(h)|| 与 cond(W_c)   定理 thm:pdf1 / 命题 prop:approx
%   M7 力矩峰值与饱和占空比   注 rem:barrier
%
% 被控对象与扰动: 文献 dou2025fractional 的 Table 1/2 与 Eq (74)。
%
% 运行:  matlab -batch "main_ppc_pdf_verify"
% =========================================================================
clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

%% ---------------- 1. 仿真设置 ----------------
cfg.Ts          = 2e-3;      % 采样步长 [s]
cfg.Tf          = 20.0;      % 仿真时长 [s]
cfg.Tref        = 8.0;       % 参考轨迹过渡时间 [s]
cfg.n           = 7;
cfg.To          = 5.0;       % 预设观测/进带时间 T_o [s]
cfg.h           = 1.0;       % 人工延迟 [s]  -> 总预设时间 T_o + 2h = 7 s
cfg.rho_inf     = 1e-3;      % 预设误差带 [rad]
cfg.save_figures = true;
cfg.show_figures = false;

cfg.t = 0:cfg.Ts:cfg.Tf;
cfg.N = numel(cfg.t);
cfg.figure_dir  = fullfile(script_dir, '..', 'figures');
cfg.result_file = fullfile(script_dir, 'results_ppc_pdf.mat');

if cfg.save_figures && ~exist(cfg.figure_dir, 'dir'), mkdir(cfg.figure_dir); end
if ~cfg.show_figures, set(0, 'DefaultFigureVisible', 'off'); end

%% ---------------- 2. 参数、参考轨迹、基准增益 ----------------
P = dou_yue_259_parameters();
n = P.n;

q_ref0  = P.q0;
q_ref_f = P.q0 + deg2rad([20; -15; 15; -20; 15; -15; 10]);
[ref.qd, ref.dqd, ref.ddqd] = quintic_joint_trajectory(q_ref0, q_ref_f, cfg.Tref, cfg.t);
ref.pe = zeros(3, cfg.N);
for k = 1:cfg.N
    ref.pe(:,k) = ffsm259_model(P, ref.qd(:,k)).pe;
end

cfg.q_init  = P.q0 + deg2rad([0.5; -0.4; 0.4; -0.5; 0.4; -0.3; 0.2]);
cfg.dq_init = P.dq0;
e0  = cfg.q_init - ref.qd(:,1);
de0 = cfg.dq_init - ref.dqd(:,1);

% 基准增益 (与既有仿真一致)
Kp = diag([30, 30, 30, 25, 25, 20, 8]);
Kd = diag([12, 12, 12, 10, 10,  9, 4]);
ctrl.K0  = [-Kp, -Kd];
ctrl.h   = cfg.h;
ctrl.pdf = build_pdf_gain(ctrl.K0, n, ctrl.h);

% 漏斗型 PPC 参数
pp = ppc_settings(cfg.To, e0, struct('rho_inf', cfg.rho_inf, 'm', 5, 'k1', 0.5, 'k2', 1.0));

%% ---------------- 3. 观测器增益: 由 (C3)(C4) 离线核验 ----------------
% (C3) 的 L_i = sup|d_dot_i|, H_i = sup|d_ddot_i| 由扰动解析式 (Eq 74) 在
% 仿真区间上取上界; 实现只用到 L_i (选增益), 不需要在线知道它。
[~, ddot_ref, dddot_ref] = dou_yue_259_disturbance(cfg.t, n);
L_i = max(abs(ddot_ref), [], 2);
H_i = max(abs(dddot_ref), [], 2);
kappa2 = 2*L_i;                 % 论文建议取值 kappa2 = 2L_i
kappa1 = 4*sqrt(L_i);           % 论文建议取值 kappa1 = 4*sqrt(L_i)
c4_ok  = (kappa2 > L_i) & (kappa1.^2 > 4*L_i.*(kappa2 + L_i)./(kappa2 - L_i));

oprm = struct('To', cfg.To, 'kappa1', kappa1, 'kappa2', kappa2, 'nsub2', 5);
% 交接整形 (第 5.x 节的单调收敛规则): 阶段 I 末段对速度误差加阻尼,
% nu <- nu - kv*de,  t in [To-delta, To); 使 e_i(To)*de_i(To) <= 0 且
% |de_i(To)| -> 0, 从而 PDF 阶段的位置误差不再先增后减。
dl = min(0.4, 0.15*cfg.To);        % 规则 R1: delta = min(0.4 s, 0.15 To)
oprm.damp = [8/dl, dl];            % k_v = 8/delta

fprintf('==================== 设置 ====================\n');
fprintf('对象: 文献 259 七自由度浮基机械臂, tau_max = %.0f Nm\n', P.tau_max(1));
fprintf('To = %.2f s, h = %.1f s  ->  T_pre = %.2f s\n', cfg.To, cfg.h, cfg.To + 2*cfg.h);
fprintf('rho0 (max) = %.3e rad, rho_inf = %.0e rad\n', max(pp.rho0), cfg.rho_inf);
fprintf('PPC 增益 k1 = %g, k2 = %g;  推论 cor:exponential 校验: 2*k*To = %.1f (需 >= %.2f) -> %d\n', ...
    pp.k1(1), pp.k2(1), pp.guide.have, pp.guide.need, pp.guide.ok);
fprintf('(C3): L_i = %s [Nm/s], H_i = %s [Nm/s^2]\n', ...
    mat2str(L_i.', 3), mat2str(H_i.', 3));
fprintf('(C4): kappa1 = %s, kappa2 = %s\n', ...
    mat2str(kappa1.', 3), mat2str(kappa2.', 4));
fprintf('      kappa1^2 - 4L(kappa2+L)/(kappa2-L) = %s -> 全部满足: %d\n', ...
    mat2str((kappa1.^2 - 4*L_i.*(kappa2 + L_i)./(kappa2 - L_i)).', 3), all(c4_ok));

%% ---------------- 4. 离线数值判据 (M6, 命题 prop:stage2 常数) ----------------
fprintf('\n---- 离线分析: Delta_K(h), Gamma_h, M_h ----\n');
an = pdf_periodian(ctrl.pdf, 1e-3, 41);
fprintf('cond(W_c)      = %.3e\n', an.cond);
fprintf('||Delta_K(h)|| = %.3e   (相对 %.2e)\n', an.deltaK, an.deltaK_rel);
fprintf('eps_Delta      = %.3e  -> 1/(1-eps_Delta) = %.3f\n', ...
    min(an.deltaK, 1-1e-12), 1/(1-min(an.deltaK, 1-1e-12)));
fprintf('Gamma_h = %.3e, E_h = %.3e, int||Kc|| = %.3e\n', an.Gamma, an.E_h, an.intKc);
fprintf('M_h (闭式上界) = %.3e   | Mop (精确算子) = %.3e\n', an.M_h, an.Mop);
fprintf('Mpos (逐关节位置行块) = %s\n', mat2str(an.Mpos.', 3));

%% ---------------- 5. 算例 ----------------
cases = struct('name', {}, 'kind', {}, 'dist', {}, 'rho_inf', {});
cases(end+1) = mk("PD  + PTDO + PDF,  rho_inf=1e-3 (消融 A)", 'pd',  true,  1e-3);
cases(end+1) = mk("PPC + PTDO + PDF,  rho_inf=1e-3 (本文)",   'ppc', true,  1e-3);
cases(end+1) = mk("PPC + PTDO + PDF,  rho_inf=2e-3",          'ppc', true,  2e-3);
cases(end+1) = mk("PPC + PTDO + PDF,  rho_inf=1e-3 (无扰动)", 'ppc', false, 1e-3);
nc = numel(cases);

R = cell(1, nc);
fprintf('\n==================== 仿真 ====================\n');
for ci = 1:nc
    ppi = ppc_settings(cfg.To, e0, struct('rho_inf', cases(ci).rho_inf, ...
                                          'm', 5, 'k1', 0.5, 'k2', 1.0));
    fprintf('[%d/%d] %-44s ', ci, nc, cases(ci).name);
    tic;
    R{ci} = sim_ppc_case(cfg, P, ctrl, ref, cases(ci), ppi, oprm);
    fprintf('耗时 %5.1f s | |e(T)|=%.2e sat=%4.1f%%  (k*To=%.1f vs 需 %.2f)\n', ...
        toc, R{ci}.raw.e_end, 100*R{ci}.m.sat_ratio, ppi.guide.have, ppi.guide.need);
end

%% ---------------- 6. 理论常数 (oracle w_bar) 与指标汇总 ----------------
fprintf('\n==================== 验收指标 (M1..M7) ====================\n');
for ci = 1:nc
    r = R{ci};
    th = ppc_theory(r.pp, e0, de0, r.m.w_bar);
    R{ci}.th = th;
    R{ci}.met = sim_metrics(r, th, r.pp, an, cfg);
end

hdr = sprintf('%-44s %8s %8s %9s %9s %9s %8s %8s', ...
    '算例', 'M1a', 'M1b', 'M2 ratio', 'M4 ratio', 'M5 ss', 'M7 sat%', 'M8 norm');
fprintf('%s\n%s\n', hdr, repmat('-', 1, 100));
for ci = 1:nc
    x = R{ci}.met;
    fprintf('%-44s %8.3f %8.3f %9.4f %9.4f %9.2e %8.2f %8.2e\n', ...
        char(R{ci}.name), x.viol_time_p1, x.viol_time_p2, x.band_ratio, ...
        x.stage2_ratio, x.ss_sup, 100*R{ci}.m.sat_ratio, R{ci}.met.mono_norm);
end
fprintf('说明: M1a/M1b = 观测阶段/PDF 阶段的包络违反时长 [s*joint]; M5 = t>=To+2h 的 sup||z||_inf\n');
fprintf('      M8 = PDF 阶段 ||e||_inf 的相对超调 (0 = 范数单调)\n');
fprintf('      归零精度由离散残差决定 (O(Ts)), 见论文第 5.4 节 (5) 的 Ts 扫描\n');

% ---- 观测器诊断 ----
fprintf('\n---- 观测器诊断 (理论: 定理 thm:obs1 / thm:sta) ----\n');
fprintf('%-44s %14s %14s %14s\n', '算例', 'sup|w| ph-I', '|d-dhat| ph-II', '|d-dhat| ss');
for ci = 1:nc
    m = R{ci}.m;
    fprintf('%-44s %14.3e %14.3e %14.3e\n', char(R{ci}.name), ...
        m.w_bar, m.d_hat_err_max, m.d_hat_err_ss);
end

% ---- 逐关节 M2 对照 (理论 vartheta_i vs 实测) ----
fprintf('\n---- M2: 进带深度 (To = %.2f s) ----\n', cfg.To);
fprintf('%-5s %12s %11s %11s %9s\n', 'joint', '|e_i(To)|', 'vartheta_i', '实测/rho_inf', 'rho_inf');
for i = 1:n
    fprintf('%-5d %12.3e %11.4f %11.4f %9.1e\n', i, ...
        R{2}.met.e_at_To(i), R{2}.th.vartheta(i), ...
        R{2}.met.e_at_To(i)/R{2}.pp.rho_inf(i), R{2}.pp.rho_inf(i));
end
fprintf('(rho_inf=1e-3 的 PPC 算例) w_bar(oracle) = %.3e, B_o = %.3e, V^max = %.3e\n', ...
    R{2}.m.w_bar, R{2}.th.B_o, R{2}.th.Vmax(1));
fprintf('判据 max_i Mpos_i*B_o/rho_inf = %.3f (需 <= 1)\n', R{2}.met.stage2_theory);

% ---- M4 / M5 明细 ----
fprintf('\n---- M4/M5 对照 ----\n');
for ci = 1:nc
    fprintf('%-44s M4: 实测 %.3f | 判据 %.3f | 闭式上界 %.1f || M5: %.2e vs %.2e\n', ...
        char(R{ci}.name), R{ci}.met.stage2_ratio, R{ci}.met.stage2_theory, ...
        R{ci}.met.stage2_theory_gr, R{ci}.met.ss_sup, R{ci}.met.ss_bound);
end

%% ---------------- 7. 保存与绘图 ----------------
save(cfg.result_file, 'cfg', 'P', 'ctrl', 'pp', 'oprm', 'an', 'cases', 'R', 'ref', 'e0', 'L_i', 'H_i');
fprintf('\n结果已保存: %s\n', cfg.result_file);

if cfg.save_figures
    plot_all(cfg, R, an, pp, fullfile(cfg.figure_dir, 'ppc_envelope.pdf'), ...
        fullfile(cfg.figure_dir, 'ppc_timeline.pdf'), ...
        fullfile(cfg.figure_dir, 'ppc_phase1_compare.pdf'), ...
        fullfile(cfg.figure_dir, 'ppc_observer.pdf'));
end
function c = mk(name, kind, dist, rho_inf)
    c = struct('name', string(name), 'kind', kind, 'dist', dist, 'rho_inf', rho_inf);
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

% ------------------------------------------------------------------------
function plot_all(cfg, R, an, pp, f_env, f_time, f_cmp, f_obs)
    t = cfg.t;  To = cfg.To;  h = cfg.h;

    % ---- 图 1: 七关节包络 ----
    fig = figure('Position', [40 40 1250 980], 'Color', 'w');
    set(fig, 'DefaultAxesFontSize', 9, 'DefaultTextFontSize', 9);
    tl = tiledlayout(fig, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:cfg.n
        nexttile(tl); hold on; grid on;
        plot(t, R{2}.raw.E(i,:), 'b-', 'LineWidth', 0.9);
        plot(t, R{2}.raw.RHO(i,:), 'r--', 'LineWidth', 0.8);
        plot(t, -R{2}.raw.RHO(i,:), 'r--', 'LineWidth', 0.8);
        yline(R{2}.pp.rho_inf(i), 'k:', 'LineWidth', 0.8);
        yline(-R{2}.pp.rho_inf(i), 'k:', 'LineWidth', 0.8);
        xline(To, 'k-', 'LineWidth', 1.0);
        xline(To + 2*h, 'k-.', 'LineWidth', 1.0);
        ylabel(sprintf('e_%d [rad]', i)); xlim([0 cfg.Tf]);
        if i == 1, title('关节误差与预设包络 (蓝: e_i, 红虚: \pm\rho_i, 黑点: \pm\rho_{i\infty})'); end
        if i >= cfg.n-1, xlabel('t [s]'); end
    end
    nexttile(tl); hold on; grid on;
    semilogy(t, max(abs(R{2}.raw.E), [], 1), 'b-', 'LineWidth', 1.0);
    semilogy(t, max(R{2}.raw.RHO, [], 1), 'r--', 'LineWidth', 0.9);
    xline(To, 'k-'); xline(To + 2*h, 'k-.');
    xlabel('t [s]'); ylabel('max_i|e_i|'); title('误差包络总览'); xlim([0 cfg.Tf]);
    export_pdf(fig, f_env);

    % ---- 图 2: 时间轴指标 ----
    fig = figure('Position', [40 40 1250 850], 'Color', 'w');
    set(fig, 'DefaultAxesFontSize', 9, 'DefaultTextFontSize', 9);
    tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile(tl); hold on; grid on;
    for ci = 1:numel(R)
        semilogy(t, max(R{ci}.raw.ZN, 1e-16), 'LineWidth', 1.1);
    end
    xline(To, 'k-'); xline(To + 2*h, 'k-.');
    yline(1e-9, 'k:');
    xlabel('t [s]'); ylabel('||z||_\infty');
    legend(cellfun(@(r) char(r.name), R, 'UniformOutput', false), 'Location', 'east');
    title('误差状态范数 (竖线: T_o, T_o+2h)');

    nexttile(tl); hold on; grid on;
    for ci = 1:numel(R)
        plot(t, max(abs(R{ci}.raw.E)./R{ci}.raw.RHO, [], 1), 'LineWidth', 1.1);
    end
    yline(1, 'r--');
    xline(To, 'k-'); xline(To + 2*h, 'k-.');
    xlabel('t [s]'); ylabel('max_i |e_i|/\rho_i');
    title('归一化包络比 (须 < 1)');
    legend(cellfun(@(r) char(r.name), R, 'UniformOutput', false), 'Location', 'east');

    nexttile(tl); hold on; grid on;
    for ci = 1:numel(R)
        semilogy(t, max(abs(R{ci}.raw.EPS2), [], 1) + 1e-16, 'LineWidth', 1.0);
    end
    set(gca, 'YScale', 'log'); ylim([1e-8, 1]);
    xline(To, 'k-'); xline(To + 2*h, 'k-.'); xlabel('t [s]'); ylabel('||w||_\infty [rad/s^2]');
    title('观测残差 w = M_e^{-1}(d-d_{est})（对数; 阶段 I 的非匹配输入）');
    legend(cellfun(@(r) char(r.name), R, 'UniformOutput', false), 'Location', 'east');
    export_pdf(fig, f_time);

    % ---- 图 3: PD vs PPC 阶段 I ----
    fig = figure('Position', [40 40 1100 460], 'Color', 'w');
    set(fig, 'DefaultAxesFontSize', 10, 'DefaultTextFontSize', 10);
    hold on; grid on;
    for ci = 1:numel(R)
        semilogy(t, max(abs(R{ci}.raw.E), [], 1), 'LineWidth', 1.2);
    end
    xline(To, 'k--'); xline(To + 2*h, 'k-.');
    xlim([0 min(cfg.Tf, To + 2*h + 3)]);
    xlabel('t [s]'); ylabel('max_i |e_i| [rad]');
    legend(cellfun(@(r) char(r.name), R, 'UniformOutput', false), 'Location', 'northeast');
    title('阶段 I 对照: 纯基准反馈 vs 漏斗型 PPC (对观测峰值免疫)');
    export_pdf(fig, f_cmp);

    % ---- 图 4: 观测器: d 与 d_hat ----
    fig = figure('Position', [40 40 1250 720], 'Color', 'w');
    set(fig, 'DefaultAxesFontSize', 9, 'DefaultTextFontSize', 9);
    tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    idxs = [1 4 5];
    for j = 1:numel(idxs)
        i = idxs(j);
        nexttile(tl); hold on; grid on;
        plot(t, R{2}.raw.DDA(i,:), 'k--', 'LineWidth', 1.1);
        plot(t, R{2}.raw.DAH(i,:), 'b-',  'LineWidth', 0.9);
        xline(To, 'k-'); xline(To + 2*h, 'k-.');
        ylabel(sprintf('d_%d, d_{est,%d} [Nm]', i, i)); xlim([0 cfg.Tf]);
        if j == 1, title('扰动真值 (黑虚) 与观测值 (蓝)'); end
        if j >= 2, xlabel('t [s]'); end
    end
    nexttile(tl); hold on; grid on;
    semilogy(t, max(abs(R{2}.raw.DDA - R{2}.raw.DAH), [], 1) + 1e-16, 'm-', 'LineWidth', 1.0);
    set(gca, 'YScale', 'log');
    ylim([1e-8, 0.3]);
    xline(To, 'k-'); xline(To + 2*h, 'k-.');
    yline(1e-4, 'k:', 'LineWidth', 0.8);
    xlabel('t [s]'); ylabel('max_i |d_i-d_{est,i}| [Nm]'); xlim([0 cfg.Tf]);
    title('观测误差 (对数)');
    export_pdf(fig, f_obs);
end
