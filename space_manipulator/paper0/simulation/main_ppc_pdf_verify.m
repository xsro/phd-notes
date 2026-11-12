%% main_ppc_pdf_verify.m
% =========================================================================
% 论文 v2 三段时序的数值验证:  漏斗 PPC -> PTDO -> PDF 单周期零化
%
%   [0, To)        PTDO 收敛 + 漏斗型预设性能律 (定理 3.2)
%   [To, To+2h)    PDF 单周期零化, 局部时间 vartheta = t - To (定理 3.3)
%   [To+2h, Tf]    稳态 (定理 4.1 (iv))
%
% 验收指标 M1..M7 与论文第 5 节表 2 一一对应:
%   M1 包络违反时长       定理 3.2
%   M2 进带深度 vs vartheta  定理 3.2
%   M3 归零时刻 vs To+2h   定理 4.1
%   M4 阶段 II 越界比 vs M_h*B_o/rho_inf  命题 4.1
%   M5 稳态残差 vs Gamma_h*Delta_r/(1-eps_Delta)  命题 4.2
%   M6 ||Delta_K(h)|| 与 cond(W_c)   定理 3.3 / 命题 4.2
%   M7 力矩峰值与饱和占空比   注 3.2
%
% 被控对象与扰动: 文献 [259] (Dou & Yue 2025) Table 1/2 与 Eq (74)。
%
% 运行:  matlab -batch "main_ppc_pdf_verify"
% =========================================================================
clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

%% ---------------- 1. 仿真设置 ----------------
cfg.Ts          = 2e-3;      % 积分步长 [s]
cfg.Tf          = 20.0;      % 仿真时长 [s]
cfg.Tref        = 8.0;       % 参考轨迹过渡时间 [s]
cfg.n           = 7;
cfg.To          = 5.0;       % 预设观测/进带时间 T_o [s]
cfg.Tc          = 3.0;       % PTDO 内部收敛时间 (须 <= T_o)
cfg.h           = 1.0;       % 人工延迟 [s]  -> 总预设时间 T_o + 2h = 7 s
cfg.rho_inf     = 1e-3;      % 预设误差带 [rad]
cfg.eta         = 0.3;
cfg.sigma       = 0.4;
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

% PTDO 参数
oprm = struct('T_o', cfg.To, 'T_c', cfg.Tc, 'eta', cfg.eta, ...
              'sigma', cfg.sigma, 'xi0', zeros(n, 1));

fprintf('==================== 设置 ====================\n');
fprintf('对象: 文献 259 七自由度浮基机械臂, tau_max = %.0f Nm\n', P.tau_max(1));
fprintf('To = %.2f s, Tc = %.2f s (Tc<=To: %d), h = %.1f s  ->  T_pre = %.2f s\n', ...
    cfg.To, cfg.Tc, cfg.Tc <= cfg.To, cfg.h, cfg.To + 2*cfg.h);
fprintf('rho0 (max) = %.3e rad, rho_inf = %.0e rad\n', max(pp.rho0), cfg.rho_inf);
fprintf('PPC 增益 k1 = %g, k2 = %g;  推论 4.1 校验: 2*k*To = %.1f (需 >= %.2f) -> %d\n', ...
    pp.k1(1), pp.k2(1), pp.guide.have, pp.guide.need, pp.guide.ok);

%% ---------------- 3. 离线数值判据 (M6, 命题 4.1 常数) ----------------
fprintf('\n---- 离线分析: Delta_K(h), Gamma_h, M_h ----\n');
an = pdf_periodian(ctrl.pdf, 1e-3, 41);
fprintf('cond(W_c)      = %.3e\n', an.cond);
fprintf('||Delta_K(h)|| = %.3e   (相对 %.2e)\n', an.deltaK, an.deltaK_rel);
fprintf('eps_Delta      = %.3e  -> 1/(1-eps_Delta) = %.3f\n', ...
    min(an.deltaK, 1-1e-12), 1/(1-min(an.deltaK, 1-1e-12)));
fprintf('Gamma_h = %.3e, E_h = %.3e, int||Kc|| = %.3e\n', an.Gamma, an.E_h, an.intKc);
fprintf('M_h (闭式上界) = %.3e   | Mop (精确算子) = %.3e\n', an.M_h, an.Mop);
fprintf('Mpos (逐关节位置行块) = %s\n', mat2str(an.Mpos.', 3));

%% ---------------- 4. 算例 ----------------
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

%% ---------------- 5. 理论常数 (oracle w_bar) 与指标汇总 ----------------
fprintf('\n==================== 验收指标 (M1..M7) ====================\n');
for ci = 1:nc
    r = R{ci};
    th = ppc_theory(r.pp, e0, de0, r.m.w_bar);
    R{ci}.th = th;
    R{ci}.met = sim_metrics(r, th, r.pp, an, cfg);
end

hdr = sprintf('%-44s %8s %8s %9s %9s %9s %9s %8s', ...
    '算例', 'M1a', 'M1b', 'M2 ratio', 'M4 ratio', 'M5 ss', 'M3 t_1e-5', 'M7 sat%');
fprintf('%s\n%s\n', hdr, repmat('-', 1, 100));
for ci = 1:nc
    x = R{ci}.met;
    fprintf('%-44s %8.3f %8.3f %9.4f %9.4f %9.2e %9s %8.2f\n', ...
        char(R{ci}.name), x.viol_time_p1, x.viol_time_p2, x.band_ratio, ...
        x.stage2_ratio, x.ss_sup, num2str(x.t_1e5, '%.2f'), 100*R{ci}.m.sat_ratio);
end
fprintf('说明: M1a/M1b = 观测阶段/PDF 阶段的包络违反时长 [s*joint]; M5 = t>=To+2h 的 sup||z||_inf\n');
fprintf('      M3 = 首次达到 ||z||_inf<=1e-5 的时刻 (1e-9 级只在无扰动理想算例中可达)\n');

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

%% ---------------- 6. 保存与绘图 ----------------
save(cfg.result_file, 'cfg', 'P', 'ctrl', 'pp', 'oprm', 'an', 'cases', 'R', 'ref', 'e0');
fprintf('\n结果已保存: %s\n', cfg.result_file);

if cfg.save_figures
    plot_all(cfg, R, an, pp, fullfile(cfg.figure_dir, 'ppc_envelope.png'), ...
        fullfile(cfg.figure_dir, 'ppc_timeline.png'), ...
        fullfile(cfg.figure_dir, 'ppc_phase1_compare.png'));
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
function plot_all(cfg, R, an, pp, f_env, f_time, f_cmp)
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
    saveas(fig, f_env); fprintf('图已保存: %s\n', f_env);

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
    semilogy(t, max(abs(R{2}.raw.EPS2), [], 1) + 1e-16, 'LineWidth', 1.1);
    xline(To, 'k-'); xlabel('t [s]'); ylabel('||\epsilon_2||_\infty');
    title('PTDO 残差 (阶段 I 的 w)');
    saveas(fig, f_time); fprintf('图已保存: %s\n', f_time);

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
    saveas(fig, f_cmp); fprintf('图已保存: %s\n', f_cmp);
end
