%% main_ppc_to_sweep.m
% =========================================================================
% 预设观测/进带时间 T_o 的扫描: 定量刻画 (C2)(C4) 与执行器约束之间的折中
%
% 观测器为论文式 (eq:ptdo) 的分段结构, 其唯一的"时间"设计量就是 T_o:
%   - T_o 越小, 性能函数 rho 收缩越快 (|rho_dot| ~ 1/T_o), 力矩需求越大;
%   - T_o 越大, 阶段 I 的残差 e2 ~ d_dot*s_min 与积分误差累积越久,
%     进带前留给 PDF 的"干净"时间越少 (总预设时间 T_o+2h 也随之增大);
%   - (C4) 的 kappa1,kappa2 只由 L_i 决定, 与 T_o 无关。
%
% 对每个 T_o, 分别运行
%   (A) PPC + PTDO + PDF  (本文)
%   (B) PD  + PTDO + PDF  (消融, 阶段 I 仅用基准反馈)
% 并记录: 阶段 I 残差峰值 sup|w|, 阶段 I 饱和占空比, 稳态残差 sup||z||_inf,
%         终端误差 |e(T_f)|, 包络违反时长 M1a/M1b, 进带深度 M2, PDF 越界比 M4.
%
% 输出: 终端表格 + figures/ppc_to_sweep.png + results_ppc_to_sweep.mat
% 运行: matlab -batch "main_ppc_to_sweep"   (约 2 min)
% =========================================================================
clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

%% ---------------- 1. 与主验证脚本一致的设置 ----------------
cfg.Ts   = 2e-3;   cfg.Tf = 20.0;   cfg.Tref = 8.0;   cfg.n = 7;
cfg.To   = 5.0;    cfg.h  = 1.0;
% 包络取与观测器瞬态相容的宽包络 (第 5.6 节): rho0 = 2|e0|, rho_inf = 2e-3
cfg.rho_inf  = 2e-3;
cfg.save_figures = true;
cfg.t = 0:cfg.Ts:cfg.Tf;   cfg.N = numel(cfg.t);
cfg.figure_dir  = fullfile(script_dir, '..', 'figures');
cfg.result_file = fullfile(script_dir, 'results_ppc_to_sweep.mat');
if cfg.save_figures && ~exist(cfg.figure_dir, 'dir'), mkdir(cfg.figure_dir); end
set(0, 'DefaultFigureVisible', 'off');

P = dou_yue_259_parameters();   n = P.n;

q_ref0  = P.q0;
q_ref_f = P.q0 + deg2rad([20; -15; 15; -20; 15; -15; 10]);
[ref.qd, ref.dqd, ref.ddqd] = quintic_joint_trajectory(q_ref0, q_ref_f, cfg.Tref, cfg.t);
ref.pe = zeros(3, cfg.N);
for k = 1:cfg.N, ref.pe(:,k) = ffsm259_model(P, ref.qd(:,k)).pe; end

cfg.q_init  = P.q0 + deg2rad([0.5; -0.4; 0.4; -0.5; 0.4; -0.3; 0.2]);
cfg.dq_init = P.dq0;
e0  = cfg.q_init - ref.qd(:,1);
de0 = cfg.dq_init - ref.dqd(:,1);

Kp = diag([30, 30, 30, 25, 25, 20, 8]);
Kd = diag([12, 12, 12, 10, 10,  9, 4]);
ctrl.K0  = [-Kp, -Kd];
ctrl.h   = cfg.h;
ctrl.pdf = build_pdf_gain(ctrl.K0, n, ctrl.h);

% 观测器增益 (由 (C3)(C4) 离线确定, 与 T_o 无关)
[~, ddot_ref] = dou_yue_259_disturbance(cfg.t, n);
L_i = max(abs(ddot_ref), [], 2);
kappa2 = 2*L_i;  kappa1 = 4*sqrt(L_i);

fprintf('==================== T_o 扫描 ====================\n');
fprintf('h = %.1f s, tau_max = %.0f Nm, rho_inf = %.0e, rho0 = 2|e0|\n', ...
    cfg.h, P.tau_max(1), cfg.rho_inf);
an = pdf_periodian(ctrl.pdf, 1e-3, 41);
fprintf('(离线判据不变: ||Delta_K(h)|| = %.2e, Gamma_h = %.3f, M_h^ap = %.2f)\n\n', ...
    an.deltaK, an.Gamma, an.M_h);

%% ---------------- 2. 扫描 ----------------
To_list = [1.0 1.5 2.0 3.0 4.0 5.0 6.0 8.0];
kinds  = {'ppc', 'pd'};
kname  = {'PPC+PTDO+PDF', 'PD+PTDO+PDF '};
nr = numel(To_list);  nk = numel(kinds);
RES = cell(nk, nr);

fprintf('%-6s %-12s %10s %8s %8s %10s %10s %8s %8s %8s\n', ...
    'To', '控制器', 'peak|w|', 'sat1%', 'sat%', 'ss||z||inf', '|e(Tf)|', 'M1a', 'M2', 'M4');
fprintf('%s\n', repmat('-', 1, 108));
for ki = 1:nk
    for ri = 1:nr
        To = To_list(ri);
        cfgi = cfg;  cfgi.To = To;
        pp = ppc_settings(To, e0, struct('rho_inf', cfg.rho_inf, 'm', 5, 'k1', 0.5, 'k2', 1.0));
        oprm = struct('To', To, 'kappa1', kappa1, 'kappa2', kappa2, 'nsub2', 5);
        cs = struct('name', sprintf('%s, To=%.1f', kname{ki}, To), ...
                    'kind', kinds{ki}, 'dist', true, 'rho_inf', cfg.rho_inf);
        x = sim_ppc_case(cfgi, P, ctrl, ref, cs, pp, oprm);
        th = ppc_theory(pp, e0, de0, x.m.w_bar);
        x.th = th;
        x.met = sim_metrics(x, th, pp, an, cfgi);
        RES{ki, ri} = x;
        fprintf('%-6.1f %-12s %10.3e %8.2f %8.2f %10.3e %10.3e %8.3f %8.3f %8.3f\n', ...
            To, kname{ki}, x.m.w_bar, 100*x.m.sat_ratio_p1, 100*x.m.sat_ratio, ...
            x.met.ss_sup, x.m.e_end, x.met.viol_time_p1, x.met.band_ratio, x.met.stage2_ratio);
    end
end

save(cfg.result_file, 'cfg', 'P', 'ctrl', 'an', 'To_list', 'kinds', 'kname', 'RES', ...
     'ref', 'e0', 'L_i', 'kappa1', 'kappa2');
fprintf('\n结果已保存: %s\n', cfg.result_file);

%% ---------------- 3. 绘图 ----------------
if cfg.save_figures
    f = fullfile(cfg.figure_dir, 'ppc_to_sweep.pdf');
    plot_sweep(cfg, RES, To_list, kname, f);
end

%% ========================================================================
function plot_sweep(cfg, RES, ratios, kname, fname)
    nk = size(RES, 1);  nr = numel(ratios);
    cols = [0.85 0.33 0.10; 0.00 0.45 0.74];      % PPC / PD
    sat1 = zeros(nk, nr);  sat = zeros(nk, nr);  peak = zeros(nk, nr);
    ss   = zeros(nk, nr);  ein = zeros(nk, nr);  m1a = zeros(nk, nr);
    m2   = zeros(nk, nr);  m4  = zeros(nk, nr);
    for ki = 1:nk
        for ri = 1:nr
            x = RES{ki, ri};
            sat1(ki,ri) = 100*x.m.sat_ratio_p1;
            sat(ki,ri)  = 100*x.m.sat_ratio;
            peak(ki,ri) = x.m.w_bar;
            ss(ki,ri)   = x.met.ss_sup;
            ein(ki,ri)  = x.m.e_end;
            m1a(ki,ri)  = x.met.viol_time_p1;
            m2(ki,ri)   = x.met.band_ratio;
            m4(ki,ri)   = x.met.stage2_ratio;
        end
    end

    fig = figure('Position', [40 40 1180 880], 'Color', 'w');
    set(fig, 'DefaultAxesFontSize', 10, 'DefaultTextFontSize', 10);
    tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ---- (a) 阶段 I 残差 + 饱和 ----
    ax1 = nexttile(tl); hold on; grid on;
    yyaxis(ax1, 'left');
    for ki = 1:nk
        semilogy(ax1, ratios, peak(ki,:), 'o-', 'Color', cols(ki,:), 'LineWidth', 1.4, ...
            'MarkerFaceColor', cols(ki,:), 'MarkerSize', 5);
    end
    set(ax1, 'YScale', 'log'); ylabel(ax1, 'sup||w||_\infty  [rad/s^2]');
    yyaxis(ax1, 'right');
    for ki = 1:nk
        plot(ax1, ratios, sat1(ki,:), 's--', 'Color', cols(ki,:), 'LineWidth', 1.1, 'MarkerSize', 5);
    end
    ylabel(ax1, '阶段 I 饱和占空比 [%]');
    xlabel('T_o [s]');
    title(sprintf('(a) 阶段 I 观测残差峰值与饱和（h = %.1f s）', cfg.h));
    legend(ax1, {sprintf('%s: 峰值', kname{1}), sprintf('%s: 峰值', kname{2}), ...
                 sprintf('%s: 饱和', kname{1}), sprintf('%s: 饱和', kname{2})}, ...
           'Location', 'east', 'FontSize', 8);

    % ---- (b) 稳态残差与终端误差 ----
    ax2 = nexttile(tl); hold on; grid on; box on;
    for ki = 1:nk
        semilogy(ax2, ratios, ss(ki,:), 'o-', 'Color', cols(ki,:), 'LineWidth', 1.4, ...
            'MarkerFaceColor', cols(ki,:), 'MarkerSize', 5);
        semilogy(ax2, ratios, ein(ki,:), 'd--', 'Color', cols(ki,:), 'LineWidth', 1.0, 'MarkerSize', 5);
    end
    set(ax2, 'YScale', 'log');
    xlabel('T_o [s]'); ylabel('||e|| [rad]');
    title('(b) 稳态残差 sup||z||_\infty（实线）与终端误差 |e(T_f)|（虚线）');
    legend(ax2, {[kname{1} ' ss'], [kname{2} ' ss'], [kname{1} ' |e(T_f)|'], [kname{2} ' |e(T_f)|']}, ...
           'Location', 'east', 'FontSize', 8);

    % ---- (c) 包络指标 ----
    ax3 = nexttile(tl); hold on;
    yyaxis(ax3, 'left');
    plot(ax3, ratios, m1a(1,:), 'o-', 'Color', cols(1,:), 'LineWidth', 1.4, ...
        'MarkerFaceColor', cols(1,:), 'MarkerSize', 5);
    plot(ax3, ratios, m1a(2,:), 's--', 'Color', cols(2,:), 'LineWidth', 1.2, 'MarkerSize', 6);
    ylabel(ax3, 'M1a 包络违反时长 [s\cdotjoint]');
    yyaxis(ax3, 'right');
    semilogy(ax3, ratios, m2(1,:), 'd-', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.2, 'MarkerSize', 6);
    semilogy(ax3, ratios, m4(1,:), '^-', 'Color', [0.5 0.2 0.6], 'LineWidth', 1.1, 'MarkerSize', 5);
    set(ax3, 'YScale', 'log'); ylabel(ax3, 'M2 / M4（相对 \rho_\infty）');
    xlabel('T_o [s]');
    title('(c) 阶段 I 包络违反时长（左）与进带深度 M2 / PDF 越界比 M4（右）');
    yyaxis(ax3, 'left'); ylim([0, 12]);
    legend(ax3, {[kname{1} ' M1a'], [kname{2} ' M1a'], 'M2（PPC）', 'M4（PPC）'}, ...
           'Location', 'east', 'FontSize', 8);

    for ax = [ax1 ax2 ax3]
        set(ax, 'XTick', ratios);
        xlim(ax, [min(ratios)*0.9, max(ratios)*1.1]);
    end
    export_pdf(fig, fname);
end

% ------------------------------------------------------------------------
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
