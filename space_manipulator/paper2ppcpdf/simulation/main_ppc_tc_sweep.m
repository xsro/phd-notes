%% main_ppc_tc_sweep.m
% =========================================================================
% 预设观测时间 T_c 的扫描: 定量刻画式 (42) 的"三角约束"
%
%   T_c <= T_o (理论前提, 假设 2.6)
%   pi/(eta*T_c), pi/(sigma*eta*T_c) 为观测器时变增益 (抖振来源)
%   ||tau|| <= tau_max (执行器约束)
%
% 对每个 T_c/T_o 比值, 分别运行
%   (A) PPC + PTDO + PDF  (本文)
%   (B) PD  + PTDO + PDF  (消融, 阶段 I 仅用基准反馈)
% 并记录: 观测器峰值 ||eps_2||_inf, 阶段 I 饱和占空比, 稳态残差 sup||z||_inf,
%         终端误差 |e(T_f)|, 包络违反时长 M1a/M1b, 进带深度 M2.
%
% 输出: 终端表格 + figures/ppc_tc_sweep.png + results_ppc_tc_sweep.mat
% 运行: matlab -batch "main_ppc_tc_sweep"   (约 2 min)
% =========================================================================
clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

%% ---------------- 1. 与主验证脚本一致的设置 ----------------
cfg.Ts   = 2e-3;   cfg.Tf = 20.0;   cfg.Tref = 8.0;   cfg.n = 7;
cfg.To   = 5.0;    cfg.h  = 1.0;
% ---- 性能包络设计 (两套, 由 cfg.env 选择) ----
cfg.env      = 'feasible';    % 'tight' : rho0 = 2|e0|, rho_inf = 1e-3 (与第 5.4 节一致)
                           % 'feasible': rho0 = 6e-2, rho_inf = 5e-3 (与观测器瞬态相容)
cfg.rho_inf  = 5e-3;
cfg.margin   = 1.0;
cfg.rho0_min = 6e-2;
cfg.eta  = 0.3;    cfg.sigma = 0.4;
cfg.save_figures = true;
cfg.t = 0:cfg.Ts:cfg.Tf;   cfg.N = numel(cfg.t);
cfg.figure_dir  = fullfile(script_dir, '..', 'figures');
cfg.result_file = fullfile(script_dir, sprintf('results_ppc_tc_sweep_%s.mat', cfg.env));
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
pp = ppc_settings(cfg.To, e0, struct('rho_inf', cfg.rho_inf, 'm', 5, 'k1', 0.5, 'k2', 1.0, ...
                                     'margin', cfg.margin, 'rho0_min', cfg.rho0_min));

fprintf('==================== T_c 扫描 ====================\n');
fprintf('T_o = %.1f s, h = %.1f s, tau_max = %.0f Nm, 包络方案 = %s (rho0=max(%.0e|e0|,%.0e), rho_inf=%.0e)\n', ...
    cfg.To, cfg.h, P.tau_max(1), cfg.env, cfg.margin, cfg.rho0_min, cfg.rho_inf);
an = pdf_periodian(ctrl.pdf, 1e-3, 41);
fprintf('(离线判据不变: ||Delta_K(h)|| = %.2e, Gamma_h = %.3f, M_h^ap = %.2f)\n\n', ...
    an.deltaK, an.Gamma, an.M_h);

%% ---------------- 2. 扫描 ----------------
ratios = [0.2 0.3 0.45 0.6 0.8 1.0 1.5 2.0 3.0 5.0];
kinds  = {'ppc', 'pd'};
kname  = {'PPC+PTDO+PDF', 'PD+PTDO+PDF '};
nr = numel(ratios);  nk = numel(kinds);
RES = cell(nk, nr);

fprintf('%-6s %-13s %10s %8s %8s %10s %10s %8s %8s %8s\n', ...
    'Tc/To', '控制器', 'peak|e2|', 'sat1%', 'sat%', 'ss||z||inf', '|e(Tf)|', 'M1a', 'M1b', 'M2');
fprintf('%s\n', repmat('-', 1, 104));
for ki = 1:nk
    for ri = 1:nr
        r  = ratios(ri);
        oprm = struct('T_o', cfg.To, 'T_c', r*cfg.To, 'eta', cfg.eta, ...
                      'sigma', cfg.sigma, 'xi0', zeros(n, 1));
        cs = struct('name', sprintf('%s, Tc/To=%.2f', kname{ki}, r), ...
                    'kind', kinds{ki}, 'dist', true, 'rho_inf', cfg.rho_inf);
        x = sim_ppc_case(cfg, P, ctrl, ref, cs, pp, oprm);
        th = ppc_theory(pp, e0, de0, x.m.w_bar);
        x.th = th;
        x.met = sim_metrics(x, th, pp, an, cfg);
        RES{ki, ri} = x;
        fprintf('%-6.2f %-13s %10.3e %8.2f %8.2f %10.3e %10.3e %8.3f %8.3f %8.4f\n', ...
            r, kname{ki}, x.m.w_bar, 100*x.m.sat_ratio_p1, 100*x.m.sat_ratio, ...
            x.met.ss_sup, x.m.e_end, x.met.viol_time_p1, x.met.viol_time_p2, x.met.band_ratio);
    end
end

save(cfg.result_file, 'cfg', 'P', 'ctrl', 'pp', 'an', 'ratios', 'kinds', 'kname', 'RES', 'ref', 'e0');
fprintf('\n结果已保存: %s\n', cfg.result_file);

%% ---------------- 3. 绘图 ----------------
if cfg.save_figures
    f = fullfile(cfg.figure_dir, sprintf('ppc_tc_sweep_%s.png', cfg.env));
    plot_sweep(cfg, RES, ratios, kname, f);
    fprintf('图已保存: %s\n', f);
end

%% ========================================================================
function plot_sweep(cfg, RES, ratios, kname, fname)
    nk = size(RES, 1);  nr = numel(ratios);
    cols = [0.85 0.33 0.10; 0.00 0.45 0.74];      % PPC / PD
    sat1 = zeros(nk, nr);  sat = zeros(nk, nr);  peak = zeros(nk, nr);
    ss   = zeros(nk, nr);  ein = zeros(nk, nr);  m1a = zeros(nk, nr);  m2 = zeros(nk, nr);
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
        end
    end

    fig = figure('Position', [40 40 1180 880], 'Color', 'w');
    set(fig, 'DefaultAxesFontSize', 10, 'DefaultTextFontSize', 10);
    tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ---- (a) 观测器峰值 + 阶段 I 饱和 ----
    ax1 = nexttile(tl); hold on; grid on;
    yyaxis(ax1, 'left');
    for ki = 1:nk
        semilogy(ax1, ratios, peak(ki,:), 'o-', 'Color', cols(ki,:), 'LineWidth', 1.4, ...
            'MarkerFaceColor', cols(ki,:), 'MarkerSize', 5);
    end
    set(ax1, 'YScale', 'log'); ylabel(ax1, '||\epsilon_2||_\infty 峰值 [rad/s^2]');
    yyaxis(ax1, 'right');
    for ki = 1:nk
        plot(ax1, ratios, sat1(ki,:), 's--', 'Color', cols(ki,:), 'LineWidth', 1.1, 'MarkerSize', 5);
    end
    ylabel(ax1, '阶段 I 饱和占空比 [%]');
    xlabel('T_c / T_o');
    title(sprintf('(a) 观测器峰值与阶段 I 饱和（T_o = %.1f s；竖虚线右側 T_c > T_o 破坏假设 2.6）', cfg.To));
    xline(ax1, 1.0, 'k--', 'LineWidth', 1.2);
    legend(ax1, {sprintf('%s: 峰值', kname{1}), sprintf('%s: 峰值', kname{2}), ...
                 sprintf('%s: 饱和', kname{1}), sprintf('%s: 饱和', kname{2})}, ...
           'Location', 'east', 'FontSize', 8);

    % ---- (b) 稳态残差与终端误差 ----
    ax2 = nexttile(tl); hold on; grid on;
    for ki = 1:nk
        semilogy(ax2, ratios, ss(ki,:), 'o-', 'Color', cols(ki,:), 'LineWidth', 1.4, ...
            'MarkerFaceColor', cols(ki,:), 'MarkerSize', 5);
        semilogy(ax2, ratios, ein(ki,:), 'd--', 'Color', cols(ki,:), 'LineWidth', 1.0, 'MarkerSize', 5);
    end
    set(ax2, 'YScale', 'log');
    xlabel('T_c / T_o'); ylabel('||e|| [rad]');
    title('(b) 稳态残差 sup||z||_\infty（实线）与终端误差 |e(T_f)|（虚线）');
    xline(ax2, 1.0, 'k--', 'LineWidth', 1.2);
    legend(ax2, {[kname{1} ' ss'], [kname{2} ' ss'], [kname{1} ' |e(T_f)|'], [kname{2} ' |e(T_f)|']}, ...
           'Location', 'east', 'FontSize', 8);

    % ---- (c) 包络指标 ----
    ax3 = nexttile(tl); hold on; grid on;
    yyaxis(ax3, 'left');
    plot(ax3, ratios, m1a(1,:), 'o-', 'Color', cols(1,:), 'LineWidth', 1.4, ...
        'MarkerFaceColor', cols(1,:), 'MarkerSize', 5);
    plot(ax3, ratios, m1a(2,:), 's--', 'Color', cols(2,:), 'LineWidth', 1.2, 'MarkerSize', 6);
    ylabel(ax3, 'M1a 包络违反时长 [s\cdotjoint]');
    yyaxis(ax3, 'right');
    semilogy(ax3, ratios, m2(1,:), 'd-', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.2, 'MarkerSize', 6);
    set(ax3, 'YScale', 'log'); ylabel(ax3, 'M2 进带深度 / \rho_\infty');
    xlabel('T_c / T_o');
    title('(c) 阶段 I 包络违反时长（左）与进带深度（右，PPC）');
    xline(ax3, 1.0, 'k--', 'LineWidth', 1.2);
    legend(ax3, {[kname{1} ' M1a'], [kname{2} ' M1a'], 'M2（PPC）'}, ...
           'Location', 'east', 'FontSize', 8);

    % 横轴取对数, 使 0.2--1 的敏感区可辨
    for ax = [ax1 ax2 ax3]
        set(ax, 'XScale', 'log');
        set(ax, 'XTick', ratios);
        set(ax, 'XTickLabel', arrayfun(@(v) sprintf('%.2g', v), ratios, 'UniformOutput', false));
        xlim(ax, [min(ratios)*0.85, max(ratios)*1.2]);
    end

    saveas(fig, fname);
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
