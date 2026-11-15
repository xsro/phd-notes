%% make_base_motion_figure.m
% =========================================================================
% 生成论文第 5 节的两张子图 (独立 MATLAB 图, 各自导出矢量 PDF):
%
%   (a) figures/ppc_base_config.pdf
%       惯性系 (初始基座系) 中 t = 0 与 t = T_f 的机械臂构型
%       (基座质心 -> 各关节 -> 末端), 以及基座质心与末端的运动轨迹;
%   (b) figures/ppc_base_position.pdf
%       基座质心位置 r_b(t) 的三个分量 (由零动量约束反算并积分)。
%
% 数据来源: results_ppc_pdf.mat (main_ppc_pdf_verify.m 的结果), 取本文主
% 算例 R{2} (PPC, rho_inf = 1e-3)。不重新仿真, 只做运动学/零动量重建。
%
% 零动量关系 (论文式 (eq:gjm)):
%   x_b_dot = -H_b^{-1} H_{bm} * dq = [v_b; w_b]  (体坐标系)
% 惯性系积分:
%   r_b(k+1) = r_b(k) + R_b(k) * v_b(k) * Ts
%   R_b(k+1) = R_b(k) * expm(skew(w_b(k)) * Ts)
%
% 运行: matlab -batch "make_base_motion_figure"
% =========================================================================
clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);
set(0, 'DefaultFigureVisible', 'off');

S   = load(fullfile(here, 'results_ppc_pdf.mat'));
P   = S.P;  cfg = S.cfg;  ref = S.ref;  r = S.R{2};
n   = P.n;  t = cfg.t;  N = cfg.N;  Ts = cfg.Ts;

%% ---------------- 1. 由跟踪误差重建关节轨迹 ----------------
q  = ref.qd  - r.raw.E;
dq = ref.dqd - r.raw.Ed;

%% ---------------- 2. 零动量反算基座速度并积分 ----------------
vb  = zeros(3, N);  wb = zeros(3, N);
CH  = cell(1, N);                      % 各时刻的关节链 (基座系)
PEb = zeros(3, N);                     % 末端位置 (基座系)
for k = 1:N
    Sk = ffsm259_model(P, q(:,k));     % 惯性块与运动学
    xi = -Sk.Hb \ (Sk.Hbm * dq(:,k));
    vb(:,k) = xi(1:3);  wb(:,k) = xi(4:6);
    CH{k}   = Sk.pos;                  % 3 x (n+1): 基座原点, 关节 1..n-1, 末端
    PEb(:,k) = Sk.pe;
end

rb  = zeros(3, N);   Rbk = eye(3);      % 惯性系基座位置/姿态
CHI = cell(1, N);   peI = zeros(3, N);
rbk = zeros(3, 1);                    % 当前基座位置
skew = @(w)[0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
for k = 1:N
    rb(:,k)  = rbk;
    CHI{k}   = rbk + Rbk * CH{k};
    peI(:,k) = rbk + Rbk * PEb(:,k);
    if k < N
        rbk = rbk + Rbk * vb(:,k) * Ts;
        Rbk = Rbk * expm(skew(wb(:,k)) * Ts);
    end
end

dr   = rb(:,end);
dang = acos(max(-1, min(1, (trace(Rbk) - 1)/2)));
fprintf('基座终点位移 = [%.1f %.1f %.1f] mm, |r_b| = %.2f mm\n', 1e3*dr, 1e3*norm(dr));
fprintf('基座姿态变化 = %.3f rad; max|v_b| = %.2f mm/s\n', dang, 1e3*max(vecnorm(vb)));
fprintf('末端起点 = [%.3f %.3f %.3f] m, 终点 = [%.3f %.3f %.3f] m\n', ...
    peI(:,1), peI(:,end));
fprintf('惯性系末端行程 = %.3f m\n', sum(vecnorm(diff(peI,1,2))));

%% ---------------- 3. 子图 (a): 惯性系构型 ----------------
figA = figure('Position', [40 40 640 560], 'Color', 'w');
set(figA, 'DefaultAxesFontSize', 8, 'DefaultTextFontSize', 8);
ax1 = axes(figA, 'Position', [0.075 0.11 0.885 0.84]);
hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
c0 = [0.00 0.20 0.60];      % 起始构型
c1 = [0.80 0.20 0.10];      % 终止构型
h1 = plot3(ax1, rb(1,:),  rb(2,:),  rb(3,:),  '-',  'Color', [0.55 0.75 0.95], 'LineWidth', 1.3);
h2 = plot3(ax1, peI(1,:), peI(2,:), peI(3,:), '-',  'Color', [0.95 0.60 0.55], 'LineWidth', 1.3);
A0 = CHI{1};   AN = CHI{end};
h3 = plot3(ax1, A0(1,:), A0(2,:), A0(3,:), '-o', 'Color', c0, ...
    'MarkerFaceColor', c0, 'MarkerSize', 4, 'LineWidth', 1.8);
h4 = plot3(ax1, AN(1,:), AN(2,:), AN(3,:), '-o', 'Color', c1, ...
    'MarkerFaceColor', 'w', 'MarkerSize', 4, 'LineWidth', 1.8);
plot3(ax1, rb(1,1), rb(2,1), rb(3,1), 'ks', 'MarkerFaceColor', c0, 'MarkerSize', 8);
plot3(ax1, rb(1,end), rb(2,end), rb(3,end), 'ks', 'MarkerFaceColor', 'w', 'MarkerSize', 8, 'LineWidth', 1.2);
plot3(ax1, peI(1,1), peI(2,1), peI(3,1), 'k^', 'MarkerFaceColor', c0, 'MarkerSize', 7);
plot3(ax1, peI(1,end), peI(2,end), peI(3,end), 'k^', 'MarkerFaceColor', 'w', 'MarkerSize', 7, 'LineWidth', 1.2);
axis(ax1, 'equal'); view(ax1, 42, 22);
xlabel(ax1, 'x [m]'); ylabel(ax1, 'y [m]'); zlabel(ax1, 'z [m]');
legend(ax1, [h1 h2 h3 h4], {'基座轨迹', '末端轨迹', ...
    '起始构型 (t=0)', '终止构型 (t=T_f)'}, 'Location', 'northwest', ...
    'FontSize', 6.5, 'NumColumns', 2);
export_pdf(figA, fullfile(here, '..', 'figures', 'ppc_base_config.pdf'), 3.05);

%% ---------------- 4. 子图 (b): 基座位置曲线 ----------------
figB = figure('Position', [40 40 640 560], 'Color', 'w');
set(figB, 'DefaultAxesFontSize', 8, 'DefaultTextFontSize', 8);
ax2 = axes(figB, 'Position', [0.15 0.13 0.80 0.82]);
hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
hb = plot(ax2, t, 1e3*rb.', 'LineWidth', 1.3);
xlabel(ax2, 't [s]'); ylabel(ax2, 'r_b [mm]');
legend(ax2, hb, {'x', 'y', 'z'}, 'Location', 'northwest', 'FontSize', 8);
xlim(ax2, [0 cfg.Tf]);
text(ax2, 0.60, 0.12, sprintf('|r_b(T_f)| = %.1f mm', 1e3*norm(dr)), ...
    'Units', 'normalized', 'FontSize', 8);
export_pdf(figB, fullfile(here, '..', 'figures', 'ppc_base_position.pdf'), 3.05);
