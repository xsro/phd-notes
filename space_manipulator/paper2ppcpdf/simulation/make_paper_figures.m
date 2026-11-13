%% make_paper_figures.m
% -------------------------------------------------------------------------
% 由 results_dou_yue_259.mat 生成论文第 6 节所需的三张图:
%   1) figures/end_effector_joint_position_trajectory.png
%      自由漂浮基座平动、末端轨迹与关节姿态变化
%   2) figures/joint_tracking_error_torque_summary.png
%      关节位置/速度跟踪、跟踪误差与控制力矩综合响应
%   3) figures/disturbance_observer.png
%      等效加速度扰动真实值、观测值与观测误差
%
% 代表算例: PDF+PTDO（本文方法）。基座运动由零动量关系
%   \\dot x_b = -H_b^{-1} H_bm \\dot q
% 由关节轨迹反算并积分; 基座姿态用角速度的一阶积分（小角度近似）。
%
% 用法: matlab -batch "make_paper_figures"
% -------------------------------------------------------------------------
clear; clc; close all;

here = fileparts(mfilename('fullpath'));
res_file = fullfile(here, 'results_dou_yue_259.mat');
fig_dir  = fullfile(here, '..', 'figures');
if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
set(0, 'DefaultFigureVisible', 'off');

S = load(res_file);
P = S.P;
cfg = S.cfg;
R = S.all_results(1, :);                 % 第 1 组: kappa 标定后的结果
names = cellfun(@(r) char(r.name), R, 'UniformOutput', false);
iPTDO = find(contains(names, 'PDF+PTDO'), 1);
if isempty(iPTDO), error('结果中找不到 PDF+PTDO 算例'); end
res = R{iPTDO};
t   = cfg.t;
N   = numel(t);
n   = P.n;

% ---- 由误差反算真实关节轨迹 ----
qd  = S.ref.qd;   dqd = S.ref.dqd;
q   = qd  - res.E;
dq  = dqd - res.Ed;

% ---- 基座运动: 零动量关系 ----
xb  = zeros(3, N);   rb = zeros(3, N);
for k = 1:N
    M = ffsm259_model(P, q(:,k), dq(:,k));
    xbdot = -M.Hb \ (M.Hbm*dq(:,k));
    rb(:,k) = xbdot(4:6);                % 角速度
    if k == 1
        xb(:,k) = xbdot(1:3)*cfg.Ts;
    else
        xb(:,k) = xb(:,k-1) + xbdot(1:3)*cfg.Ts;
    end
end
rot = zeros(3, N);                       % 基座姿态（一阶积分, 小角度）
for k = 2:N
    rot(:,k) = rot(:,k-1) + rb(:,k)*cfg.Ts;
end
pe = res.PE;                             % 末端误差范数, 仅用于核对

% 各体系原点（用于三维构型图）
M0 = ffsm259_model(P, q(:,1));
Mw = ffsm259_model(P, q(:,end));

%% ===================== 图 1: 运动与构型 =====================
fig1 = figure('Position', [40 40 1250 800]);
tl1 = tiledlayout(fig1, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- 左: 三维轨迹 + 初末构型 ---
ax = nexttile(tl1, [3 2]); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
% 由基座平动累积得到末端绝对轨迹
pe_abs = zeros(3, N);
for k = 1:N
    M = ffsm259_model(P, q(:,k));
    pe_abs(:,k) = xb(:,k) + M.pe;
end
plot3(ax, xb(1,:), xb(2,:), xb(3,:), 'b-', 'LineWidth', 1.6);
plot3(ax, pe_abs(1,:), pe_abs(2,:), pe_abs(3,:), 'r-', 'LineWidth', 1.6);
plot3(ax, xb(1,1), xb(2,1), xb(3,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(ax, xb(1,end), xb(2,end), xb(3,end), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot3(ax, pe_abs(1,1), pe_abs(2,1), pe_abs(3,1), 'b^', 'MarkerSize', 7, 'MarkerFaceColor', 'b');
plot3(ax, pe_abs(1,end), pe_abs(2,end), pe_abs(3,end), 'rv', 'MarkerSize', 7, 'MarkerFaceColor', 'r');
% 初末构型串联链（各体系原点）
Pj0 = [xb(:,1), xb(:,1) + M0.pos];   % 平移到绝对坐标
Pjw = [xb(:,end), xb(:,end) + Mw.pos];
plot3(ax, Pj0(1,:), Pj0(2,:), Pj0(3,:), '-o', 'Color', [0.55 0.55 0.55], ...
    'MarkerSize', 4, 'MarkerFaceColor', [0.55 0.55 0.55], 'LineWidth', 2);
plot3(ax, Pjw(1,:), Pjw(2,:), Pjw(3,:), '-o', 'Color', [0.15 0.15 0.15], ...
    'MarkerSize', 4, 'MarkerFaceColor', [0.15 0.15 0.15], 'LineWidth', 2);
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
title(ax, '基座轨迹、末端轨迹与初末构型');
legend(ax, {'基座质心轨迹', '末端轨迹', '基座初始', '基座终止', ...
    '末端初始', '末端终止', '初始构型', '终止构型'}, ...
    'Location', 'best', 'FontSize', 7);
view(ax, 45, 25); axis(ax, 'equal');

% --- 右: 基座位置 / 末端位置 / 关节角 ---
nexttile(tl1); hold on; grid on;
plot(t, xb(1,:), t, xb(2,:), t, xb(3,:), 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('r_b [m]');
title('基座质心位置'); legend({'x', 'y', 'z'}, 'FontSize', 7);

nexttile(tl1); hold on; grid on;
plot(t, pe_abs(1,:), t, pe_abs(2,:), t, pe_abs(3,:), 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('p_e [m]');
title('末端位置'); legend({'x', 'y', 'z'}, 'FontSize', 7);

nexttile(tl1); hold on; grid on;
plot(t, rad2deg(q).', 'LineWidth', 1.1);
xlabel('t [s]'); ylabel('q [deg]');
title('关节角');
legend(arrayfun(@(i) sprintf('q_%d', i), 1:n, 'UniformOutput', false), ...
    'FontSize', 7, 'NumColumns', 2, 'Location', 'eastoutside');

f1name = fullfile(fig_dir, 'end_effector_joint_position_trajectory.png');
export_paper(fig1, f1name, [6.3 4.4], 8);

%% ===================== 图 2: 关节跟踪/误差/力矩 =====================
fig2 = figure('Position', [40 40 1440 880]);
tl = tiledlayout(fig2, 7, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:n
    nexttile(tl, (i-1)*3 + 1);
    plot(t, rad2deg(qd(i,:)), 'k--', 'LineWidth', 1.0); hold on;
    plot(t, rad2deg(q(i,:)),  'b-',  'LineWidth', 1.0);
    grid on; ylabel(sprintf('q_%d', i), 'FontSize', 7);
    if i == 1, title('关节角 [deg]（虚线：期望）', 'FontSize', 9); end
    if i < n, set(gca, 'XTickLabel', []); else, xlabel('t [s]'); end
    set(gca, 'FontSize', 7);
end

for i = 1:n
    nexttile(tl, (i-1)*3 + 2);
    plot(t, dqd(i,:), 'k--', 'LineWidth', 1.0); hold on;
    plot(t, dq(i,:),  'r-',  'LineWidth', 1.0);
    grid on; ylabel(sprintf('dq_%d', i), 'FontSize', 7);
    if i == 1, title('关节角速度 [rad/s]（虚线：期望）', 'FontSize', 9); end
    if i < n, set(gca, 'XTickLabel', []); else, xlabel('t [s]'); end
    set(gca, 'FontSize', 7);
end

nexttile(tl, [2 1]);   % 行 1-2, 第 3 列
semilogy(t, max(vecnorm(res.E, 2, 1), 1e-16), 'b-', 'LineWidth', 1.2); grid on;
ylabel('||e|| [rad]'); title('位置/速度误差与控制力矩', 'FontSize', 9);
set(gca, 'FontSize', 7);

nexttile(tl, [2 1]);
semilogy(t, max(vecnorm(res.Ed, 2, 1), 1e-16), 'r-', 'LineWidth', 1.2); grid on;
ylabel('||de|| [rad/s]'); set(gca, 'FontSize', 7);
set(gca, 'XTickLabel', []);

nexttile(tl, [3 1]);
plot(t, max(abs(res.TAU), [], 1), 'k-', 'LineWidth', 0.4); hold on; grid on;
yline(P.tau_max(1), 'r--');
xlabel('t [s]'); ylabel('max|\tau_i| [Nm]'); set(gca, 'FontSize', 7);

f2name = fullfile(fig_dir, 'joint_tracking_error_torque_summary.png');
export_paper(fig2, f2name, [6.3 5.4], 7);

%% ===================== 图 3: 扰动观测器 =====================
fig3 = figure('Position', [40 40 1200 700]);
ctl = tiledlayout(fig3, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
idx = 1:min(n, 7);
for i = idx
    nexttile(ctl); hold on; grid on;
    plot(t, res.DDA(i,:), 'k--', 'LineWidth', 1.6);          % 真实值
    plot(t, res.DAH(i,:), 'b-',  'LineWidth', 0.4);          % 观测值
    plot([t(1) t(end)], [0 0], 'Color', [0.6 0.6 0.6]);
    ylabel(['Δa_' num2str(i)], 'FontSize', 8);
    set(gca, 'FontSize', 7);
    if i == 1
        title('等效加速度扰动：真实值(黑虚) / 观测值(蓝)', 'FontSize', 10);
    end
    if i >= numel(idx), xlabel('t [s]'); end
end
nexttile(ctl); hold on; grid on;
err = res.DDA - res.DAH;
plot(t, vecnorm(err, 2, 1), 'm-', 'LineWidth', 1.0);
plot(t, sqrt(mean(err.^2, 1)), 'k-', 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('误差', 'FontSize', 8);
title('观测误差（紫: 2-范数, 黑: RMS）', 'FontSize', 9);
set(gca, 'FontSize', 7);
fprintf('观测误差: max ||e|| = %.3e, RMS = %.3e (t >= T_o)\n', ...
    max(vecnorm(err(:, t >= 1), 2, 1)), sqrt(mean(err(:, t >= 1).^2, 'all')));

f3name = fullfile(fig_dir, 'disturbance_observer.png');
export_paper(fig3, f3name, [6.3 4.0], 8);

%% ===================== 数值摘要（供正文引用） =====================
fprintf('\n=== 代表算例 (%s) 数值摘要 ===\n', res.name);
fprintf('  |e(T)|      = %.3e rad\n', res.m.final_e);
fprintf('  |de(T)|     = %.3e rad/s\n', res.m.final_de);
fprintf('  RMS_ss(e)   = %.3e rad\n', res.m.rms_e_ss);
fprintf('  t_s (1e-3)  = %.2f s\n', res.m.settle);
fprintf('  max|tau|    = %.1f Nm (限幅 %g)\n', res.m.max_tau, P.tau_max(1));
fprintf('  饱和占空比   = %.1f %%\n', 100*res.m.sat_ratio);
fprintf('  基座最大位移 = %.3e m, 基座最大转角 = %.3e rad\n', ...
    max(vecnorm(xb,2,1)), max(vecnorm(rot,2,1)));
fprintf('  末端行程     = %.3f m\n', norm(pe_abs(:,end)-pe_abs(:,1)));

%% ===================== 全表数值（含稳态末端误差） =====================
fprintf('\n=== 全部算例 (kappa = %g) ===\n', cfg.kappa);
fprintf('%-32s %10s %10s %10s %8s %8s %8s %10s\n', ...
    'case', '|e(T)|', 'RMS_ss', 't_s', 'max|tau|', 'sat%', 'EE_ss', 'obs_err');
for i = 1:numel(R)
    r = R{i};
    ss = cfg.t >= cfg.Tref;
    ee_ss = sqrt(mean(r.PE(ss).^2));
    fprintf('%-32s %10.3e %10.3e %10.2f %8.1f %8.1f %10.3e %10.3e\n', ...
        char(r.name), r.m.final_e, r.m.rms_e_ss, r.m.settle, r.m.max_tau, ...
        100*r.m.sat_ratio, ee_ss, r.m.dob_err);
end
err = res.DDA - res.DAH;
fprintf('\nPTDO 观测误差: t in [2,20] RMS = %.3e, max = %.3e\n', ...
    sqrt(mean(err(:, cfg.t >= 2).^2, 'all')), max(vecnorm(err(:, cfg.t >= 2), 2, 1)));

% ------------------------------------------------------------------------
function export_paper(f, fname, wh, fs)
% 按论文版面物理尺寸导出: 设置 PaperPosition 后以 300 dpi 打印,
% 使图内字号在 \textwidth 处约为 fs pt。
    set(f, 'PaperUnits', 'inches');
    set(f, 'PaperPosition', [0 0 wh(1) wh(2)]);
    set(f, 'PaperSize', wh);
    ax = findall(f, 'Type', 'axes');
    for k = 1:numel(ax)
        set(ax(k), 'FontSize', fs);
        t = get(ax(k), 'Title');
        if ~isempty(t.String), set(t, 'FontSize', fs+1); end
        xl = get(ax(k), 'XLabel'); yl = get(ax(k), 'YLabel');
        if ~isempty(xl.String), set(xl, 'FontSize', fs); end
        if ~isempty(yl.String), set(yl, 'FontSize', fs); end
    end
    lg = findall(f, 'Type', 'Legend');
    for k = 1:numel(lg), set(lg(k), 'FontSize', max(fs-1, 6)); end
    print(f, '-dpng', '-r300', fname);
    fprintf('已生成: %s (%dx%d px)\n', fname, ...
        round(wh(1)*300), round(wh(2)*300));
end
