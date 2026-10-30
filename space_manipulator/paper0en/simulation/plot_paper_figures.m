function plot_paper_figures(results, ref, P, cfg)
%PLOT_PAPER_FIGURES  Export the paper figures from the simulation results.
%
%   Each panel is written as an independent PDF/PNG pair in cfg.figure_dir
%   so that the LaTeX source can assemble them with \subfloat.

dir_out = cfg.figure_dir;
n = cfg.n;

% main result: PDF + PTDO under disturbance
idx = find(strcmp({results.name}, 'PDF+PTDO, disturbed'), 1);
if isempty(idx), idx = find([results.use_observer], 1, 'last'); end
if isempty(idx), idx = numel(results); end
main = results(idx);

t = main.t;
q = main.q; dq = main.dq; tau = main.tau;
e = main.e; edot = main.edot;

% desired end-effector trajectory from the reference joint motion
ee_des = zeros(3, numel(t));
% base-frame FK of the (ideal) reference motion, placed at the actual base pose
for k = 1:numel(t)
    ee_des(:,k) = main.base_pos(:,k) + main.ee_pos(:,k)*0 + ...
        rotm_zyx(main.base_att(:,k)) * ffsm_dou_fk(P, ref.qd(:,k));
end
ee_act = main.ee_pos;

lw = 1.1;
fs = 7;

fs_fig = figure('Color','w','Position',[100 100 760 300]);
subplot(1,2,1);
plot(main.base_pos(1,:)*1e2, main.base_pos(2,:)*1e2, 'k-', 'LineWidth', 1.4); hold on; grid on;
plot(main.base_pos(1,1)*1e2, main.base_pos(2,1)*1e2, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
plot(main.base_pos(1,end)*1e2, main.base_pos(2,end)*1e2, 'ks', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
xlabel('$x_b$ [cm]'); ylabel('$y_b$ [cm]');
title('base reaction motion'); axis equal;
subplot(1,2,2);
plot3(ee_act(1,:), ee_act(2,:), ee_act(3,:), 'b-', 'LineWidth', 1.6); hold on; grid on;
plot3(ee_des(1,:), ee_des(2,:), ee_des(3,:), 'r--', 'LineWidth', 1.0);
plot3(ee_act(1,1), ee_act(2,1), ee_act(3,1), 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');
plot3(ee_act(1,end), ee_act(2,end), ee_act(3,end), 'bs', 'MarkerSize', 4, 'MarkerFaceColor', 'b');
xlabel('$x$ [m]'); ylabel('$y$ [m]'); zlabel('$z$ [m]');
legend({'actual','desired'}, 'FontSize', fs-1, 'Location', 'best');
title('end-effector path'); view(30, 20); axis tight;
export_pair(fs_fig, dir_out, 'free_floating_trajectory');

fs_fig = figure('Color','w','Position',[100 100 420 300]);
plot(t, main.base_pos, 'LineWidth', lw); grid on;
xlabel('$t$ [s]'); ylabel('base position [m]');
legend({'$x_b$','$y_b$','$z_b$'}, 'FontSize', fs-1, 'Location', 'best');
export_pair(fs_fig, dir_out, 'base_position');

fs_fig = figure('Color','w','Position',[100 100 420 300]);
plot(t, ee_act, 'LineWidth', lw); hold on; grid on;
plot(t, ee_des, '--', 'LineWidth', 0.8);
xlabel('$t$ [s]'); ylabel('end-effector position [m]');
legend({'$x$','$y$','$z$'}, 'FontSize', fs-1, 'Location', 'best');
export_pair(fs_fig, dir_out, 'end_effector_position');

fs_fig = figure('Color','w','Position',[100 100 420 300]);
plot(t, q, 'LineWidth', lw); grid on;
xlabel('$t$ [s]'); ylabel('joint angle [rad]');
legend(arrayfun(@(i) sprintf('$q_%d$', i), 1:n, 'UniformOutput', false), ...
    'FontSize', fs-2, 'Location', 'eastoutside', 'NumColumns', 2);
export_pair(fs_fig, dir_out, 'joint_posture');

fs_fig = figure('Color','w','Position',[100 100 420 300]);
plot(t, ref.qd, '--', 'LineWidth', 0.8); hold on;
plot(t, q, '-', 'LineWidth', lw); grid on;
xlabel('$t$ [s]'); ylabel('joint angle [rad]');
title('solid: actual, dashed: desired');
export_pair(fs_fig, dir_out, 'joint_position_tracking');

fs_fig = figure('Color','w','Position',[100 100 420 300]);
plot(t, dq, '-', 'LineWidth', lw); hold on;
plot(t, ref.dqd, '--', 'LineWidth', 0.8); grid on;
xlabel('$t$ [s]'); ylabel('joint rate [rad/s]');
title('solid: actual, dashed: desired');
export_pair(fs_fig, dir_out, 'joint_velocity_tracking');

%% tracking errors of the three compared controllers
names = {'PD, disturbed', 'PDF, disturbed', 'PDF+PTDO, disturbed'};
cols = lines(numel(names));
fs_fig = figure('Color','w','Position',[100 100 420 300]);
for c = 1:numel(names)
    r = results(strcmp({results.name}, names{c}));
    if isempty(r), continue; end
    semilogy(r.t, vecnorm(r.e, 2, 1), 'Color', cols(c,:), 'LineWidth', lw); hold on;
end
grid on; xlabel('$t$ [s]'); ylabel('$\|e(t)\|$ [rad]');
legend({'PD','PDF','PDF+PTDO'}, 'FontSize', fs-1, 'Location', 'best');
export_pair(fs_fig, dir_out, 'tracking_errors');

fs_fig = figure('Color','w','Position',[100 100 420 300]);
plot(t, tau, 'LineWidth', lw); hold on; grid on;
yline(20, 'k--'); yline(-20, 'k--');
xlabel('$t$ [s]'); ylabel('joint torque [N m]');
legend(arrayfun(@(i) sprintf('$\\tau_%d$', i), 1:n, 'UniformOutput', false), ...
    'FontSize', fs-2, 'Location', 'eastoutside', 'NumColumns', 2);
export_pair(fs_fig, dir_out, 'control_torque');

%% observer
fs_fig = figure('Color','w','Position',[100 100 420 320]);
subplot(2,1,1);
plot(t, main.delta_a, 'LineWidth', lw); grid on;
ylabel('$\Delta_a$');
subplot(2,1,2);
plot(t, main.delta_a_hat, 'LineWidth', lw); grid on;
xlabel('$t$ [s]'); ylabel('$\hat\Delta_a$');
export_pair(fs_fig, dir_out, 'observer_estimates');

fs_fig = figure('Color','w','Position',[100 100 420 260]);
semilogy(t, vecnorm(main.delta_a_error, 2, 1), 'LineWidth', lw); grid on;
xlabel('$t$ [s]'); ylabel('$\|\Delta_a-\hat\Delta_a\|$');
export_pair(fs_fig, dir_out, 'observer_error');

%% error-energy / activation
fs_fig = figure('Color','w','Position',[100 100 420 260]);
semilogy(t, sqrt(max(main.V, 1e-18)), 'LineWidth', lw); grid on;
xlabel('$t$ [s]'); ylabel('$\sqrt{V(t)}$');
export_pair(fs_fig, dir_out, 'error_norm');

fs_fig = figure('Color','w','Position',[100 100 420 260]);
plot(t, main.Rh, 'LineWidth', lw); hold on; grid on;
yline(0);
xlabel('$t$ [s]'); ylabel('$R_h(t-T_o)$'); ylim([-0.05 1.05]);
export_pair(fs_fig, dir_out, 'pdf_activation');

close all;
end

%% ------------------------------------------------------------------------
function R = rotm_zyx(ang)
R = [cos(ang(1))*cos(ang(2)), cos(ang(1))*sin(ang(2))*sin(ang(3)) - sin(ang(1))*cos(ang(3)), cos(ang(1))*sin(ang(2))*cos(ang(3)) + sin(ang(1))*sin(ang(3));
     sin(ang(1))*cos(ang(2)), sin(ang(1))*sin(ang(2))*sin(ang(3)) + cos(ang(1))*cos(ang(3)), sin(ang(1))*sin(ang(2))*cos(ang(3)) - cos(ang(1))*sin(ang(3));
     -sin(ang(2)),            cos(ang(2))*sin(ang(3)),                                       cos(ang(2))*cos(ang(3))];
end

function export_pair(fig, dir_out, name)
exportgraphics(fig, fullfile(dir_out, [name '.pdf']), 'ContentType', 'vector');
exportgraphics(fig, fullfile(dir_out, [name '.png']), 'Resolution', 300);
end
