%% plot_ffsm_pdf_tracking_results.m
% -------------------------------------------------------------------------
% Plot saved simulation results for the free-floating space manipulator PDF
% tracking study. Run main_ffsm_pdf_tracking_yan_params.m first to generate
% latest_results.mat.
% -------------------------------------------------------------------------

clear; clc; close all;

set(groot, 'DefaultTextInterpreter', 'latex');
set(groot, 'DefaultAxesTickLabelInterpreter', 'latex');
set(groot, 'DefaultLegendInterpreter', 'latex');

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

result_file = fullfile(script_dir, 'latest_results.mat');
if ~exist(result_file, 'file')
    error('Result file not found: %s. Run main_ffsm_pdf_tracking_yan_params.m first.', result_file);
end

S = load(result_file);
cfg = S.cfg;
ref = S.ref;
P = S.P;
results = S.results;

% Always write figures next to the paper being compiled, rather than using
% an absolute path stored in an old result file.
cfg.figure_dir = fullfile(script_dir, '..', 'figures');
if ~isfield(cfg, 'save_figures')
    cfg.save_figures = true;
end
if ~isfield(cfg, 'show_figures')
    cfg.show_figures = false;
end
if cfg.save_figures && ~exist(cfg.figure_dir, 'dir')
    mkdir(cfg.figure_dir);
end
if ~cfg.show_figures
    set(0, 'DefaultFigureVisible', 'off');
end

observer_idx = find([results.use_observer], 1, 'last');
if isempty(observer_idx)
    observer_idx = numel(results);
end
main_result = results(observer_idx);

plot_results(main_result, ref, P, cfg.figure_dir, cfg.save_figures);
plot_case_comparison(results, cfg.figure_dir, cfg.save_figures);
plot_observer_results(main_result, cfg.figure_dir, cfg.save_figures);

fprintf('Figures regenerated from: %s\n', result_file);
fprintf('Figures saved to: %s\n', cfg.figure_dir);

function plot_results(result, ref, P, figure_dir, save_figures)
% Plot the disturbed PDF case for paper figures.

    t = result.t;
    q = result.q;
    dq = result.dq;
    e = result.e;
    edot = result.edot;
    tau = result.tau;
    Rhlog = result.Rh;
    Vlog = result.V;
    n = size(q, 1);

    [base_actual, ee_actual, nodes_initial, nodes_final] = ...
        nominal_floating_trajectory(q, P);
    [~, ee_desired, ~, ~] = nominal_floating_trajectory(ref.qd, P);

    figure('Name','Free-floating trajectory','Color','w', ...
           'Position', [100, 100, 900, 700]);
    plot3(ee_actual(1,:), ee_actual(2,:), ee_actual(3,:), 'b', 'LineWidth', 1.5);
    hold on;
    plot3(ee_desired(1,:), ee_desired(2,:), ee_desired(3,:), 'r--', 'LineWidth', 1.3);
    plot3(base_actual(1,:), base_actual(2,:), base_actual(3,:), ...
          'k:', 'LineWidth', 1.6);
    plot3(nodes_initial(1,:), nodes_initial(2,:), nodes_initial(3,:), ...
          '-o', 'Color', [0.10 0.45 0.10], 'LineWidth', 1.5, ...
          'MarkerSize', 4.5, 'MarkerFaceColor', [0.10 0.45 0.10]);
    plot3(nodes_final(1,:), nodes_final(2,:), nodes_final(3,:), ...
          '-s', 'Color', [0.55 0.15 0.55], 'LineWidth', 1.5, ...
          'MarkerSize', 4.5, 'MarkerFaceColor', 'w');
    plot3(nodes_initial(1,1), nodes_initial(2,1), nodes_initial(3,1), ...
          'kp', 'MarkerSize', 10, 'MarkerFaceColor', [0.95 0.80 0.20]);
    plot3(nodes_final(1,1), nodes_final(2,1), nodes_final(3,1), ...
          'kh', 'MarkerSize', 8, 'MarkerFaceColor', [0.95 0.80 0.20]);
    plot3(nodes_final(1,end), nodes_final(2,end), nodes_final(3,end), ...
          'kd', 'MarkerSize', 8, 'MarkerFaceColor', 'c');
    for i = 2:size(nodes_initial, 2)-1
        text(nodes_initial(1,i), nodes_initial(2,i), nodes_initial(3,i), ...
             sprintf('  J%d', i-1), 'FontSize', 8, 'Color', [0.10 0.45 0.10]);
    end
    text(nodes_initial(1,1), nodes_initial(2,1), nodes_initial(3,1), ...
         '  Base(0)', 'FontSize', 8, 'Color', 'k');
    text(nodes_final(1,1), nodes_final(2,1), nodes_final(3,1), ...
         '  Base(T)', 'FontSize', 8, 'Color', 'k');
    text(nodes_final(1,end), nodes_final(2,end), nodes_final(3,end), ...
         '  EE', 'FontSize', 8, 'Color', 'k');
    grid on;
    axis tight;
    view(42, 24);
    xlabel('$x_e$ [m]');
    ylabel('$y_e$ [m]');
    zlabel('$z_e$ [m]');
    base_drift = norm(base_actual(:,end) - base_actual(:,1));
    title(sprintf('Free-floating base motion, end-effector trajectory and arm configurations (base drift %.3f m)', base_drift));
    legend('Actual EE trajectory', 'Desired EE trajectory', ...
           'Base trajectory', 'Initial arm', 'Final arm', ...
           'Initial base', 'Final base', 'Final end-effector', ...
           'Location', 'southoutside', 'NumColumns', 2, 'FontSize', 8);

    save_figure(gcf, figure_dir, 'free_floating_trajectory', save_figures);

    figure('Name','Base position','Color','w');
    plot(t, base_actual, 'LineWidth', 1.2);
    grid on;
    xlabel('$t$ [s]');
    ylabel('$p_b$ [m]');
    title('Base position');
    legend({'$x_b$', '$y_b$', '$z_b$'}, 'Location', 'eastoutside');

    save_figure(gcf, figure_dir, 'base_position', save_figures);

    figure('Name','End-effector position','Color','w');
    plot(t, ee_actual, 'LineWidth', 1.2);
    hold on;
    plot(t, ee_desired, '--', 'LineWidth', 1.0);
    grid on;
    xlabel('$t$ [s]');
    ylabel('$p_e$ [m]');
    title('End-effector position');
    legend({'$x_e$', '$y_e$', '$z_e$', '$x_d$', '$y_d$', '$z_d$'}, ...
           'Location', 'eastoutside');

    save_figure(gcf, figure_dir, 'end_effector_position', save_figures);

    figure('Name','Joint posture','Color','w');
    plot(t, q, 'LineWidth', 1.05);
    grid on;
    xlabel('$t$ [s]');
    ylabel('$q_i$ [rad]');
    title('Joint posture');
    legend(arrayfun(@(i)sprintf('$q_%d$', i), 1:n, 'UniformOutput', false), ...
           'Location', 'eastoutside');
    save_figure(gcf, figure_dir, 'joint_posture', save_figures);

    % Each plot is exported separately and combined as LaTeX subfigures.
    figure('Name','Joint position tracking','Color','w');

    clf;
    plot(t, q', 'LineWidth', 1.0); hold on;
    plot(t, ref.qd', '--', 'LineWidth', 0.9);
    grid on; xlabel('$t$ [s]'); ylabel('$q_i$ [rad]');
    legend([arrayfun(@(i)sprintf('$q_%d$',i),1:n,'UniformOutput',false), ...
            arrayfun(@(i)sprintf('$q_{d,%d}$',i),1:n,'UniformOutput',false)], ...
            'Location','eastoutside','FontSize',7);
    save_figure(gcf, figure_dir, 'joint_position_tracking', save_figures);

    figure('Name','Joint velocity tracking','Color','w');
    plot(t, dq', 'LineWidth', 1.0); hold on;
    plot(t, ref.dqd', '--', 'LineWidth', 0.9);
    grid on; xlabel('$t$ [s]'); ylabel('$\dot q_i$ [rad/s]');
    legend([arrayfun(@(i)sprintf('$\\dot q_%d$',i),1:n,'UniformOutput',false), ...
            arrayfun(@(i)sprintf('$\\dot q_{d,%d}$',i),1:n,'UniformOutput',false)], ...
            'Location','eastoutside','FontSize',7);
    save_figure(gcf, figure_dir, 'joint_velocity_tracking', save_figures);

    figure('Name','Tracking errors','Color','w');
    plot(t, e', 'LineWidth', 0.95); hold on;
    plot(t, edot', '--', 'LineWidth', 0.95);
    grid on; xlabel('$t$ [s]'); ylabel('Tracking errors');
    legend([arrayfun(@(i)sprintf('$e_%d$',i),1:n,'UniformOutput',false), ...
            arrayfun(@(i)sprintf('$\\dot e_%d$',i),1:n,'UniformOutput',false)], ...
            'Location','eastoutside','FontSize',7);
    save_figure(gcf, figure_dir, 'tracking_errors', save_figures);

    figure('Name','Control torque','Color','w');
    plot(t, tau', 'LineWidth', 0.95);
    grid on; xlabel('$t$ [s]'); ylabel('$\tau_i$ [N m]');
    legend(arrayfun(@(i)sprintf('$\\tau_%d$',i),1:n,'UniformOutput',false), ...
           'Location','eastoutside','FontSize',7);
    save_figure(gcf, figure_dir, 'control_torque', save_figures);

    % No MATLAB subplots are used; LaTeX assembles the exported figures.
    figure('Name','PDF activation','Color','w');
    plot(t, Rhlog, 'k', 'LineWidth', 1.4);
    grid on; xlabel('$t$ [s]'); ylabel('$R_h(t)$');
    save_figure(gcf, figure_dir, 'pdf_activation', save_figures);

    figure('Name','Error energy','Color','w');
    semilogy(t, Vlog + 1e-16, 'm', 'LineWidth', 1.4);
    grid on; xlabel('$t$ [s]'); ylabel('$V(t)$');
    save_figure(gcf, figure_dir, 'error_energy', save_figures);

    figure('Name','Error norm','Color','w');
    plot(t, vecnorm(e, 2, 1), 'b', 'LineWidth', 1.5); hold on;
    plot(t, vecnorm(edot, 2, 1), 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('$t$ [s]');
    ylabel('Norm');
    legend({'$\|e\|$', '$\|\dot e\|$'});
    title('Tracking error norms');
    save_figure(gcf, figure_dir, 'error_norm', save_figures);
end

function [base_pos, ee_pos, nodes_initial, nodes_final] = nominal_floating_trajectory(q_hist, P)
% Linear-momentum-consistent floating-base visualization.

    N = size(q_hist, 2);
    base_pos = zeros(3, N);
    ee_pos = zeros(3, N);
    [nodes0, com0] = nominal_relative_nodes_and_com(q_hist(:,1), P);
    total_mass = sum(P.mass);
    weighted_com0 = com0 * P.mi(:);

    for k = 1:N
        [nodes_rel, com_rel] = nominal_relative_nodes_and_com(q_hist(:,k), P);
        weighted_com = com_rel * P.mi(:);
        base_pos(:, k) = (weighted_com0 - weighted_com) / total_mass;
        nodes_global = nodes_rel + base_pos(:, k);
        ee_pos(:, k) = nodes_global(:, end);
        if k == 1
            nodes_initial = nodes_global;
        elseif k == N
            nodes_final = nodes_global;
        end
    end

    if N == 1
        nodes_initial = nodes0;
        nodes_final = nodes0;
    end
end

function [nodes, link_com] = nominal_relative_nodes_and_com(q, P)
% Return base-relative joint nodes and approximate link COM positions.

    n = numel(q);
    axes_local = [ ...
        0  0  1;
        0  1  0;
        0  1  0;
        1  0  0;
        0  1  0;
        1  0  0;
        0  0  1].';

    R = eye(3);
    p = zeros(3, 1);
    nodes = zeros(3, n+1);
    link_com = zeros(3, n);
    nodes(:,1) = p;
    for i = 1:n
        R = R * axis_angle_rotation(axes_local(:, i), q(i));
        link_vec = P.a(:, i+1);
        if norm(link_vec) < 1e-9
            link_vec = P.b(:, i+1);
        end
        link_com(:, i) = p + 0.5 * R * link_vec;
        p = p + R * link_vec;
        nodes(:, i+1) = p;
    end
end

function R = axis_angle_rotation(axis, theta)
    axis = axis / norm(axis);
    K = [0, -axis(3), axis(2);
         axis(3), 0, -axis(1);
         -axis(2), axis(1), 0];
    R = eye(3) + sin(theta)*K + (1 - cos(theta))*(K*K);
end

function plot_case_comparison(results, figure_dir, save_figures)
% Compare nominal and disturbed controller error norms.

    figure('Name','Position error comparison','Color','w');
    hold on; grid on;
    for c = 1:numel(results)
        semilogy(results(c).t, vecnorm(results(c).e, 2, 1) + 1e-16, 'LineWidth', 1.4);
    end
    xlabel('$t$ [s]');
    ylabel('$\|e\|$ [rad]');
    legend({results.name}, 'Location', 'northeast');
    title('Position error norm comparison');

    save_figure(gcf, figure_dir, 'controller_position_error', save_figures);

    figure('Name','Velocity error comparison','Color','w');
    hold on; grid on;
    for c = 1:numel(results)
        semilogy(results(c).t, vecnorm(results(c).edot, 2, 1) + 1e-16, 'LineWidth', 1.4);
    end
    xlabel('$t$ [s]');
    ylabel('$\|\dot e\|$ [rad/s]');
    legend({results.name}, 'Location', 'northeast');
    title('Velocity error norm comparison');
    save_figure(gcf, figure_dir, 'controller_velocity_error', save_figures);
end

function plot_observer_results(result, figure_dir, save_figures)
% Plot true equivalent acceleration disturbance, observer estimates and errors.

    if ~result.use_observer
        return;
    end

    t = result.t;
    n = size(result.delta_a_error, 1);

    figure('Name','Disturbance observer estimates','Color','w');
    plot(t, result.delta_a, 'LineWidth', 1.0);
    hold on;
    plot(t, result.delta_a_hat, '--', 'LineWidth', 1.0);
    grid on;
    xlabel('$t$ [s]');
    ylabel('$\Delta_{a,i},\hat{\Delta}_{a,i}$');
    title('Equivalent acceleration disturbance and observer estimates');
    legend([arrayfun(@(i)sprintf('$\\Delta_{a,%d}$', i), 1:n, 'UniformOutput', false), ...
            arrayfun(@(i)sprintf('$\\hat{\\Delta}_{a,%d}$', i), 1:n, 'UniformOutput', false)], ...
           'Location', 'eastoutside', 'FontSize', 8);

    save_figure(gcf, figure_dir, 'observer_estimates', save_figures);

    figure('Name','Disturbance observer error','Color','w');
    semilogy(t, vecnorm(result.delta_a_error, 2, 1) + 1e-16, 'k', 'LineWidth', 1.4);
    grid on;
    xlabel('$t$ [s]');
    ylabel('$\|\tilde{\Delta}_a\|$');
    title('Observer error norm');
    save_figure(gcf, figure_dir, 'observer_error', save_figures);
end

function save_figure(fig, figure_dir, stem, save_figures)
% Save each simulation figure in raster and vector formats for LaTeX use.

    if ~save_figures
        return;
    end
    if ~exist(figure_dir, 'dir')
        mkdir(figure_dir);
    end

    png_file = fullfile(figure_dir, [stem, '.png']);
    pdf_file = fullfile(figure_dir, [stem, '.pdf']);

    try
        exportgraphics(fig, png_file, 'Resolution', 300);
        exportgraphics(fig, pdf_file);
    catch
        set(fig, 'PaperPositionMode', 'auto');
        print(fig, png_file, '-dpng', '-r300');
        print(fig, pdf_file, '-dpdf', '-bestfit');
    end
end
