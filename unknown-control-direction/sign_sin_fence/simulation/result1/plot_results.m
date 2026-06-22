function plot_results(hist, par)

t = par.t;
Nt = par.Nt;
N = par.N;
m = par.m;

out_dir = fullfile(fileparts(mfilename('fullpath')), 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

p = hist.p;
p0 = hist.p0;
s = hist.s;
sdot = hist.sdot;
u = hist.u;
Bdiag = hist.Bdiag;

pair_count = N * (N - 1) / 2;
pair_dist = zeros(Nt, pair_count);
pair_labels = cell(pair_count, 1);
center_to_target = zeros(Nt, 1);
min_pair_dist = zeros(Nt, 1);

for k = 1:Nt
    p_k = squeeze(p(k,:,:));
    p0_k = p0(k,:);

    idx = 0;
    min_d = inf;
    for i = 1:N
        for j = i+1:N
            idx = idx + 1;
            dij = norm(p_k(i,:) - p_k(j,:));
            pair_dist(k, idx) = dij;
            min_d = min(min_d, dij);
            if k == 1
                pair_labels{idx} = sprintf('d_{%d%d}', i, j);
            end
        end
    end

    min_pair_dist(k) = min_d;
    center_to_target(k) = norm(mean(p_k, 1) - p0_k);
end

%% Agent and target trajectories
figure('Color', 'w', 'Position', [100 100 850 700]);
hold on; grid on; axis equal;
colors = lines(N);
for i = 1:N
    pi_hist = squeeze(p(:, i, :));
    plot(pi_hist(:,1), pi_hist(:,2), 'LineWidth', 1.4, 'Color', colors(i,:));
    plot(pi_hist(1,1), pi_hist(1,2), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
    plot(pi_hist(end,1), pi_hist(end,2), 's', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
end
plot(p0(:,1), p0(:,2), 'k--', 'LineWidth', 1.8);
plot(p0(1,1), p0(1,2), 'ko', 'MarkerFaceColor', 'k');
plot(p0(end,1), p0(end,2), 'ks', 'MarkerFaceColor', 'k');
xlabel('$x$', 'Interpreter', 'latex');
ylabel('$y$', 'Interpreter', 'latex');
title('Agent and target trajectories', 'Interpreter', 'latex');
legend([arrayfun(@(i) sprintf('Agent %d', i), 1:N, 'UniformOutput', false), {'Target'}], ...
    'Location', 'best');
saveas(gcf, fullfile(out_dir, 'fig1_trajectories.png'));

%% All pairwise inter-agent distances
figure('Color', 'w', 'Position', [130 130 950 520]);
hold on; grid on;
for idx = 1:pair_count
    plot(t, pair_dist(:, idx), 'LineWidth', 1.15);
end
yline(par.d_safe, 'r--', 'LineWidth', 1.6);
yline(par.mu, 'k:', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('$\|p_i-p_j\|$', 'Interpreter', 'latex');
title('All inter-agent distances', 'Interpreter', 'latex');
legend([pair_labels; {'d_{safe}'; '\mu'}], 'Location', 'eastoutside');
saveas(gcf, fullfile(out_dir, 'fig2_all_pair_distances.png'));

%% Minimum inter-agent distance
figure('Color', 'w', 'Position', [160 160 900 440]);
plot(t, min_pair_dist, 'b-', 'LineWidth', 1.6); hold on; grid on;
yline(par.d_safe, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('$\min_{i<j}\|p_i-p_j\|$', 'Interpreter', 'latex');
title('Minimum inter-agent distance', 'Interpreter', 'latex');
legend('minimum distance', '$d$', 'Interpreter', 'latex', 'Location', 'best');
saveas(gcf, fullfile(out_dir, 'fig3_min_pair_distance.png'));

%% Agent-center to target distance
figure('Color', 'w', 'Position', [190 190 900 440]);
plot(t, center_to_target, 'm-', 'LineWidth', 1.6); hold on; grid on;
xlabel('Time (s)');
ylabel('$\|\bar p-p_0\|$', 'Interpreter', 'latex');
title('Distance from agent center to target', 'Interpreter', 'latex');
legend('$\|\bar p-p_0\|$', 'Interpreter', 'latex', 'Location', 'best');
saveas(gcf, fullfile(out_dir, 'fig4_center_to_target.png'));

%% Sliding variables
figure('Color', 'w', 'Position', [220 120 920 620]);
for ell = 1:m
    subplot(m, 1, ell); hold on; grid on;
    for i = 1:N
        plot(t, squeeze(s(:, i, ell)), 'LineWidth', 1.0);
    end
    eps_ref = par.eps_s(1, ell);
    yline(0, 'k--');
    yline(eps_ref, 'r--');
    yline(-eps_ref, 'r--');
    ylabel(sprintf('$s_{i0,%d}$', ell), 'Interpreter', 'latex');
end
xlabel('Time (s)');
saveas(gcf, fullfile(out_dir, 'fig5_sliding_variables.png'));

%% Norms of sdot
figure('Color', 'w', 'Position', [250 150 900 440]);
hold on; grid on;
for i = 1:N
    sdot_i = squeeze(sdot(:, i, :));
    plot(t, sqrt(sum(sdot_i.^2, 2)), 'LineWidth', 1.0);
end
xlabel('Time (s)');
ylabel('$\|\dot s_{i0}\|$', 'Interpreter', 'latex');
title('Sliding variable derivative norms', 'Interpreter', 'latex');
legend(arrayfun(@(i) sprintf('Agent %d', i), 1:N, 'UniformOutput', false), 'Location', 'best');
saveas(gcf, fullfile(out_dir, 'fig6_sdot_norms.png'));

%% Input-gain signs and controls
figure('Color', 'w', 'Position', [280 180 920 620]);
for ell = 1:m
    subplot(m, 1, ell); hold on; grid on;
    for i = 1:N
        plot(t, squeeze(Bdiag(:, i, ell)), 'LineWidth', 1.0);
    end
    ylabel(sprintf('$b_{i%d}$', ell), 'Interpreter', 'latex');
end
xlabel('Time (s)');
saveas(gcf, fullfile(out_dir, 'fig7_input_gains.png'));

figure('Color', 'w', 'Position', [310 210 920 620]);
for ell = 1:m
    subplot(m, 1, ell); hold on; grid on;
    for i = 1:N
        plot(t, squeeze(u(:, i, ell)), 'LineWidth', 0.9);
    end
    ylabel(sprintf('$u_{i%d}$', ell), 'Interpreter', 'latex');
end
xlabel('Time (s)');
saveas(gcf, fullfile(out_dir, 'fig8_controls.png'));

end
