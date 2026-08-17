function plot_results(hist, par)

t = par.t;
Nt = par.Nt;
N = par.N;
m = par.m;

% Ensure output directory exists
out_dir = fullfile(fileparts(mfilename('fullpath')), 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

p  = hist.p;
v  = hist.v;
p0 = hist.p0;
v0 = hist.v0;
u  = hist.u;
s  = hist.s;
sdot = hist.sdot;
Bdiag = hist.Bdiag;

%% Extract relative velocity norms
vel_err = zeros(Nt,N);
s_norm = zeros(Nt,N);
sdot_norm = zeros(Nt,N);
dist_min = zeros(Nt,1);
dist_center_to_target = zeros(Nt,1);

for k = 1:Nt
    p_k = squeeze(p(k,:,:));
    v_k = squeeze(v(k,:,:));
    p0_k = p0(k,:);
    v0_k = v0(k,:);

    for i = 1:N
        vel_err(k,i) = norm(v_k(i,:) - v0_k);
        s_norm(k,i) = norm(squeeze(s(k,i,:)));
        sdot_norm(k,i) = norm(squeeze(sdot(k,i,:)));
    end

    % minimum inter-agent distance
    dmin = inf;
    for i = 1:N
        for j = i+1:N
            dij = norm(p_k(i,:) - p_k(j,:));
            dmin = min(dmin, dij);
        end
    end
    dist_min(k) = dmin;

    % center of all agents to target distance
    p_center = mean(p_k, 1);
    dist_center_to_target(k) = norm(p_center - p0_k);
end

%% Figure 1: trajectories
figure('Color','w','Position',[100 100 850 700]);
hold on; grid on; axis equal;

colors = lines(N);

for i = 1:N
    pi_hist = squeeze(p(:,i,:));
    plot(pi_hist(:,1), pi_hist(:,2), 'LineWidth', 1.5, 'Color', colors(i,:));
    plot(pi_hist(1,1), pi_hist(1,2), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
    plot(pi_hist(end,1), pi_hist(end,2), 's', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
end

plot(p0(:,1), p0(:,2), 'k--', 'LineWidth', 2.0);
plot(p0(1,1), p0(1,2), 'ko', 'MarkerFaceColor','k');
plot(p0(end,1), p0(end,2), 'ks', 'MarkerFaceColor','k');

xlabel('$x$','Interpreter','latex');
ylabel('$y$','Interpreter','latex');
title('Agent and target trajectories','Interpreter','latex');
legend_entries = cell(N+1,1);
for i = 1:N
    legend_entries{i} = ['Agent ', num2str(i)];
end
legend_entries{N+1} = 'Target';
legend(legend_entries, 'Location','best');
saveas(gcf, fullfile(out_dir, 'fig1_trajectories.png'));

%% Figure 2: final convex hull
figure('Color','w','Position',[150 150 750 650]);
hold on; grid on; axis equal;

p_final = squeeze(p(end,:,:));
p0_final = p0(end,:);

K = convhull(p_final(:,1), p_final(:,2));
fill(p_final(K,1), p_final(K,2), [0.8 0.9 1.0], ...
    'FaceAlpha',0.4, 'EdgeColor','b', 'LineWidth',1.5);

plot(p_final(:,1), p_final(:,2), 'ro', 'MarkerFaceColor','r', 'MarkerSize',8);
plot(p0_final(1), p0_final(2), 'kp', 'MarkerFaceColor','k', 'MarkerSize',12);

xlabel('$x$','Interpreter','latex');
ylabel('$y$','Interpreter','latex');
title('Final convex hull and target','Interpreter','latex');
legend('Convex hull','Agents','Target','Location','best');
saveas(gcf, fullfile(out_dir, 'fig2_convex_hull.png'));

%% Figure 3: sliding variables
figure('Color','w','Position',[200 100 900 700]);

for ell = 1:m
    subplot(m,1,ell); hold on; grid on;
    for i = 1:N
        plot(t, squeeze(s(:,i,ell)), 'LineWidth', 1.2);
    end
    ylabel(['$s_{i0,',num2str(ell),'}$'], 'Interpreter','latex');
    title(['Sliding variable component ', num2str(ell)], 'Interpreter','latex');

    % Draw periodic surfaces for visual reference
    eps_ref = par.eps_s(1,ell);
    yline(0,'k--');
    yline(eps_ref,'r--');
    yline(-eps_ref,'r--');
    yline(2*eps_ref,'g--');
    yline(-2*eps_ref,'g--');
end
xlabel('Time (s)');
saveas(gcf, fullfile(out_dir, 'fig3_sliding_variables.png'));

%% Figure 4: norm of sdot
figure('Color','w','Position',[250 150 900 450]);
hold on; grid on;
for i = 1:N
    plot(t, sdot_norm(:,i), 'LineWidth', 1.2);
end
xlabel('Time (s)');
ylabel('$\|\dot{s}_{i0}\|$','Interpreter','latex');
title('Norm of sliding variable derivative','Interpreter','latex');
legend(arrayfun(@(i) ['Agent ',num2str(i)], 1:N, 'UniformOutput', false));
saveas(gcf, fullfile(out_dir, 'fig4_sdot_norm.png'));

%% Figure 5: velocity tracking error
figure('Color','w','Position',[300 200 900 450]);
hold on; grid on;
for i = 1:N
    plot(t, vel_err(:,i), 'LineWidth', 1.2);
end
xlabel('Time (s)');
ylabel('$\|v_i-v_0\|$','Interpreter','latex');
title('Relative velocity error','Interpreter','latex');
legend(arrayfun(@(i) ['Agent ',num2str(i)], 1:N, 'UniformOutput', false));
saveas(gcf, fullfile(out_dir, 'fig5_velocity_error.png'));

%% Figure 6: minimum inter-agent distance
figure('Color','w','Position',[350 250 900 450]);
plot(t, dist_min, 'LineWidth', 1.5); hold on; grid on;
yline(par.d_safe, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Minimum inter-agent distance');
title('Collision avoidance distance','Interpreter','latex');
legend('min distance','$d_{\rm safe}$','Interpreter','latex');
saveas(gcf, fullfile(out_dir, 'fig6_min_distance.png'));

%% Figure 7: input gain signs
figure('Color','w','Position',[400 300 900 650]);

for ell = 1:m
    subplot(m,1,ell); hold on; grid on;
    for i = 1:N
        plot(t, squeeze(Bdiag(:,i,ell)), 'LineWidth', 1.2);
    end
    ylabel(['$b_{i',num2str(ell),'}(t)$'], 'Interpreter','latex');
    title(['Unknown sign-switching input gain component ', num2str(ell)], 'Interpreter','latex');
end
xlabel('Time (s)');
saveas(gcf, fullfile(out_dir, 'fig7_input_gain_signs.png'));

%% Figure 8: controls
figure('Color','w','Position',[450 350 900 650]);

for ell = 1:m
    subplot(m,1,ell); hold on; grid on;
    for i = 1:N
        plot(t, squeeze(u(:,i,ell)), 'LineWidth', 1.0);
    end
    ylabel(['$u_{i',num2str(ell),'}$'], 'Interpreter','latex');
    title(['Control input component ', num2str(ell)], 'Interpreter','latex');
end
xlabel('Time (s)');
saveas(gcf, fullfile(out_dir, 'fig8_controls.png'));

%% Figure 9: center of agents to target distance
figure('Color','w','Position',[500 400 900 450]);
plot(t, dist_center_to_target, 'b-', 'LineWidth', 1.5); hold on; grid on;
xlabel('Time (s)');
ylabel('$\|\bar{p}(t)-p_0(t)\|$','Interpreter','latex');
title('Distance from agent center to target','Interpreter','latex');
legend('$\|\bar{p}-p_0\|$','Interpreter','latex');
saveas(gcf, fullfile(out_dir, 'fig9_center_to_target.png'));

end