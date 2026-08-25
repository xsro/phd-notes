function render(t, x, params, ip, outdir, tend)
%RENDER 绘制运动轨迹图
%   render(t, x, params, ip, outdir, tend)
%   t: 时间向量
%   x: 状态矩阵，每行包含 [target(4); agent(8*N)]
%   params: 参数结构体
%   ip: 仿真编号 (1 或 2)
%   outdir: 输出目录
%   tend: 绘制截止时间，默认为 115

if nargin < 6 || isempty(tend)
    tend = 115;
end

N = params.N;
colors = lines(N);

% 截取到 tend
idx = t <= tend;
t = t(idx);
x = x(idx, :);

figure('Position', [100, 100, 700, 600]);
hold on;

% 绘制目标轨迹
plot(x(:,1), x(:,2), 'k-', 'LineWidth', 1.5, 'DisplayName', 'target');

% 绘制智能体轨迹
for a = 1:N
    step = 8*(a-1);
    plot(x(:, 5+step), x(:, 6+step), '-', ...
        'Color', colors(a,:), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('agent %d', a));
end

% 标记起点
plot(x(1,1), x(1,2), 'ks', 'MarkerFaceColor', 'black', 'MarkerSize', 10, ...
    'HandleVisibility', 'off');
for a = 1:N
    step = 8*(a-1);
    plot(x(1, 5+step), x(1, 6+step), 'o', ...
        'Color', colors(a,:), 'MarkerFaceColor', colors(a,:), 'MarkerSize', 8, ...
        'HandleVisibility', 'off');
end

% 标记终点
plot(x(end,1), x(end,2), 'k^', 'MarkerFaceColor', 'black', 'MarkerSize', 12, ...
    'DisplayName', 'target (final)');
for a = 1:N
    step = 8*(a-1);
    plot(x(end, 5+step), x(end, 6+step), '^', ...
        'Color', colors(a,:), 'MarkerFaceColor', colors(a,:), 'MarkerSize', 10, ...
        'DisplayName', sprintf('agent %d (final)', a));
end

hold off;
axis equal;
grid on;
xlabel('$x_1$', 'Interpreter', 'latex');
ylabel('$x_2$', 'Interpreter', 'latex');
title(sprintf('Trajectories (case %d)', ip), 'Interpreter', 'latex');
legend('Location', 'bestoutside');

exportgraphics(gcf, fullfile(outdir, sprintf('%dtraj.pdf', ip)));
end
