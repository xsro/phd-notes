% run_cfpdf_sim.m
% 仿真主程序：运行 CF-PDF+PTDO 仿真并保存结果到数据文件
% 对应论文《基于指令滤波与周期延迟反馈的自由漂浮空间机械臂预设时间轨迹跟踪控制》
% 
% 输出: sim_data.mat (包含所有仿真数据)
%
% 说明：
%   PTDO 在离散时间 Euler 积分下收敛较慢，需较长的仿真时间才能稳定。
%   控制器的预设时间收敛效果在 t=T_o+T_s=5s 时已清晰可见。

clear; clc; close all;

%% 1. 参数设置
n  = 7;                 % 关节数
Ts = 1e-4;              % 采样步长
Tf = 10;                % 总仿真时间
t  = 0:Ts:Tf; N = numel(t);

% 控制器参数
a   = 1.0;
tau2= 5/7;
Tsp = 4;                % 预设收敛时间（误差通道）
h   = Tsp/4;            % PDF 半周期
om  = 50;               % 指令滤波器带宽
rho = 0.5;              % 残余扰动上界
To  = 1;                % PTDO 预设时间

% 期望轨迹（五次多项式插值，t_f=8s）
q0 = [0; pi/3; 0; pi/4; pi/4; 0; pi/6];
qf = [pi/18; pi/6; pi/5; 0; -pi/4; -pi/3; pi/12];
[qd, dqd, ddqd] = quintic_traj(q0, qf, 8, t);

% 初始状态（含初始位形误差）
q  = zeros(n, N);  q(:,1)  = q0 + (pi/180)*[4;-3;3;-4;3;-2;2];
dq = zeros(n, N);  dq(:,1) = 0;
tau_h = zeros(n, N);

%% 2. PDF 增益（标量，各关节共用）
% R_h(t) = sin^4(pi*(t-h)/h), t∈[h,2h]
R  = @(th) (th>h & th<=2*h).*sin(pi.*(th-h)/h).^4;
Wc = integral(@(s) exp(2*a*s).*R(s), h, 2*h);
W  = 1/Wc;
% K_{(a,h)}(t) = R_h(t) * W * exp(-a*(h-2t)), t≥0
K_ah = @(th) R(th).*W.*exp(-a*(h-2*th));

% 平滑符号近似（避免 sign 抖振）
sign_eps = 1e-3;
sat = @(x) min(max(x/sign_eps, -1), 1);

%% 3. 状态初始化
% 指令滤波器
a1c = zeros(n,1); da1c = zeros(n,1);
% 误差补偿器
xi1 = zeros(n,1);
% PTDO 状态
z1o = zeros(n,1); z2o = zeros(n,1);
barxi0 = 0.1*ones(n,1);  % PTDO 调节函数幅值

% 历史缓存（环形缓冲，长度 = h/Ts 采样点）
Nbuf = round(h/Ts);
buf_u1 = zeros(n, Nbuf);  % upsilon1 历史
buf_u2 = zeros(n, Nbuf);  % upsilon2 历史

%% 4. 名义模型与扰动
% 简化的名义惯性/科氏（正定对角 + 耦合项）
Me = @(q) diag([12;10;10;8;8;6;6]) + 0.5*diag(sin(q).^2+1.1);
Ce = @(q,dq) diag(0.3*cos(q).*dq);

% 加性扰动（关节空间）
d_ext = @(t) 0.15*sin(0.7*t + 0.3*(1:n)') + 0.05*cos(1.3*t + 0.2*(1:n)');

%% 5. 主循环
fprintf('======== CF-PDF+PTDO 仿真 ========\n');
fprintf('Ts = %.1e, Tf = %.1f s, N = %d\n', Ts, Tf, N);
fprintf('n = %d DOF, To = %.1f s, Tsp = %.1f s, h = %.3f s\n', n, To, Tsp, h);
fprintf('a = %.1f, tau2 = %.3f, om = %.0f, rho = %.1f\n', a, tau2, om, rho);
fprintf('===================================\n\n');

tic;
for k = 1:N-1
    tt = t(k);
    % 当前状态
    qk = q(:,k); dqk = dq(:,k);

    % ---- 5a. PTDO（预设时间扰动观测器）----
    % 可测复合误差: eps1 = chi - p1 - xi_tilde(t)
    eps1 = dqk - z1o - barxi0*(To-tt)^2*(tt<To);
    % 名义加速度输入
    uo = Me(qk)\(tau_h(:,k) - Ce(qk,dqk)*dqk);
    % dp1/dt
    z1o_d = z2o ...
            + (pi/(0.3*To))*sigvec(eps1/0.4, 1-0.15) ...
            + (pi/(0.3*To))*sigvec(eps1/0.4, 1+0.15) ...
            + 2*barxi0*(To-tt)*(tt<To) + barxi0*(To-tt)^2*(tt<To) ...
            + uo;
    % dp2/dt
    z2o_d = (pi/(0.4*0.3*To))*sigvec(eps1/0.4, 2-0.3) ...
            + (pi/(0.4*0.3*To))*sigvec(eps1/0.4, 2+0.3) ...
            + (pi/(0.3*To))*sign(eps1/0.4) ...
            + 2*barxi0*(To-tt)*(tt<To);
    % Euler 积分
    z1o = z1o + Ts*z1o_d;
    z2o = z2o + Ts*z2o_d;
    % 力矩扰动估计: dhat = Me(qm) * p2
    dhat = Me(qk)*z2o;

    % ---- 5b. PDF 局部时间 ----
    th = tt - To;  % theta = t - T_o

    % ---- 5c. Step 1: 位置误差子通道 ----
    e  = qk - qd(:,k);          % z1 = e (跟踪误差)
    u1 = e - xi1;               % upsilon1 = z1 - xi1
    % 环形缓冲：先读后写，mod(k, Nbuf)+1 为 h 秒前写入的位置
    u1h = buf_u1(:, mod(k, Nbuf)+1);  % upsilon1(theta-h)
    K1 = (th>2*h)*K_ah(th-2*h);      % K1(theta) = K(a,h)(theta-2h)
    a1 = dqd(:,k) - a*u1 - K1*u1h;   % 虚拟控制（包含 dqd 前馈）

    % ---- 5d. 指令滤波器（一阶低通）----
    eta_flt = a1c - a1;
    da1c = -om*eta_flt;
    a1c  = a1c + Ts*da1c;

    % ---- 5e. 误差补偿系统 ----
    xi1 = xi1 + Ts*(-a*xi1 + eta_flt);  % xi2 ≡ 0

    % ---- 5f. Step 2: 速度误差子通道 ----
    z2  = dqk - a1c;            % z2 = dq - alpha1^c
    u2  = z2;                   % upsilon2 = z2 (因 xi2 ≡ 0)
    u2h = buf_u2(:, mod(k, Nbuf)+1);  % upsilon2(theta-h)
    K2  = K_ah(th)*(th>0);            % K2(theta) = K(a,h)(theta)
    % 幂分配项: sig^{2tau2-1}(u2) * |u2h|^{2(1-tau2)}
    sig2 = sigvec(u2, 2*tau2-1) .* abs(u2h).^(2*(1-tau2));
    % 辅助加速度控制
    nu = da1c ...
         - a/(2*(1-tau2))*u2 ...
         - rho*tanh(100*u2) ...         % smooth sign
         - K2/(2*(1-tau2))*sig2;

    % ---- 5g. 关节力矩 + 正动力学 ----
    tau_h(:,k) = Me(qk)*(ddqd(:,k) + nu) + Ce(qk,dqk)*dqk - dhat;
    % 实际加速度（含真实扰动）
    ddq = Me(qk)\(tau_h(:,k) + d_ext(tt) - Ce(qk,dqk)*dqk);
    % Euler 积分
    dq(:,k+1) = dqk + Ts*ddq;
    q(:,k+1)  = qk  + Ts*dq(:,k+1);

    % ---- 5h. 更新历史缓存 ----
    buf_u1(:, mod(k, Nbuf)+1) = u1;
    buf_u2(:, mod(k, Nbuf)+1) = u2;

    % 进度显示
    if mod(k, round(N/10)) == 0
        fprintf('  进度: %.0f%% (t = %.1f s)\n', 100*k/N, tt);
    end
end
toc;

%% 6. 误差计算与定量指标
err  = q - qd;
derr = dq - dqd;
err_norm  = sqrt(sum(err.^2,  1));
derr_norm = sqrt(sum(derr.^2, 1));
tau_max   = max(abs(tau_h), [], 1);

%% 7. 输出结果摘要
fprintf('\n======== 仿真结果摘要 ========\n');

fprintf('\n[1] 预设时间点的误差:\n');
t_check = [To, To+2*h, To+Tsp];  % 1s, 3s, 5s
labels  = {'T_o=1s','T_o+2h=3s','T_o+T_s=5s'};
for i = 1:3
    [~, idx] = min(abs(t - t_check(i)));
    fprintf('  %-12s: ||e||_2=%.2e rad, ||de||_2=%.2e rad/s\n', ...
        labels{i}, err_norm(idx), derr_norm(idx));
end

fprintf('\n[2] 轨迹跟踪结束 (t=8s) 时的误差:\n');
[~, idx8] = min(abs(t - 8));
fprintf('  t=8s       : ||e||_2=%.2e rad, ||de||_2=%.2e rad/s\n', ...
    err_norm(idx8), derr_norm(idx8));

fprintf('\n[3] 终端误差 (t=%.1fs):\n', Tf);
fprintf('  ||e||_2     = %.2e rad\n', err_norm(end));
fprintf('  ||de||_2    = %.2e rad/s\n', derr_norm(end));
fprintf('  max|e_i|    = %.2e rad\n', max(abs(err(:,end))));
fprintf('  max|de_i|   = %.2e rad/s\n', max(abs(derr(:,end))));

fprintf('\n[4] 关节力矩统计:\n');
fprintf('  max|tau_i|  = %.2f N·m\n', max(tau_max));
fprintf('  RMS(tau)    = %.3f N·m\n', sqrt(mean(tau_h.^2, 'all')));

fprintf('\n[5] 稳态 (t>=6s) 误差包络:\n');
[~, idx6] = min(abs(t - 6));
fprintf('  max||e||_2  = %.2e rad\n', max(err_norm(idx6:end)));
fprintf('  max||de||_2 = %.2e rad/s\n', max(derr_norm(idx6:end)));

%% 8. 保存数据
outdir = '/tmp/sim_results';
mkdir(outdir);
save(fullfile(outdir, 'sim_data.mat'), ...
    't', 'err', 'derr', 'tau_h', 'qd', 'dqd', 'ddqd', ...
    'err_norm', 'derr_norm', 'tau_max', ...
    'a', 'tau2', 'Tsp', 'h', 'om', 'rho', 'To', 'Ts', 'Tf', 'n');
fprintf('\n结果已保存到 %s/sim_data.mat\n', outdir);

%% 辅助函数
function y = sigvec(x, p)
    % sig^p(x) = |x|^p * sign(x), element-wise
    y = sign(x) .* abs(x).^p;
end

function [qd, dqd, ddqd] = quintic_traj(q0, qf, tf, t)
    % 五次多项式轨迹规划
    % 边界条件: s(0)=0, s(tf)=1, s'(0)=s'(tf)=s''(0)=s''(tf)=0
    tau = t / tf;
    s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    ds  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / tf;
    dds = (60*tau - 180*tau.^2 + 120*tau.^3) / tf^2;
    % 对 t > tf 保持常值
    s(t>tf)   = 1;
    ds(t>tf)  = 0;
    dds(t>tf) = 0;
    % 扩展维度
    qd   = q0 + (qf-q0).*s;
    dqd  = (qf-q0).*ds;
    ddqd = (qf-q0).*dds;
end