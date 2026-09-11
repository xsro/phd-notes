% main_cfpdf_tracking.m
% 指令滤波 + 周期延迟反馈 + PTDO 的 7-DOF 自由漂浮空间机械臂轨迹跟踪
% 对应论文《基于指令滤波与周期延迟反馈的自由漂浮空间机械臂预设时间轨迹跟踪控制》
%
% 使用方法: 直接运行, 输出跟踪误差图和定量指标
%
% 说明:
%   - PTDO 在 t=T_o 内估计集总扰动, 实现动力学补偿
%   - 指令滤波反步法避免虚拟控制解析求导
%   - 周期延迟反馈 (PDF) 实现预设时间镇定
%   - 使用 tanh(100*u2) 近似 sign(u2) 避免抖振

clear; clc; close all;

%% 1. 参数设置
n  = 7;                 % 关节数
Ts = 1e-4;              % 采样步长
Tf = 10;                % 总仿真时间
t  = 0:Ts:Tf; N = numel(t);

% 控制器参数
a   = 1.0;              % 线性阻尼
tau2= 5/7;              % 幂分配参数
Tsp = 4;                % 预设收敛时间（误差通道）
h   = Tsp/4;            % PDF 半周期
om  = 50;               % 指令滤波器带宽
rho = 0.5;              % 残余扰动上界
To  = 1;                % PTDO 预设时间

% 期望轨迹（五次多项式插值，t_f=8s）
q0 = [0; pi/3; 0; pi/4; pi/4; 0; pi/6];
qf = [pi/18; pi/6; pi/5; 0; -pi/4; -pi/3; pi/12];
[qd, dqd, ddqd] = quintic_traj(q0, qf, 8, t);

% 初始状态（含初始位形误差 ~2-4 deg）
q  = zeros(n, N);  q(:,1)  = q0 + (pi/180)*[4;-3;3;-4;3;-2;2];
dq = zeros(n, N);  dq(:,1) = 0;
tau_h = zeros(n, N);

%% 2. PDF 增益
R   = @(th) (th>h & th<=2*h).*sin(pi.*(th-h)/h).^4;
Wc  = integral(@(s) exp(2*a*s).*R(s), h, 2*h);
K_ah = @(th) R(th).*(1/Wc).*exp(-a*(h-2*th));

%% 3. 状态初始化
a1c = zeros(n,1); da1c = zeros(n,1);   % 指令滤波状态
xi1 = zeros(n,1);                       % 补偿器 (xi2≡0)
z1o = zeros(n,1); z2o = zeros(n,1);    % PTDO 状态
barxi0 = 0.1*ones(n,1);                % PTDO 调节函数幅值

Nbuf = round(h/Ts);                    % 历史缓存长度
buf_u1 = zeros(n, Nbuf);
buf_u2 = zeros(n, Nbuf);

%% 4. 名义模型与扰动
Me = @(q) diag([12;10;10;8;8;6;6]) + 0.5*diag(sin(q).^2+1.1);
Ce = @(q,dq) diag(0.3*cos(q).*dq);
d_ext = @(t) 0.15*sin(0.7*t+0.3*(1:n)') + 0.05*cos(1.3*t+0.2*(1:n)');

%% 5. 主循环
fprintf('======== CF-PDF+PTDO 仿真 ========\n');
fprintf('Ts=%.1e, Tf=%.1fs, n=%d, To=%.1fs, Tsp=%.1fs\n\n', Ts, Tf, n, To, Tsp);

for k = 1:N-1
    tt = t(k); qk = q(:,k); dqk = dq(:,k);

    % ---- PTDO ----
    eps1 = dqk - z1o - barxi0*(To-tt)^2*(tt<To);
    uo = Me(qk)\(tau_h(:,k) - Ce(qk,dqk)*dqk);
    z1o_d = z2o + (pi/(0.3*To))*sigvec(eps1/0.4,1-0.15) ...
            + (pi/(0.3*To))*sigvec(eps1/0.4,1+0.15) ...
            + 2*barxi0*(To-tt)*(tt<To) + barxi0*(To-tt)^2*(tt<To) + uo;
    z2o_d = (pi/(0.4*0.3*To))*sigvec(eps1/0.4,2-0.3) ...
            + (pi/(0.4*0.3*To))*sigvec(eps1/0.4,2+0.3) ...
            + (pi/(0.3*To))*sign(eps1/0.4) + 2*barxi0*(To-tt)*(tt<To);
    z1o = z1o + Ts*z1o_d; z2o = z2o + Ts*z2o_d;
    dhat = Me(qk)*z2o;

    % ---- PDF 局部时间 ----
    th = tt - To;

    % ---- Step 1: 位置误差子通道 ----
    e  = qk - qd(:,k);           % z1 = e
    u1 = e - xi1;                % upsilon1 = z1 - xi1
    u1h = buf_u1(:, mod(k, Nbuf)+1);  % upsilon1(theta-h)
    K1 = (th>2*h)*K_ah(th-2*h);      % K1(theta) = K_{(a,h)}(theta-2h)
    a1 = dqd(:,k) - a*u1 - K1*u1h;   % 虚拟控制 (含前馈)

    % ---- 指令滤波器 ----
    eta = a1c - a1; da1c = -om*eta; a1c = a1c + Ts*da1c;

    % ---- 补偿器 ----
    xi1 = xi1 + Ts*(-a*xi1 + eta);

    % ---- Step 2: 速度误差子通道 ----
    z2  = dqk - a1c;             % z2 = dq - alpha1^c
    u2  = z2;                    % upsilon2 = z2 (xi2≡0)
    u2h = buf_u2(:, mod(k, Nbuf)+1);  % upsilon2(theta-h)
    K2  = K_ah(th)*(th>0);            % K2(theta) = K_{(a,h)}(theta)
    sig2 = sigvec(u2, 2*tau2-1).*abs(u2h).^(2*(1-tau2));
    nu = da1c - a/(2*(1-tau2))*u2 - rho*tanh(100*u2) - K2/(2*(1-tau2))*sig2;

    % ---- 力矩 + 正动力学 ----
    tau_h(:,k) = Me(qk)*(ddqd(:,k)+nu) + Ce(qk,dqk)*dqk - dhat;
    ddq = Me(qk)\(tau_h(:,k) + d_ext(tt) - Ce(qk,dqk)*dqk);
    dq(:,k+1) = dqk + Ts*ddq;
    q(:,k+1)  = qk  + Ts*dq(:,k+1);

    % ---- 更新缓存 ----
    buf_u1(:, mod(k, Nbuf)+1) = u1;
    buf_u2(:, mod(k, Nbuf)+1) = u2;
end

%% 6. 定量分析
err  = q - qd;  derr = dq - dqd;
err_norm  = sqrt(sum(err.^2,  1));
derr_norm = sqrt(sum(derr.^2, 1));
tau_max   = max(abs(tau_h), [], 1);

fprintf('=== 仿真结果 ===\n');
[~,i5]=min(abs(t-(To+Tsp)));
fprintf('t=%.0fs (T_o+T_s): ||e||_2=%.2e rad, ||de||_2=%.2e rad/s\n', ...
    To+Tsp, err_norm(i5), derr_norm(i5));
[~,i10]=min(abs(t-Tf));
fprintf('t=%.0fs (终端): ||e||_2=%.2e rad, ||de||_2=%.2e rad/s\n', ...
    Tf, err_norm(i10), derr_norm(i10));
[~,i8]=min(abs(t-8));
fprintf('t=8s (轨迹结束): ||e||_2=%.2e rad, ||de||_2=%.2e rad/s\n', ...
    err_norm(i8), derr_norm(i8));
fprintf('max torque: %.1f N-m\n', max(tau_max));

%% 7. 绘图
figure('Position', [50, 50, 1000, 600]);
subplot(2,1,1);
plot(t, err', 'LineWidth', 0.8);
ylabel('e (rad)'); grid on;
title('关节位置跟踪误差'); xlim([0 Tf]);
xline(To, 'r--', 'T_o'); xline(To+Tsp, 'g--', 'T_o+T_s');
subplot(2,1,2);
plot(t, derr', 'LineWidth', 0.8);
ylabel('de (rad/s)'); xlabel('t (s)'); grid on;
title('关节速度跟踪误差'); xlim([0 Tf]);
xline(To, 'r--', 'T_o'); xline(To+2*h, 'm--', 'T_o+2h');
xline(To+Tsp, 'g--', 'T_o+T_s');

%% 辅助函数
function y = sigvec(x, p)
    y = sign(x) .* abs(x).^p;
end

function [qd, dqd, ddqd] = quintic_traj(q0, qf, tf, t)
    tau = t / tf;
    s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    ds  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / tf;
    dds = (60*tau - 180*tau.^2 + 120*tau.^3) / tf^2;
    s(t>tf)   = 1;  ds(t>tf)  = 0;  dds(t>tf) = 0;
    qd   = q0 + (qf-q0).*s;
    dqd  = (qf-q0).*ds;
    ddqd = (qf-q0).*dds;
end