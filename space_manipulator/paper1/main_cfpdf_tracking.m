% main_cfpdf_tracking.m
% 指令滤波 + 周期延迟反馈 + PTDO 的 7-DOF 自由漂浮空间机械臂轨迹跟踪
% 对应论文《基于指令滤波与周期延迟反馈的自由漂浮空间机械臂预设时间轨迹跟踪控制》
clear; clc; close all;

%% 参数
n  = 7;                 % 关节数
Ts = 1e-4;              % 采样步长
Tf = 10;                % 总仿真时间
t  = 0:Ts:Tf; N = numel(t);

% 控制器参数
a   = 1.0;
tau2= 5/7;
Tsp = 4;                % 预设收敛时间（误差通道）
h   = Tsp/4;            % PDF 半周期
om  = 50;               % 滤波器带宽
rho = 0.5;              % 残余扰动上界
To  = 1;                % 观测器预设时间

% 期望轨迹（五次多项式插值）
q0 = [0; pi/3; 0; pi/4; pi/4; 0; pi/6];
qf = [pi/18; pi/6; pi/5; 0; -pi/4; -pi/3; pi/12];
[qd, dqd, ddqd] = quintic_traj(q0, qf, 8, t);

% 初始状态
q  = zeros(n, N);  q(:,1)  = q0 + (pi/180)*[4;-3;3;-4;3;-2;2];
dq = zeros(n, N);  dq(:,1) = 0;
tau_h = zeros(n, N);

% PDF 增益（标量，各关节共用）
R  = @(th) (th>h & th<=2*h).*sin(pi*(th-h)/h).^4;
Wc = integral(@(s) exp(2*a*s).*R(s), h, 2*h);
W  = 1/Wc;
K_ah = @(th) R(th).*W.*exp(-a*(h-2*th));   % K_{(a,h)}(th)

% 状态（滤波器/补偿器/观测器）
a1c = zeros(n,1); da1c = zeros(n,1);
xi1 = zeros(n,1);
z1o = zeros(n,1); z2o = zeros(n,1);         % PTDO 状态
barxi0 = 0.1*ones(n,1);

% 历史缓存（PDF 延迟，h/Ts 步）
Nbuf = round(h/Ts);
buf_u1 = zeros(n, Nbuf);                    % upsilon1 历史
buf_u2 = zeros(n, Nbuf);                    % upsilon2 历史

% 简化的名义惯性/科氏（正定对角 + 耦合项）
Me = @(q) diag([12;10;10;8;8;6;6]) + 0.5*diag(sin(q).^2+1.1);
Ce = @(q,dq) diag(0.3*cos(q).*dq);

% 扰动
d = @(t) 0.15*sin(0.7*t + 0.3*(1:n)') + 0.05*cos(1.3*t + 0.2*(1:n)');

for k = 1:N-1
    tt = t(k);
    % 当前状态
    qk = q(:,k); dqk = dq(:,k);

    % ---- PTDO ----
    eps1 = dqk - z1o - barxi0*(To-tt)^2*(tt<To);
    uo = Me(qk)\(tau_h(:,k) - Ce(qk,dqk)*dqk);
    z1o_d = z2o + (pi/(0.3*To))*sigvec(eps1/0.4, 1-0.15) ...
            + (pi/(0.3*To))*sigvec(eps1/0.4, 1+0.15) ...
            + 2*barxi0*(To-tt)*(tt<To) + barxi0*(To-tt)^2*(tt<To) + uo;
    z2o_d = (pi/(0.4*0.3*To))*sigvec(eps1/0.4, 2-0.3) ...
            + (pi/(0.4*0.3*To))*sigvec(eps1/0.4, 2+0.3) ...
            + (pi/(0.3*To))*sign(eps1/0.4) + 2*barxi0*(To-tt)*(tt<To);
    z1o = z1o + Ts*z1o_d;
    z2o = z2o + Ts*z2o_d;
    dhat = Me(qk)*z2o;

    % ---- PDF 局部时间 ----
    th = tt - To;

    % ---- Step 1 ----
    e  = qk - qd(:,k);
    u1 = e - xi1;                        % upsilon1
    u1h = buf_u1(:, mod(k-1,Nbuf)+1);    % upsilon1(th-h)
    K1 = (th>2*h)*K_ah(th-2*h);          % 标量
    a1 = dqd(:,k) - a*u1 - K1*u1h;       % 虚拟控制

    % 指令滤波器
    eta = a1c - a1;
    da1c = -om*eta;
    a1c  = a1c + Ts*da1c;

    % 补偿器
    xi1 = xi1 + Ts*(-a*xi1 + eta);       % xi2 ≡ 0

    % ---- Step 2 ----
    z2  = dqk - a1c;
    u2  = z2;                            % upsilon2 = z2
    u2h = buf_u2(:, mod(k-1,Nbuf)+1);
    K2  = K_ah(th)*(th>0);
    sig2 = sigvec(u2, 2*tau2-1).*abs(u2h).^(2*(1-tau2));

    nu = da1c - a/(2*(1-tau2))*u2 - rho*sign(u2) ...
         - K2/(2*(1-tau2))*sig2;

    % ---- 力矩 + 动力学 ----
    tau_h(:,k) = Me(qk)*(ddqd(:,k) + nu) + Ce(qk,dqk)*dqk - dhat;
    ddq = Me(qk)\(tau_h(:,k) + d(tt) - Ce(qk,dqk)*dqk);
    dq(:,k+1) = dqk + Ts*ddq;
    q(:,k+1)  = qk  + Ts*dq(:,k+1);

    % 更新缓存
    buf_u1(:, mod(k,Nbuf)+1) = u1;
    buf_u2(:, mod(k,Nbuf)+1) = u2;
end

err = q - qd; derr = dq - dqd;
figure;
subplot(211); plot(t, err'); ylabel('e (rad)'); grid on;
subplot(212); plot(t, derr'); ylabel('de (rad/s)'); xlabel('t (s)'); grid on;

function y = sigvec(x, p)
    y = sign(x).*abs(x).^p;
end

function [qd,dqd,ddqd] = quintic_traj(q0,qf,tf,t)
    % 五次多项式：s(0)=0,s(tf)=1,s'(0)=s'(tf)=s''(0)=s''(tf)=0
    s = 10*(t/tf).^3 - 15*(t/tf).^4 + 6*(t/tf).^5;
    ds = (30*(t/tf).^2 - 60*(t/tf).^3 + 30*(t/tf).^4)/tf;
    dds = (60*(t/tf) - 180*(t/tf).^2 + 120*(t/tf).^3)/tf^2;
    qd  = q0 + (qf-q0).*s;
    dqd = (qf-q0).*ds;
    ddqd= (qf-q0).*dds;
end
