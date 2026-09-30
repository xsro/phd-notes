%% main_ffsm_pdf_tracking_yan_params.m
% -------------------------------------------------------------------------
% 自由漂浮空间机械臂光滑周期延迟反馈轨迹跟踪控制仿真
% 对应论文: 基于光滑周期延迟反馈的自由漂浮空间机械臂预设时间轨迹跟踪控制
%
% 算法结构 (论文第4节):
%   1. 预设时间扰动观测器 PTDO  (Eq (17)~(22))
%   2. 动力学补偿与误差通道规范化  (Eq (23)~(26))
%   3. 光滑周期延迟反馈 (PDF)  (Eq (27)~(33))
%
% 控制律 (Eq (33)):
%   tau = M_e(qm) * [qdd_d + K0*z(t) - Kc(t)*z(t-h)]
%         + C_e(qm,qd_m)*qd_m - d_hat(t)
%
% 物理参数: 严宇新论文表2-1 七自由度空间机械臂参数
% 名义模型: 基于表2-1参数构造的正定名义等效关节空间模型
%   (完整 Schur 补 M_e = H_m - H_bm'*H_b^{-1}*H_bm 需递推动力学实现)
%
% 仿真算例 (论文第6节, 表1):
%   1. PD, 无扰动
%   2. PDF, 无扰动
%   3. PD, 有扰动
%   4. PDF, 有扰动
%   5. PDF+PTDO, 有扰动
% -------------------------------------------------------------------------

clear; clc; close all;

%% -------------------- 1. Simulation settings ----------------------------
cfg.Ts = 1e-3;             % integration step [s]  (论文: T_s = 10^{-3} s)
cfg.Tf = 10.0;             % final simulation time [s]
cfg.t = 0:cfg.Ts:cfg.Tf;
cfg.N = numel(cfg.t);
cfg.n = 7;

cfg.save_figures = true;
cfg.show_figures = false;
cfg.save_results = true;

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
cfg.figure_dir = fullfile(script_dir, '..', 'figures');
cfg.result_file = fullfile(script_dir, 'latest_results.mat');
if cfg.save_figures && ~exist(cfg.figure_dir, 'dir')
    mkdir(cfg.figure_dir);
end
if ~cfg.show_figures
    set(0, 'DefaultFigureVisible', 'off');
end

%% -------------------- 2. Parameters from Yan Yuxin Table 2-1 ------------
P = yan_yuxin_parameters();

%% -------------------- 3. Reference trajectory and initial state ----------
% Reference initial joint angles in Section 3.7:
% q_r(0) = [0, pi/3, 0, pi/4, pi/4, 0, pi/6]^T rad
q_ref0 = [0; pi/3; 0; pi/4; pi/4; 0; pi/6];
q_ref_f = [pi/18; pi/6; pi/5; 0; -pi/4; -pi/3; pi/12];
cfg.Tref = 8.0;
[ref.qd, ref.dqd, ref.ddqd] = quintic_joint_trajectory(q_ref0, q_ref_f, cfg.Tref, cfg.t);

% Nonzero initial tracking error makes the convergence mechanism visible.
% The perturbation is moderate and keeps the initial pose close to the
% tabulated thesis configuration.
initial_error_deg = [4; -3; 3; -4; 3; -2; 2];
cfg.q_init = q_ref0 + deg2rad(initial_error_deg);
cfg.dq_init = zeros(cfg.n, 1);

%% -------------------- 4. Controller parameters --------------------------
% 人工延迟 h = 1s  (论文第6节)
cfg.h = 1.0;                         % artificial delay [s]
cfg.Nh = round(cfg.h/cfg.Ts);         % delay samples
cfg.tau_max = 200 * ones(cfg.n, 1);   % [N m], set Inf for no saturation

% 基准镇定增益 K0 = [-Kp, -Kd]  (论文第6节)
Kp = diag([25 25 25 22 22 20 20]);
Kd = diag([10 10 10  9  9  8  8]);
ctrl.K0 = [-Kp, -Kd];

% --- Gramian 型 PDF 增益 (论文 Eq (31)~(32)) ---
% Kc(t) = R_h(t) * B' * exp(-Ac'*t) * Wc^{-1} * exp(Ac*(h-t))
% Wc = integral_{h}^{2h} exp(-Ac*s)*B*Rh(s)*B'*exp(-Ac'*s) ds
% 离线积分构造 Wc, 在线查表 Kc(mod(t,2h))
ctrl.pdf = build_pdf_gain(ctrl.K0, cfg.n, cfg.h);

% 工程化 PDF (消融/调试用, 非论文理论增益)
Kp_h = diag([8 8 8 6 6 5 5]);
Kd_h = diag([3 3 3 2.5 2.5 2 2]);
ctrl.engineering_Kh = [-Kp_h, -Kd_h];
ctrl.engineering_gain = 1.0;

% --- 预设时间扰动观测器 PTDO (论文 Eq (20)) ---
% 观测通道: dot{chi} = u_o + Delta_a   (Eq (17))
% 观测误差: eps1 = chi - z1 - xi_bar(t)   (Eq (19))
obs.enabled = false;
obs.T_o = 1.0;                 % 预设观测时间 [s]  (论文第6节)
obs.T_c = 1.0;                 % 观测器内部收敛参数 [s]
obs.eta = 0.3;                 % eta in (0,1)  (论文第6节)
obs.sigma = 0.4;               % sigma > 0
obs.xi0 = zeros(cfg.n, 1);
obs.use_two_stage_pdf = true;  % 两阶段时序: 先观测后PDF
ctrl.observer = obs;

%% -------------------- 5. Run cases (论文表1 五组算例) ---------------------
cases(1) = make_case("PD, nominal", "pd", false, false);
cases(2) = make_case("PDF, nominal", "pdf_exact", false, false);
cases(3) = make_case("PD, disturbed", "pd", true, false);
cases(4) = make_case("PDF, disturbed", "pdf_exact", true, false);
cases(5) = make_case("PDF+PTDO, disturbed", "pdf_exact", true, true);

results = repmat(empty_result(), 1, numel(cases));
for c = 1:numel(cases)
    results(c) = run_case(cases(c), cfg, ref, ctrl, P);
end

metrics = collect_metrics(results);
if cfg.save_results
    save(cfg.result_file, 'cfg', 'ctrl', 'cases', 'results', 'metrics', 'ref', 'P');
end

%% -------------------- 6. Print summary ----------------------------------
fprintf('\nSimulation summary\n');
fprintf('Case                     ||e(T)|| [rad]   ||edot(T)|| [rad/s]   ');
fprintf('RMS(e) [rad]    max|tau_i| [N m]   t_settle [s]   ||obs err||\n');
for c = 1:numel(results)
    m = results(c).metrics;
    fprintf('%-24s %.6e       %.6e       %.6e    %.6f       %.3f        %.6e\n', ...
        results(c).name, m.final_e_norm, m.final_edot_norm, ...
        m.rms_e, m.max_tau, m.settle_time_e, m.final_obs_error_norm);
end
fprintf('PDF Gramian condition number: %.3e\n', ctrl.pdf.gramian_condition);
if cfg.save_results
    fprintf('Numerical results saved to: %s\n', cfg.result_file);
    fprintf('Run plot_ffsm_pdf_tracking_results.m to regenerate figures.\n');
end

%% ========================================================================
%                           Local functions
% ========================================================================

function P = yan_yuxin_parameters()
% Parameters from Yan Yuxin thesis, Table 2-1.
% Bodies: B0 base + B1...B7 links.

    P.n = 7;

    % Mass [kg]
    P.mass = [1000, 4.25, 7, 7, 4.25, 4.25, 4.25, 4.25];

    % ^i b_i [m], columns correspond to B0...B7
    P.b = [ ...
        0.6, 0.6, 1.5, 1.5, 0,    0,   0,   0.3;
        0,   0,   0,   0,  -0.5,  0,   0,   0;
        0,   0,   0.6, 0.6, 0,    0.5, 0.5, 0 ];

    % ^i a_i [m], columns correspond to B0...B7
    P.a = [ ...
        0,   0.6, 1.5, 1.5, 0,    0,   0,   0.3;
        0,   0,   0,   0,  -0.5,  0,   0,   0;
        0,   0,   0.6, 0.6, 0,    0.5, 0.5, 0 ];

    % Inertia matrices ^i I_i [kg*m^2], columns B0...B7
    Ixx = [72, 0.05, 0.09, 0.09, 0.05, 0.05, 0.05, 0.021];
    Iyy = [72, 1.28, 1.46, 1.46, 0.89, 0.89, 0.89, 0.53 ];
    Izz = [72, 1.28, 1.46, 1.46, 0.89, 0.89, 0.89, 0.53 ];
    Ixy = zeros(1,8);
    Ixz = zeros(1,8);
    Iyz = zeros(1,8);

    P.I = cell(1,8);
    for i = 1:8
        P.I{i} = [ Ixx(i), Ixy(i), Ixz(i);
                   Ixy(i), Iyy(i), Iyz(i);
                   Ixz(i), Iyz(i), Izz(i) ];
    end

    P.mi = P.mass(2:end);
    P.Ilink = P.I(2:end);
    P.Jdiag = nominal_joint_inertia_diagonal(P);
end

function Jdiag = nominal_joint_inertia_diagonal(P)
% Precompute the diagonal inertia scale used by the nominal model.

    n = P.n;
    Jdiag = zeros(n, 1);
    for i = 1:n
        mi = P.mi(i);
        Ii = P.Ilink{i};
        ai = P.a(:, i+1);
        bi = P.b(:, i+1);
        len2 = max(norm(ai)^2 + norm(bi)^2, 1e-3);
        Jdiag(i) = Ii(3,3) + 0.15*mi*len2 + 0.05;
    end
end

function [qd, dqd, ddqd] = quintic_joint_trajectory(q0, qf, Tref, t)
% Vectorized fifth-order polynomial trajectory with zero endpoint velocity
% and acceleration.

    s = min(t./Tref, 1);
    phi = 10*s.^3 - 15*s.^4 + 6*s.^5;
    dphi = (30*s.^2 - 60*s.^3 + 30*s.^4)./Tref;
    ddphi = (60*s - 180*s.^2 + 120*s.^3)./(Tref^2);
    dphi(t > Tref) = 0;
    ddphi(t > Tref) = 0;

    delta = qf - q0;
    qd = q0 + delta * phi;
    dqd = delta * dphi;
    ddqd = delta * ddphi;
end

function pdf = build_pdf_gain(K0, n, h)
% 离线构造 Gramian 型 PDF 增益  (论文 Eq (31)~(32))
%
% 可控 Gramian (Eq (31)):
%   W_c = integral_{h}^{2h} exp(-A_c*s) * B * R_h(s) * B' * exp(-A_c'*s) ds
% 其中 A_c = A + B*K0 为基准闭环矩阵。
%
% 返回结构体包含 W_c, W_c^{-1}, 条件数, 及 A,B,A_c 供在线查表。

    A = [zeros(n), eye(n); zeros(n), zeros(n)];
    B = [zeros(n); eye(n)];
    Ac = A + B*K0;

    % 梯形法数值积分
    n_grid = 4001;
    s_grid = linspace(h, 2*h, n_grid);
    W = zeros(2*n);
    for k = 1:n_grid
        s = s_grid(k);
        Rh = smooth_periodic_Rh(s, h);
        E = expm(-Ac*s);
        integrand = E * B * (Rh*eye(n)) * B.' * E.';
        weight = 1;
        if k == 1 || k == n_grid
            weight = 0.5;
        end
        W = W + weight*integrand;
    end
    W = W * (s_grid(2) - s_grid(1));
    W = 0.5*(W + W.');

    pdf.A = A;
    pdf.B = B;
    pdf.Ac = Ac;
    pdf.W = W;
    pdf.W_inv = pinv(W);
    pdf.gramian_condition = cond(W);
    pdf.h = h;
end

function case_cfg = make_case(name, controller_mode, use_disturbance, use_observer)
    case_cfg.name = char(name);
    case_cfg.controller_mode = char(controller_mode);
    case_cfg.use_disturbance = use_disturbance;
    case_cfg.use_observer = use_observer;
end

function out = empty_result()
    out.name = '';
    out.controller_mode = '';
    out.use_disturbance = false;
    out.use_observer = false;
    out.t = [];
    out.q = [];
    out.dq = [];
    out.ddq = [];
    out.tau = [];
    out.e = [];
    out.edot = [];
    out.Rh = [];
    out.V = [];
    out.delta_a = [];
    out.delta_a_hat = [];
    out.delta_a_error = [];
    out.metrics = struct();
end

function out = run_case(case_cfg, cfg, ref, ctrl, P)
% 运行单个算例 (论文第6节 五组对照算例)
%
% 在每个仿真步内:
%   1. 计算误差 z = [e; edot]  (论文 Eq (5))
%   2. 由控制器模式计算辅助加速度 nu  (论文 Eq (25)/(29))
%   3. 计算名义动力学 M_e, C_e
%   4. 若有扰动, 施加 d(t); 若有观测器, 获取 Delta_a_hat
%   5. 计算关节力矩 tau (论文 Eq (33)):
%        tau = M_e * [qdd_d + nu - Delta_a_hat] + C_e * qd_m
%   6. 更新 PTDO 状态 (论文 Eq (20))
%   7. 数值积分得到下一时刻状态

    n = cfg.n;
    N = cfg.N;
    q = zeros(n, N);
    dq = zeros(n, N);
    ddq = zeros(n, N);
    tau = zeros(n, N);
    e = zeros(n, N);
    edot = zeros(n, N);
    zlog = zeros(2*n, N);
    Rhlog = zeros(1, N);
    Vlog = zeros(1, N);
    delta_a_log = zeros(n, N);
    delta_a_hat_log = zeros(n, N);
    delta_a_error_log = zeros(n, N);

    q(:,1) = cfg.q_init;
    dq(:,1) = cfg.dq_init;
    obs_state = init_observer_state(n);

    % 两阶段时序: 观测阶段结束时间 (论文第5节 命题1-2)
    T_o = ctrl.observer.T_o;
    use_two_stage = case_cfg.use_observer && ctrl.observer.use_two_stage_pdf;

    for k = 1:N-1
        tk = cfg.t(k);

        % --- 误差状态 z = [e; edot]  (论文 Eq (5)) ---
        e(:,k) = q(:,k) - ref.qd(:,k);
        edot(:,k) = dq(:,k) - ref.dqd(:,k);
        z = [e(:,k); edot(:,k)];
        zlog(:,k) = z;

        % --- 延迟状态 z(t-h) ---
        if k > cfg.Nh
            z_delay = zlog(:, k-cfg.Nh);
        else
            z_delay = zlog(:, 1);
        end

        % --- 两阶段 PDF 时序 (论文第4节末) ---
        % Stage 1 (t < T_o): PDF 增益 = 0, 相当于纯 PD
        % Stage 2 (t >= T_o): PDF 以局部时间 vartheta = t - T_o 启动
        if use_two_stage
            control_time = max(tk - T_o, 0);
        else
            control_time = tk;
        end

        [nu, Rh] = auxiliary_acceleration(control_time, z, z_delay, ...
            case_cfg.controller_mode, ctrl, cfg.h);
        Rhlog(k) = Rh;

        % --- 名义动力学 M_e, C_e (论文 Eq (13), 名义模型约化) ---
        [H0, C0] = ffsm_dynamics_nominal(q(:,k), dq(:,k), P);

        % --- 扰动 ---
        if case_cfg.use_disturbance
            d = disturbance_torque(tk, n);
        else
            d = zeros(n, 1);
        end

        delta_a = H0 \ d;
        delta_a_hat = zeros(n, 1);
        if case_cfg.use_observer
            delta_a_hat = obs_state.z2;  % PTDO 估计值
        end

        % --- 关节力矩 tau (论文 Eq (33)) ---
        % tau = M_e * [qdd_d + nu - Delta_a_hat] + C_e * qd_m
        u_aux = ref.ddqd(:,k) + nu - delta_a_hat;
        tau_cmd = H0*u_aux + C0*dq(:,k);
        tau_cmd = max(min(tau_cmd, cfg.tau_max), -cfg.tau_max);
        tau(:,k) = tau_cmd;

        % --- PTDO 更新 (论文 Eq (20)) ---
        u_observer = H0 \ (tau_cmd - C0*dq(:,k));
        if case_cfg.use_observer
            obs_state = update_ptdo(obs_state, dq(:,k), u_observer, ...
                tk, cfg.Ts, ctrl.observer);
            delta_a_hat = obs_state.z2;
        end

        % --- 数值积分 (前向欧拉) ---
        ddq(:,k) = H0 \ (tau_cmd + d - C0*dq(:,k));
        dq(:,k+1) = dq(:,k) + cfg.Ts*ddq(:,k);
        q(:,k+1) = q(:,k) + cfg.Ts*dq(:,k+1);
        Vlog(k) = 0.5*(e(:,k).'*e(:,k) + edot(:,k).'*edot(:,k));
        delta_a_log(:,k) = delta_a;
        delta_a_hat_log(:,k) = delta_a_hat;
        delta_a_error_log(:,k) = delta_a - delta_a_hat;
    end

    % --- 填充最后一帧 ---
    e(:,N) = q(:,N) - ref.qd(:,N);
    edot(:,N) = dq(:,N) - ref.dqd(:,N);
    zlog(:,N) = [e(:,N); edot(:,N)];
    Rhlog(N) = smooth_periodic_Rh(cfg.t(N), cfg.h);
    Vlog(N) = 0.5*(e(:,N).'*e(:,N) + edot(:,N).'*edot(:,N));
    tau(:,N) = tau(:,N-1);
    ddq(:,N) = ddq(:,N-1);
    delta_a_log(:,N) = delta_a_log(:,N-1);
    delta_a_hat_log(:,N) = delta_a_hat_log(:,N-1);
    delta_a_error_log(:,N) = delta_a_error_log(:,N-1);

    out = empty_result();
    out.name = case_cfg.name;
    out.controller_mode = case_cfg.controller_mode;
    out.use_disturbance = case_cfg.use_disturbance;
    out.use_observer = case_cfg.use_observer;
    out.t = cfg.t;
    out.q = q;
    out.dq = dq;
    out.ddq = ddq;
    out.tau = tau;
    out.e = e;
    out.edot = edot;
    out.Rh = Rhlog;
    out.V = Vlog;
    out.delta_a = delta_a_log;
    out.delta_a_hat = delta_a_hat_log;
    out.delta_a_error = delta_a_error_log;
    out.metrics = compute_metrics(cfg.t, e, edot, tau, delta_a_error_log);
end

function obs_state = init_observer_state(n)
    obs_state.z1 = zeros(n, 1);
    obs_state.z2 = zeros(n, 1);
end

function obs_state = update_ptdo(obs_state, chi, u_o, t, Ts, obs)
% 预设时间扰动观测器 (Prescribed-Time Disturbance Observer, PTDO)
% 论文 Eq (20):
%   dot{z}_1 = z_2 + pi/(eta*T_c) * phi1(eps1/sigma) - dot{xi} + xi + u_o
%   dot{z}_2 = pi/(sigma*eta*T_c) * phi2(eps1/sigma) - dot{xi}
% 其中 eps1 = chi - z_1 - xi (观测复合误差, Eq (19))
%       phi1, phi2 定义见 Eq (21)~(22)
%       xi(t) 为调节函数, t >= T_o 时 xi(t)=0

    [xi, xi_dot] = observer_regulation(t, obs);
    eps1 = chi - obs_state.z1 - xi;
    scaled_eps = eps1 ./ obs.sigma;
    phi1 = signed_power(scaled_eps, 1 - obs.eta/2) + ...
           signed_power(scaled_eps, 1 + obs.eta/2);
    phi2 = signed_power(scaled_eps, 2 - obs.eta) + ...
           signed_power(scaled_eps, 2 + obs.eta) + ...
           obs.sigma * sign(scaled_eps);

    z1_dot = obs_state.z2 + (pi/(obs.eta*obs.T_c))*phi1 - xi_dot + xi + u_o;
    z2_dot = (pi/(obs.sigma*obs.eta*obs.T_c))*phi2 - xi_dot;

    obs_state.z1 = obs_state.z1 + Ts*z1_dot;
    obs_state.z2 = obs_state.z2 + Ts*z2_dot;
end

function [xi, xi_dot] = observer_regulation(t, obs)
    if t < obs.T_o
        remaining = obs.T_o - t;
        xi = obs.xi0 .* (remaining^2);
        xi_dot = -2 * obs.xi0 .* remaining;
    else
        xi = zeros(size(obs.xi0));
        xi_dot = zeros(size(obs.xi0));
    end
end

function y = signed_power(x, p)
    y = sign(x) .* (abs(x) .^ p);
end

function [nu, Rh] = auxiliary_acceleration(t, z, z_delay, mode, ctrl, h)
% 计算辅助加速度 nu (论文 Eq (29)):
%   nu(t) = K0 * z(t) - Kc(t) * z(t-h)
%
% 'pd' 模式: nu = K0 * z  (无延迟反馈)
% 'pdf_exact' 模式: nu = K0*z - Kc(t)*z(t-h), Kc 由 Gramian 构造 Eq (32)
% 'pdf_engineering' 模式: nu = K0*z + rho*Rh*Kh*z_delay (消融对比用)

    Rh = smooth_periodic_Rh(t, h);
    switch mode
        case 'pd'
            nu = ctrl.K0*z;
        case 'pdf_exact'
            Kc = exact_pdf_gain(t, ctrl.pdf);
            nu = ctrl.K0*z - Kc*z_delay;
        case 'pdf_engineering'
            nu = ctrl.K0*z + ctrl.engineering_gain*Rh*(ctrl.engineering_Kh*z_delay);
        otherwise
            error('Unknown controller mode: %s', mode);
    end
end

function Kc = exact_pdf_gain(t, pdf)
% 计算 Gramian 型 PDF 增益 Kc(t)  (论文 Eq (32)):
%   Kc(t) = R_h(theta) * B' * exp(-Ac'*theta) * Wc^{-1} * exp(Ac*(h-theta))
% 其中 theta = mod(t, 2h) 使增益 2h-周期化, 保持 act-and-wait 结构。
%
% 注意: 理论推导中 Kc 仅在第一个 [0,2h) 周期严格满足单周期零化;
%       通过 mod(t,2h) 周期延拓是工程实现中的合理近似,
%       实际效果由残余扰动上界保证 (论文第5节 命题4)。

    h = pdf.h;
    theta = mod(t, 2*h);
    Rh = smooth_periodic_Rh(theta, h);
    if Rh == 0
        Kc = zeros(size(pdf.B, 2), size(pdf.A, 1));
        return;
    end
    Kc = Rh * (pdf.B.' * expm(-pdf.Ac.'*theta) * pdf.W_inv * expm(pdf.Ac*(h-theta)));
end

function Rh = smooth_periodic_Rh(t, h)
% 光滑 2h-周期延迟反馈激活函数 R_h(t)  (论文第6节)
%   R_h(t) = 0,               theta in [0, h)
%   R_h(t) = sin^4(pi*s),     s = (theta-h)/h, theta in [h, 2h)
% 满足: R_h(0)=R_h(h)=R_h(2h)=0, C^2 光滑, [h,2h) 上严格正。
% 对应 PDF 理论的 "等待-作用" (act-and-wait) 结构:
%   前半周期 [0,h):  Kc=0, 只储存延迟历史
%   后半周期 [h,2h): Kc>0, 延迟反馈激活

    theta = mod(t, 2*h);
    if theta < h
        Rh = 0;
    else
        s = (theta - h)/h;
        Rh = sin(pi*s)^4;
    end
end

function [H, C] = ffsm_dynamics_nominal(q, dq, P)
% 名义关节空间模型  (对应论文 Eq (11)~式 (13))
%   M_e(qm) * qdd_m + C_e(qm, qd_m) * qd_m = tau
%
% 完整实现应基于 Schur 补 (论文 Eq (12)):
%   M_e = H_m - H_bm' * H_b^{-1} * H_bm
%   C_e = 由完整递推动力学 + 动量约束求导 + 坐标变换确定
%
% 当前实现: 基于 Yan 论文表2-1 参数构造的正定对角占优矩阵,
% 满足 M_e 正定、C_e 对角阻尼的基本性质, 用于验证控制结构。
% 当完整递推动力学实现可用时, 应替换为精确的 Schur 补形式。

    n = P.n;
    Jdiag = P.Jdiag;

    H = diag(Jdiag);
    for i = 1:n
        for j = i+1:n
            coupling = 0.02*sqrt(Jdiag(i)*Jdiag(j))*cos(q(i)-q(j));
            H(i,j) = coupling;
            H(j,i) = coupling;
        end
    end
    H = 0.5*(H + H.');

    mineig = min(eig(H));
    if mineig <= 1e-6
        H = H + (abs(mineig) + 1e-3)*eye(n);
    end

    C = diag(0.05 + 0.02*abs(dq));
end

function d = disturbance_torque(t, n)
% 有界时变扰动力矩  (论文第6节):
%   d_i(t) = 0.15*sin(0.7*t + 0.3*i) + 0.05*cos(1.3*t + 0.2*i)
% 该扰动作用于等效关节空间模型 Eq (13) 的右端力矩通道。

    i = (1:n).';
    d = 0.15*sin(0.7*t + 0.3*i) + 0.05*cos(1.3*t + 0.2*i);
end

function metrics = compute_metrics(t, e, edot, tau, delta_a_error)
    e_norm = vecnorm(e, 2, 1);
    edot_norm = vecnorm(edot, 2, 1);
    obs_error_norm = vecnorm(delta_a_error, 2, 1);
    metrics.final_e_norm = e_norm(end);
    metrics.final_edot_norm = edot_norm(end);
    metrics.max_e_norm = max(e_norm);
    metrics.max_edot_norm = max(edot_norm);
    metrics.rms_e = sqrt(mean(e.^2, 'all'));
    metrics.rms_edot = sqrt(mean(edot.^2, 'all'));
    metrics.max_tau = max(abs(tau(:)));
    metrics.settle_time_e = settling_time(t, e_norm, 1e-4);
    metrics.final_obs_error_norm = obs_error_norm(end);
    metrics.rms_obs_error = sqrt(mean(delta_a_error.^2, 'all'));
    metrics.max_obs_error_norm = max(obs_error_norm);
end

function ts = settling_time(t, signal, threshold)
% First time after which signal stays below threshold. Returns NaN if not
% achieved.

    ts = NaN;
    for k = 1:numel(t)
        if all(signal(k:end) <= threshold)
            ts = t(k);
            return;
        end
    end
end

function metrics = collect_metrics(results)
    metrics = struct([]);
    for c = 1:numel(results)
        metrics(c).name = results(c).name;
        metrics(c).final_e_norm = results(c).metrics.final_e_norm;
        metrics(c).final_edot_norm = results(c).metrics.final_edot_norm;
        metrics(c).max_tau = results(c).metrics.max_tau;
        metrics(c).settle_time_e = results(c).metrics.settle_time_e;
        metrics(c).final_obs_error_norm = results(c).metrics.final_obs_error_norm;
        metrics(c).rms_obs_error = results(c).metrics.rms_obs_error;
    end
end

