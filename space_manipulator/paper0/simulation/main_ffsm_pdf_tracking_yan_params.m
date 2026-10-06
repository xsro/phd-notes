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
% 仿真算例 (论文第6节, 表1 + 抗饱和扩展):
%   1. PD, 无扰动
%   2. PDF, 无扰动
%   3. PD, 有扰动
%   4. PDF, 有扰动
%   5. PDF+PTDO, 有扰动
%   6. PDF+PTDO (tau<5), 有扰动 (低饱和限幅下对比基线)
%   7. PDF+PTDO+AS (tau<5), 有扰动 (抗饱和补偿, 259文献)
%      AS: 固定时间抗饱和补偿器 (Dou & Yue 2025, IJRNC, DOI: 10.1002/rnc.7826)
%      核心: eta_dot = -f_comp(eta) + Delta_u, tau = tau_pdf + H0*eta
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
% 改进模型: H ~ 1000, 需要调整增益
Kp = diag([0.8 0.8 0.8 0.6 0.6 0.5 0.5]);
Kd = diag([0.3 0.3 0.3 0.25 0.25 0.2 0.2]);
ctrl.K0 = [-Kp, -Kd];

% --- Gramian 型 PDF 增益 (论文 Eq (31)~(32)) ---
ctrl.pdf = build_pdf_gain(ctrl.K0, cfg.n, cfg.h);

% --- Gramian 型 PDF 增益 (论文 Eq (31)~(32)) ---
% Kc(t) = R_h(t) * B' * exp(-Ac'*t) * Wc^{-1} * exp(Ac*(h-t))
% Wc = integral_{h}^{2h} exp(-Ac*s)*B*Rh(s)*B'*exp(-Ac'*s) ds
% 离线积分构造 Wc, 在线查表 Kc(mod(t,2h))
ctrl.pdf = build_pdf_gain(ctrl.K0, cfg.n, cfg.h);

% 工程化 PDF (消融/调试用, 非论文理论增益)
Kp_h = diag([0.2 0.2 0.2 0.15 0.15 0.1 0.1]);
Kd_h = diag([0.1 0.1 0.1 0.08 0.08 0.05 0.05]);
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

% --- 固定时间抗饱和补偿器 (Dou & Yue 2025, IJRNC, Eq. 37/44) ---
% 核心思想: 引入辅助变量 eta, 其动力学吸收饱和误差 Delta_u
%   eta_dot = -f_comp(eta) + Delta_u
% 其中 f_comp(eta) 保证饱和结束后 eta 固定时间收敛到零
% 修改后的控制律: tau_cmd = tau_pdf + H * eta
as.enabled = true;
as.k_eta1 = diag([0.05 0.05 0.05 0.04 0.04 0.03 0.03]);  % 固定时间收敛增益 - 匹配H_theta量级
as.k_eta2 = diag([0.03 0.03 0.03 0.025 0.025 0.02 0.02]); % 终端收敛增益
as.rho1 = 0.5;                            % 幂指数 rho1 in (0,1)
as.rho2 = 0.5;                            % 幂指数 rho2 in (0,1)
as.L_eta = 0.5;                           % 自适应增益 - 匹配H_theta量级
as.use_adaptive = false;                  % 是否启用自适应增益
as.k_feed = 0.1;                          % 前馈增益: Delta_u即时补偿
ctrl.anti_saturation = as;

%% -------------------- 5. Run cases (论文表1 + 抗饱和扩展) -----------------
cases(1) = make_case("PD, nominal", "pd", false, false, false);
cases(2) = make_case("PDF, nominal", "pdf_exact", false, false, false);
cases(3) = make_case("PD, disturbed", "pd", true, false, false);
cases(4) = make_case("PDF, disturbed", "pdf_exact", true, false, false);
cases(5) = make_case("PDF+PTDO, disturbed", "pdf_exact", true, true, false);
% 抗饱和测试: 低饱和限幅 + 更大初始误差 + 强扰动, 对比有无 AS
cfg.tau_max_as = 2.0 * ones(cfg.n, 1);
% 为测试用例单独设置更大的初始误差 (30 deg vs 4 deg)
cfg.q_init_as = q_ref0 + deg2rad([30; -20; 20; -25; 20; -15; 15]);
cases(6) = make_case("PDF+PTDO (tau<2), disturbed", "pdf_exact", true, true, false);
cases(7) = make_case("PDF+PTDO+AS (tau<2), disturbed", "pdf_exact", true, true, true);

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
fprintf('RMS(e) [rad]    max|tau_i| [N m]   sat_ratio[%%]  t_settle [s]   ||obs err||\n');
for c = 1:numel(results)
    m = results(c).metrics;
    fprintf('%-24s %.6e       %.6e       %.6e    %.1f       %.1f      %.3f        %.6e\n', ...
        results(c).name, m.final_e_norm, m.final_edot_norm, ...
        m.rms_e, m.max_tau, m.saturation_ratio*100, ...
        m.settle_time_e, m.final_obs_error_norm);
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

function case_cfg = make_case(name, controller_mode, use_disturbance, use_observer, use_anti_saturation)
    case_cfg.name = char(name);
    case_cfg.controller_mode = char(controller_mode);
    case_cfg.use_disturbance = use_disturbance;
    case_cfg.use_observer = use_observer;
    case_cfg.use_anti_saturation = use_anti_saturation;
    % 抗饱和测试: 名称含 "tau<" 的用例使用低饱和限幅
    case_cfg.use_saturation_test = contains(name, "tau<");
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
    out.eta_as = [];
    out.saturation_ratio = 0;
    out.tau_desired = [];
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
    eta = zeros(n, 1);                          % 抗饱和辅助变量 (加速通道)
    sat_count = 0;                               % 饱和计数器
    tau_desired_log = zeros(n, N);               % 记录饱和前期望力矩

    % 抗饱和测试: 使用更大的初始误差以触发强饱和
    if case_cfg.use_saturation_test && isfield(cfg, 'q_init_as')
        q(:,1) = cfg.q_init_as;
    end

    % 两阶段时序: 观测阶段结束时间 (论文第5节 命题1-2)
    T_o = ctrl.observer.T_o;
    use_two_stage = case_cfg.use_observer && ctrl.observer.use_two_stage_pdf;
    use_as = case_cfg.use_anti_saturation && ctrl.anti_saturation.enabled;

    % 饱和限幅选择: 抗饱和测试用例使用更低的限幅以触发饱和
    if case_cfg.use_saturation_test
        tau_max = cfg.tau_max_as;
    else
        tau_max = cfg.tau_max;
    end

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
        if use_two_stage
            control_time = max(tk - T_o, 0);
        else
            control_time = tk;
        end

        [nu, Rh] = auxiliary_acceleration(control_time, z, z_delay, ...
            case_cfg.controller_mode, ctrl, cfg.h);
        Rhlog(k) = Rh;

        % --- 名义动力学 M_e, C_e (论文 Eq (13)) ---
        [H0, C0] = ffsm_dynamics_nominal(q(:,k), dq(:,k), P);

        % --- 扰动: 抗饱和测试使用更强的扰动以维持持续饱和 ---
        if case_cfg.use_disturbance
            if case_cfg.use_saturation_test
                amp = 1.5;  % 更强扰动 [Nm]
            else
                amp = 1.0;  % 标准扰动幅度
            end
            d = disturbance_torque(tk, n, amp);
        else
            d = zeros(n, 1);
        end

        delta_a = H0 \ d;
        delta_a_hat = zeros(n, 1);
        if case_cfg.use_observer
            delta_a_hat = obs_state.z2;  % PTDO 估计值
        end

        % --- 关节力矩 (论文 Eq (33) + AS 补偿) ---
        % 原始 PDF:     tau_pdf = H0*(qdd_d + nu - Dhat) + C0*qd_m
        % AS 补偿:      tau_cmd = tau_pdf + H0*eta  (η 累加饱和误差)
        % 饱和:          tau_act = sat(tau_cmd)
        % 饱和误差:      Delta_u = H0\\(tau_act - tau_cmd)
        % AS更新:        eta_dot = -f_comp(eta) + Delta_u
        u_aux = ref.ddqd(:,k) + nu - delta_a_hat;
        tau_pdf = H0*u_aux + C0*dq(:,k);
        tau_desired = tau_pdf;
        if use_as
            tau_desired = tau_pdf + H0*eta;
        end
        tau_desired_log(:,k) = tau_desired;
        tau_actual = max(min(tau_desired, tau_max), -tau_max);
        tau(:,k) = tau_actual;

        % --- 抗饱和更新 (Dou & Yue 2025, Eq. 37/44) ---
        if use_as
            Delta_u = H0 \ (tau_actual - tau_desired);  % 归一化饱和误差
            % 加速因子: η 积分加速, 使 AS 在饱和初期快速建立补偿
            eta = update_anti_saturation(eta, 5.0*Delta_u, cfg.Ts, ctrl.anti_saturation);
        end
        if any(abs(tau_actual - tau_desired) > 1e-6)
            sat_count = sat_count + 1;
        end

        % --- PTDO 更新 (论文 Eq (20)) ---
        u_observer = H0 \ (tau_actual - C0*dq(:,k));
        if case_cfg.use_observer
            obs_state = update_ptdo(obs_state, dq(:,k), u_observer, ...
                tk, cfg.Ts, ctrl.observer);
            delta_a_hat = obs_state.z2;
        end

        % --- 数值积分 (前向欧拉) ---
        ddq(:,k) = H0 \ (tau_actual + d - C0*dq(:,k));
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
    tau_desired_log(:,N) = tau_desired_log(:,N-1);

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
    out.eta_as = repmat(eta, 1, N);  % 最后一帧的 eta 值
    out.saturation_ratio = sat_count / (N-1);
    out.tau_desired = tau_desired_log;
    out.metrics = compute_metrics(cfg.t, e, edot, tau, delta_a_error_log, ...
        sat_count/(N-1));
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

function eta_new = update_anti_saturation(eta, Delta_u, Ts, as)
% 固定时间抗饱和补偿器 (Dou & Yue 2025, IJRNC, Eq. 37/44)
%
% eta_dot = -L_eta * [k_eta1/rho1 * tanh(|eta|^(1-rho1)) * cosh(|eta|^2) * sign(eta)
%                     + k_eta2 * sig(eta)^rho2]
%           + Delta_u
%
% 输入:
%   eta      - 辅助变量 (n x 1), 加速度通道
%   Delta_u  - 归一化饱和误差 (n x 1), Delta_u = H0\(tau_act - tau_cmd)
%   Ts       - 采样时间
%   as       - 抗饱和参数字段
% 输出:
%   eta_new  - 更新后的辅助变量
%
% 性质 (Theorem 2): 当 Delta_u = 0 时, eta 在固定时间内收敛到零:
%   T_max < rho1*I/(k_eta1*tanh(1)) + 1/(k_eta2*(1-rho2))
% 这意味着饱和结束后, eta 快速消失, 控制律退化为原始 PDF 控制器。

    rho1 = as.rho1;
    rho2 = as.rho2;
    k1_vec = diag(as.k_eta1);   % 转为列向量 (n x 1)
    k2_vec = diag(as.k_eta2);   % 转为列向量 (n x 1)
    L_eta = as.L_eta;

    % Term 1: 固定时间收敛项 (hyperbolic-based)
    abs_eta = abs(eta);
    term1 = (L_eta * k1_vec / rho1) .* tanh(abs_eta.^(1 - rho1)) .* cosh(abs_eta.^2) .* sign(eta);

    % Term 2: 终端收敛项
    abs_eta_safe = max(abs_eta, 1e-15);
    term2 = (L_eta * k2_vec) .* (abs_eta_safe.^rho2) .* sign(eta);

    eta_dot = -term1 - term2 + Delta_u;
    eta_new = eta + Ts * eta_dot;
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
% 名义关节空间模型 (259文献 Schur补量级匹配)
%
% 使用 Schur 补缩放的结构化近似模型, 保证正定性且量级与完整
% 自由漂浮动力学一致 (H ~ 1000, 匹配1000kg基座+轻量臂)。
%
% 参考: Dou & Yue (2025) Eq. (8), H_θm = H_m - H_bm'*H_b^{-1}*H_bm

    n = P.n;
    
    % 基于 Yan 参数计算对角惯量量级
    H0 = diag(P.Jdiag);
    
    % 构造 Schur 补量级近似的 H
    % 基座质量 1000kg 通过动量守恒贡献到每个关节的等效惯量
    H = H0 + diag(1000 * ones(n, 1));  % 基座质量平移贡献
    
    % 耦合项: 考虑基座-臂动量耦合的构型依赖耦合
    for i = 1:n
        for j = i+1:n
            % 耦合强度与基座质量及关节间距成正比
            coupling = 20 * cos(q(i) - q(j)) * exp(-0.3*abs(i-j));
            H(i,j) = coupling;
            H(j,i) = coupling;
        end
    end
    H = 0.5*(H + H.');
    
    % 正定性保证
    mineig = min(eig(H));
    if mineig <= 1e-6
        H = H + (abs(mineig) + 1e-3)*eye(n);
    end
    
    % Coriolis: 使用反对称结构加阻尼
    C = zeros(n, n);
    for i = 1:n
        for j = 1:n
            if i ~= j
                C(i,j) = 0.5 * (H(i,i) - H(j,j)) * sin(q(i)-q(j)) * dq(j);
            end
        end
    end
    C = C + diag(0.5 + 0.2*abs(dq));  % 阻尼
end

function d = disturbance_torque(t, n, amp)
% 有界时变扰动力矩  (论文第6节):
%   d_i(t) = amp * [0.15*sin(0.7*t + 0.3*i) + 0.05*cos(1.3*t + 0.2*i)]
% 该扰动作用于等效关节空间模型 Eq (13) 的右端力矩通道。

    if nargin < 3
        amp = 1.0;
    end
    i = (1:n).';
    d = amp * (0.15*sin(0.7*t + 0.3*i) + 0.05*cos(1.3*t + 0.2*i));
end

function metrics = compute_metrics(t, e, edot, tau, delta_a_error, saturation_ratio)
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
    metrics.saturation_ratio = saturation_ratio;
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
        metrics(c).saturation_ratio = results(c).metrics.saturation_ratio;
        metrics(c).settle_time_e = results(c).metrics.settle_time_e;
        metrics(c).final_obs_error_norm = results(c).metrics.final_obs_error_norm;
        metrics(c).rms_obs_error = results(c).metrics.rms_obs_error;
    end
end

