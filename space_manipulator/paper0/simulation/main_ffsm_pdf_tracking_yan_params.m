%% main_ffsm_pdf_tracking_yan_params.m
% -------------------------------------------------------------------------
% Free-floating space manipulator tracking control with smooth periodic
% delayed feedback (PDF) using physical parameters from Yan Yuxin thesis.
%
% Model basis:
%   H_g(q) qdd + C_g(q,qd) qd = tau + d(t)
%
% Control:
%   tau = H0(q) [qdd_d + nu - dhat_a] + C0(q,qd) qd
%   z(t) = [e(t); edot(t)]
%   nu = K0 z(t) - Kc(t) z(t-h)
%   dhat = H0(q) dhat_a
%
% Academic note:
%   The exact H_g and C_g of the free-floating 7-DOF space manipulator
%   require full kinematic recursion and generalized Jacobian construction.
%   This runnable program uses a positive-definite nominal joint-space model
%   built from the tabulated physical parameters. Replace
%   ffsm_dynamics_nominal() by the exact Yan model implementation when it is
%   available.
% -------------------------------------------------------------------------

clear; clc; close all;

%% -------------------- 1. Simulation settings ----------------------------
cfg.Ts = 1e-4;             % integration step [s]
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
cfg.h = 1.0;                         % artificial delay [s]
cfg.Nh = round(cfg.h/cfg.Ts);         % delay samples
cfg.tau_max = 200 * ones(cfg.n, 1);   % [N m], set Inf for no saturation

% Baseline stabilizing gain for the double-integrator error channel.
Kp = diag([25 25 25 22 22 20 20]);
Kd = diag([10 10 10  9  9  8  8]);
ctrl.K0 = [-Kp, -Kd];

% Exact matrix PDF gain for the linear error channel. The gain is computed
% offline from the controllability Gramian in one 2h-period.
ctrl.pdf = build_pdf_gain(ctrl.K0, cfg.n, cfg.h);

% Engineering delayed-feedback mode kept for ablation/debugging.
Kp_h = diag([8 8 8 6 6 5 5]);
Kd_h = diag([3 3 3 2.5 2.5 2 2]);
ctrl.engineering_Kh = [-Kp_h, -Kd_h];
ctrl.engineering_gain = 1.0;

% Prescribed-time disturbance observer for the equivalent acceleration
% disturbance Delta_a = H^{-1} d. The observer is intentionally not too
% aggressive to avoid peaking in the torque command.
obs.enabled = false;
obs.T_o = 1.0;                 % prescribed observation time [s]
obs.T_c = 1.0;                 % observer internal convergence parameter [s]
obs.eta = 0.3;
obs.sigma = 0.4;
obs.xi0 = zeros(cfg.n, 1);
obs.use_two_stage_pdf = true;
ctrl.observer = obs;

%% -------------------- 5. Run cases --------------------------------------
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
fprintf('max|tau| [N m]   t_settle_e [s]   ||obs err(T)||\n');
for c = 1:numel(results)
    m = results(c).metrics;
    fprintf('%-24s %.6e       %.6e            %.6f        %.3f          %.6e\n', ...
        results(c).name, m.final_e_norm, m.final_edot_norm, ...
        m.max_tau, m.settle_time_e, m.final_obs_error_norm);
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
% Offline Gramian construction of the smooth periodic delayed feedback
% matrix gain for the normalized error channel.

    A = [zeros(n), eye(n); zeros(n), zeros(n)];
    B = [zeros(n); eye(n)];
    Ac = A + B*K0;

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
% Simulate one controller/disturbance combination.

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

    for k = 1:N-1
        tk = cfg.t(k);

        e(:,k) = q(:,k) - ref.qd(:,k);
        edot(:,k) = dq(:,k) - ref.dqd(:,k);
        z = [e(:,k); edot(:,k)];
        zlog(:,k) = z;

        if k > cfg.Nh
            z_delay = zlog(:, k-cfg.Nh);
        else
            z_delay = zlog(:, 1);
        end

        control_time = tk;
        if case_cfg.use_observer && ctrl.observer.use_two_stage_pdf
            control_time = max(tk - ctrl.observer.T_o, 0);
        end

        [nu, Rh] = auxiliary_acceleration(control_time, z, z_delay, ...
            case_cfg.controller_mode, ctrl, cfg.h);
        Rhlog(k) = Rh;

        [H0, C0] = ffsm_dynamics_nominal(q(:,k), dq(:,k), P);
        if case_cfg.use_disturbance
            d = disturbance_torque(tk, n);
        else
            d = zeros(n, 1);
        end

        delta_a = H0 \ d;
        delta_a_hat = zeros(n, 1);
        if case_cfg.use_observer
            delta_a_hat = obs_state.z2;
        end

        u_aux = ref.ddqd(:,k) + nu - delta_a_hat;
        tau_cmd = H0*u_aux + C0*dq(:,k);
        tau_cmd = max(min(tau_cmd, cfg.tau_max), -cfg.tau_max);
        tau(:,k) = tau_cmd;

        % Use the saturated input actually sent to the plant in the observer.
        u_observer = H0 \ (tau_cmd - C0*dq(:,k));
        if case_cfg.use_observer
            obs_state = update_ptdo(obs_state, dq(:,k), u_observer, ...
                tk, cfg.Ts, ctrl.observer);
            delta_a_hat = obs_state.z2;
        end

        ddq(:,k) = H0 \ (tau_cmd + d - C0*dq(:,k));
        dq(:,k+1) = dq(:,k) + cfg.Ts*ddq(:,k);
        q(:,k+1) = q(:,k) + cfg.Ts*dq(:,k+1);
        Vlog(k) = 0.5*(e(:,k).'*e(:,k) + edot(:,k).'*edot(:,k));
        delta_a_log(:,k) = delta_a;
        delta_a_hat_log(:,k) = delta_a_hat;
        delta_a_error_log(:,k) = delta_a - delta_a_hat;
    end

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
% Prescribed-time disturbance observer for chi_dot = u_o + Delta_a.

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
% Compute the auxiliary acceleration command.

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
% Periodic matrix PDF gain over the local time theta in [0, 2h).

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
% C2 smooth 2h-periodic delayed feedback shaping function.

    theta = mod(t, 2*h);
    if theta < h
        Rh = 0;
    else
        s = (theta - h)/h;
        Rh = sin(pi*s)^4;
    end
end

function [H, C] = ffsm_dynamics_nominal(q, dq, P)
% Positive-definite nominal joint-space model corresponding to:
%   H_g(q) qdd + C_g(q,dq) dq = tau
%
% Replace this function by the exact implementation:
%   H_g = H_m - H_bm' H_b^{-1} H_bm
%   C_g = H_bm' H_b^{-1}(C_b H_b^{-1}H_bm - C_bm)
%         - C_bm' H_b^{-1}H_bm + C_m

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
% Bounded time-varying disturbance torque.

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

