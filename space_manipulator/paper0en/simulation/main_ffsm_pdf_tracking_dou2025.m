%% main_ffsm_pdf_tracking_dou2025.m
% -------------------------------------------------------------------------
% Prescribed-time trajectory tracking of a free-floating space manipulator
% with smooth periodic delayed feedback (PDF).
%
% Plant model
%   Full 3-D floating-base dynamics (spatial CRBA + RNEA) of the 7-DOF
%   space manipulator whose kinematic/dynamic parameters are reported in
%     B. Dou and X. Yue, "Fractional-Order Sliding Mode Control for
%     Free-Floating Space Manipulator With Disturbance and Input
%     Saturation", Int. J. Robust Nonlinear Control, 2025,
%     doi:10.1002/rnc.7826  (Tables 1 and 2).
%   The equivalent joint-space model
%       M_e(q) ddq + C_e(q,dq) dq = tau + d(t)
%   is obtained exactly from the Schur complement of the full inertia
%   matrix and from the momentum-consistent bias forces; no nominal or
%   synthetic inertia model is used.
%
% Controller (paper Section IV)
%   1. Prescribed-time disturbance observer (PTDO)
%   2. Dynamic compensation / error linearisation
%   3. Smooth periodic delayed feedback (PDF) with Gramian gain
%   4. Optional fixed-time anti-saturation compensator (AS)
%
% Usage
%   run('main_ffsm_pdf_tracking_dou2025.m')          % full study
%   main_ffsm_pdf_tracking_dou2025('tune')           % quick diagnostics
% -------------------------------------------------------------------------

function main_ffsm_pdf_tracking_dou2025(mode)

if nargin < 1
    mode = 'full';
end

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
addpath(script_dir);

set(groot, 'DefaultTextInterpreter', 'latex');
set(groot, 'DefaultAxesTickLabelInterpreter', 'latex');
set(groot, 'DefaultLegendInterpreter', 'latex');

cfg.Ts = 1e-3;
cfg.Tf = 10.0;
cfg.t  = 0:cfg.Ts:cfg.Tf;
cfg.N  = numel(cfg.t);
cfg.n  = 7;
cfg.save_figures = ~strcmpi(mode, 'tune');
cfg.diag = ~strcmpi(mode, 'full');
cfg.figure_dir = fullfile(script_dir, '..', 'figures');
cfg.result_file = fullfile(script_dir, 'dou2025_results.mat');
if cfg.save_figures && ~exist(cfg.figure_dir, 'dir')
    mkdir(cfg.figure_dir);
end
set(0, 'DefaultFigureVisible', 'off');

%% -------------------- Plant parameters (Dou & Yue 2025) ----------------
P = dou2025_parameters();

%% -------------------- Reference trajectory and initial state -----------
q_ref0 = P.q0;
q_ref_f = P.q0 + deg2rad([15; 22; -18; 22; -22; 28; 18]);
% staggered fifth-order polynomials: joint i completes its move at T_i,
% which produces a curved end-effector path in the base frame
cfg.Tref = (8.0 + 0.25*(0:cfg.n-1)).';           % [s]
[ref.qd, ref.dqd, ref.ddqd] = quintic_joint_trajectory(q_ref0, q_ref_f, cfg.Tref, cfg.t);

% Nonzero initial tracking error, all joints inside the workspace
initial_error_deg = [0.5; -0.4; 0.4; -0.5; 0.4; -0.4; 0.3];
cfg.q_init  = q_ref0 + deg2rad(initial_error_deg);
cfg.dq_init = zeros(cfg.n, 1);

%% -------------------- Controller parameters ----------------------------
cfg.h  = 1.0;                              % artificial delay h
cfg.Nh = round(cfg.h / cfg.Ts);

% Actuator limits. The tabulated limit of the source model is 20 N m
% (Table 1 of Dou & Yue 2025). The nominal study uses a relaxed limit so
% that the prescribed-time mechanism is not masked by clipping; the
% 20 N m limit is enforced in the dedicated saturation cases 6-7.
cfg.tau_max     = 20 * ones(cfg.n, 1);
cfg.tau_max_low = 20 * ones(cfg.n, 1);

% Baseline stabilising gain K0 = [-Kp, -Kd]. A uniform gain set is used so
% that every joint of the error channel has the same natural frequency
% omega = sqrt(Kp) = 1.41 rad/s, zeta = Kd/(2 sqrt(Kp)) = 0.99. This is essential for the PDF construction:
% the Gramian of (6) is block diagonal with identical 2x2 blocks and is
% therefore well conditioned. (Per-joint inertia normalisation would equalise
% the torque per radian but spread the modal time scales over three decades
% and make the Gramian numerically singular.)
Kp = diag(2.0*ones(cfg.n,1));
Kd = diag(2.8*ones(cfg.n,1));
ctrl.K0 = [-Kp, -Kd];
cfg.Me0 = diag(ffsm_dou_reduced(P, cfg.q_init, zeros(cfg.n,1)));
cfg.dist_amp = 0.02;   % acceleration-level disturbance amplitude [rad/s^2]
ctrl.pdf = build_pdf_gain(ctrl.K0, cfg.n, cfg.h);

% Prescribed-time disturbance observer (paper Eq. (18)): homogeneous
% correction terms with time-varying gains l1(t) = m1/(T_o-t)+,
% l2(t) = m2/(T_o-t)^2+; the gains are frozen at t = T_o - delta.
obs.T_o   = 1.0;
obs.delta = 0.05;
obs.m1    = 3.0;
obs.m2    = 4.0;
obs.eta   = 0.3;
obs.sigma = 0.4;
ctrl.observer = obs;


%% -------------------- Case definitions ---------------------------------
cases(1) = make_case("PD, nominal",  "pd",  false, false);
cases(2) = make_case("PDF, nominal", "pdf", false, false);
cases(3) = make_case("PD, disturbed",  "pd",  true, false);
cases(4) = make_case("PDF, disturbed", "pdf", true, false);
cases(5) = make_case("PDF+PTDO, disturbed", "pdf", true, true);
% case 6 uses a larger initial error so that the tabulated 20 N m actuator
% limit becomes active; it quantifies the effect of input saturation.
cases(6) = make_case("PDF+PTDO, disturbed, saturated", "pdf", true, true);
cfg.q_init_sat = q_ref0 + deg2rad([3; -2.5; 2.5; -3; 2.5; -2.5; 2]);
cases(6).use_large_error = true;

if strcmpi(mode, 'tune')
    cases = cases([2 4 5]);
end

fprintf('=== FFSM PDF tracking: %d case(s), Tf = %.0f s, Ts = %.0e s ===\n', ...
    numel(cases), cfg.Tf, cfg.Ts);
fprintf('Gains: Kp = diag(%s), Kd = diag(%s)\n', ...
    mat2str(diag(Kp).', 4), mat2str(diag(Kd).', 4));
fprintf('PDF Gramian condition number: %.3e\n', ctrl.pdf.gramian_condition);

% consistency checks of the reduced model (reported in the paper)
chk = model_selfcheck(P);
fprintf(['Model checks: min eig(H_full) = %.4f, momentum residual = %.2e,\n' ...
    '              skew-symmetry residual = %.2e (scale %.2e), energy drift = %.2e\n'], ...
    chk.min_eig_full, chk.momentum_residual, chk.skew_residual, chk.skew_scale, chk.energy_drift);
if cfg.save_figures
    cfg.model_checks = chk;
end

results = repmat(empty_result(), 1, numel(cases));
for c = 1:numel(cases)
    results(c) = run_case(cases(c), cfg, ref, ctrl, P);
end

metrics = collect_metrics(results);

fprintf('\nSimulation summary\n');
fprintf('%-38s %11s %11s %11s %11s %9s %8s\n', 'Case', '|e(T)|[rad]', ...
    'max|e|>T', 'RMS(e)[rad]', 'max|tau|', 'sat[%]', 't_set[s]');
for c = 1:numel(results)
    m = results(c).metrics;
    fprintf('%-38s %11.3e %11.3e %11.3e %11.2f %9.1f %8.2f\n', ...
        results(c).name, m.final_e_norm, m.max_e_after_T, m.rms_e, ...
        m.max_tau, 100*m.saturation_ratio, m.settle_time_e);
end

if cfg.save_figures
    save(cfg.result_file, 'cfg', 'ctrl', 'cases', 'results', 'metrics', 'ref', 'P');
    plot_paper_figures(results, ref, P, cfg);
    fprintf('\nResults saved to %s\n', cfg.result_file);
end

end

%% ========================================================================
%                              Simulation core
% ========================================================================
function out = run_case(case_cfg, cfg, ref, ctrl, P)

n = cfg.n;  N = cfg.N;

q   = zeros(n, N);   dq  = zeros(n, N);   ddq = zeros(n, N);
tau = zeros(n, N);   e   = zeros(n, N);   edot = zeros(n, N);
zlog = zeros(2*n, N);
Rhlog = zeros(1, N);  Vlog = zeros(1, N);
da_log = zeros(n, N); da_hat_log = zeros(n, N); da_err_log = zeros(n, N);
base_pos = zeros(3, N); base_att = zeros(3, N);
ee_pos = zeros(3, N);
tau_des_log = zeros(n, N);
sat_count = 0;

q(:,1) = cfg.q_init;
dq(:,1) = cfg.dq_init;
if case_cfg.use_large_error
    q(:,1) = cfg.q_init_sat;
end

% base pose (inertial): position and ZYX Euler angles
pb = zeros(3,1); Rb = eye(3);

% actuator limits
if case_cfg.use_large_error
    tau_max = cfg.tau_max_low;
else
    tau_max = cfg.tau_max;
end

% observer state
o.z1 = zeros(n,1); o.z2 = zeros(n,1);
T_o = ctrl.observer.T_o;

for k = 1:N-1
    tk = cfg.t(k);

    %% errors
    e(:,k)    = q(:,k) - ref.qd(:,k);
    edot(:,k) = dq(:,k) - ref.dqd(:,k);
    z = [e(:,k); edot(:,k)];
    zlog(:,k) = z;

    if k > cfg.Nh
        z_delay = zlog(:, k - cfg.Nh);
    else
        z_delay = zlog(:, 1);
    end

    %% plant (exact, parameter-based)
    [Me, Cvec, Hbb, Hbm] = ffsm_dou_reduced(P, q(:,k), dq(:,k));

    %% lumped disturbance (torque channel)
    % The lumped disturbance is scaled by the nominal joint inertia, i.e. it
    % represents an acceleration-level disturbance
    %     Delta_a = M_e^{-1} d ~ diag(Me0/Me) * delta(t),
    %     delta_i(t) = 0.03 sin(0.7t + 0.3i) + 0.015 cos(1.3t + 0.2i),
    % which is the form produced by parameter errors Delta H, Delta C.
    if case_cfg.use_disturbance
        delta_i = cfg.dist_amp*sin(0.7*tk + 0.3*(1:n)') + 0.5*cfg.dist_amp*cos(1.3*tk + 0.2*(1:n)');
        d = cfg.Me0 .* delta_i;
    else
        d = zeros(n,1);
    end
    delta_a = Me \ d;

    %% PTDO
    da_hat = zeros(n,1);
    if case_cfg.use_observer
        da_hat = o.z2;
    end

    %% PDF control
    switch case_cfg.controller_mode
        case 'pd'
            nu = ctrl.K0 * z;
            Rh = 0;
        case 'pdf'
            [nu, Rh] = pdf_control(tk - T_o, z, z_delay, ctrl, cfg.h);
        otherwise
            error('unknown controller mode %s', case_cfg.controller_mode);
    end
    Rhlog(k) = Rh;

    u_aux = ref.ddqd(:,k) + nu - da_hat;
    tau_cmd = Me * u_aux + Cvec;

    %% actuator saturation
    tau_des_log(:,k) = tau_cmd;
    tau_k = min(max(tau_cmd, -tau_max), tau_max);
    tau(:,k) = tau_k;
    sat_count = sat_count + any(abs(tau_k - tau_cmd) > 1e-9);

    %% PTDO update
    u_obs = Me \ (tau_k - Cvec);
    if case_cfg.use_observer
        o = update_ptdo(o, dq(:,k), u_obs, tk, cfg.Ts, ctrl.observer);
    end

    %% integrate reduced dynamics
    ddq(:,k) = Me \ (tau_k + d - Cvec);
    dq(:,k+1) = dq(:,k) + cfg.Ts*ddq(:,k);
    q(:,k+1)  = q(:,k)  + cfg.Ts*dq(:,k+1);

    %% base reaction motion (momentum conservation) and end-effector motion
    vb = -Hbb \ (Hbm * dq(:,k+1));
    omega_b = vb(1:3);  v_b = vb(4:6);
    Rb = Rb * expm(skew(omega_b) * cfg.Ts);
    pb = pb + Rb * (v_b * cfg.Ts);
    base_pos(:,k+1) = pb;
    base_att(:,k+1) = rotm2eul_zyx(Rb);
    ee_pos(:,k+1) = pb + Rb * ffsm_dou_fk(P, q(:,k+1));

    if cfg.diag && abs(tk - round(tk)) < 1e-9
        fprintf('  t=%4.1f  |e|=%.3e  |ed|=%.3e  max|tau|=%.2f  sat=%d  |daerr|=%.3e\n', ...
            tk, norm(e(:,k)), norm(edot(:,k)), max(abs(tau_k)), ...
            any(abs(tau_k - tau_cmd) > 1e-9), norm(delta_a - da_hat));
    end
    Vlog(k) = 0.5*(e(:,k).'*e(:,k) + edot(:,k).'*edot(:,k));
    da_log(:,k) = delta_a;
    da_hat_log(:,k) = da_hat;
    da_err_log(:,k) = delta_a - da_hat;
end

% last sample
e(:,N) = q(:,N) - ref.qd(:,N);
edot(:,N) = dq(:,N) - ref.dqd(:,N);
zlog(:,N) = [e(:,N); edot(:,N)];
tau(:,N) = tau(:,N-1);  ddq(:,N) = ddq(:,N-1);
tau_des_log(:,N) = tau_des_log(:,N-1);
da_log(:,N) = da_log(:,N-1);  da_hat_log(:,N) = da_hat_log(:,N-1);
da_err_log(:,N) = da_err_log(:,N-1);
Vlog(N) = Vlog(N-1);  Rhlog(N) = Rhlog(N-1);
base_pos(:,N) = base_pos(:,N-1); base_att(:,N) = base_att(:,N-1);
ee_pos(:,N) = ee_pos(:,N-1);

out = empty_result();
out.name = case_cfg.name;
out.t = cfg.t;  out.q = q;  out.dq = dq;  out.ddq = ddq;  out.tau = tau;
out.tau_desired = tau_des_log;
out.e = e;  out.edot = edot;  out.z = zlog;
out.Rh = Rhlog;  out.V = Vlog;
out.delta_a = da_log;  out.delta_a_hat = da_hat_log;  out.delta_a_error = da_err_log;
out.base_pos = base_pos;  out.base_att = base_att;  out.ee_pos = ee_pos;
out.saturation_ratio = sat_count/(N-1);
out.use_observer = case_cfg.use_observer;
out.metrics = compute_metrics(cfg.t, e, edot, tau, da_err_log, ...
    out.saturation_ratio, case_cfg.use_observer);
end

%% ------------------------------------------------------------------------
function [nu, Rh] = pdf_control(t, z, z_delay, ctrl, h)
Rh = smooth_periodic_Rh(t, h);
if Rh == 0
    nu = ctrl.K0 * z;
else
    Kc = Rh * (ctrl.pdf.B' * expm(-ctrl.pdf.Ac' * mod(t, 2*h)) ...
        * ctrl.pdf.W_inv * expm(ctrl.pdf.Ac * (h - mod(t, 2*h))));
    nu = ctrl.K0 * z - Kc * z_delay;
end
end

function Rh = smooth_periodic_Rh(t, h)
theta = mod(t, 2*h);
if theta < h || t < 0
    Rh = 0;
else
    Rh = sin(pi*(theta - h)/h)^4;
end
end

function pdf = build_pdf_gain(K0, n, h)
A = [zeros(n), eye(n); zeros(n), zeros(n)];
B = [zeros(n); eye(n)];
Ac = A + B*K0;
grid = linspace(h, 2*h, 4001);
W = zeros(2*n);
for k = 1:numel(grid)
    s = grid(k);
    R = sin(pi*(s-h)/h)^4;
    E = expm(-Ac*s);
    wgt = 0.5 + 0.5*(k > 1 && k < numel(grid));
    W = W + wgt * E * B * (R*eye(n)) * B' * E';
end
W = W * (grid(2)-grid(1));
W = 0.5*(W + W');
pdf.A = A; pdf.B = B; pdf.Ac = Ac; pdf.W = W;
pdf.W_inv = pinv(W + 1e-10*eye(2*n));
pdf.gramian_condition = cond(W);
pdf.h = h;
end

function obs = update_ptdo(obs, chi, u_o, t, Ts, p)
% Prescribed-time homogeneous observer (paper Eq. (18)):
%   dot z1 = z2 + u_o + l1(t) phi1(eps1/sigma)
%   dot z2 = l2(t) phi2(eps1/sigma),   eps1 = chi - z1
% with l1(t) = m1/(T_o-t)+, l2(t) = m2/(T_o-t)^2+. The gains are frozen at
% t = T_o - delta to keep the discrete-time implementation bounded.
r  = max(p.T_o - t, p.delta);
l1 = p.m1 / r;
l2 = p.m2 / r^2;

se = (chi - obs.z1) / p.sigma;
phi1 = sign(se).*abs(se).^(1 - p.eta/2) + sign(se).*abs(se).^(1 + p.eta/2);
phi2 = sign(se).*abs(se).^(1 - p.eta) + sign(se).*abs(se).^(1 + p.eta);

z1_dot = obs.z2 + u_o + l1*phi1;
z2_dot = l2*phi2;

obs.z1 = obs.z1 + Ts*z1_dot;
obs.z2 = obs.z2 + Ts*z2_dot;
end

%% ------------------------------------------------------------------------
function [qd, dqd, ddqd] = quintic_joint_trajectory(q0, qf, Tref, t)
% Per-joint fifth-order polynomial. Tref is a scalar or an n-vector of
% completion times.
if isscalar(Tref)
    Tref = Tref * ones(numel(q0), 1);
end
s = min(t ./ Tref(:), 1);                       % n x N
phi = 10*s.^3 - 15*s.^4 + 6*s.^5;
dphi = (30*s.^2 - 60*s.^3 + 30*s.^4) ./ Tref(:);
ddphi = (60*s - 180*s.^2 + 120*s.^3) ./ (Tref(:).^2);
dphi(s >= 1) = 0;  ddphi(s >= 1) = 0;
delta = qf - q0;
qd = q0 + delta .* phi;
dqd = delta .* dphi;
ddqd = delta .* ddphi;
end

function S = skew(x)
S = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end

function ang = rotm2eul_zyx(R)
ang = [atan2(R(2,1), R(1,1));
       asin(max(min(-R(3,1), 1), -1));
       atan2(R(3,2), R(3,3))];
end

function c = make_case(name, controller_mode, use_disturbance, use_observer)
c.name = char(name);
c.controller_mode = char(controller_mode);
c.use_disturbance = use_disturbance;
c.use_observer = use_observer;
c.use_large_error = false;
end

function out = empty_result()
out.name = ''; out.t = []; out.q = []; out.dq = []; out.ddq = [];
out.tau = []; out.tau_desired = []; out.e = []; out.edot = []; out.z = [];
out.Rh = []; out.V = []; out.delta_a = []; out.delta_a_hat = [];
out.delta_a_error = []; out.base_pos = []; out.base_att = [];
out.ee_pos = []; out.saturation_ratio = 0; out.use_observer = false;
out.metrics = struct();
end

function m = compute_metrics(t, e, edot, tau, da_err, sat_ratio, use_obs)
en = vecnorm(e, 2, 1);  edn = vecnorm(edot, 2, 1);
m.final_e_norm = en(end);
m.final_edot_norm = edn(end);
m.max_e_norm = max(en);
m.rms_e = sqrt(mean(e.^2, 'all'));
m.rms_edot = sqrt(mean(edot.^2, 'all'));
m.max_tau = max(abs(tau(:)));
m.saturation_ratio = sat_ratio;
m.settle_time_e = settling_time(t, en, 1e-4);
iT = find(t >= 3.0, 1);
m.max_e_after_T = max(en(iT:end));
if use_obs
    m.final_obs_error_norm = norm(da_err(:,end));
    m.rms_obs_error = sqrt(mean(da_err.^2, 'all'));
else
    m.final_obs_error_norm = NaN;
    m.rms_obs_error = NaN;
end
end

function ts = settling_time(t, sig, thr)
ts = NaN;
for k = 1:numel(t)
    if all(sig(k:end) <= thr)
        ts = t(k); return;
    end
end
end

function m = collect_metrics(results)
m = struct([]);
for c = 1:numel(results)
    m(c).name = results(c).name;
    m(c).final_e_norm = results(c).metrics.final_e_norm;
    m(c).final_edot_norm = results(c).metrics.final_edot_norm;
    m(c).max_tau = results(c).metrics.max_tau;
    m(c).saturation_ratio = results(c).metrics.saturation_ratio;
    m(c).settle_time_e = results(c).metrics.settle_time_e;
    m(c).max_e_after_T = results(c).metrics.max_e_after_T;
    m(c).final_obs_error_norm = results(c).metrics.final_obs_error_norm;
end
end
