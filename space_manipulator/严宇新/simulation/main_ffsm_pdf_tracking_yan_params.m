%% main_ffsm_pdf_tracking_yan_params.m
% -------------------------------------------------------------------------
% Free-floating space manipulator tracking control with smooth periodic
% delayed feedback (PDF) using physical parameters from Yan Yuxin thesis.
%
% Model basis:
%   H_g(q) qdd + C_g(q,qd) qd = tau + d(t)
%
% Control:
%   tau = H0(q) [qdd_d + v_pdf] + C0(q,qd) qd
%
%   z(t) = [e(t); edot(t)]
%   v_pdf = K0 z(t) + Kh(t) z(t-h)
%
% Important academic note:
%   The exact H_g and C_g of the free-floating 7-DOF space manipulator
%   require full kinematic recursion and generalized Jacobian construction.
%   In this runnable program, a positive-definite nominal joint-space model
%   is used for demonstration. Replace ffsm_dynamics_nominal() by the exact
%   Yan model implementation if available.
%
% Author: generated for academic simulation use
% -------------------------------------------------------------------------

clear; clc; close all;

%% -------------------- 1. Simulation settings ----------------------------
Ts   = 1e-3;          % integration step [s]
Tf   = 10.0;          % final simulation time [s]
t    = 0:Ts:Tf;
N    = numel(t);

n    = 7;             % 7-DOF manipulator

save_figures = true;
show_figures = false;
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end
figure_dir = fullfile(script_dir, '..', 'figures');
if save_figures && ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end
if ~show_figures
    set(0, 'DefaultFigureVisible', 'off');
end

%% -------------------- 2. Parameters from Yan Yuxin Table 2-1 ------------
P = yan_yuxin_parameters();

%% -------------------- 3. Initial states and desired trajectory ----------
% Initial joint angles in Section 3.7:
% q(0) = [0, pi/3, 0, pi/4, pi/4, 0, pi/6]^T rad
q0 = [0; pi/3; 0; pi/4; pi/4; 0; pi/6];
dq0 = zeros(n,1);

% Choose a final joint configuration for tracking simulation.
% This is a joint-space reference used for control demonstration.
% If using Cartesian trajectory from the thesis, first generate q_d(t)
% through generalized Jacobian inverse planning, then use it here.
qf = [pi/18; pi/6; pi/5; 0; -pi/4; -pi/3; pi/12];

% Generate 5th-order polynomial desired trajectory over Tref seconds.
Tref = 8.0;
[qd, dqd, ddqd] = quintic_joint_trajectory(q0, qf, Tref, t);

%% -------------------- 4. PDF controller parameters ----------------------
% Delay h. For the scalar PDF result, prescribed time is T = 2h.
% For this nonlinear robot simulation, h is used as the artificial delay.
h = 1.0;                          % delay [s]
Nh = round(h/Ts);                 % delay samples

% Smooth periodic function R_h(t), 2h-periodic and zero on [0,h].
% It activates delayed feedback in [h,2h].
pdf.h = h;
pdf.r_order = 2;

% Baseline stabilizing gain for double-integrator tracking error.
% z = [e; edot], v = K0*z + Kh(t)*z_delay.
Kp = diag([25 25 25 22 22 20 20]);
Kd = diag([10 10 10  9  9  8  8]);

K0 = [-Kp, -Kd];                  % n x 2n

% Delayed feedback gain. Its time-varying amplitude is shaped by R_h(t).
% The following is a smooth periodic delayed feedback term for simulation.
% For a rigorous nilpotent monodromy design, Kdelay must be computed by
% the exact method in Zhou-Michiels-Chen TAC 2022.
Kp_h = diag([8 8 8 6 6 5 5]);
Kd_h = diag([3 3 3 2.5 2.5 2 2]);
Kh0  = [-Kp_h, -Kd_h];            % n x 2n

% PDF amplitude
pdf_gain = 1.0;

%% -------------------- 5. Disturbance and uncertainty settings -----------
use_disturbance = true;

% torque saturation, set Inf for no saturation
tau_max = 200 * ones(n,1);        % [N*m], illustrative

%% -------------------- 6. Preallocate variables --------------------------
q    = zeros(n,N);
dq   = zeros(n,N);
ddq  = zeros(n,N);
tau  = zeros(n,N);
e    = zeros(n,N);
edot = zeros(n,N);
zlog = zeros(2*n,N);
Rhlog = zeros(1,N);
Vlog  = zeros(1,N);

q(:,1)  = q0;
dq(:,1) = dq0;

%% -------------------- 7. Main simulation loop ---------------------------
for k = 1:N-1

    tk = t(k);

    % Current tracking error
    e(:,k)    = q(:,k)  - qd(:,k);
    edot(:,k) = dq(:,k) - dqd(:,k);
    z         = [e(:,k); edot(:,k)];
    zlog(:,k)= z;

    % Delayed error state z(t-h)
    if k > Nh
        z_delay = zlog(:,k-Nh);
    else
        % Initial history: hold initial error history constant
        z_delay = zlog(:,1);
    end

    % Smooth periodic delayed feedback activation
    Rh = smooth_periodic_Rh(tk, h);
    Rhlog(k) = Rh;

    % PDF control acceleration
    v_pdf = K0*z + pdf_gain*Rh*(Kh0*z_delay);

    % Nominal dynamics
    [H0, C0] = ffsm_dynamics_nominal(q(:,k), dq(:,k), P);

    % Computed torque with PDF acceleration
    tau_cmd = H0*(ddqd(:,k) + v_pdf) + C0*dq(:,k);

    % Saturation
    tau_cmd = max(min(tau_cmd, tau_max), -tau_max);
    tau(:,k) = tau_cmd;

    % External disturbance
    if use_disturbance
        d = disturbance_torque(tk,n);
    else
        d = zeros(n,1);
    end

    % True dynamics.
    % Here the same nominal H0,C0 are used as the plant for demonstration.
    % Replace by exact H_g,C_g if available.
    H = H0;
    C = C0;

    ddq(:,k) = H \ (tau_cmd + d - C*dq(:,k));

    % Semi-implicit Euler integration
    dq(:,k+1) = dq(:,k) + Ts*ddq(:,k);
    q(:,k+1)  = q(:,k)  + Ts*dq(:,k+1);

    % Error energy-like value
    Vlog(k) = 0.5*(e(:,k).'*e(:,k) + edot(:,k).'*edot(:,k));
end

% Last sample
e(:,N)    = q(:,N)  - qd(:,N);
edot(:,N) = dq(:,N) - dqd(:,N);
zlog(:,N)= [e(:,N); edot(:,N)];
Vlog(N)  = 0.5*(e(:,N).'*e(:,N) + edot(:,N).'*edot(:,N));
Rhlog(N) = smooth_periodic_Rh(t(N), h);
tau(:,N) = tau(:,N-1);
ddq(:,N) = ddq(:,N-1);

%% -------------------- 8. Plot results ----------------------------------
plot_results(t, q, dq, qd, dqd, e, edot, tau, Rhlog, Vlog, figure_dir, save_figures);

%% -------------------- 9. Print final error ------------------------------
fprintf('\nFinal tracking error norm:\n');
fprintf('||e(T)||     = %.6e rad\n', norm(e(:,end)));
fprintf('||edot(T)||  = %.6e rad/s\n', norm(edot(:,end)));
fprintf('Max |tau_i|  = %.6f N*m\n', max(abs(tau(:))));
if save_figures
    fprintf('Figures saved to: %s\n', figure_dir);
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

    % Base inertia
    P.mb = P.mass(1);
    P.Ib = P.I{1};

    % Link masses and inertias only
    P.mi = P.mass(2:end);
    P.Ilink = P.I(2:end);
end

function [qd, dqd, ddqd] = quintic_joint_trajectory(q0, qf, Tref, t)
% Fifth-order polynomial trajectory with zero initial/final velocity
% and acceleration.

    n = numel(q0);
    N = numel(t);

    qd   = zeros(n,N);
    dqd  = zeros(n,N);
    ddqd = zeros(n,N);

    for k = 1:N
        tk = t(k);
        if tk <= Tref
            s = tk/Tref;
            phi   = 10*s^3 - 15*s^4 + 6*s^5;
            dphi  = (30*s^2 - 60*s^3 + 30*s^4)/Tref;
            ddphi = (60*s - 180*s^2 + 120*s^3)/(Tref^2);
        else
            phi   = 1;
            dphi  = 0;
            ddphi = 0;
        end

        qd(:,k)   = q0 + phi*(qf-q0);
        dqd(:,k)  = dphi*(qf-q0);
        ddqd(:,k) = ddphi*(qf-q0);
    end
end

function Rh = smooth_periodic_Rh(t,h)
% Smooth periodic delayed feedback shaping function.
%
% Required qualitative properties:
%   2h-periodic;
%   R_h(t)=0 for t in [0,h];
%   R_h(t)>=0 for t in [h,2h];
%   smooth at switching instants.
%
% Here use:
%   R_h = sin^4(pi*(theta-h)/h), theta in [h,2h],
%   R_h = 0, theta in [0,h).
%
% sin^4 gives C^2 smoothness at h and 2h.

    theta = mod(t,2*h);

    if theta < h
        Rh = 0;
    else
        s = (theta-h)/h;
        Rh = sin(pi*s)^4;
    end
end

function [H, C] = ffsm_dynamics_nominal(q,dq,P)
% Nominal joint-space model corresponding to:
%   H_g(q) qdd + C_g(q,dq) dq = tau
%
% This is a runnable positive-definite approximation using Table 2-1 data.
% It is NOT a substitute for the exact free-floating H_g and C_g.
%
% Replace this function by exact implementation:
%   H_g = H_m - H_bm' H_b^{-1} H_bm
%   C_g = H_bm' H_b^{-1}(C_b H_b^{-1}H_bm - C_bm)
%         - C_bm' H_b^{-1}H_bm + C_m

    n = P.n;

    % Construct an illustrative diagonal inertia from link inertia and
    % parallel-axis-like length contribution. This keeps H positive definite.
    Jdiag = zeros(n,1);
    for i = 1:n
        mi = P.mi(i);
        Ii = P.Ilink{i};
        ai = P.a(:,i+1);
        bi = P.b(:,i+1);

        % Conservative positive inertia scale, not exact FFSM inertia.
        len2 = max(norm(ai)^2 + norm(bi)^2, 1e-3);
        Jdiag(i) = Ii(3,3) + 0.15*mi*len2 + 0.05;
    end

    % Add weak configuration-dependent coupling while maintaining SPD.
    H = diag(Jdiag);
    for i = 1:n
        for j = i+1:n
            coupling = 0.02*sqrt(Jdiag(i)*Jdiag(j))*cos(q(i)-q(j));
            H(i,j) = coupling;
            H(j,i) = coupling;
        end
    end

    % Ensure positive definiteness numerically.
    H = 0.5*(H+H.');
    mineig = min(eig(H));
    if mineig <= 1e-6
        H = H + (abs(mineig)+1e-3)*eye(n);
    end

    % Simple viscous/Coriolis-like term.
    % For exact model, compute C_g(q,dq).
    Dv = diag(0.05 + 0.02*abs(dq));
    C = Dv;
end

function d = disturbance_torque(t,n)
% Bounded time-varying disturbance torque.
    d = zeros(n,1);
    for i = 1:n
        d(i) = 0.15*sin(0.7*t + 0.3*i) + 0.05*cos(1.3*t + 0.2*i);
    end
end

function plot_results(t, q, dq, qd, dqd, e, edot, tau, Rhlog, Vlog, figure_dir, save_figures)

    n = size(q,1);

    figure('Name','Joint position tracking','Color','w');
    tiledlayout(4,2,'Padding','compact','TileSpacing','compact');
    for i = 1:n
        nexttile;
        plot(t, q(i,:), 'b', 'LineWidth', 1.2); hold on;
        plot(t, qd(i,:), 'r--', 'LineWidth', 1.2);
        grid on;
        xlabel('Time [s]');
        ylabel(sprintf('q_%d [rad]',i));
        if i == 1
            legend('Actual','Desired');
        end
    end
    save_figure(gcf, figure_dir, 'joint_position_tracking', save_figures);

    figure('Name','Joint velocity tracking','Color','w');
    tiledlayout(4,2,'Padding','compact','TileSpacing','compact');
    for i = 1:n
        nexttile;
        plot(t, dq(i,:), 'b', 'LineWidth', 1.2); hold on;
        plot(t, dqd(i,:), 'r--', 'LineWidth', 1.2);
        grid on;
        xlabel('Time [s]');
        ylabel(sprintf('dq_%d [rad/s]',i));
        if i == 1
            legend('Actual','Desired');
        end
    end
    save_figure(gcf, figure_dir, 'joint_velocity_tracking', save_figures);

    figure('Name','Tracking errors','Color','w');
    tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

    nexttile;
    plot(t, e, 'LineWidth', 1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('e_i [rad]');
    title('Joint position tracking errors');
    legend(arrayfun(@(i)sprintf('e_%d',i),1:n,'UniformOutput',false), ...
           'Location','eastoutside');

    nexttile;
    plot(t, edot, 'LineWidth', 1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('de_i [rad/s]');
    title('Joint velocity tracking errors');
    legend(arrayfun(@(i)sprintf('de_%d',i),1:n,'UniformOutput',false), ...
           'Location','eastoutside');
    save_figure(gcf, figure_dir, 'tracking_errors', save_figures);

    figure('Name','Control torque','Color','w');
    plot(t, tau, 'LineWidth', 1.1);
    grid on;
    xlabel('Time [s]');
    ylabel('\tau_i [N m]');
    title('Joint control torques');
    legend(arrayfun(@(i)sprintf('\\tau_%d',i),1:n,'UniformOutput',false), ...
           'Location','eastoutside');
    save_figure(gcf, figure_dir, 'control_torque', save_figures);

    figure('Name','PDF activation and error energy','Color','w');
    tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

    nexttile;
    plot(t, Rhlog, 'k', 'LineWidth', 1.4);
    grid on;
    xlabel('Time [s]');
    ylabel('R_h(t)');
    title('Smooth periodic delayed feedback activation');

    nexttile;
    semilogy(t, Vlog + 1e-16, 'm', 'LineWidth', 1.4);
    grid on;
    xlabel('Time [s]');
    ylabel('V(t)');
    title('Error energy-like quantity');
    save_figure(gcf, figure_dir, 'pdf_activation_error_energy', save_figures);

    figure('Name','Error norm','Color','w');
    plot(t, vecnorm(e,2,1), 'b', 'LineWidth', 1.5); hold on;
    plot(t, vecnorm(edot,2,1), 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Norm');
    legend('||e||','||\dot e||');
    title('Tracking error norms');
    save_figure(gcf, figure_dir, 'error_norm', save_figures);
end

function save_figure(fig, figure_dir, stem, save_figures)
% Save each simulation figure in raster and vector formats for LaTeX use.

    if ~save_figures
        return;
    end

    if ~exist(figure_dir, 'dir')
        mkdir(figure_dir);
    end

    png_file = fullfile(figure_dir, [stem, '.png']);
    pdf_file = fullfile(figure_dir, [stem, '.pdf']);

    try
        exportgraphics(fig, png_file, 'Resolution', 300);
        exportgraphics(fig, pdf_file, 'ContentType', 'vector');
    catch
        set(fig, 'PaperPositionMode', 'auto');
        print(fig, png_file, '-dpng', '-r300');
        print(fig, pdf_file, '-dpdf', '-bestfit');
    end
end
