function P = dou2025_parameters()
%DOU2025_PARAMETERS  Kinematic/dynamic parameters of the 7-DOF FFSM.
%
%   P = DOU2025_PARAMETERS() returns the parameter set reported in
%   B. Dou and X. Yue, "Fractional-Order Sliding Mode Control for
%   Free-Floating Space Manipulator With Disturbance and Input Saturation",
%   Int. J. Robust Nonlinear Control, 2025, doi:10.1002/rnc.7826
%   (Table 1: kinematic and dynamic parameters; Table 2: initial state).
%
%   Conventions adopted by this implementation (documented in the paper,
%   Section "Simulation Verification"):
%
%   * The chain is a modified-Denavit-Hartenberg (MDH) chain
%         A_i = Rx(alpha_i) Tx(L_i) Rz(theta_i) Tz(d_i),
%     so that joint i rotates about the z axis of link frame i and the
%     link vector L_i lies along the x axis of frame i. This reproduces
%     the tabulated inertia pattern (I_xx << I_yy ~= I_zz for the two
%     long links) and the tabulated link lengths L_4 = L_5 = 1.2 m with
%     the centre of mass at 0.6 m from the joint.
%   * alpha_i are the tabulated twist angles, d_i the tabulated joint
%     offsets, and theta_i = q_i + theta_off,i the joint variable with
%     the tabulated joint-angle offsets (theta_3 - 90 deg, theta_5 + 90 deg).
%   * The base frame is attached to the spacecraft centre of mass; the
%     first joint is located at (0, 0, 0.72) m in the base frame.
%   * Link inertias are given about the link centre of mass.
%
%   Masses, inertias, twists and offsets are taken verbatim from Table 1
%   of Dou & Yue (2025); no parameter is tuned.

deg = pi/180;

P.n = 7;
P.source = 'Dou & Yue, IJRC 2025 (doi:10.1002/rnc.7826), Tables 1-2';

%% Spacecraft base (rigid body 0)
P.base.mass    = 1000;                      % [kg]
P.base.inertia = diag([1200, 1200, 1200]);  % [kg m^2], about the CM
P.base.cm      = [0; 0; 0];                 % base CM in base frame

%% Links (rigid bodies 1..7)
P.mass = [35.01; 30; 22.69; 21.38; 16.75; 26.17; 18.1];   % [kg]

Ixx = [1.218, 2.1,   0.102,  0.4327, 0.3878, 0.5727, 0.17  ];
Iyy = [0.5132, 1.378, 3.378, 2.266,  0.3963, 0.5987, 0.24  ];
Izz = [1.331, 2.359, 3.359, 1.911,  0.07271, 0.1288, 0.14 ];

P.Icm = cell(1, P.n);
for i = 1:P.n
    P.Icm{i} = diag([Ixx(i), Iyy(i), Izz(i)]);
end

% Kinematics (Table 1)
P.alpha  = [90, 90, -90, 0, 0, 90, -90] * deg;      % twist angles  [rad]
P.d      = [0.18, 0.22, 0.20, 0.24, 0.22, 0.20, 0.30];   % joint offsets [m]
P.L      = [0, 0, 0, 1.2, 1.2, 0, 0];               % link lengths   [m]
P.thoff  = [0, 0, -90, 0, 90, 0, 0] * deg;           % joint offsets  [rad]

% Centre of mass of link i in link frame i (mid-link for the long links)
P.a = 0.5 * P.L;
P.rc = [P.a; zeros(1, P.n); zeros(1, P.n)];

P.base_to_j1 = [0; 0; 0.72];                        % [m], base CM -> joint 1

% Actuator limits and initial configuration (Tables 1 and 2)
P.tau_max = 20 * ones(P.n, 1);                      % [N m]
P.q0      = [0, -90, 45, -45, 90, 90, 0]' * deg;    % Table 2 initial joint angles
P.dq0     = zeros(P.n, 1);

end
