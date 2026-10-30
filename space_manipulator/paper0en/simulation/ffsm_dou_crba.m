function [H_full, kin] = ffsm_dou_crba(P, q)
%FFSM_DOU_CRBA  Composite rigid-body algorithm for the floating-base chain.
%
%   [H_full, kin] = FFSM_DOU_CRBA(P, q) returns the (6+n)x(6+n) spatial
%   inertia matrix
%
%       H_full = [ H_b   H_bm ;
%                  H_bm' H_m  ]
%
%   expressed in the base frame attached to the spacecraft centre of mass,
%   using the modified-DH chain of DOU2025_PARAMETERS:
%
%       A_i = Rx(alpha_i) Tx(L_i) Rz(theta_i) Tz(d_i),  theta_i = q_i + thoff_i
%
%   Body 0 is the base with a 6-DOF joint (motion subspace S_0 = I_6);
%   body i (i = 1..n) is link i, attached to its parent by a revolute joint
%   about z_i (S_i = e_z). Spatial algebra follows Featherstone: a motion
%   transform X maps parent coordinates to child coordinates,
%   v_child = X v_parent + S qd, and forces transform as f_parent = X' f_child.
%
%   kin.X    : {1 x n} motion transforms X_{i-1,i}
%   kin.S    : {1 x n} joint motion subspaces (in child coordinates)
%   kin.I    : {1 x n+1} spatial inertias (index 1 = base)
%   kin.F    : {1 x n+1} composite rigid-body inertias (index 1 = base)
%   kin.sig  : {1 x n} joint axes expressed in body frames, sig{i}{k} = X_{i,k} S_i
%   kin.Xcum : {1 x n} base-to-body motion transforms X_{0,i}
%   kin.T    : {1 x n} homogeneous transforms T_i^{i-1} (T_1 includes base offset)
%   kin.pj   : {1 x n+1} joint origins in the base frame

n = P.n;

%% Spatial inertia of the base (body 0), expressed at the base CM
Isp = cell(1, n + 1);
Isp{1} = spatial_inertia(P.base.mass, P.base.cm, P.base.inertia);

X = cell(1, n);
S = cell(1, n);
T = cell(1, n);
Xcum = cell(1, n);
pj = cell(1, n + 1);
pj{1} = zeros(3, 1);            % base frame origin (base CM)
Rcum = eye(3);                  % rotation from the base frame to frame i-1

%% Forward kinematics of the MDH chain
for i = 1:n
    al = P.alpha(i);
    th = q(i) + P.thoff(i);
    Li = P.L(i);
    di = P.d(i);

    R = rotx(al) * rotz(th);                                % orientation
    p = rotx(al) * ([Li; 0; 0] + rotz(th) * [0; 0; di]);    % child origin in parent

    if i == 1
        % fixed base -> joint 1 offset, then the first MDH transform
        p = p + Rcum * P.base_to_j1;   % Rcum = I
    end

    T{i} = [R, p; 0, 0, 0, 1];

    Xi = [R.', zeros(3); -R.' * skew3(p), R.'];
    X{i} = Xi;
    S{i} = [0; 0; 1; 0; 0; 0];

    if i == 1
        Xcum{i} = Xi;
    else
        Xcum{i} = Xi * Xcum{i-1};
    end

    Isp{i+1} = spatial_inertia(P.mass(i), P.rc(:, i), P.Icm{i});

    pj{i+1} = pj{i} + Rcum * p;
    Rcum = Rcum * R;
end

%% Backward accumulation: composite rigid-body inertias
F = Isp;
for i = n:-1:1
    F{i} = F{i} + X{i}.' * F{i+1} * X{i};
end
% F{1} is the composite inertia of the whole system, in the base frame

%% Joint axes expressed in every body frame: sig{i}{k} = X_{i,k} S_i, k >= i
sig = cell(1, n);
for i = 1:n
    sig{i} = cell(1, n);
    sig{i}{i} = S{i};
    for k = i+1:n
        sig{i}{k} = X{k} * sig{i}{k-1};
    end
end

%% Generalized inertia matrix
H_full = zeros(6 + n, 6 + n);

% base block (all bodies in the base frame)
H_full(1:6, 1:6) = F{1};

% base-arm coupling: X_{0,j}' F_j S_j
for j = 1:n
    H_full(1:6, 6 + j) = Xcum{j}.' * (F{j+1} * S{j});
end
H_full(7:end, 1:6) = H_full(1:6, 7:end).';

% arm block: H(i,j) = sig_i(max)' F_max sig_j(max)
for i = 1:n
    for j = 1:n
        m = max(i, j);
        H_full(6+i, 6+j) = sig{i}{m}.' * F{m+1} * sig{j}{m};
    end
end

H_full = 0.5 * (H_full + H_full.');

kin.X = X;
kin.S = S;
kin.I = Isp;
kin.F = F;
kin.sig = sig;
kin.Xcum = Xcum;
kin.T = T;
kin.pj = pj;

end

%% ------------------------------------------------------------------------
function I = spatial_inertia(m, c, Ic)
%SPATIAL_INERTIA  Spatial inertia at the frame origin for a body with mass m,
%   centre of mass c (in the frame) and rotational inertia Ic about the CM.
    C = skew3(c);
    I = [Ic - m * C * C, m * C;
         -m * C,          m * eye(3)];
    I = 0.5 * (I + I.');
end

function Sx = skew3(x)
    Sx = [    0, -x(3),  x(2);
          x(3),     0, -x(1);
         -x(2),  x(1),     0];
end

function R = rotx(a)
    R = [1, 0, 0; 0, cos(a), -sin(a); 0, sin(a), cos(a)];
end

function R = rotz(a)
    R = [cos(a), -sin(a), 0; sin(a), cos(a), 0; 0, 0, 1];
end
