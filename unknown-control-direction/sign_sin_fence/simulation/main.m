clear; clc; close all;

%% ================================================================
%  Periodic Sliding Mode Fencing Simulation
%  Agents:
%       p_dot = v
%       v_dot = B_i(t) u_i + d_i
%
%  Target:
%       p0_dot = v0
%       v0_dot = u0
%
%  Unknown sign-switching control direction:
%       B_i(t) = diag(b_i1(t), b_i2(t))
%
%  Controller:
%       u_i,l = R_i,l * sgn( sin(pi/eps_i,l * s_i,l) )
%
% ================================================================

%% Simulation parameters
par.N  = 4;             % number of agents
par.m  = 2;             % dimension, 2D
par.T  = 30;            % total simulation time
par.dt = 1e-5;          % time step
par.t  = 0:par.dt:par.T;
par.Nt = length(par.t);

%% Control parameters
par.k1 = 1.2;
par.k2 = 2.0;

par.eps_s = 0.25 * ones(par.N, par.m);     % epsilon_{i,l}
par.lambda = 0.8 * ones(par.N, par.m);     % lambda_{i,l}

% Lower bound of |b_i,l(t)|
par.b_min = 0.8 * ones(par.N, par.m);

% Disturbance and target acceleration bounds
par.d_bar = 0.20 * ones(par.N, 1);
par.u0_bar = 0.15;

%% Collision avoidance / repulsion parameters
par.d_safe = 0.45;
par.mu     = 1.20;

%% Smooth or discontinuous sign
% If use_smooth = false, ideal sign is used.
% If use_smooth = true, tanh approximation is used to reduce numerical chattering.
par.use_smooth = false;
par.phi = 0.02;

%% Initial states
% Agents are initialized around the target, not perfectly symmetric.
p = zeros(par.N, par.m);
v = zeros(par.N, par.m);
eta = zeros(par.N, par.m);

p(1,:) = [ 2.0,  0.4];
p(2,:) = [ 0.4,  1.8];
p(3,:) = [-1.8, -0.3];
p(4,:) = [-0.2, -1.9];

v(1,:) = [ 0.0,  0.1];
v(2,:) = [-0.1,  0.0];
v(3,:) = [ 0.0, -0.1];
v(4,:) = [ 0.1,  0.0];

% Target initial state
p0 = [0.0, 0.0];
v0 = [0.15, 0.0];

%% History arrays
hist.p    = zeros(par.Nt, par.N, par.m);
hist.v    = zeros(par.Nt, par.N, par.m);
hist.eta  = zeros(par.Nt, par.N, par.m);
hist.p0   = zeros(par.Nt, par.m);
hist.v0   = zeros(par.Nt, par.m);
hist.u    = zeros(par.Nt, par.N, par.m);
hist.s    = zeros(par.Nt, par.N, par.m);
hist.sdot = zeros(par.Nt, par.N, par.m);
hist.Bdiag = zeros(par.Nt, par.N, par.m);
hist.phi_rep = zeros(par.Nt, par.N, par.m);
hist.Delta   = zeros(par.Nt, par.N, par.m);

%% Main simulation loop
for k = 1:par.Nt

    t = par.t(k);

    %% Store states
    hist.p(k,:,:)   = p;
    hist.v(k,:,:)   = v;
    hist.eta(k,:,:) = eta;
    hist.p0(k,:)    = p0;
    hist.v0(k,:)    = v0;

    %% Compute controller
    [u, s, Delta, phi_rep] = controller_periodic_smc(t, p, v, eta, p0, v0, par);

    %% Compute input gain and disturbances for logging
    Bdiag = zeros(par.N, par.m);
    d = zeros(par.N, par.m);

    for i = 1:par.N
        Bdiag(i,:) = input_gain_matrix(t, i, par);
        d(i,:) = disturbance_agent(t, i, par);
    end

    u0 = target_acceleration(t, par);

    %% Compute sdot for logging
    sdot = zeros(par.N, par.m);
    for i = 1:par.N
        w_i = d(i,:) - u0;
        sdot(i,:) = Bdiag(i,:) .* u(i,:) + Delta(i,:) + w_i;
    end

    hist.u(k,:,:) = u;
    hist.s(k,:,:) = s;
    hist.sdot(k,:,:) = sdot;
    hist.Bdiag(k,:,:) = Bdiag;
    hist.phi_rep(k,:,:) = phi_rep;
    hist.Delta(k,:,:) = Delta;

    %% Stop at final sample
    if k == par.Nt
        break;
    end

    %% Euler integration
    [p_dot, v_dot, eta_dot, p0_dot, v0_dot] = ...
        agent_dynamics(t, p, v, eta, p0, v0, u, par);

    p   = p   + par.dt * p_dot;
    v   = v   + par.dt * v_dot;
    eta = eta + par.dt * eta_dot;

    p0  = p0  + par.dt * p0_dot;
    v0  = v0  + par.dt * v0_dot;
end

%% Plot
plot_results(hist, par);