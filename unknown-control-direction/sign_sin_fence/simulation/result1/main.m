clear; clc; close all;

%% ================================================================
%  Result 1 simulation for the current sliding variable
%
%      s_i0 = v_i0 + k1 p_i0 - phi_i + k2 eta_i
%      eta_dot_i = k1 p_i0 - phi_i
%
%  Agent:
%      p_dot_i = v_i
%      v_dot_i = B_i(t) u_i + d_i
%
%  Target:
%      p_dot_0 = v_0
%      v_dot_0 = u_0
%
%  Controller:
%      u_i,l = R_i,l sign(sin(pi s_i,l / eps_i,l))
%
%  The controller uses Delta_i with the analytic phi_dot_i.
% ================================================================

%% Simulation parameters
par.N  = 5;
par.m  = 2;
par.T  = 45;
par.dt = 2e-3;
par.t  = 0:par.dt:par.T;
par.Nt = numel(par.t);

%% Control parameters
par.k1 = 1.00;
par.k2 = 0.80;

par.eps_s  = 0.18 * ones(par.N, par.m);
par.lambda = 0.90 * ones(par.N, par.m);
par.b_min  = 0.75 * ones(par.N, par.m);

par.d_bar  = 0.08 * ones(par.N, 1);
par.u0_bar = 0.06;

%% Collision-avoidance parameters
par.d_safe = 0.45;
par.mu     = 1.30;
par.r_eps  = 1e-6;

%% Numerical sign choice
par.use_smooth = true;
par.phi = 0.03;

%% Initial states
p = zeros(par.N, par.m);
v = zeros(par.N, par.m);
eta = zeros(par.N, par.m);

p(1,:) = [ 2.20,  0.25];
p(2,:) = [ 0.90,  1.85];
p(3,:) = [-1.30,  1.30];
p(4,:) = [-2.10, -0.40];
p(5,:) = [ 0.25, -1.90];

v(1,:) = [-0.05,  0.06];
v(2,:) = [-0.06, -0.03];
v(3,:) = [ 0.04, -0.04];
v(4,:) = [ 0.07,  0.03];
v(5,:) = [ 0.02,  0.07];

p0 = [0.0, 0.0];
v0 = [0.10, 0.03];

%% History arrays
hist.p       = zeros(par.Nt, par.N, par.m);
hist.v       = zeros(par.Nt, par.N, par.m);
hist.eta     = zeros(par.Nt, par.N, par.m);
hist.p0      = zeros(par.Nt, par.m);
hist.v0      = zeros(par.Nt, par.m);
hist.u       = zeros(par.Nt, par.N, par.m);
hist.s       = zeros(par.Nt, par.N, par.m);
hist.sdot    = zeros(par.Nt, par.N, par.m);
hist.Bdiag   = zeros(par.Nt, par.N, par.m);
hist.phi_rep = zeros(par.Nt, par.N, par.m);
hist.phi_dot = zeros(par.Nt, par.N, par.m);
hist.Delta   = zeros(par.Nt, par.N, par.m);

%% Main simulation loop
for k = 1:par.Nt
    t = par.t(k);

    hist.p(k,:,:)   = p;
    hist.v(k,:,:)   = v;
    hist.eta(k,:,:) = eta;
    hist.p0(k,:)    = p0;
    hist.v0(k,:)    = v0;

    [u, s, Delta, phi_rep, phi_dot] = ...
        controller_periodic_smc(t, p, v, eta, p0, v0, par);

    Bdiag = zeros(par.N, par.m);
    d = zeros(par.N, par.m);
    for i = 1:par.N
        Bdiag(i,:) = input_gain_matrix(t, i, par);
        d(i,:) = disturbance_agent(t, i, par);
    end

    u0 = target_acceleration(t, par);

    sdot = zeros(par.N, par.m);
    for i = 1:par.N
        w_i = d(i,:) - u0;
        sdot(i,:) = Bdiag(i,:) .* u(i,:) + Delta(i,:) + w_i;
    end

    hist.u(k,:,:)       = u;
    hist.s(k,:,:)       = s;
    hist.sdot(k,:,:)    = sdot;
    hist.Bdiag(k,:,:)   = Bdiag;
    hist.phi_rep(k,:,:) = phi_rep;
    hist.phi_dot(k,:,:) = phi_dot;
    hist.Delta(k,:,:)   = Delta;

    if k == par.Nt
        break;
    end

    [p_dot, v_dot, eta_dot, p0_dot, v0_dot] = ...
        agent_dynamics(t, p, v, eta, p0, v0, u, par);

    p   = p   + par.dt * p_dot;
    v   = v   + par.dt * v_dot;
    eta = eta + par.dt * eta_dot;
    p0  = p0  + par.dt * p0_dot;
    v0  = v0  + par.dt * v0_dot;
end

plot_results(hist, par);
