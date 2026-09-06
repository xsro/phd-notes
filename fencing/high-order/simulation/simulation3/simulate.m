%SIMULATE  Generate data for the heterogeneous target-fencing demo.
%   Run this script directly to generate out/safe_case/result.mat and
%   out/stress_case/result.mat. Figure generation is intentionally handled by
%   save_plots.m so that data generation and plotting can be rerun separately.

clc;
close all;

base_dir = fileparts(mfilename('fullpath'));
out_root = fullfile(base_dir, 'out');
if ~exist(out_root, 'dir')
    mkdir(out_root);
end

safe_params = default_params();
safe_params.case_name = 'safe_case';
safe_results = run_case(safe_params, fullfile(out_root, 'safe_case'));

% Keep the previous one-case output path for compatibility.
params = safe_params; %#ok<NASGU>
results = safe_results; %#ok<NASGU>
save(fullfile(out_root, 'result.mat'), 'results', 'params');

stress_params = default_params();
stress_params.case_name = 'stress_case';
stress_params.initial_layout = 'head_on';
stress_params.gamma = [0.45, 0.45, 0.45, 0.45];
stress_params.ka = [1.2, 1.3, 1.4, 1.5];
stress_params.kb = [1.2, 1.3, 1.4, 1.5];
stress_params.repulsion_gain = 0.04;
stress_params.t_final = 100.0;
stress_params.t_samples = 5001;
stress_params.max_step = 0.002;
stress_results = run_case(stress_params, fullfile(out_root, 'stress_case'));

save(fullfile(out_root, 'comparison.mat'), ...
    'safe_results', 'safe_params', 'stress_results', 'stress_params');

%--------------------------------------------------------------------------
function params = default_params()
params = struct();
params.dim = 2;
params.orders = [1, 2, 3, 4];
params.N = numel(params.orders);
params.kappa = [1.00, 1.02, 1.04, 1.06];
params.gamma = [2.5, 2.5, 2.5, 2.5];
params.ka = [10.0, 11.0, 12.0, 13.0];
params.kb = [12.0, 13.0, 14.0, 15.0];
params.safe_distance = 1.25;
params.influence_distance = 10.00;
params.repulsion_gain = 50.0;   % barrier strength; sets fence radius / pairwise spacing
params.repulsion_softening = 0.08;   % (unused with the barrier form)
params.sign_eps = 1e-2;
params.t_final = 30.0;
params.t_samples = 1501;
params.max_step = 0.01;
params.solver = 'ode45';
params.initial_layout = 'regular';
params.case_name = 'safe_case';
end

%--------------------------------------------------------------------------
function results = run_case(params, outdir)
if ~exist(outdir, 'dir')
    mkdir(outdir);
end
results = run_simulation(params);
save(fullfile(outdir, 'result.mat'), 'results', 'params');
print_summary(params, results);
end

%--------------------------------------------------------------------------
function print_summary(params, results)
[min_dist, idx] = min(results.pairwise_distances, [], 'all', 'linear');
[row, col] = ind2sub(size(results.pairwise_distances), idx);
fprintf('\nSimulation 3 %s data complete.\n', params.case_name);
fprintf('Final weighted-center error: %.6f\n', results.weighted_distance(end));
fprintf('Minimum pairwise distance: %.6f at %s, t = %.3f\n', ...
    min_dist, results.pairwise_labels{row}, results.t(col));
fprintf('Safety distance: %.6f\n', params.safe_distance);
fprintf('Final maximum x-error: %.6f\n', max(results.x_error(:, end)));
fprintf('Maximum x-error: %.6f\n', max(results.x_error, [], 'all'));
fprintf('Final maximum speed error: %.6f\n', max(results.speed_errors(:, end)));
fprintf('Maximum speed error: %.6f\n', max(results.speed_errors, [], 'all'));
end

%--------------------------------------------------------------------------
function results = run_simulation(params)
%RUN_SIMULATION  Closed-loop integration for the heterogeneous target-fencing demo.
%   Implements the redesigned controller of report/main.tex:
%       s_i = sum_{k=0}^{m_i-1} binom(m_i-1,k) x_i^{(k)},
%       x_i = kappa_i (p_i - p_0) - Phi_{1,i},
%   with a super-twisting outer loop and an APF filter on the repulsive field.

[layout, x0] = build_initial_state(params);
tgrid = linspace(0, params.t_final, params.t_samples);
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9, 'MaxStep', params.max_step);

switch params.solver
    case 'ode45'
        [t, x] = ode45(@(tv, xv) rhs(tv, xv, layout, params), tgrid, x0, opts);
    case 'ode15s'
        [t, x] = ode15s(@(tv, xv) rhs(tv, xv, layout, params), tgrid, x0, opts);
    case 'ode23t'
        [t, x] = ode23t(@(tv, xv) rhs(tv, xv, layout, params), tgrid, x0, opts);
    otherwise
        error('Unknown solver: %s', params.solver);
end

results = postprocess(t, x, params, layout);
results.t = t;
results.x = x;
end

%--------------------------------------------------------------------------
function dx = rhs(t, x, layout, params)
%Right-hand side of the full closed-loop state vector.
[agents, filters, w] = unpack_state(x, layout, params);
target = target_profile(t);
[u, s, phi] = control_at_state(agents, filters, w, target, params);

dx = zeros(size(x));
for i = 1:params.N
    m = params.orders(i);
    agent = agents{i};
    filter = filters{i};
    gamma_i = params.gamma;

    ui = u(:, i);
    si = s(:, i);
    wdot = -params.kb(i) * smooth_sign(si, params.sign_eps);

    agent_dot = zeros(size(agent));
    if m == 1
        agent_dot(:, 1) = ui;
    else
        agent_dot(:, 1:(m - 1)) = agent(:, 2:m);
        agent_dot(:, m) = ui;
    end

    filter_dot = zeros(size(filter));
    filter_dot(:, 1) = -gamma_i(1) * filter(:, 1) + filter(:, 2);
    filter_dot(:, 2) = -gamma_i(2) * filter(:, 2) + filter(:, 3);
    filter_dot(:, 3) = -gamma_i(3) * filter(:, 3) + filter(:, 4);
    filter_dot(:, 4) = -gamma_i(4) * filter(:, 4) + phi(:, i);

    dx(layout.agent{i}) = reshape(agent_dot, [], 1);
    dx(layout.filter{i}) = reshape(filter_dot, [], 1);
    dx(layout.w{i}) = wdot;
end
end

%--------------------------------------------------------------------------
function [u, s, phi] = control_at_state(agents, filters, w, target, params)
%Evaluate the redesigned control law for every agent.
u = zeros(params.dim, params.N);
s = zeros(params.dim, params.N);
positions = agent_positions(agents, params);
phi = repulsive_field(positions, params);

for i = 1:params.N
    m = params.orders(i);
    kappa_i = params.kappa(i);
    gamma_i = params.gamma;
    agent = agents{i};
    filter = filters{i};
    wi = w{i};
    phi_i = phi(:, i);

    phi1_derivs = filter_chain_derivatives(filter, phi_i, gamma_i);

    p_err = agent(:, 1) - target.derivs{1};
    xsum = kappa_i * p_err - filter(:, 1);
    ueq = zeros(params.dim, 1);

    % Redesigned sliding surface with Hurwitz (binomial) coefficients:
    %   s_i = sum_{k=0}^{m-1} C(m-1,k) x_i^{(k)}  =>  (D+1)^{m-1} x_i = 0.
    for r = 1:(m - 1)
        p_err_r = agent(:, r + 1) - target.derivs{r + 1};
        xsum = xsum + nchoosek(m - 1, r) * (kappa_i * p_err_r - phi1_derivs{r});
        ueq = ueq - nchoosek(m - 1, r - 1) * kappa_i * p_err_r;
    end
    for r = 1:m
        ueq = ueq + nchoosek(m - 1, r - 1) * phi1_derivs{r};
    end

    s(:, i) = xsum;
    usta = -params.ka(i) * signed_sqrt(s(:, i), params.sign_eps) + wi;
    u(:, i) = (usta + ueq) / kappa_i;
end
end

%--------------------------------------------------------------------------
function results = postprocess(t, x, params, layout)
nT = numel(t);
positions = zeros(params.dim, params.N, nT);
target_pos = zeros(params.dim, nT);
target_vel = zeros(params.dim, nT);
weighted_center = zeros(params.dim, nT);
weighted_distance = zeros(1, nT);
x_error = zeros(params.N, nT);
pairwise_labels = {};
pairwise_distances = [];
speed_errors = zeros(params.N, nT);
controls = zeros(params.dim, params.N, nT);

pair_count = 0;
for i = 1:params.N-1
    for j = i+1:params.N
        pair_count = pair_count + 1;
        pairwise_labels{pair_count} = sprintf('d_{%d,%d}', i, j);
        pairwise_distances(pair_count, :) = 0; %#ok<AGROW>
    end
end

for k = 1:nT
    [agents, filters, w] = unpack_state(x(k, :)', layout, params);
    target = target_profile(t(k));
    positions(:, :, k) = agent_positions(agents, params);
    target_pos(:, k) = target.derivs{1};
    target_vel(:, k) = target.derivs{2};

    [u, ~, ~] = control_at_state(agents, filters, w, target, params);
    controls(:, :, k) = u;

    weights = params.kappa(:) / sum(params.kappa);
    weighted_center(:, k) = positions(:, :, k) * weights;
    weighted_distance(k) = norm(weighted_center(:, k) - target_pos(:, k));

    for i = 1:params.N
        x_i = params.kappa(i) * (positions(:, i, k) - target_pos(:, k)) - filters{i}(:, 1);
        x_error(i, k) = norm(x_i);
    end

    pair_count = 0;
    for i = 1:params.N-1
        for j = i+1:params.N
            pair_count = pair_count + 1;
            pairwise_distances(pair_count, k) = norm(positions(:, i, k) - positions(:, j, k));
        end
    end

    for i = 1:params.N
        m = params.orders(i);
        if m == 1
            vi = controls(:, i, k);
        else
            vi = agents{i}(:, 2);
        end
        speed_errors(i, k) = norm(vi - target_vel(:, k));
    end
end

results = struct();
results.t = t;
results.positions = positions;
results.target_pos = target_pos;
results.target_vel = target_vel;
results.weighted_center = weighted_center;
results.weighted_distance = weighted_distance;
results.x_error = x_error;
results.pairwise_labels = {pairwise_labels{:}};
results.pairwise_distances = pairwise_distances;
results.speed_errors = speed_errors;
results.controls = controls;
if isfield(params, 'case_name')
    results.case_name = params.case_name;
end
end

%--------------------------------------------------------------------------
function [layout, x0] = build_initial_state(params)
layout = struct();
idx = 1;
for i = 1:params.N
    m = params.orders(i);
    layout.agent{i} = idx:(idx + 2*m - 1);
    idx = idx + 2*m;
end
for i = 1:params.N
    layout.filter{i} = idx:(idx + 7);
    idx = idx + 8;
end
for i = 1:params.N
    layout.w{i} = idx:(idx + 1);
    idx = idx + 2;
end

x0 = zeros(idx - 1, 1);
target0 = target_profile(0);
p0 = target0.derivs{1};
v0 = target0.derivs{2};
a0 = target0.derivs{3};
j0 = target0.derivs{4};

[rel_pos, rel_vel, rel_acc, rel_jerk] = initial_relative_conditions(params);
for i = 1:params.N
    m = params.orders(i);
    agent_state = zeros(params.dim, m);
    agent_state(:, 1) = p0 + rel_pos(:, i);
    if m >= 2
        agent_state(:, 2) = v0 + rel_vel(:, i);
    end
    if m >= 3
        agent_state(:, 3) = a0 + rel_acc(:, i);
    end
    if m >= 4
        agent_state(:, 4) = j0 + rel_jerk(:, i);
    end
    x0(layout.agent{i}) = reshape(agent_state, [], 1);
    x0(layout.filter{i}) = zeros(8, 1);
    x0(layout.w{i}) = zeros(2, 1);
end
end

%--------------------------------------------------------------------------
function [rel_pos, rel_vel, rel_acc, rel_jerk] = initial_relative_conditions(params)
if isfield(params, 'initial_layout') && strcmp(params.initial_layout, 'head_on')
    rel_pos = [-2.80, 2.80, -0.67, 0.67;
                2.30, 2.30, 0.00, 0.00];
    rel_vel = [ 0.00, 0.00, 1.40, -1.40;
                0.00, 0.00, 0.00,  0.00];
    rel_acc = [0.00, 0.00, 0.10, -0.10;
               0.00, 0.00, 0.00,  0.00];
    rel_jerk = [0.00, 0.00, 0.00, -0.02;
                0.00, 0.00, 0.00,  0.00];
    return;
end

offsets = [2.0, 2.0, 2.0, 2.0];
base = [1, 1; -1, 1; -1, -1; 1, -1]';
rel_pos = base .* offsets;
rel_vel = [0.10, -0.09, 0.08, -0.07;
          -0.08, 0.07, -0.06, 0.09];
rel_acc = [0.03, -0.025, 0.02, -0.015;
          -0.02, 0.02, -0.015, 0.025];
rel_jerk = [0.01, -0.008, 0.006, -0.005;
           -0.008, 0.006, -0.005, 0.008];
end

%--------------------------------------------------------------------------
function [agents, filters, w] = unpack_state(x, layout, params)
agents = cell(params.N, 1);
filters = cell(params.N, 1);
w = cell(params.N, 1);
for i = 1:params.N
    m = params.orders(i);
    agents{i} = reshape(x(layout.agent{i}), params.dim, m);
    filters{i} = reshape(x(layout.filter{i}), params.dim, 4);
    w{i} = x(layout.w{i});
end
end

%--------------------------------------------------------------------------
function P = agent_positions(agents, params)
P = zeros(params.dim, params.N);
for i = 1:params.N
    P(:, i) = agents{i}(:, 1);
end
end

%--------------------------------------------------------------------------
function phi = repulsive_field(P, params)
%Barrier-type pairwise repulsion implementing A1-A2 of report/main.tex:
%   alpha_r(r) = k_r * (1/(r-d) - 1/(mu-d))  for d < r < mu,  and 0 for r >= mu.
%The field grows as r -> d. The numerical floor prevents singular integration
%and is not, by itself, a formal collision-avoidance certificate.
phi = zeros(params.dim, params.N);
d = params.safe_distance;
mu = params.influence_distance;
k_r = params.repulsion_gain;
rho_min = 1e-3;        % numerical floor; barrier still diverges in practice
eps0 = 1e-12;
for i = 1:params.N
    for j = 1:params.N
        if i == j
            continue;
        end
        delta = P(:, i) - P(:, j);
        r = norm(delta);
        if r < mu
            rho = max(r - d, rho_min);
            mag = k_r * (1.0 / rho - 1.0 / (mu - d));
            phi(:, i) = phi(:, i) + mag * delta / max(r, eps0);
        end
    end
end
end

%--------------------------------------------------------------------------
function derivs = filter_chain_derivatives(filter, phi_i, gamma)
z1 = filter(:, 1);
z2 = filter(:, 2);
z3 = filter(:, 3);
z4 = filter(:, 4);

d4_1 = -gamma(4) * z4 + phi_i;
d3_1 = -gamma(3) * z3 + z4;
d2_1 = -gamma(2) * z2 + z3;
d1_1 = -gamma(1) * z1 + z2;

d2_2 = -gamma(2) * d2_1 + d3_1;
d3_2 = -gamma(3) * d3_1 + d4_1;

d2_3 = -gamma(2) * d2_2 + d3_2;

d1_2 = -gamma(1) * d1_1 + d2_1;
d1_3 = -gamma(1) * d1_2 + d2_2;
d1_4 = -gamma(1) * d1_3 + d2_3;

derivs = cell(4, 1);
derivs{1} = d1_1;
derivs{2} = d1_2;
derivs{3} = d1_3;
derivs{4} = d1_4;
end

%--------------------------------------------------------------------------
function y = smooth_sign(x, epsval)
y = x ./ sqrt(x.^2 + epsval^2);
end

%--------------------------------------------------------------------------
function y = signed_sqrt(x, epsval)
y = sqrt(abs(x) + epsval) .* smooth_sign(x, epsval);
end

%--------------------------------------------------------------------------
function target = target_profile(t)
pos = [4.5 + 0.06*t + 0.75*sin(0.08*t);
       1.5*cos(0.08*t) + 0.20*sin(0.16*t)];
vel = [0.06 + 0.06*cos(0.08*t);
       -0.12*sin(0.08*t) + 0.032*cos(0.16*t)];
acc = [-0.0048*sin(0.08*t);
       -0.0096*cos(0.08*t) - 0.00512*sin(0.16*t)];
jerk = [-0.000384*cos(0.08*t);
        0.000768*sin(0.08*t) - 0.0008192*cos(0.16*t)];
snap = [0.00003072*sin(0.08*t);
        0.00006144*cos(0.08*t) + 0.000131072*sin(0.16*t)];
target = struct();
target.derivs = {pos, vel, acc, jerk, snap};
end
