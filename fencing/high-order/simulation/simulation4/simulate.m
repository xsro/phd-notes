%SIMULATE  Generate data for the Result 4 command-filtered backstepping demo.
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

% Keep the original one-case output path for compatibility with earlier runs.
params = safe_params; %#ok<NASGU>
results = safe_results; %#ok<NASGU>
save(fullfile(out_root, 'result.mat'), 'results', 'params');

stress_params = default_params();
stress_params.case_name = 'stress_case';
stress_params.backstepping_gain = 1.6;
stress_params.filter_tau = 0.45;
stress_params.robust_gain = 0.0;
stress_params.repulsion_gain = 0.02;
stress_params.t_final = 100.0;
stress_params.t_samples = 5001;
stress_params.max_step = 0.002;
stress_params.initial_layout = 'head_on';
stress_results = run_case(stress_params, fullfile(out_root, 'stress_case'));

save(fullfile(out_root, 'comparison.mat'), ...
    'safe_results', 'safe_params', 'stress_results', 'stress_params');

%--------------------------------------------------------------------------
function params = default_params()
params = struct();
params.dim = 2;
params.orders = [3, 3, 3, 4, 4];
params.N = numel(params.orders);

params.kappa = [0.95, 1.00, 1.05, 1.10, 1.15];
params.backstepping_gain = 8.0;
params.filter_tau = 0.045;
params.robust_gain = 0.05;
params.sign_eps = 1e-3;

params.safe_distance = 1.00;
params.influence_distance = 9.00;
params.repulsion_gain = 8.00;
params.repulsion_floor = 1e-3;

params.t_final = 35.0;
params.t_samples = 1751;
params.max_step = 0.005;
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
fprintf('\nSimulation 4 %s data complete.\n', params.case_name);
fprintf('Final weighted-center error: %.6f\n', results.weighted_distance(end));
fprintf('Minimum pairwise distance: %.6f at %s, t = %.3f\n', ...
    min_dist, results.pairwise_labels{row}, results.t(col));
fprintf('Safety distance: %.6f\n', params.safe_distance);
fprintf('Target inside hull samples: %d/%d\n', ...
    sum(results.inside_hull), numel(results.inside_hull));
fprintf('Final maximum residual norm: %.6f\n', max(results.residual_norms(:, end)));
fprintf('Maximum residual norm: %.6f\n', max(results.residual_norms, [], 'all'));
fprintf('Ideal energy initial/final/max increase step: %.6f %.6f %.6e\n', ...
    results.ideal_energy(1), results.ideal_energy(end), max(diff(results.ideal_energy)));
end

%--------------------------------------------------------------------------
function results = run_simulation(params)
%RUN_SIMULATION  Integrate the Result 4 command-filtered backstepping closed loop.

[layout, x0] = build_initial_state(params);
tgrid = linspace(0, params.t_final, params.t_samples);
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9, 'MaxStep', params.max_step);

switch params.solver
    case 'ode45'
        [t, x] = ode45(@(tv, xv) rhs(tv, xv, layout, params), tgrid, x0, opts);
    case 'ode15s'
        [t, x] = ode15s(@(tv, xv) rhs(tv, xv, layout, params), tgrid, x0, opts);
    otherwise
        error('Unknown solver: %s', params.solver);
end

results = postprocess(t, x, params, layout);
results.t = t;
results.x = x;
end

%--------------------------------------------------------------------------
function dx = rhs(t, x, layout, params)
[agents, filters] = unpack_state(x, layout, params);
target = target_profile(t);
[u, data] = control_at_state(agents, filters, target, params);

dx = zeros(size(x));
for i = 1:params.N
    m = params.orders(i);
    agent = agents{i};

    agent_dot = zeros(size(agent));
    agent_dot(:, 1:(m - 1)) = agent(:, 2:m);
    agent_dot(:, m) = u(:, i);

    filter_dot = data.filter_dot{i};

    dx(layout.agent{i}) = reshape(agent_dot, [], 1);
    dx(layout.filter{i}) = reshape(filter_dot, [], 1);
end
end

%--------------------------------------------------------------------------
function [u, data] = control_at_state(agents, filters, target, params)
P = agent_positions(agents, params);
phi = repulsive_field(P, params);
u = zeros(params.dim, params.N);

data = struct();
data.phi = phi;
data.psi = zeros(params.dim, params.N);
data.residual = zeros(params.dim, params.N);
data.filter_dot = cell(params.N, 1);
data.errors = cell(params.N, 1);

for i = 1:params.N
    m = params.orders(i);
    z = relative_chain(agents{i}, target, m);
    chi = filters{i};

    psi = -params.kappa(i) * z(:, 1) + phi(:, i);
    [chi_dot, errors] = command_filter_derivatives(z, chi, psi, params);

    e_last = errors(:, m - 1);
    if m >= 3
        e_prev = errors(:, m - 2);
    else
        e_prev = zeros(params.dim, 1);
    end

    target_m = target.derivs{m + 1};
    u(:, i) = target_m ...
        + chi_dot(:, m - 1) ...
        - params.backstepping_gain * e_last ...
        - e_prev ...
        - params.robust_gain * smooth_sign(e_last, params.sign_eps);

    data.psi(:, i) = psi;
    data.residual(:, i) = z(:, 2) - psi;
    data.filter_dot{i} = chi_dot;
    data.errors{i} = errors;
end
end

%--------------------------------------------------------------------------
function [chi_dot, errors] = command_filter_derivatives(z, chi, psi, params)
%Sequentially evaluate the command filters in Result 4.
m_minus_one = size(chi, 2);
chi_dot = zeros(size(chi));
errors = zeros(size(chi));
raw = zeros(size(chi));
raw(:, 1) = psi;
e_prev = zeros(params.dim, 1);

for r = 1:m_minus_one
    errors(:, r) = z(:, r + 1) - chi(:, r);
    chi_dot(:, r) = (-chi(:, r) + raw(:, r)) / params.filter_tau;

    if r < m_minus_one
        raw(:, r + 1) = chi_dot(:, r) ...
            - params.backstepping_gain * errors(:, r) ...
            - e_prev;
    end

    e_prev = errors(:, r);
end
end

%--------------------------------------------------------------------------
function results = postprocess(t, x, params, layout)
nT = numel(t);
positions = zeros(params.dim, params.N, nT);
target_pos = zeros(params.dim, nT);
weighted_center = zeros(params.dim, nT);
weighted_distance = zeros(1, nT);
pairwise_labels = {};
pairwise_distances = [];
residual_norms = zeros(params.N, nT);
first_error_norms = zeros(params.N, nT);
ideal_energy = zeros(1, nT);
controls = zeros(params.dim, params.N, nT);
inside_hull = false(1, nT);
hull_area = zeros(1, nT);

pair_count = 0;
for i = 1:params.N-1
    for j = i+1:params.N
        pair_count = pair_count + 1;
        pairwise_labels{pair_count} = sprintf('d_{%d,%d}', i, j); %#ok<AGROW>
        pairwise_distances(pair_count, :) = 0; %#ok<AGROW>
    end
end

for k = 1:nT
    [agents, filters] = unpack_state(x(k, :)', layout, params);
    target = target_profile(t(k));
    [u, data] = control_at_state(agents, filters, target, params);
    P = agent_positions(agents, params);

    positions(:, :, k) = P;
    target_pos(:, k) = target.derivs{1};
    controls(:, :, k) = u;

    weights = params.kappa(:) / sum(params.kappa);
    weighted_center(:, k) = P * weights;
    weighted_distance(k) = norm(weighted_center(:, k) - target_pos(:, k));

    for i = 1:params.N
        z = relative_chain(agents{i}, target, params.orders(i));
        residual_norms(i, k) = norm(data.residual(:, i));
        first_error_norms(i, k) = norm(data.errors{i}(:, 1));
        ideal_energy(k) = ideal_energy(k) ...
            + 0.5 * params.kappa(i) * norm(z(:, 1))^2;
    end
    ideal_energy(k) = ideal_energy(k) + potential_energy(P, params);

    pair_count = 0;
    for i = 1:params.N-1
        for j = i+1:params.N
            pair_count = pair_count + 1;
            pairwise_distances(pair_count, k) = norm(P(:, i) - P(:, j));
        end
    end

    [inside_hull(k), hull_area(k)] = target_in_convex_hull(P, target.derivs{1});
end

results = struct();
results.positions = positions;
results.target_pos = target_pos;
results.weighted_center = weighted_center;
results.weighted_distance = weighted_distance;
results.pairwise_labels = {pairwise_labels{:}};
results.pairwise_distances = pairwise_distances;
results.residual_norms = residual_norms;
results.first_error_norms = first_error_norms;
results.ideal_energy = ideal_energy;
results.controls = controls;
results.inside_hull = inside_hull;
results.hull_area = hull_area;
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
    layout.agent{i} = idx:(idx + params.dim * m - 1);
    idx = idx + params.dim * m;
end
for i = 1:params.N
    m = params.orders(i);
    layout.filter{i} = idx:(idx + params.dim * (m - 1) - 1);
    idx = idx + params.dim * (m - 1);
end

x0 = zeros(idx - 1, 1);
target0 = target_profile(0);
[rel_pos, rel_vel, rel_acc, rel_jerk] = initial_relative_conditions(params);
P0 = target0.derivs{1} + rel_pos;

for i = 1:params.N
    m = params.orders(i);
    agent_state = zeros(params.dim, m);
    agent_state(:, 1) = P0(:, i);
    agent_state(:, 2) = target0.derivs{2} + rel_vel(:, i);
    agent_state(:, 3) = target0.derivs{3} + rel_acc(:, i);
    if m >= 4
        agent_state(:, 4) = target0.derivs{4} + rel_jerk(:, i);
    end

    x0(layout.agent{i}) = reshape(agent_state, [], 1);

    z = relative_chain(agent_state, target0, m);
    phi0 = repulsive_field(P0, params);
    psi0 = -params.kappa(i) * z(:, 1) + phi0(:, i);

    chi0 = zeros(params.dim, m - 1);
    chi0(:, 1) = psi0;
    for r = 2:m - 1
        chi0(:, r) = z(:, r + 1);
    end
    x0(layout.filter{i}) = reshape(chi0, [], 1);
end
end

%--------------------------------------------------------------------------
function [rel_pos, rel_vel, rel_acc, rel_jerk] = initial_relative_conditions(params)
if isfield(params, 'initial_layout') && strcmp(params.initial_layout, 'head_on')
    rel_pos = [-0.60, 0.60, 0.00, -2.70, 2.70;
                0.00, 0.00, 2.80, -2.20, -2.20];
    rel_vel = [ 1.25, -1.25, 0.00, 0.20, -0.20;
                0.00,  0.00, -0.10, 0.10, 0.10];
    rel_acc = zeros(params.dim, params.N);
    rel_jerk = zeros(params.dim, params.N);
    return;
end

angles = linspace(0, 2*pi, params.N + 1);
angles(end) = [];
radii = [3.2, 2.9, 3.1, 3.3, 3.0];
rel_pos = zeros(params.dim, params.N);
for i = 1:params.N
    rel_pos(:, i) = radii(i) * [cos(angles(i)); sin(angles(i))];
end

rel_vel = [0.12, -0.09, 0.06, -0.08, 0.07;
          -0.08, 0.10, 0.08, -0.07, -0.10];
rel_acc = [0.03, -0.02, 0.025, -0.018, 0.02;
          -0.02, 0.025, 0.015, -0.02, -0.018];
rel_jerk = [0.006, -0.005, 0.004, -0.006, 0.005;
           -0.004, 0.006, 0.005, -0.004, -0.006];
end

%--------------------------------------------------------------------------
function [agents, filters] = unpack_state(x, layout, params)
agents = cell(params.N, 1);
filters = cell(params.N, 1);
for i = 1:params.N
    m = params.orders(i);
    agents{i} = reshape(x(layout.agent{i}), params.dim, m);
    filters{i} = reshape(x(layout.filter{i}), params.dim, m - 1);
end
end

%--------------------------------------------------------------------------
function z = relative_chain(agent, target, m)
z = zeros(size(agent));
for r = 1:m
    z(:, r) = agent(:, r) - target.derivs{r};
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
phi = zeros(params.dim, params.N);
d = params.safe_distance;
mu = params.influence_distance;
k_r = params.repulsion_gain;
rho_min = params.repulsion_floor;
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
function U = potential_energy(P, params)
U = 0;
d = params.safe_distance;
mu = params.influence_distance;
k_r = params.repulsion_gain;
rho_min = params.repulsion_floor;

for i = 1:params.N-1
    for j = i+1:params.N
        r = norm(P(:, i) - P(:, j));
        if r < mu
            rho = max(r - d, rho_min);
            U = U + k_r * (log((mu - d) / rho) - (mu - r) / (mu - d));
        end
    end
end
end

%--------------------------------------------------------------------------
function [inside, area] = target_in_convex_hull(P, target_pos)
try
    K = convhull(P(1, :), P(2, :));
    area = polyarea(P(1, K), P(2, K));
    inside = inpolygon(target_pos(1), target_pos(2), P(1, K), P(2, K));
catch
    inside = false;
    area = 0;
end
end

%--------------------------------------------------------------------------
function y = smooth_sign(x, epsval)
y = x ./ sqrt(x.^2 + epsval^2);
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
