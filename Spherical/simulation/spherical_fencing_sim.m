function [t_hist, P_hist, P0_hist, dist_hist, centroid_err] = ...
    spherical_fencing_sim(dim, n, r, ka, kr, ko, T, dt, ...
                          P_init, P0_init, P0_vel, adj, w)
% SPHERICAL_FENCING_SIM  Simulate the spherical fencing control algorithm.
%
%   [t, P, P0, dist, centroid] = spherical_fencing_sim(...)
%
%   Inputs:
%     dim     - spatial dimension (2 or 3)
%     n       - number of agents
%     r       - desired fencing radius
%     ka      - attraction gain
%     kr      - repulsion gain
%     ko      - radial tracking gain
%     T       - total simulation time
%     dt      - time step
%     P_init  - n x dim, initial agent positions
%     P0_init - 1 x dim, initial target position
%     P0_vel  - 1 x dim, target velocity (zero if empty)
%     adj     - n x n, adjacency matrix (symmetric, 0/1)
%     w       - n x n, weight matrix
%
%   Outputs:
%     t_hist       - 1 x Nsteps, time stamps
%     P_hist       - Nsteps x n x dim, agent trajectories
%     P0_hist      - Nsteps x dim, target trajectory
%     dist_hist    - Nsteps x n, distance from each agent to target
%     centroid_err - Nsteps x 1, centroid-to-target error

    % ---- Defaults ----
    if isempty(P0_vel), P0_vel = zeros(1, dim); end

    % ---- Pre-compute neighbor lists ----
    neighbors = cell(n, 1);
    for i = 1:n
        neighbors{i} = find(adj(i, :) & (1:n) ~= i);
    end

    % ---- Regularization to avoid division by zero ----
    eps_reg = 1e-8;

    % ---- Initialize storage ----
    Nsteps = round(T / dt);
    t_hist       = zeros(1, Nsteps);
    P_hist       = zeros(Nsteps, n, dim);
    P0_hist      = zeros(Nsteps, dim);
    dist_hist    = zeros(Nsteps, n);
    centroid_err = zeros(Nsteps, 1);

    P  = P_init;
    P0 = P0_init;

    % ---- Euler integration loop ----
    for k = 1:Nsteps
        t = (k - 1) * dt;

        % Record state
        t_hist(k) = t;
        P_hist(k, :, :)  = P;
        P0_hist(k, :)    = P0;

        % Compute metrics
        for i = 1:n
            dist_hist(k, i) = norm(P(i,:) - P0);
        end
        centroid = mean(P, 1);
        centroid_err(k) = norm(centroid - P0);

        % Compute control inputs
        U = zeros(n, dim);
        for i = 1:n
            diff_i = P(i,:) - P0;
            norm_i = norm(diff_i);
            x_i = r * diff_i / norm_i;  % radial projection

            % Term 1: radial tracking
            u_radial = -ko * (norm_i - r) * diff_i / norm_i;

            % Term 2: inter-agent interaction
            u_interact = zeros(1, dim);
            for jj = 1:length(neighbors{i})
                j = neighbors{i}(jj);
                diff_j = P(j,:) - P0;
                x_j = r * diff_j / norm(diff_j);

                dx = x_i - x_j;
                dx_sq = dot(dx, dx) + eps_reg;
                d_ij = x_j - (dot(x_i, x_j) / r^2) * x_i;
                f_ij = (ka - kr / dx_sq) * d_ij;
                u_interact = u_interact + w(i,j) * f_ij;
            end

            % Term 3: feedforward (target velocity)
            U(i,:) = u_radial + u_interact + P0_vel;
        end

        % Update positions (Euler step)
        P = P + U * dt;

        % Update target position
        P0 = P0 + P0_vel * dt;
    end

    % ---- Final snapshot ----
    t_hist(Nsteps+1)   = Nsteps * dt;
    P_hist(Nsteps+1,:,:) = P;
    P0_hist(Nsteps+1,:)  = P0;
    for i = 1:n
        dist_hist(Nsteps+1, i) = norm(P(i,:) - P0);
    end
    centroid_err(Nsteps+1) = norm(mean(P,1) - P0);
    t_hist = t_hist(1:Nsteps+1);
    P_hist = P_hist(1:Nsteps+1,:,:);
    P0_hist = P0_hist(1:Nsteps+1,:);
    dist_hist = dist_hist(1:Nsteps+1,:);
    centroid_err = centroid_err(1:Nsteps+1);

    % ---- Warn if agents converged too close ----
    for i = 1:n
        for jj = (i+1):n
            if norm(P(i,:) - P(jj,:)) < 1e-3
                warning('Agents %d and %d converged to nearly the same position.', i, jj);
            end
        end
    end
end
