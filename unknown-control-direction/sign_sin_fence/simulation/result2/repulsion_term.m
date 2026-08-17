function phi_i = repulsion_term(i, p, par)
% Repulsive term:
%   phi_i = sum_j rho(||p_ij||) * p_ij / ||p_ij||
%
% rho(s) = max{ 1/(s-d) - 1/(mu-d), 0 }, s > d

N = par.N;
m = par.m;

phi_i = zeros(1,m);

for j = 1:N
    if j == i
        continue;
    end

    p_ij = p(i,:) - p(j,:);
    r = norm(p_ij);

    % Avoid numerical singularity if collision occurs in simulation
    if r < par.d_safe + 1e-6
        r = par.d_safe + 1e-6;
    end

    if r < par.mu
        rho = 1/(r - par.d_safe) - 1/(par.mu - par.d_safe);
        rho = max(rho, 0);
        phi_i = phi_i + rho * p_ij / r;
    end
end

end