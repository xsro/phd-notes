function [phi_i, phi_dot_i] = repulsion_term(i, p, v, par)
% Dynamic-neighbor repulsion and its time derivative.
%
% N_i(t) = {j ~= i : ||p_i - p_j|| < mu}
% phi_i = sum_{j in N_i(t)} rho(r_ij) e_ij
% rho(r) = 1/(r-d) - 1/(mu-d), d < r < mu
%
% The derivative is computed on intervals where N_i(t) is fixed.

N = par.N;
m = par.m;

phi_i = zeros(1, m);
phi_dot_i = zeros(1, m);

for j = 1:N
    if j == i
        continue;
    end

    q = p(i,:) - p(j,:);
    qdot = v(i,:) - v(j,:);
    r = norm(q);

    if r >= par.mu
        continue;
    end

    r_safe = max(r, par.d_safe + par.r_eps);
    e = q / r_safe;
    r_dot = dot(e, qdot);

    rho = 1 / (r_safe - par.d_safe) - 1 / (par.mu - par.d_safe);
    rho = max(rho, 0);

    rho_dot = -r_dot / (r_safe - par.d_safe)^2;
    e_dot = (qdot - e * r_dot) / r_safe;

    phi_i = phi_i + rho * e;
    phi_dot_i = phi_dot_i + rho_dot * e + rho * e_dot;
end

end
