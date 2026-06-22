function [u, s, Delta, phi_rep, phi_dot] = ...
    controller_periodic_smc(t, p, v, eta, p0, v0, par)
% Periodic SMC for
%   s_i0 = v_i0 + k1 p_i0 - phi_i + k2 eta_i.

N = par.N;
m = par.m;

u       = zeros(N, m);
s       = zeros(N, m);
Delta   = zeros(N, m);
phi_rep = zeros(N, m);
phi_dot = zeros(N, m);

for i = 1:N
    p_i0 = p(i,:) - p0;
    v_i0 = v(i,:) - v0;

    [phi_i, phi_dot_i] = repulsion_term(i, p, v, par);

    g_i = par.k1 * p_i0 - phi_i;
    s_i = v_i0 + g_i + par.k2 * eta(i,:);

    Delta_i = -phi_dot_i + par.k1 * v_i0 + par.k2 * g_i;

    phi_rep(i,:) = phi_i;
    phi_dot(i,:) = phi_dot_i;
    s(i,:) = s_i;
    Delta(i,:) = Delta_i;

    for ell = 1:m
        eps_il = par.eps_s(i, ell);
        lam_il = par.lambda(i, ell);
        bmin_il = par.b_min(i, ell);

        R_il = (abs(Delta_i(ell)) + par.d_bar(i) + par.u0_bar + lam_il) / bmin_il;
        z = sin(pi / eps_il * s_i(ell));

        if par.use_smooth
            sig = sat_sign(z, par.phi);
        else
            sig = sign(z);
        end

        u(i, ell) = R_il * sig;
    end
end

end
