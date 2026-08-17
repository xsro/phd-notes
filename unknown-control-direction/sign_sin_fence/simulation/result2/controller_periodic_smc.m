function [u, s, Delta, phi_rep] = controller_periodic_smc(t, p, v, eta, p0, v0, par)
% Periodic sliding mode controller for unknown sign-switching input gains.

N = par.N;
m = par.m;

u       = zeros(N,m);
s       = zeros(N,m);
Delta   = zeros(N,m);
phi_rep = zeros(N,m);

for i = 1:N

    p_i0 = p(i,:) - p0;
    v_i0 = v(i,:) - v0;

    %% Repulsion term
    phi_i = repulsion_term(i, p, par);
    phi_rep(i,:) = phi_i;

    %% Sliding variable
    s_i = eta(i,:) + par.k2 * p_i0 + v_i0;
    s(i,:) = s_i;

    %% Delta_i = k1 p_i0 - phi_i + k2 v_i0
    Delta_i = par.k1 * p_i0 - phi_i + par.k2 * v_i0;
    Delta(i,:) = Delta_i;

    %% Periodic SMC control component-wise
    for ell = 1:m

        eps_il = par.eps_s(i,ell);
        lam_il = par.lambda(i,ell);
        bmin_il = par.b_min(i,ell);

        R_il = (abs(Delta_i(ell)) + par.d_bar(i) + par.u0_bar + lam_il) / bmin_il;

        z = sin(pi/eps_il * s_i(ell));

        if par.use_smooth
            sig = sat_sign(z, par.phi);
        else
            sig = sign(z);
        end

        u(i,ell) = R_il * sig;
    end
end

end