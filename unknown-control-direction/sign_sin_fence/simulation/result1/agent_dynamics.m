function [p_dot, v_dot, eta_dot, p0_dot, v0_dot] = ...
    agent_dynamics(t, p, v, eta, p0, v0, u, par)

N = par.N;
m = par.m;

p_dot = zeros(N, m);
v_dot = zeros(N, m);
eta_dot = zeros(N, m);

u0 = target_acceleration(t, par);
p0_dot = v0;
v0_dot = u0;

for i = 1:N
    Bdiag_i = input_gain_matrix(t, i, par);
    d_i = disturbance_agent(t, i, par);

    p_i0 = p(i,:) - p0;
    phi_i = repulsion_term(i, p, v, par);

    p_dot(i,:) = v(i,:);
    v_dot(i,:) = Bdiag_i .* u(i,:) + d_i;
    eta_dot(i,:) = par.k1 * p_i0 - phi_i;
end

end
