function d_i = disturbance_agent(t, i, par)
% Bounded matched disturbance for agent i.

m = par.m;
d_i = zeros(1, m);
amp = par.d_bar(i);

if m >= 1
    d_i(1) = 0.75 * amp * sin(1.20 * t + 0.30 * i);
end

if m >= 2
    d_i(2) = 0.65 * amp * cos(1.05 * t + 0.50 * i);
end

for ell = 3:m
    d_i(ell) = 0.60 * amp * sin((0.80 + 0.20 * ell) * t + 0.25 * i);
end

end
