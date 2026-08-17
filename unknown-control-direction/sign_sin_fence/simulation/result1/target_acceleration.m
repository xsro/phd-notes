function u0 = target_acceleration(t, par)
% Bounded target acceleration.

m = par.m;
u0 = zeros(1, m);
amp = par.u0_bar;

if m >= 1
    u0(1) = 0.75 * amp * sin(0.45 * t);
end

if m >= 2
    u0(2) = 0.70 * amp * cos(0.40 * t);
end

for ell = 3:m
    u0(ell) = 0.50 * amp * sin(0.30 * t + ell);
end

end
