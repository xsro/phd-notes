function u0 = target_acceleration(t, par)
% Bounded target acceleration.
% ||u0||_inf <= par.u0_bar approximately.

m = par.m;
u0 = zeros(1,m);

amp = par.u0_bar;

if m >= 1
    u0(1) = 0.8 * amp * sin(0.6*t);
end

if m >= 2
    u0(2) = 0.8 * amp * cos(0.5*t);
end

for ell = 3:m
    u0(ell) = 0.5 * amp * sin(0.4*t + ell);
end

end