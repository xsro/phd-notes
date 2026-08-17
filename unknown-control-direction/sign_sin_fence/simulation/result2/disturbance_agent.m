function d_i = disturbance_agent(t, i, par)
% Bounded disturbance for agent i.
% ||d_i||_inf <= approximately par.d_bar(i)

m = par.m;
d_i = zeros(1,m);

amp = par.d_bar(i);

if m >= 1
    d_i(1) = 0.8 * amp * sin(1.7*t + 0.4*i);
end

if m >= 2
    d_i(2) = 0.7 * amp * cos(1.3*t + 0.6*i);
end

% For dimensions greater than 2
for ell = 3:m
    d_i(ell) = 0.6 * amp * sin((1.0+0.2*ell)*t + 0.3*i);
end

end