function Bdiag_i = input_gain_matrix(t, i, par)
% Unknown sign-switching diagonal input gain.
% The controller does NOT use this sign information.
%
% B_i(t) = diag(b_i1(t), b_i2(t))
%
% Each component switches sign at different times.

m = par.m;
Bdiag_i = zeros(1,m);

for ell = 1:m

    % Magnitude is time-varying but always above b_min.
    mag = 1.0 + 0.2*sin(0.4*t + 0.7*i + 0.3*ell);

    % Make sure magnitude is not smaller than b_min.
    mag = max(mag, par.b_min(i,ell));

    % Sign switching pattern.
    % Different agents/components switch at different times.
    switch_signal = sin(0.35*t + 0.9*i + 0.5*ell);

    if switch_signal >= 0
        sgn_b = 1;
    else
        sgn_b = -1;
    end

    Bdiag_i(ell) = sgn_b * mag;
end

end