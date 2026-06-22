function Bdiag_i = input_gain_matrix(t, i, par)
% Unknown diagonal input gain. The controller only uses b_min.

m = par.m;
Bdiag_i = zeros(1, m);

for ell = 1:m
    mag = 1.00 + 0.15 * sin(0.35 * t + 0.40 * i + 0.25 * ell);
    mag = max(mag, par.b_min(i, ell));

    if t < 14
        sgn_b = (-1)^(i + ell);
    elseif t < 28
        sgn_b = (-1)^(i + ell + 1);
    else
        sgn_b = (-1)^ell;
    end

    Bdiag_i(ell) = sgn_b * mag;
end

end
