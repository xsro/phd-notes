function y = sat_sign(x, phi)
% Smooth approximation of sign(x).

y = tanh(x / phi);

end
