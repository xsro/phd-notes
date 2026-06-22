function y = sat_sign(x, phi)
% Smooth approximation of sign function using tanh.
%
% y = tanh(x/phi)
%
% phi > 0 controls the boundary layer width.

y = tanh(x / phi);

end