function [Cb, Cm] = ffsm_dou_bias(P, q, dq, vb, kin)
%FFSM_DOU_BIAS  Velocity-dependent (bias) forces of the floating-base chain.
%
%   [Cb, Cm] = FFSM_DOU_BIAS(P, q, dq, vb, kin) evaluates the Coriolis /
%   centrifugal generalized forces C(q, nu) nu of the full (6+n)-DOF system
%   at the generalized velocity nu = [vb; dq], with the base and joint
%   accelerations set to zero (recursive Newton-Euler algorithm).
%
%   Cb (6x1)  base wrench rate; it balances the base rows of the equation of
%             motion, H_b a_b + H_bm qdd + C_b = 0.
%   Cm (nx1)  joint-space bias force.
%
%   Spatial algebra: v_child = X v_parent + S qd,
%                    a_child = X a_parent + S qdd + v_child x (S qd),
%                    f = I a + v x* (I v),  f_parent += X' f_child.

n = P.n;

%% Forward pass: velocities and bias accelerations
v = cell(1, n);
a = cell(1, n);
v_prev = vb;
a_prev = zeros(6, 1);

for i = 1:n
    vi = kin.X{i} * v_prev + kin.S{i} * dq(i);
    ai = kin.X{i} * a_prev + crossm(vi, kin.S{i} * dq(i));
    v{i} = vi;
    a{i} = ai;
    v_prev = vi;
    a_prev = ai;
end

%% Backward pass: body forces and joint torques
f_child = zeros(6, 1);
Cm = zeros(n, 1);

for i = n:-1:1
    fi = kin.I{i+1} * a{i} + crossf(v{i}, kin.I{i+1} * v{i}) + f_child;
    Cm(i) = kin.S{i}.' * fi;
    f_child = kin.X{i}.' * fi;
end

% base body (a_0 = 0)
Cb = kin.I{1} * zeros(6, 1) + crossf(vb, kin.I{1} * vb) + f_child;

end

%% ------------------------------------------------------------------------
function y = crossm(v, w)
%CROSSM  Spatial motion cross product v x w.
    y = [cross(v(1:3), w(1:3));
         cross(v(1:3), w(4:6)) + cross(v(4:6), w(1:3))];
end

function y = crossf(v, f)
%CROSSF  Spatial force cross product v x* f.
    y = [cross(v(1:3), f(1:3)) + cross(v(4:6), f(4:6));
         cross(v(1:3), f(4:6))];
end
