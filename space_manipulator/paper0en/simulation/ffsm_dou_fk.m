function [pe, pj, Re] = ffsm_dou_fk(P, q)
%FFSM_DOU_FK  Forward kinematics of the 7-DOF FFSM in the base frame.
%
%   [pe, pj, Re] = FFSM_DOU_FK(P, q) returns the end-effector position pe
%   (origin of link frame 7), the joint origins pj (3 x (n+1), first column
%   is the base CM) and the end-effector rotation matrix Re, all expressed
%   in the base frame attached to the spacecraft centre of mass.

[~, kin] = ffsm_dou_crba(P, q);
pj = [kin.pj{:}];

T = eye(4);
for i = 1:P.n
    T = T * kin.T{i};
end

pe = T(1:3, 4);
Re = T(1:3, 1:3);

end
