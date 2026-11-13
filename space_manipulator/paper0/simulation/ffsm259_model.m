function out = ffsm259_model(P, q, dq)
% FFSM259_MODEL  7-DOF free-floating space manipulator with DH kinematics.
%
% 惯性矩阵采用"质心雅可比求和"法构造 (等价于 CRBA), 运动学使用
% Table 1 的标准 DH 参数 (带 alpha 扭转), 而非全绕 z 的简化串联链:
%
%   w_i   = w_b + sum_{k<=i} qdot_k * z_{k-1}
%   v_ci  = v_b + w_b x p_ci + sum_{k<=i} qdot_k * z_{k-1} x (p_ci - p_{k-1})
%   H     = sum_i [ m_i J_vi' J_vi + J_wi' (R_i I_i R_i') J_wi ]
%
% 广义坐标 x = [x_b(6); q(n)], 状态向量顺序与原文 Eq (6) 一致。
%
% 输入
%   P   参数结构体 (dou_yue_259_parameters)
%   q   [n x 1] 关节角
%   dq  [n x 1] 关节角速度 (可选; 给出则计算 C_theta_m*dq 与广义雅可比)
%
% 输出 (struct out)
%   H        [6+n x 6+n] 完整惯性矩阵
%   Hb,Hbm,Hm            分块
%   Me       [n x n] 等效关节空间惯性 (Schur 补, 原文 Eq (8))
%   Ce_qd    [n x 1] 等效科里奥利项 C_theta_m*dq
%   pe       [3 x 1] 末端位置 (基座标系)
%   J        广义雅可比 [3 x n] (原文 Eq (4), 仅位置行)
%   T        1 x (n+1) 元胞, 各体系相对基座系的齐次变换
%   pos      3 x (n+1) 各体系原点位置 (基座系)
%   R        1 x (n+1) 元胞, 各体系姿态

    n = P.n;
    N = 6 + n;

    % ================= 1. 运动学: DH 正解 =================
    T   = cell(1, n+1);
    R   = cell(1, n+1);
    pos = zeros(3, n+1);

    T{1} = eye(4);   % 基座系原点 = 基座质心
    R{1} = eye(3);
    for i = 1:n
        th = q(i) + P.dh.theta_off(i);
        Rn = rotz(th) * rotx(P.dh.alpha_seg(i));
        pn = rotz(th) * [P.dh.l_seg(i); 0; P.dh.d_seg(i)];
        if i == 1
            pn = pn + [0; 0; P.dh.base_offset];   % 基座质心 -> 关节 1 (沿 z_0)
        end
        T{i+1} = T{i} * [Rn, pn; 0 0 0 1];
        R{i+1} = T{i+1}(1:3,1:3);
        pos(:,i+1) = T{i+1}(1:3,4);
    end

    % 关节 i 的转轴 = z_{i-1}
    zax = zeros(3, n);
    for i = 1:n
        zax(:,i) = R{i}(:,3);
    end

    % ================= 2. 质心位置 =================
    pc = zeros(3, n+1);
    pc(:,1) = [0;0;0];                      % 基座质心即基座系原点
    for i = 1:n
        pc(:,i+1) = pos(:,i+1) + R{i+1} * P.a(:,i+1);
    end

    % ================= 3. H = sum J'*I*J =================
    H = zeros(N, N);
    for i = 0:n
        idx = i + 1;
        Jw = zeros(3, N);
        Jv = zeros(3, N);
        Jw(:,4:6) = eye(3);                        % 基座角速度
        Jv(:,1:3) = eye(3);                        % 基座线速度
        Jv(:,4:6) = -skew(pc(:,idx));              % w_b x p_c
        for k = 1:i
            Jw(:,6+k) = zax(:,k);
            Jv(:,6+k) = cross(zax(:,k), pc(:,idx) - pos(:,k));
        end
        H = H + P.mass(idx) * (Jv.' * Jv) + Jw.' * (R{idx} * P.I{idx} * R{idx}.') * Jw;
    end
    H = 0.5*(H + H.');

    Hb  = H(1:6, 1:6);
    Hbm = H(1:6, 7:end);
    Hm  = H(7:end, 7:end);

    % Schur 补 (原文 Eq (8))
    Hb_inv = Hb \ eye(6);
    Me = Hm - Hbm.' * Hb_inv * Hbm;
    Me = 0.5*(Me + Me.');

    out.H   = H;
    out.Hb  = Hb;
    out.Hbm = Hbm;
    out.Hm  = Hm;
    out.Me  = Me;
    out.T   = T;
    out.R   = R;
    out.pos = pos;
    out.pc  = pc;
    out.zax = zax;

    % ================= 4. 末端位姿与广义雅可比 =================
    pe = pos(:, n+1);
    Jbe = [eye(3), -skew(pe)];            % v_e = v_b + w_b x p_e
    Jme = zeros(3, n);
    for k = 1:n
        Jme(:,k) = cross(zax(:,k), pe - pos(:,k));
    end
    out.pe = pe;
    out.J  = Jme - Jbe * Hb_inv * Hbm;    % 原文 Eq (4)
    out.Jme = Jme;
    out.Jbe = Jbe;

    % ================= 5. 等效科里奥利矩阵 (可选) =================
    % 零动量流形上的动能 T = 0.5*dq'*Me(q)*dq, 约化动力学仍是拉格朗日
    % 系统, 因此 C_theta_m 直接由 Me 的 Christoffel 符号给出:
    %   C_ij = 0.5 * sum_k ( dM_ij,k + dM_ik,j - dM_jk,i ) * dq_k
    % dM 用中心差分逐列求取, 与全模型共享同一 Me 求值函数。
    if nargin >= 3 && ~isempty(dq)
        [Cmat, Ce_qd] = reduced_coriolis(P, q, dq);
        out.Cmat  = Cmat;
        out.Ce_qd = Ce_qd;
    else
        out.Cmat  = [];
        out.Ce_qd = [];
    end
end

% ------------------------------------------------------------------------
function [C, c] = reduced_coriolis(P, q, dq)
    n = P.n;
    h = 1e-6;
    dM = zeros(n, n, n);
    for k = 1:n
        qp = q; qp(k) = qp(k) + h;
        qm = q; qm(k) = qm(k) - h;
        dM(:,:,k) = (schur_me(P, qp) - schur_me(P, qm)) / (2*h);
    end
    Mdot = zeros(n, n);
    for k = 1:n
        Mdot = Mdot + dM(:,:,k) * dq(k);
    end
    v = zeros(n, n);                 % v(:,j) = dM(:,:,j)*dq
    for j = 1:n
        v(:,j) = dM(:,:,j) * dq;
    end
    C = 0.5*(Mdot + v - v.');        % C(i,j) = 0.5*(Mdot_ij + v_j(i) - v_i(j))
    c = C*dq;
end

% ------------------------------------------------------------------------
function Me = schur_me(P, q)
% 只返回 Schur 补, 用于 Christoffel 差分的快速求值。
    n = P.n;
    N = 6 + n;
    R   = cell(1, n+1);  R{1} = eye(3);
    pos = zeros(3, n+1);
    for i = 1:n
        th = q(i) + P.dh.theta_off(i);
        Rn = rotz(th) * rotx(P.dh.alpha_seg(i));
        pn = rotz(th) * [P.dh.l_seg(i); 0; P.dh.d_seg(i)];
        if i == 1
            pn = pn + [0; 0; P.dh.base_offset];
        end
        R{i+1} = R{i} * Rn;
        pos(:,i+1) = pos(:,i) + R{i} * pn;
    end
    zax = zeros(3, n);
    for i = 1:n
        zax(:,i) = R{i}(:,3);
    end
    pc = zeros(3, n+1);
    for i = 1:n
        pc(:,i+1) = pos(:,i+1) + R{i+1} * P.a(:,i+1);
    end
    H = zeros(N, N);
    for i = 0:n
        idx = i + 1;
        Jw = zeros(3, N);  Jv = zeros(3, N);
        Jw(:,4:6) = eye(3);
        Jv(:,1:3) = eye(3);
        Jv(:,4:6) = -skew(pc(:,idx));
        for k = 1:i
            Jw(:,6+k) = zax(:,k);
            Jv(:,6+k) = cross(zax(:,k), pc(:,idx) - pos(:,k));
        end
        H = H + P.mass(idx) * (Jv.'*Jv) + Jw.'*(R{idx}*P.I{idx}*R{idx}.' )*Jw;
    end
    Hb  = H(1:6,1:6);  Hbm = H(1:6,7:end);  Hm = H(7:end,7:end);
    Me = Hm - Hbm.' * (Hb \ Hbm);
    Me = 0.5*(Me + Me.');
end

% ------------------------------------------------------------------------
function A = skew(p)
    A = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
end

function R = rotz(t)
    R = [cos(t), -sin(t), 0; sin(t), cos(t), 0; 0, 0, 1];
end

function R = rotx(t)
    R = [1, 0, 0; 0, cos(t), -sin(t); 0, sin(t), cos(t)];
end
