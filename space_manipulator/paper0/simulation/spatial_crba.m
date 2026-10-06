function [H, C, Me, Ce] = spatial_crba(P, q_joint, dq_joint)
% 空间 CRBA: 计算完整 (6+n)×(6+n) 惯性矩阵 + 科里奥利力 (数值)
    n = P.n;
    N = 6 + n;
    
    % ====== CRBA 核心: 只算 H (不递归) ======
    [H, Ic] = crba_h_only(P, q_joint);
    
    % ====== 科里奥利用数值 Christoffel ======
    if nargout >= 2
        eps = 1e-6;
        dq_full = [zeros(6,1); dq_joint];
        
        Hdot_dq = zeros(N, 1);
        for k = 1:n
            qp = q_joint; qp(k) = q_joint(k) + eps;
            qm = q_joint; qm(k) = q_joint(k) - eps;
            Hp = crba_h_only(P, qp);
            Hm = crba_h_only(P, qm);
            dH = (Hp - Hm) / (2*eps);
            Hdot_dq = Hdot_dq + (dH * dq_full) * dq_joint(k);
        end
        
        KE_grad = zeros(N, 1);
        for k = 1:n
            qp = q_joint; qp(k) = q_joint(k) + eps;
            qm = q_joint; qm(k) = q_joint(k) - eps;
            Hp = crba_h_only(P, qp);
            Hm = crba_h_only(P, qm);
            KEp = 0.5 * dq_full' * Hp * dq_full;
            KEm = 0.5 * dq_full' * Hm * dq_full;
            KE_grad(6+k) = (KEp - KEm) / (2*eps);
        end
        
        C = Hdot_dq - KE_grad;
    else
        C = [];
    end
    
    % ====== Schur 补 ======
    if nargout >= 3
        Hb = H(1:6,1:6); Hbm = H(1:6,7:end); Hm = H(7:end,7:end);
        Cb = C(1:6); Cm = C(7:end);
        Me = Hm - Hbm' * (Hb \ Hbm);
        Ce = Cm - Hbm' * (Hb \ Cb);
    else
        Me = []; Ce = [];
    end
end

function [H, Ic] = crba_h_only(P, q_joint)
% CRBA 核心: 只计算 H (无递归, 无数值微分)
    n = P.n;
    N = 6 + n;
    S = [0;0;1;0;0;0];
    
    % 正向运动学
    X = cell(1, n+2); X{1} = eye(4);
    T = cell(1, n+2); T{1} = eye(4);
    for i = 1:n
        qi = q_joint(i);
        X{i+1} = [rotz(qi), P.a(:,i+1); 0 0 0 1];
        T{i+1} = T{i} * X{i+1};
    end
    
    % 体惯性
    I = cell(1, n+1);
    for i = 1:n+1
        I{i} = sp_inertia(P.mass(i), P.b(:,i), P.I{i});
    end
    
    % 复合惯性: I_c{i} = 体 i-1 + 下游, 在体 i-1 系
    % inv(X{i+1}) = body i-1 → body i
    % adj(inv)' 变换力: body i → body i-1
    Ic = cell(1, n+1);
    Ic{n+1} = I{n+1};
    for i = n:-1:1
        R = X{i+1}(1:3,1:3); p = X{i+1}(1:3,4);
        Xi = [R', -R'*p; 0 0 0 1];
        Ic{i} = I{i} + adjoint(Xi)' * Ic{i+1} * adjoint(Xi);
    end
    
    % 提取 H
    H = zeros(N, N);
    H(1:6, 1:6) = Ic{1}(1:6, 1:6);
    
    for j = 1:n
        % S_j in body j frame
        Rj = X{j+1}(1:3,1:3); pj = X{j+1}(1:3,4);
        Xj_inv = [Rj', -Rj'*pj; 0 0 0 1];
        Sj_body = adjoint(Xj_inv) * S;
        
        % f_j = Ic{j+1} * Sj_body → f_0 = adj(inv(T_j+1))' * f_j
        fj = Ic{j+1} * Sj_body;
        R0 = T{j+1}(1:3,1:3); p0 = T{j+1}(1:3,4);
        Tj_inv = [R0', -R0'*p0; 0 0 0 1];
        f0 = adjoint(Tj_inv)' * fj;
        
        H(1:6, 6+j) = f0;
        H(6+j, 1:6) = f0';
        
        for i = 1:j
            Si_j = S;
            for k = i:j
                Rk = X{k+1}(1:3,1:3); pk = X{k+1}(1:3,4);
                Si_j = adjoint([Rk', -Rk'*pk; 0 0 0 1]) * Si_j;
            end
            H(6+i, 6+j) = Si_j' * Ic{j+1} * Sj_body;
            H(6+j, 6+i) = H(6+i, 6+j);
        end
    end
end

function A = adjoint(T)
    R = T(1:3,1:3); p = T(1:3,4);
    px = [0,-p(3),p(2); p(3),0,-p(1); -p(2),p(1),0];
    A = [R, zeros(3); px*R, R];
end

function I = sp_inertia(m, c, Icm)
    cx = [0,-c(3),c(2); c(3),0,-c(1); -c(2),c(1),0];
    I = [Icm + m*cx*cx', m*cx; m*cx', m*eye(3)];
end

function R = rotz(q)
    c = cos(q); s = sin(q);
    R = [c, -s, 0; s, c, 0; 0, 0, 1];
end