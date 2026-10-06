function C_dq = compute_coriolis_force(P, q_joint, dq_joint)
% 用数值 Christoffel 符号计算完整科里奥利/离心力 C(q,dq)*dq
%
% 公式: C(q,dq)*dq = [dH/dt] * dq - 1/2 * ∇(dq' * H * dq)
% 其中: H 为完整 (6+n)×(6+n) 惯性矩阵
%       [dH/dt] = Σ_{k=1..n} ∂H/∂q_k * dq_joint(k)
%       ∇(dq'*H*dq) 是动能对 q 的梯度
%
% 输出: C_dq ∈ ℝ^(6+n) 为完整科里奥利力向量

    n = P.n;
    N = 6 + n;
    epsilon = 1e-6;
    
    dq_full = [zeros(6,1); dq_joint];
    
    % H(q) 在标称点
    [H_q, ~, ~, ~] = compute_floating_base_inertia(P, q_joint);
    
    % === 计算 [dH/dt] * dq ===
    % [dH/dt] = Σ_{k=1..n} ∂H/∂q_k * dq_joint(k)
    % [dH/dt] * dq = Σ_{k=1..n} (∂H/∂q_k * dq) * dq_joint(k)
    Hdot_dq = zeros(N, 1);
    
    for k = 1:n
        q_plus = q_joint; q_plus(k) = q_joint(k) + epsilon;
        q_minus = q_joint; q_minus(k) = q_joint(k) - epsilon;
        
        [H_plus, ~, ~, ~] = compute_floating_base_inertia(P, q_plus);
        [H_minus, ~, ~, ~] = compute_floating_base_inertia(P, q_minus);
        
        dH_dqk = (H_plus - H_minus) / (2*epsilon);  % 13×13
        Hdot_dq = Hdot_dq + (dH_dqk * dq_full) * dq_joint(k);
    end
    
    % === 计算 1/2 * ∇(dq' * H * dq) ===
    KE = 0.5 * dq_full' * H_q * dq_full;
    grad_KE = zeros(N, 1);
    
    for k = 1:n
        q_plus = q_joint; q_plus(k) = q_joint(k) + epsilon;
        q_minus = q_joint; q_minus(k) = q_joint(k) - epsilon;
        
        [H_plus, ~, ~, ~] = compute_floating_base_inertia(P, q_plus);
        [H_minus, ~, ~, ~] = compute_floating_base_inertia(P, q_minus);
        
        KE_plus = 0.5 * dq_full' * H_plus * dq_full;
        KE_minus = 0.5 * dq_full' * H_minus * dq_full;
        
        grad_KE(6+k) = (KE_plus - KE_minus) / (2*epsilon);
    end
    
    % C*dq = Hdot*dq - grad_KE
    C_dq = Hdot_dq - grad_KE;
end