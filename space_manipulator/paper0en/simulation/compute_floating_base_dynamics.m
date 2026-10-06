function [H_full, C_full, M_e, C_e] = compute_floating_base_dynamics(P, q_joint, dq_joint)
% 计算自由漂浮空间机械臂的完整动力学及等效关节空间模型
%
% 输入:
%   P         - 参数结构体 (含 mass, b, a, I 等字段)
%   q_joint   - 关节角度 [n×1]
%   dq_joint  - 关节角速度 [n×1]
%
% 输出:
%   H_full    - 完整 (6+n)×(6+n) 惯性矩阵
%               [H_b  H_bm;  H_bm'  H_m]
%   C_full    - 完整科里奥利/离心项 (6+n)×1
%   M_e       - 等效关节空间惯性矩阵 (Schur 补) [n×n]
%   C_e       - 等效关节空间科里奥利项 [n×n] (使得 M_e(q)ddq + C_e(q,dq)dq = τ)
%
% 算法: 空间复合刚体算法 (Composite Rigid Body Algorithm, CRBA)
%       适用于浮基 (base) + 串联机械臂 (links 1..n)

    n = P.n;
    
    %========== 1. 构建空间运动学树 ==========
    % 体 0: 基座 (base)
    % 体 i (i=1..n): 连杆 i
    % 关节 i 连接体 i-1 和体 i
    
    % 各体在自身坐标系下的空间惯性
    I_spatial = cell(1, n+1);
    
    % 基座 (体 0)
    I_spatial{1} = spatial_inertia(P.mass(1), [0;0;0], P.I{1});
    
    % 连杆 (体 1..n)
    for i = 1:n
        I_spatial{i+1} = spatial_inertia(P.mi(i), P.b(:, i+1), P.Ilink{i});
    end
    
    %========== 2. 正向运动学：计算各体坐标系到惯性系的变换 ==========
    % 由于基座不受控，我们需要知道基座姿态和位置
    % 这里我们在体 0 坐标系下计算，基座质心即为坐标系原点
    
    % 齐次变换: T_i_to_0 从体 i 到体 0（基座）的变换
    T_to_base = cell(1, n+1);
    T_to_base{1} = eye(4);  % 基座到基座
    
    for i = 1:n
        % 从基座到连杆 i 的变换 = T_{i-1→0} * T_{i→i-1}
        % 其中 T_{i→i-1} 由关节 i 的旋转 + 固定偏移 (a_i) 组成
        
        % 关节 i 的旋转矩阵 (绕 Z 轴)
        qi = q_joint(i);
        R_joint = [cos(qi), -sin(qi), 0;
                   sin(qi),  cos(qi), 0;
                   0,        0,       1];
        
        % 固定偏移: 从关节 i 坐标系原点到体 i-1 坐标系原点的向量
        % 注意: a(:,i+1) 是 Yan 参数表中从 joint i 到 link i 的质心偏移
        % 但实际上 a_i 是从体 i-1 原点到体 i 原点的向量
        p_offset = P.a(:, i+1);
        
        T_i_to_im1 = [R_joint, p_offset; 0, 0, 0, 1];
        
        if i == 1
            T_to_base{i+1} = T_i_to_im1;
        else
            T_to_base{i+1} = T_to_base{i} * T_i_to_im1;
        end
    end
    
    %========== 3. 速度正向传递 ==========
    % 基座速度 (体 0): 未知，在浮基模式下由动量守恒确定
    % 但我们先计算各体的"关节诱发"速度，再处理基座
    % 对于 CRBA，我们需要各体在惯性系下的空间雅可比
    
    % 关节 i 的运动旋量轴 (在体 i-1 坐标系中)
    % 旋转关节: 绕 Z 轴
    S_body = [0; 0; 1; 0; 0; 0];  % 在关节坐标系中
    
    %========== 4. 空间 CRBA 反向递推 ==========
    % 计算复合刚体惯性
    
    % 第一步: 将各体空间惯性变换到基座坐标系
    I_base_frame = cell(1, n+1);
    for i = 0:n
        idx = i + 1;
        Ti = T_to_base{idx};
        Ad_i = adjoint_transform(Ti);
        I_base_frame{idx} = Ad_i' * I_spatial{idx} * Ad_i;
    end
    
    % 反向递推: 计算复合刚体惯性 (在基座坐标系中)
    I_composite = cell(1, n+1);
    % I_composite{i} = body_i + Σ children's I_composite
    
    % 首先直接复制各体惯性
    for i = 0:n
        I_composite{i+1} = I_base_frame{i+1};
    end
    
    % 从最后一个体开始累加
    % 体 i 的复合惯性 = 体 i 的惯性 + 体 i+1 的复合惯性 (经过变换)
    for i = n:-1:1
        % 体 i 包含体 i+1 (如果存在)
        if i < n
            I_composite{i+1} = I_composite{i+1} + I_composite{i+2};
        end
    end
    % 注意: 以上简化处理假设所有体在基座坐标系中直接相加
    % 对于串联链，这是正确的，因为所有体都已变换到基座坐标系
    
    %========== 5. 提取完整惯性矩阵 H_full = [H_b  H_bm;  H_bm'  H_m] ==========
    
    % 基座空间惯性 (6×6): 所有体的基座坐标系惯性之和
    H_b = zeros(6, 6);
    for i = 0:n
        H_b = H_b + I_base_frame{i+1};
    end
    
    % 机械臂惯性 (n×n): 标准固定基座惯性矩阵
    H_m = zeros(n, n);
    for i = 1:n
        for j = 1:n
            % H_m(i,j) = S_i' * I_composite{max(i,j)+1} * S_j
            % 其中 S_i 是关节 i 在基座坐标系中的运动旋量轴
            max_idx = max(i, j);
            % 使用体坐标系下的运动旋量，变换到基座坐标系
            S_i_base = compute_joint_axis_in_base(P, q_joint, i, T_to_base);
            S_j_base = compute_joint_axis_in_base(P, q_joint, j, T_to_base);
            H_m(i, j) = S_i_base' * I_composite{max_idx+1} * S_j_base;
        end
    end
    
    % 耦合惯性 (6×n): H_bm
    H_bm = zeros(6, n);
    for j = 1:n
        S_j_base = compute_joint_axis_in_base(P, q_joint, j, T_to_base);
        H_bm(:, j) = I_composite{j+1} * S_j_base;
    end
    
    % 组装完整惯性矩阵
    H_full = [H_b, H_bm; H_bm', H_m];
    
    %========== 6. 计算等效关节空间模型 ==========
    % Schur 补: M_e = H_m - H_bm' * H_b^{-1} * H_bm
    M_e = H_m - H_bm' * (H_b \ H_bm);
    
    %========== 7. 科里奥利/离心项 ==========
    % 使用 Christoffel 符号计算 C(q,dq)
    % 对 (6+n) 维完整系统
    
    epsilon = 1e-6;
    C_full = zeros(6+n, 1);
    
    % 数值计算科里奥利项: C(q,dq) = d(H)/dt * dq - 1/2 * d(dq'*H*dq)/dq
    % 使用简单数值微分
    
    % 对每个广义坐标计算 H 的偏导数
    q_full = [zeros(6,1); q_joint];  % 基座位置/姿态暂为 0
    
    for k = 1:n
        % 扰动关节 k
        q_plus = q_joint; q_plus(k) = q_joint(k) + epsilon;
        q_minus = q_joint; q_minus(k) = q_joint(k) - epsilon;
        
        % 扰动后的惯性矩阵
        [~, dq_joint_zero] = deal(zeros(n,1));
        [H_plus_full, ~, ~, ~] = compute_floating_base_inertia_fast(P, q_plus, T_to_base, I_spatial);
        [H_minus_full, ~, ~, ~] = compute_floating_base_inertia_fast(P, q_minus, T_to_base, I_spatial);
        
        dH_dqk = (H_plus_full - H_minus_full) / (2*epsilon);
        
        % 科里奥利项: C_k = Σ_j Σ_i (∂H_kj/∂q_i - 1/2*∂H_ij/∂q_k) * dq_i * dq_j
        dq_full = [zeros(6,1); dq_joint];
        
        for i = 1:n
            for j = 1:n
                C_full(6+k) = C_full(6+k) + ...
                    (dH_dqk(6+k, 6+j) - 0.5 * dH_dqk(6+i, 6+j)) * dq_full(6+i) * dq_full(6+j);
            end
        end
    end
    
    % 提取等效关节空间 C_e
    % 对于等效模型: τ = M_e(q)ddq + C_e(q,dq)
    % 其中 C_e 应包括: 从 H_m 的 Christoffel 符号 + H_bm'*H_b^{-1}*C_b 的贡献
    % 这里我们直接从完整 C 中提取
    
    C_b = C_full(1:6);          % 基座科里奥利 (对应基座加速度)
    C_m = C_full(7:end);        % 关节科里奥利
    
    % 等效 C_e = C_m - H_bm' * H_b^{-1} * C_b
    C_e_vec = C_m - H_bm' * (H_b \ C_b);
    
    % 返回 C_e 作为矩阵形式 (C_e * dq = C_e_vec)
    % 注意: 这里我们返回 C_e 矩阵的近似
    % 对于控制器设计, 我们主要需要 C_e_vec = C_e * dq
    % 因此我们返回 C_e_vec 作为列向量, 但在控制器中直接使用
    
    % 构造 C_e 矩阵 (n×n) 的近似
    % 使用数值方法
    C_e = zeros(n, n);
    for j = 1:n
        dq_pert = zeros(n, 1);
        dq_pert(j) = 1.0;
        [~, ~, ~, C_vec_j] = compute_floating_base_coriolis(P, q_joint, dq_pert);
        C_e(:, j) = C_vec_j;
    end
    
end

function S_base = compute_joint_axis_in_base(P, q_joint, joint_idx, T_to_base)
% 计算关节 joint_idx 在基座坐标系中的运动旋量轴
    
    % 关节 i 在连杆 i-1 坐标系中为 Z 轴旋转
    S_local = [0; 0; 1; 0; 0; 0];  % [ω; v]
    
    % 变换到基座坐标系
    T_i = T_to_base{joint_idx};
    Ad_i = adjoint_transform(T_i);
    
    S_base = Ad_i * S_local;
end

function [H_full, H_b, H_bm, H_m] = compute_floating_base_inertia_fast(P, q_joint, T_to_base_in, I_spatial)
% 快速计算完整惯性矩阵 (复用已计算的变换)

    n = P.n;
    
    % 计算正向运动学
    T_to_base = cell(1, n+1);
    T_to_base{1} = eye(4);
    
    for i = 1:n
        qi = q_joint(i);
        R_joint = [cos(qi), -sin(qi), 0;
                   sin(qi),  cos(qi), 0;
                   0,        0,       1];
        p_offset = P.a(:, i+1);
        T_i_to_im1 = [R_joint, p_offset; 0, 0, 0, 1];
        
        if i == 1
            T_to_base{i+1} = T_i_to_im1;
        else
            T_to_base{i+1} = T_to_base{i} * T_i_to_im1;
        end
    end
    
    % 将各体惯性变换到基座坐标系
    I_base_frame = cell(1, n+1);
    for i = 0:n
        idx = i+1;
        Ti = T_to_base{idx};
        Ad_i = adjoint_transform(Ti);
        I_base_frame{idx} = Ad_i' * I_spatial{idx} * Ad_i;
    end
    
    % 复合惯性
    I_composite = cell(1, n+1);
    for i = 0:n
        I_composite{i+1} = I_base_frame{i+1};
    end
    for i = n:-1:1
        if i < n
            I_composite{i+1} = I_composite{i+1} + I_composite{i+2};
        end
    end
    
    % H_b
    H_b = zeros(6, 6);
    for i = 0:n
        H_b = H_b + I_base_frame{i+1};
    end
    
    % H_m
    H_m = zeros(n, n);
    for i = 1:n
        for j = 1:n
            max_idx = max(i, j);
            S_i = compute_joint_axis_in_base(P, q_joint, i, T_to_base);
            S_j = compute_joint_axis_in_base(P, q_joint, j, T_to_base);
            H_m(i, j) = S_i' * I_composite{max_idx+1} * S_j;
        end
    end
    
    % H_bm
    H_bm = zeros(6, n);
    for j = 1:n
        S_j = compute_joint_axis_in_base(P, q_joint, j, T_to_base);
        H_bm(:, j) = I_composite{j+1} * S_j;
    end
    
    H_full = [H_b, H_bm; H_bm', H_m];
end

function [C_b, C_m] = compute_floating_base_coriolis(P, q_joint, dq_joint)
% 计算完整科里奥利项 C(q,dq) = [C_b(6×1); C_m(n×1)]

    n = P.n;
    dq_full = [zeros(6,1); dq_joint];
    N = 6 + n;
    
    % 空间惯性
    I_spatial = cell(1, n+1);
    I_spatial{1} = spatial_inertia(P.mass(1), [0;0;0], P.I{1});
    for i = 1:n
        I_spatial{i+1} = spatial_inertia(P.mi(i), P.b(:, i+1), P.Ilink{i});
    end
    
    % 用数值微分计算
    epsilon = 1e-6;
    C_full = zeros(N, 1);
    
    for k = 1:n
        q_plus = q_joint; q_plus(k) = q_joint(k) + epsilon;
        q_minus = q_joint; q_minus(k) = q_joint(k) - epsilon;
        
        [H_plus, ~, ~, ~] = compute_floating_base_inertia_fast(P, q_plus, [], I_spatial);
        [H_minus, ~, ~, ~] = compute_floating_base_inertia_fast(P, q_minus, [], I_spatial);
        
        dH_dqk = (H_plus - H_minus) / (2*epsilon);
        
        for i = 1:n
            for j = 1:n
                C_full(6+k) = C_full(6+k) + ...
                    (dH_dqk(6+k, 6+j) - 0.5 * dH_dqk(6+i, 6+j)) * dq_full(6+i) * dq_full(6+j);
            end
        end
    end
    
    C_b = C_full(1:6);
    C_m = C_full(7:end);
end