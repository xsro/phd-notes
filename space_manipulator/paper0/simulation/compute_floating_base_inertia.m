function [H_full, H_b, H_bm, H_m] = compute_floating_base_inertia(P, q_joint)
% 使用 Jacobian 求和法计算自由漂浮空间机械臂的完整 (6+n)×(6+n) 惯性矩阵
%
% 方法: 对串联链 {基座, 连杆1, ..., 连杆n}，每体速度可写为
%   v_i = J_i * [x_b_dot; q_dot]
% 其中 J_i ∈ ℝ^(6×(6+n)) 为体 i 的空间雅可比。
% 完整惯性矩阵为 H = Σ_i J_i^T * I_i * J_i
%
% 基座 (体 0) 的雅可比: J_0 = [I_6, 0_(6×n)]
% 连杆 i 的雅可比:      J_i = [I_6, 0_(6×(i-1)), Ad_{Ti}^{-1}*S_1, ..., Ad_{Ti}^{-1}*S_i, 0]

    n = P.n;
    
    % ---- 1. 各体空间惯性 ----
    I_body = cell(1, n+1);      % 体 i 在自身坐标系中的 6×6 空间惯性
    I_body{1} = spatial_inertia(P.mass(1), [0;0;0], P.I{1});  % 基座
    for i = 1:n
        I_body{i+1} = spatial_inertia(P.mi(i), P.b(:, i+1), P.Ilink{i});
    end
    
    % ---- 2. 正向运动学：各体坐标系相对于基座坐标系的变换 ----
    T_base_to_body = cell(1, n+1);
    T_base_to_body{1} = eye(4);  % 基座到基座
    
    for i = 1:n
        % 关节 i 的旋转 (Z 轴) × 固定偏移 a(:, i+1)
        qi = q_joint(i);
        R_z = [cos(qi), -sin(qi), 0;
               sin(qi),  cos(qi), 0;
               0,        0,       1];
        p = P.a(:, i+1);
        T_joint = [R_z, p; 0 0 0 1];
        
        if i == 1
            T_base_to_body{i+1} = T_joint;
        else
            T_base_to_body{i+1} = T_base_to_body{i} * T_joint;
        end
    end
    
    % ---- 3. 关节轴在基座坐标系中的表示 ----
    % 关节 i 在体 i-1 坐标系中为 Z 轴，变换到基座坐标系
    S_base = cell(1, n);
    S_local = [0;0;1;0;0;0];  % 旋转关节，Z 轴
    for i = 1:n
        Ad_i = adjoint_transform(T_base_to_body{i});  % 体 i-1 → 基座
        S_base{i} = Ad_i * S_local;
    end
    
    % ---- 4. 构造各体雅可比并累加 H ----
    H_full = zeros(6+n, 6+n);
    
    for body = 0:n          % 体 0..n
        idx = body + 1;
        
        % 构造 J_body: 6×(6+n)
        J = zeros(6, 6+n);
        
        % 前 6 列: 基座速度贡献 (总是 I_6)
        J(:, 1:6) = eye(6);
        
        % 第 7..(6+n) 列: 关节速度贡献
        % 体 0 (基座): 无关节贡献
        if body >= 1
            for j = 1:body
                % 关节 j 对体 body 速度的贡献 = Ad_{T_body}^{-1} * S_j_base
                % 其中 S_j_base 是关节 j 的轴在基座坐标系中
                % 变换到体 body 坐标系: Ad_{T_body_to_base} 的逆 = Ad_{base_to_body}
                % v_body = Ad_{base_to_body} * v_base
                % v_base 中关节 j 的贡献为 S_base{j} * dq_j
                % 所以在体 body 中: Ad_{base_to_body} * S_base{j} * dq_j
                % 但我们需要在体 body 自身坐标系的表示，用于 I_body * v_body
                % 实际上雅可比应使得 v_body = J_body * q_dot
                % 在体 body 自身的 Plücker 坐标中
                
                % 从基座到体 body 的伴随
                Ad_base_to_body = adjoint_transform(T_base_to_body{idx});
                
                % 体 body 的局部速度中，关节 j 的贡献
                J(:, 6+j) = Ad_base_to_body * S_base{j};
            end
        end
        
        % 累加 H += J^T * I_body * J
        I_local = I_body{idx};
        H_full = H_full + J' * I_local * J;
    end
    
    % ---- 5. 提取子块 ----
    H_b = H_full(1:6, 1:6);
    H_bm = H_full(1:6, 7:end);
    H_m = H_full(7:end, 7:end);
end


function I_s = spatial_inertia(m, c, I_cm)
% 计算刚体的 6×6 空间惯性矩阵 (Plücker 基, [ω; v] 排列)
    cx = [0,    -c(3), c(2);
          c(3),  0,   -c(1);
         -c(2),  c(1), 0];
    I_s = zeros(6, 6);
    I_s(1:3, 1:3) = I_cm + m * cx * cx';
    I_s(1:3, 4:6) = m * cx;
    I_s(4:6, 1:3) = m * cx';
    I_s(4:6, 4:6) = m * eye(3);
end


function Ad = adjoint_transform(T)
% 齐次变换 T 对应的 6×6 伴随矩阵 (Plücker [ω; v] 坐标)
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    px = [0, -p(3), p(2);
          p(3), 0, -p(1);
         -p(2), p(1), 0];
    Ad = eye(6);
    Ad(1:3, 1:3) = R;
    Ad(4:6, 1:3) = px * R;
    Ad(4:6, 4:6) = R;
end