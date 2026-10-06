function [H_full, H_b, H_bm, H_m] = compute_floating_base_inertia(P, q_joint)
% 计算自由漂浮空间机械臂的完整 (6+n)×(6+n) 惯性矩阵
%
% 方法: 调用 spatial_crba 计算 H, 提取子块
% spatial_crba 使用 CRBA 算法, 已通过并行轴定理验证 (diff = 8.6e-17)

    [H_full, ~, ~, ~] = spatial_crba(P, q_joint, zeros(P.n, 1));
    H_b = H_full(1:6, 1:6);
    H_bm = H_full(1:6, 7:end);
    H_m = H_full(7:end, 7:end);
end