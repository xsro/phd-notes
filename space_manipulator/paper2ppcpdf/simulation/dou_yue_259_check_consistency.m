%% dou_yue_259_check_consistency.m
% -------------------------------------------------------------------------
% 核查文献 [259] Table 1 的内部一致性 (见 dou_yue_259_parameters 的
% KNOWN_ISSUES 说明):
%   (1) 按 DH 参数 (alpha_i, l_i, d_i) 计算的连杆长度 |[l_i,0,d_i]|
%       与表中 a_i + b_i 的长度 |a_i+b_i| 的对应关系;
%   (2) 基座质心到关节 1 的距离;
%   (3) 惯量的物理合理性 (正定 + 三角不等式);
%   (4) 初始位形下的末端位置与可达性。
% -------------------------------------------------------------------------
clear; clc;
P = dou_yue_259_parameters();
n = P.n;

fprintf('=== Table 1 连杆长度: DH 行 vs a_i + b_i ===\n');
fprintf('%-6s %14s %12s %10s\n', 'link', '|[l_i,0,d_i]|', '|a_i+b_i|', 'diff');
Ldh = zeros(1,n); Lab = zeros(1,n);
for i = 1:n
    Ldh(i) = norm([P.dh.l(i); 0; P.dh.d(i)]);
    Lab(i) = norm(P.a(:,i+1) + P.b(:,i+1));
    fprintf('%-6d %14.4f %12.4f %10.4f\n', i, Ldh(i), Lab(i), Lab(i)-Ldh(i));
end
fprintf('\nDH 行 长度序列        : %s\n', mat2str(Ldh, 4));
fprintf('a_i + b_i 长度序列     : %s\n', mat2str(Lab, 4));
fprintf('DH 行 右移一位         : %s\n', mat2str(Ldh(2:end), 4));
fprintf('a_i + b_i 前 6 位      : %s\n', mat2str(Lab(1:6), 4));

fprintf('\n观察:\n');
fprintf('  * |a_i + b_i| = |[l_{i+1},0,d_{i+1}]| 对 i = 1..6 成立 (整体错一位)\n');
fprintf('  * |a_7 + b_7| = 0.10 m 对应 d_8 = 0.10 m (连杆 7 自身长度)\n');
fprintf('  * d_1 = 0.18 m 是基座质心到关节 1 的偏置\n');
fprintf('  * 错位读法下 a_3 = [0.6,0,0] 与连杆 3 长 1.2 m 自洽 (质心在连杆内)\n');
fprintf('  => ffsm259_model 采用: 连杆 i 取 (alpha_i, l_{i+1}, d_{i+1})\n');
fprintf('     串联链总长 = %.3f m, 表值 a+b 总和 = %.3f m\n', ...
    P.dh.base_offset + sum(sqrt(P.dh.l_seg.^2 + P.dh.d_seg.^2)), sum(Lab));

fprintf('\n=== 基座-关节 1 距离 ===\n');
fprintf('  |b_0| = %.4f m,  d_1 = %.4f m  (表内不自洽, 本实现取 d_1)\n', ...
    norm(P.b(:,1)), P.dh.d(1));

fprintf('\n=== 惯量合理性 ===\n');
ok = true;
for i = 1:n+1
    Ii = P.I{i};
    e = eig(Ii);
    tri = [Ii(1,1) <= Ii(2,2)+Ii(3,3), ...
           Ii(2,2) <= Ii(1,1)+Ii(3,3), ...
           Ii(3,3) <= Ii(1,1)+Ii(2,2)];
    if any(e <= 0) || ~all(tri)
        ok = false;
        fprintf('  body %d: 惯量不正定或不满足三角不等式\n', i-1);
    end
end
if ok
    fprintf('  全部 8 个刚体惯量正定且满足三角不等式 [OK]\n');
end
fprintf('  连杆质量和 = %.2f kg, 基座/连杆质量比 = %.1f\n', ...
    sum(P.mass(2:end)), P.mass(1)/sum(P.mass(2:end)));

fprintf('\n=== 初始位形 ===\n');
S0 = ffsm259_model(P, P.q0);
fprintf('  q0 = %s deg\n', mat2str(rad2deg(P.q0).', 4));
fprintf('  末端位置 (基座系) = %s m, |p_e| = %.3f m\n', mat2str(S0.pe.', 4), norm(S0.pe));
fprintf('  串联链总长        = %.3f m\n', ...
    P.dh.base_offset + sum(sqrt(P.dh.l_seg.^2 + P.dh.d_seg.^2)));
fprintf('  Me 对角 (q0)      = %s kg m^2\n', mat2str(diag(S0.Me).', 4));
fprintf('  广义雅可比 cond   = %.3e\n', cond(S0.J));
