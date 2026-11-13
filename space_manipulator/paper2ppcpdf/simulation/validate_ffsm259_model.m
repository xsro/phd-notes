%% validate_ffsm259_model.m
% -------------------------------------------------------------------------
% 校核 ffsm259_model.m 的运动学与惯性矩阵
%   1. H 对称性 / 正定性 (随机位形)
%   2. 正运动学与 MATLAB Robotics System Toolbox rigidBodyTree 对比 (若可用)
%   3. 固定基座质量矩阵 Hm 与 rigidBodyTree massMatrix 对比 (若可用)
%   4. 质心雅可比数值差分二次校核
%   5. 无力矩自由漂浮的能量/动量守恒 (端到端自洽性)
%   6. 性质 2: Mdot - 2C 反对称
% -------------------------------------------------------------------------
clear; clc;

P = dou_yue_259_parameters();
n = P.n;
rng(0);

fprintf('=== 1. H 对称性 / 正定性 ===\n');
worst_asym = 0; min_eig = inf; min_eig_Me = inf;
for trial = 1:200
    q = (rand(n,1)-0.5)*2*pi;
    S = ffsm259_model(P, q);
    worst_asym = max(worst_asym, max(abs(S.H - S.H.'), [], 'all'));
    min_eig = min(min_eig, min(eig(S.H)));
    min_eig_Me = min(min_eig_Me, min(eig(S.Me)));
end
fprintf('  max|H-H''| = %.3e\n', worst_asym);
fprintf('  min eig(H_full) = %.4f   min eig(M_e) = %.4f\n', min_eig, min_eig_Me);
assert(worst_asym < 1e-9 && min_eig > 0, 'H 不对称或非正定');
fprintf('  [OK]\n');

fprintf('\n=== 2/3. 与 rigidBodyTree 对比 ===\n');
try
    rb = build_rbt(P);
    err_p = 0; err_M = 0;
    for trial = 1:20
        q = (rand(n,1)-0.5)*pi;
        S = ffsm259_model(P, q);
        cfg = homeConfiguration(rb);
        for i = 1:n
            cfg(i) = q(i) + P.dh.theta_off(i);
        end
        T = getTransform(rb, cfg, sprintf('link%d', n));
        err_p = max(err_p, norm(T(1:3,4) - S.pe));
        M = massMatrix(rb, cfg);
        err_M = max(err_M, max(abs(M - S.Hm), [], 'all') / max(abs(M), [], 'all'));
    end
    fprintf('  末端位置最大偏差      = %.3e m\n', err_p);
    fprintf('  固定基座质量矩阵相对误差 = %.3e\n', err_M);
    if err_p < 1e-9 && err_M < 1e-9
        fprintf('  [OK] 运动学与惯性矩阵与工具箱一致\n');
    else
        fprintf('  [!] 存在偏差, 请检查 DH 约定\n');
    end
catch ME
    fprintf('  跳过 (Robotics System Toolbox 不可用: %s)\n', ME.message);
end

fprintf('\n=== 4. 质心位置雅可比数值校核 (基座锁定) ===\n');
q = (rand(n,1)-0.5)*pi;
S0 = ffsm259_model(P, q);
h = 1e-6;
Mnum = zeros(3*n, n);   % [dp_c1/dq ; ... ; dp_cn/dq]
for j = 1:n
    qp = q; qp(j) = qp(j)+h; Sp = ffsm259_model(P, qp);
    qm = q; qm(j) = qm(j)-h; Sm = ffsm259_model(P, qm);
    d = (Sp.pc(:,2:end) - Sm.pc(:,2:end))/(2*h);
    Mnum(:,j) = d(:);
end
% 由质心雅可比重建固定基座质量矩阵的平动贡献
Hm_t = zeros(n);
for i = 1:n
    J = Mnum(3*(i-1)+1:3*i, :);
    Hm_t = Hm_t + P.mass(i+1)*(J.'*J);
    Hw = zeros(3,n);
    for k = 1:i, Hw(:,k) = S0.zax(:,k); end   % 关节轴 z_{i-1} 近似(基座锁定)
    Hm_t = Hm_t + Hw.'*(S0.R{i+1}*P.I{i+1}*S0.R{i+1}.' )*Hw;
end
fprintf('  平动+转动重建 Hm 与模型 Hm 相对偏差 = %.3e (近似, 含轴取法误差)\n', ...
    max(abs(Hm_t - S0.Hm),[],'all')/max(abs(S0.Hm),[],'all'));

fprintf('\n=== 5. 无力矩自由漂浮: 能量 / 动量守恒 (Ts = 2e-3, 2 s) ===\n');
Ts = 2e-3; Tf = 2.0; t = 0:Ts:Tf; N = numel(t);
q = P.q0; dq = [0.15; -0.1; 0.2; -0.15; 0.1; 0.12; -0.08];
KE = zeros(1,N); Mom = zeros(6,N);
for k = 1:N
    S = ffsm259_model(P, q, dq);
    xbd = -S.Hb \ (S.Hbm*dq);
    KE(k) = 0.5*dq.'*S.Me*dq;
    Mom(:,k) = S.Hb*xbd + S.Hbm*dq;
    ddq = S.Me \ (-S.Ce_qd);
    dq = dq + Ts*ddq;
    q  = q + Ts*dq;
end
fprintf('  KE: 初值 %.6f, 末值 %.6f, 相对漂移 %.3e\n', KE(1), KE(end), abs(KE(end)-KE(1))/KE(1));
fprintf('  动量最大模 %.3e\n', max(vecnorm(Mom,2,1)));

fprintf('\n=== 6. 性质 2: Mdot - 2C 反对称 ===\n');
q = (rand(n,1)-0.5)*pi; dq = randn(n,1);
h = 1e-6;
dM = zeros(n,n,n);
for k = 1:n
    qp = q; qp(k) = qp(k)+h;  Mp = ffsm259_model(P, qp).Me;
    qm = q; qm(k) = qm(k)-h;  Mm = ffsm259_model(P, qm).Me;
    dM(:,:,k) = (Mp-Mm)/(2*h);
end
C = zeros(n);
for i = 1:n
    for j = 1:n
        acc = 0;
        for k = 1:n
            acc = acc + 0.5*(dM(i,j,k) + dM(i,k,j) - dM(j,k,i))*dq(k);
        end
        C(i,j) = acc;
    end
end
Mdot = zeros(n);
for k = 1:n, Mdot = Mdot + dM(:,:,k)*dq(k); end
Sk = Mdot - 2*C;
fprintf('  max|Sk+Sk''|/|Mdot|      = %.3e\n', max(abs(Sk+Sk.'),[],'all')/max(abs(Mdot),[],'all'));
fprintf('  max|C*dq - 模型 Ce_qd|   = %.3e\n', max(abs(C*dq - ffsm259_model(P,q,dq).Ce_qd)));
fprintf('  |C*dq| 典型量级          = %.3e\n', max(abs(C*dq)));

function c = reduced_C(P, q, dq)
    S = ffsm259_model(P, q, dq);
    c = S.Ce_qd;
end

function rb = build_rbt(P)
    rb = rigidBodyTree('DataFormat','column');
    parent = rb.BaseName;
    for i = 1:P.n
        b = rigidBody(sprintf('link%d', i));
        b.Mass = P.mass(i+1);
        b.CenterOfMass = P.a(:, i+1).';
        % 注意: rigidBody.Inertia 定义在"体系原点"而非质心 (已用单连杆测试确认),
        % 因此需把绕质心的 I 平移到体系原点: I0 = Icm + m*(|a|^2*E - a*a')
        ac = P.a(:, i+1);
        I0 = P.I{i+1} + P.mass(i+1)*((ac.'*ac)*eye(3) - ac*ac.');
        b.Inertia = [diag(I0).' 0 0 0];
        jnt = rigidBodyJoint(sprintf('joint%d', i), 'revolute');
        % 标准 DH: T = Rz(theta)*Tz(d)*Tx(l)*Rx(alpha)
        dseg = P.dh.d_seg(i);
        if i == 1
            dseg = dseg + P.dh.base_offset;   % 基座质心 -> 关节 1 的偏置
        end
        setFixedTransform(jnt, [P.dh.l_seg(i), P.dh.alpha_seg(i), dseg, 0], 'dh');
        b.Joint = jnt;
        addBody(rb, b, parent);
        parent = b.Name;
    end
end

function R = rotx(t)
    R = [1,0,0; 0,cos(t),-sin(t); 0,sin(t),cos(t)];
end
