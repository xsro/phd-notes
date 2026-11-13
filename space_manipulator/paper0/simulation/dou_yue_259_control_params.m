function C = dou_yue_259_control_params()
% DOU_YUE_259_CONTROL_PARAMS  Controller gains from Dou & Yue (2025) Table 3.
%
% 文献: OKB document id 259, DOI 10.1002/rnc.7826, Table 3
%       "Contorl parameters of Part (1)."  (原文拼写如此)
%
% 说明: 原文的滑模面定义在末端笛卡尔空间 (Eq 48) 与基座姿态 (Eq 52),
% 维数为 3。本工程在等价的关节空间实施 (Eq 15 的滑模面作用于关节误差),
% 维数为 n = 7, 因此把 diag(4,4,4) 之类的 3x3 增益按同一标量扩为 nxn。
%
% 输出:
%   C.fonsm   分数阶非奇异终端滑模面 (Eq 15)
%   C.ftdob   有限时间扰动观测器 (Eq 35-36)
%   C.ftasc   固定时间抗饱和补偿器 (Eq 37/44/45) —— Case (1)
%   C.ftac    有限时间抗饱和补偿器 (Eq 46) —— Case (2), 文献 [29]
%   C.fosm    Shao 等人的分数阶滑模面 (Eq 77), 第二部分对比对象

    n = 7;
    on = @(v) v*eye(n);

    % ---- Eq (15)/(48)/(52): FONTSM 滑模面 ------------------------------
    C.fonsm.C1        = on(4);        % C_E1 = diag(4,4,4)
    C.fonsm.C2        = on(4);        % C_E2 = diag(4,4,4)
    C.fonsm.xi        = 0.9;          % 分数阶次
    C.fonsm.b         = 0.8;
    C.fonsm.r1        = 0.5;
    C.fonsm.r2        = 1.5;
    C.fonsm.delta     = 0.1;
    % lambda_E1 = lambda_B1 = diag(4,4,4); lambda_E2 = lambda_B2 = diag(2,2,2)
    C.fonsm.lambda1   = on(4);
    C.fonsm.lambda2   = on(2);
    C.fonsm.rho3      = 0.5;   % 原文未给出 rho_E3, 本实现取 0.5

    % ---- Eq (35)/(36): FTDOB ------------------------------------------
    C.ftdob.k1  = on(10);
    C.ftdob.k2  = on(10);
    C.ftdob.mu1 = 1;
    C.ftdob.mu2 = 1;
    C.ftdob.alpha = 2/3;
    C.ftdob.beta  = 3/2;
    C.ftdob.phat0 = zeros(n, 1);

    % ---- Eq (37)/(44)/(45): 固定时间抗饱和补偿器 (Case 1) -------------
    C.ftasc.k_eta1 = on(1);           % k_etaE1 = diag(1,1,1)
    C.ftasc.k_eta2 = on(100);         % k_etaE2 = diag(100,100,100)
    C.ftasc.rho1   = 0.1;             % rho_E1
    C.ftasc.rho2   = 0.9;             % rho_E2
    C.ftasc.kL     = [10, 0.1, 0.01, 0.001];   % k_L1..k_L4
    C.ftasc.a      = [1.5, 0.5];               % a_1, a_2
    C.ftasc.cL     = 0.001;
    C.ftasc.L0     = 0.001;           % L_eta(0)
    C.ftasc.PL     = 0.001;           % P^L
    C.ftasc.PU     = 0.1;             % P^U

    % ---- Eq (46): 有限时间抗饱和补偿器 (Case 2, 文献 [29]) -----------
    C.ftac.k_eta3 = on(0.01);         % k_etaE3 = diag(0.01,...)
    C.ftac.k_eta4 = on(0.1);          % k_etaE4 = diag(0.1,...)
    C.ftac.alpha  = 0.5;              % alpha_etaE

    % ---- Eq (77): FOSM (Shao et al.) ---------------------------------
    C.fosm.C1 = on(2);   % Part II: C_E1 = C_E2 = diag(2,2,2)
    C.fosm.C2 = on(2);
    C.fosm.xi = C.fonsm.xi;
    C.fosm.b  = C.fonsm.b;

    % ---- 本工程方法：预设时间扰动观测器 PTDO (论文 Eq (19)-(22)) ----
    % 用于在文献 259 的被控对象上验证本文第 4 节的 PTDO / PDF 结构
    % T_c 需与被控对象的时间尺度匹配: T_c = 1 s 时观测器增益 pi/(eta*T_c)
    % 与 pi/(sigma*eta*T_c) 过大, 在 Ts = 2 ms 离散下产生高频抖振
    % (见 tune_ptdo.m); 整定后取 T_c = 5 s。
    C.ptdo.T_o   = 1.0;      % 预设观测时间 [s]
    C.ptdo.T_c   = 5.0;      % 观测器内部收敛时间常数 [s]（整定值）
    C.ptdo.eta   = 0.3;      % eta in (0,1)
    C.ptdo.sigma = 0.4;      % 边界层参数 sigma > 0
    C.ptdo.xi0   = zeros(n, 1);

    % 未整定对照 (原设定 T_c = 1 s)
    C.ptdo_tc1 = C.ptdo;
    C.ptdo_tc1.T_c = 1.0;
end
