function P = dou_yue_259_parameters()
% DOU_YUE_259_PARAMETERS  7-DOF free-floating space manipulator parameters.
%
% 文献来源 (OKB 文献库 document id = 259):
%   Bo Dou, Xiaokui Yue, "Fractional-Order Sliding Mode Control for
%   Free-Floating Space Manipulator With Disturbance and Input Saturation",
%   Int. J. Robust Nonlinear Control, 2025, 35(8): 3094-3115.
%   DOI: 10.1002/rnc.7826
%
% 本函数逐项转录该文:
%   Table 1  "Parameters of the 7-DOF FFSM in 3-D scenario."
%   Table 2  "Initial state values of the FFSM."
%
% ------------------------------------------------------------------------
% 运动学约定 (本实现采用, 见文件末尾的 KNOWN_ISSUES 说明)
% ------------------------------------------------------------------------
% 标准 DH 变换, 关节 i 绕 z_{i-1} 旋转:
%     T_i = Rz(theta_i) * Tz(d_i) * Tx(l_i) * Rx(alpha_i)
% 其中 theta_i = q_i + theta_off_i (Table 1 的 theta_m,i 列给出偏置:
% 关节 3 为 theta_m3 - 90deg, 关节 5 为 theta_m5 + 90deg)。
% 体 i 的质心相对本体系原点偏移 a_i, 转动惯量 I_i 为绕质心在本体系下的
% 主惯量 (Table 1 只给 Ixx/Iyy/Izz, 非对角项为 0)。
%
% 输出字段:
%   n            自由度
%   mass(1+n)    质量 [kg], 第 1 项为基座
%   I{1+n}       3x3 转动惯量 [kg m^2], 第 1 项为基座
%   a(3,1+n)     质心偏移 [m] (本体系)
%   b(3,1+n)     Table 1 原表 b_i (质心 -> 下一关节原点), 仅用于一致性校核
%   dh           运动学参数: alpha[rad], l[m], d[m], theta_off[rad]
%   tau_max(n,1) 关节力矩上限 [Nm] (对称, tau_min = -tau_max)
%   q0, dq0      初始关节角/角速度 (Table 2)
%   base0        初始基座位姿 [r(3); quat(4)] (Table 2: (0,0,0, 1,0,0,0))
%   unc          第二部分算例的"真实"参数摄动 (m7, I7), 用于状态演化
%   ref          文献元数据

    n = 7;
    P.n = n;

    % ---- Table 1: 质量 [kg] (Base, Link 1..7) --------------------------
    P.mass = [1000, 35.01, 30, 22.69, 21.38, 16.75, 26.17, 18.1];

    % ---- Table 1: 转动惯量 [kg m^2] ------------------------------------
    Ixx = [1200, 1.218,  2.1,    0.102, 0.4327, 0.3878,  0.5727,  0.17  ];
    Iyy = [1200, 0.5132, 1.378,  3.378, 2.266,  0.3963,  0.5987,  0.24  ];
    Izz = [1200, 1.331,  2.359,  3.359, 1.911,  0.07271, 0.1288,  0.14  ];
    P.I = cell(1, n+1);
    for i = 1:n+1
        P.I{i} = diag([Ixx(i), Iyy(i), Izz(i)]);
    end

    % ---- Table 1: a_i [m], 关节 i 原点到 Link i 质心 --------------------
    P.a = [ 0,  0,     0,    0.6,  0.6,  0,     0,     0    ;
            0, -0.11,  0.1,  0,    0,   -0.1,   0.15,  0    ;
            0,  0,     0,    0,    0,    0,     0,     0.05 ];

    % ---- Table 1: b_i [m], Link i 质心到关节 i+1 原点 -------------------
    % 原表给出; 与 DH 参数交叉校核存在一阶错位 (见 KNOWN_ISSUES),
    % 因此动力学计算只使用 a_i + DH, b 仅保留用于核查。
    P.b = [ 0.72,  0,     0,    0.6,  0.6,  0,     0,     0    ;
            0.78, -0.11,  0.1,  0,    0,   -0.1,   0.15,  0    ;
            0.52,  0,     0,    0.24, 0.22, 0,     0,     0.05 ];

    % ---- Table 1: DH 参数 ---------------------------------------------
    P.dh.alpha     = deg2rad([ 90,  90, -90,   0,    0,   90,  -90]);
    P.dh.l         =          [  0,   0,   0,   1.2,  1.2,  0,    0 ];
    P.dh.d         =          [0.18, 0.22, 0.20, 0.24, 0.22, 0.20, 0.30];
    P.dh.theta_off = deg2rad([  0,   0, -90,   0,   90,   0,    0 ]);

    % ---- DH 行与关节编号的错位修正 (见 KNOWN_ISSUES 第 1 条) ----------
    % |a_i + b_i| 与 |[l_{i+1}, 0, d_{i+1}]| 在 i = 1..6 上完全一致, 且
    % a_3 = [0.6,0,0] / b_3 = [0.6,0,0.24] 说明连杆 3 长为 1.2 m。
    % 因此第 i 段连杆 (关节 i -> 关节 i+1) 使用 (alpha_i, l_{i+1}, d_{i+1}),
    % 基座质心 -> 关节 1 用 d_1, 末端用 d_8 = 0.10 m (= |a_7 + b_7|)。
    P.dh.base_offset = P.dh.d(1);                       % 基座质心到关节 1 [m]
    P.dh.l_seg = [P.dh.l(2:end), 0   ];                 % 连杆 1..7 的 l_i
    P.dh.d_seg = [P.dh.d(2:end), 0.10];                 % 连杆 1..7 的 d_i
    P.dh.alpha_seg = P.dh.alpha;                        % 连杆 1..7 的 alpha_i

    % ---- Table 1: 关节力矩饱和 [Nm] -----------------------------------
    P.tau_max = 20 * ones(n, 1);
    P.tau_min = -P.tau_max;

    % ---- Table 2: 初始状态 --------------------------------------------
    P.q0  = deg2rad([0, -90, 45, -45, 90, 90, 0]).';
    P.dq0 = zeros(n, 1);
    P.base0 = [0; 0; 0; 1; 0; 0; 0];   % r_b [m]; quaternion (scalar first)

    % ---- 第二部分算例的真实参数摄动 (捕获非合作目标后) ----------------
    % 文中: m7 = 40 kg, I7xx = I7yy = I7zz = 2.5 kg m^2
    P.unc.mass = P.mass;             P.unc.mass(8) = 40;
    P.unc.I    = P.I;                P.unc.I{8}    = 2.5*eye(3);

    % ---- 文献元数据 ----------------------------------------------------
    P.ref.id      = 259;
    P.ref.doi     = '10.1002/rnc.7826';
    P.ref.title   = ['Fractional-Order Sliding Mode Control for Free-Floating ' ...
                     'Space Manipulator With Disturbance and Input Saturation'];
    P.ref.authors = 'Bo Dou, Xiaokui Yue';
    P.ref.venue   = 'Int. J. Robust Nonlinear Control, 2025, 35(8): 3094-3115';
    P.ref.source  = 'OKB document id 259';
end

% ========================================================================
%  KNOWN_ISSUES -- Table 1 的内部一致性问题 (本实现的处理方式)
% ========================================================================
% 1) 连杆长度错位: 按 (alpha_i, l_i, d_i) 计算的关节 i->i+1 线段长度序列为
%        [0.18 0.22 0.20 1.224 1.22 0.20 0.30]
%    而由表中 a_i + b_i 得到的长度序列为
%        [0.22 0.20 1.224 1.22 0.20 0.30 0.10]
%    两者整体错一位 (d_{i+1} 对应到 link i), 且末项 0.10 m 在 DH 表中没有
%    对应项 (疑为末端工具偏移)。
%    处理: 动力学只使用 DH 参数 + 质量 + 惯量 + a_i, 这是自洽且完整的描述;
%    串联链总长两种读法相同 (~3.5 m, 与期望末端位置 3.2 m 量级一致),
%    因此该错位不影响关节空间对比的公平性。运行
%    dou_yue_259_check_consistency 可打印逐连杆偏差。
%
% 2) 表头 "a_i, m" 与 "b_i, m" 各占 3 行 (x/y/z 分量), 表头行数与列数在
%    PDF 抽取文本中会被拆散, 本文件按 3 行分量读取。
%
% 3) Table 3 未给出 rho_E3 (即 lambda_2 sig(y)^{rho_E3} 的幂次), 本实现取
%    rho3 = 0.5 并在 dou_yue_259_control_params 中显式标注。
