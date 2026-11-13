function an = pdf_periodian(pdf, Ts, n_s)
% PDF_PERIODIAN  由 PDF 增益数值计算单周期映射与周期放大系数
%
% 计算三组离线量, 对应论文中的三处判据:
%
%   (1) Delta_K(h) = Z(2h), 其中 Z(theta) 为齐次延迟闭环
%           dZ/dtheta = A_c*Z(theta) - B*K_c(theta)*Z(theta-h),  Z(0)=I, Z(<0)=0
%       的基解矩阵。定理 3.3 要求 ||Delta_K(h)|| = 0; 数值上为 eps_Delta
%       (命题 4.2 的近似零化参数)。
%
%   (2) Gamma_h = max_theta int_0^theta ||S_h(theta,s)*B|| ds   (论文式 (38))
%       由单位面积脉冲响应的数值积分得到 (n_s 个脉冲位置)。
%
%   (3) 命题 4.1 的包络判据常数
%           E_h = max_{theta in [0,h]} ||exp(A_c*theta)||
%           M_h = E_h^2 * (1 + ||B|| * int_h^{2h} ||K_c(s)|| ds)   (先验闭式上界)
%           Mop = max_theta ||T(theta,0)||                         (精确算子范数)
%           Mpos(i) = max_theta ||T_i(theta,0)||_2                  (逐关节位置行块)
%       由 |e_i(t)| <= Mpos(i)*||z(To)||_2 得到可离线校验的包络判据。
%
% 输入
%   pdf  build_pdf_gain 输出
%   Ts   积分步长 [s] (默认 1e-3)
%   n_s  脉冲位置数 (默认 41)
% 输出
%   an   DeltaK, deltaK, deltaK_rel, Gamma, E_h, M_h, intKc, cond

    if nargin < 2 || isempty(Ts),  Ts  = 1e-3; end
    if nargin < 3 || isempty(n_s), n_s = 41;   end

    N  = 2*pdf.n;
    Ac = pdf.Ac;  B = pdf.B;  h = pdf.h;

    M  = round(h/Ts);                 % 半周期步数
    F1 = expm(Ac*Ts);
    F0 = Ac \ (F1 - eye(N));          % int_0^{Ts} exp(Ac*(Ts-s)) ds

    % ---------------- (1) 单周期映射 ----------------
    Z = zeros(N, N, 2*M+1);           % Z(:,:,k) 对应 t = (k-1)*Ts
    Z(:,:,1) = eye(N);
    for k = 1:2*M
        th = (k-1)*Ts;
        Kc = pdf_gain_at(pdf, th);
        if k - M >= 1
            Xd = Z(:,:,k-M);          % z(t-h)
        else
            Xd = zeros(N);            % 历史为零 (t < h)
        end
        Z(:,:,k+1) = F1*Z(:,:,k) - F0*(B*Kc*Xd);
    end
    DeltaK = Z(:,:,end);

    % 周期内传输算子 (命题 4.1 的精确形式): M_h = max_theta ||T(theta,0)||
    Mop = 0;
    Mpos = zeros(pdf.n, 1);            % 逐关节位置行块范数 ||T_i(theta,0)||_2
    for k = 1:2*M+1
        Mop = max(Mop, norm(Z(:,:,k)));
        for i = 1:pdf.n
            Mpos(i) = max(Mpos(i), norm(Z(i,:,k)));
        end
    end

    % ---------------- (2) 周期放大系数 Gamma_h ----------------
    ss  = linspace(0, 2*h, n_s);
    ds  = ss(2) - ss(1);
    acc = zeros(2*M+1, 1);            % acc(k) 对应 t = (k-1)*Ts
    Xh  = zeros(N, pdf.n, 2*M+2);     % 每次脉冲传播内的历史缓存

    for j = 1:n_s
        k0 = round(ss(j)/Ts) + 1;
        if k0 > 2*M, continue; end
        X = B;                        % 单位面积脉冲: z(s_j^+) = B
        Xh(:,:,1) = X;
        for k = k0:2*M
            th = (k-1)*Ts;
            Kc = pdf_gain_at(pdf, th);
            idx_d = k - M - k0 + 1;
            if idx_d >= 1
                Xd = Xh(:,:,idx_d);
            else
                Xd = zeros(N, pdf.n);
            end
            X = F1*X - F0*(B*Kc*Xd);
            Xh(:,:,k-k0+2) = X;
            acc(k+1) = acc(k+1) + norm(X)*ds;
        end
    end
    Gamma = max(acc);

    % ---------------- (3) E_h 与 M_h ----------------
    th_grid = linspace(0, h, 400);
    E_h = 0;
    for k = 1:numel(th_grid)
        E_h = max(E_h, norm(expm(Ac*th_grid(k))));
    end

    th_tab  = pdf.theta_grid;
    Kc_norm = zeros(size(th_tab));
    for k = 1:numel(th_tab)
        Kc_norm(k) = norm(pdf.Kc_table(:,:,k));
    end
    msk   = th_tab >= h;
    intKc = trapz(th_tab(msk), Kc_norm(msk));
    M_h   = E_h^2 * (1 + norm(B)*intKc);

    an = struct('DeltaK', DeltaK, 'deltaK', norm(DeltaK), ...
        'deltaK_rel', norm(DeltaK)/max(1, norm(expm(Ac*2*h))), ...
        'Gamma', Gamma, 'E_h', E_h, 'M_h', M_h, 'intKc', intKc, ...
        'Mop', Mop, 'Mpos', Mpos, ...
        'cond', pdf.condition_number, 'Ts', Ts, 'n_s', n_s);
end
