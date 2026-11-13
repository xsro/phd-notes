function pdf = build_pdf_gain(K0, n, h, n_grid)
% BUILD_PDF_GAIN  Gramian-type periodic delay feedback gain (offline).
%
% 与原 main_ffsm_pdf_tracking_yan_params.m (已删除) 的局部函数一致:
%   W_c = \int_h^{2h} exp(-A_c s) B R_h(s) B' exp(-A_c' s) ds      (Eq 31)
%   K_c(theta) = R_h(theta) B' exp(-A_c' theta) W_c^{-1} exp(A_c (h-theta))
%                                                                  (Eq 32)
% 另预计算 theta in [0,2h) 上的 K_c 查找表, 供在线快速取值。
%
% 输入
%   K0      [n x 2n] 基准镇定增益 [-Kp, -Kd]
%   n       自由度
%   h       人工延迟 [s]
%   n_grid  (可选) 积分/查表网格点数, 默认 4001
%
% 输出 struct pdf: A, B, Ac, W, W_inv, h, n, condition_number,
%                  theta_grid, Kc_table (2n x n x n_grid)

    if nargin < 4 || isempty(n_grid)
        n_grid = 4001;
    end

    A  = [zeros(n), eye(n); zeros(n), zeros(n)];
    B  = [zeros(n); eye(n)];
    Ac = A + B*K0;

    % ---- 梯形法积分 W_c ----
    s_grid = linspace(h, 2*h, n_grid);
    ds = s_grid(2) - s_grid(1);
    W = zeros(2*n);
    for k = 1:n_grid
        s = s_grid(k);
        Rh = Rh_fun(s, h);
        E = expm(-Ac*s);
        w = 1;
        if k == 1 || k == n_grid
            w = 0.5;
        end
        W = W + w * (E * B * (Rh*eye(n)) * B' * E');
    end
    W = W*ds;
    W = 0.5*(W + W');

    pdf.A = A;
    pdf.B = B;
    pdf.Ac = Ac;
    pdf.W = W;
    pdf.W_inv = pinv(W);
    pdf.h = h;
    pdf.n = n;
    pdf.condition_number = cond(W);

    % ---- 预计算 K_c(theta) 查找表 ----
    th = linspace(0, 2*h, n_grid);
    Kc = zeros(n, 2*n, n_grid);
    for k = 1:n_grid
        Kc(:,:,k) = pdf_Kc(pdf, th(k));
    end
    pdf.theta_grid = th;
    pdf.Kc_table = Kc;
end

% ------------------------------------------------------------------------
function Kc = pdf_Kc(pdf, theta)
    Rh = Rh_fun(theta, pdf.h);
    if Rh == 0
        Kc = zeros(pdf.n, 2*pdf.n);
        return;
    end
    Kc = Rh * (pdf.B' * expm(-pdf.Ac'*theta) * pdf.W_inv * expm(pdf.Ac*(pdf.h-theta)));
end

% ------------------------------------------------------------------------
function Rh = Rh_fun(t, h)
% 光滑 2h-周期激活函数: theta in [0,h) 为 0; [h,2h) 为 sin^4
    theta = mod(t, 2*h);
    if theta < h
        Rh = 0;
    else
        s = (theta - h)/h;
        Rh = sin(pi*s)^4;
    end
end
