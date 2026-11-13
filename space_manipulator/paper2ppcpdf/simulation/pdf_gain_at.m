function Kc = pdf_gain_at(pdf, t)
% PDF_GAIN_AT  在线取 Gramian 型 PDF 增益 K_c(t) (查表 + 线性插值)。
%
%   Kc = pdf_gain_at(pdf, t)  返回 [n x 2n]
%
% 时间按 mod(t, 2h) 周期延拓, 保持 act-and-wait 结构 (见论文第 5 节)。

    th = mod(t, 2*pdf.h);
    n_grid = numel(pdf.theta_grid);
    ds = pdf.theta_grid(2) - pdf.theta_grid(1);
    k = floor(th/ds) + 1;
    k = min(max(k, 1), n_grid-1);
    a = (th - pdf.theta_grid(k)) / ds;
    Kc = (1-a)*pdf.Kc_table(:,:,k) + a*pdf.Kc_table(:,:,k+1);
end
