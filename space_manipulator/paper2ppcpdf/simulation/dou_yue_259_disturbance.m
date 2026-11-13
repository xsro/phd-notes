function d = dou_yue_259_disturbance(t, n)
% DOU_YUE_259_DISTURBANCE  External joint-torque disturbance, Eq (74).
%
% 文献: OKB document id 259 (Dou & Yue 2025), Eq (74):
%   tau_d1..4 = 0.2 sin(0.1 t + phi_i) + 0.05 sin(0.01 pi t + phi_i)
%   tau_d5..7 = 0.05 * ( 2 sin(0.1 t + pi/6) + sin(0.01 pi t + pi/6) )
% 相位 phi_i = pi/4, pi/2, pi/3, pi/5 (i = 1..4)。
%
% 输出: d [n x 1]  >>  注意文件中同时支持列矢量化与标量时间。

    if nargin < 2
        n = 7;
    end
    t = t(:).';
    phi = [pi/4, pi/2, pi/3, pi/5];

    d = zeros(n, numel(t));
    for i = 1:min(4, n)
        d(i, :) = 0.2*sin(0.1*t + phi(i)) + 0.05*sin(0.01*pi*t + phi(i));
    end
    for i = 5:n
        d(i, :) = 0.05 * ( 2*sin(0.1*t + pi/6) + sin(0.01*pi*t + pi/6) );
    end
end
