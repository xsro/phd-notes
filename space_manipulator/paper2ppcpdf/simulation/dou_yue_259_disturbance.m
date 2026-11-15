function [d, d_dot, d_ddot] = dou_yue_259_disturbance(t, n)
% DOU_YUE_259_DISTURBANCE  External joint-torque disturbance, Eq (74).
%
% 文献: OKB document id 259 (Dou & Yue 2025), Eq (74):
%   tau_d1..4 = 0.2 sin(0.1 t + phi_i) + 0.05 sin(0.01 pi t + phi_i)
%   tau_d5..7 = 0.05 * ( 2 sin(0.1 t + pi/6) + sin(0.01 pi t + pi/6) )
% 相位 phi_i = pi/4, pi/2, pi/3, pi/5 (i = 1..4)。
%
% 输出
%   d       [n x T] 扰动力矩
%   d_dot   [n x T] 一阶导数  (可选, 用于核验 (C3) 的 L_i)
%   d_ddot  [n x T] 二阶导数  (可选, 用于核验 (C3) 的 H_i)
% 注意: 时间同时支持行向量与列向量。

    if nargin < 2
        n = 7;
    end
    t = t(:).';
    phi = [pi/4, pi/2, pi/3, pi/5];
    w2  = 0.01*pi;

    d = zeros(n, numel(t));
    for i = 1:min(4, n)
        d(i, :) = 0.2*sin(0.1*t + phi(i)) + 0.05*sin(w2*t + phi(i));
    end
    for i = 5:n
        d(i, :) = 0.05 * ( 2*sin(0.1*t + pi/6) + sin(w2*t + pi/6) );
    end

    if nargout >= 2
        d_dot = zeros(size(d));
        for i = 1:min(4, n)
            d_dot(i, :) = 0.02*cos(0.1*t + phi(i)) + 0.05*w2*cos(w2*t + phi(i));
        end
        for i = 5:n
            d_dot(i, :) = 0.05 * ( 0.2*cos(0.1*t + pi/6) + w2*cos(w2*t + pi/6) );
        end
    end

    if nargout >= 3
        d_ddot = zeros(size(d));
        for i = 1:min(4, n)
            d_ddot(i, :) = -0.002*sin(0.1*t + phi(i)) - 0.05*w2^2*sin(w2*t + phi(i));
        end
        for i = 5:n
            d_ddot(i, :) = 0.05 * ( -0.02*sin(0.1*t + pi/6) - w2^2*sin(w2*t + pi/6) );
        end
    end
end
