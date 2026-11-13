function [rho, rho_dot, rho_ddot] = perf_rho(t, pp)
% PERF_RHO  指定时间可达性能函数 (论文式 (11))
%
%   rho_i(t) = rho_inf_i + (rho0_i - rho_inf_i)*phi(t/T_o),  0 <= t < T_o
%   rho_i(t) = rho_inf_i ,                                   t >= To
%
% 归一化包络 phi 取五次 smootherstep 的补:
%   phi(tau) = 1 - (10*tau^3 - 15*tau^4 + 6*tau^5),  tau = t/T_o in [0,1]
% 其性质:
%   phi(0)=1, phi(1)=0,  phi'(0)=phi'(1)=0,  phi''(0)=phi''(1)=0
% 因此
%   (i)  rho_dot(To) = rho_ddot(To) = 0  -> 切换点 To 处不产生 rho'' 力矩跳变
%        (论文引理 2.2 与注 4.1);
%   (ii) rho_dot(0) = 0  -> 初始时刻性能函数不收缩, 避免"漏斗立即收缩而速度
%        误差为零"所造成的初始力矩冲击 (若取 rho ~ (1-t/To)^m, 则 |rho_dot|
%        在 t=0 处达到最大值 m*(rho0-rho_inf)/To, 在重惯量关节上会直接触发
%        饱和, 见数值实验);
%   (iii) rho 在 [0,To] 上 C^2, 二阶导数有界: |rho''| <= 5.77*(rho0-rho_inf)/To^2.
%
% 输入
%   t   标量时间 [s]
%   pp  ppc_settings 生成的结构体 (字段 To, rho0, rho_inf)
% 输出
%   rho, rho_dot, rho_ddot   [n x 1]

    if t < pp.To
        tau = t/pp.To;
        tau = min(max(tau, 0), 1);
        dphi   = -30*tau^2*(1-tau)^2;
        ddphi  = -60*tau*(1-tau)*(1-2*tau);
        phi    = 1 - (10*tau^3 - 15*tau^4 + 6*tau^5);
        dB     = pp.rho0 - pp.rho_inf;
        rho      = pp.rho_inf + dB*phi;
        rho_dot  = dB*dphi/pp.To;
        rho_ddot = dB*ddphi/pp.To^2;
    else
        rho      = pp.rho_inf;
        rho_dot  = zeros(size(pp.rho_inf));
        rho_ddot = zeros(size(pp.rho_inf));
    end
end
