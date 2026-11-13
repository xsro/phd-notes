function [tau_cmd, c, log] = fonsm259_step(c, q, dq, qd, dqd, ddqd, M, Cmat, tau_act, tau_cmd_prev)
% FONSM259_STEP  一步 Dou & Yue (2025) FONTSM/FOSM 控制器
%
% 由式 (15)(35)(36)(37)(44)(50)(57) 在关节空间实现:
%
%   滑模面   s   = edot + C1*chi(e) + C2*D^{xi-1}[sig(e)^b]           (15)
%   观测器   p   = M*dq
%            p_hat'  = k1*phi1(p-p_hat) + C'*dq + tau_act + tau_hat   (35)
%            tau_hat'= k2*phi2(p-p_hat)                                (36)
%   补偿器   eta' = -L*(k1/rho1)tanh(|eta|)^{1-rho1}cosh(|eta|)^2 sign(eta)
%                    - L*k2*sig(eta)^{rho2} + Du                       (37)
%   参考加速度  ddq_r = ddq_d + C1*h(e)*edot + C2*D^xi[sig(e)^b]
%                    + [同上 eta 项] + lam1*y + lam2*sig(y)^{rho3}      (50)
%   控制律   tau_cmd = C*dq + M*ddq_r - tau_hat                       (57)
%
% 输入
%   c            控制器状态 (fonsm259_init)
%   q,dq         实际关节角/角速度
%   qd,dqd,ddqd  参考关节角/角速度/角加速度
%   M   (n x n)  名义等效惯性矩阵 M_theta_m
%   Cmat(n x n)  名义等效科里奥利矩阵 C_theta_m
%   tau_act      上一时刻实际施加的力矩 (含饱和偏差)
%   tau_cmd_prev 上一时刻指令力矩 (用于计算 Du)
%
% 输出
%   tau_cmd  力矩指令 (未限幅)
%   c        更新后的状态
%   log      struct: s, y, eta, tau_hat, L, chi

    n = c.n;
    Ts = c.Ts;

    % ========== 1. 误差 ==========
    e  = qd - q;
    de = dqd - dq;

    % ========== 2. 分数阶记忆 ==========
    A = sig_pow(e, c.b);                       % sig(e)^b
    % d/dt sig(e)^b = b*|e|^(b-1)*edot  (|e| 取下限避免奇异)
    ab = max(abs(e), 1e-9);
    dB = c.b * ab.^(c.b - 1) .* de;
    c.memA = [A.'; c.memA(1:end-1, :)];
    c.memB = [dB.'; c.memB(1:end-1, :)];
    c.cnt = min(c.cnt + 1, c.mem_len);
    k = c.cnt;
    scale = Ts^(-c.nu);
    Aint = scale * (c.w(1:k).' * c.memA(1:k, :));      % D^{xi-1}[sig(e)^b]
    Bint = scale * (c.w(1:k).' * c.memB(1:k, :));      % D^{xi}[sig(e)^b]
    Aint = Aint(:);  Bint = Bint(:);

    % ========== 3. 有限时间扰动观测器 (35)(36) ==========
    [c.dob, tau_hat] = ftdob_step(c.dob, M, Cmat, dq, tau_act, Ts);
    if isempty(c.dob) || ~isfield(c.dob, 'enabled') || ~c.dob.enabled
        tau_hat = zeros(n, 1);
    end

    % ========== 4. 抗饱和补偿器 (37)/(44)/(45)/(46) ==========
    Du = M \ (tau_act - tau_cmd_prev);       % Du = H^{-1}*Delta_tau  (Eq 47)
    switch c.asc.mode
        case 'ftasc'
            [c.asc.eta, c.asc.Leta, fcomp] = ...
                ftasc_update(c.asc, c.asc.eta, c.asc.Leta, Du, Ts);
        case 'ftac'
            [c.asc.eta, fcomp] = ftac_update(c.asc, c.asc.eta, Du, Ts);
        otherwise
            c.asc.eta = zeros(n, 1);
            fcomp = zeros(n, 1);
    end
    eta = c.asc.eta;

    % ========== 5. 滑模面与参考加速度 ==========
    [chi, hder] = chi_fun(e, c);
    s = de + c.C1*chi + c.C2*Aint;
    y = s - eta;

    ddq_r = ddqd + c.kappa*( c.C1*(hder.*de) + c.C2*Bint + fcomp ...
          + c.lambda1*y + c.lambda2*sig_pow(y, c.rho3) );

    % ========== 6. 控制律 (57) ==========
    tau_cmd = Cmat*dq + M*ddq_r - tau_hat;

    log.s = s;  log.y = y;  log.eta = eta;
    log.tau_hat = tau_hat;  log.L = c.asc.Leta;  log.chi = chi;
end

% ========================================================================
function y = sig_pow(x, r)
    y = abs(x).^r .* sign(x);
end

% ------------------------------------------------------------------------
function [chi, hder] = chi_fun(e, c)
% Eq (15)-(16): 非线性项 chi 及其偏导 h = dchi/de
% FOSM (Eq 77) 退化为线性项 chi = e, h = 1。
    if strcmp(c.method, 'fosm')
        chi  = e;
        hder = ones(size(e));
        return;
    end
    chi  = zeros(size(e));
    hder = zeros(size(e));
    for i = 1:numel(e)
        ae = abs(e(i));
        if ae >= c.delta
            chi(i)  = sign(e(i))*ae^c.r1;
            hder(i) = c.r1*ae^(c.r1 - 1);
        else
            chi(i)  = c.kchi1*e(i) + c.kchi2*sign(e(i))*ae^c.r2;
            hder(i) = c.kchi1 + c.kchi2*c.r2*ae^(c.r2 - 1);
        end
    end
end

% ------------------------------------------------------------------------
function [eta, Leta, fcomp] = ftasc_update(a, eta, Leta, Du, Ts)
% Eq (37) + 自适应增益 (44)(45)
    damp = (a.k_eta1/a.rho1) * (tanh(abs(eta)).^(1 - a.rho1)) .* cosh(abs(eta)).^2 .* sign(eta) ...
         + a.k_eta2 * sig_pow(eta, a.rho2);

    use_adapt = isfield(a, 'adaptive') && a.adaptive;
    if use_adapt
        fcomp = Leta * damp;
    else
        Leta = 1;                       % 不启用 Eq (44) 时退化为式 (37)
        fcomp = damp;
    end
    eta = eta + Ts*(-fcomp + Du);

    if use_adapt
        if any(abs(Du) > 1e-12)
            Ldot = -a.kL(1)*Leta - a.kL(2)*sqrt(Leta) + a.kL(1)*a.PL;
        else
            Ldot = a.kL(3)*Leta^a.a(1) + a.kL(4)*Leta^(-a.a(2)) + a.cL;
            if Leta >= a.PU
                Ldot = 0;               % Proj(.)  (Eq 45)
            end
        end
        Leta = min(max(Leta + Ts*Ldot, a.PL), a.PU);
    end
end

% ------------------------------------------------------------------------
function [eta, fcomp] = ftac_update(a, eta, Du, Ts)
% Eq (46)/(75): 有限时间抗饱和补偿器 (文献 [29])
    fcomp = a.k_eta3*eta + a.k_eta4*sig_pow(eta, a.alpha);
    eta = eta + Ts*(-fcomp + Du);
end
