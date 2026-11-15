function [st, d_hat, dg] = ptdo_step(st, y, u_zoh, u_s, t, Ts, obs)
% PTDO_STEP  分段预设时间动量观测器 (论文式 (eq:ptdo)), 一步更新
%
% 观测通道 (论文式 (eq:momentum)-(eq:momentum_channel)):
%
%   y = M_e(q_m) * dq_m,      y_dot = u_p + d(t),
%   u_p = tau_act - C_e(q_m,dq_m)*dq_m + Mdot_e(q_m,dq_m)*dq_m
%       = u_zoh + u_s(t)
%
% 其中 u_zoh = tau_act 在采样间隔内为零阶保持 (实际施加的力矩本来就是),
% u_s = -C_e*dq + Mdot_e*dq 是平滑项。被估计量直接为关节力矩扰动 d,
% 补偿量取 d_hat = z2 (式 (eq:hatd))。
%
% ------------------------------------------------------------------------
% 阶段 I:  0 <= t < T_o   (奇异增益, 定理 thm:obs1)
%
%   z1_dot = u_p + z2 + 6/(T_o-t) * (y - z1)
%   z2_dot =           6/(T_o-t)^2 * (y - z1)
%
% 阶段 II: t >= T_o       (超螺旋, 定理 thm:sta)
%
%   z1_dot = u_p + z2 + kappa1 .* sig^{1/2}(y - z1)
%   z2_dot =            kappa2 .* sgn(y - z1)
%
% T_o 处不重置状态: 阶段 II 直接以阶段 I 的左极限状态继续积分
% (论文第 3.1 节 (b) 的交接说明)。
% ------------------------------------------------------------------------
% 离散实现
%   阶段 I 的增益在 t -> T_o 时奇异, 显式 Euler 必然失稳。实现改在定理
%   证明所用的压缩时间坐标
%       rho = log(T_o/(T_o-t)),   s = T_o - t = T_o*exp(-rho)
%   下积分, 状态取 (z1, z2):
%       d z1 / d rho = -6*z1 + s*z2 + s*u_p + 6*y
%       d z2 / d rho = (6/s) * (y - z1)
%   两式右端在 rho -> inf 时不放大 (y - z1 = O(s^3), 故第二式的强迫为
%   O(s^2)), 用 rho 方向定步长 RK4 (步长 drho <= 0.05) 推进即可, 与物理
%   步长 Ts 解耦。步内 y 用一阶预测 y' = u_p + z2 外插 (含本拍已知的
%   u_zoh/u_s 与当前估计 z2), u_s 用差分斜率外插, u_zoh 零阶保持; 这样
%   强迫项的积分误差为 O(drho^2)。
%   采样实现中 s 不可能真的取到 0: 步内的 y 由采样值外插, 其误差会被
%   1/s 放大。故在 s < s_min (= 0.02 s, 可通过 obs.s_min 调整) 处把奇异
%   增益冻结为 6/s_min, 6/s_min^2 (即 s_eff = max(s,s_min))。冻结后的误差
%   系统仍线性稳定, 交接残差为 e2 ~ d_dot*s_min = O(1e-3) Nm, 由阶段 II
%   的超螺旋在有限时间内收掉; 这正是命题 prop:approx 中 Delta_r 的离散
%   分量。末步保留 s >= Ts/2。
%   阶段 II 无奇异项, 用显式 Euler 并在步内细分为 nsub2 个子步
%   (sig^{1/2} 与 sgn 均为 Filippov 集内的单值选取, sgn(0)=0)。细分不改变
%   算法, 只是把 sgn 的离散切换误差从 O(kappa2*Ts) 降到 O(kappa2*Ts/nsub2),
%   对应命题 prop:approx 中 Delta_r 的离散分量。
%
% 输入
%   st     观测器状态 (z1, z2, y_prev, u_s_prev)
%   y      动量通道量测 M_e(q_m)*dq_m [n x 1]
%   u_zoh  零阶保持输入 tau_act [n x 1]
%   u_s    平滑输入 -C_e*dq + Mdot_e*dq (当前采样点) [n x 1]
%   t      当前时间 [s]
%   Ts     采样步长 [s]
%   obs    结构体: .To, .kappa1, .kappa2, .nsub2 (可选)
%
% 输出
%   st     更新到 t+Ts 的状态
%   d_hat  扰动估计 z2(t+Ts) [n x 1] (力矩单位)
%   dg     诊断: .phase (1/2), .e1 (y-z1), .drho

    n = numel(y);
    k1 = expand(obs.kappa1, n);
    k2 = expand(obs.kappa2, n);
    if ~isfield(st, 'y_prev') || isempty(st.y_prev),   st.y_prev   = y;     end
    if ~isfield(st, 'u_s_prev') || isempty(st.u_s_prev), st.u_s_prev = u_s; end

    % 步内 y 的一阶预测: y' = u_p + d ~ u_zoh + u_s + z2 (用本拍已知输入与
    % 当前估计, 而非过去两拍的差分)。这样在力矩发生跳变时预测斜率仍正确,
    % 把步内预测误差降为 O(Ts^2)。
    vy = u_zoh + u_s + st.z2;
    vu = (u_s - st.u_s_prev)/Ts;      % 平滑输入仍用差分斜率 (量级很小)

    if t < obs.To
        % ---------------- 阶段 I: 压缩时间 RK4 ----------------
        s0 = obs.To - t;
        s1 = obs.To - (t + Ts);
        s1 = max(s1, 0.5*s0);
        drho_tot = log(s0/s1);
        nsub = max(1, ceil(drho_tot/0.05));
        drho = drho_tot/nsub;

        s_min = 0.02;                     % 奇异增益正则化: s_eff = max(s, s_min)
        if isfield(obs, 's_min') && ~isempty(obs.s_min), s_min = obs.s_min; end

        pi1 = st.z1;
        pz2 = st.z2;
        rho = 0;
        for j = 1:nsub
            [F1, F2] = f_rho(rho,        pi1,           pz2,           s0, s_min, y, vy, u_zoh, u_s, vu);
            [G1, G2] = f_rho(rho+drho/2, pi1+drho/2*F1, pz2+drho/2*F2, s0, s_min, y, vy, u_zoh, u_s, vu);
            [H1, H2] = f_rho(rho+drho/2, pi1+drho/2*G1, pz2+drho/2*G2, s0, s_min, y, vy, u_zoh, u_s, vu);
            [K1, K2] = f_rho(rho+drho,   pi1+drho*H1,   pz2+drho*H2,   s0, s_min, y, vy, u_zoh, u_s, vu);
            pi1 = pi1 + drho/6*(F1 + 2*G1 + 2*H1 + K1);
            pz2 = pz2 + drho/6*(F2 + 2*G2 + 2*H2 + K2);
            rho = rho + drho;
        end
        st.z1 = pi1;
        st.z2 = pz2;
        dg = struct('phase', 1, 'e1', y - st.z1, 'drho', drho_tot);
    else
        % ---------------- 阶段 II: 超螺旋 ----------------
        nsub2 = 5;
        if isfield(obs, 'nsub2') && ~isempty(obs.nsub2), nsub2 = obs.nsub2; end
        hh = Ts/nsub2;
        for j = 1:nsub2
            tau_j = (j-1)*hh;
            yj = y   + vy*tau_j;
            uj = u_zoh + u_s + vu*tau_j;
            e1 = yj - st.z1;                  % 用更新前的 z1 (显式 Euler)
            sq = sign(e1) .* sqrt(abs(e1));   % sig^{1/2}(e1)
            st.z1 = st.z1 + hh*(uj + st.z2 + k1.*sq);
            st.z2 = st.z2 + hh*(k2.*sign(e1));
        end
        dg = struct('phase', 2, 'e1', y - st.z1, 'drho', 0);
    end

    st.y_prev   = y;
    st.u_s_prev = u_s;
    d_hat = st.z2;
end

% ------------------------------------------------------------------------
function [dz1, dz2] = f_rho(rho, z1, z2, s0, s_min, y0, vy, u_zoh, u_s, vu)
% 压缩时间坐标下的右端 (s = T_o*exp(-rho)), 奇异增益在 s < s_min 处冻结
%   s_eff = max(s, s_min)  ->  物理时间增益 6/s_eff, 6/s_eff^2
%   步内插值: y = y0 + vy*(s0-s),  u_s = u_s0 + vu*(s0-s)
    s     = s0*exp(-rho);
    s_eff = max(s, s_min);
    dt = s0 - s;
    yy = y0   + vy*dt;
    uu = u_zoh + u_s + vu*dt;
    dz1 = -6*(s/s_eff)*z1 + s*z2 + s*uu + 6*(s/s_eff)*yy;
    dz2 = (6*s/s_eff^2) * (yy - z1);
end

% ------------------------------------------------------------------------
function v = expand(v, n)
    if isscalar(v), v = v*ones(n, 1); end
    v = v(:);
end
