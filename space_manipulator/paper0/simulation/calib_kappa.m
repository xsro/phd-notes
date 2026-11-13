%% calib_kappa.m
% 增益标定: 对 FONTSM+FTDOB+FTASC 单算例扫描 kappa, 选使饱和占空比
% 与 PDF 工况 (约 5%) 同量级的最大 kappa。
clear; clc;

cfg.Ts = 2e-3; cfg.Tf = 20; cfg.Tref = 8; cfg.n = 7; cfg.mem_len = 750;
cfg.t = 0:cfg.Ts:cfg.Tf;  cfg.N = numel(cfg.t);

P  = dou_yue_259_parameters();
CP = dou_yue_259_control_params();
n  = P.n;

q_ref0 = P.q0;
q_ref_f = P.q0 + deg2rad([20; -15; 15; -20; 15; -15; 10]);
s = min(cfg.t/cfg.Tref,1);
phi = 10*s.^3-15*s.^4+6*s.^5;
dphi = (30*s.^2-60*s.^3+30*s.^4)/cfg.Tref;
ddphi = (60*s-180*s.^2+120*s.^3)/cfg.Tref^2;
dphi(cfg.t>cfg.Tref)=0; ddphi(cfg.t>cfg.Tref)=0;
dl = q_ref_f - q_ref0;
qd = q_ref0 + dl*phi; dqd = dl*dphi; ddqd = dl*ddphi;

q0 = P.q0 + deg2rad([0.5;-0.4;0.4;-0.5;0.4;-0.3;0.2]);

for kappa = [0.005 0.01 0.02]
    q = q0; dq = zeros(n,1);
    C = fonsm259_init('fonsm', CP.fonsm, CP.ftdob, CP.ftasc, n, cfg.Ts, cfg.mem_len, kappa);
    C.dob.enabled = true;
    C.asc = CP.ftasc; C.asc.mode = 'ftasc'; C.asc.eta = zeros(n,1); C.asc.Leta = CP.ftasc.L0;
    C.asc.adaptive = false;
    tp = zeros(n,1); ta = zeros(n,1);
    en = zeros(1,cfg.N); mx = zeros(1,cfg.N); sat = false(1,cfg.N);
    tic;
    for k = 1:cfg.N-1
        S = ffsm259_model(P, q, dq);
        [tau, C] = fonsm259_step(C, q, dq, qd(:,k), dqd(:,k), ddqd(:,k), ...
            S.Me, S.Cmat, ta, tp);
        ta = min(max(tau, P.tau_min), P.tau_max);
        sat(k) = any(abs(tau) > P.tau_max);
        en(k) = norm(qd(:,k) - q);
        mx(k) = max(abs(tau));
        d = dou_yue_259_disturbance(cfg.t(k), n);
        ddq = S.Me \ (ta + d - S.Ce_qd);
        dq = dq + cfg.Ts*ddq;  q = q + cfg.Ts*dq;
        tp = tau;
    end
    fprintf('kappa=%.3f: |e(T)|=%.2e  RMS_ss=%.2e  max|tau|=%5.1f  sat=%5.1f%%  (%.0f s)\n', ...
        kappa, norm(qd(:,end)-q), sqrt(mean(en(cfg.t>=cfg.Tref).^2)), ...
        max(mx), 100*mean(sat), toc);
end
