%% tune_ptdo.m  扫描 PTDO 参数 (T_c, eta, sigma) 对抖振与跟踪的影响
clear; clc;
cfg.Ts = 2e-3; cfg.Tf = 20; cfg.Tref = 8; cfg.n = 7;
cfg.t = 0:cfg.Ts:cfg.Tf; cfg.N = numel(cfg.t);

here = fileparts(mfilename('fullpath'));
load(fullfile(here, 'results_dou_yue_259.mat'), 'ref', 'P', 'ctrl');
n = P.n;
q0 = P.q0 + deg2rad([0.5; -0.4; 0.4; -0.5; 0.4; -0.3; 0.2]);

sets = [1.0 0.3 0.4; 3.0 0.3 0.4; 5.0 0.3 0.4; 3.0 0.1 0.4; 5.0 0.5 0.8];
for s = 1:size(sets,1)
    obs = struct('T_o', 1.0, 'T_c', sets(s,1), 'eta', sets(s,2), ...
                 'sigma', sets(s,3), 'xi0', zeros(n,1));
    q = q0; dq = zeros(n,1);
    Nz = round(ctrl.h/cfg.Ts) + 1;
    z_hist = zeros(2*n, Nz);
    ost = struct('z1', zeros(n,1), 'z2', zeros(n,1));
    ta = zeros(n,1);
    en = zeros(1,cfg.N); mxt = zeros(1,cfg.N); sat = false(1,cfg.N);
    dah = zeros(n,cfg.N); dda = zeros(n,cfg.N);

    for k = 1:cfg.N-1
        S = ffsm259_model(P, q, dq);
        z = [q - ref.qd(:,k); dq - ref.dqd(:,k)];
        z_hist = [z, z_hist(:,1:end-1)];
        nu = ctrl.K0*z - pdf_gain_at(ctrl.pdf, cfg.t(k))*z_hist(:,end);
        u_o = S.Me \ (ta - S.Cmat*dq);
        [ost, dah_k] = ptdo_step(ost, dq, u_o, cfg.t(k), cfg.Ts, obs);
        tau = S.Cmat*dq + S.Me*(ref.ddqd(:,k) + nu - dah_k);
        ta = min(max(tau, -P.tau_max), P.tau_max);
        sat(k) = any(abs(tau) > P.tau_max);

        d = dou_yue_259_disturbance(cfg.t(k), n);
        dda(:,k) = S.Me \ d;
        dah(:,k) = dah_k;

        en(k)  = norm(ref.qd(:,k) - q);
        mxt(k) = max(abs(tau));
        ddq = S.Me \ (ta + d - S.Ce_qd);
        dq = dq + cfg.Ts*ddq;
        q  = q  + cfg.Ts*dq;
    end
    ss = cfg.t >= cfg.Tref;
    e_obs = dda - dah;
    fprintf(['T_c=%.1f eta=%.1f sig=%.1f | |e(T)|=%.2e RMS_ss=%.2e ' ...
             'max|tau|=%5.1f sat=%5.1f%% | obs RMS=%.4f max=%.3f\n'], ...
        sets(s,1), sets(s,2), sets(s,3), norm(ref.qd(:,end)-q), ...
        sqrt(mean(en(ss).^2)), max(mxt), 100*mean(sat), ...
        sqrt(mean(e_obs(:,ss).^2, 'all')), max(vecnorm(e_obs(:,ss), 2, 1)));
end
