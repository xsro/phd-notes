%% run_final_paper_sim.m
% 自由漂浮空间机械臂 — 论文仿真
% 动力学: Jacobian 求和法 (已验证, 正定)
% 控制器: PD / PDF / PDF+PTDO / Terminal SMC
% =========================================================================
clear; clc; close all;

n = 7; Ts = 2e-3; Tf = 10; Tref = 8;
t = 0:Ts:Tf; N = numel(t);

% Figure dir
fig_dir = fullfile(fileparts(mfilename('fullpath')),'..','figures');
if ~exist(fig_dir,'dir'), mkdir(fig_dir); end

% Reference trajectory
q0=[0;pi/3;0;pi/4;pi/4;0;pi/6]; qf=[pi/18;pi/6;pi/5;0;-pi/4;-pi/3;pi/12];
[qd,dqd,ddqd]=deal(zeros(n,N));
for k=1:N
    s=min(t(k)/Tref,1); ph=10*s^3-15*s^4+6*s^5; dp=(30*s^2-60*s^3+30*s^4)/Tref; ddp=(60*s-180*s^2+120*s^3)/Tref^2;
    if t(k)>Tref, dp=0; ddp=0; end
    dq=qf-q0; qd(:,k)=q0+dq*ph; dqd(:,k)=dq*dp; ddqd(:,k)=dq*ddp;
end

% Parameters (Yan Table 2-1)
P.n=n; P.mass=[1000,4.25,7,7,4.25,4.25,4.25,4.25];
P.b=[0.6,0.6,1.5,1.5,0,0,0,0.3;0,0,0,0,-0.5,0,0,0;0,0,0.6,0.6,0,0.5,0.5,0];
P.a=[0,0.6,1.5,1.5,0,0,0,0.3;0,0,0,0,-0.5,0,0,0;0,0,0.6,0.6,0,0.5,0.5,0];
Ixx=[72,0.05,0.09,0.09,0.05,0.05,0.05,0.021]; Iyy=[72,1.28,1.46,1.46,0.89,0.89,0.89,0.53]; Izz=[72,1.28,1.46,1.46,0.89,0.89,0.89,0.53];
P.I=cell(1,8); for i=1:8, P.I{i}=diag([Ixx(i),Iyy(i),Izz(i)]); end
P.mi=P.mass(2:end); P.Ilink=P.I(2:end);

% Controller gains
Kp=diag([25,25,25,22,22,20,20]); Kd=diag([10,10,10,9,9,8,8]);
K0=[-Kp,-Kd]; h=1; Nh=round(h/Ts);

% PDF Gramian
A=[zeros(n),eye(n);zeros(n),zeros(n)]; B=[zeros(n);eye(n)]; Ac=A+B*K0;
sg=linspace(h,2*h,2001); W=zeros(2*n);
for kk=1:numel(sg)
    s=sg(kk); Rh=sin(pi*(s-h)/h)^4; E=expm(-Ac*s);
    W=W+(0.5+(kk>1&&kk<numel(sg))*0.5)*(E*B*(Rh*eye(n))*B.'*E.');
end
W=W*(sg(2)-sg(1)); W=0.5*(W+W.');
Wi=inv(W+1e-3*eye(2*n));

% Initial state
init_q = q0 + deg2rad([4;-3;3;-4;3;-2;2]);
init_dq = zeros(n,1);

% Cases
case_names = {'PD(no dist)','PDF(no dist)','PD(dist)','PDF(dist)','PDF+PTDO(dist)'};
use_dist = [false,false,true,true,true];
use_ptdo = [false,false,false,false,true];
nc = 5;

fprintf('=== Simulation: %d cases, %.0f s @ Ts=%.0e ===\n', nc, Tf, Ts);

S = cell(nc,1);  % results

tic;
for cid = 1:nc
    fprintf('Case %d/%d: %s ', cid, nc, case_names{cid});
    
    q=init_q; dq=init_dq; xb=zeros(6,1); dxb=zeros(6,1);
    zlog=zeros(2*n,N); taul=zeros(n,N); dah=zeros(n,N);
    z1=zeros(n,1); z2=zeros(n,1);
    
    [Hk,~,~,~]=compute_floating_base_inertia(P,q);
    Ck=compute_coriolis_force(P,q,dq);
    
    for k=1:N-1
        tk=t(k);
        e=q-qd(:,k); ed=dq-dqd(:,k);
        z=[e;ed]; zlog(:,k)=z;
        zd=zlog(:,max(k-Nh+1,1));
        
        Hb=Hk(1:6,1:6); Hbm=Hk(1:6,7:end); Hm=Hk(7:end,7:end);
        Cb=Ck(1:6); Cm=Ck(7:end);
        Me=Hm-Hbm'*(Hb\Hbm); Ce=Cm-Hbm'*(Hb\Cb);
        
        dt=use_dist(cid)*(0.15*sin(0.7*tk+0.3*(1:n)')+0.05*cos(1.3*tk+0.2*(1:n)'));
        
        if use_ptdo(cid)
            if tk<1, r=1-tk; xi=r^2*ones(n,1); xd=-2*r*ones(n,1); else xi=zeros(n,1); xd=zeros(n,1); end
            ep=dq-z1-xi; se=ep/0.4;
            p1=sign(se).*abs(se).^0.85+sign(se).*abs(se).^1.15;
            p2=sign(se).*abs(se).^1.7+sign(se).*abs(se).^2.3+0.4*sign(se);
            uo=Me\(taul(:,max(k,1))-Ce);
            z1=z1+Ts*(z2+(pi/0.3)*p1-xd+xi+uo);
            z2=z2+Ts*((pi/(0.12))*p2-xd);
            dah(:,k)=z2;
        end
        
        % PDF control
        th=tk-use_ptdo(cid)*1; if th<0, th=0; end
        thm=mod(th,2*h); Rh=0; if thm>=h, Rh=sin(pi*(thm-h)/h)^4; end
        Kcz=0; if Rh>0, Kcz=Rh*B.'*expm(-Ac.'*thm)*Wi*expm(Ac*(h-thm))*zd; end
        
        nu = K0*z - Kcz - use_ptdo(cid)*z2;
        taul(:,k) = Me*(ddqd(:,k)+nu)+Ce;
        
        ddqf=Hk\([zeros(6,1);taul(:,k)+dt]-Ck);
        dq=dq+Ts*ddqf(7:end); q=q+Ts*dq;
        dxb=dxb+Ts*ddqf(1:6); xb=xb+Ts*dxb;
        
        if k<N-1
            [Hk,~,~,~]=compute_floating_base_inertia(P,q);
            Ck=compute_coriolis_force(P,q,dq);
        end
    end
    
    % Final error
    ef=q-qd(:,N); edf=dq-dqd(:,N);
    S{cid}=struct('fe',norm(ef),'rms',sqrt(mean(ef.^2,'all')),'mt',max(abs(taul(:))),...
        'q',q,'dq',dq,'xb',xb,'zlog',zlog,'tau',taul,'dah',dah);
    fprintf(' |e(T)|=%.2e RMS=%.2e max|tau|=%.1f\n', S{cid}.fe, S{cid}.rms, S{cid}.mt);
end
fprintf('Time: %.1f s\n', toc);

%% Summary table
fprintf('\n========== RESULTS ==========\n');
fprintf('%-20s | %12s | %12s | %10s\n','Controller','|e(T)| [rad]','RMS(e) [rad]','max|tau|');
fprintf('%s\n',repmat('-',1,60));
for cid=1:nc
    r=S{cid}; fprintf('%-20s | %12.3e | %12.3e | %10.1f\n',case_names{cid},r.fe,r.rms,r.mt);
end

%% Momentum check
[Hc,~,~,~]=compute_floating_base_inertia(P,S{5}.q);
xbd5=-Hc(1:6,1:6)\(Hc(1:6,7:end)*S{5}.dq);
fprintf('\nMomentum violation: %.2e\n', norm(Hc*[xbd5;S{5}.dq]));

%% Figures
figure('Position',[50,50,1200,400]); cols=lines(nc);
subplot(1,3,1); hold on;
for cid=1:nc
    en=vecnorm(S{cid}.zlog(1:n,:),2,1);
    plot(t,en,'Color',cols(cid,:),'LineWidth',1);
end
set(gca,'YScale','log'); xlabel('t [s]'); ylabel('||e|| [rad]');
title('Position Error'); legend(case_names,'FontSize',6); grid on;

subplot(1,3,2); hold on;
for cid=1:nc
    edn=vecnorm(S{cid}.zlog(n+1:2*n,:),2,1);
    plot(t,edn,'Color',cols(cid,:),'LineWidth',1);
end
set(gca,'YScale','log'); xlabel('t [s]'); ylabel('||edot|| [rad/s]');
title('Velocity Error'); grid on;

subplot(1,3,3); hold on;
for cid=1:nc
    mt=max(abs(S{cid}.tau),[],1);
    plot(t,mt,'Color',cols(cid,:),'LineWidth',1);
end
xlabel('t [s]'); ylabel('max|\tau_i| [Nm]');
title('Control Torque'); grid on;

saveas(gcf,fullfile(fig_dir,'sim_overview.png'));
fprintf('Figure: sim_overview.png\n');

% Base reaction
figure;
r5=S{5};
plot3(r5.xb(1,:),r5.xb(2,:),r5.xb(3,:),'b-','LineWidth',1.5);
hold on; grid on;
plot3(r5.xb(1,1),r5.xb(2,1),r5.xb(3,1),'go','MarkerSize',8,'MarkerFaceColor','g');
plot3(r5.xb(1,end),r5.xb(2,end),r5.xb(3,end),'rs','MarkerSize',8,'MarkerFaceColor','r');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Base Reaction (PDF+PTDO)'); axis equal; view(45,30);
saveas(gcf,fullfile(fig_dir,'base_reaction.png'));

save(fullfile(fileparts(mfilename('fullpath')),'final_results.mat'),'S','case_names','t','qd');
fprintf('Done.\n');