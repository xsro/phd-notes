% implemented by xsro.github.io
%  Li Wei Kou, Zhi Yong Chen, and Ji Xiang. Cooperative fencing control of multiple vehicles for a moving target with an
% unknown velocity. IEEE Transactions on Automatic Control, 67(2):1008–1015, February 2022


params=struct();
params.N=6;
agents=zeros(4,params.N);
for i=1:params.N
    agents(1:2,i)=[10*cos((i-1)*pi/3);
    10*sin((i-1)*pi/3)];
end
target=[0;20];
params.targetv=[3;1];

params.k1=0.5;
params.k2=0.5;

params.d=5;
params.mu=9;

params.initial=reshape([agents(:);target],[],1);
params.idx=reshape(1:4*params.N,4,params.N);

[dxdt0,s0]=rhs(0,params.initial,params);
opt=odeset("RelTol",1e-4,"AbsTol",1e-4);
[t,x]=ode45(@(t,x)rhs(t,x,params),0:0.01:60,params.initial,opt);
s=repmat(s0,[1,length(t)]);
dxdt=repmat(dxdt0',[length(t),1]);
for it=1:length(t)
    [dxdti,s(it)]=rhs(t(it),x(it,:)',params);
    dxdt(it,:)=dxdti';
end
p=params;
p.t=t;p.x=x;p.s=s;
p.dxdt=dxdt;

%%
figure;
tiledlayout(3,1)
nexttile
y=horzcat(p.s.debug_p1);
plot(p.t,y);
grid on;
ylabel("$\|\bar{x}-x_0\|$","Interpreter","latex");
nexttile
y=horzcat(p.s.debug_pv);
plot(p.t,y);
grid on;
ylabel("$\|\bar{v}-v_0\|$","Interpreter","latex");
exportgraphics(gcf,sprintf("%d-p1.png",p.N),"Resolution",300)

nexttile;hold on
for ia=1:p.N
    for ja=1:p.N
        if ja==ia
            break
        end
        xij=horzcat(p.x(:,p.idx(1:2,ia)))-horzcat(p.x(:,p.idx(1:2,ja)));
        xijn=vecnorm(xij,2,2);
        plot(p.t,xijn);
    end
end
ylabel("$\|x_{ij}\|$","Interpreter","latex");

%%
figure;
tiledlayout(3,1)
nexttile;hold on
for ia=1:p.N
    x10=horzcat(p.x(:,p.idx(1:2,ia)))-p.x(:,end-1:end);
    x10n=vecnorm(x10,2,2);
    plot(p.t,x10n);
end
grid on;
ylabel("$\|x_{i0}\|$","Interpreter","latex");
nexttile
y=horzcat(p.s.debug_pvn);
plot(p.t,y);
ylabel("$\|\dot{x}_{i0}\|$","Interpreter","latex");
ylim([0 10])
nexttile
y=horzcat(p.s.debug_pv2);
plot(p.t,y);
grid on;
ylabel("$\langle \dot{x}_{i0}/\|x_{i0}\|,R x_i0/\|x_{i0}\|\rangle$","Interpreter","latex");
ylim([-10 10])
grid on;
exportgraphics(gcf,sprintf("%d-p2.png",p.N),"Resolution",300)

%% 
figure();
y=horzcat(p.s.debug_pi0phi);
ysum=sum(y,1);
ylabel("phi's component along Rpi0")
plot(p.t,ysum);




