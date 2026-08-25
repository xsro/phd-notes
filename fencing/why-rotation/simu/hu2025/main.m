%% 参数定义
params=struct();
target=[2;8;0.5;0.5];         %目标位置
agents=[-7 7 7 -7;-7 -7 7 7]; %智能体位置
params.max_distance=10;       %邻居距离
params.min_distance=2;        %避障距离
params.s1=-0.1;
params.k=[2.2 6 0.1 3 20];    %控制器增益
params.N=size(agents,2);      %智能体个数

results=cell(2,1);

str="";
for i=1:4
    str=str+"$p_"+i+"=["+agents(1,i)+","+agents(2,i)+"]$\n";
end

if ~exist("out","dir")
    mkdir out
end

%% 运行仿真
for ip=1:2
    if ip==2
        % special case when the formation is not rigid
        agents=[-7 7 7 -7;-7 -7 7.2 7.1]; 
    end
    initial=reshape([agents;zeros(6,params.N)],[],1);      % 闭合多智能体系统所有状态
    [dxdt0,s0]=rhs(0,[target;initial],params);  % 
    idx=reshape(5:4+length(initial),[],params.N);
    
    opt=odeset("OutputSel",idx(1,:),"OutputFcn","odeplot");
    [t,x]=ode45(@(t,x)rhs(t,x,params),0:0.1:250,...
        [target;initial],opt);

    s=repmat(s0,length(t),1);
    for it=1:length(t)
        [dxdti,si]=rhs(t(it),x(it,:)',params);
        s(it)=si;
    end
    results{ip}=struct("t",t,"x",x,"s",s);

    %% 绘图集群中心收敛到目标的指标图
    figure();
    tiledlayout(2,1)
    nexttile;
    data=vecnorm(horzcat(s.debug_p1),2);
    plot(t,data)
    xlim([0 80])
    ylabel("$\Vert\bar{x}-x_d \Vert$","Interpreter","latex");
    nexttile;
    vx=sum(x(:,7:8:end),2)/4-x(:,3);
    vy=sum(x(:,8:8:end),2)/4-x(:,4);
    vn=vecnorm([vx vy],2,2);
    plot(t,vn);
    ylabel("$\Vert\bar{v}-v_d \Vert$","Interpreter","latex");
    xlim([0 80])
    xlabel("time/s")
    exportgraphics(gcf,"out/"+ip+"p1.pdf")
    
    %% 绘制智能体速度相对半径的夹角图
    vi0parallel=zeros(4,length(t));
    vi0perpendicular=zeros(4,length(t));
    for ia=1:4
        for it=1:length(t)
            pi=x(it,5+8*(ia-1):6+8*(ia-1));
            p0=x(it,1:2);
            vi=x(it,7+8*(ia-1):8+8*(ia-1));
            v0=x(it,3:4);
            pi0=pi-p0;
            vi0=vi-v0;
            vi0parallel(ia,it)=dot(vi0,pi0)/norm(pi0);
            vi0perpendicular(ia,it)=sqrt(vi0*vi0'-vi0parallel(ia,it)^2);
        end
    end
    figure();
    subplot(2,1,1)
    hold on;
    plot(t,vi0parallel(1,:))
    plot(t,vi0parallel(2,:))
    plot(t,vi0parallel(3,:))
    plot(t,vi0parallel(4,:))
    ylabel("Parallel velocity")
    grid on
    subplot(2,1,2)
    hold on;
    plot(t,vi0perpendicular(1,:))
    plot(t,vi0perpendicular(2,:))
    plot(t,vi0perpendicular(3,:))
    plot(t,vi0perpendicular(4,:))
    ylabel("Perpendicular velocity")
    grid on
    exportgraphics(gcf,"out/"+ip+"pv.pdf")

    %% 绘制运动轨迹图
    render(t, x, params, ip, "out")
end












