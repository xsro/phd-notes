close all

%% 参数定义
params=struct();
params.target=[3;10];        %目标位置
agents=[0 0 5 5 10 10;0 5 0 5 0 5];    %智能体位置
params.max_distance=3;       %邻居距离
params.min_distance=1;       %避障距离
params.delta=deg2rad(10);    %共线容许最大角度
params.epsilon=0.05;         %旋转项幅值
params.k=1;                  %目标吸引增益
params.N=size(agents,2);     %智能体个数
G=graph(1:6,[2:6 1]);
Bvalue=[0 1 1 0 0 0
0 1 1 0 1 0
0 1 1 0 0 1
0 1 1 0 1 1];
for Bi=1:4
    B=Bvalue(Bi,:);
params.M=diag(B)+laplacian(G);

initial=reshape(agents,[],1);      % 闭合多智能体系统所有状态
[dxdt0,s0]=rhs(0,initial,params);  % 

%% 运行仿真
opt=odeset("OutputSel",[1,3,5,7],"OutputFcn","odeplot");
[t,x]=ode45(@(t,x)rhs(t,x,params),[0,100],...
    reshape(initial,[],1),opt);



%%
figure();
hold on;
for i=1:6
    p=plot(x(:,2*i-1),x(:,2*i));
    if B(i)==1
    plot(x(end,2*i-1),x(end,2*i),"o","MarkerSize",14,"Color",p.Color,"MarkerFaceColor",p.Color);
    else
    plot(x(end,2*i-1),x(end,2*i),"o","MarkerSize",14,"Color",p.Color);
    end
    text(x(end,2*i-1),x(end,2*i),num2str(i),"HorizontalAlignment","center")
end
plot(3,10,"s","MarkerSize",16,"Color","black","MarkerFaceColor","black");
axis equal
filename="b";
for i=1:length(B)
    if B(i)==1
        filename=filename+num2str(i);
    end
end
exportgraphics(gca(),"out/"+filename+".pdf")
end






