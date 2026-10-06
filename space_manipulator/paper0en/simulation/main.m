%% 1. 论文给定参数
clear; clc; close all
mu1 = 1.2;    mu2 = 1.0;
a1  = 3.0;    a2  = 5.0;
rho = a2/a1;
gamma = 1e-3;
d_bar = 0.5;
d_n = d_bar / a1;
c_xi = 0.1;

%% 2. 定义符号幂、phi1、phi2函数
signPow = @(s,alpha) abs(s).^alpha .* sign(s);

phi1 = @(s) mu1 * signPow(s, 1/2) + mu2 * s;

phi2Fun = @(s,mu1,mu2) (mu1^2/2)*sign(s) + (3*mu1*mu2/2)*signPow(s,1/2) + mu2^2 * s;
phi2 = @(s) phi2Fun(s,mu1,mu2);

%% 3. Theta函数
Theta = @(z,xi,delta) (-(z-xi)+gamma*xi).*(-rho*phi2(z)+delta) + c_xi*xi.*phi2(xi);

%% 4. 网格采样，缩小全局区间，加密对角线附近采样
range = linspace(-3,3,300);  % 缩小范围，聚焦动态区间
[Z,XI] = meshgrid(range,range);
maxRatio = 0;
deltaList = linspace(-d_n, d_n, 2000);
ratioMat = zeros(size(Z));

%% 5. 遍历扰动求最大比值场
for dIdx = 1:length(deltaList)
    delta = deltaList(dIdx);
    ThetaVal = Theta(Z,XI,delta);
    ThetaPlus = max(ThetaVal, 0);
    
    diffPhi = phi1(Z) - phi1(XI);
    denom = diffPhi .^ 2;
    
    mask = denom > 1e-8;
    ratio = zeros(size(Z));
    ratio(mask) = ThetaPlus(mask) ./ denom(mask);
    
    currentMax = max(ratio(:));
    if currentMax > maxRatio
        maxRatio = currentMax;
    end
    % 保存所有δ下的最大值包络（最坏情况）
    ratioMat = max(ratioMat, ratio);
end

%% 输出Cδ数值
fprintf('网格遍历得到C_δ近似上界 ≈ %.4f\n', maxRatio);

%% ========== 绘图1：3D曲面图（不变，无报错） ==========
figure('Color','w');
surf(Z,XI,ratioMat,'EdgeColor','none');
shading interp; colorbar;
xlabel('z'); ylabel('\xi'); zlabel('[\Theta]_+ / (\phi_1(z)-\phi_1(\xi))^2');
title('C_\delta 比值3D热力曲面 (z,\xi\in[-3,3])');
view(3); grid on;

%% ========== 绘图2：替换contourf为pcolor热力平面图（解决常量报错） ==========
figure('Color','w');
pcolor(Z,XI,ratioMat);
shading interp; colorbar;
xlabel('z'); ylabel('\xi');
title('C_\delta 比值平面热力图（pcolor无等高线报错）');

%% ========== 绘图3：对角线z≈xi局部放大，单独看峰值区域 ==========
% 截取 z,xi ∈ [-0.5,0.5] 对角线邻域（峰值集中区）
rangeSmall = linspace(-0.5,0.5,400);
[Zs,XIs] = meshgrid(rangeSmall,rangeSmall);
ratioSmall = zeros(size(Zs));
for dIdx = 1:length(deltaList)
    delta = deltaList(dIdx);
    ThetaVal = Theta(Zs,XIs,delta);
    ThetaPlus = max(ThetaVal, 0);
    diffPhi = phi1(Zs) - phi1(XIs);
    denom = diffPhi .^ 2;
    mask = denom > 1e-8;
    rTmp = zeros(size(Zs));
    rTmp(mask) = ThetaPlus(mask) ./ denom(mask);
    ratioSmall = max(ratioSmall, rTmp);
end

figure('Color','w');
pcolor(Zs,XIs,ratioSmall);
shading interp; colorbar;
xlabel('z'); ylabel('\xi');
title('对角线邻域 z,\xi\in[-0.5,0.5] 比值放大热力图');