# 方向一：残余扰动下的预设时间实际有界性理论

## 1. 目标

将当前论文的理想结论

\[
z(t)=0,\qquad t\ge T_o+2h
\]

扩展为存在观测误差、模型不确定性、执行器饱和和数值实现误差时的定量结果：

\[
\|z(t)\|\le \Gamma_h\|\Delta_r\|_\infty,
\qquad t\ge T_o+2h.
\]

该方向最适合作为当前论文的主创新增强项。

## 2. 统一残余扰动模型

采用名义模型控制器后，将所有非理想因素统一写成等效加速度扰动：

\[
\Delta_r(t)=M_e^{-1}(q_m)
\left[d(t)-\hat d(t)+\Delta_m(t)+\Delta_\tau(t)\right],
\]

其中：

- \(d-\hat d\)：扰动观测误差；
- \(\Delta_m\)：模型不确定性和未建模动力学；
- \(\Delta_\tau\)：执行器饱和、量化和输入实现误差。

PDF 闭环变为：

\[
\dot z(t)=A_cz(t)-BK_c(t)z(t-h)+B\Delta_r(t).
\]

假设：

\[
\|\Delta_r(t)\|\le \bar\Delta_r.
\]

## 3. 单周期受扰动状态映射

对于局部 PDF 时间 \(\vartheta\in[0,2h]\)，写出状态转移表达式：

\[
z(2h)=\Delta_K(h)z(0)+
\int_0^{2h}\Psi(2h,s)B\Delta_r(s)\,\mathrm ds.
\]

理想 Gramian 型 PDF 增益满足：

\[
\Delta_K(h)=0.
\]

因此：

\[
\|z(2h)\|
\le
\int_0^{2h}\|\Psi(2h,s)B\|\,\mathrm ds\,\bar\Delta_r.
\]

定义：

\[
\Gamma_h=
\int_0^{2h}\|\Psi(2h,s)B\|\,\mathrm ds,
\]

得到：

\[
\|z(2h)\|\le \Gamma_h\bar\Delta_r.
\]

更严格的做法是分别计算周期内的状态响应上界：

\[
\Gamma_h^{\mathrm{max}}
=
\max_{\theta\in[0,2h]}
\int_0^{\theta}
\|\Psi(\theta,s)B\|\,\mathrm ds,
\]

从而给出：

\[
\|z(t)\|
\le \Gamma_h^{\mathrm{max}}\bar\Delta_r,
\qquad t\ge T_o+2h.
\]

## 4. 需要证明的定理形式

**定理候选：** 若 \((A,B)\) 可控，\(K_c(t)\) 按 Gramian 型 PDF 公式设计，且残余扰动满足 \(\|\Delta_r\|_\infty\le\bar\Delta_r\)，则在 \(T_o+2h\) 后闭环误差进入有界集合：

\[
\mathcal E_h=
\left\{z:\|z\|\le \Gamma_h^{\mathrm{max}}\bar\Delta_r\right\}.
\]

若 \(\Delta_r=0\)，则退化为：

\[
\mathcal E_h=\{0\}.
\]

## 5. 可进一步增强的结果

### 5.1 近似 PDF 零化

实际数值计算中可能只有：

\[
\|\Delta_K(h)\|\le \varepsilon_\Delta.
\]

则周期递推满足：

\[
\|z_{k+1}\|
\le
\varepsilon_\Delta\|z_k\|+
\Gamma_h\bar\Delta_r.
\]

若 \(\varepsilon_\Delta<1\)，则：

\[
\limsup_{k\to\infty}\|z_k\|
\le
\frac{\Gamma_h\bar\Delta_r}{1-\varepsilon_\Delta}.
\]

这可以直接描述 Gramian 数值误差和离散 PDF 实现的影响。

### 5.2 观测器误差显式进入上界

若：

\[
\|d(t)-\hat d(t)\|\le \bar d_o,
\]

且：

\[
\|M_e^{-1}(q)\|\le \bar m^{-1},
\]

则可得到：

\[
\bar\Delta_r
\le
\bar m^{-1}
\left(\bar d_o+\bar\Delta_m+\bar\Delta_\tau\right).
\]

最终误差界为：

\[
\|z(t)\|
\le
\Gamma_h\bar m^{-1}
\left(\bar d_o+\bar\Delta_m+\bar\Delta_\tau\right).
\]

## 6. 论文中的创新表述

可表述为：

> 针对光滑周期延迟反馈在实际空间机械臂中受到观测误差、模型不确定性和执行器非理想性的影响，建立统一残余加速度扰动模型，并基于单周期状态映射推导预设时间后的显式跟踪误差界，从而将理想单周期精确零化结论推广为非理想条件下的预设时间实际有界性结论。

## 7. 所需仿真

暂不修改现有仿真，后续建议增加：

- 扰动观测误差幅值扫描；
- 模型惯性参数摄动；
- 扭矩饱和；
- \(\Delta_K(h)\) 人为扰动；
- 理论误差界与实际误差对比。

## 8. 风险

1. 需要准确写出受扰动延迟系统的状态转移矩阵；
2. “进入邻域”需要区分单周期时刻和周期内任意时刻；
3. 若闭环单周期映射并非严格零化，而只是指数稳定，需要使用递推界；
4. 该结果更适合称为“预设时间实际有界性”，不应继续称为严格预设时间精确跟踪。
