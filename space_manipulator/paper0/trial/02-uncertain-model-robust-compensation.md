# 方向二：名义模型下的鲁棒动力学补偿

## 1. 目标

当前控制器默认可以准确获得：

\[
M_e(q),\qquad C_e(q,\dot q).
\]

实际空间机械臂中，惯性、质心、连杆参数和基座状态均可能存在不确定性。因此考虑名义模型：

\[
\hat M_e(q),\qquad \hat C_e(q,\dot q).
\]

目标是在不依赖精确动力学模型的前提下，保持 PDF 对误差通道的主要作用，并抑制模型误差。

## 2. 名义计算力矩控制器

定义：

\[
v=\ddot q_d+K_0z-K_c(t)z(t-h).
\]

采用名义计算力矩形式：

\[
\tau=\hat M_e(q)v+
\hat C_e(q,\dot q)\dot q-
\hat d+\tau_r.
\]

其中 \(\tau_r\) 为鲁棒补偿项。

## 3. 模型误差展开

令：

\[
M_e=\hat M_e+\Delta M_e,
\qquad
C_e=\hat C_e+\Delta C_e.
\]

代入真实动力学：

\[
M_e\ddot q+C_e\dot q=\tau+d.
\]

可得到：

\[
\ddot e=
\nu+M_e^{-1}
\left[
(d-\hat d)-\Delta M_e v-\Delta C_e\dot q+\tau_r
\right].
\]

因此模型误差可以被统一映射为残余加速度扰动：

\[
\Delta_r=M_e^{-1}
\left[
(d-\hat d)-\Delta M_e v-\Delta C_e\dot q+\tau_r
\right].
\]

## 4. 鲁棒项候选设计

定义滑模型辅助变量：

\[
s=\dot e+\Lambda e,
\qquad \Lambda=\Lambda^\top>0.
\]

可采用平滑饱和项：

\[
\tau_r=-k_r\operatorname{sat}\left(\frac{s}{\varphi}\right),
\]

其中 \(k_r>0\) 为鲁棒增益，\(\varphi>0\) 为边界层宽度。

也可以使用连续高阶项：

\[
\tau_r=-k_1\operatorname{sat}(s/\varphi)
-k_2\operatorname{sat}(\dot s/\varphi_2).
\]

但后者需要额外状态或加速度估计，暂不推荐作为第一版实现。

## 5. 与 PDF 理论的关系

加入 \(\tau_r\) 后，闭环不再是纯线性延迟系统，因此不能直接沿用：

\[
\Delta_K(h)=0
\]

推出的精确零化结论。

合理的理论结构是：

1. PDF 项负责实现理想误差通道的预设时间压缩；
2. 鲁棒项抑制模型误差和观测误差；
3. 最终建立预设时间实际有界性：
   \[
   \|z(t)\|\le \varepsilon_r,
   \qquad t\ge T_o+2h.
   \]

## 6. 可能的理论假设

假设存在已知或未知上界：

\[
\|\Delta M_e(q)\|\le \bar M,
\]

\[
\|\Delta C_e(q,\dot q)\|\le \bar C_0+\bar C_1\|\dot q\|,
\]

\[
\|d-\hat d\|\le \bar d.
\]

若 \(v\) 和 \(\dot q\) 有界，则：

\[
\|\Delta_r\|
\le
\|M_e^{-1}\|
\left(
\bar d+\bar M\|v\|+
(\bar C_0+\bar C_1\|\dot q\|)\|\dot q\|
+\|\tau_r\|
\right).
\]

需要注意，若鲁棒项是用于补偿误差，其上界不能简单地放进右端而不作进一步分析。应通过 Lyapunov 不等式证明它能够降低总残余项。

## 7. 推荐的论文定位

该方向适合形成如下贡献：

> 在仅获得名义自由漂浮动力学模型的条件下，将 PDF 预设时间反馈与平滑鲁棒补偿相结合，并建立模型误差、观测误差和周期延迟反馈共同作用下的实际跟踪界。

## 8. 优点

- 更贴合空间机械臂真实应用；
- 能自然解释模型近似和参数不确定性；
- 可在现有控制器上增加少量内容；
- 便于通过参数摄动仿真验证。

## 9. 风险

1. 鲁棒项会破坏原有线性 PDF 闭环结构；
2. 需要重新组织稳定性证明；
3. 饱和函数边界层会带来稳态误差；
4. 若使用滑模符号函数，可能与“控制输入光滑”这一卖点冲突。

## 10. 推荐选择

优先使用连续饱和函数，而不是不连续符号函数：

\[
\operatorname{sat}(x)=
\begin{cases}
 x,&|x|\le1,\\
 \operatorname{sgn}(x),&|x|>1.
\end{cases}
\]

如果论文强调“光滑控制输入”，可进一步改用：

\[
\tanh(s/\varphi)
\]

但需要重新给出扰动抑制性能界。
