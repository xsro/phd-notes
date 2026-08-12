# 量化控制理论：数学模型、根本极限与常用方法

> A Survey on Quantized Control: Mathematical Formulation, Fundamental Limits and Common Methods
>
> 调研基于控制理论文献知识库（okb-assist / MCP 工具），参考文献见文末。

---

## 摘要

网络化控制系统（Networked Control Systems, NCS）中，传感器与控制器、控制器与执行器之间往往经由数字信道传递信息。由于通信带宽、存储与能耗受限，信号必须以**有限字长**（有限比特率）进行量化与编解码。量化控制研究的核心问题是：在“反馈信息率”存在根本约束的前提下，系统能否被镇定？若可以，需要多大的数据率，以及应采用何种“编码—控制”协同策略？

本报告以数学语言给出量化控制的统一建模框架，介绍均匀量化器、对数量化器与动态“变焦（zooming）”量化器等信号模型，阐述以**数据率定理（Data Rate Theorem）**为代表的根本性能极限，并系统梳理静态对数镇定、动态变焦、混合反馈、有限/固定时间齐次量化、事件触发与规定时间控制、滑模控制以及多智能体分布式量化等主流方法，最后结合近年文献指出研究趋势。

---

## 1. 引言

经典控制理论的反馈回路被假定为连续、无限精度。然而在工程实践中，信号需经模数转换、编码与有限带宽信道传输，因而必然被**量化（quantization）**——即用有限个离散符号近似连续值。这一限制带来两个本质困难：

- **(i) 信息损失**：量化引入有界（或随尺度变化的）误差，破坏经典 Lyapunov 镇定分析所依赖的“控制律连续依赖状态”的假设；
- **(ii) 率—性能折衷**：信道数据率（bits/sample 或 bits/s）决定了可区分状态的分辨率，进而决定了可镇定的动态范围。

量化控制将**控制**与**信源/信道编码**耦合在同一反馈回路中，其根本问题可概括为（参见 *Encyclopedia of Systems and Control* 词条 “Quantized Control and Data Rate Constraints” [1]）：

> 在一个数据率受限的数字信道上，稳定（或达成某一控制目标）一个线性时不变（LTI）对象所需的最小信息率是多少？对应的编解码—控制闭环应如何设计？

---

## 2. 问题建模

考虑离散时间 LTI 对象（Nair 等人的标准设定 [2, 1]）：

$$
\begin{aligned}
x_{k+1} &= A\,x_k + B\,u_k + F\,v_k, \qquad &&\text{(状态方程)} \\
y_k     &= C\,x_k + w_k,                 \qquad &&\text{(测量方程)}
\end{aligned}
\tag{1}
$$

其中 $k\in\mathbb{Z}_{\ge 0}$ 为时间步，$x_k\in\mathbb{R}^n$ 为状态（初值 $x_0$ 未知），$u_k\in\mathbb{R}^m$ 为控制输入，$y_k\in\mathbb{R}^p$ 为测量输出，$v_k\in\mathbb{R}^n$、$w_k\in\mathbb{R}^p$ 分别为未知过程噪声与测量噪声，$A,B,C,F$ 为已知常矩阵。

**量化反馈回路**在传感器端与执行器端分别插入量化/编码环节。记量化器（更一般地，编码器）为一个将连续信号映射到有限字母表 $\mathcal{S}=\{s_1,\dots,s_M\}$ 的映射：

$$
q:\;\mathbb{R}^d \longrightarrow \mathcal{S},\qquad |\mathcal{S}| = M = 2^R,
\tag{2}
$$

其中 $R$ 为每步可用比特数（数据率）。符号流经数字信道传输，解码器据历史符号重建状态/控制量。受量化影响的信号可以是**状态/输出**（状态反馈量化）、**控制输入**（输入量化）或二者兼具。

闭环结构可抽象为

```
Plant --y_k--> Encoder/Quantizer q(·) --channel--> Decoder/Controller κ(·) --u_k--> Plant
                  ↑___状态估计/反馈___________________|
```

**定义（可镇定性与最小数据率）。** 称系统在数据率 $R$ 下*可强镇定*（strongly stabilizable），若存在编解码—控制策略使得对所有满足某类有界/分布假设的 $(x_0,v_k,w_k)$，闭环轨迹满足期望的稳定性（如一致渐近稳定、均方稳定或有限时间稳定）。使系统可镇定的 $R$ 的下确界称为**最小数据率**（minimal data rate）。

---

## 3. 信号量化模型

量化器的结构决定了量化误差的几何形态，是后续分析与设计的基础。

### 3.1 均匀量化器

对称均匀量化器将实数轴按等距网格划分（Sun [3]）：

$$
q_u(x) = \delta_u \left\lfloor \frac{x}{\delta_u} \right\rceil,\qquad x\in\mathbb{R},
\tag{3}
$$

其中 $\delta_u>0$ 为量化步长，$\lfloor\cdot\rceil$ 表示取最近整数。其量化误差恒有界：

$$
|q_u(x)-x| \le \frac{\delta_u}{2},\qquad \forall x\in\mathbb{R}.
\tag{4}
$$

均匀量化误差与信号幅值无关，属于**加性有界扰动**模型。当 $|x|$ 超出量化器饱和范围时输出“溢出”符号。

### 3.2 对数量化器

为在全局范围（含无界状态）实现镇定，经典做法采用**对数量化器**（Fu & Xie [4]；亦见 Chen & Ren [5]）。设 $\rho\in(0,1)$，定义死区参数与量化电平族

$$
\delta = \frac{1-\rho}{1+\rho},\qquad
U_\rho = \bigl\{\pm \rho^k u : k=0,1,2,\dots\bigr\}\cup\{0\},\; u>0.
$$

对数量化器 $\log_\delta:\mathbb{R}\to U_\rho$ 满足（取 $x>0$ 的情形）：

$$
\log_\delta(x)=
\begin{cases}
\displaystyle \left(\frac{1+\delta}{1-\delta}\right)^{\!l}, &
  \frac{(1+\delta)^{l-1}}{(1-\delta)^{l}} \le x \le
  \frac{(1+\delta)^{l}}{(1-\delta)^{l+1}},\quad l=0,1,2,\dots,\\[1.4ex]
0, & x=0,\\[0.6ex]
-\log_\delta(-x), & x<0.
\end{cases}
\tag{5}
$$

对数量化器具有两个关键性质（Sun [3]）：

- **保符号性**：$q_l(x)\,x \ge 0$ 对所有 $x\in\mathbb{R}$ 成立，且等号当且仅当 $x=0$；
- **相对误差有界**：量化误差满足 $|q_l(x)-x| \le \delta\,|x|$。

即误差随信号幅值成比例缩放——大信号处分辨率粗、小信号处分辨率细。这是其能“全局镇定”的本质原因。

### 3.3 动态变焦（zooming）量化器

静态量化器的范围固定，难以兼顾“初始大状态”与“收敛后高精度”。**变焦量化器**（Brockett & Liberzon [6]）引入随时间演化的动态范围 $l_k>0$：量化器在每一步根据最新符号调整 $l_k$。若最新符号为“溢出区”指示符，则外扩

$$
l_{k+1} := \phi_{\mathrm{out}}\, l_k,\qquad \phi_{\mathrm{out}}>1;
$$

若状态落于内部区间，则按需收缩。于是“粗—细”分辨率随状态演化自适应切换，在有限比特率下实现全局渐近镇定。

---

## 4. 根本极限：数据率定理（Data Rate Theorem）

量化控制最深刻的结论是：**最小数据率由被控对象的“不稳定程度”精确决定**，而与具体控制器结构无关。

### 4.1 标量情形

考虑标量对象 $x_{k+1}=a\,x_k+u_k$ 与 $[-1,1]$ 上的 $M$ 级均匀量化器。Wong & Brockett [7] 与 Baillieul [8] 独立证明：

$$
\boxed{\;M > |a|\;}
\tag{6}
$$

是（一致）镇定的充要条件；等价地，以比特率 $R=\log_2 M$ 表示：

$$
\boxed{\;R > \log_2 |a|\;}.
\tag{7}
$$

直观上：每步必须至少分辨出对象一拍内的“膨胀倍数” $|a|$，否则状态会“跳出”最粗量化区间而失控。**这是数据率定理的第一个实例。**

### 4.2 向量 LTI 情形

借助体积分割（volume-partitioning）论证与 Jordan 标准形，上述结论推广到向量情形，得到**必要且充分**的条件（Nair & Evans [9]；Baillieul [10]）：

$$
\boxed{\;R > \sum_{i:\,|\lambda_i|\ge 1} \log_2 |\lambda_i| \;=:\, H\;}
\tag{8}
$$

其中 $\lambda_1,\dots,\lambda_n$ 为 $A$ 的特征值，$H$ 正是使所有不稳定模态可分辨所需的最小信息率（即拓扑反馈熵 topological feedback entropy 的一种体现）。该判据具有惊人普适性：对多种设定与目标均紧（tight），例如：

- 随机无界初值、无噪声下的 $r$ 阶矩渐近可镇定（Nair & Evans [11]）；
- 有界初值、无噪声下的一致可镇定（Baillieul [10]）；
- 有界初值、有界噪声下的一致可镇定。

### 4.3 含噪声与随机数据率

- **随机无界噪声**：均方稳定可由**时变比特率**达成，其平均值可任意逼近但严格大于 $H$（Nair & Evans [12]）。
- **常比特率约束**：最小常速率为（Kostina 等 [13]）

$$
R_{\min} = \log_2\!\left\lfloor 1 + \prod_{i:\,|\lambda_i|\ge 1} |\lambda_i| \right\rfloor.
\tag{9}
$$

### 4.4 非理想数字信道

当信道非无差错时，信息论特征凸显。对随机离散无记忆信道（DMC），以其普通容量 $C$ 为度量，数据率定理 (8) 可推广，但具体形式高度依赖设定与稳定性目标 [1]。

---

## 5. 常用控制方法

### 5.1 静态对数量化器 + Lyapunov 镇定（Fu & Xie 框架）

将量化误差建模为相对误差有界扰动：$u_k = K \hat x_k$，且 $|\Delta x| \le \delta|\hat x|$。通过选取足够小的 $\delta$（即足够密的电平），使闭环矩阵谱半径小于 1，从而由公共 Lyapunov 函数保证指数稳定 [4]。该方法简洁、无需状态估计的记忆，但要求**无限多个量化电平**（对数量化器本质如此）。

### 5.2 动态变焦量化 + 混合反馈（Brockett & Liberzon）

以有限（甚至 1 bit 溢出标志 + 少量电平）实现全局镇定：

- **变焦外（zoom-out）模式**：用粗量化估计状态所在区间，逐拍外扩范围；
- **变焦内（zoom-in）模式**：范围足够大后切换到控制律，逐步收缩范围。

Liberzon [14] 进一步提出**混合反馈（hybrid feedback stabilization）**：在“估计”与“控制”两个流形（mode）间切换，统一处理状态与输入量化，并给出全局渐近稳定的严格证明。

### 5.3 有限/固定时间齐次量化镇定

全局渐近镇定器在“有限静态量化”下通常失效（Bullo & Liberzon [15]）。Zhou, Polyakov & Zheng [16, 17] 提出**齐次球量化器**（homogeneous spherical quantizer）与基于齐次性的控制律：

$$
\nu_{\mathrm{hom}}(x) = \|x\|_{\mathbf d}^{\,1+\mu}\,K\,\mathbf d(-\ln\|x\|_{\mathbf d})\,x,
$$

证明：当量化足够密时，齐次状态反馈可保证闭环系统的**有限/固定时间稳定**（finite/fixed-time stability），并给出多输入 LTI 系统的充分条件。这是近年将“齐次性工具”引入量化控制的重要进展。

### 5.4 事件触发 + 规定时间 + 动态量化

为提升网络资源效率，近年工作将量化与**事件触发机制（ETM）**结合。Liu, Xu & Ma [18] 针对一类不确定非线性系统，提出**自适应规定时间（prescribed-time, PT）事件触发控制**框架：

- 设计动态事件触发机制与动态事件驱动量化器，构成非周期离散控制；
- 基于自适应参数估计构造 PT 事件触发自适应控制器与 PT 采样—量化自适应控制器；
- 在无需输入—状态稳定（ISS）假设下，保证系统**全局规定时间稳定**，且事件触发采样**无 Zeno 现象**；
- 相比反步法（backstepping），所提“一步控制器”显著降低虚拟控制器计算负担。

### 5.5 滑模控制（SMC）与量化

量化信息下设计滑模面与到达律是另一主流方向。Bandyopadhyay & Behera [19] 系统研究**事件触发滑模控制**在量化状态测量下的设计；Lian & Li [20] 进一步处理含扰动与不确定性的切换线性系统，通过动态量化 + 事件触发采样克服量化饱和与量化误差带来的挑战。

### 5.6 多智能体分布式量化控制

在多智能体系统中，通信与传感均受量化约束。Chen, Li & Gao [21] 针对合作—竞争网络下的非线性多智能体系统，提出**分布式量化控制协议**，利用通用光滑逼近（universal smoothness）处理反馈信道与通信信道中的量化信号，克服反步法在量化信号上的应用障碍。Chen & Ren [5] 的综述还对比了均匀量化器与对数量化器在编队控制中的差异（对数量化器在接近目标编队时分辨率更高）。Wang, He & Huang [22] 则针对无人机（UAV）编队跟踪，借助最终控制信号变换实现**极粗量化**（量化参数可接近 1）下的自适应反步镇定。

### 5.7 方法小结

| 方法 | 量化器类型 | 关键思想 | 典型结果 |
|---|---|---|---|
| 静态对数镇定 [4] | 静态对数（无限电平） | 相对误差有界 + Lyapunov | 指数稳定 |
| 动态变焦/混合 [6,14] | 动态（有限电平） | zoom-out 估计 / zoom-in 控制 | 全局渐近稳定 |
| 齐次有限时间 [16,17] | 齐次球量化 | 齐次性 | 有限/固定时间稳定 |
| 事件触发+规定时间 [18] | 动态事件驱动 | PT 框架 + ETM | 全局规定时间稳定，无 Zeno |
| 滑模量化 [19,20] | 均匀/动态 | 滑模面 + 事件触发 | 鲁棒到达 |
| 分布式多智能体 [21,5,22] | 均匀/对数 | 光滑逼近 / 反步 | 一致有界 / 编队跟踪 |

---

## 6. 近期文献与研究趋势

基于知识库的检索，近年量化控制呈现出以下趋势（代表性文献）：

1. **与事件触发/自适应的深度融合**：在保障稳定性的同时最小化通信负载，如 Liu 等 [18] 的规定时间事件触发量化控制。
2. **有限/固定时间性能**：从“渐近”走向“有限/固定时间”，借助齐次性工具突破静态量化限制（Zhou 等 [16,17]）。
3. **面向非线性与多智能体**：处理非完整约束、合作—竞争拓扑、输入量化等复杂结构（Wang 等 [22]；Chen 等 [21]）。
4. **鲁棒性与不匹配量化**：关注编码器/解码器初始化失配、量化饱和等实际非理想因素（见 Zhou 等 [17] 对 mismatched quantization 的讨论）。
5. **根本极限的精细化**：在随机噪声、非理想信道下对数据率定理的推广与最优率—性能权衡仍是开放方向 [1]。

> **开放性方向**（据 [1] 总结）：除少数特例外，在给定数据率下设计**最优**编码—控制方案、确定**最优性能**的工作仍很有限；多回路、网络化的大规模系统的根本极限也远未解决。

---

## 7. 结论

量化控制揭示了“控制”与“信息”之间的深刻耦合：数据率定理表明，镇定一个 LTI 对象所需的最小信息率恰等于其不稳定模态的对数乘积之和（拓扑反馈熵），这一极限与具体控制结构无关。在工程可用的方法谱系中，**静态对数量化器**实现简单但需无限电平；**动态变焦/混合反馈**用有限比特达成全局镇定；**齐次量化**与**事件触发/规定时间控制**则分别代表“有限时间性能”与“资源效率”两个近年热点；滑模与分布式量化则分别面向鲁棒性与多智能体协同。未来研究将更聚焦于非线性、大规模网络下的最优编码—控制协同与更精细的极限刻画。

---

## 参考文献

[1] G. N. Nair, “Quantized Control and Data Rate Constraints,” in *Encyclopedia of Systems and Control*, 2nd ed., J. Baillieul and T. Samad, Eds. Springer, 2021.

[2] G. N. Nair, F. Fagnani, S. Zampieri, and R. J. Evans, “Feedback control under data-rate constraints: An overview,” *Proceedings of the IEEE*, vol. 95, no. 1, pp. 108–137, 2007.

[3] Z. Sun, *Cooperative Coordination and Formation Control for Multi-agent Systems*. 2018.

[4] M. Fu and L. Xie, “The sector bound approach to quantized feedback control,” *IEEE Trans. Autom. Control*, vol. 50, no. 11, pp. 1698–1711, 2005.

[5] F. Chen and W. Ren, “On the control of multi-agent systems: A survey,” *Foundations and Trends in Systems and Control*, 2019.

[6] R. W. Brockett and D. Liberzon, “Quantized feedback stabilization of linear systems,” *IEEE Trans. Autom. Control*, vol. 45, no. 7, pp. 1279–1289, 2000.

[7] W. S. Wong and R. W. Brockett, “Systems with finite communication bandwidth constraints—part I/II,” *IEEE Trans. Autom. Control*, 1999.

[8] J. Baillieul, “Feedback coding for information-based control of dynamical systems,” in *Proc. IEEE Conf. Decision and Control*, 1999.

[9] G. N. Nair and R. J. Evans, “Stabilizability of stochastic linear systems with finite feedback data rates,” *SIAM J. Control Optim.*, vol. 43, no. 2, pp. 413–436, 2004. (earlier: 2000, 2003)

[10] J. Baillieul, “Control over communications networks: The interplay between local and global,” *IEEE Control Systems Magazine*, 2002.

[11] G. N. Nair and R. J. Evans, “Exponential stabilizability of finite-dimensional linear systems with limited data rates,” *Automatica*, 2003.

[12] G. N. Nair and R. J. Evans, “Mean square stabilizability of stochastic linear systems with constrained feedback data rates,” 2004.

[13] V. Kostina, Y. Peres, M. Z. Posner, and A. S. Willsky, “Lossy compression of countable teams,” *IEEE Trans. Inform. Theory*, 2015 (data-rate bound for constant-rate stabilization).

[14] D. Liberzon, “Hybrid feedback stabilization of systems with quantized signals,” *Automatica*, vol. 39, no. 9, pp. 1543–1554, 2003.

[15] F. Bullo and D. Liberzon, “Quantized control via locational optimization,” *IEEE Trans. Autom. Control*, 2006.

[16] Y. Zhou, A. Polyakov, and G. Zheng, “Finite/fixed-time stabilization of linear systems with state quantization,” *IEEE Trans. Autom. Control*, 2025.

[17] Y. Zhou, A. Polyakov, and G. Zheng, “Robust finite-time stabilization of linear systems with limited state,” *Automatica*, 2025.

[18] W. Liu, S. Xu, and Q. Ma, “Adaptive prescribed-time event-triggered control of nonlinear networked systems under dynamic quantization,” *IEEE Trans. Cybernetics*, 2025. (DOI: 10.1109/TCYB.2025.3551364)

[19] B. Bandyopadhyay and A. K. Behera, *Event-triggered Sliding Mode Control*. Springer, 2018.

[20] J. Lian and C. Li, “Event-triggered sliding mode control of uncertain switched systems via hybrid quantized feedback,” *IEEE Trans. Autom. Control*, 2021.

[21] C. Chen, J. Li, and R. Gao, “Distributed quantized control of nonlinear multi-agent systems under cooperative-competitive networks,” *IEEE Trans. Automation Science and Engineering*, 2026.

[22] Y. Wang, L. He, and C. Q. Huang, “Adaptive time-varying formation tracking control of unmanned aerial vehicles with quantized input,” *ISA Transactions*, vol. 84, pp. 91–104, 2019.

---

*本报告由控制理论文献知识库（okb-assist）检索生成，覆盖经典数据率定理与 2018–2026 年代表性文献。数学表述以标准设定（Nair、Liberzon、Fu & Xie、Brockett 等）为准。*
