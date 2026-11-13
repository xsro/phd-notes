# 方向二（v2）：名义模型下的鲁棒补偿 —— 把惯性误差"除掉"，而不是"抗掉"

> **本版修订说明（v1 → v2）**
>
> | v1 的说法 | v2 的修正 |
> |---|---|
> | 模型误差统一塞进加性 \(\Delta_r\) | ❌ 结构性错误：\(\Delta_r\) 里含 \(\|v\|\)，而 \(v\) 含 \(K_c z(t-h)\)，其幅值 \(\sim e^{\|\lambda\|h}\)，于是"扰动上界 ∝ 状态 ∝ 扰动上界"，递推无法闭合 |
> | 鲁棒项 \(\tau_r\) 破坏线性结构，需重新证明 | 部分不成立：**惯性误差 \(\Delta M\) 恰好不破坏零化结构**，它是控制输入上的静态乘性增益，可精确消除 |
> | \(\Delta M\) 与 \(\Delta C\) 同等对待 | ❌ 二者作用完全不同：\(\Delta M\) 破坏零化，\(\Delta C\) **不破坏**，只降精度 |
> | 用饱和/滑模项压制残余 | ❌ 在精确零化段，\(\tau_r\) 只能**加法**进入误差界，不能减小它；仅在 Phase I 有（有限）价值 |
> | （未讨论） | ✅ 新增：残差增益与零化残差的**精确关系**、容差与 \(e^{-\|\lambda\|h}\) 的定量标度、与现有 `calib_kappa.m` 的衔接 |

## 0. 结论

**可以，但技术路线与 v1 直觉相反。**

- ✅ **惯性误差 \(\Delta M\) 可以被精确消除**，条件是知道或估计出 \(n\times n\) 的"有效增益" \(\bar B_o=M_e^{-1}\hat M_e\)，并在 PDF 辅助输入上左乘其逆。此时**单周期零化精确恢复，不需要任何鲁棒项**。
- ✅ 该修正的物理等价物就是"把计算力矩里的 \(\hat M_e\) 换成 \(M_e\)"——但它把"要辨识 10n 个惯性参数"降级为"要估计一个 \(n\times n\) 的增益矩阵"，进而可再降为**一个标量**，与现有 `calib_kappa.m` 的 \(\kappa\) 完全同构。
- ❌ **科氏误差 \(\Delta C\) 无法消除**，但它只进加性扰动，不破坏零化结构。
- ❌ **鲁棒饱和项 \(\tau_r\) 不是解决方案**：在精确零化段它对误差界无改善（§7 给证明），在 Phase I 其价值被方向 6 的包络机制覆盖。
- ⚠️ 量化门槛：零化残差 \(\propto \|\Delta M\|\cdot e^{\|\lambda\|h}\)。**现有参数（\(h=1\,\mathrm s\)，闭环极点达 \(-8.45\)）下相对惯性误差需 \(\lesssim 10^{-4}\)**；把 \(h\) 缩到 \(\lesssim 0.1\,\mathrm s\) 后容差放宽到 \(O(0.1\text{–}1)\)。这条定量关系是本方向最有价值的产品。

## 1. 精确闭环推导（符号一次定清）

真实对象与控制律：

\[
M_e\ddot q+C_e\dot q=\tau+d,\qquad
\tau=\hat M_e v+\hat C_e\dot q-\hat d .
\]

记 \(\Delta M=M_e-\hat M_e\)，\(\Delta C=C_e-\hat C_e\)，\(e_d=d-\hat d\)。代入：

\[
M_e\ddot q=\hat M_e v-\Delta C\dot q+e_d
\ \Longrightarrow\
\ddot q=\underbrace{(I-M_e^{-1}\Delta M)}_{\textstyle \bar B_o}v-M_e^{-1}\Delta C\dot q+M_e^{-1}e_d .
\]

**核心结构**（\(\bar B_o=M_e^{-1}\hat M_e\)，注意它是 \(n\times n\)，可逆性等价于 \(\hat M_e\) 可逆）：

\[
\boxed{\;\ddot q=\bar B_o\,v-M_e^{-1}\Delta C\dot q+M_e^{-1}e_d\;}
\]

取 \(v=\ddot q_d+\nu\)，则误差通道

\[
\boxed{\;\ddot e=\bar B_o\,\nu
\underbrace{-\,M_e^{-1}\Delta M\,\ddot q_d}_{\text{已知信号、乘性误差}}
+\,\underbrace{M_e^{-1}\bigl(e_d-\Delta C\dot q\bigr)}_{\text{加性扰动}}\;}
\tag{2.1}
\]

状态形式（\(\bar B:=B\bar B_o\)）：

\[
\dot z=Az+\bar B\nu+Bw,\qquad
w=-M_e^{-1}\Delta M\ddot q_d+M_e^{-1}(e_d-\Delta C\dot q).
\tag{2.2}
\]

## 2. 三类模型误差的作用完全不同

由 (2.2) 立刻看出：

| 误差源 | 进入位置 | 与 \(K_c\) 的关系 | 是否破坏 \(\Delta_K(h)=0\) | 处理方式 |
|---|---|---|---|---|
| \(\Delta M\)（惯性） | \(\nu\) 前的乘性增益 | **与 \(K_c\) 相乘** | ✅ 破坏 | 估计 + 求逆；或算容差 |
| \(\Delta C\)（科氏） | 加性 \(w\) | 不相乘 | ❌ 不破坏 | 归入方向 1 的 \(\bar\Delta_r\) |
| \(e_d\)（观测残差） | 加性 \(w\) | 不相乘 | ❌ 不破坏 | 同上 |
| \(\tau_r\)（鲁棒项） | 加性 \(w\) | 不相乘 | ❌ 不破坏 | 但见 §7，收益可疑 |

**这条区分是 v1 缺失的关键。** v1 把三者统一塞进 \(\Delta_r\)，导致上界依赖于 \(\|v\|\)，而 \(\|v\|\ge\|K_c\|_\infty\|z\|\)，于是

\[
\bar\Delta_r\ \gtrsim\ \|\Delta M\|\,\|K_c\|_\infty\,\|z\|_\infty
\quad\Longrightarrow\quad
\|z\|_\infty\le\Gamma_h\bar\Delta_r
\]

形成"界 ∝ 界"的循环，只能靠 \(\Gamma_h\|\Delta M\|\|K_c\|<1\) 这种不可验证的条件解套。**规范做法是不要把乘性误差写成加性扰动。**

## 3. 关键结果 R1：惯性误差可被精确消除

### 3.1 修正律

在 PDF 辅助输入上左乘增益逆：

\[
\boxed{\;\tau=\hat M_e\Bigl[\ddot q_d+\hat B_o^{-1}\bigl(K_0z-K_c(t)z(t-h)\bigr)+\hat B_o^{-1}\hat M_e^{-1}\widehat{\Delta M}\,\ddot q_d\Bigr]
+\hat C_e\dot q-\hat d\;}
\tag{3.1}
\]

其中 \(\hat B_o=I-\hat M_e^{-1}\widehat{\Delta M}\)，第三项是 \(\ddot q_d\) 的已知信号前馈补偿。

**物理等价**：若 \(\hat B_o=\bar B_o\)，则 \(\hat M_e\hat B_o^{-1}=M_e\)，即 (3.1) 就是**用真实惯性矩阵的标准计算力矩**。所以本结果并不神秘——它说明的是：**惯性误差在 PDF 框架里的"正确身份"是一个可求逆的静态增益，而不是一个需要高增益压制的加性扰动。**

### 3.2 定理 3′（惯性修正下的精确零化）

设 \(\hat B_o=\bar B_o=M_e^{-1}\hat M_e\)（在 \([0,2h]\) 内视为常值）。则由 (2.1)，

\[
\ddot e=K_0z-K_c(t)z(t-h)+M_e^{-1}\bigl(e_d-\Delta C\dot q\bigr),
\]

即误差通道与名义情形 (引理 1)**逐项相同**。因此定理 2 的结论原样成立：

\[
\Delta_K(h)=0,\qquad
z(t)=0\ \ \forall t\ge2h\qquad(e_d=\Delta C=0\ \text{时}).
\]

**证明**：直接代入，\(\bar B_o\hat B_o^{-1}=I\)，其余同定理 2 的证明。无需任何新分析。∎

**与 v1 的差别**：v1 需要"重新组织稳定性证明"，实际上需要的只是一个 \(n\times n\) 的乘法改写，**定理 2 的证明一行都不用改**。

### 3.3 若不愿动控制律，也要知道残差是多少（v1 完全缺失）

设仍用名义设计（\(K_c^{nom}\) 由 \((A_c,B)\) 构造），实际对象为 \(\bar B_o\neq I\)，则**精确**地

\[
\Delta_K(h)=e^{\bar A_c2h}-\int_h^{2h}e^{\bar A_c(2h-s)}\,\bar B\,K_c^{nom}(s)\,e^{\bar A_c(s-h)}\,\mathrm ds,
\qquad \bar A_c=A+\bar B K_0 .
\]

在 δ=0 处展开（\(\delta=\bar B_o-I=-M_e^{-1}\Delta M\)）一阶项给

\[
\boxed{\;\|\Delta_K(h)\|\ \lesssim\
\|\delta\|\Bigl(\|K_0\|\,J_1+\sup_{s\in[h,2h]}\|K_c^{nom}(s)\|\,J_2\Bigr)\;}
\tag{3.2}
\]

\[
J_1=\int_0^{2h}\|e^{A_c(2h-\tau)}B\|\,\|e^{A_c\tau}\|\,\mathrm d\tau,
\qquad
J_2=\int_h^{2h}\|e^{A_c(2h-s)}B\|\,\|e^{A_c(s-h)}\|\,\mathrm ds .
\]

注意 \(J_1,J_2\) 中的传播子时间参数都在 \([0,h]\) 内（收缩段），故 \(J_1\sim\kappa\|B\|h\)，\(J_2\sim\kappa^2h\) 都是 \(O(1)\)；**全部放大都集中在 \(\|K_c^{nom}\|_\infty\sim e^{\|\lambda_\ast\|h}\)**。于是容差

\[
\boxed{\;\frac{\|\Delta M\|}{\underline m}\ \lesssim\ \frac{1-\epsilon_\Delta}{\|K_0\|J_1+\|K_c\|_\infty J_2}\ \sim\ e^{-\|\lambda_\ast\|h}\;}\tag{3.3}
\]

其中 \(\|\lambda_\ast\|=\max|\mathrm{Re}\,\lambda(A_c)|\)。

### 3.4 代入现有参数（这是"能否实现"的定量答案）

`simulation/main_dou_yue_259_compare.m` 中 \(K_p=\mathrm{diag}(30,30,30,25,25,20,8)\)、\(K_d=\mathrm{diag}(12,12,12,10,10,9,4)\)，\(\ddot e=-K_pe-K_d\dot e\) 的极点为

\[
\lambda=\tfrac{-K_d\pm\sqrt{K_d^2-4K_p}}{2}\ \in\ [-3.55,\,-8.45]\ \mathrm{rad/s},
\qquad h=1.0\ \mathrm s .
\]

| 量 | 估计 |
|---|---|
| \(\|\lambda_\ast\|h\) | \(8.45\) |
| \(e^{\|\lambda_\ast\|h}\) | \(\approx 4.7\times10^{3}\) |
| \(\mathrm{cond}(W_c)\)（论文实测） | \(10^5\sim10^6\) ✅ 与 \(e^{2\|\lambda_\ast\|h}\approx 2\times10^{7}\) 同量级 |
| 相对惯性误差容差 (3.3) | \(\sim 10^{-4}\) |

**解读**：在现有 \(h=1\,\mathrm s\) 下，只要参数误差超过约 \(10^{-2}\%\)，\(\varepsilon_\Delta\) 就逼近甚至超过 1，零化失效、退化为有界跟踪。这与论文 Remark 中"\(n=7\) 时 PDF 与 PD 终端误差一致"的仿真现象**互为印证**（该 Remark 归因于 Gramian 病态，本方向给出第二个、更根本的归因）。

**设计处方**：\(h\) 必须与闭环最快极点匹配

\[
\boxed{\;\|\lambda_\ast\|\,h\ \lesssim\ 1\;}\qquad
\Rightarrow\quad h\lesssim\frac{1}{8.45}\approx 0.12\ \mathrm s .
\]

取 \(h=0.1\,\mathrm s\)、\(T_o=1\,\mathrm s\) 时 \(T_{\mathrm{pre}}=1.2\,\mathrm s\)（**优于现有的 \(3.0\,\mathrm s\)**），同时 \(\varepsilon_\Delta\) 从 \(\gg1\) 降到 \(O(\|\Delta M\|/\underline m)\)，容差提升到 \(10^{-1}\) 量级。**这是一个可以直接改进主论文数值结果的结论**，建议优先验证（只需改 `ctrl.h` 一行）。

## 4. 未知 \(\Delta M\) 的两级实现

### 4.1 L1（不估计）：只做容差核算

保持名义控制律，用 (3.2)(3.3) 说明"在何种参数误差下仍保持有界"，结论并入方向 1。**优点：零改动；缺点：\(h=1\) 时几乎必然退化，说服力弱。**

### 4.2 L2（低维增益估计，推荐）

不必辨识 10n 个惯性参数——只需估计 \(n\times n\) 的 \(\bar B_o\)。三级降维：

**(a) 标量级（最实用）**：若误差主要是整体惯量尺度失配（质心/载荷质量误差的主效应），则

\[
M_e\approx c\,\hat M_e\ \Longrightarrow\ \bar B_o\approx \tfrac1c I\ \Longrightarrow\
\hat B_o=\beta I,\quad \beta>0\ \text{标量}.
\]

估计 \(\beta\) 只需一次标量最小二乘。由 (2.1)，

\[
\ddot q\approx\beta\,u,\quad u:=\ddot q_d+\nu
\ \Longrightarrow\
\hat\beta=\frac{u^\top\ddot q_k}{u^\top u}\ \ (\text{递归形式：}\ \hat\beta_{k+1}=\hat\beta_k+\gamma\tfrac{u_k(\ddot q_k-\hat\beta_ku_k)}{1+\gamma u_k^\top u_k}).
\]

**这与现有 `calib_kappa.m` 标定 \(\kappa\) 的做法完全同构**——现有代码已经在做方向二的核心动作，只是没有把它与"\(\bar B_o^{-1}\) 预乘"这一理论结论连起来。建议把 \(\kappa\) 重新解释为 \(\beta\)，并给出 (3.1) 形式的实施位置（**在 \(\nu\) 上左乘，而不是在 \(\tau\) 上右乘**）。

**(b) 对角级**：各关节主导惯量失配不同 → \(\hat B_o=\mathrm{diag}(\beta_i)\)，7 个标量，逐关节标量 RLS。

**(c) 全矩阵级**：参数辨识 \(\hat\pi\) → 按名义参数重算 Schur 补得 \(\hat M_e\) → \(\hat B_o=I-\hat M_e^{-1}\widehat{\Delta M}\)。精度最高但有两个坑（§8）：Schur 补对 \(\pi\) **非线性**（\(H_b^{-1}\) 出现），故不能直接线性回归；且需要基座加速度信息。

**关键实施约束**：\(\hat B_o\) **只在 Phase I（\(t<T_o\)）在线更新，\(t\ge T_o\) 冻结**。理由：
1. \(K_c(t)\) 与 \(W_c\) 是离线算的，\(\hat B_o\) 若在周期内变化，等价于要求方向 7 的 \(\Phi\) 框架；
2. 冻结后 \(\bar B_o\) 在一个周期内视为常值，定理 3′ 严格可用；
3. 冻结时刻的残余误差 \(\epsilon_B=\|(\bar B_o-\hat B_o)\hat B_o^{-1}\|\) 直接进 (3.3) 的 \(\|\delta\|\) 位置。

### 4.3 残差增益下的结果

设 \(\hat B_o\neq\bar B_o\)，记 \(\epsilon_B=\|\bar B_o\hat B_o^{-1}-I\|\)。由 (3.1) 得

\[
\dot z=Az+B\bigl[I+\epsilon\text{-项}\bigr]\bigl(K_0z-K_cz(t-h)\bigr)+Bw
\]

即回到 (3.2) 的形式，只是 \(\|\delta\|\to\epsilon_B\)。于是

\[
\|\Delta_K(h)\|\le\epsilon_B\Bigl(\|K_0\|J_1+\|K_c\|_\infty J_2\Bigr)=:\varepsilon_\Delta,
\qquad
\limsup_{k\to\infty}\|z_k\|\le\frac{\Gamma_h\bar\Delta_r}{1-\varepsilon_\Delta}.
\]

**这给出了完整的"可实现性判据"**：给定参数误差与 \(h\)，先算 \(\varepsilon_\Delta\)，若 \(<1\) 则 PDF 仍有零化意义，若 \(\ge1\) 则本方向对该工况无收益，应回退到方向 1 的有界结论。

## 5. 与 v1 的关键分歧：\(\tau_r\) 要不要？

**结论：第一版不要。**

理由见 §7 的定量分析。若一定要保留，必须做两处修正（v1 未提）：

1. **左乘总惯性估计，而不是名义估计**：取
   \[
   \tau_r=-\bar M_e k_r\tanh(s/\varphi),\qquad s=\dot e+\Lambda e,
   \]
   使 \(M_e^{-1}\tau_r\approx-k_r\tanh(s/\varphi)\) 干净地进入加速度通道；若用 \(\hat M_e\) 左乘则残留 \(M_e^{-1}\hat M_e=\bar B_o\) 的增益误差，鲁棒项自己又引入一个新的不确定性。
2. **只在 Phase I 使用**（\(K_c=0\) 时段），\(t\ge T_o\) 置零，以保零化精确性。

## 6. 加入 \(\Delta C\) 与 \(\tau_r\) 后的实际有界定理

设 \(\bar B_o=\hat B_o\)（惯性误差已修正），\(\|M_e^{-1}(e_d-\Delta C\dot q+\tau_r)\|\le\bar w\)。由 \(\dot z=Az+B(K_0z-K_cz(t-h))+Bw\)，直接套方向 1 的定理 4：

\[
\|z(T_o+2kh)\|\le\Gamma_h\bar w,\qquad
\|z(t)\|\le\Gamma_h^*\bar w .
\]

显式化：若 \(\|\Delta C\|\le\bar C_0+\bar C_1\|\dot q\|\)，\(\|\dot q\|\le\bar v_q\)（由 \(\dot q_d\) 有界 + \(\|z\|\) 有界推出），则

\[
\bar w\le\bar m^{-1}\bigl(\bar d_o+\bar C_0\bar v_q+\bar C_1\bar v_q^2+\|\tau_r\|_\infty\bigr).
\tag{6.1}
\]

**注意 (6.1) 不需要 \(\bar v_q\) 的先验值**：\(\bar v_q\) 只出现在右端的独立项里，而 \(\|\tau_r\|_\infty=k_r\) 是已知常数。这与 v1 的循环结构完全不同。**这是 v2 相对 v1 在"理论可用性"上的本质改善。**

## 7. 为什么 \(\tau_r\) 在精确零化段无效

在 \(\Delta_K(h)=0\) 的周期里，误差满足

\[
z(2h)=\int_0^{2h}\mathcal S_h(2h,s)B\,M_e^{-1}\tau_r(s)\,\mathrm ds
\]

（因为齐次项被零化掉）。取范数：

\[
\|z(2h)\|\le\Gamma_h\sup_s\|M_e^{-1}\tau_r(s)\|\le\Gamma_h\,\bar m^{-1}k_r\ \ (\text{用 }\tanh\text{ 的界}).
\]

即 **\(\tau_r\) 的上界以"加法"进入**，与 \(w\) 的贡献同号叠加，除非 \(\tau_r\) 与 \(w\) 逐点反号（无法先验保证）。因此

> 在精确零化段，鲁棒饱和项的**最坏情形界比不加更差**（\(k_r\) 直接加进 \(\Gamma_h\bar w\)）。

只有在"阻尼能真正减小 \(\sup\|z\|\)"时才可能有利，而这在"每周期被强制打回零"的机制下并不能保证。**结论：\(\tau_r\) 的作用域只剩 Phase I，而 Phase I 用方向 6 的包络（或直接提高 \(K_0\)）更干净。**

## 8. 风险

1. **Schur 补非线性**：\(M_e=H_m-H_{bm}^\top H_b^{-1}H_{bm}\) 对惯性参数 \(\pi\) 非线性（含 \(H_b^{-1}\)），故 (a) 不能构造 \(M_e\) 的线性回归量；(b) 间接自适应（估 \(\pi\) 后重算）只能做**一阶局部**分析，需参数集有界 + 投影。全维 13 维动力学对 \(\pi\) 线性，但需要基座加速度，工程上要靠动量求导重构，含噪。
2. **需要 \(\ddot q\) 才能估 \(\beta\)**：测速差分噪声会直接注入 \(\hat\beta\)。可选缓解：用一阶滤波后的 \(\ddot q\)（如 PTDO 的 \(z_2\) 通道）、或用 \(\tau\)-\(\ddot q\) 的积分形式估计；但精度会下降。
3. **\(\bar B_o\) 的时变性**：\(\Delta M(q)\) 随构型变化，严格处理需方向 7 的 \(\Phi\) 框架。冻结近似误差 \(\sim h\|\dot q\|\|\partial_q(M_e^{-1}\Delta M)\|\)。
4. **条件数与容错的二次惩罚**：\(h\) 大 → \(W_c\) 病态 → \(K_c\) 数值失准（\(\varepsilon_\Delta\) 已有底噪）**且** 容差 \(e^{-\|\lambda\|h}\) 变小，两个劣化同向叠加。这解释了为什么"大 \(h\) 的 PDF"在实践中从未跑出理论性能。
5. **论文定位风险**：本方向的严格结论（定理 3′）等价于"用精确惯性做计算力矩"，容易被审稿人视为平凡。**必须把价值落在 (3.3) 的容差标度与 §3.4 的 \(h\) 设计处方上**，而不是落在新定理上。

## 9. 最小仿真验证（可直接跑，用现有代码）

### 9.1 决定性实验：\(\|\Delta_K(h)\|\) 对参数误差的敏感度（~40 行）

```matlab
% 在当前构型冻结 DeltaM，数值构造真实周期映射残差
Kp = diag([30,30,30,25,25,20,8]);  Kd = diag([12,12,12,10,10,9,4]);
K0 = [-Kp,-Kd];  h = 1.0;                       % 再扫 h = 0.05:0.05:1
pdf = build_pdf_gain(K0, 7, h);
P = dou_yue_259_parameters();  S = ffsm259_model(P, P.q0, P.dq0);
for eps_M = [0 0.01 0.05 0.1 0.3]               % 相对惯性误差
    Me_bar = (1+eps_M)*S.Me;                    % 标量幅值误差 (L2-a 情形)
    Bo     = S.Me \ Me_bar;                     % 真实有效增益 (此处 = (1+eps) I)
    Ac_act = pdf.A + pdf.B*Bo*K0;               % 真实基线
    % 数值积分 Phi_act(s,0) 并按 (3.2) 组装 Delta_K
    ...
end
```
判据：**画出 \(\|\Delta_K(h)\|\) 对 \(\varepsilon_M\) 与 \(h\) 的等值线，找到 \(\|\Delta_K\|=1\) 的边界**。这条曲线就是本方向可行域的边界，比任何定理都更有说服力。

### 9.2 控制律对比（4 个算例）

| 算例 | 说明 | 预期 |
|---|---|---|
| A | 名义 PDF（现有基线） | 参考 |
| B | 名义 PDF + 参数扰动 \(\pm30\%\)（无补偿） | 显著退化，末端误差升 \(10^{-2}\) |
| C | B + (3.1) 的标量 \(\hat B_o=\hat\beta I\)（\(\hat\beta\) 取真值，验证上界） | 恢复 \(10^{-9}\) 量级 → 验证定理 3′ |
| D | C 但 \(\hat\beta\) 用 §4.2(a) 的 RLS 在线估计（Phase I 更新，\(T_o\) 冻结） | 接近 C，说明可实现 |

补充扫描：\(\hat\beta\) 故意偏离真值 1%, 5%, 10%，画 \(\|e(T)\|\) 对 \(\|\hat\beta/\beta-1\|\) 的曲线，与 (3.3) 的预测斜率对照。

## 10. 论文定位

> 在仅获得名义自由漂浮动力学模型的条件下，指出惯性模型误差在周期延迟反馈框架中并非加性扰动，而是作用于辅助输入的静态乘性增益；据此在辅助输入上引入增益逆修正，使单周期零化的代数条件在惯性误差存在时精确保持，并给出零化残差与参数误差、延迟时间之间的显式标度关系，从而定量界定"模型精度—预设时间"的可行域。

**建议写入位置**：
- 定理 3′ + (3.1) → 主论文 §4 新增一小节（约半页）；
- (3.2)(3.3) + 容差表 → §5 的 Remark（与现有 Gramian 病态 Remark 合并，形成"数值条件 + 模型条件"双重判据）；
- §3.4 的 \(h\lesssim1/\|\lambda_\ast\|\) → 直接进入 §6 仿真参数讨论，**很可能顺带把 \(n=7\) 的 PDF≈PD 现象改成 PDF 明显优于 PD**。

## 11. 与其它方向的接口

| 方向 | 关系 |
|---|---|
| 1 | 本方向是 1 的"乘性误差补丁"：1 管加性 \(w\)，本方向管 \(\bar B_o\)；合并后 (6.1) 才是完整界 |
| 3 | \(\bar B_o\) 随构型变化 → 需方向 7 的 \(\Phi\) 框架才能严格处理 |
| 4 | \(h\lesssim1/\|\lambda_\ast\|\) 是给 4 的**硬约束**，比"力矩峰值"约束更早触发 |
| 5 | \(h\) 的选择同时受 4 条约束（条件数、容差、力矩峰值、\(T_{\mathrm{pre}}\)），建议 5 直接把 (3.3) 纳入协同设计 |
| 6 | Phase I 由 6 管，Phase II 由本方向保证零化，两者不重叠、不冲突 |
| 7 | 本方向是 7 的动机来源之一（时变 \(\bar B_o\) 是最自然的时变基线） |
