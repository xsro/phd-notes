# okb-assist 中 Automatica 与 IEEE TAC 上的 CBF / HOCBF 文献：它们到底在处理什么问题

> 数据来源：`okb-assist` 知识库（`search_info` / `grep_search` / `get_document_abstract` / `read_markdown`），
> 检索日期：本次会话；`doc:` 后数字为知识库文档编号，可用 `read_markdown(id=...)` 复读原文。
>
> 范围界定：**期刊 = Automatica（IFAC）与 IEEE Transactions on Automatic Control（TAC）**，
> 主题 = control barrier function (CBF) / high-order CBF (HOCBF) / barrier certificate / 安全集前向不变性。
> IEEE/CAA Journal of Automatica Sinica 另有 2 篇，单列于附录 C（不同期刊，勿混算）。

---

## 0. TL;DR —— 十二个问题簇

Automatica / TAC 上的 CBF 文献**不是**在反复证明"barrier 保证安全"这条老定理，
而是在啃 CBF-QP 范式落地时的 **12 类"卡点"**：

| # | 问题簇 | 一句话概括 | 该簇文献数（可重叠，约） |
|---|---|---|---|
| A | **相对度 / 高阶推广** | 约束出现在 $\dot h$ 里没有 $u$（相对度 ≥2），怎么办；HOCBF 的构造、稳定性、保守性 | 10 |
| B | **QP 可行性 / 多约束共存** | 约束一多、控制界一紧，QP 就 infeasible，"安全"变成"无解" | 9 |
| C | **不期望平衡点 / CLF-CBF 相容** | CBF-QP 会**自己造出**稳定的伪平衡，把系统钉死在安全集内部 | 3 |
| D | **不确定性 / 扰动 / 自适应安全** | 模型未知、匹配或**非匹配**扰动下如何不保守 | 12 |
| E | **离散时间 / 采样数据 / 事件触发** | 连续时间定理落到数字控制、零阶保持、异步时钟上就失效 | 6 |
| F | **随机系统 / 概率安全** | 噪声下"安全"只能是概率 1 / 概率下界，且已有定理被证伪过 | 6 |
| G | **时滞 / 混杂 / 不连续 / 非光滑安全集** | 无限维、切换、$\dot x$ 不连续、安全集是并交嵌套 | 7 |
| H | **分布式多智能体：连通性 / 编队 / 避碰 / 共识** | 局部信息 + 通信受限下，谁保证全局安全 | 12 |
| I | **安全 vs. 性能/优化目标冲突** | 安全与跟踪/最优性冲突时，谁来让步、让步多少 | 9 |
| J | **输出反馈 / 状态估计 / 观测器** | CBF 要求全状态；只能测一部分时怎么办 | 4 |
| K | **理论完备性（逆定理 / 安全集稳定性 / ISS）** | 安全集本身是否渐近稳定、是否 ISS、逆定理是否成立 | 4 |
| L | **应用牵引（车/航天/机器人/UAV/吊挂）** | CBF 作为"约束翻译器"进入具体任务 | 10 |

（同一篇常落在多个簇，故计数之和 > 61；簇内清单见 §3，完整表见附录 A。）

**三条最有信息量的观察**（详见 §4–§5）：

1. **HOCBF 这条线在 TAC 上被"接着做"了 4 次**：提出（doc:1649）→ 稳定性/ISS 充分条件（doc:630）→
   更弱的 HOBF（不需要 forward completeness，doc:849）→ 不确定+HOCBF（doc:1793）。
   而 Automatica 上出现的是对"能不能用好"的追问：doc:2176 明确"**避免构造 HOCBF**"，
   理由是它会带来**保守的输入可行集**；doc:3409 批计算复杂度、doc:4553 批离散化后相对度不再固定（5 类质疑见 §4.1）。
2. **QP 的"病"有两种，Automatica 2024–2026 集中清账**：不可行（feasibility）与**伪平衡**（undesired equilibria，
   doc:4531/4576/4558）。后者尤其重要——它是"安全控制器把任务搞死"的机理级解释。
3. **TAC 偏"框架改造"（高阶、自适应、事件触发、对抗、分布式证书），Automatica 偏"问题诊断 + 新构造"**
   （可行性充分条件、非光滑集、逆定理、随机/时滞、伪平衡分析）。

---

## 1. 检索与筛选口径（可复现）

```text
# 元数据检索（去重取并集）
search_info(query ∈ {control barrier function, high-order control barrier, barrier certificate,
                     safety-critical control, CBF, barrier, forward invariance, safety filter,
                     safe set, invariance control, reach-avoid, safety certificate, zeroing barrier,
                     exponential control barrier, discrete-time control barrier, collision avoidance safety,
                     safety-critical, safe control, safety constraints, barrier Lyapunov function,
                     constraint satisfaction safety, multi-agent collision avoidance})
   → 过滤 journal ∈ {Automatica*, IEEE Transactions on Automatic Control*}
# 全文检索补齐（journal 过滤 + start_after_id 游标翻页）
grep_search(query="barrier function"|"forward invariant", journal="Automatica"|"Transactions on Automatic Control")
```

* 主题命中并集 **150 篇**；其中 Automatica + TAC **纳入 61 篇**（TAC 15 / Automatica 46），
  另有 2 篇"评论 + 作者回复"、5 篇边缘相关、2 篇 JAS（附录 A/C）。
* **数据质量提示**：少数 in-press 文献（doc:4592/4527/4559/4612）库内**尚未生成 Markdown**，本报告对它们的描述仅基于 `get_document_abstract`，颗粒度低于其他条目；doc:4509 / 4527 的 `title` 字段在库内被截断（附录中标注 `…`）。
* 剔除原则：① 只是"barrier function"字面命中的**自适应滑模 BAR 类**（barrier-function adaptive sliding mode，
  doc:54/910/3132/3188/2934…，其 barrier 是"增益上界"不是安全集）；
  ② **barrier Lyapunov function（BLF）**控制（doc:1069/3285，靠 backstepping 不靠 QP）；
  ③ **log-barrier 罚函数**优化（doc:2268）；
  ④ **reach-avoid 微分博弈**（doc:731/733/4121，barrier 是"半透面"）；
  ⑤ prescribed-performance / PPC 类（doc:3853/2805，形式相近但非 CBF）。清单见 §6。

---

## 2. 全景统计

### 2.1 年份 × 期刊

| 年 | 2016 | 2018 | 2021 | 2022 | 2023 | 2024 | 2025 | 2026 | 2027 |
|---|---|---|---|---|---|---|---|---|---|
| **Automatica** | 1 | 2 | 3 | 6 | 4 | 9 | 6 | 13 | 2 |
| **TAC** | 1 | – | – | 4 | 3 | 2 | 1 | 4 | – |

要点：**2026 年是爆发年**（Automatica 13 篇，多为 in-press / 卷期未定；TAC 4 篇），
且 Automatica 的 2016–2022 段明显是"**打地基**"（CLBF、robust CBF、control-sharing、逆定理），
2024 之后转向"**框架体检**"（可行性、伪平衡、离散化、非光滑集）。

### 2.2 两刊的风格差异（在 CBF 主题上）

| 维度 | IEEE TAC | Automatica |
|---|---|---|
| 典型产出 | 既有框架的**推广/加固**：高阶、自适应、事件触发、对抗、分布式证书 | **新问题识别 + 新构造 + 完备性证明**：可行性充分条件、非光滑/随机/时滞、逆定理、伪平衡分析 |
| 数学口径 | Lyapunov/ISS、微分包含、set invariance、QP 灵敏度 | 非光滑分析、集合论、SOS/LMI、Krasovskii、半透面以外的集合不变性 |
| 例子密度 | 大多数值仿真 + 少量硬件（doc:1793 有履带机器人实验） | 仿真为主，偶有实验（doc:1531 采集实验、doc:3417 吊车实验） |
| 对 HOCBF 的态度 | **主战场**（1649 提出、630 稳定、849 弱化、1793 鲁棒化） | **既用又疑**：3409/3417/1426 用并修补（保可行、降保守）；2176 明确"不构造 HOCBF"；4553 指出离散化下递归前提失效（详见 §4.1） |

---

## 3. 十二个问题簇逐一展开

### A. 相对度与高阶推广：HOCBF 的"为什么需要"与"代价"

**问题**：安全约束 $h(x)\ge 0$ 常是位置/姿态约束，其 $\dot h$ 不含 $u$（相对度 $m\ge 2$），
一级 CBF 条件无法映射成对 $u$ 的约束 → 早期只能靠 input–output 线性化 + 极点配置（指数 CBF），或 backstepping（仅 $m=2$）。

| doc | 做了什么 |
|---|---|
| **1649** (TAC'22) | **HOCBF 定义**：递归构造 $\psi_0=h,\ \psi_i=\dot\psi_{i-1}+\alpha_i(\psi_{i-1})$，用 Lyapunov 型条件保证**多个集合之交**前向不变 ⇒ 原约束满足；并提出 **penalty method / parameterization method** 两种办法治"QP 不可行"（把可行性问题与高阶问题一起处理） |
| **630** (TAC'23) | 补 HOCBF 的**稳定性理论缺口**：给 $\mathcal{C}$ **一致渐近稳定**的充分条件（允许集合闭但不必紧），并证明结论对"HOCBF 诱导的输入约束所对应**微分包含**"成立——从而**绕开**对优化型控制律连续性的要求；再给 ISS 充分条件（用最坏情形比较系统） |
| **849** (TAC'22) | **HOBF**：比 HOCBF 更弱的高阶 zeroing barrier，①**不需要 forward completeness**；②保证前向不变集的**渐近稳定性**（利于鲁棒）；③放松"一致相对度"要求并给出**无奇异、局部 Lipschitz** 的控制；④"performance-critical"：安全集的一个子集里**任何**有界控制律都可用。案例：刚体姿态 + 互联单元格区域 |
| **1426** (Automatica'23) | 高阶 + **输入受限 + 有界扰动**：给相对度 2 的两种、更高相对度的第三种 **Robust CBF** 构造，使零下水平集 ⊂ 原约束集，且 RCBF 条件**在零下水平集内恒可行**；用**切换**方式施加（远离边界时不施加，可调"贴边程度"）；小行星飞越案例 |
| **1793** (TAC'25) | 匹配 + **非匹配**不确定性下的鲁棒 CBF：把 CBF 与**不确定性估计器**耦合，用估计误差上界构造安全控制器；扩到 **HOCBF**（相对度差异 ≤1 ⇒ 二阶锥约束）；弹性执行器仿真 + 履带机器人实验 |
| **3409** (Automatica'26) | 输出跟踪（**软**约束）+ 安全（**硬**约束）统一在**分级（hierarchical）CBF**下，用**双超水平集**处理高相对度输出；用 **Faà di Bruno 公式**把 $n$ 阶 Lie 导数写成代数条件，构造在线 Lyapunov-like 求解器，避免传统 HOCBF 的计算复杂度；松弛变量保可行 |
| **4553** (Automatica'26) | **任意相对度的离散时间 CBF**（GD-CBF）：指出离散化会**改变**相对度、且相对度在状态空间里可能**逐点变化**；引入"安全预测时域" |
| **4451** (TAC'23) | **采样数据 + 高相对度**安全约束下的**对抗韧性**：只保证"正常 agent"把安全集撑住，部分 agent 恶意；含输入约束、时钟异步、扰动、分布式求解 |
| **2176** (Automatica'26) | **反向立场**：为 high-relative-degree 安全约束**不构造 HOCBF**（高阶可行域太保守），改用 **CBF-QP 生成安全虚拟控制 + BLF backstepping 跟随** 的 co-design，兼顾跟踪性能（对比 HOCBF 方法更好） |
| **4592** (Automatica'22) | 固定时间收敛 + **时空约束/输入约束**：先建立"固定时间稳定的吸引域 ⇔ 所需收敛时间/控制权限"关系，再用 QP 在安全集内合成（摘要未显式使用 CBF 术语，属**同族框架**，仅据摘要收录） |

**小结**：这一簇的共同结论是——**HOCBF 解决了"能不能写约束"，但没解决"写了之后可行不可行、稳不稳定"**。
TAC 路线靠"定义更弱 + 证明更强"（849/630），Automatica 路线靠"换构造"（3409/4553/2176）。

---

### B. QP 可行性 / 多约束共存：安全 ≠ 有解

**问题**：$\min \|u\|^2$ s.t. CLF、多个 CBF、$u\in\mathcal{U}$，在紧控制界、约束多、状态贴边时**必然**可能 infeasible；
而 infeasible 在实机上等于"控制器掉线"。

| doc | 招法 |
|---|---|
| **1649** (TAC'22) | penalty / parameterization 两种松弛化，把可行性写进设计 |
| **1368** (TAC'22) | **aCBF**：引入时变函数改造 CBF 定义（PACBF 参数自适应 / RACBF 松弛自适应），时变函数本身当作**带辅助动力学的 HOCBF**（由 CLF 镇定）⇒ 原优化问题可行则 QP 可行性**有保证**；覆盖控制界时变与动态噪声（巡航控制案例） |
| **3880** (Automatica'18) | **control-sharing 性质**：多个 CBF 的约束交集非空的**充要条件**；用于 input–output 线性化系统 + 多时变输出约束，QP 可行性由 control-sharing 保证，跟踪目标在不冲突时达成 |
| **4509** (Automatica'22) | 给 QP 可行的**充分条件**，并把它压缩成**一条由 CBF 施加的额外约束**（与既有约束恒兼容，故不会毁掉可行域）——专治紧控制界 + 高相对度约束 |
| **2267** (Automatica'18) | 改 QP 结构：barrier 约束不激活时保证 $\dot V<0$，并在标准最小假设下给**局部渐近稳定**；**去掉 CBF 相对度** 1 的假设；给出**闭式**控制律（不必在线解 QP），两参数调"激进程度" |
| **4502** (Automatica'24) | 不确定性（最坏情形/概率误差描述）下 CLF-CBF 变成 **SOCC（二阶锥）**：分析其**可行性**与各类控制器的**正则性** |
| **1180** (Automatica'26) | 多智能体：引入**截断函数**实现通信边集的平滑增删，CBF 只在**边界区域**需要、且**放宽可行性条件**（每子系统局部约束）；分布式局部 Lipschitz 安全控制律 |
| **3084** (TAC'26) | 分布式 **CLF-CBF-QP**：编队目标写成分布式 CLF 条件、安全写成 CBF 条件，并给**控制器可行的充分条件**；允许临时偏离队形躲避危险后自动恢复 |
| **1537** (Automatica'24) | 时间驱动重采样导致 QP 序列**逐步 infeasible**：改用 **event-triggered / self-triggered** 触发下一个 QP，既消除不可行又省通信（CAV 交叉口） |
| **4527** (Automatica'25) | 执行器约束在**凸多胞体**时的两个新构造：**soft-min barrier**（用已知 backup 控制做有限时域预测）与 **soft-max barrier**；LP/QP 仿射约束下**可证可行**、控制连续 |

**小结**：可行性问题在 KB 里有 4 条互不相同的技术路线——松弛化（1649/1368/4527）、
结构性的"恒兼容附加约束"（4509）、**理论判据**（3880 control-sharing / 3084 / 1180）、
以及**采样触发时机**（1537）。这四条路线在同一篇里很少被统一比较，是一个明显的空白。

---

### C. 不期望平衡点（undesired equilibria）：CBF-QP 会把任务"钉死"

**问题**：CBF 硬约束 + 名义性能目标（CLF）在安全集内部会产生**新的平衡点**，且可能**渐近稳定**；
agent 停在障碍前不动、又满足所有约束——安全却失效。这是 CBF 版的"局部极小"。

| doc | 结论 |
|---|---|
| **4531** (Automatica'24) | 对一般非线性控制仿射系统，**刻画**闭环平衡点的存在性与位置、解析给出设计参数的影响；给一个**只改 QP 的修补**（不加新假设）——保留前向不变性，**完全消除**安全集内部与一类边界平衡点，并保证原点局部渐近稳定 |
| **4576** (Automatica'26) | 多 CBF 情形下平衡点的**形成与稳定性**研究：稳定性取决于 **CLF 与 CBF 的几何关系**；提出 **CLF-CBF compatibility**（该配对不引入除 CLF 全局极小外的稳定平衡点） |
| **4558** (Automatica'26) | 给定**同一个安全集**，取不同 CBF（无穷多种）时闭环平衡点/稳定性是否变化？——研究"**CBF 选择的无关性**"，把"选 CBF"从技艺变成可分析对象 |

**与本次调研主题的衔接**：这三篇是
《collision_avoidance_local_min_singular.md》§1（局部极小）在**CBF 语境**下的严格版本，
也是"安全方法自身的病"，而不是"势场方法的病"。

---

### D. 不确定性、扰动与自适应安全（数量最多的一簇）

**问题**：真实系统有参数误差、匹配/非匹配扰动、未知动态、时变控制界；鲁棒 CBF 一保守就不可行，
自适应 CBF 一松弛就丢保证。

| doc | 处理的不确定性类型 | 手段 |
|---|---|---|
| **2267** (Automatica'18) | 有界扰动 | **Robust-CBF** + ISS-CLF，闭式 |
| **1368** (TAC'22) | 时变控制界 + 动态噪声 | PACBF / RACBF |
| **561** (Automatica'24) | 参数未知 + CLF/CBF **冲突** | 触发式 **Batch Least-Squares Identifier**（BaLSI）+ 松弛项；避免估计发散/过参数化；安全触发条件排除 Zeno |
| **1793** (TAC'25) | 匹配 + 非匹配 | 不确定性估计 + 误差上界 + HOCBF → SOCC；实验 |
| **3335** (TAC'26) | **一般有界扰动（匹配/非匹配、可微/不可微）** | **DRCBF**：knowledge-based（kDRCBF）与 **reciprocal-compensated（rDRCBF，无需扰动信息）**；明确批评现有 robust CBF 只能处理匹配扰动或依赖扰动可微 |
| **745** (Automatica'25) | strict-feedback + **非匹配扰动** + 可能未知动态 | **Proxy CBF**：拆"代理子系统（CBF）+ 虚拟跟踪子系统（Lyapunov）"，模块化；只需代理子系统模型；滤波型扰动观测器 |
| **2822** (Automatica'24) | 未知动态 | **RISE 型扰动观测器**（指数辨识）+ 优化型安全控制，随辨识进程**扩大可运行区域**；给控制律局部 Lipschitz 条件 |
| **1232** (TAC'24, note) | 扰动输入矩阵**非线性**（PDE 难解/相对度不存在） | **I&I 型扰动观测器**，误差全局一致最终有界；再与 CBF 结合成 **filter-based 安全控制** |
| **4505** (Automatica'27) | 参数不确定 + 扰动（**离散时间**） | **robust adaptive barrier certificate**：对任意在线参数估计算法成立；估计模块与安全滤波器**可分开设计**；给实时性与固有鲁棒性 |
| **2176** (Automatica'26) | 参数不确定 strict-feedback | CBF + BLF co-design（见 A 簇） |
| **4009** (Automatica'25) | 动力学不确定 + 姿态禁区/必守区 | dynamic identifier + 滑模 actor-critic RL + **robust CBF-QP** |
| **3172** (TAC'26) | 扰动 + 障碍 | 扰动观测器 + CBF；**可调 funnel**：避碰动作触发时**主动放宽** funnel 边界，事后恢复队形 |
| **1426** (Automatica'23) | 输入受限 + 有界扰动（高相对度） | RCBF 三构造 + 切换施加 |

---

### E. 离散时间 / 采样数据 / 事件触发

**问题**：CBF 定理几乎都是连续时间；数字实现要做零阶保持/离散化，
于是出现"相对度被改变""逐步可行性丢失""事件间隔内违约"。

| doc | 处理对象 | 关键点 |
|---|---|---|
| **4553** (Automatica'26) | 任意相对度、**相对度可逐点变化** | GD-CBF + 安全预测时域 |
| **4451** (TAC'23) | 采样数据 + 时钟异步 | 高相对度安全集在对抗、输入受限下仍前向不变 |
| **2116** (TAC'23) | **未知动力学**的多智能体 | 自适应仿射动力学（按实时误差更新）+ **事件触发**，降低在线辨识/通信负担 |
| **1537** (Automatica'24) | CAV 交叉口 QP 序列 | 事件/自触发消除时间驱动采样引起的不可行 |
| **4548** (Automatica'26) | 离散时间 + 一般性能目标 | **robust predictive CBF**（slack 最小化问题的最优值函数）+ warm-start 下降约束 ⇒ 目标集**鲁棒渐近稳定**；证连续性；空间交会/换道 |
| **4505** (Automatica'27) | 离散时间 + 参数不确定 | robust adaptive barrier certificate（含"固有鲁棒性"） |
| **4592** (Automatica'22) | 固定时间 + 输入约束 | QP 框架下的时空约束（见 A 簇） |
| **3395** (Automatica'26, brief) | **量化多执行器 / 采样数据** | **Discrete Control Barrier Proximal Dynamics**（barrier + 收缩理论/近端动力学），是"离散 CBF 不靠 QP"的一条新路 |

---

### F. 随机系统与概率安全（含一次"同行纠错"事件）

| doc | 内容 |
|---|---|
| **4565** (Automatica'21) | 高斯过程 + 量测噪声下的 CBF：reciprocal/zero 两类构造 ⇒ **概率 1 安全**；扩到高相对度、线性动力学 + 仿射安全约束；状态需估计（估计落在界内时仍概率 1） |
| **4566** (Automatica'25) | **评论**：指出上文 Theorem 3（以及依赖同一技巧的 Theorem 2）证明有缺陷 |
| **4496** (Automatica'25) | 作者**回复**：承认 Theorem 3 错误，给出 Theorem 2 的修正证明 |
| **4507** (Automatica'21) | **有限时域**随机系统验证：barrier certificate 条件（期望的时变上界）+ **SOS** 优化，给出更紧的越界概率界（连续/离散时间） |
| **4497** (Automatica'22) | **连续时间随机混杂系统网络**的组合式 CBF：子系统算 pseudo-barrier，用 **small-gain** 条件组合；保证有限时域内满足自动机逻辑规范（概率意义） |
| **4541** (Automatica'26) | 随机网络化系统**安全共识**：用 **ECBF** 把 reach-avoid-stay 翻译成"以概率安全共识"，给**精确概率下界**；用 **SDP** 代替 QP（并指出在线 QP 框架可能算不出控制器） |
| **3136** (Automatica'24) | 随机**时滞**系统安全镇定：Krasovskii 路线，给 stochastic CL-Krasovskii functional 与 **stochastic control barrier-Krasovskii functional**，用滑模把两者合并 |

> 这组文献提醒一个容易忽视的事实：KB 里包含**已被证伪的定理**（4565 Thm 3）与**修正答复**（4496）。
> 引用 stochastic CBF 时必须以 4566/4496 为准。

---

### G. 时滞 / 混杂 / 不连续 / 非光滑安全集

| doc | 情形 | 贡献 |
|---|---|---|
| **1392** (Automatica'22) | 非线性**时滞** | Razumikhin 与 Krasovskii 两条路线各给 CLF/CLBF（functional），把"镇定 + 安全"用滑模面泛函合并，允许**闭式**控制器 |
| **3136** (Automatica'24) | 随机时滞 | Krasovskii + 随机 barrier-Krasovskii functional（见 F） |
| **4497** (Automatica'22) | 随机混杂 + 自动机规范 | 组合式构造 + small-gain（见 F） |
| **4580** (Automatica'26) | **$\dot x$ 不连续 + 安全集非光滑**（0-超水平集任意并/交嵌套） | 指出"只满足逐点激活的约束"**不一定**安全——连续性论证被打破；提出 **transition functions**（把**未激活**约束也纳入、但不保守），允许轨迹从一个连通分量"过渡"到另一个；控制器 = 凸优化解，给可行性与连续性 |
| **4534** (Automatica'25) | **微分包含** + 无界安全域 + 非光滑 barrier | 区分 **uniform / non-uniform robust safety**，只用 barrier 与右端项给充分条件；应用于安全自触发控制、快慢动力学、assume-guarantee 合同 |
| **4559** (Automatica'26) | **切换系统** | 安全关键最优控制 + 弱拓扑；给切换系统安全的充要条件；**single barrier function (SBF)**：不要求每个子系统在**整个**安全集内安全 |
| **4509/4514** | 见 B / K | — |

---

### H. 分布式多智能体：连通性、编队、避碰、共识

| doc | 任务 | CBF 的角色 / 难点 |
|---|---|---|
| **1531** (Automatica'23) | 全局连通性保持 | **代数连通度（Laplacian 特征值）当 CBF**，本质**非光滑**（特征值重数变化）；用集合值分析 + 非光滑分析 + 图论给正则性，使约束能与其他约束平滑共存；控制连续；资源收集实验 |
| **1180** (Automatica'26) | 二阶 MAS（共识/编队/位置交换） | 截断函数 + 平滑增删链路；CBF 只需在边界区域、局部可行性（见 B） |
| **3084** (TAC'26) | 反应式编队 | 分布式 CLF-CBF-QP + 可行充分条件；允许临时偏离队形 |
| **3172** (TAC'26) | 空地集群编队机动 | funnel + 扰动观测器 + CBF；避碰时**放宽 funnel**，事后恢复 |
| **1497** (TAC'22) | 多 Euler–Lagrange 系统**分布式最优协调** | **safety barrier certificate（路径积分闭式）** ⇒ 可证无碰撞；凸优化 + 自适应非线性控制 |
| **4540** (Automatica'27) | 无完美状态信息的避碰 | **分布式 ISS 观测器** + CBF，仅用局部量测/通信；扩到 LTI MAS（相对/绝对信息） |
| **4567** (Automatica'24) | 有限通信半径 + 避碰 + 连通保持 | PPC + **exponential zeroing control barrier Lyapunov function (EZCBF)**；给相对位置/速度误差的时变性能约束 |
| **1606** (Automatica'25) | 多车"无序"编队合并 | 把纵向间距调节、同车道避碰、横向收敛、邻车避碰**全部编码成 CBF 约束**，分布式约束优化；可行性与收敛性分析 |
| **2055** (Automatica'24) | 移动目标护航（长期任务执行） | 把"接近目标 / 感知邻居避碰"写进**约束**而非代价 + 松弛变量；在线约束优化 |
| **814** (TAC'24) | 分布式协同输出调节 + 避碰 | **双动态 barrier function + 非线性分布式 command governor**；进/出覆盖区时自适应重构队形 |
| **2743** (TAC'16) | 多机器人网络协调（避碰/保持邻近/到达） | **Lyapunov-like barrier function**（recentered barrier + 最大值近似）把多目标编码成**单一** Lyapunov-like 函数 ⇒ 梯度型分布式控制（CBF 之前夜） |
| **4341** (TAC'26) | 异构 MAS 最优输出共识 + 输出须留在安全集 | "两层"：决策层**扩张**约束集的投影优化 + 控制层**收缩**安全集的参考跟踪安全控制器（**时变 CBF**）；小增益 + 时变 CBF 理论；结果显示"优化会推边界、安全会拉回来" |

#### H.1 "分布式 CBF"其实是三个不同的问题（先分清再谈有没有）

| 说法 | 含义 | KB 对应 |
|---|---|---|
| ① **分布式约束** | 安全约束本身是 agent 对（边）或邻居集合上的，天然可分解 | 两两避碰、连接保持：`1180/4567/4540/1497/2055` |
| ② **分布式求解** | 全局 QP 拆成每个 agent 一个**局部** QP，只用自己的量测 + 邻居通信 | `3084`（分布式 CLF-CBF-QP）、`1180`、`4451`（"distributed calculation of control inputs"）、`1531`（分布式优化型反馈） |
| ③ **分布式证书** | 把**全局**不变性证书分解/组合出来（compositional certificate） | `4497`（pseudo-barrier + small-gain 组合网络证书）；`1497`（路径积分闭式 barrier certificate）；`2743`（单一 Lyapunov-like BF 覆盖多目标） |

现实中有"分布式"字样的论文，多数是 ② + ①；③ 才是理论上最难、KB 内最稀缺的一类。

#### H.2 分布式 CBF 的四个真难点

| 难点 | 为什么会坏 | KB 里的对策 |
|---|---|---|
| **局部可行 ≠ 全局安全** | 每个 agent 的局部 QP 都有解，合起来仍可能违约（或大家都"谦让"而停滞） | `3084` 显式给**分布式可行充分条件**；`1180` 把可行性**放宽到局部/边界区域**；集中版判据是 `3880` 的 control-sharing |
| **耦合约束是全局量** | 连通性（$\lambda_2$）、编队形状、共识目标都不属于任何单个 agent | `1531` 用**代数连通度当 CBF** + 非光滑分析；`4341` 用"扩张/收缩两层"把优化与安全解耦；`3172` 用 funnel 吸收冲突 |
| **通信受限 / 拓扑时变** | 有限感知半径下邻居会进出、链路会跳变 ⇒ 约束不连续 | `1180` 用**截断函数**平滑增删边；`4567` 用 PPC + EZCBF 保连通；`4451` 处理时钟异步与采样数据 |
| **信息不对称 / 无全状态** | 相对距离约束需要双方一致的信息；状态只能估 | `4540` **分布式 ISS 观测器 + CBF-QP**（只用局部量测/通信）；`4497` 用 small-gain 组合；`4451` 处理**部分 agent 恶意** |

#### H.3 KB 内明确的三处空白

1. **分布式 HOCBF 缺失**：`3084` 是分布式 CLF-CBF 但相对度低；`1793` 是高阶鲁棒但**单机**；`3172` 分布式 + 观测器但相对度低。**"分布式 + 高相对度 + 输入受限"三件套没有任何一篇**。
2. **分布式 + 输入约束下的可行性理论**：`3084` 给了充分条件但未含紧输入界；`4509/1426/4527` 的可行性工具全是**集中式**的。
3. **分布式 + 伪平衡**：`4531/4576/4558` 的 undesired equilibria 分析只在单机 CLF-CBF-QP 上做；多机"互相谦让导致停滞"（分布式版伪平衡）**无人形式化**。

---

### I. 安全 vs. 性能/优化目标冲突

| doc | 冲突形态 | 处理 |
|---|---|---|
| **4504** (Automatica'16) | 镇定 vs. 安全 | **CLBF**：CLF 与 CBF 可**独立设计**再合并，支持多 CBF（不安全集不唯一）；机械系统 + 移动机器人导航 |
| **3880** (Automatica'18) | 跟踪 vs. 多时变输出约束 | control-sharing ⇒ 不冲突时跟踪达成（见 B） |
| **4570** (Automatica'21) | 最优轨迹 vs. 实时安全 | 用 Hamiltonian 分析得可解最优解（线性/线性化），再用 **HOCBF + CLF-QP** 兜底安全；桥接"最优规划"与"安全控制"；AV 应用 |
| **4514** (Automatica'22) | 稳定 vs. 安全 vs. reach-avoid-stay | **光滑逆 Lyapunov-barrier 定理**：把三类性质统一（见 K）；指出 Lyapunov 与 barrier 条件"看似竞争"其实可在逆定理层面统一 |
| **4521** (Automatica'24) | 跟踪 vs. 安全 | 先设计"无安全考虑"的跟踪控制器，再叠 CBF 安全控制器；提出**性能指标函数量化让步**：不冲突则渐近跟踪，冲突则在安全下尽量减小误差 |
| **2106** (Automatica'23) | 安全 + 探索（MBRL） | **LCBF（Lyapunov-like CBF）**：保留 CBF 的最小侵入性 + 正半定等 Lyapunov 性质 ⇒ 安全探索；可处理更一般的安全约束 |
| **3418** (Automatica'26, 边缘) | 部分可观 + 输入受限 + 最优 | LMI 观测器 + ADP + **recentered robust Lyapunov-like barrier** 进代价函数（不是硬约束 QP） |
| **4608** (Automatica'26, 边缘) | 安全 vs. 最优（零和博弈） | 障碍 barrier 进代价 + 阻尼常数平衡 |
| **4612** (Automatica'26) | 极值搜索 vs. 安全 | **Assignably Safe Extremum Seeking**：用被测 CBF 保持"实际安全"，并给**可指派的安全吸引速率** |
| **1606 / 4341 / 3084 / 2055** | 任务约束 vs. 安全约束 | 全部塞进在线约束优化（见 H） |

---

### J. 输出反馈 / 状态估计 / 观测器

| doc | 问题 | 处理 |
|---|---|---|
| **4540** (Automatica'27) | 无完美状态 | 分布式 ISS 观测器 + CBF-QP（见 H） |
| **4623** (Automatica'26, 边缘) | 观测器给不出**紧的**估计误差界，难以和 CBF 对接 | Savitzky–Golay 微分估计 + **在线时变误差界**（比最坏情形常值界更紧、且计算便宜） |
| **1232 / 2822 / 745 / 1793 / 3172** | 扰动/动态未知 | 观测器（I&I / RISE / filter-based / 估计上界）与 CBF 耦合（见 D） |
| **3418** (Automatica'26, 边缘) | 部分可观 + 安全 | 见 I |

**共性观察**：KB 里"估计器 + CBF"的组合几乎都要求**误差界可解析**（ISS 界、估计误差上界、在线界），
这正是 doc:4623 这篇"看起来不像 CBF"的论文被反复引用的原因。

---

### K. 理论完备性：安全集本身的性质

| doc | 问题 | 结论 |
|---|---|---|
| **4514** (Automatica'22) | Lyapunov 与 barrier 条件"竞争"且缺逆定理 | 给"渐近稳定 + 安全"与"reach-avoid-stay"的光滑**逆** Lyapunov-barrier 定理：性质成立则**存在**光滑函数满足对应条件 ⇒ 两类条件在逆定理层面统一 |
| **630** (TAC'23) | HOCBF 只保证前向不变，**安全集是否稳定**？ | 一致渐近稳定（集合闭不必紧）+ 对输入约束微分包含成立 + ISS（见 A） |
| **849** (TAC'22) | 需要 forward completeness 才算完整 | HOBF 去掉该假设 + 给集合渐近稳定性 + 无需一致相对度（见 A） |
| **4534** (Automatica'25) | 鲁棒安全的**定义**本身就有强弱 | 区分 uniform / non-uniform robust safety；无界安全域 + 非光滑 barrier 下的充分条件（见 G） |

---

### L. 应用牵引（CBF 当"约束翻译器"）

| 场景 | doc |
|---|---|
| 车辆/交通（ACC、交叉口、换道、车道变换、车队合并） | **1649**（ACC 案例）、**1368**（巡航控制）、**1537**（CAV 冲突区）、**1606**（多车合并）、**4622**（CAV 换道，边缘）、**4570**（AV 最优+安全） |
| 航天/卫星 | **1426**（小行星飞越、低推力）、**4009**（航天器姿态，禁区/必守区）、**4548**（空间交会，离散时间） |
| 机器人/机械系统（Euler–Lagrange） | **4511**（EL 的 state+input 约束，ZCBF 构造）、**188**（多球障碍 + 速度约束，内外环 QP）、**1497**（多 EL 协调）、**4504**（机械系统镇定 + 移动机器人导航） |
| UAV / 空地 | **873**（JAS，固定翼 MPF + robust CBF-QP，见附录 C）、**3172**（空地集群） |
| 吊挂负载 / 欠驱动 | **3417**（绳吊负载：CBF + HOCBF + 预测代价的 **SPTM** 轨迹修正；动态障碍、绳索碰撞、状态与工作空间约束；吊车实验） |
| 姿态（刚体） | **849**（刚体姿态 + 互联单元格安全域） |
| 过程/多执行器（量化、采样） | **3395**（量化多执行器 / 采样数据，brief paper） |
| 无线/工业 IoT | 见 §6（4062 仅"安全约束"字面命中，未纳入） |

---

## 4. 一条主线：TAC 的 HOCBF 谱系 vs Automatica 侧对 HOCBF 的"可用性质疑"

$$
\underbrace{1649}_{\text{定义 HOCBF, TAC'22}}
\to \underbrace{630}_{\text{集合渐近稳定 + ISS, TAC'23}}
\to \underbrace{849}_{\text{HOBF: 去 forward completeness / 去一致相对度, TAC'22}}
\to \underbrace{1793}_{\text{非匹配不确定 + HOCBF $\to$ SOCC, TAC'25}}
$$

而 Automatica 侧：

* **3409**（hierarchical CBF + Faà di Bruno，绕开"递归 HOCBF + QP"的计算与保守）
* **4553**（离散时间任意/变相对度 GD-CBF）
* **2176**（**明确不构造 HOCBF**，用 CBF-QP 给虚拟控制 + BLF backstepping，牺牲一点"纯安全"换可行域与跟踪性能）
* **1426**（高阶 + 输入约束下，RCBF 要"可证可行"才算完成，且**切换**施加以减保守）
* **4530**（governor-parameterized barrier：把"参考调节器"与 barrier 合并成 PBF，量化"安全余量 vs 输出调节能量"的折中，面向**局部感知**的时变约束）

### 4.1 所谓"质疑"具体指什么（5 条，附原文出处）

这里的"质疑"**不是**"HOCBF 定理错了"，而是**"HOCBF 在真实输入约束 / 离散实现 / 有限算力下好不好用"**。
KB 内的批判性表述可归为 5 类，各自的原话如下：

| # | 质疑点 | 原文出处（KB） |
|---|---|---|
| ① | **可行域保守 → QP infeasible** | doc:2176 贡献段：co-design "**successfully avoids constructing high-order CBFs which lead to conservative feasible sets of inputs** for ensuring high-relative-degree safety constraints"；doc:1368（TAC，HOCBF 同门）引言："the associated QPs **can easily be infeasible** when both state constraints (enforced by **HOCBFs**) and tight control bounds are involved"；doc:1649 自己也承认："the QPs can easily become infeasible, in particular for problems with many safety constraints and tight control limitations" |
| ② | **计算/推导复杂度** | doc:3409："To **circumvent the computational complexity of traditional high-order CBF methods**, an online Lyapunov-like condition solver is first proposed"（用 Faà di Bruno 公式降低高阶 Lie 导数的解析推导负担） |
| ③ | **离散化让"相对度"前提失效** | doc:4553："the relative degree may be **altered during the time-discretization process** … Additionally, since the relative degree is a local property, it **may vary across the state space**" ⇒ HOCBF 的递归构造前提不再成立，改 GD-CBF |
| ④ | **只保证前向不变，不管安全集稳定性 / 伪平衡** | doc:630（TAC，补的就是这个缺口）、doc:4531/4576/4558（伪平衡与 CLF-CBF compatibility）：安全≠任务能完成 |
| ⑤ | **性能代价（跟踪被压死）** | doc:2176 仿真结论："achieves **better tracking performance** compared with the existing high-order CBF-based method"；doc:1426 用**切换**施加 RCBF（"enforce the RCBF condition only near the boundary"）以减少对轨迹的限制 |

> **口径上的诚实说明（避免过度概括）**：KB 里**只有 doc:2176 一篇明确说"不构造 HOCBF"**；
doc:1368/1649 是同门在**自适应/松弛化**里顺手承认可行性缺陷；
doc:1426/4509/1180/3417 属于"**修**（保可行、降保守）"而不是"**废**"；
doc:3409 批的是"计算复杂度"，doc:4553 批的是"离散化"，doc:4531/4576/4558 批的是"稳定性/伪平衡"。
> 因此更准确的说法是：**Automatica 侧对 HOCBF 的态度是"可用性质疑 + 补救"，而非路线否定**。

**结论（可直接写进论文 related work）**：HOCBF 在 TAC 上被"补完理论"，在 Automatica 上被"追问可用性"。
若你的工作是"高相对度安全约束下的可行性/性能折中"，那么 1649 + 630 + 849 + 2176 + 1426 + 3409 这 6 篇
是目前 KB 里最完整的对照组（另加 4509 的"恒兼容附加约束"作为可行性工具）。

---

## 5. 三张"问题—文献"速查表

### 5.1 按"你想解决什么"索引

| 你的问题 | 先读 |
|---|---|
| 约束相对度 ≥2，QP 老 infeasible | 1649 → 1368 → 4509 → 2176 |
| 安全控制器把 agent 卡住不动 | 4531 → 4576 → 4558（配《collision_avoidance_local_min_singular.md》） |
| 有非匹配/不可微扰动 | 3335 → 1793 → 745 → 2822 → 1232 |
| 分布式多机、通信受限、要连通性 | 1531 → 1180 → 3084 → 4540 → 4567 |
| 离散实现/量化/事件触发 | 4553 → 1537 → 2116 → 3395 → 4505 |
| 随机/概率安全 | 4566+4496（先看纠错）→ 4507 → 4497 → 4541 |
| 时滞/切换/非光滑安全集 | 1392 → 3136 → 4580 → 4559 → 4534 |
| 安全与最优/跟踪的权衡 | 4504 → 4514 → 4521 → 4570 → 2106 |
| 只有部分状态可测 | 4540 → 4623 → 1232 |
| 想论证"安全集是渐近稳定的" | 630 → 849 → 4514 → 4534 |

### 5.2 按 CBF 的"数学角色"索引

| 角色 | 文献 |
|---|---|
| QP 硬约束（标准范式） | 1649, 1368, 3880, 4509, 1537, 3084, 1180 |
| 证书/验证（不产生控制器） | 4507, 4534, 4565, 4497 |
| 进代价函数（软，安全换最优） | 2106, 3418, 4608, 4612 |
| 分布式约束（局部信息） | 1180, 3084, 4540, 1497, 814, 2055 |
| 观测器/估计器的一环 | 1793, 1232, 2822, 745, 4540, 4623 |
| 触发/采样时机 | 1537, 2116, 4451, 4553, 4548 |

### 5.3 按"年份 → 时代主题"

| 时期 | 时代主题 |
|---|---|
| 2016–2018 | **地基**：CLBF（4504）、robust CBF 与去相对度假设（2267）、多 CBF 共存（3880）、Lyapunov-like BF（2743） |
| 2021–2023 | **推广**：HOCBF/HOBF/aCBF（1649/849/1368）、随机（4565/4507/4497）、时滞（1392）、连通性非光滑 CBF（1531）、MBRL 安全探索（2106） |
| 2024–2027 | **体检 + 收口**：伪平衡（4531/4576/4558）、可行性/正则性（4502/4509 后续）、离散/量化（4553/3395/4505）、非光滑/切换/微分包含（4580/4559/4534）、分布式安全共识/编队（4341/3084/3172/4540） |

---

## 6. 被剔除的"barrier"命中（噪声清单与理由）

| 类别 | 代表 doc | 为什么不是 CBF/HOCBF |
|---|---|---|
| Barrier-function 自适应滑模（增益上界用 barrier 表示） | 54, 910, 3132, 3188, 2934, 3133, 781, 1216, 587 | barrier 描述"增益随误差逼近上界增长"，与安全集前向不变无关 |
| Barrier Lyapunov Function（BLF）+ backstepping | 1069, 3285, 1955, 3853, 2805, 2312 | 靠 Lyapunov 有界性而非控制约束映射；**注意 2176 是 BLF 与 CBF 的 co-design，属例外** |
| log-barrier / 罚函数（优化） | 2268, 2886 | 凸优化的内点罚项 |
| reach-avoid 微分博弈（barrier = 半透面 / 安全面） | 731, 733, 4121, 971, 4199 | 博弈论对象，非控制 barrier 函数 |
| 轨迹规划/MPC 里"formal safety"，无 CBF | 3361, 4457, 3373, 3079 | 用可达性/误差界/MPC 保证，不构造 CBF |
| 安全量化/数据驱动（stochastic，无 CBF） | 3369 | 用数据估计越界概率 |
| 采样数据数值实现（无 barrier 语义） | 4616, 4062 | 仅"安全约束"字面命中 |
| 前向完备性（与 CBF 相关的纯理论） | 3876 | Lyapunov-like 前向完备性条件，未用 barrier |
| 其他（滑模离散实现、Nussbaum、时变约束优化） | 1156, 2213, 2710, 2759, 3135, 3390, 3415, 3570, 3213, 3281, 3344, 3434, 3457, 3797, 3337, 2822* | *2822 已纳入（CBF + RISE DOB）；其余无 CBF |

---

## 7. 从这批文献看出去：KB 里明显的空白（可做切入点）

1. **HOCBF 保守性的定量刻画缺失**。doc:2176 只是"不构造 HOCBF"的经验反例，
   没有"高阶约束 + 输入界 ⇒ 可行域体积/保守度"的解析结果。**可用 4527（soft-min barrier）或 4509（恒兼容附加约束）**
   的机制去量化。
2. **伪平衡（4531/4576/4558）与局部极小（APF/flocking）没有打通**。
   两个社区用两套语言描述同一现象（"安全控制器把任务钉死"），KB 里没有任何一篇把 CBF-QP 伪平衡与势场局部极小统一建模。
3. **可行性四路线未被统一比较**：松弛化（1649/1368/4527）、恒兼容约束（4509）、
   control-sharing 理论判据（3880/3084/1180）、触发时机（1537）。缺一篇"统一可行性框架"。
4. **分布式 + 高阶 + 不确定 三者交集很少**：3084 是分布式 CLF-CBF 但相对度低；
   1793 是高阶鲁棒但单机；3172 是分布式 + 观测器但相对度低。**分布式 HOCBF** 是空白。
5. **事件触发 + 高相对度 + 对抗**只有 4451 一篇（TAC'23），且只做采样数据。
6. **随机 CBF 的正确性只有 4496 的修正证明**，且仅覆盖均值型条件；
   Theorem 3 类（概率 1 的 reciprocal CBF）目前**没有**可信版本 ⇒ 该方向要么补证、要么绕开。
7. **非光滑安全集（4580）与切换系统（4559）与连通性 CBF（1531）三者都涉及"集合的分量"**，
   但没有交叉引用式的工作：**切换图上做非光滑 CBF** 尚无人做（KB 内）。

---

## 附录 A：完整清单（61 篇正文 + 2 篇评论/回复）

> 年份取库内 `year` 字段（in-press 论文可能为预计年份）；个别 `title` 在库内被截断，已标注 `…`。

### A.1 IEEE Transactions on Automatic Control（15）

| doc | 年 | 题目 | 处理的问题 | 关键手段 |
|---|---|---|---|---|
| 2743 | 2016 | Distributed coordination control for multi-robot networks using Lyapunov-like barrier functions | 多目标（避碰/邻近/到达）统一编码 | recentered barrier + max 近似 ⇒ 单一 Lyapunov-like BF，梯度型分布式 |
| 1649 | 2022 | High-Order Control Barrier Functions | 相对度 ≥2；QP 不可行 | HOCBF 递归构造 + penalty / parameterization |
| 1368 | 2022 | Adaptive Control Barrier Functions | 时变控制界 + 噪声 + QP 可行性 | PACBF / RACBF（时变函数当 HOCBF 镇定） |
| 849 | 2022 | High-Order Barrier Functions: Robustness, Safety, Performance-Critical Control | forward completeness 假设、一致相对度、奇异性 | HOBF + 集合渐近稳定 + 无奇异 Lipschitz 控制 |
| 1497 | 2022 | Collision-free distributed optimal coordination for multiple Euler–Lagrangian systems | 最优协调 vs. 避碰协同设计 | 路径积分闭式 safety barrier certificate |
| 630 | 2023 | Sufficient Conditions for Uniform Asymptotic Stability and ISS Using HOCBFs | 安全集稳定性与 ISS 缺理论 | 集合（可非紧）UAS + 微分包含 + 比较系统 |
| 2116 | 2023 | Event-Triggered Control for Safety-Critical Systems With Unknown Dynamics | 未知动力学 + 多智能体 + 计算/通信 | 自适应仿射动力学 + 事件触发 CBF-CLF-QP |
| 4451 | 2023 | Adversarial Resilience for Sampled-Data Systems Under High-Relative-Degree Safety Constraints | 部分 agent 恶意 + 采样数据 + 时钟异步 | 高相对度 CBF + 分布式正常 agent 协同 |
| 814 | 2024 | Collisions-free distributed cooperative output regulation of nonlinear MASs | 输出调节 + 避碰 | 双动态 barrier function + 分布式 command governor |
| 1232 | 2024 | Immersion and Invariance-Based Disturbance Observer and Its Application to Safe Control | 扰动输入矩阵非线性（PDE/相对度假设过强） | I&I-DOB + filter-based CBF 安全控制 |
| 1793 | 2025 | Robust CBF using uncertainty estimation with application to mobile robots | 匹配/非匹配不确定性（含高相对度） | 不确定性估计 + 误差上界 + HOCBF ⇒ SOCC；实验 |
| 3335 | 2026 | Disturbance rejection control barrier functions | 现有 robust CBF 只及匹配扰动/要求可微 | kDRCBF（需扰动知识）/ rDRCBF（无需） |
| 4341 | 2026 | Distributed Optimal Output Consensus of Heterogeneous MASs With Safety Constraints | 最优输出共识 vs. 输出须留安全集 | 扩张约束投影优化 + 收缩安全集（时变 CBF）+ 小增益 |
| 3084 | 2026 | Distributed Control Lyapunov and Control Barrier Certificates for Reactive Formation | 编队形状 vs. 安全冲突；分布式 QP 可行性 | 分布式 CLF-CBF-QP + 可行充分条件 + 临时偏离/恢复 |
| 3172 | 2026 | Tunable funnel-based distributed safety-critical formation maneuvering of air-ground swarms | 扰动 + 障碍 + 编队恢复 | funnel + 扰动观测器 + CBF（避碰时放宽 funnel） |

### A.2 Automatica（46）

| doc | 年 | 题目 | 处理的问题 | 关键手段 |
|---|---|---|---|---|
| 4504 | 2016 | Stabilization with guaranteed safety using Control Lyapunov-Barrier Function | 镇定与安全独立设计 | CLBF 合并 CLF + 多 CBF |
| 2267 | 2018 | Robust CBF for constrained stabilization of nonlinear systems | QP 稳定性缺保证；CBF 需一致相对度 1 | 改 QP ⇒ barrier 不激活时 $\dot V<0$；闭式控制；Robust-CBF + ISS-CLF |
| 3880 | 2018 | Constrained control of input–output linearizable systems using control sharing barrier functions | 多 CBF 约束交集可能为空 | control-sharing 充要条件 ⇒ QP 可行性 |
| 4570 | 2021 | Bridging the gap between optimal trajectory planning and safety-critical control | 最优解算不起，CBF 解次优 | Hamiltonian 最优解 + HOCBF/CLF-QP 兜底 |
| 4565 | 2021 | Control barrier functions for stochastic systems | 高斯过程/量测噪声下的安全 | reciprocal/zero CBF ⇒ 概率 1；扩到高相对度与估计状态（**含错误定理，见 4566**） |
| 4507 | 2021 | A barrier function approach to finite-time stochastic system verification and control | 有限时域概率界太松 | 期望的时变上界 + SOS |
| 1392 | 2022 | Razumikhin and Krasovskii approaches for safe stabilization | 时滞系统安全镇定 | CLF/CLBF（functional）+ 滑模面泛函合并 |
| 4511 | 2022 | Safe-by-design control for Euler-Lagrange systems | EL 的 state + input 约束难以构造 ZCBF | 为 EL 定制 ZCBF 构造（多状态约束） |
| 4509 | 2022 | Sufficient conditions for feasibility of optimal control problems using … CBFs | QP 可行性（紧输入界 + 高相对度） | 一条**恒兼容**的 CBF 附加约束 ⇒ 可行 |
| 4514 | 2022 | Smooth converse Lyapunov-barrier theorems … reach-avoid-stay | Lyapunov 与 barrier 条件"竞争"、缺逆定理 | 光滑逆定理统一三类性质 |
| 4497 | 2022 | Compositional construction of CBFs for continuous-time stochastic hybrid systems | 网络化随机混杂 + 逻辑规范 | pseudo-barrier + small-gain 组合 |
| 4592 | 2022 | Fixed-time control under spatiotemporal and input constraints: A QP based approach | 固定时间收敛 + 时空/输入约束 | 吸引域—时间—控制权限关系 + QP（摘要无 CBF 术语，仅据摘要收录） |
| 1531 | 2023 | Nonsmooth CBF design of continuous constraints for network connectivity maintenance | 代数连通度非光滑（特征值重数变化） | 集合值/非光滑分析 + 图论 ⇒ 连续可行约束 |
| 1426 | 2023 | Robust CBF under high relative degree and input constraints for satellite trajectories | 高阶 + 输入受限 + 扰动 ⇒ RCBF 条件可能不可行 | 三种 RCBF 构造（零下水平集恒可行）+ 切换施加 |
| 2106 | 2023 | Safe exploration in model-based RL using CBFs | 学习期安全 | LCBF（Lyapunov-like CBF） |
| 4530 | 2023 | Governor-parameterized barrier function for safe output tracking with locally sensed constraints | 在线传感得到的**时变**约束 + 输出跟踪 | 虚拟 governor + PBF 量化安全/能量折中 |
| 561 | 2024 | Adaptive safety with CBFs and triggered Batch Least-Squares Identifier | 参数未知 + CLF/CBF 冲突 + 自适应律发散 | BaLSI 触发辨识 + 松弛 QP + 防 Zeno |
| 1537 | 2024 | Optimal control of connected automated vehicles with event/self-triggered CBFs | 时间驱动重采样导致 QP 逐步不可行 | 事件/自触发 QP |
| 4531 | 2024 | On the undesired equilibria induced by CBF based quadratic programs | CBF-QP 造出稳定伪平衡 | 平衡点解析刻画 + 只改 QP 的消除法 |
| 4502 | 2024 | Feasibility and regularity analysis of safe stabilizing controllers under uncertainty | 不确定性下 SOCC 的可行性与控制器正则性 | 最坏情形/概率误差 ⇒ SOCC + 正则性分析 |
| 4567 | 2024 | Prescribed performance formation control with connectivity and collision constraints | 有限通信半径 + 连通保持 + 避碰 + 性能 | PPC + EZCBF |
| 2055 | 2024 | Ordering-flexible multi-robot coordination for moving target convoying | 长期任务执行中的避碰与护航 | 任务写进约束 + 松弛变量（在线约束优化） |
| 2822 | 2024 | Adaptive safety with a RISE-based disturbance observer | 不确定动态下 CBF 过于保守 | RISE DOB 指数辨识 + 扩大可运行区域 |
| 3136 | 2024 | Safe stabilization for stochastic time-delay systems | 随机 + 时滞的安全镇定 | Krasovskii：CL-Krasovskii + barrier-Krasovskii + 滑模 |
| 4521 | 2024 | Safety critical control design for nonlinear system with tracking and safety objectives | 跟踪与安全的让步规则 | 跟踪控制器 + CBF 安全控制器 + 性能指标函数 |
| 4527 | 2025 | Soft-minimum and soft-maximum barrier functions for safety with … | 执行器约束（凸多胞体）下的连续安全控制 | soft-min（backup 控制预测）/ soft-max + 可证可行 LP/QP |
| 745 | 2025 | Proxy control barrier functions | strict-feedback + 未知动态 + 非匹配扰动 | 代理子系统 CBF + 虚拟跟踪 Lyapunov + 滤波 DOB |
| 188 | 2025 | Continuous safety-critical control of Euler–Lagrange systems … | 多球障碍 + 速度约束 + QP 可能不连续 | 内外环：外环 CBF-QP 出速度参考，内环能量型速度跟踪 |
| 4534 | 2025 | Sufficient conditions for robust safety in differential inclusions using barrier functions | 鲁棒安全定义强弱混杂 | uniform / non-uniform robust safety 的 barrier 充分条件 |
| 1606 | 2025 | C2TE: coordinated constrained task execution … multi-vehicle platoon merging | 无序编队合并 + 避碰 + 可行收敛 | 各子任务全编码成 CBF 约束的分布式优化 |
| 4009 | 2025 | Data-driven identifier–actor–critic learning for cooperative spacecraft attitude tracking | 不确定动力学 + 姿态禁区/必守区 | dynamic identifier + 滑模 actor-critic + robust CBF-QP |
| 4566 | 2025 | Comment(s) on "CBFs for stochastic systems" | 上文 Theorem 3/2 证明有缺陷 | 评论 |
| 4496 | 2025 | Author's Reply to the Comment(s) | 定理错误确认与修正 | 修正 Thm 2 证明 |
| 1180 | 2026 | Distributed safety-critical control of nonlinear multi-agent systems | 时变/位置相关通信、局部可行性 | 截断函数平滑增删链路 + 边界区域 CBF + 局部 Lipschitz 律 |
| 3409 | 2026 | Output tracking and safety-critical control for affine nonlinear systems | 高相对度输出跟踪（软）+ 安全（硬）；HOCBF 计算复杂 | 双超水平集 + 分级 CBF + Faà di Bruno + 在线 Lyapunov-like 求解 + 松弛 |
| 3417 | 2026 | Safe motion planning of underactuated rope-suspended payload | 吊挂负载多约束实时安全规划 | CBF + HOCBF + 预测代价的 SPTM + 定制松弛 |
| 2176 | 2026 | CBF-based safety design for adaptive control of uncertain strict-feedback systems | HOCBF 高阶可行域太保守 | **不建 HOCBF**：CBF-QP 出安全虚拟控制 + BLF backstepping |
| 4576 | 2026 | On the stability of undesirable equilibria in the QP framework for safety-critical control | 伪平衡的稳定性来源 | CLF-CBF 几何关系 + **compatibility** 概念 |
| 4558 | 2026 | Independence of closed-loop equilibria and stability from the choice of CBF | 同一安全集不同 CBF 会不会改变闭环行为 | CBF 选择无关性分析 |
| 4553 | 2026 | Enhanced safe control for arbitrary relative degree: generalized discrete-time CBF | 离散化改变相对度；相对度逐点变化 | GD-CBF + 安全预测时域 |
| 4548 | 2026 | A multiobjective approach to robust predictive CBFs for discrete-time systems | 离散时间：安全 + 一般最优目标 | slack 最小化的最优值函数当 robust PCBF + warm-start 下降约束 |
| 4541 | 2026 | Safe consensus control for stochastic networked systems | 随机网络的安全共识概率保证 | ECBF ⇒ reach-avoid-stay + SDP（替代在线 QP） |
| 4580 | 2026 | Safety-critical control of discontinuous systems with nonsmooth safe sets | $\dot x$ 不连续 + 安全集非光滑：逐点约束不足 | transition functions（纳入未激活约束）+ 凸优化、可行与连续 |
| 4559 | 2026 | Safety-critical optimal control of switched systems | 切换系统安全（子系统未必处处安全） | 充要条件 + single barrier function |
| 3395 | 2026 | Discrete Control Barrier Proximal Dynamics: quantized multi-actuator and sampled-data systems | 量化多执行器/采样数据下的离散安全 | barrier + 近端动力学/收缩理论（非 QP 路线） |
| 4612 | 2026 | Local practically safe extremum seeking … | 极值搜索（优化）过程的安全 + 吸引速率 | 被测 CBF 保持实际安全 + 可指派吸引速率 |
| 4540 | 2027 | Distributed state estimation-based CBFs for collision-free multi-agent systems | 无完美状态信息的多机避碰 | 分布式 ISS 观测器 + CBF-QP（扩到 LTI MAS） |
| 4505 | 2027 | Robust adaptive discrete-time control barrier certificate | 离散时间 + 参数不确定 + 扰动 | 对任意在线估计算法成立的 barrier certificate；估计与安全滤波器解耦 |

### A.3 边缘相关（未计入 61，5 篇）

| doc | 年 | 刊 | 为什么算"边缘" |
|---|---|---|---|
| 4623 | 2026 | Automatica | 非线性估计 + **在线误差界**（为与 CBF 对接而生），本身不构造 CBF |
| 4622 | 2026 | Automatica | 换道最优控制 + CBF 做鲁棒安全兜底，主题是交通优化 |
| 3418 | 2026 | Automatica | 输出反馈 ADP，安全靠"Lyapunov-like barrier 进代价"而非 CBF-QP |
| 4608 | 2026 | Automatica | 安全零和博弈，障碍 barrier 进代价函数 |
| 3876 | 2025 | Automatica | 前向完备性的 Lyapunov-like 条件（CBF 理论必需假设之一），不用 barrier |

---

## 附录 B：复现脚本（codemode / okb MCP）

```js
// 1) 元数据检索（并集去重），再按期刊过滤
const queries = ["control barrier function","high-order control barrier","barrier certificate",
                 "safety-critical control","CBF","barrier","forward invariance","safety filter",
                 "safe set","invariance control","safety certificate","zeroing barrier"];
const all = new Map();
for (const q of queries) for (let p = 1; p <= 10; p++) {
  const d = JSON.parse((await tools.mcp__okb__search_info({query:q, page:p, limit:50,
    fields:["id","title","year","journal"]})).content[0].text);
  for (const it of (d.results||[])) all.set(it.id, it);
  if ((d.results||[]).length < 50) break;
}
const sel = [...all.values()].filter(x => /automatica|transactions on automatic control/i.test(x.journal||""));

// 2) 全文补齐（游标翻页，避免漏掉标题里没有 "CBF" 的文献）
for (const journal of ["Automatica","Transactions on Automatic Control"]) {
  let last = 0;
  for (let i = 0; i < 15; i++) {
    const d = JSON.parse((await tools.mcp__okb__grep_search({query:"barrier function|forward invariant",
      journal, start_after_id:last, limit:50, max_context_chars:0})).content[0].text);
    // ...记录 id...
    if ((d.results||[]).length < 50) break;
    last = d.last_id;
  }
}
```

---

## 附录 C：IEEE/CAA Journal of Automatica Sinica（2 篇，另计）

| doc | 年 | 题目 | 说明 |
|---|---|---|---|
| 873 | 2023 | Constrained moving path following control for UAV with robust control barrier function | 固定翼无人机**移动路径跟随**：二审时变控制问题 + 高阶扰动观测器 + **robust CBF-QP** 保证跟随误差不出界 |
| 2157 | 2023 | A survey on the CLF and CBF for nonlinear-affine control systems | 综述：把 CLF/CBF 统一到约束 QP 框架，讨论不确定下的 ISS / ISSf（input-to-state safety） |

---

## 附录 D：与其他调研笔记的关系

* 本篇是 **"CBF/HOCBF 方法论侧"** 的问题地图；
* 《collision_avoidance_local_min_singular.md》是**失效模式侧**的地图（局部极小 / 特殊队形 / QP 不可行）——
  本报告 §3.C（doc:4531/4576/4558）正是其 §1/§3 在 CBF 语境下的严格版本；
* 《formation_control_collision_avoidance_survey.md》《formation_tracking_collision_avoidance.md》
  《swarm_control_agent_collision_avoidance.md》提供编队/集群侧的对照（本报告 §3.H 与之直接对应）。

---

## 附录 E：CBF 对"被控系统"的要求清单（四层）

> 回答"什么系统能上 CBF"这个问题。四层要求按"越往下越容易被忽略"排序；
> "KB 里谁放宽了"只列本报告纳入的 Automatica/TAC 文献。

### E.1 模型层：得是一个"能定义解"的有限维动力系统

| 要求 | 标准 CBF 是否必需 | KB 里谁放宽/处理了 | 不满足时的出路 |
|---|---|---|---|
| 有限维 ODE $\dot x=f(x)+g(x)u$（非 PDE / 非 DAE） | **必需**（KB 内全部框架如此） | 无（doc:4472 等 PDE 系统不进 CBF 框架） | 先降维/近似，或换 MPC/可达性 |
| $f,g$ 局部 Lipschitz ⇒ 解存在唯一 | **必需**（否则"前向不变"无意义） | doc:4580（$\dot x$ **不连续**）、doc:4534（**微分包含**，用 Filippov 解）、doc:4559（**切换**，解不唯一） | 用微分包含/混杂系统语言重写不变性 |
| 前向完备（解对所有 $t\ge0$ 存在） | 一级 CBF 定理常需要 | **doc:849 直接去掉**（HOBF 不需要 forward completeness） | 用 HOBF/HOCBF 的集合稳定性版本，或单独证前向完备（doc:3876） |
| 连续时间 | 常见标准 | doc:4553（离散时间 GD-CBF）、doc:4548（离散预测 CBF）、doc:3395（离散近端动力学）、doc:4505（离散自适应 barrier certificate） | 离散 CBF（注意相对度会变体） |
| 状态可全测 | 标准做法 | doc:4540（分布式 ISS 观测器）、doc:1232/2822/1793（扰动观测器）、doc:4623（在线误差界）、doc:3418（输出反馈 ADP） | 观测器 + **可解析的估计误差界**（这是关键，不是"随便估一下"） |
| 确定性 | 标准 | doc:4565/4507/4497/4541（随机）、doc:3136（随机 + 时滞） | 概率安全（概率 1 / 概率下界），且 doc:4565 的 Thm 3 已被 doc:4566 证伪 |
| 无时滞 | 标准 | doc:1392（Razumikhin / Krasovskii）、doc:3136 | 用 CLF/barrier **泛函** 代替函数 |

### E.2 函数层：安全集与 $h$ 的形状

| 要求 | 说明 | KB 里谁放宽了 |
|---|---|---|
| 安全集写成**单个函数的 0-超水平集** $\mathcal{C}=\{h\ge 0\}$ | 标准形式 | 多个 CBF 分别描述（doc:3880/4504）；**非光滑集的任意并/交嵌套**：doc:4580（transition functions）、doc:1531（代数连通度这种非光滑 $h$） |
| $h$ 至少 $\mathcal{C}^1$（一阶条件用 $\dot h$） | 标准 | doc:1531/4580/4534 明确处理**非光滑 $h$**（需集合值/非光滑分析，且要解决"约束不连续 ⇒ 控制器不连续"） |
| $h$ 沿 $g$ 的 Lie 导数 $L_gh\ne0$（一阶） | 否则一阶 CBF 条件里没有 $u$ | 相对度 $m\ge2$ 全靠 HOCBF/HOBF：doc:1649/630/849/1793/1426/3409/4553 |
| **一致**相对度（相对度在安全集上恒定） | 传统 HOCBF 构造的前提 | **doc:849 明确放松**（relax uniform relative degree + 无奇异控制）；doc:4553（相对度逐点变化） |
| 安全集内点到目标"可达"（$\mathcal{C}$ 里目标可控） | 否则 CBF 只管"不越界"，任务另说 | doc:2176（跟踪与安全可能冲突）、doc:4521（让步规则）、doc:4504/4514（与 CLF 的统一） |

### E.3 输入层：控制怎么进场、边界有多紧

| 要求 | 说明 | KB 里谁放宽/处理了 |
|---|---|---|
| **控制仿射**（$u$ 线性进入，$\dot x=f+gu$） | QP 化的前提；非仿射就变成非线性规划 | KB 内全部如此（这是"行业默认"） |
| 输入约束 $u\in\mathcal{U}$（凸集/多胞体/箱）下 **CBF 条件仍可行** | **最容易被忽视的隐含要求**：$L_fh+\alpha(h)+L_gh\,u\ge0$ 可能在 $\mathcal{U}$ 内无解 | doc:1649（penalty/parameterization）、doc:1368（aCBF）、doc:4509（**恒兼容**附加约束）、doc:1426（RCBF 零下水平集内恒可行 + 切换施加）、doc:4527（soft-min/soft-max barrier，可证可行）、doc:1180（放宽为局部/边界区域可行性）、doc:2176（干脆换掉 HOCBF） |
| 多个约束**同时**可行 | 约束越多越易冲突 | doc:3880（control-sharing 充要条件）、doc:3084（分布式 CLF-CBF 可行充分条件）、doc:4531/4576/4558（冲突的另一种后果：伪平衡） |
| 输入界随状态/时间变化也要成立 | 时变控制界 | doc:1368（PACBF/RACBF） |
| 匹配扰动可被"吃进" $L_f$ | 标准鲁棒化 | 非匹配/不可微扰动：doc:3335（DRCBF）、doc:1793（估计 + 上界）、doc:745（proxy CBF）、doc:2822（RISE DOB）、doc:1232（I&I DOB） |

### E.4 闭环层：安全以外还得"活着"

| 要求 | 说明 | KB 里谁处理了 |
|---|---|---|
| 闭环解有意义（控制器不必连续） | 优化型控制律常不连续，需在**微分包含**意义下论证 | **doc:630 正是为此**（对 HOCBF 诱导的输入约束所对应微分包含证 UAS/ISS，绕开连续性） |
| 安全集本身（集合）渐近稳定 | 否则扰动下会"贴着边界漂" | doc:630、doc:849、doc:4534 |
| **不引入新的稳定平衡点** | 安全了但任务被钉死 | doc:4531/4576/4558（CLF-CBF compatibility） |
| 分布式：局部信息下仍可行 | 每个 agent 只见邻居 | doc:1180/3084/4540（可行性条件显式写出）、doc:1497/1531 |

### E.5 硬门槛（KB 内基本无法在 CBF 框架内绕过）

1. **控制非仿射**：能定义 CBF，但"解 QP"这条路断了（只能解 NLP）。
2. **$u$ 根本不进入约束的某阶导数且相对度无限**（如 doc:1232 中带非线性扰动矩阵、需解 PDE 的情形）：标准递归构造无从下手，得先改造观测/坐标。
3. **解不唯一或状态空间非开集**：需先转到微分包含/混杂框架（doc:4534/4580/4559），不能直接用教科书 CBF。
4. **状态不可测且**给不出**紧的**误差界：CBF 会退化成"凭感觉安全"（doc:4623 的动机正是这个）。
5. **安全集与目标集在输入界内本质冲突**：此时"安全"与"任务"只能取舍（doc:4521/2176/4341），不存在两全的 CBF。

### E.6 自检清单（拿到一个新问题先过 8 条）

1. $\dot x=f+gu$ 是否仿射？$f,g$ 是否局部 Lipschitz？
2. 安全集能否写成**有限个** $h_i\ge0$ 的（并/交）？$h_i$ 是否 $\mathcal{C}^1$？
3. 每个 $h_i$ 的相对度是多少？**是否一致**？（不一致 → 849 的 HOBF）
4. $L_gh_i\ne0$ 吗？若相对度 $m\ge2$，能否接受 HOCBF 的保守性？（不能 → 2176/3409 路线）
5. 输入界 $\mathcal{U}$ 下，CBF 条件**是否处处有解**？多约束**是否 control-sharing**？（→ 3880 / 4509 / 1426 / 3084）
6. 状态是否全测？观测器误差界是否**可解析**？
7. 是否有扰动/时滞/随机/切换？有 → 对应章节 D/F/G 的专用 CBF。
8. 安全与任务冲突时谁让步？让步后会不会产生**伪平衡**？（→ 4531/4576/4558 检查）
