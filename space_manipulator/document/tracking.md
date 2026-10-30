# 空间机械臂轨迹跟踪（Space Manipulator Trajectory Tracking）调研报告

> 关注点：**已有文献解决了哪些问题**（问题 → 解法 → 代表文献 → 遗留代价）。
> 与本目录 `capture.md`（捕获三阶段）和 `model.md`（动力学模型）配套阅读。

---

## 0. 检索与取证说明

**数据源**：okb-assist 知识库（`get_stats`：3987 篇，全部 `markdown_done`；article 2433 / journalArticle 497 / conference 261+32 / thesis 82 / book 114）。

**检索方法**（KB 的 `search_info` 是短语/子串匹配，长句会返回 0，故必须拆词；随后用 `grep_search` + `read_markdown` 读正文取证）：

- 短语检索词：`space manipulator(s)`、`space robot(s)`、`space robotic`、`free-floating`、`free-flying`、`on-orbit servicing`、`trajectory tracking`、`reactionless`、`reaction null space`、`zero reaction`、`prescribed performance`、`fixed-time`、`predefined-time`、`finite-time`、`nonsingular`、`terminal sliding mode`、`disturbance observer`、`actuator saturation`、`quantized`、`event-triggered`、`fault-tolerant`、`flexible`、`vibration`、`cable-driven`、`tendon`、`detumbling`、`tumbling`、`manipulability`、`dual-arm`、`model predictive`、`differential evolution`、`convex optimization` 等 60 余个。
- 去重：命中约 130 篇空间机器人相关文献，其中与轨迹跟踪直接相关约 55 篇；同文多副本已合并（如 `KB#1813`≡`KB#3846`、`KB#3505`≡`KB#3507`≡`KB#3508`）。
- **引用链回溯**（"文献中的参考文献"）：对核心文献读 Introduction/Related Work 与 References，把其综述性引用的奠基文献补齐（这些经典文献**多数不在库内**，见附录 B，按"转引自 KB#xxx"标注）。

**标注约定**：

- `[KB#3150]` = 知识库文档 ID，可直接用 `read_markdown {id:3150}` 回查。
- `[→KB#151 之[6]]` = 该经典文献不在库内，是本报告从 KB#151 参考文献表回溯得到。
- 本报告只复述文献**自己声明的贡献与限制**，不做未验证的效果推断。

---

## 1. 问题界定：空间机械臂语境下“trajectory tracking”的四种不同所指

### 1.1 系统工作模式决定了跟踪问题的性质

文献普遍按基座是否受控分为三类 `[KB#151]`、`[KB#1105]`：

| 模式 | 基座姿态 | 基座位置 | 跟踪问题的本质 |
|---|---|---|---|
| 自由漂浮 free-floating | 不控 | 不控 | 强耦合 + 非完整（动量守恒约束）；关节轨迹跟踪为主 |
| 自由飞行 free-flying | 主动控制 | 不控 | 耦合弱化，可做任务空间位姿跟踪 |
| 受控飞行 | 主动控制 | 主动控制 | 退化为地面机械臂问题（但耗燃料） |

`KB#151` 明确指出：在接近/抓捕阶段，**姿态受控的自由飞行模式最常用**，因为末端灵巧性更高；而 `KB#1105`、`KB#3143` 指出为省燃料，任务中通常选自由漂浮模式，即"关节轨迹跟踪控制"。这一区分非常重要：**同一篇论文里的"trajectory tracking"可能指完全不同的被控对象**。

### 1.2 被跟踪的量（任务空间 vs 关节空间 vs 其他）

- 关节空间跟踪：`q → q_d`（最常见，`KB#1105`、`KB#1072`、`KB#1087`）。
- 任务空间/位姿跟踪：`x_e → x_d`，需经广义雅可比矩阵（GJM）映射（`KB#3150`、`KB#184`、`KB#1072`）。
- 全位姿 SE(3) 几何跟踪：避免欧拉角/四元数表示奇异与不唯一性 `[KB#3]`。
- 图像空间跟踪：`s → s*`（视觉伺服），`[KB#925]`、`[KB#754]`、`[KB#3151]`、`[KB#1464]`。
- 位置/力混合：`[KB#3847]`（自适应雅可比位置/力跟踪）。

### 1.3 与地面机械臂的四个结构性差异（这四点定义了"新问题"）

1. **基座-机械臂动力学耦合**：任意关节运动会扰动基座位姿，反之亦然（`KB#3150`、`KB#259`、`KB#2143`）。
2. **非完整性**：零动量自由漂浮下，角动量守恒不可积分，末端位姿是关节**历史**的函数，而非当前关节角的函数（`KB#1784`、`KB#3148`、`KB#1678`、`KB#2093` §1.2.2）。
3. **动态奇异（Dynamic Singularity, DS）**：由系统质量/惯量等动力学参数决定、在工作空间中路径依赖的奇异，地面机械臂的雅可比奇异分析无法预测（`KB#227`、`KB#1209`、`KB#3148`）。
4. **非零动量**：微小残余角动量/线动量会改变动力学与控制律设计（`KB#1207`、`KB#1898`、`KB#3506`）。

### 1.4 性能维度（文献评价"解决了问题"的六把尺子）

收敛时间类别（渐近 / 有限 / 固定 / 预定义-prescribed / 实际预设时间）· 稳态精度 · 抖振（chattering）· 基座姿态扰动幅度 · 约束可行性（力矩/速度/关节限位/避障/输出包络）· 计算量/通信量/可靠性。

---

## 2. 建模层：已解决的问题

### 2.1 耦合动力学的可计算表达（已基本解决）

| 建模路线 | 解决的痛点 | 代表文献 |
|---|---|---|
| 广义雅可比矩阵 GJM | 把末端速度写成关节速度的线性映射，使分解运动速度控制可用于漂浮基座 | `[→KB#151 之[6] Umetani & Yoshida 1989]`、`[→KB#259 之[6]]` |
| 虚拟机械臂 VM | 用"虚拟固定基座"消除基座自由度，便于工作空间与路径分析 | `[→KB#151 之[7] Vafa & Dubowsky 1990]`、`KB#3154`（全文含“virtual manipulator”讨论） |
| 重心/质心（barycentric）公式 | 用系统质心统一刻画动量守恒与运动学，得到固定基座形式的方程 | `[KB#227]` |
| 动态等价机械臂 DEM | 把自由漂浮系统等价成固定基座机械臂，且**允许动力学方程线性参数化**，从而能直接用自适应控制 | `[KB#1813]`≡`[KB#3846]`、`[KB#1072]` 讨论 |
| 递推 Newton-Euler / articulated-body | 把 O(n³) 的 Lagrange 计算降到 O(n)，支持实时/在线优化 | `[KB#1481]`、`[KB#1072]`（螺旋理论 + NE 递推）、`[KB#2093]` 第2章 |
| Kane 方法 | 柔性多体（含挠性附件）建模 | `[KB#3149]`、`[KB#157]` |
| 螺旋理论/Lie 群 + 准坐标 | 避免欧拉角奇异、四元数非最小表示破坏配置空间拓扑 | `[KB#1493]`、`[KB#1072]`、`[KB#3]`（SE(3)） |
| 二阶全驱系统（FAS） | 避免把二阶动力学降阶为一阶状态空间带来的病态矩阵与物理意义丢失 | `[KB#3150]`、`[KB#2085]`、`[KB#3985]`、`[KB#1249]` |
| 等效关节空间约化 | 消去基座加速度，得到 `H_g(q)q̈ + C_g(q,q̇)q̇ = τ`，把问题变回标准关节空间跟踪 | `KB#2093`、`KB#1072` |

**结论**：到 2020s，"如何写出一个既准确、又适合控制器设计的空间机械臂模型"已不是开放问题。真正被反复讨论的是**参数化方式**（避免奇异 vs 保持拓扑 vs 可线性参数化）与**计算量**。

### 2.2 奇异性问题（部分解决）

- **已解决**：动态奇异的存在性、产生机理与预测方法（`KB#227`、`KB#1209`）；无奇异动力学方程（准坐标表述，`KB#1493`）；给定末端轨迹时**如何选取初始构型以规避 DS**（`KB#3148`，平面/空间系统、含/不含初始角动量均适用）；非零动量下的"奇异性鲁棒全位姿工作空间控制"（`KB#3506`）。
- **遗留**：`KB#3148` 的方案是**离线**选择初值，不适用于在线改变任务的场景；DS 与其它约束（避障、力矩饱和、性能包络）的**联合**处理尚未见系统结果。

### 2.3 非零动量与非完整约束（已建立理论工具）

`KB#1207`、`KB#1898` 系统处理了存在非零线动量与角动量时的动力学与控制，`KB#3506` 把"非零动量"纳入工作空间控制，`KB#1784` 则把角动量守恒的**非完整特性当作可用资源**（模仿猫落地反射做基座-机械臂-目标组合体的姿态重定向）。

---

## 3. 跟踪的前置层：轨迹规划已解决的问题

### 3.1 基座姿态扰动抑制（文献最集中的一条主线）

1. **扰动图 DM / 增强扰动图 EDM**：在关节空间画出"扰动最大/最小"的运动方向 `[→KB#797 之[18] Torres & Dubowsky 1992 JGCD 15(4):1010–1017]`、`[→KB#2093 之[47]]`。已解决问题：给出几何直观的扰动最小化轨迹；遗留：每个构型都要做奇异值分解，计算量大；零扰动仅在扰动矩阵奇异时可能。
2. **反应零空间 RNS**：由角动量守恒构造"对基座零反作用"的关节运动流形 `[→KB#2093 之[48] Nenchev, Yoshida & Uchiyama 1996 CDC]`、`[KB#1307]`、`[KB#2047]`、`[KB#3505]`、`[→KB#797 之[19] Sone & Nenchev 2016 Acta Astronautica 128:707–721]`；已在 ETS-VII 上做过零反作用机动的**飞行验证** `[KB#5]`、`[KB#1926]`；`KB#712` 把 RNS 推广到挠性结构安装的机械臂；`KB#454` 用 VFF-RLS + SSADE-ELM 做自适应 RNS 规划与控制。
   已解决：冗余机械臂上"零基座扰动 + 末端跟踪"可同时成立；遗留：**需要运动学冗余**，非冗余机械臂不适用（`KB#2093` §1.2.2 明确批评此点）。
3. **平衡臂/多臂补偿**、**优化类**方法用于非冗余情形：`KB#562`、`KB#184`（把姿态稳定约束写成含时衰减项，同时优化可操作度）、`KB#1489`（7-DOF 最小基座扰动规划）、`KB#301`（直接轨迹优化减小航天器位姿变化）。

### 3.2 优化式轨迹规划（工具已成熟）

- 群智能：PSO `[KB#18]`、差分进化 `[KB#2080]`、`[KB#454]`。
- 凸优化/伪谱：任务约束规划 `[KB#1786]`、伪谱凸规划 `[KB#1516]`、**双臂自由漂浮时间最优路径跟踪**（凸化 + 直接转录，显式处理力矩/速度/加速度约束）`[KB#113]`。
- 多目标：`KB#2016`（预冲击轨迹，自组装重载）、`KB#184`（姿态稳定 + 可操作度）。
- **已解决**：在给定路径/任务下生成满足动力学与执行器约束、且基座扰动小的可行轨迹；**遗留**：多数为离线优化，在线重规划能力弱。

### 3.3 层级/任务优先级与避障（已成为主流范式）

`KB#1643`（加速度级、任务优先级矩阵的层级规划，显式处理基座-机械臂耦合对笛卡尔任务精度的影响）、`KB#2093`（零空间投影 + 动态任务层级重构，实时协同"末端位姿跟踪"与"静态/动态避障"，并把基座姿态稳定与关节限位压到次优先级层）、`KB#3842`（对退化 6-DOF 空间机械臂，用"最小角度调整航天器姿态"换取避障能力）、`KB#3152`（多臂空间机动机器人路径规划与碰撞避免）、`KB#2954`（欧拉-拉格朗日系统带速度约束的自适应饱和避障跟踪）。
**已解决**：多任务冲突下的优先级仲裁与实时避障；**遗留**：任务切换时的抖动（`KB#2093` 自己列为待解决）、高维冗余下的星载算力。

### 3.4 在线/预测式轨迹生成与学习式规划

- NMPC 在线生成（并显式处理表示奇异与计算效率）：`KB#995`。
- 随机/数据驱动预测控制：`KB#2173`（稀疏高斯过程 SGP-SMPC，预捕获阶段的参数不确定/关节摩擦/外扰）、`KB#603`（GP + 事件触发预测，多臂后冲击阶段）。
- 学习式规划：`KB#1678`（预规划 + 后处理，解决"历史依赖"与不确定性）、`KB#500`、`KB#1380`、`KB#472`。
**已解决**：把规划从"离线一次"推进到"在线可适应"；**遗留**：约束保证（可行性/安全性）在学习类方法中通常缺失或仅概率性成立。

### 3.5 非完整姿态重定向（较新且已给出可行解）

`KB#1784`：捕获后基座-机械臂-目标系统（BMTS）姿态不确定，利用角动量守恒的非完整性 + "猫落地反射"启发做姿态重定向规划。已解决：不耗燃料、不加飞轮即可重定向组合体姿态；遗留：文中自陈此类研究"很少"（`[12][13]` 仅两条），且相对姿态需近似不变。

---

## 4. 核心层：轨迹跟踪控制已解决的问题

按"解决了哪个问题"组织。**这是本报告的主干**。

### 4.1 名义模型下的精确控制（问题：耦合与非线性使经典控制失效）

- 分解运动速度/加速度控制（GJM-based resolved motion rate/acceleration），把任务空间指令转成关节指令 `[→KB#151 之[6]]`。
- **分数阶分解加速度控制**：在分解加速度框架内引入分数阶项以提升瞬态/稳态性能并抑制不确定性 `[KB#2143]`。
- 全驱系统（FAS）直接设计二阶控制器，不必降阶、不改物理意义 `[KB#3150]`、`[KB#2085]`。
- 已解决：把"空间"问题化归为"地面"问题（`KB#227` 的奠基结论：几乎任何固定基座算法都可迁移，附加条件只有"测量/估计基座姿态"和"避开动态奇异"）。

### 4.2 参数不确定 + 未知扰动（问题：模型永远不精确）

| 思路 | 代表文献 | 已解决的问题 |
|---|---|---|
| 线性参数化 + 自适应（DEM） | `KB#1813`/`KB#3846`、`KB#3844` | 让自适应控制可用于非线性参数化的漂浮系统 |
| 自适应雅可比（不需基座加速度、不需末端速度） | `KB#1318`、`KB#3847`、`[→KB#151 之[11] Wang & Xie 2009]` | 免去对基座/末端高阶量的测量 |
| 自适应 + 预设性能协调控制 | `KB#1335` | 不确定下的瞬态/稳态包络 |
| 自适应参考模型 + SDRE | `KB#1129` | 不确定非线性系统的调节与跟踪统一框架 |
| 非线性干扰观测器 NDO / ESO | `KB#2019`、`KB#710`、`KB#3150`、`KB#3143`、`KB#3985`、`KB#2984` | 把集总扰动在线估计并前馈补偿 |
| 时延估计 TDE（无模型） | `KB#12`、`KB#1031` | 不依赖精确模型/扰动上界 |
| 复合学习/参数并行学习 | `KB#2983` | 捕获后目标动力学参数未知时的在线辨识 + 无反应控制 |

**已解决**：常值/慢时变参数不确定与有界外扰下的渐近或实用收敛。**遗留**：`KB#2093` §1.2.3 指出，多数观测器设计**依赖扰动上界或扰动导数有界的先验**，且**观测时间不可调**，这直接限制了收敛时间的可设计性。

### 4.3 抖振与不连续（问题：SMC 的工程可用性）

演进的每一步都在解决一个具体缺陷：

1. 常规 SMC → 对被控对象模型要求低、响应快，但抖振大。
2. 终端滑模 TSM → 有限时间，但接近原点时奇异。
3. **非奇异终端滑模 NTSM**：`KB#1486`（含扰动观测器）、`[→KB#259 之[14] Feng, Yu & Man 2002 Automatica 38(12):2159–2167]`；解决奇异，但进入原点邻域后**牺牲有限时间收敛**（`KB#259` 明确指出这一 trade-off）。
4. **分数阶 NTSM**：`KB#259`（分数阶积分项保证全阶段有限时间 + 有限时间干扰观测器抑制抖振）、`KB#2984`（扰动下分数阶滑模）、`KB#2143`。
5. **时间延迟对数滑模**：`KB#1031`，用对数型滑模面 + TDE 同时抑制抖振与模型依赖。
6. **模糊/神经网络补偿**：`KB#432`（T-S 模糊补偿 SMC）、`KB#2077`（模糊 SMC）、`KB#1463`（神经积分 SMC）、`KB#1072`（RBFNN + 无抖振固定时间 SMC）。
7. **强化学习调参**：`KB#3843`（RL-SMC，并用 LuGre 摩擦模型提升建模精度）。

**已解决**：抖振幅度可压到工程可接受区间；**遗留**：无抖振 ⇔ 严格有限/固定时间通常是矛盾的，目前多靠"连续化 + 观测器补偿"折中。

### 4.4 收敛时间：四代指标（这是近五年最密集的战场）

| 代 | 问题 | 代表文献 | 关键点 |
|---|---|---|---|
| 渐近 | 无限时间才收敛，不满足在轨时效 | `KB#1170`（实验验证的自适应鲁棒）、`KB#1335`、`KB#2077` | 工程上够用，但无法给出时间承诺 |
| 有限时间 | 收敛时间随初值变化，无法预估上界 | `KB#1466`、`KB#1486`、`KB#259`、`KB#103`（双腕有限时间 SDRE）、`KB#2093` §1.2.3 综述 | 齐次性/终端滑模/幂积分三条路线 |
| 固定时间 | 上界与初值无关，但依赖固定时间稳定性定理、收敛速度仍可更快 | `KB#1072`、`KB#1087`、`KB#1105`、`KB#1478`、`KB#1654`、`KB#2093` 第4章、`KB#2197` | 三类：加幂积分 / 齐次 / 固定时间终端滑模 |
| 预定义/预设时间 | 收敛时间由**设计者指定**，且要兼顾约束与可实现性 | `KB#3`（SE(3) 几何动态面）、`KB#3984`（绳驱刚柔耦合 + NN）、`KB#3985`（FASA + ESO，时间可调）、`KB#1249`（腱驱，practical prescribed-time）、`KB#1087`（preassigned fixed-time）、`KB#2093` 第5章（指定时间可达性能函数） | 从"无初值依赖"走向"用户可指定"，并开始处理"非光滑/增益奇异"的工程代价 |

**已解决**：把"什么时候收敛"变成设计参数；**遗留**：`KB#2093`、`paper0` 关注的"终端时刻增益奇异/控制跳变"问题，只有部分文献（`KB#3` 的几何动态面、`KB#1031` 的对数滑模、`KB#1087` 的通用障碍函数）给出规避手段。

### 4.5 约束：执行器饱和、量化、输出/性能包络、关节限位

- **输入饱和**：辅助系统/饱和函数/抗饱和补偿 `KB#1466`、`KB#259`、`KB#1123`（推力上限形式化为饱和函数）、`KB#3143`（饱和 + 输出反馈 + 故障）。
- **输入量化**：`KB#1087` 提出"预分配固定时间性能函数（PFPF）+ 通用障碍函数"，同时处理量化输入与输出约束（属少数）。
- **输出/状态约束与预设性能 PPC**：BLF 类 `KB#151`（全状态反馈 + 输出反馈，RBFNN 逼近）；非对称边界 + 用人工势函数**动态调整边界以规避 PPC 的"脆弱性"（fragility：初始误差稍越界即失效）** `KB#3143`；`KB#1335`、`KB#1072`、`KB#1478`、`KB#1654`、`KB#2093` 第5章。
- **关节限位/避障嵌入控制**：梯度型最优方法把约束写进代价函数再沿梯度修正增益 `KB#3150`（作者自陈比 BLF/势函数计算更省）、`KB#2954`；任务优先级法 `KB#1643`、`KB#2093`。
- **速度不可测**：`KB#151`（输出反馈）、`KB#3143`（ESO + 输出反馈）、`KB#2049`（无速度测量的挠性机械臂）。

**已解决**：单一或两类约束下的稳定性证明已相当成熟；**遗留**：`KB#3143`、`KB#1072` 都指出，**多类约束同时存在**（饱和 + 故障 + 输出约束 + 量化）时的保守性与可行性仍是难点。

### 4.6 故障容错（问题：执行器故障、关节自由摆动）

- 执行器故障 + 预设性能 + 固定时间：`KB#1105`（自由漂浮，故障下仍保证固定时间收敛与包络）。
- 输出约束 + 固定时间神经自适应 FTC：`KB#1478`。
- 随机系统 + 故障 + 事件触发 + 弹性 PPC：`KB#1654`。
- 输出反馈 + 故障 + 饱和 + 非对称 PPC：`KB#3143`。
- **自由摆动关节失效**下的容错**运动规划 + 控制**（用"可控度"指标）：`KB#255` —— 这是少数把"故障"从控制器层延伸到规划层的工作。
- 绳驱空间机械臂 FTC + PPC：`KB#3987`。

**已解决**：把故障当作集总扰动/未知输入补偿，在给定故障模式下仍保证收敛；**遗留**：故障**检测与重构**（而非预设故障模式）、故障下**规划-控制一体化**仍稀缺。

### 4.7 观测/传感受限与资源受限

- 视觉伺服：eye-in-hand 自适应视觉跟踪（深度无关矩阵 + 目标点位置观测器，目标运动未知）`KB#925`；直接图像视觉伺服 `KB#754`；翻滚卫星适配环的约束视觉伺服 `KB#1464`。
- 图像空间 + 事件触发 + 离策略 RL + 改进干扰观测器（兼顾性能与通信/算力）`KB#3151`。
- 事件触发：`KB#3151`、`KB#1654`、`KB#603`（三篇，说明该方向在空间机械臂上仍**很薄**）。

**已解决**：在"传感器不可靠/通信受限"下仍能闭环；**遗留**：事件触发阈值与预设性能/收敛时间的耦合分析几乎没有。

### 4.8 基座姿态与末端同时受控（reactionless tracking）

- 无反应控制（reactionless）：把末端当作"虚拟基座"、直接以末端变量为广义坐标，从而**不需要解逆运动学、不需要 GJM 导数** `KB#797`；带参数不确定与输入扰动 `KB#775`；自适应零反作用 `KB#3505`、`KB#1490`；预抓取自旋目标 `KB#12`；捕获后目标消旋 `KB#1064`、`KB#2983`、`KB#72`。
**已解决**：不耗燃料即可"末端跟轨迹 + 基座姿态不漂"；**遗留**：一般要求冗余或特定构型，且目标动力学未知时需额外辨识。

### 4.9 MPC 与学习类跟踪

- MPC：`KB#439`（伪线性化扩展状态空间 + Laguerre 函数 + 任务空间滑模变量，处理约束）、`KB#1870`（NMPC 控制系统）、`KB#1202`（LPV 离线 MPC）、`KB#1218`（带避障的非线性 MPC）、`KB#2085`（**分布鲁棒 MPC + FASA**，处理随机不确定与角度/角速度/力矩多约束）、`KB#3543`（物理信息神经网络 PINN + 鲁棒 MPC）、`KB#3986`（过驱动绳驱机械臂 MPC）、`KB#2173`、`KB#603`。
- 学习：`KB#3843`（RL 调 SMC 增益 + LuGre 摩擦）、`KB#3151`（离策略 RL + 事件触发）、`KB#1463`、`KB#1678`、`KB#1380`。

**已解决**：把"约束 + 预测 + 不确定"放进一个优化框架；**遗留**：`KB#3143` 明确指出 **MPC 依赖精确模型**，模型误差会直接降低性能；`KB#1105` 指出 RL **训练依赖大量交互数据，难以在轨部署**；`KB#3150` 指出复杂控制器结构的**星载算力与可靠性**代价。

### 4.10 双臂/多臂

`KB#113`（双臂时间最优路径跟踪，凸规划）、`KB#210`（双臂仿人自由漂浮机器人轨迹优化与控制）、`KB#473`（多臂捕获后冲击建模 + 无反应控制）、`KB#103`（仿人双臂有限时间 SDRE）、`KB#3152`（多臂机动路径规划）、`KB#603`（多臂后冲击预测控制）、`KB#255`。
**已解决**：协调运动学/时间最优；**遗留**：闭链内力/负载分配与跟踪精度的联合设计仍少。

### 4.11 任务阶段视角（与 `capture.md` 对齐）

- **接近/预抓取**：在线轨迹生成 `KB#995`、随机预测 `KB#2173`、RNS + TDE-SMC `KB#12`、DRL 主动跟踪 `KB#1380`、视觉伺服 `KB#1464`。
- **抓捕/接触冲击**：碰撞动力学 + 输出 SDRE 控制器 `KB#2050`、碰撞动力学建模与控制 `KB#1010`、动量传递式抓取 `KB#1298`、多臂冲击建模与无反应控制 `KB#473`、后冲击事件触发预测控制 `KB#603`、绳驱冗余机械臂对翻滚目标的时变约束自适应快速捕获 `KB#179`。
- **捕获后**：目标消旋 `KB#644`、`KB#1064`、组合体稳定 `KB#72`、参数并行学习 + 无反应控制 `KB#2983`、姿态重定向非完整规划 `KB#1784`、预设性能姿态控制 `KB#2093` 第5章。

### 4.12 实验/飞行验证（稀缺资源）

`KB#1926`（ETS-VII 飞行实验）、`KB#5`（零反作用机动的飞行验证并推广到冗余臂）、`KB#1170`（自由漂浮空间机械臂自适应鲁棒控制器的**实验**研究）。除此之外，本次检索到的跟踪控制论文几乎全部只有数值仿真。

---

## 5. 已解决问题总表（问题 → 解法 → 代表文献 → 遗留代价）

| # | 已解决的问题 | 文献给出的解法 | 代表文献 | 遗留代价/边界 |
|---|---|---|---|---|
| P1 | 耦合动力学建模不可用于控制设计 | GJM / VM / DEM / NE 递推 / 螺旋-Lie / FAS | `KB#227`、`KB#1813`、`KB#1481`、`KB#1493`、`KB#1072`、`KB#3150` | 计算量 vs 精度 vs 可线性参数化三者难兼得 |
| P2 | 动态奇异导致笛卡尔运动受限 | DS 理论 + 初值优化规避 + 奇异性鲁棒工作空间控制 | `KB#1209`、`KB#3148`、`KB#3506` | 规避方案多为离线/依赖初值 |
| P3 | 机械臂运动扰动基座姿态 | DM/EDM、RNS、平衡臂、优化、无反应控制 | `KB#797`、`KB#1307`、`KB#3505`、`KB#184`、`KB#5` | 多需冗余；非冗余时只能"最小化"不能"消除" |
| P4 | 参数不确定 | 线性参数化自适应、自适应雅可比、并行学习 | `KB#1813`、`KB#1318`、`KB#2983` | 需持续激励/结构假设 |
| P5 | 未知外扰与未建模动态 | NDO/ESO/TDE/GP | `KB#2019`、`KB#3150`、`KB#12`、`KB#2173` | 依赖扰动上界或导数有界先验 |
| P6 | SMC 抖振 | 分数阶、非奇异终端、对数滑模、模糊/NN 补偿、RL 调参 | `KB#259`、`KB#1486`、`KB#1031`、`KB#1072`、`KB#3843` | 无抖振与严格有限时间常相互冲突 |
| P7 | 收敛时间不可设计 | 有限→固定→预定义/预设时间 | `KB#1466`、`KB#1072`、`KB#3`、`KB#3985`、`KB#1249` | 终端增益奇异/输入跳变；理论多需理想补偿 |
| P8 | 执行器饱和 | 辅助系统/饱和函数/BLF/ESO | `KB#1466`、`KB#259`、`KB#1123`、`KB#3143` | 饱和与性能包络的可行性条件保守 |
| P9 | 输入量化 | 预分配固定时间性能函数 + 通用障碍函数 | `KB#1087` | 仅少量工作，缺与故障/饱和的联合 |
| P10 | 输出/状态约束与瞬态包络 | BLF、PPC、非对称边界 + 势函数调边界 | `KB#151`、`KB#1335`、`KB#3143`、`KB#1072` | PPC 脆弱性、初值敏感 |
| P11 | 速度/加速度不可测 | 输出反馈、ESO、自适应雅可比 | `KB#151`、`KB#3143`、`KB#2049`、`KB#1318` | 输出反馈下约束处理更保守 |
| P12 | 执行器故障/关节失效 | 固定时间 FTC、可控度容错规划 | `KB#1105`、`KB#1478`、`KB#1654`、`KB#255`、`KB#3987` | 故障检测与规划-控制一体化缺失 |
| P13 | 关节限位/避障 | 任务优先级、梯度型最优、调基座姿态 | `KB#1643`、`KB#2093`、`KB#3150`、`KB#3842` | 任务切换抖动、星载算力 |
| P14 | 挠性振动 | 奇异摄动 + 慢/快子系统、VSCMG 作动、模糊 PI + 分数阶 DO | `KB#2049`、`KB#3149`、`KB#1567`、`KB#157` | 挠性 + 接触 + 故障的联合几乎空白 |
| P15 | 关节摩擦 | LuGre 模型 + RL-SMC | `KB#3843`、`KB#432` | 参数辨识与磨损时变 |
| P16 | 目标运动未知（视觉） | 深度无关矩阵 + 观测器 + 自适应 | `KB#925`、`KB#754`、`KB#1464` | 视野/遮挡/照度约束 |
| P17 | 通信与算力受限 | 事件触发 + 干扰观测器 + RL | `KB#3151`、`KB#1654`、`KB#603` | 触发机制与收敛时间未耦合分析 |
| P18 | 接触冲击与捕获后复合体 | 冲击建模、动量传递、无反应控制、消旋 | `KB#2050`、`KB#471`、`KB#473`、`KB#1298`、`KB#1064`、`KB#644` | 冲击瞬间的跟踪中断与恢复缺统一框架 |
| P19 | 双臂/多臂协调 | 凸规划时间最优、SDRE、协调控制 | `KB#113`、`KB#210`、`KB#103`、`KB#3152` | 闭链内力、负载分配 |
| P20 | 在线轨迹生成与约束预测 | NMPC、LPV-MPC、SGP-SMPC、PINN-MPC、分布鲁棒 MPC | `KB#995`、`KB#1202`、`KB#2173`、`KB#3543`、`KB#2085` | 模型精度依赖、实时性 |
| P21 | 摩擦/柔顺/软体/绳驱对象 | 刚柔耦合建模、绳驱预定义时间控制、软体配置控制 | `KB#3984`、`KB#3985`、`KB#179`、`KB#471`、`KB#940` | 理论工具尚不成体系 |
| P22 | 非零动量 | 动量增广模型 + 全位姿工作空间控制 | `KB#1207`、`KB#1898`、`KB#3506` | 与跟踪控制的结合少 |
| P23 | 非完整姿态重定向 | 猫落地反射式规划 | `KB#1784` | 文献极少，约束条件强 |
| P24 | 方法是否真能落地 | ETS-VII 飞行验证、地面实验 | `KB#1926`、`KB#5`、`KB#1170` | 绝大多数论文无实验 |

---

## 6. 文献整体仍未解决的问题（按"是否被多篇文献共同指出"分级）

### 一级（多篇文献共同承认的核心空白）

1. **多源约束的一体化**：饱和 + 量化 + 输出包络 + 故障 + 关节限位同时存在时，`KB#3143`、`KB#1072`、`KB#1654` 各自只覆盖其中 2–3 类，且都承认保守性/可行性未解决。
2. **观测器收敛时间不可调、依赖扰动先验**：`KB#2093` §1.2.3 明确列为待解决（"亟需一种无需集总扰动先验信息、具有普适性与指定时间收敛性的扰动观测器"）。这正是 `paper0` 中 PTDO 的切入点。
3. **收敛时间的工程可实现性**：`KB#2093` 指出固定时间终端滑模"接近平衡点时收敛速度仍有提升空间"；终端时刻的增益奇异/控制跳变被 `paper0` 引言单独点出，文献中仅零星规避（`KB#3`、`KB#1031`、`KB#1087`）。
4. **规划层与控制层的缝**：分层设计导致任务切换抖动（`KB#2093` §1.2.2 第 2 条）、规划可行性不进入控制稳定性证明。
5. **实验/基准缺失**：仅 `KB#1926`、`KB#5`、`KB#1170` 有硬件/飞行结果；没有统一的仿真算例与指标（同一"平面二杆"与"7-DOF"混用，结果不可横比）。

### 二级（单篇明确指出，但尚无系列工作跟进）

6. **动态奇异 + 多约束 + 收敛时间**的联合处理（`KB#3148` 只做离线初值选择）。
7. **非零动量 + 挠性 + 故障**的一体化（`KB#3506`、`KB#2049`、`KB#3143` 各自独立）。
8. **事件触发/量化与预设性能、预设时间的耦合分析**（`KB#3151`、`KB#1654`、`KB#1087` 尚未交叉）。
9. **学习类方法的在轨部署与安全保证**：`KB#1105`（数据需求）、`KB#3150`（结构复杂度/算力）、`KB#3143`（MPC 的模型依赖）分别给出反面证据。
10. **故障检测与重构 + 容错规划-控制一体化**（`KB#255` 是少数例外）。
11. **接触冲击瞬间的跟踪中断与恢复**：`KB#2050`、`KB#473`、`KB#1298`、`KB#603` 各自处理冲击建模或后冲击控制，缺"冲击前-中-后"统一跟踪框架。
12. **绳驱/软体/连续体空间机械臂的跟踪理论**：`KB#3984`、`KB#3985`、`KB#3986`、`KB#3987`、`KB#179` 集中在 2025–2026，属新兴且工具不统一。
13. **非合作翻滚目标同步跟踪的时变约束**：`KB#179`、`KB#1464`、`KB#12` 分别从绳驱/视觉/规划入手，缺统一处理。
14. **多臂闭链内力与跟踪精度联合设计**（`KB#113`、`KB#473`）。
15. **输出反馈框架下的"脆弱性"与可行性条件**：`KB#3143` 提出非对称边界 + 势函数调边界，但只针对单类系统。

---

## 7. 对本课题（预定义时间 PDF + PTDO 路线）的启示

结合 `paper0`（`扰动观测—动力学补偿—误差线性化—周期延迟反馈`）与 `paper1`：

1. **定位选择**：现有预定义时间工作已覆盖 SE(3) 几何跟踪（`KB#3`）、绳驱/腱驱（`KB#3984`、`KB#3985`、`KB#1249`）、固定基座绳驱 FTC（`KB#3987`）。**"自由漂浮 + 等效关节空间约化 + 光滑（无非奇异滑模）+ 可设计收敛时间"** 这一组合尚不多见，`KB#1031`（对数滑模）、`KB#3150`（FASA）、`KB#1087`（通用障碍函数）是最近的邻居，可作对比基线。
2. **最强的 gap 论证链**：`KB#2093` 明说"观测器依赖扰动先验、观测时间不可调"→ `KB#3143`/`KB#1072` 明说"多约束下的保守性"→ `KB#3`/`KB#1031` 只部分规避终端增益奇异。三者叠加即为 PTDO + PDF 的动机，且**逐条有文献背书**，比泛泛说"提高精度"更有力。
3. **必须处理的对手工作**：`KB#1087`（预分配固定时间 + 量化 + 输出约束）、`KB#1105`（固定时间 FTC + 预设性能）、`KB#3985`（FASA + ESO + 可调预定义时间）、`KB#3150`（FASA + NDO + 多约束 + 增益整定）、`KB#3`（SE(3) 预定义时间）。这五篇基本覆盖了"预定义/固定时间 + 约束/故障"的现有边界。
4. **可比性建议**：选一个与 `KB#1072`、`KB#439` 同类的平面二杆算例做"最低可比验证"，再加 7-DOF 空间机械臂（`paper0` 已搭）做工程性验证；指标至少报**收敛时间实测 vs 设计值**、**峰值力矩/是否饱和**、**基座姿态漂移**、**抖振（控制量高频能量）**。这四项恰好对应 §1.4 的六把尺子，也是当前文献最缺的横向可比指标。
5. **可顺带补的空白**：①在 `KB#151`/`KB#3143` 的饱和 + 输出反馈框架下验证 PDF 的鲁棒性；②用 `KB#3148` 的 DS 规避思路检查参考轨迹是否穿越动态奇异；③若时间允许，把 `KB#255` 的"故障下规划"与 PDF 的"故障下跟踪"对比，形成规划-控制一体的讨论。

---

## 附录 A：KB 内核心文献清单（按主题分组，`id | 年份 | 期刊/类型 | 标题`）

**A1 建模/动力学/奇异/动量**
```
227  |1991| IEEE T-RA            | On the Nature of Control Algorithms for Free-Floating Space Manipulators
1209 |1993| ASME JDSMC           | Dynamic Singularities in Free-Floating Space Manipulators
1493 |2011| Acta Astronautica    | Singularity-free dynamic equations of spacecraft-manipulator systems
1481 |1997| IEEE T-RA            | An articulated-body model for a free-flying robot and its use for adaptive motion control
1813 |2004| Robot. Auton. Syst.  | Adaptive control of free-floating space manipulators using dynamically equivalent manipulator model
3846 |2004| Robot. Auton. Syst.  | (同上，副本)
3154 |2016| IEEE TAES            | Hybrid modeling and analysis method for dynamic coupling of space robots
1207 |2017| Front. Robot. AI     | On the Dynamics and Control of Free-floating Space Manipulator Systems in the Presence of Angular Momentum
1898 |2016| IEEE CDC             | Dynamics and control of a free-floating space robot in presence of nonzero linear and angular momenta
3506 |2023| Acta Astronautica    | Singularity-robust full-pose workspace control of space manipulators with non-zero momentum
3148 |2015| IEEE TAES            | Avoiding dynamic singularities in cartesian motions of free-floating manipulators
486  |    | article              | Analysis of a Redundant Free-Flying Spacecraft/Manipulator System
```

**A2 无反应/反应零空间/基座扰动抑制**
```
5    |    | article              | Zero Reaction Maneuver: Flight Validation with ETS-VII ... Extension to Kinematically Redundant Arm
1926 |2003| Int. J. Robot. Res.  | ETS-VII Flight Experiments for Space Robot Dynamics and Control
712  |1999| IEEE T-RA            | Reaction null-space control of flexible structure mounted manipulator systems
1307 |2013| Mechanical Sciences  | Reaction Null Space of a multibody system with applications in robotics
2047 |2012| IEEE ICRA            | End-link dynamics of redundant robotic limbs: The RNS approach
797  |2020| IEEE TAES            | Reactionless control of free-floating space manipulators
775  |2024| IET Control Theory Ap| Reactionless control of FF space manipulators with parameter uncertainty and input disturbance
3505 |2016| IEEE TAES            | Adaptive zero reaction motion control for free-floating space manipulators
1490 |2014| WCICA                | On the adaptive zero reaction control of free-floating space manipulators
12   |2022| article              | TDE-based reactionless augmented adaptive SMC of a space manipulator's pregrasping a target
454  |2019| article              | Adaptive RNS Planning and Control (VFF–RLS / SSADE–ELM)
1064 |2022| Adv. Space Res.      | Adaptive reactionless control for detumbling a target with dynamic uncertainties
2983 |    | Nonlinear Dynamics   | Parameters concurrent learning and reactionless control in post-capture of unknown targets
562  |2018| Engineering Notes    | Novel Synthesis Method for Minimizing Attitude Disturbance of the Free-Floating Space Robots
184  |2020| article              | Trajectory Planning of FFSM With Spacecraft Attitude Stabilization and Manipulability Optimization
1489 |2016| IJARS                | Trajectory Planning of 7-DOF Space Manipulator for Minimizing Base Disturbance
301  |    | article              | Direct Trajectory Optimization of FFSM for Reducing Spacecraft Variation
```

**A3 轨迹规划/避障/层级任务**
```
18   |2015| article              | Trajectory planning of free-floating space robot using PSO
2080 |2018| Adv. Space Res.      | Optimal trajectory planning of FFSM using differential evolution
1786 |2017| JGCD                 | Task-constrained trajectory planning of free-floating space-robotic systems using convex optimization
1516 |2023| Space: Sci. & Tech.  | Pseudospectral Convex Programming for FFSM Path Planning
113  |2023| IEEE TAES            | Time-optimal path tracking for dual-arm FFSM system using convex programming
2016 |2021| IJARS                | Multiobjective preimpact trajectory planning for self-assembling a heavy payload
1643 |2022| Nonlinear Dynamics   | Hierarchical motion planning at the acceleration level based on task priority matrix
2093 |2025| 博士学位论文          | 面向在轨捕获的空间机械臂运动规划与控制方法研究
1356 |2013| 博士学位论文          | 在轨服务空间机械臂运动及任务规划方法研究
3842 |2025| J. Phys. Conf. Ser.  | Obstacle avoidance by adjusting spacecraft attitude at minimum angles
3152 |2018| IEEE TAES            | Path planning and collision avoidance for a multi-arm space maneuverable robot
1784 |2025| IEEE TAES            | Nonholonomic trajectory planning of postcapture BMTS for attitude reorientation
1678 |2025| IEEE TAES            | Learning-based task space trajectory planning for uncertain FFSR
500  |    | article              | Path planning of 6-DOF free-floating space robotic manipulators using RL
1380 |2022| Adv. Space Res.      | Active object tracking of free floating space manipulators based on DRL
```

**A4 跟踪控制（渐近/鲁棒/自适应/观测器）**
```
1170 |2011| Control Eng. Pract.  | Experimental investigation on adaptive robust controller designs (FFSM)
1318 |2012| IEEE TAES            | Prediction error based adaptive Jacobian tracking for FFSM
3847 |2009| Robot. Auton. Syst.  | Adaptive Jacobian position/force tracking control of free-flying manipulators
1335 |2019| Nonlinear Dynamics   | Adaptive coordinated control of uncertain FFSM with prescribed control performance
1129 |2019| Aerosp. Sci. Technol.| Model reference adaptive SDRE: regulation and tracking of FFSM
2143 |2021| Aerosp. Sci. Technol.| Fractional-order resolved acceleration control for FFSM with system uncertainty
2019 |2008| IEEE Systems Journal | Disturbance Observer-Based Robust Control of FFSM
710  |2014| IEEE Systems Journal | Fuzzy Adaptive DOB Robust Tracking Control of Electrically Driven FFSM
2984 |2023| Aerosp. Sci. Technol.| DOB-based fractional-order SMC for FFSM with disturbance
432  |    | article              | T-S Fuzzy Compensation Based SMC for a Free-Floating Space Manipulator
2077 |2023| AI & Digitalization  | Fuzzy SMC Based Trajectory Tracking of Free Flying Space Robot Manipulator
1463 |2023| Adv. Space Res.      | Intelligent Neural Integral Sliding-mode Controller for space manipulator
2197 |2024| Acta Astronautica     | Intelligent and robust control of space manipulator for debris removal
```

**A5 有限/固定/预定义时间**
```
1466 |2020| IEEE T-IE            | Finite-Time Trajectory Tracking Control of Space Manipulator Under Actuator Saturation
1486 |2021| Acta Astronautica    | Nonsingular terminal sliding mode control for FFSM with disturbance
259  |2025| article              | Fractional-Order SMC for FFSM With Disturbance and Input Saturation
1031 |2025| J. Comput. Des. Eng. | Trajectory tracking of FFSM using time-delay logarithmic SMC
1072 |2024| IEEE TAES            | Constrained fixed-time TSMC with prescribed performance for space manipulator
1087 |2025| IEEE TAES            | Preassigned Fixed-Time Control for Quantized Space Manipulator System
1105 |2025| 中国科学:技术科学       | Fixed-time FTC for FFSM with prescribed performance and actuator faults
1478 |2023| Acta Astronautica    | Fixed-time neural adaptive FTC for space manipulator under output constraints
1654 |2025| IEEE TAES            | Error-Triggered Flexible PPC for FFSR Stochastic Systems With Actuator Faults
3143 |2025| Acta Astronautica    | Saturated output feedback control with fragility-avoidance PPC
3    |2024| IEEE TAES            | Predefined-time tracking control of a free-flying space robot on SE(3)
3984 |2026| Aerosp. Sci. Technol.| Predefined-time tracking of a rigid-flexible cable-driven space manipulator using NN
3985 |2026| Acta Astronautica    | Tunable predefined-time tracking for cable-driven space manipulators (ESO + FASA)
3987 |2026| IEEE TAES            | Fixed-Time Fault-Tolerant Tracking for Fixed-Based Cable-Driven Space Manipulators
103  |    | article              | Finite-time SDRE regulation of anthropomorphic dual-arm space manipulator
```

**A6 约束/FAS/MPC/学习**
```
151  |    | article              | Adaptive tracking of a free-flying space manipulator with prescribed performance and actuator saturation
3150 |2024| IEEE TAES            | High-precision tracking for FFSM with multiple constraints and system uncertainties
2085 |2025| IEEE TAES            | Distributionally robust MPC for trajectory tracking based on FASA
1249 |2025| IEEE/ASME T-Mech     | Practical Prescribed-Time Tracking for a Tendon-Driven Space Manipulator via FASA
439  |2019| 控制理论与应用         | 自由漂浮空间机器人轨迹跟踪的模型预测控制
1870 |2017| J. Intell. Robot. Syst.| Control System for FFSM Based on NMPC
1202 |2021| IEEE TAES            | LPV-based offline MPC for free-floating space robots
1218 |2016| Adv. Space Res.      | A non-linear MPC with obstacle avoidance for a space robot
995  |2026| JGCD                 | Online Trajectory Generation for Space Manipulator via NMPC
2173 |2025| IEEE TAES            | SGP-based stochastic predictive control of FFSR in precapture phase
603  |2023| JGCD                 | Data-Driven Event-Triggered Predictive Post-Impact Control of Space Robot
3543 |2026| Aerosp. Sci. Technol.| PINN-based robust MPC for free-floating space robots
3986 |2026| Control Eng. Pract.  | A novel MPC law for over-actuated space cable-driven manipulators
1123 |2020| IEEE TAES            | Optimal Concurrent Control for Space Manipulators Rendezvous and Capturing under Saturation
3843 |2026| IET Cyber-Syst. Robot.| RL-based nonlinear tracking for space flexible manipulator with LuGre friction
3151 |2025| IEEE TAES            | Event-triggered image-space tracking using off-policy RL with disturbance observers
2954 |2025| IJRNC                | Adaptive saturated obstacle avoidance tracking for Euler–Lagrange systems
```

**A7 挠性/软体/绳驱/视觉/多臂/任务阶段**
```
901  |2015| Acta Astronautica    | Observer-based augmented adaptive control of flexible-joint FFSM
2049 |2022| IEEE TAES            | Velocity-free trajectory tracking and active vibration control of flexible space manipulator
3149 |2018| IEEE TAES            | Maneuver and active vibration suppression of free-flying space robot
1567 |2024| Aerosp. Sci. Technol.| Fuzzy PI vibration suppression for double flexible telescopic manipulator
157  |2022| 学位论文              | 带挠性附件的浮动基座空间机械臂系统跟踪控制研究
91   |2016| 学位论文              | 分段约束阻尼层结构及其在空间机械臂减振中的应用
905  |2017| 学位论文              | 空间机械臂传感系统及关节振动抑制研究
471  |2024| Acta Astronautica    | Comprehensive dynamic model and configuration control of soft manipulator-spacecraft
940  |2025| JGCD                 | Modeling and Control of a Hybrid Soft-Rigid Space Manipulator System
179  |2025| IEEE TAES            | Adaptive and rapid capture of tumbling targets by cable-driven redundant manipulator
925  |2017| IEEE TAES            | Eye-in-hand tracking control of a free-floating space manipulator
754  |2016| Aerosp. Sci. Technol.| Direct image-based visual servoing of FFSM
1464 |2025| IEEE TAES            | Constrained visual servoing for capturing adapter rings on tumbling satellites
210  |2022| JGCD                 | Trajectory optimization and control of a free-floating two-arm humanoid robot
473  |2021| Acta Astronautica    | Impact modeling and reactionless control for post-capturing with a multi-arm space robot
255  |2022| article              | Fault tolerant motion planning and control with free-swinging joint failure
2050 |2024| Aerosp. Sci. Technol.| FFSM impacting a floating object: modeling and output SDRE controller
1010 |2017| 学位论文              | 空间机械臂在轨捕获碰撞动力学及控制研究
1298 |2026| Aerosp. Sci. Technol.| Momentum transfer by impact: grasping of non-cooperative space debris
644  |2025| IEEE TAES            | Control Optimization for Space Robots in Target Detumbling
72   |2022| 学位论文              | 空间机械臂抓捕目标后的复合体稳定策略研究
262  |2021| 航空学报              | 空间机械臂技术综述及展望
3844 |1998| thesis               | Adaptive control of free-floating and free-flying robotic manipulators
```

---

## 附录 B：奠基文献清单（seminal works）

按"解决的问题"分四组。**"在库"一列**：`是 KB#xxx` 表示知识库里有全文可读；`否` 表示库内没有，只能从其它论文的参考文献表回溯题录，需另行获取原文（多数可在 IEEE/Elsevier 上按 DOI 直接找到）。

### B1 建模与运动学奠基（1987–1993）

| 文献 | 在库 | 回溯出处 |
|---|---|---|
| Z. Vafa, S. Dubowsky, *On the dynamics of manipulators in space using the virtual manipulator approach*, ICRA 1987, 579–585（期刊展开版：*The kinematics and dynamics of space manipulators: the virtual manipulator approach*, IJRR 9(4), 1990, 3–21） | 否 | `KB#797` 之[17]、`KB#2093` 之[46]、`KB#151` 之[7] |
| Y. Umetani, K. Yoshida, *Resolved motion rate control of space manipulators with generalized Jacobian matrix*, IEEE T-RA 5(3), 1989, 303–314 | 否 | `KB#151` 之[6]、`KB#259` 之[6]、`KB#3148` 之[9] |
| E. Papadopoulos, S. Dubowsky, *On the nature of control algorithms for free-floating space manipulators*, IEEE T-RA 7(6), 1991, 750–758 | **是 `KB#227`** | `KB#151` 之[8] |
| S. Dubowsky, E. Papadopoulos, *The kinematics, dynamics, and control of free-flying and free-floating space robotic systems*, IEEE T-RA 9(5), 1993, 531–543 | 否 | `KB#151` 之[9]、`KB#1064` |
| E. Papadopoulos, S. Dubowsky, *Dynamic singularities in free-floating space manipulators*, ASME JDSMC 115(1), 1993, 44–52 | **是 `KB#1209`** | `KB#3148` 之[3] |
| M. A. Torres, S. Dubowsky, *Minimizing spacecraft attitude disturbances in space manipulator systems*（扰动图 DM 的原始文献）, JGCD 15(4), 1992, 1010–1017 | 否 | `KB#797` 之[18]、`KB#2093` 之[47] |

### B2 零反作用 / 反应零空间（RNS）

| 文献 | 在库 | 回溯出处 |
|---|---|---|
| D. N. Nenchev, K. Yoshida, M. Uchiyama, *Reaction null-space based control of flexible structure mounted manipulator systems*, IEEE CDC 1996, 4118–4123（RNS 原始提出） | 否 | `KB#2093` 之[48] |
| D. Nenchev, *A controller for a redundant free-flying space robot with spacecraft attitude/manipulator motion coordination*, IEEE/RSJ IROS 1993, 2108–2114 | 否 | `KB#259` 之[3] |
| D. N. Nenchev, *Reaction null space of a multibody system with applications in robotics*, Mechanical Sciences 4(1), 2013, 97–112（RNS 综述） | **是 `KB#1307`** | `KB#797` 之[12] |
| K. Yoshida, K. Hashizume, S. Abiko, *Zero reaction maneuver: flight validation with ETS-VII space robot and extension to kinematically redundant arm*, ICRA 2001, 441–446 | **是 `KB#5`** | `KB#2093` 之[49] |
| H. Sone, D. Nenchev, *Reactionless camera inspection with a free-flying space robot under reaction null-space motion control*, Acta Astronautica 128, 2016, 707–721 | 否 | `KB#797` 之[19] |

### B3 被反复借用的控制理论工具

| 文献 | 在库 | 回溯出处 |
|---|---|---|
| Y. Feng, X. Yu, Z. Man, *Nonsingular terminal sliding mode control of rigid manipulators*, Automatica 38(12), 2002, 2159–2167 | 否 | `KB#1466` 之[15]、`KB#259` 之[14] |
| S. Yu, X. Yu, B. Shirinzadeh, Z. Man, *Continuous finite-time control for robotic manipulators with terminal sliding mode*, Automatica 41(11), 2005, 1957–1964 | 否 | `KB#1466` 之[16] |
| Y. Wang, L. Gu, Y. Xu, J. Cao, *Practical tracking control of robot manipulators with continuous fractional-order nonsingular terminal sliding mode*, IEEE T-IE 63(10), 2016, 6194–6204 | 否 | `KB#259` 之[16] |
| J. E. Bobrow, S. Dubowsky, J. S. Gibson, *Time-optimal control of robotic manipulators along specified paths*, IJRR 4(3), 1985, 3–17 | 否 | `KB#113` 之[7] |
| O. Khatib, *Real-time obstacle avoidance for manipulators and mobile robots*, IJRR 5(1), 1986, 90–98 | 否 | `KB#2093` 之[27] |
| A. Polyakov, *Nonlinear feedback design for fixed-time stabilization of linear control systems*, IEEE TAC 57(8), 2012, 2106–2110 | 否 | `KB#2093` 之[88]、`paper0` 之 `polyakov2012` |
| W.-H. Chen, D. J. Ballance, P. J. Gawthrop, J. O'Reilly, *A nonlinear disturbance observer for robotic manipulators*, IEEE T-IE 47(4), 2000, 932–938 | **是 `KB#2985`** | `KB#2093` 之[104] |
| M. H. Korayem, S. R. Nekoo, *Finite-time state-dependent Riccati equation for time-varying nonaffine systems: rigid and flexible joint manipulator control*, ISA Trans 54, 2015, 125–144 | 否 | `KB#1105` 之[18] |

### B4 空间机械臂跟踪控制的关键前驱（2013–2024，"某类问题第一次被做出来"）

| 文献 | 解决的问题 | 在库 | 回溯出处 |
|---|---|---|---|
| N. Kumar, V. Panwar, J. H. Borm, et al., *Adaptive neural controller for space robot system with an attitude controlled base*, NCAA 23(7–8), 2013, 2333–2340 | 不需关节加速度测量的自适应神经网络跟踪 | 否 | `KB#2093` 之[66] |
| S. Guo, D. Li, Y. Meng, et al., *Task space control of free-floating space robots using constrained adaptive RBF-NTSM*, Sci China Tech Sci 57(4), 2014, 828–837 | 任务空间 + 约束 + 自适应 RBF-NTSM 的有限时间跟踪 | 否 | `KB#2093` 之[83] |
| H. S. Jayakody, L. Shi, J. Katupitiya, et al., *Robust adaptive coordination controller for a spacecraft equipped with a robotic manipulator*, JGCD 39(12), 2016, 2699–2711 | 航天器 + 机械臂的鲁棒自适应**协调**控制 | 否 | `KB#151` 之[17]、`KB#2093` 之[72] |
| Z. Zhou, Y. Zhang, D. Zhou, *Robust prescribed performance tracking control for free-floating space manipulators with kinematic and dynamic uncertainty*, AST 71, 2017, 568–579 | SMC + 预设性能（运动学与动力学不确定并存） | 否 | `KB#2093` 之[74] |
| Y. Jia, A. K. Misra, *Robust trajectory tracking control of a dual-arm space robot actuated by control moment gyroscopes*, Acta Astronautica 137, 2017, 287–301 | 双臂 + CMG 作动的鲁棒滑模跟踪 | 否 | `KB#151` 之[20]、`KB#2093` 之[73] |
| B. Zhang, B. Liang, Z. Wang, et al., *Coordinated stabilization for space robot after capturing a noncooperative target with large inertia*, Acta Astronautica 134, 2017, 75–84 | 捕获大惯量非合作目标后的协调镇定 | 否 | `KB#151` 之[19] |
| T. Rybus, K. Seweryn, J. Z. Sasiadek, *Control system for free-floating space manipulator based on NMPC*, JIRS 85, 2017, 491–509 | 把 NMPC 用于自由漂浮空间机械臂控制系统 | **是 `KB#1870`** | `KB#151` 之[18] |
| A. Seddaoui, C. M. Saaj, *Combined nonlinear H∞ controller for a controlled-floating space robot*, JGCD 42(8), 2019, 1878–1885 | 姿态受控漂浮机器人的非线性 H∞ 跟踪 | 否 | `KB#2093` 之[65] |
| R. Jin, P. Rocco, Y. Geng, *Observer-based fixed-time tracking control for space robots in task space*, Acta Astronautica 184, 2021, 35–45 | 幂积分 + ESO 的任务空间固定时间跟踪 | 否 | `KB#2093` 之[95] |
| Q. Yao, *Disturbance observer-based robust fixed-time integrated trajectory tracking control for space manipulator*, Robotica 40, 2022, 3214–3232 | 观测器 + 固定时间的"一体化"跟踪 | 否 | `KB#2093` 之[108] |
| Y. Yan, H. Cui, P. Han, *Fixed-time control for free-floating space manipulators with prescribed constraints and input saturation*, Aeronautical Journal 128, 2024, 1219–1244 | 固定时间 + 预设约束 + 输入饱和（同一作者群后续即 `KB#1105`） | 否 | `KB#1105` 之[19] |
| Y. T. Li, Z. C. Xu, X. H. Yang, et al., *Identification and high-precision trajectory tracking control for space robotic manipulator*, Acta Astronautica 214, 2024, 484–495 | 参数辨识 + 高精度跟踪（ESO-MPC 路线） | 否 | `KB#2093` 之[69] |

### B5 三篇必引综述（写引言时的"领域坐标"）

| 文献 | 在库 | 回溯出处 |
|---|---|---|
| A. Flores-Abad, O. Ma, K. Pham, S. Ulrich, *A review of space robotics technologies for on-orbit servicing*, Prog. Aerosp. Sci. 68, 2014, 1–26 | 否 | `KB#151` 之[1]、`KB#259` 之[1] |
| W.-J. Li 等, *On-orbit service (OOS) of spacecraft: a review of engineering developments*, Prog. Aerosp. Sci. 108, 2019, 32–120 | 否 | `KB#151` 之[3] |
| E. Papadopoulos, F. Aghili, O. Ma, et al., *Robotic manipulation and capture in space: a survey*, Front. Robot. AI 8, 2021, 228 | 否 | `KB#259` 之[2] |

> **最高效的回溯入口（三处）**：
> 1. `KB#2093` §1.2.3 的参考文献表（［64］–［108］），是"空间机械臂轨迹跟踪"最完整的一张引文网络，按渐近→有限→固定时间三条线排好。
> 2. `KB#151` 的参考文献表（[1]–[21]），覆盖"三类工作模式 + 自适应/输出反馈/饱和"这条线。
> 3. `KB#797` + `KB#3148` 的参考文献表，覆盖"无反应控制 + 动态奇异"这条线。
> 统计：本附录 34 条中，库内仅 6 条（`KB#227`、`KB#1209`、`KB#1307`、`KB#5`、`KB#2985`、`KB#1870`），即**这套知识库在"奠基文献"层面覆盖很弱，需要外部补齐**。

### 附：可复现的检索命令示例

```
# 元数据检索（短语匹配，务必拆词）
search_info {query:"space manipulator", limit:100, fields:["id","title","year"]}
search_info {query:"free-floating", limit:100}
search_info {query:"trajectory tracking", limit:100}

# 全文检索（支持正则）
grep_search {query:"prescribed.?time|predefined.?time", limit:20, context:2}

# 读某篇文献的 Introduction / References
get_toc {id:3150}
read_markdown {id:3150, line_start:24, line_count:32}
read_markdown {id:151, section:"References"}
```
