# simulation/result4 — 数值检验（result 4）

对应文档：`../../result4-output-feedback.tex`（第 9 节「可实现性检验与算法诊断」）。

## 结论先说

本目录的数值实验说明：**TAC-2022（知识库 4693）的"输入驱动滤波器 + tan 障碍 + 方向函数"
不能直接移植到本文的未知控制方向合围问题**。可移植的只有其中的**调谐函数** $\phi(t)$。

## 文件

| 文件 | 作用 | 运行方式 |
|---|---|---|
| `sim_result4.py` | **被检方案**（TF+BF+OF，即 tex 的 (2.1)–(2.5)） | `uv run --no-project sim_result4.py` |
| `sim_result4_fixed.py` | 朴素修正：高增益观测器 + 周期滑模律（用于说明"换个观测器也不行"） | `uv run --no-project sim_result4_fixed.py` |
| `_tune.py` | 参数扫描脚本（调试用，可删） | `uv run --no-project _tune.py` |
| `main.m` + 4 个 .m | 被检方案的 MATLAB 移植（未用 uv 路径验证，保留作参考） | — |
| `metrics.txt` / `metrics_fixed.txt` | 脚本输出（论文第 9.2 节的数字来源） | 自动生成 |
| `figures/` | 上述脚本生成的 PNG | 自动生成 |

## 复现的核心数字（`sim_result4.py`）

参数：$N=4$、$m=2$、$T=20$、$dt=10^{-4}$、$\tau=8$、$k_1=k_2=1$、$\kappa_1=1$、$\kappa_2=2$、
$c_1=c_2=20$、$a_1=a_2=1$、$b\in[0.8,1.4]$ 且符号在 $t\approx10\sim19$ 切换、$|w|\le0.35$、
$d=0.45$、$\mu=1.20$。

| 指标 | 定理声称 | 实测 |
|---|---|---|
| $\max |e_1|/\kappa_1$ | $<1$ | $0.9999$（越界 501 次） |
| $\max |e_2|/\kappa_2$ | $<1$ | $0.9999$（越界 3144 次） |
| $\max |u|$ | $\le\rho_2$ | $10^4$（触及限幅） |
| $\mathrm{dist}(p_0,\mathrm{co}(p))$ 后 25\% | $\le\kappa$ | $0.171$（$\kappa_{\rm meas}=14.0$） |
| $\min_{i<j}\|p_{ij}\|$ | $>0.45$ | $0.303$，**碰撞** |
| $\max\|v_i-v_0\|$ | $\le\sqrt m\bar V$ | $63.58$ |

关闭符号切换（`b_switch = [25, 45]`）时：$|e_1|/\kappa_1=0.731$、$|u|_{\max}=765$、
$\mathrm{dist}_{tail}=0.0010$、$\kappa_{\rm meas}=0.72$、$\min\|p_{ij}\|=0.303$、
$\max\|v_i-v_0\|=21.8$。

## 注意

- `sim_result4.py` 中的 `TAN_CLIP`（$0.9999$）与 `U_MAX` 是**诊断用限幅**：
  一旦统计到 `clip events > 0`，就说明该级漏斗已经被越界（数值上表现为 $|e|/\kappa\to1$）。
- `metrics.txt` 里的 `max |e|/kappa = 0.9999` 是限幅值，不是真实值。
- 图 `figures/fig4_funnel.png` 中曲线贴住 1.0 即表示失效。
