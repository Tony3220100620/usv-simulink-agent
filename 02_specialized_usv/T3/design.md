# 无人船航向 PID 控制 Simulink 模型设计图

> 版本：v1.4（实施定稿版）　｜　状态：**已实现并通过仿真验证**
> 修订记录：
> - v1.1：船体/控制器参数改为读取 `E:\0203\usv_params.m`；取消自动切换状态机，改为固定时刻切换；X=0 原地转向。
> - v1.2：期望航向指令改为 MATLAB Function 块，函数体内直接写 `if t < xxx ... elseif ...` 分段赋值。
> - v1.3：试跑标定完成，回填 T_SW1/T_SW2 与 PID 整定值；模型实际为 13 个模块；docx 由 `gen_report.py`（python-docx）生成。
> - v1.4：图 1 改为展开角连续显示（用户确认）——实际航向无 0°/360° 折返，230° 以等价 -130° 表示；数据文件同时保留 0~360° 映射列。

---

## 1 任务概述

构建无人船航向闭环 PID 控制 Simulink 模型，并用一个 `.m` 脚本一键完成建模、参数配置、仿真、数据处理、docx 实验报告与工作日志输出。

| 项目 | 内容 |
|---|---|
| 模型功能 | 无人船航向闭环 PID 控制（三阶段期望航向，固定时刻切换） |
| 模型文件 | `usv_heading_pid.slx`（14 个模块） |
| 脚本文件 | `build_usv_heading_pid.m` |
| 仿真配置 | 固定步长 ode4，步长 0.01 s，总时长 300 s |

### 1.1 航向指令时序（固定时刻切换，实测标定）

| 阶段 | 时间段 | 期望航向 ψd | 实测收敛时刻 |
|---|---|---|---|
| 阶段 1 | [0, 42) s | 30° | t = 29.93 s |
| 阶段 2 | [42, 104) s | 230° | t = 91.53 s（切换后 49.53 s） |
| 阶段 3 | [104, 300] s | 30° | t = 153.53 s（切换后 49.53 s） |

**切换时刻标定过程：** 试跑（宽松切换 80 s / 200 s）测得阶段 1 误差首次稳定进入 ±0.1° 的时刻 t_c1 = 29.93 s、阶段 2 为 t_c2 = 91.53 s；按 `T_SW = ceil(t_c + 10) + 2`（稳定 10 s + 2 s 余量）取整，得 T_SW1 = 42 s、T_SW2 = 104 s，复跑校验通过（切换前 10 s 窗口最大误差 0.0800° / 0.0767° < 0.1°）。

---

## 2 环境与模块合法性核查（已实测）

### 2.1 usv_lib 模块清单核对（必做项）

已读取 `...\usv_lib\modules.md` 并逐项核对：

| 设计所需模块 | 清单条目 | 文件 | 结论 |
|---|---|---|---|
| 无人船运动模型 | `1. usv_model`（USV.m，S-Function `USV`） | 存在 | ✅ 合法可用 |
| 航向 PID 控制器 | `2. heading_pid`（Heading_PID.m，S-Function `Heading_PID`） | 存在 | ✅ 合法可用 |

**结论：所需模块全部在清单内，无需补充。** 脚本每次运行都会把 `USV.m`、`Heading_PID.m` 复制到 `E:\0203` 后使用。

### 2.2 运行环境实测

| 检查项 | 结果 | 影响 |
|---|---|---|
| MATLAB / Simulink | R2024b，Simulink License 可用 | ✅ |
| MSS 工具箱（USV.m 依赖 `m2c`、`eulerang`） | `D:\MATLAB\R2024b\toolbox\MSS`，脚本自动 addpath | ✅ |
| MATLAB Function 块 | Simulink 自带（普通仿真无需编译器） | ✅ |
| MATLAB Report Generator | 无 License | ⚠ 改用 python-docx 生成 docx |
| python-docx / Word | 均可用 | ✅ |
| 用户参数文件 `usv_params.m` | 脚本 `run` 载入（MRB/MA/Dl/Dn/T/N_max/N_min） | ✅ |

---

## 3 模型总体结构

```
 Clock ──t──▶ [MATLAB Function: 期望航向指令] ──ψd──┬──────────────┐
                                                    │              │
                       Mux [ψd; ψ] ──▶ Heading_PID ──N──┐           │
                                                            ▼       │
                       USV 运动模型 ◀── [0; 0; N] ── Mux(3) ◀── N    │
                            │ 6 维状态 [x_u,y,ψ,u,v,r]               │
                            ├─ Demux → ψ ──────────────────────────┘
                            ├─▶ Scope_航向 / To Workspace(sim_psi_d, sim_psi)
                            └─▶ Scope_力矩 / To Workspace(sim_torque)
```

单层模型（14 个模块），无子系统嵌套，信号流左→右布置。

---

## 4 模块设计表

### 4.1 模块清单（实施结果）

| 模块ID | 模块名称 | 类型 / 来源地址 | 关键配置参数 | 输入端口 | 输出端口 | 连线关系 |
|---|---|---|---|---|---|---|
| B01 | 仿真时钟 | Clock；`simulink/Sources/Clock` | 采样时间 0（连续） | — | t（s） | Out1→B02.In1 |
| B02 | 期望航向指令生成 | MATLAB Function；`simulink/User-Defined Functions/MATLAB Function` | 代码见 4.2，T_SW1=42、T_SW2=104 | In1：t | Out1：ψd（rad） | Out1→B03.In1、B09.In1、B11.In1 |
| B03 | 控制器输入合成 | Mux；`simulink/Signal Routing/Mux` | 输入数 = 2 | In1：ψd；In2：ψ | [ψd; ψ] | Out1→B06.In1 |
| B04 | 模型输入合成 | Mux；`simulink/Signal Routing/Mux` | 输入数 = 3 | In1：X=0；In2：Y=0；In3：N | [X; Y; N] | Out1→B07.In1 |
| B05 | 纵荡/横荡力 0 | Constant；`simulink/Sources/Constant` | Value = 0 | — | 0 | Out1→B04.In1、In2 |
| B06 | 航向PID控制器 | S-Function；`simulink/User-Defined Functions/S-Function`；`Heading_PID`（usv_lib） | 参数 `cfg_pid` | In1：[ψd; ψ] | N（N·m） | Out1→B04.In3、B10.In1、B13.In1 |
| B07 | USV 运动模型 | S-Function；同上；`USV`（usv_lib） | 参数 `cfg_usv` | In1：[X,Y,N] | 6 维状态 | Out1→B08.In1 |
| B08 | 状态解包 | Demux；`simulink/Signal Routing/Demux` | 输出数 = 6 | In1：6 维状态 | Out3：ψ | Out3→B03.In2、B09.In2、B12.In1 |
| B09 | Scope-航向 | Scope；`simulink/Sinks/Scope` | 2 输入端口 | In1：ψd；In2：ψ | — | — |
| B10 | Scope-力矩 | Scope；`simulink/Sinks/Scope` | 1 输入端口 | In1：N | — | — |
| B11 | 期望航向记录 | To Workspace；`simulink/Sinks/To Workspace` | `sim_psi_d`，Timeseries，0.01 s | In1：ψd | — | — |
| B12 | 实际航向记录 | To Workspace；同上 | `sim_psi`，Timeseries，0.01 s | In1：ψ | — | — |
| B13 | 力矩记录 | To Workspace；同上 | `sim_torque`，Timeseries，0.01 s | In1：N | — | — |
| B14 | 角速度记录 | To Workspace；同上 | `sim_r`，Timeseries，0.01 s | In1：r（Demux 第 6 路） | — | — |

### 4.2 期望航向指令 MATLAB Function 代码（最终版）

```matlab
function psid = fcn(t)
%#codegen
if t < 42
    psid = 30*pi/180;
elseif t < 104
    psid = 230*pi/180;
else
    psid = 30*pi/180;
end
```

- 无反馈、无状态机，仅按时间分段赋值；
- 由脚本通过 Stateflow API（`sfroot().find` → `Stateflow.EMChart.Script`）写入块内，无需手工操作。

---

## 5 参数配置表

### 5.1 船体参数（直接读取 `E:\0203\usv_params.m`，不改动原文件）

| 变量 | 取值 | 用途 |
|---|---|---|
| MRB | `diag([13.5, 13.5, 0.37])` | 本体惯量矩阵 |
| MA | `diag([6.36, 7.12, 0.222])` | 附加惯量矩阵 |
| Dl | `diag([13.7; 0; 0])` | 线性阻尼 |
| Dn | `diag([141; 217; 1.5])` | 非线性阻尼 |
| T | 0.01 s | 控制器周期 |
| N_max / N_min | 22 / −22 N·m | 力矩限幅 |
| x0 | `[0;0;0;0;0;0]` | 初始航向 0°（脚本补充） |

### 5.2 PID 增益（实测整定值）

| 字段 | 取值 | 说明 |
|---|---|---|
| Kp | 6 | 比例（N·m/rad） |
| Ki | 0.6 | 积分 |
| Kd | 380 | 微分 |
| T | 0.01 s | 来自 usv_params.m |
| N_max / N_min | ±22 | 来自 usv_params.m |

> 说明：`Heading_PID.m` 的微分项按逐拍误差差分实现（未除控制周期），因此等效微分增益为 Kd·T ≈ 3.8 N·m/(rad/s)，Kd 需取 10² 量级。整定以"收敛快、超调可控、稳态误差 < 0.1°"为目标，经 3 批共 13 组增益扫描确定。

### 5.3 仿真配置

| 项目 | 取值 |
|---|---|
| 求解器 | ode4 固定步长 |
| 固定步长 | 0.01 s |
| 仿真时长 | 300 s |
| 运行模式 | Normal（S-Function 使用 `gcb`，禁用加速模式） |

---

## 6 数据采集、指标与报告方案

### 6.1 采集信号

- `sim_psi_d`：ψd（timeseries，0.01 s）
- `sim_psi`：ψ（timeseries，0.01 s）
- `sim_torque`：N（timeseries，0.01 s）

> 数据预处理：§5 生成连续展开角（`unwrap`，图 1 使用）与 0~360° 映射列（`angleTo360`），连同误差写入 `results/heading_data.csv`；期望航向按等价的 -180~180 显示（230° → -130°），绘图段不做额外角度换算。

### 6.2 指标计算（脚本后处理）

| 指标 | 方法 |
|---|---|
| 各阶段收敛时刻 | 阶段起点起，误差首次进入 ±0.1° 且此后不再越界的时刻 |
| 各阶段收敛耗时 | 收敛时刻 − 阶段起点 |
| 切换前稳定性校验 | [T_SW1−10, T_SW1]、[T_SW2−10, T_SW2] 内 \|e\| 全程 < 0.1°，不满足即报错 |
| 超调量 | 越过目标后的最大同向偏差（按短路径方向分别取极值） |
| 稳态误差 | 各阶段最后 5 s 的 \|ψd−ψ\| 均值 |
| 力矩峰值 | max\|N\| |

### 6.3 图件（`E:\0203\results\`）

1. `fig_model.png`：Simulink 模型结构截图（报告图 1，由 `print('-s...')` 自动输出）
2. `fig_heading.png`：ψd、ψ 响应（展开角连续显示，230° 以等价 -130° 表示，无过零折返；纵轴按曲线实际范围自适应取 [-180°, 60°]）+ 切换时刻标注
3. `fig_error.png`：误差绝对值（对数坐标）+ 0.1° 容差线
4. `fig_torque.png`：N 时程 + ±22 N·m 限幅线
5. `fig_yawrate.png`：航向角速度 r 时程曲线（°/s）

### 6.4 docx 报告章节

1. 实验目的与模型结构（模块清单表 + Simulink 模型截图及说明）
2. 仿真参数设置（船体参数、PID 参数、仿真配置 + 标定说明）
3. 控制响应曲线（3 幅插图）
4. 各阶段收敛时间与指标数据（2 张表）
5. 控制效果分析

---

## 7 脚本结构（build_usv_heading_pid.m）

| 段 | 内容 |
|---|---|
| §1 | 初始化：环境检查、MSS 自动加载、复制 `USV.m`/`Heading_PID.m` 到 `E:\0203`、addpath |
| §2 | 参数：`run('usv_params.m')` → `cfg_usv`（补 x0）、`cfg_pid`（Kp/Ki/Kd + 文件中的 T、限幅）；T_SW1/T_SW2/T_END |
| §3 | 建模：`new_system` → 13 个模块 → 配置（Stateflow API 写 MATLAB Function 代码）→ 连线 → 求解器配置 → 保存 `.slx` |
| §4 | 仿真：`sim` 执行，校验数据生成 |
| §5 | 指标：收敛时刻、超调、稳态误差、切换前稳定性校验；写 `results/metrics.csv` |
| §6 | 绘图：3 幅 PNG |
| §7 | 报告：写 `results/report_data.json` → 调用 `python gen_report.py` → `USV航向PID控制仿真实验报告.docx` |
| §8 | 汇总：打印结果、写 `results/timing.json`（各段耗时） |

失败处理：任一段出错即打印完整错误信息、非零退出，便于迭代修复（≤5 轮）。

---

## 8 交付物

| 文件 | 说明 |
|---|---|
| `E:\0203\USV.m`、`E:\0203\Heading_PID.m` | usv_lib 复制件（脚本自动复制） |
| `E:\0203\build_usv_heading_pid.m` | 主脚本 |
| `E:\0203\gen_report.py` | docx 排版辅助脚本（python-docx） |
| `E:\0203\usv_heading_pid.slx` | 自动生成模型（14 模块） |
| `E:\0203\USV航向PID控制仿真实验报告.docx` | 实验报告（6 表 5 图） |
| `E:\0203\工作日志.md` | 工作日志 |
| `E:\0203\results\` | 图件（含模型截图 `fig_model.png`）、metrics.csv、report_data.json、timing.json |

---

## 9 实施结果

| 项目 | 结果 |
|---|---|
| 阶段 1 收敛 | 29.93 s，超调 1.672°，稳态误差 0.0362° |
| 阶段 2 收敛 | 91.53 s（切换后 49.53 s），超调 12.665°，稳态误差 0.0347° |
| 阶段 3 收敛 | 153.53 s（切换后 49.53 s），超调 12.666°，稳态误差 ≈ 0° |
| 切换前 10 s 稳定性 | [32,42] s 最大误差 0.0800°；[94,104] s 最大误差 0.0767°，均 < 0.1° |
| 力矩峰值 | 22.00 N·m（限幅 ±22 N·m，仅在大角度转向段短时饱和） |
| 单次脚本总耗时 | 23.2 s（建模 8.3 s、仿真 8.4 s、绘图 5.4 s、报告 0.4 s） |
