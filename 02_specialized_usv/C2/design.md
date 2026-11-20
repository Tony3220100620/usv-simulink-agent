# 基于 3 自由度 Fossen 方程的无人船开环运动仿真模型 — 设计图

## 1. 设计目标

用 MATLAB 脚本以纯命令行方式（`new_system`/`add_block`/`add_line`/`set_param`）搭建一个 Simulink 开环仿真模型，对无人船依次施加单独的纵荡力 X、横荡力 Y、艏摇力矩 N，观测其纵荡、横荡、艏摇响应，仿真时长 12 s。

模型只做"执行器力/力矩 → 船体运动"的开环传递，不含任何控制器、不含参考轨迹、不含环境扰动。

## 2. 模型总体结构

```
X_cmd ─┐
Y_cmd ─┼─ Mux(3) ─→ USV(S-Function) ─→ Demux(6) ─┬─ [1][2] ─ Mux(2) ─ XY Graph
N_cmd ─┘                                        ├─ [3]    ─ Scope(ψ)
                                                ├─ [4][5][6] ─ Mux(3) ─ Scope(u,v,r)
                                                └─ [1..6] ─ To Workspace
```

三个工况由脚本循环设置三个 Constant 的数值后分别 `sim` 三次，模型本身不区分工况，因此模型中不存在任何时间分段或切换逻辑。

## 3. 模块设计表

| 模块名称 | 模块ID | 模块类型 | 来源地址 | 可配置参数 | 输入端口说明 | 输出端口说明 |
|---|---|---|---|---|---|---|
| USV 三自由度运动模型 | B01 | S-Function（usv_lib 专用模块） | `E:\学校课程\海洋数据挖掘技术\作业\02_specialized_usv\usv_lib\USV.m`，运行时复制到本任务工作目录后使用 | 模块参数：`cfg`（工作区结构体）；块参数 `FunctionName=USV`、`Parameters=cfg`、`Sizes` 由 S-Function 自描述 | 1 个输入端口 `u`，3 维列向量 `[X; Y; N]`（N、N·m） | 1 个输出端口，6 维列向量 `[x; y; ψ; u; v; r]`（m、m、rad、m/s、m/s、rad/s） |
| 纵荡力输入 | B02 | Simulink 官方库 Constant | `simulink/Sources/Constant` | `Value`（脚本按工况写入 `10`/`0`/`0`）；`SampleTime=0`（连续常量） | 无 | 1 维，纵荡力 X（N） |
| 横荡力输入 | B03 | Simulink 官方库 Constant | `simulink/Sources/Constant` | `Value`（脚本按工况写入 `0`/`5`/`0`）；`SampleTime=0` | 无 | 1 维，横荡力 Y（N） |
| 艏摇力矩输入 | B04 | Simulink 官方库 Constant | `simulink/Sources/Constant` | `Value`（脚本按工况写入 `0`/`0`/`5`）；`SampleTime=0` | 无 | 1 维，艏摇力矩 N（N·m） |
| 输入合成 | B05 | Simulink 官方库 Mux | `simulink/Signal Routing/Mux` | `Inputs=3`，`DisplayOption=bar` | 3 个标量输入：X、Y、N | 1 个 3 维向量 `[X;Y;N]` |
| 输出分解 | B06 | Simulink 官方库 Demux | `simulink/Signal Routing/Demux` | `Outputs=6`，`DisplayOption=bar` | 1 个 6 维向量 | 6 个标量输出：x、y、ψ、u、v、r |
| 位置合成 | B07 | Simulink 官方库 Mux | `simulink/Signal Routing/Mux` | `Inputs=2` | 2 个标量输入：x、y | 1 个 2 维向量 `[x;y]` |
| 轨迹显示 | B08 | Simulink 官方库 XY Graph | `simulink/Sinks/XY Graph` | `xmin=-2, xmax=12, ymin=-6, ymax=6`（按 12 s 行程预置） | 1 个 2 维输入 `[x;y]` | 无（仅显示北东平面轨迹） |
| 航向角显示 | B09 | Simulink 官方库 Scope | `simulink/Sinks/Scope` | `NumInputPorts=1`，`TimeSpan=12`（自动） | 1 个标量输入 ψ | 无 |
| 速度合成 | B10 | Simulink 官方库 Mux | `simulink/Signal Routing/Mux` | `Inputs=3` | 3 个标量输入：u、v、r | 1 个 3 维向量 `[u;v;r]` |
| 速度显示 | B11 | Simulink 官方库 Scope | `simulink/Sinks/Scope` | `NumInputPorts=1`，`TimeSpan=12`（自动） | 1 个 3 维输入 `[u;v;r]` | 无 |
| 数据记录 | B12 | Simulink 官方库 To Workspace | `simulink/Sinks/To Workspace` | `VariableName=usv_out`、`SaveFormat=Structure With Time`、`SampleTime=0.02`、`MaxDataPoints=inf` | 1 个 6 维输入 | 无（写入工作区结构体，供脚本绘图/出报告） |

## 4. 端口连线关系

| 序号 | 起点（模块.端口） | 终点（模块.端口） | 信号维度/含义 |
|---|---|---|---|
| L01 | B02 输出 | B05 输入 1 | 1，X |
| L02 | B03 输出 | B05 输入 2 | 1，Y |
| L03 | B04 输出 | B05 输入 3 | 1，N |
| L04 | B05 输出 | B01 输入 1 | 3，`[X;Y;N]` |
| L05 | B01 输出 | B06 输入 1 | 6，`[x;y;ψ;u;v;r]` |
| L06 | B06 输出 1 | B07 输入 1 | 1，x |
| L07 | B06 输出 2 | B07 输入 2 | 1，y |
| L08 | B07 输出 | B08 输入 1 | 2，`[x;y]` |
| L09 | B06 输出 3 | B09 输入 1 | 1，ψ |
| L10 | B06 输出 4 | B10 输入 1 | 1，u |
| L11 | B06 输出 5 | B10 输入 2 | 1，v |
| L12 | B06 输出 6 | B10 输入 3 | 1，r |
| L13 | B10 输出 | B11 输入 1 | 3，`[u;v;r]` |
| L14 | B01 输出 | B12 输入 1 | 6，全状态量 |

## 5. 模型级配置

| 项目 | 取值 |
|---|---|
| 模型名 | `usv_openloop_3dof.slx` |
| 求解器 | `ode45`（变步长），`MaxStep=0.01`，`RelTol=1e-6` |
| 仿真时长 | `StopTime=12` |
| 初始状态 | `cfg.x0 = zeros(6,1)`（静止于原点，ψ=0） |
| 信号记录 | `SaveTime=on`、`SaveOutput=on`，输出格式 `Structure With Time` |

## 6. 船体参数 cfg（脚本内定义，单位 SI）

| 字段 | 取值 | 说明 |
|---|---|---|
| `MRB` | `diag([25, 25, 2.5])` | 刚体惯性：质量 25 kg，绕 z 轴转动惯量 2.5 kg·m²，重心与几何中心重合（不做耦合项） |
| `MA` | `diag([5, 15, 1.0])` | 附加质量：纵荡 5 kg、横荡 15 kg、艏摇 1.0 kg·m² |
| `Dl` | `diag([10, 20, 20])` | 线性阻尼：X 向 10 N·s/m、Y 向 20 N·s/m、N 向 20 N·m·s/rad |
| `Dn` | `diag([1, 2, 0.5])` | 非线性阻尼（平方项，与 `diag(|ν|)` 相乘）：1、2、0.5 |
| `x0` | `zeros(6,1)` | 初始位姿与速度均为零 |

由此得到的开环稳态预估（供审核时判断量级是否合理）：

- 工况 1（X=10 N）：`u_ss` 由 `10 - 10u - u² = 0` 解得 ≈ 0.916 m/s，12 s 内前进约 8.5~9 m，`v=0、r=0、ψ=0`。
- 工况 2（Y=5 N）：`v_ss` 由 `5 - 20v - 2v² = 0` 解得 ≈ 0.237 m/s，仅横荡、无艏摇，12 s 内横向位移约 2.5 m，`ψ` 保持 0。
- 工况 3（N=5 N·m）：`r_ss` 由 `5 - 20r - 0.5r² = 0` 解得 ≈ 0.246 rad/s，12 s 内累计转角约 2.9 rad（≈166°），`x、y` 因 `u=v=0` 基本不变。

## 7. 脚本执行流程（第二段实现，本次不写代码）

1. 建立工作目录、把 `usv_lib\USV.m` 复制到本任务目录并 `addpath`。
2. 定义 `cfg`，写为结构体变量。
3. `new_system` → 按表 3 逐个 `add_block`（含 BlockID 定位、命名）→ 按表 4 逐个 `add_line`。
4. 设置表 5 的模型级参数，`save_system` 生成 `usv_openloop_3dof.slx`。
5. 三个工况循环：`set_param` 改写 B02/B03/B04 的 `Value` → `sim` → 取 `usv_out` 存为 `res_X`、`res_Y`、`res_N`。
6. 依据三组结果作图并打印关键指标（终值、稳态值、是否产生对应运动），写入 `sim_report_data.mat`，供报告生成使用。

## 8. 需要用户确认的点

1. 船体参数（表 6）是否沿用这组"25 kg 小型无人船"数值，还是换成你指定船型的 `MRB/MA/Dl/Dn`。
2. 三个工况是否按"依次施加、各自独立仿真"处理（每个工况都从静止状态起算），而不是在同一次 12 s 仿真内分三段施加。
3. 模块清单中只使用 `usv_model`（B01），其余全部为 Simulink 官方自带库，是否认可。
