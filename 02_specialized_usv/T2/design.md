# 无人船 3 自由度 Fossen 开环运动仿真模型 设计图

## 1 任务概述

用 MATLAB 脚本在 Simulink 中搭建无人船 3 自由度（纵荡、横荡、艏摇）开环运动模型：输入为力 X、Y 与力矩 N，输出为位置 (x, y)、姿态 ψ 与速度 (u, v, r)。模型建成后由同一脚本依次运行三个单独施力工况，仿真时长 12 s，用于观察前进、横向与转向运动是否产生。

模型文件名：`usv_openloop_3dof.slx`
脚本文件名：`build_usv_openloop_3dof.m`

## 2 模型总体结构

信号流（单向，无反馈，故为开环）：

```
[In_X: 常数 tauX] ─┐
[In_Y: 常数 tauY] ─┼─→ [Mux3] ─→ [USV (3 自由度 Fossen S-Function)] ─→ [Demux6] ─┬─→ [Scope_Pose: x, y, ψ]
[In_N: 常数 tauN] ─┘                                                             ├─→ [Scope_Vel: u, v, r]
                                                                                 ├─→ [XY Graph: y - x 轨迹]
                                                                                 └─→ [To_Workspace: 6 维状态向量]
```

说明：三个常值输入模块分别给出纵荡力、横荡力、艏摇力矩，经 Mux 合成 3 维控制输入向量 τ = [X, Y, N]ᵀ 送入 usv_lib 的无人船模型；模型内部积分得到 6 维状态 [x_u, y, ψ, u, v, r]ᵀ，再经 Demux 拆成单个信号供示波器、XY 图和 To Workspace 记录使用。三个工况通过脚本在每次运行前改写基础工作区的 `tauX / tauY / tauN` 实现，模型本身只建一次。

## 3 模块设计表

| 序号 | 模块名称 | 模块ID | 模块类型 | 来源地址 | 可配置参数 | 输入端口说明 | 输出端口说明 | 端口连线关系 |
|---|---|---|---|---|---|---|---|---|
| 1 | 纵荡力输入 | B1 `In_X` | Constant（常值） | Simulink 官方库 `simulink/Sources/Constant` | Constant value = `tauX`；Sample time = `inf` | 无 | out1：标量 X（N） | out1 → B4.in1 |
| 2 | 横荡力输入 | B2 `In_Y` | Constant（常值） | Simulink 官方库 `simulink/Sources/Constant` | Constant value = `tauY`；Sample time = `inf` | 无 | out1：标量 Y（N） | out1 → B4.in2 |
| 3 | 艏摇力矩输入 | B3 `In_N` | Constant（常值） | Simulink 官方库 `simulink/Sources/Constant` | Constant value = `tauN`；Sample time = `inf` | 无 | out1：标量 N（N·m） | out1 → B4.in3 |
| 4 | 输入合成 | B4 `Mux3` | Mux（多路合成） | Simulink 官方库 `simulink/Signal Routing/Mux` | Number of inputs = `3`；Display option = `bar` | in1、in2、in3：三个标量 | out1：3 维向量 [X; Y; N] | in1←B1.out1，in2←B2.out1，in3←B3.out1；out1 → B5.in1 |
| 5 | 无人船 3 自由度模型 | B5 `USV_Model` | S-Function（usv_lib 模块） | usv_lib：`USV.m`（复制到本任务工作路径后使用）；模块库路径 `simulink/User-Defined Functions/S-Function` | S-function name = `USV`；S-function parameters = `cfg` | in1：3 维控制输入 [X; Y; N] | out1：6 维状态 [x_u; y; ψ; u; v; r] | in1←B4.out1；out1 → B6.in1，out1 → B9.in1 |
| 6 | 状态拆分 | B6 `Demux6` | Demux（信号拆分） | Simulink 官方库 `simulink/Signal Routing/Demux` | Number of outputs = `6` | in1：6 维状态向量 | out1：x_u（m）、out2：y（m）、out3：ψ（rad）、out4：u（m/s）、out5：v（m/s）、out6：r（rad/s） | in1←B5.out1；out1、out2、out3 → B7.in1/in2/in3；out4、out5、out6 → B8.in1/in2/in3；out1 → B10.x，out2 → B10.y |
| 7 | 位姿示波器 | B7 `Scope_Pose` | Scope（示波器） | Simulink 官方库 `simulink/Sinks/Scope` | Number of input ports = `3`；输入端口维度均为标量 | in1：x_u、in2：y、in3：ψ | 无 | in1←B6.out1，in2←B6.out2，in3←B6.out3 |
| 8 | 速度示波器 | B8 `Scope_Vel` | Scope（示波器） | Simulink 官方库 `simulink/Sinks/Scope` | Number of input ports = `3`；输入端口维度均为标量 | in1：u、in2：v、in3：r | 无 | in1←B6.out4，in2←B6.out5，in3←B6.out6 |
| 9 | 状态记录 | B9 `To_Workspace` | To Workspace（输出到工作区） | Simulink 官方库 `simulink/Sinks/To Workspace` | Variable name = `usv_out`；Save format = `Array`；Sample time = `-1`（继承） | in1：6 维状态向量 | 无（写入基础工作区变量 `usv_out`） | in1←B5.out1 |
| 10 | 平面轨迹图 | B10 `XY_Graph` | XY Graph（平面绘图） | Simulink 官方库 `simulink/Sinks/XY Graph` | x-axis limits = `[-5 5]`；y-axis limits = `[-5 5]`；Sample time = `-1` | x：x_u、y：y | 无 | x←B6.out1，y←B6.out2 |

## 4 模型级配置

| 配置项 | 取值 |
|---|---|
| 求解器 | `ode45`（变步长） |
| 仿真时长 StopTime | `12` |
| SaveTime | `on`（记录时间向量 `tout`） |
| 工作区变量名 | 不自动生成 `simout`（统一由 B9 的 `usv_out` 记录） |
| 模型保存位置 | 当前任务工作路径 `usv_openloop_3dof.slx` |

## 5 模型参数（cfg，由脚本写入基础工作区）

参数数值全部取自工作路径下的 `usv_params.m`，脚本运行时读取该文件，不在模型内写死。

| 字段 | 取值 | 说明 |
|---|---|---|
| `x0` | `[0;0;0;0;0;0]` | 状态初值，静止于原点、初始航向 0 |
| `MRB` | `diag([13.5, 13.5, 0.37])` | 刚体惯量矩阵 |
| `MA` | `diag([6.36, 7.12, 0.222])` | 附加惯量矩阵 |
| `Dl` | `diag([13.7, 0, 0])` | 线性阻尼矩阵 |
| `Dn` | `diag([141, 217, 1.5])` | 非线性阻尼矩阵 |

## 6 仿真工况表

| 工况 | tauX (N) | tauY (N) | tauN (N·m) | 观察目标 |
|---|---|---|---|---|
| C1 前进 | 10 | 0 | 0 | u 上升、x_u 持续增大，船沿 x 轴前进 |
| C2 横向 | 0 | 5 | 0 | v 上升、y 持续增大，船产生横向漂移 |
| C3 转向 | 0 | 0 | 5 | r 上升、ψ 持续变化，船原地转向 |

每个工况单独 `sim` 一次，结果按工况分别记录并绘图对比。

## 7 脚本结构（仅描述步骤，不写代码）

1. 清理环境：关闭模型、清空基础工作区变量、设置工作路径；
2. 写入 `cfg` 结构体参数；
3. 若已存在同名 `.slx` 则先关闭并删除，新建空模型并命名；
4. 按第 3 节模块表逐块 `add_block`，按第 4 节设置模型与模块参数，逐条 `add_line` 连线；
5. 设置模型求解器与 StopTime，自动布局并对保存 `.slx`；
6. 依次对 C1、C2、C3 赋值 `tauX / tauY / tauN` 并运行 `sim`，把 `tout`、`usv_out` 存入工况结构体；
7. 绘图：三个工况的 x/y/ψ 时序、u/v/r 时序，以及 x-y 平面轨迹，图片保存到 `figures/` 供报告使用；
8. 在命令行输出各工况末时刻状态量，用于判定“是否产生前进 / 横向 / 转向运动”。

## 8 模块合法性核对结论

- usv_lib 清单（`modules.md`）中共有 1 个模块 `usv_model`（`USV.m`，输入 3 维 [X,Y,N]，输出 6 维 [x_u,y,ψ,u,v,r]，参数 `cfg`），本设计所需无人船模型正好对应，清单存在，无需补充模块；搭建时将 `USV.m` 复制到本任务工作路径后使用。
- `USV.m` 内部调用 `eulerang`、`m2c`（MSS 工具箱），已确认位于 `D:\Matlab\R2024b\toolbox\MSS\LIBRARY` 并已在 MATLAB 搜索路径中，可直接运行。
- 其余模块全部为 Simulink 官方自带公共库模块，符合来源限制。

## 9 待确认事项

1. 是否接受“只建一个模型、由脚本切换三个常值输入分别运行”的方案（相对“一个模型内放三个子系统分别仿真”更简洁，结果对比在同一张图上）。
2. 物理参数采用上表的小型无人船量级（m = 55 kg，阻尼线性），是否同意；如需改用其他船型参数请一并说明。
3. 是否需要额外的 x-y 轨迹图与末端数值对比（当前设计已包含）。
