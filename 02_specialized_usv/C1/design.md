# 三自由度 Fossen 无人船开环运动 Simulink 模型设计说明

## 1. 任务目标

用 MATLAB 脚本自动生成一个 Simulink 模型，实现基于三自由度 Fossen 方程（纵荡 surge、横荡 sway、艏摇 yaw）的无人船开环运动仿真：

- 模型输入：纵向力 X（N）、横向力 Y（N）、艏摇力矩 N（N·m）；
- 模型输出：位置 (x, y)（m）、姿态 ψ（rad）、速度 (u, v, r)（m/s、m/s、rad/s）；
- 仿真时长 12 s，初始状态为静止（u₀ = v₀ = r₀ = 0，x₀ = y₀ = ψ₀ = 0）；
- 分别运行三种单一激励工况，观察无人船的运动响应：
  1. 仅施加 X = 10 N，观察是否产生前进运动；
  2. 仅施加 Y = 5 N，观察是否产生横向运动；
  3. 仅施加 N = 5 N·m，观察是否产生转向运动。

## 2. 数学模型与假设

采用 Fossen 三自由度船舶运动模型：

- 运动学：η̇ = R(ψ)·ν，其中 η = [x, y, ψ]ᵀ，ν = [u, v, r]ᵀ，
  R(ψ) = [[cosψ, −sinψ, 0], [sinψ, cosψ, 0], [0, 0, 1]]；
- 动力学：M·ν̇ + C(ν)·ν + D·ν = τ，τ = [X, Y, N]ᵀ。

假设惯性矩阵与阻尼矩阵均为对角阵（M = diag(m₁₁, m₂₂, m₃₃)，D = diag(d₁₁, d₂₂, d₃₃)），阻尼取线性形式。代入 C(ν) 后展开为三个标量微分方程，模型即按此实现：

| 通道 | 方程 |
|---|---|
| 纵荡 | m₁₁·u̇ = X + m₂₂·v·r − d₁₁·u |
| 横荡 | m₂₂·v̇ = Y − m₁₁·u·r − d₂₂·v |
| 艏摇 | m₃₃·ṙ = N + (m₁₁ − m₂₂)·u·v − d₃₃·r |
| 运动学 | ẋ = u·cosψ − v·sinψ；ẏ = u·sinψ + v·cosψ；ψ̇ = r |

说明：模型只做"方程 → 框图"的连接，所有方程系数以增益形式出现在框图上，不编写任何自定义 S-Function、MATLAB Function 或脚本实现的模块内部逻辑；全模型由 Simulink 官方基础模块（积分器、增益、求和、乘法、三角函数等）连接构成。

## 3. 模块来源核对结论

本模型全部使用 Simulink 官方内置公共库模块（Sources / Math Operations / Continuous / Signal Routing / Sinks），不使用任何无人船专用模块，因此不涉及 `usv_lib/modules.md` 的模块核对，也无模块缺失风险（当前 `usv_lib` 目录为空，本任务也不依赖它）。

## 4. 信号流总体结构

```
X ──► Sum_U(+,+,-) ──► 1/m11 ──► ∫ ──► u ─┬──► 运动学（η̇ 计算）──► ∫ ──► x, y, ψ
          ▲   ▲                          ├──► d11 反馈
          │   └── m22·(v·r)              ├──► u·r（供横荡方程）
Y ──► Sum_V(+,-,-) ──► 1/m22 ──► ∫ ──► v ─┼──► u·v（供艏摇方程）
          ▲   ▲                          ├──► v·r（供纵荡方程）
          │   └── m11·(u·r)              └──► 运动学
N ──► Sum_R(+,+,-) ──► 1/m33 ──► ∫ ──► r ──► ∫ ──► ψ ──► sin/cos ──► 运动学
          │   └── (m11−m22)·(u·v)
          └── d33 反馈
输出：(x, y, ψ) → Mux3 → Scope_Pos / To Workspace(velData 对应 posData)；(u, v, r) → Mux3 → Scope_Vel / To Workspace(velData)
```

## 5. 模块设计表

### 5.1 输入模块

| 模块名称 | 模块ID | 模块类型 | 来源地址 | 可配置参数 | 输入端口说明 | 输出端口说明 | 端口连线关系 |
|---|---|---|---|---|---|---|---|
| 纵向力输入 | Const_X | Constant | simulink/Sources/Constant | Value = Xcmd（工况1为 10，其余工况为 0）；SampleTime = 0（连续） | 无 | 输出端口 1：常值纵向力 X | 输出端口 1 → Sum_U 输入端口 1 |
| 横向力输入 | Const_Y | Constant | simulink/Sources/Constant | Value = Ycmd（工况2为 5，其余工况为 0）；SampleTime = 0 | 无 | 输出端口 1：常值横向力 Y | 输出端口 1 → Sum_V 输入端口 1 |
| 艏摇力矩输入 | Const_N | Constant | simulink/Sources/Constant | Value = Ncmd（工况3为 5，其余工况为 0）；SampleTime = 0 | 无 | 输出端口 1：常值艏摇力矩 N | 输出端口 1 → Sum_R 输入端口 1 |

### 5.2 纵荡（surge）通道

| 模块名称 | 模块ID | 模块类型 | 来源地址 | 可配置参数 | 输入端口说明 | 输出端口说明 | 端口连线关系 |
|---|---|---|---|---|---|---|---|
| 纵荡加速度求和 | Sum_U | Sum | simulink/Math Operations/Sum | Inputs = '++-'；IconShape = round | 端口1：X；端口2：m₂₂·v·r；端口3：d₁₁·u | 输出端口 1：m₁₁·u̇（合外力） | 端口1 ← Const_X；端口2 ← Gain_M22；端口3 ← Gain_D11；输出 → Gain_InvM11 |
| 纵荡惯性逆增益 | Gain_InvM11 | Gain | simulink/Math Operations/Gain | Gain = 1/m₁₁（默认 1/30） | 端口1：m₁₁·u̇ | 输出端口 1：u̇ | 输入 ← Sum_U；输出 → Int_U |
| 纵荡速度积分 | Int_U | Integrator | simulink/Continuous/Integrator | InitialCondition = 0；LimitOutput = off | 端口1：u̇ | 输出端口 1：u | 输入 ← Gain_InvM11；输出 → Gain_D11、Prod_UR、Prod_Ucos、Prod_Usin、Mux_Vel |
| 纵荡阻尼增益 | Gain_D11 | Gain | simulink/Math Operations/Gain | Gain = d₁₁（默认 12） | 端口1：u | 输出端口 1：d₁₁·u | 输入 ← Int_U；输出 → Sum_U 端口3 |
| 科氏项乘法（v·r） | Prod_VR | Product | simulink/Math Operations/Product | Inputs = 2；Multiplication = Element-wise | 端口1：v；端口2：r | 输出端口 1：v·r | 端口1 ← Int_V；端口2 ← Int_R；输出 → Gain_M22 |
| 科氏项增益（m₂₂） | Gain_M22 | Gain | simulink/Math Operations/Gain | Gain = m₂₂（默认 45） | 端口1：v·r | 输出端口 1：m₂₂·v·r | 输入 ← Prod_VR；输出 → Sum_U 端口2 |

### 5.3 横荡（sway）通道

| 模块名称 | 模块ID | 模块类型 | 来源地址 | 可配置参数 | 输入端口说明 | 输出端口说明 | 端口连线关系 |
|---|---|---|---|---|---|---|---|
| 横荡加速度求和 | Sum_V | Sum | simulink/Math Operations/Sum | Inputs = '+--'；IconShape = round | 端口1：Y；端口2：m₁₁·u·r；端口3：d₂₂·v | 输出端口 1：m₂₂·v̇ | 端口1 ← Const_Y；端口2 ← Gain_M11；端口3 ← Gain_D22；输出 → Gain_InvM22 |
| 横荡惯性逆增益 | Gain_InvM22 | Gain | simulink/Math Operations/Gain | Gain = 1/m₂₂（默认 1/45） | 端口1：m₂₂·v̇ | 输出端口 1：v̇ | 输入 ← Sum_V；输出 → Int_V |
| 横荡速度积分 | Int_V | Integrator | simulink/Continuous/Integrator | InitialCondition = 0；LimitOutput = off | 端口1：v̇ | 输出端口 1：v | 输入 ← Gain_InvM22；输出 → Gain_D22、Prod_VR、Prod_UV、Prod_Vsin、Prod_Vcos、Mux_Vel |
| 横荡阻尼增益 | Gain_D22 | Gain | simulink/Math Operations/Gain | Gain = d₂₂（默认 30） | 端口1：v | 输出端口 1：d₂₂·v | 输入 ← Int_V；输出 → Sum_V 端口3 |
| 科氏项乘法（u·r） | Prod_UR | Product | simulink/Math Operations/Product | Inputs = 2；Multiplication = Element-wise | 端口1：u；端口2：r | 输出端口 1：u·r | 端口1 ← Int_U；端口2 ← Int_R；输出 → Gain_M11 |
| 科氏项增益（m₁₁） | Gain_M11 | Gain | simulink/Math Operations/Gain | Gain = m₁₁（默认 30） | 端口1：u·r | 输出端口 1：m₁₁·u·r | 输入 ← Prod_UR；输出 → Sum_V 端口2 |

### 5.4 艏摇（yaw）通道

| 模块名称 | 模块ID | 模块类型 | 来源地址 | 可配置参数 | 输入端口说明 | 输出端口说明 | 端口连线关系 |
|---|---|---|---|---|---|---|---|
| 艏摇角加速度求和 | Sum_R | Sum | simulink/Math Operations/Sum | Inputs = '++-'；IconShape = round | 端口1：N；端口2：(m₁₁−m₂₂)·u·v；端口3：d₃₃·r | 输出端口 1：m₃₃·ṙ | 端口1 ← Const_N；端口2 ← Gain_Mdiff；端口3 ← Gain_D33；输出 → Gain_InvM33 |
| 艏摇惯性逆增益 | Gain_InvM33 | Gain | simulink/Math Operations/Gain | Gain = 1/m₃₃（默认 1/8） | 端口1：m₃₃·ṙ | 输出端口 1：ṙ | 输入 ← Sum_R；输出 → Int_R |
| 艏摇角速度积分 | Int_R | Integrator | simulink/Continuous/Integrator | InitialCondition = 0；LimitOutput = off | 端口1：ṙ | 输出端口 1：r | 输入 ← Gain_InvM33；输出 → Gain_D33、Prod_VR、Prod_UR、Int_Psi、Mux_Vel |
| 艏摇阻尼增益 | Gain_D33 | Gain | simulink/Math Operations/Gain | Gain = d₃₃（默认 8） | 端口1：r | 输出端口 1：d₃₃·r | 输入 ← Int_R；输出 → Sum_R 端口3 |
| 科氏项乘法（u·v） | Prod_UV | Product | simulink/Math Operations/Product | Inputs = 2；Multiplication = Element-wise | 端口1：u；端口2：v | 输出端口 1：u·v | 端口1 ← Int_U；端口2 ← Int_V；输出 → Gain_Mdiff |
| 科氏项增益（m₁₁−m₂₂） | Gain_Mdiff | Gain | simulink/Math Operations/Gain | Gain = m₁₁ − m₂₂（默认 −15） | 端口1：u·v | 输出端口 1：(m₁₁−m₂₂)·u·v | 输入 ← Prod_UV；输出 → Sum_R 端口2 |

### 5.5 运动学通道

| 模块名称 | 模块ID | 模块类型 | 来源地址 | 可配置参数 | 输入端口说明 | 输出端口说明 | 端口连线关系 |
|---|---|---|---|---|---|---|---|
| 艏向角积分 | Int_Psi | Integrator | simulink/Continuous/Integrator | InitialCondition = 0 | 端口1：r | 输出端口 1：ψ | 输入 ← Int_R；输出 → Trig_Sin、Trig_Cos、Mux_Pos |
| 正弦函数 | Trig_Sin | Trigonometric Function | simulink/Math Operations/Trigonometric Function | Operator = sin；ApproximationMethod = none（连续精确） | 端口1：ψ | 输出端口 1：sinψ | 输入 ← Int_Psi；输出 → Prod_Vsin、Prod_Usin |
| 余弦函数 | Trig_Cos | Trigonometric Function | simulink/Math Operations/Trigonometric Function | Operator = cos；ApproximationMethod = none | 端口1：ψ | 输出端口 1：cosψ | 输入 ← Int_Psi；输出 → Prod_Ucos、Prod_Vcos |
| ẋ 乘法项（u·cosψ） | Prod_Ucos | Product | simulink/Math Operations/Product | Inputs = 2 | 端口1：u；端口2：cosψ | 输出端口 1：u·cosψ | 端口1 ← Int_U；端口2 ← Trig_Cos；输出 → Sum_Xd 端口1 |
| ẋ 乘法项（v·sinψ） | Prod_Vsin | Product | simulink/Math Operations/Product | Inputs = 2 | 端口1：v；端口2：sinψ | 输出端口 1：v·sinψ | 端口1 ← Int_V；端口2 ← Trig_Sin；输出 → Sum_Xd 端口2 |
| ẋ 求和 | Sum_Xd | Sum | simulink/Math Operations/Sum | Inputs = '+-' | 端口1：u·cosψ；端口2：v·sinψ | 输出端口 1：ẋ | 输出 → Int_X |
| 位置 x 积分 | Int_X | Integrator | simulink/Continuous/Integrator | InitialCondition = 0 | 端口1：ẋ | 输出端口 1：x | 输入 ← Sum_Xd；输出 → Mux_Pos |
| ẏ 乘法项（u·sinψ） | Prod_Usin | Product | simulink/Math Operations/Product | Inputs = 2 | 端口1：u；端口2：sinψ | 输出端口 1：u·sinψ | 端口1 ← Int_U；端口2 ← Trig_Sin；输出 → Sum_Yd 端口1 |
| ẏ 乘法项（v·cosψ） | Prod_Vcos | Product | simulink/Math Operations/Product | Inputs = 2 | 端口1：v；端口2：cosψ | 输出端口 1：v·cosψ | 端口1 ← Int_V；端口2 ← Trig_Cos；输出 → Sum_Yd 端口2 |
| ẏ 求和 | Sum_Yd | Sum | simulink/Math Operations/Sum | Inputs = '++' | 端口1：u·sinψ；端口2：v·cosψ | 输出端口 1：ẏ | 输出 → Int_Y |
| 位置 y 积分 | Int_Y | Integrator | simulink/Continuous/Integrator | InitialCondition = 0 | 端口1：ẏ | 输出端口 1：y | 输入 ← Sum_Yd；输出 → Mux_Pos |

### 5.6 输出与观测模块

| 模块名称 | 模块ID | 模块类型 | 来源地址 | 可配置参数 | 输入端口说明 | 输出端口说明 | 端口连线关系 |
|---|---|---|---|---|---|---|---|
| 位置/姿态合成 | Mux_Pos | Mux | simulink/Signal Routing/Mux | Inputs = 3 | 端口1：x；端口2：y；端口3：ψ | 输出端口 1：向量 [x, y, ψ] | 端口1 ← Int_X；端口2 ← Int_Y；端口3 ← Int_Psi；输出 → Scope_Pos、ToWs_Pos |
| 位置/姿态示波器 | Scope_Pos | Scope | simulink/Sinks/Scope | NumInputPorts = 1（输入为 3 路向量）；OpenAtSimulationStart = off | 端口1：[x, y, ψ] | 无 | 输入 ← Mux_Pos |
| 速度合成 | Mux_Vel | Mux | simulink/Signal Routing/Mux | Inputs = 3 | 端口1：u；端口2：v；端口3：r | 输出端口 1：向量 [u, v, r] | 端口1 ← Int_U；端口2 ← Int_V；端口3 ← Int_R；输出 → Scope_Vel、ToWs_Vel |
| 速度示波器 | Scope_Vel | Scope | simulink/Sinks/Scope | NumInputPorts = 1（输入为 3 路向量）；OpenAtSimulationStart = off | 端口1：[u, v, r] | 无 | 输入 ← Mux_Vel |
| 位置数据记录 | ToWs_Pos | To Workspace | simulink/Sinks/To Workspace | VariableName = posData；SaveFormat = Structure With Time；LimitDataPoints = off | 端口1：[x, y, ψ] | 无（写工作区） | 输入 ← Mux_Pos |
| 速度数据记录 | ToWs_Vel | To Workspace | simulink/Sinks/To Workspace | VariableName = velData；SaveFormat = Structure With Time；LimitDataPoints = off | 端口1：[u, v, r] | 无（写工作区） | 输入 ← Mux_Vel |

### 5.7 模块与连线统计

- 模块总数 38 个：Constant×3、Sum×5、Gain×9、Integrator×6、Product×7、Trigonometric Function×2、Mux×2、Scope×2、To Workspace×2。
- 连线总数 44 条，覆盖第 5.1～5.6 节"端口连线关系"列的全部连接；其中 Int_U、Int_V、Int_R 的输出各有多路分支，用对同一输出端口多次 `add_line` 实现分支，不额外增加 Branch 模块。

## 6. 布局坐标设计

| 区域 | 模块ID | 左上角坐标 [x, y] | 说明 |
|---|---|---|---|
| 输入列 | Const_X / Const_Y / Const_N | [40,40] / [40,180] / [40,320] | 左端纵向排列 |
| 求和列 | Sum_U / Sum_V / Sum_R | [160,25] / [160,165] / [160,305] | 与对应输入同一行 |
| 逆增益列 | Gain_InvM11 / Gain_InvM22 / Gain_InvM33 | [260,40] / [260,180] / [260,320] | 求和模块右侧 |
| 积分列 | Int_U / Int_V / Int_R | [380,35] / [380,175] / [380,315] | 速度状态积分 |
| 阻尼反馈 | Gain_D11 / Gain_D22 / Gain_D33 | [380,100] / [380,240] / [380,380] | 各积分器正下方，反馈回求和模块 |
| 科氏项区 | Prod_VR / Prod_UR / Prod_UV 及其增益 | [180,430] 起三行排列 | 位于下方，连回上方求和模块 |
| 运动学区 | Int_Psi / Trig_Sin / Trig_Cos | [540,230] / [540,300] / [540,380] | 艏向角与正余弦 |
| 运动学乘法与求和 | Prod_Ucos、Prod_Vsin、Sum_Xd、Int_X；Prod_Usin、Prod_Vcos、Sum_Yd、Int_Y | x = 640/740/820，y = 40～280 | 左右两行分别计算 ẋ、ẏ |
| 输出区 | Mux_Pos、Scope_Pos、ToWs_Pos；Mux_Vel、Scope_Vel、ToWs_Vel | x = 920～1050，y = 50～500 | 最右侧 |

连线统一使用 `add_line(...,'autorouting','on')` 自动正交走线，坐标仅用于模块摆放，不要求线段绝对水平。

## 7. 参数配置入口（脚本顶部集中定义）

| 变量名 | 含义 | 默认值 | 影响模块 |
|---|---|---|---|
| m11、m22、m33 | 惯性（含附加质量）参数（kg、kg、kg·m²） | 30、45、8 | Gain_InvM11/M22/M33、Gain_M22、Gain_M11、Gain_Mdiff |
| d11、d22、d33 | 线性阻尼系数（N·s/m、N·s/m、N·m·s/rad） | 12、30、8 | Gain_D11/D22/D33 |
| Xcmd、Ycmd、Ncmd | 当前工况输入（N、N、N·m） | 工况1：10、0、0 | Const_X/Y/N |
| stopTimeSec | 仿真停止时间（s） | 12 | 模型 StopTime |
| modelName | 模型文件名 | usv_3dof_fossen_openloop | new_system / save_system |

## 8. 三种工况运行方案

脚本在同一模型上依次修改三个 Constant 的 Value，调用 `sim` 完成 3 次仿真，并把每次的 posData、velData 存入结构体数组，供报告绘图使用；每次仿真前后无需重建模型。

| 工况 | X (N) | Y (N) | N (N·m) | 观测量 | 理论稳态值 |
|---|---|---|---|---|---|
| 1 前进 | 10 | 0 | 0 | u、x；(v, r, y, ψ 保持 0) | u_ss = X/d₁₁ ≈ 0.833 m/s；x(12) ≈ 7.93 m |
| 2 横移 | 0 | 5 | 0 | v、y；(u, r, x, ψ 保持 0) | v_ss = Y/d₂₂ ≈ 0.167 m/s；y(12) ≈ 1.75 m |
| 3 转向 | 0 | 0 | 5 | r、ψ；(u, v, x, y 保持 0) | r_ss = N/d₃₃ = 0.625 rad/s；ψ(12) ≈ 6.88 rad（≈394°） |

上述理论值按一阶惯性环节计算（时间常数 τ = m/d），用于报告中与仿真曲线对照验证模型正确性。工况 3 中 u = v = 0，科氏耦合项为 0，因此船体原地转向，x、y 恒为 0，这是该三自由度模型的必然结果，报告中会加以说明。

## 9. 脚本结构（第二段实现时按此编写）

| 代码段 | 功能 |
|---|---|
| 段1 参数区 | 集中定义第 7 节全部参数，注释"可自定义修改" |
| 段2 模型创建 | 关闭并覆盖同名旧模型，`new_system`、`open_system` |
| 段3 添加模块 | 按第 5 节逐块 `add_block`，并 `set_param` 设置 Position 与参数 |
| 段4 自动连线 | 按"端口连线关系"列逐条 `add_line`，启用 autorouting |
| 段5 仿真配置 | StopTime = 12，求解器保持默认变步长 |
| 段6 保存模型 | `save_system` 保存为 usv_3dof_fossen_openloop.slx |
| 段7 三工况仿真 | 依次设置 Const_X/Y/N，`sim` 运行 3 次并保存数据到结果结构体 |
| 段8 结果绘图 | 每个工况输出曲线图（u/x、v/y、r/ψ），保存为 figures 目录下的 PNG，供报告使用 |

## 10. 交付物

| 文件 | 说明 |
|---|---|
| design.md | 本设计说明（第一段产出，已提交审核） |
| build_usv_3dof_openloop.m | 模型搭建 + 三工况仿真 + 绘图脚本 |
| usv_3dof_fossen_openloop.slx | 生成的 Simulink 模型 |
| 3自由度Fossen无人船开环运动仿真实验报告.docx | 仿真实验报告（含模型截图、三种工况曲线与结论） |
| 工作日志.md | 记录操作步骤及建模耗时、报告生成耗时、总耗时 |

中间图片、临时数据与备份统一放在 E:\学校课程\build\海洋数据挖掘技术\C1 下，最终交付物放在本任务目录。

## 11. 风险点与兼容性

1. 全部使用 `add_block` / `add_line` / `set_param` / `save_system` / `sim` 稳定 API，兼容 R2016b 及以上版本。
2. Sum 的符号必须通过 `Inputs` 参数显式设置（'++-'、'+--'、'+-'、'++'），否则默认全加会导致阻尼项符号错误，仿真发散。
3. 三个 Integrator 的初始条件均为 0，保证"从静止开始"；若初速非零，单一激励下会出现科氏耦合，与工况说明不符。
4. 三角函数使用连续精确（ApproximationMethod = none），避免查表近似导致低速工况出现微小误差。
5. `sim` 返回值在不同版本中既有 SimulationOutput 对象也有写回基础工作区的方式，脚本统一用 `simOut.get('posData')` 读取，并对旧版本做兼容判断。
6. To Workspace 的 SaveFormat 固定为 'Structure With Time'，保证时间轴与三路信号同时记录，便于报告绘图。
7. 若同名 .slx 已在 MATLAB 中打开，保存前先 `close_system(mdl, 0)`，避免文件占用报错。

## 12. 待确认事项

1. 模型参数默认值（m₁₁=30、m₂₂=45、m₃₃=8；d₁₁=12、d₂₂=30、d₃₃=8）是否认可；若课程有指定船型参数，请提供，我按给定值修改。
2. 三种工况采用"同一模型 + 脚本修改 Constant 值依次仿真 3 次"的方案是否认可（而非建立 3 个独立模型文件）。
3. 输出观测采用"2 个 Scope（位置/姿态、速度）+ 2 个 To Workspace 记录数据"是否满足要求，是否需要额外增加 x–y 轨迹图（XY Graph）模块。
4. 文件名是否采用 build_usv_3dof_openloop.m、usv_3dof_fossen_openloop.slx、3自由度Fossen无人船开环运动仿真实验报告.docx、工作日志.md。

以上确认后进入第二段：生成 .m 脚本、运行模型、输出 .docx 报告与 .md 工作日志。
