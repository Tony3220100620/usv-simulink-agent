%% ========================================================================
%  无人船 3 自由度 Fossen 开环运动仿真模型：建模 + 三工况仿真（MATLAB 脚本）
%
%  功能：
%    1) 用 add_block / add_line 等标准命令搭建 usv_openloop_3dof.slx
%    2) 依次运行三个单独施力工况（X=10 N、Y=5 N、N=5 N·m），仿真 12 s
%    3) 绘制位置、姿态、速度与平面轨迹图，并输出末端状态数值
%
%  模块来源：
%    - 无人船模型：usv_lib 的 USV.m（S-Function，3 自由度 Fossen 方程）
%    - 其余模块：Simulink 官方公共库
%
%  运行方式：在 MATLAB 中直接运行本脚本即可（脚本位于模型所在目录）
% =========================================================================

clear; close all; clc;

%% 0 路径与目录准备 -------------------------------------------------------
rootDir  = fileparts(mfilename('fullpath'));   % 脚本所在目录 = 任务工作路径
cd(rootDir);
addpath(rootDir);

figDir = fullfile(rootDir, 'figures');         % 结果图片目录
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

modelName = 'usv_openloop_3dof';
modelFile = fullfile(rootDir, [modelName '.slx']);

%% 1 读取无人船动力学参数 -------------------------------------------------
% usv_params.m 定义 MRB / MA / Dl / Dn 四个矩阵
run(fullfile(rootDir, 'usv_params.m'));

cfg = struct();
cfg.x0  = zeros(6, 1);      % 状态初值 [x_u; y; psi; u; v; r]，静止于原点
cfg.MRB = MRB;              % 刚体惯量矩阵
cfg.MA  = MA;               % 附加惯量矩阵
cfg.Dl  = Dl;               % 线性阻尼矩阵
cfg.Dn  = Dn;               % 非线性阻尼矩阵

%% 2 复制 usv_lib 模块文件到工作路径 ---------------------------------------
% 规则要求：使用 usv_lib 模块时须复制到当前工作路径后再搭建，不直接引用原目录
libSfun = 'E:\学校课程\海洋数据挖掘技术\作业\02_specialized_usv\usv_lib\USV.m';
localSfun = fullfile(rootDir, 'USV.m');
if ~isfile(localSfun)
    copyfile(libSfun, localSfun);
end

%% 3 新建模型 -------------------------------------------------------------
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if isfile(modelFile)
    delete(modelFile);
end
new_system(modelName);       % 新建空模型，载入内存

%% 4 添加模块 -------------------------------------------------------------
% 布局原则：按信号流向从左到右分列排列（输入列 -> 合成列 -> 模型列 -> 拆分列 -> 显示列），
% 各列之间保持固定水平间距，同一列内模块按端口序号自上而下排布，避免模块相互遮挡。

% ---- 4.1 三个常值输入（力 X、力 Y、力矩 N），数值来自工作区变量 ----
add_block('simulink/Sources/Constant', [modelName '/In_X'], ...
    'Value', 'tauX', 'SampleTime', 'inf', 'Position', [40 60 100 90]);

add_block('simulink/Sources/Constant', [modelName '/In_Y'], ...
    'Value', 'tauY', 'SampleTime', 'inf', 'Position', [40 150 100 180]);

add_block('simulink/Sources/Constant', [modelName '/In_N'], ...
    'Value', 'tauN', 'SampleTime', 'inf', 'Position', [40 240 100 270]);

% ---- 4.2 输入合成：3 维控制输入向量 [X; Y; N] ----
add_block('simulink/Signal Routing/Mux', [modelName '/Mux3'], ...
    'Inputs', '3', 'DisplayOption', 'bar', 'Position', [155 105 160 225]);

% ---- 4.3 无人船 3 自由度模型（usv_lib 的 USV.m，S-Function 模块） ----
add_block('simulink/User-Defined Functions/S-Function', [modelName '/USV_Model'], ...
    'FunctionName', 'USV', 'Parameters', 'cfg', 'Position', [215 120 335 210]);

% ---- 4.4 状态拆分：6 维状态 -> 6 个标量信号 ----
add_block('simulink/Signal Routing/Demux', [modelName '/Demux6'], ...
    'Outputs', '6', 'Position', [395 80 400 250]);

% ---- 4.5 平面轨迹图（x-y），置于显示列顶部，与端口 1、2 同侧 ----
add_block('simulink/Sinks/XY Graph', [modelName '/XY_Graph'], ...
    'xmin', '-5', 'xmax', '5', 'ymin', '-5', 'ymax', '5', ...
    'Position', [495 40 585 120]);

% ---- 4.6 位姿示波器（x, y, psi），与 XY 图保持足够纵向间距，避免图名压线 ----
add_block('simulink/Sinks/Scope', [modelName '/Scope_Pose'], ...
    'NumInputPorts', '3', 'Position', [495 200 535 260]);

% ---- 4.7 速度示波器（u, v, r） ----
add_block('simulink/Sinks/Scope', [modelName '/Scope_Vel'], ...
    'NumInputPorts', '3', 'Position', [495 310 535 370]);

% ---- 4.8 状态记录到工作区，置于模型下方，由模型输出引出支路 ----
add_block('simulink/Sinks/To Workspace', [modelName '/To_Workspace'], ...
    'VariableName', 'usv_out', 'SaveFormat', 'Array', ...
    'Position', [215 400 335 440]);

%% 5 连线 ---------------------------------------------------------------
add_line(modelName, 'In_X/1', 'Mux3/1', 'autorouting', 'on');
add_line(modelName, 'In_Y/1', 'Mux3/2', 'autorouting', 'on');
add_line(modelName, 'In_N/1', 'Mux3/3', 'autorouting', 'on');

add_line(modelName, 'Mux3/1', 'USV_Model/1', 'autorouting', 'on');

add_line(modelName, 'USV_Model/1', 'Demux6/1', 'autorouting', 'on');

% 平面轨迹：x_u -> XY Graph 的 x 端口，y -> y 端口（先连，保证分支从主干就近引出）
add_line(modelName, 'Demux6/1', 'XY_Graph/1', 'autorouting', 'on');
add_line(modelName, 'Demux6/2', 'XY_Graph/2', 'autorouting', 'on');

% 位姿：x_u -> Scope_Pose 1，y -> 2，psi -> 3
add_line(modelName, 'Demux6/1', 'Scope_Pose/1', 'autorouting', 'on');
add_line(modelName, 'Demux6/2', 'Scope_Pose/2', 'autorouting', 'on');
add_line(modelName, 'Demux6/3', 'Scope_Pose/3', 'autorouting', 'on');

% 速度：u -> Scope_Vel 1，v -> 2，r -> 3
add_line(modelName, 'Demux6/4', 'Scope_Vel/1', 'autorouting', 'on');
add_line(modelName, 'Demux6/5', 'Scope_Vel/2', 'autorouting', 'on');
add_line(modelName, 'Demux6/6', 'Scope_Vel/3', 'autorouting', 'on');

% 状态记录：直接记录 6 维状态向量
add_line(modelName, 'USV_Model/1', 'To_Workspace/1', 'autorouting', 'on');

%% 6 模型级配置与保存 -----------------------------------------------------
set_param(modelName, ...
    'Solver',   'ode45', ...   % 变步长求解器
    'StopTime', '12', ...      % 仿真时长 12 s
    'SaveTime', 'on', ...      % 记录时间向量 tout
    'SaveOutput', 'off', ...
    'SaveState', 'off');

save_system(modelName, modelFile);

% 导出模型整体布局截图（供仿真报告插图使用）
print(['-s' modelName], '-dpng', '-r300', fullfile(figDir, 'model_layout.png'));

%% 7 三工况仿真 ----------------------------------------------------------
% 每个工况单独运行一次，通过改写基础工作区 tauX / tauY / tauN 实现
caseName = {'C1_单施X_10N', 'C2_单施Y_5N', 'C3_单施N_5Nm'};
tauList  = [10 0 0;          % C1：仅纵荡力 10 N
             0 5 0;          % C2：仅横荡力 5 N
             0 0 5];         % C3：仅艏摇力矩 5 N·m

res = struct('name', {}, 't', {}, 'x', {}, 'y', {}, 'psi', {}, ...
             'u', {}, 'v', {}, 'r', {});

for k = 1:size(tauList, 1)
    tauX = tauList(k, 1);
    tauY = tauList(k, 2);
    tauN = tauList(k, 3);

    simOut = sim(modelName);   % 单输出模式：时间与状态记录在 SimulationOutput 中

    % 取出时间向量与 6 维状态矩阵（记录格式可能为 timeseries 或结构体）
    tVec    = simOut.tout;
    dataMat = simOut.usv_out;
    if isa(dataMat, 'timeseries')
        dataMat = dataMat.Data;
    elseif isstruct(dataMat)
        dataMat = dataMat.signals.values;
    end
    dataMat = reshape(dataMat, numel(tVec), []);   % 统一为 N×6 矩阵

    res(k).name = caseName{k};
    res(k).t    = tVec;
    res(k).x    = dataMat(:, 1);   % 纵荡位置 x_u (m)
    res(k).y    = dataMat(:, 2);   % 横荡位置 y (m)
    res(k).psi  = dataMat(:, 3);   % 航向角 psi (rad，输出已折算到 (-pi, pi])
    res(k).u    = dataMat(:, 4);   % 纵荡速度 u (m/s)
    res(k).v    = dataMat(:, 5);   % 横荡速度 v (m/s)
    res(k).r    = dataMat(:, 6);   % 艏摇角速度 r (rad/s)
end

%% 8 绘图 ----------------------------------------------------------------
% 说明：图例条目是含下划线的工况标识字符串（如 C1_单施X_10N），必须设置
% Interpreter = 'none'，否则 '_' 会被 TeX 解释器当作下标标记，把后续字符
% 排成下标；而坐标轴标签、标题等需要数学排版的文本仍保留默认 tex 解释器。
colorSet = [0.00 0.45 0.74;    % 蓝：C1
            0.85 0.33 0.10;    % 橙：C2
            0.47 0.67 0.19];   % 绿：C3

% 8.1 位置与姿态
figPose = figure('Name', '位置与姿态', 'Color', 'w', 'Position', [100 100 760 720]);
poseName = {'x (m)', 'y (m)', '\psi (rad)'};
poseField = {'x', 'y', 'psi'};
for i = 1:3
    subplot(3, 1, i);
    hold on; grid on; box on;
    for k = 1:numel(res)
        plot(res(k).t, res(k).(poseField{i}), 'LineWidth', 1.4, ...
            'Color', colorSet(k, :));
    end
    ylabel(poseName{i});
    if i == 1
        title('三种单自由度输入工况：位置与姿态');
        legend(caseName, 'Location', 'northwest', 'Interpreter', 'none');
    end
    if i == 3
        xlabel('时间 t (s)');
    end
end
exportgraphics(figPose, fullfile(figDir, 'case_pose.png'), 'Resolution', 160);

% 8.2 速度
figVel = figure('Name', '速度', 'Color', 'w', 'Position', [100 100 760 720]);
velName = {'u (m/s)', 'v (m/s)', 'r (rad/s)'};
velField = {'u', 'v', 'r'};
for i = 1:3
    subplot(3, 1, i);
    hold on; grid on; box on;
    for k = 1:numel(res)
        plot(res(k).t, res(k).(velField{i}), 'LineWidth', 1.4, ...
            'Color', colorSet(k, :));
    end
    ylabel(velName{i});
    if i == 1
        title('三种单自由度输入工况：速度');
        legend(caseName, 'Location', 'northwest', 'Interpreter', 'none');
    end
    if i == 3
        xlabel('时间 t (s)');
    end
end
exportgraphics(figVel, fullfile(figDir, 'case_vel.png'), 'Resolution', 160);

% 8.3 x-y 平面轨迹
figXY = figure('Name', '平面轨迹', 'Color', 'w', 'Position', [100 100 560 500]);
hold on; grid on; box on;
for k = 1:numel(res)
    plot(res(k).x, res(k).y, 'LineWidth', 1.6, 'Color', colorSet(k, :));
end
xlabel('x (m)'); ylabel('y (m)');
title('三种单自由度输入工况：x-y 平面轨迹');
legend(caseName, 'Location', 'best', 'Interpreter', 'none');
axis equal;
exportgraphics(figXY, fullfile(figDir, 'case_xy.png'), 'Resolution', 160);

%% 9 末端数值输出 -------------------------------------------------------
fprintf('\n================ 仿真末端状态（t = %.1f s） ================\n', res(1).t(end));
fprintf('%-16s %9s %9s %9s %9s %9s %9s\n', ...
    '工况', 'x(m)', 'y(m)', 'psi(rad)', 'u(m/s)', 'v(m/s)', 'r(rad/s)');
for k = 1:numel(res)
    fprintf('%-16s %9.3f %9.3f %9.3f %9.4f %9.4f %9.4f\n', ...
        res(k).name, res(k).x(end), res(k).y(end), res(k).psi(end), ...
        res(k).u(end), res(k).v(end), res(k).r(end));
end

% 运动判定（阈值仅用于说明“是否产生明显运动”）
thPos = 0.05;      % 位移阈值 0.05 m
thPsi = 0.05;      % 航向变化阈值 0.05 rad
fprintf('\n---------------- 运动判定 ----------------\n');
fprintf('C1 前进运动：|dx| = %.3f m -> %s\n', abs(res(1).x(end)), ...
    ternary(abs(res(1).x(end)) > thPos, '是', '否'));
fprintf('C2 横向运动：|dy| = %.3f m -> %s\n', abs(res(2).y(end)), ...
    ternary(abs(res(2).y(end)) > thPos, '是', '否'));
fprintf('C3 转向运动：|dpsi| 累计转动（psi 为折算角，按 r 积分） = %.3f rad -> %s\n', ...
    abs(trapz(res(3).t, res(3).r)), ...
    ternary(abs(trapz(res(3).t, res(3).r)) > thPsi, '是', '否'));

%% 10 局部函数 ---------------------------------------------------------
function s = ternary(cond, a, b)
%TERNARY 简易条件选择，避免在脚本中写冗长的 if-else
if cond
    s = a;
else
    s = b;
end
end
