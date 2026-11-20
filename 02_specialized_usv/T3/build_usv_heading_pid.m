%% =========================================================================
%  无人船航向 PID 控制 Simulink 模型自动生成与仿真
%  -------------------------------------------------------------------------
%  功能：
%    1) 自动搭建 usv_heading_pid.slx（12 个模块，含 2 个 usv_lib S-Function、
%       1 个 MATLAB Function 指令生成块）
%    2) 自动完成参数配置并执行仿真（ode4 固定步长 0.01 s，300 s）
%    3) 输出响应曲线、三阶段收敛指标、切换时刻稳定性校验
%    4) 生成 docx 实验报告
%  运行：直接运行本脚本（无需手工操作 Simulink 界面）
%  依赖：Simulink、MSS 工具箱（USV.m 需要 m2c/eulerang）、python-docx
%
%  航向指令时序（固定时刻切换，matlab function 内直接写 if t < xxx）：
%    0 <= t < T_SW1   : 30°
%    T_SW1 <= t < T_SW2: 230°
%    t >= T_SW2        : 30°
%  T_SW1 / T_SW2 由试跑实测收敛时刻 + 10 s 稳定要求 + 2 s 余量取整得到。
%% =========================================================================
clear; close all; clc;

T0 = tic;
ROOT = fileparts(mfilename('fullpath'));
if isempty(ROOT), ROOT = pwd; end
cd(ROOT);
RES = fullfile(ROOT, 'results');
if ~exist(RES, 'dir'), mkdir(RES); end

MODEL   = 'usv_heading_pid';
USV_LIB = 'E:\学校课程\海洋数据挖掘技术\作业\02_specialized_usv\usv_lib';

% ---- 切换时刻常量（试跑标定后回填）----
T_SW1 = 42;      % s，阶段1 -> 阶段2（实测收敛 29.93 s + 10 s + 2 s 余量）
T_SW2 = 104;     % s，阶段2 -> 阶段3（切换后实测收敛 49.55 s + 10 s + 2 s 余量）
T_END = 300;     % s，仿真总时长
TS    = 0.01;    % s，固定步长

timeRec = struct();

%% [1/8] 环境检查与 usv_lib 模块复制
fprintf('==== [1/8] 环境检查与模块复制 ====\n');
tSec = tic;
assert(~isempty(ver('simulink')), '未检测到 Simulink');

if exist('m2c', 'file') == 0 || exist('eulerang', 'file') == 0
    cand = {fullfile(matlabroot, 'toolbox', 'MSS'), ...
            'D:\MATLAB\R2024b\toolbox\MSS', ...
            'D:\MATLAB\R2024a\toolbox\MSS'};
    for k = 1:numel(cand)
        if exist(cand{k}, 'dir')
            addpath(genpath(cand{k}));
            fprintf('  已加载 MSS 路径: %s\n', cand{k});
            break;
        end
    end
end
assert(exist('m2c', 'file') ~= 0 && exist('eulerang', 'file') ~= 0, ...
    '未找到 MSS 工具箱函数 m2c/eulerang（USV.m 依赖），请安装 MSS 后重试');

mods = {'USV.m', 'Heading_PID.m'};
for k = 1:numel(mods)
    src = fullfile(USV_LIB, mods{k});
    assert(exist(src, 'file') == 2, sprintf('usv_lib 缺少模块文件：%s', src));
    dst = fullfile(ROOT, mods{k});
    if ~strcmpi(src, dst)
        copyfile(src, dst, 'f');
    end
    fprintf('  模块就绪: %s\n', dst);
end
addpath(ROOT);
timeRec.env = toc(tSec);

%% [2/8] 读取 usv_params.m 并组装配置
fprintf('==== [2/8] 读取 usv_params.m 并组装配置 ====\n');
tSec = tic;
run(fullfile(ROOT, 'usv_params.m'));      % MRB, MA, Dl, Dn, T, N_max, N_min

cfg_usv = struct('x0', zeros(6, 1), 'MRB', MRB, 'MA', MA, 'Dl', Dl, 'Dn', Dn);

Kp = 6;  Ki = 0.6;  Kd = 380;            % PID 增益（实测整定值）
cfg_pid = struct('T', T, 'Kp', Kp, 'Ki', Ki, 'Kd', Kd, ...
                 'N_max', N_max, 'N_min', N_min);

assignin('base', 'cfg_usv', cfg_usv);
assignin('base', 'cfg_pid', cfg_pid);
fprintf('  cfg_pid: Kp=%.3g, Ki=%.3g, Kd=%.3g, T=%.3g s, N∈[%.3g, %.3g] N·m\n', ...
    Kp, Ki, Kd, T, N_min, N_max);
timeRec.params = toc(tSec);

%% [3/8] 自动生成 Simulink 模型
fprintf('==== [3/8] 生成 Simulink 模型 ====\n');
tSec = tic;
if bdIsLoaded(MODEL), close_system(MODEL, 0); end
new_system(MODEL);
load_system(MODEL);

% ---------- 添加模块 ----------
addB(MODEL, 'simulink/Sources/Clock', 'Clock', [20 195 50 225]);
addB(MODEL, 'simulink/User-Defined Functions/MATLAB Function', ...
     'HeadingCmd', [100 180 210 240]);
addB(MODEL, 'simulink/Signal Routing/Mux', 'MuxCmd', [280 190 290 270], ...
     'Inputs', '2');
addB(MODEL, 'simulink/User-Defined Functions/S-Function', 'HeadingPID', ...
     [360 200 460 260], 'FunctionName', 'Heading_PID', 'Parameters', 'cfg_pid');
addB(MODEL, 'simulink/Sources/Constant', 'ZeroForce', [500 200 530 235], ...
     'Value', '0');
addB(MODEL, 'simulink/Signal Routing/Mux', 'MuxIn', [540 200 550 300], ...
     'Inputs', '3');
addB(MODEL, 'simulink/User-Defined Functions/S-Function', 'USVModel', ...
     [620 200 740 300], 'FunctionName', 'USV', 'Parameters', 'cfg_usv');
addB(MODEL, 'simulink/Signal Routing/Demux', 'Demux', [800 200 810 300], ...
     'Outputs', '6');
addB(MODEL, 'simulink/Sinks/Scope', 'ScopeHeading', [880 70 920 160], ...
     'NumInputPorts', '2');
addB(MODEL, 'simulink/Sinks/Scope', 'ScopeTorque', [640 55 675 95], ...
     'NumInputPorts', '1');
addB(MODEL, 'simulink/Sinks/To Workspace', 'ToWsPsiD', [250 70 350 115], ...
     'VariableName', 'sim_psi_d', 'SaveFormat', 'Timeseries', ...
     'SampleTime', '0.01');
addB(MODEL, 'simulink/Sinks/To Workspace', 'ToWsPsi', [880 190 980 240], ...
     'VariableName', 'sim_psi', 'SaveFormat', 'Timeseries', ...
     'SampleTime', '0.01');
addB(MODEL, 'simulink/Sinks/To Workspace', 'ToWsTorque', [700 90 790 130], ...
     'VariableName', 'sim_torque', 'SaveFormat', 'Timeseries', ...
     'SampleTime', '0.01');
addB(MODEL, 'simulink/Sinks/To Workspace', 'ToWsR', [880 260 980 300], ...
     'VariableName', 'sim_r', 'SaveFormat', 'Timeseries', ...
     'SampleTime', '0.01');

% ---------- MATLAB Function 写入期望航向分段指令 ----------
cmdCode = sprintf(['function psid = fcn(t)\n', ...
    '%%#codegen\n', ...
    'if t < %.6g\n', ...
    '    psid = 30*pi/180;\n', ...
    'elseif t < %.6g\n', ...
    '    psid = 230*pi/180;\n', ...
    'else\n', ...
    '    psid = 30*pi/180;\n', ...
    'end\n'], T_SW1, T_SW2);

chart = [];
charts = sfroot().find('-isa', 'Stateflow.EMChart');
for k = 1:numel(charts)
    if strcmp(charts(k).Path, [MODEL '/HeadingCmd'])
        chart = charts(k);
        break;
    end
end
assert(~isempty(chart), '未找到 HeadingCmd 对应的 MATLAB Function 图对象');
chart.Script = cmdCode;

% ---------- 连线 ----------
add_line(MODEL, 'Clock/1',        'HeadingCmd/1',  'autorouting', 'on');
add_line(MODEL, 'HeadingCmd/1',   'MuxCmd/1',      'autorouting', 'on');
add_line(MODEL, 'Demux/3',        'MuxCmd/2',      'autorouting', 'on');
add_line(MODEL, 'MuxCmd/1',       'HeadingPID/1',  'autorouting', 'on');
add_line(MODEL, 'HeadingPID/1',   'MuxIn/3',       'autorouting', 'on');
add_line(MODEL, 'ZeroForce/1',    'MuxIn/1',       'autorouting', 'on');
add_line(MODEL, 'ZeroForce/1',    'MuxIn/2',       'autorouting', 'on');
add_line(MODEL, 'MuxIn/1',        'USVModel/1',    'autorouting', 'on');
add_line(MODEL, 'USVModel/1',     'Demux/1',       'autorouting', 'on');
add_line(MODEL, 'HeadingCmd/1',   'ScopeHeading/1','autorouting', 'on');
add_line(MODEL, 'Demux/3',        'ScopeHeading/2','autorouting', 'on');
add_line(MODEL, 'HeadingPID/1',   'ScopeTorque/1', 'autorouting', 'on');
add_line(MODEL, 'HeadingCmd/1',   'ToWsPsiD/1',    'autorouting', 'on');
add_line(MODEL, 'Demux/3',        'ToWsPsi/1',     'autorouting', 'on');
add_line(MODEL, 'HeadingPID/1',   'ToWsTorque/1',  'autorouting', 'on');
add_line(MODEL, 'Demux/6',        'ToWsR/1',       'autorouting', 'on');

% ---------- 求解器与仿真配置 ----------
set_param(MODEL, ...
    'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '0.01', 'StartTime', '0', 'StopTime', num2str(T_END), ...
    'SimulationMode', 'normal', 'ReturnWorkspaceOutputs', 'off', ...
    'SaveTime', 'off', 'SaveOutput', 'off', 'SaveState', 'off', ...
    'SignalLogging', 'off', 'EnableMultiTasking', 'off');

save_system(MODEL, fullfile(ROOT, [MODEL '.slx']));
fprintf('  模型已保存: %s\n', fullfile(ROOT, [MODEL '.slx']));
timeRec.build = toc(tSec);

%% [4/8] 执行仿真
fprintf('==== [4/8] 执行仿真（%g s）====\n', T_END);
tSec = tic;
sim(MODEL);
timeRec.sim = toc(tSec);
assert(exist('sim_psi_d', 'var') == 1 && exist('sim_psi', 'var') == 1 && ...
       exist('sim_torque', 'var') == 1 && exist('sim_r', 'var') == 1, ...
       '仿真未产生 sim_psi_d / sim_psi / sim_torque / sim_r 数据');

t     = sim_psi.Time;
psi_d = sim_psi_d.Data;
psi   = sim_psi.Data;
N     = sim_torque.Data;
r     = sim_r.Data;                % 航向角速度（rad/s）
e     = angDiff(psi_d - psi);      % 归一化航向误差（rad）
fprintf('  仿真完成：%d 个采样点，耗时 %.1f s\n', numel(t), timeRec.sim);

%% [5/8] 数据处理与指标计算
fprintf('==== [5/8] 数据处理与阶段指标 ====\n');
tSec = tic;

% ---- 数据预处理：原始航向 [-180,180]，生成连续展开角 + 0~360° 映射 ----
psiDegRaw  = psi   * 180 / pi;             % 实际航向原始值（°）
psiDdegRaw = psi_d * 180 / pi;             % 期望航向原始值（°）
fprintf('  原始航向范围: 实际 [%.2f, %.2f]°，期望 [%.2f, %.2f]°\n', ...
    min(psiDegRaw), max(psiDegRaw), min(psiDdegRaw), max(psiDdegRaw));
assert(all(abs(psiDegRaw) <= 180 + 1e-6), ...
    '无人船模型输出的实际航向超出 [-180,180]，角度映射前提不成立');

% 图 1 使用：连续展开角（无 0/360 跳变），期望 230° 映射为等价 -130°
psiDegUnw   = unwrap(psiDegRaw * pi / 180) * 180 / pi;
psiDdegDisp = angleTo180(psiDdegRaw);

% 数据文件同时保留 0~360° 映射结果
psiDeg360  = angleTo360(psiDegRaw);
psiDdeg360 = angleTo360(psiDdegRaw);
errDeg     = e * 180 / pi;                 % 归一化航向误差（°）

assert(~any(isnan(psiDegUnw)) && ~any(isnan(psiDdegDisp)), '角度处理结果含 NaN');
fprintf('  展开角连续性检查: 相邻采样最大变化 %.3f°（无跳变）\n', ...
    max(abs(diff(psiDegUnw))));

T_heading = table(t, psiDdegDisp, psiDegUnw, errDeg, psiDdeg360, psiDeg360, ...
    'VariableNames', {'t_s', 'psi_d_deg', 'psi_deg', 'err_deg', ...
                      'psi_d_deg360', 'psi_deg360'});
writetable(T_heading, fullfile(RES, 'heading_data.csv'));

tolRad  = 0.1 * pi / 180;
targets = [30, 230, 30] * pi / 180;
tB      = [0, T_SW1, T_SW2, T_END];
stageName = {sprintf('阶段1：0°→30°  [0, %g)', T_SW1), ...
             sprintf('阶段2：30°→230°  [%g, %g)', T_SW1, T_SW2), ...
             sprintf('阶段3：230°→30°  [%g, %g]', T_SW2, T_END)};

tConvAbs = nan(1, 3); tConv = nan(1, 3);
osDeg = nan(1, 3);    sseDeg = nan(1, 3);
psi0All = nan(1, 3);

for k = 1:3
    if k < 3
        m = t >= tB(k) & t < tB(k + 1);
    else
        m = t >= tB(k) & t <= tB(k + 1);
    end
    tk = t(m); ek = e(m); pk = psi(m);

    % 收敛时刻：误差首次进入 ±0.1° 且此后不再越界
    bad = find(abs(ek) >= tolRad, 1, 'last');
    if isempty(bad)
        iC = 1;
    else
        iC = bad + 1;
    end
    if iC <= numel(tk)
        tConvAbs(k) = tk(iC);
        tConv(k)    = tk(iC) - tB(k);
    end

    % 超调量（以阶段起点到目标的短路径方向判定）
    target = targets(k);
    psi0 = pk(1);
    psi0All(k) = psi0;
    delta = angDiff(target - psi0);
    tAdj  = psi0 + delta;
    dev   = pk - tAdj;
    iX = find(sign(dev) == sign(delta) & abs(dev) > tolRad, 1);
    if isempty(iX)
        osDeg(k) = 0;
    elseif delta > 0
        osDeg(k) = max(0, max(dev(iX:end))) * 180 / pi;
    else
        osDeg(k) = max(0, -min(dev(iX:end))) * 180 / pi;
    end

    % 稳态误差：阶段最后 5 s 的平均 |e|
    tail = tk > (tB(k + 1) - 5);
    if any(tail)
        sseDeg(k) = mean(abs(ek(tail))) * 180 / pi;
    end
end

% 切换前 10 s 窗口内最大误差
win1 = t >= max(T_SW1 - 10, 0) & t < T_SW1;
win2 = t >= max(T_SW2 - 10, 0) & t < T_SW2;
m1max = max(abs(e(win1))) * 180 / pi;
m2max = max(abs(e(win2))) * 180 / pi;
ok1 = m1max < 0.1;
ok2 = m2max < 0.1;

fprintf('  标定参考: t_c1 = %.2f s, t_c2 = %.2f s\n', tConvAbs(1), tConvAbs(2));

% 指标 CSV
T_metrics = table((1:3)', {stageName{1}; stageName{2}; stageName{3}}, ...
    tB(1:3)', tB(2:4)', tConvAbs', tConv', osDeg', sseDeg', ...
    'VariableNames', {'stage', 'name', 't_start', 't_end', ...
                      't_conv_abs', 't_conv_elapsed', 'overshoot_deg', 'sse_deg'});
writetable(T_metrics, fullfile(RES, 'metrics.csv'));
timeRec.metrics = toc(tSec);

% 校验失败则给出明确提示（先保存数据，便于标定）
assert(ok1, sprintf(['阶段1切换前 10 s 窗口内误差未保持在 ±0.1° 以内' ...
    '（最大 %.4f°），请增大 T_SW1 后重跑'], m1max));
assert(ok2, sprintf(['阶段2切换前 10 s 窗口内误差未保持在 ±0.1° 以内' ...
    '（最大 %.4f°），请增大 T_SW2 后重跑'], m2max));
assert(~any(isnan(tConv)), '存在未收敛阶段，请检查 PID 增益或延长仿真时间');

%% [6/8] 绘制响应曲线
fprintf('==== [6/8] 绘制响应曲线 ====\n');
tSec = tic;
set(0, 'DefaultAxesFontName', 'Microsoft YaHei', ...
       'DefaultTextFontName', 'Microsoft YaHei');

% Simulink 模型结构截图（报告图 1）
print(['-s' MODEL], '-dpng', '-r200', fullfile(RES, 'fig_model.png'));

f1 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 920 400]);
plot(t, psiDdegDisp, '--', 'LineWidth', 1.2, 'Color', [0.85 0.33 0.10]); hold on;
plot(t, psiDegUnw, '-', 'LineWidth', 1.4, 'Color', [0 0.45 0.74]);
% 纵轴范围由连续曲线的实际取值决定（上下各留 10° 余量，按 30° 取整）
yAll = [psiDegUnw; psiDdegDisp];
yLo = floor((min(yAll) - 10) / 30) * 30;
yHi = ceil((max(yAll) + 10) / 30) * 30;
ylim([yLo yHi]); yticks(yLo:30:yHi);
yl = ylim;
plot([T_SW1 T_SW1], yl, '-.k', 'LineWidth', 1.0);
plot([T_SW2 T_SW2], yl, '-.k', 'LineWidth', 1.0);
text(T_SW1, yl(1) + 0.90 * diff(yl), ' 切换至 230°（-130°）', 'FontSize', 9);
text(T_SW2, yl(1) + 0.90 * diff(yl), ' 切换至 30°', 'FontSize', 9);
xlabel('时间 t / s'); ylabel('航向角 \psi / (°)'); grid on;
legend({'期望航向 \psi_d', '实际航向 \psi'}, 'Location', 'best');
title('无人船航向闭环响应（展开角连续显示，230°以等价-130°表示）');
exportgraphics(f1, fullfile(RES, 'fig_heading.png'), 'Resolution', 150);
close(f1);

f2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 920 380]);
semilogy(t, max(abs(errDeg), 1e-6), 'LineWidth', 1.3, 'Color', [0.2 0.5 0.2]);
hold on;
yline(0.1, 'r--', '0.1° 容差', 'LineWidth', 1.1, 'LabelHorizontalAlignment', 'left');
grid on; xlabel('时间 t / s'); ylabel('|航向误差| / (°)');
title('航向误差收敛过程（对数坐标）');
exportgraphics(f2, fullfile(RES, 'fig_error.png'), 'Resolution', 150);
close(f2);

f3 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 920 380]);
plot(t, N, 'LineWidth', 1.3, 'Color', [0.35 0.25 0.65]); hold on;
yline(N_max, 'r--', 'LineWidth', 1.0);
yline(N_min, 'r--', 'LineWidth', 1.0);
grid on; xlabel('时间 t / s'); ylabel('航向力矩 N / (N·m)');
title('PID 控制器输出航向力矩');
exportgraphics(f3, fullfile(RES, 'fig_torque.png'), 'Resolution', 150);
close(f3);

f4 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 920 380]);
plot(t, r * 180 / pi, 'LineWidth', 1.3, 'Color', [0.80 0.30 0.10]); hold on;
yline(0, 'k-', 'LineWidth', 0.8);
grid on; xlabel('时间 t / s'); ylabel('航向角速度 r / ((°)/s)');
title('航向角速度响应');
exportgraphics(f4, fullfile(RES, 'fig_yawrate.png'), 'Resolution', 150);
close(f4);
timeRec.figures = toc(tSec);

%% [7/8] 生成 docx 实验报告
fprintf('==== [7/8] 生成 docx 实验报告 ====\n');
tSec = tic;

RD = struct();
RD.meta = struct('title', '无人船航向 PID 控制仿真实验报告', ...
                 'model', [MODEL '.slx'], ...
                 'script', 'build_usv_heading_pid.m', ...
                 'date', datestr(now, 'yyyy-mm-dd'), ...
                 'sim_seconds', timeRec.sim);

RD.structure = struct();
RD.structure.text = { ...
    ['本实验以 usv_lib 模块库中的三自由度无人船运动模型（USV.m）与航向 PID 控制器' ...
     '（Heading_PID.m）为核心，搭建航向闭环控制 Simulink 模型。模型由期望航向指令生成、' ...
     'PID 控制器、无人船运动模型和数据采集输出四部分组成。'], ...
    ['期望航向由 MATLAB Function 块按时间分段给出：0~T_SW1 为 30°，T_SW1~T_SW2 为 230°，' ...
     'T_SW2 之后回到 30°。切换时刻为根据试跑实测收敛时间标定后的固定常量，不依赖在线判稳逻辑。'], ...
    ['图 1 为自动生成的 Simulink 模型截图：Clock 提供仿真时间，MATLAB Function 块按时间分段输出期望航向 ψd；' ...
     'ψd 与实际航向 ψ 经 Mux 合成后送入航向 PID 控制器（Heading_PID，S-Function），输出航向力矩 N；' ...
     'N 与置零的 X、Y 经 Mux 合成后送入 USV 运动模型（S-Function），其六维状态经 Demux 解包，' ...
     '取第 3 路航向角 ψ 反馈至控制器；ψd、ψ、N 同时送入 Scope 与 To Workspace 记录。']};
RD.structure.header = {'模块ID', '模块名称', '类型 / 来源', '功能说明'};
RD.structure.rows = { ...
    {'B01', '仿真时钟', 'Clock（Simulink 标准库）', '输出仿真时间 t'}, ...
    {'B02', '期望航向指令生成', 'MATLAB Function（Simulink 标准库）', '按 t 分段输出 30°/230°/30°'}, ...
    {'B03', '控制器输入合成', 'Mux（Simulink 标准库）', '合成 [ψd; ψ]'}, ...
    {'B04', '模型输入合成', 'Mux（Simulink 标准库）', '合成 [X; Y; N]'}, ...
    {'B05', '纵荡/横荡力置零', 'Constant（Simulink 标准库）', 'X = Y = 0'}, ...
    {'B06', '航向 PID 控制器', 'S-Function Heading_PID（usv_lib）', '[ψd; ψ] → N'}, ...
    {'B07', '无人船运动模型', 'S-Function USV（usv_lib）', '[X,Y,N] → 6 维状态'}, ...
    {'B08', '状态解包', 'Demux（Simulink 标准库）', '取第 3 路 ψ'}, ...
    {'B09', 'Scope-航向', 'Scope（Simulink 标准库）', '显示 ψd、ψ'}, ...
    {'B10', 'Scope-力矩', 'Scope（Simulink 标准库）', '显示 N'}, ...
    {'B11', '期望航向记录', 'To Workspace（Simulink 标准库）', '记录 sim_psi_d'}, ...
    {'B12', '实际航向记录', 'To Workspace（Simulink 标准库）', '记录 sim_psi'}, ...
    {'B13', '力矩数据记录', 'To Workspace（Simulink 标准库）', '记录 sim_torque'}, ...
    {'B14', '角速度记录', 'To Workspace（Simulink 标准库）', '记录 sim_r（航向角速度）'}};
RD.structure.figure = struct('path', fullfile(RES, 'fig_model.png'), ...
    'caption', '无人船航向 PID 控制 Simulink 模型结构');

RD.params_ship = struct();
RD.params_ship.header = {'参数', '取值', '说明'};
RD.params_ship.rows = { ...
    {'MRB', mat2str(MRB), '本体惯量矩阵'}, ...
    {'MA',  mat2str(MA),  '附加惯量矩阵'}, ...
    {'Dl',  mat2str(Dl),  '线性阻尼矩阵'}, ...
    {'Dn',  mat2str(Dn),  '非线性阻尼矩阵'}, ...
    {'x0',  mat2str(cfg_usv.x0(:)'), '状态初值（初始航向 0°）'}};

RD.params_pid = struct();
RD.params_pid.header = {'参数', '取值', '说明'};
RD.params_pid.rows = { ...
    {'Kp', sprintf('%.4g', Kp), '比例增益 / (N·m/rad)'}, ...
    {'Ki', sprintf('%.4g', Ki), '积分增益'}, ...
    {'Kd', sprintf('%.4g', Kd), '微分增益'}, ...
    {'T',  sprintf('%.4g s', T), '控制周期'}, ...
    {'N_max / N_min', sprintf('±%.4g N·m', N_max), '航向力矩限幅'}};

RD.params_sim = struct();
RD.params_sim.header = {'项目', '取值', '说明'};
RD.params_sim.rows = { ...
    {'求解器', 'ode4', '固定步长四阶 Runge-Kutta'}, ...
    {'固定步长', '0.01 s', '与控制器周期一致'}, ...
    {'仿真时长', sprintf('%g s', T_END), '覆盖三个阶段'}, ...
    {'切换时刻', sprintf('T_SW1 = %g s, T_SW2 = %g s', T_SW1, T_SW2), '固定时刻切换'}, ...
    {'运行模式', 'Normal', 'S-Function 依赖 gcb'}};

RD.calibration = struct();
RD.calibration.text = { ...
    sprintf(['切换时刻标定：试跑实测阶段 1 误差首次稳定进入 ±0.1° 的时刻为 t_c1 = %.2f s，' ...
             '阶段 2 为 t_c2 = %.2f s。按“收敛后再保持 10 s，另加 2 s 余量”取整，' ...
             '得到 T_SW1 = %g s、T_SW2 = %g s，并以常量形式直接写入期望航向生成函数。'], ...
             tConvAbs(1), tConvAbs(2), T_SW1, T_SW2), ...
    ['本报告中的响应曲线为最终固定时刻切换方案的实际仿真结果，切换前 10 s 窗口的' ...
     '误差校验结果见表 6。']};

RD.figures = struct( ...
    'path', {fullfile(RES, 'fig_heading.png'), ...
             fullfile(RES, 'fig_error.png'), ...
             fullfile(RES, 'fig_torque.png'), ...
             fullfile(RES, 'fig_yawrate.png')}, ...
    'caption', {'航向角响应（展开角连续显示，230°以等价-130°表示）', ...
                '航向误差绝对值收敛过程', ...
                'PID 输出航向力矩', ...
                '航向角速度响应曲线'});

RD.metrics = struct();
RD.metrics.header = {'阶段', '时间段 / s', '收敛时刻 / s', ...
                     '收敛耗时 / s', '超调量 / (°)', '稳态误差 / (°)'};
RD.metrics.rows = { ...
    {stageName{1}, sprintf('%g ~ %g', tB(1), tB(2)), sprintf('%.2f', tConvAbs(1)), ...
     sprintf('%.2f', tConv(1)), sprintf('%.3f', osDeg(1)), sprintf('%.4f', sseDeg(1))}, ...
    {stageName{2}, sprintf('%g ~ %g', tB(2), tB(3)), sprintf('%.2f', tConvAbs(2)), ...
     sprintf('%.2f', tConv(2)), sprintf('%.3f', osDeg(2)), sprintf('%.4f', sseDeg(2))}, ...
    {stageName{3}, sprintf('%g ~ %g', tB(3), tB(4)), sprintf('%.2f', tConvAbs(3)), ...
     sprintf('%.2f', tConv(3)), sprintf('%.3f', osDeg(3)), sprintf('%.4f', sseDeg(3))}};

RD.checks = struct();
RD.checks.header = {'校验项', '时间窗口 / s', '窗口内最大误差 / (°)', '结论'};
RD.checks.rows = { ...
    {'阶段1切换前稳定保持', sprintf('[%g, %g]', max(T_SW1 - 10, 0), T_SW1), ...
     sprintf('%.4f', m1max), ternaryStr(ok1, '通过', '不通过')}, ...
    {'阶段2切换前稳定保持', sprintf('[%g, %g]', max(T_SW2 - 10, 0), T_SW2), ...
     sprintf('%.4f', m2max), ternaryStr(ok2, '通过', '不通过')}};

peakN = max(abs(N));
RD.analysis = { ...
    sprintf(['本次仿真时长 %g s，固定步长 %g s，共 %d 个采样点。期望航向按 30° → 230° → 30° ' ...
             '三阶段切换，实际航向在 PID 控制下逐段收敛。'], T_END, TS, numel(t)), ...
    sprintf(['阶段 1（0° → 30°）：实际航向在 %.2f s 时首次进入 ±0.1° 误差带并保持稳定，' ...
             '耗时 %.2f s，超调量 %.3f°，阶段末稳态误差 %.4f°。'], ...
             tConvAbs(1), tConv(1), osDeg(1), sseDeg(1)), ...
    sprintf(['阶段 2（30° → 230°）：控制器按归一化误差走最短路径（由 30° 顺时针转向 230°，' ...
             '图中以等价 -130° 显示），于 %.2f s 收敛，耗时 %.2f s，超调量 %.3f°，' ...
             '阶段末稳态误差 %.4f°。'], ...
             tConvAbs(2), tConv(2), osDeg(2), sseDeg(2)), ...
    sprintf(['阶段 3（230° → 30°）：再次回切后于 %.2f s 收敛，耗时 %.2f s，超调量 %.3f°，' ...
             '阶段末稳态误差 %.4f°。'], tConvAbs(3), tConv(3), osDeg(3), sseDeg(3)), ...
    sprintf(['两处切换时刻前 10 s 窗口内的最大航向误差分别为 %.4f° 与 %.4f°，均小于 0.1°，' ...
             '满足“误差小于 ±0.1° 且保持稳定 10 s 以上”的切换条件。全程控制力矩峰值 %.2f N·m，' ...
             '远低于 ±%g N·m 的限幅；航向角速度峰值 %.2f °/s，各阶段收敛后角速度趋于 0，' ...
             '表明航向闭环系统收敛平稳、无持续振荡，稳态精度满足 0.1° 要求。'], ...
             m1max, m2max, peakN, N_max, max(abs(r))*180/pi)};

jsonPath = fullfile(RES, 'report_data.json');
fid = fopen(jsonPath, 'w', 'n', 'UTF-8');
fprintf(fid, '%s', jsonencode(RD));
fclose(fid);

docxTmp = fullfile(RES, 'report_tmp.docx');
docxOut = fullfile(ROOT, 'USV航向PID控制仿真实验报告.docx');
pyScript = fullfile(ROOT, 'gen_report.py');
[st, pyVer] = system('python --version');
assert(st == 0, '未找到 python，无法生成 docx 报告');
fprintf('  %s\n', strtrim(pyVer));
[st, pyOut] = system(sprintf('python "%s" "%s" "%s"', pyScript, jsonPath, docxTmp));
assert(st == 0, 'python 报告生成失败：\n%s', pyOut);
movefile(docxTmp, docxOut, 'f');
fprintf('  报告已生成: %s\n', docxOut);
timeRec.report = toc(tSec);

%% [8/8] 汇总与耗时记录
fprintf('==== [8/8] 汇总 ====\n');
timeRec.total = toc(T0);
fid = fopen(fullfile(RES, 'timing.json'), 'w', 'n', 'UTF-8');
fprintf(fid, '%s', jsonencode(timeRec));
fclose(fid);

fprintf('\n================ 仿真结果汇总 ================\n');
fprintf('阶段1 收敛: t = %6.2f s（耗时 %5.2f s）, 超调 %5.3f°, 稳态误差 %.4f°\n', ...
    tConvAbs(1), tConv(1), osDeg(1), sseDeg(1));
fprintf('阶段2 收敛: t = %6.2f s（耗时 %5.2f s）, 超调 %5.3f°, 稳态误差 %.4f°\n', ...
    tConvAbs(2), tConv(2), osDeg(2), sseDeg(2));
fprintf('阶段3 收敛: t = %6.2f s（耗时 %5.2f s）, 超调 %5.3f°, 稳态误差 %.4f°\n', ...
    tConvAbs(3), tConv(3), osDeg(3), sseDeg(3));
fprintf('切换前窗口最大误差: %.4f° / %.4f°（阈值 0.1°）\n', m1max, m2max);
fprintf('力矩峰值: %.2f N·m（限幅 ±%g N·m）\n', peakN, N_max);
fprintf('模型: %s\n报告: %s\n', fullfile(ROOT, [MODEL '.slx']), docxOut);
fprintf('脚本总耗时: %.1f s\n', timeRec.total);

%% ---------------- 局部函数 ----------------
function addB(mdl, lib, name, pos, varargin)
% 添加模块并设置位置/参数
add_block(lib, [mdl '/' name], 'Position', pos, varargin{:});
end

function d = angDiff(x)
% 角度归一化到 [-pi, pi]
d = atan2(sin(x), cos(x));
end

function s = ternaryStr(cond, a, b)
if cond, s = a; else, s = b; end
end

function y = angleTo180(deg)
% 角度映射函数：将任意角度映射为等价的 (-180,180] 角度
% 用于绘图时把 230° 表示为与其同方位的 -130°（与 PID 最短路径一致）
y = mod(deg + 180, 360) - 180;
end

function y = angleTo360(deg)
% 角度映射函数：将角度统一映射为 [0,360)
% 结果写入 heading_data.csv 的 0~360° 列
y = mod(deg, 360);
y(abs(y - 360) < 1e-9) = 0;
end
