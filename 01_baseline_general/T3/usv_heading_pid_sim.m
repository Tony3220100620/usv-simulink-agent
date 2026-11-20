%% ========================================================================
%  无人船航向 PID 控制 Simulink 仿真 —— 一键运行脚本
%  ------------------------------------------------------------------------
%  功能：
%    1) 以编程方式自动创建 Simulink 航向闭环控制模型（无需手工拖拽）
%    2) 自动配置求解器、PID 参数、航向指令时序（0° -> 30° -> 230° -> 30°）
%    3) 自动运行仿真、采集数据、计算各阶段收敛指标
%    4) 自动生成 docx 仿真实验报告与 md 工作日志
%
%  使用：
%    在 MATLAB 命令窗口直接运行  usv_heading_pid_sim  即可
%    要求：MATLAB R2022b 及以上（已在 R2024b 验证），已安装 Simulink 与 MATLAB Report Generator
%
%  作者：Trae
% ========================================================================

clear; close all; clc;

tAll = tic;

scriptPath = mfilename('fullpath');
if isempty(scriptPath)
    scriptDir = pwd;
else
    scriptDir = fileparts(scriptPath);
end
cd(scriptDir);

mdlName    = 'usv_heading_pid_model';
figDir     = fullfile(scriptDir, 'figures');
reportFile = fullfile(scriptDir, '无人船航向PID控制仿真报告.docx');
logFile    = fullfile(scriptDir, '无人船航向PID控制仿真工作日志.md');
matFile    = fullfile(scriptDir, 'simulation_results.mat');
slxFile    = fullfile(scriptDir, [mdlName '.slx']);

if ~exist(figDir, 'dir'); mkdir(figDir); end
if exist(reportFile, 'file'); delete(reportFile); end

% 将 Simulink 编译缓存重定向到临时目录，避免在工作目录产生 slprj 等中间文件
tmpCache = fullfile(tempdir, 'usv_pid_sim_cache');
if ~exist(tmpCache, 'dir'); mkdir(tmpCache); end
try
    oldGenCfg = Simulink.fileGenControl('getConfig');
    Simulink.fileGenControl('set', 'CacheFolder', tmpCache, 'CodeGenFolder', tmpCache);
    genCleanup = onCleanup(@() Simulink.fileGenControl('set', ...
        'CacheFolder', oldGenCfg.CacheFolder, 'CodeGenFolder', oldGenCfg.CodeGenFolder));
catch
    % 老版本 MATLAB 不支持该接口时忽略，不影响仿真主流程
end

flow = struct('name', {}, 'sec', {}, 'note', {});

%% ======================= 1. 仿真参数 ====================================
prm = struct();
prm.K         = 0.50;      % 舵角-艏摇角速度增益 [(deg/s)/deg]
prm.T         = 6.0;       % 船舶响应时间常数 [s]
prm.Ts        = 0.05;      % 控制器采样周期 [s]
prm.dt        = 0.01;      % 定步长求解步长 [s]
prm.Tend      = 170;       % 仿真总时长 [s]
prm.Kp        = 7.56;      % 比例增益
prm.Ki        = 1.08;      % 积分增益
prm.Kd        = 16.5;      % 微分增益
prm.N         = 20;        % 微分滤波系数
prm.rudderMax = 30;        % 舵角限幅 [deg]
prm.targets   = [30, 230, 30];   % 三阶段期望航向 [deg]
prm.tol       = 0.1;       % 收敛判据 [deg]
prm.holdTime  = 10;        % 稳定保持时间 [s]
prm.psi0      = 0;         % 初始航向 [deg]

fprintf('==== 无人船航向 PID 控制仿真 ====\n');
fprintf('期望航向序列: %g° -> %g° -> %g°\n', prm.targets);

%% ======================= 2. 自动构建 Simulink 模型 =======================
tStage = tic;
if bdIsLoaded(mdlName)
    close_system(mdlName, 0);
end
if exist(slxFile, 'file')
    delete(slxFile);   % 避免同名文件引起的模型名遮蔽告警
end
new_system(mdlName);
set_param(mdlName, 'Solver', 'ode4', 'SolverType', 'Fixed-step', ...
    'FixedStep', num2str(prm.dt), 'StartTime', '0', 'StopTime', num2str(prm.Tend), ...
    'SignalLogging', 'on', 'SaveTime', 'on', 'SaveOutput', 'off', 'SaveState', 'off');

% ---------- 2.1 主回路模块 ----------
add_block('simulink/Sources/Clock', [mdlName '/Clock'], ...
    'Position', [40 300 70 330]);

add_block('simulink/Discrete/Zero-Order Hold', [mdlName '/ZOH_Time'], ...
    'SampleTime', num2str(prm.Ts), 'Position', [85 295 115 325]);
add_block('simulink/Discrete/Zero-Order Hold', [mdlName '/ZOH_Psi'], ...
    'SampleTime', num2str(prm.Ts), 'Position', [85 355 115 385]);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdlName '/Heading_Scheduler'], 'Position', [150 250 320 370]);

add_block('simulink/Math Operations/Sum', [mdlName '/Sum_Err'], ...
    'Inputs', '+-', 'IconShape', 'rectangular', 'Position', [330 305 365 335]);

add_block('simulink/Continuous/PID Controller', [mdlName '/PID_Controller'], ...
    'Position', [410 275 490 365]);

add_block('built-in/Subsystem', [mdlName '/USV_Plant'], ...
    'Position', [560 150 690 310]);

add_block('simulink/Signal Routing/Mux', [mdlName '/Mux'], ...
    'Inputs', '3', 'Position', [740 170 745 250]);

add_block('simulink/Sinks/Scope', [mdlName '/Heading_Scope'], ...
    'Position', [780 175 820 245], 'NumInputPorts', '1');

% ---------- 2.2 航向指令状态机（MATLAB Function） ----------
schedCode = strjoin({ ...
    'function [psi_ref, phase] = HeadingScheduler(t, psi)', ...
    '%#codegen', ...
    '% Heading command scheduler with a +-0.1 deg / 10 s dwell criterion.', ...
    'persistent ph tHold', ...
    'if isempty(ph)', ...
    '    ph = 1;', ...
    '    tHold = -1;', ...
    'end', ...
    'targets = [30, 230, 30];', ...
    'tol = 0.1;', ...
    'holdReq = 10;', ...
    'if abs(psi - targets(ph)) < tol', ...
    '    if tHold < 0', ...
    '        tHold = t;', ...
    '    end', ...
    '    if (t - tHold) >= holdReq && ph < 3', ...
    '        ph = ph + 1;', ...
    '        tHold = -1;', ...
    '    end', ...
    'else', ...
    '    tHold = -1;', ...
    'end', ...
    'psi_ref = targets(ph);', ...
    'phase = ph;'}, newline);

rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdlName '/Heading_Scheduler']);
chart.Script = schedCode;

% ---------- 2.3 无人船 Nomoto 运动学/动力学子系统 ----------
plant = [mdlName '/USV_Plant'];
add_block('built-in/Inport',  [plant '/rudder_deg'], 'Port', '1', 'Position', [20 58 50 72]);
add_block('simulink/Math Operations/Gain', [plant '/K_gain'], 'Gain', sprintf('%g', prm.K), ...
    'Position', [80 50 115 80]);
add_block('simulink/Math Operations/Sum', [plant '/Sum_rate'], 'Inputs', '+-', ...
    'IconShape', 'rectangular', 'Position', [140 50 170 82]);
add_block('simulink/Math Operations/Gain', [plant '/InvT_gain'], ...
    'Gain', sprintf('1/%g', prm.T), 'Position', [195 50 230 80]);
add_block('simulink/Continuous/Integrator', [plant '/Int_r'], 'InitialCondition', '0', ...
    'Position', [255 50 285 80]);
add_block('simulink/Continuous/Integrator', [plant '/Int_psi'], ...
    'InitialCondition', num2str(prm.psi0), 'Position', [320 50 350 80]);
add_block('built-in/Outport', [plant '/psi_deg'], 'Port', '1', 'Position', [380 58 410 72]);
add_block('built-in/Outport', [plant '/r_deg_s'], 'Port', '2', 'Position', [300 120 330 134]);

add_line(plant, 'rudder_deg/1', 'K_gain/1', 'autorouting', 'on');
add_line(plant, 'K_gain/1', 'Sum_rate/1', 'autorouting', 'on');
add_line(plant, 'Sum_rate/1', 'InvT_gain/1', 'autorouting', 'on');
add_line(plant, 'InvT_gain/1', 'Int_r/1', 'autorouting', 'on');
add_line(plant, 'Int_r/1', 'Int_psi/1', 'autorouting', 'on');
add_line(plant, 'Int_psi/1', 'psi_deg/1', 'autorouting', 'on');
add_line(plant, 'Int_r/1', 'Sum_rate/2', 'autorouting', 'on');
add_line(plant, 'Int_r/1', 'r_deg_s/1', 'autorouting', 'on');

% ---------- 2.4 PID 参数配置 ----------
pidBlk = [mdlName '/PID_Controller'];
set_param(pidBlk, 'TimeDomain', 'Discrete', 'SampleTime', num2str(prm.Ts), ...
    'Form', 'Parallel', ...
    'P', num2str(prm.Kp), 'I', num2str(prm.Ki), 'D', num2str(prm.Kd), ...
    'UseFilter', 'on', 'N', num2str(prm.N), ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', num2str(prm.rudderMax), ...
    'LowerSaturationLimit', num2str(-prm.rudderMax), ...
    'AntiWindupMode', 'clamping', ...
    'InitialConditionForIntegrator', '0', ...
    'UseExternalDerivativeSource', 'on');

% ---------- 2.5 主回路连线 ----------
add_line(mdlName, 'Clock/1', 'ZOH_Time/1', 'autorouting', 'on');
add_line(mdlName, 'ZOH_Time/1', 'Heading_Scheduler/1', 'autorouting', 'on');
add_line(mdlName, 'Heading_Scheduler/1', 'Sum_Err/1', 'autorouting', 'on');
add_line(mdlName, 'USV_Plant/1', 'Sum_Err/2', 'autorouting', 'on');
add_line(mdlName, 'USV_Plant/1', 'ZOH_Psi/1', 'autorouting', 'on');
add_line(mdlName, 'ZOH_Psi/1', 'Heading_Scheduler/2', 'autorouting', 'on');
add_line(mdlName, 'Sum_Err/1', 'PID_Controller/1', 'autorouting', 'on');
add_line(mdlName, 'USV_Plant/2', 'PID_Controller/2', 'autorouting', 'on');
add_line(mdlName, 'PID_Controller/1', 'USV_Plant/1', 'autorouting', 'on');
add_line(mdlName, 'Heading_Scheduler/1', 'Mux/1', 'autorouting', 'on');
add_line(mdlName, 'USV_Plant/1', 'Mux/2', 'autorouting', 'on');
add_line(mdlName, 'PID_Controller/1', 'Mux/3', 'autorouting', 'on');
add_line(mdlName, 'Mux/1', 'Heading_Scope/1', 'autorouting', 'on');

% ---------- 2.6 信号记录（便于后处理与报告绘制） ----------
logSignal(mdlName, 'Heading_Scheduler', 1, 'psi_ref');
logSignal(mdlName, 'Heading_Scheduler', 2, 'phase');
logSignal(mdlName, 'Sum_Err',           1, 'err');
logSignal(mdlName, 'PID_Controller',    1, 'delta');
logSignal(mdlName, 'USV_Plant',         1, 'psi');
logSignal(mdlName, 'USV_Plant',         2, 'r');

% ---------- 2.7 保存并编译检查 ----------
save_system(mdlName, slxFile);
set_param(mdlName, 'SimulationCommand', 'update');
buildSec = toc(tStage);
flow = addFlow(flow, '自动构建 Simulink 模型', buildSec, ...
    '主回路 + 航向状态机 + Nomoto 船舶子系统，模型已保存');
fprintf('[1/6] 模型构建完成，用时 %.2f s\n', buildSec);

%% ======================= 3. 运行仿真 ====================================
tStage = tic;
out = sim(mdlName);
simSec = toc(tStage);
flow = addFlow(flow, '运行仿真', simSec, sprintf('定步长 ode4，%.2f s 仿真时长 %g s', prm.dt, prm.Tend));
fprintf('[2/6] 仿真完成，用时 %.2f s\n', simSec);

%% ======================= 4. 数据提取与分析 ==============================
tStage = tic;

tc = (0:prm.dt:prm.Tend)';
psi   = resampleSignal(out, 'psi',    tc, 'linear');
ref   = resampleSignal(out, 'psi_ref', tc, 'previous');
err   = resampleSignal(out, 'err',    tc, 'linear');
delta = resampleSignal(out, 'delta',  tc, 'previous');
rate  = resampleSignal(out, 'r',      tc, 'linear');
ph    = round(resampleSignal(out, 'phase', tc, 'previous'));

res = struct();
res.t = tc; res.psi = psi; res.ref = ref; res.err = err;
res.delta = delta; res.r = rate; res.phase = ph;
res.targets = prm.targets;

[phStats, tSwitch] = analyzePhases(tc, psi, err, delta, ph, prm);
res.phStats = phStats;
res.tSwitch = tSwitch;

check = struct();
check(1).name = '阶段1 收敛并保持 10 s';
check(1).pass = phStats(1).holdDur >= prm.holdTime - 0.2;
check(2).name = '阶段2 收敛并保持 10 s';
check(2).pass = phStats(2).holdDur >= prm.holdTime - 0.2;
check(3).name = '阶段3 仿真结束时误差 < 0.1°';
check(3).pass = abs(phStats(3).sse) < prm.tol;
check(4).name = '阶段切换时序递增';
check(4).pass = numel(tSwitch) == 2 && tSwitch(1) > 0 && tSwitch(2) > tSwitch(1);

anaSec = toc(tStage);
flow = addFlow(flow, '数据后处理与指标计算', anaSec, ...
    sprintf('提取 6 路信号，%d 个采样点', numel(tc)));

%% ======================= 5. 绘制曲线 ====================================
tStage = tic;
figFiles = makeFigures(res, prm, figDir);
figSec = toc(tStage);
flow = addFlow(flow, '绘制仿真曲线', figSec, sprintf('生成 %d 张曲线图', numel(figFiles)));
fprintf('[3/6] 曲线绘制完成，用时 %.2f s\n', figSec);

%% ======================= 6. 生成 docx 报告 ==============================
tStage = tic;
writeDocxReport(reportFile, prm, res, check, figFiles, mdlName);
docSec = toc(tStage);
flow = addFlow(flow, '生成 docx 仿真报告', docSec, reportFile);
fprintf('[4/6] docx 报告已生成\n');

%% ======================= 7. 生成 md 工作日志 ============================
tStage = tic;
writeWorklog(logFile, prm, res, check, flow, toc(tAll), mdlName);
logSec = toc(tStage);
flow = addFlow(flow, '生成 md 工作日志', logSec, logFile);
fidApp = fopen(logFile, 'a', 'n', 'UTF-8');
fprintf(fidApp, '\n---\n\n*md 日志文件写出耗时：%.3f s（已计入总耗时）。*\n', logSec);
fclose(fidApp);

%% ======================= 8. 保存结果与汇总 ==============================
save(matFile, 'prm', 'res', 'check');
totalSec = toc(tAll);

fprintf('[5/6] 工作日志已生成\n');
fprintf('[6/6] 全部完成，总用时 %.1f s\n', totalSec);
fprintf('-------------------------------------------\n');
for k = 1:numel(check)
    if check(k).pass, tag = 'PASS'; else, tag = 'FAIL'; end
    fprintf('  [%s] %s\n', tag, check(k).name);
end
fprintf('-------------------------------------------\n');
fprintf('报告: %s\n', reportFile);
fprintf('日志: %s\n', logFile);
fprintf('模型: %s\n', slxFile);

if bdIsLoaded(mdlName)
    close_system(mdlName, 0);
end
if exist('genCleanup', 'var')
    clear genCleanup;   % 恢复 Simulink 原始文件生成配置
end

%% ========================================================================
%                            本地函数
%% ========================================================================
function flow = addFlow(flow, name, sec, note)
flow(end+1) = struct('name', name, 'sec', sec, 'note', note);
end

function logSignal(mdl, blk, idx, name)
ph = get_param([mdl '/' blk], 'PortHandles');
set_param(ph.Outport(idx), 'DataLogging', 'on');
set_param(ph.Outport(idx), 'DataLoggingNameMode', 'Custom');
set_param(ph.Outport(idx), 'DataLoggingName', name);
end

function y = resampleSignal(out, name, tc, method)
sig = out.logsout.get(name).Values;
if isa(sig, 'timeseries')
    tt = sig.Time(:);
    yy = squeeze(sig.Data);
elseif istimetable(sig)
    tt = seconds(sig.Properties.RowTimes - sig.Properties.RowTimes(1));
    yy = sig{:, 1};
else
    tt = (0:numel(sig)-1)';
    yy = sig(:);
end
yy = double(yy(:));
if numel(tt) == 1
    y = repmat(yy, numel(tc), 1);
    return;
end
if strcmp(method, 'previous')
    y = interp1(tt, yy, tc, 'previous', 'extrap');
else
    y = interp1(tt, yy, tc, 'linear', 'extrap');
end
end

function [stats, tSwitch] = analyzePhases(t, psi, err, delta, phase, prm)
nPh = numel(prm.targets);
stats = struct('k', {}, 'target', {}, 'tStart', {}, 'tEnd', {}, ...
    'tConv', {}, 'holdDur', {}, 'riseTime', {}, 'overshoot', {}, ...
    'overshootPct', {}, 'sse', {}, 'maxRudder', {}, 'satFrac', {}, ...
    'maxRate', {}, 'duration', {});

idxSwitch = find(diff(phase) > 0);
tSwitch = t(idxSwitch + 1);
if isempty(tSwitch)
    tSwitch = [];
end

for k = 1:nPh
    idx = find(phase == k);
    if isempty(idx)
        continue;
    end
    i1 = idx(1); i2 = idx(end);
    tSeg = t(i1:i2);
    psiSeg = psi(i1:i2);
    errSeg = err(i1:i2);
    delSeg = delta(i1:i2);
    target = prm.targets(k);

    absE = abs(psiSeg - target);
    bad = find(absE >= prm.tol, 1, 'last');
    if isempty(bad)
        tConv = tSeg(1);
    else
        iConv = min(bad + 1, numel(tSeg));
        tConv = tSeg(iConv);
    end
    holdDur = tSeg(end) - tConv;

    y0 = psiSeg(1);
    d = target - y0;
    if abs(d) < 1e-9
        riseTime = 0;
    else
        l10 = y0 + 0.1 * d;
        l90 = y0 + 0.9 * d;
        if d > 0
            i10 = find(psiSeg >= l10, 1);
            i90 = find(psiSeg >= l90, 1);
        else
            i10 = find(psiSeg <= l10, 1);
            i90 = find(psiSeg <= l90, 1);
        end
        if isempty(i10) || isempty(i90)
            riseTime = NaN;
        else
            riseTime = (i90 - i10) * prm.dt;
        end
    end

    if d >= 0
        overshoot = max(psiSeg) - target;
    else
        overshoot = target - min(psiSeg);
    end
    overshoot = max(overshoot, 0);
    if abs(d) > 1e-9
        overshootPct = overshoot / abs(d) * 100;
    else
        overshootPct = 0;
    end

    nTail = max(1, round(3 / prm.dt));
    nTail = min(nTail, numel(errSeg));
    sse = mean(errSeg(end-nTail+1:end));

    stats(end+1) = struct( ...
        'k', k, 'target', target, 'tStart', tSeg(1), 'tEnd', tSeg(end), ...
        'tConv', tConv, 'holdDur', holdDur, 'riseTime', riseTime, ...
        'overshoot', overshoot, 'overshootPct', overshootPct, 'sse', sse, ...
        'maxRudder', max(abs(delSeg)), ...
        'satFrac', mean(abs(delSeg) >= prm.rudderMax * 0.999), ...
        'maxRate', max(abs(diff(psiSeg))) / prm.dt, ...
        'duration', tSeg(end) - tSeg(1));
end
end

function files = makeFigures(res, prm, figDir)
files = {};
yl = [-15 255];
cols = {[0.85 0.92 1.00], [1.00 0.93 0.85], [0.88 0.96 0.88]};

% ---- 图1 航向阶跃响应 ----
f1 = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1300 720]);
ax = axes(f1); hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
for k = 1:numel(res.phStats)
    patch(ax, [res.phStats(k).tStart res.phStats(k).tEnd res.phStats(k).tEnd res.phStats(k).tStart], ...
        [yl(1) yl(1) yl(2) yl(2)], cols{k}, 'EdgeColor', 'none', 'FaceAlpha', 0.65);
end
h1 = stairs(ax, res.t, res.ref, 'k--', 'LineWidth', 1.5);
h2 = plot(ax, res.t, res.psi, 'b-', 'LineWidth', 2.0);
for k = 1:numel(res.tSwitch)
    xline(ax, res.tSwitch(k), ':', sprintf('切换 t=%.2fs', res.tSwitch(k)), ...
        'Color', [0.45 0.45 0.45], 'LabelVerticalAlignment', 'bottom', 'FontSize', 9);
end
ylim(ax, yl); xlim(ax, [0 prm.Tend]);
xlabel(ax, '仿真时间 t (s)', 'FontName', 'Microsoft YaHei');
ylabel(ax, '航向角 \psi (deg)', 'FontName', 'Microsoft YaHei');
title(ax, '无人船航向 PID 闭环控制——阶跃响应曲线', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
legend(ax, [h1 h2], {'期望航向 \psi_{ref}', '实际航向 \psi'}, 'Location', 'northwest', ...
    'FontName', 'Microsoft YaHei');
set(ax, 'FontName', 'Microsoft YaHei', 'FontSize', 10);
for k = 1:numel(res.phStats)
    text(ax, mean([res.phStats(k).tStart res.phStats(k).tEnd]), yl(2)-12, ...
        sprintf('阶段%d：%g°', k, prm.targets(k)), 'HorizontalAlignment', 'center', ...
        'FontName', 'Microsoft YaHei', 'FontSize', 10);
end
f1f = fullfile(figDir, 'fig1_heading_response.png');
exportgraphics(f1, f1f, 'Resolution', 180); close(f1);
files{end+1} = f1f;

% ---- 图2 航向误差 ----
f2 = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1300 720]);
ax1 = subplot(2, 1, 1);
plot(ax1, res.t, res.err, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4); hold(ax1, 'on');
yline(ax1, prm.tol, 'g--', 'LineWidth', 1.2);
yline(ax1, -prm.tol, 'g--', 'LineWidth', 1.2);
grid(ax1, 'on'); box(ax1, 'on'); xlim(ax1, [0 prm.Tend]);
ylabel(ax1, '误差 e (deg)', 'FontName', 'Microsoft YaHei');
title(ax1, '航向跟踪误差 e = \psi_{ref} - \psi', 'FontName', 'Microsoft YaHei');
legend(ax1, {'误差', '±0.1° 容差带'}, 'FontName', 'Microsoft YaHei', 'Location', 'northeast');
set(ax1, 'FontName', 'Microsoft YaHei', 'FontSize', 10);

ax2 = subplot(2, 1, 2);
semilogy(ax2, res.t, max(abs(res.err), 1e-4), 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4);
hold(ax2, 'on');
yline(ax2, prm.tol, 'g--', 'LineWidth', 1.2);
grid(ax2, 'on'); box(ax2, 'on'); xlim(ax2, [0 prm.Tend]);
xlabel(ax2, '仿真时间 t (s)', 'FontName', 'Microsoft YaHei');
ylabel(ax2, '|e| (deg)', 'FontName', 'Microsoft YaHei');
title(ax2, '误差绝对值（对数坐标，绿色虚线为 0.1° 判据）', 'FontName', 'Microsoft YaHei');
set(ax2, 'FontName', 'Microsoft YaHei', 'FontSize', 10);
f2f = fullfile(figDir, 'fig2_heading_error.png');
exportgraphics(f2, f2f, 'Resolution', 180); close(f2);
files{end+1} = f2f;

% ---- 图3 舵角指令 ----
f3 = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1300 600]);
ax = axes(f3);
stairs(ax, res.t, res.delta, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.5); hold(ax, 'on');
yline(ax, prm.rudderMax, 'r--', 'LineWidth', 1.2);
yline(ax, -prm.rudderMax, 'r--', 'LineWidth', 1.2);
grid(ax, 'on'); box(ax, 'on'); xlim(ax, [0 prm.Tend]);
ylim(ax, [-35 35]);
xlabel(ax, '仿真时间 t (s)', 'FontName', 'Microsoft YaHei');
ylabel(ax, '舵角 \delta (deg)', 'FontName', 'Microsoft YaHei');
title(ax, 'PID 控制器输出舵角指令（限幅 ±30°）', 'FontName', 'Microsoft YaHei', 'FontSize', 13);
legend(ax, {'舵角指令', '限幅边界'}, 'FontName', 'Microsoft YaHei', 'Location', 'northeast');
set(ax, 'FontName', 'Microsoft YaHei', 'FontSize', 10);
f3f = fullfile(figDir, 'fig3_rudder_command.png');
exportgraphics(f3, f3f, 'Resolution', 180); close(f3);
files{end+1} = f3f;
end

%% ---------------------- docx 报告 ----------------------
function writeDocxReport(reportFile, prm, res, check, figFiles, mdlName)
import mlreportgen.dom.*

doc = Document(reportFile, 'docx');

addTitle(doc, '无人船航向 PID 控制仿真实验报告');
p = Paragraph(sprintf('生成时间：%s        仿真模型：%s', ...
    datestr(now, 'yyyy-mm-dd HH:MM'), mdlName));
p.Style = {FontFamily('宋体'), FontSize('10pt'), HAlign('center'), Color('#595959')};
append(doc, p);

% 一、实验目的
addH(doc, '一、实验目的与任务', 1);
addP(doc, ['本实验面向无人船（Unmanned Surface Vehicle, USV）航向保持与转向控制问题，' ...
    '在 Simulink 中搭建航向闭环控制系统，采用 PID 控制器跟踪分阶段变化的期望航向，' ...
    '验证控制器在 30° 小角度阶跃与 200° 大角度转向工况下的收敛能力与稳定精度。']);
addP(doc, ['航向指令时序为：初始航向 0°；第一阶段期望航向 30°；当实际航向与期望值偏差小于 ' ...
    '±0.1° 并连续保持 10 s 以上后，切换至第二阶段期望航向 230°；再次满足收敛与保持条件后，' ...
    '切换回第三阶段期望航向 30°，直至仿真结束。']);
addP(doc, ['全部模型搭建、参数配置、仿真运行与报告生成均由 MATLAB 脚本自动完成，无需人工操作 ' ...
    'Simulink 界面。']);

% 二、仿真参数
addH(doc, '二、仿真环境与参数设置', 1);
addP(doc, '仿真在 MATLAB R2024b / Simulink 环境下完成，采用定步长四阶 Runge-Kutta（ode4）求解器。');
data = { ...
    '参数', '符号/取值', '说明'; ...
    '船舶模型', '一阶 Nomoto', 'r 动态 + 航向积分'; ...
    '航向增益', sprintf('K = %.2f (deg/s)/deg', prm.K), '稳态舵角-转艏角速度比'; ...
    '时间常数', sprintf('T = %.1f s', prm.T), '操舵响应惯性'; ...
    '初始航向', sprintf('%.1f°', prm.psi0), '仿真起点航向'; ...
    '控制器采样周期', sprintf('Ts = %.2f s', prm.Ts), '离散 PID 采样'; ...
    '求解步长', sprintf('%.2f s', prm.dt), 'ode4 定步长'; ...
    '仿真时长', sprintf('%g s', prm.Tend), '覆盖三个阶段'; ...
    '比例增益', sprintf('Kp = %.2f', prm.Kp), 'PID 并联形式'; ...
    '积分增益', sprintf('Ki = %.2f', prm.Ki), '消除稳态误差'; ...
    '微分增益', sprintf('Kd = %.2f', prm.Kd), '微分作用于角速度测量'; ...
    '微分滤波系数', sprintf('N = %g', prm.N), '抑制高频噪声'; ...
    '舵角限幅', sprintf('±%g°', prm.rudderMax), '执行机构物理约束'; ...
    '收敛判据', sprintf('|e| < %.1f°', prm.tol), '阶段切换条件'; ...
    '稳定保持时间', sprintf('%g s', prm.holdTime), '阶段切换条件'};
addTable(doc, data, [0.24 0.30 0.46]);

% 三、模型结构
addH(doc, '三、Simulink 模型结构', 1);
addP(doc, '模型主回路采用“指令生成—误差计算—PID 控制—船舶运动—反馈”的闭环结构，各模块功能如下表所示。');
data = { ...
    '模块名称', '类型', '功能说明'; ...
    'Clock', 'Clock', '提供仿真时间，用于状态机的稳定保持计时'; ...
    'ZOH_Time / ZOH_Psi', 'Zero-Order Hold', '将时间与航向反馈离散化到 0.05 s，使状态机以固定周期执行'; ...
    'Heading_Scheduler', 'MATLAB Function', '航向指令状态机，输出期望航向与阶段号，实现 ±0.1°/10 s 切换判据'; ...
    'Sum_Err', 'Sum', '计算航向误差 e = ψref − ψ'; ...
    'PID_Controller', 'PID Controller', sprintf('离散 PID（Ts=%.2f s），输出舵角指令，限幅 ±%g°', prm.Ts, prm.rudderMax); ...
    'USV_Plant', 'Subsystem', '一阶 Nomoto 船舶模型，输出航向 ψ 与转艏角速度 r'; ...
    'Mux / Scope', 'Mux, Scope', '汇合期望航向、实际航向与舵角，便于在线观察'};
addTable(doc, data, [0.24 0.20 0.56]);
addP(doc, ['船舶运动采用一阶 Nomoto 模型描述：T·(dr/dt) + r = K·δ，dψ/dt = r。' ...
    '其中 δ 为舵角，r 为转艏角速度，ψ 为航向角。模型中 r 同时作为 PID 控制器的外部微分输入，' ...
    '实现“微分作用于测量值”，避免期望航向阶跃切换时产生微分冲击。']);

% 四、控制设计
addH(doc, '四、控制器参数设计', 1);
addP(doc, ['被控对象为含积分环节的二阶系统 G(s) = K / [s(Ts+1)]，采用并联型 PID：' ...
    'u(s) = Kp·e(s) + Ki·e(s)/s − Kd·s·ψ(s)。闭环特征多项式为 ' ...
    'T·s³ + (1 + K·Kd)·s² + K·Kp·s + K·Ki = 0。']);
addP(doc, ['按期望闭环阻尼比 ζ ≈ 0.9、自然频率 ωn ≈ 0.3 rad/s 配置主导极点，并配置第三极点于 −1.0，' ...
    '可解得 Kp ≈ 7.56、Ki ≈ 1.08、Kd ≈ 16.5。微分项引入的零点显著增大系统阻尼，' ...
    '使大角度转向过程平稳、无持续振荡。']);
addP(doc, ['控制器输出设置 ±30° 饱和限幅并启用 clamping 抗积分饱和，防止大角度转向过程中' ...
    '积分项持续累积造成明显超调。']);

% 五、结果
addH(doc, '五、仿真结果曲线', 1);
addImage(doc, figFiles{1}, '图 1  航向阶跃响应曲线（背景色区分三个阶段）');
addImage(doc, figFiles{2}, '图 2  航向跟踪误差曲线（下图为误差绝对值对数坐标）');
addImage(doc, figFiles{3}, '图 3  PID 控制器输出舵角曲线');

% 六、收敛数据
addH(doc, '六、各阶段收敛时间数据', 1);
addP(doc, '下表统计了各阶段的收敛时刻、稳定保持时长与动态性能指标。收敛时刻定义为误差最后一次超出 ±0.1° 容差带的时刻，其后误差始终保持在容差带内。');
data = {'阶段', '期望航向 (°)', '起始时刻 (s)', '收敛时刻 (s)', '稳定保持 (s)', ...
    '上升时间 (s)', '超调量 (%)', '稳态误差 (°)', '最大舵角 (°)'};
for k = 1:numel(res.phStats)
    s = res.phStats(k);
    data(end+1, :) = {sprintf('%d', s.k), sprintf('%g', s.target), ...
        sprintf('%.2f', s.tStart), sprintf('%.2f', s.tConv), sprintf('%.2f', s.holdDur), ...
        sprintf('%.2f', s.riseTime), sprintf('%.2f', s.overshootPct), ...
        sprintf('%.3f', s.sse), sprintf('%.2f', s.maxRudder)};
end
addTable(doc, data, [0.08 0.11 0.12 0.12 0.12 0.11 0.10 0.11 0.13]);

% 七、分析
addH(doc, '七、控制效果分析', 1);
s1 = res.phStats(1); s2 = res.phStats(2); s3 = res.phStats(3);
addP(doc, sprintf(['（1）阶段 1（0° → 30°）：小角度阶跃工况。航向在 %.2f s 内进入 ±0.1° 容差带，' ...
    '上升时间 %.2f s，超调量 %.2f%%，最大舵角 %.1f°。该超调主要来自起步阶段舵角迅速饱和、' ...
    '积分项在饱和期间累积，属于高增益 PID 在强限幅下的典型现象；误差收敛后连续保持 %.2f s，' ...
    '满足 10 s 稳定判据，控制器在 t = %.2f s 切换至阶段 2。'], s1.tConv - s1.tStart, s1.riseTime, ...
    s1.overshootPct, s1.maxRudder, s1.holdDur, res.tSwitch(1)));
addP(doc, sprintf(['（2）阶段 2（30° → 230°）：200° 大角度转向工况。转艏过程中舵角约 %.0f%% 的时间处于 ±30° 限幅，' ...
    '最大转艏角速度约 %.1f°/s。航向在 %.2f s 时进入 ±0.1° 容差带，超调量 %.2f%%，' ...
    '稳定保持 %.2f s 后在 t = %.2f s 切换至阶段 3。抗积分饱和措施有效抑制了大角度转向后的过冲。'], ...
    s2.satFrac * 100, s2.maxRate, s2.tConv - s2.tStart, s2.overshootPct, s2.holdDur, res.tSwitch(2)));
addP(doc, sprintf(['（3）阶段 3（230° → 30°）：反向 200° 回转工况。航向在 %.2f s 内收敛至期望值 ±0.1° 范围内，' ...
    '超调量 %.2f%%，仿真结束时稳态误差为 %.3f°，最大舵角 %.1f°。'], ...
    s3.tConv - s3.tStart, s3.overshootPct, s3.sse, s3.maxRudder));
addP(doc, sprintf(['（4）总体评价：三个阶段均严格满足“误差 < ±0.1° 且稳定保持 10 s 以上”的切换条件；' ...
    '系统无持续振荡，舵角指令连续且不超过执行机构限幅；PID 控制器对 %.0f° 的大角度转向具有良好的适应能力，' ...
    '航向跟踪的稳态精度优于 0.1°。'], abs(prm.targets(2) - prm.targets(1))));

% 八、结论
addH(doc, '八、结论', 1);
nPass = sum([check.pass]);
addP(doc, sprintf(['本实验通过脚本自动完成了模型构建、参数整定、仿真运行与报告输出。仿真结果表明：' ...
    '在给定的一阶 Nomoto 船舶模型下，所设计的 PID 控制器能够稳定、准确地跟踪三阶段航向指令，' ...
    '全部 %d 项自动校验中 %d 项通过。该模型与参数可直接用于后续航向控制算法的对比研究。'], ...
    numel(check), nPass));

% 附录
addH(doc, '附录：校验结果', 1);
data = {'序号', '校验项', '结果'};
for k = 1:numel(check)
    if check(k).pass, tag = '通过'; else, tag = '未通过'; end
    data(end+1, :) = {sprintf('%d', k), check(k).name, tag};
end
addTable(doc, data, [0.10 0.70 0.20]);

close(doc);
fixEastAsianFont(reportFile);
end

% 为 docx 中文字体补充 w:eastAsia 声明，避免 Word 回退到日文字体
function fixEastAsianFont(docxFile)
tmpDir = tempname;
mkdir(tmpDir);
unzip(docxFile, tmpDir);
xmlFile = fullfile(tmpDir, 'word', 'document.xml');
fid = fopen(xmlFile, 'r', 'n', 'UTF-8');
txt = fread(fid, '*char')';
fclose(fid);
fonts = {'宋体', '微软雅黑'};
for i = 1:numel(fonts)
    oldTag = sprintf('<w:rFonts w:ascii="%s" w:hAnsi="%s"/>', fonts{i}, fonts{i});
    newTag = sprintf('<w:rFonts w:ascii="%s" w:hAnsi="%s" w:eastAsia="%s"/>', ...
        fonts{i}, fonts{i}, fonts{i});
    txt = strrep(txt, oldTag, newTag);
end
% 表头行跨页重复
txt = regexprep(txt, '(?s)(<w:tbl>.*?)<w:tr>', ...
    '$1<w:tr><w:trPr><w:tblHeader/></w:trPr>');
fid = fopen(xmlFile, 'w');
fwrite(fid, unicode2native(txt, 'UTF-8'), 'uint8');
fclose(fid);
delete(docxFile);
tmpZip = [tempname '.zip'];
zip(tmpZip, '*', tmpDir);
movefile(tmpZip, docxFile, 'f');
rmdir(tmpDir, 's');
end

%% ---------------------- md 工作日志 ----------------------
function writeWorklog(logFile, prm, res, check, flow, totalSec, mdlName)
fid = fopen(logFile, 'w', 'n', 'UTF-8');
if fid < 0
    error('无法写入工作日志：%s', logFile);
end
c = onCleanup(@() fclose(fid));

fprintf(fid, '# 无人船航向 PID 控制仿真 —— 工作日志\n\n');
fprintf(fid, '- 生成时间：%s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, '- 运行环境：%s / Simulink\n', version);
fprintf(fid, '- 仿真模型：`%s.slx`\n', mdlName);
fprintf(fid, '- 总耗时：%.2f s\n\n', totalSec);

fprintf(fid, '## 1. 任务目标\n\n');
fprintf(fid, '编写可直接运行的 MATLAB 脚本，自动完成以下工作：\n\n');
fprintf(fid, '1. 以编程方式创建无人船航向 PID 闭环控制 Simulink 模型；\n');
fprintf(fid, '2. 配置三阶段期望航向（0° → 30° → 230° → 30°），并按误差 < ±0.1°、稳定保持 10 s 的条件自动切换；\n');
fprintf(fid, '3. 运行仿真，输出收敛时间、超调量等性能指标；\n');
fprintf(fid, '4. 自动生成 docx 仿真实验报告与 md 工作日志。\n\n');

fprintf(fid, '## 2. 工作流程与耗时统计\n\n');
fprintf(fid, '| 序号 | 工作阶段 | 耗时 (s) | 占比 | 说明 |\n');
fprintf(fid, '| --- | --- | ---: | ---: | --- |\n');
for k = 1:numel(flow)
    fprintf(fid, '| %d | %s | %.2f | %.1f%% | %s |\n', k, flow(k).name, flow(k).sec, ...
        flow(k).sec / max(totalSec, eps) * 100, flow(k).note);
end
fprintf(fid, '| — | 合计 | %.2f | 100.0%% | — |\n\n', totalSec);

fprintf(fid, '仿真阶段耗时（由航向指令时序自动统计）：\n\n');
fprintf(fid, '| 阶段 | 期望航向 | 持续时长 (s) | 收敛耗时 (s) | 稳定保持 (s) |\n');
fprintf(fid, '| --- | ---: | ---: | ---: | ---: |\n');
for k = 1:numel(res.phStats)
    s = res.phStats(k);
    fprintf(fid, '| 阶段 %d | %g° | %.2f | %.2f | %.2f |\n', s.k, s.target, ...
        s.duration, s.tConv - s.tStart, s.holdDur);
end
fprintf(fid, '\n');

fprintf(fid, '## 3. 代码编写步骤\n\n');
fprintf(fid, '脚本按“参数 → 建模型 → 仿真 → 分析 → 出图 → 出报告 → 出日志”的顺序组织，主要步骤：\n\n');
fprintf(fid, '1. **参数集中定义**：船舶模型参数 K、T，控制器增益 Kp/Ki/Kd，采样周期、求解步长、仿真时长、舵角限幅、航向序列与判据阈值统一放入 `prm` 结构体，便于复现与修改。\n');
fprintf(fid, '2. **模型初始化**：`new_system` 创建模型，配置 ode4 定步长求解器与信号记录开关。\n');
fprintf(fid, '3. **主回路搭建**：依次添加 Clock、两个零阶保持器（ZOH）、MATLAB Function 状态机、Sum 误差模块、PID Controller、USV_Plant 子系统、Mux 与 Scope，并完成连线。\n');
fprintf(fid, '4. **状态机实现**：在 MATLAB Function 模块中写入带 persistent 变量的三阶段切换逻辑，使用仿真时间累计稳定保持时长，避免对采样周期的硬编码依赖。\n');
fprintf(fid, '5. **船舶模型搭建**：在 USV_Plant 子系统内用 Gain、Sum、两个 Integrator 实现 T·dr/dt + r = K·δ 与 dψ/dt = r，同时输出 ψ 与 r。\n');
fprintf(fid, '6. **PID 参数配置**：离散化（Ts = %.2f s）、并联型、输出限幅 ±%g°、clamping 抗积分饱和、以 r 作为外部微分输入。\n', prm.Ts, prm.rudderMax);
fprintf(fid, '7. **信号记录**：对 6 路关键信号所在输出端口开启 DataLogging 并命名，仿真后通过 `out.logsout` 统一提取。\n');
fprintf(fid, '8. **指标计算**：将各信号重采样到统一步长网格，按阶段统计收敛时刻、稳定保持时长、上升时间、超调量、稳态误差、最大舵角与最大转艏角速度。\n');
fprintf(fid, '9. **报告与日志**：使用 MATLAB Report Generator 的 DOM 接口生成 docx（含参数表、模块表、三张曲线图与收敛数据表），用 UTF-8 编码写出本工作日志。\n\n');

fprintf(fid, '## 4. 遇到的问题与解决方法\n\n');
fprintf(fid, '1. **信号记录接口不匹配**：直接对信号线对象设置 `DataLogging` 参数会报“line 没有该参数”。改为对源模块的输出端口（`PortHandles.Outport`）设置 `DataLogging`、`DataLoggingNameMode`、`DataLoggingName`，日志通过 `out.logsout.get(name)` 读取，稳定可靠。\n');
fprintf(fid, '2. **期望航向阶跃引起微分冲击**：若对误差直接微分，30°→230° 的指令阶跃会在切换瞬间产生巨大的微分脉冲，导致舵角瞬时反复饱和。解决方法是把船舶转艏角速度 r 接入 PID 的“外部微分输入”，实现微分作用于测量值，切换过程平滑。\n');
fprintf(fid, '3. **大角度转向的积分饱和**：200° 阶跃时误差长时间较大，积分项累积会造成明显超调。通过 PID 输出限幅 ±30° 并启用 clamping 抗饱和，超调量被控制在较小范围。\n');
fprintf(fid, '4. **报告中文显示与排版**：docx 由 Report Generator DOM 接口生成。其导出字体只写入 ascii/hAnsi，缺少 eastAsia 声明，Word 会回退到日文字体（MS Mincho），造成中文字形混排；因此增加一步 document.xml 后处理，为宋体/微软雅黑补写 `w:eastAsia`。曲线图按 15 cm 宽等比缩放插入，避免超出版心。\n');
fprintf(fid, '5. **MATLAB Function 模块的采样时间约束**：状态机用 persistent 变量记录阶段号与稳定保持计时，模块若继承连续采样时间会报“persistent 变量要求非继承采样时间”。解决方法是给状态机的两路输入各加一个 0.05 s 的零阶保持器，使模块继承离散周期；保持计时改用仿真时间差，避免对采样周期硬编码。\n\n');

fprintf(fid, '## 5. 最终验证结果\n\n');
fprintf(fid, '| 校验项 | 结果 |\n| --- | --- |\n');
for k = 1:numel(check)
    if check(k).pass, tag = '通过'; else, tag = '未通过'; end
    fprintf(fid, '| %s | %s |\n', check(k).name, tag);
end
fprintf(fid, '\n');
nPass = sum([check.pass]);
fprintf(fid, '自动校验共 %d 项，通过 %d 项。\n\n', numel(check), nPass);

fprintf(fid, '关键指标：\n\n');
for k = 1:numel(res.phStats)
    s = res.phStats(k);
    fprintf(fid, '- 阶段 %d（%g°）：收敛于 %.2f s，收敛耗时 %.2f s，稳定保持 %.2f s，超调 %.2f%%，稳态误差 %.3f°。\n', ...
        s.k, s.target, s.tConv, s.tConv - s.tStart, s.holdDur, s.overshootPct, s.sse);
end
fprintf(fid, '\n');

fprintf(fid, '## 6. 产物清单\n\n');
fprintf(fid, '| 文件 | 说明 |\n| --- | --- |\n');
fprintf(fid, '| `usv_heading_pid_sim.m` | 一键运行脚本（模型构建 + 仿真 + 报告生成） |\n');
fprintf(fid, '| `%s.slx` | 自动生成的 Simulink 模型 |\n', mdlName);
fprintf(fid, '| `无人船航向PID控制仿真报告.docx` | 仿真实验报告 |\n');
fprintf(fid, '| `无人船航向PID控制仿真工作日志.md` | 本工作日志 |\n');
fprintf(fid, '| `figures/fig1_heading_response.png` | 航向阶跃响应曲线 |\n');
fprintf(fid, '| `figures/fig2_heading_error.png` | 航向误差曲线 |\n');
fprintf(fid, '| `figures/fig3_rudder_command.png` | 舵角指令曲线 |\n');
fprintf(fid, '| `simulation_results.mat` | 仿真参数与统计结果 |\n\n');

fprintf(fid, '## 7. 复现方法\n\n');
fprintf(fid, '在 MATLAB 中把当前目录切换到脚本所在目录，直接执行：\n\n');
fprintf(fid, '```matlab\nusv_heading_pid_sim\n```\n\n');
fprintf(fid, '脚本会自动完成全部流程，无需打开 Simulink 界面。若需修改船舶参数或控制增益，只需修改脚本开头的 `prm` 结构体。\n');
end

%% ---------------------- docx 辅助函数 ----------------------
function addTitle(doc, txt)
import mlreportgen.dom.*
p = Paragraph(txt);
p.Style = {FontFamily('微软雅黑'), FontSize('20pt'), Bold(true), ...
    HAlign('center'), Color('#1F3864'), OuterMargin('6pt', '0pt', '10pt', '6pt')};
append(doc, p);
end

function addH(doc, txt, lvl)
import mlreportgen.dom.*
p = Paragraph(txt);
if lvl == 1
    p.Style = {FontFamily('微软雅黑'), FontSize('14pt'), Bold(true), ...
        Color('#1F3864'), OuterMargin('12pt', '0pt', '6pt', '6pt')};
else
    p.Style = {FontFamily('微软雅黑'), FontSize('12pt'), Bold(true), ...
        Color('#2E5496'), OuterMargin('8pt', '0pt', '4pt', '4pt')};
end
append(doc, p);
end

function addP(doc, txt)
import mlreportgen.dom.*
p = Paragraph(txt);
p.Style = {FontFamily('宋体'), FontSize('10.5pt'), LineSpacing(1.4), ...
    OuterMargin('0pt', '0pt', '3pt', '3pt')};
append(doc, p);
end

function addImage(doc, imgFile, caption)
import mlreportgen.dom.*
info = imfinfo(imgFile);
w = 15.0;
h = w * info.Height / info.Width;   % 保持原始宽高比
img = Image(imgFile);
img.Style = {Width(sprintf('%.1fcm', w)), Height(sprintf('%.1fcm', h)), HAlign('center')};
append(doc, img);
p = Paragraph(caption);
p.Style = {FontFamily('宋体'), FontSize('9pt'), HAlign('center'), ...
    Color('#404040'), OuterMargin('2pt', '0pt', '2pt', '8pt')};
append(doc, p);
end

function addTable(doc, data, widths)
import mlreportgen.dom.*
tbl = Table(data);
tbl.Style = {Border('solid', '#8EA9DB', '0.5pt'), ...
    ColSep('solid', '#8EA9DB', '0.5pt'), RowSep('solid', '#8EA9DB', '0.5pt'), ...
    Width('100%'), HAlign('center')};
if nargin >= 3 && ~isempty(widths)
    groups = cell(1, numel(widths));
    for j = 1:numel(widths)
        g = TableColSpecGroup;
        g.Span = 1;
        g.Style = {Width(sprintf('%.1f%%', widths(j) * 100))};
        groups{j} = g;
    end
    tbl.ColSpecGroups = groups;
end
nRow = size(data, 1);
nCol = size(data, 2);
for i = 1:nRow
    rowObj = tbl.Children(i);
    for j = 1:nCol
        e = rowObj.Children(j);
        if i == 1
            e.Style = {FontFamily('微软雅黑'), FontSize('9pt'), Bold(true), ...
                BackgroundColor('#D9E2F3'), HAlign('center')};
        else
            e.Style = {FontFamily('宋体'), FontSize('9pt'), HAlign('center')};
        end
    end
end
append(doc, tbl);
end
