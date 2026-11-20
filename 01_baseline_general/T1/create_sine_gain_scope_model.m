function create_sine_gain_scope_model()
%CREATE_SINE_GAIN_SCOPE_MODEL 以编程方式自动创建正弦-增益-示波器 Simulink 模型
%
%   功能说明：
%       1) 自动新建 Simulink 模型文件（若同名模型已存在则先关闭并删除）；
%       2) 自动添加并配置 Sine Wave、Gain、Scope 三个模块；
%       3) 自动连线：Sine -> Gain -> Scope，以及 Sine -> Scope；
%       4) 自动排版，保证模块不重叠、信号流向自左向右；
%       5) 自动按信号周期计算并设置仿真停止时间；
%       6) 脚本结束时自动打开模型，可直接点击“运行”按钮开始仿真。
%
%   使用方法：
%       直接在 MATLAB 中运行本函数（编辑器点击“运行”，或命令行输入
%       create_sine_gain_scope_model）。所有可调参数集中在下方“参数配置区”。
%
%   适用版本：
%       MATLAB R2020a 及以上（已在 R2024b 上验证），依赖 Simulink 基础库，
%       不依赖任何附加工具箱。

%% ==================== 1. 参数配置区（可按需修改） ====================

% ---- 1.1 模型文件 ----
cfg.ModelName = 'SineGainScopeModel';   % 模型名称，保存为 <ModelName>.slx
cfg.OutputDir = pwd;                    % 模型保存目录，默认当前工作目录

% ---- 1.2 正弦信号参数（Sine Wave 模块）----
cfg.SineAmplitude  = 1;        % 幅值
cfg.SineBias       = 0;        % 直流偏置
cfg.SineFrequency  = 2*pi;     % 角频率（rad/s），2*pi 对应 1 Hz
cfg.SinePhase      = 0;        % 初相位（rad）
cfg.SineSampleTime = 0;        % 采样时间，0 表示连续采样

% ---- 1.3 增益参数（Gain 模块）----
cfg.Gain = 3;                  % 放大倍数

% ---- 1.4 仿真与显示参数 ----
cfg.NumPeriods        = 5;     % 仿真停止时间包含的信号周期数
cfg.MaxStepPerPeriod  = 100;   % 每个信号周期内的最大仿真步数，数值越大曲线越平滑
cfg.ScopeYSpan        = 1.2;   % 示波器纵轴范围系数（峰值乘以该系数作为纵轴上下限）

%% ==================== 2. 计算派生参数 ====================

% 由角频率换算出频率与周期：f = omega/(2*pi)，T = 1/f
sineFreqHz = cfg.SineFrequency / (2*pi);
sinePeriod = 1 / sineFreqHz;

% 仿真停止时间 = 周期数 × 周期，保证能完整观测多个周期
simStopTime = cfg.NumPeriods * sinePeriod;

% 放大后信号的峰值 = |增益| × (|幅值| + |偏置|)，用于设置示波器纵轴范围
peakValue = abs(cfg.Gain) * (abs(cfg.SineAmplitude) + abs(cfg.SineBias));

% 数值转 Simulink 参数字符串：保留 12 位有效数字，避免 2*pi 等参数被截断引入频率误差
toParamStr = @(x) sprintf('%.12g', x);

%% ==================== 3. 新建模型（自动清理同名模型） ====================

modelPath = fullfile(cfg.OutputDir, [cfg.ModelName '.slx']);

% 若同名模型已被加载，先在内存中关闭（不保存）
if bdIsLoaded(cfg.ModelName)
    close_system(cfg.ModelName, 0);
end

% 若磁盘上已存在同名模型文件，先删除，保证每次运行都得到全新模型
if exist(modelPath, 'file') == 4
    delete(modelPath);
end

% 新建并加载空白模型
new_system(cfg.ModelName);
load_system(cfg.ModelName);
fprintf('[1/6] 已新建模型：%s\n', cfg.ModelName);

%% ==================== 4. 添加并配置三个核心模块 ====================

% 模块在画布上的位置，格式为 [左 上 右 下]
% 三个模块纵向中心均位于 y = 120，形成自左向右的直线信号流，避免重叠
posSine  = [ 40 100 100 140];   % 正弦信号模块
posGain  = [210 105 270 135];   % 增益模块
posScope = [370  85 404 155];   % 示波器模块

% ---- 4.1 Sources 库：Sine Wave ----
sinePath = [cfg.ModelName '/Sine Wave'];
add_block('simulink/Sources/Sine Wave', sinePath, 'Position', posSine);

% 按参数配置区设定正弦参数（后续只需改配置区即可）
set_param(sinePath, ...
    'Amplitude',  toParamStr(cfg.SineAmplitude), ...
    'Bias',       toParamStr(cfg.SineBias), ...
    'Frequency',  toParamStr(cfg.SineFrequency), ...
    'Phase',      toParamStr(cfg.SinePhase), ...
    'SampleTime', toParamStr(cfg.SineSampleTime));

% ---- 4.2 Math Operations 库：Gain ----
gainPath = [cfg.ModelName '/Gain'];
add_block('simulink/Math Operations/Gain', gainPath, 'Position', posGain);

% 设定增益值，并指定乘法方式，避免不同版本默认值差异带来的歧义
set_param(gainPath, 'Gain', toParamStr(cfg.Gain), 'Multiplication', 'Element-wise(K.*u)');

% ---- 4.3 Sinks 库：Scope ----
scopePath = [cfg.ModelName '/Scope'];
add_block('simulink/Sinks/Scope', scopePath, 'Position', posScope);

% 示波器需要两路输入：第 1 路为放大后信号，第 2 路为原始正弦信号
set_param(scopePath, 'NumInputPorts', '2');

% 示波器显示参数：时间跨度覆盖整个仿真时长、仿真开始时自动弹窗关闭、
% 纵轴范围按放大后峰值设定，保证两路波形都完整落在画面内
% 注意：ScopeConfiguration 的 TimeSpan 只接受字符串或 'Auto'，不能直接赋数值
scopeCfg = get_param(scopePath, 'ScopeConfiguration');
scopeCfg.TimeSpan = sprintf('%.6g', simStopTime);   % 横轴显示完整仿真时长
scopeCfg.OpenAtSimulationStart = false;             % 仿真时不自动弹出示波器窗口
scopeCfg.ShowLegend = true;                         % 显示图例，便于区分两路信号

% 纵轴范围属于显示细节，不同版本属性名略有差异，用 try/catch 保证兼容性
% 两路输入共用同一坐标轴，纵轴范围按两者中较大的放大后峰值统一设定
try
    scopeCfg.YLimits = [-cfg.ScopeYSpan*peakValue, cfg.ScopeYSpan*peakValue];
catch
    warning('当前版本不支持通过 ScopeConfiguration 设置纵轴范围，已跳过该设置。');
end

fprintf('[2/6] 已添加并配置模块：Sine Wave、Gain、Scope\n');

%% ==================== 5. 自动连线 ====================

% 需求①：正弦信号输出端 -> 增益放大器输入端
add_line(cfg.ModelName, 'Sine Wave/1', 'Gain/1', 'autorouting', 'on');

% 需求①：增益放大器输出端 -> 示波器第 1 路输入端
add_line(cfg.ModelName, 'Gain/1', 'Scope/1', 'autorouting', 'on');

% 需求②：正弦信号输出端 -> 示波器第 2 路输入端（便于对比放大前后的波形）
add_line(cfg.ModelName, 'Sine Wave/1', 'Scope/2', 'autorouting', 'on');

fprintf('[3/6] 已完成信号线连接\n');

%% ==================== 6. 设置仿真停止时间 ====================

% 停止时间覆盖 cfg.NumPeriods 个完整周期，数值用 %.6g 格式书写，避免科学计数法
set_param(cfg.ModelName, 'StopTime', sprintf('%.6g', simStopTime));

% 限制最大仿真步长为一个周期的 1/MaxStepPerPeriod，保证波形描点足够密、曲线平滑
set_param(cfg.ModelName, 'MaxStep', sprintf('%.6g', sinePeriod / cfg.MaxStepPerPeriod));

fprintf('[4/6] 仿真停止时间已设置为 %s s（%.6g Hz，共 %d 个周期）\n', ...
    sprintf('%.6g', simStopTime), sineFreqHz, cfg.NumPeriods);

%% ==================== 7. 保存模型 ====================

save_system(cfg.ModelName, modelPath);
fprintf('[5/6] 模型已保存：%s\n', modelPath);

%% ==================== 8. 打开模型 ====================

% 供用户直接查看模型并点击“运行”按钮开始仿真。
% 无界面模式（matlab -batch）下无法弹出窗口，此时仅加载模型，不影响建模结果。
try
    open_system(cfg.ModelName);
catch
    warning('当前为无界面运行环境，模型未弹出窗口，但已生成并保存在：%s', modelPath);
end

fprintf('[6/6] 模型创建完成，可直接点击仿真按钮查看放大后的正弦波形。\n');

end
