%% ================================================================
%  正弦信号增益放大与示波器显示 —— Simulink 模型自动创建脚本
%  文件：usv_sine_gain_scope_build.m
%  适用版本：MATLAB R2016b 及以上（本机验证：R2024b）
%
%  功能说明：
%    1) 自动新建 Simulink 模型（无需手动新建）
%    2) 添加并配置：Sine Wave（正弦信号）、Gain（增益）、Scope（示波器）
%    3) 自动连线：正弦 → 增益 → 示波器端口2；正弦 → 示波器端口1
%    4) 自动布局，模块不重叠
%    5) 自动设置仿真停止时间，可观测多个完整周期
%    6) 保存并自动打开模型，可直接点击"运行"按钮查看波形
%    7) 脚本末尾自动运行一次仿真并导出数据/波形图（供实验报告使用）
% ================================================================
clear; clc; close all;
t_script_start = tic;                 % 记录脚本耗时（工作日志用）

%% ---------------- 0. 可自定义参数区（按需修改此处） ----------------
MODEL_NAME = 'usv_sine_gain_scope';   % 模型名称（不含 .slx 扩展名）

A_SINE   = 1;        % 【可修改】正弦信号幅值
F_SINE   = 1;        % 【可修改】正弦信号频率，单位 Hz
PHASE    = 0;        % 【可修改】正弦信号初相位，单位 rad
BIAS     = 0;        % 【可修改】正弦信号直流偏置
T_SAMPLE = 0;        % 【可修改】正弦模块采样时间，0 表示连续信号

K_GAIN   = 10;       % 【可修改】增益倍数（放大倍数）

T_STOP   = 5;        % 【可修改】仿真停止时间，单位 s（1 Hz 下含 5 个完整周期）

DO_SIM      = true;  % 是否在建模完成后自动运行一次仿真（true/false）
EXPORT_DATA = true;  % 是否导出仿真数据（CSV）与波形图（PNG）供实验报告使用

%% ---------------- 1. 模型路径与工作目录处理 ----------------
script_dir = fileparts(mfilename('fullpath'));   % 当前脚本所在目录
if isempty(script_dir)
    script_dir = pwd;                            % 兼容通过命令行调用的情形
end
slx_file   = fullfile(script_dir, [MODEL_NAME '.slx']);   % 模型保存完整路径
fig_dir    = fullfile(script_dir, 'figures');             % 报告插图与数据目录
csv_file   = fullfile(fig_dir, 'sim_waveform.csv');       % 仿真数据文件
png_file   = fullfile(fig_dir, 'waveform_compare.png');   % 对比波形图文件

if EXPORT_DATA && ~exist(fig_dir, 'dir')
    mkdir(fig_dir);                              % 目录不存在时自动创建
end

% 若模型已在内存中打开，先关闭且不保存，避免同名冲突
if bdIsLoaded(MODEL_NAME)
    close_system(MODEL_NAME, 0);                 % 0 表示不保存直接关闭
end

%% ---------------- 2. 新建模型并配置仿真参数 ----------------
% 重复运行脚本时，同名 .slx 已存在于当前目录，会触发"模型名遮蔽"提示；
% 该提示不影响建模流程，此处仅临时屏蔽这一条警告
warn_state = warning('off', 'Simulink:Engine:MdlFileShadowing');
new_system(MODEL_NAME);                          % 创建空白模型
warning(warn_state);                             % 恢复警告设置
open_system(MODEL_NAME);                         % 打开模型窗口

% 仿真求解器与时间参数：ode45 变步长，通用性好
% 最大步长取正弦周期的 1/100，保证示波器与导出波形足够平滑
set_param(MODEL_NAME, ...
    'StartTime',  '0', ...
    'StopTime',   num2str(T_STOP, 15), ...
    'SolverType', 'Variable-step', ...
    'Solver',     'ode45', ...
    'MaxStep',    num2str(1/(100*F_SINE), 15));

%% ---------------- 3. 添加三个核心模块 ----------------
% 模块来源均为 Simulink 官方自带公共库
add_block('simulink/Sources/Sine Wave',        [MODEL_NAME '/Sine Wave']);
add_block('simulink/Math Operations/Gain',     [MODEL_NAME '/Gain']);
add_block('simulink/Sinks/Scope',              [MODEL_NAME '/Scope']);

%% ---------------- 4. 配置模块参数 ----------------
% 4.1 正弦信号模块：时间型正弦，角频率 = 2*pi*f（Simulink 要求输入 rad/s）
set_param([MODEL_NAME '/Sine Wave'], ...
    'SineType',   'Time based', ...
    'Amplitude',  num2str(A_SINE, 15), ...
    'Bias',       num2str(BIAS, 15), ...
    'Frequency',  num2str(2*pi*F_SINE, 15), ...
    'Phase',      num2str(PHASE, 15), ...
    'SampleTime', num2str(T_SAMPLE, 15));

% 4.2 增益模块：逐元素相乘 K.*u，采样时间 -1 表示继承输入
set_param([MODEL_NAME '/Gain'], ...
    'Gain',           num2str(K_GAIN, 15), ...
    'Multiplication', 'Element-wise(K.*u)', ...
    'SampleTime',     '-1');

% 4.3 示波器模块：2 个输入端口、上下 2x1 分屏、仿真开始时自动打开、显示网格与图例
%     同时开启示波器数据记录（保存到工作区变量 ScopeData），供实验报告取数
set_param([MODEL_NAME '/Scope'], ...
    'NumInputPorts',              '2', ...
    'LayoutDimensionsString',     '[2 1]', ...
    'TimeSpan',                   'auto', ...
    'OpenAtSimulationStart',      'on', ...
    'ShowGrid',                   'on', ...
    'ShowLegend',                 'on', ...
    'DataLogging',                'on', ...
    'DataLoggingVariableName',    'ScopeData', ...
    'DataLoggingSaveFormat',      'Dataset', ...
    'DataLoggingLimitDataPoints', 'off');

%% ---------------- 5. 模块布局（避免重叠，信号自左向右） ----------------
% 正弦输出口（y=98）与增益输出口（y=184）分别对准示波器两个输入口，连线为水平直线
set_param([MODEL_NAME '/Sine Wave'], 'Position', [ 80  78 130 118]);
set_param([MODEL_NAME '/Gain'],      'Position', [260 164 310 204]);
set_param([MODEL_NAME '/Scope'],     'Position', [480  56 530 226]);

%% ---------------- 6. 自动连线 ----------------
% L1：正弦信号输出 → 增益输入（主通路）
add_line(MODEL_NAME, 'Sine Wave/1', 'Gain/1', 'autorouting', 'on');

% L2：增益输出 → 示波器第 2 输入端口（放大后波形，下方分屏）
add_line(MODEL_NAME, 'Gain/1', 'Scope/2', 'autorouting', 'on');

% L3：正弦信号输出 → 示波器第 1 输入端口（原始波形，上方分屏，与 L1 同源分支）
add_line(MODEL_NAME, 'Sine Wave/1', 'Scope/1', 'autorouting', 'on');

%% ---------------- 7. 保存模型并自动打开 ----------------
save_system(MODEL_NAME, slx_file);               % 保存为 .slx 文件
open_system(MODEL_NAME);                         % 末尾自动打开，方便直接运行
fprintf('模型已创建并保存：%s\n', slx_file);

%% ---------------- 8. 自动运行一次仿真并导出数据（报告用） ----------------
if DO_SIM
    fprintf('开始仿真（StopTime = %g s）...\n', T_STOP);
    out = sim(MODEL_NAME, 'ReturnWorkspaceOutputs', 'on');

    % 从示波器记录数据 ScopeData 中取回两条信号（元素 1 为端口 1，元素 2 为端口 2）
    sig_raw = out.ScopeData{1}.Values;
    sig_amp = out.ScopeData{2}.Values;

    t     = sig_raw.Time;      % 时间向量，单位 s
    y_raw = sig_raw.Data;      % 原始正弦信号
    y_amp = sig_amp.Data;      % 放大后信号

    % 统计指标（供实验报告定量分析）
    peak_raw = max(abs(y_raw));                  % 原始信号峰值
    peak_amp = max(abs(y_amp));                  % 放大后信号峰值
    ratio    = peak_amp / peak_raw;               % 实测放大倍数

    % 由正向过零点估算信号周期（纯基础运算，不依赖任何工具箱）
    k_cross = find(y_amp(1:end-1) < 0 & y_amp(2:end) >= 0);
    if numel(k_cross) >= 2
        period_est = t(k_cross(2)) - t(k_cross(1));
    else
        period_est = NaN;
    end

    fprintf('原始信号峰值 = %.4f，放大后峰值 = %.4f，实测放大倍数 = %.4f\n', ...
        peak_raw, peak_amp, ratio);
    fprintf('由过零点估算信号周期 = %.4f s（理论值 %.4f s）\n', period_est, 1/F_SINE);

    % 导出仿真数据为 CSV，便于在报告中引用
    if EXPORT_DATA
        writetable(table(t, y_raw, y_amp, ...
            'VariableNames', {'time_s', 'sine_raw', 'sine_amp'}), csv_file);

        % 原始信号与放大信号绘制在同一幅图中，便于直接比较
        fig = figure('Visible', 'off', 'Color', 'w', ...
            'Units', 'pixels', 'Position', [100 100 800 420]);
        plot(t, y_raw, 'b-', 'LineWidth', 1.5);
        hold on; grid on;
        plot(t, y_amp, 'r-', 'LineWidth', 1.5);
        xlabel('时间 t / s', 'FontName', 'Microsoft YaHei', 'FontSize', 11);
        ylabel('幅值', 'FontName', 'Microsoft YaHei', 'FontSize', 11);
        title(sprintf('正弦信号与增益放大信号对比（A=%g，f=%g Hz，K=%g）', ...
            A_SINE, F_SINE, K_GAIN), 'FontName', 'Microsoft YaHei', 'FontSize', 12);
        legend({'原始正弦信号', '增益放大后信号'}, ...
            'FontName', 'Microsoft YaHei', 'FontSize', 10, 'Location', 'northeast');
        xlim([0 T_STOP]);
        y_max = 1.1 * max([abs(y_raw(:)); abs(y_amp(:))]);   % 纵轴留 10% 余量，峰值不被裁切
        ylim([-y_max, y_max]);
        print(fig, png_file, '-dpng', '-r200');
        close(fig);
        fprintf('仿真数据已导出：%s\n波形对比图已导出：%s\n', csv_file, png_file);
    end
end

fprintf('脚本总耗时：%.2f s\n', toc(t_script_start));
