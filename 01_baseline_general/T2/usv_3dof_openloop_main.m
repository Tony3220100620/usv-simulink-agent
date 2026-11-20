%% ========================================================================
%  无人船三自由度 Fossen 模型开环运动仿真
%  ------------------------------------------------------------------------
%  功能：
%    1) 以 MATLAB 脚本方式自动搭建 Simulink 模型 usv_3dof_openloop.slx
%    2) 分别施加 X = 10 N、Y = 5 N、N = 5 N*m 三种单输入工况并仿真 12 s
%    3) 输出位置、姿态与速度响应曲线（PNG），并把状态数据存为 CSV
%
%  坐标系：北-东-地 (NED)
%    状态量 eta = [x; y; psi]  位置与艏向角
%    速度量 nu  = [u; v; r]    纵向速度、横向速度、艏摇角速度
%    输入量 tau = [X; Y; N]    纵向力、横向力、艏摇力矩
%
%  模型方程（线性阻尼三自由度 Fossen 模型）：
%    eta_dot = R(psi) * nu
%    M * nu_dot + C(nu) * nu + D * nu = tau
%    其中 M = diag(m11, m22, m33)，D = diag(d11, d22, d33)，
%         C(nu) = [0, 0, -m22*v; 0, 0, m11*u; m22*v, -m11*u, 0]
%
%  运行方式：在 MATLAB 中直接运行本脚本
%  ========================================================================

clear; clc; close all;

%% ---------------- 1. 模型参数 ----------------
% 船体参数（小型无人船量级，质量与惯量均已包含附加质量）
P.m11 = 40;    % 纵向质量（含附加质量）[kg]
P.m22 = 50;    % 横向质量（含附加质量）[kg]
P.m33 = 12;    % 艏摇转动惯量（含附加质量）[kg*m^2]
P.d11 = 8;     % 纵向线性阻尼系数 [N*s/m]
P.d22 = 25;    % 横向线性阻尼系数 [N*s/m]
P.d33 = 25;    % 艏摇线性阻尼系数 [N*m*s/rad]

T_end  = 12;   % 仿真时长 [s]
dt     = 0.01; % 固定步长 [s]

% 输出目录：默认为本脚本所在目录下的 results 子文件夹
scriptDir = fileparts(mfilename('fullpath'));
outDir    = fullfile(scriptDir, 'results');
if ~exist(outDir, 'dir'), mkdir(outDir); end

modelName = 'usv_3dof_openloop';

%% ---------------- 2. 以脚本方式搭建 Simulink 模型 ----------------
% 删除同名模型，保证脚本可重复运行
if bdIsLoaded(modelName), close_system(modelName, 0); end
if isfile([modelName '.slx']), delete([modelName '.slx']); end

new_system(modelName);
load_system(modelName);

% 求解器设置：四阶龙格-库塔定步长，步长 0.01 s，仿真 12 s
set_param(modelName, 'StopTime', num2str(T_end), ...
    'StartTime', '0', 'Solver', 'ode4', 'FixedStep', num2str(dt), ...
    'SaveTime', 'on', 'SignalLogging', 'off', 'SaveOutput', 'off');

% --- 输入常量：X、Y、N（三种工况通过修改 Value 切换）---
add_block('simulink/Sources/Constant', [modelName '/X_force'], ...
    'Value', '10', 'Position', [40 55 140 85]);
add_block('simulink/Sources/Constant', [modelName '/Y_force'], ...
    'Value', '0', 'Position', [40 125 140 155]);
add_block('simulink/Sources/Constant', [modelName '/N_moment'], ...
    'Value', '0', 'Position', [40 195 140 225]);

% --- 输入合成 ---
add_block('simulink/Signal Routing/Mux', [modelName '/tau_mux'], ...
    'Inputs', '3', 'Position', [175 55 180 227]);

% --- 动力学模块（MATLAB Function）---
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/FossenDynamics'], 'Position', [215 75 345 190]);

% --- 速度积分与分解 ---
add_block('simulink/Continuous/Integrator', [modelName '/nu_int'], ...
    'InitialCondition', '[0;0;0]', 'Position', [380 100 410 140]);
add_block('simulink/Signal Routing/Demux', [modelName '/nu_demux'], ...
    'Outputs', '3', 'Position', [445 20 450 190]);

% --- 运动学模块（MATLAB Function）---
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/Kinematics'], 'Position', [445 260 575 375]);

% --- 位置积分与分解 ---
add_block('simulink/Continuous/Integrator', [modelName '/eta_int'], ...
    'InitialCondition', '[0;0;0]', 'Position', [610 285 640 325]);
add_block('simulink/Signal Routing/Demux', [modelName '/eta_demux'], ...
    'Outputs', '3', 'Position', [675 205 680 385]);

% --- 输出汇总与记录 ---
add_block('simulink/Signal Routing/Mux', [modelName '/nu_mux'], ...
    'Inputs', '3', 'Position', [485 30 490 180]);
add_block('simulink/Sinks/To Workspace', [modelName '/nu_log'], ...
    'VariableName', 'nu_out', 'SaveFormat', 'timeseries', ...
    'MaxDataPoints', 'inf', 'SampleTime', '-1', ...
    'Position', [515 70 635 100]);

add_block('simulink/Signal Routing/Mux', [modelName '/eta_mux'], ...
    'Inputs', '3', 'Position', [715 220 720 370]);
add_block('simulink/Sinks/To Workspace', [modelName '/eta_log'], ...
    'VariableName', 'eta_out', 'SaveFormat', 'timeseries', ...
    'MaxDataPoints', 'inf', 'SampleTime', '-1', ...
    'Position', [745 260 865 290]);

% --- 写入 MATLAB Function 模块内部代码 ---
% 参数以数值形式写入代码，使保存后的 .slx 可独立运行
dynCode = sprintf([ ...
    'function nu_dot = fossen_dynamics(tau, nu)\n' ...
    '%%#codegen\n' ...
    '%% 三自由度 Fossen 动力学：M*nu_dot = tau - C(nu)*nu - D*nu\n' ...
    'm11 = %.6g; m22 = %.6g; m33 = %.6g;\n' ...
    'd11 = %.6g; d22 = %.6g; d33 = %.6g;\n' ...
    'X = tau(1); Y = tau(2); N = tau(3);\n' ...
    'u = nu(1); v = nu(2); r = nu(3);\n' ...
    'u_dot = (X + m22*v*r - d11*u) / m11;\n' ...
    'v_dot = (Y - m11*u*r - d22*v) / m22;\n' ...
    'r_dot = (N - d33*r) / m33;\n' ...
    'nu_dot = [u_dot; v_dot; r_dot];\n'], ...
    P.m11, P.m22, P.m33, P.d11, P.d22, P.d33);

kinCode = sprintf([ ...
    'function eta_dot = usv_kinematics(nu, psi)\n' ...
    '%%#codegen\n' ...
    '%% 三自由度运动学：eta_dot = R(psi)*nu\n' ...
    'u = nu(1); v = nu(2); r = nu(3);\n' ...
    'eta_dot = [ u*cos(psi) - v*sin(psi);\n' ...
    '            u*sin(psi) + v*cos(psi);\n' ...
    '            r ];\n']);

sfRoot = sfroot;
chDyn = sfRoot.find('-isa', 'Stateflow.EMChart', ...
    'Path', [modelName '/FossenDynamics']);
chDyn.Script = dynCode;
chKin = sfRoot.find('-isa', 'Stateflow.EMChart', ...
    'Path', [modelName '/Kinematics']);
chKin.Script = kinCode;

% --- 连线 ---
add_line(modelName, 'X_force/1', 'tau_mux/1', 'autorouting', 'on');
add_line(modelName, 'Y_force/1', 'tau_mux/2', 'autorouting', 'on');
add_line(modelName, 'N_moment/1', 'tau_mux/3', 'autorouting', 'on');

add_line(modelName, 'tau_mux/1', 'FossenDynamics/1', 'autorouting', 'on');
add_line(modelName, 'FossenDynamics/1', 'nu_int/1', 'autorouting', 'on');
add_line(modelName, 'nu_int/1', 'nu_demux/1', 'autorouting', 'on');
add_line(modelName, 'nu_int/1', 'FossenDynamics/2', 'autorouting', 'on');
add_line(modelName, 'nu_int/1', 'Kinematics/1', 'autorouting', 'on');

add_line(modelName, 'Kinematics/1', 'eta_int/1', 'autorouting', 'on');
add_line(modelName, 'eta_int/1', 'eta_demux/1', 'autorouting', 'on');
add_line(modelName, 'eta_demux/3', 'Kinematics/2', 'autorouting', 'on');

add_line(modelName, 'nu_demux/1', 'nu_mux/1', 'autorouting', 'on');
add_line(modelName, 'nu_demux/2', 'nu_mux/2', 'autorouting', 'on');
add_line(modelName, 'nu_demux/3', 'nu_mux/3', 'autorouting', 'on');
add_line(modelName, 'nu_mux/1', 'nu_log/1', 'autorouting', 'on');

add_line(modelName, 'eta_demux/1', 'eta_mux/1', 'autorouting', 'on');
add_line(modelName, 'eta_demux/2', 'eta_mux/2', 'autorouting', 'on');
add_line(modelName, 'eta_demux/3', 'eta_mux/3', 'autorouting', 'on');
add_line(modelName, 'eta_mux/1', 'eta_log/1', 'autorouting', 'on');

% 保存模型
save_system(modelName, fullfile(pwd, [modelName '.slx']));
fprintf('模型已保存：%s\n', fullfile(pwd, [modelName '.slx']));

%% ---------------- 3. 三种单输入工况仿真 ----------------
caseDef = struct( ...
    'name', {'X = 10 N', 'Y = 5 N', 'N = 5 N*m'}, ...
    'X',    {10, 0, 0}, ...
    'Y',    {0, 5, 0}, ...
    'N',    {0, 0, 5});

results = struct();
for k = 1:numel(caseDef)
    % 设置当前工况输入
    set_param([modelName '/X_force'], 'Value', num2str(caseDef(k).X));
    set_param([modelName '/Y_force'], 'Value', num2str(caseDef(k).Y));
    set_param([modelName '/N_moment'], 'Value', num2str(caseDef(k).N));

    simOut = sim(modelName, 'StopTime', num2str(T_end));
    [eta, t] = getSimData(simOut, 'eta_out');   % [x, y, psi]
    nu       = getSimData(simOut, 'nu_out');    % [u, v, r]

    results(k).name = caseDef(k).name;
    results(k).t    = t;
    results(k).x    = eta(:,1);
    results(k).y    = eta(:,2);
    results(k).psi  = eta(:,3);
    results(k).u    = nu(:,1);
    results(k).v    = nu(:,2);
    results(k).r    = nu(:,3);

    % 保存状态数据
    writematrix([t, eta, nu], fullfile(outDir, sprintf('case%d_states.csv', k)));
end

%% ---------------- 4. 结果绘图 ----------------
figName = {'case1_X10N', 'case2_Y5N', 'case3_N5Nm'};
for k = 1:numel(results)
    R  = results(k);
    fig = figure('Name', R.name, 'Color', 'w', ...
        'Position', [100 100 900 660]);
    tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    % (a) 平面运动轨迹
    nexttile;
    plot(R.x, R.y, 'b-', 'LineWidth', 1.6); hold on;
    plot(R.x(1), R.y(1), 'ko', 'MarkerFaceColor', 'k');
    plot(R.x(end), R.y(end), 'rs', 'MarkerFaceColor', 'r');
    grid on; axis equal;
    xlabel('x [m]'); ylabel('y [m]');
    title('(a) 平面运动轨迹');

    % (b) 线速度
    nexttile;
    plot(R.t, R.u, 'b-', 'LineWidth', 1.5); hold on;
    plot(R.t, R.v, 'r--', 'LineWidth', 1.5);
    grid on; xlabel('t [s]'); ylabel('速度 [m/s]');
    legend('u', 'v', 'Location', 'best');
    title('(b) 纵向/横向速度');

    % (c) 艏向角
    nexttile;
    plot(R.t, R.psi * 180/pi, 'b-', 'LineWidth', 1.5);
    grid on; xlabel('t [s]'); ylabel('\psi [deg]');
    title('(c) 艏向角');

    % (d) 艏摇角速度
    nexttile;
    plot(R.t, R.r * 180/pi, 'b-', 'LineWidth', 1.5);
    grid on; xlabel('t [s]'); ylabel('r [deg/s]');
    title('(d) 艏摇角速度');

    sgtitle(R.name, 'FontWeight', 'bold', 'FontSize', 13);

    % 统一坐标区字体，保证插入报告后清晰可读
    axs = findobj(fig, 'Type', 'axes');
    set(axs, 'FontName', 'Microsoft YaHei', 'FontSize', 10.5);
    for a = axs'
        a.Title.FontSize = 11;
        a.XLabel.FontSize = 10.5;
        a.YLabel.FontSize = 10.5;
    end

    exportgraphics(fig, fullfile(outDir, [figName{k} '.png']), ...
        'Resolution', 200);
end

%% ---------------- 5. 结果汇总 ----------------
fprintf('\n================ 仿真结果汇总（t = %.0f s）================\n', T_end);
fprintf('工况            u_end(m/s)  v_end(m/s)  r_end(deg/s)  psi_end(deg)  x_end(m)  y_end(m)\n');
for k = 1:numel(results)
    R = results(k);
    fprintf('%-14s  %9.4f  %9.4f  %11.4f  %11.3f  %8.3f  %8.3f\n', ...
        R.name, R.u(end), R.v(end), R.r(end)*180/pi, ...
        R.psi(end)*180/pi, R.x(end), R.y(end));
end

% 汇总表写入文件，供报告生成使用
fid = fopen(fullfile(outDir, 'summary.csv'), 'w');
fprintf(fid, 'case,u_end,v_end,r_end_deg,psi_end_deg,x_end,y_end,max_abs_u,max_abs_v,max_abs_r_deg\n');
for k = 1:numel(results)
    R = results(k);
    fprintf(fid, '%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
        R.name, R.u(end), R.v(end), R.r(end)*180/pi, R.psi(end)*180/pi, ...
        R.x(end), R.y(end), max(abs(R.u)), max(abs(R.v)), ...
        max(abs(R.r))*180/pi);
end
fclose(fid);

fprintf('结果文件已输出到：%s\n', outDir);

%% ---------------- 6. 局部函数 ----------------
function [data, t] = getSimData(simOut, varName)
% 从仿真输出中读取 To Workspace 记录的 timeseries 数据
% simOut : sim 返回的 SimulationOutput
% varName: To Workspace 模块中设置的变量名
% data   : 时间序列数据矩阵（行数为采样点，列数为信号数）
% t      : 对应的时间向量
    try
        ts = simOut.get(varName);
    catch
        ts = evalin('base', varName);
    end
    data = ts.Data;
    t    = ts.Time;
end
