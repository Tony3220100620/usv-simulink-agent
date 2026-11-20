%
%=============================================================================
% @brief   无人船航向 PID 控制器
% @param   ~: 时间, 未用到
% @param   ~: 状态变量, 未用到
% @param   u: 输入变量, [期望航向; 实际航向], 2×1
% @param   flag: 执行阶段标志
% @param   cfg: 配置结构体
%          T: 控制周期
%          Kp: 比例参数
%          Ki: 积分参数
%          Kd: 微分参数
%          N_max: 航向力矩最大值
%          N_min: 航向力矩最小值
% @retval  sys: 输出, 依 flag 而定:
%          0-初始化参数; 3-航向力矩
% @retval  x0: 初始状态, 未用到
% @retval  str: 保留空字段
% @retval  ts: 采样时间, [period offset]
% @note    参考文献: [1] FOSSEN T I. Handbook of Marine Craft Hydrodynamics
%          and Motion Control[M]. Hoboken, NJ, USA: John Wiley & Sons, 2011.
% @note    注意事项: 使用本文件需安装 MSS 库
%=============================================================================
%
function [sys,x0,str,ts] = Heading_PID(~,~,u,flag,cfg)

switch flag
    case 0 
        % 初始化                                                
        [sys,x0,str,ts] = mdlInitializeSizes(cfg);
    case 3
        % 输出
        sys = mdlOutputs(u,cfg);
    case {1 2 4 9}
        % 未使用的 flag 值
        sys = [];
    otherwise
        % 错误处理
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

% end velFTISMC

%
%=============================================================================
% @brief   模块初始化函数
%=============================================================================
%
function [sys,x0,str,ts] = mdlInitializeSizes(cfg)

% 设置本模块的输入/输出端口数量、状态个数、采样时间等
sizes = simsizes;
sizes.NumInputs = 2;          % 输入: 无人船期望航向 ψd 与实际航向 ψ
sizes.NumOutputs = 1;         % 输出: 航向力矩 N
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [];
str = [];
ts = [cfg.T 0];

% 初始化上一时刻误差、误差积分
ud = struct('last_err', zeros(1,1), ...
            'inte', zeros(1,1));
set_param(gcb, 'UserData', ud);

% end mdlInitializeSizes

%
%=============================================================================
% @brief   模块输出函数
%=============================================================================
%
function sys = mdlOutputs(u,cfg)

% 获取控制周期
T = cfg.T;

% 获取无人船期望航向、实际航向
psid = u(1);
psi  = u(2);

% 获取上一时刻误差、误差积分
ud = get_param(gcb, 'UserData');
last_err = ud.last_err;
inte = ud.inte;

% 计算当前时刻误差并进行归一化
err = wrapToPi(psid - psi);

% 计算控制力矩
N = cfg.Kp * err + cfg.Ki * inte + cfg.Kd * (wrapToPi(err - last_err));

% 控制量限幅
N_max = cfg.N_max;
N_min = cfg.N_min;
N = min(max(N, N_min), N_max);

% 更新上一时刻误差、误差积分
inte = inte + T*err;
ud.last_err = err;
ud.inte = inte;
set_param(gcb, 'UserData', ud);

% 输出航向力矩
sys = N;

% end mdlOutputs
